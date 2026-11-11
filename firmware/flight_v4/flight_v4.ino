/*
 * ESP32-1S-Basic-R4-E07  ·  飞控固件 flight_v1
 * ============================================================
 * 参考成熟开源飞控（Betaflight 的混控/串级 PID/airspeed 饱和处理、
 * ESP-Drone 的双核任务划分 + Mahony 姿态、Crazyflie 的解锁条件）
 * 精简到这块板能跑的规模：
 *
 *   · 姿态：加计 + 陀螺 互补滤波（roll/pitch），yaw 纯陀螺积分
 *   · 控制：串级 PID —— 角度外环（P）→ 角速度内环（PID）
 *   · 混控：X 型 4 路，带饱和缩放（保住控制权，不会因为顶到头而失稳）
 *   · 安全：解锁条件、链路超时、倾角保护、低电保护、怠速
 *   · 实时：控制跑在 core 1 的独立任务（250 Hz），WiFi/HTTP 在 loop，
 *           互不阻塞 —— 这是"能飞"和"能转"的分界线
 *
 * 热点：SSID = DroneBench（开放）  http://192.168.4.1/
 *
 * 串口（115200），保留给电脑端调试：
 *   D,<ms>,<vb>,<pg>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>,<o1>,<o2>,<o3>,<o4>,<arm>,<fl>
 *   #VER / #BOOT / #ACK / #ERR / #WIFI
 *   V            版本
 *   R            陀螺零偏标定（需停机）
 *   A,0|1        上锁 / 解锁
 *   G,<0..1000>  油门指令（‰）
 *   S,<0..1000>  起转测试（四路等值直出，不经混控）
 *   J,<0-4>,<pm>,<ms>  单路点动（0=四路同开），到点自停
 *   L            水平归零
 *   P            环内状态回读（调试用）
 *   E,<k>,<v>    在线改参数
 *   C            打印全部参数（#PARAMS,{json}）
 *   W / U / F    保存参数 / 重载参数 / 恢复默认
 *   O            OTA 能力自检（运行分区 / 可写分区 / 容量）
 *   X            全部停止 + 上锁
 *   Z            自检（混控符号 / 限幅 / PID 阶跃），打印 PASS/FAIL
 */

#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Wire.h>
#include <Preferences.h>
#include <Update.h>
#include <Arduino.h>
#include <esp_mac.h>
#include <esp_timer.h>
#include <esp_ota_ops.h>

#define FW_VER "flight-v4"

// ---------------------------------------------------------------- 硬件
static const char* AP_SSID = "DroneBench";
static const uint8_t AP_CH = 6;

static const int PIN_PWM[4] = {4, 5, 6, 7};   // M1..M4
static const int PIN_SDA = 10, PIN_SCL = 11;
static const int PIN_VBAT = 2, PIN_PGOOD = 9;
static const uint8_t IMU = 0x6B;

static const float VBAT_DIV  = 2.0f;
static const int   DUTY_MAX  = 1000;
static const uint32_t CTRL_HZ = 250;           // 控制任务频率
static const float PI_F = 3.14159265f;
static const float R2D  = 57.2957795f;

// 位置定义（板子决定，不可改）：M1 前左 / M2 前右 / M3 后左 / M4 后右
static const char* POS_NAME[4] = {"前左", "前右", "后左", "后右"};

// ---------------------------------------------------------------- 参数
struct Params {
  float angP;                 // 角度环 P（dps / deg）
  float rp, ri, rd;           // roll 角速度环
  float pp, pi, pd;           // pitch 角速度环
  float yp;                   // yaw 角速度环 P（0 = 不做 yaw 控制）
  int   idle;                 // 怠速 ‰（起飞后电机不停转）
  int   mthr;                 // 最大油门 ‰
  float mang;                 // 自稳最大倾角 deg
  int   sgR, sgP, sgY;        // IMU 轴符号（±1）
  float trMax;                // 角速度环设定上限 dps
  float trimR, trimP;         // 水平归零：把当前姿态当作 0°（补机架/桌面/传感器安装倾角）
};

static Params P = {
  6.0f,
  3.0f, 0.15f, 0.06f,
  3.0f, 0.15f, 0.06f,
  0.0f,
  0, 700, 25.0f,
  1, 1, -1,
  250.0f,
  0.0f, 0.0f
};

static Preferences prefs;

static void paramsDefault() {
  P.angP = 6.0f;
  P.rp = 3.0f; P.ri = 0.15f; P.rd = 0.06f;
  P.pp = 3.0f; P.pi = 0.15f; P.pd = 0.06f;
  P.yp = 0.0f;
  P.idle = 0; P.mthr = 700; P.mang = 25.0f;
  P.sgR = 1; P.sgP = 1; P.sgY = -1;
  P.trMax = 250.0f;
  P.trimR = 0.0f; P.trimP = 0.0f;
}

static void paramsSave() {
  prefs.begin("flight", false);
  prefs.putFloat("angP", P.angP);
  prefs.putFloat("rp", P.rp); prefs.putFloat("ri", P.ri); prefs.putFloat("rd", P.rd);
  prefs.putFloat("pp", P.pp); prefs.putFloat("pi", P.pi); prefs.putFloat("pd", P.pd);
  prefs.putFloat("yp", P.yp);
  prefs.putInt("idle", P.idle); prefs.putInt("mthr", P.mthr);
  prefs.putFloat("mang", P.mang);
  prefs.putInt("sgR", P.sgR); prefs.putInt("sgP", P.sgP); prefs.putInt("sgY", P.sgY);
  prefs.putFloat("trMax", P.trMax);
  prefs.putFloat("trimR", P.trimR); prefs.putFloat("trimP", P.trimP);
  prefs.end();
}

static void paramsLoad() {
  paramsDefault();
  prefs.begin("flight", true);
  if (prefs.isKey("angP")) {
    P.angP = prefs.getFloat("angP", P.angP);
    P.rp = prefs.getFloat("rp", P.rp); P.ri = prefs.getFloat("ri", P.ri); P.rd = prefs.getFloat("rd", P.rd);
    P.pp = prefs.getFloat("pp", P.pp); P.pi = prefs.getFloat("pi", P.pi); P.pd = prefs.getFloat("pd", P.pd);
    P.yp = prefs.getFloat("yp", P.yp);
    P.idle = prefs.getInt("idle", P.idle); P.mthr = prefs.getInt("mthr", P.mthr);
    P.mang = prefs.getFloat("mang", P.mang);
    P.sgR = prefs.getInt("sgR", P.sgR); P.sgP = prefs.getInt("sgP", P.sgP); P.sgY = prefs.getInt("sgY", P.sgY);
    P.trMax = prefs.getFloat("trMax", P.trMax);
    P.trimR = prefs.getFloat("trimR", P.trimR); P.trimP = prefs.getFloat("trimP", P.trimP);
  }
  prefs.end();
  // 越界修正（NVS 里可能存过坏值）
  if (P.idle < 0) P.idle = 0; if (P.idle > 300) P.idle = 300;
  if (P.mthr < 100) P.mthr = 100; if (P.mthr > 1000) P.mthr = 1000;
  if (P.mang < 5.0f) P.mang = 5.0f; if (P.mang > 45.0f) P.mang = 45.0f;
  if (P.trMax < 30.0f) P.trMax = 30.0f; if (P.trMax > 500.0f) P.trMax = 500.0f;
  if (P.sgR != -1) P.sgR = 1;
  if (P.sgP != -1) P.sgP = 1;
  if (P.sgY != -1) P.sgY = 1;
}

// ---------------------------------------------------------------- 飞行状态
enum Mode { M_IDLE = 0, M_FLY = 1, M_SPIN = 2 };

static volatile int      g_mode = M_IDLE;
static volatile bool     g_armed = false;
static volatile int      g_thr = 0;        // 油门指令 0..1000
static volatile int      g_spin = 0;       // 起转测试 ppm
static volatile float    g_stkRoll = 0;    // -1..1
static volatile float    g_stkPitch = 0;   // -1..1
static volatile float    g_stkYaw = 0;     // -1..1
static volatile uint32_t g_lastStick = 0;  // 链路心跳（ms）

static volatile float    g_roll = 0, g_pitch = 0, g_yaw = 0;      // deg
static volatile float    g_gr = 0, g_gp = 0, g_gy = 0;            // dps（机体轴）
static volatile float    g_ax = 0, g_ay = 0, g_az = 0;            // g
static volatile int      g_out[4] = {0, 0, 0, 0};
static volatile uint32_t g_loopHz = 0;
static volatile uint32_t g_fault = 0;
static volatile uint32_t g_armMs = 0;

// 单路点动（未解锁时用于验证接线/桨向，到点自停）
static volatile int      g_jogPm[4]  = {0, 0, 0, 0};
static volatile uint32_t g_jogEnd[4] = {0, 0, 0, 0};

static uint32_t vbatMv = 0;
static bool imuOK = false;
static uint32_t i2cFail = 0;

// 故障位
#define F_IMU   0x01
#define F_LINK  0x02
#define F_TILT  0x04
#define F_BATT  0x08

// ---------------------------------------------------------------- PID
struct Pid {
  float kp, ki, kd;
  float it, pm, df, out;
  Pid() : kp(0), ki(0), kd(0), it(0), pm(0), df(0), out(0) {}
  void reset(float m) { it = 0; pm = m; df = 0; out = 0; }
  float step(float sp, float m, float dt, float imax, float dfc) {
    const float e = sp - m;
    it += ki * e * dt;
    if (it > imax) it = imax; else if (it < -imax) it = -imax;
    // 差分：先限幅再低通。250 Hz 下 (m-pm)/dt 会把陀螺噪声放大 250 倍，
    // 不限制的话 D 项一项就能把输出顶到满舵（这是台面上"输出乱跳"的真凶）。
    float dm = (m - pm) / dt;
    pm = m;
    if (dm >  1500.0f) dm =  1500.0f;
    if (dm < -1500.0f) dm = -1500.0f;
    const float rc = 1.0f / (2.0f * PI_F * dfc);
    const float a = dt / (rc + dt);
    df += (dm - df) * a;
    float dtm = kd * df;
    if (dtm >  250.0f) dtm =  250.0f;
    if (dtm < -250.0f) dtm = -250.0f;
    out = kp * e + it - dtm;
    if (out > 900.0f) out = 900.0f;
    if (out < -900.0f) out = -900.0f;
    return out;
  }
};

static Pid pidR, pidP, pidY;
static const float IMAX = 150.0f;    // 积分限幅 ppm
static const float DFC  = 30.0f;     // D 项低通 Hz（越低越不放大高频振动）
static float armRamp = 0;            // 解锁软启动 0→1

// 桌面调试回读（'P' 命令）：看清环内每个量，别靠猜
struct DbgT { float spR, spP, oR, oP, oY, thr, auth; };
static DbgT dbg = {0, 0, 0, 0, 0, 0, 1};

// ---------------------------------------------------------------- IMU
static float gb[3] = {0, 0, 0};      // 陀螺零偏 dps
static float fa[3] = {0, 0, 0};      // 加计低通
static float fg[3] = {0, 0, 0};      // 陀螺低通
static bool  zeroReq = false;
static volatile uint32_t g_zeroMs = 0;
static volatile bool levelReq = false;

static bool imuW(uint8_t reg, uint8_t v) {
  Wire.beginTransmission(IMU); Wire.write(reg); Wire.write(v);
  return Wire.endTransmission() == 0;
}
static bool imuR(uint8_t reg, uint8_t* b, uint8_t n) {
  Wire.beginTransmission(IMU); Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)IMU, (int)n) != n) return false;
  for (uint8_t i = 0; i < n; i++) b[i] = Wire.read();
  return true;
}
static bool imuInit() {
  // 上电瞬间 I2C 偶发失败，重试到 1.5 秒。WHO_AM_I 读不到就先别判死。
  uint8_t who = 0;
  for (int k = 0; k < 30; k++) {
    if (imuR(0x00, &who, 1) && who == 0x05) {
      imuW(0x02, 0x40);    // CTRL1 地址自增
      imuW(0x03, 0x24);    // ±8 g
      imuW(0x04, 0x54);    // ±512 dps
      imuW(0x08, 0x03);    // CTRL7 使能
      delay(120);
      return true;
    }
    delay(50);
  }
  Serial.printf("#ERR,IMU_WHO,0x%02X\n", who);
  return false;
}

// 读一帧原始数据，换算成 g / dps
static bool imuRead(float a[3], float g[3]) {
  uint8_t b[12];
  if (!imuR(0x35, b, 12)) return false;
  a[0] = (float)(int16_t)(b[1]  << 8 | b[0])  / 4096.0f;
  a[1] = (float)(int16_t)(b[3]  << 8 | b[2])  / 4096.0f;
  a[2] = (float)(int16_t)(b[5]  << 8 | b[4])  / 4096.0f;
  g[0] = (float)(int16_t)(b[7]  << 8 | b[6])  / 64.0f;
  g[1] = (float)(int16_t)(b[9]  << 8 | b[8])  / 64.0f;
  g[2] = (float)(int16_t)(b[11] << 8 | b[10]) / 64.0f;
  return true;
}

// ---------------------------------------------------------------- 电机
static void writeMotors(int o[4]) {
  for (int i = 0; i < 4; i++) {
    int v = o[i];
    if (v < 0) v = 0; if (v > DUTY_MAX) v = DUTY_MAX;
    g_out[i] = v;
    ledcWrite(PIN_PWM[i], (uint32_t)v * 1023u / 1000u);
  }
}

static void motorsOff() { int o[4] = {0, 0, 0, 0}; writeMotors(o); }

static void boardInit() {
  for (int i = 0; i < 4; i++) {
    pinMode(PIN_PWM[i], OUTPUT);
    digitalWrite(PIN_PWM[i], LOW);
  }
  delay(2);
  for (int i = 0; i < 4; i++) { ledcAttach(PIN_PWM[i], 20000, 10); ledcWrite(PIN_PWM[i], 0); }
  motorsOff();
}

// ---------------------------------------------------------------- 混控
// 机体轴定义：pitch+ = 抬头，roll+ = 右滚，yaw+ = 机头右转
//   M1 前左： thr -pitch +roll +yaw
//   M2 前右： thr -pitch -roll -yaw
//   M3 后左： thr +pitch +roll -yaw
//   M4 后右： thr +pitch -roll +yaw
// 依据：抬头要让后侧推力大；右滚要让左侧推力大；机头右转要让逆时针桨（M1 M4）加速。
static void mixOutput(int thr, float oR, float oP, float oY, int o[4]) {
  float c[4];
  c[0] = -oP + oR + oY;
  c[1] = -oP - oR - oY;
  c[2] =  oP + oR - oY;
  c[3] =  oP - oR + oY;

  // 饱和缩放：顶到上下限时按比例缩小控制量，而不是丢掉控制权
  float mx = 0;
  for (int i = 0; i < 4; i++) if (fabsf(c[i]) > mx) mx = fabsf(c[i]);
  if (mx > 1.0f) {
    const float roomUp   = (float)(P.mthr - thr);
    const float roomDown = (float)thr;
    float room = roomUp < roomDown ? roomUp : roomDown;
    if (room < 1.0f) room = 1.0f;
    float sc = room / mx;
    if (sc > 1.0f) sc = 1.0f;
    if (sc < 0.12f) sc = 0.12f;              // 保留最低控制权，宁可短暂饱和
    for (int i = 0; i < 4; i++) c[i] *= sc;
  }
  for (int i = 0; i < 4; i++) {
    int v = thr + (int)lrintf(c[i]);
    if (v < 0) v = 0;
    if (v > P.mthr) v = P.mthr;
    o[i] = v;
  }
}

// ---------------------------------------------------------------- 姿态
static float rollF = 0, pitchF = 0, yawF = 0;
static bool  attSeeded = false;

static inline float lpfStep(float y, float x, float dt, float fc) {
  const float rc = 1.0f / (2.0f * PI_F * fc);
  return y + (x - y) * (dt / (rc + dt));
}

// ---------------------------------------------------------------- 控制核心
struct BBRec { uint32_t ms; int16_t thr, roll, pitch, gr, gp, gy, spR, spP, oR, oP, o1, o2, o3, o4, itR, itP, stR, stP; };
static const int BBN = 1200;
static BBRec* bb = nullptr;
static volatile int bbHead = 0, bbCount = 0;
static volatile uint32_t bbReason = 0, bbSeq = 0;
static volatile bool bbFrozen = true;
static int g_iThr = 250;   // 输出油门低于这个（‰）时不积分：还在地上时积分会越攒越歪，一离地就猛地一歪
static void bbStart() { bbHead = 0; bbCount = 0; bbReason = 0; bbFrozen = false; }

static void disarm(uint32_t fault) {
  if (g_armed && !bbFrozen) { bbFrozen = true; bbReason = fault; bbSeq++; }
  g_armed = false;
  g_mode = M_IDLE;
  g_thr = 0;
  g_spin = 0;
  g_fault |= fault;
  armRamp = 0;
  pidR.reset(0); pidP.reset(0); pidY.reset(0);
  motorsOff();
}

// 陀螺零偏标定：先等传感器稳定，丢掉前 40% 样本，再对剩下取平均。
// 结果超出 ±60 dps 说明机身当时在动，判定失败 —— 用坏偏置比不飞更危险。
static bool calibBias(uint32_t ms, uint32_t settleMs) {
  if (settleMs) delay(settleMs);
  float s[3] = {0, 0, 0};
  int n = 0;
  float aa[3], gg[3];
  const int64_t t0 = esp_timer_get_time();
  const int64_t dur = (int64_t)ms * 1000;
  while (esp_timer_get_time() - t0 < dur) {
    const bool ok = imuRead(aa, gg);
    const int64_t el = esp_timer_get_time() - t0;
    if (ok && el > dur * 2 / 5) { s[0] += gg[0]; s[1] += gg[1]; s[2] += gg[2]; n++; }
    delay(3);
  }
  if (n < 30) return false;
  float b[3];
  for (int i = 0; i < 3; i++) {
    b[i] = s[i] / (float)n;
    if (fabsf(b[i]) > 60.0f) return false;
  }
  for (int i = 0; i < 3; i++) gb[i] = b[i];
  g_zeroMs = millis();
  return true;
}

static void controlStep(float dt) {
  float a[3], gr[3];
  const bool ok = imuRead(a, gr);
  if (!ok) { i2cFail++; g_fault |= F_IMU; }
  else {
    // 低通（振动抑制，60 Hz 足够，再低会吃掉相位裕度）
    for (int i = 0; i < 3; i++) { fa[i] = lpfStep(fa[i], a[i], dt, 60.0f); fg[i] = lpfStep(fg[i], gr[i], dt, 60.0f); }
    g_ax = fa[0]; g_ay = fa[1]; g_az = fa[2];

    // 陀螺去零偏
    const float gx = fg[0] - gb[0];
    const float gy = fg[1] - gb[1];
    const float gz = fg[2] - gb[2];

    // 轴映射到机体（sg 同时作用于角度和速率，二者始终自洽）
    const float br = (float)P.sgR * gx;     // 右滚速率为正
    const float bp = (float)P.sgP * gy;     // 抬头速率为正
    const float by = (float)P.sgY * gz;     // 机头右转为正
    g_gr = br; g_gp = bp; g_gy = by;

    // 加计倾角（减掉水平归零的 trim）
    const float amag = sqrtf(fa[0] * fa[0] + fa[1] * fa[1] + fa[2] * fa[2]);
    float rollA  = (float)P.sgR * atan2f(fa[1], fa[2]) * R2D - P.trimR;
    float pitchA = (float)P.sgP * atan2f(-fa[0], sqrtf(fa[1] * fa[1] + fa[2] * fa[2])) * R2D - P.trimP;

    if (!attSeeded && amag > 0.4f) {
      rollF = rollA; pitchF = pitchA; attSeeded = true;
    } else {
      // 互补滤波：陀螺积分做高频，加计做低频（K 随 dt 归一）
      const float K = 0.03f;
      rollF  = (rollF  + br * dt) * (1.0f - K) + rollA  * K;
      pitchF = (pitchF + bp * dt) * (1.0f - K) + pitchA * K;
    }
    yawF += by * dt;
    if (yawF >  180.0f) yawF -= 360.0f;
    if (yawF < -180.0f) yawF += 360.0f;

    // 加计失效（摔了/剧烈加速）时不要用错误角度污染姿态
    if (amag < 0.4f || amag > 1.8f) { /* 只信陀螺，这帧不融合 —— 上面已做过 */ }

    g_roll = rollF; g_pitch = pitchF; g_yaw = yawF;
  }

  // ---- 零偏标定（控制任务里做，避免 I2C 跨任务竞争）----
  if (zeroReq) {
    zeroReq = false;
    if (!g_armed && g_mode != M_SPIN) {
      if (calibBias(700, 60)) {
        imuOK = true;
        Serial.printf("#ACK,ZERO,%.2f,%.2f,%.2f\n", gb[0], gb[1], gb[2]);
      }
      else Serial.println("#ERR,ZERO_FAIL,机身当时在动");
    } else Serial.println("#ERR,ARMED");
  }

  // ---- 水平归零（把当前姿态当作 0°，补机架/传感器安装倾角）----
  if (levelReq) {
    levelReq = false;
    if (g_armed || g_mode == M_SPIN) { Serial.println("#ERR,ARMED"); }
    else {
      P.trimR += rollF;
      P.trimP += pitchF;
      rollF = 0; pitchF = 0; attSeeded = true;
      pidR.reset(0); pidP.reset(0);
      Serial.printf("#ACK,LEVEL,trimR=%.2f,trimP=%.2f\n", (double)P.trimR, (double)P.trimP);
    }
  }

  // ---- 运行模式判定 ----
  const uint32_t nowMs = millis();

  // 单路点动：只要有一路还在计时就只跑这一路，姿态环完全不介入
  // （这是接线/桨向检查，不是飞行，所以不走混控）
  {
    int jo[4] = {0, 0, 0, 0};
    bool any = false;
    for (int i = 0; i < 4; i++) {
      if (g_jogPm[i] > 0 && (int32_t)(nowMs - g_jogEnd[i]) < 0) { jo[i] = g_jogPm[i]; any = true; }
      else if (g_jogPm[i] > 0) { g_jogPm[i] = 0; }      // 到时归零
    }
    if (any) {
      if (g_armed) disarm(0);
      if (g_mode == M_SPIN) { g_spin = 0; g_mode = M_IDLE; }
      writeMotors(jo);
      return;
    }
  }

  if (g_mode == M_SPIN) {
    if ((int32_t)(nowMs - g_lastStick) > 1000) { disarm(0); }
  } else if (g_armed) {
    if ((int32_t)(nowMs - g_lastStick) > 900) disarm(F_LINK);
  }

  // ---- 输出 ----
  if (g_mode == M_SPIN) {
    int o[4] = {g_spin, g_spin, g_spin, g_spin};
    writeMotors(o);
    return;
  }
  if (!g_armed) { motorsOff(); return; }

  // 姿态保护
  const float tilt = sqrtf(rollF * rollF + pitchF * pitchF);
  static uint32_t tiltMs = 0;
  if (tilt > 55.0f) {
    if (!tiltMs) tiltMs = nowMs;
    if (nowMs - tiltMs > 400) { disarm(F_TILT); return; }
  } else tiltMs = 0;

  if (vbatMv > 1000 && vbatMv < 3250) { disarm(F_BATT); return; }

  // ---- 油门与权限（先算，积分项要按权限缩放）----
  // 油门：0 指令 = 怠速，满指令 = 最大油门
  int thr = P.idle + (int)((float)g_thr * (float)(P.mthr - P.idle) / 1000.0f);
  if (thr < 0) thr = 0; if (thr > P.mthr) thr = P.mthr;

  // 控制权限随油门渐入：台面上（油门≈0）姿态环不介入，四路同速、不抖；
  // 推到 6% 行程开始给权限，25% 行程给满。
  const int range = P.mthr - P.idle;
  float auth = 1.0f;
  if (range > 20) {
    const float x = (float)(thr - P.idle) / (float)range;
    const float X0 = 0.06f, X1 = 0.25f;
    auth = (x - X0) / (X1 - X0);
    if (auth < 0) auth = 0; if (auth > 1) auth = 1;
  }
  armRamp += dt * 2.0f;
  if (armRamp > 1.0f) armRamp = 1.0f;
  const float k = armRamp * auth;

  // 参数同步。I 项的累加也乘权限：油门没推起来时不积累积分，避免一起飞就满舵。
  const float kI = (thr >= g_iThr) ? k : 0.0f;
  if (thr < g_iThr) { pidR.it = 0; pidP.it = 0; }
  pidR.kp = P.rp; pidR.ki = P.ri * kI; pidR.kd = P.rd;
  pidP.kp = P.pp; pidP.ki = P.pi * kI; pidP.kd = P.pd;
  pidY.kp = P.yp; pidY.ki = 0;       pidY.kd = 0;

  // 角度外环 → 角速度设定
  const float tRoll  = g_stkRoll  * P.mang;
  const float tPitch = g_stkPitch * P.mang;
  float spR = P.angP * (tRoll  - rollF);
  float spP = P.angP * (tPitch - pitchF);
  float spY = g_stkYaw * 120.0f;
  if (spR >  P.trMax) spR =  P.trMax; if (spR < -P.trMax) spR = -P.trMax;
  if (spP >  P.trMax) spP =  P.trMax; if (spP < -P.trMax) spP = -P.trMax;

  // 角速度内环
  const float oR = pidR.step(spR, g_gr, dt, IMAX, DFC);
  const float oP = pidP.step(spP, g_gp, dt, IMAX, DFC);
  const float oY = (P.yp > 0.001f) ? pidY.step(spY, g_gy, dt, IMAX, DFC) : 0.0f;

  // X 型混控 + 饱和缩放（控制端与自检共用同一个函数，避免"测的不是跑的那套"）
  dbg.spR = spR; dbg.spP = spP; dbg.oR = oR; dbg.oP = oP; dbg.oY = oY;
  dbg.thr = (float)thr; dbg.auth = k;

  int o[4];
  mixOutput(thr, oR * k, oP * k, oY * k, o);
  writeMotors(o);
  if (bb && !bbFrozen) {
    BBRec& r = bb[bbHead];
    r.ms = millis(); r.thr = thr;
    r.roll = (int16_t)(rollF * 10); r.pitch = (int16_t)(pitchF * 10);
    r.gr = (int16_t)(g_gr * 10); r.gp = (int16_t)(g_gp * 10); r.gy = (int16_t)(g_gy * 10);
    r.spR = (int16_t)spR; r.spP = (int16_t)spP; r.oR = (int16_t)(oR * k); r.oP = (int16_t)(oP * k);
    r.o1 = o[0]; r.o2 = o[1]; r.o3 = o[2]; r.o4 = o[3];
    r.itR = (int16_t)pidR.it; r.itP = (int16_t)pidP.it;
    r.stR = (int16_t)(g_stkRoll * 100); r.stP = (int16_t)(g_stkPitch * 100);
    bbHead = (bbHead + 1) % BBN;
    if (bbCount < BBN) bbCount++;
  }
}

// 控制任务：core 1，250 Hz。WiFi 在 core 0，互不抢时间。
static void ctrlTask(void*) {
  int64_t prev = esp_timer_get_time();
  TickType_t last = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(1000 / CTRL_HZ);
  uint32_t hzAcc = 0;
  int64_t hzT0 = prev;

  for (;;) {
    const int64_t now = esp_timer_get_time();
    float dt = (float)(now - prev) * 1e-6f;
    prev = now;
    if (dt < 0.0008f) dt = 0.0008f;
    if (dt > 0.0200f) dt = 0.0200f;

    controlStep(dt);

    hzAcc++;
    if (now - hzT0 >= 1000000) {
      g_loopHz = hzAcc;
      hzAcc = 0;
      hzT0 = now;
    }
    vTaskDelayUntil(&last, period);
  }
}

// ---------------------------------------------------------------- 页面
static const char PAGE_HTML[] PROGMEM = R"HTML(<!DOCTYPE html>
<html lang="zh-CN"><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no,viewport-fit=cover">
<meta name="theme-color" content="#080f17">
<meta name="apple-mobile-web-app-capable" content="yes">
<meta name="apple-mobile-web-app-title" content="飞控调试">
<meta name="format-detection" content="telephone=no,email=no,address=no">
<title>飞控调试台</title>
<style>
:root{
  --bg:#080f17;--card:#0f1a26;--card2:#0a131d;--line:#1a2a3b;--line2:#26405a;
  --tx:#e4edf9;--dim:#8ba0b8;--dim2:#617d99;
  --acc:#2fd3c0;--red:#ff5a5f;--amber:#f5b544;--grn:#35e07f;
}
*{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
html,body{margin:0;padding:0;height:100%;overflow:hidden;overscroll-behavior:none;background:var(--bg);color:var(--tx)}
body{font:15px/1.45 -apple-system,BlinkMacSystemFont,"PingFang SC","Hiragino Sans GB","Microsoft YaHei",system-ui,sans-serif;
 -webkit-font-smoothing:antialiased;user-select:none;-webkit-user-select:none;-webkit-touch-callout:none}
#app{display:flex;flex-direction:column;height:100vh;height:100dvh;max-width:560px;margin:0 auto;position:relative}

/* ---------- 顶栏 ---------- */
header{flex:none;display:flex;align-items:center;gap:7px;min-height:50px;
 padding:calc(env(safe-area-inset-top) + 6px) 12px 6px;background:#0a131d;border-bottom:1px solid var(--line)}
.dot{width:8px;height:8px;border-radius:50%;background:var(--grn);box-shadow:0 0 8px var(--grn);flex:none;transition:background .2s}
.dot.off{background:var(--red);box-shadow:0 0 8px var(--red)}
#vb{font:600 14px ui-monospace,SFMono-Regular,Menlo,monospace;letter-spacing:-.02em}
.pill{font-size:11px;font-weight:600;padding:3px 9px;border-radius:20px;background:#16232f;color:var(--dim);white-space:nowrap}
.pill.ok{background:rgba(53,224,127,.16);color:#6ef0a8}
.pill.warn{background:rgba(245,181,68,.16);color:#ffcd75}
.pill.err{background:rgba(255,90,95,.16);color:#ff9aa0}
.grow{flex:1}
#hz{font:11px ui-monospace,Menlo,monospace;color:var(--dim2)}
.stopbtn{flex:none;height:38px;min-width:66px;padding:0 12px;border-radius:10px;border:1px solid #ff8f94;
 background:linear-gradient(180deg,#e8545b,#c0333a);color:#fff;font:800 14px/1 inherit;letter-spacing:.06em}
.stopbtn:active{transform:scale(.96)}
.linkbtn{flex:none;height:38px;padding:0 12px;border-radius:10px;border:1px solid var(--line2);
 background:#16232f;color:var(--acc);font:700 12.5px/1 inherit}

.banner{flex:none;padding:8px 13px;font-size:12.5px;font-weight:600;
 background:rgba(255,90,95,.14);color:#ff9aa0;border-bottom:1px solid rgba(255,90,95,.25);text-align:center}
.banner[hidden]{display:none}
.banner.warn{background:rgba(245,181,68,.13);color:#ffcd75;border-bottom-color:rgba(245,181,68,.25)}

/* ---------- 内容 ---------- */
main{flex:1;overflow-y:auto;-webkit-overflow-scrolling:touch;padding:10px 12px 18px;scrollbar-width:none}
main::-webkit-scrollbar{display:none}
.pane{display:none}
.pane.on{display:block}
.card{background:var(--card);border:1px solid var(--line);border-radius:14px;padding:12px;margin-bottom:9px}
.ctitle{display:flex;align-items:center;gap:8px;font-size:11.5px;font-weight:700;color:var(--dim2);letter-spacing:.05em;margin-bottom:9px}
.ctitle .tag{margin-left:auto;font-weight:600;color:var(--dim);letter-spacing:0;font-size:11px}
.hint{font-size:11.5px;line-height:1.75;color:var(--dim2);margin-top:9px}
.hint b{color:#b9cde0;font-weight:600}
.hint i{color:var(--amber);font-style:normal}

/* 姿态球 */
.att{display:flex;gap:13px;align-items:center}
.ahi{position:relative;width:124px;height:124px;flex:none;border-radius:50%;overflow:hidden;
 border:1px solid #2a4258;background:#0c1622;box-shadow:inset 0 0 18px rgba(0,0,0,.55)}
.band{position:absolute;left:-70%;top:-70%;width:240%;height:240%;
 background:linear-gradient(180deg,#1d4062 0%,#183450 49.55%,#e9f4f3 49.55%,#e9f4f3 50.45%,#6d4a2b 50.45%,#553920 100%);
 will-change:transform}
.ahi::before,.ahi::after{content:'';position:absolute;left:50%;top:50%;background:var(--amber);border-radius:2px;z-index:2}
.ahi::before{width:6px;height:6px;margin:-3px 0 0 -3px;border-radius:50%}
.ahi::after{width:68px;height:2px;margin:-1px 0 0 -34px;opacity:.85}
.attn{flex:1;min-width:0}
.arow{display:flex;align-items:baseline;gap:6px;padding:2px 0}
.arow span{font-size:11.5px;color:var(--dim2);width:46px;flex:none}
.arow b{font:600 22px/1.1 ui-monospace,SFMono-Regular,Menlo,monospace;letter-spacing:-.03em}
.arow i{font-style:normal;font-size:11px;color:var(--dim2)}

/* 电机输出条 */
.mrow{display:flex;align-items:center;gap:9px;height:30px}
.mrow .nm{width:58px;flex:none;font-size:11.5px;color:var(--dim)}
.mrow .bar{flex:1;height:10px;border-radius:6px;background:#16232f;overflow:hidden}
.mrow .bar i{display:block;height:100%;width:0;border-radius:6px;
 background:linear-gradient(90deg,#1c8f84,#2fd3c0);transition:width .1s linear}
.mrow.hot .bar i{background:linear-gradient(90deg,#c98b1e,#f5b544)}
.mrow .vv{width:42px;flex:none;text-align:right;font:600 12.5px ui-monospace,Menlo,monospace;color:#cfe3f5}

.grid2{display:grid;grid-template-columns:1fr 1fr;gap:9px;margin-bottom:9px}
.grid3{display:grid;grid-template-columns:repeat(3,1fr);gap:6px}
.mini{display:flex;flex-direction:column;gap:2px;margin:0;padding:10px 12px}
.mini span{font-size:10.5px;color:var(--dim2)}
.mini b{font:600 19px/1.2 ui-monospace,Menlo,monospace;letter-spacing:-.02em}
.cell{background:var(--card2);border:1px solid var(--line);border-radius:9px;padding:6px 8px;display:flex;flex-direction:column;gap:1px}
.cell span{font-size:10px;color:var(--dim2)}
.cell b{font:600 13px ui-monospace,Menlo,monospace;color:#cfe3f5}

.log{height:108px;overflow-y:auto;font:11px/1.55 ui-monospace,SFMono-Regular,Menlo,monospace;color:#7d95ad;
 background:var(--card2);border:1px solid var(--line);border-radius:10px;padding:7px 9px;word-break:break-all}
.log div{margin-bottom:2px}
.log div.w{color:var(--amber)}
.log div.e{color:#ff8b90}

/* 通用按钮 */
button{font:inherit;border:0;background:none;color:inherit;cursor:pointer;touch-action:manipulation}
.btn{display:flex;align-items:center;justify-content:center;width:100%;min-height:46px;border-radius:11px;
 border:1px solid var(--line2);background:#16232f;color:#d5e2f2;font-size:14px;font-weight:700;
 transition:background .12s,border-color .12s,transform .06s}
.btn:active{transform:scale(.98)}
.btn:disabled{opacity:.38}
.btn.ghost{background:#111c28;color:#9fb2c9}
.btn.acc{border-color:#2fd3c0;background:#0f3238;color:#d6fffa}
.btn.red{border-color:#ff8f94;background:linear-gradient(180deg,#e8545b,#c0333a);color:#fff}
.btn.warn{border-color:#f7c76a;background:linear-gradient(180deg,#f2b544,#d99521);color:#20160a}
.btn.wide{margin-bottom:9px}
.two{display:grid;grid-template-columns:1fr 1fr;gap:8px}
.three{display:grid;grid-template-columns:1fr 1fr 1fr;gap:8px}

/* 电机测试卡 */
.mgrid{display:grid;grid-template-columns:1fr 1fr;gap:9px;margin-bottom:9px}
.mcard{position:relative;overflow:hidden;height:76px;border-radius:14px;border:1px solid var(--line);
 background:var(--card);display:flex;flex-direction:column;align-items:flex-start;justify-content:center;
 gap:2px;padding:0 14px;transition:border-color .12s}
.mcard b{font:800 19px/1 ui-monospace,Menlo,monospace;color:var(--tx)}
.mcard span{font-size:11.5px;color:var(--dim2)}
.mcard em{position:absolute;right:13px;bottom:10px;font:600 13px/1 ui-monospace,Menlo,monospace;
 font-style:normal;color:var(--dim2)}
.mcard .pp{position:absolute;left:0;bottom:0;height:3px;width:0;background:var(--acc);transition:none}
.mcard:active{border-color:var(--acc)}
.mcard:disabled{opacity:.4}
.mcard.dis{border-style:dashed}

/* 力度 chip */
.chips{display:flex;gap:7px}
.chips button{flex:1;min-height:42px;border-radius:10px;border:1px solid var(--line2);
 background:#111c28;color:var(--dim);font:700 13px/1 inherit}
.chips button.on{border-color:var(--acc);background:rgba(47,211,192,.13);color:#a8f7ee}

/* 滑杆 */
input[type=range]{-webkit-appearance:none;appearance:none;width:100%;height:44px;background:transparent;
 margin:0;touch-action:none;display:block}
input[type=range]::-webkit-slider-runnable-track{height:12px;border-radius:7px;background:#1b2a3b}
input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;width:34px;height:34px;margin-top:-11px;
 border-radius:50%;background:var(--acc);border:3px solid #08121b;box-shadow:0 0 0 1px var(--acc),0 2px 8px rgba(0,0,0,.5)}
input[type=range]::-moz-range-track{height:12px;border-radius:7px;background:#1b2a3b}
input[type=range]::-moz-range-thumb{width:30px;height:30px;border:3px solid #08121b;border-radius:50%;background:var(--acc)}
input[type=range]:disabled::-webkit-slider-thumb{background:#3b4c5e;box-shadow:none}
input[type=range]:disabled{opacity:.5}

.big{display:flex;align-items:baseline;justify-content:space-between;gap:8px}
.big b{font:700 34px/1 ui-monospace,SFMono-Regular,Menlo,monospace;color:var(--acc);letter-spacing:-.04em}
.big span{font-size:12px;color:var(--dim2)}

/* 解锁键 */
.armbtn{position:relative;overflow:hidden;width:100%;height:64px;margin-bottom:9px;border-radius:16px;
 border:1px solid #2c7f56;background:linear-gradient(180deg,#2bbd74,#17925a);color:#04180e;
 font-size:16.5px;font-weight:800;letter-spacing:.02em}
.armbtn span{position:relative;z-index:2}
.armbtn .fill{position:absolute;left:0;top:0;bottom:0;width:0;background:rgba(255,255,255,.3)}
.armbtn.hold .fill{width:100%;transition:width .8s linear}
.armbtn.on{border-color:#ff8f94;background:linear-gradient(180deg,#e8545b,#c0333a);color:#fff}
.armbtn:disabled{opacity:.45}

/* 摇杆 */
.pad{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;touch-action:none}
.pad button{height:58px;border-radius:13px;border:1px solid var(--line2);background:#16232f;
 color:#d5e2f2;font-size:19px;font-weight:700;touch-action:none}
.pad button.on{border-color:var(--amber);background:rgba(245,181,68,.22);color:#ffd48a}
.pad .mid{font-size:12px;font-weight:600;color:var(--dim);border-style:dashed}
.pad .sp{visibility:hidden}

/* 参数行 */
.pr{display:flex;align-items:center;gap:8px;height:46px}
.pr>span{flex:1;font-size:13px;color:var(--dim);min-width:0;overflow:hidden;text-overflow:ellipsis;white-space:nowrap}
.pr .stp{width:44px;height:44px;flex:none;border-radius:11px;border:1px solid var(--line2);
 background:#16232f;color:var(--tx);font:700 20px/1 system-ui}
.pr .stp:active{background:#1d3346;border-color:var(--acc)}
.pr input{width:78px;height:38px;flex:none;text-align:center;background:var(--card2);color:#dff0ff;
 border:1px solid var(--line2);border-radius:9px;font:700 16px ui-monospace,Menlo,monospace;
 -moz-appearance:textfield}
.pr input::-webkit-outer-spin-button,.pr input::-webkit-inner-spin-button{-webkit-appearance:none;margin:0}
.pr input:focus{outline:none;border-color:var(--acc)}
.sgrow{display:grid;grid-template-columns:1fr 1fr 1fr;gap:8px}
.sg{display:flex;flex-direction:column;align-items:center;justify-content:center;gap:3px;height:58px;
 border-radius:11px;border:1px solid var(--line2);background:#111c28;color:var(--dim2);font-size:11px}
.sg b{font:700 13px/1 inherit;color:#cfe3f5}
.sg.neg{border-color:var(--amber);background:rgba(245,181,68,.12)}
.sg.neg b{color:#ffd48a}

details{margin-top:2px}
summary{list-style:none;cursor:pointer;font-size:12px;color:var(--dim2);padding:4px 0}
summary::-webkit-details-marker{display:none}
summary::before{content:"▸ ";color:var(--acc)}
details[open] summary::before{content:"▾ "}
.mono{font:11px/1.8 ui-monospace,Menlo,monospace;color:#8ba0b8;white-space:pre-wrap}

/* ---------- 底部导航 ---------- */
nav{flex:none;display:flex;gap:5px;padding:6px 8px calc(env(safe-area-inset-bottom) + 6px);
 background:#0a131d;border-top:1px solid var(--line)}
.tab{flex:1;height:50px;border-radius:12px;display:flex;flex-direction:column;align-items:center;
 justify-content:center;gap:3px;color:var(--dim2);font-size:10.5px;font-weight:600;position:relative}
.tab svg{width:21px;height:21px;stroke:currentColor;fill:none;stroke-width:1.7;
 stroke-linecap:round;stroke-linejoin:round}
.tab.on{color:var(--acc);background:rgba(47,211,192,.1)}
.tab.on::before{content:'';position:absolute;top:0;left:50%;width:22px;height:2px;margin-left:-11px;
 border-radius:2px;background:var(--acc)}

/* ---------- 提示 ---------- */
.toast{position:absolute;left:50%;bottom:calc(76px + env(safe-area-inset-bottom));transform:translate(-50%,10px);
 max-width:88%;padding:11px 18px;border-radius:12px;background:#1d3346;border:1px solid var(--line2);
 color:#e4edf9;font-size:13.5px;font-weight:600;text-align:center;opacity:0;pointer-events:none;
 transition:opacity .18s,transform .18s;z-index:50}
.toast.on{opacity:1;transform:translate(-50%,0)}
.toast.warn{background:#3a2a14;border-color:#7c5a1f;color:#ffd48a}
.toast.err{background:#3a1a20;border-color:#6b2b34;color:#ff9aa0}
</style></head><body>
<div id="app">

<header>
  <span class="dot off" id="dot"></span>
  <b id="vb">--</b>
  <span class="pill" id="md">待机</span>
  <span class="grow"></span>
  <span id="hz">--</span>
  <button class="linkbtn" id="link" hidden>连接</button>
  <button class="stopbtn" id="stop">停止</button>
</header>

<div class="banner" id="offline">连接中…</div>
<div class="banner warn" id="fault" hidden></div>

<main id="main">

<!-- ================= 状态 ================= -->
<section class="pane on" id="p-att">
  <div class="card att">
    <div class="ahi"><div class="band" id="band"></div></div>
    <div class="attn">
      <div class="arow"><span>横滚</span><b id="nR">0.0</b><i>°</i></div>
      <div class="arow"><span>俯仰</span><b id="nP">0.0</b><i>°</i></div>
      <div class="arow"><span>偏航</span><b id="nY">0</b><i>°</i></div>
    </div>
  </div>

  <div class="card">
    <div class="ctitle">电机输出 <span class="tag" id="mtag">‰</span></div>
    <div id="mots"></div>
  </div>

  <div class="grid2">
    <div class="card mini"><span>电池</span><b id="vbb">--</b></div>
    <div class="card mini"><span>环频</span><b id="hzb">--</b></div>
  </div>

  <div class="card">
    <div class="ctitle">IMU 原始</div>
    <div class="grid3">
      <div class="cell"><span>陀螺 X</span><b id="gx">0.0</b></div>
      <div class="cell"><span>陀螺 Y</span><b id="gy">0.0</b></div>
      <div class="cell"><span>陀螺 Z</span><b id="gz">0.0</b></div>
      <div class="cell"><span>加计 X</span><b id="ax">0.00</b></div>
      <div class="cell"><span>加计 Y</span><b id="ay">0.00</b></div>
      <div class="cell"><span>加计 Z</span><b id="az">0.00</b></div>
    </div>
  </div>

  <div class="card">
    <div class="ctitle">日志 <span class="tag" id="clr" style="color:#2fd3c0">清空</span></div>
    <div class="log" id="log"></div>
  </div>
</section>

<!-- ================= 电机 ================= -->
<section class="pane" id="p-mot">
  <div class="card">
    <div class="ctitle">测试力度 <span class="tag" id="pwrt">15% · 1.5 秒</span></div>
    <div class="chips" id="pwr">
      <button data-v="100">10%</button>
      <button data-v="150" class="on">15%</button>
      <button data-v="200">20%</button>
      <button data-v="250">25%</button>
    </div>
  </div>

  <div class="mgrid">
    <button class="mcard" data-i="0" data-need-disarm><b>M1</b><span>前左</span><em id="mo0">0</em><i class="pp"></i></button>
    <button class="mcard" data-i="1" data-need-disarm><b>M2</b><span>前右</span><em id="mo1">0</em><i class="pp"></i></button>
    <button class="mcard" data-i="2" data-need-disarm><b>M3</b><span>后左</span><em id="mo2">0</em><i class="pp"></i></button>
    <button class="mcard" data-i="3" data-need-disarm><b>M4</b><span>后右</span><em id="mo3">0</em><i class="pp"></i></button>
  </div>

  <button class="btn acc wide" data-i="-1" data-need-disarm id="jall">四路同开 1.5 秒</button>

  <div class="card">
    <div class="ctitle">起转点测试 · 不锁 <span class="tag" id="sv">0 ‰</span></div>
    <input type="range" id="spin" min="0" max="250" step="5" value="0" data-need-disarm>
    <div class="two">
      <button class="btn ghost" id="spinStop" data-need-disarm>停</button>
      <button class="btn ghost" id="spinSave" data-need-disarm>记为怠速</button>
    </div>
    <div class="hint">慢慢加，看到<b>四路都能稳定转起来</b>（不是离地）就把这一档记下来。
    记为怠速后解锁瞬间四路同步慢转；<i>留 0 也完全能飞</i>。</div>
  </div>

  <div class="card hint" style="margin-bottom:0">
    <b>接线检查</b><br>
    ① 每个桨的风都<i>朝下吹</i>（推力向上）<br>
    ② 对角同向、相邻反向（M1 M4 一组，M2 M3 一组）<br>
    ③ 四路同开 1.5 秒，机身<i>不该自转</i>
  </div>
</section>

<!-- ================= 飞行 ================= -->
<section class="pane" id="p-fly">
  <button class="armbtn" id="arm"><i class="fill"></i><span id="armlab">按住 0.8 秒解锁</span></button>
  <div class="hint" id="armhint" style="margin:-2px 0 10px">油门必须为 0，机身放平（倾角 &lt; 20°）。</div>

  <div class="card">
    <div class="big"><span>油门</span><b id="tv">0%</b></div>
    <input type="range" id="thr" min="0" max="100" step="1" value="0" disabled>
    <div class="three">
      <button class="btn ghost" id="tm" disabled>− 1%</button>
      <button class="btn ghost" id="tz" disabled>归零</button>
      <button class="btn ghost" id="tp" disabled>+ 1%</button>
    </div>
    <div class="hint">推过 <b>20%</b> 自稳才介入，低于 20% 四路同步、姿态环不参与。</div>
  </div>

  <div class="card">
    <div class="ctitle">杆位 <span class="tag" id="stickt">0.00 / 0.00 / 0.00</span></div>
    <div class="pad">
      <button class="sp"></button>
      <button id="up">▲</button>
      <button class="sp"></button>
      <button id="lf">◀</button>
      <button class="mid" id="ctr">回中</button>
      <button id="rt">▶</button>
      <button class="sp"></button>
      <button id="dn">▼</button>
      <button class="sp"></button>
    </div>
    <div class="two" style="margin-top:8px">
      <button class="btn ghost" id="yl">↺ 左转</button>
      <button class="btn ghost" id="yr">↻ 右转</button>
    </div>
    <div class="hint">按住生效、松手自动回中。<b>手指滑出按钮也不会卡住</b>。</div>
  </div>

  <div class="card">
    <details>
      <summary>起飞前 5 步（点开看）</summary>
      <div class="hint">
        ① 电机检查：四路逐个点，风朝下、对角同向、四路同开不自转<br>
        ② 端起机身：右滚 roll 变大、抬头 pitch 变大 —— 反了去「参数 · 方向修正」<br>
        ③ 起转点：找到四路都能稳转的档位（可留 0）<br>
        ④ 解锁 → 油门 12% 看四路是否一起加速、机身不偏<br>
        ⑤ 每次加 3%，直到离地 —— 那一刻就是悬停油门<br>
        <i>关页面 / 切后台 0.9 秒后自动上锁。倾角超 55° 持续 0.4 秒也会自动停。</i>
      </div>
    </details>
  </div>
</section>

<!-- ================= 参数 ================= -->
<section class="pane" id="p-par">
  <div id="pform"></div>

  <div class="card">
    <div class="ctitle">方向修正</div>
    <div class="sgrow">
      <button class="sg" data-k="sgR"><span>Roll</span><b>正向</b></button>
      <button class="sg" data-k="sgP"><span>Pitch</span><b>正向</b></button>
      <button class="sg" data-k="sgY"><span>Yaw</span><b>正向</b></button>
    </div>
    <div class="hint">右滚 roll 应变大、抬头 pitch 应变大、机头右转 yaw 应变大。反了就点成「反向」。</div>
  </div>

  <div class="card">
    <div class="ctitle">校准</div>
    <div class="two">
      <button class="btn ghost" id="zero" data-need-disarm>陀螺零偏</button>
      <button class="btn ghost" id="level" data-need-disarm>水平归零</button>
    </div>
    <div class="hint">零偏：机身完全静止约 1 秒。水平归零：把当前姿态记为 0°。</div>
  </div>

  <div class="card">
    <div class="ctitle">存盘</div>
    <div class="three">
      <button class="btn ghost" id="save">保存</button>
      <button class="btn ghost" id="load">重载</button>
      <button class="btn ghost" id="defs">默认</button>
    </div>
    <div class="hint">改参数立即生效但只在内存里；<i>保存</i> 才写进 Flash，断电不丢。</div>
  </div>

  <div class="card" id="otacard">
    <div class="ctitle">固件</div>
    <a href="/update" style="display:flex;align-items:center;justify-content:center;min-height:46px;
     border-radius:11px;border:1px solid var(--line2);background:#111c28;color:#9fb2c9;
     text-decoration:none;font-size:14px;font-weight:700">上传 .bin 刷固件（OTA）</a>
    <div class="hint">手机连 <b>DroneBench</b> 热点就能刷，不用电脑。刷完自动重启，热点会断一下。</div>
  </div>

  <div class="card">
    <details>
      <summary>混控表（X 型）</summary>
      <div class="mono">
M1 前左  thr −pitch +roll +yaw
M2 前右  thr −pitch −roll −yaw
M3 后左  thr +pitch +roll −yaw
M4 后右  thr +pitch −roll +yaw

抬头(+) → 后两路加速
右滚(+) → 左侧两路加速
机头右(+) → M1 M4 加速</div>
    </details>
  </div>
</section>

</main>

<nav>
  <button class="tab on" data-tab="att">
    <svg viewBox="0 0 24 24"><circle cx="12" cy="12" r="8.2"/><path d="M12 12l4.2-4.2"/><path d="M12 3.8v1.6M12 18.6v1.6M3.8 12h1.6M18.6 12h1.6"/></svg>
    <span>状态</span>
  </button>
  <button class="tab" data-tab="mot">
    <svg viewBox="0 0 24 24"><circle cx="12" cy="12" r="2.6"/><path d="M12 3v5M12 16v5M3 12h5M16 12h5"/></svg>
    <span>电机</span>
  </button>
  <button class="tab" data-tab="fly">
    <svg viewBox="0 0 24 24"><path d="M3.5 11.2 20.5 4l-7.2 17-2.4-6.6-7.4-3.2z"/></svg>
    <span>飞行</span>
  </button>
  <button class="tab" data-tab="par">
    <svg viewBox="0 0 24 24"><path d="M4 7h9M19 7h1M4 17h3M13 17h7"/><circle cx="16" cy="7" r="2.2"/><circle cx="10" cy="17" r="2.2"/></svg>
    <span>参数</span>
  </button>
</nav>

<div class="toast" id="toast"></div>
</div>


<script>
'use strict';
var $=function(s,r){return (r||document).querySelector(s);};
var $$=function(s,r){return Array.prototype.slice.call((r||document).querySelectorAll(s));};
var clamp=function(v,a,b){return v<a?a:(v>b?b:v);};

/* ================= 传输层（WiFi 版：HTTP；USB 版由上方脚本替换） ================= */
function httpNet(){
  return {
    req:async function(path){
      var ac=new AbortController(), to=setTimeout(function(){ac.abort();},1500);
      try{
        var r=await fetch(path,{cache:'no-store',signal:ac.signal});
        var t=await r.text();
        if(!r.ok){
          if(r.status===409) return {ok:0,why:'电机还在转，先停一下'};
          return {ok:0,why:'HTTP '+r.status};
        }
        try{ return JSON.parse(t); }catch(e){ return null; }
      }catch(e){ return undefined; }
      finally{ clearTimeout(to); }
    }
  };
}
var SERIAL=!!window.__NET;
var NET=window.__NET||httpNet();

/* ================= 小工具 ================= */
var toastEl=$('#toast'), toastT=0;
function toast(msg,kind){
  toastEl.textContent=msg;
  toastEl.className='toast on'+(kind?' '+kind:'');
  clearTimeout(toastT);
  toastT=setTimeout(function(){ toastEl.className='toast'; },1900);
}
var logEl=$('#log');
function pad2(n){ return n<10?'0'+n:''+n; }
function log(msg,kind){
  var d=document.createElement('div');
  var t=new Date();
  d.textContent='['+pad2(t.getHours())+':'+pad2(t.getMinutes())+':'+pad2(t.getSeconds())+'] '+msg;
  if(kind) d.className=kind;
  logEl.insertBefore(d,logEl.firstChild);
  while(logEl.children.length>60) logEl.removeChild(logEl.lastChild);
}
function txt(node,v){ if(node&&node.__t!==v){ node.__t=v; node.textContent=v; } }
function buzz(ms){ try{ if(navigator.vibrate) navigator.vibrate(ms); }catch(e){} }

/* ================= 连接状态 ================= */
var dotEl=$('#dot'), vbEl=$('#vb'), mdEl=$('#md'), hzEl=$('#hz'), offEl=$('#offline');
var online=null, fails=0, everOn=false;

function setOnline(on){
  if(online===on) return;
  online=on;
  dotEl.classList.toggle('off',!on);
  offEl.hidden=on;
  if(on) everOn=true;
  else {
    txt(vbEl,'--'); txt(hzEl,'--');
    offEl.textContent=everOn?'连接断开 · 正在重连…':'连接中…';
  }
}
function setPill(node,cls,label){
  var k=cls+'|'+label;
  if(node.__k!==k){ node.__k=k; node.className=cls; node.textContent=label; }
}

/* ================= 本地界面状态 ================= */
var armed=false, sgs={sgR:1,sgP:1,sgY:1};
var thrEl=$('#thr'), tvEl=$('#tv'), armBtn=$('#arm'), armLab=$('#armlab');
var stk={thr:0,r:0,p:0,y:0};
var hold={up:0,dn:0,lf:0,rt:0,yl:0,yr:0};
var dirty=false;

/* ================= Tab ================= */
function showTab(name){
  $$('.tab').forEach(function(b){ b.classList.toggle('on',b.dataset.tab===name); });
  $$('.pane').forEach(function(p){ p.classList.toggle('on',p.id==='p-'+name); });
  $('#main').scrollTop=0;
  try{ localStorage.setItem('dronetab',name); }catch(e){}
}
$$('.tab').forEach(function(b){ b.onclick=function(){ showTab(b.dataset.tab); }; });
try{ var lt=localStorage.getItem('dronetab'); if(lt) showTab(lt); }catch(e){}

/* ================= 状态刷新 ================= */
var mrowEls=[];
(function buildRows(){
  var POS=['前左','前右','后左','后右'];
  var box=$('#mots');
  for(var i=0;i<4;i++){
    var d=document.createElement('div');
    d.className='mrow';
    var nm=document.createElement('span'); nm.className='nm'; nm.textContent='M'+(i+1)+' '+POS[i];
    var bar=document.createElement('span'); bar.className='bar';
    var fill=document.createElement('i');
    var v=document.createElement('span'); v.className='vv'; v.textContent='0';
    bar.appendChild(fill); d.appendChild(nm); d.appendChild(bar); d.appendChild(v);
    box.appendChild(d);
    mrowEls.push({row:d,fill:fill,val:v});
  }
})();

var FAULTS=[[1,'IMU 未就绪'],[2,'链路中断'],[4,'倾角保护'],[8,'低电压']];
var faultEl=$('#fault');

function applyState(s){
  setOnline(true);

  txt(vbEl, s.vb>800 ? (s.vb/1000).toFixed(2)+'V' : '--');
  txt(hzEl, s.hz+'Hz');

  var cls='pill', lab='待机';
  if(!s.imu||(s.f&1)){ cls='pill err'; lab='IMU 异常'; }
  else if(s.f&2){ cls='pill err'; lab='链路断开'; }
  else if(s.f&4){ cls='pill err'; lab='倾角保护'; }
  else if(s.f&8){ cls='pill err'; lab='低电压'; }
  else if(s.arm){ cls='pill ok'; lab='飞行中'; }
  else if(s.mode===2){ cls='pill warn'; lab='起转测试'; }
  setPill(mdEl,cls,lab);

  // 故障条
  var f=[];
  for(var i=0;i<FAULTS.length;i++) if(s.f&FAULTS[i][0]) f.push(FAULTS[i][1]);
  if(f.length){ faultEl.hidden=false; txt(faultEl,'上次停机原因：'+f.join(' · ')+'（重新解锁后清除）'); }
  else faultEl.hidden=true;

  // 电机输出
  for(var k=0;k<4;k++){
    var d=s.d[k]|0, r=mrowEls[k];
    txt(r.val,d);
    var w=clamp(d/10,0,100);
    r.fill.style.width=w+'%';
    r.row.classList.toggle('hot',d>250);
    var mo=$('#mo'+k); if(mo) txt(mo,d);
    var mc=document.querySelector('.mcard[data-i="'+k+'"]');
    if(mc) mc.classList.toggle('dis',!!s.arm);
  }

  // 姿态
  txt($('#nR'),(s.roll).toFixed(1));
  txt($('#nP'),(s.pitch).toFixed(1));
  txt($('#nY'),Math.round(s.yaw));
  $('#band').style.transform='rotate('+(-s.roll).toFixed(1)+'deg) translateY('+clamp(s.pitch*1.15,-70,70).toFixed(1)+'px)';

  // 遥测
  txt($('#vbb'), s.vb>800?(s.vb/1000).toFixed(2)+' V':'--');
  txt($('#hzb'), s.hz+' Hz');
  txt($('#gx'),(s.gr).toFixed(1)); txt($('#gy'),(s.gp).toFixed(1)); txt($('#gz'),(s.gy).toFixed(1));
  txt($('#ax'),(s.ax).toFixed(2)); txt($('#ay'),(s.ay).toFixed(2)); txt($('#az'),(s.az).toFixed(2));

  // 解锁状态切换
  if(!!s.arm!==armed){
    armed=!!s.arm;
    if(armed){ toast('已解锁'); log('已解锁',''); }
    else{
      toast('已上锁'); log('已上锁');
      stk.thr=0; thrEl.value=0; txt(tvEl,'0%');
      clearHolds();
    }
    paintArm(); syncLock();
  }
  // 上锁状态下油门必须显示 0
  if(!armed && +thrEl.value!==0){ thrEl.value=0; stk.thr=0; txt(tvEl,'0%'); }
}

function clearHolds(){
  for(var k in hold) hold[k]=0;
  $$('.pad button').forEach(function(b){ b.classList.remove('on'); });
  recompute();
}

/* ================= 轮询 ================= */
var polling=false, paramsLoaded=false;
async function loop(){
  if(!document.hidden && !polling){
    polling=true;
    var s=await NET.req('/api/state');
    polling=false;
    if(s && typeof s.roll==='number'){
      fails=0; applyState(s);
      if(!paramsLoaded){ paramsLoaded=true; loadParams(); }
    }
    else if(++fails>=3) setOnline(false);
  }
  setTimeout(loop,180);
}
loop();
document.addEventListener('visibilitychange',function(){ if(!document.hidden) fails=0; });

/* ================= 停止 ================= */
$('#stop').onclick=async function(){
  var r=await NET.req('/api/stop');
  buzz(30);
  stk.thr=0; thrEl.value=0; txt(tvEl,'0%');
  clearHolds();
  if(!r){ toast('没连上，但已发停止','err'); }
  else if(!r.ok){ toast('停止失败：'+(r.why||'?'),'err'); }
  else { toast('已停止'); log('紧急停止'); }
};

/* ================= 摇杆 ================= */
function recompute(){
  stk.r=(hold.rt?1:0)-(hold.lf?1:0);
  stk.p=(hold.up?1:0)-(hold.dn?1:0);
  stk.y=(hold.yr?1:0)-(hold.yl?1:0);
  txt($('#stickt'), 'R'+(stk.r>0?'+':'')+stk.r+'  P'+(stk.p>0?'+':'')+stk.p+'  Y'+(stk.y>0?'+':'')+stk.y);
  dirty=true;
}
function bindHold(id,k){
  var b=$('#'+id), act=false;
  b.addEventListener('pointerdown',function(e){
    e.preventDefault();
    if(act) return;
    act=true;
    try{ b.setPointerCapture(e.pointerId); }catch(err){}
    b.classList.add('on');
    hold[k]=1; recompute();
  });
  function end(){
    if(!act) return;
    act=false;
    b.classList.remove('on');
    hold[k]=0; recompute();
  }
  ['pointerup','pointercancel','lostpointercapture'].forEach(function(t){ b.addEventListener(t,end); });
  b.addEventListener('contextmenu',function(e){ e.preventDefault(); });
}
['up','dn','lf','rt','yl','yr'].forEach(function(k){ bindHold(k,k); });
$('#ctr').onclick=function(){ clearHolds(); toast('杆位回中'); };

thrEl.addEventListener('input',function(){
  stk.thr=+thrEl.value;
  txt(tvEl,stk.thr+'%');
  dirty=true;
});
function setThr(v){
  stk.thr=clamp(Math.round(v),0,100);
  thrEl.value=stk.thr;
  txt(tvEl,stk.thr+'%');
  dirty=true;
}
$('#tm').onclick=function(){ setThr(stk.thr-1); };
$('#tp').onclick=function(){ setThr(stk.thr+1); };
$('#tz').onclick=function(){ setThr(0); log('油门归零'); };

/* 飞行中每 300 ms 重发当前杆位：丢包能补回来，页面卡死也会被固件看门狗收掉 */
var busy=false, lastSend=0;
async function pump(){
  if(armed && !busy){
    var now=performance.now();
    if(dirty || now-lastSend>300){
      dirty=false; lastSend=now; busy=true;
      await NET.req('/api/stick?thr='+Math.round(stk.thr*10)+
        '&r='+stk.r+'&p='+stk.p+'&y='+stk.y);
      busy=false;
    }
  } else if(!armed){ dirty=false; }
  setTimeout(pump,70);
}
pump();

/* ================= 解锁 ================= */
var holdArm=false, armT=0;
function paintArm(){
  armBtn.classList.toggle('on',armed);
  armLab.textContent=armed?'点一下 上锁':'按住 0.8 秒解锁';
  $('#armhint').innerHTML=armed
    ? '已解锁 · 推油门看四路是否一起加速，推过 <b>20%</b> 自稳才介入。'
    : '油门必须为 0，机身放平（倾角 &lt; 20°）。解锁后四路先同步待命。';
}
async function doArm(){
  var r=await NET.req('/api/arm?on=1');
  if(!r){ toast('网络异常，没发出去','err'); }
  else if(!r.ok){ toast('解锁被拒：'+(r.why||'条件不满足'),'warn'); log('解锁被拒：'+(r.why||''),'w'); }
  else{ toast('解锁指令已发'); log('解锁'); }
}
async function doDisarm(){
  var r=await NET.req('/api/stop');
  stk.thr=0; thrEl.value=0; txt(tvEl,'0%'); clearHolds();
  buzz(25);
  toast(r&&r.ok?'已上锁':'已发上锁');
}
armBtn.addEventListener('pointerdown',function(e){
  e.preventDefault();
  if(armed){ doDisarm(); return; }
  if(!online){ toast('还没连上，等一下','warn'); return; }
  if(holdArm) return;
  holdArm=true;
  try{ armBtn.setPointerCapture(e.pointerId); }catch(err){}
  armBtn.classList.add('hold');
  armLab.textContent='保持…';
  armT=setTimeout(function(){
    if(!holdArm) return;
    holdArm=false;
    armBtn.classList.remove('hold');
    doArm().then(paintArm);
  },800);
});
function cancelArm(){
  if(!holdArm) return;
  holdArm=false;
  clearTimeout(armT);
  armBtn.classList.remove('hold');
  paintArm();
}
['pointerup','pointercancel','lostpointercapture'].forEach(function(t){ armBtn.addEventListener(t,cancelArm); });

/* ================= 电机点动 ================= */
var power=150;
$('#pwr').addEventListener('click',function(e){
  var b=e.target.closest('button'); if(!b||!b.dataset.v) return;
  power=+b.dataset.v;
  $$('#pwr button').forEach(function(x){ x.classList.toggle('on',x===b); });
  txt($('#pwrt'),(power/10)+'% · 1.5 秒');
});
function jog(card,i){
  if(armed){ toast('解锁中不能点电机','warn'); return; }
  NET.req('/api/jog?i='+i+'&pm='+power+'&ms=1500');
  var pp=card.querySelector('.pp');
  if(pp){
    pp.style.transition='none'; pp.style.width='0%';
    void pp.offsetWidth;
    pp.style.transition='width 1500ms linear'; pp.style.width='100%';
    setTimeout(function(){ pp.style.transition='none'; pp.style.width='0%'; },1560);
  }
  log((i<0?'四路同开':('M'+(i+1)))+' '+(power/10)+'% / 1.5s');
}
$$('.mcard').forEach(function(c){ c.addEventListener('click',function(){ jog(c,+c.dataset.i); }); });
$('#jall').addEventListener('click',function(){ jog($('#jall'),-1); });

/* ================= 起转测试 ================= */
var spinV=0;
$('#spin').addEventListener('input',function(){
  spinV=+this.value;
  txt($('#sv'),spinV+' ‰');
  NET.req('/api/spin?pm='+spinV);
});
setInterval(function(){
  if(spinV>0 && !armed) NET.req('/api/spin?pm='+spinV);
},500);
function spinOff(){
  spinV=0; $('#spin').value=0; txt($('#sv'),'0 ‰'); NET.req('/api/spin?pm=0');
}
$('#spinStop').onclick=function(){ spinOff(); log('起转测试停止'); };
$('#spinSave').onclick=function(){
  if(spinV<5){ toast('先把起转点调出来','warn'); return; }
  var v=spinV;
  NET.req('/api/param?k=idle&v='+v).then(function(){
    setParamUI('idle',v);
    toast('怠速记为 '+v+' ‰');
    log('怠速 = '+v+' ‰');
  });
  spinOff();
};

/* ================= 参数 ================= */
var PCFG=[
  ['姿态环',[['angP','角度环 P',0.5,0,20,2]]],
  ['横滚 Roll',[['rp','P',0.1,0,8,2],['ri','I',0.005,0,0.3,3],['rd','D',0.02,0,3,2]]],
  ['俯仰 Pitch',[['pp','P',0.1,0,8,2],['pi','I',0.005,0,0.3,3],['pd','D',0.02,0,3,2]]],
  ['偏航 Yaw',[['yp','P',0.1,0,8,2]]],
  ['限幅 / 油门',[['idle','怠速 ‰',5,0,300,0],['mthr','最大油门 ‰',10,100,1000,0],
                 ['mang','最大倾角 °',1,5,45,0],['trMax','角速度上限 dps',10,30,500,0]]]
];
var PUI={}, PVAL={}, PQ={}, PQT=0;

(function buildParams(){
  var host=$('#pform');
  PCFG.forEach(function(g){
    var card=document.createElement('div'); card.className='card';
    var t=document.createElement('div'); t.className='ctitle'; t.textContent=g[0];
    card.appendChild(t);
    g[1].forEach(function(it){
      var k=it[0], lab=it[1], step=it[2], mn=it[3], mx=it[4], dec=it[5];
      var row=document.createElement('div'); row.className='pr';
      var nm=document.createElement('span'); nm.textContent=lab;
      var mi=document.createElement('button'); mi.className='stp'; mi.textContent='-';
      var inp=document.createElement('input');
      inp.type='number'; inp.inputMode='decimal'; inp.step=String(step);
      inp.value=(0).toFixed(dec);
      var pl=document.createElement('button'); pl.className='stp'; pl.textContent='+';
      row.appendChild(nm); row.appendChild(mi); row.appendChild(inp); row.appendChild(pl);
      card.appendChild(row);
      PUI[k]={inp:inp,dec:dec};
      PVAL[k]=0;

      function set(v,notify){
        v=clamp(v,mn,mx);
        v=+v.toFixed(dec);
        PVAL[k]=v;
        inp.value=v.toFixed(dec);
        if(notify) queueParam(k,v);
      }
      mi.onclick=function(){ set(PVAL[k]-step,true); };
      pl.onclick=function(){ set(PVAL[k]+step,true); };
      inp.addEventListener('change',function(){
        var v=parseFloat(inp.value);
        if(isNaN(v)){ inp.value=PVAL[k].toFixed(dec); return; }
        set(v,true);
      });
      inp.addEventListener('blur',function(){ inp.value=PVAL[k].toFixed(dec); });
    });
    host.appendChild(card);
  });
})();

function setParamUI(k,v){
  var u=PUI[k]; if(!u) return;
  PVAL[k]=+Number(v).toFixed(u.dec);
  u.inp.value=PVAL[k].toFixed(u.dec);
}
function queueParam(k,v){
  PQ[k]=v;
  clearTimeout(PQT);
  PQT=setTimeout(flushParams,220);
}
async function flushParams(){
  var ks=Object.keys(PQ);
  if(!ks.length) return;
  var tmp={},i;
  for(i=0;i<ks.length;i++){ tmp[ks[i]]=PQ[ks[i]]; delete PQ[ks[i]]; }
  var ok=true;
  for(i=0;i<ks.length;i++){
    var r=await NET.req('/api/param?k='+ks[i]+'&v='+tmp[ks[i]]);
    if(!r||!r.ok) ok=false;
  }
  if(ok) log('参数已下发：'+ks.join(', '));
  else toast('参数下发失败','err');
}

async function loadParams(){
  var j=await NET.req('/api/params');
  if(!j||typeof j.angP!=='number'){ return; }
  Object.keys(PUI).forEach(function(k){ if(typeof j[k]==='number') setParamUI(k,j[k]); });
  sgs.sgR=j.sgR; sgs.sgP=j.sgP; sgs.sgY=j.sgY;
  paintSg();
  log('参数已载入');
}
function paintSg(){
  ['sgR','sgP','sgY'].forEach(function(k){
    var b=document.querySelector('.sg[data-k="'+k+'"]');
    if(!b) return;
    var neg=sgs[k]<0;
    b.classList.toggle('neg',neg);
    b.querySelector('b').textContent=neg?'反向':'正向';
  });
}
$$('.sg').forEach(function(b){
  b.onclick=async function(){
    var k=b.dataset.k;
    sgs[k]=-(sgs[k]||1);
    paintSg();
    var r=await NET.req('/api/param?k='+k+'&v='+sgs[k]);
    if(r&&r.ok) log(k+' 已设为 '+(sgs[k]<0?'反向':'正向'));
    else { sgs[k]=-sgs[k]; paintSg(); toast('改方向失败','err'); }
  };
});

/* 两段式确认，防止误触 */
function twoStep(node,cb){
  var t=0;
  node.addEventListener('click',function(){
    if(node.dataset.ready==='1'){
      clearTimeout(t);
      node.dataset.ready='0';
      node.classList.remove('warn');
      node.textContent=node.dataset.lab||'';
      cb();
      return;
    }
    node.dataset.lab=node.textContent;
    node.dataset.ready='1';
    node.classList.add('warn');
    node.textContent='再点一次确认';
    t=setTimeout(function(){
      node.dataset.ready='0';
      node.classList.remove('warn');
      node.textContent=node.dataset.lab||'';
    },2600);
  });
}

$('#zero').onclick=async function(){
  var r=await NET.req('/api/zero');
  if(r&&r.ok){ toast('零偏校准中，别动机身'); log('零偏校准…'); }
  else toast(r?('校准失败：'+(r.why||'')):'没连上','err');
};
$('#level').onclick=async function(){
  var r=await NET.req('/api/level');
  if(r&&r.ok){ toast('已把当前姿态记为 0°'); log('水平归零'); setTimeout(loadParams,400); }
  else toast('归零失败','err');
};
twoStep($('#save'),async function(){
  var r=await NET.req('/api/save');
  if(r&&r.ok){ toast('已写入 Flash'); log('参数已保存'); }
  else toast('保存失败','err');
});
$('#load').onclick=async function(){
  var r=await NET.req('/api/load');
  if(r&&r.ok){ toast('已重载'); log('参数已重载'); loadParams(); }
  else toast('重载失败','err');
};
twoStep($('#defs'),async function(){
  var r=await NET.req('/api/defs');
  if(r&&r.ok){ toast('已恢复默认'); log('恢复默认参数'); loadParams(); }
  else toast('恢复失败','err');
});

/* ================= 锁定/解锁相关控件 ================= */
function syncLock(){
  $$('[data-need-disarm]').forEach(function(n){ n.disabled=armed; });
  $('#jall').disabled=armed;
  [thrEl,$('#tm'),$('#tp'),$('#tz')].forEach(function(n){ n.disabled=!armed; });
}

/* ================= 串口版钩子 ================= */
if(SERIAL){
  $('#otacard').hidden=true;
  $('#link').hidden=false;
  log('USB 直连模式');
} else {
  log('已连接热点 '+(location.host||'DroneBench'));
}
window.addEventListener('netsay',function(e){
  var d=e.detail||{};
  if(d.t) log(d.t,d.k);
  if(d.toast) toast(d.t,d.k);
});

/* ================= 启动 ================= */
paintArm(); paintSg(); syncLock(); recompute();
setTimeout(function(){ if(!everOn) log('连不上就在手机 Wi-Fi 里选 DroneBench（无密码）','w'); },4000);
</script>
</body></html>
)HTML";

// ---------------------------------------------------------------- HTTP
static WebServer server(80);
static DNSServer dns;

// ---------------------------------------------------------------- 联网（v2）
// 热点 DroneBench 一直开着（兜底）；另外连一个保存的 WiFi，电脑在同一网络就能无线调试 / 刷固件。
static String netSsid, netPass;
static uint32_t netTry = 0, netChk = 0;
static bool netWasUp = false, mdnsOn = false;
static void netLoad() { prefs.begin("net", true); netSsid = prefs.getString("ssid", ""); netPass = prefs.getString("pass", ""); prefs.end(); }
static void netSave() { prefs.begin("net", false); prefs.putString("ssid", netSsid); prefs.putString("pass", netPass); prefs.end(); }
static void netBegin() { if (netSsid.length()) { WiFi.begin(netSsid.c_str(), netPass.c_str()); netTry = millis(); } }
static void netStatus() {
  const bool up = WiFi.status() == WL_CONNECTED;
  Serial.printf("#NET,%s,%d,%s\n", up ? "ok" : (netSsid.length() ? "wait" : "none"), up ? (int)WiFi.RSSI() : 0,
                up ? WiFi.localIP().toString().c_str() : "-");
}
static String hexDec(const char* h) {
  String o; int n = strlen(h);
  for (int i = 0; i + 1 < n; i += 2) { char b[3] = {h[i], h[i + 1], 0}; o += (char)strtol(b, nullptr, 16); }
  return o;
}
static void netLoop() {
  const uint32_t now = millis();
  if (now - netChk < 1000) return;
  netChk = now;
  const bool up = WiFi.status() == WL_CONNECTED;
  if (up != netWasUp) {
    netWasUp = up;
    if (up && !mdnsOn) { mdnsOn = MDNS.begin("minidrone"); if (mdnsOn) MDNS.addService("http", "tcp", 80); }
    netStatus();
  }
  if (!up && netSsid.length() && now - netTry > 20000) netBegin();
}
// 电脑每 40 ms 发一包：U,<seq>,<live>,<thr>,<r>,<p>,<y>
// live=1 且已解锁 → 更新油门/杆位并喂心跳；不管怎样都回一包遥测：
// T,<seq>,<vb>,<arm>,<fault>,<roll>,<pitch>,<gr>,<gp>,<gy>,<o1>,<o2>,<o3>,<o4>,<ax>,<ay>,<az>,<rssi>,<pg>
static WiFiUDP udp;
static void udpTask(void*) {
  char b[128], o[220];
  for (;;) {
    int n = udp.parsePacket();
    while (n > 0) {
      int l = udp.read(b, sizeof(b) - 1);
      b[l < 0 ? 0 : l] = 0;
      unsigned seq = 0; int live = 0, t = 0; float r = 0, p = 0, y = 0;
      if (b[0] == 'X') {             // 急停也走 UDP，不等网页服务
        disarm(0);
        for (int k = 0; k < 4; k++) g_jogPm[k] = 0;
        g_spin = 0;
      }
      else if (b[0] == 'U' && sscanf(b, "U,%u,%d,%d,%f,%f,%f", &seq, &live, &t, &r, &p, &y) == 6) {
        if (live && g_armed) {
          if (t < 0) t = 0; if (t > 1000) t = 1000;
          g_thr = t;
          g_stkRoll  = r < -1 ? -1 : (r > 1 ? 1 : r);
          g_stkPitch = p < -1 ? -1 : (p > 1 ? 1 : p);
          g_stkYaw   = y < -1 ? -1 : (y > 1 ? 1 : y);
          g_lastStick = millis();
        }
        snprintf(o, sizeof(o), "T,%u,%lu,%d,%lu,%.1f,%.1f,%.1f,%.1f,%.1f,%d,%d,%d,%d,%.2f,%.2f,%.2f,%d,%d",
                 seq, (unsigned long)vbatMv, g_armed ? 1 : 0, (unsigned long)g_fault,
                 (double)g_roll, (double)g_pitch, (double)g_gr, (double)g_gp, (double)g_gy,
                 (int)g_out[0], (int)g_out[1], (int)g_out[2], (int)g_out[3],
                 (double)g_ax, (double)g_ay, (double)g_az, (int)WiFi.RSSI(), digitalRead(PIN_PGOOD));
        udp.beginPacket(udp.remoteIP(), udp.remotePort());
        udp.write((const uint8_t*)o, strlen(o));
        udp.endPacket();
      }
      n = udp.parsePacket();
    }
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

static void netScan() {
  if (g_armed || g_mode == M_SPIN) { Serial.println("#ERR,ARMED"); return; }
  const int n = WiFi.scanNetworks(false, false);
  for (int i = 0; i < n && i < 25; i++) {
    String s = WiFi.SSID(i), h;
    for (size_t k = 0; k < s.length(); k++) { char b[3]; snprintf(b, 3, "%02X", (uint8_t)s[k]); h += b; }
    Serial.printf("#SCAN,%d,%d,%d,%s\n", (int)WiFi.RSSI(i), (int)WiFi.channel(i), (int)WiFi.encryptionType(i), h.c_str());
    delay(2);
  }
  WiFi.scanDelete();
  Serial.println("#SCAN,end");
}

static void sendJson(const char* s) {
  // iOS Safari 对 GET 缓存很激进，API 一律 no-store
  server.sendHeader("Cache-Control", "no-store");
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json; charset=utf-8", s);
}

// 状态 / 参数的 JSON 由 HTTP 与串口共用，保证"本地页和手机页看到的是同一份数据"
static void buildStateJson(char* j, size_t n) {
  snprintf(j, n,
           "{\"vb\":%lu,\"arm\":%d,\"mode\":%d,\"thr\":%d,\"sp\":%d,"
           "\"d\":[%d,%d,%d,%d],"
           "\"roll\":%.1f,\"pitch\":%.1f,\"yaw\":%.0f,"
           "\"gr\":%.1f,\"gp\":%.1f,\"gy\":%.1f,"
           "\"ax\":%.2f,\"ay\":%.2f,\"az\":%.2f,"
           "\"hz\":%lu,\"dt\":%.1f,\"f\":%lu,\"imu\":%d}",
           (unsigned long)vbatMv, g_armed ? 1 : 0, (int)g_mode, (int)g_thr, (int)g_spin,
           (int)g_out[0], (int)g_out[1], (int)g_out[2], (int)g_out[3],
           (double)g_roll, (double)g_pitch, (double)g_yaw,
           (double)g_gr, (double)g_gp, (double)g_gy,
           (double)g_ax, (double)g_ay, (double)g_az,
           (unsigned long)g_loopHz, g_loopHz ? (1000.0 / (double)g_loopHz) : 0.0,
           (unsigned long)g_fault, imuOK ? 1 : 0);
}

static void buildParamsJson(char* j, size_t n) {
  snprintf(j, n,
           "{\"angP\":%.2f,\"rp\":%.2f,\"ri\":%.3f,\"rd\":%.2f,"
           "\"pp\":%.2f,\"pi\":%.3f,\"pd\":%.2f,\"yp\":%.2f,"
           "\"idle\":%d,\"mthr\":%d,\"mang\":%.0f,\"trMax\":%.0f,"
           "\"trimR\":%.2f,\"trimP\":%.2f,"
           "\"sgR\":%d,\"sgP\":%d,\"sgY\":%d}",
           (double)P.angP,
           (double)P.rp, (double)P.ri, (double)P.rd,
           (double)P.pp, (double)P.pi, (double)P.pd,
           (double)P.yp, P.idle, P.mthr, (double)P.mang, (double)P.trMax,
           (double)P.trimR, (double)P.trimP,
           P.sgR, P.sgP, P.sgY);
}

static void handleState() {
  char j[512];
  buildStateJson(j, sizeof(j));
  sendJson(j);
}

static void handleParams() {
  char j[512];
  buildParamsJson(j, sizeof(j));
  sendJson(j);
}

// 解锁条件（对应成熟飞控的 arm check）
static void handleBB() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Cache-Control", "no-store");
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  char head[160];
  snprintf(head, sizeof(head), "# seq=%lu reason=%lu frozen=%d n=%d angP=%.2f rp=%.2f ri=%.3f rd=%.3f iThr=%d\n",
           (unsigned long)bbSeq, (unsigned long)bbReason, bbFrozen ? 1 : 0, bbCount, P.angP, P.rp, P.ri, P.rd, g_iThr);
  server.send(200, "text/plain", head);
  server.sendContent("ms,thr,roll,pitch,gr,gp,gy,spR,spP,oR,oP,o1,o2,o3,o4,itR,itP,stR,stP\n");
  if (!bb) return;
  String chunk;
  const int n = bbCount, start = (bbHead - n + BBN) % BBN;
  for (int i = 0; i < n; i++) {
    const BBRec& r = bb[(start + i) % BBN];
    char l[140];
    snprintf(l, sizeof(l), "%lu,%d,%.1f,%.1f,%.1f,%.1f,%.1f,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%.2f,%.2f\n",
             (unsigned long)r.ms, r.thr, r.roll / 10.0, r.pitch / 10.0, r.gr / 10.0, r.gp / 10.0, r.gy / 10.0,
             r.spR, r.spP, r.oR, r.oP, r.o1, r.o2, r.o3, r.o4, r.itR, r.itP, r.stR / 100.0, r.stP / 100.0);
    chunk += l;
    if (chunk.length() > 1800) { server.sendContent(chunk); chunk = ""; }
  }
  if (chunk.length()) server.sendContent(chunk);
}

static bool canArm(String& why) {
  if (!imuOK) { why = "IMU 未就绪"; return false; }
  if (g_mode == M_SPIN) { why = "起转测试中"; return false; }
  if (g_thr > 0) { why = "油门不在 0"; return false; }
  if (fabsf(g_roll) > 20.0f || fabsf(g_pitch) > 20.0f) { why = "机身没放平（倾角需 <20°）"; return false; }
  if (fabsf(g_gr) > 25.0f || fabsf(g_gp) > 25.0f) { why = "机身还在动"; return false; }
  if (vbatMv > 1000 && vbatMv < 3300) { why = "电池低于 3.3V"; return false; }
  return true;
}

static void handleArm() {
  const int on = server.arg("on").toInt();
  if (on) {
    String why;
    if (!canArm(why)) {
      char j[128];
      snprintf(j, sizeof(j), "{\"ok\":0,\"why\":\"%s\"}", why.c_str());
      sendJson(j);
      return;
    }
    g_thr = 0;
    g_stkRoll = g_stkPitch = g_stkYaw = 0;
    g_fault = 0;
    pidR.reset(g_gr); pidP.reset(g_gp); pidY.reset(g_gy);
    g_lastStick = millis();
    g_armMs = millis(); bbStart();
    g_mode = M_FLY;
    g_armed = true;
    Serial.println("#ACK,ARM");
  } else {
    disarm(0);
    Serial.println("#ACK,DISARM");
  }
  sendJson("{\"ok\":1}");
}

static void handleStick() {
  if (!g_armed) { sendJson("{\"ok\":0}"); return; }
  int t = server.arg("thr").toInt();
  if (t < 0) t = 0; if (t > 1000) t = 1000;
  g_thr = t;
  float r = server.arg("r").toFloat();
  float p = server.arg("p").toFloat();
  float y = server.arg("y").toFloat();
  g_stkRoll  = r < -1 ? -1 : (r > 1 ? 1 : r);
  g_stkPitch = p < -1 ? -1 : (p > 1 ? 1 : p);
  g_stkYaw   = y < -1 ? -1 : (y > 1 ? 1 : y);
  g_lastStick = millis();
  sendJson("{\"ok\":1}");
}

static void handleSpin() {
  if (g_armed) { server.send(409, "text/plain", "armed"); return; }
  int pm = server.arg("pm").toInt();
  if (pm < 0) pm = 0;
  if (pm > 250) pm = 250;      // 25% 已远超起转点，再高四路同开可能真离地
  g_spin = pm;
  g_mode = pm > 0 ? M_SPIN : M_IDLE;
  g_lastStick = millis();
  sendJson("{\"ok\":1}");
}

// 单路点动：i = 0..3 指定某一路，i = -1 四路同开。到点自停，不需要再发停止。
static void jogStart(int i, int pm, int ms) {
  if (pm < 0) pm = 0;
  if (pm > 300) pm = 300;
  if (ms < 100) ms = 100;
  if (ms > 4000) ms = 4000;
  if (g_mode == M_SPIN) { g_spin = 0; g_mode = M_IDLE; }
  for (int k = 0; k < 4; k++) g_jogPm[k] = 0;
  if (pm == 0) return;
  const uint32_t end = millis() + (uint32_t)ms;
  if (i < 0) { for (int k = 0; k < 4; k++) { g_jogPm[k] = pm; g_jogEnd[k] = end; } }
  else if (i < 4) { g_jogPm[i] = pm; g_jogEnd[i] = end; }
}

static void handleJog() {
  if (g_armed) { server.send(409, "text/plain", "armed"); return; }
  const int i = server.arg("i").toInt();
  jogStart(i, server.arg("pm").toInt(), server.arg("ms").toInt());
  sendJson("{\"ok\":1}");
}

static void handleStop() {
  disarm(0);
  for (int k = 0; k < 4; k++) g_jogPm[k] = 0;
  Serial.println("#ACK,STOP");
  sendJson("{\"ok\":1}");
}

static void handleZero() {
  if (g_armed) { server.send(409, "text/plain", "armed"); return; }
  zeroReq = true;
  sendJson("{\"ok\":1}");
}

static void handleLevel() {
  if (g_armed) { server.send(409, "text/plain", "armed"); return; }
  levelReq = true;
  sendJson("{\"ok\":1}");
}

static void handleParam() {
  const String k = server.arg("k");
  const float v = server.arg("v").toFloat();
  if (k == "angP") P.angP = v;
  else if (k == "rp") P.rp = v;
  else if (k == "ri") P.ri = v;
  else if (k == "rd") P.rd = v;
  else if (k == "pp") P.pp = v;
  else if (k == "pi") P.pi = v;
  else if (k == "pd") P.pd = v;
  else if (k == "yp") P.yp = v;
  else if (k == "idle") P.idle = (int)v;
  else if (k == "mthr") P.mthr = (int)v;
  else if (k == "mang") P.mang = v;
  else if (k == "trMax") P.trMax = v;
  else if (k == "trimR") P.trimR = v;
  else if (k == "trimP") P.trimP = v;
  else if (k == "sgR") P.sgR = (v < 0) ? -1 : 1;
  else if (k == "sgP") P.sgP = (v < 0) ? -1 : 1;
  else if (k == "sgY") P.sgY = (v < 0) ? -1 : 1;
  else if (k == "iThr") g_iThr = (int)v;
  else { server.send(400, "text/plain", "bad key"); return; }

  // 边界收敛
  if (P.angP < 0) P.angP = 0; if (P.angP > 20) P.angP = 20;
  if (P.idle < 0) P.idle = 0; if (P.idle > 300) P.idle = 300;
  if (P.mthr < 100) P.mthr = 100; if (P.mthr > 1000) P.mthr = 1000;
  if (P.idle > P.mthr) P.idle = P.mthr;
  if (P.mang < 5) P.mang = 5; if (P.mang > 45) P.mang = 45;
  if (P.trMax < 30) P.trMax = 30; if (P.trMax > 500) P.trMax = 500;
  sendJson("{\"ok\":1}");
}

static void handleSave() { paramsSave(); sendJson("{\"ok\":1}"); }
static void handleLoad() { paramsLoad(); sendJson("{\"ok\":1}"); }
static void handleDefs() { paramsDefault(); sendJson("{\"ok\":1}"); }

// ---------------------------------------------------------------- 网页 OTA
// 手机连热点后打开 /update 选 .bin 就能刷。第一次刷机仍要 USB（空片没有网页）。
static const char OTA_HTML[] PROGMEM = R"HTML(<!DOCTYPE html>
<html lang="zh-CN"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>固件升级</title>
<style>
body{margin:0;background:#0a111a;color:#dce8f6;font:14px -apple-system,"PingFang SC",sans-serif;padding:16px}
h1{font-size:16px;margin:6px 0 14px;color:#2fd3c0}
.card{background:#111c2c;border:1px solid #1e2c40;border-radius:12px;padding:14px;margin-bottom:10px}
input[type=file]{width:100%;font-size:12px;color:#9fb2c9}
button{width:100%;margin-top:12px;padding:13px;font-size:14px;font-weight:600;border-radius:10px;
 border:1px solid #2fd3c0;background:#123641;color:#d6fffa}
button:disabled{opacity:.45}
.hl{font-size:11.5px;color:#7d90a9;line-height:1.8}
.hl b{color:#cfe0f4}.hl i{color:#f2b544;font-style:normal}
#bar{height:6px;background:#16233a;border-radius:3px;overflow:hidden;margin-top:10px;display:none}
#bar i{display:block;height:100%;width:0;background:#2fd3c0;transition:width .2s}
#msg{font-size:12px;margin-top:10px;color:#9fb2c9;white-space:pre-wrap}
</style></head><body>
<h1>固件升级 · DroneBench</h1>
<div class="card">
  <input type="file" id="f" accept=".bin">
  <button id="go" disabled>上传并重启</button>
  <div id="bar"><i></i></div>
  <div id="msg">选一个 .bin（arduino-cli 编译产物 flight_v1.ino.bin）</div>
</div>
<div class="card hl">
  <b>注意事项</b><br>
  ① 上传中不要断电、不要关页面，约 10–20 秒<br>
  ② 只接受本机编译的 <i>flight_v1.ino.bin</i>，别传错型号的固件<br>
  ③ 刷完板子自动重启，热点会断一下，重连即可<br>
  ④ <i>全新空板第一次还是得用 USB 刷</i>，之后所有更新都能走这里
</div>
<script>
const f=document.getElementById('f'),go=document.getElementById('go'),
      bar=document.getElementById('bar'),inn=bar.firstElementChild,
      msg=document.getElementById('msg');
f.onchange=()=>{go.disabled=!f.files.length;};
go.onclick=()=>{
  const file=f.files[0]; if(!file) return;
  if(!confirm('确认刷入 '+file.name+' ？')) return;
  go.disabled=true; bar.style.display='block'; msg.textContent='上传中…';
  const x=new XMLHttpRequest();
  x.open('POST','/api/update');
  x.upload.onprogress=e=>{
    if(e.lengthComputable){const p=Math.round(e.loaded*100/e.total);
      inn.style.width=p+'%'; msg.textContent='已上传 '+p+'%';}
  };
  x.onload=()=>{msg.textContent = x.status===200 ? '刷写完成，板子正在重启…' : '失败：'+x.responseText;};
  x.onerror=()=>{msg.textContent='连接中断（刷写中重启是正常的，等 10 秒重连热点）';};
  const fd=new FormData(); fd.append('firmware',file,'fw.bin');
  x.send(fd);
};
</script></body></html>)HTML";

static void handleOtaPage() {
  server.send_P(200, "text/html; charset=utf-8", OTA_HTML);
}

static void handleOtaDone() {
  const bool ok = !Update.hasError();
  server.sendHeader("Connection", "close");
  // 失败必须回非 200，页面才能显示"失败"而不是"刷写完成"
  server.send(ok ? 200 : 500, "text/plain", ok ? "OK" : "FLASH_FAIL");
  if (ok) {
    Serial.println("#OTA,reboot");
    delay(400);
    ESP.restart();
  }
}

static void handleOtaUpload() {
  HTTPUpload& up = server.upload();
  if (up.status == UPLOAD_FILE_START) {
    disarm(0);
    Serial.printf("#OTA,start,%s,%u\n", up.filename.c_str(), (unsigned)up.totalSize);
    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) Update.printError(Serial);
  } else if (up.status == UPLOAD_FILE_WRITE) {
    if (Update.write(up.buf, up.currentSize) != up.currentSize) Update.printError(Serial);
  } else if (up.status == UPLOAD_FILE_END) {
    if (Update.end(true)) Serial.printf("#OTA,done,%u\n", (unsigned)up.totalSize);
    else Update.printError(Serial);
  }
}

static void handleNotFound() {
  server.sendHeader("Location", String("http://") + WiFi.softAPIP().toString() + "/", true);
  server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate");
  server.send(302, "text/plain", "");
}

// ---------------------------------------------------------------- 串口
static void sendVer() {
  uint8_t mac[6];
  esp_efuse_mac_get_default(mac);
  Serial.printf("#VER,%s,%02X:%02X:%02X:%02X:%02X:%02X,imu=0x%02X,div=1.0,dutymax=%d,hz=%d\n",
                FW_VER, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
                (unsigned)IMU, DUTY_MAX, (int)CTRL_HZ);
  Serial.printf("#WIFI,ssid=%s,open,ip=%s,http://%s/\n",
                AP_SSID, WiFi.softAPIP().toString().c_str(), WiFi.softAPIP().toString().c_str());
}

static void handleLine(char* s) {
  if (!s[0]) return;
  const char c = s[0];
  if (c == 'N') {   // 联网：N? 状态 / NS 扫描 / NX 忘记 / N,<hex ssid>,<hex pass>
    if (s[1] == '?') { netStatus(); return; }
    if (s[1] == 'S') { netScan(); return; }
    if (s[1] == 'X') { netSsid = ""; netPass = ""; netSave(); WiFi.disconnect(false); Serial.println("#ACK,NX"); netStatus(); return; }
    if (s[1] == ',') {
      char* p2 = strchr(s + 2, ',');
      if (!p2) { Serial.println("#ERR,NET_FMT"); return; }
      *p2 = 0;
      netSsid = hexDec(s + 2); netPass = hexDec(p2 + 1);
      netSave(); WiFi.disconnect(false); delay(50); netBegin(); netWasUp = false;
      Serial.println("#ACK,NET");
    }
    return;
  }
  if (c == 'V') { sendVer(); return; }
  // O —— OTA 能力自检：报告当前运行分区 / 下一个可写分区 / 最大可刷字节
  if (c == 'O') {
    const esp_partition_t* run = esp_ota_get_running_partition();
    const esp_partition_t* nxt = esp_ota_get_next_update_partition(NULL);
    Serial.printf("#OTA,run=%s,addr=0x%X,next=%s,nextaddr=0x%X,nextsize=%u,can=%d\n",
                  run ? run->label : "?", run ? (unsigned)run->address : 0,
                  nxt ? nxt->label : "none", nxt ? (unsigned)nxt->address : 0,
                  nxt ? (unsigned)nxt->size : 0,
                  (nxt && nxt->size >= 900 * 1024) ? 1 : 0);
    return;
  }
  if (c == 'K') { g_lastStick = millis(); return; }
  if (c == 'X') { disarm(0); Serial.println("#ACK,STOP"); return; }
  if (c == 'R') { if (g_armed) { Serial.println("#ERR,ARMED"); return; } zeroReq = true; return; }
  if (c == 'L') { levelReq = true; return; }

  if (c == 'A') {
    int on = 0;
    if (sscanf(s, "A,%d", &on) == 1) {
      if (on) {
        String why;
        if (!canArm(why)) { Serial.printf("#ERR,ARM_FAIL,%s\n", why.c_str()); return; }
        g_thr = 0; g_stkRoll = g_stkPitch = g_stkYaw = 0; g_fault = 0;
        pidR.reset(g_gr); pidP.reset(g_gp); pidY.reset(g_gy);
        g_lastStick = millis(); g_armMs = millis(); bbStart();
        g_mode = M_FLY; g_armed = true;
        Serial.println("#ACK,ARM");
      } else { disarm(0); Serial.println("#ACK,DISARM"); }
    }
    return;
  }
  if (c == 'G') {
    int pm = 0;
    if (sscanf(s, "G,%d", &pm) == 1) {
      if (pm < 0) pm = 0; if (pm > 1000) pm = 1000;
      g_thr = pm;
      g_lastStick = millis();
      Serial.printf("#ACK,G,%d,arm=%d\n", pm, g_armed ? 1 : 0);
    }
    return;
  }
  if (c == 'E') {   // E,<name>,<val> 在线改参数（不写 Flash），桌面调参用
    char k[12] = {0};
    float v = 0;
    if (sscanf(s, "E,%11[^,],%f", k, &v) == 2) {
      if (!strcmp(k, "rp")) P.rp = v;
      else if (!strcmp(k, "ri")) P.ri = v;
      else if (!strcmp(k, "rd")) P.rd = v;
      else if (!strcmp(k, "pp")) P.pp = v;
      else if (!strcmp(k, "pi")) P.pi = v;
      else if (!strcmp(k, "pd")) P.pd = v;
      else if (!strcmp(k, "yp")) P.yp = v;
      else if (!strcmp(k, "angP")) P.angP = v;
      else if (!strcmp(k, "idle")) P.idle = (int)v;
      else if (!strcmp(k, "mthr")) P.mthr = (int)v;
      else if (!strcmp(k, "mang")) P.mang = v;
      else if (!strcmp(k, "trMax")) P.trMax = v;
      else if (!strcmp(k, "trimR")) P.trimR = v;
      else if (!strcmp(k, "trimP")) P.trimP = v;
      else if (!strcmp(k, "sgR")) P.sgR = v < 0 ? -1 : 1;
      else if (!strcmp(k, "sgP")) P.sgP = v < 0 ? -1 : 1;
      else if (!strcmp(k, "sgY")) P.sgY = v < 0 ? -1 : 1;
      else if (!strcmp(k, "iThr")) g_iThr = (int)v;
      else { Serial.printf("#ERR,NO_KEY,%s\n", k); return; }
      Serial.printf("#ACK,E,%s=%.2f\n", k, (double)v);
    }
    return;
  }
  if (c == 'Q') {   // Q,<roll>,<pitch>,<yaw>  -1..1，桌面验证混控方向用
    float r = 0, p = 0, y = 0;
    if (sscanf(s, "Q,%f,%f,%f", &r, &p, &y) == 3) {
      if (r < -1) r = -1; if (r > 1) r = 1;
      if (p < -1) p = -1; if (p > 1) p = 1;
      if (y < -1) y = -1; if (y > 1) y = 1;
      g_stkRoll = r; g_stkPitch = p; g_stkYaw = y;
      g_lastStick = millis();
      Serial.printf("#ACK,Q,%.2f,%.2f,%.2f\n", (double)r, (double)p, (double)y);
    }
    return;
  }
  if (c == 'S') {
    int pm = 0;
    if (sscanf(s, "S,%d", &pm) == 1) {
      if (g_armed) { Serial.println("#ERR,ARMED"); return; }
      if (pm < 0) pm = 0; if (pm > 250) pm = 250;
      g_spin = pm;
      g_mode = pm > 0 ? M_SPIN : M_IDLE;
      g_lastStick = millis();
      Serial.printf("#ACK,S,%d\n", pm);
    }
    return;
  }
  if (c == 'P') {   // 环内状态回读
    Serial.printf("#DBG,roll=%.2f,pitch=%.2f,gr=%.2f,gp=%.2f,spR=%.1f,spP=%.1f,"
                  "oR=%.1f,oP=%.1f,oY=%.1f,itR=%.1f,itP=%.1f,thr=%.0f,k=%.2f\n",
                  (double)rollF, (double)pitchF, (double)g_gr, (double)g_gp,
                  (double)dbg.spR, (double)dbg.spP,
                  (double)dbg.oR, (double)dbg.oP, (double)dbg.oY,
                  (double)pidR.it, (double)pidP.it, (double)dbg.thr, (double)dbg.auth);
    return;
  }
  if (c == 'J') {   // J,<ch 0-4>,<pm>,<ms>  ch=0 表示四路同开
    int ch = 0, pm = 0, ms = 0;
    if (sscanf(s, "J,%d,%d,%d", &ch, &pm, &ms) == 3) {
      if (g_armed) { Serial.println("#ERR,ARMED"); return; }
      jogStart(ch - 1, pm, ms);
      Serial.printf("#ACK,J,%d,%d,%d\n", ch, pm, ms);
    }
    return;
  }
  if (c == 'T') {   // 兼容旧点动：T,<ch>,<pm>,<ms> —— 走起转测试通道（四路同值，简化）
    int pm = 0, ms = 0;
    if (sscanf(s, "T,%*d,%d,%d", &pm, &ms) == 2) {
      if (g_armed) { Serial.println("#ERR,ARMED"); return; }
      g_spin = pm; g_mode = M_SPIN; g_lastStick = millis();
      Serial.printf("#ACK,T,%d,%d\n", pm, ms);
    }
    return;
  }
  // C —— 打印当前全部参数（本地 Web Serial 页读参数用，内容同 /api/params）
  if (c == 'C') {
    char j[512];
    buildParamsJson(j, sizeof(j));
    Serial.printf("#PARAMS,%s\n", j);
    return;
  }
  if (c == 'W') { paramsSave();    Serial.println("#ACK,SAVE"); return; }
  if (c == 'U') { paramsLoad();    Serial.println("#ACK,LOAD"); return; }
  if (c == 'F') { paramsDefault(); Serial.println("#ACK,DEFS"); return; }

  if (c == 'Z') {
    // ---- 自检：全部调用真实运行的那套函数，不另写一份"看起来对"的 ----
    int pass = 0, fail = 0;
    int o[4];

    // 1) 抬头(+) 应当后两路加速、前两路减速
    mixOutput(300, 0, 100, 0, o);
    {
      bool g = (o[2] > o[0]) && (o[3] > o[1]) && (o[0] == o[1]) && (o[2] == o[3]);
      Serial.printf("#TEST,mix_pitch,%s,%d,%d,%d,%d\n", g ? "PASS" : "FAIL", o[0], o[1], o[2], o[3]);
      g ? pass++ : fail++;
    }
    // 2) 右滚(+) 应当左侧（M1 M3）加速
    mixOutput(300, 100, 0, 0, o);
    {
      bool g = (o[0] > o[1]) && (o[2] > o[3]) && (o[0] == o[2]) && (o[1] == o[3]);
      Serial.printf("#TEST,mix_roll,%s,%d,%d,%d,%d\n", g ? "PASS" : "FAIL", o[0], o[1], o[2], o[3]);
      g ? pass++ : fail++;
    }
    // 3) 机头右转(+) 应当 M1 M4 加速（这两路是逆时针桨，加速 → 机头右）
    mixOutput(300, 0, 0, 100, o);
    {
      bool g = (o[0] > o[1]) && (o[3] > o[2]);
      Serial.printf("#TEST,mix_yaw,%s,%d,%d,%d,%d\n", g ? "PASS" : "FAIL", o[0], o[1], o[2], o[3]);
      g ? pass++ : fail++;
    }
    // 4) 全行程扫描：任何油门/任何控制量下输出都必须落在 [0, mthr]
    {
      bool bad = false;
      int worst = 0;
      const float cs[3] = {-600.0f, 0.0f, 600.0f};
      for (int t = 0; t <= P.mthr; t += 20) {
        for (int a = 0; a < 3; a++) for (int b = 0; b < 3; b++) for (int d = 0; d < 3; d++) {
          mixOutput(t, cs[a], cs[b], cs[d], o);
          for (int i = 0; i < 4; i++) {
            if (o[i] < 0 || o[i] > P.mthr) { bad = true; }
            if (o[i] > worst) worst = o[i];
          }
        }
      }
      Serial.printf("#TEST,range,%s,max=%d,thr_max=%d\n", bad ? "FAIL" : "PASS", worst, P.mthr);
      bad ? fail++ : pass++;
    }
    // 5) 速率环方向：正误差应产生正输出
    {
      Pid t; t.kp = P.rp; t.ki = P.ri; t.kd = P.rd;
      t.reset(0);
      const float u = t.step(50.0f, 0.0f, 0.004f, IMAX, DFC);
      bool g = (u > 0);
      Serial.printf("#TEST,pid_dir,%s,out=%.1f\n", g ? "PASS" : "FAIL", (double)u);
      g ? pass++ : fail++;
    }
    // 6) 闭环仿真：给 20° 滚转目标，用真实 PID 跑 2.4 秒，看是否稳定收敛到目标
    {
      Pid t; t.kp = P.rp; t.ki = P.ri; t.kd = P.rd;
      t.reset(0);
      float ang = 0, rate = 0, sp = 0;
      const float dt = 0.004f;
      for (int i = 0; i < 600; i++) {
        sp = P.angP * (20.0f - ang);
        if (sp >  P.trMax) sp =  P.trMax;
        if (sp < -P.trMax) sp = -P.trMax;
        const float u = t.step(sp, rate, dt, IMAX, DFC);
        rate += (u * 0.2f - rate) * (dt * 10.0f);   // 执行机构一阶滞后
        ang  += rate * dt;
      }
      const bool g = (ang > 15.0f && ang < 25.0f) && (fabsf(rate) < 25.0f);
      Serial.printf("#TEST,closed_loop,%s,ang=%.1f,rate=%.1f\n", g ? "PASS" : "FAIL", (double)ang, (double)rate);
      g ? pass++ : fail++;
    }
    // 7) 零控制时必须四路等值（台面上不该有差速；也是"解锁后不抖"的保证）
    {
      bool g = true;
      const int ts[3] = {0, P.idle, P.mthr};
      for (int k = 0; k < 3; k++) {
        mixOutput(ts[k], 0, 0, 0, o);
        if (!(o[0] == ts[k] && o[1] == ts[k] && o[2] == ts[k] && o[3] == ts[k])) g = false;
      }
      Serial.printf("#TEST,mix_zero,%s,%d,%d,%d,%d\n", g ? "PASS" : "FAIL", o[0], o[1], o[2], o[3]);
      g ? pass++ : fail++;
    }
    Serial.printf("#TEST,SUMMARY,pass=%d,fail=%d\n", pass, fail);
    return;
  }
}

// ---------------------------------------------------------------- setup
void setup() {
  boardInit();
  bb = (BBRec*)malloc(sizeof(BBRec) * BBN);
  Serial.setTxBufferSize(2048);
  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);   // USB 插着但没人读时绝不等待，否则主循环卡死、WiFi 断
  delay(180);

  Wire.begin(PIN_SDA, PIN_SCL, 400000);
  Wire.setClock(400000);
  imuOK = imuInit();

  pinMode(PIN_PGOOD, INPUT);
  pinMode(PIN_VBAT, INPUT);
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  paramsLoad();

  // 开机陀螺零偏：能稳定读到数据且偏置合理才算 IMU 就绪。
  // （只信 WHO_AM_I 不够；偏置采歪了姿态环会一直满舵，比不飞更危险）
  {
    bool ok = calibBias(500, 250);
    if (!ok) { delay(150); ok = calibBias(500, 100); }
    imuOK = ok;
    Serial.printf("#IMU,ok=%d,bias=%.2f,%.2f,%.2f\n",
                  imuOK ? 1 : 0, (double)gb[0], (double)gb[1], (double)gb[2]);
    if (!imuOK) Serial.println("#ERR,IMU_ZERO,静止放置后点一次「零偏校准」");
  }

  // 电池采样
  {
    uint32_t acc = 0;
    for (int i = 0; i < 8; i++) { acc += analogReadMilliVolts(PIN_VBAT); delay(2); }
    vbatMv = (uint32_t)(acc / 8.0f * VBAT_DIV);
  }

  // ---- 热点 ----
  WiFi.persistent(false);
  WiFi.mode(WIFI_AP_STA);
  WiFi.setSleep(false);
  IPAddress ip(192, 168, 4, 1), gw(192, 168, 4, 1), mask(255, 255, 255, 0);
  WiFi.softAPConfig(ip, gw, mask);
  WiFi.softAP(AP_SSID, NULL, AP_CH, 0, 4);
  netLoad();
  netBegin();
  delay(150);
  dns.setErrorReplyCode(DNSReplyCode::NoError);
  dns.start(53, "*", WiFi.softAPIP());

  server.on("/", []() { server.send_P(200, "text/html; charset=utf-8", PAGE_HTML); });
  // favicon 不许落进 302，否则浏览器每次都拉一遍整页
  server.on("/favicon.ico", []() { server.send(204, "text/plain", ""); });
  server.on("/api/state", handleState);
  server.on("/api/bb", handleBB);
  server.on("/api/params", handleParams);
  server.on("/api/arm", handleArm);
  server.on("/api/stick", handleStick);
  server.on("/api/spin", handleSpin);
  server.on("/api/jog", handleJog);
  server.on("/api/stop", handleStop);
  server.on("/api/zero", handleZero);
  server.on("/api/level", handleLevel);
  server.on("/api/param", handleParam);
  server.on("/api/save", handleSave);
  server.on("/api/load", handleLoad);
  server.on("/api/defs", handleDefs);
  server.on("/update", handleOtaPage);
  server.on("/api/update", HTTP_POST, handleOtaDone, handleOtaUpload);
  server.onNotFound(handleNotFound);
  server.begin();
  udp.begin(4210);
  xTaskCreatePinnedToCore(udpTask, "udp", 4096, nullptr, 3, nullptr, 0);

  // ---- 控制任务 ----
  g_lastStick = millis();
  xTaskCreatePinnedToCore(ctrlTask, "ctrl", 4096, nullptr, 5, nullptr, 1);

  Serial.println();
  Serial.println("#BOOT,ESP32-1S-Basic-R4-E07," FW_VER ",AP_MODE,X4,PID");
  sendVer();
  if (!imuOK) Serial.println("#ERR,IMU_INIT");
}

// ---------------------------------------------------------------- loop
void loop() {
  dns.processNextRequest();
  server.handleClient();
  netLoop();

  while (Serial.available()) {
    static char rxBuf[255];
    static uint8_t rxLen = 0;
    const char ch = (char)Serial.read();
    if (ch == '\n' || ch == '\r') { rxBuf[rxLen] = 0; if (rxLen) handleLine(rxBuf); rxLen = 0; }
    else if (rxLen < sizeof(rxBuf) - 1) rxBuf[rxLen++] = ch;
  }

  // 电池 & 遥测（20 ms）
  static uint32_t lastD = 0;
  const uint32_t now = millis();
  if (now - lastD >= 20) {
    lastD = now;
    static uint32_t acc = 0; static int n = 0;
    acc += analogReadMilliVolts(PIN_VBAT);
    if (++n >= 5) { vbatMv = (uint32_t)(acc / 5.0f * VBAT_DIV); acc = 0; n = 0; }

    if (Serial.availableForWrite() > 160) Serial.printf("D,%lu,%lu,%d,%.3f,%.3f,%.3f,%.1f,%.1f,%.1f,%d,%d,%d,%d,%d,%lu\n",
                  (unsigned long)now, (unsigned long)vbatMv, digitalRead(PIN_PGOOD),
                  (double)g_ax, (double)g_ay, (double)g_az,
                  (double)g_gr, (double)g_gp, (double)g_gy,
                  (int)g_out[0], (int)g_out[1], (int)g_out[2], (int)g_out[3],
                  g_armed ? 1 : 0, (unsigned long)g_fault);
  }

  // 状态快照（200 ms）：本地 Web Serial 页靠这个刷界面，内容与 /api/state 完全一致
  static uint32_t lastS = 0;
  if (now - lastS >= 200) {
    lastS = now;
    char js[512];
    buildStateJson(js, sizeof(js));
    if (Serial.availableForWrite() > 600) Serial.printf("#S,%s\n", js);
  }
  delay(1);
}
