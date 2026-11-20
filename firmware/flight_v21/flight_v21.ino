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
#include <esp_wifi.h>
#include "mbedtls/sha1.h"
#include "mbedtls/base64.h"
#include <Arduino.h>
#include <esp_mac.h>
#include <esp_timer.h>
#include <esp_ota_ops.h>

#define FW_VER "flight-v21"

// ---------------------------------------------------------------- 硬件
static const char* AP_SSID = "DroneBench";
static const uint8_t AP_CH = 6;

static const int PIN_PWM[4] = {4, 5, 6, 7};   // M1..M4
static const int PIN_SDA = 10, PIN_SCL = 11;
static const int PIN_VBAT = 2, PIN_PGOOD = 9;
static const uint8_t IMU = 0x6B;

static const float VBAT_DIV  = 2.0f;
static const int   DUTY_MAX  = 1000;
static const uint32_t CTRL_HZ = 1000;           // 控制任务频率
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

static float g_rot = 0.0f;
static float g_bR = 0.0f, g_bP = 0.0f;   // 重心补偿（‰），起飞前就加上
static int   g_crate = 500, g_lp2 = 1, g_cap = 1;   // v18 控制频率 / 二阶滤波 / 顶满整体平移
static float g_dfc = 0.0f;                          // v18 D 项低通 Hz（0 = 不滤）
static float g_kff = 0.4f;
static float g_irel = 40.0f;   // v20 积分放松阈值（°/s，设定角速度变化这么快时积分停；0 = 关）Betaflight iterm_relax
static float g_ag = 2.0f;      // v20 抗重力：油门变化快时积分最多放大到 1+ag 倍（0 = 关）
static float g_nf = 0.0f;      // v20 陀螺陷波中心频率 Hz（0 = 关），看振动频谱定
static float g_nq = 3.0f;      // v20 陷波 Q（越大越窄）
static float g_mg[4] = {1, 1, 1, 1};                // v19 每个电机推力补偿系数（弱的 >1）                          // v19 杆量前馈（°/s 每 °/s 目标变化率）
static float g_yff = -60.0f;  // v17 偏航前馈（‰，混控 oY 单位），离地后加，存 NVS
static float g_hov = 405.0f;  // 学到的悬停油门（‰，电压补偿前），存 NVS
static float g_ah = 0.0f;     // 定高开关/力度（0 = 关；v13 实测估计会漂，默认关）
static float g_kdz = 300.0f;  // 压上下弹：每 1 m/s 竖直速度（高通）反向 g_kdz ‰（0 = 关）
static float g_tz = 0.8f;     // 竖直速度高通时间常数（s）
static float g_dzmax = 80.0f; // 压弹最多加减多少 ‰
static float g_tramp = 0.3f;  // 起飞油门软启动（s）
static float g_ht = 0.4f;     // 起飞目标高度（m）
static float g_climb = 0.3f;  // 爬升速度（m/s）
static float g_khp = 150.0f, g_khv = 250.0f, g_khi = 60.0f;   // ‰/m、‰/(m/s)、‰/(m·s)
static float g_vz = 0, g_hz = 0, g_htg = 0;   // 竖直速度、高度、目标高度（记录用）
static float g_ya = 3.0f;     // 机头锁定：每偏 1° 往回转 g_ya °/s（0 = 关）
static float g_vh = 8.0f;     // 速度刹车：每 1 m/s 反向倾 g_vh 度（0 = 关）
static float g_ph = 3.0f;     // 回原点：每 1 m 反向倾 g_ph 度
static float g_vleak = 0.3f;  // 推算速度慢慢回零（1/s），防误差越积越大
static float g_hmax = 8.0f;   // 修正角最多几度
static float g_vx = 0, g_vy = 0, g_px = 0, g_py = 0;
static float g_accR = 0, g_accP = 0;   // 只用加计算的倾角（度，减过 trim）：地上 = 倾斜，空中 ≈ 阻力
static float g_apf = 0, g_arf = 0;     // 空中低通后的 tan(加计倾角)
static float g_mu = 0.5f;              // 阻力系数（1/s）：阻力加速度 = μ·速度；负数 = 方向反过来
static float g_dk = 0.0f;              // 推算速度往阻力测速拉的力度（1/s），0 = 关
static float g_vdx = 0, g_vdy = 0;     // 阻力测出的速度（m/s）   // 推算速度（m/s）、位移（m），机体系：x = pitch+ 方向，y = roll+ 方向
static float g_yi = 0.5f;    // 偏航积分（RAM，页面每次连上写入）   // IMU 安装角（度），存 NVS "rot"

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
  prefs.putFloat("rot", g_rot);
  prefs.putFloat("bR", g_bR); prefs.putFloat("bP", g_bP);
  prefs.putFloat("hov", g_hov); prefs.putFloat("yff", g_yff);
  prefs.putFloat("mg1", g_mg[0]); prefs.putFloat("mg2", g_mg[1]); prefs.putFloat("mg3", g_mg[2]); prefs.putFloat("mg4", g_mg[3]);
  prefs.putFloat("irel", g_irel); prefs.putFloat("ag", g_ag); prefs.putFloat("nf", g_nf); prefs.putFloat("nq", g_nq);
  prefs.putFloat("yi", g_yi); prefs.putFloat("ya", g_ya);
  prefs.putFloat("kff", g_kff); prefs.putFloat("kdz", g_kdz); prefs.putFloat("tz", g_tz); prefs.putFloat("dzmax", g_dzmax);
  prefs.putInt("crate", g_crate); prefs.putInt("lp2", g_lp2); prefs.putInt("cap", g_cap); prefs.putFloat("dfc", g_dfc);
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
    g_rot = prefs.getFloat("rot", 0.0f);
    g_bR = prefs.getFloat("bR", 0.0f); g_bP = prefs.getFloat("bP", 0.0f);
    g_hov = prefs.getFloat("hov", g_hov); g_yff = prefs.getFloat("yff", g_yff);
    g_mg[0] = prefs.getFloat("mg1", 1); g_mg[1] = prefs.getFloat("mg2", 1); g_mg[2] = prefs.getFloat("mg3", 1); g_mg[3] = prefs.getFloat("mg4", 1);
    g_irel = prefs.getFloat("irel", g_irel); g_ag = prefs.getFloat("ag", g_ag); g_nf = prefs.getFloat("nf", g_nf); g_nq = prefs.getFloat("nq", g_nq);
    g_yi = prefs.getFloat("yi", g_yi); g_ya = prefs.getFloat("ya", g_ya);
    g_kff = prefs.getFloat("kff", g_kff); g_kdz = prefs.getFloat("kdz", g_kdz); g_tz = prefs.getFloat("tz", g_tz); g_dzmax = prefs.getFloat("dzmax", g_dzmax);
    g_crate = prefs.getInt("crate", g_crate); g_lp2 = prefs.getInt("lp2", g_lp2); g_cap = prefs.getInt("cap", g_cap); g_dfc = prefs.getFloat("dfc", g_dfc);
    if (g_crate != 250 && g_crate != 1000) g_crate = 500;
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
static volatile int      g_spin = 0;
static volatile int g_spinCh = 0;   // v19 推力测试：0 = 四个一起，1~4 = 只转这一个
static bool g_spinRef = false; static float g_spinR0 = 0, g_spinP0 = 0;   // v18 推力测试的参考姿态       // 起转测试 ppm
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
  float irel = 1.0f;   // v20 积分缩放（放松/抗重力）
  Pid() : kp(0), ki(0), kd(0), it(0), pm(0), df(0), out(0) {}
  void reset(float m) { it = 0; pm = m; df = 0; out = 0; }
  float step(float sp, float m, float dt, float imax, float dfc) {
    const float e = sp - m;
    it += ki * irel * e * dt;
    if (it > imax) it = imax; else if (it < -imax) it = -imax;
    // 差分：先限幅再低通。250 Hz 下 (m-pm)/dt 会把陀螺噪声放大 250 倍，
    // 不限制的话 D 项一项就能把输出顶到满舵（这是台面上"输出乱跳"的真凶）。
    float dm = (m - pm) / dt;
    pm = m;
    if (dm >  1500.0f) dm =  1500.0f;
    if (dm < -1500.0f) dm = -1500.0f;
    if (dfc <= 0.0f) df = dm;            // v18：0 = D 项不滤
    else {
      const float rc = 1.0f / (2.0f * PI_F * dfc);
      const float a = dt / (rc + dt);
      df += (dm - df) * a;
    }
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
static volatile bool imuDumpReq = false;
static char imuDump[520] = "";

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
      imuW(0x60, 0xB0);    // v16：软复位（重焊/上电不干净时传感器可能停在没开的状态）
      delay(30);
      for (int w = 0; w < 20 && !(imuR(0x00, &who, 1) && who == 0x05); w++) delay(10);
      imuW(0x02, 0x40);    // CTRL1 地址自增
      imuW(0x03, 0x22);    // v18 ±8 g，ODR 代码 2 = 1793.6 Hz（6 轴）
      imuW(0x04, 0x72);    // v18 ±2048 dps（以前 ±512 翻滚时会顶满），1793.6 Hz
      imuW(0x06, 0x77);    // v18 CTRL5 片内低通：陀螺/加计都开，模式 3 = 13.37% ODR ≈ 240 Hz（抗混叠）
      imuW(0x08, 0x03);    // CTRL7 使能
      delay(120);
      {
        uint8_t c1 = 0, c2 = 0, c3 = 0, c7 = 0, raw[12] = {0};
        imuR(0x02, &c1, 1); imuR(0x03, &c2, 1); imuR(0x04, &c3, 1); imuR(0x08, &c7, 1); imuR(0x35, raw, 12);
        Serial.printf("#IMU,cfg,c1=%02X,c2=%02X,c3=%02X,c7=%02X,raw=%02X%02X,%02X%02X,%02X%02X,%02X%02X\n",
                      c1, c2, c3, c7, raw[1], raw[0], raw[3], raw[2], raw[5], raw[4], raw[7], raw[6]);
        if (c7 != 0x03) { imuW(0x08, 0x03); delay(60); }
      }
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
  g[0] = (float)(int16_t)(b[7]  << 8 | b[6])  / 16.0f;   // ±2048 dps → 16 LSB/dps
  g[1] = (float)(int16_t)(b[9]  << 8 | b[8])  / 16.0f;
  g[2] = (float)(int16_t)(b[11] << 8 | b[10]) / 16.0f;
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
  // v17：先排横滚/俯仰，偏航只用剩下的余量
  static const float sy[4] = {1.0f, -1.0f, -1.0f, 1.0f};
  float c[4];
  c[0] = -oP + oR;
  c[1] = -oP - oR;
  c[2] =  oP + oR;
  c[3] =  oP - oR;
  if (g_cap) {   // v18 Crazyflie 式：最高那路超出上限多少，四路一起减多少
    float hi = -1e9f;
    for (int i = 0; i < 4; i++) if (c[i] > hi) hi = c[i];
    // 11-19 改（底油门下限）：以前没有下限，纠正量特别大时（斜靠在架子限位上、撞了一下）底油门被一路减到 0，
    // 三个电机停转、推力全没。现在底油门最多让出 35%，剩下放不下的纠正量交给后面的按比例缩小。
    if ((float)thr + hi > (float)P.mthr) { const int lo = (int)((float)thr * 0.65f); thr = (int)((float)P.mthr - hi); if (thr < lo) thr = lo; }
  }
  const float roomUp   = (float)(P.mthr - thr);
  const float roomDown = (float)thr;

  // 饱和缩放：顶到上下限时按比例缩小控制量，而不是丢掉控制权
  float mx = 0;
  for (int i = 0; i < 4; i++) if (fabsf(c[i]) > mx) mx = fabsf(c[i]);
  if (mx > 1.0f) {
    float room = roomUp < roomDown ? roomUp : roomDown;
    if (room < 1.0f) room = 1.0f;
    float sc = room / mx;
    if (sc > 1.0f) sc = 1.0f;
    if (sc < 0.12f) sc = 0.12f;              // 保留最低控制权，宁可短暂饱和
    for (int i = 0; i < 4; i++) c[i] *= sc;
  }
  // 偏航：每个电机按被推的方向算还剩多少余量，取最小，超出的偏航先不给
  float lim = 1e9f;
  const float sg = oY >= 0 ? 1.0f : -1.0f;
  for (int i = 0; i < 4; i++) {
    const float rm = (sy[i] * sg > 0) ? (roomUp - c[i]) : (roomDown + c[i]);
    if (rm < lim) lim = rm;
  }
  if (lim < 0) lim = 0;
  float y = oY; if (y > lim) y = lim; if (y < -lim) y = -lim;
  for (int i = 0; i < 4; i++) c[i] += sy[i] * y;
  for (int i = 0; i < 4; i++) {
    int v = thr + (int)lrintf(c[i]);
    if (v < 0) v = 0;
    if (v > P.mthr) v = P.mthr;
    o[i] = v;
  }
}

// ---------------------------------------------------------------- 姿态
static float rollF = 0, pitchF = 0, yawF = 0;
// 两状态卡尔曼：x = [角度, 陀螺零偏]，量测 = 加计角度。单位：度、度/秒
struct Kal {
  float ang = 0, bias = 0, P00 = 0, P01 = 0, P10 = 0, P11 = 0;
  void seed(float a) { ang = a; bias = 0; P00 = P01 = P10 = P11 = 0; }
  float step(float meas, float rate, float dt, float R) {
    const float Qa = 0.001f, Qb = 0.003f;
    ang += dt * (rate - bias);
    P00 += dt * (dt * P11 - P01 - P10 + Qa);
    P01 -= dt * P11; P10 -= dt * P11; P11 += Qb * dt;
    const float S = P00 + R, K0 = P00 / S, K1 = P10 / S;
    float y = meas - ang; if (y > 180) y -= 360; if (y < -180) y += 360;
    ang += K0 * y; bias += K1 * y;
    if (ang > 180) ang -= 360; if (ang < -180) ang += 360;
    if (bias > 20) bias = 20; if (bias < -20) bias = -20;
    const float p00 = P00, p01 = P01;
    P00 -= K0 * p00; P01 -= K0 * p01; P10 -= K1 * p00; P11 -= K1 * p01;
    return ang;
  }
};
static Kal kR, kP;
static bool  attSeeded = false;

static inline float lpfStep(float y, float x, float dt, float fc) {
  const float rc = 1.0f / (2.0f * PI_F * fc);
  return y + (x - y) * (dt / (rc + dt));
}

// ---------------------------------------------------------------- 控制核心
struct BBRec { uint32_t ms; int16_t thr, roll, pitch, gr, gp, gy, spR, spP, oR, oP, o1, o2, o3, o4, itR, itP, stR, stP, oY, az, yaw, vx, vy, px, py, ap, ar, vdx, vdy, vz, hz, htg; };
static const int BBN = 1200;
static BBRec* bb = nullptr;
static volatile int bbHead = 0, bbCount = 0;
static volatile uint32_t bbReason = 0, bbSeq = 0;
static volatile bool bbFrozen = true;
static int g_iThr = 250;
static int g_vcomp = 1;
static float g_angI = 1.5f;   // 角度积分（°/s 每 °·s），只在空中积，地上清零      // 电压补偿开关
static int g_yawProbe = 1;   // 没开防自转时：离地 1.5 s 后轻推一下偏航（+60‰ 0.25 s、-60‰ 0.25 s），黑匣子里看方向
static bool g_probeDone = false;
static volatile bool g_needSave = false;
static float g_probeD = 0;
static float g_oY = 0;   // 输出油门低于这个（‰）时不积分：还在地上时积分会越攒越歪，一离地就猛地一歪
static void bbStart() { bbHead = 0; bbCount = 0; bbReason = 0; bbFrozen = false; g_probeDone = false; yawF = 0; }

static volatile uint8_t g_ctlSrc = 0;   // v18 当前独占控制的通道（0 = 还没定）
static void disarm(uint32_t fault) {
  g_ctlSrc = 0;
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
  float s[3] = {0, 0, 0}, am = 0;
  int n = 0;
  float aa[3], gg[3];
  const int64_t t0 = esp_timer_get_time();
  const int64_t dur = (int64_t)ms * 1000;
  while (esp_timer_get_time() - t0 < dur) {
    const bool ok = imuRead(aa, gg);
    const int64_t el = esp_timer_get_time() - t0;
    if (ok && el > dur * 2 / 5) { s[0] += gg[0]; s[1] += gg[1]; s[2] += gg[2]; am += sqrtf(aa[0] * aa[0] + aa[1] * aa[1] + aa[2] * aa[2]); n++; }
    delay(3);
  }
  if (n < 30) return false;
  am /= (float)n;
  if (am < 0.5f || am > 1.5f) { Serial.printf("#ERR,IMU_ACC,%.2f\n", (double)am); return false; }   // v16：数据全 0 也算坏
  float b[3];
  for (int i = 0; i < 3; i++) {
    b[i] = s[i] / (float)n;
    if (fabsf(b[i]) > 60.0f) return false;
  }
  for (int i = 0; i < 3; i++) gb[i] = b[i];
  g_zeroMs = millis();
  return true;
}

static volatile uint32_t g_tImu = 0, g_tImuMax = 0, g_tStep = 0, g_tStepMax = 0, g_tJit = 0, g_tCtl = 0, g_tCtlMax = 0, g_jHist[6] = {0};
// v18 二阶巴特沃斯低通（Crazyflie lpf2p.c 同一公式）
struct Lpf2p {
  float b0 = 1, b1 = 0, b2 = 0, a1 = 0, a2 = 0, d1 = 0, d2 = 0;
  void init(float fs, float fc) {
    const float fr = fs / fc, ohm = tanf(PI_F / fr), c = 1.0f + 2.0f * cosf(PI_F / 4.0f) * ohm + ohm * ohm;
    b0 = ohm * ohm / c; b1 = 2.0f * b0; b2 = b0;
    a1 = 2.0f * (ohm * ohm - 1.0f) / c; a2 = (1.0f - 2.0f * cosf(PI_F / 4.0f) * ohm + ohm * ohm) / c;
    d1 = d2 = 0;
  }
  void seed(float x) { const float d = x / (b0 + b1 + b2); d1 = d2 = d; }
  float apply(float x) {
    float d0 = x - d1 * a1 - d2 * a2;
    if (!isfinite(d0)) d0 = x;
    const float y = d0 * b0 + d1 * b1 + d2 * b2;
    d2 = d1; d1 = d0; return y;
  }
};
static Lpf2p lfG[3], lfA[3];
struct Notch {   // v20 陷波（RBJ）
  float b0 = 1, b1 = 0, b2 = 0, a1 = 0, a2 = 0, x1 = 0, x2 = 0, y1 = 0, y2 = 0;
  void init(float fs, float f0, float q) {
    const float w = 2.0f * PI_F * f0 / fs, al = sinf(w) / (2.0f * q), c = cosf(w), a0 = 1.0f + al;
    b0 = 1.0f / a0; b1 = -2.0f * c / a0; b2 = 1.0f / a0; a1 = -2.0f * c / a0; a2 = (1.0f - al) / a0;
  }
  float apply(float x) { float y = b0 * x + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2; if (!isfinite(y)) y = x; x2 = x1; x1 = x; y2 = y1; y1 = y; return y; }
};
static Notch ntG[3]; static float ntF = -1, ntQ = -1;
// v20 长黑匣子：PSRAM，每个传感器拍（1 kHz）一条，存最后 LBN 条
struct __attribute__((packed)) LBRec { uint32_t us; int16_t gr[3], gf[3], ac[3], roll, pitch, spR, spP; uint16_t o[4]; int16_t thr; int8_t stR, stP; };
static LBRec* lb = nullptr; static int LBN = 0;
static volatile int lbHead = 0, lbCount = 0; static volatile bool lbOn = false; static volatile uint32_t lbSeq = 0;
static bool g_imuOk = false, g_lfInit = false;
static void sensorTick(float dt) {
  float a[3], gr[3];
  const int64_t ti0 = esp_timer_get_time();
  const bool ok = imuRead(a, gr);
  { static uint32_t acc = 0, n = 0, mx = 0; const uint32_t d = (uint32_t)(esp_timer_get_time() - ti0);
    acc += d; if (d > mx) mx = d; if (++n >= 1000) { g_tImu = acc / n; g_tImuMax = mx; acc = n = mx = 0; } }
  g_imuOk = ok;
  if (!ok) { i2cFail++; g_fault |= F_IMU; return; }
  float graw[3] = {gr[0], gr[1], gr[2]};
  if (g_nf > 1.0f) {
    if (g_nf != ntF || g_nq != ntQ) { for (int i = 0; i < 3; i++) ntG[i].init(1000.0f, g_nf, g_nq); ntF = g_nf; ntQ = g_nq; }
    for (int i = 0; i < 3; i++) gr[i] = ntG[i].apply(gr[i]);
  }
  if (!g_lfInit) {
    for (int i = 0; i < 3; i++) { lfG[i].init(1000.0f, 80.0f); lfA[i].init(1000.0f, 30.0f); lfG[i].seed(gr[i]); lfA[i].seed(a[i]); fa[i] = a[i]; fg[i] = gr[i]; }
    g_lfInit = true;
  }
  if (g_lp2) {   // Crazyflie：陀螺 80 Hz、加计 30 Hz 二阶
    for (int i = 0; i < 3; i++) { fg[i] = lfG[i].apply(gr[i]); fa[i] = lfA[i].apply(a[i]); }
  } else {       // 旧：一阶 60 Hz（同时让二阶滤波跟着走，切换时不跳）
    for (int i = 0; i < 3; i++) { fa[i] = lpfStep(fa[i], a[i], dt, 60.0f); fg[i] = lpfStep(fg[i], gr[i], dt, 60.0f); lfG[i].apply(gr[i]); lfA[i].apply(a[i]); }
  }
  if (lb && lbOn) {
    LBRec& r = lb[lbHead];
    r.us = (uint32_t)esp_timer_get_time();
    for (int i = 0; i < 3; i++) { r.gr[i] = (int16_t)(graw[i] * 10); r.gf[i] = (int16_t)(fg[i] * 10); r.ac[i] = (int16_t)(a[i] * 1000); }
    r.roll = (int16_t)(rollF * 10); r.pitch = (int16_t)(pitchF * 10); r.spR = (int16_t)dbg.spR; r.spP = (int16_t)dbg.spP;
    for (int i = 0; i < 4; i++) r.o[i] = (uint16_t)g_out[i];
    r.thr = (int16_t)dbg.thr; r.stR = (int8_t)(g_stkRoll * 100); r.stP = (int8_t)(g_stkPitch * 100);
    lbHead = (lbHead + 1) % LBN; if (lbCount < LBN) lbCount++;
  }
}
static void controlStep(float dt) {
  const bool ok = g_imuOk;
  if (!ok) { }
  else {
    g_ax = fa[0]; g_ay = fa[1]; g_az = fa[2];

    // 陀螺去零偏
    const float rc = cosf(g_rot / R2D), rs = sinf(g_rot / R2D);
    const float gx0 = fg[0] - gb[0], gy0 = fg[1] - gb[1];
    const float gx = gx0 * rc - gy0 * rs;
    const float gy = gx0 * rs + gy0 * rc;
    const float fax = fa[0] * rc - fa[1] * rs;
    const float fay = fa[0] * rs + fa[1] * rc;
    const float gz = fg[2] - gb[2];

    // 轴映射到机体（sg 同时作用于角度和速率，二者始终自洽）
    const float br = (float)P.sgR * gx;     // 右滚速率为正
    const float bp = (float)P.sgP * gy;     // 抬头速率为正
    const float by = (float)P.sgY * gz;     // 机头右转为正
    g_gr = br; g_gp = bp; g_gy = by;

    // 加计倾角（减掉水平归零的 trim）
    const float amag = sqrtf(fa[0] * fa[0] + fa[1] * fa[1] + fa[2] * fa[2]);
    float rollA  = (float)P.sgR * atan2f(fay, fa[2]) * R2D - P.trimR;
    float pitchA = (float)P.sgP * atan2f(-fax, sqrtf(fay * fay + fa[2] * fa[2])) * R2D - P.trimP;
    g_accR = rollA; g_accP = pitchA;

    if (!attSeeded && amag > 0.4f) {
      rollF = rollA; pitchF = pitchA; kR.seed(rollA); kP.seed(pitchA); attSeeded = true;
    } else {
      // 卡尔曼融合：地上加计可信（R 小）；空中加计主要感到推力、量不准倾斜（R 大 100 倍）；
      // 加速度模长偏离 1 g 太多（碰撞、急加速）时几乎不信加计
      const int outAvg = (g_out[0] + g_out[1] + g_out[2] + g_out[3]) / 4;
      float R = (g_armed && outAvg > g_iThr) ? 3.0f : 0.03f;
      if (fabsf(amag - 1.0f) > 0.15f) R = 30.0f;
      R *= 0.004f / dt;   // v18：R 原来按 250 Hz 调的，频率变了按步长折算，融合快慢不变
      rollF  = kR.step(rollA,  br, dt, R);
      pitchF = kP.step(pitchA, bp, dt, R);
    }
    // 偏航补偿：地上、没解锁、机身静止时，慢慢更新陀螺零偏（三轴），减少偏航角漂移
    if (!g_armed && g_mode == M_IDLE && fabsf(gx0) < 2.0f && fabsf(gy0) < 2.0f && fabsf(gz) < 2.0f && fabsf(amag - 1.0f) < 0.05f) {
      for (int i = 0; i < 3; i++) gb[i] += 0.002f * ((fg[i]) - gb[i]);
    }
    yawF += by * dt;
    if (yawF >  180.0f) yawF -= 360.0f;
    if (yawF < -180.0f) yawF += 360.0f;

    // 加计失效（摔了/剧烈加速）时不要用错误角度污染姿态
    if (amag < 0.4f || amag > 1.8f) { /* 只信陀螺，这帧不融合 —— 上面已做过 */ }

    g_roll = rollF; g_pitch = pitchF; g_yaw = yawF;
  }

  if (imuDumpReq) {
    imuDumpReq = false;
    uint8_t r[0x61]; memset(r, 0xEE, sizeof(r));
    bool ok = true;
    for (int a = 0; a < 0x61; a += 16) { const int n = (0x61 - a) < 16 ? (0x61 - a) : 16; ok &= imuR(a, r + a, n); }
    uint8_t q[12] = {0}; const bool okq = imuR(0x35, q, 12);
    float fa[3] = {0, 0, 0}, fg2[3] = {0, 0, 0}; const bool okf = imuRead(fa, fg2);
    char* p = imuDump; int left = sizeof(imuDump);
    int w = snprintf(p, left, "{\"r35\":\"%d:%02X%02X%02X%02X%02X%02X\",\"rd\":\"%d:%.3f,%.3f,%.3f\",",
                     okq ? 1 : 0, q[0], q[1], q[2], q[3], q[4], q[5], okf ? 1 : 0, (double)fa[0], (double)fa[1], (double)fa[2]);
    p += w; left -= w;
    w = snprintf(p, left, "\"ok\":%d,\"ts\":%u,\"temp\":%.2f,\"st0\":\"%02X\",\"st1\":\"%02X\",\"regs\":\"",
                     ok ? 1 : 0, (unsigned)(r[0x30] | (r[0x31] << 8) | (r[0x32] << 16)),
                     (double)((int8_t)r[0x34] + r[0x33] / 256.0f), r[0x2E], r[0x2F]);
    p += w; left -= w;
    for (int a = 0; a < 0x61 && left > 4; a++) { w = snprintf(p, left, "%02X", r[a]); p += w; left -= w; }
    snprintf(p, left, "\"}");
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
    else {   // v18 推力测试：和开转那一刻的姿态比（倒着粘在秤上也能用），变化超过 25° = 松脱了，停
      if (!g_spinRef) { g_spinRef = true; g_spinR0 = rollF; g_spinP0 = pitchF; }
      float dr = rollF - g_spinR0; if (dr > 180) dr -= 360; if (dr < -180) dr += 360;
      if (g_spin > 250 && (fabsf(dr) > 25.0f || fabsf(pitchF - g_spinP0) > 25.0f)) disarm(F_TILT);
    }
  } else if (g_armed) {
    if ((int32_t)(nowMs - g_lastStick) > 900) disarm(F_LINK);
  }

  // ---- 输出 ----
  if (g_mode == M_SPIN) {
    int o[4] = {g_spin, g_spin, g_spin, g_spin};   // 原始占空比（不过电压补偿），推力测试要的就是这个
    if (g_spinCh >= 1 && g_spinCh <= 4) for (int i = 0; i < 4; i++) if (i != g_spinCh - 1) o[i] = 0;
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

  {
    static uint32_t lowMs = 0;
    if (vbatMv > 1000 && vbatMv < 3250) { if (!lowMs) lowMs = nowMs; } else lowMs = 0;
    if (lowMs) g_fault |= 0x10;   // 低电量警告（不上锁）
    const int thrNow = P.idle + (int)((float)g_thr * (float)(P.mthr - P.idle) / 1000.0f);
    if (lowMs && nowMs - lowMs > 1000 && thrNow < g_iThr) { disarm(F_BATT); return; }   // 只在地上才因低电量上锁
  }

  // ---- 油门与权限（先算，积分项要按权限缩放）----
  // 油门：0 指令 = 怠速，满指令 = 最大油门
  int thr = P.idle + (int)((float)g_thr * (float)(P.mthr - P.idle) / 1000.0f);
  if (thr < 0) thr = 0; if (thr > P.mthr) thr = P.mthr;
  {
    static float az0 = 1.0f, iz = 0, iSum = 0, vzh = 0, tOn = 0; static int iN = 0, thr0 = -1; static bool on = false, wasOn = false;
    const float tilt = cosf(rollF * 0.0174533f) * cosf(pitchF * 0.0174533f);
    if (!g_armed || g_thr == 0) {
      // 地上：竖直加计读数当 1 g（机身静止时慢慢跟）
      if (fabsf(g_gr) < 5.0f && fabsf(g_gp) < 5.0f && g_az > 0.5f) az0 += (g_az * tilt - az0) * dt / 0.5f;
      if (wasOn && iN > 250) {   // 这次定高跑过：按积分平均值修正悬停油门
        g_hov += 0.6f * (iSum / (float)iN);
        if (g_hov < 250) g_hov = 250; if (g_hov > 650) g_hov = 650;
        g_needSave = true;
      }
      g_vz = g_hz = g_htg = 0; iz = iSum = 0; iN = 0; thr0 = -1; on = wasOn = false; vzh = 0; tOn = 0;
    } else {
      if (thr0 < 0) thr0 = g_thr;
      const bool moved = abs(g_thr - thr0) > 30;
      const float av = (g_az * tilt / az0 - 1.0f) * 9.81f;
      g_vz += av * dt; g_hz += g_vz * dt;
      vzh += (av - vzh / (g_tz > 0.05f ? g_tz : 0.05f)) * dt;   // 高通竖直速度：只看快的上下动
      tOn += dt;
      if (g_hz < 0) { g_hz = 0; if (g_vz < 0) g_vz = 0; }
      if (moved) on = false;
      else if (!wasOn && g_ah > 0.001f) on = true;      // 只在起飞那一下接管
      if (on) {
        wasOn = true;
        float vt = g_climb, rem = g_ht - g_htg;
        if (vt > 1.5f * rem) vt = 1.5f * rem; if (vt < 0) vt = 0;
        g_htg += vt * dt;
        const float eh = g_htg - g_hz;
        iz += eh * dt;
        const float imax = 150.0f / (g_khi > 1.0f ? g_khi : 1.0f);
        if (iz > imax) iz = imax; if (iz < -imax) iz = -imax;
        float d = g_ah * (g_khp * eh + g_khv * (vt - g_vz) + g_khi * iz);
        if (d > 200) d = 200; if (d < -200) d = -200;
        thr = (int)(g_hov + d);
        if (thr < 0) thr = 0; if (thr > P.mthr) thr = P.mthr;
        if (g_htg >= 0.98f * g_ht) { iSum += g_ah * g_khi * iz; iN++; }
      } else {
        // 软启动：起飞那一下 g_tramp 秒内从 0 爬到指令油门
        { static int lc = -1; if (lc < 0 || abs((int)g_thr - lc) >= 5) vzh = 0; lc = (int)g_thr; }   // 11-19 飞手改油门 → 不抵消
        if (g_tramp > 0.01f && tOn < g_tramp) thr = (int)(thr * (tOn / g_tramp));
        // 压上下弹（离地后才有意义；油门太低时不加）
        if (thr >= g_iThr && tOn > 0.1f) {
          float dz = -g_kdz * vzh;
          if (dz > g_dzmax) dz = g_dzmax; if (dz < -g_dzmax) dz = -g_dzmax;
          thr += (int)dz; if (thr < 0) thr = 0; if (thr > P.mthr) thr = P.mthr;
          g_htg = dz / 100.0f;      // 记录：压弹量（‰/100）
        }
        g_vz = vzh;                 // 记录：高通竖直速度
      }
    }
  }
  float vcc = 1.0f;   // v17：同一个系数也乘到纠正量上
  if (g_vcomp && vbatMv > 3000 && thr > 0) {
    float vc = 3950.0f / (float)vbatMv;
    if (vc < 0.92f) vc = 0.92f; if (vc > 1.2f) vc = 1.2f;
    thr = (int)(thr * vc); if (thr > P.mthr) thr = P.mthr;
    vcc = vc;
  }

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
  if (thr < g_iThr) {
    // 解锁且推了油门：积分放成重心补偿；没推油门（电机停）就清零
    pidR.it = (thr > 0) ? g_bR : 0.0f; pidP.it = (thr > 0) ? g_bP : 0.0f;
  }
  pidR.kp = P.rp; pidR.ki = P.ri * kI; pidR.kd = P.rd;
  pidP.kp = P.pp; pidP.ki = P.pi * kI; pidP.kd = P.pd;
  pidY.kp = P.yp; pidY.ki = g_yi * kI; pidY.kd = 0;
  if (thr < g_iThr) pidY.it = 0;

  // 角度外环 → 角速度设定
  float tRoll  = g_stkRoll  * P.mang;
  float tPitch = g_stkPitch * P.mang;
  {
    const bool stick = fabsf(g_stkRoll) > 0.01f || fabsf(g_stkPitch) > 0.01f;
    if (thr >= g_iThr) {   // 阻力测速：空中就算（不管开没开），方便记录
      g_apf += (tanf(g_accP * 0.0174533f) - g_apf) * dt / 0.25f;
      g_arf += (tanf(g_accR * 0.0174533f) - g_arf) * dt / 0.25f;
      const float mu = (fabsf(g_mu) < 0.05f) ? 0.05f : g_mu;
      g_vdx = g_apf * 9.81f / mu; g_vdy = g_arf * 9.81f / mu;
    } else { g_apf = g_arf = 0; g_vdx = g_vdy = 0; }
    if (thr >= g_iThr && !stick && g_vh > 0.001f) {
      g_vx += (9.81f * tanf(pitchF * 0.0174533f) - g_vleak * g_vx + g_dk * (g_vdx - g_vx)) * dt;
      g_vy += (9.81f * tanf(rollF  * 0.0174533f) - g_vleak * g_vy + g_dk * (g_vdy - g_vy)) * dt;
      g_px += g_vx * dt; g_py += g_vy * dt;
      if (g_px > 1.5f) g_px = 1.5f; if (g_px < -1.5f) g_px = -1.5f;
      if (g_py > 1.5f) g_py = 1.5f; if (g_py < -1.5f) g_py = -1.5f;
      float cP = -(g_vh * g_vx + g_ph * g_px), cR = -(g_vh * g_vy + g_ph * g_py);
      if (cP > g_hmax) cP = g_hmax; if (cP < -g_hmax) cP = -g_hmax;
      if (cR > g_hmax) cR = g_hmax; if (cR < -g_hmax) cR = -g_hmax;
      tPitch += cP; tRoll += cR;
    } else { g_vx = g_vy = g_px = g_py = 0; }
  }
  float spR = P.angP * (tRoll  - rollF);
  float spP = P.angP * (tPitch - pitchF);
  {
    static float aiR = 0, aiP = 0;
    if (thr >= g_iThr && g_angI > 0.001f) {
      aiR += (tRoll - rollF) * dt; aiP += (tPitch - pitchF) * dt;
      const float lim = 30.0f / g_angI;
      if (aiR > lim) aiR = lim; if (aiR < -lim) aiR = -lim;
      if (aiP > lim) aiP = lim; if (aiP < -lim) aiP = -lim;
    } else { aiR = 0; aiP = 0; }
    spR += g_angI * aiR; spP += g_angI * aiP;
  }
  {
    static float fR = 0, fP = 0;
    const float tau = 0.08f, a = dt / (tau + dt);
    float ffR = g_kff * (tRoll - fR) / tau, ffP = g_kff * (tPitch - fP) / tau;
    fR += (tRoll - fR) * a; fP += (tPitch - fP) * a;
    if (thr < g_iThr) { ffR = ffP = 0; }
    if (ffR > 150) ffR = 150; if (ffR < -150) ffR = -150;
    if (ffP > 150) ffP = 150; if (ffP < -150) ffP = -150;
    spR += ffR; spP += ffP;
  }
  float spY = g_stkYaw * 120.0f;
  {
    static float yref = 0;
    if (thr < g_iThr || fabsf(g_stkYaw) > 0.01f || g_ya < 0.001f) yref = yawF;   // 地上 / 手动转向时跟着走
    else {
      float e = yawF - yref; if (e > 180) e -= 360; if (e < -180) e += 360;
      float c = -g_ya * e; if (c > 60) c = 60; if (c < -60) c = -60;
      spY += c;
    }
  }
  if (spR >  P.trMax) spR =  P.trMax; if (spR < -P.trMax) spR = -P.trMax;
  if (spP >  P.trMax) spP =  P.trMax; if (spP < -P.trMax) spP = -P.trMax;

  // 角速度内环
  {
    static float lpR = 0, lpP = 0, thrL = 0;
    const float a = dt / (1.0f / (2.0f * PI_F * 15.0f) + dt);
    lpR += (spR - lpR) * a; lpP += (spP - lpP) * a;
    float fR = 1.0f, fP = 1.0f;
    if (g_irel > 1.0f) { fR = 1.0f - fabsf(spR - lpR) / g_irel; fP = 1.0f - fabsf(spP - lpP) / g_irel; if (fR < 0) fR = 0; if (fP < 0) fP = 0; }
    thrL += ((float)thr - thrL) * (dt / (0.1f + dt));
    float ag = 1.0f;
    if (g_ag > 0.01f) { float x = fabsf((float)thr - thrL) / 80.0f; if (x > 1) x = 1; ag = 1.0f + g_ag * x; }
    pidR.irel = fR * ag; pidP.irel = fP * ag;
  }
  const float oR = pidR.step(spR, g_gr, dt, IMAX, g_dfc);
  const float oP = pidP.step(spP, g_gp, dt, IMAX, g_dfc);
  float oY = (P.yp > 0.001f) ? pidY.step(spY, g_gy, dt, IMAX, g_dfc) : 0.0f;
  if (P.yp > 0.001f && thr >= g_iThr) oY += g_yff;   // v17 偏航前馈
  {
    static uint32_t airMs = 0;
    if (thr >= g_iThr + 60) { if (!airMs) airMs = nowMs; } else airMs = 0;
    if (g_yawProbe && P.yp < 0.001f && !g_probeDone && airMs) {
      const uint32_t ta = nowMs - airMs;
      static float gy0p = 0, gy1p = 0;
      if (ta <= 1500) gy0p = g_gy;
      if (ta > 1500 && ta <= 1800) { oY = 90; gy1p = g_gy; }
      else if (ta > 1800 && ta <= 2100) oY = -90;
      else if (ta > 2100) {
        g_probeDone = true;
        // 推 +60 时偏航角速度应上升、推 -60 时应下降；本身的自转加速度两段一样，相减抵掉
        const float d = (gy1p - gy0p) - (g_gy - gy1p);
        g_probeD = d;
        if (fabsf(d) > 8.0f) {
          if (d < 0) P.sgY = -P.sgY;
          P.yp = 0.8f; g_yi = 0.5f;
          pidY.reset(0);
          g_needSave = true;
        }
      }
    }
  }
  g_oY = oY;

  // X 型混控 + 饱和缩放（控制端与自检共用同一个函数，避免"测的不是跑的那套"）
  dbg.spR = spR; dbg.spP = spP; dbg.oR = oR; dbg.oP = oP; dbg.oY = oY;
  dbg.thr = (float)thr; dbg.auth = k;

  int o[4];
  mixOutput(thr, oR * k * vcc, oP * k * vcc, oY * k * vcc, o);
  for (int i = 0; i < 4; i++) { int v = (int)lrintf((float)o[i] * g_mg[i]); if (v > P.mthr) v = P.mthr; if (v < 0) v = 0; o[i] = v; }
  writeMotors(o);
  static int bbDec = 0;
  if (bb && !bbFrozen && ++bbDec >= g_crate / 250) {
    bbDec = 0;
    BBRec& r = bb[bbHead];
    r.ms = millis(); r.thr = thr;
    r.roll = (int16_t)(rollF * 10); r.pitch = (int16_t)(pitchF * 10);
    r.gr = (int16_t)(g_gr * 10); r.gp = (int16_t)(g_gp * 10); r.gy = (int16_t)(g_gy * 10);
    r.spR = (int16_t)spR; r.spP = (int16_t)spP; r.oR = (int16_t)(oR * k); r.oP = (int16_t)(oP * k);
    r.o1 = o[0]; r.o2 = o[1]; r.o3 = o[2]; r.o4 = o[3];
    r.itR = (int16_t)pidR.it; r.itP = (int16_t)pidP.it;
    r.stR = (int16_t)(g_stkRoll * 100); r.stP = (int16_t)(g_stkPitch * 100);
    r.oY = (int16_t)(oY * k); r.az = (int16_t)(g_az * 1000); r.yaw = (int16_t)(yawF * 10);
    r.vx = (int16_t)(g_vx * 100); r.vy = (int16_t)(g_vy * 100); r.px = (int16_t)(g_px * 100); r.py = (int16_t)(g_py * 100);
    r.ap = (int16_t)(tanf(g_accP * 0.0174533f) * 10000); r.ar = (int16_t)(tanf(g_accR * 0.0174533f) * 10000);
    r.vdx = (int16_t)(g_vdx * 100); r.vdy = (int16_t)(g_vdy * 100);
    r.vz = (int16_t)(g_vz * 100); r.hz = (int16_t)(g_hz * 100); r.htg = (int16_t)(g_htg * 100);
    bbHead = (bbHead + 1) % BBN;
    if (bbCount < BBN) bbCount++;
  }
}

// 控制任务：core 1，250 Hz。WiFi 在 core 0，互不抢时间。
TaskHandle_t g_ctrlH = nullptr;
static void ctrlTickCb(void*) { if (g_ctrlH) xTaskNotifyGive(g_ctrlH); }
static void ctrlTask(void*) {
  g_ctrlH = xTaskGetCurrentTaskHandle();
  esp_timer_handle_t th = nullptr;
  esp_timer_create_args_t ta = {};
  ta.callback = ctrlTickCb; ta.name = "ctl";
  esp_timer_create(&ta, &th);
  esp_timer_start_periodic(th, 1000000 / CTRL_HZ);
  int64_t prev = esp_timer_get_time(), ctlPrev = prev;
  uint32_t hzAcc = 0, tick = 0;
  int64_t hzT0 = prev;

  for (;;) {
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(5));
    const int64_t now = esp_timer_get_time();
    float dt = (float)(now - prev) * 1e-6f;
    prev = now;
    if (dt < 0.0002f) dt = 0.0002f;
    if (dt > 0.0200f) dt = 0.0200f;

    sensorTick(dt);
    {
      static bool act0 = false; static uint32_t stopAt = 0;
      const bool act = g_armed || g_mode == M_SPIN;
      if (act && !act0) { lbHead = 0; lbCount = 0; lbOn = true; stopAt = 0; }
      if (!act && act0) stopAt = millis() + 500;
      if (stopAt && (int32_t)(millis() - stopAt) > 0) { lbOn = false; stopAt = 0; lbSeq++; }
      act0 = act;
    }
    const uint32_t div = 1000u / (uint32_t)g_crate;
    if (++tick >= div) {
      tick = 0;
      float cdt = (float)(now - ctlPrev) * 1e-6f; ctlPrev = now;
      if (cdt < 0.0005f) cdt = 0.0005f; if (cdt > 0.0200f) cdt = 0.0200f;
      { const int64_t c0 = esp_timer_get_time(); controlStep(cdt);
        static uint32_t ca = 0, cn = 0, cm = 0; const uint32_t cd = (uint32_t)(esp_timer_get_time() - c0);
        ca += cd; if (cd > cm) cm = cd; if (++cn >= 250) { g_tCtl = ca / cn; g_tCtlMax = cm; ca = cn = cm = 0; } }
    }
    { static uint32_t acc = 0, n = 0, mx = 0, jm = 0;
      const uint32_t d = (uint32_t)(esp_timer_get_time() - now);
      const uint32_t j = (uint32_t)fabsf(dt * 1e6f - 1e6f / CTRL_HZ);
      acc += d; if (d > mx) mx = d; if (j > jm) jm = j;
      g_jHist[j < 20 ? 0 : j < 50 ? 1 : j < 100 ? 2 : j < 200 ? 3 : j < 500 ? 4 : 5]++;   // 节拍偏差分布（us）
      if (++n >= 250) { g_tStep = acc / n; g_tStepMax = mx; g_tJit = jm; acc = n = mx = jm = 0; } }

    hzAcc++;
    if (now - hzT0 >= 1000000) {
      g_loopHz = hzAcc;
      hzAcc = 0;
      hzT0 = now;
    }
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
static volatile int g_apRssi = 0;   // 热点上连着的设备里最弱的信号（dBm，0 = 没人）
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
  {
    wifi_sta_list_t sl; int m = 0;
    if (esp_wifi_ap_get_sta_list(&sl) == ESP_OK)
      for (int i = 0; i < sl.num; i++) if (m == 0 || sl.sta[i].rssi < m) m = sl.sta[i].rssi;
    g_apRssi = m;
  }
  // v16：解锁中、或热点上有设备在用时不去连路由器（连接/扫描会让热点换信道、掉线）
  if (!up && netSsid.length() && now - netTry > 20000 && !g_armed && WiFi.softAPgetStationNum() == 0) netBegin();
}
// 电脑每 40 ms 发一包：U,<seq>,<live>,<thr>,<r>,<p>,<y>
// live=1 且已解锁 → 更新油门/杆位并喂心跳；不管怎样都回一包遥测：
// T,<seq>,<vb>,<arm>,<fault>,<roll>,<pitch>,<gr>,<gp>,<gy>,<o1>,<o2>,<o3>,<o4>,<ax>,<ay>,<az>,<rssi>,<pg>
static int telem(char* o, size_t n, unsigned seq, bool ap) {
  return snprintf(o, n, "T,%u,%lu,%d,%lu,%.1f,%.1f,%.1f,%.1f,%.1f,%d,%d,%d,%d,%.2f,%.2f,%.2f,%d,%d,%.0f",
                  seq, (unsigned long)vbatMv, g_armed ? 1 : 0, (unsigned long)(g_fault | (imuOK ? 0 : F_IMU)),
                  (double)g_roll, (double)g_pitch, (double)g_gr, (double)g_gp, (double)g_gy,
                  (int)g_out[0], (int)g_out[1], (int)g_out[2], (int)g_out[3],
                  (double)g_ax, (double)g_ay, (double)g_az, ap ? (int)g_apRssi : (int)WiFi.RSSI(),
                  digitalRead(PIN_PGOOD), (double)g_yaw);
}
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
        if (live && g_armed && (g_ctlSrc == 0 || g_ctlSrc == 1)) {
          g_ctlSrc = 1;
          if (t < 0) t = 0; if (t > 1000) t = 1000;
          g_thr = t;
          g_stkRoll  = r < -1 ? -1 : (r > 1 ? 1 : r);
          g_stkPitch = p < -1 ? -1 : (p > 1 ? 1 : p);
          g_stkYaw   = y < -1 ? -1 : (y > 1 ? 1 : y);
          g_lastStick = millis();
        }
        { const IPAddress ri = udp.remoteIP(); telem(o, sizeof(o), seq, ri[0] == 192 && ri[1] == 168 && ri[2] == 4); }
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

// ---------------------------------------------------------------- 手机遥控通道（v16）
// 手机网页不能发 UDP → WebSocket（端口 81，core 0 独立任务，不受网页服务卡顿影响）。
// 收 "U,seq,live,thr,r,p,y"（和 UDP 一样）回 T 遥测；"X" 急停。同时只认一个设备：
// 旧连接还活着（1 s 内有包）就拒绝新连接（回 "B" = 忙）；旧的没动静了才让新的接管（刷新页面 / 换手机）。
static WiFiServer wsSrv(81);
static bool wsSend(WiFiClient& c, uint8_t op, const char* s, size_t n) {
  uint8_t f[260]; size_t h;
  if (n > 250) n = 250;
  f[0] = 0x80 | op;
  if (n < 126) { f[1] = (uint8_t)n; h = 2; } else { f[1] = 126; f[2] = 0; f[3] = (uint8_t)n; h = 4; }
  memcpy(f + h, s, n);
  return c.write(f, h + n) == h + n;
}
static bool wsHandshake(WiFiClient& c) {
  char req[1200]; size_t n = 0; const uint32_t t0 = millis();
  req[0] = 0;
  while (millis() - t0 < 1000) {
    while (c.available() && n < sizeof(req) - 1) req[n++] = (char)c.read();
    req[n] = 0;
    if (strstr(req, "\r\n\r\n")) break;
    if (n >= sizeof(req) - 1 || !c.connected()) return false;
    vTaskDelay(pdMS_TO_TICKS(3));
  }
  const char* k = nullptr;
  for (const char* p = req; *p; p++) if (!strncasecmp(p, "Sec-WebSocket-Key:", 18)) { k = p + 18; break; }
  if (!k) return false;
  while (*k == ' ') k++;
  char key[100]; size_t kl = 0;
  while (k[kl] && k[kl] != '\r' && k[kl] != '\n' && kl < 60) { key[kl] = k[kl]; kl++; }
  memcpy(key + kl, "258EAFA5-E914-47DA-95CA-C5AB0DC85B11", 36); kl += 36;
  unsigned char sha[20], acc[40]; size_t al = 0;
  mbedtls_sha1((const unsigned char*)key, kl, sha);
  mbedtls_base64_encode(acc, sizeof(acc) - 1, &al, sha, 20); acc[al] = 0;
  char resp[200];
  const int rl = snprintf(resp, sizeof(resp),
    "HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Accept: %s\r\n\r\n", acc);
  c.setNoDelay(true);
  return c.write((const uint8_t*)resp, rl) == (size_t)rl;
}
static void wsTask(void*) {
  WiFiClient cli; bool on = false;
  static uint8_t rx[320]; size_t rn = 0; uint32_t last = 0;
  char o[240], p[208];
  for (;;) {
    WiFiClient nc = wsSrv.accept();
    if (nc) {
      if (on && millis() - last < 1000) {
        // 已有设备在控制（1 s 内有包）：不顶掉它。没解锁时告诉后来者“忙”，解锁中直接关（握手会占住任务）
        if (!g_armed && wsHandshake(nc)) wsSend(nc, 1, "B", 1);
        nc.stop();
      }
      else if (wsHandshake(nc)) { if (on) cli.stop(); cli = nc; on = true; rn = 0; last = millis(); }
      else nc.stop();
    }
    if (on && (!cli.connected() || millis() - last > 3000)) { cli.stop(); on = false; rn = 0; }
    if (on) {
      int av = cli.available();
      if (av > 0 && rn < sizeof(rx)) { int g = cli.read(rx + rn, sizeof(rx) - rn); if (g > 0) rn += g; }
      while (on && rn >= 2) {
        const uint8_t op = rx[0] & 0x0F; const bool mk = rx[1] & 0x80;
        size_t len = rx[1] & 0x7F, h = 2;
        if (len == 126) { if (rn < 4) break; len = ((size_t)rx[2] << 8) | rx[3]; h = 4; }
        if (len > 200 || (rx[1] & 0x7F) == 127) { cli.stop(); on = false; rn = 0; break; }
        const size_t mh = h + (mk ? 4 : 0);
        if (rn < mh + len) break;
        for (size_t i = 0; i < len; i++) p[i] = (char)(rx[mh + i] ^ (mk ? rx[h + (i & 3)] : 0));
        p[len] = 0;
        memmove(rx, rx + mh + len, rn - mh - len); rn -= mh + len;
        last = millis();
        if (op == 8) { wsSend(cli, 8, "", 0); cli.stop(); on = false; rn = 0; break; }
        if (op == 9) { wsSend(cli, 10, p, len); continue; }
        if (op != 1) continue;
        unsigned seq = 0; int live = 0, t = 0; float r = 0, pp = 0, y = 0;
        if (p[0] == 'X') {
          disarm(0);
          for (int k = 0; k < 4; k++) g_jogPm[k] = 0;
          g_spin = 0;
        } else if (p[0] == 'U' && sscanf(p, "U,%u,%d,%d,%f,%f,%f", &seq, &live, &t, &r, &pp, &y) == 6) {
          if (live && g_armed && (g_ctlSrc == 0 || g_ctlSrc == 2)) {
            g_ctlSrc = 2;
            if (t < 0) t = 0; if (t > 1000) t = 1000;
            g_thr = t;
            g_stkRoll  = r < -1 ? -1 : (r > 1 ? 1 : r);
            g_stkPitch = pp < -1 ? -1 : (pp > 1 ? 1 : pp);
            g_stkYaw   = y < -1 ? -1 : (y > 1 ? 1 : y);
            g_lastStick = millis();
          }
          const int ol = telem(o, sizeof(o), seq, true);
          if (!wsSend(cli, 1, o, ol > 0 ? ol : 0)) { cli.stop(); on = false; rn = 0; }
        }
      }
      if (on && rn >= sizeof(rx)) { cli.stop(); on = false; rn = 0; }
    }
    vTaskDelay(pdMS_TO_TICKS(2));
  }
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
           "\"hz\":%lu,\"dt\":%.1f,\"f\":%lu,\"imu\":%d,\"tm\":[%lu,%lu,%lu,%lu,%lu]}",
           (unsigned long)vbatMv, g_armed ? 1 : 0, (int)g_mode, (int)g_thr, (int)g_spin,
           (int)g_out[0], (int)g_out[1], (int)g_out[2], (int)g_out[3],
           (double)g_roll, (double)g_pitch, (double)g_yaw,
           (double)g_gr, (double)g_gp, (double)g_gy,
           (double)g_ax, (double)g_ay, (double)g_az,
           (unsigned long)g_loopHz, g_loopHz ? (1000.0 / (double)g_loopHz) : 0.0,
           (unsigned long)g_fault, imuOK ? 1 : 0,
           (unsigned long)g_tImu, (unsigned long)g_tImuMax, (unsigned long)g_tStep, (unsigned long)g_tStepMax, (unsigned long)g_tJit);
}

extern TaskHandle_t g_ctrlH;
static void buildPerfJson(char* j, size_t n) {
  snprintf(j, n,
    "{\"fw\":\"%s\",\"up\":%lu,\"mhz\":%lu,\"hz\":%lu,\"crate\":%d,"
    "\"tImu\":%lu,\"tImuMax\":%lu,\"tCtl\":%lu,\"tCtlMax\":%lu,\"tStep\":%lu,\"tStepMax\":%lu,\"jit\":%lu,"
    "\"jh\":[%lu,%lu,%lu,%lu,%lu,%lu],\"i2cFail\":%lu,"
    "\"heap\":%lu,\"heapMin\":%lu,\"heapTot\":%lu,\"ps\":%lu,\"psTot\":%lu,"
    "\"sketch\":%lu,\"sketchFree\":%lu,\"stkCtl\":%lu,\"stkLoop\":%lu,\"lbN\":%d,\"rssi\":%d}",
    FW_VER, (unsigned long)(millis() / 1000), (unsigned long)getCpuFrequencyMhz(), (unsigned long)g_loopHz, (int)g_crate,
    (unsigned long)g_tImu, (unsigned long)g_tImuMax, (unsigned long)g_tCtl, (unsigned long)g_tCtlMax,
    (unsigned long)g_tStep, (unsigned long)g_tStepMax, (unsigned long)g_tJit,
    (unsigned long)g_jHist[0], (unsigned long)g_jHist[1], (unsigned long)g_jHist[2], (unsigned long)g_jHist[3], (unsigned long)g_jHist[4], (unsigned long)g_jHist[5],
    (unsigned long)i2cFail,
    (unsigned long)ESP.getFreeHeap(), (unsigned long)ESP.getMinFreeHeap(), (unsigned long)ESP.getHeapSize(),
    (unsigned long)ESP.getFreePsram(), (unsigned long)ESP.getPsramSize(),
    (unsigned long)ESP.getSketchSize(), (unsigned long)ESP.getFreeSketchSpace(),
    (unsigned long)(g_ctrlH ? uxTaskGetStackHighWaterMark(g_ctrlH) : 0), (unsigned long)uxTaskGetStackHighWaterMark(NULL),
    LBN, (int)WiFi.RSSI());
}

static void buildParamsJson(char* j, size_t n) {
  snprintf(j, n,
           "{\"angP\":%.2f,\"rp\":%.3f,\"ri\":%.3f,\"rd\":%.4f,"
           "\"pp\":%.3f,\"pi\":%.3f,\"pd\":%.4f,\"yp\":%.2f,"
           "\"idle\":%d,\"mthr\":%d,\"mang\":%.0f,\"trMax\":%.0f,"
           "\"trimR\":%.2f,\"trimP\":%.2f,"
           "\"sgR\":%d,\"sgP\":%d,\"sgY\":%d,\"rot\":%.1f,\"bR\":%.1f,\"bP\":%.1f,\"hov\":%.1f,\"yff\":%.1f,\"crate\":%d,\"lp2\":%d,\"cap\":%d,\"dfc\":%.1f,\"kff\":%.2f,\"kdz\":%.0f,\"tz\":%.2f,\"dzmax\":%.0f,\"mg1\":%.3f,\"mg2\":%.3f,\"mg3\":%.3f,\"mg4\":%.3f,\"yi\":%.2f,\"ya\":%.1f,\"irel\":%.0f,\"ag\":%.1f,\"nf\":%.0f,\"nq\":%.1f}",
           (double)P.angP,
           (double)P.rp, (double)P.ri, (double)P.rd,
           (double)P.pp, (double)P.pi, (double)P.pd,
           (double)P.yp, P.idle, P.mthr, (double)P.mang, (double)P.trMax,
           (double)P.trimR, (double)P.trimP,
           P.sgR, P.sgP, P.sgY, (double)g_rot, (double)g_bR, (double)g_bP, (double)g_hov, (double)g_yff, g_crate, g_lp2, g_cap, (double)g_dfc, (double)g_kff, (double)g_kdz, (double)g_tz, (double)g_dzmax, (double)g_mg[0], (double)g_mg[1], (double)g_mg[2], (double)g_mg[3], (double)g_yi, (double)g_ya, (double)g_irel, (double)g_ag, (double)g_nf, (double)g_nq);
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
static float yawJog(int i, int pm, int ms) {
  jogStart(i, pm, ms);
  float s = 0; const uint32_t t0 = millis();
  while (millis() - t0 < (uint32_t)ms) { s += g_gy * 0.004f; delay(4); }
  jogStart(i, 0, 100);
  delay(500);
  return s;
}
static void handleYawTest() {
  if (g_armed) { server.send(409, "text/plain", "armed"); return; }
  int pm = server.arg("pm").toInt(); if (pm <= 0) pm = 250; if (pm > 300) pm = 300;
  int ms = server.arg("ms").toInt(); if (ms <= 0) ms = 700; if (ms > 1500) ms = 1500;
  const float a1 = yawJog(0, pm, ms);
  const float a2 = yawJog(1, pm, ms);
  const float a3 = yawJog(2, pm, ms);
  const float a4 = yawJog(3, pm, ms);
  char j[140];
  snprintf(j, sizeof(j), "{\"m1\":%.2f,\"m2\":%.2f,\"m3\":%.2f,\"m4\":%.2f,\"sgY\":%d}", a1, a2, a3, a4, P.sgY);
  sendJson(j);
}

static void handlePerf() { char j[760]; buildPerfJson(j, sizeof(j)); server.send(200, "application/json", j); }
static void handleLB() {
  if (!lb || lbOn) { server.send(409, "text/plain", lbOn ? "recording" : "no lb"); return; }
  const int n = lbCount, start = (lbHead - n + LBN) % LBN;
  char head[160];
  snprintf(head, sizeof(head), "#LB v20 seq=%lu n=%d rec=%d crate=%d nf=%.0f\n", (unsigned long)lbSeq, n, (int)sizeof(LBRec), g_crate, (double)g_nf);
  server.setContentLength(strlen(head) + (size_t)n * sizeof(LBRec));
  server.send(200, "application/octet-stream", head);
  int i = 0;
  while (i < n) {
    const int idx = (start + i) % LBN;
    int k = 96; if (k > n - i) k = n - i; if (k > LBN - idx) k = LBN - idx;
    server.sendContent((const char*)&lb[idx], (size_t)k * sizeof(LBRec));
    i += k;
  }
}
static void handleBB() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Cache-Control", "no-store");
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  char head[160];
  snprintf(head, sizeof(head), "# seq=%lu reason=%lu frozen=%d n=%d angP=%.2f rp=%.2f ri=%.3f rd=%.3f iThr=%d yp=%.2f sgY=%d probeD=%.1f\n",
           (unsigned long)bbSeq, (unsigned long)bbReason, bbFrozen ? 1 : 0, bbCount, P.angP, P.rp, P.ri, P.rd, g_iThr, P.yp, P.sgY, (double)g_probeD);
  server.send(200, "text/plain", head);
  server.sendContent("ms,thr,roll,pitch,gr,gp,gy,spR,spP,oR,oP,o1,o2,o3,o4,itR,itP,stR,stP,oY,az,yaw,vx,vy,px,py,ap,ar,vdx,vdy,vz,hz,dz\n");
  if (!bb) return;
  String chunk;
  const int n = bbCount, start = (bbHead - n + BBN) % BBN;
  for (int i = 0; i < n; i++) {
    const BBRec& r = bb[(start + i) % BBN];
    char l[280];
    snprintf(l, sizeof(l), "%lu,%d,%.1f,%.1f,%.1f,%.1f,%.1f,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%.2f,%.2f,%d,%.3f,%.1f,%.2f,%.2f,%.2f,%.2f,%.4f,%.4f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
             (unsigned long)r.ms, r.thr, r.roll / 10.0, r.pitch / 10.0, r.gr / 10.0, r.gp / 10.0, r.gy / 10.0,
             r.spR, r.spP, r.oR, r.oP, r.o1, r.o2, r.o3, r.o4, r.itR, r.itP, r.stR / 100.0, r.stP / 100.0,
             r.oY, r.az / 1000.0, r.yaw / 10.0, r.vx / 100.0, r.vy / 100.0, r.px / 100.0, r.py / 100.0,
             r.ap / 10000.0, r.ar / 10000.0, r.vdx / 100.0, r.vdy / 100.0, r.vz / 100.0, r.hz / 100.0, r.htg / 100.0);
    chunk += l;
    if (chunk.length() > 1800) { server.sendContent(chunk); chunk = ""; }
  }
  if (chunk.length()) server.sendContent(chunk);
}

static bool canArm(String& why) {
  if (!imuOK) { why = "IMU 未就绪"; return false; }
  if (g_mode == M_SPIN) { why = "起转测试中"; return false; }
  if (g_thr > 0) { why = "油门不在 0"; return false; }
  if (fabsf(g_roll) > 45.0f || fabsf(g_pitch) > 45.0f) { why = "机身太歪（倾角需 <45°）"; return false; }
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
  if (g_ctlSrc != 0 && g_ctlSrc != 3) { sendJson("{\"ok\":0,\"why\":\"busy\"}"); return; }
  g_ctlSrc = 3;
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

static void handleHSpin() {   // v18 推力测试（WiFi 版的 H,pm）：最高 950‰，1 秒没续命自停
  if (g_armed) { server.send(409, "text/plain", "armed"); return; }
  int pm = server.arg("pm").toInt();
  { const int ch = server.arg("ch").toInt(); g_spinCh = (ch >= 1 && ch <= 4) ? ch : 0; }
  if (pm < 0) pm = 0; if (pm > 950) pm = 950;
  if (g_mode != M_SPIN) g_spinRef = false;
  g_spin = pm;
  g_mode = pm > 0 ? M_SPIN : M_IDLE;
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

static void handleImuReg() {
  imuDump[0] = 0; imuDumpReq = true;
  for (int i = 0; i < 40 && !imuDump[0]; i++) delay(5);
  sendJson(imuDump[0] ? imuDump : "{\"ok\":0}");
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
  else if (k == "angI") g_angI = v;
  else if (k == "ya") g_ya = v;
  else if (k == "tramp") g_tramp = v;
  else if (k == "dzmax") g_dzmax = v;
  else if (k == "tz") g_tz = v;
  else if (k == "kdz") g_kdz = v;
  else if (k == "khi") g_khi = v;
  else if (k == "khv") g_khv = v;
  else if (k == "khp") g_khp = v;
  else if (k == "climb") g_climb = v;
  else if (k == "ht") g_ht = v;
  else if (k == "ah") g_ah = v;
  else if (k == "hov") g_hov = v;
  else if (k == "mg1") g_mg[0] = (v < 0.7f ? 0.7f : (v > 1.4f ? 1.4f : v));
  else if (k == "mg2") g_mg[1] = (v < 0.7f ? 0.7f : (v > 1.4f ? 1.4f : v));
  else if (k == "mg3") g_mg[2] = (v < 0.7f ? 0.7f : (v > 1.4f ? 1.4f : v));
  else if (k == "mg4") g_mg[3] = (v < 0.7f ? 0.7f : (v > 1.4f ? 1.4f : v));
  else if (k == "irel") g_irel = (v < 0 ? 0 : (v > 500 ? 500 : v));
  else if (k == "ag") g_ag = (v < 0 ? 0 : (v > 10 ? 10 : v));
  else if (k == "nf") g_nf = (v < 0 ? 0 : (v > 450 ? 450 : v));
  else if (k == "nq") g_nq = (v < 0.5f ? 0.5f : (v > 20 ? 20 : v));
  else if (k == "kff") g_kff = (v < 0 ? 0 : (v > 2 ? 2 : v));
  else if (k == "crate") g_crate = (v >= 900 ? 1000 : (v <= 300 ? 250 : 500));
  else if (k == "lp2") g_lp2 = v > 0.5f ? 1 : 0;
  else if (k == "cap") g_cap = v > 0.5f ? 1 : 0;
  else if (k == "dfc") g_dfc = (v < 0 ? 0 : (v > 200 ? 200 : v));
  else if (k == "yff") g_yff = (v > 200 ? 200 : (v < -200 ? -200 : v));
  else if (k == "dk") g_dk = v;
  else if (k == "mu") g_mu = v;
  else if (k == "hmax") g_hmax = v;
  else if (k == "vleak") g_vleak = v;
  else if (k == "ph") g_ph = v;
  else if (k == "vh") g_vh = v;
  else if (k == "bP") g_bP = (v > 80 ? 80 : (v < -80 ? -80 : v));
  else if (k == "bR") g_bR = (v > 80 ? 80 : (v < -80 ? -80 : v));
  else if (k == "yi") g_yi = v;
  else if (k == "yprobe") g_yawProbe = (int)v;
  else if (k == "vcomp") g_vcomp = (int)v;
  else if (k == "rot") { g_rot = v; attSeeded = false; }
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
static const char PHONE_HTML[] PROGMEM = R"HTML(<!DOCTYPE html>
<html lang="zh-CN"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no,viewport-fit=cover">
<meta name="mobile-web-app-capable" content="yes"><meta name="apple-mobile-web-app-capable" content="yes">
<meta name="theme-color" content="#0a111a">
<title>mini 无人机</title>
<style>
:root{--bg:#0a111a;--card:#111c2c;--line:#1e2c40;--tx:#dce8f6;--dim:#7d90a9;--ok:#2fd3c0;--warn:#f2b544;--bad:#ff5a5f}
*{box-sizing:border-box;-webkit-tap-highlight-color:transparent;-webkit-user-select:none;user-select:none;-webkit-touch-callout:none}
html,body{margin:0;height:100%;background:var(--bg);color:var(--tx);font:14px -apple-system,"PingFang SC","Microsoft YaHei",sans-serif;overflow:hidden;touch-action:none;overscroll-behavior:none}
button{font:inherit;color:inherit}
#app{position:fixed;inset:0;display:flex;flex-direction:column;padding:env(safe-area-inset-top) env(safe-area-inset-right) env(safe-area-inset-bottom) env(safe-area-inset-left)}
.top{display:flex;align-items:center;gap:10px;padding:7px 12px;font-size:13px;border-bottom:1px solid var(--line);white-space:nowrap}
.dot{width:9px;height:9px;border-radius:50%;background:var(--bad);flex:none}.dot.on{background:var(--ok)}.dot.weak{background:var(--warn)}
.sig{display:inline-flex;align-items:flex-end;gap:2px;height:13px;vertical-align:-1px}.sig i{width:3px;background:#3d4a58;border-radius:1px}
.sp{flex:1}
.ib{background:#16233a;border:1px solid var(--line);border-radius:9px;padding:5px 10px;font-size:15px;line-height:1}
.main{flex:1;min-height:0;display:grid;grid-template-columns:minmax(0,1fr) minmax(0,1.35fr) minmax(0,1fr);gap:10px;padding:10px}
@media (orientation:portrait){.main{grid-template-columns:minmax(0,1fr) minmax(0,1.3fr);grid-template-rows:auto minmax(0,1fr)}.mid{grid-column:1/3;order:-1}}
@media (max-width:440px){.top{gap:7px;font-size:12px;padding:7px 10px}#conn{display:none}.tele{gap:8px}.mots{width:96px}}
.card{background:var(--card);border:1px solid var(--line);border-radius:16px;min-height:0;min-width:0}
/* 油门 */
.thr{display:flex;flex-direction:column;align-items:center;padding:10px 8px;gap:6px;touch-action:none;position:relative}
.thr.on{border-color:var(--ok)}
.thr .cap{text-align:center}
.yawi{width:80%;height:6px;border-radius:3px;background:#0d1622;position:relative}
.yawi i{position:absolute;top:-3px;left:50%;width:12px;height:12px;margin-left:-6px;border-radius:50%;background:#4a6a90}
.cap{font-size:12px;color:var(--dim)}
.track{flex:1;width:70px;border-radius:14px;background:#0d1622;border:1px solid var(--line);position:relative;overflow:hidden;min-height:90px;touch-action:none}
.fill{position:absolute;left:0;right:0;bottom:0;height:0;background:linear-gradient(#2fd3c0,#17675f)}
.tgt{position:absolute;left:0;right:0;height:3px;bottom:0;background:#fff;opacity:.85}
.tkm{position:absolute;left:0;right:0;border-top:2px dashed var(--warn);bottom:64%}
.tkm b{position:absolute;right:3px;top:-15px;font-size:10px;color:var(--warn);font-weight:400}
.big{font-size:22px;font-weight:700;font-variant-numeric:tabular-nums}
.row{display:flex;gap:6px}
.sb{background:#16233a;border:1px solid var(--line);border-radius:10px;width:38px;height:34px;font-size:18px}
.sb:active,.ib:active{background:#21344f}
/* 中间 */
.mid{display:flex;flex-direction:column;gap:8px;padding:10px}
.st{font-size:17px;font-weight:700;text-align:center}.st.ok{color:var(--ok)}.st.err{color:var(--bad)}.st.warn{color:var(--warn)}
.tele{display:flex;gap:12px;align-items:center;justify-content:center}
.ball{width:58px;height:58px;border-radius:50%;border:1px solid #2b3e58;position:relative;flex:none;overflow:hidden;background:#0d1622}
.ball:before,.ball:after{content:"";position:absolute;background:#2b3e58}.ball:before{left:50%;top:0;bottom:0;width:1px}.ball:after{top:50%;left:0;right:0;height:1px}
.bub{position:absolute;width:12px;height:12px;border-radius:50%;background:var(--ok);left:23px;top:23px}
.nums{font-size:12px;color:var(--dim);line-height:1.6;font-variant-numeric:tabular-nums}.nums b{color:var(--tx);font-weight:600}
.mots{display:grid;grid-template-columns:1fr 1fr;gap:3px 8px;width:110px;font-size:10px;color:var(--dim)}
.mb{height:6px;border-radius:3px;background:#0d1622;overflow:hidden;margin-top:1px}.mb i{display:block;height:100%;width:0;background:var(--ok)}
.btns{display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-top:auto}
.btn{border:none;border-radius:14px;padding:14px 0;font-size:18px;font-weight:700;color:#fff;position:relative;overflow:hidden}
.btn:active{filter:brightness(1.2)}
.go{background:#178a5a}.land{background:#2a5d9f}.stop{grid-column:1/3;background:#c62f35;font-size:20px;padding:15px 0}
.go .pg{position:absolute;left:0;bottom:0;height:4px;width:0;background:#b9ffd9}
.btn small{display:block;font-size:11px;font-weight:400;opacity:.8}
/* 摇杆 */
.joy{display:flex;flex-direction:column;align-items:center;justify-content:center;padding:10px;gap:8px}
.pad{width:min(100%,200px);aspect-ratio:1/1;border-radius:50%;background:radial-gradient(circle,#15263b 0,#0d1622 70%);border:1px solid var(--line);position:relative;touch-action:none;flex:none}
.pad:before{content:"前";position:absolute;top:6px;left:0;right:0;text-align:center;font-size:11px;color:var(--dim)}
.knob{position:absolute;width:40%;height:40%;left:30%;top:30%;border-radius:50%;background:#2a3d57;border:2px solid #4a6a90;pointer-events:none}
.pad.on .knob{background:#2f5f73;border-color:var(--ok)}
.yaw{display:flex;gap:10px}
.yb{background:#16233a;border:1px solid var(--line);border-radius:12px;width:62px;height:38px;font-size:19px}
.yb.on{background:#2f5f73;border-color:var(--ok)}
/* 提示 / 设置 */
.toast{position:fixed;left:50%;top:52px;transform:translateX(-50%);background:#1d2d44;border:1px solid #2f4664;padding:8px 14px;border-radius:10px;font-size:14px;opacity:0;transition:opacity .2s;pointer-events:none;z-index:9;max-width:90vw;text-align:center}
.toast.on{opacity:1}.toast.err{background:#3a1418;border-color:#7a2a30}
.off{position:fixed;left:0;right:0;bottom:0;background:#3a2a10;color:#ffd99a;font-size:13px;padding:8px 12px;text-align:center;z-index:8}
.sheet{position:fixed;inset:0;background:rgba(0,0,0,.55);display:flex;align-items:center;justify-content:center;z-index:10}
.sheet[hidden]{display:none}
.box{background:var(--card);border:1px solid var(--line);border-radius:16px;padding:14px 16px;width:min(94vw,420px);max-height:90vh;overflow:auto;touch-action:pan-y}
.box h3{margin:0 0 10px;font-size:15px}
.box.wide{width:min(96vw,900px)}
.box canvas{width:100%;height:auto;background:#0d1622;border-radius:10px}
.cvm{display:flex;flex-wrap:wrap;gap:6px 14px;font-size:12px;color:var(--dim);padding:6px 0}.cvm b{color:var(--tx)}
.prm{display:flex;flex-direction:column;gap:6px;padding-bottom:6px}
.pr{display:grid;grid-template-columns:72px 38px 1fr 38px;align-items:center;gap:6px;font-size:13px}
.pr b{text-align:center;font-variant-numeric:tabular-nums}.pr i{grid-column:1/-1;font-style:normal;font-size:11px;color:var(--dim);margin-top:-3px}
.sec{border-top:1px solid var(--line);padding:10px 0;display:flex;align-items:center;gap:10px;flex-wrap:wrap}
.sec .lab{flex:1;min-width:120px;font-size:13px}.sec .lab i{display:block;font-style:normal;font-size:11px;color:var(--dim)}
.tpad{display:grid;grid-template-columns:repeat(3,38px);gap:4px}
.tpad span{display:block}
.lk{display:block;padding:10px 12px;border-radius:10px;background:#16233a;border:1px solid var(--line);color:var(--tx);text-decoration:none;font-size:13px;flex:1;text-align:center}
.ver{font-size:11px;color:var(--dim);margin-top:6px}
</style></head><body>
<div id="app">
  <div class="top">
    <span class="dot" id="dot"></span><span id="conn">连接中…</span>
    <span id="bat">🔋 --</span>
    <span id="rs"></span>
    <span class="sp"></span>
    <button class="ib" id="fs" title="全屏">⛶</button>
    <button class="ib" id="cfg" title="设置">⚙</button>
  </div>
  <div class="main">
    <div class="card thr">
      <div class="cap">左手：上下拖 = 油门 · 左右 = 转向</div>
      <div class="yawi" id="yawi"><i></i></div>
      <div class="track" id="track"><div class="fill" id="fill"></div><div class="tkm" id="tkm"><b>起飞</b></div><div class="tgt" id="tgt"></div></div>
      <div class="big" id="tpct">0%</div>
      <div class="row"><button class="sb" id="tm">−</button><button class="sb" id="tp">+</button></div>
    </div>
    <div class="card mid">
      <div class="st" id="st">连接中…</div>
      <div class="tele">
        <div class="ball"><div class="bub" id="bub"></div></div>
        <div class="nums">横滚 <b id="nR">--</b>°<br>俯仰 <b id="nP">--</b>°<br>机头 <b id="nY">--</b>°</div>
        <div class="mots">
          <div>M1<div class="mb"><i id="m0"></i></div></div><div>M2<div class="mb"><i id="m1"></i></div></div>
          <div>M3<div class="mb"><i id="m2"></i></div></div><div>M4<div class="mb"><i id="m3"></i></div></div>
        </div>
      </div>
      <div class="btns">
        <button class="btn go" id="go"><span id="goT">起飞</span><small id="goS">按住 0.6 秒</small><span class="pg" id="gopg"></span></button>
        <button class="btn land" id="land">降落<small>慢慢落下并上锁</small></button>
        <button class="btn stop" id="stop">⏹ 急停</button>
      </div>
    </div>
    <div class="card joy">
      <div class="cap">方向（按机头）</div>
      <div class="pad" id="pad"><div class="knob" id="knob"></div></div>
      <div class="yaw"><button class="yb" id="yl">⟲</button><button class="yb" id="yr">⟳</button></div>
    </div>
  </div>
</div>
<div class="toast" id="toast"></div>
<div class="off" id="off" hidden>没连上飞机：手机 WiFi 连 <b>DroneBench</b>，再打开 <b>192.168.4.1</b></div>
<div class="sheet" id="crv" hidden><div class="box wide">
  <h3>上次飞行（最后 4.8 秒）</h3>
  <canvas id="cv" width="900" height="440"></canvas>
  <div class="cvm" id="cvM"></div>
  <div class="sec"><button class="lk" id="crvC">关闭</button></div>
</div></div>
<div class="sheet" id="sheet" hidden><div class="box">
  <h3>设置</h3>
  <div class="sec"><div class="lab">一键起飞油门<i>带保护圈 49 g：悬停约 70%</i></div>
    <button class="sb" id="km">−</button><b class="big" id="kv" style="width:56px;text-align:center">64%</b><button class="sb" id="kp">+</button></div>
  <div class="sec"><div class="lab">校准水平<i>飞机放在平的桌面上、别碰它，再点</i></div>
    <button class="lk" id="lvl" style="flex:none;padding:10px 18px">⊙ 校准</button></div>
  <div class="sec"><div class="lab">配平（每点 0.1°）<i>总往哪飘就点反方向</i><i id="trimT">--</i></div>
    <div class="tpad">
      <span></span><button class="sb" data-t="P,1">▲</button><span></span>
      <button class="sb" data-t="R,-1">◀</button><span></span><button class="sb" data-t="R,1">▶</button>
      <span></span><button class="sb" data-t="P,-1">▼</button><span></span>
    </div></div>
  <div class="sec"><div class="lab">飞行参数<i>点 ± 马上写进飞机并存盘（飞行中不能改）</i></div></div>
  <div class="prm" id="prm"></div>
  <div class="sec"><div class="lab">上次飞行曲线<i>落地后自动弹出：灰 = 杆量目标，彩色 = 实际（最后 4.8 秒）</i></div>
    <button class="lk" id="crvB" style="flex:none;padding:10px 18px">📈 看曲线</button></div>
  <div class="sec"><a class="lk" href="/bench">电机 / 校准（调试页）</a><a class="lk" href="/update">固件升级</a></div>
  <div class="sec"><button class="lk" id="close">关闭</button></div>
  <div class="ver" id="ver"></div>
</div></div>
<script>
'use strict';
var $=function(s){return document.querySelector(s);};
function lsGet(k,d){try{var v=localStorage.getItem(k);return v==null?d:+v;}catch(e){return d;}}
function lsSet(k,v){try{localStorage.setItem(k,v);}catch(e){}}
function txt(el,v){if(el.__t!==v){el.__t=v;el.textContent=v;}}
var tt=0;function toast(m,err){var t=$('#toast');t.textContent=m;t.className='toast on'+(err?' err':'');clearTimeout(tt);tt=setTimeout(function(){t.className='toast';},2200);}
function buzz(ms){try{if(navigator.vibrate)navigator.vibrate(ms);}catch(e){}}
async function get(p){var ac=new AbortController(),to=setTimeout(function(){ac.abort();},2500);
  try{var r=await fetch(p,{cache:'no-store',signal:ac.signal});var t=await r.text();try{return JSON.parse(t);}catch(e){return {ok:r.ok?1:0,why:t};}}
  catch(e){return null;}finally{clearTimeout(to);}}

/* ---------- 状态 ---------- */
var S={on:false,armed:false,f:0,vb:0,roll:0,pitch:0,yaw:0,o:[0,0,0,0],rssi:0,usb:false};
var C={cmd:0,target:0,landing:false,r:0,p:0,y:0,mine:false,hov:0,hovN:0,lt:0,lf:0,touch:0};
var busy=false, tk=lsGet('tk3',52), lastRx=0, seq=0, ws=null, armT=0, trimDirty=false, P=null, vbF=0, pctF=-1;
var FAULT={1:'IMU 异常',2:'信号断了',4:'倾斜太大',8:'电池没电'};

/* ---------- WebSocket（端口 81）：每 50 ms 发杆量，回遥测；0.9 秒收不到飞机自己停 ---------- */
function connect(){
  try{ws=new WebSocket('ws://'+(location.hostname||'192.168.4.1')+':81/');}catch(e){ws=null;setTimeout(connect,1000);return;}
  ws.onclose=function(){ws=null;setTimeout(connect,busy?2000:700);};
  ws.onerror=function(){try{ws.close();}catch(e){}};
  ws.onmessage=function(e){
    if(e.data==='B'){busy=true;return;}
    busy=false;
    var a=String(e.data).split(',');
    if(a[0]!=='T'||a.length<19)return;
    lastRx=performance.now();
    S.vb=+a[2];var arm=a[3]==='1';S.f=+a[4];S.roll=+a[5];S.pitch=+a[6];
    S.o=[+a[10],+a[11],+a[12],+a[13]];S.az=+a[16];S.rssi=+a[17];S.usb=a[18]==='0';S.yaw=a.length>19?+a[19]:0;
    if(arm!==S.armed&&performance.now()-armT>400){
      S.armed=arm;
      if(!arm){C.cmd=C.target=0;C.landing=false;C.mine=false;if(S.wasMine)setTimeout(bbLoad,1500);S.wasMine=false;
        if(S.f){var w=[];for(var k in FAULT)if(S.f&k)w.push(FAULT[k]);toast('飞机自己停了：'+(w.join('、')||'代码 '+S.f),1);buzz(200);}
        if(trimDirty){trimDirty=false;setTimeout(function(){get('/api/save');},300);}}
    }
  };
}
connect();
function wsSend(s){if(ws&&ws.readyState===1&&ws.bufferedAmount<256){ws.send(s);return true;}return false;}

setInterval(function(){
  if(S.armed&&C.mine&&!C.landing&&C.cmd>40){C.hovN++;if(C.hovN>20)C.hov=C.hov?C.hov+(C.cmd-C.hov)*0.03:C.cmd;}else if(!C.landing)C.hovN=0;
  if(C.landing){   // 11-18：以前线性收油门（25%/s），高处会直接掉下来摔坏电机 → 改成按悬停油门缓降
    var el=(performance.now()-C.lt)/1000,hv=C.hov||C.lf,desc=Math.round(hv*0.92),tg;
    if(!C.touch&&el>0.4&&S.az&&Math.abs(S.az-0.94)>0.35)C.touch=el;
    if(!C.touch&&el<3.5)tg=Math.min(C.lf,desc+Math.max(0,(C.lf-desc)*(1-el/0.3)));
    else tg=Math.max(0,desc*(1-(el-(C.touch||3.5))/0.4));
    C.target=C.cmd=Math.max(0,tg);if(C.target<=0){C.landing=false;C.touch=0;disarm('落地 ✓');}}
  if(C.target>C.cmd)C.cmd=Math.min(C.target,C.cmd+2.5);else C.cmd=C.target;      // 往上最快 50%/秒，往下立刻
  wsSend('U,'+(++seq)+','+(S.armed&&C.mine?1:0)+','+Math.round(C.cmd*10)+','+C.r.toFixed(2)+','+C.p.toFixed(2)+','+C.y.toFixed(2));
  render();
},50);

/* ---------- 显示 ---------- */
var BT=[[4.15,100],[4.05,90],[3.95,78],[3.87,65],[3.80,52],[3.75,40],[3.70,28],[3.65,18],[3.58,9],[3.48,3],[3.30,0]];
function vpct(v){if(v>=BT[0][0])return 100;for(var i=1;i<BT.length;i++)if(v>=BT[i][0]){var a=BT[i-1],b=BT[i];return b[1]+(v-b[0])/(a[0]-b[0])*(a[1]-b[1]);}return 0;}
function render(){
  var now=performance.now(),gap=now-lastRx,on=gap<1500;
  if(on!==S.on){S.on=on;$('#off').hidden=on;if(!on){S.armed=false;C.cmd=C.target=0;C.landing=false;}}
  if(!on)txt($('#off'),busy?'另一台设备正在遥控这架飞机（关掉那边的页面，这里会自动接上）':'没连上飞机：手机 WiFi 连 DroneBench，再打开 192.168.4.1');
  var d=$('#dot');d.className='dot'+(on?(gap>400?' weak':' on'):'');
  txt($('#conn'),on?(gap>400?'信号差':'已连接'):'没连上');
  if(on&&S.vb>2500){
    var v=S.vb/1000;vbF=vbF?vbF+(v-vbF)*0.03:v;
    if(!S.armed){var p=vpct(vbF-(S.usb?0.05:0));pctF=pctF<0?p:pctF+(p-pctF)*0.05;}
    txt($('#bat'),(S.usb?'⚡':'🔋')+' '+Math.round(Math.max(pctF,0))+'% · '+v.toFixed(2)+' V');
  } else txt($('#bat'),'🔋 --');
  var q=!on||!S.rssi?0:S.rssi>-55?4:S.rssi>-65?3:S.rssi>-75?2:1;
  var rk=q+'|'+S.rssi;if($('#rs').__k!==rk){$('#rs').__k=rk;var h='<span class="sig">';
    for(var i=1;i<=4;i++)h+='<i style="height:'+(3+i*2.5)+'px;background:'+(i<=q?(q>=3?'#2fd3c0':q==2?'#f2b544':'#ff5a5f'):'#3d4a58')+'"></i>';
    $('#rs').innerHTML=h+'</span>'+(S.rssi?' '+S.rssi:'');}
  var st=$('#st'),s,c;
  if(!on){s='没连上';c='err';}
  else if(S.armed&&!C.mine){s='电脑在遥控';c='warn';}
  else if(S.armed){s=C.landing?'缓降中…':'飞行中';c='ok';}
  else if(S.f&1){s='IMU 异常，不能起飞';c='err';}
  else if(S.vb&&S.vb<3400){s='电量低，先充电';c='warn';}
  else{s='待机 · 可以起飞';c='';}
  txt(st,s);st.className='st '+c;
  txt($('#nR'),S.roll.toFixed(1));txt($('#nP'),S.pitch.toFixed(1));txt($('#nY'),Math.round(S.yaw)+'');
  $('#bub').style.transform='translate('+Math.max(-22,Math.min(22,S.roll*1.2)).toFixed(1)+'px,'+Math.max(-22,Math.min(22,-S.pitch*1.2)).toFixed(1)+'px)';
  for(var k=0;k<4;k++)$('#m'+k).style.width=Math.max(0,Math.min(100,S.o[k]/10))+'%';
  var show=S.armed?C.cmd:0;
  $('#fill').style.height=show+'%';$('#tgt').style.bottom='calc('+(S.armed?C.target:0)+'% - 1px)';
  txt($('#tpct'),S.armed?Math.round(show)+'%':'起飞 '+tk+'%');
  $('#tkm').style.bottom=tk+'%';
  txt($('#goT'),S.armed?'回到 '+tk+'%':'起飞');txt($('#goS'),S.armed?'点一下':'按住 0.6 秒 → '+tk+'%');
}

/* ---------- 起飞 / 降落 / 急停 ---------- */
async function takeoff(){
  if(!S.on){toast('还没连上飞机',1);return;}
  if(S.armed){C.landing=false;C.target=tk;C.cmd=Math.max(C.cmd,tk);return;}
  C.cmd=C.target=0;
  var r=await get('/api/arm?on=1');
  if(!r){toast('网络不通，没起飞',1);return;}
  if(!r.ok){toast('不能起飞：'+(r.why||'条件不满足'),1);return;}
  S.armed=true;C.mine=true;S.wasMine=true;armT=performance.now();C.landing=false;C.target=C.cmd=tk;buzz(40);
}
function disarm(msg){
  C.cmd=C.target=0;C.landing=false;wsSend('X');get('/api/arm?on=0');
  S.armed=false;armT=performance.now();if(msg)toast(msg);
  if(trimDirty){trimDirty=false;setTimeout(function(){get('/api/save');},300);}
}
function estop(){wsSend('X');get('/api/stop');C.cmd=C.target=0;C.landing=false;S.armed=false;armT=performance.now();buzz(120);toast('⏹ 已急停');}
$('#stop').addEventListener('pointerdown',function(e){e.preventDefault();estop();});
$('#land').addEventListener('click',function(){if(S.armed&&C.mine&&!C.landing){C.landing=true;C.touch=0;C.lt=performance.now();C.lf=C.cmd;toast('缓降中，触地自动停');}});
(function(){var b=$('#go'),pg=$('#gopg'),t=0,t0=0,raf=0;
  function anim(){var p=Math.min(1,(performance.now()-t0)/600);pg.style.width=p*100+'%';if(p<1)raf=requestAnimationFrame(anim);}
  function cancel(){clearTimeout(t);cancelAnimationFrame(raf);pg.style.width='0';t=0;}
  b.addEventListener('pointerdown',function(e){e.preventDefault();
    if(S.armed){takeoff();return;}
    try{b.setPointerCapture(e.pointerId);}catch(x){}
    t0=performance.now();anim();t=setTimeout(function(){cancel();takeoff();},600);});
  ['pointerup','pointercancel','lostpointercapture'].forEach(function(ev){b.addEventListener(ev,function(){if(t)cancel();});});
})();
/* 切出页面 / 锁屏 = 急停（手机切后台网页会被冻住，不能让飞机继续飞） */
document.addEventListener('visibilitychange',function(){if(document.hidden&&S.armed)estop();});
window.addEventListener('pagehide',function(){if(S.armed)estop();});

/* ---------- 油门条 ---------- */
/* 左手：整块油门卡片都能拖。上下 = 相对调油门（按下不跳，拖满卡片高度 = 60%），左右 = 转机头（松手回中）。
   和右手摇杆是两个独立的触点，可以同时按 */
(function(){var el=$('.thr'),act=null,y0=0,x0=0,t0=0,yi=$('#yawi i');
  function end(e){if(e.pointerId!==act)return;act=null;el.classList.remove('on');C.y=0;yi.style.transform='';}
  el.addEventListener('pointerdown',function(e){
    if(e.target.closest&&e.target.closest('button'))return;   // −/+ 按钮照常
    e.preventDefault();if(act!==null)return;
    if(!S.armed||!C.mine){toast('先按“起飞”');return;}
    act=e.pointerId;try{el.setPointerCapture(act);}catch(x){}el.classList.add('on');y0=e.clientY;x0=e.clientX;t0=C.target;});
  el.addEventListener('pointermove',function(e){
    if(e.pointerId!==act||!S.armed)return;
    var h=el.getBoundingClientRect().height||300;
    C.target=Math.max(0,Math.min(100,Math.round(t0+(y0-e.clientY)/h*60)));C.landing=false;
    var dx=e.clientX-x0,k=Math.abs(dx)<10?0:Math.max(-1,Math.min(1,(dx-(dx>0?10:-10))/70));
    C.y=k*0.5;yi.style.transform='translateX('+(k*40).toFixed(0)+'px)';});
  ['pointerup','pointercancel','lostpointercapture'].forEach(function(ev){el.addEventListener(ev,end);});
})();
function nudge(d){
  if(!S.armed||!C.mine){tk=Math.max(30,Math.min(90,tk+d));lsSet('tk3',tk);showK();toast('起飞油门 '+tk+'%');return;}   // 没起飞：调起飞油门；飞行中：调当前油门
  C.landing=false;C.target=Math.max(0,Math.min(100,Math.round(C.target)+d));}
$('#tm').onclick=function(){nudge(-1);};$('#tp').onclick=function(){nudge(1);};

/* ---------- 摇杆：满杆 0.3 ≈ 倾 7.5°；松手回中 ---------- */
(function(){var pad=$('#pad'),kn=$('#knob'),act=null,MAXT=0.4;
  function upd(e){var r=pad.getBoundingClientRect(),R=r.width*0.3;
    var x=(e.clientX-(r.left+r.width/2))/R,y=(e.clientY-(r.top+r.height/2))/R,m=Math.hypot(x,y);
    if(m>1){x/=m;y/=m;m=1;}
    kn.style.transform='translate('+(x*R).toFixed(0)+'px,'+(y*R).toFixed(0)+'px)';
    var k=m<0.1?0:(m-0.1)/0.9/m;C.r=x*k*MAXT;C.p=-y*k*MAXT;}
  function end(e){if(e.pointerId!==act)return;act=null;pad.classList.remove('on');kn.style.transform='';C.r=C.p=0;}
  pad.addEventListener('pointerdown',function(e){e.preventDefault();if(act!==null)return;act=e.pointerId;try{pad.setPointerCapture(act);}catch(x){}pad.classList.add('on');upd(e);});
  pad.addEventListener('pointermove',function(e){if(e.pointerId===act)upd(e);});
  ['pointerup','pointercancel','lostpointercapture'].forEach(function(ev){pad.addEventListener(ev,end);});
})();
function yawBtn(id,v){var b=$(id),act=null;
  b.addEventListener('pointerdown',function(e){e.preventDefault();act=e.pointerId;try{b.setPointerCapture(act);}catch(x){}b.classList.add('on');C.y=v;});
  ['pointerup','pointercancel','lostpointercapture'].forEach(function(ev){b.addEventListener(ev,function(e){if(e.pointerId!==act)return;act=null;b.classList.remove('on');C.y=0;});});}
yawBtn('#yl',-0.4);yawBtn('#yr',0.4);

/* ---------- 设置 ---------- */
function showK(){txt($('#kv'),tk+'%');}
showK();
$('#km').onclick=function(){tk=Math.max(30,tk-1);lsSet('tk3',tk);showK();};
$('#kp').onclick=function(){tk=Math.min(90,tk+1);lsSet('tk3',tk);showK();};
function trimShow(){txt($('#trimT'),P?'现在：前后 '+(+P.trimP).toFixed(1)+'° · 左右 '+(+P.trimR).toFixed(1)+'°':'读取中…');}
$('#cfg').onclick=async function(){$('#sheet').hidden=false;trimShow();
  var p=await get('/api/params');if(p&&typeof p.trimR==='number'){P=p;trimShow();}
  var v=await get('/api/ver');if(v&&v.v)txt($('#ver'),'固件 '+v.v);};
$('#close').onclick=function(){$('#sheet').hidden=true;};
$('#sheet').addEventListener('click',function(e){if(e.target===this)this.hidden=true;});
Array.prototype.forEach.call(document.querySelectorAll('.tpad button'),function(b){b.addEventListener('click',async function(){
  if(!P){toast('还没读到参数');return;}
  var a=b.dataset.t.split(','),k=a[0]==='P'?'trimP':'trimR';
  P[k]=Math.round(((+P[k]||0)+0.1*(+a[1]))*100)/100;
  var r=await get('/api/param?k='+k+'&v='+P[k]);
  if(!r||!r.ok){toast('没写进去，再点一次',1);return;}
  if(S.armed)trimDirty=true;else get('/api/save');
  trimShow();toast((a[0]==='P'?(+a[1]>0?'往前':'往后'):(+a[1]>0?'往右':'往左'))+' 0.1° ✓');
});});
/* 校准水平：把现在的倾角记进配平并存盘（固件 /api/level 在控制任务里做） */
$('#lvl').onclick=async function(){
  var b=this;if(b.disabled)return;
  if(!S.on){toast('还没连上飞机',1);return;}
  if(S.armed){toast('飞行中不能校准',1);return;}
  if(S.f&1){toast('IMU 异常，先断电重启',1);return;}
  if(Math.abs(S.roll)>12||Math.abs(S.pitch)>12){toast('歪得太多（>12°），放平再点',1);return;}
  b.disabled=true;txt(b,'校准中…');
  var r=await get('/api/level');
  if(r&&r.ok){await new Promise(function(f){setTimeout(f,400);});r=await get('/api/save');}
  if(r&&r.ok){var p=await get('/api/params');if(p&&typeof p.trimR==='number')P=p;trimShow();toast('水平已校准 ✓');buzz(40);}
  else toast('没成功，再点一次',1);
  b.disabled=false;txt(b,'⊙ 校准');
};
/* ---------- 飞行参数（和电脑调试台同一套，写进飞机并存盘） ---------- */
var PR=[{k:'rp',ks:['rp','pp'],n:'角速度 P',st:0.25,d:2,lo:0.5,hi:6,h:'软、慢晃 → 加；嗡嗡抖 → 减'},
  {k:'rd',ks:['rd','pd'],n:'角速度 D',st:0.01,d:3,lo:0,hi:0.2,h:'冲过头 → 加；电机烫/高频抖 → 减'},
  {k:'angP',ks:['angP'],n:'角度 P',st:0.5,d:1,lo:1,hi:10,h:'回正慢 → 加；回正冲过头 → 减'},
  {k:'kff',ks:['kff'],n:'杆量前馈',st:0.1,d:1,lo:0,hi:1.5,h:'打杆反应慢 → 加；一打杆就冲 → 减'},
  {k:'kdz',ks:['kdz'],n:'高度阻尼',st:50,d:0,lo:0,hi:500,h:'只压快速上下窜，太大会抢油门，300 左右'}];
$('#prm').innerHTML=PR.map(function(p){return '<div class="pr" data-k="'+p.k+'"><span>'+p.n+'</span><button class="sb" data-d="-1">−</button><b>--</b><button class="sb" data-d="1">+</button><i>'+p.h+'</i></div>';}).join('');
function prShow(){PR.forEach(function(p){var v=P&&P[p.k];txt(document.querySelector('.pr[data-k="'+p.k+'"] b'),typeof v==='number'?v.toFixed(p.d):'--');});}
Array.prototype.forEach.call(document.querySelectorAll('.pr'),function(row){
  var p=PR.filter(function(x){return x.k===row.dataset.k;})[0];
  Array.prototype.forEach.call(row.querySelectorAll('button'),function(b){b.addEventListener('click',async function(){
    if(!P){toast('还没读到参数');return;} if(S.armed){toast('先落地再改',1);return;}
    var v=Math.max(p.lo,Math.min(p.hi,Math.round(((+P[p.k]||0)+p.st*(+b.dataset.d))/p.st)*p.st));v=+v.toFixed(p.d+1);
    for(var i=0;i<p.ks.length;i++){var r=await get('/api/param?k='+p.ks[i]+'&v='+v);if(!r||!r.ok){toast('没写进去，再点一次',1);return;}}
    await get('/api/save');var q=await get('/api/params');if(q&&typeof q.trimR==='number')P=q;prShow();toast(p.n+' → '+v+' ✓');
  });});
});
setInterval(prShow,1000);

/* ---------- 曲线：落地后读飞机里的黑匣子（250 Hz，最后 4.8 秒），画「目标 vs 实际」 ---------- */
async function bbLoad(){
  var ac=new AbortController(),to=setTimeout(function(){ac.abort();},6000),txt0='';
  try{txt0=await (await fetch('/api/bb',{cache:'no-store',signal:ac.signal})).text();}catch(e){toast('读不到黑匣子',1);return;}finally{clearTimeout(to);}
  var L=txt0.trim().split('\n');if(L.length<20){toast('飞机里还没有飞行记录');return;}var h=L[1].split(','),ix={};h.forEach(function(k,i){ix[k]=i;});
  var rows=L.slice(2).map(function(l){return l.split(',').map(Number);}).filter(function(a){return a.length===h.length;});
  var i1=rows.length;for(var i=0;i<rows.length;i++){if(Math.abs(rows[i][ix.roll])>35||Math.abs(rows[i][ix.pitch])>35){i1=Math.max(10,i-75);break;}}
  rows=rows.slice(0,i1).filter(function(a){return a[ix.thr]>0;});if(rows.length<20){toast('这次没飞起来，没有曲线');return;}
  var t0=rows[0][ix.ms],cv=$('#cv'),g=cv.getContext('2d'),W=cv.width,H=cv.height,half=H/2;
  g.fillStyle='#0d1622';g.fillRect(0,0,W,H);
  var am=8;rows.forEach(function(a){am=Math.max(am,1.5*Math.abs(a[ix.stR]*25),1.5*Math.abs(a[ix.stP]*25));});
  function panel(y0,tk,ak,col,lab){
    var X=function(a){return 40+(a[ix.ms]-t0)/(rows[rows.length-1][ix.ms]-t0||1)*(W-50);},Y=function(v){return y0+half/2-Math.max(-am,Math.min(am,v))/am*(half/2-14);};
    g.strokeStyle='#22324a';g.lineWidth=1;g.beginPath();g.moveTo(40,Y(0));g.lineTo(W-10,Y(0));g.stroke();
    g.fillStyle='#8a97a6';g.font='20px sans-serif';g.fillText(lab,46,y0+24);
    [[function(a){return a[ix[tk]]*25;},'#8a97a6'],[function(a){return a[ix[ak]];},col]].forEach(function(s){
      g.strokeStyle=s[1];g.lineWidth=s[1]==='#8a97a6'?2:3;g.beginPath();rows.forEach(function(a,i){var x=X(a),y=Y(s[0](a));if(i)g.lineTo(x,y);else g.moveTo(x,y);});g.stroke();});
  }
  panel(0,'stR','roll','#2fd3c0','横滚 °（±'+Math.round(am)+'）');panel(half,'stP','pitch','#ff7a59','俯仰 °');
  var q=rows.filter(function(a){return Math.abs(a[ix.stR])<0.01&&Math.abs(a[ix.stP])<0.01;});
  var sd=function(k){if(!q.length)return 0;var m=q.reduce(function(s,a){return s+a[ix[k]];},0)/q.length;return Math.sqrt(q.reduce(function(s,a){return s+(a[ix[k]]-m)*(a[ix[k]]-m);},0)/q.length);};
  var sat=rows.filter(function(a){return Math.max(a[ix.o1],a[ix.o2],a[ix.o3],a[ix.o4])>=940;}).length/rows.length;
  $('#cvM').innerHTML='不打杆晃动 <b>横滚 ±'+sd('roll').toFixed(1)+'° · 俯仰 ±'+sd('pitch').toFixed(1)+'°</b> · 有电机顶满 <b>'+Math.round(100*sat)+'%</b>'+(i1<L.length-2?' · <b>结尾翻/撞了（之后不画）</b>':'');
  $('#crv').hidden=false;
}
$('#crvB').onclick=function(){$('#sheet').hidden=true;bbLoad();};
$('#crvC').onclick=function(){$('#crv').hidden=true;};
$('#crv').addEventListener('click',function(e){if(e.target===this)this.hidden=true;});
$('#fs').onclick=function(){var d=document.documentElement;
  try{(d.requestFullscreen||d.webkitRequestFullscreen).call(d).then(function(){try{screen.orientation.lock('landscape').catch(function(){});}catch(e){}}).catch(function(){});}catch(e){}};
document.addEventListener('contextmenu',function(e){e.preventDefault();});
/* 长按不选字、不弹复制菜单：按住类的控件直接吃掉 touchstart（它们只用 pointer 事件），全局禁止选区 */
document.addEventListener('selectstart',function(e){if(!(e.target.closest&&e.target.closest('input,textarea')))e.preventDefault();});
document.addEventListener('selectionchange',function(){var s=window.getSelection&&window.getSelection();if(s&&s.rangeCount&&!(document.activeElement&&/INPUT|TEXTAREA/.test(document.activeElement.tagName)))s.removeAllRanges();});
['#stop','#go','.thr','#pad','#yl','#yr'].forEach(function(s){var el=$(s);if(el)el.addEventListener('touchstart',function(e){if(s==='.thr'&&e.target.closest&&e.target.closest('button'))return;if(e.cancelable)e.preventDefault();},{passive:false});});
render();
</script></body></html>
)HTML";

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
  <div id="msg">选一个 .bin（编译产物 flight_vNN.ino.bin）</div>
</div>
<div class="card hl">
  <b>注意事项</b><br>
  ① 上传中不要断电、不要关页面，约 10–20 秒<br>
  ② 只接受本机编译的 <i>flight_vNN.ino.bin</i>，别传错型号的固件<br>
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
      else if (!strcmp(k, "angI")) g_angI = v;
      else if (!strcmp(k, "ya")) g_ya = v;
      else if (!strcmp(k, "tramp")) g_tramp = v;
      else if (!strcmp(k, "dzmax")) g_dzmax = v;
      else if (!strcmp(k, "tz")) g_tz = v;
      else if (!strcmp(k, "kdz")) g_kdz = v;
      else if (!strcmp(k, "khi")) g_khi = v;
      else if (!strcmp(k, "khv")) g_khv = v;
      else if (!strcmp(k, "khp")) g_khp = v;
      else if (!strcmp(k, "climb")) g_climb = v;
      else if (!strcmp(k, "ht")) g_ht = v;
      else if (!strcmp(k, "ah")) g_ah = v;
      else if (!strcmp(k, "hov")) g_hov = v;
      else if (!strcmp(k, "mg1")) g_mg[0] = (v < 0.7f ? 0.7f : (v > 1.4f ? 1.4f : v));
      else if (!strcmp(k, "mg2")) g_mg[1] = (v < 0.7f ? 0.7f : (v > 1.4f ? 1.4f : v));
      else if (!strcmp(k, "mg3")) g_mg[2] = (v < 0.7f ? 0.7f : (v > 1.4f ? 1.4f : v));
      else if (!strcmp(k, "mg4")) g_mg[3] = (v < 0.7f ? 0.7f : (v > 1.4f ? 1.4f : v));
      else if (!strcmp(k, "irel")) g_irel = (v < 0 ? 0 : (v > 500 ? 500 : v));
      else if (!strcmp(k, "ag")) g_ag = (v < 0 ? 0 : (v > 10 ? 10 : v));
      else if (!strcmp(k, "nf")) g_nf = (v < 0 ? 0 : (v > 450 ? 450 : v));
      else if (!strcmp(k, "nq")) g_nq = (v < 0.5f ? 0.5f : (v > 20 ? 20 : v));
      else if (!strcmp(k, "kff")) g_kff = (v < 0 ? 0 : (v > 2 ? 2 : v));
      else if (!strcmp(k, "kdz")) g_kdz = v;
      else if (!strcmp(k, "tz")) g_tz = v;
      else if (!strcmp(k, "dzmax")) g_dzmax = v;
      else if (!strcmp(k, "crate")) g_crate = (v >= 900 ? 1000 : (v <= 300 ? 250 : 500));
      else if (!strcmp(k, "lp2")) g_lp2 = v > 0.5f ? 1 : 0;
      else if (!strcmp(k, "cap")) g_cap = v > 0.5f ? 1 : 0;
      else if (!strcmp(k, "dfc")) g_dfc = (v < 0 ? 0 : (v > 200 ? 200 : v));
      else if (!strcmp(k, "yff")) g_yff = (v > 200 ? 200 : (v < -200 ? -200 : v));
      else if (!strcmp(k, "dk")) g_dk = v;
      else if (!strcmp(k, "mu")) g_mu = v;
      else if (!strcmp(k, "hmax")) g_hmax = v;
      else if (!strcmp(k, "vleak")) g_vleak = v;
      else if (!strcmp(k, "ph")) g_ph = v;
      else if (!strcmp(k, "vh")) g_vh = v;
      else if (!strcmp(k, "bP")) g_bP = (v > 80 ? 80 : (v < -80 ? -80 : v));
      else if (!strcmp(k, "bR")) g_bR = (v > 80 ? 80 : (v < -80 ? -80 : v));
      else if (!strcmp(k, "yi")) g_yi = v;
      else if (!strcmp(k, "yprobe")) g_yawProbe = (int)v;
      else if (!strcmp(k, "vcomp")) g_vcomp = (int)v;
      else if (!strcmp(k, "rot")) { g_rot = v; attSeeded = false; }
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
  if (c == 'H') {
    int pm = 0;
    int ch = 0;
    if (sscanf(s, "H,%d,%d", &pm, &ch) >= 1) {
      g_spinCh = (ch >= 1 && ch <= 4) ? ch : 0;
      if (g_armed) { Serial.println("#ERR,ARMED"); return; }
      if (pm < 0) pm = 0; if (pm > 950) pm = 950;
      if (g_mode != M_SPIN) g_spinRef = false;   // 新的一次：重新记参考姿态
      g_spin = pm;
      g_mode = pm > 0 ? M_SPIN : M_IDLE;
      g_lastStick = millis();
      Serial.printf("#ACK,H,%d\n", pm);
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
  if (c == 'M') { char j[760]; buildPerfJson(j, sizeof(j)); Serial.printf("#PERF,%s\n", j); return; }   // v21 性能指标
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
static uint32_t g_bootCnt = 0; static int g_rstRsn = 0;
void setup() {
  g_rstRsn = (int)esp_reset_reason();
  prefs.begin("diag", false); g_bootCnt = prefs.getUInt("bc", 0) + 1; prefs.putUInt("bc", g_bootCnt); prefs.end();
  boardInit();
  bb = (BBRec*)malloc(sizeof(BBRec) * BBN);
  if (psramFound()) { LBN = 30000; lb = (LBRec*)ps_malloc(sizeof(LBRec) * LBN); }   // v20 30 s × 1 kHz ≈ 1.3 MB
  if (!lb) { LBN = 2000; lb = (LBRec*)malloc(sizeof(LBRec) * LBN); }
  Serial.printf("#LB,%d,%d\n", LBN, (int)sizeof(LBRec));
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
  WiFi.setAutoReconnect(false);   // v16：别让路由器连不上时反复扫描，热点会跟着断
  IPAddress ip(192, 168, 4, 1), gw(192, 168, 4, 1), mask(255, 255, 255, 0);
  WiFi.softAPConfig(ip, gw, mask);
  WiFi.softAP(AP_SSID, NULL, AP_CH, 0, 4);
  netLoad();
  netBegin();
  delay(150);
  dns.setErrorReplyCode(DNSReplyCode::NoError);
  dns.start(53, "*", WiFi.softAPIP());

  server.on("/", []() { server.send_P(200, "text/html; charset=utf-8", PHONE_HTML); });
  server.on("/bench", []() { server.send_P(200, "text/html; charset=utf-8", PAGE_HTML); });
  server.on("/api/imureg", handleImuReg);
  server.on("/api/ver", []() {
    char j[120];
    snprintf(j, sizeof(j), "{\"v\":\"" FW_VER "\",\"up\":%lu,\"rr\":%d,\"bc\":%lu,\"vb\":%d}",
             (unsigned long)millis(), g_rstRsn, (unsigned long)g_bootCnt, (int)vbatMv);
    sendJson(j);
  });
  // favicon 不许落进 302，否则浏览器每次都拉一遍整页
  server.on("/favicon.ico", []() { server.send(204, "text/plain", ""); });
  server.on("/api/state", handleState);
  server.on("/api/bb", handleBB);
  server.on("/api/lb", handleLB);
  server.on("/api/perf", handlePerf);
  server.on("/api/yawtest", handleYawTest);
  server.on("/api/params", handleParams);
  server.on("/api/arm", handleArm);
  server.on("/api/stick", handleStick);
  server.on("/api/spin", handleSpin);
  server.on("/api/hspin", handleHSpin);
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
  wsSrv.begin(); wsSrv.setNoDelay(true);
  xTaskCreatePinnedToCore(wsTask, "ws", 6144, nullptr, 3, nullptr, 0);

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
  if (g_needSave && !g_armed) { g_needSave = false; paramsSave(); Serial.printf("#ACK,YAWSET,sgY=%d,d=%.1f\n", P.sgY, (double)g_probeD); }

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
