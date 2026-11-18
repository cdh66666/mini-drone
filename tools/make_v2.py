# 由 flight_v1 生成 flight_v2：加联网（AP+STA、扫描、mDNS）、CORS、长行串口缓冲
import sys
src, dst = sys.argv[1], sys.argv[2]
t = open(src, encoding='utf-8').read()
def rep(a, b):
    global t
    assert t.count(a) == 1, (a, t.count(a)); t = t.replace(a, b)
t = t.replace("  const char c = s[0];\n\n  if (c == 'V')", "  const char c = s[0];\n  if (c == 'V')", 1)
rep('#define FW_VER "flight-v1"', '#define FW_VER "flight-v2"')
rep('#include <WiFi.h>\n', '#include <WiFi.h>\n#include <ESPmDNS.h>\n')
rep('static DNSServer dns;\n', r'''static DNSServer dns;

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
''')
rep('''  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json; charset=utf-8", s);''', '''  server.sendHeader("Cache-Control", "no-store");
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json; charset=utf-8", s);''')
rep('''  const char c = s[0];
  if (c == 'V') { sendVer(); return; }''', '''  const char c = s[0];
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
  if (c == 'V') { sendVer(); return; }''')
rep('''  WiFi.mode(WIFI_AP);''', '''  WiFi.mode(WIFI_AP_STA);
  WiFi.setSleep(false);''')
rep('''  WiFi.softAP(AP_SSID, NULL, AP_CH, 0, 4);''', '''  WiFi.softAP(AP_SSID, NULL, AP_CH, 0, 4);
  netLoad();
  netBegin();''')
rep('''    static char rxBuf[64];''', '''    static char rxBuf[255];''')
rep('''  dns.processNextRequest();
  server.handleClient();''', '''  dns.processNextRequest();
  server.handleClient();
  netLoop();''')
rep('''  Serial.begin(115200);''', '''  Serial.setTxBufferSize(2048);
  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);   // USB 插着但没人读时绝不等待，否则主循环卡死、WiFi 断''')
# 遥测：缓冲放不下就整行丢掉，不阻塞
rep('''    Serial.printf("D,%lu,''', '''    if (Serial.availableForWrite() > 160) Serial.printf("D,%lu,''')
rep('''    Serial.printf("#S,%s\\n", js);''', '''    if (Serial.availableForWrite() > 600) Serial.printf("#S,%s\\n", js);''')
# ---------------- v3：UDP 控制通道（独立任务，不受网页服务卡顿影响） ----------------
rep('#define FW_VER "flight-v2"', '#define FW_VER "flight-v3"')
rep('#include <ESPmDNS.h>\n', '#include <ESPmDNS.h>\n#include <WiFiUdp.h>\n')
rep('''static void netScan() {''', r'''// 电脑每 40 ms 发一包：U,<seq>,<live>,<thr>,<r>,<p>,<y>
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

static void netScan() {''')
rep('''  server.begin();''', '''  server.begin();
  udp.begin(4210);
  xTaskCreatePinnedToCore(udpTask, "udp", 4096, nullptr, 3, nullptr, 0);''')
# ---------------- v4：防起飞侧翻 + 黑匣子 + 串口 E 支持全部参数 ----------------
rep('#define FW_VER "flight-v3"', '#define FW_VER "flight-v4"')
# 积分上限 350‰ 太大（一次能把一侧推 35%），压到 150
rep('static const float IMAX = 350.0f;', 'static const float IMAX = 150.0f;')
# 黑匣子：解锁期间 250 Hz 记最近 4.8 s，上锁时冻结，HTTP /api/bb 取 CSV
rep('static void disarm(uint32_t fault) {\n', r'''struct BBRec { uint32_t ms; int16_t thr, roll, pitch, gr, gp, gy, spR, spP, oR, oP, o1, o2, o3, o4, itR, itP, stR, stP; };
static const int BBN = 1200;
static BBRec* bb = nullptr;
static volatile int bbHead = 0, bbCount = 0;
static volatile uint32_t bbReason = 0, bbSeq = 0;
static volatile bool bbFrozen = true;
static int g_iThr = 250;   // 输出油门低于这个（‰）时不积分：还在地上时积分会越攒越歪，一离地就猛地一歪
static void bbStart() { bbHead = 0; bbCount = 0; bbReason = 0; bbFrozen = false; }

static void disarm(uint32_t fault) {
  if (g_armed && !bbFrozen) { bbFrozen = true; bbReason = fault; bbSeq++; }
''')
rep('''  pidR.kp = P.rp; pidR.ki = P.ri * k; pidR.kd = P.rd;
  pidP.kp = P.pp; pidP.ki = P.pi * k; pidP.kd = P.pd;''', '''  const float kI = (thr >= g_iThr) ? k : 0.0f;
  if (thr < g_iThr) { pidR.it = 0; pidP.it = 0; }
  pidR.kp = P.rp; pidR.ki = P.ri * kI; pidR.kd = P.rd;
  pidP.kp = P.pp; pidP.ki = P.pi * kI; pidP.kd = P.pd;''')
rep('''  mixOutput(thr, oR * k, oP * k, oY * k, o);
  writeMotors(o);''', '''  mixOutput(thr, oR * k, oP * k, oY * k, o);
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
  }''')
# 两处解锁都开始新记录
t = t.replace('g_armMs = millis();', 'g_armMs = millis(); bbStart();')
assert t.count('bbStart();') == 2
# 串口 E 支持全部参数（USB 下也能改方向符号）
rep('''      else if (!strcmp(k, "mthr")) P.mthr = (int)v;''', '''      else if (!strcmp(k, "mthr")) P.mthr = (int)v;
      else if (!strcmp(k, "mang")) P.mang = v;
      else if (!strcmp(k, "trMax")) P.trMax = v;
      else if (!strcmp(k, "trimR")) P.trimR = v;
      else if (!strcmp(k, "trimP")) P.trimP = v;
      else if (!strcmp(k, "sgR")) P.sgR = v < 0 ? -1 : 1;
      else if (!strcmp(k, "sgP")) P.sgP = v < 0 ? -1 : 1;
      else if (!strcmp(k, "sgY")) P.sgY = v < 0 ? -1 : 1;
      else if (!strcmp(k, "iThr")) g_iThr = (int)v;''')
rep('''  else if (k == "sgY") P.sgY = (v < 0) ? -1 : 1;''', '''  else if (k == "sgY") P.sgY = (v < 0) ? -1 : 1;
  else if (k == "iThr") g_iThr = (int)v;''')
# HTTP 取黑匣子
rep('''static bool canArm(String& why) {''', r'''static void handleBB() {
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

static bool canArm(String& why) {''')
rep('''  server.on("/api/state", handleState);''', '''  server.on("/api/state", handleState);
  server.on("/api/bb", handleBB);''')
rep('''  boardInit();''', '''  boardInit();
  bb = (BBRec*)malloc(sizeof(BBRec) * BBN);''')
# ---------------- v5：传感器绕竖直轴的安装角 rot（度），陀螺和加计的 x/y 一起转 ----------------
rep('#define FW_VER "flight-v4"', '#define FW_VER "flight-v5"')
rep('static void paramsDefault() {', 'static float g_rot = 0.0f;   // IMU 安装角（度），存 NVS "rot"\n\nstatic void paramsDefault() {')
rep('''  prefs.putFloat("trimR", P.trimR); prefs.putFloat("trimP", P.trimP);''',
    '''  prefs.putFloat("trimR", P.trimR); prefs.putFloat("trimP", P.trimP);
  prefs.putFloat("rot", g_rot);''')
rep('''    P.trimR = prefs.getFloat("trimR", P.trimR); P.trimP = prefs.getFloat("trimP", P.trimP);''',
    '''    P.trimR = prefs.getFloat("trimR", P.trimR); P.trimP = prefs.getFloat("trimP", P.trimP);
    g_rot = prefs.getFloat("rot", 0.0f);''')
rep('''    const float gx = fg[0] - gb[0];
    const float gy = fg[1] - gb[1];''', '''    const float rc = cosf(g_rot / R2D), rs = sinf(g_rot / R2D);
    const float gx0 = fg[0] - gb[0], gy0 = fg[1] - gb[1];
    const float gx = gx0 * rc - gy0 * rs;
    const float gy = gx0 * rs + gy0 * rc;
    const float fax = fa[0] * rc - fa[1] * rs;
    const float fay = fa[0] * rs + fa[1] * rc;''')
rep('''    float rollA  = (float)P.sgR * atan2f(fa[1], fa[2]) * R2D - P.trimR;
    float pitchA = (float)P.sgP * atan2f(-fa[0], sqrtf(fa[1] * fa[1] + fa[2] * fa[2])) * R2D - P.trimP;''',
    '''    float rollA  = (float)P.sgR * atan2f(fay, fa[2]) * R2D - P.trimR;
    float pitchA = (float)P.sgP * atan2f(-fax, sqrtf(fay * fay + fa[2] * fa[2])) * R2D - P.trimP;''')
rep('''      else if (!strcmp(k, "iThr")) g_iThr = (int)v;''', '''      else if (!strcmp(k, "iThr")) g_iThr = (int)v;
      else if (!strcmp(k, "rot")) { g_rot = v; attSeeded = false; }''')
rep('''  else if (k == "iThr") g_iThr = (int)v;''', '''  else if (k == "iThr") g_iThr = (int)v;
  else if (k == "rot") { g_rot = v; attSeeded = false; }''')
rep('''           "\\"sgR\\":%d,\\"sgP\\":%d,\\"sgY\\":%d}",''', '''           "\\"sgR\\":%d,\\"sgP\\":%d,\\"sgY\\":%d,\\"rot\\":%.1f}",''')
rep('''           P.sgR, P.sgP, P.sgY);
}''', '''           P.sgR, P.sgP, P.sgY, (double)g_rot);
}''')

# ---------------- v6：偏航（防自转）积分 + 自转方向检查 ----------------
rep('#define FW_VER "flight-v5"', '#define FW_VER "flight-v6"')
rep('static float g_rot = 0.0f;', 'static float g_rot = 0.0f;\nstatic float g_yi = 0.0f;    // 偏航积分（RAM，页面每次连上写入）')
rep('  pidY.kp = P.yp; pidY.ki = 0;       pidY.kd = 0;', '  pidY.kp = P.yp; pidY.ki = g_yi * kI; pidY.kd = 0;\n  if (thr < g_iThr) pidY.it = 0;')
rep('''      else if (!strcmp(k, "iThr")) g_iThr = (int)v;''', '''      else if (!strcmp(k, "iThr")) g_iThr = (int)v;
      else if (!strcmp(k, "yi")) g_yi = v;''')
rep('''  else if (k == "iThr") g_iThr = (int)v;''', '''  else if (k == "iThr") g_iThr = (int)v;
  else if (k == "yi") g_yi = v;''')
# /api/yawtest：飞机放桌上，依次点动 M1、M2，积分机身偏航角速度，看被扭向哪边
rep('''static void handleBB() {''', r'''static float yawJog(int i, int pm, int ms) {
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
  char j[96];
  snprintf(j, sizeof(j), "{\"m1\":%.2f,\"m2\":%.2f,\"sgY\":%d}", a1, a2, P.sgY);
  sendJson(j);
}

static void handleBB() {''')
rep('''  server.on("/api/bb", handleBB);''', '''  server.on("/api/bb", handleBB);
  server.on("/api/yawtest", handleYawTest);''')

# ---------------- v7：自转方向检查测 4 个电机 ----------------
rep('#define FW_VER "flight-v6"', '#define FW_VER "flight-v7"')
rep('''  const float a1 = yawJog(0, pm, ms);
  const float a2 = yawJog(1, pm, ms);
  char j[96];
  snprintf(j, sizeof(j), "{\\"m1\\":%.2f,\\"m2\\":%.2f,\\"sgY\\":%d}", a1, a2, P.sgY);''', '''  const float a1 = yawJog(0, pm, ms);
  const float a2 = yawJog(1, pm, ms);
  const float a3 = yawJog(2, pm, ms);
  const float a4 = yawJog(3, pm, ms);
  char j[140];
  snprintf(j, sizeof(j), "{\\"m1\\":%.2f,\\"m2\\":%.2f,\\"m3\\":%.2f,\\"m4\\":%.2f,\\"sgY\\":%d}", a1, a2, a3, a4, P.sgY);''')

# ---------------- v8：空中少信加计 + 电压补油门 + 空中偏航试探 + 黑匣子记偏航/az ----------------
rep('#define FW_VER "flight-v7"', '#define FW_VER "flight-v8"')
# 悬停时加计只感到推力、几乎测不到倾斜 → 互补滤波会把“歪着”估成“平的”，飞机就一直往一边跑。
# 空中把加计权重降到 1/8（时间常数 ~1 s），地上照旧。
rep('''      const float K = 0.03f;''', '''      const int outAvg = (g_out[0] + g_out[1] + g_out[2] + g_out[3]) / 4;
      const float K = (g_armed && outAvg > g_iThr) ? 0.004f : 0.03f;''')
# 电池电压下降时补油门（以 3.95 V 为基准），免得越飞越低
rep('''  if (thr < 0) thr = 0; if (thr > P.mthr) thr = P.mthr;
''', '''  if (thr < 0) thr = 0; if (thr > P.mthr) thr = P.mthr;
  if (g_vcomp && vbatMv > 3000 && thr > 0) {
    float vc = 3950.0f / (float)vbatMv;
    if (vc < 0.92f) vc = 0.92f; if (vc > 1.2f) vc = 1.2f;
    thr = (int)(thr * vc); if (thr > P.mthr) thr = P.mthr;
  }
''')
rep('static int g_iThr = 250;', 'static int g_iThr = 250;\nstatic int g_vcomp = 1;      // 电压补偿开关\nstatic int g_yawProbe = 1;   // 没开防自转时：离地 1.5 s 后轻推一下偏航（+60‰ 0.25 s、-60‰ 0.25 s），黑匣子里看方向\nstatic bool g_probeDone = false;\nstatic float g_oY = 0;')
rep('static void bbStart() { bbHead = 0; bbCount = 0; bbReason = 0; bbFrozen = false; }',
    'static void bbStart() { bbHead = 0; bbCount = 0; bbReason = 0; bbFrozen = false; g_probeDone = false; yawF = 0; }')
rep('''  const float oY = (P.yp > 0.001f) ? pidY.step(spY, g_gy, dt, IMAX, DFC) : 0.0f;''', '''  float oY = (P.yp > 0.001f) ? pidY.step(spY, g_gy, dt, IMAX, DFC) : 0.0f;
  {
    static uint32_t airMs = 0;
    if (thr >= g_iThr + 60) { if (!airMs) airMs = nowMs; } else airMs = 0;
    if (g_yawProbe && P.yp < 0.001f && !g_probeDone && airMs) {
      const uint32_t ta = nowMs - airMs;
      if (ta > 1500 && ta <= 1750) oY = 60;
      else if (ta > 1750 && ta <= 2000) oY = -60;
      else if (ta > 2000) g_probeDone = true;
    }
  }
  g_oY = oY;''')
rep('struct BBRec { uint32_t ms; int16_t thr, roll, pitch, gr, gp, gy, spR, spP, oR, oP, o1, o2, o3, o4, itR, itP, stR, stP; };',
    'struct BBRec { uint32_t ms; int16_t thr, roll, pitch, gr, gp, gy, spR, spP, oR, oP, o1, o2, o3, o4, itR, itP, stR, stP, oY, az, yaw; };')
rep('''    r.stR = (int16_t)(g_stkRoll * 100); r.stP = (int16_t)(g_stkPitch * 100);''',
    '''    r.stR = (int16_t)(g_stkRoll * 100); r.stP = (int16_t)(g_stkPitch * 100);
    r.oY = (int16_t)(oY * k); r.az = (int16_t)(g_az * 1000); r.yaw = (int16_t)(yawF * 10);''')
rep('''ms,thr,roll,pitch,gr,gp,gy,spR,spP,oR,oP,o1,o2,o3,o4,itR,itP,stR,stP\\n''', '''ms,thr,roll,pitch,gr,gp,gy,spR,spP,oR,oP,o1,o2,o3,o4,itR,itP,stR,stP,oY,az,yaw\\n''')
rep('''    char l[140];
    snprintf(l, sizeof(l), "%lu,%d,%.1f,%.1f,%.1f,%.1f,%.1f,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%.2f,%.2f\\n",''',
    '''    char l[170];
    snprintf(l, sizeof(l), "%lu,%d,%.1f,%.1f,%.1f,%.1f,%.1f,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%.2f,%.2f,%d,%.3f,%.1f\\n",''')
rep('''             r.spR, r.spP, r.oR, r.oP, r.o1, r.o2, r.o3, r.o4, r.itR, r.itP, r.stR / 100.0, r.stP / 100.0);''',
    '''             r.spR, r.spP, r.oR, r.oP, r.o1, r.o2, r.o3, r.o4, r.itR, r.itP, r.stR / 100.0, r.stP / 100.0,
             r.oY, r.az / 1000.0, r.yaw / 10.0);''')
# UDP 遥测末尾加偏航角
rep('''        snprintf(o, sizeof(o), "T,%u,%lu,%d,%lu,%.1f,%.1f,%.1f,%.1f,%.1f,%d,%d,%d,%d,%.2f,%.2f,%.2f,%d,%d",''',
    '''        snprintf(o, sizeof(o), "T,%u,%lu,%d,%lu,%.1f,%.1f,%.1f,%.1f,%.1f,%d,%d,%d,%d,%.2f,%.2f,%.2f,%d,%d,%.0f",''')
rep('''                 (double)g_ax, (double)g_ay, (double)g_az, (int)WiFi.RSSI(), digitalRead(PIN_PGOOD));''',
    '''                 (double)g_ax, (double)g_ay, (double)g_az, (int)WiFi.RSSI(), digitalRead(PIN_PGOOD), (double)g_yaw);''')
# ---- 卡尔曼融合（角度 + 陀螺零偏）代替互补滤波；空中加计噪声 R 调大；地上静止时持续更新陀螺零偏（偏航补偿） ----
rep('''static float rollF = 0, pitchF = 0, yawF = 0;''', '''static float rollF = 0, pitchF = 0, yawF = 0;
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
static Kal kR, kP;''')
rep('''    if (!attSeeded && amag > 0.4f) {
      rollF = rollA; pitchF = pitchA; attSeeded = true;
    } else {
      // 互补滤波：陀螺积分做高频，加计做低频（K 随 dt 归一）
      const int outAvg = (g_out[0] + g_out[1] + g_out[2] + g_out[3]) / 4;
      const float K = (g_armed && outAvg > g_iThr) ? 0.004f : 0.03f;
      rollF  = (rollF  + br * dt) * (1.0f - K) + rollA  * K;
      pitchF = (pitchF + bp * dt) * (1.0f - K) + pitchA * K;
    }''', '''    if (!attSeeded && amag > 0.4f) {
      rollF = rollA; pitchF = pitchA; kR.seed(rollA); kP.seed(pitchA); attSeeded = true;
    } else {
      // 卡尔曼融合：地上加计可信（R 小）；空中加计主要感到推力、量不准倾斜（R 大 100 倍）；
      // 加速度模长偏离 1 g 太多（碰撞、急加速）时几乎不信加计
      const int outAvg = (g_out[0] + g_out[1] + g_out[2] + g_out[3]) / 4;
      float R = (g_armed && outAvg > g_iThr) ? 3.0f : 0.03f;
      if (fabsf(amag - 1.0f) > 0.15f) R = 30.0f;
      rollF  = kR.step(rollA,  br, dt, R);
      pitchF = kP.step(pitchA, bp, dt, R);
    }
    // 偏航补偿：地上、没解锁、机身静止时，慢慢更新陀螺零偏（三轴），减少偏航角漂移
    if (!g_armed && g_mode == M_IDLE && fabsf(gx0) < 2.0f && fabsf(gy0) < 2.0f && fabsf(gz) < 2.0f && fabsf(amag - 1.0f) < 0.05f) {
      for (int i = 0; i < 3; i++) gb[i] += 0.002f * ((fg[i]) - gb[i]);
    }''')

# ---- 空中偏航试探：自己判断方向、打开防自转、落地后存进飞机（只做一次）----
rep('''      if (ta > 1500 && ta <= 1750) oY = 60;
      else if (ta > 1750 && ta <= 2000) oY = -60;
      else if (ta > 2000) g_probeDone = true;''', '''      static float gy0p = 0, gy1p = 0;
      if (ta <= 1500) gy0p = g_gy;
      if (ta > 1500 && ta <= 1750) { oY = 60; gy1p = g_gy; }
      else if (ta > 1750 && ta <= 2000) oY = -60;
      else if (ta > 2000) {
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
      }''')
rep('static bool g_probeDone = false;', 'static bool g_probeDone = false;\nstatic volatile bool g_needSave = false;\nstatic float g_probeD = 0;')
rep('''  dns.processNextRequest();
  server.handleClient();
  netLoop();''', '''  dns.processNextRequest();
  server.handleClient();
  netLoop();
  if (g_needSave && !g_armed) { g_needSave = false; paramsSave(); Serial.printf("#ACK,YAWSET,sgY=%d,d=%.1f\\n", P.sgY, (double)g_probeD); }''')
# 偏航积分默认 0.5（只有 yp>0 才起作用）
rep('static float g_yi = 0.0f;', 'static float g_yi = 0.5f;')
rep('''  snprintf(head, sizeof(head), "# seq=%lu reason=%lu frozen=%d n=%d angP=%.2f rp=%.2f ri=%.3f rd=%.3f iThr=%d\\n",
           (unsigned long)bbSeq, (unsigned long)bbReason, bbFrozen ? 1 : 0, bbCount, P.angP, P.rp, P.ri, P.rd, g_iThr);''',
    '''  snprintf(head, sizeof(head), "# seq=%lu reason=%lu frozen=%d n=%d angP=%.2f rp=%.2f ri=%.3f rd=%.3f iThr=%d yp=%.2f sgY=%d probeD=%.1f\\n",
           (unsigned long)bbSeq, (unsigned long)bbReason, bbFrozen ? 1 : 0, bbCount, P.angP, P.rp, P.ri, P.rd, g_iThr, P.yp, P.sgY, (double)g_probeD);''')
# ---- v9：角度外环加积分（消掉长期 ~1° 的静差 → 少往一边飘）；偏航试探推力加大、时间加长 ----
rep('#define FW_VER "flight-v8"', '#define FW_VER "flight-v9"')
rep('static int g_vcomp = 1;', 'static int g_vcomp = 1;\nstatic float g_angI = 1.5f;   // 角度积分（°/s 每 °·s），只在空中积，地上清零')
rep('''  float spR = P.angP * (tRoll  - rollF);
  float spP = P.angP * (tPitch - pitchF);''', '''  float spR = P.angP * (tRoll  - rollF);
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
  }''')
rep('''      if (ta > 1500 && ta <= 1750) { oY = 60; gy1p = g_gy; }
      else if (ta > 1750 && ta <= 2000) oY = -60;
      else if (ta > 2000) {''', '''      if (ta > 1500 && ta <= 1800) { oY = 90; gy1p = g_gy; }
      else if (ta > 1800 && ta <= 2100) oY = -90;
      else if (ta > 2100) {''')
rep('''      else if (!strcmp(k, "iThr")) g_iThr = (int)v;''', '''      else if (!strcmp(k, "iThr")) g_iThr = (int)v;
      else if (!strcmp(k, "angI")) g_angI = v;''')
rep('''  else if (k == "iThr") g_iThr = (int)v;''', '''  else if (k == "iThr") g_iThr = (int)v;
  else if (k == "angI") g_angI = v;''')
for key, var in (("vcomp", "g_vcomp"), ("yprobe", "g_yawProbe")):
    rep('''      else if (!strcmp(k, "yi")) g_yi = v;''', '''      else if (!strcmp(k, "yi")) g_yi = v;
      else if (!strcmp(k, "%s")) %s = (int)v;''' % (key, var))
    rep('''  else if (k == "yi") g_yi = v;''', '''  else if (k == "yi") g_yi = v;
  else if (k == "%s") %s = (int)v;''' % (key, var))
# ---- v10：重心补偿 bR/bP（‰）。地上不再把角速度积分清零，而是放成 bR/bP，
# 起飞瞬间重的那边就已经多给力，不会一离地先往重的那边歪（之前每次起飞都往后歪 4~14°）。
# 存 NVS；电脑端每次飞完按空中积分平均值自动学。
rep('#define FW_VER "flight-v9"', '#define FW_VER "flight-v10"')
rep('static float g_rot = 0.0f;', 'static float g_rot = 0.0f;\nstatic float g_bR = 0.0f, g_bP = 0.0f;   // 重心补偿（‰），起飞前就加上')
rep('''  prefs.putFloat("rot", g_rot);''', '''  prefs.putFloat("rot", g_rot);
  prefs.putFloat("bR", g_bR); prefs.putFloat("bP", g_bP);''')
rep('''    g_rot = prefs.getFloat("rot", 0.0f);''', '''    g_rot = prefs.getFloat("rot", 0.0f);
    g_bR = prefs.getFloat("bR", 0.0f); g_bP = prefs.getFloat("bP", 0.0f);''')
rep('''  if (thr < g_iThr) { pidR.it = 0; pidP.it = 0; }''',
    '''  if (thr < g_iThr) {
    // 解锁且推了油门：积分放成重心补偿；没推油门（电机停）就清零
    pidR.it = (thr > 0) ? g_bR : 0.0f; pidP.it = (thr > 0) ? g_bP : 0.0f;
  }''')
rep('''           "\\"sgR\\":%d,\\"sgP\\":%d,\\"sgY\\":%d,\\"rot\\":%.1f}",''',
    '''           "\\"sgR\\":%d,\\"sgP\\":%d,\\"sgY\\":%d,\\"rot\\":%.1f,\\"bR\\":%.1f,\\"bP\\":%.1f}",''')
rep('''           P.sgR, P.sgP, P.sgY, (double)g_rot);''', '''           P.sgR, P.sgP, P.sgY, (double)g_rot, (double)g_bR, (double)g_bP);''')
for key in ("bR", "bP"):
    rep('''      else if (!strcmp(k, "angI")) g_angI = v;''', '''      else if (!strcmp(k, "angI")) g_angI = v;
      else if (!strcmp(k, "%s")) g_%s = (v > 80 ? 80 : (v < -80 ? -80 : v));''' % (key, key))
    rep('''  else if (k == "angI") g_angI = v;''', '''  else if (k == "angI") g_angI = v;
  else if (k == "%s") g_%s = (v > 80 ? 80 : (v < -80 ? -80 : v));''' % (key, key))
# ---- v11：推算位置保持。空中用姿态积分出水平速度/位移（和录像对过：1.5 s 推算 0.46 m、实测 0.48 m），
# 反向加角度目标：刹住速度、慢慢拉回起飞点。按方向键时不管、并清零。
rep('#define FW_VER "flight-v10"', '#define FW_VER "flight-v11"')
rep('static float g_bR = 0.0f, g_bP = 0.0f;   // 重心补偿（‰），起飞前就加上',
    '''static float g_bR = 0.0f, g_bP = 0.0f;   // 重心补偿（‰），起飞前就加上
static float g_vh = 8.0f;     // 速度刹车：每 1 m/s 反向倾 g_vh 度（0 = 关）
static float g_ph = 3.0f;     // 回原点：每 1 m 反向倾 g_ph 度
static float g_vleak = 0.3f;  // 推算速度慢慢回零（1/s），防误差越积越大
static float g_hmax = 8.0f;   // 修正角最多几度
static float g_vx = 0, g_vy = 0, g_px = 0, g_py = 0;   // 推算速度（m/s）、位移（m），机体系：x = pitch+ 方向，y = roll+ 方向''')
rep('''  const float tRoll  = g_stkRoll  * P.mang;
  const float tPitch = g_stkPitch * P.mang;''', '''  float tRoll  = g_stkRoll  * P.mang;
  float tPitch = g_stkPitch * P.mang;
  {
    const bool stick = fabsf(g_stkRoll) > 0.01f || fabsf(g_stkPitch) > 0.01f;
    if (thr >= g_iThr && !stick && g_vh > 0.001f) {
      g_vx += (9.81f * tanf(pitchF * 0.0174533f) - g_vleak * g_vx) * dt;
      g_vy += (9.81f * tanf(rollF  * 0.0174533f) - g_vleak * g_vy) * dt;
      g_px += g_vx * dt; g_py += g_vy * dt;
      if (g_px > 1.5f) g_px = 1.5f; if (g_px < -1.5f) g_px = -1.5f;
      if (g_py > 1.5f) g_py = 1.5f; if (g_py < -1.5f) g_py = -1.5f;
      float cP = -(g_vh * g_vx + g_ph * g_px), cR = -(g_vh * g_vy + g_ph * g_py);
      if (cP > g_hmax) cP = g_hmax; if (cP < -g_hmax) cP = -g_hmax;
      if (cR > g_hmax) cR = g_hmax; if (cR < -g_hmax) cR = -g_hmax;
      tPitch += cP; tRoll += cR;
    } else { g_vx = g_vy = g_px = g_py = 0; }
  }''')
rep('struct BBRec { uint32_t ms; int16_t thr, roll, pitch, gr, gp, gy, spR, spP, oR, oP, o1, o2, o3, o4, itR, itP, stR, stP, oY, az, yaw; };',
    'struct BBRec { uint32_t ms; int16_t thr, roll, pitch, gr, gp, gy, spR, spP, oR, oP, o1, o2, o3, o4, itR, itP, stR, stP, oY, az, yaw, vx, vy, px, py; };')
rep('''    r.oY = (int16_t)(oY * k); r.az = (int16_t)(g_az * 1000); r.yaw = (int16_t)(yawF * 10);''',
    '''    r.oY = (int16_t)(oY * k); r.az = (int16_t)(g_az * 1000); r.yaw = (int16_t)(yawF * 10);
    r.vx = (int16_t)(g_vx * 100); r.vy = (int16_t)(g_vy * 100); r.px = (int16_t)(g_px * 100); r.py = (int16_t)(g_py * 100);''')
rep('''  server.sendContent("ms,thr,roll,pitch,gr,gp,gy,spR,spP,oR,oP,o1,o2,o3,o4,itR,itP,stR,stP,oY,az,yaw\\n");''',
    '''  server.sendContent("ms,thr,roll,pitch,gr,gp,gy,spR,spP,oR,oP,o1,o2,o3,o4,itR,itP,stR,stP,oY,az,yaw,vx,vy,px,py\\n");''')
rep('''    char l[170];
    snprintf(l, sizeof(l), "%lu,%d,%.1f,%.1f,%.1f,%.1f,%.1f,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%.2f,%.2f,%d,%.3f,%.1f\\n",''',
    '''    char l[200];
    snprintf(l, sizeof(l), "%lu,%d,%.1f,%.1f,%.1f,%.1f,%.1f,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%.2f,%.2f,%d,%.3f,%.1f,%.2f,%.2f,%.2f,%.2f\\n",''')
rep('''             r.oY, r.az / 1000.0, r.yaw / 10.0);''',
    '''             r.oY, r.az / 1000.0, r.yaw / 10.0, r.vx / 100.0, r.vy / 100.0, r.px / 100.0, r.py / 100.0);''')
for key in ("vh", "ph", "vleak", "hmax"):
    rep('''      else if (!strcmp(k, "angI")) g_angI = v;''', '''      else if (!strcmp(k, "angI")) g_angI = v;
      else if (!strcmp(k, "%s")) g_%s = v;''' % (key, key))
    rep('''  else if (k == "angI") g_angI = v;''', '''  else if (k == "angI") g_angI = v;
  else if (k == "%s") g_%s = v;''' % (key, key))
# ---- v12：阻力测速。空中加计只感到推力 + 空气阻力，推力沿机身 z，所以“加计倾角”（减过 trim）
# 在空中 ≈ 阻力 / g ≈ μ·v / g：飞机往哪漂、漂多快，加计能直接看到，和水平校准差多少无关。
# 低通后换算成速度 vdx/vdy = tan(加计倾角)·g/μ，用 dk（1/s）把推算速度往它拉（dk=0 = 关，和 v11 一样）。
# 先只记录（黑匣子加 ap/ar/vdx/vdy 四列），飞几次用数据拟合 μ（符号也由数据定），再 E,dk 打开。
rep('#define FW_VER "flight-v11"', '#define FW_VER "flight-v12"')
rep('static float g_vx = 0, g_vy = 0, g_px = 0, g_py = 0;',
    '''static float g_vx = 0, g_vy = 0, g_px = 0, g_py = 0;
static float g_accR = 0, g_accP = 0;   // 只用加计算的倾角（度，减过 trim）：地上 = 倾斜，空中 ≈ 阻力
static float g_apf = 0, g_arf = 0;     // 空中低通后的 tan(加计倾角)
static float g_mu = 0.5f;              // 阻力系数（1/s）：阻力加速度 = μ·速度；负数 = 方向反过来
static float g_dk = 0.0f;              // 推算速度往阻力测速拉的力度（1/s），0 = 关
static float g_vdx = 0, g_vdy = 0;     // 阻力测出的速度（m/s）''')
rep('''    float pitchA = (float)P.sgP * atan2f(-fax, sqrtf(fay * fay + fa[2] * fa[2])) * R2D - P.trimP;''',
    '''    float pitchA = (float)P.sgP * atan2f(-fax, sqrtf(fay * fay + fa[2] * fa[2])) * R2D - P.trimP;
    g_accR = rollA; g_accP = pitchA;''')
rep('''    if (thr >= g_iThr && !stick && g_vh > 0.001f) {
      g_vx += (9.81f * tanf(pitchF * 0.0174533f) - g_vleak * g_vx) * dt;
      g_vy += (9.81f * tanf(rollF  * 0.0174533f) - g_vleak * g_vy) * dt;''',
    '''    if (thr >= g_iThr) {   // 阻力测速：空中就算（不管开没开），方便记录
      g_apf += (tanf(g_accP * 0.0174533f) - g_apf) * dt / 0.25f;
      g_arf += (tanf(g_accR * 0.0174533f) - g_arf) * dt / 0.25f;
      const float mu = (fabsf(g_mu) < 0.05f) ? 0.05f : g_mu;
      g_vdx = g_apf * 9.81f / mu; g_vdy = g_arf * 9.81f / mu;
    } else { g_apf = g_arf = 0; g_vdx = g_vdy = 0; }
    if (thr >= g_iThr && !stick && g_vh > 0.001f) {
      g_vx += (9.81f * tanf(pitchF * 0.0174533f) - g_vleak * g_vx + g_dk * (g_vdx - g_vx)) * dt;
      g_vy += (9.81f * tanf(rollF  * 0.0174533f) - g_vleak * g_vy + g_dk * (g_vdy - g_vy)) * dt;''')
rep('az, yaw, vx, vy, px, py; };', 'az, yaw, vx, vy, px, py, ap, ar, vdx, vdy; };')
rep('''    r.vx = (int16_t)(g_vx * 100); r.vy = (int16_t)(g_vy * 100); r.px = (int16_t)(g_px * 100); r.py = (int16_t)(g_py * 100);''',
    '''    r.vx = (int16_t)(g_vx * 100); r.vy = (int16_t)(g_vy * 100); r.px = (int16_t)(g_px * 100); r.py = (int16_t)(g_py * 100);
    r.ap = (int16_t)(tanf(g_accP * 0.0174533f) * 10000); r.ar = (int16_t)(tanf(g_accR * 0.0174533f) * 10000);
    r.vdx = (int16_t)(g_vdx * 100); r.vdy = (int16_t)(g_vdy * 100);''')
rep('stP,oY,az,yaw,vx,vy,px,py\\n");', 'stP,oY,az,yaw,vx,vy,px,py,ap,ar,vdx,vdy\\n");')
rep('''    char l[200];''', '''    char l[240];''')
rep('''%.3f,%.1f,%.2f,%.2f,%.2f,%.2f\\n",''', '''%.3f,%.1f,%.2f,%.2f,%.2f,%.2f,%.4f,%.4f,%.2f,%.2f\\n",''')
rep('''r.vx / 100.0, r.vy / 100.0, r.px / 100.0, r.py / 100.0);''',
    '''r.vx / 100.0, r.vy / 100.0, r.px / 100.0, r.py / 100.0,
             r.ap / 10000.0, r.ar / 10000.0, r.vdx / 100.0, r.vdy / 100.0);''')
for key in ("mu", "dk"):
    rep('''      else if (!strcmp(k, "angI")) g_angI = v;''', '''      else if (!strcmp(k, "angI")) g_angI = v;
      else if (!strcmp(k, "%s")) g_%s = v;''' % (key, key))
    rep('''  else if (k == "angI") g_angI = v;''', '''  else if (k == "angI") g_angI = v;
  else if (k == "%s") g_%s = v;''' % (key, key))
# ---- v13：平稳起飞 + 定高（只用自己的加计）。数据：60% 一键起飞后贴地 10~15 cm 上下弹（地效 + 固定油门）。
# 地上（没解锁或油门 0）把竖直加计读数当 1 g 标定；空中积分出竖直速度 vz、高度 hz。
# 起飞后目标高度从 0 平滑升到 ht（爬升 climb，快到时放慢），油门 = 学到的悬停油门 hov + PID(高度, 速度, 积分)，
# 不用一键油门值（60/70% 都一样）。动了油门（降落按钮、拉杆）就交回手动。落地后按积分平均值修正 hov 并存 NVS。
rep('#define FW_VER "flight-v12"', '#define FW_VER "flight-v13"')
rep('static float g_vh = 8.0f;', '''static float g_hov = 405.0f;  // 学到的悬停油门（‰，电压补偿前），存 NVS
static float g_ah = 1.0f;     // 定高开关/力度（0 = 关）
static float g_ht = 0.4f;     // 起飞目标高度（m）
static float g_climb = 0.3f;  // 爬升速度（m/s）
static float g_khp = 150.0f, g_khv = 250.0f, g_khi = 60.0f;   // ‰/m、‰/(m/s)、‰/(m·s)
static float g_vz = 0, g_hz = 0, g_htg = 0;   // 竖直速度、高度、目标高度（记录用）
static float g_vh = 8.0f;''')
rep('''  prefs.putFloat("bR", g_bR); prefs.putFloat("bP", g_bP);''', '''  prefs.putFloat("bR", g_bR); prefs.putFloat("bP", g_bP);
  prefs.putFloat("hov", g_hov);''')
rep('''    g_bR = prefs.getFloat("bR", 0.0f); g_bP = prefs.getFloat("bP", 0.0f);''', '''    g_bR = prefs.getFloat("bR", 0.0f); g_bP = prefs.getFloat("bP", 0.0f);
    g_hov = prefs.getFloat("hov", g_hov);''')
rep('''  if (thr < 0) thr = 0; if (thr > P.mthr) thr = P.mthr;
  if (g_vcomp && vbatMv > 3000 && thr > 0) {''', '''  if (thr < 0) thr = 0; if (thr > P.mthr) thr = P.mthr;
  {
    static float az0 = 1.0f, iz = 0, iSum = 0; static int iN = 0, thr0 = -1; static bool on = false, wasOn = false;
    const float tilt = cosf(rollF * 0.0174533f) * cosf(pitchF * 0.0174533f);
    if (!g_armed || g_thr == 0) {
      // 地上：竖直加计读数当 1 g（机身静止时慢慢跟）
      if (fabsf(g_gr) < 5.0f && fabsf(g_gp) < 5.0f && g_az > 0.5f) az0 += (g_az * tilt - az0) * dt / 0.5f;
      if (wasOn && iN > 250) {   // 这次定高跑过：按积分平均值修正悬停油门
        g_hov += 0.6f * (iSum / (float)iN);
        if (g_hov < 250) g_hov = 250; if (g_hov > 650) g_hov = 650;
        g_needSave = true;
      }
      g_vz = g_hz = g_htg = 0; iz = iSum = 0; iN = 0; thr0 = -1; on = wasOn = false;
    } else {
      if (thr0 < 0) thr0 = g_thr;
      const bool moved = abs(g_thr - thr0) > 30;
      const float av = (g_az * tilt / az0 - 1.0f) * 9.81f;
      g_vz += av * dt; g_hz += g_vz * dt;
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
      }
    }
  }
  if (g_vcomp && vbatMv > 3000 && thr > 0) {''')
rep('vx, vy, px, py, ap, ar, vdx, vdy; };', 'vx, vy, px, py, ap, ar, vdx, vdy, vz, hz, htg; };')
rep('''    r.vdx = (int16_t)(g_vdx * 100); r.vdy = (int16_t)(g_vdy * 100);''', '''    r.vdx = (int16_t)(g_vdx * 100); r.vdy = (int16_t)(g_vdy * 100);
    r.vz = (int16_t)(g_vz * 100); r.hz = (int16_t)(g_hz * 100); r.htg = (int16_t)(g_htg * 100);''')
rep('px,py,ap,ar,vdx,vdy\\n");', 'px,py,ap,ar,vdx,vdy,vz,hz,htg\\n");')
rep('''    char l[240];''', '''    char l[280];''')
rep('''%.4f,%.4f,%.2f,%.2f\\n",''', '''%.4f,%.4f,%.2f,%.2f,%.2f,%.2f,%.2f\\n",''')
rep('''             r.ap / 10000.0, r.ar / 10000.0, r.vdx / 100.0, r.vdy / 100.0);''',
    '''             r.ap / 10000.0, r.ar / 10000.0, r.vdx / 100.0, r.vdy / 100.0, r.vz / 100.0, r.hz / 100.0, r.htg / 100.0);''')
rep('''\\"rot\\":%.1f,\\"bR\\":%.1f,\\"bP\\":%.1f}",''', '''\\"rot\\":%.1f,\\"bR\\":%.1f,\\"bP\\":%.1f,\\"hov\\":%.1f}",''')
rep('''(double)g_rot, (double)g_bR, (double)g_bP);''', '''(double)g_rot, (double)g_bR, (double)g_bP, (double)g_hov);''')
for key in ("hov", "ah", "ht", "climb", "khp", "khv", "khi"):
    rep('''      else if (!strcmp(k, "angI")) g_angI = v;''', '''      else if (!strcmp(k, "angI")) g_angI = v;
      else if (!strcmp(k, "%s")) g_%s = v;''' % (key, key))
    rep('''  else if (k == "angI") g_angI = v;''', '''  else if (k == "angI") g_angI = v;
  else if (k == "%s") g_%s = v;''' % (key, key))
# ---- v14：v13 定高实测不行（振动让竖直加计偏几 % g、随油门变，积分 2 s 就差几十 cm；起飞一冲到 550）→ 默认关。
# 改成：起飞油门 0.3 s 软启动 + 只压上下弹（竖直速度高通 τ 0.6 s，慢的偏差不累积），高度仍由油门决定。
# 数据：60% 时飞机浮在地效里 10~15 cm、上下弹 ±0.25 m/s（周期 ~1.5 s）；仿真压弹后摆幅 22 → 5~7 cm。
rep('#define FW_VER "flight-v13"', '#define FW_VER "flight-v14"')
rep('static float g_ah = 1.0f;     // 定高开关/力度（0 = 关）',
    '''static float g_ah = 0.0f;     // 定高开关/力度（0 = 关；v13 实测估计会漂，默认关）
static float g_kdz = 200.0f;  // 压上下弹：每 1 m/s 竖直速度（高通）反向 g_kdz ‰（0 = 关）
static float g_tz = 0.6f;     // 竖直速度高通时间常数（s）
static float g_dzmax = 60.0f; // 压弹最多加减多少 ‰
static float g_tramp = 0.3f;  // 起飞油门软启动（s）''')
rep('''    static float az0 = 1.0f, iz = 0, iSum = 0; static int iN = 0, thr0 = -1; static bool on = false, wasOn = false;''',
    '''    static float az0 = 1.0f, iz = 0, iSum = 0, vzh = 0, tOn = 0; static int iN = 0, thr0 = -1; static bool on = false, wasOn = false;''')
rep('''      g_vz = g_hz = g_htg = 0; iz = iSum = 0; iN = 0; thr0 = -1; on = wasOn = false;''',
    '''      g_vz = g_hz = g_htg = 0; iz = iSum = 0; iN = 0; thr0 = -1; on = wasOn = false; vzh = 0; tOn = 0;''')
rep('''      g_vz += av * dt; g_hz += g_vz * dt;''',
    '''      g_vz += av * dt; g_hz += g_vz * dt;
      vzh += (av - vzh / (g_tz > 0.05f ? g_tz : 0.05f)) * dt;   // 高通竖直速度：只看快的上下动
      tOn += dt;''')
rep('''        if (g_htg >= 0.98f * g_ht) { iSum += g_ah * g_khi * iz; iN++; }
      }''', '''        if (g_htg >= 0.98f * g_ht) { iSum += g_ah * g_khi * iz; iN++; }
      } else {
        // 软启动：起飞那一下 g_tramp 秒内从 0 爬到指令油门
        if (g_tramp > 0.01f && tOn < g_tramp) thr = (int)(thr * (tOn / g_tramp));
        // 压上下弹（离地后才有意义；油门太低时不加）
        if (thr >= g_iThr && tOn > 0.1f) {
          float dz = -g_kdz * vzh;
          if (dz > g_dzmax) dz = g_dzmax; if (dz < -g_dzmax) dz = -g_dzmax;
          thr += (int)dz; if (thr < 0) thr = 0; if (thr > P.mthr) thr = P.mthr;
          g_htg = dz / 100.0f;      // 记录：压弹量（‰/100）
        }
        g_vz = vzh;                 // 记录：高通竖直速度
      }''')
rep('px,py,ap,ar,vdx,vdy,vz,hz,htg\\n");', 'px,py,ap,ar,vdx,vdy,vz,hz,dz\\n");')
for key in ("kdz", "tz", "dzmax", "tramp"):
    rep('''      else if (!strcmp(k, "angI")) g_angI = v;''', '''      else if (!strcmp(k, "angI")) g_angI = v;
      else if (!strcmp(k, "%s")) g_%s = v;''' % (key, key))
    rep('''  else if (k == "angI") g_angI = v;''', '''  else if (k == "angI") g_angI = v;
  else if (k == "%s") g_%s = v;''' % (key, key))
# ---- v15：机头锁定。18:17~18:19 的飞行里机头每次转 25~55°，方向键（按机头算）就乱了方向。
# 空中没按偏航键时，目标偏航角速度加 −ya·(当前偏航 − 起飞时偏航)，限 ±60°/s。yawF 是同一套陀螺积分（解锁清零），符号自洽。
rep('#define FW_VER "flight-v14"', '#define FW_VER "flight-v15"')
rep('static float g_vh = 8.0f;', 'static float g_ya = 2.0f;     // 机头锁定：每偏 1° 往回转 g_ya °/s（0 = 关）\nstatic float g_vh = 8.0f;')
rep('''  float spY = g_stkYaw * 120.0f;''', '''  float spY = g_stkYaw * 120.0f;
  {
    static float yref = 0;
    if (thr < g_iThr || fabsf(g_stkYaw) > 0.01f || g_ya < 0.001f) yref = yawF;   // 地上 / 手动转向时跟着走
    else {
      float e = yawF - yref; if (e > 180) e -= 360; if (e < -180) e += 360;
      float c = -g_ya * e; if (c > 60) c = 60; if (c < -60) c = -60;
      spY += c;
    }
  }''')
rep('''      else if (!strcmp(k, "angI")) g_angI = v;''', '''      else if (!strcmp(k, "angI")) g_angI = v;
      else if (!strcmp(k, "ya")) g_ya = v;''')
rep('''  else if (k == "angI") g_angI = v;''', '''  else if (k == "angI") g_angI = v;
  else if (k == "ya") g_ya = v;''')
# ---- v16：手机遥控 + 热点优先（11-15，电脑装了 USB 网卡直连热点 DroneBench）
# ① 手机网页不能发 UDP，原手机页用 HTTP 发心跳，网页服务一卡就 0.9 s 链路超时 → 加 WebSocket（端口 81，独立任务），
#    收发格式和 UDP 一样（U → T，X 急停）。新手机遥控页放 "/"，原调试页挪到 "/bench"。
# ② 连保存的 WiFi（SCF-WiFi）失败时会反复扫描/重连，热点跟着换信道、手机掉线 → 关自动重连；
#    解锁中或热点上有设备时不去连。
# ③ 遥测 rssi：发给热点上的设备时报热点这边的信号（以前报的是连路由器那边的）。
# ④ 机头锁定 ya 默认 3（以前靠电脑页面每次发），手机单飞也一样。
import os as _os
rep('#define FW_VER "flight-v15"', '#define FW_VER "flight-v16"')
rep('static float g_ya = 2.0f;', 'static float g_ya = 3.0f;')
rep('#include <Update.h>\n', '#include <Update.h>\n#include <esp_wifi.h>\n#include "mbedtls/sha1.h"\n#include "mbedtls/base64.h"\n')
rep('''  WiFi.setSleep(false);\n''', '''  WiFi.setSleep(false);\n  WiFi.setAutoReconnect(false);   // v16：别让路由器连不上时反复扫描，热点会跟着断\n''')
rep('''static void netLoop() {''', '''static volatile int g_apRssi = 0;   // 热点上连着的设备里最弱的信号（dBm，0 = 没人）
static void netLoop() {''')
rep('''  if (!up && netSsid.length() && now - netTry > 20000) netBegin();''',
    '''  {
    wifi_sta_list_t sl; int m = 0;
    if (esp_wifi_ap_get_sta_list(&sl) == ESP_OK)
      for (int i = 0; i < sl.num; i++) if (m == 0 || sl.sta[i].rssi < m) m = sl.sta[i].rssi;
    g_apRssi = m;
  }
  // v16：解锁中、或热点上有设备在用时不去连路由器（连接/扫描会让热点换信道、掉线）
  if (!up && netSsid.length() && now - netTry > 20000 && !g_armed && WiFi.softAPgetStationNum() == 0) netBegin();''')
rep('''static WiFiUDP udp;''', '''static int telem(char* o, size_t n, unsigned seq, bool ap) {
  return snprintf(o, n, "T,%u,%lu,%d,%lu,%.1f,%.1f,%.1f,%.1f,%.1f,%d,%d,%d,%d,%.2f,%.2f,%.2f,%d,%d,%.0f",
                  seq, (unsigned long)vbatMv, g_armed ? 1 : 0, (unsigned long)(g_fault | (imuOK ? 0 : F_IMU)),
                  (double)g_roll, (double)g_pitch, (double)g_gr, (double)g_gp, (double)g_gy,
                  (int)g_out[0], (int)g_out[1], (int)g_out[2], (int)g_out[3],
                  (double)g_ax, (double)g_ay, (double)g_az, ap ? (int)g_apRssi : (int)WiFi.RSSI(),
                  digitalRead(PIN_PGOOD), (double)g_yaw);
}
static WiFiUDP udp;''')
rep('''        snprintf(o, sizeof(o), "T,%u,%lu,%d,%lu,%.1f,%.1f,%.1f,%.1f,%.1f,%d,%d,%d,%d,%.2f,%.2f,%.2f,%d,%d,%.0f",
                 seq, (unsigned long)vbatMv, g_armed ? 1 : 0, (unsigned long)g_fault,
                 (double)g_roll, (double)g_pitch, (double)g_gr, (double)g_gp, (double)g_gy,
                 (int)g_out[0], (int)g_out[1], (int)g_out[2], (int)g_out[3],
                 (double)g_ax, (double)g_ay, (double)g_az, (int)WiFi.RSSI(), digitalRead(PIN_PGOOD), (double)g_yaw);''',
    '''        { const IPAddress ri = udp.remoteIP(); telem(o, sizeof(o), seq, ri[0] == 192 && ri[1] == 168 && ri[2] == 4); }''')
rep('''static void sendJson(const char* s) {''', r'''// ---------------------------------------------------------------- 手机遥控通道（v16）
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
          if (live && g_armed) {
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

static void sendJson(const char* s) {''')
_page = open(_os.path.join(_os.path.dirname(_os.path.abspath(__file__)), 'phone_page.html'), encoding='utf-8').read()
assert ')HTML"' not in _page
rep('''static const char OTA_HTML[] PROGMEM''', 'static const char PHONE_HTML[] PROGMEM = R"HTML(' + _page + ''')HTML";

static const char OTA_HTML[] PROGMEM''')
rep('''  server.on("/", []() { server.send_P(200, "text/html; charset=utf-8", PAGE_HTML); });''',
    '''  server.on("/", []() { server.send_P(200, "text/html; charset=utf-8", PHONE_HTML); });
  server.on("/bench", []() { server.send_P(200, "text/html; charset=utf-8", PAGE_HTML); });
  server.on("/api/ver", []() { sendJson("{\\"v\\":\\"" FW_VER "\\"}"); });''')
rep('''  xTaskCreatePinnedToCore(udpTask, "udp", 4096, nullptr, 3, nullptr, 0);''',
    '''  xTaskCreatePinnedToCore(udpTask, "udp", 4096, nullptr, 3, nullptr, 0);
  wsSrv.begin(); wsSrv.setNoDelay(true);
  xTaskCreatePinnedToCore(wsTask, "ws", 6144, nullptr, 3, nullptr, 0);''')
# ⑤ IMU：11-15 重焊电机线后 WHO_AM_I 正常、数据全 0，原逻辑仍判“就绪”还能解锁 → 先软复位再配置、回读核对并打印；
#    零偏标定时加计模长必须在 0.5~1.5 g，否则判 IMU 故障（禁止解锁）。
rep('''    if (imuR(0x00, &who, 1) && who == 0x05) {
      imuW(0x02, 0x40);    // CTRL1 地址自增''', '''    if (imuR(0x00, &who, 1) && who == 0x05) {
      imuW(0x60, 0xB0);    // v16：软复位（重焊/上电不干净时传感器可能停在没开的状态）
      delay(30);
      for (int w = 0; w < 20 && !(imuR(0x00, &who, 1) && who == 0x05); w++) delay(10);
      imuW(0x02, 0x40);    // CTRL1 地址自增''')
rep('''      imuW(0x08, 0x03);    // CTRL7 使能
      delay(120);
      return true;''', '''      imuW(0x08, 0x03);    // CTRL7 使能
      delay(120);
      {
        uint8_t c1 = 0, c2 = 0, c3 = 0, c7 = 0, raw[12] = {0};
        imuR(0x02, &c1, 1); imuR(0x03, &c2, 1); imuR(0x04, &c3, 1); imuR(0x08, &c7, 1); imuR(0x35, raw, 12);
        Serial.printf("#IMU,cfg,c1=%02X,c2=%02X,c3=%02X,c7=%02X,raw=%02X%02X,%02X%02X,%02X%02X,%02X%02X\\n",
                      c1, c2, c3, c7, raw[1], raw[0], raw[3], raw[2], raw[5], raw[4], raw[7], raw[6]);
        if (c7 != 0x03) { imuW(0x08, 0x03); delay(60); }
      }
      return true;''')
rep('''  float s[3] = {0, 0, 0};
  int n = 0;
  float aa[3], gg[3];''', '''  float s[3] = {0, 0, 0}, am = 0;
  int n = 0;
  float aa[3], gg[3];''')
rep('''    if (ok && el > dur * 2 / 5) { s[0] += gg[0]; s[1] += gg[1]; s[2] += gg[2]; n++; }''',
    '''    if (ok && el > dur * 2 / 5) { s[0] += gg[0]; s[1] += gg[1]; s[2] += gg[2]; am += sqrtf(aa[0] * aa[0] + aa[1] * aa[1] + aa[2] * aa[2]); n++; }''')
rep('''  if (n < 30) return false;
  float b[3];''', '''  if (n < 30) return false;
  am /= (float)n;
  if (am < 0.5f || am > 1.5f) { Serial.printf("#ERR,IMU_ACC,%.2f\\n", (double)am); return false; }   // v16：数据全 0 也算坏
  float b[3];''')
# ⑥ IMU 诊断：GET /api/imureg → 控制任务里读 0x00~0x60 全部寄存器（避免 I2C 跨任务），回十六进制 + 时间戳/温度
rep('''static volatile bool levelReq = false;''', '''static volatile bool levelReq = false;
static volatile bool imuDumpReq = false;
static char imuDump[520] = "";''')
rep('''  // ---- 零偏标定（控制任务里做，避免 I2C 跨任务竞争）----''', '''  if (imuDumpReq) {
    imuDumpReq = false;
    uint8_t r[0x61]; memset(r, 0xEE, sizeof(r));
    bool ok = true;
    for (int a = 0; a < 0x61; a += 16) { const int n = (0x61 - a) < 16 ? (0x61 - a) : 16; ok &= imuR(a, r + a, n); }
    uint8_t q[12] = {0}; const bool okq = imuR(0x35, q, 12);
    float fa[3] = {0, 0, 0}, fg2[3] = {0, 0, 0}; const bool okf = imuRead(fa, fg2);
    char* p = imuDump; int left = sizeof(imuDump);
    int w = snprintf(p, left, "{\\"r35\\":\\"%d:%02X%02X%02X%02X%02X%02X\\",\\"rd\\":\\"%d:%.3f,%.3f,%.3f\\",",
                     okq ? 1 : 0, q[0], q[1], q[2], q[3], q[4], q[5], okf ? 1 : 0, (double)fa[0], (double)fa[1], (double)fa[2]);
    p += w; left -= w;
    w = snprintf(p, left, "\\"ok\\":%d,\\"ts\\":%u,\\"temp\\":%.2f,\\"st0\\":\\"%02X\\",\\"st1\\":\\"%02X\\",\\"regs\\":\\"",
                     ok ? 1 : 0, (unsigned)(r[0x30] | (r[0x31] << 8) | (r[0x32] << 16)),
                     (double)((int8_t)r[0x34] + r[0x33] / 256.0f), r[0x2E], r[0x2F]);
    p += w; left -= w;
    for (int a = 0; a < 0x61 && left > 4; a++) { w = snprintf(p, left, "%02X", r[a]); p += w; left -= w; }
    snprintf(p, left, "\\"}");
  }
  // ---- 零偏标定（控制任务里做，避免 I2C 跨任务竞争）----''')
rep('''static void handleZero() {''', '''static void handleImuReg() {
  imuDump[0] = 0; imuDumpReq = true;
  for (int i = 0; i < 40 && !imuDump[0]; i++) delay(5);
  sendJson(imuDump[0] ? imuDump : "{\\"ok\\":0}");
}
static void handleZero() {''')
rep('''  server.on("/api/ver", []() {''', '''  server.on("/api/imureg", handleImuReg);
  server.on("/api/ver", []() {''')
rep('选一个 .bin（arduino-cli 编译产物 flight_v1.ino.bin）', '选一个 .bin（编译产物 flight_vNN.ino.bin）')
rep('只接受本机编译的 <i>flight_v1.ino.bin</i>', '只接受本机编译的 <i>flight_vNN.ino.bin</i>')
# ================= v17（11-16）：不动硬件，软件补 M2/M3 弱 =================
# 实测悬停时偏航要一直顶 oY≈-30~-100（o2/o3 比 o1/o4 高 ~25%）才不转；
# ① 偏航前馈 yff：离地就先加上，偏航 PID 不用从 0 慢慢顶；② 混控里偏航优先级最低：
#    先保横滚/俯仰，偏航只用剩下的余量（顶满时宁可慢慢转，也不让偏航把姿态挤掉而翻）
rep('#define FW_VER "flight-v16"', '#define FW_VER "flight-v17"')
rep('static float g_hov = 405.0f;', 'static float g_yff = -60.0f;  // v17 偏航前馈（‰，混控 oY 单位），离地后加，存 NVS\nstatic float g_hov = 405.0f;')
rep('  prefs.putFloat("hov", g_hov);', '  prefs.putFloat("hov", g_hov); prefs.putFloat("yff", g_yff);')
rep('    g_hov = prefs.getFloat("hov", g_hov);', '    g_hov = prefs.getFloat("hov", g_hov); g_yff = prefs.getFloat("yff", g_yff);')
rep('  else if (k == "hov") g_hov = v;', '  else if (k == "hov") g_hov = v;\n  else if (k == "yff") g_yff = (v > 200 ? 200 : (v < -200 ? -200 : v));')
rep('      else if (!strcmp(k, "hov")) g_hov = v;', '      else if (!strcmp(k, "hov")) g_hov = v;\n      else if (!strcmp(k, "yff")) g_yff = (v > 200 ? 200 : (v < -200 ? -200 : v));')
rep('\\"bP\\":%.1f,\\"hov\\":%.1f}",', '\\"bP\\":%.1f,\\"hov\\":%.1f,\\"yff\\":%.1f}",')
rep('(double)g_bR, (double)g_bP, (double)g_hov);', '(double)g_bR, (double)g_bP, (double)g_hov, (double)g_yff);')
rep('''  float oY = (P.yp > 0.001f) ? pidY.step(spY, g_gy, dt, IMAX, DFC) : 0.0f;''',
    '''  float oY = (P.yp > 0.001f) ? pidY.step(spY, g_gy, dt, IMAX, DFC) : 0.0f;
  if (P.yp > 0.001f && thr >= g_iThr) oY += g_yff;   // v17 偏航前馈''')
rep('''  float c[4];
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
    int v = thr + (int)lrintf(c[i]);''', '''  // v17：先排横滚/俯仰，偏航只用剩下的余量
  static const float sy[4] = {1.0f, -1.0f, -1.0f, 1.0f};
  float c[4];
  c[0] = -oP + oR;
  c[1] = -oP - oR;
  c[2] =  oP + oR;
  c[3] =  oP - oR;
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
    int v = thr + (int)lrintf(c[i]);''')
# v17 诊断：/api/ver 带开机时长 up(ms)、上次重启原因 rr（esp_reset_reason：1 上电 3 软件 4 异常 9 欠压…）、开机次数 bc
rep('''void setup() {
  boardInit();''', '''static uint32_t g_bootCnt = 0; static int g_rstRsn = 0;
void setup() {
  g_rstRsn = (int)esp_reset_reason();
  prefs.begin("diag", false); g_bootCnt = prefs.getUInt("bc", 0) + 1; prefs.putUInt("bc", g_bootCnt); prefs.end();
  boardInit();''')
rep('''  server.on("/api/ver", []() { sendJson("{\\"v\\":\\"" FW_VER "\\"}"); });''',
    '''  server.on("/api/ver", []() {
    char j[120];
    snprintf(j, sizeof(j), "{\\"v\\":\\"" FW_VER "\\",\\"up\\":%lu,\\"rr\\":%d,\\"bc\\":%lu,\\"vb\\":%d}",
             (unsigned long)millis(), g_rstRsn, (unsigned long)g_bootCnt, (int)vbatMv);
    sendJson(j);
  });''')
# v17 电压补偿也作用到姿态/偏航纠正量（以前只补油门 → 电越低越软、扶不住）
rep('''  if (g_vcomp && vbatMv > 3000 && thr > 0) {
    float vc = 3950.0f / (float)vbatMv;
    if (vc < 0.92f) vc = 0.92f; if (vc > 1.2f) vc = 1.2f;
    thr = (int)(thr * vc); if (thr > P.mthr) thr = P.mthr;
  }''', '''  float vcc = 1.0f;   // v17：同一个系数也乘到纠正量上
  if (g_vcomp && vbatMv > 3000 && thr > 0) {
    float vc = 3950.0f / (float)vbatMv;
    if (vc < 0.92f) vc = 0.92f; if (vc > 1.2f) vc = 1.2f;
    thr = (int)(thr * vc); if (thr > P.mthr) thr = P.mthr;
    vcc = vc;
  }''')
rep('''  mixOutput(thr, oR * k, oP * k, oY * k, o);''', '''  mixOutput(thr, oR * k * vcc, oP * k * vcc, oY * k * vcc, o);''')
rep('''           "\\"hz\\":%lu,\\"dt\\":%.1f,\\"f\\":%lu,\\"imu\\":%d}",''', '''           "\\"hz\\":%lu,\\"dt\\":%.1f,\\"f\\":%lu,\\"imu\\":%d,\\"tm\\":[%lu,%lu,%lu,%lu,%lu]}",''')
rep('''           (unsigned long)g_fault, imuOK ? 1 : 0);
}''', '''           (unsigned long)g_fault, imuOK ? 1 : 0,
           (unsigned long)g_tImu, (unsigned long)g_tImuMax, (unsigned long)g_tStep, (unsigned long)g_tStepMax, (unsigned long)g_tJit);
}''')
# v17 计时：控制一圈耗时（平均/最大 us）、读 IMU 耗时、周期最大偏差 → /api/state 的 "tm"
rep('''static void controlStep(float dt) {
  float a[3], gr[3];
  const bool ok = imuRead(a, gr);''', '''static volatile uint32_t g_tImu = 0, g_tImuMax = 0, g_tStep = 0, g_tStepMax = 0, g_tJit = 0;
static void controlStep(float dt) {
  float a[3], gr[3];
  const int64_t ti0 = esp_timer_get_time();
  const bool ok = imuRead(a, gr);
  { static uint32_t acc = 0, n = 0, mx = 0; const uint32_t d = (uint32_t)(esp_timer_get_time() - ti0);
    acc += d; if (d > mx) mx = d; if (++n >= 250) { g_tImu = acc / n; g_tImuMax = mx; acc = n = mx = 0; } }''')
rep('''    controlStep(dt);

    hzAcc++;''', '''    controlStep(dt);
    { static uint32_t acc = 0, n = 0, mx = 0, jm = 0;
      const uint32_t d = (uint32_t)(esp_timer_get_time() - now);
      const uint32_t j = (uint32_t)fabsf(dt * 1e6f - 1e6f / CTRL_HZ);
      acc += d; if (d > mx) mx = d; if (j > jm) jm = j;
      if (++n >= 250) { g_tStep = acc / n; g_tStepMax = mx; g_tJit = jm; acc = n = mx = jm = 0; } }

    hzAcc++;''')
# ================= v18（11-17）：照 Crazyflie / ESP-Drone 对齐，每项可开关 =================
# 板子 IMU_INT 接的是 QMI8658A 的 INT1；数据手册 Rev D：DRDY 只能出在 INT2（没接）→ 用高精度定时器 1 kHz。
# IMU 改 1793.6 Hz 输出 + 片内低通（13.37% ODR ≈ 240 Hz，抗混叠），每次读到的数据最多旧 0.56 ms。
# 参数（NVS + /api/param + E,k,v）：crate 控制频率 250/500/1000（默认 500，传感器始终 1 kHz）
#   lp2 1=二阶 lpf2p 陀螺 80 Hz/加计 30 Hz（Crazyflie 同款），0=旧一阶 60 Hz
#   dfc D 项低通 Hz，0=不滤（Crazyflie 默认不滤）；cap 1=顶满整体平移（Crazyflie），0=旧的按比例缩
rep('#define FW_VER "flight-v17"', '#define FW_VER "flight-v18"')
rep('static const uint32_t CTRL_HZ = 250;', 'static const uint32_t CTRL_HZ = 1000;')
rep('static float g_yff = -60.0f;', '''static int   g_crate = 500, g_lp2 = 1, g_cap = 1;   // v18 控制频率 / 二阶滤波 / 顶满整体平移
static float g_dfc = 0.0f;                          // v18 D 项低通 Hz（0 = 不滤）
static float g_yff = -60.0f;''')
rep('  prefs.putFloat("hov", g_hov); prefs.putFloat("yff", g_yff);',
    '  prefs.putFloat("hov", g_hov); prefs.putFloat("yff", g_yff);\n  prefs.putInt("crate", g_crate); prefs.putInt("lp2", g_lp2); prefs.putInt("cap", g_cap); prefs.putFloat("dfc", g_dfc);')
rep('    g_hov = prefs.getFloat("hov", g_hov); g_yff = prefs.getFloat("yff", g_yff);',
    '''    g_hov = prefs.getFloat("hov", g_hov); g_yff = prefs.getFloat("yff", g_yff);
    g_crate = prefs.getInt("crate", g_crate); g_lp2 = prefs.getInt("lp2", g_lp2); g_cap = prefs.getInt("cap", g_cap); g_dfc = prefs.getFloat("dfc", g_dfc);
    if (g_crate != 250 && g_crate != 1000) g_crate = 500;''')
_v18set = '''
  else if (k == "crate") g_crate = (v >= 900 ? 1000 : (v <= 300 ? 250 : 500));
  else if (k == "lp2") g_lp2 = v > 0.5f ? 1 : 0;
  else if (k == "cap") g_cap = v > 0.5f ? 1 : 0;
  else if (k == "dfc") g_dfc = (v < 0 ? 0 : (v > 200 ? 200 : v));'''
rep('  else if (k == "hov") g_hov = v;', '  else if (k == "hov") g_hov = v;' + _v18set)
rep('      else if (!strcmp(k, "hov")) g_hov = v;', '''      else if (!strcmp(k, "hov")) g_hov = v;
      else if (!strcmp(k, "crate")) g_crate = (v >= 900 ? 1000 : (v <= 300 ? 250 : 500));
      else if (!strcmp(k, "lp2")) g_lp2 = v > 0.5f ? 1 : 0;
      else if (!strcmp(k, "cap")) g_cap = v > 0.5f ? 1 : 0;
      else if (!strcmp(k, "dfc")) g_dfc = (v < 0 ? 0 : (v > 200 ? 200 : v));''')
rep('''"{\\"angP\\":%.2f,\\"rp\\":%.2f,\\"ri\\":%.3f,\\"rd\\":%.2f,"
           "\\"pp\\":%.2f,\\"pi\\":%.3f,\\"pd\\":%.2f,\\"yp\\":%.2f,"''', '''"{\\"angP\\":%.2f,\\"rp\\":%.3f,\\"ri\\":%.3f,\\"rd\\":%.4f,"
           "\\"pp\\":%.3f,\\"pi\\":%.3f,\\"pd\\":%.4f,\\"yp\\":%.2f,"''')
rep('\\"hov\\":%.1f,\\"yff\\":%.1f}",', '\\"hov\\":%.1f,\\"yff\\":%.1f,\\"crate\\":%d,\\"lp2\\":%d,\\"cap\\":%d,\\"dfc\\":%.1f}",')
rep('(double)g_hov, (double)g_yff);', '(double)g_hov, (double)g_yff, g_crate, g_lp2, g_cap, (double)g_dfc);')
# ① IMU：±8 g / ±2048 dps，输出 1793.6 Hz（6 轴），片内低通 13.37% ODR
rep('''      imuW(0x03, 0x24);    // ±8 g
      imuW(0x04, 0x54);    // ±512 dps''', '''      imuW(0x03, 0x22);    // v18 ±8 g，ODR 代码 2 = 1793.6 Hz（6 轴）
      imuW(0x04, 0x72);    // v18 ±2048 dps（以前 ±512 翻滚时会顶满），1793.6 Hz
      imuW(0x06, 0x77);    // v18 CTRL5 片内低通：陀螺/加计都开，模式 3 = 13.37% ODR ≈ 240 Hz（抗混叠）''')
rep('''  g[0] = (float)(int16_t)(b[7]  << 8 | b[6])  / 64.0f;
  g[1] = (float)(int16_t)(b[9]  << 8 | b[8])  / 64.0f;
  g[2] = (float)(int16_t)(b[11] << 8 | b[10]) / 64.0f;''', '''  g[0] = (float)(int16_t)(b[7]  << 8 | b[6])  / 16.0f;   // ±2048 dps → 16 LSB/dps
  g[1] = (float)(int16_t)(b[9]  << 8 | b[8])  / 16.0f;
  g[2] = (float)(int16_t)(b[11] << 8 | b[10]) / 16.0f;''')
# ② 传感器 1 kHz（sensorTick），控制按 crate 跑（controlStep）
rep('''static void controlStep(float dt) {
  float a[3], gr[3];
  const int64_t ti0 = esp_timer_get_time();
  const bool ok = imuRead(a, gr);
  { static uint32_t acc = 0, n = 0, mx = 0; const uint32_t d = (uint32_t)(esp_timer_get_time() - ti0);
    acc += d; if (d > mx) mx = d; if (++n >= 250) { g_tImu = acc / n; g_tImuMax = mx; acc = n = mx = 0; } }
  if (!ok) { i2cFail++; g_fault |= F_IMU; }
  else {
    // 低通（振动抑制，60 Hz 足够，再低会吃掉相位裕度）
    for (int i = 0; i < 3; i++) { fa[i] = lpfStep(fa[i], a[i], dt, 60.0f); fg[i] = lpfStep(fg[i], gr[i], dt, 60.0f); }
    g_ax = fa[0]; g_ay = fa[1]; g_az = fa[2];''', '''// v18 二阶巴特沃斯低通（Crazyflie lpf2p.c 同一公式）
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
static bool g_imuOk = false, g_lfInit = false;
static void sensorTick(float dt) {
  float a[3], gr[3];
  const int64_t ti0 = esp_timer_get_time();
  const bool ok = imuRead(a, gr);
  { static uint32_t acc = 0, n = 0, mx = 0; const uint32_t d = (uint32_t)(esp_timer_get_time() - ti0);
    acc += d; if (d > mx) mx = d; if (++n >= 1000) { g_tImu = acc / n; g_tImuMax = mx; acc = n = mx = 0; } }
  g_imuOk = ok;
  if (!ok) { i2cFail++; g_fault |= F_IMU; return; }
  if (!g_lfInit) {
    for (int i = 0; i < 3; i++) { lfG[i].init(1000.0f, 80.0f); lfA[i].init(1000.0f, 30.0f); lfG[i].seed(gr[i]); lfA[i].seed(a[i]); fa[i] = a[i]; fg[i] = gr[i]; }
    g_lfInit = true;
  }
  if (g_lp2) {   // Crazyflie：陀螺 80 Hz、加计 30 Hz 二阶
    for (int i = 0; i < 3; i++) { fg[i] = lfG[i].apply(gr[i]); fa[i] = lfA[i].apply(a[i]); }
  } else {       // 旧：一阶 60 Hz（同时让二阶滤波跟着走，切换时不跳）
    for (int i = 0; i < 3; i++) { fa[i] = lpfStep(fa[i], a[i], dt, 60.0f); fg[i] = lpfStep(fg[i], gr[i], dt, 60.0f); lfG[i].apply(gr[i]); lfA[i].apply(a[i]); }
  }
}
static void controlStep(float dt) {
  const bool ok = g_imuOk;
  if (!ok) { }
  else {
    g_ax = fa[0]; g_ay = fa[1]; g_az = fa[2];''')
# ③ D 项：dfc<=0 不滤（Crazyflie 默认）
rep('''    const float rc = 1.0f / (2.0f * PI_F * dfc);
    const float a = dt / (rc + dt);
    df += (dm - df) * a;''', '''    if (dfc <= 0.0f) df = dm;            // v18：0 = D 项不滤
    else {
      const float rc = 1.0f / (2.0f * PI_F * dfc);
      const float a = dt / (rc + dt);
      df += (dm - df) * a;
    }''')
rep('  const float oR = pidR.step(spR, g_gr, dt, IMAX, DFC);', '  const float oR = pidR.step(spR, g_gr, dt, IMAX, g_dfc);')
rep('  const float oP = pidP.step(spP, g_gp, dt, IMAX, DFC);', '  const float oP = pidP.step(spP, g_gp, dt, IMAX, g_dfc);')
rep('  float oY = (P.yp > 0.001f) ? pidY.step(spY, g_gy, dt, IMAX, DFC) : 0.0f;', '  float oY = (P.yp > 0.001f) ? pidY.step(spY, g_gy, dt, IMAX, g_dfc) : 0.0f;')
# ④ 顶满：cap=1 时先整体往下平移（保住电机之间的差值），平移不够才按比例缩
rep('''  const float roomUp   = (float)(P.mthr - thr);
  const float roomDown = (float)thr;

  // 饱和缩放''', '''  if (g_cap) {   // v18 Crazyflie 式：最高那路超出上限多少，四路一起减多少
    float hi = -1e9f;
    for (int i = 0; i < 4; i++) if (c[i] > hi) hi = c[i];
    if ((float)thr + hi > (float)P.mthr) { thr = (int)((float)P.mthr - hi); if (thr < 0) thr = 0; }
  }
  const float roomUp   = (float)(P.mthr - thr);
  const float roomDown = (float)thr;

  // 饱和缩放''')
rep('''      if (fabsf(amag - 1.0f) > 0.15f) R = 30.0f;''', '''      if (fabsf(amag - 1.0f) > 0.15f) R = 30.0f;
      R *= 0.004f / dt;   // v18：R 原来按 250 Hz 调的，频率变了按步长折算，融合快慢不变''')
# ⑤ 黑匣子仍按 250 Hz 记（4.8 秒）
rep('''  if (bb && !bbFrozen) {
    BBRec& r = bb[bbHead];''', '''  static int bbDec = 0;
  if (bb && !bbFrozen && ++bbDec >= g_crate / 250) {
    bbDec = 0;
    BBRec& r = bb[bbHead];''')
# ⑥ 控制任务：esp_timer 1 kHz 唤醒（不用 1 ms 系统节拍的 vTaskDelayUntil），传感器每拍、控制每 1000/crate 拍
rep('''static void ctrlTask(void*) {
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

    controlStep(dt);''', '''static TaskHandle_t g_ctrlH = nullptr;
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
    const uint32_t div = 1000u / (uint32_t)g_crate;
    if (++tick >= div) {
      tick = 0;
      float cdt = (float)(now - ctlPrev) * 1e-6f; ctlPrev = now;
      if (cdt < 0.0005f) cdt = 0.0005f; if (cdt > 0.0200f) cdt = 0.0200f;
      controlStep(cdt);
    }''')
rep('''    vTaskDelayUntil(&last, period);
  }
}''', '''  }
}''')
# v18 推力测试：H,pm 四路同占空比（原始占空比，不过电压补偿），最高 950‰；1 秒没续命自停；倾斜 >30° 自停
rep('''  if (c == 'S') {
    int pm = 0;''', '''  if (c == 'H') {
    int pm = 0;
    if (sscanf(s, "H,%d", &pm) == 1) {
      if (g_armed) { Serial.println("#ERR,ARMED"); return; }
      if (pm < 0) pm = 0; if (pm > 950) pm = 950;
      if (g_mode != M_SPIN) g_spinRef = false;   // 新的一次：重新记参考姿态
      g_spin = pm;
      g_mode = pm > 0 ? M_SPIN : M_IDLE;
      g_lastStick = millis();
      Serial.printf("#ACK,H,%d\\n", pm);
    }
    return;
  }
  if (c == 'S') {
    int pm = 0;''')
rep('''  if (g_mode == M_SPIN) {
    if ((int32_t)(nowMs - g_lastStick) > 1000) { disarm(0); }''', '''  if (g_mode == M_SPIN) {
    if ((int32_t)(nowMs - g_lastStick) > 1000) { disarm(0); }
    else {   // v18 推力测试：和开转那一刻的姿态比（倒着粘在秤上也能用），变化超过 25° = 松脱了，停
      if (!g_spinRef) { g_spinRef = true; g_spinR0 = rollF; g_spinP0 = pitchF; }
      float dr = rollF - g_spinR0; if (dr > 180) dr -= 360; if (dr < -180) dr += 360;
      if (g_spin > 250 && (fabsf(dr) > 25.0f || fabsf(pitchF - g_spinP0) > 25.0f)) disarm(F_TILT);
    }''')
rep('''static void handleSpin() {
  if (g_armed) { server.send(409, "text/plain", "armed"); return; }''', '''static void handleHSpin() {   // v18 推力测试（WiFi 版的 H,pm）：最高 950‰，1 秒没续命自停
  if (g_armed) { server.send(409, "text/plain", "armed"); return; }
  int pm = server.arg("pm").toInt();
  if (pm < 0) pm = 0; if (pm > 950) pm = 950;
  if (g_mode != M_SPIN) g_spinRef = false;
  g_spin = pm;
  g_mode = pm > 0 ? M_SPIN : M_IDLE;
  g_lastStick = millis();
  sendJson("{\\"ok\\":1}");
}
static void handleSpin() {
  if (g_armed) { server.send(409, "text/plain", "armed"); return; }''')
rep('  server.on("/api/spin", handleSpin);', '  server.on("/api/spin", handleSpin);\n  server.on("/api/hspin", handleHSpin);')
rep('static volatile int      g_spin = 0;', 'static volatile int      g_spin = 0;\nstatic bool g_spinRef = false; static float g_spinR0 = 0, g_spinP0 = 0;   // v18 推力测试的参考姿态')
rep('''  if (g_mode == M_SPIN) {
    int o[4] = {g_spin, g_spin, g_spin, g_spin};''', '''  if (g_mode == M_SPIN) {
    int o[4] = {g_spin, g_spin, g_spin, g_spin};   // 原始占空比（不过电压补偿），推力测试要的就是这个''')
# v18 空中不因低电量上锁（用户要求）：这块电池大油门会压到 3.2 V，以前 <3250 mV 立即上锁 = 空中断电。
# 现在：低于 3250 mV 只置警告位 0x10；只有在地上（油门低于 iThr）且持续 1 秒才上锁。
rep('''  if (vbatMv > 1000 && vbatMv < 3250) { disarm(F_BATT); return; }''', '''  {
    static uint32_t lowMs = 0;
    if (vbatMv > 1000 && vbatMv < 3250) { if (!lowMs) lowMs = nowMs; } else lowMs = 0;
    if (lowMs) g_fault |= 0x10;   // 低电量警告（不上锁）
    const int thrNow = P.idle + (int)((float)g_thr * (float)(P.mthr - P.idle) / 1000.0f);
    if (lowMs && nowMs - lowMs > 1000 && thrNow < g_iThr) { disarm(F_BATT); return; }   // 只在地上才因低电量上锁
  }''')
# v18 允许解锁的倾角 20° → 40°（用户要求 ≥35°：在测试架上默认就是歪的）
rep('''  if (fabsf(g_roll) > 20.0f || fabsf(g_pitch) > 20.0f) { why = "机身没放平（倾角需 <20°）"; return false; }''',
    '''  if (fabsf(g_roll) > 40.0f || fabsf(g_pitch) > 40.0f) { why = "机身太歪（倾角需 <40°）"; return false; }''')
# v18 解锁后只听一个遥控端：11-18 实测手机页（WebSocket，thr=0）和电脑（UDP，thr=730）同时发，
# 油门每秒被拉回 0 十几次 → 软启动反复重来，飞不起来。规则：解锁后第一个发来有效杆量的通道独占，上锁后释放。
# 1=UDP（电脑服务） 2=WebSocket（手机） 3=HTTP /api/stick。急停 X / stop 任何通道都有效。
rep('static void disarm(uint32_t fault) {', '''static volatile uint8_t g_ctlSrc = 0;   // v18 当前独占控制的通道（0 = 还没定）
static void disarm(uint32_t fault) {
  g_ctlSrc = 0;''')
rep('''        if (live && g_armed) {
          if (t < 0) t = 0; if (t > 1000) t = 1000;
          g_thr = t;''', '''        if (live && g_armed && (g_ctlSrc == 0 || g_ctlSrc == 1)) {
          g_ctlSrc = 1;
          if (t < 0) t = 0; if (t > 1000) t = 1000;
          g_thr = t;''')
rep('''          if (live && g_armed) {
            if (t < 0) t = 0; if (t > 1000) t = 1000;
            g_thr = t;''', '''          if (live && g_armed && (g_ctlSrc == 0 || g_ctlSrc == 2)) {
            g_ctlSrc = 2;
            if (t < 0) t = 0; if (t > 1000) t = 1000;
            g_thr = t;''')
rep('''  if (!g_armed) { sendJson("{\\"ok\\":0}"); return; }
  int t = server.arg("thr").toInt();''', '''  if (!g_armed) { sendJson("{\\"ok\\":0}"); return; }
  if (g_ctlSrc != 0 && g_ctlSrc != 3) { sendJson("{\\"ok\\":0,\\"why\\":\\"busy\\"}"); return; }
  g_ctlSrc = 3;
  int t = server.arg("thr").toInt();''')
# ================= v19（11-18）：借鉴 Betaflight 的杆量前馈；高度起伏压得更狠 =================
rep('#define FW_VER "flight-v18"', '#define FW_VER "flight-v19"')
# ① 杆量前馈：目标角先过一个 80 ms 平滑，平滑后的变化率 × kff 直接加到角速度设定上（不等角度误差慢慢攒）
rep('static float g_dfc = 0.0f;                          // v18 D 项低通 Hz（0 = 不滤）',
    'static float g_dfc = 0.0f;                          // v18 D 项低通 Hz（0 = 不滤）\nstatic float g_kff = 0.4f;                          // v19 杆量前馈（°/s 每 °/s 目标变化率）')
rep('''    spR += g_angI * aiR; spP += g_angI * aiP;
  }''', '''    spR += g_angI * aiR; spP += g_angI * aiP;
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
  }''')
# ② 高度：压上下弹的高通从 0.6 s 拉到 1.5 s（能压住 ~3 s 一个来回的慢起伏），力度 200→350、上限 60→100‰
rep('static float g_kdz = 200.0f;', 'static float g_kdz = 300.0f;')
rep('static float g_tz = 0.6f;', 'static float g_tz = 0.8f;')
rep('static float g_dzmax = 60.0f;', 'static float g_dzmax = 80.0f;')
# 存 NVS + 参数 JSON（调试台参数页能调、能回读）
rep('  prefs.putInt("crate", g_crate);', '  prefs.putFloat("kff", g_kff); prefs.putFloat("kdz", g_kdz); prefs.putFloat("tz", g_tz); prefs.putFloat("dzmax", g_dzmax);\n  prefs.putInt("crate", g_crate);')
rep('    g_crate = prefs.getInt("crate", g_crate);', '    g_kff = prefs.getFloat("kff", g_kff); g_kdz = prefs.getFloat("kdz", g_kdz); g_tz = prefs.getFloat("tz", g_tz); g_dzmax = prefs.getFloat("dzmax", g_dzmax);\n    g_crate = prefs.getInt("crate", g_crate);')
rep('  else if (k == "crate") g_crate =', '  else if (k == "kff") g_kff = (v < 0 ? 0 : (v > 2 ? 2 : v));\n  else if (k == "crate") g_crate =')
rep('      else if (!strcmp(k, "crate")) g_crate =', '      else if (!strcmp(k, "kff")) g_kff = (v < 0 ? 0 : (v > 2 ? 2 : v));\n      else if (!strcmp(k, "kdz")) g_kdz = v;\n      else if (!strcmp(k, "tz")) g_tz = v;\n      else if (!strcmp(k, "dzmax")) g_dzmax = v;\n      else if (!strcmp(k, "crate")) g_crate =')
rep('\\"cap\\":%d,\\"dfc\\":%.1f}",', '\\"cap\\":%d,\\"dfc\\":%.1f,\\"kff\\":%.2f,\\"kdz\\":%.0f,\\"tz\\":%.2f,\\"dzmax\\":%.0f}",')
rep('g_crate, g_lp2, g_cap, (double)g_dfc);', 'g_crate, g_lp2, g_cap, (double)g_dfc, (double)g_kff, (double)g_kdz, (double)g_tz, (double)g_dzmax);')
# v19 推力测试可以单测一个电机：H,pm,ch / hspin?pm=&ch=（ch 0 = 四个一起，1~4 = 单个）
rep('static bool g_spinRef = false;', 'static volatile int g_spinCh = 0;   // v19 推力测试：0 = 四个一起，1~4 = 只转这一个\nstatic bool g_spinRef = false;')
rep('''    int o[4] = {g_spin, g_spin, g_spin, g_spin};   // 原始占空比（不过电压补偿），推力测试要的就是这个''',
    '''    int o[4] = {g_spin, g_spin, g_spin, g_spin};   // 原始占空比（不过电压补偿），推力测试要的就是这个
    if (g_spinCh >= 1 && g_spinCh <= 4) for (int i = 0; i < 4; i++) if (i != g_spinCh - 1) o[i] = 0;''')
rep('''    if (sscanf(s, "H,%d", &pm) == 1) {''', '''    int ch = 0;
    if (sscanf(s, "H,%d,%d", &pm, &ch) >= 1) {
      g_spinCh = (ch >= 1 && ch <= 4) ? ch : 0;''')
rep('''  int pm = server.arg("pm").toInt();
  if (pm < 0) pm = 0; if (pm > 950) pm = 950;''', '''  int pm = server.arg("pm").toInt();
  { const int ch = server.arg("ch").toInt(); g_spinCh = (ch >= 1 && ch <= 4) ? ch : 0; }
  if (pm < 0) pm = 0; if (pm > 950) pm = 950;''')
# v19 每个电机的推力补偿系数 mg1~mg4（由推力页单测算出来）：混控之后乘上，让四个电机出力一样
rep('static float g_kff = 0.4f;', 'static float g_kff = 0.4f;\nstatic float g_mg[4] = {1, 1, 1, 1};                // v19 每个电机推力补偿系数（弱的 >1）')
rep('''  mixOutput(thr, oR * k * vcc, oP * k * vcc, oY * k * vcc, o);
  writeMotors(o);''', '''  mixOutput(thr, oR * k * vcc, oP * k * vcc, oY * k * vcc, o);
  for (int i = 0; i < 4; i++) { int v = (int)lrintf((float)o[i] * g_mg[i]); if (v > P.mthr) v = P.mthr; if (v < 0) v = 0; o[i] = v; }
  writeMotors(o);''')
rep('  prefs.putFloat("kff", g_kff);', '  prefs.putFloat("mg1", g_mg[0]); prefs.putFloat("mg2", g_mg[1]); prefs.putFloat("mg3", g_mg[2]); prefs.putFloat("mg4", g_mg[3]);\n  prefs.putFloat("kff", g_kff);')
rep('    g_kff = prefs.getFloat("kff", g_kff);', '    g_mg[0] = prefs.getFloat("mg1", 1); g_mg[1] = prefs.getFloat("mg2", 1); g_mg[2] = prefs.getFloat("mg3", 1); g_mg[3] = prefs.getFloat("mg4", 1);\n    g_kff = prefs.getFloat("kff", g_kff);')
_mg = ''.join('\n  else if (k == "mg%d") g_mg[%d] = (v < 0.7f ? 0.7f : (v > 1.4f ? 1.4f : v));' % (i + 1, i) for i in range(4))
rep('  else if (k == "kff") g_kff =', _mg.lstrip('\n') + '\n  else if (k == "kff") g_kff =')
_mg2 = ''.join('\n      else if (!strcmp(k, "mg%d")) g_mg[%d] = (v < 0.7f ? 0.7f : (v > 1.4f ? 1.4f : v));' % (i + 1, i) for i in range(4))
rep('      else if (!strcmp(k, "kff")) g_kff =', _mg2.lstrip('\n') + '\n      else if (!strcmp(k, "kff")) g_kff =')
rep('\\"dzmax\\":%.0f}",', '\\"dzmax\\":%.0f,\\"mg1\\":%.3f,\\"mg2\\":%.3f,\\"mg3\\":%.3f,\\"mg4\\":%.3f}",')
rep('(double)g_tz, (double)g_dzmax);', '(double)g_tz, (double)g_dzmax, (double)g_mg[0], (double)g_mg[1], (double)g_mg[2], (double)g_mg[3]);')
open(dst, 'w', encoding='utf-8').write(t)
print('ok')
