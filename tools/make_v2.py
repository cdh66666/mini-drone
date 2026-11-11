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
open(dst, 'w', encoding='utf-8').write(t)
print('ok')
