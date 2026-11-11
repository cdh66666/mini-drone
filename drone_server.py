#!/usr/bin/env python3
"""小飞机本地服务：给 fly.html 用，侧栏浏览器 / 任何浏览器都能连飞机。

  USB  ：自己开串口（VID 0x303A），原样转发命令和数据。
  WiFi ：电脑连上飞机热点 DroneBench 后，把同样的命令翻译成 http://192.168.4.1/api/*，
         再把 /api/state 翻译成串口一样的 D 帧 / #DBG 行。页面不用管走的哪条路。

接口（同源）：
  GET  /api/link?mode=auto|usb|wifi   连接（auto：先 WiFi 再 USB）-> {"ok":1,"mode":"usb"}
  GET  /api/unlink
  GET  /api/poll?since=N              -> {"n":M,"lines":[...],"mode":...}
  POST /api/send                       正文：一行或多行命令
用法：python -I drone_server.py [端口=8771] [飞机地址=http://192.168.4.1]
"""
import json, os, sys, threading, time, collections
import urllib.request, urllib.parse
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import cam_rec

HERE = os.path.dirname(os.path.abspath(__file__))
PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 8771
FIXED = sys.argv[2].rstrip("/") if len(sys.argv) > 2 else None   # 测试用：固定飞机地址
DRONE = FIXED or "http://192.168.4.1"
IPFILE = os.path.join(HERE, "drone_ip.txt")
FWDIR = os.path.join(HERE, "firmware", "build")


def candidates():
    if FIXED:
        return [FIXED]
    c = []
    try:
        ip = open(IPFILE).read().strip()
        if ip: c.append("http://" + ip)
    except Exception:
        pass
    c += ["http://minidrone.local", "http://192.168.4.1"]
    return c


def find_drone():
    """并行探测，最多等 2.5 秒，按优先顺序取能回 /api/state 的"""
    global DRONE
    res = []
    order = candidates()
    # 先试上次记住的地址（最快），通了就直接用，不等别的
    try:
        with urllib.request.urlopen(order[0] + "/api/state", timeout=0.8) as r:
            json.loads(r.read()); DRONE = order[0]; return True
    except Exception:
        pass
    def probe(u):
        try:
            with urllib.request.urlopen(u + "/api/state", timeout=1.5) as r:
                json.loads(r.read()); res.append(u)
        except Exception:
            pass
    ts = [threading.Thread(target=probe, args=(u,), daemon=True) for u in order]
    [x.start() for x in ts]
    end = time.time() + 2.5
    for x in ts:
        x.join(max(0.0, end - time.time()))
    got = [u for u in order if u in res]
    if got:
        DRONE = got[0]
        if not FIXED and "192.168.4.1" not in DRONE:
            try: open(IPFILE, "w").write(DRONE.replace("http://", "").split(":")[0])
            except Exception: pass
    return bool(got)

lock = threading.Lock()
LINES = collections.deque(maxlen=3000)   # (序号, 文本)
SEQ = [0]
LINK = {"mode": None, "obj": None}


def push(line):
    with lock:
        SEQ[0] += 1
        LINES.append((SEQ[0], line))


# ------------------------------------------------------------------ USB
class UsbLink:
    def __init__(self):
        import serial, serial.tools.list_ports as lp
        port = None
        for p in lp.comports():
            if p.vid == 0x303A:
                port = p.device
                break
        if not port:
            raise RuntimeError("没找到 USB 飞机")
        s = serial.Serial()
        s.port, s.baudrate, s.timeout = port, 115200, 0.1
        s.dtr = False; s.rts = False          # 不触发复位
        s.open()
        self.s, self.port, self.alive = s, port, True
        threading.Thread(target=self._rx, daemon=True).start()

    def _rx(self):
        buf = b""
        while self.alive:
            try:
                b = self.s.read(4096)
            except Exception:
                push("#LINK,lost"); self.alive = False; break
            if not b:
                continue
            buf += b
            while b"\n" in buf:
                l, buf = buf.split(b"\n", 1)
                l = l.decode("utf-8", "replace").strip()
                if l:
                    if l.startswith("#NET,ok,"):
                        try: open(IPFILE, "w").write(l.split(",")[3])
                        except Exception: pass
                    push(l)

    def send(self, lines):
        for l in lines:
            self.s.write((l + "\n").encode())

    def close(self):
        self.alive = False
        try: self.s.close()
        except Exception: pass


# ------------------------------------------------------------------ WiFi
def http_get(path, timeout=0.6):
    with urllib.request.urlopen(DRONE + path, timeout=timeout) as r:
        return r.read().decode("utf-8", "replace")


class WifiLink:
    """控制和遥测走 UDP 4210（固件 v3+，独立任务，网页服务卡了也不影响）；其它命令走 HTTP。
    固件不支持 UDP 时退回 HTTP 轮询。每秒的丢包 / 延迟写进 drone_link.log。"""
    def __init__(self):
        if not find_drone():
            raise RuntimeError("WiFi 上找不到飞机")
        import socket
        self.alive = True
        self.q = collections.deque()
        self.ev = threading.Event()
        self.thr, self.r, self.p, self.y = 0, 0.0, 0.0, 0.0
        self.last_cmd = 0.0
        self.fail_since = 0
        self.host = DRONE.replace("http://", "").split(":")[0]
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.2)
        self.seq = 0
        self.sent = {}
        self.st = {"tx": 0, "rx": 0, "rtt": [], "rssi": 0, "last_rx": time.time(), "gap": 0.0}
        self.udp = self._probe()
        if self.udp:
            threading.Thread(target=self._udp_tx, daemon=True).start()
            threading.Thread(target=self._udp_rx, daemon=True).start()
            threading.Thread(target=self._stats, daemon=True).start()
        else:
            threading.Thread(target=self._state, daemon=True).start()
        threading.Thread(target=self._cmd, daemon=True).start()
        push("#VER,wifi," + self.host + (",udp" if self.udp else ",http"))

    def _probe(self):
        for _ in range(4):
            try:
                self.sock.sendto(b"U,0,0,0,0,0,0", (self.host, 4210))
                d, _a = self.sock.recvfrom(512)
                if d.startswith(b"T,"):
                    return True
            except Exception:
                pass
        return False

    def _udp_tx(self):
        while self.alive:
            t0 = time.time()
            self.seq += 1
            live = 1 if t0 - self.last_cmd < 0.4 else 0     # 页面 0.4 秒没发油门就不再喂心跳 → 飞机自己停
            try:
                self.sock.sendto(("U,%d,%d,%d,%.2f,%.2f,%.2f" % (self.seq, live, self.thr, self.r, self.p, self.y)).encode(),
                                 (self.host, 4210))
                self.sent[self.seq] = t0
                self.st["tx"] += 1
                if len(self.sent) > 200:
                    for k in sorted(self.sent)[:100]: self.sent.pop(k, None)
            except Exception:
                pass
            time.sleep(max(0.0, 0.04 - (time.time() - t0)))

    def _udp_rx(self):
        n = 0
        while self.alive:
            try:
                d, _a = self.sock.recvfrom(512)
            except Exception:
                continue
            a = d.decode("utf-8", "replace").split(",")
            if a[0] != "T" or len(a) < 19:
                continue
            now = time.time()
            s0 = self.sent.pop(int(a[1]), None)
            if s0: self.st["rtt"].append((now - s0) * 1000)
            self.st["gap"] = max(self.st["gap"], now - self.st["last_rx"])
            self.st["last_rx"] = now
            self.st["rx"] += 1
            self.st["rssi"] = int(a[17])
            arm = a[3] == "1"
            # 整次飞行记录（解锁→上锁，25 Hz）：电压、油门、姿态、自转、电机，用来算续航
            if arm:
                if not getattr(self, "was_arm", False):
                    self.full = []; self.full_t0 = now
                    cam_rec.get().start(now)
                self.full.append("%.3f,%s,%d,%.2f,%.2f,%s,%s,%s,%s,%s,%s,%s,%s,%s" % (
                    now - self.full_t0, a[2], self.thr, self.r, self.p, a[5], a[6], a[19] if len(a) > 19 else "0", a[9],
                    a[10], a[11], a[12], a[13], a[17]))
            if getattr(self, "was_arm", False) and not arm:
                threading.Thread(target=save_bb, args=(self.host, a[4]), daemon=True).start()
                rows = getattr(self, "full", [])
                fn = time.strftime("%Y%m%d_%H%M%S") + ".csv"
                cam_rec.get().stop(fn[:-4] if len(rows) > 10 else None)
                if len(rows) > 10:
                    try:
                        d = os.path.join(HERE, "flights", "full"); os.makedirs(d, exist_ok=True)
                        with open(os.path.join(d, fn), "w", encoding="utf-8") as f:
                            NL = chr(10); f.write("t,vb_mv,thr_cmd,stick_r,stick_p,roll,pitch,yaw,gy,o1,o2,o3,o4,rssi" + NL + NL.join(rows) + NL)
                    except Exception:
                        pass
                self.full = []
            self.was_arm = arm
            self.fail_since = 0
            n += 1
            if n % 2:   # 12 帧/秒给页面就够
                continue
            push("D,%d,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s" % (
                int(now * 1000) % 10**9, a[2], a[18], a[14], a[15], a[16], a[7], a[8], a[9],
                a[10], a[11], a[12], a[13], a[3], a[4]))
            push("#DBG,roll=%s,pitch=%s,yaw=%s,gy=%s" % (a[5], a[6], a[19] if len(a) > 19 else "0", a[9]))

    def _stats(self):
        logf = os.path.join(HERE, "drone_link.log")
        lost_said = 0
        while self.alive:
            time.sleep(1.0)
            s = self.st
            tx, rx, rtt = s["tx"], s["rx"], s["rtt"]
            s["tx"], s["rx"], s["rtt"] = 0, 0, []
            gap = max(s["gap"], time.time() - s["last_rx"]); s["gap"] = 0.0
            loss = 0 if not tx else max(0, round(100 * (tx - rx) / tx))
            avg = sum(rtt) / len(rtt) if rtt else -1
            mx = max(rtt) if rtt else -1
            push("#LQ,%d,%d,%d,%d" % (loss, avg, s["rssi"], gap * 1000))
            try:
                with open(logf, "a", encoding="utf-8") as f:
                    f.write("%s tx=%d rx=%d loss=%d%% rtt=%.0f/%.0fms rssi=%d gap=%.0fms\n" % (
                        time.strftime("%H:%M:%S"), tx, rx, loss, avg, mx, s["rssi"], gap * 1000))
            except Exception:
                pass
            if time.time() - s["last_rx"] > 4 and time.time() - lost_said > 4:
                push("#LINK,lost"); lost_said = time.time()

    def _state(self):
        while self.alive:
            t0 = time.time()
            try:
                s = json.loads(http_get("/api/state"))
                self.fail_since = 0
                d = s.get("d", [0, 0, 0, 0])
                push("D,%d,%d,1,%.3f,%.3f,%.3f,%.1f,%.1f,%.1f,%d,%d,%d,%d,%d,%d" % (
                    int(t0 * 1000) % 10**9, s.get("vb", 0), s.get("ax", 0), s.get("ay", 0), s.get("az", 0),
                    s.get("gr", 0), s.get("gp", 0), s.get("gy", 0), d[0], d[1], d[2], d[3],
                    s.get("arm", 0), s.get("f", 0)))
                push("#DBG,roll=%.2f,pitch=%.2f" % (s.get("roll", 0), s.get("pitch", 0)))
            except Exception:
                if not self.fail_since:
                    self.fail_since = time.time()
                elif time.time() - self.fail_since > 2:
                    push("#LINK,lost"); self.fail_since = time.time()
            time.sleep(max(0.0, 0.1 - (time.time() - t0)))

    def send(self, lines):
        for l in lines:
            a = l.split(",")
            if self.udp and a[0] == "G":
                self.thr = int(float(a[1])); self.last_cmd = time.time(); continue
            if self.udp and a[0] == "Q":
                self.r, self.p, self.y = float(a[1]), float(a[2]), float(a[3]); self.last_cmd = time.time(); continue
            if self.udp and a[0] == "K":
                self.last_cmd = time.time(); continue
            if self.udp and a[0] == "P":
                continue
            if l.startswith("X"):            # 急停：UDP 连发 3 次 + HTTP
                self.thr = 0; self.last_cmd = 0
                if self.udp:
                    for _ in range(3):
                        try: self.sock.sendto(b"X", (self.host, 4210))
                        except Exception: pass
                self.q.clear()
                self.q.appendleft(l)
            else:
                self.q.append(l)
        self.ev.set()

    def _call(self, path):
        try:
            return http_get(path)
        except Exception:
            return None

    def _cmd(self):
        while self.alive:
            self.ev.wait(0.5); self.ev.clear()
            while self.q:
                l = self.q.popleft()
                a = l.split(",")
                c = a[0]
                if c == "X":
                    self._call("/api/stop"); push("#ACK,STOP")
                elif c == "G":
                    self.thr = int(float(a[1]))
                elif c in ("Q", "K"):
                    if c == "Q":
                        self.r, self.p, self.y = float(a[1]), float(a[2]), float(a[3])
                    self._call("/api/stick?thr=%d&r=%.2f&p=%.2f&y=%.2f" % (self.thr, self.r, self.p, self.y))
                elif c == "A":
                    res = self._call("/api/arm?on=" + a[1])
                    try: j = json.loads(res or "{}")
                    except Exception: j = {}
                    if a[1] == "1":
                        push("#ACK,ARM" if j.get("ok") else "#ERR,ARM_FAIL," + j.get("why", "没响应"))
                    else:
                        push("#ACK,DISARM")
                elif c == "J":
                    ch = int(a[1])
                    self._call("/api/jog?i=%d&pm=%s&ms=%s" % (ch - 1 if ch > 0 else -1, a[2], a[3]))
                elif c == "S":
                    self._call("/api/spin?pm=" + a[1])
                elif c == "R":
                    self._call("/api/zero")
                elif c == "L":
                    if self._call("/api/level") is not None:
                        time.sleep(0.15); push("#ACK,LEVEL")
                elif c == "E":
                    if self._call("/api/param?k=%s&v=%s" % (urllib.parse.quote(a[1]), a[2])) is not None:
                        push("#ACK,E,%s=%s" % (a[1], a[2]))
                elif c == "W":
                    if self._call("/api/save") is not None: push("#ACK,SAVE")
                elif c == "U":
                    if self._call("/api/load") is not None: push("#ACK,LOAD")
                elif c == "C":
                    res = self._call("/api/params")
                    if res: push("#PARAMS," + res.strip())
                elif c == "V":
                    push("#VER,wifi," + DRONE.replace("http://", ""))
                # P：姿态已由 _state 推送，忽略

    def close(self):
        self.alive = False
        try: self.sock.close()
        except Exception: pass


def save_bb(host, fault="0"):
    """上锁后把飞机里的黑匣子（最后 4.8 秒，250 Hz）存到 flights/"""
    time.sleep(0.3)
    try:
        with urllib.request.urlopen("http://%s/api/bb" % host, timeout=8) as r:
            txt = r.read().decode("utf-8", "replace")
    except Exception as e:
        push("#BB,fail," + str(e)[:60]); return
    if txt.count(chr(10)) < 5:
        return
    d = os.path.join(HERE, "flights"); os.makedirs(d, exist_ok=True)
    fn = time.strftime("%Y%m%d_%H%M%S") + ".csv"
    open(os.path.join(d, fn), "w", encoding="utf-8").write(txt)
    push("#BB,saved," + fn + "," + str(fault))
    try: learn_bias(host, txt)
    except Exception as e: push("#BB,learn_fail," + str(e)[:60])


def learn_bias(host, txt):
    """重心补偿自学（固件 v10+）：看这次平稳悬停时角速度积分平均多少，把 bR/bP 往那边挪 60%。
    下次起飞前就先加上，重的那边一离地就有足够推力，不会先歪。"""
    def get(p):
        with urllib.request.urlopen("http://%s%s" % (host, p), timeout=3) as r:
            return r.read().decode("utf-8", "replace")
    par = json.loads(get("/api/params"))
    if "bP" not in par:
        return                                   # 老固件
    rows = [l.split(",") for l in txt.splitlines() if l and l[0].isdigit()]
    hdr = [l for l in txt.splitlines() if l.startswith("ms,")][0].split(",")
    ix = {k: i for i, k in enumerate(hdr)}
    it_r, it_p, t_air = [], [], None
    for r in rows:
        thr, ms = int(r[ix["thr"]]), int(r[ix["ms"]])
        if thr < 310:
            t_air = None; continue
        t_air = t_air or ms
        if ms - t_air < 600:                     # 刚离地那 0.6 秒不算
            continue
        if abs(float(r[ix["roll"]])) > 15 or abs(float(r[ix["pitch"]])) > 15:
            continue                             # 翻了 / 大动作不算
        if abs(float(r[ix["stR"]])) > 0.01 or abs(float(r[ix["stP"]])) > 0.01:
            continue                             # 按了方向键不算
        it_r.append(float(r[ix["itR"]])); it_p.append(float(r[ix["itP"]]))
    if len(it_p) < 200:                          # 少于 0.8 秒平稳悬停，不学
        return
    out = {}
    for k, v in (("bR", it_r), ("bP", it_p)):
        old = float(par[k]); m = sorted(v)[len(v) // 2]
        new = max(-60.0, min(60.0, old + 0.6 * (m - old)))
        if abs(new - old) >= 1.0:
            get("/api/param?k=%s&v=%.1f" % (k, new)); out[k] = (old, new)
    if out:
        get("/api/save")
        msg = " ".join("%s %.1f→%.1f" % (k, a, b) for k, (a, b) in out.items())
        push("#NOTE,重心补偿已自动更新：" + msg)
        with open(os.path.join(HERE, "notes.log"), "a", encoding="utf-8") as f:
            f.write(time.strftime("%m-%d %H:%M:%S ") + "bias " + msg + " n=%d\n" % len(it_p))


PENDING = os.path.join(HERE, "pending.json")


def run_pending():
    """连上飞机后自动做的事（Claude 准备好、用户不用操作）：先更新固件，再调配平"""
    try:
        p = json.load(open(PENDING, encoding="utf-8"))
    except Exception:
        return
    time.sleep(1.5)
    if LINK["mode"] != "wifi":
        return
    if p.get("flash"):
        p["flash"] = False
        json.dump(p, open(PENDING, "w", encoding="utf-8"))
        push("#NOTE,正在更新固件，约 20 秒，别断电…")
        r = flash()
        push("#NOTE," + ("固件更新好了 ✓ 等 5 秒再点连接" if r.get("ok") else "固件没更新成功"))
        push("#LINK,lost")
        return
    if p.get("trimP_add"):
        try:
            cur = json.loads(http_get("/api/params", 2))
            new = round(cur["trimP"] + p["trimP_add"], 2)
            http_get("/api/param?k=trimP&v=%s" % new, 2); http_get("/api/save", 2)
            push("#NOTE,配平已往前调 %s°（%s → %s）✓" % (p["trimP_add"], cur["trimP"], new))
            with open(os.path.join(HERE, "notes.log"), "a", encoding="utf-8") as f:
                f.write(time.strftime("%m-%d %H:%M:%S ") + "trimP %s -> %s\n" % (cur["trimP"], new))
            p.pop("trimP_add")
            json.dump(p, open(PENDING, "w", encoding="utf-8"))
        except Exception as e:
            push("#NOTE,配平没调成：" + str(e)[:40])


def link(mode):
    unlink()
    errs = []
    order = {"auto": ["wifi", "usb"], "usb": ["usb"], "wifi": ["wifi"]}.get(mode, ["wifi", "usb"])
    for m in order:
        try:
            LINK["obj"] = UsbLink() if m == "usb" else WifiLink()
            LINK["mode"] = m
            if m == "wifi":
                threading.Thread(target=run_pending, daemon=True).start()
            return {"ok": 1, "mode": m}
        except Exception as e:
            errs.append("%s: %s" % (m, e))
    return {"ok": 0, "why": "; ".join(errs)}


def unlink():
    if LINK["obj"]:
        try:
            LINK["obj"].send(["X"]); time.sleep(0.05)
        except Exception:
            pass
        LINK["obj"].close()
    LINK["obj"], LINK["mode"] = None, None


def latest_fw():
    fs = [os.path.join(FWDIR, f) for f in os.listdir(FWDIR) if f.endswith(".ino.bin")] if os.path.isdir(FWDIR) else []
    return max(fs, key=os.path.getmtime) if fs else None


def flash():
    fw = latest_fw()
    if not fw:
        return {"ok": 0, "why": "没有固件文件"}
    mode = LINK["mode"]
    if mode == "wifi":
        LINK["obj"].send(["X"]); time.sleep(0.3)
        data = open(fw, "rb").read()
        bd = "----minidrone%d" % int(time.time())
        CRLF = bytes([13, 10]).decode()
        head = "--" + bd + CRLF + 'Content-Disposition: form-data; name="firmware"; filename="' + os.path.basename(fw) + '"' + CRLF + "Content-Type: application/octet-stream" + CRLF + CRLF
        body = head.encode() + data + (CRLF + "--" + bd + "--" + CRLF).encode()
        req = urllib.request.Request(DRONE + "/api/update", data=body, method="POST",
                                     headers={"Content-Type": "multipart/form-data; boundary=" + bd})
        try:
            with urllib.request.urlopen(req, timeout=90) as r:
                txt = r.read().decode("utf-8", "replace")
        except Exception as e:
            return {"ok": 0, "why": str(e)}
        unlink()
        return {"ok": 1, "how": "wifi", "fw": os.path.basename(fw), "resp": txt[:200]}
    if mode == "usb":
        port = LINK["obj"].port
        unlink(); time.sleep(0.3)
        import subprocess
        r = subprocess.run([sys.executable, "-I", "-m", "esptool", "--chip", "esp32s3", "-p", port, "--no-stub",
                            "write-flash", "0x10000", fw], capture_output=True, text=True, timeout=600)
        ok = r.returncode == 0
        return {"ok": 1 if ok else 0, "how": "usb", "fw": os.path.basename(fw), "why": "" if ok else r.stdout[-300:] + r.stderr[-300:]}
    return {"ok": 0, "why": "没连接"}


class H(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _json(self, o, code=200):
        b = json.dumps(o, ensure_ascii=False).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(b)))
        self.end_headers(); self.wfile.write(b)

    def do_GET(self):
        u = urllib.parse.urlparse(self.path)
        q = urllib.parse.parse_qs(u.query)
        if u.path == "/api/link":
            return self._json(link(q.get("mode", ["auto"])[0]))
        if u.path == "/api/probe":   # 电脑能不能直接连到飞机这个 IP
            ip = q.get("ip", [""])[0]
            try:
                with urllib.request.urlopen("http://%s/api/state" % ip, timeout=2) as r:
                    json.loads(r.read())
                return self._json({"ok": 1})
            except Exception as e:
                return self._json({"ok": 0, "why": str(e)[:100]})
        if u.path == "/api/yawtest":   # 转给飞机：点动 M1、M2，看机身被扭向哪边
            try:
                with urllib.request.urlopen(DRONE + "/api/yawtest?" + u.query, timeout=15) as r:
                    j = json.loads(r.read())
                with open(os.path.join(HERE, "notes.log"), "a", encoding="utf-8") as f:
                    f.write(time.strftime("%m-%d %H:%M:%S ") + "yawtest " + json.dumps(j) + "\n")
                return self._json(j)
            except Exception as e:
                return self._json({"err": str(e)[:80]})
        if u.path == "/api/note":    # 页面留的测量记录（方向检查原始数据等）
            try:
                with open(os.path.join(HERE, "notes.log"), "a", encoding="utf-8") as f:
                    f.write(time.strftime("%m-%d %H:%M:%S ") + q.get("t", [""])[0] + "\n")
            except Exception:
                pass
            return self._json({"ok": 1})
        if u.path == "/api/tune":    # Claude 调好的基准参数
            try: return self._json(json.load(open(os.path.join(HERE, "tune.json"), encoding="utf-8")))
            except Exception: return self._json({})
        if u.path == "/api/flash":
            return self._json(flash())
        if u.path == "/api/fwinfo":
            fw = latest_fw()
            return self._json({"fw": os.path.basename(fw) if fw else None,
                               "time": time.strftime("%m-%d %H:%M", time.localtime(os.path.getmtime(fw))) if fw else None})
        if u.path == "/api/cam":       # 摄像头状态
            return self._json(cam_rec.get().info())
        if u.path == "/api/cam.mjpg":  # 摄像头实时画面（MJPEG，约 10 帧/秒），页面 <img> 直接用
            cam = cam_rec.get()
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frm")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            last = None
            try:
                while True:
                    b = cam.snapshot()
                    if b and b is not last:
                        last = b
                        self.wfile.write(b"--frm\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n" % len(b) + b + b"\r\n")
                        self.wfile.flush()
                    time.sleep(0.05)
            except Exception:
                return                     # 页面关了 / 断开
        if u.path == "/api/cam.jpg":   # 摄像头当前画面
            b = cam_rec.get().snapshot()
            if not b:
                self.send_response(404); self.end_headers(); return
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(b)))
            self.end_headers(); self.wfile.write(b); return
        if u.path == "/api/unlink":
            unlink(); return self._json({"ok": 1})
        if u.path == "/api/poll":
            since = int(q.get("since", ["0"])[0])
            with lock:
                out = [l for n, l in LINES if n > since]
                n = SEQ[0]
            return self._json({"n": n, "lines": out[-400:], "mode": LINK["mode"]})
        # 静态：只给页面
        if u.path in ("/", "/fly.html"):
            b = open(os.path.join(HERE, "fly.html"), "rb").read()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(b)))
            self.end_headers(); self.wfile.write(b); return
        if u.path in ("/replay.html", "/flights/replay.json"):   # 历史回放
            if u.path.endswith(".json"):
                try:
                    import subprocess
                    subprocess.run([sys.executable, "-I", os.path.join(HERE, "tools", "replay_build.py")], timeout=600,
                                   capture_output=True)
                except Exception:
                    pass
            fp = os.path.join(HERE, u.path.lstrip("/").replace("/", os.sep))
            b = open(fp, "rb").read()
            self.send_response(200)
            self.send_header("Content-Type", "application/json" if fp.endswith(".json") else "text/html; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(b)))
            self.end_headers(); self.wfile.write(b); return
        self.send_response(404); self.end_headers()

    def do_POST(self):
        if self.path.startswith("/api/send"):
            n = int(self.headers.get("Content-Length", 0))
            body = self.rfile.read(n).decode("utf-8", "replace")
            lines = [l.strip() for l in body.splitlines() if l.strip()]
            if not LINK["obj"]:
                return self._json({"ok": 0, "why": "没连接"})
            try:
                LINK["obj"].send(lines)
            except Exception as e:
                push("#LINK,lost"); return self._json({"ok": 0, "why": str(e)})
            return self._json({"ok": 1})
        self.send_response(404); self.end_headers()


if __name__ == "__main__":
    print("drone server http://127.0.0.1:%d  drone=%s" % (PORT, DRONE), flush=True)
    cam_rec.get()   # 摄像头先开着，解锁时马上能录
    ThreadingHTTPServer(("127.0.0.1", PORT), H).serve_forever()
