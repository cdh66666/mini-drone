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
    def __init__(self):
        if not find_drone():
            raise RuntimeError("WiFi 上找不到飞机")
        self.alive = True
        self.q = collections.deque()
        self.ev = threading.Event()
        self.thr, self.r, self.p, self.y = 0, 0.0, 0.0, 0.0
        self.fail_since = 0
        threading.Thread(target=self._state, daemon=True).start()
        threading.Thread(target=self._cmd, daemon=True).start()
        push("#VER,wifi," + DRONE.replace("http://", ""))

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
            if l.startswith("X"):            # 急停插队
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


def link(mode):
    unlink()
    errs = []
    order = {"auto": ["wifi", "usb"], "usb": ["usb"], "wifi": ["wifi"]}.get(mode, ["wifi", "usb"])
    for m in order:
        try:
            LINK["obj"] = UsbLink() if m == "usb" else WifiLink()
            LINK["mode"] = m
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
        if u.path == "/api/flash":
            return self._json(flash())
        if u.path == "/api/fwinfo":
            fw = latest_fw()
            return self._json({"fw": os.path.basename(fw) if fw else None,
                               "time": time.strftime("%m-%d %H:%M", time.localtime(os.path.getmtime(fw))) if fw else None})
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
    ThreadingHTTPServer(("127.0.0.1", PORT), H).serve_forever()
