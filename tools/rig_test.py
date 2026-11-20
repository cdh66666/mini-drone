"""定高测试架：通过本地 drone_server（WiFi 链路）飞，摄像头模板匹配测飞机高度。服务器自己会记整次飞行 CSV 和录像。

用法：python -I tools/rig_test.py <标签> <模式> [k=v ...]
  模式 lift  ：从 LO 慢慢推到 HI（油门 %），飞机离开下限位就记下离地油门，回到 0
  模式 steps ：推到 BASE% 稳住，然后 +STEP / -STEP / 回 BASE，各 HOLD 秒
  k=v：LO=35 HI=65 RAMP=12 BASE=50 STEP=5 HOLD=2.5；P_xxx=v 先改飞机参数（只改内存）
画面：飞机模板取自解锁前的那一帧（ROI 里），之后每帧在竖直条带里找它 → 高度 px（向上为正）
安全：高度异常/倾角 >50° 立刻 X；结束一定 X。
"""
import json, os, sys, time, threading, urllib.request
import numpy as np, cv2

TAG, MODE = sys.argv[1], sys.argv[2]
KV = dict(a.split("=", 1) for a in sys.argv[3:])
f = lambda k, d: float(KV.get(k, d))
HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRV = "http://127.0.0.1:8771"
OUT = os.path.join(HERE, "flights", "rig"); os.makedirs(OUT, exist_ok=True)
O = urllib.request.build_opener(urllib.request.ProxyHandler({}))
ROI = [int(v) for v in KV.get("ROI", "560,40,700,330").split(",")]     # x0,y0,x1,y1（1280x720 画面）
BOX = [int(v) for v in KV.get("BOX", "592,148,668,206").split(",")]    # 飞机模板框

def get(p, t=2.0):
    with O.open(SRV + p, timeout=t) as r: return r.read()
def send(*l):
    O.open(urllib.request.Request(SRV + "/api/send?cid=rigtest", data="\n".join(l).encode(), method="POST"), timeout=2).read()
def frame():
    b = get("/api/cam.jpg", 2.0); return cv2.imdecode(np.frombuffer(b, np.uint8), cv2.IMREAD_GRAYSCALE)

# 改参数（只改内存）
for k, v in KV.items():
    if k.startswith("P_"): send("E,%s,%s" % (k[2:], v))
f0 = frame(); tpl = f0[BOX[1]:BOX[3], BOX[0]:BOX[2]].copy(); y_ref = BOX[1]
cv2.imwrite(os.path.join(OUT, "tpl_%s.png" % TAG), tpl)
since = [json.loads(get("/api/poll?since=0"))["n"]]
log = []; stop = [False]; cmd = {"thr": 0}; stk = {"r": 0.0, "p": 0.0, "y": 0.0}; lock = threading.Lock()
st = {"y": 0.0, "score": 0, "roll": 0.0, "pitch": 0.0, "arm": 0, "vb": 0}

def track():
    while not stop[0]:
        try: g = frame()
        except Exception: time.sleep(0.05); continue
        r = g[ROI[1]:ROI[3], ROI[0]:ROI[2]]
        m = cv2.matchTemplate(r, tpl, cv2.TM_CCOEFF_NORMED); _, sc, _, loc = cv2.minMaxLoc(m)
        y = ROI[1] + loc[1]
        st["y"] = float(y_ref - y); st["score"] = float(sc)
        time.sleep(0.03)
def telem():
    while not stop[0]:
        try:
            p = json.loads(get("/api/poll?since=%d" % since[0])); since[0] = p["n"]
            for l in p["lines"]:
                if l.startswith("D,"):
                    a = l.split(","); st["vb"] = int(a[2]); st["arm"] = int(a[14]) if len(a) > 14 else 0
                if l.startswith("#S,"):
                    j = json.loads(l[3:]); st["roll"] = j["roll"]; st["pitch"] = j["pitch"]; st["arm"] = j["arm"]; st["vb"] = j["vb"]
        except Exception: pass
        time.sleep(0.1)
def pump():   # 每 0.1 s 给一次油门（服务器 0.4 s 没收到就断心跳）
    while not stop[0]:
        with lock: t = cmd["thr"]
        try: send("G,%d" % t, "Q,%.2f,%.2f,%.2f" % (stk["r"], stk["p"], stk["y"]))
        except Exception: pass
        time.sleep(0.1)
IP = open(os.path.join(HERE, "drone_ip.txt")).read().strip(); PL = []
def fwpoll():   # 固件内部：辨识出的悬停推力、动态/静态推力系数比（≈1 悬空，≈0 贴着）、是否接管
    while not stop[0]:
        try:
            with O.open("http://%s/api/params" % IP, timeout=0.6) as r: j = json.loads(r.read())
            PL.append((round(time.time() - T0, 2) if "T0" in globals() else 0, j.get("hovE"), j.get("kdr"), j.get("air"), j.get("altOn"), j.get("kN")))
        except Exception: pass
        time.sleep(0.25)
for fn in (track, telem, fwpoll): threading.Thread(target=fn, daemon=True).start()
time.sleep(0.6)
print("start y=%.0f score=%.2f vb=%d" % (st["y"], st["score"], st["vb"]))

def setthr(p):
    with lock: cmd["thr"] = int(p * 10)
def run(dur, fn):
    t0 = time.time()
    while time.time() - t0 < dur:
        el = time.time() - t0
        p = fn(el)
        if p is not None: setthr(p)
        log.append((round(time.time() - T0, 3), cmd["thr"], round(st["y"], 1), round(st["score"], 2), st["roll"], st["pitch"], st["vb"]))
        if abs(st["roll"]) > 50 or abs(st["pitch"]) > 50: raise RuntimeError("倾角 %.0f/%.0f" % (st["roll"], st["pitch"]))
        time.sleep(0.05)

T0 = time.time(); res = {"tag": TAG, "mode": MODE}
try:
    send("X"); time.sleep(0.2); send("A,1"); time.sleep(0.4)
    threading.Thread(target=pump, daemon=True).start()
    if MODE == "lift":
        LO, HI, RAMP = f("LO", 35), f("HI", 65), f("RAMP", 12)
        lift = [None]
        def ramp(el):
            p = LO + (HI - LO) * min(1, el / RAMP)
            if lift[0] is None and st["y"] > 12 and st["score"] > 0.5: lift[0] = p; print("离地 @ %.1f%%" % p)
            return p
        run(0.8, lambda el: LO * el / 0.8)
        run(RAMP, ramp)
        res["lift"] = lift[0]
    elif MODE == "steps":
        B, S, H = f("BASE", 50), f("STEP", 5), f("HOLD", 2.5)
        run(1.0, lambda el: B * el)
        for p in (B, B + S, B, B - S, B, B + S, B):
            run(H, lambda el, p=p: p)
    elif MODE == "updown":   # 慢升到顶 → 慢降到落：离地油门和下落油门的差 = 摩擦
        LO, HI, RAMP = f("LO", 55), f("HI", 76), f("RAMP", 8)
        run(1.0, lambda el: LO * el)
        run(RAMP, lambda el: LO + (HI - LO) * el / RAMP)
        run(0.8, lambda el: HI)
        run(RAMP * 1.5, lambda el: HI - (HI - LO) * el / (RAMP * 1.5))
    elif MODE == "att":      # 顶在上限位上做姿态阶跃：横滚、俯仰（AMP×25°）、偏航（YAMP 杆量）
        B, A, YA, H = f("BASE", 74), f("AMP", 0.24), f("YAMP", 0.5), f("HOLD", 1.2)
        def stick(r=0, p=0, y=0): stk.update(r=r, p=p, y=y)
        run(1.0, lambda el: B * el); run(1.5, lambda el: B)
        for kv in ({"r": A}, {}, {"r": -A}, {}, {"p": A}, {}, {"p": -A}, {}, {"y": YA}, {}, {"y": -YA}, {}):
            stick(**kv); run(H, lambda el: B)
    elif MODE == "alt":      # 定高：先开 alt，推到 TK 离地，接管后中立 → 降 → 中立 → 升 → 中立
        TK, D, H = f("TK", 66), f("D", 4), f("HOLD", 3.0)
        send("E,alt,1"); time.sleep(0.2)
        run(1.0, lambda el: TK * min(1, el / 0.8))
        run(H + 1.0, lambda el: TK)
        for p in (TK - D, TK, TK + D, TK):
            run(1.0 if p != TK else H, lambda el, p=p: p)
    run(0.8, lambda el: max(0, cmd["thr"] / 10 - 60 * el))   # 收油门
finally:
    stop[0] = True; time.sleep(0.15); send("G,0", "A,0", "X")
    if MODE == "alt": send("E,alt,0")
a = np.array([r[:3] for r in log]) if log else np.zeros((0, 3))
fn = os.path.join(OUT, time.strftime("%Y%m%d_%H%M%S") + "_" + TAG + ".json")
json.dump({**res, "rows": log, "fw": PL}, open(fn, "w", encoding="utf-8"), ensure_ascii=False)
if len(a):
    print("y px: min %.0f max %.0f；结束 vb %d；%s" % (a[:, 2].min(), a[:, 2].max(), st["vb"], json.dumps(res, ensure_ascii=False)))
print("saved", fn)
