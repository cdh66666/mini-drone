"""台架自动测试：解锁 → 慢推到 THR → 悬停 → 横滚/俯仰阶跃 → 上锁 → 读黑匣子 → 算指标。

用法：python -I tools/stand_test.py <飞机地址> <标签> [k=v ...]
  k=v 是测前要设的参数（只改内存，不存盘），例如 rp=0.8 pp=0.8
  另外可选：THR=550（油门杆 ‰）、AMP=0.24（阶跃杆量，×25° = 目标角）
结果：flights/stand/<时间>_<标签>.csv（黑匣子原样）+ 一行指标追加到 flights/stand/summary.csv
安全：倾角 >30° 或通信失败立即急停；推力小于机重，飞机靠自重压在球头上。
"""
import json, os, sys, threading, time, http.client, urllib.parse
import numpy as np

HOST = sys.argv[1]; TAG = sys.argv[2]
KV = dict(a.split("=", 1) for a in sys.argv[3:])
THR = int(KV.pop("THR", 550)); AMP = float(KV.pop("AMP", 0.24))
HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(HERE, "flights", "stand"); os.makedirs(OUT, exist_ok=True)

def get(path, timeout=2.0):
    c = http.client.HTTPConnection(HOST, timeout=timeout)
    try:
        c.request("GET", path); return c.getresponse().read().decode("utf-8", "replace")
    finally:
        c.close()

def stop():
    for _ in range(3):
        try: get("/api/stop", 1.0)
        except Exception: pass

# 设参数（只改内存）
for k, v in KV.items():
    get("/api/param?k=%s&v=%s" % (k, v))
par = json.loads(get("/api/params"))
st0 = json.loads(get("/api/state"))
print("params", {k: par.get(k) for k in ("angP", "rp", "ri", "rd", "pp", "pi", "pd", "yff", "crate", "lp2", "dfc", "cap")}, "vb", st0["vb"])

import socket
cmd = {"thr": 0, "r": 0.0, "p": 0.0, "y": 0.0}; alive = [True]; mx = [0.0]; last_rx = [time.time()]
sk = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); sk.settimeout(0.02)
def pump():   # UDP 50 Hz：U,seq,live,thr,r,p,y（和正式飞行同一条链路），回包 T 里有姿态
    seq = 0
    while alive[0]:
        seq += 1
        try: sk.sendto(("U,%d,1,%d,%.3f,%.3f,%.3f" % (seq, cmd["thr"], cmd["r"], cmd["p"], cmd["y"])).encode(), (HOST, 4210))
        except Exception: pass
        try:
            while True:
                b, _ = sk.recvfrom(512); a = b.decode("utf-8", "replace").split(",")
                if a[0] == "T" and len(a) > 6:
                    last_rx[0] = time.time()
                    tl = max(abs(float(a[5])), abs(float(a[6]))); mx[0] = max(mx[0], tl)
                    if a[3] == "1" and tl > 30: print("!! 倾角 %.0f°，急停" % tl); alive[0] = False; stop(); return
        except socket.timeout:
            pass
        except Exception:
            pass
        if time.time() - last_rx[0] > 0.6: print("!! 回包断了，急停"); alive[0] = False; stop(); return
        time.sleep(0.015)
def watch():
    pass

r = json.loads(get("/api/arm?on=1"))
if not r.get("ok"):
    print("解锁失败:", r); sys.exit(1)
threading.Thread(target=pump, daemon=True).start(); threading.Thread(target=watch, daemon=True).start()
# 时间表（秒）：慢推 1.0 → 稳住 1.6 → 横滚 +0.7 → 回 0.7 → 俯仰 +0.7 → 回 0.7（后 4.8 秒进黑匣子）
plan = [(1.0, "ramp"), (3.0, None), (1.0, ("r", AMP)), (1.0, ("r", 0)), (1.0, ("p", AMP)), (1.0, ("p", 0))]   # 11-19：平稳段拉长到 3 s（长黑匣子全程都有）
t0 = time.time()
try:
    for dur, act in plan:
        te = time.time() + dur
        while time.time() < te and alive[0]:
            if act == "ramp": cmd["thr"] = int(THR * min(1.0, (time.time() - t0) / dur))
            elif act: cmd[act[0]] = act[1]
            time.sleep(0.01)
        if not alive[0]: break
finally:
    alive[0] = False; time.sleep(0.06)
    cmd.update(thr=0, r=0, p=0)
    try: get("/api/arm?on=0")
    except Exception: pass
    stop()
time.sleep(0.9)
# v20 长黑匣子（1 kHz 原始陀螺）：存 .bin，并算振动频谱
try:
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import lb_read
    c = http.client.HTTPConnection(HOST, timeout=15); c.request("GET", "/api/lb"); rb = c.getresponse()
    lbb = rb.read() if rb.status == 200 else b""
    if lbb:
        open(os.path.join(OUT, time.strftime("%Y%m%d_%H%M%S") + "_" + TAG + ".lb"), "wb").write(lbb)
        sp = lb_read.spectrum(lb_read.parse(lbb)[1])
        if sp: print("vib peaks", sp["peaks"], "rms raw/filt", sp["rms_raw"], sp["rms_filt"])
except Exception as e:
    print("lb fail", e)
txt = get("/api/bb", 8.0)
fn = time.strftime("%Y%m%d_%H%M%S") + "_" + TAG + ".csv"
open(os.path.join(OUT, fn), "w", encoding="utf-8").write(txt)

# ---- 指标 ----
L = txt.splitlines(); hdr = L[1].split(",")
d = np.array([[float(x) for x in l.split(",")] for l in L[2:] if l and l[0].isdigit()])
D = {k: d[:, i] for i, k in enumerate(hdr)}
t = (D["ms"] - D["ms"][0]) / 1000.0
on = D["thr"] > 300
def seg(mask): return mask & on
res = {"tag": TAG, "vb0": st0["vb"], "maxtilt": round(mx[0], 1)}
tilt = np.maximum(np.abs(D["roll"]), np.abs(D["pitch"]))
res["lim%"] = round(float(100 * (tilt[on] > 17).mean()), 0) if on.any() else -1   # 贴着架子限位的时间占比
res["tilt_rms"] = round(float(np.sqrt((tilt[on] ** 2).mean())), 1) if on.any() else -1
# 稳住段：没给杆
calm = seg((np.abs(D["stR"]) < 0.01) & (np.abs(D["stP"]) < 0.01))
ci = np.where(calm)[0]
if len(ci) > 100:
    w = ci[: min(len(ci), 350)]   # 第一段静止（阶跃前）
    for ax, g in (("roll", "gr"), ("pitch", "gp")):
        x = D[ax][w] - D[ax][w].mean()
        res[ax + "_std"] = round(float(x.std()), 2)
        res[g + "_std"] = round(float(D[g][w].std()), 1)
        F = np.abs(np.fft.rfft(x * np.hanning(len(x)))); fr = np.fft.rfftfreq(len(x), 0.004)
        m = fr > 0.5; res[ax + "_pk_hz"] = round(float(fr[m][np.argmax(F[m])]), 1)
    # 高频噪声（>20 Hz）占比：判断有没有开始抖
    for g in ("gr", "gp"):
        x = D[g][w] - D[g][w].mean(); P = np.abs(np.fft.rfft(x)) ** 2; fr = np.fft.rfftfreq(len(x), 0.004)
        res[g + "_hf%"] = round(float(100 * P[fr > 20].sum() / max(P.sum(), 1e-9)), 1)
# 阶跃：目标 = 杆量 × 25°，相对阶跃前的角度
for ax, sk in (("roll", "stR"), ("pitch", "stP")):
    s = D[sk]; i = np.where((s[1:] > 0.1) & (s[:-1] < 0.1))[0]
    if len(i) == 0: continue
    i0 = i[0] + 1; base = D[ax][max(0, i0 - 25):i0].mean(); tgt = s[i0] * 25.0
    y = D[ax][i0:i0 + 175] - base          # 0.7 s
    if len(y) < 50: continue
    tt = np.arange(len(y)) * 0.004
    k90 = np.where(y >= 0.9 * tgt)[0]
    res[ax + "_rise_ms"] = int(tt[k90[0]] * 1000) if len(k90) else -1
    res[ax + "_over%"] = round(float(100 * (y.max() - tgt) / tgt), 0)
    res[ax + "_final"] = round(float(y[-25:].mean() / tgt), 2)
res["vb_min"] = int(json.loads(get("/api/state"))["vb"])
print(json.dumps(res, ensure_ascii=False))
sp = os.path.join(OUT, "summary.csv")
new = not os.path.exists(sp)
with open(sp, "a", encoding="utf-8") as f:
    f.write(json.dumps({"time": time.strftime("%H:%M:%S"), **res, "par": {k: par.get(k) for k in ("angP", "rp", "rd", "ri", "dfc", "lp2", "crate")}}, ensure_ascii=False) + "\n")
