"""卡顿压力测试：飞机固定在架子上，分几段加负载，每秒读一次固件性能统计（串口 M 命令，走本地 drone_server）。

用法：python -I tools/stall_test.py <标签> [段秒数=15] [转速‰=400]
负载段：空闲 → WiFi 狂发（UDP 50 Hz + HTTP 20 Hz）→ 电机转 → 电机转 + WiFi → 电机转 + 下载黑匣子 → 空闲 + 存参数
结果：flights/perf/<时间>_<标签>.json（每秒原始统计）+ 屏幕上每段一行汇总
安全：电机只用推力测试模式（H，四路同占空比，不经姿态控制），每 0.2 s 续命，结束/出错都发 X 停转。
"""
import json, os, socket, sys, threading, time, urllib.request

TAG = sys.argv[1]; SEG = float(sys.argv[2]) if len(sys.argv) > 2 else 15; PM = int(sys.argv[3]) if len(sys.argv) > 3 else 400
HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRV = "http://127.0.0.1:8771"
IP = open(os.path.join(HERE, "drone_ip.txt")).read().strip()
OUT = os.path.join(HERE, "flights", "perf"); os.makedirs(OUT, exist_ok=True)
NOPROXY = urllib.request.build_opener(urllib.request.ProxyHandler({}))

def get(url, t=2.0):
    with NOPROXY.open(url, timeout=t) as r: return r.read()
def send(*lines):
    req = urllib.request.Request(SRV + "/api/send?cid=stalltest", data="\n".join(lines).encode(), method="POST")
    try: NOPROXY.open(req, timeout=2).read()
    except Exception: pass
since = [json.loads(get(SRV + "/api/poll?since=0"))["n"]]
def perf():
    for _ in range(4):   # 串口偶尔丢一行：最多重试 4 次
        send("M"); time.sleep(0.25)
        try:
            p = json.loads(get(SRV + "/api/poll?since=%d" % since[0])); since[0] = p["n"]
        except Exception:
            continue
        js = [json.loads(l[6:]) for l in p["lines"] if l.startswith("#PERF,")]
        if js: return js[-1]
    return None

state = {"spin": 0, "wifi": False, "stop": False}
def spin_loop():
    while not state["stop"]:
        if state["spin"]: send("H,%d,0" % state["spin"])
        time.sleep(0.2)
def wifi_loop():
    sk = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); seq = 0; k = 0
    while not state["stop"]:
        if state["wifi"]:
            seq += 1; k += 1
            try: sk.sendto(("U,%d,0,0,0,0,0" % seq).encode(), (IP, 4210))   # live=0：只是链路包，不会控制电机
            except Exception: pass
            if k % 2 == 0:
                try: get("http://%s/api/state" % IP, 0.5)
                except Exception: pass
        time.sleep(0.02)
for f in (spin_loop, wifi_loop): threading.Thread(target=f, daemon=True).start()

def lb_dl():
    try: get("http://%s/api/lb" % IP, 15)
    except Exception: pass
def save_loop(end):
    while time.time() < end and not state["stop"]:
        send("W"); time.sleep(2)

PH = [("空闲", 0, False, None), ("WiFi 狂发", 0, True, None), ("电机转", PM, False, None),
      ("电机转+WiFi", PM, True, None), ("电机转+下载黑匣子", PM, True, "lb"), ("空闲+存参数", 0, False, "save")]
rows = []; summ = []
p0 = perf()
print("fw", p0 and p0.get("fw"), "节拍来源", "硬件定时器" if p0 and p0.get("tsrc") == 1 else "esp_timer")
try:
    for name, sp, wf, extra in PH:
        state["spin"] = sp; state["wifi"] = wf
        if sp == 0: send("H,0", "X")
        time.sleep(1.0)
        a = perf(); end = time.time() + SEG; mx = {"gapMax": 0, "wakeMax": 0, "tStepMax": 0}
        if extra == "lb":
            def lbs():
                while time.time() < end and not state["stop"]: lb_dl()
            threading.Thread(target=lbs, daemon=True).start()
        if extra == "save": threading.Thread(target=save_loop, args=(end,), daemon=True).start()
        while time.time() < end:
            time.sleep(0.75); q = perf()
            if not q: continue
            q["phase"] = name; rows.append(q)
            for k in mx: mx[k] = max(mx[k], q.get(k, 0))
        b = perf() or q
        nt = b["nTick"] - a["nTick"]; ng = b["nGap"] - a["nGap"]
        jh = [y - x for x, y in zip(a["jh"], b["jh"])]; tot = max(1, sum(jh))
        s = {"phase": name, "ticks": nt, "hz": round(nt / (SEG + 0.5), 0), "gaps": ng, "gapMax": mx["gapMax"], "wakeMax": mx["wakeMax"],
             "stepMax": mx["tStepMax"], "jit_lt20": round(100 * jh[0] / tot, 2), "jit_gt200": round(100 * (jh[4] + jh[5]) / tot, 3),
             "i2cFail": b["i2cFail"], "tImu": b["tImu"]}
        summ.append(s)
        print("%-14s 拍 %6d 卡顿(>1.5ms) %3d 最长 %5d us 唤醒最慢 %4d us 最慢一拍 %4d us  准(<20us) %5.1f%%  偏>200us %.3f%%  I2C失败 %d" %
              (name, nt, ng, mx["gapMax"], mx["wakeMax"], mx["tStepMax"], s["jit_lt20"], s["jit_gt200"], b["i2cFail"]))
finally:
    state["stop"] = True; state["spin"] = 0; time.sleep(0.3); send("H,0", "X")
fn = os.path.join(OUT, time.strftime("%Y%m%d_%H%M%S") + "_" + TAG + ".json")
json.dump({"tag": TAG, "seg": SEG, "pm": PM, "summary": summ, "rows": rows}, open(fn, "w", encoding="utf-8"), ensure_ascii=False)
print("saved", fn)
