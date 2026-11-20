"""飞行时后台记录固件内部状态（定高/辨识），每 0.25 s 读一次 /api/params + /api/state，存 flights/fwlog/<时间>.jsonl。
用法：python -I tools/fw_log.py [秒数=300]"""
import json, os, sys, time, urllib.request
HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
IP = open(os.path.join(HERE, "drone_ip.txt")).read().strip(); D = os.path.join(HERE, "flights", "fwlog"); os.makedirs(D, exist_ok=True)
O = urllib.request.build_opener(urllib.request.ProxyHandler({})); T = float(sys.argv[1]) if len(sys.argv) > 1 else 300
fn = os.path.join(D, time.strftime("%Y%m%d_%H%M%S") + ".jsonl"); t0 = time.time()
with open(fn, "w", encoding="utf-8") as f:
    while time.time() - t0 < T:
        try:
            with O.open("http://%s/api/params" % IP, timeout=0.6) as r: p = json.loads(r.read())
            with O.open("http://%s/api/state" % IP, timeout=0.6) as r: s = json.loads(r.read())
            f.write(json.dumps({"t": round(time.time(), 2), "arm": s.get("arm"), "thr": s.get("thr"), "vb": s.get("vb"), "az": s.get("az"),
                                **{k: p.get(k) for k in ("alt", "altOn", "hovE", "kN", "air", "kdr", "hadj", "akv", "aki")}}) + "\n"); f.flush()
        except Exception: pass
        time.sleep(0.25)
print("saved", fn)
