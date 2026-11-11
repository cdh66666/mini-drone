# 每 5 秒记一次电压和是否插 USB（从本地服务的遥测里取），写 charge_log.csv
import json, time, urllib.request, sys
n = 0; since = 0
dur = float(sys.argv[1]) if len(sys.argv) > 1 else 180
t0 = time.time()
with open("charge_log.csv", "a", encoding="utf-8") as f:
    while time.time() - t0 < dur:
        try:
            j = json.loads(urllib.request.urlopen("http://127.0.0.1:8771/api/poll?since=%d" % since, timeout=3).read())
            since = j["n"]
            D = [l for l in j["lines"] if l.startswith("D,")]
            if D:
                a = D[-1].split(","); vb = sum(int(x.split(",")[2]) for x in D[-10:]) / len(D[-10:])
                f.write("%s,%.0f,%s\n" % (time.strftime("%H:%M:%S"), vb, a[3])); f.flush()
        except Exception as e:
            pass
        time.sleep(5)
