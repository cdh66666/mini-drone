"""通过本地服务模拟页面：先空闲 10 s，再逐个点动电机，再解锁（油门 0）保持 20 s 发心跳，最后急停。
看 #LQ（丢包/延迟/最长空档）和解锁有没有被意外断掉。"""
import json, time, urllib.request
B = "http://127.0.0.1:8771"
def post(s): urllib.request.urlopen(urllib.request.Request(B + "/api/send", data=s.encode(), method="POST")).read()
def poll(since): return json.loads(urllib.request.urlopen(B + "/api/poll?since=%d" % since).read())
n = poll(0)["n"]
def drain(tag):
    global n
    p = poll(n); n = p["n"]
    lq = [l for l in p["lines"] if l.startswith("#LQ")]
    arm = [l.split(",")[14] for l in p["lines"] if l.startswith("D,")]
    other = [l for l in p["lines"] if l.startswith("#") and not l.startswith(("#LQ", "#DBG"))]
    print(tag, "LQ(loss%,rtt,rssi,gap):", [l[4:] for l in lq], "arm:", "".join(arm[-40:]), other[-4:])
time.sleep(10); drain("idle")
for ch in (1, 2, 3, 4):
    post("J,%d,300,1500" % ch); time.sleep(2.2)
drain("jog")
post("X"); time.sleep(0.3); post("A,1"); time.sleep(0.5)
t0 = time.time()
while time.time() - t0 < 20:
    post("G,0\nQ,0.00,0.00,0"); time.sleep(0.1)
drain("armed")
post("X"); time.sleep(0.5); drain("stop")
