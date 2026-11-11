"""逐个点动电机，摄像头在转的时候拍一张，存 test/spin_M*.jpg，再拍一张静止的作对比。"""
import sys, time, threading, urllib.request
import cv2

IP = sys.argv[1]
PM = int(sys.argv[2]) if len(sys.argv) > 2 else 300

def get(p):
    return urllib.request.urlopen("http://%s%s" % (IP, p), timeout=2).read()

c = cv2.VideoCapture(0, cv2.CAP_MSMF)
c.set(3, 1280); c.set(4, 720)
for _ in range(15): c.read()
ok, f = c.read(); cv2.imwrite("test/spin_still.jpg", f)
for ch in range(4):
    get("/api/jog?i=%d&pm=%d&ms=1500" % (ch, PM))
    t0 = time.time()
    while time.time() - t0 < 0.9: c.read()
    ok, f = c.read()
    cv2.imwrite("test/spin_M%d.jpg" % (ch + 1), f)
    time.sleep(1.2)
get("/api/stop")
c.release()
print("ok")
