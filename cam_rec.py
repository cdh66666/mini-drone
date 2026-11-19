"""飞行录像：摄像头一直开着（曝光稳定、解锁时不用等开机），解锁时开始写，上锁 1 秒后停。

存到 flights/video/<名字>.mp4，旁边 <名字>_t.csv 记每一帧相对解锁时刻的秒数，
名字和 flights/full/<名字>.csv 一样，方便和推算轨迹对齐。
默认关（不开摄像头、不录像）。要开就建 camera.txt，里面写摄像头编号（比如 0）。没装 OpenCV 或打不开也什么都不做。
"""
import os, time, threading, collections

HERE = os.path.dirname(os.path.abspath(__file__))
VDIR = os.path.join(HERE, "flights", "video")
PRE, POST = 1.0, 1.0          # 解锁前多录 1 秒、上锁后多录 1 秒


class CamRec:
    def __init__(self):
        self.lock = threading.Lock()
        self.last = None          # 最新一帧 (t, frame)
        self.ring = collections.deque()   # 解锁前的预录
        self.rec = None           # 正在录：{"t0", "frames": [(t, jpg)]}
        self.stop_at = None
        self.status = "未启动"
        self.fps = 0.0
        self.dark = False
        threading.Thread(target=self._run, daemon=True).start()

    def _open(self):
        import cv2
        idx = 0
        try: idx = int(open(os.path.join(HERE, "camera.txt")).read().strip())
        except Exception: pass
        cap = cv2.VideoCapture(idx, cv2.CAP_DSHOW)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280); cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        cap.set(cv2.CAP_PROP_FPS, 30)
        return cv2, cap

    def _run(self):
        try:
            cv2, cap = self._open()
        except Exception as e:
            self.status = "没有 OpenCV：" + str(e)[:40]; return
        if not cap.isOpened():
            self.status = "打不开摄像头"; return
        self.status = "开着"
        n, tn, dark_since = 0, time.time(), None
        while True:
            ok, f = cap.read()
            now = time.time()
            if not ok:
                time.sleep(0.05); continue
            n += 1
            if now - tn > 2:
                self.fps = n / (now - tn); n, tn = 0, now
                self.dark = bool(f[::16, ::16].mean() < 4)
                # 一直黑（被别的程序占着 / 挡住）：没在录时每 5 秒重开一次，别的程序关了就能恢复
                dark_since = (dark_since or now) if self.dark else None
                if dark_since and now - dark_since > 5 and self.rec is None:
                    cap.release(); time.sleep(0.5)
                    cv2, cap = self._open(); dark_since = now
                    continue
            # 测高：追踪选定颜色的色块，画在画面上（录像里也看得到）
            try: self._track(cv2, f, now)
            except Exception: pass
            # 存 JPEG：内存小，预录和截图都用它
            jpg = cv2.imencode(".jpg", f, [cv2.IMWRITE_JPEG_QUALITY, 80])[1].tobytes()
            with self.lock:
                self.last = (now, jpg)
                if self.rec is not None:
                    self.rec["frames"].append((now, jpg))
                    if self.stop_at and now >= self.stop_at:
                        r, self.rec, self.stop_at = self.rec, None, None
                        threading.Thread(target=self._save, args=(r,), daemon=True).start()
                else:
                    self.ring.append((now, jpg))
                    while self.ring and now - self.ring[0][0] > PRE:
                        self.ring.popleft()

    def start(self, t0):
        """解锁时调用"""
        with self.lock:
            if self.rec is not None:          # 上次还没收尾，直接接着录
                self.stop_at = None; return
            self.rec = {"t0": t0, "frames": list(self.ring), "name": None}
            self.ring.clear()

    def stop(self, name):
        """上锁时调用，name = 整次记录的文件名（不带 .csv）"""
        with self.lock:
            if self.rec is None: return
            self.rec["name"] = name
            self.stop_at = time.time() + POST

    def _save(self, r):
        try:
            import cv2, numpy as np
            fr = r["frames"]
            if len(fr) < 5 or not r["name"]: return
            os.makedirs(VDIR, exist_ok=True)
            dur = fr[-1][0] - fr[0][0]
            fps = max(5.0, min(60.0, (len(fr) - 1) / dur)) if dur > 0 else 30.0
            first = cv2.imdecode(np.frombuffer(fr[0][1], np.uint8), cv2.IMREAD_COLOR)
            h, w = first.shape[:2]
            fp = os.path.join(VDIR, r["name"] + ".mp4")
            vw = cv2.VideoWriter(fp, cv2.VideoWriter_fourcc(*"mp4v"), fps, (w, h))
            for _, j in fr:
                vw.write(cv2.imdecode(np.frombuffer(j, np.uint8), cv2.IMREAD_COLOR))
            vw.release()
            with open(os.path.join(VDIR, r["name"] + "_t.csv"), "w") as f:
                f.write("frame,t\n" + "".join("%d,%.3f\n" % (i, t - r["t0"]) for i, (t, _) in enumerate(fr)))
        except Exception:
            pass

    # ---------------- 摄像头测高（定高测试架用） ----------------
    # cam_track.json：{"hsv":[h,s,v], "tol":[h,s,v], "p0":[y像素, cm], "p1":[y像素, cm]}
    # 在调试台画面上点色块 = 选颜色；点两个刻度 = 标定。高度 = 色块中心 y 按两点线性换算成 cm。
    TFILE = os.path.join(HERE, "cam_track.json")

    def _tcfg(self):
        if not hasattr(self, "_cfg"):
            try:
                import json
                self._cfg = json.load(open(self.TFILE, encoding="utf-8"))
            except Exception:
                self._cfg = {}
        return self._cfg

    def _tsave(self):
        import json
        json.dump(self._cfg, open(self.TFILE, "w", encoding="utf-8"))

    def _track(self, cv2, f, now):
        import numpy as np
        c = self._tcfg(); self.frame_hw = f.shape[:2]
        if "hsv" not in c: self.trk = None; return
        s = 2   # 缩小一半再找，省时间
        sm = cv2.resize(f, (f.shape[1] // s, f.shape[0] // s))
        hsv = cv2.cvtColor(sm, cv2.COLOR_BGR2HSV)
        h, sa, v = c["hsv"]; th, ts, tv = c.get("tol", [10, 70, 70])
        lo = np.array([max(0, h - th), max(0, sa - ts), max(0, v - tv)]); hi = np.array([min(179, h + th), min(255, sa + ts), min(255, v + tv)])
        m = cv2.inRange(hsv, lo, hi)
        if h - th < 0 or h + th > 179:   # 红色跨 0
            lo2, hi2 = lo.copy(), hi.copy()
            if h - th < 0: lo2[0], hi2[0] = 180 + h - th, 179
            else: lo2[0], hi2[0] = 0, h + th - 180
            m = cv2.bitwise_or(m, cv2.inRange(hsv, lo2, hi2))
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        n, lab, st, cen = cv2.connectedComponentsWithStats(m)
        if n <= 1: self.trk = None; return
        i = 1 + int(np.argmax(st[1:, cv2.CC_STAT_AREA]))
        if st[i, cv2.CC_STAT_AREA] < 15: self.trk = None; return
        cx, cy = cen[i][0] * s, cen[i][1] * s
        cm = None
        if "p0" in c and "p1" in c and c["p1"][0] != c["p0"][0]:
            (y0, c0), (y1, c1) = c["p0"], c["p1"]
            cm = c0 + (cy - y0) * (c1 - c0) / (y1 - y0)
        self.trk = (now, cx, cy, cm)
        cv2.circle(f, (int(cx), int(cy)), 14, (0, 255, 255), 2)
        if cm is not None:
            cv2.putText(f, "%.1f cm" % cm, (int(cx) + 18, int(cy) + 6), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2)
        for k in ("p0", "p1"):
            if k in c: cv2.line(f, (0, int(c[k][0])), (40, int(c[k][0])), (255, 0, 255), 2)

    def track(self):
        """最新的测高结果：{"ok", "cm", "cx", "cy", "age"}"""
        t = getattr(self, "trk", None)
        if not t: return {"ok": 0, "cfg": self._tcfg()}
        return {"ok": 1, "cm": None if t[3] is None else round(t[3], 1), "cx": int(t[1]), "cy": int(t[2]),
                "age": round(time.time() - t[0], 2), "cfg": self._tcfg()}

    def track_cm(self):
        t = getattr(self, "trk", None)
        if not t or t[3] is None or time.time() - t[0] > 0.3: return ""
        return "%.1f" % t[3]

    def pick(self, x, y):
        """点画面选颜色：取点击处 9x9 的 HSV 中位数"""
        import cv2, numpy as np
        with self.lock:
            j = self.last[1] if self.last else None
        if j is None: return {"ok": 0}
        f = cv2.imdecode(np.frombuffer(j, np.uint8), cv2.IMREAD_COLOR)
        x, y = int(x), int(y); p = f[max(0, y - 4):y + 5, max(0, x - 4):x + 5]
        hsv = cv2.cvtColor(p, cv2.COLOR_BGR2HSV).reshape(-1, 3)
        c = self._tcfg(); c["hsv"] = [int(v) for v in np.median(hsv, axis=0)]; c.setdefault("tol", [10, 70, 70]); self._tsave()
        return {"ok": 1, "hsv": c["hsv"]}

    def cal(self, i, y, cm):
        c = self._tcfg(); c["p%d" % (1 if int(i) else 0)] = [float(y), float(cm)]; self._tsave()
        return {"ok": 1, "cfg": c}

    def snapshot(self):
        with self.lock:
            return self.last[1] if self.last else None

    def info(self):
        return {"status": self.status, "fps": round(self.fps, 1), "dark": self.dark,
                "rec": self.rec is not None}


class _Off:
    """摄像头关着：什么都不做，不开摄像头、不占内存"""
    def start(self, t0): pass
    def stop(self, name): pass
    def snapshot(self): return None
    def info(self): return {"status": "关", "fps": 0, "dark": False, "rec": False}
    def track(self): return {"ok": 0}
    def track_cm(self): return ""
    def pick(self, x, y): return {"ok": 0}
    def cal(self, i, y, cm): return {"ok": 0}


CAM = None


def enabled():
    """默认关。要开：在 camera.txt 里写摄像头编号（比如 0）"""
    try:
        return open(os.path.join(HERE, "camera.txt")).read().strip().isdigit()
    except Exception:
        return False


def get():
    global CAM
    if CAM is None:
        CAM = CamRec() if enabled() else _Off()
    return CAM
