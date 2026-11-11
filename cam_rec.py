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
