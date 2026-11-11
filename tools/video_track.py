"""从飞行录像里找飞机、算它实际跑了多远。

用法：python -I tools/video_track.py [名字]      名字 = flights/video/<名字>.mp4，不写就用最新一个
输出：
  flights/video/<名字>_track.csv   每帧：t、像素中心 cx/cy、框宽 w、相对起飞点的米数 dx（画面左右，右为正）/ dy（画面上下，上为正）
  flights/video/<名字>_check.jpg   飞机附近的放大抽帧（带框和轨迹），给人看追得对不对
方法：解锁前 1 秒的画面取中位数当背景（飞机停在地上），之后每帧和背景比，只要“小块”（飞机大小，
排除人这种大块），从起飞那一刻开始按“预测位置最近”一路跟。比例尺：飞机宽 DRONE_W 米 = 地上时框宽像素。
摄像头从侧面拍：左右准；画面上下是“高度 + 前后远近”混在一起，只能看趋势。
"""
import os, sys, glob, csv
import numpy as np, cv2

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
VDIR = os.path.join(HERE, "flights", "video")
DRONE_W = 0.10      # 飞机最宽处（桨尖到桨尖），米
MAXSZ = 160         # 比这大的块（人、手）不要


def pick(name):
    if name:
        return os.path.join(VDIR, name + ".mp4")
    v = sorted(glob.glob(os.path.join(VDIR, "*.mp4")))
    return v[-1] if v else None


def blobs(g, bg):
    d = cv2.absdiff(g, bg)
    _, m = cv2.threshold(d, 22, 255, cv2.THRESH_BINARY)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    m = cv2.dilate(m, np.ones((9, 9), np.uint8))
    n, lab, st, cen = cv2.connectedComponentsWithStats(m)
    out, big = [], []
    for i in range(1, n):
        x, y, w, h, a = st[i]
        if w > MAXSZ or h > MAXSZ:
            big.append((x, y, w, h)); continue
        if a < 60: continue
        out.append((x + w / 2, y + h / 2, w, h))
    return out


def main():
    fp = pick(sys.argv[1] if len(sys.argv) > 1 else None)
    if not fp or not os.path.exists(fp):
        print("没有录像"); return
    name = os.path.basename(fp)[:-4]
    with open(fp[:-4] + "_t.csv") as f:
        ts = np.array([float(r["t"]) for r in csv.DictReader(f)])
    cap = cv2.VideoCapture(fp); frames = []
    while True:
        ok, f = cap.read()
        if not ok: break
        frames.append(f)
    n = min(len(frames), len(ts)); frames, ts = frames[:n], ts[:n]
    gray = [cv2.GaussianBlur(cv2.cvtColor(f, cv2.COLOR_BGR2GRAY), (5, 5), 0) for f in frames]
    if n < 10 or np.mean([g.mean() for g in gray[:5]]) < 4:
        print("录像太短或全黑"); return
    pre = [i for i in range(n) if ts[i] < 0] or [0, 1, 2]
    bg = np.median(np.stack([gray[i] for i in pre]), axis=0).astype(np.uint8)

    # 起飞时刻：整次记录里油门第一次 > 0 之后约 0.2 秒；没有就从解锁开始找
    t_go = 0.0
    fl = os.path.join(HERE, "flights", "full", name + ".csv")
    if os.path.exists(fl):
        with open(fl) as f:
            for r in csv.DictReader(f):
                if float(r["thr_cmd"]) > 0: t_go = float(r["t"]); break

    B = [blobs(gray[i], bg) if ts[i] >= t_go else [] for i in range(n)]

    def follow(i0, b0):
        R = np.full((n, 5), np.nan); R[:, 0] = ts; R[i0, 1:] = b0
        pos, vel, miss, left = np.array(b0[:2]), np.zeros(2), 0, False
        for i in range(i0 + 1, n):
            pred = pos + vel
            if np.hypot(*(pos - b0[:2])) > 60: left = True
            # 飞机离开后，起飞点留下的“空位”也会和背景不同；离开过就不再回那里
            cand = [b for b in B[i] if np.hypot(b[0] - pred[0], b[1] - pred[1]) < min(120, 80 + 20 * miss)
                    and not (left and np.hypot(b[0] - b0[0], b[1] - b0[1]) < 40)]
            if not cand:
                miss += 1
                if miss > 5: break
                continue
            b = min(cand, key=lambda b: np.hypot(b[0] - pred[0], b[1] - pred[1]))
            newp = np.array(b[:2]); vel = 0.6 * vel + 0.4 * (newp - pos) / (miss + 1); pos = newp; miss = 0
            R[i, 1:] = b
        return R

    # 起点：起飞后 1.5 秒内出现的每个小块都试着往下跟，挑“跟得久、又真的在动”的那条
    # （背景里闪的灯、屏幕不会挪地方；飞机一起飞就会动）
    # 起飞点提示：命令行 x,y，或上次记住的 start.txt；只在它附近 150 像素找起点
    hint = None
    sfile = os.path.join(VDIR, "start.txt")
    try:
        hint = tuple(map(float, (sys.argv[2] if len(sys.argv) > 2 else open(sfile).read()).split(",")))
    except Exception:
        pass
    best, bscore = None, 0
    for i in range(n):
        if not (t_go <= ts[i] <= t_go + 1.5): continue
        for b in B[i]:
            if hint and np.hypot(b[0] - hint[0], b[1] - hint[1]) > 150: continue
            R = follow(i, b); okk = ~np.isnan(R[:, 1])
            if okk.sum() < 5: continue
            p = R[okk, 1:3]
            span = np.max(np.hypot(p[:, 0] - p[0, 0], p[:, 1] - p[0, 1]))
            score = okk.sum() * min(span, 300) if span > 15 else 0
            if score > bscore: best, bscore, i0 = R, score, i
    if best is None:
        print("没追到飞机（画面里没找到会动的小块）"); return
    R = best
    ok = ~np.isnan(R[:, 1])
    with open(sfile, "w") as f:
        f.write("%.0f,%.0f" % (R[i0, 1], R[i0, 2]))
    # 比例尺：地上时飞机宽。用背景里起点附近那块（飞机静止时的样子）量
    x0, y0 = R[i0, 1], R[i0, 2]
    # 飞机宽：起飞后 0.3~1.5 秒的框宽中位数，减掉膨胀加的 8 像素
    sel = ok & (R[:, 0] > R[i0, 0] + 0.3) & (R[:, 0] < R[i0, 0] + 1.5)
    w_px = float(np.median(R[sel, 3]) - 8) if sel.sum() >= 3 else float(np.nanmedian(R[i0:i0 + 4, 3]))
    mpp = DRONE_W / max(w_px, 10)
    dx = (R[:, 1] - x0) * mpp
    dy = -(R[:, 2] - y0) * mpp

    with open(os.path.join(VDIR, name + "_track.csv"), "w") as f:
        f.write("t,cx,cy,w,h,dx_m,dy_m\n")
        for r, a, b in zip(R, dx, dy):
            f.write("%.3f,%.1f,%.1f,%.1f,%.1f,%.3f,%.3f\n" % (r[0], r[1], r[2], r[3], r[4], a, b))

    # 放大拼图：以起点为中心的 640x400 区域
    H, W = gray[0].shape
    cx0 = int(min(max(x0, 320), W - 320)); cy0 = int(min(max(y0, 200), H - 200))
    last = np.where(ok)[0][-1]
    idx = np.linspace(max(0, i0 - 3), min(n - 1, last + 3), 18).astype(int)
    tiles = []
    for i in idx:
        f = frames[i].copy()
        pts = [(int(R[j, 1]), int(R[j, 2])) for j in range(i0, i + 1) if ok[j]]
        for a, b in zip(pts, pts[1:]): cv2.line(f, a, b, (0, 255, 0), 2)
        if ok[i]:
            c = R[i]
            cv2.rectangle(f, (int(c[1] - c[3] / 2), int(c[2] - c[4] / 2)), (int(c[1] + c[3] / 2), int(c[2] + c[4] / 2)), (0, 0, 255), 2)
        crop = f[cy0 - 200:cy0 + 200, cx0 - 320:cx0 + 320]
        cv2.putText(crop, "%.1fs" % ts[i], (8, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2)
        tiles.append(cv2.resize(crop, (320, 200)))
    grid = np.vstack([np.hstack(tiles[r:r + 6]) for r in range(0, 18, 6)])
    cv2.imwrite(os.path.join(VDIR, name + "_check.jpg"), grid)

    seg = ok & (R[:, 0] <= R[i0, 0] + 5)
    tt = R[seg, 0] - R[i0, 0]
    print("录像 %s：%.1f fps，比例 %.1f 毫米/像素（飞机宽 %d 像素 = %.0f 厘米）" % (
        name, (n - 1) / (ts[-1] - ts[0]), mpp * 1000, w_px, DRONE_W * 100))
    print("从起飞起追到 %.1f 秒%s" % (tt[-1], "（之后出画面/丢了）" if R[last, 0] < ts[-1] - 0.5 else ""))
    for s in (0.5, 1.0, 1.5, 2.0, 3.0, 5.0):
        k = np.where(seg & (R[:, 0] - R[i0, 0] <= s))[0]
        if len(k) and R[k[-1], 0] - R[i0, 0] > s - 0.15:
            print("  %.1f s：左右 %+.2f m，上下 %+.2f m" % (s, dx[k[-1]], dy[k[-1]]))
    print("拼图：", os.path.join(VDIR, name + "_check.jpg"))


if __name__ == "__main__":
    main()
