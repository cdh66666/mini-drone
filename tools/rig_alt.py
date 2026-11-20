"""定高架录像 → 飞机高度（mm），和整次飞行 CSV 对齐，画图。

用法：python -I tools/rig_alt.py <记录名，如 20261120_145802> [输出 png]
测高：在飞机所在的竖条里，从塔顶往上找第一段连续亮的行 = 飞机白色机身的底边（塔顶以上背景是深色门框/棕色横梁，比机身暗）。
比例：塔顶面离底座 394 mm（test_stand/alt_stand.py 的 H），画面里量出来约 0.92 mm/像素。
"""
import csv, os, sys
import numpy as np, cv2
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
plt.rcParams["font.sans-serif"] = ["Microsoft YaHei"]; plt.rcParams["axes.unicode_minus"] = False

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
NAME = sys.argv[1]; PNG = sys.argv[2] if len(sys.argv) > 2 and "=" not in sys.argv[2] else os.path.join(HERE, "flights", "rig", NAME + "_alt.png")
KV = dict(a.split("=", 1) for a in sys.argv[3:] if "=" in a)
X0, X1, YTOP, TH, MMPX = int(KV.get("X0", 846)), int(KV.get("X1", 860)), int(KV.get("YTOP", 238)), int(KV.get("TH", 175)), float(KV.get("MMPX", 0.866))   # 11-20 新架子机位

def bottom(g):
    col = g[0:YTOP, X0:X1].max(1).astype(int)
    run = 0
    for y in range(YTOP - 1, 0, -1):
        run = run + 1 if col[y] > TH else 0
        if run >= 3: return y + 3
    return None

cap = cv2.VideoCapture(os.path.join(HERE, "flights", "video", NAME + ".mp4"))
ft = [float(r["t"]) for r in csv.DictReader(open(os.path.join(HERE, "flights", "video", NAME + "_t.csv")))]
G = []
for t in ft:
    ok, f = cap.read()
    if not ok: break
    G.append(cv2.cvtColor(f, cv2.COLOR_BGR2GRAY))
ft = ft[:len(G)]
if "X0" not in KV:   # 自动找飞机：解锁后 0.5~3 s 桨在转，画面变化最大的地方就是飞机；再往下找白色塔顶
    idx = [i for i, t in enumerate(ft) if 0.5 < t < 3.0] or list(range(min(30, len(G))))
    A = np.array([G[i] for i in idx], np.float32); S = np.std(A, axis=0)
    S[cv2.dilate((A.mean(0) > 215).astype(np.uint8), np.ones((25, 25), np.uint8)) > 0] = 0   # 灯管会闪：太亮的地方不算
    S = cv2.GaussianBlur(S, (31, 31), 0)
    cy, cx = np.unravel_index(np.argmax(S[:500]), S[:500].shape)
    g0 = G[0].astype(int); col = g0[:, cx - 4:cx + 5].min(1)
    yt = next((y for y in range(cy + 20, 700) if (col[y:y + 12] > 195).all()), cy + 90)
    X0, X1, YTOP = cx - 7, cx + 7, yt - 4
    print("自动定位：飞机 x=%d y=%d，塔顶 y=%d" % (cx, cy, yt))
ys = [bottom(g) for g in G]
ft = np.array(ft[:len(ys)]); yb = np.array([np.nan if v is None else v for v in ys], float)
y0 = np.nanmedian(yb[ft < 0]) if (ft < 0).any() else np.nanmax(yb)
h = (y0 - yb) * MMPX
R = list(csv.DictReader(open(os.path.join(HERE, "flights", "full", NAME + ".csv"))))
t = np.array([float(r["t"]) for r in R]); thr = np.array([float(r["thr_cmd"]) for r in R]) / 10
vb = np.array([float(r["vb_mv"]) for r in R]) / 1000
o = np.array([[float(r["o%d" % i]) for i in range(1, 5)] for r in R]).mean(1) / 10
ro = np.array([float(r["roll"]) for r in R]); pi = np.array([float(r["pitch"]) for r in R])

fig, ax = plt.subplots(3, 1, figsize=(11, 8), sharex=True, gridspec_kw={"height_ratios": [1.4, 1, 0.8]})
ax[0].plot(ft, h, "o-", ms=3, color="#0b7a83", label="摄像头测的高度")
ax[0].set_ylabel("离下限位 mm"); ax[0].grid(alpha=.3); ax[0].legend(loc="upper left")
ax[1].plot(t, thr, color="#3b9eff", label="油门杆 %"); ax[1].plot(t, o, color="#f0b429", label="电机平均输出 %")
ax[1].plot(t, (vb - 3) * 100, color="#8a97a6", label="电池（3.0~4.0 V）"); ax[1].set_ylim(0, 105); ax[1].grid(alpha=.3); ax[1].legend(loc="upper left", fontsize=8)
ax[2].plot(t, ro, color="#3cc2cc", label="横滚 °"); ax[2].plot(t, pi, color="#ef7a64", label="俯仰 °"); ax[2].set_ylim(-15, 15)
ax[2].grid(alpha=.3); ax[2].legend(loc="upper left", fontsize=8); ax[2].set_xlabel("解锁后 s")
fig.suptitle("定高架 " + NAME); fig.tight_layout(); fig.savefig(PNG, dpi=90)
# 带标注的录像：飞机底边横线 + 高度/油门/电压/姿态，左侧高度条；另存最低/最高两帧对比图（检查摄像头是否拍全行程）
thr_v = np.interp(ft, t, thr); vb_v = np.interp(ft, t, vb); ro_v = np.interp(ft, t, ro); pi_v = np.interp(ft, t, pi)
H0, W0 = G[0].shape; vw = cv2.VideoWriter(PNG.replace("_alt.png", "_ann.mp4"), cv2.VideoWriter_fourcc(*"mp4v"), 10, (W0, H0))
cap = cv2.VideoCapture(os.path.join(HERE, "flights", "video", NAME + ".mp4")); hmax = max(10.0, float(np.nanmax(h)))
for i in range(len(G)):
    ok, fr = cap.read()
    if not ok: break
    if not np.isnan(yb[i]):
        cv2.line(fr, (X0 - 60, int(yb[i])), (X1 + 60, int(yb[i])), (0, 220, 255), 2)
    cv2.line(fr, (X0 - 60, int(y0)), (X1 + 60, int(y0)), (120, 120, 120), 1)
    cv2.rectangle(fr, (20, 20), (470, 175), (0, 0, 0), -1)
    for k2, s in enumerate(("t %+.2f s" % ft[i], "height %5.1f mm" % (0 if np.isnan(h[i]) else h[i]), "throttle %4.1f %%" % thr_v[i],
                            "battery %.2f V" % vb_v[i], "roll %+.1f  pitch %+.1f" % (ro_v[i], pi_v[i]))):
        cv2.putText(fr, s, (32, 50 + 28 * k2), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2)
    bh = int(0 if np.isnan(h[i]) else 400 * h[i] / hmax)
    cv2.rectangle(fr, (500, 600 - 400), (530, 600), (80, 80, 80), 2); cv2.rectangle(fr, (500, 600 - bh), (530, 600), (0, 220, 255), -1)
    vw.write(fr)
vw.release()
i_lo, i_hi = int(np.nanargmin(h)), int(np.nanargmax(h))
cx = (X0 + X1) // 2; x0c = max(0, cx - 160)
cap = cv2.VideoCapture(os.path.join(HERE, "flights", "video", NAME + ".mp4")); fr2 = []
for i in range(len(G)):
    ok, fr = cap.read()
    if i in (i_lo, i_hi): fr2.append(fr[0:H0, x0c:x0c + 320].copy())
if len(fr2) == 2: cv2.imwrite(PNG.replace("_alt.png", "_range.jpg"), np.hstack(fr2))
on = thr > 5
print("最高 %.0f mm；电池最低 %.2f V；姿态 ±%.1f°/±%.1f°（油门>5%%）" % (np.nanmax(h), vb.min(), np.std(ro[on]) if on.any() else 0, np.std(pi[on]) if on.any() else 0))
np.savetxt(PNG.replace(".png", ".csv"), np.c_[ft, h], delimiter=",", header="t,h_mm", comments="", fmt="%.3f")
print("saved", PNG)
