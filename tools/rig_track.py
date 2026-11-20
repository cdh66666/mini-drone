"""定高架逐帧（30 Hz）追踪 + 动力学分析。

用法：python -I tools/rig_track.py <记录名，如 20261120_172249> [k=v ...]
画面：相机横放，塔在画面里水平，“上” = 画面向右（UPX=1）。比例：塔长 394 mm 对应画面像素（MMPX，默认 0.63）。
追踪：
  ① 飞机：解锁前那一帧在 BAND 条带里截模板，之后每帧沿 x 方向匹配（只允许水平移动）→ 飞机高度
  ② 黄色滑块（导杆底部限位环）：HSV 黄色、在塔槽附近的最大色块 → 滑块高度（飞起来才露出来）
对齐：视频 _t.csv（相对解锁 s）、整次飞行 CSV（25 Hz：油门、电压、电机）、黑匣子 .lb（1 kHz：加计 az、电机）。
动力学：只取在两端限位之间“自由滑动”的时段，拟合  a/g = K·eff − 1 − F·sign(v)
  eff = 平均输出 × 电压 / 3.95 V（‰），K = 推力系数，F = 摩擦（占运动部分重量的比例）
输出：flights/rig/<名>_track.png（高度/速度/加速度/推力）、_track.csv、_trk.mp4（逐帧标注录像）
"""
import csv, json, os, sys
import numpy as np, cv2
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
plt.rcParams["font.sans-serif"] = ["Microsoft YaHei"]; plt.rcParams["axes.unicode_minus"] = False
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import lb_read
def savgol_filter(y, w, p, deriv=0, delta=1.0):
    """Savitzky-Golay（不依赖 scipy）：每个点用前后 w 个点拟合 p 次多项式，取该点的值/导数"""
    h = w // 2; x = np.arange(-h, h + 1, dtype=float)
    A = np.vander(x, p + 1, increasing=True); C = np.linalg.pinv(A)[deriv] * __import__("math").factorial(deriv) / delta ** deriv
    yp = np.r_[np.full(h, y[0]), y, np.full(h, y[-1])]
    return np.convolve(yp, C[::-1], "valid")

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
NAME = sys.argv[1]
KV = dict(a.split("=", 1) for a in sys.argv[2:] if "=" in a)
MMPX = float(KV.get("MMPX", 0.63))
BAND = [int(v) for v in KV.get("BAND", "330,500").split(",")]      # 飞机所在的 y 条带
XR = [int(v) for v in KV.get("XR", "640,1100").split(",")]         # 飞机可能出现的 x 范围
BOX = [int(v) for v in KV.get("BOX", "725,370,815,430").split(",")]  # 解锁前飞机模板框 x0,y0,x1,y1
SLOT = [int(v) for v in KV.get("SLOT", "40,380,700,520").split(",")]  # 黄色滑块搜索区 x0,y0,x1,y1
OUT = os.path.join(HERE, "flights", "rig")

cap = cv2.VideoCapture(os.path.join(HERE, "flights", "video", NAME + ".mp4"))
ft = np.array([float(r["t"]) for r in csv.DictReader(open(os.path.join(HERE, "flights", "video", NAME + "_t.csv")))])
frames = []
while True:
    ok, f = cap.read()
    if not ok: break
    frames.append(f)
ft = ft[:len(frames)]
g0 = cv2.cvtColor(frames[0], cv2.COLOR_BGR2GRAY)
tpl = g0[BOX[1]:BOX[3], BOX[0]:BOX[2]].astype(np.float32)
dx, sc, ys, ysx = [], [], [], []
def yblobs(f):
    hsv = cv2.cvtColor(f[SLOT[1]:SLOT[3], SLOT[0]:SLOT[2]], cv2.COLOR_BGR2HSV)
    n, lab, st, cen = cv2.connectedComponentsWithStats(cv2.inRange(hsv, (10, 40, 120), (45, 255, 255)))
    return [(SLOT[0] + cen[i][0], st[i, 4]) for i in range(1, n) if st[i, 4] >= 15]
# 解锁前就在的黄色（不动的东西，或者躺在底部的滑块）记下来，之后在它附近的色块不算
STATIC = [x for x, a in yblobs(frames[0])] + [x for x, a in yblobs(frames[min(5, len(frames) - 1)])]
for f in frames:
    g = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY).astype(np.float32)
    strip = g[BOX[1] - 6:BOX[3] + 6, XR[0]:XR[1]]
    m = cv2.matchTemplate(strip, tpl, cv2.TM_CCOEFF_NORMED); _, s, _, loc = cv2.minMaxLoc(m)
    dx.append(XR[0] + loc[0] - BOX[0]); sc.append(s)
    bl = [(x, a) for x, a in yblobs(f) if all(abs(x - s) > 12 for s in STATIC)]
    ys.append(max(bl, key=lambda b: b[1])[0] if bl else np.nan)
dx = np.array(dx, float); sc = np.array(sc); ysl = np.array(ys, float)
zd = dx * MMPX                                  # 飞机（模板）离下限位 mm（向右 = 向上）
zd[sc < 0.5] = np.nan
# 黄色滑块和飞机刚性相连：用两者都可信的低处那段对出偏移，之后以滑块为准（背景干净，顶部不丢）
zs = ysl * MMPX
both = np.isfinite(zs) & np.isfinite(zd) & (zd > 3) & (zd < 80)
off = float(np.median(zd[both] - zs[both])) if both.sum() > 5 else np.nan
zs = zs + off if np.isfinite(off) else np.full_like(zs, np.nan)
z = np.where(np.isfinite(zs), zs, zd)
z[np.isfinite(zd) & (zd < 2) & ~np.isfinite(zs)] = 0
# 平滑求导（30 Hz）
zi = z.copy(); ok = np.isfinite(zi); zi[~ok] = np.interp(np.flatnonzero(~ok), np.flatnonzero(ok), zi[ok]) if ok.any() else 0
dt = np.median(np.diff(ft)); W = 9
zf = savgol_filter(zi, W, 2); v = savgol_filter(zi, W, 2, deriv=1, delta=dt); a = savgol_filter(zi, 11, 2, deriv=2, delta=dt)

R = list(csv.DictReader(open(os.path.join(HERE, "flights", "full", NAME + ".csv"))))
tc = np.array([float(r["t"]) for r in R]); thr = np.array([float(r["thr_cmd"]) for r in R]) / 10
vb = np.array([float(r["vb_mv"]) for r in R]) / 1000
oc = np.array([[float(r["o%d" % i]) for i in range(1, 5)] for r in R]).mean(1)
lbp = os.path.join(OUT, NAME + ".lb"); azv = None
if os.path.exists(lbp):
    h, L = lb_read.parse(open(lbp, "rb").read())
    tl = (L["us"] - L["us"][0]) / 1e6; az = L["ac"][:, 2] / 1000.0; ol = L["o"].mean(1); tl_thr = L["thr"]
    # 对齐：黑匣子第一次 thr>0 ↔ CSV 第一次 thr>0
    t0l = tl[np.argmax(tl_thr > 0)]; t0c = tc[np.argmax(thr > 0)]; tl = tl - t0l + t0c
    az0 = np.median(az[tl < t0c - 0.05]) if (tl < t0c - 0.05).any() else 1.0
    k = 33; azs = np.convolve(az / az0, np.ones(k) / k, "same"); ols = np.convolve(ol, np.ones(k) / k, "same")
    azv = np.interp(ft, tl, azs); ov = np.interp(ft, tl, ols)
else:
    ov = np.interp(ft, tc, oc)
vbv = np.interp(ft, tc, vb); thv = np.interp(ft, tc, thr)
eff = ov * vbv / 3.95                                     # ‰（3.95 V 等效）
# 自由滑动：离两端限位 >4 mm、速度 >8 mm/s、匹配可信
zmax = np.nanpercentile(z, 99) if np.isfinite(z).any() else 0
free = (zf > 4) & (zf < zmax - 4) & (np.abs(v) > 8) & (sc > 0.5) & (thv > 20)
res = {"frames": len(frames), "fps": round(1 / dt, 1), "travel_mm": round(float(zmax), 1)}
if free.sum() > 15:
    A = np.c_[eff[free], -np.sign(v[free]), np.ones(free.sum())]
    y = a[free] / 9810.0 + 1.0                                # a 是 mm/s² → g
    sol, *_ = np.linalg.lstsq(A[:, :2], y, rcond=None)        # y = K·eff − F·sign(v)
    K, F = sol; res.update(K=float(K), F_pct=round(100 * float(F), 1), hover_eff=round(1 / K, 0), n=int(free.sum()),
                           resid_g=round(float(np.std(y - A[:, :2] @ sol)), 3))
    up = free & (v > 0); dn = free & (v < 0)
    res["up_n"], res["dn_n"] = int(up.sum()), int(dn.sum())
# 离地 / 落回 时的推力
on = np.flatnonzero(zf > 3)
if len(on):
    i0 = on[0]; res["lift_eff"] = round(float(eff[i0]), 0); res["lift_thr"] = round(float(thv[i0]), 1); res["lift_t"] = round(float(ft[i0]), 2)
    i1 = on[-1]; res["drop_eff"] = round(float(eff[i1]), 0); res["drop_thr"] = round(float(thv[i1]), 1); res["drop_t"] = round(float(ft[i1]), 2)
print(json.dumps(res, ensure_ascii=False))

fig, ax = plt.subplots(4, 1, figsize=(12, 10), sharex=True, gridspec_kw={"height_ratios": [1.4, 1, 1, 1]})
ax[0].plot(ft, zd, ".", ms=2, color="#8a97a6", label="飞机模板匹配")
ax[0].plot(ft, z, ".", ms=3, color="#0b7a83", label="高度（滑块为主，30 Hz）"); ax[0].plot(ft, zf, "-", lw=1, color="#0b7a83")
if np.isfinite(zs).any(): ax[0].plot(ft, zs, ".", ms=3, color="#d4a017", label="黄色滑块")
ax[0].scatter(ft[free], zf[free], s=8, color="#c2412d", label="自由滑动段（拟合用）", zorder=3)
ax[0].set_ylabel("高度 mm"); ax[0].grid(alpha=.3); ax[0].legend(fontsize=8, loc="upper left")
ax[1].plot(ft, v, color="#5b4bb7"); ax[1].set_ylabel("速度 mm/s"); ax[1].grid(alpha=.3)
ax[2].plot(ft, a / 9810, color="#c2412d", label="相机算的加速度 g");
if azv is not None: ax[2].plot(ft, azv - 1, color="#3cc2cc", lw=1, label="机载加计 az−1 g")
if "K" in res: ax[2].plot(ft, res["K"] * eff - 1, color="#8a97a6", lw=1, ls="--", label="模型 K·eff−1（不含摩擦）")
ax[2].set_ylim(-1.2, 1.2); ax[2].set_ylabel("g"); ax[2].grid(alpha=.3); ax[2].legend(fontsize=8, loc="upper left")
ax[3].plot(ft, thv, color="#3b9eff", label="油门杆 %"); ax[3].plot(ft, ov / 10, color="#f0b429", label="电机平均输出 %")
ax[3].plot(ft, eff / 10, color="#2f855a", label="等效推力 eff/10（按 3.95 V 折算）"); ax[3].plot(ft, (vbv - 3) * 100, color="#8a97a6", label="电池（3~4 V）")
ax[3].set_ylim(0, 105); ax[3].grid(alpha=.3); ax[3].legend(fontsize=8, loc="upper left"); ax[3].set_xlabel("解锁后 s")
ttl = "定高架逐帧分析 " + NAME
if "K" in res: ttl += "   悬停等效推力 %.0f‰ · 摩擦 ±%.1f%% · 拟合残差 %.3f g" % (res["hover_eff"], abs(res["F_pct"]), res["resid_g"])
fig.suptitle(ttl); fig.tight_layout(); fig.savefig(os.path.join(OUT, NAME + "_track.png"), dpi=90)
np.savetxt(os.path.join(OUT, NAME + "_track.csv"), np.c_[ft, z, zf, v, a, zs, thv, ov, vbv, eff, (azv if azv is not None else np.full_like(ft, np.nan))],
           delimiter=",", header="t,z_mm,z_f,v_mms,a_mms2,slider_mm,thr,out,vb,eff,az", comments="", fmt="%.4f")
# 标注录像
H0, W0 = frames[0].shape[:2]; vw = cv2.VideoWriter(os.path.join(OUT, NAME + "_trk.mp4"), cv2.VideoWriter_fourcc(*"mp4v"), round(1 / dt), (W0, H0))
for i, f in enumerate(frames):
    f = f.copy(); x = int(BOX[0] + dx[i])
    cv2.rectangle(f, (x, BOX[1]), (x + BOX[2] - BOX[0], BOX[3]), (0, 220, 255) if sc[i] > 0.5 else (0, 0, 255), 2)
    if np.isfinite(ysl[i]): cv2.circle(f, (int(ysl[i]), (SLOT[1] + SLOT[3]) // 2), 10, (0, 255, 255), 2)
    cv2.rectangle(f, (10, 10), (520, 190), (0, 0, 0), -1)
    for k2, s in enumerate(("t %+.2f s   %s" % (ft[i], "FREE" if free[i] else ""), "height %6.1f mm" % zf[i], "vel %+6.0f mm/s  acc %+.2f g" % (v[i], a[i] / 9810),
                            "thr %4.1f%%  out %4.1f%%" % (thv[i], ov[i] / 10), "batt %.2f V  eff %.0f" % (vbv[i], eff[i]))):
        cv2.putText(f, s, (22, 42 + 34 * k2), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
    vw.write(f)
vw.release()
print("saved", os.path.join(OUT, NAME + "_track.png"))
