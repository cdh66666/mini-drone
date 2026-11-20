"""姿态（欧拉角）目标 vs 实际：横滚、俯仰用 1 kHz 黑匣子（杆量×25° = 目标角），偏航用整次飞行 CSV（杆量×120 °/s = 目标角速度）。

用法：python -I tools/att_plot.py <记录名>   （需要 flights/rig/<名>.lb 和 flights/full/<名>.csv）
每个阶跃算：上升时间（10→90%）、过冲、稳态误差（阶跃后 0.4~0.9 s 的平均）。
"""
import csv, json, os, sys
import numpy as np
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
plt.rcParams["font.sans-serif"] = ["Microsoft YaHei"]; plt.rcParams["axes.unicode_minus"] = False
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import lb_read
HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__))); NAME = sys.argv[1]
h, L = lb_read.parse(open(os.path.join(HERE, "flights", "rig", NAME + ".lb"), "rb").read())
t = (L["us"] - L["us"][0]) / 1e6; on = L["thr"] > 0; t0 = t[np.argmax(on)]; t = t - t0
R = list(csv.DictReader(open(os.path.join(HERE, "flights", "full", NAME + ".csv"))))
tc = np.array([float(r["t"]) for r in R]); tc = tc - tc[np.argmax(np.array([float(r["thr_cmd"]) for r in R]) > 0)]
gy = np.array([float(r["gy"]) for r in R]); sy = np.array([float(r.get("stick_y") or 0) for r in R]) * 120

def steps(tt, tg, ac):
    out = []; d = np.diff(tg); idx = np.flatnonzero(np.abs(d) > 0.5 * max(1e-6, np.abs(tg).max()))
    for i in idx:
        a0, a1 = tg[i], tg[i + 1]; st = a1 - a0
        if abs(st) < 1: continue
        m = (tt > tt[i + 1]) & (tt < tt[i + 1] + 0.9); y = ac[m] - ac[i]; x = tt[m] - tt[i + 1]
        if len(y) < 10: continue
        yn = y / st
        r10 = x[np.argmax(yn >= 0.1)] if (yn >= 0.1).any() else np.nan; r90 = x[np.argmax(yn >= 0.9)] if (yn >= 0.9).any() else np.nan
        ss = np.mean(yn[(x > 0.4) & (x < 0.9)]) if ((x > 0.4) & (x < 0.9)).any() else np.nan
        out.append({"t": round(float(tt[i + 1]), 2), "step": round(float(st), 1), "rise_ms": round(1000 * (r90 - r10), 0) if np.isfinite(r90) else None,
                    "over_pct": round(100 * (yn.max() - 1), 0), "final_pct": round(100 * ss, 0)})
    return out

res = {}
fig, ax = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
for k, (name, tk, ak, col) in enumerate((("横滚", "stR", "roll", "#3cc2cc"), ("俯仰", "stP", "pitch", "#ef7a64"))):
    tg = L[tk].astype(float) / 100 * 25; ac = L[ak].astype(float) / 10
    ax[k].plot(t, tg, color="#8a97a6", lw=1.5, ls="--", label="目标 " + name + "（杆量×25°）"); ax[k].plot(t, ac, color=col, lw=1.2, label="实际")
    res[ak] = steps(t, tg, ac)
    s = "  ".join("%+.0f°: 上升 %s ms 过冲 %s%% 稳态 %s%%" % (q["step"], q["rise_ms"], q["over_pct"], q["final_pct"]) for q in res[ak][:4])
    ax[k].set_title(name + "   " + s, fontsize=9, loc="left"); ax[k].set_ylabel("°"); ax[k].grid(alpha=.3); ax[k].legend(fontsize=8, loc="upper right")
ax[2].plot(tc, sy, color="#8a97a6", lw=1.5, ls="--", label="目标偏航角速度（杆量×120 °/s）"); ax[2].plot(tc, gy, color="#5b4bb7", lw=1.2, label="实际（陀螺 z）")
res["yaw"] = steps(tc, sy, gy)
s = "  ".join("%+.0f°/s: 上升 %s ms 稳态 %s%%" % (q["step"], q["rise_ms"], q["final_pct"]) for q in res["yaw"][:4])
ax[2].set_title("偏航   " + s, fontsize=9, loc="left"); ax[2].set_ylabel("°/s"); ax[2].grid(alpha=.3); ax[2].legend(fontsize=8, loc="upper right"); ax[2].set_xlabel("解锁后 s")
lo, hi = -0.5, max(t[on].max() - t0 if on.any() else 10, tc.max())
for a in ax: a.set_xlim(lo, hi)
fig.suptitle("姿态 目标 vs 实际  " + NAME); fig.tight_layout()
out = os.path.join(HERE, "flights", "rig", NAME + "_att.png"); fig.savefig(out, dpi=90)
print(json.dumps(res, ensure_ascii=False)); print("saved", out)
