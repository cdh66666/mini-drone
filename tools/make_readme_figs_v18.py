"""README 用图（v18）：最好的一次飞行、台架调参过程、推力曲线。输出到 docs/img/。"""
import csv, json, os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

plt.rcParams["font.sans-serif"] = ["Microsoft YaHei"]; plt.rcParams["axes.unicode_minus"] = False
HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
IMG = os.path.join(HERE, "docs", "img")
C1, C2, C3, GR = "#0b7a83", "#c2412d", "#b7791f", "#8a97a6"

# 1) 11-18 15:51 那次 25 秒飞行：杆量目标 vs 实际姿态
R = list(csv.DictReader(open(os.path.join(HERE, "flights", "full", "20261118_155152.csv"))))
f = lambda k: np.array([float(r[k]) for r in R])
t, cmd, sr, sp, ro, pi, vb = [f(k) for k in ("t", "thr_cmd", "stick_r", "stick_p", "roll", "pitch", "vb_mv")]
o = np.mean([f("o%d" % i) for i in range(1, 5)], axis=0)
m = (cmd > 0) & (t < t[cmd > 0][-1] - 0.7)          # 去掉最后撞到东西那一下
t0 = t[m][0]; tt = t[m] - t0
fig, ax = plt.subplots(3, 1, figsize=(10, 6.4), sharex=True, gridspec_kw={"height_ratios": [2, 2, 1.2]})
ax[0].plot(tt, sr[m] * 25, color=GR, lw=1.2, label="横滚目标（杆量×25°）"); ax[0].plot(tt, ro[m], color=C1, lw=1.4, label="实际横滚")
ax[1].plot(tt, sp[m] * 25, color=GR, lw=1.2, label="俯仰目标"); ax[1].plot(tt, pi[m], color=C2, lw=1.4, label="实际俯仰")
ax[2].plot(tt, o[m] / 10, color=C3, lw=1.2, label="四电机平均输出 %"); ax2 = ax[2].twinx(); ax2.plot(tt, vb[m] / 1000, color=GR, lw=1, label="电池 V")
ax2.set_ylabel("电池 V", color=GR)
for a, y in zip(ax, ("横滚 °", "俯仰 °", "输出 %")):
    a.set_ylabel(y); a.grid(alpha=.3); a.legend(loc="upper left", fontsize=9, frameon=False)
ax[2].set_xlabel("秒"); ax[0].set_title("11-18 最好的一次：空中 25 秒，姿态跟着杆走（v18，P3.5/D0.08/I1.0/角度P6）", fontsize=12)
fig.tight_layout(); fig.savefig(os.path.join(IMG, "flight_v18_best.png"), dpi=110); plt.close(fig)

# 2) 台架调参：每轮倾角均方根
rows = [json.loads(l) for l in open(os.path.join(HERE, "flights", "stand", "summary.csv"), encoding="utf-8") if l.strip()]
rows = [r for r in rows if r.get("tilt_rms", -1) > 0 and r["tag"] not in ("N_a7_i1",)]
lab = {"rec_p05": "P0.5", "p0.8": "P0.8", "p1.2": "P1.2", "p1.6": "P1.6\nD不滤", "A_p1.6_d30": "P1.6\nD滤30", "B_p2.0_d30": "P2.0",
       "C_p2.0_dlo": "P2.0\nD小", "D_p2.5_d30": "P2.5", "E_p2.5_a3": "P2.5\n角P3", "F_p2.5_dhi": "P2.5\nD大", "G_p2.0_f60": "P2.0\nD滤60",
       "H_p3.0": "P3.0", "I_p3_d09": "P3.0\nD0.09", "J_p3_i1": "P3.0\nI1.0", "K_p3.5_d08": "P3.5", "L_a5_i1": "P3.5\n角P5",
       "M_a6_i1": "P3.5\n角P6", "M2_rep": "复测", "M3_rep": "复测", "O_f60": "D滤60", "P_d10": "D0.1", "Q_thr65a": "65%油门", "Q_thr65b": "65%油门"}
x = np.arange(len(rows)); v = [r["tilt_rms"] for r in rows]
col = [C2 if r.get("lim%", 0) > 20 else (C1 if r["tag"].startswith(("M", "Q")) else GR) for r in rows]
fig, a = plt.subplots(figsize=(11, 4))
a.bar(x, v, color=col); a.set_xticks(x); a.set_xticklabels([lab.get(r["tag"], r["tag"]) for r in rows], fontsize=8)
a.set_ylabel("倾角均方根 °（越小越稳）"); a.grid(axis="y", alpha=.3)
a.set_title("测试架上自动调参：红 = 贴着限位来回甩，青 = 最终选的那组（含复测）", fontsize=12)
for i, val in enumerate(v): a.text(i, val + .3, "%.1f" % val, ha="center", fontsize=8)
fig.tight_layout(); fig.savefig(os.path.join(IMG, "stand_tuning.png"), dpi=110); plt.close(fig)

# 3) 推力曲线
T = [r for r in csv.DictReader(open(os.path.join(HERE, "flights", "thrust_log.csv"))) if not (r["pm"] == "800" and r["thrust_g"] == "38")]
d = np.array([float(r["pm"]) / 1000 for r in T]); V = np.array([float(r["vb_mv"]) / 1000 for r in T]); g = np.array([float(r["thrust_g"]) for r in T])
u = d * V; k = float(np.linalg.lstsq(np.c_[u], g, rcond=None)[0][0])
fig, a = plt.subplots(1, 2, figsize=(10, 4))
sc = a[0].scatter(d * 100, g, c=V, cmap="viridis", s=40); fig.colorbar(sc, ax=a[0], label="当时电池电压 V")
a[0].axhline(49.3, color=C2, ls="--", lw=1); a[0].text(21, 50.5, "机重 49.3 g", color=C2, fontsize=9)
a[0].set_xlabel("占空比 %"); a[0].set_ylabel("推力 g"); a[0].set_title("占空比 vs 推力（电压越低越弱）", fontsize=11); a[0].grid(alpha=.3)
xx = np.linspace(0, 3.2, 10)
a[1].scatter(u, g, color=C1, s=36); a[1].plot(xx, k * xx, color=C3, lw=1.5, label="推力 ≈ %.1f g/V × (占空比×电压)" % k)
a[1].axhline(49.3, color=C2, ls="--", lw=1)
a[1].set_xlabel("占空比 × 电池电压 (V)"); a[1].set_ylabel("推力 g"); a[1].set_title("换成“电机电压”就是一条直线", fontsize=11); a[1].legend(fontsize=9, frameon=False); a[1].grid(alpha=.3)
fig.tight_layout(); fig.savefig(os.path.join(IMG, "thrust_curve.png"), dpi=110); plt.close(fig)
print("ok", k)
