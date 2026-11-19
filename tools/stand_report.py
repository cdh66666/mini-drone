"""台架对比图：用长黑匣子（.lb，1 kHz）比较几组参数。

用法：python -I tools/stand_report.py 输出.png 标签1 标签2 ...（标签 = stand_test 的 TAG，取最新那个 .lb）
图：上 = 横滚/俯仰阶跃（6°，叠在一起比），中 = 平稳段姿态，下 = 振动频谱（原始 vs 滤波后）。
指标（只看平稳段：油门到位、没打杆、起步扶正后 0.8 s 起）：姿态晃动 std、滤波后陀螺 rms、阶跃上升/过冲。
"""
import glob, os, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import lb_read

plt.rcParams["font.sans-serif"] = ["Microsoft YaHei"]; plt.rcParams["axes.unicode_minus"] = False
HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
out, tags = sys.argv[1], sys.argv[2:]
COL = ["#0b7a83", "#c2412d", "#b7791f", "#5b4bb7", "#2f855a", "#8a97a6"]


def load(tag):
    fs = sorted(glob.glob(os.path.join(HERE, "flights", "stand", "*_" + tag + ".lb")))
    if not fs: return None
    return lb_read.parse(open(fs[-1], "rb").read())[1]


def metrics(r):
    t = (r["us"] - r["us"][0]) / 1e6
    thr = r["thr"]; top = np.percentile(thr, 90)
    st = (np.abs(r["stR"]) > 5) | (np.abs(r["stP"]) > 5)
    first = np.argmax(st) if st.any() else len(r)
    on = np.where(thr >= 0.9 * top)[0]
    if not len(on): return None
    t_on = t[on[0]]
    idx = np.arange(len(r)); calm = (idx < first) & (idx >= first - 1000) & (thr >= 300)   # 阶跃前最后 1 秒（起步扶正那段不算）
    m = {"t": t}
    m["calm"] = calm
    m["roll_std"] = float(np.std(r["roll"][calm] / 10)) if calm.sum() > 50 else np.nan
    m["pitch_std"] = float(np.std(r["pitch"][calm] / 10)) if calm.sum() > 50 else np.nan
    gf = r["gf"][calm].astype(float) / 10
    m["gyro_rms"] = float(np.sqrt(np.mean((gf - gf.mean(0)) ** 2))) if calm.sum() > 50 else np.nan
    steps = {}
    for ax, sk in (("roll", "stR"), ("pitch", "stP")):
        s = r[sk].astype(float) / 100
        i = np.where((s[1:] > 0.1) & (s[:-1] <= 0.1))[0]
        if not len(i): continue
        i0 = i[0] + 1; base = r[ax][max(0, i0 - 100):i0].mean() / 10; tgt = s[i0] * 25
        y = r[ax][i0:i0 + 700] / 10 - base; tt = np.arange(len(y)) / 1000.0
        k90 = np.where(y >= 0.9 * tgt)[0]
        steps[ax] = {"t": tt, "y": y / tgt, "rise": (tt[k90[0]] * 1000 if len(k90) else np.nan), "over": (y.max() - tgt) / tgt * 100}
    m["steps"] = steps
    return m


data = [(tg, load(tg)) for tg in tags]
data = [(tg, r, metrics(r)) for tg, r in data if r is not None]
fig = plt.figure(figsize=(12, 9))
gs = fig.add_gridspec(3, 2, height_ratios=[1.1, 1, 1.1])
for j, ax in enumerate(("roll", "pitch")):
    a = fig.add_subplot(gs[0, j])
    for k, (tg, r, m) in enumerate(data):
        if m and ax in m["steps"]:
            s = m["steps"][ax]; a.plot(s["t"] * 1000, s["y"], color=COL[k % len(COL)], lw=1.6,
                                        label="%s  上升 %.0f ms · 过冲 %.0f%%" % (tg, s["rise"], s["over"]))
    a.axhline(1, color="#8a97a6", ls="--", lw=1); a.set_ylim(-0.3, 2.2); a.set_xlabel("ms"); a.set_ylabel("实际 / 目标")
    a.set_title(("横滚" if ax == "roll" else "俯仰") + " 6° 阶跃（1 = 刚好到目标）"); a.grid(alpha=.3); a.legend(fontsize=8, frameon=False)
a = fig.add_subplot(gs[1, :])
for k, (tg, r, m) in enumerate(data):
    if not m: continue
    c = m["calm"]; t = m["t"][c]
    if len(t) < 50: continue
    a.plot(t - t[0], r["roll"][c] / 10, color=COL[k % len(COL)], lw=1.2, label="%s  横滚 ±%.2f° 俯仰 ±%.2f° 陀螺 %.1f dps" % (tg, m["roll_std"], m["pitch_std"], m["gyro_rms"]))
    a.plot(t - t[0], r["pitch"][c] / 10, color=COL[k % len(COL)], lw=1.0, ls=":")
a.set_title("平稳段姿态（实线横滚、点线俯仰；没打杆）"); a.set_xlabel("s"); a.set_ylabel("°"); a.grid(alpha=.3); a.legend(fontsize=8, frameon=False)
for j, key in enumerate(("gr", "gf")):
    a = fig.add_subplot(gs[2, j])
    for k, (tg, r, m) in enumerate(data):
        sp = lb_read.spectrum(r)
        if not sp: continue
        f = np.array(sp["f"]); amp = np.array(sp[key + "x"]) + np.array(sp[key + "y"])
        a.plot(f, amp, color=COL[k % len(COL)], lw=1.1, label=tg)
    a.set_xlim(0, 300); a.set_xlabel("Hz"); a.set_ylabel("幅度 dps"); a.grid(alpha=.3); a.legend(fontsize=8, frameon=False)
    a.set_title("振动频谱 · " + ("原始陀螺（滤波前）" if key == "gr" else "滤波后（控制用的）"))
fig.tight_layout(); fig.savefig(out, dpi=100)
for tg, r, m in data:
    if m:
        print(tg, "calm roll±%.2f pitch±%.2f gyro %.1f" % (m["roll_std"], m["pitch_std"], m["gyro_rms"]),
              " ".join("%s rise %.0fms over %.0f%%" % (k, v["rise"], v["over"]) for k, v in m["steps"].items()))
