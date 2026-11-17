"""侧面剖视示意图（y=0 剖面）：python side_view.py <输出.png>"""
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import trimesh
from matplotlib.patches import Polygon, Arc

plt.rcParams["font.sans-serif"] = ["Microsoft YaHei"]
plt.rcParams["axes.unicode_minus"] = False
ASM = Path(__file__).resolve().parent / "stl" / "assembly"
PIVOT_H = 100.0
CG_ABOVE = 18.0   # 估算：电机座 16 g、电机 10 g、电池 8 g、PCB 8 g… 见 README

COL = {"base": "#2f6db5", "post": "#2f6db5", "adapter": "#f07a1a", "lock": "#8c4fcc",
       "pcb": "#0c4a22", "battery": "#b9bcc4", "mount": "#9a9a96", "prop": "#333"}

fig, ax = plt.subplots(figsize=(11, 8.2))
for f in sorted(ASM.glob("asm_*.stl")):
    key = f.stem[4:]
    kind = next(k for k in COL if key.startswith(k))
    m = trimesh.load(f)
    for y in (0.0, 34.4) if kind in ("mount", "prop") else (0.0,):
        # 电机座在对角线上，取 y=34.4 那排（x=±34.4），投影到同一张图
        s = m.section(plane_origin=[0, y, 0], plane_normal=[0, 1, 0])
        if s is None:
            continue
        for e in s.discrete:
            ax.add_patch(Polygon(e[:, [0, 2]], closed=True, fc=COL[kind], ec="k", lw=0.4,
                                 alpha=0.35 if kind == "prop" else 0.9))
# 转动中心、重心
ax.plot(0, PIVOT_H, "o", ms=9, mfc="white", mec="red", mew=2, zorder=10)
ax.annotate("转动中心（球心）\n离桌面 100 mm", (0, PIVOT_H), (-115, 95), fontsize=11,
            arrowprops=dict(arrowstyle="->", color="red"), color="red")
ax.plot(0, PIVOT_H + CG_ABOVE, "P", ms=12, color="#c00", zorder=10)
ax.annotate(f"整机重心（估）\n球心上方约 {CG_ABOVE:.0f} mm", (0, PIVOT_H + CG_ABOVE), (-115, 150),
            fontsize=11, arrowprops=dict(arrowstyle="->", color="#c00"), color="#c00")
# ±30° 限位
for sg in (1, -1):
    a = np.radians(90 + sg * 30)
    ax.plot([0, 80 * np.cos(a)], [PIVOT_H, PIVOT_H + 80 * np.sin(a)], "--", color="red", lw=1)
ax.add_patch(Arc((0, PIVOT_H), 90, 90, theta1=60, theta2=120, color="red", lw=1.2))
ax.text(0, PIVOT_H + 50, "±30° 限位\n(球窝下沿碰球颈)", ha="center", fontsize=10, color="red")
ax.annotate("单轴卡扣：导槽夹住\n托板下的导向鳍", (24, PIVOT_H - 10), (70, 55), fontsize=10,
            arrowprops=dict(arrowstyle="->"), color="#6a2fa8")
ax.annotate("托板 + 绑带孔\n（皮筋/扎带穿过 PCB 两条槽）", (12, PIVOT_H + 9), (55, 165), fontsize=10,
            arrowprops=dict(arrowstyle="->"), color="#c25a00")
ax.annotate("压重物的坑 ×4\n胶带耳 ×4", (52, 4), (85, 30), fontsize=10,
            arrowprops=dict(arrowstyle="->"), color="#2f6db5")
ax.axhline(0, color="k", lw=1.2)
ax.set_aspect("equal")
ax.set_xlim(-130, 130)
ax.set_ylim(-5, 185)
ax.set_xlabel("mm（剖面 y=0；电机座/桨取对角那排投影）")
ax.set_title("mini-drone 调参测试架 · 侧面剖视", fontsize=14)
ax.grid(alpha=0.25)
plt.tight_layout()
plt.savefig(sys.argv[1], dpi=110)
