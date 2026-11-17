"""推力测试加高柱：飞机倒过来（桨朝下、电池朝上）架在厨房秤上。
python thrust_post.py   → stl/thrust_post.stl

坐标：z=0 是秤盘，X/Y 是 PCB 坐标（以板中心为原点，Y 负 = 机头，KiCad v5 工程量的）。
飞机倒置时朝下的是 PCB 正面：
  · ESP32-S3-WROOM-1 模组 18×25.5×3.1，在 X ±9、Y -26.4~-0.9（模组偏机头，天线伸出板外）
  · 两条长槽 X ±11.5~13.5、Y -3~+3（背面绑电池用的那两条）
托法：模组压在口袋底上（主承力），两根小销插进两条长槽定位、肩台托住槽边防后仰。
"""
import sys
from pathlib import Path

import cadquery as cq

sys.stdout.reconfigure(encoding="utf-8")
HERE = Path(__file__).resolve().parent

GUARD_ABOVE_PCB = 15.25   # 保护圈顶比 PCB 正面高（电机座 STEP 量的）
LOW_GAP = 40.0            # 倒置后最低点（保护圈顶）离秤盘
MOD_H = 3.1
Z_T = LOW_GAP + GUARD_ABOVE_PCB      # PCB 正面高度
Z_F = Z_T - MOD_H                    # 模组口袋底
Z_S = Z_T - 0.2                      # 肩台顶（比 PCB 低 0.2，主要靠模组承力）
PLATE_T = 2.5
COL_A, COL_Y = 18.0, -6.0            # 柱子方截面边长、中心 Y
COL_TOP = Z_F - PLATE_T - 11.0
BASE_D, BASE_T = 40.0, 3.0

MOD_X, MOD_Y0, MOD_Y1 = 9.0, -26.4, -0.9
CLR = 0.4
WALL_H = 1.8                         # 口袋围边高（离 PCB 还有 1.3 mm，避开小元件）
FRONT = -16.0                        # 前端只到这里：离前面两个电机座的辐条 3.4 mm
SLOT_X0, SLOT_X1, SLOT_Y = 11.5, 13.5, 3.0


def build():
    # 底座：平底 Ø40，底面十字浅槽让热熔胶咬住
    base = cq.Workplane("XY").circle(BASE_D / 2).extrude(BASE_T).edges(">Z").fillet(1.0)
    for ang in (0, 45, 90, 135):
        base = base.cut(cq.Workplane("XY").rect(BASE_D + 2, 1.6).extrude(0.8)
                        .rotate((0, 0, 0), (0, 0, 1), ang))
    # 柱子（方截面倒角）+ 4 片肋
    col = (cq.Workplane("XY").center(0, COL_Y).rect(COL_A, COL_A).extrude(COL_TOP)
           .edges("|Z").chamfer(2.0))
    for k in range(4):
        rib = (cq.Workplane("XZ").polyline([(COL_A / 2 - 1, BASE_T), (BASE_D / 2 - 1.5, BASE_T),
                                             (COL_A / 2 - 1, BASE_T + 14)]).close()
               .extrude(1.2, both=True).rotate((0, 0, 0), (0, 0, 1), 45 + 90 * k)
               .translate((0, COL_Y, 0)))
        col = col.union(rib)
    # 喇叭口：方柱 → 托板（≤45°，不要支撑）
    px, py0, py1 = MOD_X + CLR + 1.6, FRONT, 5.0
    flare = (cq.Workplane("XY").workplane(offset=COL_TOP).center(0, COL_Y).rect(COL_A, COL_A)
             .workplane(offset=Z_F - PLATE_T - COL_TOP)
             .center(0, (py0 + py1) / 2 - COL_Y).rect(2 * px, py1 - py0).loft())
    plate = (cq.Workplane("XY").workplane(offset=Z_F - PLATE_T).center(0, (py0 + py1) / 2)
             .rect(2 * px, py1 - py0).extrude(PLATE_T))
    # 模组口袋围边（前面敞开，天线那头伸出去）
    ix, iy1 = MOD_X + CLR, MOD_Y1 + CLR
    rim = (cq.Workplane("XY").workplane(offset=Z_F).center(0, (FRONT + iy1 + 1.6) / 2)
           .rect(2 * px, iy1 + 1.6 - FRONT).extrude(WALL_H))
    rim = rim.cut(cq.Workplane("XY").workplane(offset=Z_F - 1).center(0, (FRONT - 5 + iy1) / 2)
                  .rect(2 * ix, iy1 - FRONT + 5).extrude(WALL_H + 2))
    rim = rim.edges(">Z").chamfer(0.5)
    # 肩台 + 定位销（插进两条长槽）
    post = base.union(col).union(flare).union(plate).union(rim)
    for sx in (1, -1):
        sh = (cq.Workplane("XZ").polyline([(px - 0.5, Z_F - PLATE_T - 4.5), (SLOT_X1 + 1.3, Z_F - PLATE_T),
                                            (SLOT_X1 + 1.3, Z_S), (px - 0.5, Z_S)]).close()
              .extrude(SLOT_Y + 1.5, both=True))
        pin = (cq.Workplane("XY").workplane(offset=Z_S - 0.01)
               .slot2D(2 * SLOT_Y - 1.4, (SLOT_X1 - SLOT_X0) - 0.6, angle=90)
               .extrude(0.2 + 1.6 + 0.8).faces(">Z").edges().chamfer(0.3)
               .translate(((SLOT_X0 + SLOT_X1) / 2, 0, 0)))
        part = sh.union(pin)
        if sx < 0:
            part = part.mirror("YZ")
        post = post.union(part)
    # 保险扎带孔：左右贯通，扎带绕过机身
    post = post.cut(cq.Workplane("YZ").workplane(offset=-30).center(-10, Z_F - PLATE_T / 2 - 0.2)
                    .rect(3.2, 1.4).extrude(60))
    return post


if __name__ == "__main__":
    p = build()
    out = HERE / "stl" / "thrust_post.stl"
    cq.exporters.export(p, str(out), tolerance=0.02, angularTolerance=0.15)
    import trimesh
    m = trimesh.load(out)
    bb = m.bounds
    print(f"PCB 正面高 {Z_T:.2f}  模组口袋底 {Z_F:.2f}  水密 {m.is_watertight}  体积 {m.volume / 1000:.1f} cm³"
          f"  尺寸 {(bb[1] - bb[0]).round(1)}")
