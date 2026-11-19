"""推力测试柱 v2：飞机倒过来（桨朝下、电池朝上）锁在柱顶，放在厨房秤上测单个电机推力。
python thrust_post.py            → stl/thrust_post_v2.stl
python thrust_post.py --check    → 倒置姿态碰撞检查 + stl/thrust_asm/（装配体，渲染用）

坐标：z=0 是秤盘，X/Y 是 PCB 坐标（板中心为原点，Y 负 = 机头，KiCad v5；长槽长度按实板照片 5.4）。
倒置时朝下的是 PCB 正面（ESP32 模组那面），朝上的是背面（电池那面）。
固定：和调参测试架 v4.1 同一套——两根硬片从正面穿过两条长槽，背面冒出来的横孔里各插一根插销（同一个 key.stl），
      把 PCB 夹在柱顶的垫台上；模组压在浅口袋里只做辅助支撑。不用胶。
底座：Ø110 × 4 平圆盘，4 个放硬币的坑。直接平放在秤盘上（≥120 mm 的盘都放得下）。
"""
import sys
from pathlib import Path

import cadquery as cq

sys.stdout.reconfigure(encoding="utf-8")
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import stand as S  # noqa: E402  长槽、硬片、横孔、插销尺寸和测试架共用

GUARD_ABOVE_PCB = 15.25   # 保护圈顶比 PCB 正面高（电机座 STEP 量的）
LOW_GAP = 40.0            # 倒置后最低点（保护圈顶）离秤盘
MOD_H = 3.1
Z_T = LOW_GAP + GUARD_ABOVE_PCB      # PCB 正面（朝下）高度 = 垫台顶
Z_B = Z_T + S.PCB_T                  # PCB 背面（朝上）高度 = 插销底
Z_F = Z_T - MOD_H - 0.15             # 模组口袋底（比模组低 0.15，夹紧力在垫台上，模组只防晃）
BASE_D, BASE_T = 110.0, 4.0
COIN_D, COIN_DEPTH, COIN_R = 26.0, 2.6, 40.0

MOD_X, MOD_Y1 = 9.0, -0.9
CLR = 0.4
WALL_H = 1.8                         # 口袋围边高（离 PCB 还有 1.3 mm，避开小元件）
FRONT_CH = 6.0                       # 前面两个竖棱倒大角，让开前电机座的辐条
FRONT = -16.0                        # 前端只到这里：离前面两个电机座的辐条 ≥4 mm
SLOT_C = (S.SLOT_Y0 + S.SLOT_Y1) / 2  # 长槽中心离板中心 12.5（测试架里是 y，这里是 X）
TAB_X0 = SLOT_C - S.TAB_T / 2
TAB_X1 = TAB_X0 + S.TAB_T
PAD_X0 = 10.2                        # 垫台内沿（再往里正面有 R2）
PAD_HY = 4.0                         # 垫台半长（Y ±4，再往外正面有 C9/C10）


def build():
    hx = S.SLOT_Y1 + 1.3                 # 柱子半宽 14.8：刚好盖住两条长槽外沿
    y0, y1 = FRONT, 4.5
    cy = (y0 + y1) / 2
    top = Z_F + WALL_H                   # 平时顶面：比 PCB 低 1.3 mm
    # 底座：Ø110 平圆盘 + 4 个硬币坑（放在电机正下方之间，压稳）
    base = cq.Workplane("XY").circle(BASE_D / 2).extrude(BASE_T).edges(">Z").fillet(1.2)
    for k in range(4):
        base = base.cut(cq.Workplane("XY").workplane(offset=BASE_T - COIN_DEPTH)
                        .center(COIN_R, 0).circle(COIN_D / 2).extrude(5)
                        .rotate((0, 0, 0), (0, 0, 1), 90 * k))
    col = (cq.Workplane("XY").center(0, cy).rect(2 * hx, y1 - y0).extrude(top)
           .edges("|Z and <Y").chamfer(FRONT_CH).edges("|Z and >Y").chamfer(2.0))
    # 柱根 45° 加强台
    root = (cq.Workplane("XY").workplane(offset=BASE_T - 0.01).center(0, cy).rect(2 * hx + 8, y1 - y0 + 8)
            .workplane(offset=4).rect(2 * hx, y1 - y0).loft())
    root = root.intersect(cq.Workplane("XY").center(0, cy).rect(2 * hx + 8, y1 - y0 + 8).extrude(20)
                          .edges("|Z and <Y").chamfer(FRONT_CH + 3).edges("|Z and >Y").chamfer(5.0))
    post = base.union(col).union(root)
    # 模组口袋（前面敞开，天线那头伸出去）
    ix, iy1 = MOD_X + CLR, MOD_Y1 + CLR
    post = post.cut(cq.Workplane("XY").workplane(offset=Z_F).center(0, (y0 - 5 + iy1) / 2)
                    .rect(2 * ix, iy1 - y0 + 5).extrude(20))
    # 垫台（托住 PCB 正面）+ 硬片（穿长槽，背面开横孔）
    z_top = Z_B + S.WIN_TOP + S.TAB_TOP
    for sx in (1, -1):
        pad = (cq.Workplane("XY").workplane(offset=top - 0.01).center((PAD_X0 + hx) / 2, 0)
               .rect(hx - PAD_X0, 2 * PAD_HY).extrude(Z_T - top + 0.01))
        tab = (cq.Workplane("XZ").polyline([(TAB_X0, Z_T - 0.01), (TAB_X1, Z_T - 0.01), (TAB_X1, z_top - 0.5),
                                            (TAB_X1 - 0.5, z_top), (TAB_X0 + 0.5, z_top), (TAB_X0, z_top - 0.5)])
               .close().extrude(S.TAB_HX, both=True))
        tab = tab.cut(cq.Workplane("XY").workplane(offset=Z_B + S.WIN_BOT).center(SLOT_C, 0)
                      .rect(S.TAB_T + 2, 2 * S.WIN_HX).extrude(S.WIN_TOP - S.WIN_BOT))
        part = pad.union(tab)
        if sx < 0:
            part = part.mirror("YZ")
        post = post.union(part)
    return post


def key_here(sx):
    """插销卡紧位置（柱子坐标）：沿 X 从外往里插，底面贴 PCB 背面"""
    k = S.make_key()                                    # 测试架坐标：尖端 y=0、往 +y，底面 z=S.PCB_TOP
    back = S.KEY_L * (S.KEY_T1 - S.WIN_TOP) / (S.KEY_T1 - S.KEY_T0)
    k = k.translate((0, TAB_X1 - S.KEY_L + back, Z_B - S.PCB_TOP))
    k = k.rotate((0, 0, 0), (0, 0, 1), -90)             # +y → +X
    return k if sx > 0 else k.mirror("YZ")


def check(post):
    """倒置姿态：飞机（测试架坐标）绕 x 轴翻 180°，PCB 正面对到 Z_T；柱子坐标 (X, Y) 对应机体 (-Y, X)"""
    S.PCB_BOT = 0.0
    S.PLATE_TOP = -S.BAT_H
    flip = lambda w: w.rotate((0, 0, 0), (1, 0, 0), 180).translate((0, 0, Z_T + S.PCB_T))
    to_drone = lambda w: w.rotate((0, 0, 0), (0, 0, 1), 90)
    fixed = to_drone(post.union(key_here(1)).union(key_here(-1)))
    board, comps = S.make_board()
    _, bat = S.make_drone_simple()
    items = [("PCB", board), ("电池", bat)] + list(comps.items())
    items += [(f"电机座{i}", m) for i, m in enumerate(S.make_motor_mounts())]
    items += [(f"桨{i}", m) for i, m in enumerate(S.make_props())]
    print("== 推力柱 + 插销 vs 倒置的飞机：重叠体积、最近距离")
    low = 1e9
    for k, v in items:
        v = flip(v)
        if k.startswith("电机座"):
            low = min(low, v.val().BoundingBox().zmin)
        ov = fixed.intersect(v)
        vol = ov.val().Volume() if ov.solids().size() else 0.0
        d = fixed.val().distance(v.val())
        if vol > 1e-3 or d < 1.0:
            print(f"  {k:8s} 重叠 {vol:.3f}  最近 {d:.2f} mm{'  ← 碰！' if vol > 1e-3 else ''}")
    print(f"  其余都 ≥1 mm；倒置最低点（保护圈顶）离秤盘 {low:.1f} mm")
    # 装配体（渲染用）
    out = HERE / "stl" / "thrust_asm"
    for f in out.glob("asm_*.stl"):
        f.unlink()
    asm = {"post": fixed, "pcb": flip(board), "battery": flip(bat),
           "lock_module": flip(S.make_board()[1]["U1 模组"])}
    for i, m in enumerate(S.make_motor_mounts()):
        asm[f"mount{i}"] = flip(m)
    for i, m in enumerate(S.make_props()):
        asm[f"prop{i}"] = flip(m)
    for k, v in asm.items():
        cq.exporters.export(v, str(out / f"asm_{k}.stl"), tolerance=0.05, angularTolerance=0.3)


if __name__ == "__main__":
    p = build()
    old = HERE / "stl" / "thrust_post.stl"
    old.unlink(missing_ok=True)
    out = HERE / "stl" / "thrust_post_v2.stl"
    cq.exporters.export(p, str(out), tolerance=0.02, angularTolerance=0.15)
    import trimesh
    m = trimesh.load(out)
    bb = m.bounds
    print(f"PCB 正面 z={Z_T:.2f}  背面 z={Z_B:.2f}  模组口袋底 {Z_F:.2f}  水密 {m.is_watertight}"
          f"  体积 {m.volume / 1000:.1f} cm³  尺寸 {(bb[1] - bb[0]).round(1)}")
    if "--check" in sys.argv:
        check(p)
