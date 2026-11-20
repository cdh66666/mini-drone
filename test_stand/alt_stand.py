"""定高测试架：飞机坐在一根能上下滑的轻导杆顶上（球铰，横滚/俯仰 ±15°，偏航放开），
导杆在一根竖直空心塔里穿两个导向孔滑动。
    python alt_stand.py            → stl/alt_*.stl
    python alt_stand.py --check    → 限位 / 碰撞 / 运动件重量 + stl/alt_asm/（装配体，渲染用）

为什么让导杆跟着飞机动（而不是固定导杆、滑块在上面跑）：
    固定导杆会从飞机中间穿过去；把导杆挪到桨外面又要一根长悬臂，滑块被别住、卡滞。
    让导杆跟着飞机走，受力全在导杆轴线上，两个导向孔一上一下（间距 65 mm）只受很小的侧力，摩擦最小。
现成件：Ø4 碳纤维管 400 mm（推荐 4×3 薄壁，约 3.4 g；4×2 约 5.8 g；或 Ø4 烧烤竹签 40 cm）。
坐标：z=0 是桌面，导杆在 x=y=0。刻度窗在塔的 +x 面。
"""
import math
import sys
from pathlib import Path

import cadquery as cq

sys.stdout.reconfigure(encoding="utf-8")
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import stand as S  # noqa: E402  托架（holder）、插销、球窝尺寸共用

# ---------------- 参数 ----------------
ROD_D = 4.0             # 导杆直径（碳管 / 竹签，量了改这里）
ROD_L = 400.0
BUSH_CLR = 1.0          # 导向孔比导杆大（直径）：卡就加大（11-20 用户实测 0.4 阻力大 → 1.0）
PRESS = 0.05            # 限位环套导杆的孔比导杆大（FDM 孔会小一点，正好压紧）
GLUE_CLR = 0.9          # 球头套导杆的孔比导杆大（直径）：用户用热熔胶固定，留胶层（11-20 改，原来压紧太紧）
TW, TWALL = 20.0, 2.4   # 塔：方管外边长 / 壁厚
H = 394.0               # 塔顶面高度
IB_TOP = 329.0          # 塔里第二个导向孔（隔板）顶面
PLATE = 4.0             # 导向孔隔板厚
FUN = (TW - 2 * TWALL) / 2 - (ROD_D + BUSH_CLR) / 2   # 隔板下面 45° 漏斗高：隔板不悬空，免支撑（11-20）
SEG = (4.0, 134.0, 264.0, H)   # 三节塔的分段高度（每节 130 mm，任何打印机都放得下）
SPIGOT = 10.0           # 插接榫长度
BASE_D, BASE_T = 200.0, 4.0
COIN_D, COIN_R = 26.0, 72.0
WIN_W = 6.0             # 刻度观察窗宽

NECK_R = 4.5            # 球头颈：配 2 号托架 → 限位约 ±15°（1 号约 ±18°，3 号约 ±12°）
SLEEVE_OD, SLEEVE_L, FLANGE_OD = 8.0, 14.0, 12.0
BALL_ROD_TOP = -8.0     # 导杆顶在球心下方 8 mm
COLLAR_OD, COLLAR_L = 10.0, 14.0
TAB_T, TAB_W, TAB_H = 0.9, 3.0, 2.5   # 弹性片（软限位）

# ---------------- 推导：行程 ----------------
CAP_TAB_TIP = H + 3.0                         # 塔顶弹性片尖
FLANGE_BOT = -(6.5 + SLEEVE_L + 1.5)          # 球头法兰底（相对球心）
PIVOT_MIN = CAP_TAB_TIP - FLANGE_BOT          # 最低：法兰落在塔顶弹性片上
RB_OF_PIVOT = BALL_ROD_TOP - ROD_L            # 导杆底 = 球心 + 这个
RB_MIN = PIVOT_MIN + RB_OF_PIVOT
RB_MAX = IB_TOP - PLATE - FUN + (TW / 2 - TWALL - 6.4) - COLLAR_L - TAB_H   # 最高：限位环弹性片尖（r 6.4）顶到第二个导向孔下面的漏斗
PIVOT_MAX = RB_MAX - RB_OF_PIVOT
MARK0 = RB_MIN + COLLAR_L                     # 刻度 0 = 最低时限位环上沿


def lim_angle(snap, neck):
    r_lip, h, *_ = S.lip_geom(snap)
    hb = h + S.LIP_BAND
    t = 0.0
    while r_lip * math.cos(math.radians(t)) - hb * math.sin(math.radians(t)) > neck:
        t += 0.1
    return t


# ---------------- 零件 ----------------
def make_ball():
    """杆顶球头（球心为原点）：Ø12 球 + 颈 + 套导杆的套筒 + 底部法兰（落在塔顶弹性片上）。竖着打，球朝上。"""
    b = S.ball(S.RB)
    neck = cq.Workplane("XY").workplane(offset=-6.6).circle(NECK_R).extrude(6.6 - 1)
    sleeve = (cq.Workplane("XY").workplane(offset=FLANGE_BOT + 1.5).circle(SLEEVE_OD / 2)
              .extrude(SLEEVE_L + 0.1))
    flange = cq.Workplane("XY").workplane(offset=FLANGE_BOT).circle(FLANGE_OD / 2).extrude(1.5)
    flange = flange.edges("<Z").chamfer(0.4)
    p = b.union(neck).union(sleeve).union(flange)
    hole = (cq.Workplane("XY").workplane(offset=FLANGE_BOT - 1).circle((ROD_D + GLUE_CLR) / 2)
            .extrude(BALL_ROD_TOP - FLANGE_BOT + 1))
    lead = (cq.Workplane("XY").workplane(offset=FLANGE_BOT - 0.01).circle((ROD_D + GLUE_CLR) / 2 + 0.6)
            .workplane(offset=0.61).circle((ROD_D + GLUE_CLR) / 2).loft())   # 下口倒角，好插好灌胶
    tip = (cq.Workplane("XY").workplane(offset=BALL_ROD_TOP - 0.01).circle((ROD_D + GLUE_CLR) / 2)
           .workplane(offset=(ROD_D + GLUE_CLR) / 2 - 0.3).circle(0.3).loft())   # 孔顶 45° 尖顶，不搭桥
    return p.cut(hole).cut(lead).cut(tip)


def make_collar():
    """杆底限位环（底面 z=0 = 导杆底）：套在导杆最下端，顶上 4 片斜着往外的弹性片 = 上端软限位，
    防止飞机把导杆拔出塔。用醒目颜色打，从刻度窗里看它的上沿读高度。"""
    c = cq.Workplane("XY").circle(COLLAR_OD / 2).extrude(COLLAR_L)
    c = c.cut(cq.Workplane("XY").workplane(offset=1.0).circle((ROD_D + PRESS) / 2).extrude(COLLAR_L))
    for k in range(4):
        tab = (cq.Workplane("XZ").polyline([(3.0, COLLAR_L - 0.01), (3.0 + TAB_T, COLLAR_L - 0.01),
                                             (3.0 + TAB_T + TAB_H, COLLAR_L + TAB_H),
                                             (3.0 + TAB_H, COLLAR_L + TAB_H)]).close()
               .extrude(TAB_W / 2, both=True).rotate((0, 0, 0), (0, 0, 1), 45 + 90 * k))
        c = c.union(tab)
    return c


def _tube(z0, z1):
    t = cq.Workplane("XY").workplane(offset=z0).rect(TW, TW).extrude(z1 - z0).edges("|Z").fillet(1.5)
    return t.cut(cq.Workplane("XY").workplane(offset=z0 - 1).rect(TW - 2 * TWALL, TW - 2 * TWALL)
                 .extrude(z1 - z0 + 2))


def _plate_with_hole(z_top):
    inner = TW - 2 * TWALL
    p = cq.Workplane("XY").workplane(offset=z_top - PLATE - FUN).rect(inner + 0.2, inner + 0.2).extrude(PLATE + FUN)
    hr = (ROD_D + BUSH_CLR) / 2
    p = p.cut(cq.Workplane("XY").workplane(offset=z_top - PLATE - FUN - 0.01).rect(inner + 0.2, inner + 0.2)
              .workplane(offset=FUN + 0.01).rect(2 * hr, 2 * hr).loft())   # 下面 45° 方漏斗：自支撑，导杆也好穿
    return p.cut(cq.Workplane("XY").workplane(offset=z_top - PLATE - FUN - 1).circle((ROD_D + BUSH_CLR) / 2)
                 .extrude(PLATE + FUN + 2))


def _scale(z0, z1):
    """+x 面的 cm 刻度：每 1 cm 短线、每 5 cm 长线 + 数字（0 = 最低位置）"""
    out = None
    for k in range(0, 31):
        z = MARK0 + 10 * k
        if not (z0 + 1 < z < z1 - 1):
            continue
        ln = 5.0 if k % 5 == 0 else 2.5
        t = (cq.Workplane("XY").workplane(offset=z - 0.4).center(TW / 2 + 0.3, WIN_W / 2 + 0.6 + ln / 2)
             .rect(0.8, ln).extrude(0.8))
        out = t if out is None else out.union(t)
        if k % 5 == 0 and z + 2.6 < z1 - 0.5 and z - 2.6 > z0 + 0.5:
            txt = (cq.Workplane("YZ").workplane(offset=TW / 2 - 0.1).center(-WIN_W / 2 - 4.5, z)
                   .text(str(k), 4.5, 0.7, combine=False, halign="center", valign="center"))
            out = out.union(txt)
    return out


def make_tower(i):
    """第 i 节塔（1~3）。1、2 节顶上有插接榫；第 3 节有两个导向孔隔板 + 塔顶弹性片（下端软限位）。"""
    z0, z1 = SEG[i - 1], SEG[i]
    t = _tube(z0, z1)
    if i < 3:   # 顶上插接榫（前面开口，刻度窗不被挡）
        inner = TW - 2 * TWALL
        sp = (cq.Workplane("XY").workplane(offset=z1 - 0.01).rect(inner - 0.3, inner - 0.3).extrude(SPIGOT)
              # 下半截伸进塔里 5 mm、和塔壁熔成一体（不然榫是悬空的）
              .union(cq.Workplane("XY").workplane(offset=z1 - 5).rect(inner + 0.4, inner + 0.4).extrude(5))
              .cut(cq.Workplane("XY").workplane(offset=z1 - 6).rect(inner - 3.5, inner - 3.5).extrude(SPIGOT + 8))
              .cut(cq.Workplane("XY").workplane(offset=z1 - 6).center(TW / 2, 0).rect(TW, WIN_W).extrude(SPIGOT + 8)))
        t = t.union(sp)
    else:
        t = t.union(_plate_with_hole(IB_TOP)).union(_plate_with_hole(H))
        t = t.cut(cq.Workplane("XY").workplane(offset=H - 1.0).circle((ROD_D + BUSH_CLR) / 2 + 1.0)
                  .workplane(offset=1.01).circle((ROD_D + BUSH_CLR) / 2).loft())
        for k in range(4):   # 塔顶 4 片往里斜的弹性片：球头法兰落在上面
            tab = (cq.Workplane("XZ").polyline([(7.6, H - 0.01), (7.6 + TAB_T, H - 0.01),
                                                 (4.9 + TAB_T, H + 3.0), (4.9, H + 3.0)]).close()
                   .extrude(TAB_W / 2, both=True).rotate((0, 0, 0), (0, 0, 1), 45 + 90 * k))
            t = t.union(tab)
    # 刻度观察窗（+x 面）
    w0 = max(z0 + 3, MARK0 - 6) if i == 1 else z0 + 3
    w1 = min(z1 - 3, IB_TOP - PLATE - FUN - 1)
    if w1 > w0:
        t = t.cut(cq.Workplane("XY").workplane(offset=w0).center(TW / 2, 0).rect(TWALL * 3, WIN_W).extrude(w1 - w0))
    sc = _scale(z0, z1)
    if sc is not None:
        t = t.union(sc)
    return t


def make_base():
    """Ø200 平底盘 + 方插座（塔插进去 16 mm）+ 6 个硬币坑"""
    b = cq.Workplane("XY").circle(BASE_D / 2).extrude(BASE_T).edges(">Z").fillet(1.5)
    boss = cq.Workplane("XY").rect(TW + 8, TW + 8).extrude(BASE_T + 16).edges("|Z").fillet(3)
    cone = (cq.Workplane("XY").workplane(offset=BASE_T - 0.01).rect(TW + 24, TW + 24)
            .workplane(offset=8).rect(TW + 8, TW + 8).loft())
    b = b.union(boss).union(cone)
    b = b.cut(cq.Workplane("XY").workplane(offset=BASE_T).rect(TW + 0.4, TW + 0.4).extrude(40))
    for k in range(6):
        b = b.cut(cq.Workplane("XY").workplane(offset=BASE_T - 2.6).center(COIN_R, 0).circle(COIN_D / 2)
                  .extrude(5).rotate((0, 0, 0), (0, 0, 1), 60 * k))
    return b


def make_rod():
    return cq.Workplane("XY").circle(ROD_D / 2).extrude(ROD_L)


# ---------------- 输出 / 检查 ----------------
def export(wp, name):
    out = HERE / "stl" / f"{name}.stl"
    bb = wp.val().BoundingBox()
    wp = wp.translate((0, 0, -bb.zmin))
    cq.exporters.export(wp, str(out), tolerance=0.02, angularTolerance=0.15)
    import trimesh
    m = trimesh.load(out)
    if not m.is_watertight:
        m.merge_vertices()
        m.fill_holes()
        m.export(out)
    print(f"  {name}.stl  水密={m.is_watertight}  体积 {m.volume / 1000:.2f} cm³  高 {bb.zmax - bb.zmin:.1f}")
    return m


def est_mass(m, walls=2, infill=0.15, dens=1.27):
    """粗估打印重量：外壳（壁厚 0.42×walls）+ 内部按填充率"""
    shell = min(m.area * 0.42 * walls, m.volume)
    return (shell + (m.volume - shell) * infill) / 1000 * dens


def build():
    print(f"行程：球心 {PIVOT_MIN:.0f} → {PIVOT_MAX:.0f} mm（{(PIVOT_MAX - PIVOT_MIN) / 10:.1f} cm），"
          f"导杆底 {RB_MIN:.0f} → {RB_MAX:.0f}")
    for i, sn in enumerate(S.SNAP_TESTS, 1):
        print(f"  配 {i} 号托架：横滚/俯仰限位 ±{lim_angle(sn, NECK_R):.1f}°")
    parts = {"alt_ball": make_ball(), "alt_collar": make_collar(), "alt_base": make_base()}
    for i in (1, 2, 3):
        parts[f"alt_tower_{i}"] = make_tower(i)
    meshes = {k: export(v, k) for k, v in parts.items()}
    return parts, meshes


def check(parts, meshes):
    pz = PIVOT_MIN
    holder = S.make_adapter(S.SNAP).union(S.key_seated(1)).union(S.key_seated(-1))
    board, comps = S.make_board()
    body, bat = S.make_drone_simple()
    mounts = S.make_motor_mounts()
    props = S.make_props()
    drone = holder.union(body).union(bat)
    ball = parts["alt_ball"]
    lim = lim_angle(S.SNAP, NECK_R)
    print(f"== 球头 vs 托架（2 号）：倾 {lim - 1:.1f}° 不碰 / 倾 {lim + 1.5:.1f}° 碰")
    for tilt in (lim - 1.0, lim + 1.5):
        n = 0
        for az in range(0, 360, 30):
            ax = (math.cos(math.radians(az)), math.sin(math.radians(az)), 0)
            if S.hit(drone.rotate((0, 0, 0), ax, tilt), ball):
                n += 1
        print(f"  倾 {tilt:.1f}°：{n}/12 个方向碰")
    # 最低位置 + 倾到限位：桨、保护圈、电机座 vs 塔
    tower = parts["alt_tower_3"].translate((0, 0, -pz))
    low = 1e9
    dmin = 1e9
    for az in range(0, 360, 45):
        ax = (math.cos(math.radians(az)), math.sin(math.radians(az)), 0)
        for m in mounts + props:
            r = m.rotate((0, 0, 0), ax, lim)
            dmin = min(dmin, r.val().distance(tower.val()))
            low = min(low, r.val().BoundingBox().zmin)
    print(f"== 最低位置倾到 ±{lim:.0f}°：电机座/桨离塔最近 {dmin:.1f} mm；最低点离桌面 {low + pz:.0f} mm")
    # 运动件重量
    print("== 运动件（算进起飞重量）粗估")
    import trimesh
    hm = trimesh.load(HERE / "stl" / "holder_2.stl")
    km = trimesh.load(HERE / "stl" / "key.stl")
    rod = math.pi / 4 * (ROD_D ** 2 - 3.0 ** 2) * ROD_L / 1000 * 1.55
    w = {"托架 holder_2（2 墙 10%）": est_mass(hm, 2, 0.10), "插销 ×2": 2 * est_mass(km, 4, 1.0),
         "球头（3 墙 20%）": est_mass(meshes["alt_ball"], 3, 0.2), "限位环（3 墙 20%）": est_mass(meshes["alt_collar"], 3, 0.2),
         "Ø4×3 碳管 400 mm": rod}
    for k, v in w.items():
        print(f"  {k:22s} {v:.1f} g")
    print(f"  合计约 {sum(w.values()):.1f} g")
    # 装配体
    out = HERE / "stl" / "alt_asm"
    out.mkdir(exist_ok=True)
    for f in out.glob("asm_*.stl"):
        f.unlink()
    mid = (PIVOT_MIN + PIVOT_MAX) / 2 - 60
    up = (0, 0, mid)
    asm = {"base": parts["alt_base"], "post_t1": parts["alt_tower_1"], "post_t2": parts["alt_tower_2"],
           "post_t3": parts["alt_tower_3"], "adapter": holder.translate(up), "lock_ball": ball.translate(up),
           "rod": make_rod().translate((0, 0, mid + RB_OF_PIVOT)),
           "collar": parts["alt_collar"].translate((0, 0, mid + RB_OF_PIVOT)),
           "pcb": board.translate(up), "battery": bat.translate(up)}
    for i, m in enumerate(mounts):
        asm[f"mount{i}"] = m.translate(up)
    for i, m in enumerate(props):
        asm[f"prop{i}"] = m.translate(up)
    for k, v in asm.items():
        cq.exporters.export(v, str(out / f"asm_{k}.stl"), tolerance=0.05, angularTolerance=0.3)
    print(f"  装配体：球心在 {mid:.0f} mm（刻度读数约 {(mid + RB_OF_PIVOT + COLLAR_L - MARK0) / 10:.1f} cm）")


def bambu_ball():
    """11-20 重打：球头（孔改大、热熔胶固定）+ 第 3 节塔（导向孔改大）→ 球头和第3节塔_A1.3mf"""
    import make_plates as MP
    MP.main(FILES=["alt_ball.stl", "alt_tower_3.stl"],
            PLACE={"alt_ball.stl": [(1, 150, 128)], "alt_tower_3.stl": [(1, 110, 128)]}, EXTRUDER={},
            PER_OBJ={"alt_ball.stl": {"wall_loops": "3", "sparse_infill_density": "20%"},
                     "alt_tower_3.stl": {"wall_loops": "3"}},
            PLATE_NAMES={1: "定高-重打-PLA"}, OUT=HERE / "球头和第3节塔_A1.3mf")


if __name__ == "__main__":
    parts, meshes = build()
    if "--check" in sys.argv:
        check(parts, meshes)
    if "--ball3mf" in sys.argv:
        bambu_ball()
