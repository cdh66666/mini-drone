"""推力刚性连接件：替换调参测试架上的球窝托架，把飞机锁死在现有立柱顶上（立柱、底座不重打），
整架放厨房秤上测单个电机推力。
    python rigid.py            → stl/rigid_up.stl（正装）、rigid_inv.stl（倒装）、rigid_fork.stl（楔形叉）
    python rigid.py --3mf      → 推力连接件_A1.3mf（A1 一盘摆好、切片验证）
    python rigid.py --check    → 两种装法的碰撞检查 + stl/rigid_asm_up/、rigid_asm_inv/（渲染用）

坐标同 stand.py：球心为原点，x 朝机头，y 朝左，z 朝上。
锁法（两种装法共用下半截）：
    · 八角套：套住立柱球颈下面那块 16 mm 八角块（间隙 0.15）→ 偏航锁死，也挡住左右晃；
    · 锥顶 + 楔形叉：套子里顶是 45° 锥面压在球顶上，从机头方向横插一把楔形叉（U 形口跨过球颈）托住球底，
      越推越紧 → 球被上下夹死，俯仰/横滚/上拔都锁死。
上半截：
    · rigid_up  正装（桨朝上）：和 v4.1 托架同一套横梁 + 垫台 + 硬片 + 插销，飞机高度也一样；
    · rigid_inv 倒装（桨朝下）：和推力柱 v2 同一套（模组口袋 + 垫台 + 硬片，插销在背面）。
"""
import math
import sys
from pathlib import Path

import cadquery as cq

sys.stdout.reconfigure(encoding="utf-8")
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import stand as S  # noqa: E402

OUT = HERE / "stl"
# ---------------- 下半截：八角套 + 锥顶 + 楔形叉 ----------------
OCT_CLR = 0.15            # 八角孔比八角块大（每边）
SLV_WALL = 2.4
Z_SLV_BOT = -17.5         # 套子下沿（八角块 -18 ~ -10，套住 7.5 mm）
Z_WF = -9.6               # 叉槽底（叉子底面）：比八角块顶高 0.4
BORE_R = S.RB + 0.2       # 球外的圆孔
CONE_GAP = 0.2            # 锥面离球（名义）：叉子推紧后被吃掉
CONE_C = S.RB * math.sqrt(2) + CONE_GAP * math.sqrt(2)   # 锥面 z + r = CONE_C（45°，压在球 45° 纬线上）
Z_TOP = S.BRIDGE_TOP      # 套子顶（= 正装横梁顶 10.5）
WIN_HY = 7.3              # 叉槽半宽（y）
FORK_HY = 7.0             # 叉子半宽
SLOT_HY = 4.3             # 叉口半宽（球颈 Ø8.18，每边 0.21）
SLOPE = 0.12              # 楔度：叉子每往里推 1 mm，托高 0.12 mm
NECK_Z = -math.sqrt(S.RB ** 2 - SLOT_HY ** 2)            # 叉口边线处球底高度（-4.18）
T0 = NECK_Z - Z_WF        # 叉子在 x=0（球正下方）的名义厚度 5.42
FORK_TIP, FORK_END = -15.0, 14.5    # 叉子尖 / 叉身末端（名义插到位时的 x）
SLOT_END = 4.0            # 叉口圆底中心 x：再往里推 3.8 mm 才顶到球颈
HEAD_L, HEAD_HY, HEAD_H = 3.0, 9.0, 7.5
Z_WT = Z_WF + T0 + (FORK_END) * SLOPE + 0.3              # 叉槽顶


def octo(a, ch, z0, h, off=0.0):
    s = a / 2 + off
    c = ch + off * (2 - math.sqrt(2))
    pts = [(s, s - c), (s - c, s), (-s + c, s), (-s, s - c), (-s, -s + c), (-s + c, -s), (s - c, -s), (s, -s + c)]
    return cq.Workplane("XY").workplane(offset=z0).polyline(pts).close().extrude(h)


def sleeve_outer(z1=Z_TOP):
    o = octo(S.HEAD_A, S.HEAD_CH, Z_SLV_BOT, z1 - Z_SLV_BOT, OCT_CLR + SLV_WALL)
    return o.faces("<Z").edges().chamfer(0.5)


def cavity():
    """套子里的空腔：八角孔 + 圆孔 + 45° 锥顶 + 八角孔顶的 45° 过渡（只在 y 两侧，x 两侧留给叉槽底）+ 叉槽"""
    pk = octo(S.HEAD_A, S.HEAD_CH, Z_SLV_BOT - 1, Z_WF - Z_SLV_BOT + 1, OCT_CLR)
    pk = pk.union(octo(S.HEAD_A, S.HEAD_CH, Z_SLV_BOT - 1, 1.5, OCT_CLR + 0.4))     # 下口防象脚
    zc = CONE_C - BORE_R
    bore = (cq.Workplane("XZ").polyline([(0, Z_WF - 0.01), (BORE_R, Z_WF - 0.01), (BORE_R, zc), (0, CONE_C)])
            .close().revolve(360, (0, 0, 0), (0, 1, 0)))
    r0 = S.HEAD_A / 2 * 1.25
    tr = (cq.Workplane("XY").workplane(offset=Z_WF - 0.01).circle(r0)
          .workplane(offset=r0 - BORE_R).circle(BORE_R).loft())
    tr = tr.intersect(cq.Workplane("XY").box(S.HEAD_A + 2 * OCT_CLR, 40, 40))
    # 叉槽：顶上做成 45° 尖顶（立着打不用搭桥，切片也不报悬空）
    win = (cq.Workplane("YZ").polyline([(-WIN_HY, Z_WF), (WIN_HY, Z_WF), (WIN_HY, Z_WT), (0, Z_WT + WIN_HY),
                                        (-WIN_HY, Z_WT)]).close().extrude(30, both=True))
    return pk.union(bore).union(tr).union(win)


def make_fork(dx=0.0):
    """楔形叉：从 +x（机头方向）往 -x 插；底面平，顶面往叉柄方向升高（楔）。dx = 比名义位置往外退多少。"""
    L = FORK_END - FORK_TIP
    prof = [(FORK_TIP, Z_WF), (FORK_END, Z_WF), (FORK_END, Z_WF + T0 + FORK_END * SLOPE),
            (FORK_TIP, Z_WF + T0 + FORK_TIP * SLOPE)]
    body = cq.Workplane("XZ").polyline(prof).close().extrude(FORK_HY, both=True)
    slot = (cq.Workplane("XY").workplane(offset=Z_WF - 1).center((FORK_TIP - 1 + SLOT_END) / 2, 0)
            .rect(SLOT_END - FORK_TIP + 1, 2 * SLOT_HY).extrude(20)
            .union(cq.Workplane("XY").workplane(offset=Z_WF - 1).center(SLOT_END, 0).circle(SLOT_HY).extrude(20)))
    body = body.cut(slot)
    # 叉尖内侧倒角，好对准球颈
    for sy in (1, -1):
        body = body.cut(cq.Workplane("XY").workplane(offset=Z_WF - 1).polyline(
            [(FORK_TIP - 1, sy * (SLOT_HY - 0.01)), (FORK_TIP + 2.0, sy * (SLOT_HY - 0.01)),
             (FORK_TIP - 1, sy * (SLOT_HY + 2.0))]).close().extrude(20))
    head = (cq.Workplane("XY").workplane(offset=Z_WF).center(FORK_END + HEAD_L / 2 - 0.01, 0)
            .rect(HEAD_L, 2 * HEAD_HY).extrude(HEAD_H).edges("|Z").fillet(1.0))
    del L
    return body.union(head).translate((dx, 0, 0))


# ---------------- 上半截：正装 ----------------
def make_up():
    _yz, _taper = S._yz, S._taper
    beam = _yz([(0, S.BRIDGE_BOT), (S.TOWER_Y1, S.BRIDGE_BOT), (S.TOWER_Y1, S.BRIDGE_TOP), (0, S.BRIDGE_TOP)],
               S.BRIDGE_HX)
    y_in = 8.8                                     # 斜撑从套子侧壁里面起
    gus = _yz([(y_in, S.BRIDGE_BOT - 1.5 * (S.TOWER_Y1 - y_in)), (S.TOWER_Y1, S.BRIDGE_BOT + 0.01),
               (y_in, S.BRIDGE_BOT + 0.01)], S.BRIDGE_HX)
    zt = S.PCB_BOT - 1.0 - (S.BRIDGE_HX - S.PAD_HX)
    blk = _yz([(S.TAB_Y0, S.BRIDGE_TOP - 0.01), (S.TOWER_Y1, S.BRIDGE_TOP - 0.01), (S.TOWER_Y1, zt),
               (S.TAB_Y0, zt)], S.BRIDGE_HX)
    blk = blk.union(_taper(S.TAB_Y0, S.TOWER_Y1, zt - 0.01, S.BRIDGE_HX, S.PCB_BOT - 1.0, S.PAD_HX))
    blk = blk.union(_yz([(S.TOWER_Y0, S.PCB_BOT - 1.01), (S.TOWER_Y1, S.PCB_BOT - 1.01),
                         (S.TOWER_Y1, S.PCB_BOT), (S.TOWER_Y0, S.PCB_BOT)], S.PAD_HX))
    z_top = S.PCB_TOP + S.WIN_TOP + S.TAB_TOP
    tab = _yz([(S.TAB_Y0, zt), (S.TAB_Y1, zt), (S.TAB_Y1, z_top - 0.5), (S.TAB_Y1 - 0.5, z_top),
               (S.TAB_Y0 + 0.5, z_top), (S.TAB_Y0, z_top - 0.5)], S.TAB_HX)
    tab = tab.cut(cq.Workplane("XY").workplane(offset=S.PCB_TOP + S.WIN_BOT).center(0, (S.TAB_Y0 + S.TAB_Y1) / 2)
                  .rect(2 * S.WIN_HX, S.TAB_T + 2).extrude(S.WIN_TOP - S.WIN_BOT))
    half = beam.union(gus).union(blk).union(tab)
    return sleeve_outer().union(half).union(half.mirror("XZ")).cut(cavity())


# ---------------- 上半截：倒装（推力柱 v2 的柱顶，坐标先按 thrust_post：X = 长槽方向，Y 负 = 机头） ----------------
MOD_H, CLR, WALL_H = 3.1, 0.4, 1.8
INV_ZF = 14.0                              # 模组口袋底（球心上方）
INV_ZT = INV_ZF + MOD_H + 0.15             # PCB 正面（朝下）
INV_ZB = INV_ZT + S.PCB_T                  # PCB 背面（朝上）= 插销底
INV_HX = S.SLOT_Y1 + 1.3
INV_Y0, INV_Y1 = -16.0, 4.5
SLOT_C = (S.SLOT_Y0 + S.SLOT_Y1) / 2
# 11-19 用户：硬片太窄、松。按 KiCad v5 原板长槽 2.0 × 6.0（两头 R1）重做：硬片 1.7 × 5.7，竖棱倒 0.5（让开圆头）
INV_TAB_T, INV_TAB_HY = 1.7, 2.85
TAB_X0 = SLOT_C - INV_TAB_T / 2
TAB_X1 = TAB_X0 + INV_TAB_T
Z_COL0 = 12.0                              # 柱子直段起点；下面 45° 收到套子上
Z_LOFT0 = 3.0


def make_inv():
    top = INV_ZF + WALL_H
    cy = (INV_Y0 + INV_Y1) / 2

    def foot(z0, h):
        return (cq.Workplane("XY").workplane(offset=z0).center(0, cy).rect(2 * INV_HX, INV_Y1 - INV_Y0).extrude(h)
                .edges("|Z and <Y").chamfer(6.0).edges("|Z and >Y").chamfer(2.0))
    col = foot(Z_COL0, top - Z_COL0)
    a = 8.5
    loft = (cq.Workplane("XY").workplane(offset=Z_LOFT0).rect(2 * a, 2 * a)
            .workplane(offset=Z_COL0 - Z_LOFT0).center(0, cy).rect(2 * INV_HX, INV_Y1 - INV_Y0).loft())
    loft = loft.intersect(foot(Z_LOFT0, Z_COL0 - Z_LOFT0 + 0.01))
    p = col.union(loft)
    ix, iy1 = 9.0 + CLR, -0.9 + CLR
    p = p.cut(cq.Workplane("XY").workplane(offset=INV_ZF).center(0, (INV_Y0 - 5 + iy1) / 2)
              .rect(2 * ix, iy1 - INV_Y0 + 5).extrude(20))
    z_top = INV_ZB + S.WIN_TOP + S.TAB_TOP
    for sx in (1, -1):
        pad = (cq.Workplane("XY").workplane(offset=top - 0.01).center((10.2 + INV_HX) / 2, 0)
               .rect(INV_HX - 10.2, 8.0).extrude(INV_ZT - top + 0.01))
        tab = (cq.Workplane("XZ").polyline([(TAB_X0, INV_ZT - 0.01), (TAB_X1, INV_ZT - 0.01), (TAB_X1, z_top - 0.5),
                                            (TAB_X1 - 0.5, z_top), (TAB_X0 + 0.5, z_top), (TAB_X0, z_top - 0.5)])
               .close().extrude(INV_TAB_HY, both=True).edges("|Z").chamfer(0.5))
        tab = tab.cut(cq.Workplane("XY").workplane(offset=INV_ZB + S.WIN_BOT).center(SLOT_C, 0)
                      .rect(INV_TAB_T + 2, 2 * S.WIN_HX).extrude(S.WIN_TOP - S.WIN_BOT))
        part = pad.union(tab)
        p = p.union(part if sx > 0 else part.mirror("YZ"))
    p = p.rotate((0, 0, 0), (0, 0, 1), 90)          # 转到机体坐标：长槽方向 → y，机头 → +x
    return sleeve_outer().union(p).cut(cavity())


def key_inv(sx):
    k = S.make_key()
    back = S.KEY_L * (S.KEY_T1 - S.WIN_TOP) / (S.KEY_T1 - S.KEY_T0)
    k = k.translate((0, TAB_X1 - S.KEY_L + back, INV_ZB - S.PCB_TOP))
    k = k.rotate((0, 0, 0), (0, 0, 1), -90)
    k = k if sx > 0 else k.mirror("YZ")
    return k.rotate((0, 0, 0), (0, 0, 1), 90)


# ---------------- 检查 ----------------
def vol(a, b):
    try:
        r = a.intersect(b)
        return r.val().Volume() if r.solids().size() else 0.0
    except Exception:
        return 0.0


def report(fixed, items, post, tag):
    print(f"== {tag}：连接件 + 叉 + 插销 vs 飞机（重叠体积 / 最近距离，只列 <1.5 mm 的）")
    for k, v in items:
        ov = vol(fixed, v)
        d = fixed.val().distance(v.val())
        if ov > 1e-3 or d < 1.5:
            print(f"  {k:8s} 重叠 {ov:.3f}  最近 {d:.2f} mm{'  ← 碰！' if ov > 1e-3 else ''}")
    drone = [v for k, v in items]
    dmin = min(post.val().distance(v.val()) for v in drone)
    print(f"  飞机任何零件离立柱最近 {dmin:.1f} mm")


def drone_items():
    board, comps = S.make_board()
    _, bat = S.make_drone_simple()
    it = [("PCB", board), ("电池", bat)] + list(comps.items())
    it += [(f"电机座{i}", m) for i, m in enumerate(S.make_motor_mounts())]
    it += [(f"桨{i}", m) for i, m in enumerate(S.make_props())]
    return it


def check(up, inv, fork):
    post = S.make_post()
    base = S.make_base().translate((0, 0, -S.PIVOT_H))
    print("== 连接件 / 叉 vs 立柱（名义位置；叉托住球底是贴合，退 0.1 mm 应为 0）")
    print(f"  正装件 vs 立柱 重叠 {vol(up, post):.3f}   倒装件 vs 立柱 {vol(inv, post):.3f}")
    print(f"  叉(退 0.1) vs 立柱 {vol(make_fork(0.1), post):.3f}   叉 vs 正装件 {vol(make_fork(), up):.3f}"
          f"   叉 vs 倒装件 {vol(make_fork(), inv):.3f}")
    neck = cq.Workplane("XY").workplane(offset=S.HEAD_TOP).circle(S.NECK_R).extrude(-S.HEAD_TOP - S.RB + 1.5)
    print(f"  叉再往里推 3.5 mm：叉口 vs 球颈 {vol(make_fork(-3.5), neck):.3f}（0 = 还能推，可多夹 {3.5 * SLOPE:.2f} mm）")
    # 正装
    fixed = up.union(fork).union(S.key_seated(1)).union(S.key_seated(-1))
    it = drone_items()
    report(fixed, it, post, "正装")
    zl = min(v.val().BoundingBox().zmin for k, v in it if k.startswith("桨"))
    print(f"  桨盘离底座盘面 {zl + S.PIVOT_H - S.BASE_T:.0f} mm（桨 Ø46：{(zl + S.PIVOT_H - S.BASE_T) / 46:.1f} 倍桨径）")
    asm_up = {"base": base, "post": post, "rigid": up, "fork": fork,
              "key1": S.key_seated(1), "key2": S.key_seated(-1)} | {f"d_{k}": v for k, v in it}
    # 倒装：飞机绕 x 轴翻 180°，PCB 正面落到 INV_ZT
    save = S.PCB_BOT, S.PLATE_TOP, S.SLOT_HX
    S.PCB_BOT, S.PLATE_TOP, S.SLOT_HX = 0.0, -S.BAT_H, 3.0     # 槽长按 KiCad 原板 6.0
    flip = lambda w: w.rotate((0, 0, 0), (1, 0, 0), 180).translate((0, 0, INV_ZT + S.PCB_T))  # noqa: E731
    it = [(k, flip(v)) for k, v in drone_items()]
    S.PCB_BOT, S.PLATE_TOP, S.SLOT_HX = save
    fixed = inv.union(fork).union(key_inv(1)).union(key_inv(-1))
    report(fixed, it, post.union(base), "倒装")
    zl = min(v.val().BoundingBox().zmin for k, v in it)
    zp = min(v.val().BoundingBox().zmin for k, v in it if k.startswith("桨"))
    print(f"  倒装最低点离桌面 {zl + S.PIVOT_H:.0f} mm（底座凸台顶 {S.BOSS_TOP} mm），桨盘离底座盘面 {zp + S.PIVOT_H - S.BASE_T:.0f} mm")
    asm_inv = {"base": base, "post": post, "rigid": inv, "fork": fork,
               "key1": key_inv(1), "key2": key_inv(-1)} | {f"d_{k}": v for k, v in it}
    for name, asm in (("rigid_asm_up", asm_up), ("rigid_asm_inv", asm_inv)):
        d = OUT / name
        d.mkdir(exist_ok=True)
        for f in d.glob("*.stl"):
            f.unlink()
        en = {"PCB": "pcb", "电池": "battery"}
        for i, (k, v) in enumerate(asm.items()):
            k2 = k[2:] if k.startswith("d_") else k
            kind = ("mount" if "电机座" in k2 else "prop" if "桨" in k2 else en.get(k2, "comp" if k.startswith("d_") else k2))
            cq.exporters.export(v.translate((0, 0, S.PIVOT_H)), str(d / f"asm_{kind}_{i:02d}.stl"),
                                tolerance=0.05, angularTolerance=0.3)
    # 秤上受力：整架质量（PLA 1.24 g/cm³，按体积 60% 实心估）
    import trimesh
    g = {n: trimesh.load(OUT / f"{n}.stl").volume / 1000 * 1.24 * 0.6 for n in ("base", "post", "rigid_up")}
    tot = sum(g.values()) + 49
    print(f"== 秤上整架 ≈ {tot:.0f} g（底座 {g['base']:.0f} + 立柱 {g['post']:.0f} + 连接件 {g['rigid_up']:.0f} + 飞机 49）")
    # 正装单电机 15 g 拉在对角 48.7 mm：想掀翻要绕底座边（R75）转，力矩 15×(75+34) vs 整架重 × 75
    print(f"  正装：四个全开 ~50 g < 整架 {tot:.0f} g，拉不起来；单电机掀翻力矩 {15 * (75 + 34.4):.0f} g·mm"
          f" vs 自重扶正 {tot * 75:.0f} g·mm → 不用硬币、不用胶带")


def export(wp, name):
    f = OUT / f"{name}.stl"
    cq.exporters.export(wp, str(f), tolerance=0.02, angularTolerance=0.15)
    import trimesh
    m = trimesh.load(f)
    if not m.is_watertight:
        m = max(m.split(only_watertight=False), key=lambda b: abs(b.volume))   # 丢掉锥尖处 0 体积碎片
        m.merge_vertices()
        m.fill_holes()
        trimesh.repair.fix_normals(m)
        m.export(f)
    print(f"  {name}.stl  水密={m.is_watertight}  体积 {m.volume / 1000:.1f} cm³  尺寸 {(m.bounds[1] - m.bounds[0]).round(1)}")


def bambu():
    """用户只用倒装：一盘黑 PLA，只放 rigid_inv（叉子已经打过）→ 推力连接件_A1.3mf"""
    import make_plates as MP
    MP.main(FILES=["rigid_inv.stl"], PLACE={"rigid_inv.stl": [(1, 128, 128)]}, EXTRUDER={},
            PER_OBJ={"rigid_inv.stl": {"wall_loops": "4", "sparse_infill_density": "40%"}},
            PLATE_NAMES={1: "推力连接件-倒装-PLA"}, OUT=HERE / "推力连接件_A1.3mf")


if __name__ == "__main__":
    print(f"叉子 x=0 处厚 {T0:.2f}，楔度 {SLOPE}；叉槽 z {Z_WF}~{Z_WT:.2f}；锥面压球 45° 纬线，名义空隙 {CONE_GAP}")
    print(f"正装 PCB 背面 z={S.PCB_BOT:.1f}（同 v4.1）；倒装 PCB 正面 z={INV_ZT:.2f}")
    up, inv, fork = make_up(), make_inv(), make_fork()
    for w, n in ((up, "rigid_up"), (inv, "rigid_inv")):
        export(w.translate((0, 0, -Z_SLV_BOT)), n)           # 套子下口贴床，立着打
    export(fork.translate((0, 0, -Z_WF)), "rigid_fork")    # 平放打
    if "--check" in sys.argv:
        check(up, inv, fork)
    if "--3mf" in sys.argv:
        bambu()

