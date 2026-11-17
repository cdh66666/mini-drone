"""mini-drone 纯 3D 打印调参测试架（CadQuery 参数化源文件）

用法：
    python stand.py            # 生成 STL 到 stl/（+ 装配体 assembly_*.stl）
    python stand.py --check    # 再做碰撞/限位检查（慢，约 1~3 分钟）

坐标：以球心（转动中心）为原点，x 朝机头，y 朝左，z 朝上。单位 mm。
零件：
    base     底座（压重物 / 胶带固定）
    post     立柱 + 球头（一体）
    adapter  飞机转接件：电池托板 + 球窝 + 两片导向鳍（前 = 俯仰，左 = 横滚）
    lock     单轴卡扣：卡在球头下方的八角块上，导槽夹住一片鳍 → 只剩一个轴能转
"""
import math
import io
import sys
from pathlib import Path

import cadquery as cq

sys.stdout.reconfigure(encoding="utf-8")

HERE = Path(__file__).resolve().parent
OUT = HERE / "stl"

# ---------------- 参数（改这里） ----------------
# 飞机（量自己的飞机改）
BAT_L, BAT_W, BAT_H = 30.0, 16.0, 7.0  # 电池长(前后)/宽(左右)/厚：PCB 底面到电池底面的距离填 BAT_H
PCB_T = 1.6
STRAP_SLOT_Y = 9.3      # PCB 背面两条绑带槽离中心的距离（左右各一条，在机身正中）
BODY_X, BODY_Y = 40.0, 28.0   # 机身方块（前后 × 左右）
ARM_TIP_R = 43.8        # 机臂端点到中心（对角线）
MOTOR_R = 48.7          # 电机中心到机身中心（对角线）

# 球铰
BALL_D = 12.0
FIT = 0.25              # 球窝比球大（半径方向）
SOCK_WALL = 1.4
SNAP = 0.30             # 卡口比球小（半径方向），越大越难掰开
SLOTS = 6
SLOT_W = 1.0
SLOT_TOP = 5.0          # 开槽到 z=+5，指长约 7.5 mm
LIP_CHAMFER = 0.5
TILT_MAX = 30.0         # 限位角：球窝下沿碰到球颈

# 托板
PLATE_T = 3.0
TUNNEL_W, TUNNEL_H = 4.0, 1.3   # 托板里的绑带孔（左右方向）
FIN_T = 2.0             # 导向鳍厚
FIN_R_OUT = 32.0

# 立柱 / 底座
PIVOT_H = 100.0         # 球心离桌面高度
POST_A = 12.0           # 方柱边长
HEAD_A, HEAD_CH = 16.0, 2.5   # 八角块（卡扣定位）边长 / 倒角
HEAD_TOP, HEAD_BOT = -10.0, -18.0
BASE_D, BASE_T = 150.0, 5.0
BOSS_D, BOSS_TOP = 34.0, 40.0
HOLE_CLR = 0.4          # 方孔比方柱大
HOLE_BOTTOM = 7.0       # 方孔底离桌面

# 单轴卡扣
LOCK_GAP = 0.35         # 鳍两侧间隙
LOCK_WALL_T = 1.6
CLIP_CLR, CLIP_WALL = 0.15, 2.2
CLIP_OPEN = 14.0        # 卡口开口宽（八角块 16，掰开约 1 mm 卡上）
WALL_TOP = 14.0         # 导槽墙最高处（球心为 0）
WALL_R = 35.0           # 导槽墙外半径

# ---------------- 推导量 ----------------
RB = BALL_D / 2
RC = RB + FIT
RO = RC + SOCK_WALL
R_LIP = RB - SNAP
H_LIP = math.sqrt(RC ** 2 - R_LIP ** 2)          # 卡口在球心下方的深度
_rx = R_LIP + LIP_CHAMFER
_t = math.radians(TILT_MAX)
NECK_R = _rx * math.cos(_t) - H_LIP * math.sin(_t)  # 倾斜 TILT_MAX 时下沿刚碰球颈
PLATE_BOT = RO - 0.25                               # 托板底（与球窝顶融合）
PLATE_TOP = PLATE_BOT + PLATE_T
PCB_BOT = PLATE_TOP + BAT_H
PLATE_HX = BAT_L / 2 + 1.5
PLATE_HY = max(BAT_W / 2 + 1.5, STRAP_SLOT_Y + 0.2)


def ball(r):
    """球（两极削平 0.05 mm，避免 STL 极点破面）"""
    return cq.Workplane("XY").sphere(r).intersect(cq.Workplane("XY").box(3 * r, 3 * r, 2 * r - 0.1))


def octagon(a, ch, z0, h):
    s = a / 2
    pts = [(s, s - ch), (s - ch, s), (-s + ch, s), (-s, s - ch),
           (-s, -s + ch), (-s + ch, -s), (s - ch, -s), (s, -s + ch)]
    return cq.Workplane("XY").workplane(offset=z0).polyline(pts).close().extrude(h)


# ---------------- 立柱 + 球头 ----------------
def make_post():
    knob = ball(RB)
    neck = cq.Workplane("XY").workplane(offset=HEAD_TOP).circle(NECK_R).extrude(-HEAD_TOP - 1)
    head = octagon(HEAD_A, HEAD_CH, HEAD_BOT, HEAD_TOP - HEAD_BOT)
    head = head.faces(">Z").edges().chamfer(0.6)
    # 方柱 → 八角块的 45° 过渡（打印不用支撑）
    s0, s1 = POST_A / 2, HEAD_A / 2
    tr_h = s1 - s0
    trans = (cq.Workplane("XY").workplane(offset=HEAD_BOT - tr_h).rect(POST_A, POST_A)
             .workplane(offset=tr_h).rect(HEAD_A - 0.01, HEAD_A - 0.01).loft())
    trans = trans.intersect(octagon(HEAD_A, HEAD_CH, HEAD_BOT - tr_h, tr_h))
    z_bot = HOLE_BOTTOM - PIVOT_H
    col = (cq.Workplane("XY").workplane(offset=z_bot).rect(POST_A, POST_A)
           .extrude(HEAD_BOT - tr_h - z_bot + 0.01).edges("|Z").chamfer(1.0)
           .faces("<Z").edges().chamfer(0.8))
    return col.union(trans).union(head).union(neck).union(knob)


# ---------------- 底座 ----------------
def make_base():
    """桌面坐标：z=0 桌面。"""
    disc = cq.Workplane("XY").circle(BASE_D / 2).extrude(BASE_T).edges(">Z").fillet(1.5)
    boss = cq.Workplane("XY").circle(BOSS_D / 2).extrude(BOSS_TOP).edges(">Z").chamfer(1.0)
    ribs = None
    for k in range(4):
        rib = (cq.Workplane("XZ").polyline([(BOSS_D / 2 - 1, BASE_T), (60, BASE_T),
                                             (BOSS_D / 2 - 1, BOSS_TOP - 6)]).close()
               .extrude(1.8, both=True).rotate((0, 0, 0), (0, 0, 1), 45 + 90 * k))
        ribs = rib if ribs is None else ribs.union(rib)
    b = disc.union(boss).union(ribs)
    # 压重物的圆坑（1 元硬币 Ø25，或螺母/钢珠/沙子）
    for k in range(4):
        a = math.radians(90 * k)
        b = b.cut(cq.Workplane("XY").workplane(offset=1.5).center(52 * math.cos(a), 52 * math.sin(a))
                  .circle(13.0).extrude(10))
    # 胶带耳朵（贴纸胶带 / 美纹纸压在桌上）
    for k in range(4):
        ear = (cq.Workplane("XY").center(BASE_D / 2 + 4, 0).rect(22, 26).extrude(1.2)
               .edges("|Z").fillet(4).rotate((0, 0, 0), (0, 0, 1), 45 + 90 * k))
        b = b.union(ear)
    # 方孔 + 4 条挤压筋
    hs = POST_A + HOLE_CLR
    b = b.cut(cq.Workplane("XY").workplane(offset=HOLE_BOTTOM).rect(hs, hs).extrude(BOSS_TOP))
    b = b.cut(cq.Workplane("XY").workplane(offset=BOSS_TOP - 1.2).rect(hs, hs)
              .workplane(offset=1.21).rect(hs + 2.4, hs + 2.4).loft())   # 入口倒角
    for k in range(4):
        rib = (cq.Workplane("XY").workplane(offset=HOLE_BOTTOM).center(hs / 2, 0)
               .rect(0.5, 1.6).extrude(BOSS_TOP - HOLE_BOTTOM - 3)
               .rotate((0, 0, 0), (0, 0, 1), 90 * k))
        b = b.union(rib)
    return b


# ---------------- 飞机转接件 ----------------
def fin_profile(u_p):
    """鳍的 (u, z) 轮廓：u 沿鳍方向，z 向上。顶边 45°（倒着打不要支撑）。"""
    u0 = u_p - 0.3
    z0 = PLATE_BOT + 0.2
    # 45° 线与 r=FIN_R_OUT 圆的交点
    u = u0
    while math.hypot(u, z0 - (u - u0)) < FIN_R_OUT:
        u += 0.05
    pa = math.radians(-32)
    p_out = (FIN_R_OUT * 0.985 * math.cos(pa), FIN_R_OUT * 0.985 * math.sin(pa))
    p_in = (22.0 * math.cos(pa), 22.0 * math.sin(pa))
    return [(5.5, z0), (u0, z0), (u, z0 - (u - u0)), p_out, p_in,
            (16.0, -4.0), (11.0, 0.5), (8.8, SLOT_TOP + 0.6), (5.5, SLOT_TOP + 0.6)]


def make_fin(direction):
    """direction: 'x'（机头，俯仰用）或 'y'（左侧，横滚用）"""
    u_p = PLATE_HX if direction == "x" else PLATE_HY
    f = cq.Workplane("XZ").polyline(fin_profile(u_p)).close().extrude(FIN_T / 2, both=True)
    if direction == "y":
        f = f.rotate((0, 0, 0), (0, 0, 1), 90)
    return f


def make_adapter():
    # 托板（顶面贴电池）
    plate = (cq.Workplane("XY").workplane(offset=PLATE_BOT)
             .rect(2 * PLATE_HX, 2 * PLATE_HY).extrude(PLATE_T).edges("|Z").fillet(2.0))
    # 绑带孔：左右方向穿过托板（绑带从 PCB 两条槽穿下来，包住托板）
    tz = PLATE_BOT + (PLATE_T - TUNNEL_H) / 2 + 0.3
    plate = plate.cut(cq.Workplane("XY").workplane(offset=tz).rect(TUNNEL_W, 2 * PLATE_HY + 2)
                      .extrude(TUNNEL_H))
    # 球窝
    sock = (cq.Workplane("XY").workplane(offset=-H_LIP).circle(RO)
            .extrude(PLATE_BOT + 0.5 + H_LIP))
    sock = sock.cut(ball(RC))
    sock = sock.cut(cq.Workplane("XY").workplane(offset=-H_LIP - 1).circle(R_LIP + 0.01).extrude(2))
    cham = (cq.Workplane("XY").workplane(offset=-H_LIP - 0.01).circle(R_LIP + LIP_CHAMFER)
            .workplane(offset=LIP_CHAMFER + 0.01).circle(R_LIP - 0.01).loft())
    sock = sock.cut(cham)
    a = plate.union(sock).union(make_fin("x")).union(make_fin("y"))
    # 弹性开槽（避开 0°/90° 的鳍）
    for k in range(SLOTS):
        ang = 15 + 360 / SLOTS * k
        s = (cq.Workplane("XY").workplane(offset=-H_LIP - 1).center(RO / 2 + 1, 0)
             .rect(RO + 2, SLOT_W).extrude(SLOT_TOP + H_LIP + 1)
             .rotate((0, 0, 0), (0, 0, 1), ang))
        a = a.cut(s)
    # 机头方向箭头（托板底面刻线，装的时候对准机头）
    arrow = (cq.Workplane("XY").workplane(offset=PLATE_BOT - 0.01)
             .polyline([(PLATE_HX - 1.0, 0), (PLATE_HX - 5.0, 3.0), (PLATE_HX - 5.0, -3.0)]).close()
             .extrude(0.6))
    return a.cut(arrow)


# ---------------- 飞机示意（只用于渲染和碰撞检查） ----------------
def make_drone_simple():
    body = (cq.Workplane("XY").workplane(offset=PCB_BOT).rect(BODY_X, BODY_Y).extrude(PCB_T))
    for k in range(4):
        arm = (cq.Workplane("XY").workplane(offset=PCB_BOT).center(ARM_TIP_R / 2, 0)
               .rect(ARM_TIP_R, 3.2).extrude(PCB_T).rotate((0, 0, 0), (0, 0, 1), 45 + 90 * k))
        body = body.union(arm)
    bat = (cq.Workplane("XY").workplane(offset=PLATE_TOP + 0.05).rect(BAT_L, BAT_W)
           .extrude(BAT_H - 0.1))
    return body, bat


def make_motor_mounts():
    """新电机座 STEP（保护圈朝上），放到四个对角。"""
    step = HERE / "ref" / "motor_mount_guard_20261115.step"
    m = cq.importers.importStep(str(step))
    # STEP 里：保护圈在 z 0~9.8，电机管到 z 37，PCB 机臂卡在 z 15.25~17.1，机臂朝 -y
    m = m.rotate((0, 0, 0), (1, 0, 0), 180)          # 翻过来：保护圈朝上，机臂朝 +y
    m = m.translate((0, 0, PCB_BOT + PCB_T / 2 + 16.18))
    out = []
    for k in range(4):
        phi = 45 + 90 * k
        mk = (m.rotate((0, 0, 0), (0, 0, 1), phi + 90)
              .translate((MOTOR_R * math.cos(math.radians(phi)), MOTOR_R * math.sin(math.radians(phi)), 0)))
        out.append(mk)
    return out


def make_props():
    z = PCB_BOT + PCB_T / 2 + 16.18 + 5.0
    out = []
    for k in range(4):
        phi = math.radians(45 + 90 * k)
        out.append(cq.Workplane("XY").workplane(offset=z)
                   .center(MOTOR_R * math.cos(phi), MOTOR_R * math.sin(phi)).circle(23).extrude(0.6))
    return out


# ---------------- 单轴卡扣 ----------------
def rotY(shape, deg):
    return shape.rotate((0, 0, 0), (0, 1, 0), deg)


def make_lock(moving):
    """moving：飞机侧所有会动的零件（不含鳍）。导槽墙 = 板条区 − 俯仰/横滚两种模式下的扫掠体。"""
    hs = HEAD_A / 2 + CLIP_CLR
    outer = octagon(HEAD_A + 2 * (CLIP_CLR + CLIP_WALL), HEAD_CH + 0.9, HEAD_BOT, HEAD_TOP - HEAD_BOT)
    inner = octagon(HEAD_A + 2 * CLIP_CLR, HEAD_CH, HEAD_BOT - 1, HEAD_TOP - HEAD_BOT + 2)
    clip = outer.cut(inner)
    clip = clip.cut(cq.Workplane("XY").workplane(offset=HEAD_BOT - 1)
                    .center(-hs - 5, 0).rect(10 + 2 * (hs - (HEAD_A / 2 - HEAD_CH) - 0.8), CLIP_OPEN)
                    .extrude(20))
    # 导槽两片墙
    y0 = FIN_T / 2 + LOCK_GAP
    x0 = hs
    walls = None
    for sgn in (1, -1):
        w = (cq.Workplane("XY").workplane(offset=HEAD_BOT)
             .center(x0 + (WALL_R + 1 - x0) / 2, sgn * (y0 + LOCK_WALL_T / 2))
             .rect(WALL_R + 1 - x0, LOCK_WALL_T).extrude(40))
        walls = w if walls is None else walls.union(w)
    region = (cq.Workplane("XY").workplane(offset=HEAD_BOT - 1)
              .center(x0 + (WALL_R + 1 - x0) / 2, 0).rect(WALL_R + 1 - x0 + 0.2, 2 * (y0 + LOCK_WALL_T) + 0.2).extrude(42))
    # 外端只保留 r ≤ 33
    walls = walls.intersect(cq.Workplane("XZ").circle(WALL_R).extrude(20, both=True))
    walls = walls.cut(cq.Workplane("XY").workplane(offset=WALL_TOP).rect(100, 100).extrude(50))
    # 扫掠切除（俯仰模式：鳍在 +x；横滚模式：整机转 -90°，左鳍转到 +x）
    for rz in (0, -90):
        mv = moving.rotate((0, 0, 0), (0, 0, 1), rz)
        th = -32.0
        while th <= 32.01:
            piece = rotY(mv, th).intersect(region)
            if piece.val().isValid() and piece.solids().size() > 0:
                walls = walls.cut(piece)
            th += 1.0
    # 两片墙外端用一段圆弧连起来（鳍最远只到 r≈32），加强
    arc = (cq.Workplane("XZ").circle(WALL_R).circle(FIN_R_OUT + 1.2).extrude(y0 + 0.2, both=True)
           .intersect(walls.union(cq.Workplane("XY").box(200, 2 * y0 + 0.4, 200)))
           .intersect(cq.Workplane("XY").workplane(offset=HEAD_BOT).center(50 + x0, 0)
                      .rect(100, 2 * (y0 + 0.2)).extrude(WALL_TOP - HEAD_BOT)))
    for rz in (0, -90):
        mv = moving.rotate((0, 0, 0), (0, 0, 1), rz)
        for th in range(-32, 33, 2):
            arc = arc.cut(rotY(mv, th))
    walls = walls.union(arc)
    # 去掉孤立小块：只保留和卡箍相连的
    lock = clip.union(walls)
    solids = sorted(lock.solids().vals(), key=lambda s: -s.Volume())
    return cq.Workplane("XY").add(solids[0])


# ---------------- 主程序 ----------------
def export(wp, name, rot=None, zmin_to0=True):
    s = wp
    if rot:
        s = s.rotate((0, 0, 0), rot[0], rot[1])
    if zmin_to0:
        bb = s.val().BoundingBox() if s.size() == 1 else cq.Compound.makeCompound(s.vals()).BoundingBox()
        s = s.translate((0, 0, -bb.zmin))
    OUT.mkdir(exist_ok=True)
    f = OUT / f"{name}.stl"
    cq.exporters.export(s, str(f), tolerance=0.02, angularTolerance=0.15)
    try:  # 球面极点处偶尔有 1~2 个破面，补一下，切片软件更省心
        import trimesh
        m = trimesh.load(f)
        if not m.is_watertight:
            m.merge_vertices()
            m.fill_holes()
            trimesh.repair.fix_normals(m)
            m.export(f)
        print(f"  {name}.stl  水密={m.is_watertight}  体积 {m.volume / 1000:.1f} cm³")
    except ImportError:
        pass
    return s


def build():
    print(f"球 Ø{BALL_D}  球颈 Ø{2*NECK_R:.2f}  卡口深 {H_LIP:.2f}  托板 {PLATE_BOT:.2f}~{PLATE_TOP:.2f}"
          f"  PCB 底 z={PCB_BOT:.2f}（球心上方）")
    post = make_post()
    base = make_base()
    adapter = make_adapter()
    body, bat = make_drone_simple()
    moving = adapter.union(body).union(bat)
    # 扫掠时不考虑鳍：鳍在导槽缝里
    moving_nofin = (adapter.cut(cq.Workplane("XY").box(80, FIN_T + 0.02, 80))
                    .cut(cq.Workplane("XY").box(FIN_T + 0.02, 80, 80))
                    .union(body).union(bat))
    lock = make_lock(moving_nofin)
    parts = dict(post=post, base=base, adapter=adapter, lock=lock, body=body, bat=bat)
    # 打印朝向
    export(base, "base", zmin_to0=False)
    export(post, "post")
    export(adapter, "adapter", rot=((1, 0, 0), 180))
    export(lock, "lock")
    # 装配体（桌面坐标）
    up = (0, 0, PIVOT_H)
    asm = {
        "asm_base": base,
        "asm_post": post.translate(up),
        "asm_adapter": adapter.translate(up),
        "asm_lock": lock.translate(up),
        "asm_pcb": body.translate(up),
        "asm_battery": bat.translate(up),
    }
    for i, m in enumerate(make_motor_mounts()):
        asm[f"asm_mount{i}"] = m.translate(up)
    for i, p in enumerate(make_props()):
        asm[f"asm_prop{i}"] = p.translate(up)
    (OUT / "assembly").mkdir(exist_ok=True)
    for k, v in asm.items():
        cq.exporters.export(v, str(OUT / "assembly" / f"{k}.stl"), tolerance=0.05, angularTolerance=0.3)
    return parts


def hit(a, b):
    try:
        v = a.intersect(b).val().Volume()
    except Exception:
        return False
    return v > 0.01


def check(p):
    post, adapter, lock, body, bat = p["post"], p["adapter"], p["lock"], p["body"], p["bat"]
    drone = adapter.union(body).union(bat)
    mounts = make_motor_mounts()
    fixed3 = post
    print("== 三轴模式：任意方向倾斜（每 15°）")
    worst_free = []
    for tilt in (TILT_MAX - 1.0, TILT_MAX + 1.5):
        n_hit = 0
        for az in range(0, 360, 15):
            ax = (math.cos(math.radians(az)), math.sin(math.radians(az)), 0)
            if hit(drone.rotate((0, 0, 0), ax, tilt), fixed3):
                n_hit += 1
        print(f"  倾斜 {tilt:.1f}°：{n_hit}/24 个方向碰到立柱")
        worst_free.append(n_hit)
    print("== 单轴（俯仰）模式：卡扣导槽在机头方向")
    fixed1 = post.union(lock)
    for name, ax, ang in [("俯仰 +29", (0, 1, 0), 29), ("俯仰 -29", (0, 1, 0), -29),
                          ("俯仰 +31.5", (0, 1, 0), 31.5), ("横滚 3°", (1, 0, 0), 3),
                          ("横滚 -3°", (1, 0, 0), -3), ("偏航 3°", (0, 0, 1), 3)]:
        print(f"  {name:10s} 碰撞={hit(drone.rotate((0, 0, 0), ax, ang), fixed1)}")
    print("== 单轴（横滚）模式：卡扣转 90° 卡在左侧")
    fixedR = post.union(lock.rotate((0, 0, 0), (0, 0, 1), 90))
    for name, ax, ang in [("横滚 +29", (1, 0, 0), 29), ("横滚 -29", (1, 0, 0), -29),
                          ("俯仰 3°", (0, 1, 0), 3), ("偏航 3°", (0, 0, 1), 3)]:
        print(f"  {name:10s} 碰撞={hit(drone.rotate((0, 0, 0), ax, ang), fixedR)}")
    # 鳍始终在导槽里（俯仰到底时也夹得住）
    print("== 最低点（倾斜到限位时离桌面）")
    low = 1e9
    allm = drone
    for m in mounts:
        allm = allm.union(m)
    for az in range(0, 360, 45):
        ax = (math.cos(math.radians(az)), math.sin(math.radians(az)), 0)
        bb = allm.rotate((0, 0, 0), ax, TILT_MAX).val().BoundingBox()
        low = min(low, bb.zmin)
    print(f"  最低点 z={low + PIVOT_H:.1f} mm（底座最高 {BOSS_TOP} mm）")


if __name__ == "__main__":
    parts = build()
    if "--check" in sys.argv:
        check(parts)
