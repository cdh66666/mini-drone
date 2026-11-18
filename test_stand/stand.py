"""mini-drone 纯 3D 打印调参测试架（CadQuery 参数化源文件）

用法：
    python stand.py            # 生成 STL 到 stl/（+ 装配体 assembly_*.stl）
    python stand.py --check    # 再做碰撞/限位检查

坐标：以球心（转动中心）为原点，x 朝机头，y 朝左，z 朝上。单位 mm。
零件：
    base     底座（压重物 / 胶带固定）
    post     立柱 + 球头（一体）
    adapter_clip_*  飞机转接件：两只弹性卡钩直插 PCB 两条长槽 + 卡扣式球窝（11-18 v3，不用胶/扎带）
    socket_test_*  只有球窝那一小段的紧度试件（几分钟打完，扣到球头上挑一个）

11-18 用户试装反馈：球窝太松容易脱、凸出的细支架不要 →
    卡口下加一圈直壁（原来的斜口把卡口吃掉了，实际只卡 0.17 mm），间隙改 0.2；
    删掉导向鳍和单轴卡扣、底座三角肋和胶带耳。球颈粗细保持已打印的 Ø8.18（立柱不用重打）。
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

BODY_X, BODY_Y = 40.0, 28.0   # 机身方块（前后 × 左右）
ARM_TIP_R = 43.8        # 机臂端点到中心（对角线）
MOTOR_R = 48.7          # 电机中心到机身中心（对角线）

# 球铰
BALL_D = 12.0
FIT = 0.20              # 球窝比球大（半径方向）
SOCK_WALL = 1.8
SNAP = 0.50             # 卡口比球小（半径方向）：越大越难拔，试件 0.35 / 0.50 / 0.65
SNAP_TESTS = (0.35, 0.50, 0.65)
LIP_BAND = 0.3          # 卡口直壁高度（卡住球的就是这一圈）
SLOTS = 6
SLOT_W = 1.0
SLOT_TOP = 5.0          # 开槽到 z=+5
NECK_R = 8.18 / 2       # 球颈（已打印的立柱就是这个粗细，别改）

# 托架 + 插销（v4）：托架两根硬片穿 PCB 两条长槽，露出正面的地方开横孔，插销从外往里插进横孔压住 PCB
SLOT_Y0, SLOT_Y1, SLOT_HX = 11.5, 13.5, 2.7   # 长槽：实板照片量宽约 2.0、长约 5.4（KiCad 是 6）
BRIDGE_HX = 8.0         # 横梁宽（x 方向 ±8）
BRIDGE_T = 3.5
BAT_GAP = 3.0           # 横梁顶到电池底的空隙（11-18 实测顶电池：0.8 → 3.0，PCB 背面到横梁共 10 mm）
TAB_T = 1.6             # 硬片厚（槽宽 2.0，两边各 0.2）
TAB_HX = 2.1            # 硬片半宽（4.2 宽，槽长 5.4）
WIN_HX = 1.3            # 横孔半宽（宽 2.6）
WIN_BOT = -0.4          # 横孔下沿：比 PCB 正面低 0.4（托架没推到底也能插）
WIN_TOP = 1.75          # 横孔上沿：比 PCB 正面高 1.75（插销在这里卡紧）
TAB_TOP = 2.0           # 横孔上面再留的高度
TOWER_T = 3.0           # 垫台（顶住 PCB 背面）厚
PAD_HX = 3.0            # 垫台顶面半宽（避开背面 D3/D4、C13/C14）
KEY_HX = 1.15           # 插销半宽（2.3 宽）
KEY_T0, KEY_T1 = 1.2, 1.9   # 插销尖端厚 → 根部厚（斜面插紧）
KEY_L = 3.5             # 楔子长：头顶到硬片就停，此时尖端在 y=±9.8，戳不到模组和 R2

# 立柱 / 底座
PIVOT_H = 100.0         # 球心离桌面高度
POST_A = 12.0           # 方柱边长
HEAD_A, HEAD_CH = 16.0, 2.5   # 球颈下的八角块
HEAD_TOP, HEAD_BOT = -10.0, -18.0
BASE_D, BASE_T = 150.0, 5.0
BOSS_D, BOSS_TOP = 34.0, 40.0
HOLE_CLR = 0.4          # 方孔比方柱大
HOLE_BOTTOM = 7.0       # 方孔底离桌面

# ---------------- 推导量 ----------------
RB = BALL_D / 2
RC = RB + FIT
RO = RC + SOCK_WALL


def lip_geom(snap):
    """卡口几何：返回 (卡口半径, 卡口顶深, 底面深, 倒角, 限位角°)。
    限位角 = 卡口直壁下沿碰到球颈的角度（球颈粗细固定，卡口越紧角度越小）。"""
    r_lip = RB - snap
    h = math.sqrt(RC ** 2 - r_lip ** 2)       # 球面和卡口直壁相接处（球心下方）
    hb = h + LIP_BAND
    t = 0.0
    while r_lip * math.cos(math.radians(t)) - hb * math.sin(math.radians(t)) > NECK_R:
        t += 0.1
    cr = 0.6                                  # 45° 外扩倒角（倒角面比直壁下沿更靠外，不影响限位）
    return r_lip, h, hb + cr, cr, t


R_LIP, H_LIP, Z_SOCK_BOT, LIP_CR, TILT_LIM = lip_geom(SNAP)
BRIDGE_BOT = RO - 1.0                               # 横梁底（与球窝顶融合）
BRIDGE_TOP = BRIDGE_BOT + BRIDGE_T
PLATE_TOP = BRIDGE_TOP + BAT_GAP                    # = 电池底
PCB_BOT = PLATE_TOP + BAT_H
TAB_Y0 = (SLOT_Y0 + SLOT_Y1) / 2 - TAB_T / 2       # 硬片内侧
TAB_Y1 = TAB_Y0 + TAB_T
TOWER_Y0 = TAB_Y1                                   # 垫台紧贴硬片外侧（一起加固）
TOWER_Y1 = TOWER_Y0 + TOWER_T
PCB_TOP = PCB_BOT + PCB_T


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
    # 凸台根部一圈 45° 锥，代替原来的细三角肋
    cone = (cq.Workplane("XY").workplane(offset=BASE_T - 0.01).circle(BOSS_D / 2 + 9)
            .workplane(offset=9).circle(BOSS_D / 2 - 0.01).loft())
    b = disc.union(boss).union(cone)
    # 压重物的圆坑（1 元硬币 Ø25，或螺母/钢珠/沙子）
    for k in range(4):
        a = math.radians(90 * k)
        b = b.cut(cq.Workplane("XY").workplane(offset=1.5).center(52 * math.cos(a), 52 * math.sin(a))
                  .circle(13.0).extrude(10))
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
def make_socket(snap, top):
    """卡扣式球窝：球面腔 + 卡口直壁 + 外扩倒角 + 6 条弹性槽。top = 球窝柱顶面高度。"""
    r_lip, h, zb, cr, _ = lip_geom(snap)
    sock = cq.Workplane("XY").workplane(offset=-zb).circle(RO).extrude(top + zb)
    sock = sock.cut(ball(RC))
    sock = sock.cut(cq.Workplane("XY").workplane(offset=-zb - 1).circle(r_lip).extrude(zb + 1 - h + 0.3))
    sock = sock.cut(cq.Workplane("XY").workplane(offset=-zb - 0.01).circle(r_lip + cr)
                    .workplane(offset=cr + 0.01).circle(r_lip).loft())
    for k in range(SLOTS):
        sl = (cq.Workplane("XY").workplane(offset=-zb - 1).center(RO / 2 + 1, 0)
              .rect(RO + 2, SLOT_W).extrude(SLOT_TOP + zb + 1)
              .rotate((0, 0, 0), (0, 0, 1), 15 + 360 / SLOTS * k))
        sock = sock.cut(sl)
    return sock


def _yz(pts, hx):
    """YZ 平面轮廓，沿 x 对称拉伸 ±hx（整件基本是这种 2.5D 形状：侧躺打印时每层都一样）"""
    return cq.Workplane("YZ").polyline(pts).close().extrude(hx, both=True)


def _taper(y0, y1, z0, hx0, z1, hx1):
    """x 方向收窄的 45° 过渡（侧躺打印时不悬空）"""
    yc, t = (y0 + y1) / 2, y1 - y0
    return (cq.Workplane("XY").workplane(offset=z0).center(0, yc).rect(2 * hx0, t)
            .workplane(offset=z1 - z0).rect(2 * hx1, t).loft())


def make_adapter(snap=SNAP):
    """v4 托架：球窝（口朝下）+ 横梁 + 两头 45° 斜撑落到床面（立着打不用支撑）
    + 垫台顶住 PCB 背面 + 两根硬片穿长槽、正面开横孔。前后左右对称，不分方向。"""
    z_bed = -Z_SOCK_BOT
    beam = _yz([(0, BRIDGE_BOT), (TOWER_Y1, BRIDGE_BOT), (TOWER_Y1, BRIDGE_TOP), (0, BRIDGE_TOP)], BRIDGE_HX)
    # 斜撑：内侧离球窝 0.8（不碰弹性指），外侧 45° 往上接横梁两头
    y_in = RO + 0.8
    y_foot = y_in + 3.0
    leg = _yz([(y_in, z_bed), (y_foot, z_bed), (TOWER_Y1, z_bed + (TOWER_Y1 - y_foot)),
               (TOWER_Y1, BRIDGE_BOT + 0.01), (y_in, BRIDGE_BOT + 0.01)], BRIDGE_HX)
    # 球窝和横梁之间的加强楔（在弹性槽上面）
    wedge = _yz([(RO - 0.6, SLOT_TOP + 0.3), (RO - 0.6, BRIDGE_BOT + 0.01), (y_in + 0.01, BRIDGE_BOT + 0.01),
                 (y_in + 0.01, SLOT_TOP + 0.3)], BRIDGE_HX)
    # 垫台 + 硬片下段（一整块，x 方向 45° 收窄到 ±PAD_HX，避开背面元件）
    zt = PCB_BOT - 1.0 - (BRIDGE_HX - PAD_HX)
    blk = _yz([(TAB_Y0, BRIDGE_TOP - 0.01), (TOWER_Y1, BRIDGE_TOP - 0.01), (TOWER_Y1, zt), (TAB_Y0, zt)],
              BRIDGE_HX)
    blk = blk.union(_taper(TAB_Y0, TOWER_Y1, zt - 0.01, BRIDGE_HX, PCB_BOT - 1.0, PAD_HX))
    blk = blk.union(_yz([(TOWER_Y0, PCB_BOT - 1.01), (TOWER_Y1, PCB_BOT - 1.01),
                         (TOWER_Y1, PCB_BOT), (TOWER_Y0, PCB_BOT)], PAD_HX))
    # 硬片：从垫台收窄段一直伸到 PCB 正面上方，开横孔，顶上倒角好对槽
    z_top = PCB_TOP + WIN_TOP + TAB_TOP
    tab = _yz([(TAB_Y0, zt), (TAB_Y1, zt), (TAB_Y1, z_top - 0.5), (TAB_Y1 - 0.5, z_top),
               (TAB_Y0 + 0.5, z_top), (TAB_Y0, z_top - 0.5)], TAB_HX)
    tab = tab.cut(cq.Workplane("XY").workplane(offset=PCB_TOP + WIN_BOT).center(0, (TAB_Y0 + TAB_Y1) / 2)
                  .rect(2 * WIN_HX, TAB_T + 2).extrude(WIN_TOP - WIN_BOT))
    half = beam.union(leg).union(wedge).union(blk).union(tab)
    whole = half.union(half.mirror("XZ"))
    return whole.union(make_socket(snap, BRIDGE_BOT + 0.5))


def make_key():
    """插销（打 2 个）：扁楔子，尖端在 y=0、往 +y 变厚，头在 +y 端。底面贴 PCB 正面（z=PCB_TOP），平放打印。"""
    wedge = _yz([(0, PCB_TOP), (KEY_L, PCB_TOP), (KEY_L, PCB_TOP + KEY_T1), (0, PCB_TOP + KEY_T0)], KEY_HX)
    head = (cq.Workplane("XY").workplane(offset=PCB_TOP).center(0, KEY_L + 1.25).rect(4.5, 2.5).extrude(3.0)
            .edges("|Y").fillet(0.6))
    return wedge.union(head)


def key_seated(side=1):
    """插销卡紧的位置（名义尺寸）：楔子厚度在横孔外沿处 = 横孔高（装配图和碰撞检查用）"""
    back = KEY_L * (KEY_T1 - WIN_TOP) / (KEY_T1 - KEY_T0)    # 在横孔外沿厚度 = 孔高时卡住，离插到底还差这么多
    k = make_key().translate((0, TAB_Y1 - KEY_L + back, 0))
    return k if side > 0 else k.mirror("XZ")


def make_socket_test(snap, idx):
    """紧度试件：一段球窝 + 顶上 Ø22 圆片（好捏），圆片边上 idx 个缺口 = 第几号。"""
    disc_z = RC + 0.8
    t = make_socket(snap, disc_z + 0.5)
    t = t.cut(ball(RC))
    disc = cq.Workplane("XY").workplane(offset=disc_z).circle(11).extrude(2.0)
    for k in range(idx):
        disc = disc.cut(cq.Workplane("XY").workplane(offset=disc_z - 1).center(11, 0).circle(1.3)
                        .extrude(4).rotate((0, 0, 0), (0, 0, 1), 25 * k))
    return t.union(disc)


# ---------------- 飞机示意（只用于渲染和碰撞检查） ----------------
def make_drone_simple():
    body = (cq.Workplane("XY").workplane(offset=PCB_BOT).rect(BODY_X, BODY_Y).extrude(PCB_T))
    # 机身中段更宽（KiCad 板框：y_k 89~113 处 x_k 101~139，即左右 ±19）
    body = body.union(cq.Workplane("XY").workplane(offset=PCB_BOT).center(1, 0).rect(24, 38).extrude(PCB_T))
    for k in range(4):
        arm = (cq.Workplane("XY").workplane(offset=PCB_BOT).center(ARM_TIP_R / 2, 0)
               .rect(ARM_TIP_R, 3.2).extrude(PCB_T).rotate((0, 0, 0), (0, 0, 1), 45 + 90 * k))
        body = body.union(arm)
    bat = (cq.Workplane("XY").workplane(offset=PLATE_TOP + 0.05).rect(BAT_L, BAT_W)
           .extrude(BAT_H - 0.1))
    return body, bat


# KiCad v5 元件（x_k, y_k, X 向尺寸, Y 向尺寸, 高, 面）：只列卡钩附近和高的
PARTS = {
    "U1 模组": (120, 88.36, 18, 25.5, 3.1, "top"), "J1 USB-C": (120, 118.3, 8.94, 7.55, 3.2, "top"),
    "L1 电感": (106.64, 112.65, 5.6, 6.2, 3.0, "top"), "U2": (112, 112.8, 4.95, 4.2, 1.0, "top"),
    "U4": (132, 112.2, 4.4, 4.4, 1.0, "top"),
    "D1": (105.7, 91.5, 4.32, 2.6, 2.3, "bot"), "D2": (134.3, 91.5, 4.32, 2.6, 2.3, "bot"),
    "D3": (105.7, 108.5, 4.32, 2.6, 2.3, "bot"), "D4": (134.3, 108.35, 4.32, 2.6, 2.3, "bot"),
    "Q1": (105.03, 95.5, 2.55, 2.9, 1.1, "bot"), "Q2": (134.97, 95.5, 2.55, 2.9, 1.1, "bot"),
    "Q3": (105.03, 112.5, 2.55, 2.9, 1.1, "bot"), "Q4": (134.97, 112.5, 2.55, 2.9, 1.1, "bot"),
    "C13": (105.7, 106.7, 1, 0.5, 0.5, "bot"), "C14": (134.3, 106.7, 1, 0.5, 0.5, "bot"),
    "R7": (108.2, 94.55, 1, 0.5, 0.4, "bot"), "R8": (131.8, 96.45, 1, 0.5, 0.4, "bot"),
    "R9": (108.2, 111.55, 1, 0.5, 0.4, "bot"), "R10": (131.8, 113.45, 1, 0.5, 0.4, "bot"),
    "R11": (107.77, 96, 0.5, 1, 0.4, "bot"), "R12": (132.23, 95, 0.5, 1, 0.4, "bot"),
    "R13": (107.77, 113, 0.5, 1, 0.4, "bot"), "R14": (132.23, 112, 0.5, 1, 0.4, "bot"),
    "C11": (105.7, 89.5, 1, 0.5, 0.5, "bot"), "C12": (134.3, 89.5, 1, 0.5, 0.5, "bot"),
    "C1": (108, 89.6, 1, 0.5, 0.5, "bot"), "C4": (132, 89.7, 1, 0.5, 0.5, "bot"),
    "C9": (108.5, 107.72, 1.24, 2, 0.8, "top"), "C10": (105.6, 107.72, 1.24, 2, 0.8, "top"),
    "C16": (136, 112.6, 1.24, 2, 0.8, "top"), "C17": (127.8, 111.3, 1.24, 2, 0.8, "top"),
}


def make_board():
    """带两条长槽的 PCB + 元件块（机体坐标：x = -(y_k-102)，y = -(x_k-120)）"""
    body, _ = make_drone_simple()
    for sy in (1, -1):
        slot = (cq.Workplane("XY").workplane(offset=PCB_BOT - 1).center(0, sy * (SLOT_Y0 + SLOT_Y1) / 2)
                .slot2D(2 * SLOT_HX, SLOT_Y1 - SLOT_Y0, angle=0).extrude(PCB_T + 2))
        body = body.cut(slot)
    parts = {}
    for k, (xk, yk, sx, sy_, h, side) in PARTS.items():
        z0 = PCB_BOT + PCB_T if side == "top" else PCB_BOT - h
        parts[k] = (cq.Workplane("XY").workplane(offset=z0).center(-(yk - 102), -(xk - 120))
                    .rect(sy_, sx).extrude(h))
    return body, parts


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
    print(f"PCB 底 z={PCB_BOT:.2f}  PCB 顶 z={PCB_TOP:.2f}  硬片 y {TAB_Y0:.2f}~{TAB_Y1:.2f}  垫台 y {TOWER_Y0:.2f}~{TOWER_Y1:.2f}")
    print(f"球 Ø{BALL_D}  球颈 Ø{2*NECK_R:.2f}  卡口 Ø{2*R_LIP:.2f}（深 {H_LIP:.2f}）  限位 ±{TILT_LIM:.1f}°"
          f"  横梁 {BRIDGE_BOT:.2f}~{BRIDGE_TOP:.2f}  PCB 底 z={PCB_BOT:.2f}（球心上方）")
    post = make_post()
    base = make_base()
    adapter = make_adapter()
    body, bat = make_drone_simple()
    parts = dict(post=post, base=base, adapter=adapter, body=body, bat=bat)
    # 打印朝向
    export(base, "base", zmin_to0=False)
    export(post, "post")
    for i, sn in enumerate(SNAP_TESTS, 1):
        print(f"  {i} 号：卡口比球小 {sn} mm，限位 ±{lip_geom(sn)[4]:.1f}°")
        export(make_socket_test(sn, i), f"socket_test_{i}", rot=((1, 0, 0), 180))
        # 侧躺打印：x=-8 那一面贴床，卡钩臂平躺，弯曲沿走线方向（不靠层间粘合）
        export(make_adapter(sn), f"holder_{i}")          # 立着打：球窝口贴床
    export(make_key(), "key")                             # 平放打，打 2 个
    for old in ["lock.stl", "adapter.stl", "assembly/asm_lock.stl"] +                [f"{n}_{i}.stl" for n in ("adapter", "adapter_clip", "clip_test") for i in (1, 2, 3)]:
        (OUT / old).unlink(missing_ok=True)
    # 装配体（桌面坐标）
    up = (0, 0, PIVOT_H)
    asm = {
        "asm_base": base,
        "asm_post": post.translate(up),
        "asm_adapter": adapter.translate(up),
        "asm_adapter_key1": key_seated(1).translate(up),
        "asm_adapter_key2": key_seated(-1).translate(up),
        "asm_pcb": make_board()[0].translate(up),
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
    post, adapter, body, bat = p["post"], p["adapter"], p["body"], p["bat"]
    adapter = adapter.union(key_seated(1)).union(key_seated(-1))
    board, comps = make_board()
    print("== 托架 + 插销 vs PCB（带槽）/ 电池 / 元件：重叠体积、最近距离")
    for k, v in [("PCB", board), ("电池", bat)] + list(comps.items()):
        ov = adapter.intersect(v)
        vol = ov.val().Volume() if ov.solids().size() else 0.0
        d = adapter.val().distance(v.val())
        flag = "  ← 碰！" if vol > 1e-3 else ""
        print(f"  {k:8s} 重叠 {vol:.3f}  最近 {d:.2f} mm{flag}")
    drone = adapter.union(body).union(bat)
    mounts = make_motor_mounts()
    fixed3 = post
    print("== 三轴模式：任意方向倾斜（每 15°）")
    worst_free = []
    for tilt in (TILT_LIM - 1.0, TILT_LIM + 1.5):
        n_hit = 0
        for az in range(0, 360, 15):
            ax = (math.cos(math.radians(az)), math.sin(math.radians(az)), 0)
            if hit(drone.rotate((0, 0, 0), ax, tilt), fixed3):
                n_hit += 1
        print(f"  倾斜 {tilt:.1f}°：{n_hit}/24 个方向碰到立柱")
        worst_free.append(n_hit)
    print("== 最低点（倾斜到限位时离桌面）")
    low = 1e9
    allm = drone
    for m in mounts:
        allm = allm.union(m)
    for az in range(0, 360, 45):
        ax = (math.cos(math.radians(az)), math.sin(math.radians(az)), 0)
        bb = allm.rotate((0, 0, 0), ax, TILT_LIM).val().BoundingBox()
        low = min(low, bb.zmin)
    print(f"  最低点 z={low + PIVOT_H:.1f} mm（底座最高 {BOSS_TOP} mm）")


if __name__ == "__main__":
    parts = build()
    if "--check" in sys.argv:
        check(parts)
