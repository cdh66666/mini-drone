"""Blender 渲染（装配图 / 爆炸图）。
用法：blender -b -P render_blender.py -- <stl/assembly 目录> <输出目录>
"""
import math
import sys
from pathlib import Path

import bpy
from mathutils import Vector, Matrix

argv = sys.argv[sys.argv.index("--") + 1:]
ASM = Path(argv[0])
OUTD = Path(argv[1])
PIVOT_H = 100.0

COL = {
    "base": (0.16, 0.36, 0.62, 1), "post": (0.16, 0.36, 0.62, 1),
    "adapter": (0.95, 0.45, 0.10, 1), "lock": (0.55, 0.30, 0.80, 1),
    "pcb": (0.02, 0.16, 0.07, 1), "battery": (0.75, 0.75, 0.78, 1),
    "mount": (0.80, 0.80, 0.78, 1), "prop": (0.10, 0.10, 0.12, 1),
}
DRONE = ("adapter", "pcb", "battery", "mount", "prop")


def mat(name, rgba, alpha=1.0, rough=0.45):
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    b = m.node_tree.nodes["Principled BSDF"]
    b.inputs["Base Color"].default_value = rgba
    b.inputs["Roughness"].default_value = rough
    if alpha < 1:
        b.inputs["Alpha"].default_value = alpha
    return m


def setup():
    bpy.ops.wm.read_factory_settings(use_empty=True)
    sc = bpy.context.scene
    sc.render.engine = "CYCLES"
    sc.cycles.samples = 96
    sc.cycles.use_denoising = True
    sc.render.resolution_x, sc.render.resolution_y = 1400, 1050
    sc.render.film_transparent = True
    w = bpy.data.worlds.new("w")
    sc.world = w
    w.use_nodes = True
    w.node_tree.nodes["Background"].inputs[0].default_value = (0.93, 0.94, 0.96, 1)
    w.node_tree.nodes["Background"].inputs[1].default_value = 0.9
    sc.view_settings.view_transform = "Standard"
    # 地面
    bpy.ops.mesh.primitive_plane_add(size=2000, location=(0, 0, -0.05))
    g = bpy.context.object
    g.is_shadow_catcher = True
    # 灯
    for loc, e, s in [((250, -300, 450), 8e5, 300), ((-350, 150, 300), 3e5, 400)]:
        bpy.ops.object.light_add(type="AREA", location=loc)
        L = bpy.context.object
        L.data.energy = e
        L.data.size = s
        L.rotation_euler = (Vector((0, 0, 80)) - Vector(loc)).to_track_quat("-Z", "Y").to_euler()


def load():
    objs = {}
    for f in sorted(ASM.glob("asm_*.stl")):
        bpy.ops.wm.stl_import(filepath=str(f))
        o = bpy.context.selected_objects[0]
        key = f.stem[4:]
        kind = next(k for k in COL if key.startswith(k))
        alpha = 0.55 if kind == "prop" else 1.0
        o.data.materials.append(mat(key, COL[kind], alpha))
        bpy.ops.object.shade_auto_smooth(angle=math.radians(35))
        objs[key] = (o, kind)
    return objs


def camera(loc, target, lens=50):
    bpy.ops.object.camera_add(location=loc)
    c = bpy.context.object
    c.data.lens = lens
    c.rotation_euler = (Vector(target) - Vector(loc)).to_track_quat("-Z", "Y").to_euler()
    bpy.context.scene.camera = c
    return c


def render(path):
    bpy.context.scene.render.filepath = str(path)
    bpy.ops.render.render(write_still=True)


def tilt_drone(objs, axis, deg):
    R = (Matrix.Translation((0, 0, PIVOT_H)) @ Matrix.Rotation(math.radians(deg), 4, axis)
         @ Matrix.Translation((0, 0, -PIVOT_H)))
    for o, k in objs.values():
        if k in DRONE:
            o.matrix_world = R @ o.matrix_world


OUTD.mkdir(parents=True, exist_ok=True)

# 1) 装配图（三轴模式，无卡扣，飞机倾 18°）
setup()
objs = load()
objs["lock"][0].hide_render = True
tilt_drone(objs, Vector((0.6, 0.8, 0)).normalized(), 18)
camera((300, -330, 230), (0, 0, 75), lens=55)
render(OUTD / "stand_hero.png")

# 2) 单轴模式近景（卡扣装上，俯仰 22°，桨和电机座隐藏）
setup()
objs = load()
for o, k in objs.values():
    if k in ("prop", "mount"):
        o.hide_render = True
tilt_drone(objs, Vector((0, 1, 0)), 22)
camera((95, -150, 150), (8, 0, 92), lens=70)
render(OUTD / "stand_lock.png")

# 3) 爆炸图
setup()
objs = load()
lift = {"base": 0, "post": 25, "lock": 55, "adapter": 95, "pcb": 135, "battery": 135,
        "mount": 135, "prop": 135}
for o, k in objs.values():
    o.location.z += lift[k]
camera((420, -460, 300), (0, 0, 120), lens=50)
render(OUTD / "stand_exploded.png")
