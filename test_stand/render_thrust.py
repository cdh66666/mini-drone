"""推力加高柱渲染：blender -b -P render_thrust.py -- stl/thrust_asm <输出目录>"""
import math
import sys
from pathlib import Path

import bpy
from mathutils import Vector

argv = sys.argv[sys.argv.index("--") + 1:]
ASM, OUTD = Path(argv[0]), Path(argv[1])
COL = {"post": (0.95, 0.45, 0.10, 1), "pcb": (0.02, 0.16, 0.07, 1), "battery": (0.75, 0.75, 0.78, 1),
       "lock_module": (0.55, 0.56, 0.60, 1), "mount": (0.65, 0.85, 0.45, 1), "prop": (0.1, 0.1, 0.12, 1),
       "scale": (0.25, 0.25, 0.27, 1)}


def mat(name, rgba, alpha=1.0, metal=0.0):
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    b = m.node_tree.nodes["Principled BSDF"]
    b.inputs["Base Color"].default_value = rgba
    b.inputs["Roughness"].default_value = 0.45
    b.inputs["Metallic"].default_value = metal
    if alpha < 1:
        b.inputs["Alpha"].default_value = alpha
    return m


def scene(only_post=False):
    bpy.ops.wm.read_factory_settings(use_empty=True)
    sc = bpy.context.scene
    sc.render.engine = "CYCLES"
    sc.cycles.samples = 96
    sc.cycles.use_denoising = True
    sc.render.resolution_x, sc.render.resolution_y = 1000, 1000
    sc.render.film_transparent = True
    sc.view_settings.view_transform = "Standard"
    w = bpy.data.worlds.new("w")
    sc.world = w
    w.use_nodes = True
    w.node_tree.nodes["Background"].inputs[0].default_value = (0.93, 0.94, 0.96, 1)
    bpy.ops.mesh.primitive_plane_add(size=2000, location=(0, 0, -6.05))
    bpy.context.object.is_shadow_catcher = True
    # 秤盘
    bpy.ops.mesh.primitive_cube_add(size=1, location=(0, 0, -3), scale=(150, 150, 6))
    bpy.context.object.data.materials.append(mat("scale", COL["scale"], metal=0.6))
    for loc, e, s in [((200, -250, 350), 5e5, 250), ((-300, 120, 250), 2e5, 300)]:
        bpy.ops.object.light_add(type="AREA", location=loc)
        L = bpy.context.object
        L.data.energy, L.data.size = e, s
        L.rotation_euler = (Vector((0, 0, 40)) - Vector(loc)).to_track_quat("-Z", "Y").to_euler()
    for f in sorted(ASM.glob("asm_*.stl")):
        key = f.stem[4:]
        if only_post and key != "post":
            continue
        kind = next(k for k in COL if key.startswith(k))
        bpy.ops.wm.stl_import(filepath=str(f))
        o = bpy.context.selected_objects[0]
        o.data.materials.append(mat(key, COL[kind], 0.5 if kind == "prop" else 1.0,
                                    0.7 if kind == "lock_module" else 0.0))
        bpy.ops.object.shade_auto_smooth(angle=math.radians(35))


def shot(loc, target, lens, path):
    bpy.ops.object.camera_add(location=loc)
    c = bpy.context.object
    c.data.lens = lens
    c.rotation_euler = (Vector(target) - Vector(loc)).to_track_quat("-Z", "Y").to_euler()
    bpy.context.scene.camera = c
    bpy.context.scene.render.filepath = str(path)
    bpy.ops.render.render(write_still=True)


OUTD.mkdir(parents=True, exist_ok=True)
scene()
shot((230, -260, 150), (0, 0, 42), 50, OUTD / "thrust_asm.png")
scene(only_post=True)
shot((110, -130, 130), (0, -4, 40), 55, OUTD / "thrust_post.png")
