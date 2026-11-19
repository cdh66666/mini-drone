"""推力刚性连接件渲染：blender -b -P render_rigid.py -- stl <输出目录>"""
import math
import sys
from pathlib import Path

import bpy
from mathutils import Vector

argv = sys.argv[sys.argv.index("--") + 1:]
STL, OUTD = Path(argv[0]), Path(argv[1])
COL = {"base": (0.16, 0.36, 0.62, 1), "post": (0.16, 0.36, 0.62, 1), "rigid": (0.95, 0.45, 0.10, 1),
       "fork": (0.95, 0.85, 0.15, 1), "key": (0.55, 0.30, 0.80, 1), "pcb": (0.02, 0.16, 0.07, 1),
       "battery": (0.75, 0.75, 0.78, 1), "comp": (0.3, 0.3, 0.32, 1), "mount": (0.65, 0.85, 0.45, 1),
       "prop": (0.10, 0.10, 0.12, 1), "scale": (0.25, 0.25, 0.27, 1)}


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


def scene(asm, hide=()):
    bpy.ops.wm.read_factory_settings(use_empty=True)
    sc = bpy.context.scene
    sc.render.engine = "CYCLES"
    sc.cycles.samples = 96
    sc.cycles.use_denoising = True
    sc.render.film_transparent = True
    sc.view_settings.view_transform = "Standard"
    w = bpy.data.worlds.new("w")
    sc.world = w
    w.use_nodes = True
    w.node_tree.nodes["Background"].inputs[0].default_value = (0.93, 0.94, 0.96, 1)
    bpy.ops.mesh.primitive_plane_add(size=3000, location=(0, 0, -12.05))
    bpy.context.object.is_shadow_catcher = True
    if "scale" not in hide:   # 厨房秤
        bpy.ops.mesh.primitive_cube_add(size=1, location=(0, 0, -6), scale=(180, 180, 12))
        bpy.context.object.data.materials.append(mat("scale", COL["scale"], metal=0.6))
    for loc, e, s in [((400, -500, 600), 2e6, 400), ((-600, 250, 450), 8e5, 500)]:
        bpy.ops.object.light_add(type="AREA", location=loc)
        L = bpy.context.object
        L.data.energy, L.data.size = e, s
        L.rotation_euler = (Vector((0, 0, 100)) - Vector(loc)).to_track_quat("-Z", "Y").to_euler()
    for f in sorted((STL / asm).glob("asm_*.stl")):
        kind = next(k for k in COL if f.stem[4:].startswith(k))
        if kind in hide:
            continue
        bpy.ops.wm.stl_import(filepath=str(f))
        o = bpy.context.selected_objects[0]
        o.data.materials.append(mat(f.stem, COL[kind], 0.5 if kind == "prop" else 1.0))
        bpy.ops.object.shade_auto_smooth(angle=math.radians(35))


def shot(loc, target, lens, path, res=(1000, 1000)):
    sc = bpy.context.scene
    sc.render.resolution_x, sc.render.resolution_y = res
    bpy.ops.object.camera_add(location=loc)
    c = bpy.context.object
    c.data.lens = lens
    c.data.clip_end = 20000
    c.rotation_euler = (Vector(target) - Vector(loc)).to_track_quat("-Z", "Y").to_euler()
    sc.camera = c
    sc.render.filepath = str(path)
    bpy.ops.render.render(write_still=True)


OUTD.mkdir(parents=True, exist_ok=True)
scene("rigid_asm_up")
shot((260, -300, 220), (0, 0, 78), 50, OUTD / "rigid_up.png")
scene("rigid_asm_inv")
shot((260, -300, 220), (0, 0, 78), 50, OUTD / "rigid_inv.png")
scene("rigid_asm_up", hide=("pcb", "battery", "comp", "mount", "prop", "key", "scale", "base"))
shot((75, -95, 135), (0, 0, 100), 60, OUTD / "rigid_clamp.png")
