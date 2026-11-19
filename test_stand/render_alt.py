"""定高测试架渲染：blender -b -P render_alt.py -- stl/alt_asm <输出目录>"""
import math
import sys
from pathlib import Path

import bpy
from mathutils import Vector

argv = sys.argv[sys.argv.index("--") + 1:]
ASM, OUTD = Path(argv[0]), Path(argv[1])
COL = {"base": (0.16, 0.36, 0.62, 1), "post": (0.16, 0.36, 0.62, 1), "adapter": (0.95, 0.45, 0.10, 1),
       "lock": (0.55, 0.30, 0.80, 1), "rod": (0.06, 0.06, 0.07, 1), "collar": (0.9, 0.1, 0.1, 1),
       "pcb": (0.02, 0.16, 0.07, 1), "battery": (0.75, 0.75, 0.78, 1), "mount": (0.65, 0.85, 0.45, 1),
       "prop": (0.10, 0.10, 0.12, 1)}


def mat(name, rgba, alpha=1.0):
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    b = m.node_tree.nodes["Principled BSDF"]
    b.inputs["Base Color"].default_value = rgba
    b.inputs["Roughness"].default_value = 0.45
    if alpha < 1:
        b.inputs["Alpha"].default_value = alpha
    return m


def scene(hide=()):
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
    bpy.ops.mesh.primitive_plane_add(size=3000, location=(0, 0, -0.05))
    bpy.context.object.is_shadow_catcher = True
    for loc, e, s in [((500, -600, 900), 3e6, 500), ((-700, 300, 600), 1.2e6, 600)]:
        bpy.ops.object.light_add(type="AREA", location=loc)
        L = bpy.context.object
        L.data.energy, L.data.size = e, s
        L.rotation_euler = (Vector((0, 0, 300)) - Vector(loc)).to_track_quat("-Z", "Y").to_euler()
    for f in sorted(ASM.glob("asm_*.stl")):
        key = f.stem[4:]
        kind = next(k for k in COL if key.startswith(k))
        if kind in hide:
            continue
        bpy.ops.wm.stl_import(filepath=str(f))
        o = bpy.context.selected_objects[0]
        o.data.materials.append(mat(key, COL[kind], 0.5 if kind == "prop" else 1.0))
        bpy.ops.object.shade_auto_smooth(angle=math.radians(35))


def shot(loc, target, lens, path, res=(1000, 1400)):
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
scene()
shot((900, -1050, 650), (0, 0, 290), 45, OUTD / "alt_whole.png")
scene(hide=("prop", "mount"))
shot((170, -210, 560), (0, 0, 470), 55, OUTD / "alt_top.png", res=(1000, 1000))
shot((170, -60, 150), (10, 0, 150), 60, OUTD / "alt_scale.png", res=(1000, 1000))
