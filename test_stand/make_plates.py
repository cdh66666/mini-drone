"""把定高测试架的零件摆好盘，生成 Bambu Studio 项目（Bambu Lab A1，256×256）：
    python make_plates.py   → 定高测试架_A1.3mf
盘 1（黑色 PLA，AMS 1 号）：底座 + 三节塔 + 球头 + 插销 ×2
盘 2（黄色 PETG，AMS 4 号）：杆底限位环 + 托架 holder_2（一盘一种料，不出擦料塔）
做法：用 Bambu Studio 命令行载入展开好的 A1 系统预设（含 include 模板里的起始/结束 G-code）导出 3MF，
再改成手动摆放（A1 右后角 226~256 × 224~256 是缠料检测区，不放东西）和分盘、按对象设置墙数/填充，
最后用命令行切一遍确认两盘都能切。
"""
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
import zipfile
from pathlib import Path

sys.stdout.reconfigure(encoding="utf-8")
HERE = Path(__file__).resolve().parent
STL = HERE / "stl"
OUT = HERE / "定高测试架_A1.3mf"
BS = r"C:/Program Files/Bambu Studio/bambu-studio.exe"
PROF = Path(r"C:/Program Files/Bambu Studio/resources/profiles/BBL")
BED = 256.0
STRIDE = BED * 1.2              # Bambu 里第 2 块盘在第 1 块右边 1.2 倍床宽处
COLORS = ["#000000", "#4D5054", "#BCE194", "#EFE255"]   # 用户 AMS：黑 PLA / 灰 PLA-CF / 浅绿 PETG / 黄 PETG

FILES = ["alt_base.stl", "alt_tower_1.stl", "alt_tower_2.stl", "alt_tower_3.stl", "alt_ball.stl",
         "key.stl", "key.stl", "alt_collar.stl", "holder_2.stl"]
# (盘, 中心 x, 中心 y)
PLACE = {"alt_base.stl": [(1, 128, 128)], "alt_tower_1.stl": [(1, 18, 18)], "alt_tower_2.stl": [(1, 18, 238)],
         "alt_tower_3.stl": [(1, 238, 18)], "alt_ball.stl": [(1, 205, 240)],
         "key.stl": [(1, 50, 246), (1, 246, 50)],
         "alt_collar.stl": [(2, 150, 128)], "holder_2.stl": [(2, 110, 128)]}
EXTRUDER = {"alt_collar.stl": 4, "holder_2.stl": 4}
PER_OBJ = {"holder_2.stl": {"wall_loops": "2", "sparse_infill_density": "10%"},
           "alt_tower_1.stl": {"wall_loops": "3"}, "alt_tower_2.stl": {"wall_loops": "3"},
           "alt_tower_3.stl": {"wall_loops": "3"}, "alt_base.stl": {"wall_loops": "3"},
           "alt_ball.stl": {"wall_loops": "3", "sparse_infill_density": "20%"},
           "alt_collar.stl": {"wall_loops": "3", "sparse_infill_density": "20%"},
           "key.stl": {"wall_loops": "4", "sparse_infill_density": "100%"}}
PLATE_NAMES = {1: "定高测试架-PLA", 2: "定高测试架-PETG"}


def resolve(sub, name):
    """展开系统预设：inherits 链 + include 模板"""
    d = json.load(open(PROF / sub / f"{name}.json", encoding="utf8"))
    base = resolve(sub, d["inherits"]) if "inherits" in d else {}
    inc = d.get("include")
    if inc:
        names = eval(inc) if isinstance(inc, str) else inc
        for n in names:
            t = json.load(open(PROF / sub / f"{n}.json", encoding="utf8"))
            for k, v in t.items():
                if k not in ("name", "instantiation", "type", "from", "inherits"):
                    base[k] = v
    base.update({k: v for k, v in d.items() if k != "include"})
    return base


def main():
    tmp = Path(tempfile.mkdtemp())
    presets = {}
    for sub, name, fn in [("machine", "Bambu Lab A1 0.4 nozzle", "machine"),
                          ("process", "0.20mm Standard @BBL A1", "process"),
                          ("filament", "Bambu PLA Basic @BBL A1", "pla"),
                          ("filament", "Bambu PLA-CF @BBL A1", "placf"),
                          ("filament", "Bambu PETG Basic @BBL A1", "petg")]:
        d = resolve(sub, name)
        d.pop("inherits", None)
        d.update({"from": "system", "name": name})
        presets[fn] = tmp / f"{fn}.json"
        json.dump(d, open(presets[fn], "w", encoding="utf8"), ensure_ascii=False, indent=1)
    assert "A1" in json.load(open(presets["machine"], encoding="utf8"))["machine_start_gcode"][:60]
    raw = tmp / "raw.3mf"
    subprocess.run([BS, "--load-settings", f"{presets['machine']};{presets['process']}",
                    "--load-filaments", ";".join(str(presets[k]) for k in ("pla", "placf", "petg", "petg")),
                    "--arrange", "1", "--orient", "0", "--export-3mf", str(raw)] + FILES,
                   cwd=STL, check=True, capture_output=True)
    z = tmp / "z"
    zipfile.ZipFile(raw).extractall(z)
    m = (z / "3D/3dmodel.model").read_text(encoding="utf8")
    ms = (z / "Metadata/model_settings.config").read_text(encoding="utf8")
    names = dict(re.findall(r'<object id="(\d+)">\s*<metadata key="name" value="([^"]+)"', ms))
    # 每个对象自身的包围盒（组件网格 + 组件偏移）
    bbox = {}
    for oid, path, tr in re.findall(r'<object id="(\d+)"[^>]*>\s*<components>\s*<component p:path="([^"]+)"'
                                    r' objectid="\d+"[^>]*transform="([^"]+)"', m):
        t = (z / path.lstrip("/")).read_text(encoding="utf8")
        xs = [float(a) for a in re.findall(r'<vertex x="([-\d.e]+)"', t)]
        ys = [float(a) for a in re.findall(r'<vertex x="[-\d.e]+" y="([-\d.e]+)"', t)]
        off = [float(a) for a in tr.split()[9:12]]
        bbox[oid] = ((min(xs) + max(xs)) / 2 + off[0], (min(ys) + max(ys)) / 2 + off[1])
    queue = {k: list(v) for k, v in PLACE.items()}
    plate_of = {}
    for oid in sorted(names, key=int):
        n = names[oid]
        p, cx, cy = queue[n].pop(0)
        cx += (p - 1) * STRIDE
        plate_of[oid] = p
        bx, by = bbox[oid]
        m = re.sub(rf'(<item objectid="{oid}" p:UUID="[^"]*" transform="1 0 0 0 1 0 0 0 1 )[-\d.e]+ [-\d.e]+ [-\d.e]+"',
                   rf'\g<1>{cx - bx:.4f} {cy - by:.4f} 0"', m)
    (z / "3D/3dmodel.model").write_text(m, encoding="utf8")

    def fixobj(mt):
        oid, n = mt.group(1), mt.group(2)
        s = f'<object id="{oid}">\n    <metadata key="name" value="{n}"/>\n    <metadata key="extruder" value="{EXTRUDER.get(n, 1)}"/>'
        for k, v in PER_OBJ.get(n, {}).items():
            s += f'\n    <metadata key="{k}" value="{v}"/>'
        return s
    ms = re.sub(r'<object id="(\d+)">\s*<metadata key="name" value="([^"]+)"/>\s*<metadata key="extruder" value="\d+"/>',
                fixobj, ms)
    inst = {}
    for b in re.findall(r'\s*<model_instance>.*?</model_instance>', ms, re.S):
        oid = re.search(r'object_id" value="(\d+)"', b).group(1)
        inst[oid] = b
        ms = ms.replace(b, "", 1)
    a = ms.index("<plate>")
    b = ms.index("</plate>") + len("</plate>")
    plates = ""
    for p in (1, 2):
        plates += (f'<plate>\n    <metadata key="plater_id" value="{p}"/>\n    <metadata key="plater_name" value="{PLATE_NAMES[p]}"/>\n'
                   f'    <metadata key="locked" value="false"/>\n    <metadata key="filament_map_mode" value="Auto For Flush"/>'
                   + "".join(inst[o] for o in sorted(inst, key=int) if plate_of[o] == p) + "\n  </plate>\n  ")
    ms = ms[:a] + plates.rstrip() + ms[b:]
    (z / "Metadata/model_settings.config").write_text(ms, encoding="utf8")
    for f in z.glob("Metadata/*.png"):
        f.unlink()                       # 旧缩略图（命令行自动摆放时的）删掉，Bambu 打开时会重画
    c = json.load(open(z / "Metadata/project_settings.config", encoding="utf8"))
    c["filament_colour"] = COLORS
    c["curr_bed_type"] = "Textured PEI Plate"
    json.dump(c, open(z / "Metadata/project_settings.config", "w", encoding="utf8"), ensure_ascii=False, indent=4)
    OUT.unlink(missing_ok=True)
    with zipfile.ZipFile(OUT, "w", zipfile.ZIP_DEFLATED) as zf:
        for root, _, fs in os.walk(z):
            for f in fs:
                p = Path(root) / f
                zf.write(p, p.relative_to(z).as_posix())
    # 切一遍确认
    sl = tmp / "slice"
    sl.mkdir()
    subprocess.run([BS, "--slice", "0", "--outputdir", str(sl), str(OUT)], capture_output=True)
    r = json.load(open(sl / "result.json", encoding="utf8"))
    print("切片：", r["error_string"])
    for p in r.get("sliced_plates", []):
        g = sum(f["main_used_g"] for f in p["filaments"])
        print(f"  盘 {p['id']}：{g:.0f} g，约 {p['total_predication'] / 3600:.1f} 小时；"
              + "，".join(o["name"] for o in p["objects"]) + (f"  警告：{p['warning_message']}" if p["warning_message"] else ""))
    shutil.rmtree(tmp, ignore_errors=True)
    print("→", OUT)


if __name__ == "__main__":
    main()
