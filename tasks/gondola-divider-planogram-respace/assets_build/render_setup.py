"""Render the setup image (start and planogram end state) from the official calibrated top camera's pose.

Run: blender -b --factory-startup --python assets_build/render_setup.py -- <pkg_dir> <out.png>

Loads the package's own visual USDs (shelf, dividers, glove boxes, bench props) at the task's start layout (middle
dividers at holes +-2) and at the planogram end (holes +-3), renders both at 640 x 352 with Cycles and writes them side
by side. Camera: station top camera (calibrated), 0.944 m above the table top, pitched 60 deg down, 78 deg HFOV.
"""
import math
import sys
from pathlib import Path

import bpy
from mathutils import Matrix

argv = sys.argv[sys.argv.index("--") + 1:]
PKG = Path(argv[0]).resolve()
OUT = Path(argv[1]).resolve()
PITCH, SHELF_Z, GLOVE_X, GLOVE_Y, GLOVE_H = 0.0254, 0.015, 0.229, 0.165, 0.056
CAM_POS = (-0.1665, -0.009, 1.6918 - 0.7475)  # top camera, z from the table top
CAM_ROT = ((0.0, 0.8660, -0.5), (-1.0, 0.0, 0.0), (0.0, 0.5, 0.8660))  # camera -> world (OpenGL axes, as Blender)
FX, W0 = 392.1956, 640


def load(path, loc):
    before = set(bpy.data.objects)
    bpy.ops.wm.usd_import(filepath=str(path))
    new = [o for o in bpy.data.objects if o not in before]
    roots = [o for o in new if o.parent is None or o.parent not in new]
    for o in roots:
        o.location = (o.location[0] + loc[0], o.location[1] + loc[1], o.location[2] + loc[2])
    return new


bpy.ops.wm.read_factory_settings(use_empty=True)
scene = bpy.context.scene
scene.render.engine = "CYCLES"
scene.cycles.samples = 48
scene.cycles.device = "CPU"
scene.render.resolution_x, scene.render.resolution_y = 640, 352
world = bpy.data.worlds.new("w")
world.use_nodes = True
world.node_tree.nodes["Background"].inputs[0].default_value = (0.62, 0.63, 0.65, 1)
world.node_tree.nodes["Background"].inputs[1].default_value = 0.8
scene.world = world
bpy.ops.mesh.primitive_plane_add(size=1.0, location=(0.36, 0.0, -0.0002))
bench = bpy.context.active_object
bench.scale = (1.10, 1.30, 1.0)
m = bpy.data.materials.new("bench_laminate")
m.use_nodes = True
b = next(n for n in m.node_tree.nodes if n.type == "BSDF_PRINCIPLED")
b.inputs["Base Color"].default_value = (0.55, 0.45, 0.33, 1)
b.inputs["Roughness"].default_value = 0.6
bench.data.materials.append(m)
sun = bpy.data.objects.new("sun", bpy.data.lights.new("sun", "SUN"))
sun.data.energy = 3.0
sun.rotation_euler = (math.radians(30), math.radians(-20), math.radians(20))
scene.collection.objects.link(sun)
cam = bpy.data.objects.new("cam", bpy.data.cameras.new("cam"))
scene.collection.objects.link(cam)
scene.camera = cam
cam.data.sensor_fit = "HORIZONTAL"
cam.data.sensor_width = 36.0
cam.data.lens = FX / W0 * 36.0
cam.matrix_world = Matrix.Translation(CAM_POS) @ Matrix(CAM_ROT).to_4x4()

vis = PKG / "assets" / "visual"
props = PKG / "assets" / "props"
load(vis / "shelf.usdc", (0, 0, 0))
load(props / "bench_props.usda", (0, 0, 0))
for s in (1, -1):
    load(props / "glove_box.usda", (GLOVE_X, s * GLOVE_Y, SHELF_Z + GLOVE_H / 2))
dividers = [load(vis / "divider.usdc", (0, s * 2 * PITCH, SHELF_Z)) for s in (1, -1)]

frames = []
for tag, k in (("start", 2), ("end", 3)):
    for objs, s in zip(dividers, (1, -1)):
        roots = [o for o in objs if o.parent is None or o.parent not in objs]
        for o in roots:
            o.location.y = s * k * PITCH
    path = OUT.parent / f"_setup_{tag}.png"
    scene.render.filepath = str(path)
    bpy.ops.render.render(write_still=True)
    frames.append(path)

imgs = [bpy.data.images.load(str(p)) for p in frames]
w, h = imgs[0].size
out = bpy.data.images.new("setup", width=2 * w + 8, height=h)
px = [1.0] * (4 * (2 * w + 8) * h)
for i, im in enumerate(imgs):
    src = list(im.pixels)
    for y in range(h):
        row = src[4 * w * y: 4 * w * (y + 1)]
        start = 4 * ((2 * w + 8) * y + i * (w + 8))
        px[start: start + 4 * w] = row
out.pixels = px
out.filepath_raw = str(OUT)
out.file_format = "PNG"
out.save()
for p in frames:
    p.unlink()
print("wrote", OUT)
