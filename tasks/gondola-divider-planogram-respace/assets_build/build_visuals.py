"""Author the visual meshes of the gondola shelf section and its wire dividers in Blender; one USD per rigid body.

Run: blender -b --factory-startup --python assets_build/build_visuals.py -- <pkg_dir> [preview_dir]

Every mesh is in its body's part frame (metres, Z up):
  shelf.usdc   -- shelf part frame = station x, y; z = 0 on the bench: deck with two rows of 1/4 in holes on 1 in
                  centres, bullnose front lip with a price-label channel and blank labels, rear flange, bench stands,
                  3 in front wire fence (vertical wires on 1 in centres) and the two fixed outer dividers (holes +-10).
  divider.usdc -- divider part frame: y = 0 on the divider's hole line, z = 0 on the deck top.
Dimensions follow the frozen constants of gondola_divider_planogram_respace.py (catalog values: 1/4 in holes on 1 in
centres, 3 in fencing, divider 2 3/4 in shorter than the 13 in shelf; wire sizes estimated). No maker marks.
"""
import math
import sys
from pathlib import Path

import bmesh
import bpy

argv = sys.argv[sys.argv.index("--") + 1:]
PKG = Path(argv[0]).resolve()
PREVIEW = Path(argv[1]) if len(argv) > 1 else None
OUT = PKG / "assets" / "visual"
OUT.mkdir(parents=True, exist_ok=True)
MM = 0.001

# frozen task constants (mm)
PITCH, HOLE_D, N_HOLES = 25.4, 6.35, 17
SHELF_Z, SHEET = 15.0, 1.2
X_FRONT, X_BACK, HALF_LEN = 115.0, 445.0, 457.0
X_FENCE, X_HOLE, X_FRONT_ROW = 130.0, 390.0, 152.0
FENCE_H, FENCE_R = 76.2, 2.25
FENCE_ZC = FENCE_H - FENCE_R
WIRE_R = 2.0
X_POST, X_HOOK = X_FENCE + 6.0, X_FENCE - 6.0
RAIL_Z = FENCE_ZC + FENCE_R + WIRE_R + 0.5
HOOK_LEG, FOOT = 14.0, 10.0
STAYS = (X_POST + 85.0, X_HOLE - 85.0)
BASE_W, BASE_X = 24.0, (X_POST + 10.0, X_HOLE - 12.0)
K_OUTER = 10

bpy.ops.wm.read_factory_settings(use_empty=True)
scene = bpy.context.scene


def srgb(c):
    return tuple(((x / 255.0 + 0.055) / 1.055) ** 2.4 for x in c)


def material(name, color, rough, metal=0.0):
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    bsdf = next(n for n in m.node_tree.nodes if n.type == "BSDF_PRINCIPLED")
    bsdf.inputs["Base Color"].default_value = (*color, 1.0)
    bsdf.inputs["Roughness"].default_value = rough
    bsdf.inputs["Metallic"].default_value = metal
    return m


POWDER = material("powder_coat_platinum_steel", srgb((196, 197, 192)), 0.55, metal=0.15)
CHROME = material("zinc_plated_wire", srgb((200, 204, 210)), 0.28, metal=1.0)  # catalog standard finish ZNC
STAND = material("stand_black_powder_coat", srgb((52, 53, 56)), 0.5, metal=0.2)
CHANNEL = material("label_channel_pvc_grey", srgb((168, 170, 172)), 0.35)
LABEL = material("blank_shelf_label_paper", srgb((242, 242, 236)), 0.7)
HOLE_DARK = material("hole_shadow", srgb((40, 40, 42)), 0.8)

GROUPS = {"shelf": [], "divider": []}


def finish(obj, group, mat, smooth=False):
    obj.data.materials.clear()
    obj.data.materials.append(mat)
    for p in obj.data.polygons:
        p.use_smooth = smooth
    GROUPS[group].append(obj)
    return obj


def box(name, center, size, group, mat):
    bpy.ops.mesh.primitive_cube_add(size=1.0, location=tuple(c * MM for c in center))
    obj = bpy.context.active_object
    obj.name = name
    obj.scale = tuple(s * MM for s in size)
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    return finish(obj, group, mat)


def wire(name, pts, r, group, mat, seg=10):
    """A bent wire through the (x, y, z) mm points, round section of radius r mm, rounded joints."""
    cu = bpy.data.curves.new(name, "CURVE")
    cu.dimensions = "3D"
    sp = cu.splines.new("POLY")
    sp.points.add(len(pts) - 1)
    for p, q in zip(sp.points, pts):
        p.co = (q[0] * MM, q[1] * MM, q[2] * MM, 1.0)
    cu.bevel_depth = r * MM
    cu.bevel_resolution = 3
    cu.use_fill_caps = True
    obj = bpy.data.objects.new(name, cu)
    scene.collection.objects.link(obj)
    bpy.context.view_layer.objects.active = obj
    bpy.ops.object.select_all(action="DESELECT")
    obj.select_set(True)
    bpy.ops.object.convert(target="MESH")
    obj = bpy.context.active_object
    for i, q in enumerate(pts):  # weld beads at the bends
        if 0 < i < len(pts) - 1:
            bpy.ops.mesh.primitive_uv_sphere_add(segments=12, ring_count=8, radius=r * 1.04 * MM,
                                                 location=(q[0] * MM, q[1] * MM, q[2] * MM))
            finish(bpy.context.active_object, group, mat, smooth=True)
    return finish(obj, group, mat, smooth=True)


def divider(prefix, y, z0, group):
    """One wire divider (seated pose): hook leg, top rail, rear post into the foot; front post into the bottom rail;
    two stays; a cross-wire base at each end."""
    zr = z0 + RAIL_Z
    r = WIRE_R
    wire(f"{prefix}_rail", [(X_HOOK, y, zr - HOOK_LEG), (X_HOOK, y, zr), (X_HOLE, y, zr), (X_HOLE, y, z0 - FOOT)],
         r, group, CHROME)
    wire(f"{prefix}_base", [(X_POST, y, zr), (X_POST, y, z0 + r), (X_HOLE, y, z0 + r)], r, group, CHROME)
    for i, x in enumerate(STAYS):
        wire(f"{prefix}_stay{i}", [(x, y, z0 + r), (x, y, zr)], r * 0.85, group, CHROME)
    for i, x in enumerate(BASE_X):
        wire(f"{prefix}_cross{i}", [(x, y - BASE_W / 2, z0 + r), (x, y + BASE_W / 2, z0 + r)], r, group, CHROME)


# -- shelf (part frame: station x, y; z = 0 on the bench) ----------------------------------------------------------
deck = box("deck", ((X_FRONT + X_BACK) / 2, 0.0, SHELF_Z - SHEET / 2), (X_BACK - X_FRONT, 2 * HALF_LEN, SHEET),
           "shelf", POWDER)
cutters = []
for row_x in (X_FRONT_ROW, X_HOLE):
    for k in range(-N_HOLES, N_HOLES + 1):
        bpy.ops.mesh.primitive_cylinder_add(vertices=20, radius=HOLE_D / 2 * MM, depth=6 * MM,
                                            location=(row_x * MM, k * PITCH * MM, SHELF_Z * MM))
        cutters.append(bpy.context.active_object)
bpy.ops.object.select_all(action="DESELECT")
for c in cutters:
    c.select_set(True)
bpy.context.view_layer.objects.active = cutters[0]
bpy.ops.object.join()
cutter = bpy.context.active_object
mod = deck.modifiers.new("holes", "BOOLEAN")
mod.operation = "DIFFERENCE"
mod.object = cutter
bpy.context.view_layer.objects.active = deck
bpy.ops.object.modifier_apply(modifier=mod.name)
bpy.data.objects.remove(cutter)
# dark floor under the holes (the shelf's shadowed underside seen through them)
box("under", ((X_FRONT + X_BACK) / 2, 0.0, SHELF_Z - 9.0), (X_BACK - X_FRONT - 4, 2 * HALF_LEN - 70, 0.6), "shelf",
    HOLE_DARK)
# bullnose front lip with its price-label channel, rear flange
box("front_lip", (X_FRONT + 0.6, 0.0, SHELF_Z - 6.5), (SHEET, 2 * HALF_LEN, 13.0), "shelf", POWDER)
bpy.ops.mesh.primitive_cylinder_add(vertices=24, radius=1.6 * MM, depth=2 * HALF_LEN * MM,
                                    location=((X_FRONT + 0.6) * MM, 0.0, SHELF_Z * MM), rotation=(math.radians(90), 0, 0))
finish(bpy.context.active_object, "shelf", POWDER, smooth=True)
box("label_channel", (X_FRONT - 0.8, 0.0, SHELF_Z - 6.5), (1.6, 2 * HALF_LEN, 11.0), "shelf", CHANNEL)
for k in (-14, -6, 0, 6, 14):
    box(f"label_{k}", (X_FRONT - 1.7, k * PITCH, SHELF_Z - 6.5), (0.3, 38.0, 9.0), "shelf", LABEL)
box("rear_flange", (X_BACK - 0.6, 0.0, SHELF_Z - 6.5), (SHEET, 2 * HALF_LEN, 13.0), "shelf", POWDER)
for s in (1, -1):
    box(f"stand_{s}", ((X_FRONT + X_BACK) / 2, s * (HALF_LEN - 30.0), (SHELF_Z - SHEET) / 2),
        (X_BACK - X_FRONT - 10, 40.0, SHELF_Z - SHEET), "shelf", STAND)
    box(f"end_cap_{s}", ((X_FRONT + X_BACK) / 2, s * (HALF_LEN - 0.6), SHELF_Z - 6.5), (X_BACK - X_FRONT, SHEET, 13.0),
        "shelf", POWDER)
# 3 in front wire fence: top and bottom wires, vertical wires on 1 in centres between the holes
wire("fence_top", [(X_FENCE, -HALF_LEN + 4, SHELF_Z + FENCE_ZC), (X_FENCE, HALF_LEN - 4, SHELF_Z + FENCE_ZC)],
     FENCE_R, "shelf", CHROME)
wire("fence_bottom", [(X_FENCE, -HALF_LEN + 4, SHELF_Z + 6.0), (X_FENCE, HALF_LEN - 4, SHELF_Z + 6.0)], 1.5, "shelf",
     CHROME)
for k in range(-N_HOLES - 1, N_HOLES + 1):
    y = (k + 0.5) * PITCH
    if abs(y) < HALF_LEN - 6:
        wire(f"fence_v{k}", [(X_FENCE, y, SHELF_Z + 4.5), (X_FENCE, y, SHELF_Z + FENCE_ZC)], 1.5, "shelf", CHROME)
for s in (1, -1):
    divider(f"outer_{s}", s * K_OUTER * PITCH, SHELF_Z, "shelf")

# -- moving divider (part frame: y = 0 on its hole line, z = 0 on the deck top) -----------------------------------
divider("div", 0.0, 0.0, "divider")

for group, objs in GROUPS.items():
    bpy.ops.object.select_all(action="DESELECT")
    for o in objs:
        o.hide_set(False)
        o.select_set(True)
    bpy.ops.wm.usd_export(filepath=str(OUT / f"{group}.usdc"), selected_objects_only=True, export_materials=True,
                          generate_preview_surface=True, relative_paths=True, export_uvmaps=True, export_normals=True,
                          root_prim_path="/visual", convert_orientation=False, export_animation=False)
    print("exported", group, len(objs))

if PREVIEW:
    scene.render.engine = "CYCLES"
    scene.cycles.samples = 32
    scene.cycles.device = "CPU"
    scene.render.resolution_x, scene.render.resolution_y = 1280, 704
    world = bpy.data.worlds.new("w")
    world.use_nodes = True
    world.node_tree.nodes["Background"].inputs[0].default_value = (0.55, 0.56, 0.58, 1)
    world.node_tree.nodes["Background"].inputs[1].default_value = 0.9
    scene.world = world
    for o in GROUPS["divider"]:  # show the moving divider at hole +2
        o.location.y += 2 * PITCH * MM
        o.location.z += SHELF_Z * MM
    bpy.ops.mesh.primitive_plane_add(size=2.0, location=(0.3, 0, -0.0001))
    bpy.context.active_object.data.materials.append(material("bench", srgb((200, 180, 140)), 0.6))
    sun = bpy.data.objects.new("sun", bpy.data.lights.new("sun", "SUN"))
    sun.data.energy = 3.5
    sun.rotation_euler = (math.radians(35), math.radians(15), math.radians(30))
    scene.collection.objects.link(sun)
    cam = bpy.data.objects.new("cam", bpy.data.cameras.new("cam"))
    scene.collection.objects.link(cam)
    scene.camera = cam
    PREVIEW.mkdir(parents=True, exist_ok=True)
    for tag, loc, rot, lens in (("top", (0.28, 0.0, 0.9), (0, 0, math.radians(-90)), 35),
                                ("close", (0.02, 0.16, 0.22), (math.radians(58), 0, math.radians(-125)), 40)):
        cam.location, cam.rotation_euler, cam.data.lens = loc, rot, lens
        scene.render.filepath = str(PREVIEW / f"preview_{tag}.png")
        bpy.ops.render.render(write_still=True)
