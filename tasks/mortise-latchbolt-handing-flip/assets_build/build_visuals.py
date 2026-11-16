"""Author the visual meshes of the mortise lock, its latchbolt and the bench holder in Blender; one USD per rigid body.

Run: blender -b --factory-startup --python assets_build/build_visuals.py -- <pkg_dir> [preview_dir]

Every mesh is in its body's part frame (metres, Z up, z = 0 on the bench), so labkit references it under the body:
  lockcase.usdc (case + armored front + holder), latch_base.usdc (tail guide), latch_bolt.usdc (round shank),
  latch_head.usdc (beveled head with its anti-friction tongue), guide.usdc (linear guide on the bench) and
  jamb.usdc (jamb section with the strike, at its closed position).
Dimensions follow the frozen constants of mortise_latchbolt_handing_flip.py: case 113 x 154 x 25 mm (maker's
installation instructions), armored front 8 x 1-1/4 x 1/8 in with a 17.5 x 29.5 mm opening, head 16 x 28 mm standing
out 19 mm. Case details (hub with a square spindle hole at 2-3/4 in backset, case screws, rivets) follow the maker's
catalogue photos; no maker marks are applied.
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
HOLDER_FLOOR, CASE_DEPTH, CASE_LEN, CASE_T = 12.0, 113.0, 154.0, 25.0
FRONT_T, FRONT_W, FRONT_LEN = 3.2, 32.0, 203.0
FACE = HOLDER_FLOOR + CASE_DEPTH + FRONT_T  # 128.2
THROW, HEAD_T, HEAD_L, HEAD_IN = 19.0, 16.0, 28.0, 6.5
HOLE_T, HOLE_L = 17.5, 29.5
CHANNEL = THROW + HEAD_IN + 2.0
BEVEL_FLAT, LAND = 2.0, 3.5  # bevel from 2 mm above the faceplate to the land
BACKSET = 69.85  # 2-3/4 in: hub centre from the case front

bpy.ops.wm.read_factory_settings(use_empty=True)
scene = bpy.context.scene


def srgb(c):
    return tuple(((x / 255.0 + 0.055) / 1.055) ** 2.4 for x in c)


def material(name, color, rough, metal=0.0, coat=0.0):
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    bsdf = next(n for n in m.node_tree.nodes if n.type == "BSDF_PRINCIPLED")
    bsdf.inputs["Base Color"].default_value = (*color, 1.0)
    bsdf.inputs["Roughness"].default_value = rough
    bsdf.inputs["Metallic"].default_value = metal
    if coat and "Coat Weight" in bsdf.inputs:
        bsdf.inputs["Coat Weight"].default_value = coat
    return m


ZINC = material("case_zinc_plated_steel", srgb((168, 170, 168)), 0.42, metal=0.85)
SS_626 = material("satin_chrome_626", srgb((196, 198, 200)), 0.30, metal=1.0)
SS_BRUSH = material("brushed_stainless_630", srgb((186, 188, 190)), 0.26, metal=1.0)
STEEL_DARK = material("black_oxide_steel", srgb((48, 49, 52)), 0.45, metal=0.7)
ALU = material("anodised_aluminium", srgb((150, 152, 156)), 0.38, metal=0.9)
RUBBER = material("black_rubber_pad", srgb((26, 26, 28)), 0.85)
HUB = material("hub_sintered_steel", srgb((120, 121, 122)), 0.5, metal=0.8)
OAK = material("oak_jamb_satin", srgb((176, 128, 82)), 0.55, coat=0.2)
POCKET = material("strike_box_dark", srgb((30, 30, 32)), 0.7, metal=0.3)

GROUPS: dict[str, list] = {k: [] for k in ("lockcase", "latch_base", "latch_bolt", "latch_head", "guide", "jamb")}


def finish(obj, group, mat, bevel=0.0, smooth=False):
    obj.data.materials.clear()
    obj.data.materials.append(mat)
    if bevel:
        mod = obj.modifiers.new("bevel", "BEVEL")
        mod.width = bevel * MM
        mod.segments = 3
        mod.limit_method = "ANGLE"
        bpy.context.view_layer.objects.active = obj
        bpy.ops.object.modifier_apply(modifier=mod.name)
    for p in obj.data.polygons:
        p.use_smooth = smooth
    GROUPS[group].append(obj)
    return obj


def box(name, center, size, group, mat, bevel=0.4):
    bpy.ops.mesh.primitive_cube_add(size=1.0, location=tuple(c * MM for c in center))
    obj = bpy.context.active_object
    obj.name = name
    obj.scale = tuple(s * MM for s in size)
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    return finish(obj, group, mat, bevel)


def cyl(name, center, r, h, group, mat, axis="Z", seg=48, bevel=0.3):
    bpy.ops.mesh.primitive_cylinder_add(vertices=seg, radius=r * MM, depth=h * MM, location=tuple(c * MM for c in center))
    obj = bpy.context.active_object
    obj.name = name
    if axis == "X":
        obj.rotation_euler[1] = math.pi / 2
    elif axis == "Y":
        obj.rotation_euler[0] = math.pi / 2
    bpy.ops.object.transform_apply(location=False, rotation=True, scale=False)
    return finish(obj, group, mat, bevel, smooth=True)


def cut(target, cutter):
    mod = target.modifiers.new("cut", "BOOLEAN")
    mod.operation = "DIFFERENCE"
    mod.object = cutter
    bpy.context.view_layer.objects.active = target
    bpy.ops.object.modifier_apply(modifier=mod.name)
    for g in GROUPS.values():
        if cutter in g:
            g.remove(cutter)
    bpy.data.objects.remove(cutter, do_unlink=True)


def prism(name, profile_xz, y0, y1, group, mat, bevel=0.35):
    """Extrude a closed (x, z) mm polygon along Y from y0 to y1 mm."""
    me = bpy.data.meshes.new(name)
    bm = bmesh.new()
    a = [bm.verts.new((x * MM, y0 * MM, z * MM)) for x, z in profile_xz]
    b = [bm.verts.new((x * MM, y1 * MM, z * MM)) for x, z in profile_xz]
    bm.faces.new(a)
    bm.faces.new(list(reversed(b)))
    n = len(a)
    for i in range(n):
        j = (i + 1) % n
        bm.faces.new((a[j], a[i], b[i], b[j]))
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    bm.to_mesh(me)
    bm.free()
    obj = bpy.data.objects.new(name, me)
    scene.collection.objects.link(obj)
    return finish(obj, group, mat, bevel)


# -- lock case (part frame: holder centre on the bench) -------------------------------------------------------------
zc = FACE - FRONT_T  # case front face
case = box("case_body", (0, 0, (HOLDER_FLOOR + zc) / 2), (CASE_T, CASE_LEN, zc - HOLDER_FLOOR), "lockcase", ZINC, 0.8)
slot = box("tmp_channel", (0, 0, zc - (CHANNEL - FRONT_T) / 2 + 0.5), (HOLE_T, HOLE_L, CHANNEL - FRONT_T + 1.0),
           "lockcase", ZINC, 0.0)
cut(case, slot)
hub_z = zc - BACKSET
for sx in (1, -1):
    cyl(f"hub_{sx}", (sx * (CASE_T / 2 + 0.4), 0, hub_z), 14.3, 1.6, "lockcase", HUB, axis="X")
    box(f"spindle_hole_{sx}", (sx * (CASE_T / 2 + 1.25), 0, hub_z), (0.4, 7.9, 7.9), "lockcase", STEEL_DARK, 0.0)
    for k, (yy, zz) in enumerate(((-62, 20), (62, 20), (-62, 100), (62, 100), (-30, 50), (35, 30))):
        cyl(f"rivet_{sx}_{k}", (sx * (CASE_T / 2 + 0.3), yy, zz), 2.6 if k < 4 else 2.1, 0.8, "lockcase", ZINC,
            axis="X", seg=24, bevel=0.2)
    # mounting tabs at the case ends (front side)
    box(f"tab_{sx}", (0, sx * (CASE_LEN / 2 - 4), zc - 6), (CASE_T - 6, 8, 12), "lockcase", ZINC, 0.4)
# case front plate under the armored front
box("case_front", (0, 0, zc - 1.0), (CASE_T, CASE_LEN + 2, 2.0), "lockcase", ZINC, 0.3)
front = box("armored_front", (0, 0, FACE - FRONT_T / 2), (FRONT_W, FRONT_LEN, FRONT_T), "lockcase", SS_BRUSH, 0.5)
cut(front, box("tmp_hole", (0, 0, FACE - FRONT_T / 2), (HOLE_T, HOLE_L, FRONT_T + 2), "lockcase", SS_BRUSH, 0.0))
for sy in (1, -1):  # countersunk armored-front screws (Phillips)
    y = sy * (FRONT_LEN / 2 - 9.5)
    cyl(f"front_screw_{sy}", (0, y, FACE + 0.15), 3.6, 0.6, "lockcase", SS_626, seg=32, bevel=0.25)
    box(f"front_screw_x_{sy}a", (0, y, FACE + 0.46), (4.2, 0.9, 0.2), "lockcase", STEEL_DARK, 0.0)
    box(f"front_screw_x_{sy}b", (0, y, FACE + 0.46), (0.9, 4.2, 0.2), "lockcase", STEEL_DARK, 0.0)
# inner guide seen through the opening (dark)
box("guide_floor", (0, 0, FACE - CHANNEL + 0.6), (HOLE_T - 0.2, HOLE_L - 0.2, 1.2), "lockcase", STEEL_DARK, 0.0)

# padded bench holder: anodised base plate, two cheeks, rubber pads on the inner faces
box("holder_base", (0, 0, HOLDER_FLOOR / 2), (80, 190, HOLDER_FLOOR), "lockcase", ALU, 1.2)
for sx in (1, -1):
    box(f"holder_cheek_{sx}", (sx * (CASE_T / 2 + 9.0), 0, HOLDER_FLOOR + 22.5), (14.0, 130, 45), "lockcase", ALU, 1.0)
    box(f"holder_pad_{sx}", (sx * (CASE_T / 2 + 1.0), 0, HOLDER_FLOOR + 21.0), (2.0, 120, 40), "lockcase", RUBBER, 0.5)
    for sy in (1, -1):
        cyl(f"holder_cap_screw_{sx}{sy}", (sx * (CASE_T / 2 + 9.0), sy * 50, HOLDER_FLOOR + 45.3), 4.2, 0.8,
            "lockcase", STEEL_DARK, seg=24, bevel=0.2)

# -- latchbolt ----------------------------------------------------------------------------------------------------
box("tail_guide", (0, 0, FACE - CHANNEL + 0.7), (4.0, 4.0, 1.4), "latch_base", STEEL_DARK, 0.2)
cyl("shank", (0, 0, FACE - 3.0), 5.0, 6.0, "latch_bolt", SS_626, seg=32)
# head profile (x, z rel. FACE): back face at x = -8 full height, 3.5 mm land, bevel down to 6 mm on the +x side
t = HEAD_T / 2
xl = -t + LAND
profile = [(-t, -HEAD_IN), (t, -HEAD_IN), (t, BEVEL_FLAT), (xl, THROW), (-t, THROW)]
head = prism("head", [(x, FACE + z) for x, z in profile], -HEAD_L / 2, HEAD_L / 2, "latch_head", SS_626, 0.45)
# anti-friction tongue: a narrow steel lever in a slot along the bevel face
ang = math.atan2(THROW - BEVEL_FLAT, t - xl)
mx, mz = (t + xl) / 2, (BEVEL_FLAT + THROW) / 2
slot_c = box("tmp_tongue_slot", (mx, 0, FACE + mz), (3.0, 7.0, 15.0), "latch_head", SS_626, 0.0)
slot_c.rotation_euler[1] = -(math.pi / 2 - ang)
cut(head, slot_c)
tongue = box("tongue", (mx - 0.4, 0, FACE + mz - 0.6), (2.0, 6.4, 14.0), "latch_head", STEEL_DARK, 0.3)
tongue.rotation_euler[1] = -(math.pi / 2 - ang)
bpy.context.view_layer.objects.active = tongue
bpy.ops.object.select_all(action="DESELECT")
tongue.select_set(True)
bpy.ops.object.transform_apply(location=False, rotation=True, scale=False)

# -- strike side (frozen constants of the task file, mm, part frame at the closed position) -----------------------------
JZ0, JH, ST = FACE + 3.0, 25.0, 1.6  # jamb face 3 mm above the faceplate, jamb depth, strike plate
OX0, OX1, LIP, JX0, OY = -11.0, 9.0, 34.0, -36.0, 16.0
JY0, JY1, SY = -75.0, 185.0, 62.0
jamb = box("jamb_oak", ((JX0 + LIP) / 2, (JY0 + JY1) / 2, JZ0 + JH / 2), (LIP - JX0, JY1 - JY0, JH), "jamb", OAK, 0.8)
cut(jamb, box("tmp_strike_mortise", ((OX0 - 10 + LIP) / 2, 0, JZ0 + ST / 2 - 0.5), (LIP - OX0 + 10 + 0.2, 2 * SY, ST + 1.0),
              "jamb", OAK, 0.0))
cut(jamb, box("tmp_pocket", ((OX0 + OX1) / 2, 0, JZ0 + (JH - 3.0) / 2 - 0.5), (OX1 - OX0, 2 * OY, JH - 3.0 + 1.0), "jamb", OAK, 0.0))
strike = box("strike_plate", ((OX0 - 10 + LIP) / 2, 0, JZ0 + ST / 2), (LIP - OX0 + 10, 2 * SY, ST), "jamb", SS_BRUSH, 0.3)
cut(strike, box("tmp_strike_hole", ((OX0 + OX1) / 2, 0, JZ0 + ST / 2), (OX1 - OX0, 2 * OY, ST + 2), "jamb", SS_BRUSH, 0.0))
# strike box (dust box) lining the pocket, dark
for sx in (OX0 + 0.4, OX1 - 0.4):
    box(f"box_wall_x{sx:+.0f}", (sx, 0, JZ0 + ST + (JH - 3.0 - ST) / 2), (0.8, 2 * OY, JH - 3.0 - ST), "jamb", POCKET, 0.0)
for sy in (OY - 0.4, -OY + 0.4):
    box(f"box_wall_y{sy:+.0f}", ((OX0 + OX1) / 2, sy, JZ0 + ST + (JH - 3.0 - ST) / 2), (OX1 - OX0, 0.8, JH - 3.0 - ST), "jamb", POCKET, 0.0)
box("box_roof", ((OX0 + OX1) / 2, 0, JZ0 + JH - 3.4), (OX1 - OX0, 2 * OY, 0.8), "jamb", POCKET, 0.0)
for sy in (1, -1):  # countersunk strike screws
    y = sy * (SY - 10.0)
    cyl(f"strike_screw_{sy}", (4.0, y, JZ0 - 0.15), 4.2, 0.6, "jamb", SS_626, seg=32, bevel=0.25)
    box(f"strike_screw_x_{sy}a", (4.0, y, JZ0 - 0.46), (5.0, 0.9, 0.2), "jamb", STEEL_DARK, 0.0)
    box(f"strike_screw_x_{sy}b", (4.0, y, JZ0 - 0.46), (0.9, 5.0, 0.2), "jamb", STEEL_DARK, 0.0)
# push block on the back end (over the guide) and the guide carriage under the +Y end
box("push_block", (JX0 + 15.0, 170.0, JZ0 + JH + 10.0), (30.0, 50.0, 20.0), "jamb", OAK, 1.2)
box("guide_carriage", ((JX0 + LIP) / 2, 170.0, JZ0 - 0.7), (LIP - JX0 - 6, 26.0, 1.4), "jamb", ALU, 0.3)
# linear guide on the bench (fixed body)
box("guide_rail", (-60.0, 170.0, (JZ0 - 1.5) / 2), (200.0, 20.0, JZ0 - 1.5), "guide", ALU, 0.8)
box("guide_foot", (-60.0, 170.0, 4.0), (220.0, 60.0, 8.0), "guide", ALU, 1.0)
box("guide_stop", (-60.0 + 100.0 - 3.0, 170.0, JZ0 - 1.5 + 4.0), (6.0, 20.0, 8.0), "guide", RUBBER, 0.6)
for k, xx in enumerate((-150.0, -60.0, 30.0)):
    for sy in (1, -1):
        cyl(f"foot_screw_{k}_{sy}", (xx, 170.0 + sy * 22.0, 8.3), 3.2, 0.6, "guide", STEEL_DARK, seg=24, bevel=0.2)

# -- export -------------------------------------------------------------------------------------------------------
for group, objs in GROUPS.items():
    bpy.ops.object.select_all(action="DESELECT")
    for o in objs:
        o.select_set(True)
    bpy.ops.wm.usd_export(filepath=str(OUT / f"{group}.usdc"), selected_objects_only=True, export_materials=True,
                          generate_preview_surface=True, relative_paths=True, export_uvmaps=True, export_normals=True,
                          root_prim_path="/visual", convert_orientation=False, export_animation=False)
    print("exported", group, len(objs))

if PREVIEW:
    # place the latch at its start (bevel +x) beside a copy turned 180 deg, light + camera, render
    scene.render.engine = "CYCLES"
    scene.cycles.samples = 48
    scene.cycles.device = "CPU"
    scene.render.resolution_x, scene.render.resolution_y = 1280, 704
    world = bpy.data.worlds.new("w")
    world.use_nodes = True
    world.node_tree.nodes["Background"].inputs[0].default_value = (0.55, 0.56, 0.58, 1)
    world.node_tree.nodes["Background"].inputs[1].default_value = 0.9
    scene.world = world
    bpy.ops.mesh.primitive_plane_add(size=2.0, location=(0, 0, -0.0001))
    bpy.context.active_object.data.materials.append(material("bench", srgb((200, 180, 140)), 0.6))
    sun = bpy.data.objects.new("sun", bpy.data.lights.new("sun", "SUN"))
    sun.data.energy = 3.5
    sun.rotation_euler = (math.radians(35), math.radians(15), math.radians(30))
    scene.collection.objects.link(sun)
    cam = bpy.data.objects.new("cam", bpy.data.cameras.new("cam"))
    scene.collection.objects.link(cam)
    scene.camera = cam
    PREVIEW.mkdir(parents=True, exist_ok=True)
    for tag, loc, rot, lens in (("top", (0.0, 0.0, 0.55), (0, 0, math.radians(-90)), 50),
                                ("persp", (0.16, -0.20, 0.27), (math.radians(58), 0, math.radians(38)), 60),
                                ("close", (0.06, -0.07, 0.19), (math.radians(52), 0, math.radians(40)), 80)):
        cam.location, cam.rotation_euler, cam.data.lens = loc, rot, lens
        scene.render.filepath = str(PREVIEW / f"preview_{tag}.png")
        bpy.ops.render.render(write_still=True)
