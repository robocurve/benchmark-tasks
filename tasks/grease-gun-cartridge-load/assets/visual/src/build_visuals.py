"""Author the grease-gun visual meshes in Blender and export one USD per part body (part-frame coordinates, Z up, metres).

    blender -b --factory-startup --python build_visuals.py -- OUT_DIR [PREVIEW_PNG]

Geometry follows the collision shapes in grease_gun_cartridge_load.py (barrel 56 mm OD / 52 mm bore x 250 mm, keyhole end
cap, follower rod with neck and T-handle, 50 x 240 mm 14 oz grease cartridge, rubber bench holder with loading V) so
contacts look right. Authored to real-product size (README); procedural PBR materials plus tex/*.png from
make_textures.py.
"""

import math
import sys
from pathlib import Path

import bmesh
import bpy
from mathutils import Vector

argv = sys.argv[sys.argv.index("--") + 1:]
OUT = Path(argv[0]).resolve()
PREVIEW = argv[1] if len(argv) > 1 else None
TEX = OUT / "tex"

bpy.ops.wm.read_factory_settings(use_empty=True)
scene = bpy.context.scene
COLL = {}
def coll(name):
    if name not in COLL:
        c = bpy.data.collections.new(name)
        scene.collection.children.link(c)
        COLL[name] = c
    return COLL[name]


# -- materials --------------------------------------------------------------------------------------------------------
MATS = {}


def mat(name, color=(0.8, 0.8, 0.8), rough=0.5, metal=0.0, tex=None, rough_tex=None, glass=False, alpha=1.0):
    if name in MATS:
        return MATS[name]
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    nt = m.node_tree
    p = next(n for n in nt.nodes if n.type == "BSDF_PRINCIPLED")
    p.inputs["Base Color"].default_value = (*color, 1)
    p.inputs["Roughness"].default_value = rough
    p.inputs["Metallic"].default_value = metal
    if glass:
        p.inputs["Transmission Weight"].default_value = 1.0
        p.inputs["IOR"].default_value = 1.58
        p.inputs["Roughness"].default_value = 0.02
    if alpha < 1:
        p.inputs["Alpha"].default_value = alpha
    if tex:
        t = nt.nodes.new("ShaderNodeTexImage")
        t.image = bpy.data.images.load(str(TEX / tex))
        nt.links.new(t.outputs["Color"], p.inputs["Base Color"])
    if rough_tex:
        t = nt.nodes.new("ShaderNodeTexImage")
        t.image = bpy.data.images.load(str(TEX / rough_tex))
        t.image.colorspace_settings.name = "Non-Color"
        nt.links.new(t.outputs["Color"], p.inputs["Roughness"])
    MATS[name] = m
    return m


def finish(obj, material, group, bevel=0.0, segs=3, smooth=True, uv=True):
    obj.data.materials.append(MATS[material])
    coll(group).objects.link(obj)
    if bevel > 0:
        m = obj.modifiers.new("bevel", "BEVEL")
        m.width = bevel
        m.segments = segs
        m.limit_method = "ANGLE"
    if uv:
        bpy.context.view_layer.objects.active = obj
        for o in bpy.context.selected_objects:
            o.select_set(False)
        obj.select_set(True)
        bpy.ops.object.mode_set(mode="EDIT")
        bpy.ops.mesh.select_all(action="SELECT")
        bpy.ops.uv.cube_project(cube_size=0.12)
        bpy.ops.object.mode_set(mode="OBJECT")
    if smooth:
        for p in obj.data.polygons:
            p.use_smooth = True
        obj.modifiers.new("ws", "WEIGHTED_NORMAL").keep_sharp = True
    return obj


def new_obj(name, bm):
    me = bpy.data.meshes.new(name)
    bm.to_mesh(me)
    bm.free()
    return bpy.data.objects.new(name, me)


def box(name, c, s, material, group, bevel=0.0015, segs=3):
    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=1.0)
    for v in bm.verts:
        v.co = Vector((v.co.x * s[0] + c[0], v.co.y * s[1] + c[1], v.co.z * s[2] + c[2]))
    return finish(new_obj(name, bm), material, group, bevel, segs)


def cyl(name, c, r, h, material, group, axis="Z", segs=48, bevel=0.0008):
    bm = bmesh.new()
    bmesh.ops.create_cone(bm, cap_ends=True, segments=segs, radius1=r, radius2=r, depth=h)
    rot = {"Z": None, "X": ("Y", math.pi / 2), "Y": ("X", math.pi / 2)}[axis]
    obj = new_obj(name, bm)
    if rot:
        obj.rotation_euler = {"X": (rot[1], 0, 0), "Y": (0, rot[1], 0)}[rot[0]]
    obj.location = c
    return finish(obj, material, group, bevel, 2)


def cut(obj, cutters):
    for c in cutters:
        m = obj.modifiers.new("cut", "BOOLEAN")
        m.operation = "DIFFERENCE"
        m.solver = "EXACT"
        m.object = c
        c.hide_render = True
        c.hide_viewport = True
    # boolean before bevel/normals
    order = [m for m in obj.modifiers if m.type == "BOOLEAN"] + [m for m in obj.modifiers if m.type != "BOOLEAN"]
    for i, m in enumerate(order):
        bpy.context.view_layer.objects.active = obj
        bpy.ops.object.modifier_move_to_index(modifier=m.name, index=i)
    return obj


def cutter(c, s):
    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=1.0)
    for v in bm.verts:
        v.co = Vector((v.co.x * s[0] + c[0], v.co.y * s[1] + c[1], v.co.z * s[2] + c[2]))
    o = new_obj("cutter", bm)
    coll("_cutters").objects.link(o)
    return o


def rrect(w, h, r, n=8):
    """Rounded-rectangle outline (counter-clockwise), 4 * (n + 1) points."""
    pts = []
    for cx, cy, a0 in ((w / 2 - r, h / 2 - r, 0), (-w / 2 + r, h / 2 - r, 90), (-w / 2 + r, -h / 2 + r, 180),
                       (w / 2 - r, -h / 2 + r, 270)):
        for i in range(n + 1):
            a = math.radians(a0 + 90 * i / n)
            pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
    return pts


def loft(name, rings, material, group, cap_bottom=True, cap_top=False, smooth=True):
    """Rings: list of (points, z, dx); consecutive rings bridged with quads."""
    bm = bmesh.new()
    vs = [[bm.verts.new((x + dx, y, z)) for x, y in pts] for pts, z, dx in rings]
    for a, b in zip(vs, vs[1:]):
        n = len(a)
        for i in range(n):
            bm.faces.new((a[i], a[(i + 1) % n], b[(i + 1) % n], b[i]))
    if cap_bottom:
        bm.faces.new(list(reversed(vs[0])))
    if cap_top:
        bm.faces.new(vs[-1])
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    return finish(new_obj(name, bm), material, group, 0.0, 2, smooth)


def rod(name, a, b, r, material, group, segs=12):
    a, b = Vector(a), Vector(b)
    o = cyl(name, (a + b) / 2, r, (b - a).length, material, group, segs=segs, bevel=0)
    o.rotation_mode = "QUATERNION"
    o.rotation_quaternion = Vector((0, 0, 1)).rotation_difference(b - a)
    return o


# -- grease-gun specific helpers (axis along +Y) ------------------------------------------------------------------------


def rev_y(name, profile, material, group, segs=64, uv_label=False, a0=0.0, a1=2 * math.pi, closed=True):
    """Surface of revolution about the Y axis: profile = [(r, y), ...] (open polyline, revolved)."""
    bm = bmesh.new()
    n = segs if closed else segs + 1
    rings = []
    for r, y in profile:
        ring_v = []
        for k in range(n):
            a = a0 + (a1 - a0) * k / segs
            ring_v.append(bm.verts.new((r * math.cos(a), y, r * math.sin(a))))
        rings.append(ring_v)
    uv_layer = bm.loops.layers.uv.new("UVMap")
    faces = []
    for i, (ra, rb) in enumerate(zip(rings, rings[1:])):
        for k in range(segs):
            k2 = (k + 1) % n if closed else k + 1
            f = bm.faces.new((ra[k], ra[k2], rb[k2], rb[k]))
            faces.append((f, i, k))
    if uv_label:
        y0, y1 = profile[0][1], profile[-1][1]
        for f, i, k in faces:
            for loop in f.loops:
                v = loop.vert.co
                a = math.atan2(v.z, v.x) % (2 * math.pi)
                u = a / (2 * math.pi)
                if k == segs - 1 and u < 0.5:
                    u += 1.0
                loop[uv_layer].uv = (u, (v.y - y0) / (y1 - y0))
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    return finish(new_obj(name, bm), material, group, 0.0, 2, True, uv=not uv_label)


def tube_y(name, r_out, r_in, y0, y1, material, group, segs=64):
    return rev_y(name, [(r_in, y0), (r_out, y0), (r_out, y1), (r_in, y1), (r_in, y0)], material, group, segs)


def solid_y(name, r, y0, y1, material, group, segs=48):
    return rev_y(name, [(0.0, y0), (r, y0), (r, y1), (0.0, y1)], material, group, segs)


def solid_x(name, r, x0, x1, c, material, group, segs=32):
    o = cyl(name, ((x0 + x1) / 2, c[1], c[2]), r, x1 - x0, material, group, axis="X", segs=segs, bevel=0.0006)
    return o


def prism_y(name, poly_xz, y0, y1, material, group, bevel=0.0015):
    """Extrude a convex-or-not polygon in the XZ plane along Y."""
    bm = bmesh.new()
    a = [bm.verts.new((x, y0, z)) for x, z in poly_xz]
    b = [bm.verts.new((x, y1, z)) for x, z in poly_xz]
    n = len(poly_xz)
    for i in range(n):
        bm.faces.new((a[i], a[(i + 1) % n], b[(i + 1) % n], b[i]))
    bm.faces.new(list(reversed(a)))
    bm.faces.new(b)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    return finish(new_obj(name, bm), material, group, bevel, 2, False)


def vblock_poly(r, axis_z, floor_z, top_z, half_w):
    """V-block section: faces tangent to a circle of radius r (centre at x 0, z axis_z) 45 deg either side of the bottom."""
    tx, tz = r * math.sin(math.pi / 4), axis_z - r * math.cos(math.pi / 4)
    x_top = tx + (top_z - tz)  # slope 1 lines through the tangent points
    x_bot = max(0.004, tx + (floor_z + 0.003 - tz))
    return [(-half_w, floor_z), (half_w, floor_z), (half_w, top_z), (x_top, top_z), (x_bot, floor_z + 0.003),
            (-x_bot, floor_z + 0.003), (-x_top, top_z), (-half_w, top_z)]


# -- materials --------------------------------------------------------------------------------------------------------


def materials():
    mat("zinc", (0.78, 0.79, 0.80), 0.28, 0.9, "steel_albedo.png")
    mat("steel_bare", (0.70, 0.70, 0.71), 0.22, 1.0)
    mat("steel_dark", (0.22, 0.23, 0.25), 0.35, 0.8)
    mat("alu", (0.80, 0.81, 0.83), 0.35, 0.85)
    mat("rubber", (0.045, 0.045, 0.05), 0.85)
    mat("grip_red", (0.70, 0.06, 0.04), 0.45)
    mat("leather", (0.32, 0.20, 0.11), 0.75)
    mat("label", (1, 1, 1), 0.45, tex="cartridge_label.png")
    mat("tin", (0.82, 0.82, 0.80), 0.25, 1.0)
    mat("grease", (0.62, 0.38, 0.10), 0.18)
    mat("mat_rubber", (0.09, 0.09, 0.10), 0.8, tex="mat_albedo.png", rough_tex="mat_rough.png")
    mat("rag", (0.6, 0.16, 0.14), 0.95, tex="rag_albedo.png")
    mat("hose_black", (0.03, 0.03, 0.035), 0.55)
    mat("brass", (0.78, 0.60, 0.25), 0.3, 1.0)
    mat("head_paint", (0.10, 0.11, 0.12), 0.4, 0.3)


# geometry constants (match grease_gun_cartridge_load.py)
AXIS_H = 0.036
MOUTH_Y, CAP_IN_Y, CAP_OUT_Y = 0.125, -0.125, -0.128
BORE_R, BARREL_R, COLLAR_R = 0.026, 0.028, 0.031
TBAR_Y = CAP_OUT_Y - 0.025 - 0.006
NECK_Y0, NECK_Y1 = 0.109, 0.118
CART_HALF = 0.120
FLOOR = -AXIS_H  # mat top in the barrel frame


def build_gun():
    g = "gun"
    tube_y("barrel", BARREL_R, BORE_R, CAP_IN_Y, MOUTH_Y - 0.012, "zinc", g)
    # threaded mouth (the head screws on here)
    prof = [(BORE_R, MOUTH_Y - 0.012)]
    for k in range(6):
        y = MOUTH_Y - 0.012 + k * 0.002
        prof += [(0.0277, y), (0.0272, y + 0.001)]
    prof += [(0.0277, MOUTH_Y), (BORE_R, MOUTH_Y), (BORE_R, MOUTH_Y - 0.012)]
    rev_y("thread", prof, "steel_bare", g)
    # end cap: knurled collar + keyhole plate
    tube_y("collar", COLLAR_R, BARREL_R + 0.0002, CAP_OUT_Y, -0.110, "steel_dark", g, segs=48)
    plate = solid_y("cap_plate", COLLAR_R, CAP_OUT_Y, CAP_IN_Y, "steel_dark", g)
    hole = solid_y("hole_cut", 0.008, CAP_OUT_Y - 0.002, CAP_IN_Y + 0.002, "steel_dark", "_cutters")
    hole.hide_render = True
    cut(plate, [hole, cutter((0.0, (CAP_OUT_Y + CAP_IN_Y) / 2, -0.0083), (0.0065, 0.007, 0.0165))])
    # holder: aluminium rail, rubber V-blocks under the barrel and the loading V in front of the mouth
    prism_y("rail", [(-0.036, FLOOR), (0.036, FLOOR), (0.036, FLOOR + 0.004), (-0.036, FLOOR + 0.004)], -0.115, 0.335,
            "alu", g, 0.001)
    for y0, y1 in ((-0.1085, -0.0785), (0.060, 0.090)):
        prism_y(f"vblock_{y0:+.3f}", vblock_poly(BARREL_R, 0.0, FLOOR + 0.004, -0.010, 0.035), y0, y1, "rubber", g)
    for y0, y1 in ((0.135, 0.165), (0.300, 0.330)):
        prism_y(f"trough_{y0:+.3f}", vblock_poly(0.025, -0.001, FLOOR + 0.004, -0.012, 0.035), y0, y1, "rubber", g)
    # clamp strap over the barrel on the rear block, with its two bolts
    rev_y("strap", [(BARREL_R + 0.0002, -0.100), (BARREL_R + 0.0022, -0.100), (BARREL_R + 0.0022, -0.088),
                    (BARREL_R + 0.0002, -0.088), (BARREL_R + 0.0002, -0.100)], "steel_bare", g, segs=32,
          a0=math.radians(-25), a1=math.radians(205), closed=False)
    for sx in (-1, 1):
        box(f"strap_ear_{sx}", (sx * 0.033, -0.094, -0.0115), (0.010, 0.012, 0.002), "steel_bare", g, 0.0005)
        cyl(f"bolt_{sx}", (sx * 0.033, -0.094, -0.009), 0.0035, 0.004, "steel_bare", g, segs=6, bevel=0.0004)


def build_follower():
    g = "follower"
    rev_y("follower_cup", [(0.0, 0.118), (0.0225, 0.118), (0.0235, 0.120), (0.0235, 0.124), (0.0, 0.124)], "leather", g)


def build_rod():
    g = "rod"
    solid_y("rod", 0.005, TBAR_Y, NECK_Y0, "steel_bare", g, segs=24)
    solid_y("neck", 0.0025, NECK_Y0, NECK_Y1, "steel_bare", g, segs=16)
    solid_x("tbar", 0.006, -0.050, 0.050, (0.0, TBAR_Y, 0.0), "steel_bare", g)
    for sx in (-1, 1):
        solid_x(f"grip_{sx}", 0.0068, min(sx * 0.010, sx * 0.050), max(sx * 0.010, sx * 0.050), (0.0, TBAR_Y, 0.0),
                "grip_red", g)


def build_cartridge():
    g = "cartridge"
    rev_y("label", [(0.0249, -CART_HALF + 0.003), (0.0249, CART_HALF - 0.003)], "label", g, segs=72, uv_label=True)
    for s in (-1, 1):  # rolled tinplate end rims
        y = s * (CART_HALF - 0.0015)
        rev_y(f"rim_{s}", [(0.0240, y - 0.0015), (0.0250, y - 0.0015), (0.0251, y), (0.0250, y + 0.0015),
                           (0.0240, y + 0.0015)], "tin", g, segs=72)
    rev_y("grease_face", [(0.0, -CART_HALF + 0.003), (0.0240, -CART_HALF + 0.003)], "grease", g, segs=48)  # open end
    rev_y("lid", [(0.0240, CART_HALF - 0.001), (0.0, CART_HALF - 0.0005)], "tin", g, segs=48)  # tab-seal lid
    rod("pull_tab", (0.0, CART_HALF - 0.0002, -0.006), (0.0, CART_HALF - 0.0002, 0.010), 0.0018, "tin", g, segs=8)


def build_rest():
    g = "rest"
    for yc in (-0.080, 0.080):
        prism_y(f"rest_{yc:+.3f}", vblock_poly(0.025, 0.032, 0.0, 0.020, 0.030), yc - 0.015, yc + 0.015, "rubber", g)


def build_mat():
    g = "mat"
    o = loft("mat", [(rrect(0.560, 0.640, 0.015), 0.0, 0), (rrect(0.560, 0.640, 0.015), 0.002, 0)], "mat_rubber", g,
             cap_bottom=True, cap_top=True)
    bpy.context.view_layer.objects.active = o
    for x in bpy.context.selected_objects:
        x.select_set(False)
    o.select_set(True)
    bpy.ops.object.mode_set(mode="EDIT")
    bpy.ops.mesh.select_all(action="SELECT")
    bpy.ops.uv.cube_project(cube_size=0.3)
    bpy.ops.object.mode_set(mode="OBJECT")


def build_props():
    """Render-only bench props in the mat frame (origin at the mat centre, mat top z 0.002): the unscrewed lever-gun head
    with its hose, and a shop rag. No colliders."""
    g = "bench_props"
    hx, hy, z0 = 0.215, 0.110, 0.002
    # lever-gun head lying on its side: threaded socket (56 mm), pump body, lever, then the hose and coupler
    cyl("head_socket", (hx, hy, z0 + 0.032), 0.032, 0.024, "head_paint", g, axis="X", segs=48)
    box("head_body", (hx, hy - 0.050, z0 + 0.022), (0.026, 0.070, 0.040), "head_paint", g, 0.004)
    box("pump_boss", (hx, hy - 0.095, z0 + 0.018), (0.022, 0.030, 0.030), "head_paint", g, 0.003)
    lever = box("lever", (hx + 0.006, hy - 0.020, z0 + 0.050), (0.012, 0.200, 0.008), "steel_dark", g, 0.002)
    lever.rotation_euler = (math.radians(-8), 0, 0)
    box("lever_grip", (hx + 0.006, hy + 0.060, z0 + 0.058), (0.018, 0.080, 0.012), "grip_red", g, 0.004)
    pts = [(hx, hy - 0.110, z0 + 0.010), (hx - 0.020, hy - 0.180, z0 + 0.006), (hx - 0.010, hy - 0.260, z0 + 0.006),
           (hx + 0.030, hy - 0.300, z0 + 0.006), (hx + 0.060, hy - 0.280, z0 + 0.006)]
    cu = bpy.data.curves.new("hose", "CURVE")
    cu.dimensions = "3D"
    sp = cu.splines.new("BEZIER")
    sp.bezier_points.add(len(pts) - 1)
    for bp, p in zip(sp.bezier_points, pts):
        bp.co = p
        bp.handle_left_type = bp.handle_right_type = "AUTO"
    cu.bevel_depth = 0.0055
    cu.bevel_resolution = 4
    hose = bpy.data.objects.new("hose", cu)
    hose.data.materials.append(MATS["hose_black"])
    coll(g).objects.link(hose)
    bpy.context.view_layer.objects.active = hose
    for o in bpy.context.selected_objects:
        o.select_set(False)
    hose.select_set(True)
    bpy.ops.object.convert(target="MESH")
    cyl("coupler", (hx + 0.072, hy - 0.272, z0 + 0.007), 0.0075, 0.030, "brass", g, axis="X", segs=6)
    # shop rag: a folded cotton cloth
    bpy.ops.mesh.primitive_grid_add(x_subdivisions=40, y_subdivisions=40, size=1.0)
    rag = bpy.context.object
    rag.name = "rag"
    rag.scale = (0.17, 0.13, 1.0)
    bpy.ops.object.transform_apply(scale=True)
    import random
    rnd = random.Random(5)
    for v in rag.data.vertices:
        x, y = v.co.x, v.co.y
        v.co.z = 0.003 + 0.004 * (math.sin(x * 60 + rnd.random()) * math.cos(y * 45)) ** 2 + 0.0015 * rnd.random()
    rag.location = (0.225, -0.185, z0)
    rag.rotation_euler = (0, 0, math.radians(18))
    for c in rag.users_collection:
        c.objects.unlink(rag)
    finish(rag, "rag", g, 0.0, 2, True, uv=True)
    rag.modifiers.new("solid", "SOLIDIFY").thickness = 0.0015


materials()
build_gun()
build_follower()
build_rod()
build_cartridge()
build_rest()
build_mat()
build_props()

for name in ("gun", "follower", "rod", "cartridge", "rest", "mat", "bench_props"):
    for o in bpy.context.selected_objects:
        o.select_set(False)
    for o in COLL[name].objects:
        o.select_set(True)
    bpy.ops.wm.usd_export(filepath=str(OUT / f"{name}.usda"), selected_objects_only=True, export_materials=True,
                          generate_preview_surface=True, export_textures_mode="KEEP", relative_paths=True,
                          root_prim_path="/root", author_blender_name=False, export_custom_properties=False,
                          convert_world_material=False, export_lights=False, export_cameras=False,
                          evaluation_mode="RENDER", triangulate_meshes=False)
    print("exported", name, len(COLL[name].objects))

if PREVIEW:
    offs = {"gun": (0.320, 0, 0.038), "follower": (0.320, -0.237, 0.038), "rod": (0.320, -0.237, 0.038 - 0.014),
            "cartridge": (0.320, 0.007, 0.038), "rest": (0.185, 0.215, 0.002), "mat": (0.270, 0, 0),
            "bench_props": (0.270, 0, 0)}
    for name, off in offs.items():
        for o in COLL[name].objects:
            if o.parent is None:
                o.location = Vector(o.location) + Vector(off)
    bpy.ops.mesh.primitive_plane_add(size=3, location=(0.3, 0, -0.0005))
    bpy.context.object.data.materials.append(mat("table", (0.78, 0.70, 0.52), 0.6))
    cam = bpy.data.objects.new("cam", bpy.data.cameras.new("cam"))
    scene.collection.objects.link(cam)
    cam.location = (0.62, 0.0, 0.55)
    cam.rotation_euler = (math.radians(42), 0, math.radians(90))
    cam.data.lens = 30
    scene.camera = cam
    sun = bpy.data.objects.new("sun", bpy.data.lights.new("sun", "SUN"))
    sun.data.energy = 4.0
    sun.rotation_euler = (math.radians(30), math.radians(-20), 0)
    scene.collection.objects.link(sun)
    world = bpy.data.worlds.new("w")
    world.use_nodes = True
    world.node_tree.nodes["Background"].inputs[0].default_value = (0.8, 0.82, 0.85, 1)
    world.node_tree.nodes["Background"].inputs[1].default_value = 0.6
    scene.world = world
    scene.render.engine = "CYCLES"
    scene.cycles.samples = 24
    scene.render.resolution_x, scene.render.resolution_y = 960, 540
    scene.render.filepath = PREVIEW
    bpy.ops.render.render(write_still=True)
