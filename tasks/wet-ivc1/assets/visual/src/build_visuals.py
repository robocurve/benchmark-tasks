"""Author the wet-ivc1 visual meshes in Blender and export one USD per part body (part-frame coordinates, Z up, metres).

    blender -b --factory-startup --python build_visuals.py -- OUT_DIR [PREVIEW_PNG]

Geometry follows the collision boxes in wet_ivc1.py (mouse IVC cage 391 x 199 x 160 mm, IVC water bottle
55 x 55 x 129 mm, IVC rack bay) so contacts look right; meshes named GLASS_<kind> get Omniverse glass at spawn time.
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


def materials():
    mat("poly_clear", (0.93, 0.96, 0.98), glass=True)
    mat("water", (0.78, 0.90, 1.0), glass=True)
    mat("house_red", (0.85, 0.10, 0.08), glass=True)
    mat("steel_brushed", (0.75, 0.76, 0.78), 0.3, 0.55, "steel_brushed_albedo.png", "steel_brushed_rough.png")  # satin 304: lit bay
    mat("steel_wire", (0.80, 0.81, 0.83), 0.22, 1.0)
    mat("steel_dark", (0.62, 0.64, 0.67), 0.35, 0.5, "steel_brushed_albedo.png")
    mat("anodized_slate", (0.26, 0.28, 0.32), 0.35, 0.3)
    mat("flag_orange", (0.96, 0.30, 0.02), 0.28)
    mat("cap_blue", (0.06, 0.26, 0.66), 0.32)
    mat("silicone", (0.86, 0.86, 0.83), 0.55)
    mat("handle_grey", (0.62, 0.66, 0.70), 0.4)
    mat("filter_white", (0.95, 0.95, 0.95), 0.92, tex="filter_albedo.png")
    mat("bedding", (0.75, 0.62, 0.40), 0.95, tex="bedding_albedo.png")
    mat("pellet", (0.48, 0.36, 0.22), 0.9)
    mat("nest_white", (0.96, 0.95, 0.92), 1.0)
    mat("card", (1, 1, 1), 0.6, tex="cage_card.png")
    mat("label", (1, 1, 1), 0.5, tex="bottle_label.png")
    mat("plate", (1, 1, 1), 0.4, tex="rack_plate.png")
    mat("rubber", (0.03, 0.03, 0.03), 0.8)
    mat("led_green", (0.1, 0.9, 0.2), 0.3)
    mat("silicone_mat", (0.2, 0.33, 0.43), 0.65, tex="mat_albedo.png")


# -- geometry helpers ---------------------------------------------------------------------------------------------------


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


def ring(name, outer, inner, z0, z1, material, group):
    bm = bmesh.new()
    lv = [[bm.verts.new((x, y, z)) for x, y in pts] for pts, z in ((outer, z0), (outer, z1), (inner, z1), (inner, z0))]
    n = len(outer)
    for k in range(4):
        a, b = lv[k], lv[(k + 1) % 4]
        for i in range(n):
            bm.faces.new((a[i], a[(i + 1) % n], b[(i + 1) % n], b[i]))
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    return finish(new_obj(name, bm), material, group, 0.0, 2)


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


def rod(name, a, b, r, material, group, segs=12):
    a, b = Vector(a), Vector(b)
    o = cyl(name, (a + b) / 2, r, (b - a).length, material, group, segs=segs, bevel=0)
    o.rotation_mode = "QUATERNION"
    o.rotation_quaternion = Vector((0, 0, 1)).rotation_difference(b - a)
    return o


# -- cage: mouse IVC cage (part frame: bottom centre, +X nose into the rack) --------------------------


def build_cage():
    g = "cage"
    # clear polycarbonate tub, slight draft, 20 mm corner radius
    rings = [(rrect(0.356, 0.140, 0.020), 0.0, 0), (rrect(0.360, 0.143, 0.021), 0.004, 0),
             (rrect(0.368, 0.148, 0.022), 0.125, 0)]
    tub = loft("GLASS_poly_tub", rings, "poly_clear", g)
    s = tub.modifiers.new("solid", "SOLIDIFY")
    s.thickness = 0.0025
    s.offset = -1
    # peripheral flange rim 391 x 199 x 10 mm
    ring("GLASS_poly_flange", rrect(0.391, 0.199, 0.012), rrect(0.362, 0.142, 0.020), 0.125, 0.135, "poly_clear", g)
    # filter top (own loose body, drawn in the cage frame where it seats): 3 mm moulded shell, open underneath, recessed
    # side-handle pockets beside a 70 x 80 mm grip spine, the front bottle bay, and the plug skirt that carries the gasket
    lg = "lid"
    lid = box("GLASS_poly_lid", (-0.006, 0.0, 0.1475), (0.376, 0.164, 0.025), "poly_clear", lg, bevel=0.004)
    cut(lid, [cutter((-0.006, 0.0, 0.1435), (0.370, 0.158, 0.027)),
              cutter((-0.033, 0.0615, 0.1475), (0.070, 0.043, 0.04)), cutter((-0.033, -0.0615, 0.1475), (0.070, 0.043, 0.04)),
              cutter((0.046, 0.0, 0.15), (0.088, 0.080, 0.04))])
    ring("GLASS_poly_lid_skirt", rrect(0.356, 0.136, 0.010), rrect(0.348, 0.128, 0.008), 0.124, 0.136, "poly_clear", lg)
    ring("lid_gasket", rrect(0.358, 0.138, 0.010), rrect(0.352, 0.132, 0.009), 0.131, 0.135, "silicone", lg)
    # filter sheet under a clear grid retainer
    box("filter_sheet", (-0.130, 0.0, 0.1603), (0.095, 0.100, 0.0026), "filter_white", lg, bevel=0.002)
    for i in range(5):
        box(f"GLASS_poly_grid_x{i}", (-0.130 + (i - 2) * 0.022, 0.0, 0.1614), (0.004, 0.100, 0.0024), "poly_clear", lg, 0.0008)
    for i in range(4):
        box(f"GLASS_poly_grid_y{i}", (-0.130, (i - 1.5) * 0.030, 0.1614), (0.095, 0.004, 0.0024), "poly_clear", lg, 0.0008)
    # radial silicone gasket round the base rim (no latches on the real cage)
    ring("gasket", rrect(0.384, 0.160, 0.016), rrect(0.374, 0.150, 0.014), 0.133, 0.139, "silicone", g)
    # air-valve housings in the cage front and their spring-closed pistons (separate bodies on the valve slides)
    for sy, vg in ((1, "valve_l"), (-1, "valve_r")):
        cyl(f"valve_housing_{sy}", (0.191, sy * 0.050, 0.150), 0.0085, 0.010, "handle_grey", g, axis="X", segs=32)
        cyl(f"valve_piston_{sy}", (0.1985, sy * 0.050, 0.150), 0.0055, 0.014, "steel_wire", vg, axis="X", segs=32)
        cyl(f"valve_face_{sy}", (0.2050, sy * 0.050, 0.150), 0.0052, 0.001, "silicone", vg, axis="X", segs=32)
    # grey side handles in the cutouts
    for sy in (1, -1):
        box(f"handle_{sy}", (-0.027, sy * 0.09375, 0.1455), (0.082, 0.0115, 0.021), "handle_grey", g, 0.003)
        for k in range(5):
            box(f"handle_rib_{sy}_{k}", (-0.055 + k * 0.014, sy * 0.0997, 0.1455), (0.004, 0.0012, 0.016), "handle_grey", g, 0.0005)
    # bottle bay: clear cradle box (open top), 60 x 60 mm, floor top at z = 0.033
    bay = box("GLASS_poly_bay", (0.045, 0.0, 0.0835), (0.068, 0.068, 0.107), "poly_clear", g, bevel=0.003)
    cut(bay, [cutter((0.045, 0.0, 0.088), (0.060, 0.060, 0.110))])
    # corncob bedding with a lumpy surface
    bm = bmesh.new()
    bmesh.ops.create_grid(bm, x_segments=70, y_segments=28, size=0.5)
    import random
    random.seed(3)
    for v in bm.verts:
        x, y = v.co.x * 0.350, v.co.y * 0.134
        bump = 0.004 * math.sin(x * 37) * math.cos(y * 51) + random.uniform(-0.0015, 0.0015)
        if x < -0.08:
            bump += 0.006 * math.exp(-((x + 0.17) ** 2) / 0.002)  # mice push bedding to the rear corner
        v.co = Vector((x, y, 0.016 + bump))
    bed = new_obj("bedding", bm)
    finish(bed, "bedding", g, 0, 2, uv=False)
    bpy.context.view_layer.objects.active = bed
    for o in bpy.context.selected_objects:
        o.select_set(False)
    bed.select_set(True)
    bpy.ops.object.mode_set(mode="EDIT")
    bpy.ops.mesh.select_all(action="SELECT")
    bpy.ops.uv.cube_project(cube_size=0.08)
    bpy.ops.object.mode_set(mode="OBJECT")
    sb = bed.modifiers.new("solid", "SOLIDIFY")
    sb.thickness = 0.012
    # red-tinted polycarbonate mouse house (arched tunnel with a round door)
    bm = bmesh.new()
    pts = [(0.040 * math.cos(math.pi * i / 24), 0.052 * math.sin(math.pi * i / 24)) for i in range(25)]
    v0 = [bm.verts.new((-0.045, x, z)) for x, z in pts]
    v1 = [bm.verts.new((0.045, x, z)) for x, z in pts]
    for i in range(24):
        bm.faces.new((v0[i], v0[i + 1], v1[i + 1], v1[i]))
    bm.faces.new(v0)
    bm.faces.new(list(reversed(v1)))
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    house = new_obj("GLASS_red_house", bm)
    house.location = (0.135, 0.022, 0.018)
    finish(house, "house_red", g, 0, 2)
    hs = house.modifiers.new("solid", "SOLIDIFY")
    hs.thickness = 0.002
    door = bpy.data.objects.new("door", bpy.data.meshes.new("door"))
    dbm = bmesh.new()
    bmesh.ops.create_cone(dbm, cap_ends=True, segments=32, radius1=0.016, radius2=0.016, depth=0.03)
    dbm.to_mesh(door.data)
    dbm.free()
    coll("_cutters").objects.link(door)
    door.rotation_euler = (0, math.pi / 2, 0)
    door.location = (0.090, 0.022, 0.034)
    cut(house, [door])
    # nesting paper tuft
    bm = bmesh.new()
    bmesh.ops.create_icosphere(bm, subdivisions=3, radius=0.026)
    random.seed(5)
    for v in bm.verts:
        v.co *= 1 + random.uniform(-0.25, 0.25)
        v.co.z *= 0.55
    nest = new_obj("nest", bm)
    nest.location = (0.125, -0.044, 0.026)
    finish(nest, "nest_white", g, 0, 2, smooth=False)
    # stainless wire-bar lid inside the tub with a V food hopper and pellets
    z = 0.119
    for k in range(20):
        x = -0.176 + k * 0.0095
        if x > 0.006:
            break
        rod(f"wire_{k}", (x, -0.068, z), (x, 0.068, z), 0.0011, "steel_wire", g)
    for sy in (1, -1):
        rod(f"wire_frame_{sy}", (-0.178, sy * 0.068, z), (0.008, sy * 0.068, z), 0.0018, "steel_wire", g)
    rod("wire_frame_r", (-0.178, -0.068, z), (-0.178, 0.068, z), 0.0018, "steel_wire", g)
    rod("wire_frame_f", (0.008, -0.068, z), (0.008, 0.068, z), 0.0018, "steel_wire", g)
    for k in range(8):
        x = -0.073 + k * 0.010
        rod(f"hopper_a_{k}", (x, -0.040, z), (x, -0.012, 0.072), 0.0010, "steel_wire", g)
        rod(f"hopper_b_{k}", (x, 0.040, z), (x, 0.012, 0.072), 0.0010, "steel_wire", g)
        rod(f"hopper_c_{k}", (x, -0.012, 0.072), (x, 0.012, 0.072), 0.0010, "steel_wire", g)
    random.seed(11)
    for k in range(45):
        x = random.uniform(-0.071, -0.005)
        h = random.uniform(0.076, 0.112)
        wy = 0.012 + (h - 0.072) * (0.028 / 0.047) - 0.006
        o = cyl(f"pellet_{k}", (x, random.uniform(-wy, wy), h), 0.0055, 0.016, "pellet", g, segs=10, bevel=0.002)
        o.rotation_euler = (random.uniform(0, 3), random.uniform(0, 3), random.uniform(0, 3))
    # cage card in a clear holder on the operator (-X) end
    box("card", (-0.1855, 0.0, 0.080), (0.0008, 0.100, 0.060), "card", g, 0.0)
    holder = box("GLASS_poly_cardholder", (-0.1868, 0.0, 0.078), (0.0012, 0.106, 0.066), "poly_clear", g, 0.0005)
    card = bpy.data.objects["card"]
    card.rotation_euler = (0, 0, 0)
    # map the card texture onto its -X face
    uvl = card.data.uv_layers.active
    for poly in card.data.polygons:
        for li in poly.loop_indices:
            v = card.data.vertices[card.data.loops[li].vertex_index].co
            uvl.data[li].uv = ((0.05 - v.y) / 0.100, (v.z - 0.050) / 0.060)
    del holder


# -- bottle: IVC water bottle, 260 mL, inverted with the sipper cap down (part frame: bottle centre) ---------------


def build_bottle():
    g = "bottle"
    r = 0.009
    body = loft("GLASS_poly_bottle",
                [(rrect(0.030, 0.030, 0.012), -0.046, 0), (rrect(0.050, 0.050, r), -0.040, 0),
                 (rrect(0.055, 0.055, r), -0.034, 0), (rrect(0.055, 0.055, r), 0.058, 0),
                 (rrect(0.052, 0.052, r), 0.0635, 0), (rrect(0.040, 0.040, 0.008), 0.0645, 0)],
                "poly_clear", g, cap_bottom=True, cap_top=True)
    sb = body.modifiers.new("solid", "SOLIDIFY")
    sb.thickness = 0.0016
    sb.offset = -1
    loft("GLASS_water_fill",
         [(rrect(0.026, 0.026, 0.010), -0.0445, 0), (rrect(0.046, 0.046, 0.007), -0.039, 0),
          (rrect(0.0515, 0.0515, 0.0075), -0.033, 0), (rrect(0.0515, 0.0515, 0.0075), 0.038, 0)],
         "water", g, cap_bottom=True, cap_top=True)
    # blue screw cap with grip ribs, steel sipper nub
    cyl("cap", (0, 0, -0.0555), 0.0185, 0.018, "cap_blue", g, bevel=0.0012)
    for k in range(28):
        a = 2 * math.pi * k / 28
        o = box(f"cap_rib_{k}", (0.0188 * math.cos(a), 0.0188 * math.sin(a), -0.0555), (0.0016, 0.0016, 0.016), "cap_blue", g, 0.0004)
        o.rotation_euler = (0, 0, a)
    box("cap_seat", (0, 0, -0.0630), (0.050, 0.050, 0.003), "cap_blue", g, 0.0012)
    cyl("sipper", (0, 0, -0.0648), 0.0035, 0.004, "steel_wire", g, segs=20)
    # graduation label on the operator (-X) face
    lab = box("label", (-0.02765, 0.0, 0.010), (0.0004, 0.036, 0.054), "label", g, 0.0)
    uvl = lab.data.uv_layers.active
    for poly in lab.data.polygons:
        for li in poly.loop_indices:
            v = lab.data.vertices[lab.data.loops[li].vertex_index].co
            uvl.data[li].uv = ((0.018 - v.y) / 0.036, (v.z + 0.017) / 0.054)


# -- rack: IVC rack bay (part frame: RACK_POS on the table) ---------------------------------------------


def build_rack():
    g = "rack"
    box("cell_floor", (0.020, 0.0, 0.002), (0.390, 0.216, 0.004), "steel_dark", g, 0.001)
    for sy in (1, -1):
        wall = box(f"side_{sy}", (0.020, sy * 0.114, 0.139), (0.390, 0.012, 0.278), "steel_brushed", g, 0.002)
        cut(wall, [cutter((0.090 + k * 0.022, sy * 0.120, 0.215), (0.008, 0.006, 0.070)) for k in range(5)])
        box(f"side_lip_{sy}", (-0.172, sy * 0.116, 0.139), (0.006, 0.016, 0.278), "steel_brushed", g, 0.0015)
        box(f"runner_{sy}", (0.000, sy * 0.0995, 0.141), (0.410, 0.017, 0.008), "steel_brushed", g, 0.0012)
        box(f"runner_web_{sy}", (0.020, sy * 0.1065, 0.120), (0.390, 0.003, 0.036), "steel_brushed", g, 0.0006)
        box(f"retainer_{sy}", (0.1575, sy * 0.0985, 0.172), (0.115, 0.016, 0.008), "steel_dark", g, 0.0012)
        cyl(f"rivet_{sy}", (-0.150, sy * 0.1205, 0.141), 0.003, 0.002, "steel_wire", g, axis="Y", segs=16)
    box("plenum", (0.215, 0.0, 0.139), (0.016, 0.240, 0.278), "steel_brushed", g, 0.002)
    cyl("grommet", (0.205, 0.0, 0.185), 0.018, 0.020, "anodized_slate", g, axis="X")
    cyl("grommet_ring", (0.1945, 0.0, 0.185), 0.012, 0.002, "rubber", g, axis="X")
    # fixed rear stop plate with bumpers, and the inlet/outlet air nozzles on the plenum
    box("stop_plate", (0.2025, 0.0, 0.1325), (0.009, 0.060, 0.045), "steel_brushed", g, 0.0015)
    for sy in (1, -1):
        cyl(f"bumper_{sy}", (0.1975, sy * 0.020, 0.1325), 0.007, 0.002, "rubber", g, axis="X", segs=24)
        cyl(f"nozzle_{sy}", (0.208, sy * 0.050, 0.170), 0.005, 0.014, "silicone", g, axis="X", segs=32)
        cyl(f"nozzle_collar_{sy}", (0.2145, sy * 0.050, 0.170), 0.009, 0.005, "steel_dark", g, axis="X", segs=32)
    # header bar with the cell plate and LED
    box("header_top", (-0.1175, 0.0, 0.276), (0.075, 0.240, 0.008), "anodized_slate", g, 0.0015)
    plate = box("plate", (-0.1552, 0.080, 0.276), (0.0006, 0.060, 0.007), "plate", g, 0.0)
    uvl = plate.data.uv_layers.active
    for poly in plate.data.polygons:
        for li in poly.loop_indices:
            v = plate.data.vertices[plate.data.loops[li].vertex_index].co
            uvl.data[li].uv = ((0.110 - v.y) / 0.060, (v.z - 0.2725) / 0.007)
    cyl("led", (-0.1555, -0.080, 0.276), 0.0025, 0.002, "led_green", g, axis="X", segs=16)


def build_mat():
    g = "mat"
    o = loft("mat", [(rrect(0.400, 0.260, 0.012), 0.0, 0), (rrect(0.400, 0.260, 0.012), 0.002, 0)], "silicone_mat", g,
             cap_bottom=True, cap_top=True)
    bpy.context.view_layer.objects.active = o
    for x in bpy.context.selected_objects:
        x.select_set(False)
    o.select_set(True)
    bpy.ops.object.mode_set(mode="EDIT")
    bpy.ops.mesh.select_all(action="SELECT")
    bpy.ops.uv.cube_project(cube_size=0.25)
    bpy.ops.object.mode_set(mode="OBJECT")


materials()
build_cage()
build_bottle()
build_rack()
build_mat()

for name in ("cage", "lid", "valve_l", "valve_r", "bottle", "rack", "mat"):
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
    # quick Cycles look at the layout from the station top-camera direction
    import mathutils
    offs = {"cage": (0.210, 0, 0.002), "lid": (0.202, 0, 0.002), "valve_l": (0.210, 0, 0.002), "valve_r": (0.210, 0, 0.002),
            "bottle": (0.255, 0, 0.1095), "rack": (0.610, 0, 0), "mat": (0.200, 0, 0)}
    for name, off in offs.items():
        for o in COLL[name].objects:
            if o.parent is None:
                o.location = Vector(o.location) + Vector(off)
    bpy.ops.mesh.primitive_plane_add(size=3, location=(0.3, 0, -0.0005))
    t = bpy.context.object
    t.data.materials.append(mat("table", (0.78, 0.70, 0.52), 0.6))
    cam = bpy.data.objects.new("cam", bpy.data.cameras.new("cam"))
    scene.collection.objects.link(cam)
    cam.location = (-0.35, 0.0, 0.85)
    cam.rotation_euler = mathutils.Euler((math.radians(40), 0, math.radians(-90)))
    cam.data.lens = 28
    scene.camera = cam
    sun = bpy.data.objects.new("sun", bpy.data.lights.new("sun", "SUN"))
    sun.data.energy = 3.5
    sun.rotation_euler = (math.radians(30), math.radians(-20), 0)
    scene.collection.objects.link(sun)
    world = bpy.data.worlds.new("w")
    world.use_nodes = True
    world.node_tree.nodes["Background"].inputs["Color"].default_value = (0.55, 0.55, 0.58, 1)
    world.node_tree.nodes["Background"].inputs["Strength"].default_value = 0.8
    scene.world = world
    scene.render.engine = "CYCLES"
    scene.cycles.samples = 48
    scene.cycles.device = "CPU"
    scene.render.resolution_x, scene.render.resolution_y = 1280, 704
    scene.render.filepath = PREVIEW
    bpy.ops.render.render(write_still=True)
