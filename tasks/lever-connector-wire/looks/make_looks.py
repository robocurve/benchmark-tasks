"""make_looks.py: model the scene's visual meshes in Blender and export them as USD next to this file.

    blender -b -P looks/make_looks.py          (Blender 4.2 or later, from the task folder)

The physics stays the task file's boxes. Each look is a render-only mesh that ``looks.py`` puts on one of those
boxes, whose plain cuboid is then hidden but still collides. Every look is built in the connector's frame of the task
file (u from the near port's face into the connector as x, v across it as y, w up from the wiring board's top as z,
metres) around an empty at its box's centre, and exported with that empty at the origin.

- ``board.usdc``: the plywood wiring board, the connector's grey housing (the near port, its clamp window and
  contact bar, the far port), the far orange lever, the far brown wire and the cable it comes from.
- ``lever.usdc``: the near orange lever and the steel clamp under it.
- ``wire.usdc``: the loose wire, brown insulation and a bare copper end, inside its 14.2 x 0.7 x 0.7 cm box (checked
  when exported).

Every datablock this script makes is named with the task's folder name first, so it can run in a shared Blender
session: it removes only what carries that prefix. Textures are drawn here with numpy and saved in ``textures/``.
Nothing is downloaded.
"""

import math
import os

import bmesh
import bpy
import numpy as np
from mathutils import Matrix, Vector

HERE = os.path.dirname(os.path.abspath(__file__))
TEX = os.path.join(HERE, "textures")
P = "lever-connector-wire__"  # every datablock's name starts with this
RNG = np.random.default_rng(8)

# -- the task file's layout (lever_connector_wire.py), connector frame, metres ----------------------------------------
BOARD_LO, BOARD_HI = (-0.09, -0.06, -0.010), (0.125, 0.06, 0.0)
HALF_W, BORE_HALF, FLOOR_T, BORE_TOP, HOUSING_TOP = 0.010, 0.0055, 0.0, 0.011, 0.014
CEILING_FROM, STOP_U, NEAR_LEN, FAR_LEN = 0.023, 0.030, 0.033, 0.033
BAR, BAR_TOP = (0.011, 0.020), 0.0015
PIVOT = (0.0, 0.0, 0.0145)
HANDLE = ((0.0, -0.007, 0.0145), (0.033, 0.007, 0.0225))
TOOTH = ((0.012, -0.0045, 0.0052), (0.018, 0.0045, 0.0145))
WIRE_AXIS_W = FLOOR_T + 0.0035
FAR_WIRE_END = NEAR_LEN + FAR_LEN + 0.045
INS_LEN, BARE_LEN, INS_D, BARE_D = 0.120, 0.022, 0.007, 0.004
WIRE_LEN = INS_LEN + BARE_LEN


def centre(lo, hi):
    return tuple((a + b) / 2 for a, b in zip(lo, hi))


def size(lo, hi):
    return tuple(b - a for a, b in zip(lo, hi))


# -- materials -------------------------------------------------------------------------------------------------------
_MATS = {}


def lin(c):
    """sRGB to linear."""
    return tuple(((v + 0.055) / 1.055) ** 2.4 if v > 0.04045 else v / 12.92 for v in c)


def material(name, color=(0.5, 0.5, 0.5), rough=0.5, metal=0.0, image=None):
    name = P + name
    if name in _MATS:
        return _MATS[name]
    m = bpy.data.materials.get(name) or bpy.data.materials.new(name)
    m.use_nodes = True
    nodes, links = m.node_tree.nodes, m.node_tree.links
    nodes.clear()
    out = nodes.new("ShaderNodeOutputMaterial")
    bsdf = nodes.new("ShaderNodeBsdfPrincipled")
    links.new(bsdf.outputs["BSDF"], out.inputs["Surface"])
    bsdf.inputs["Base Color"].default_value = (*color, 1.0)
    bsdf.inputs["Roughness"].default_value = rough
    bsdf.inputs["Metallic"].default_value = metal
    if image is not None:
        tex = nodes.new("ShaderNodeTexImage")
        tex.image = image
        tex.extension = "REPEAT"
        links.new(tex.outputs["Color"], bsdf.inputs["Base Color"])
    m.diffuse_color = (*color, 1.0)
    _MATS[name] = m
    return m


def materials():
    return {
        "housing": material("housing_grey", lin((0.80, 0.81, 0.82)), 0.45),
        "port": material("port_dark", lin((0.10, 0.10, 0.11)), 0.6),
        "lever": material("lever_orange", lin((0.98, 0.42, 0.02)), 0.40),
        "steel": material("spring_steel", lin((0.70, 0.71, 0.73)), 0.30, 1.0),
        "tin": material("tinned_bar", lin((0.80, 0.80, 0.78)), 0.25, 1.0),
        "brown": material("insulation_brown", lin((0.36, 0.20, 0.10)), 0.45),
        "blue": material("insulation_blue", lin((0.10, 0.25, 0.60)), 0.45),
        "yellow": material("insulation_yellow", lin((0.75, 0.70, 0.10)), 0.45),
        "sheath": material("sheath_grey", lin((0.62, 0.63, 0.64)), 0.55),
        "copper": material("copper", lin((0.86, 0.50, 0.30)), 0.30, 1.0),
        "screw": material("screw_zinc", lin((0.72, 0.72, 0.70)), 0.30, 1.0),
        "mark": material("gauge_mark", lin((0.20, 0.20, 0.21)), 0.6),
        "cap": material("wire_cap", lin((0.85, 0.85, 0.83)), 0.5),
    }


# -- textures --------------------------------------------------------------------------------------------------------


def _noise(h, w, cell_y, cell_x):
    gh, gw = h // cell_y + 2, w // cell_x + 2
    g = RNG.random((gh, gw))
    ys, xs = np.linspace(0, gh - 2, h), np.linspace(0, gw - 2, w)
    y0, x0 = ys.astype(int), xs.astype(int)
    fy, fx = (ys - y0)[:, None], (xs - x0)[None, :]
    a, b = g[y0][:, x0], g[y0][:, x0 + 1]
    c, d = g[y0 + 1][:, x0], g[y0 + 1][:, x0 + 1]
    return a * (1 - fx) * (1 - fy) + b * fx * (1 - fy) + c * (1 - fx) * fy + d * fx * fy


def _save(name, rgb):
    """Save an sRGB image (rows from the bottom, as Blender stores them) as JPEG in textures/ and load it."""
    os.makedirs(TEX, exist_ok=True)
    path = os.path.join(TEX, name + ".jpg")
    h, w, _ = rgb.shape
    for old in [i for i in bpy.data.images if i.name.startswith(P + name)]:
        bpy.data.images.remove(old)
    img = bpy.data.images.new(P + name, w, h, alpha=False)
    rgba = np.concatenate([np.clip(rgb, 0, 1), np.ones((h, w, 1))], axis=2).astype(np.float32)
    img.pixels.foreach_set(rgba.ravel())
    img.filepath_raw = path
    img.file_format = "JPEG"
    img.save()
    bpy.data.images.remove(img)
    img = bpy.data.images.load(path)
    img.name = P + name
    return img


def plywood_texture():
    """Birch plywood: pale wood with long grain along u, a few darker streaks and knots."""
    w, h = 1600, 1200  # u across the image, v up it
    grain = 0.6 * _noise(h, w, 3, 120) + 0.4 * _noise(h, w, 1, 40)
    streak = np.clip((_noise(h, w, 8, 400) - 0.62) * 4, 0, 1)
    tone = 0.93 + 0.10 * grain - 0.12 * streak
    y, x = np.mgrid[:h, :w]
    for _ in range(3):
        cx, cy, r = RNG.uniform(0, w), RNG.uniform(0, h), RNG.uniform(10, 22)
        d = np.hypot((x - cx) / 2.5, y - cy)
        tone -= 0.18 * np.exp(-(d / r) ** 2)
    rgb = np.array([0.80, 0.66, 0.47])[None, None, :] * tone[..., None]
    return _save("plywood", rgb)


# -- geometry --------------------------------------------------------------------------------------------------------


class Look:
    def __init__(self, name, origin, box=None):
        self.name, self.origin, self.box = name, Vector(origin), box
        self.coll = bpy.data.collections.new(P + "look_" + name)
        bpy.context.scene.collection.children.link(self.coll)
        self.root = bpy.data.objects.new(P + name, None)
        self.root.location = self.origin
        self.coll.objects.link(self.root)
        self.cutters = bpy.data.collections.new(P + "cutters_" + name)
        bpy.context.scene.collection.children.link(self.cutters)

    def mesh(self, name, bm, mat, bevel=0.0, smooth=False, to=None):
        bmesh.ops.translate(bm, vec=-self.origin, verts=bm.verts)
        me = bpy.data.meshes.new(P + name)
        bm.to_mesh(me)
        bm.free()
        if smooth:
            me.shade_smooth()
        ob = bpy.data.objects.new(P + name, me)
        (to or self.coll).objects.link(ob)
        ob.parent = self.root
        me.materials.append(mat)
        if bevel:
            m = ob.modifiers.new("bevel", "BEVEL")
            m.width, m.segments, m.limit_method = bevel, 2, "ANGLE"
        return ob

    def cutter(self, name, bm):
        ob = self.mesh(name, bm, material("cutter"), to=self.cutters)
        ob.display_type, ob.hide_render = "WIRE", True
        return ob

    def cut(self, ob, cutters):
        group = bpy.data.collections.new(P + f"cut_{ob.name[len(P):]}")
        self.cutters.children.link(group)
        for c in cutters:
            group.objects.link(c)
        m = ob.modifiers.new("cut", "BOOLEAN")
        m.operation, m.operand_type, m.collection, m.solver = "DIFFERENCE", "COLLECTION", group, "EXACT"
        ob.modifiers.move(len(ob.modifiers) - 1, 0)  # cut before bevelling


def bm_box(lo, hi, uv=None):
    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=1.0)
    c, s = Vector(centre(lo, hi)), Vector(size(lo, hi))
    for v in bm.verts:
        v.co = c + Vector((v.co.x * s.x, v.co.y * s.y, v.co.z * s.z))
    if uv:
        layer = bm.loops.layers.uv.new("UVMap")
        x0, y0, x1, y1 = uv
        for f in bm.faces:
            for loop in f.loops:
                co = loop.vert.co
                loop[layer].uv = ((co.x - x0) / (x1 - x0), (co.y - y0) / (y1 - y0))
    return bm


def _axis_matrix(axis):
    return {"Z": Matrix.Identity(4), "Y": Matrix.Rotation(math.pi / 2, 4, "X"),
            "X": Matrix.Rotation(math.pi / 2, 4, "Y")}[axis]


def bm_cyl(center, r, h, axis="Z", segs=32, r2=None):
    bm = bmesh.new()
    bmesh.ops.create_cone(bm, cap_ends=True, segments=segs, radius1=r, radius2=r if r2 is None else r2, depth=h,
                          matrix=Matrix.Translation(Vector(center)) @ _axis_matrix(axis))
    return bm


def bm_prism(pts, a0, a1, axis="Y"):
    """A polygon extruded along an axis. ``pts`` are (u, v) in the plane across the axis: (x, z) for Y, (y, z) for
    X, (x, y) for Z."""
    def co(u, v, a):
        return {"Y": (u, a, v), "X": (a, u, v), "Z": (u, v, a)}[axis]
    bm = bmesh.new()
    lo = [bm.verts.new(co(u, v, a0)) for u, v in pts]
    hi = [bm.verts.new(co(u, v, a1)) for u, v in pts]
    bm.faces.new(lo)
    bm.faces.new(hi)
    n = len(pts)
    for i in range(n):
        bm.faces.new((lo[i], lo[(i + 1) % n], hi[(i + 1) % n], hi[i]))
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    return bm


def _fillet(pts, radius, n=6):
    out = [pts[0]]
    for a, b, c in zip(pts, pts[1:], pts[2:]):
        u, v = (a - b).normalized(), (c - b).normalized()
        p0, p1 = b + u * min(radius, (a - b).length / 2), b + v * min(radius, (c - b).length / 2)
        for i in range(n + 1):  # a quadratic Bezier through the corner
            t = i / n
            out.append(p0 * (1 - t) ** 2 + b * 2 * t * (1 - t) + p1 * t * t)
    out.append(pts[-1])
    return out


def pipe(look, name, pts, r, mat, fillet=None):
    """A tube along a polyline, its corners rounded."""
    cu = bpy.data.curves.new(P + name, "CURVE")
    cu.dimensions, cu.bevel_depth, cu.bevel_resolution, cu.use_fill_caps = "3D", r, 3, True
    sp = cu.splines.new("POLY")
    pts = _fillet([Vector(p) for p in pts], fillet or 2.5 * r)
    sp.points.add(len(pts) - 1)
    for p, co in zip(sp.points, pts):
        p.co = (*co, 1.0)
    tmp = bpy.data.objects.new(P + name + "_curve", cu)
    bpy.context.scene.collection.objects.link(tmp)
    bpy.context.view_layer.update()
    me = bpy.data.meshes.new_from_object(tmp.evaluated_get(bpy.context.evaluated_depsgraph_get()))
    bpy.data.objects.remove(tmp)
    bpy.data.curves.remove(cu)
    bm = bmesh.new()
    bm.from_mesh(me)
    bpy.data.meshes.remove(me)
    return look.mesh(name, bm, mat, smooth=True)


def lever_profile(u0, u1, w0, w1, front):
    """The lever's side profile in (u, w): a 3 mm lip standing up at its free end, a thumb dip behind the lip, a
    rounded hinge end. ``front`` is -1 when the free end is at u0, +1 when it is at u1."""
    t = w1 - w0
    pts = [(0.000, 0.0), (0.0, t), (0.003, t), (0.006, t - 0.003), (0.010, t - 0.0025), (u1 - u0 - 0.004, t - 0.0025),
           (u1 - u0 - 0.0015, t - 0.0035), (u1 - u0, t - 0.0055), (u1 - u0, 0.0)]
    if front < 0:
        return [(u0 + a, w0 + b) for a, b in pts]
    return [(u1 - a, w0 + b) for a, b in reversed(pts)]


# -- the board and the connector ---------------------------------------------------------------------------------------


def build_board(M):
    L = Look("board", centre(BOARD_LO, BOARD_HI))
    (u0, v0, w0), (u1, v1, w1) = BOARD_LO, BOARD_HI
    L.mesh("plywood", bm_box(BOARD_LO, BOARD_HI, uv=(u0, v0, u1, v1)),
           material("plywood", (0.8, 0.66, 0.47), 0.7, image=plywood_texture()), bevel=0.0012)

    # the housing: the near half with its port, clamp window and contact bar, the far half with its port
    near = L.mesh("housing_near", bm_box((0.0, -HALF_W, 0.0), (NEAR_LEN, HALF_W, HOUSING_TOP)), M["housing"],
                  bevel=0.0006)
    L.cut(near, [L.cutter("near_bore", bm_box((-0.001, -BORE_HALF, FLOOR_T - 0.0005), (STOP_U, BORE_HALF, BORE_TOP))),
                 L.cutter("near_window", bm_box((-0.001, -BORE_HALF, BORE_TOP - 0.001),
                                                (CEILING_FROM, BORE_HALF, HOUSING_TOP + 0.001)))])
    far = L.mesh("housing_far", bm_box((NEAR_LEN, -HALF_W, 0.0), (NEAR_LEN + FAR_LEN, HALF_W, HOUSING_TOP)),
                 M["housing"], bevel=0.0006)
    L.cut(far, [L.cutter("far_bore", bm_box((NEAR_LEN + 0.003, -BORE_HALF, FLOOR_T - 0.0005),
                                            (NEAR_LEN + FAR_LEN + 0.001, BORE_HALF, BORE_TOP)))])
    for name, u0, u1 in (("near_floor", 0.0, STOP_U), ("far_floor", NEAR_LEN + 0.003, NEAR_LEN + FAR_LEN)):
        L.mesh(name, bm_box((u0, -BORE_HALF, -0.001), (u1, BORE_HALF, 0.0001)), M["housing"])  # the ports' floors
    L.mesh("chamber_back", bm_box((STOP_U - 0.0004, -BORE_HALF, FLOOR_T), (STOP_U, BORE_HALF, BORE_TOP)), M["port"])
    L.mesh("far_chamber_back", bm_box((NEAR_LEN + 0.003, -BORE_HALF, FLOOR_T),
                                      (NEAR_LEN + 0.0034, BORE_HALF, BORE_TOP)), M["port"])
    L.mesh("contact_bar", bm_box((BAR[0], -BORE_HALF + 0.0002, FLOOR_T), (BAR[1], BORE_HALF - 0.0002, BAR_TOP)),
           M["tin"], bevel=0.0003)
    L.mesh("far_contact_bar", bm_box((NEAR_LEN + FAR_LEN - BAR[1], -BORE_HALF + 0.0002, FLOOR_T),
                                     (NEAR_LEN + FAR_LEN - BAR[0], BORE_HALF - 0.0002, BAR_TOP)), M["tin"])
    # hinge bosses either side of each lever
    for side in (-1, 1):
        for name, u in (("near", PIVOT[0] + 0.0026), ("far", NEAR_LEN + FAR_LEN - 0.0026)):
            L.mesh(f"boss_{name}_{side}", bm_cyl((u, side * 0.0088, PIVOT[2] + 0.001), 0.0026, 0.0024, "Y", 20),
                   M["housing"], smooth=True)
    # mounting ears with screws, low on the housing's sides
    for side in (-1, 1):
        ear = (NEAR_LEN - 0.006, side * (HALF_W + 0.006), 0.0)
        L.mesh(f"ear_{side}", bm_box((ear[0] - 0.006, min(side * HALF_W, ear[1] + side * 0.006), 0.0),
                                     (ear[0] + 0.006, max(side * HALF_W, ear[1] + side * 0.006), 0.0025)),
               M["housing"], bevel=0.0006)
        L.mesh(f"screw_{side}", bm_cyl((ear[0], ear[1], 0.0032), 0.0028, 0.0014, "Z", 24), M["screw"], smooth=True,
               bevel=0.0004)
        L.mesh(f"slot_{side}", bm_box((ear[0] - 0.0024, ear[1] - 0.0004, 0.0036), (ear[0] + 0.0024, ear[1] + 0.0004,
                                                                                     0.0041)), M["port"])
    # a strip gauge moulded on the near half's right side: ticks every 2 mm, the 22 mm strip length marked long
    for i in range(0, 12):
        u = 0.002 * i
        tall = 0.006 if i in (0, 11) else 0.003
        L.mesh(f"tick_{i}", bm_box((u - 0.0002, -HALF_W - 0.0003, HOUSING_TOP - 0.0015 - tall),
                                   (u + 0.0002, -HALF_W + 0.0001, HOUSING_TOP - 0.0015)), M["mark"])
    # the far lever, shut, mirrored: its hinge at the far port's face, its lip toward the middle, clear of the near
    # lever's sweep
    L.mesh("far_lever", bm_prism(lever_profile(NEAR_LEN + 0.008, NEAR_LEN + FAR_LEN, HANDLE[0][2], HANDLE[1][2], -1),
                                 -0.007, 0.007, "Y"), M["lever"], bevel=0.0005)
    # the far wire: copper on the far contact bar, brown insulation out of the far port, into a grey cable
    u_end = NEAR_LEN + FAR_LEN
    L.mesh("far_copper", bm_cyl((u_end - 0.011 - 0.001, 0.0, WIRE_AXIS_W), BARE_D / 2, BARE_LEN, "X", 16), M["copper"],
           smooth=True)
    sheath_u = 0.106
    L.mesh("far_insulation", bm_cyl(((u_end - 0.001 + sheath_u) / 2, 0.0, WIRE_AXIS_W), INS_D / 2,
                                    sheath_u - u_end + 0.001, "X", 24), M["brown"], smooth=True)
    pipe(L, "sheath", [(sheath_u, 0.0, WIRE_AXIS_W + 0.001), (0.122, 0.0, WIRE_AXIS_W + 0.001),
                       (0.135, 0.008, -0.002), (0.150, 0.018, -0.0045)], 0.0055, M["sheath"], fillet=0.008)
    for name, mat, side in (("blue", M["blue"], 1), ("yellow", M["yellow"], -1)):
        pts = [(sheath_u + 0.002, 0.0, WIRE_AXIS_W + 0.001), (sheath_u - 0.006, side * 0.010, WIRE_AXIS_W + 0.001),
               (sheath_u - 0.010, side * 0.030, WIRE_AXIS_W)]
        pipe(L, f"wire_{name}", pts, 0.0032, mat, fillet=0.006)
        end = Vector(pts[-1])
        L.mesh(f"cap_{name}", bm_cyl((end.x - 0.001, end.y + side * 0.007, end.z), 0.0048, 0.014, "Y", 24, r2=0.0036),
               M["cap"], smooth=True)
    return L


def build_lever(M):
    L = Look("lever", centre(*HANDLE))
    (u0, v0, w0), (u1, v1, w1) = HANDLE
    L.mesh("handle", bm_prism(lever_profile(u0, u1, w0, w1, +1), v0, v1, "Y"), M["lever"], bevel=0.0005)
    for i, u in enumerate((0.013, 0.017, 0.021)):  # grip ridges in front of the dip by the lip
        L.mesh(f"ridge_{i}", bm_box((u - 0.0005, v0 + 0.0015, w1 - 0.0026), (u + 0.0005, v1 - 0.0015, w1 - 0.0016)),
               M["lever"])
    (a0, b0, c0), (a1, b1, c1) = TOOTH
    L.mesh("clamp", bm_box((a0 + 0.0002, b0 + 0.0003, c0), (a1 - 0.0002, b1 - 0.0003, c1)), M["steel"], bevel=0.0004)
    return L


def build_wire(M):
    L = Look("wire", (0.0, 0.0, 0.0), box=(WIRE_LEN, INS_D, INS_D))
    x0, x_ins = -WIRE_LEN / 2, -WIRE_LEN / 2 + INS_LEN
    r = INS_D / 2 - 0.00001
    L.mesh("insulation", bm_cyl(((x0 + x_ins) / 2, 0.0, 0.0), r, INS_LEN, "X", 32), M["brown"], smooth=True,
           bevel=0.0006)
    L.mesh("bare", bm_cyl(((x_ins + WIRE_LEN / 2) / 2 - 0.0002, 0.0, 0.0), BARE_D / 2 - 0.00001, BARE_LEN - 0.0004, "X",
                          20), M["copper"], smooth=True, bevel=0.0004)
    L.mesh("cut_end", bm_cyl((x0 + 0.0002, 0.0, 0.0), BARE_D / 2, 0.0004, "X", 20), M["copper"], smooth=True)
    return L


# -- export ----------------------------------------------------------------------------------------------------------


def _local_bounds(look):
    dg = bpy.context.evaluated_depsgraph_get()
    lo, hi = Vector((1e9,) * 3), Vector((-1e9,) * 3)
    inv = look.root.matrix_world.inverted()
    for ob in look.coll.objects:
        if ob.type != "MESH":
            continue
        ev = ob.evaluated_get(dg)
        me = ev.to_mesh()
        mw = inv @ ob.matrix_world
        for v in me.vertices:
            p = mw @ v.co
            lo = Vector(map(min, lo, p))
            hi = Vector(map(max, hi, p))
        ev.to_mesh_clear()
    return lo, hi


def export(look):
    bpy.context.view_layer.update()
    lo, hi = _local_bounds(look)
    if look.box is not None:
        half = Vector(look.box) / 2
        over = max(max(-lo[i] - half[i], hi[i] - half[i]) for i in range(3))
        if over > 1e-5:
            raise ValueError(f"{look.name}: the look sticks {over * 1000:.2f} mm out of its box ({lo}, {hi})")
    look.root.location = (0, 0, 0)
    bpy.context.view_layer.update()
    path = os.path.join(HERE, look.name + ".usdc")
    bpy.ops.wm.usd_export(filepath=path, collection=look.coll.name, export_materials=True,
                          generate_preview_surface=True, export_textures_mode="KEEP", relative_paths=True,
                          root_prim_path="/look", export_lights=False, export_cameras=False,
                          convert_world_material=False, export_custom_properties=False, author_blender_name=False,
                          evaluation_mode="RENDER", export_uvmaps=True, export_normals=True)
    look.root.location = look.origin
    bpy.context.view_layer.update()
    return {"file": os.path.basename(path), "kB": round(os.path.getsize(path) / 1024),
            "low": tuple(round(v, 4) for v in lo), "high": tuple(round(v, 4) for v in hi)}


def clear():
    """Remove what an earlier run of this script made (names starting with the prefix), and nothing else."""
    for coll in [c for c in bpy.data.collections if c.name.startswith(P)]:
        for ob in list(coll.objects):
            if ob.name.startswith(P):
                bpy.data.objects.remove(ob)
    for coll in [c for c in bpy.data.collections if c.name.startswith(P)]:
        bpy.data.collections.remove(coll)
    for ob in [o for o in bpy.data.objects if o.name.startswith(P)]:
        bpy.data.objects.remove(ob)
    for block in (bpy.data.meshes, bpy.data.curves, bpy.data.materials, bpy.data.images):
        for item in [i for i in block if i.name.startswith(P)]:
            if item.users == 0:
                block.remove(item)
    _MATS.clear()


def build():
    clear()
    M = materials()
    looks = [build_board(M), build_lever(M), build_wire(M)]
    return {look.name: export(look) for look in looks}


if __name__ == "__main__":
    print(build())
