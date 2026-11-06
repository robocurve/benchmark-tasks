"""make_looks.py: model the scene's visual meshes in Blender and export them as USD next to this file.

    blender -b -P looks/make_looks.py          (Blender 4.2 or later, from the task folder)

The physics stays the task file's boxes. Each look is a render-only mesh that ``looks.py`` puts on one of those
boxes, whose plain cuboid is then hidden but still collides. Every look is built in world coordinates (metres, the
task file's frame) and exported with the centre of its box at the origin.

- ``profile.usdc``: the 45 x 45 mm aluminium T-slot profile, its end caps and the red tape mark, on the ``profile``
  box (the solid core under the channel). The top slot matches the colliders: a 16 mm opening between 5 mm lips onto
  a 30 mm wide, 9 mm high channel. The three other slots are drawn only (smaller, as on a real profile's sides).
- ``tnut.usdc``: the yellow-zinc drop-in T-nut and its threaded stud, inside the T-nut's 24 x 10 x 46 mm box
  (checked when exported).
- ``mat.usdc``: the green self-healing cutting mat with its grid.

Everything this script makes in Blender is named with the prefix ``t-nut-quarter-turn__``, and only those datablocks
are removed when it runs again. Textures are drawn here with numpy and saved in ``textures/``. Nothing is downloaded.
"""

import math
import os

import bmesh
import bpy
import numpy as np
from mathutils import Matrix, Vector

HERE = os.path.dirname(os.path.abspath(__file__))
TEX = os.path.join(HERE, "textures")
P = "t-nut-quarter-turn__"
RNG = np.random.default_rng(15)

# -- the task file's layout (t_nut_quarter_turn.py), metres ------------------------------------------------------------
TOP = 0.7475
MAT_T = 0.002
MAT_TOP = TOP + MAT_T
MAT_XY, MAT_SIZE = (0.28, -0.08), (0.28, 0.36)
PX = 0.30  # the slot's line
PY0, PLEN = -0.21, 0.30
PY1 = PY0 + PLEN
PW = 0.045
SLOT_OPEN, SLOT_IN, LIP_T, CHANNEL_H = 0.016, 0.030, 0.005, 0.009
PTOP = MAT_TOP + PW
LIP_BOTTOM = PTOP - LIP_T
FLOOR = LIP_BOTTOM - CHANNEL_H
PZ = MAT_TOP + PW / 2  # the cross-section's centre
CAP_T = 0.004
MARK_Y, MARK_W = -0.12, 0.006
CORE_C = (PX, PY0 + PLEN / 2, (MAT_TOP + FLOOR) / 2)  # the profile box's centre
NUT = (0.024, 0.010, 0.006)
STUD_R, STUD_H = 0.004, 0.040
TNUT_SIZE = (NUT[0], NUT[1], NUT[2] + STUD_H)
TNUT_C = (0.21, -0.14, MAT_TOP + TNUT_SIZE[2] / 2)

# -- materials (linear colours) --------------------------------------------------------------------------------------
_MATS = {}


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


# -- textures --------------------------------------------------------------------------------------------------------


def _noise(h, w, cell):
    gh, gw = h // cell + 2, w // cell + 2
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
    img = bpy.data.images.new(P + name + "_draw", w, h, alpha=False)
    rgba = np.concatenate([np.clip(rgb, 0, 1), np.ones((h, w, 1))], axis=2).astype(np.float32)
    img.pixels.foreach_set(rgba.ravel())
    img.filepath_raw = path
    img.file_format = "JPEG"
    img.save(filepath=path, quality=90)
    bpy.data.images.remove(img)
    loaded = bpy.data.images.load(path)
    loaded.name = P + name
    return loaded


def mat_texture():
    """A green self-healing cutting mat: 1 cm grid, a bolder line every 5 cm, a 45 degree guide, worn cuts."""
    k = 4000  # pixels per metre
    w, h = int(MAT_SIZE[0] * k), int(MAT_SIZE[1] * k)
    y, x = np.mgrid[:h, :w]
    shade = 0.95 + 0.07 * _noise(h, w, 90) + 0.04 * _noise(h, w, 3)
    rgb = np.array([0.10, 0.36, 0.22])[None, None, :] * shade[..., None]
    # the grid is drawn in world centimetres, so it lines up with the profile
    wx = (MAT_XY[0] - MAT_SIZE[0] / 2) + x / k
    wy = (MAT_XY[1] - MAT_SIZE[1] / 2) + y / k
    border = 0.012
    inner = (wx > MAT_XY[0] - MAT_SIZE[0] / 2 + border) & (wx < MAT_XY[0] + MAT_SIZE[0] / 2 - border) & \
            (wy > MAT_XY[1] - MAT_SIZE[1] / 2 + border) & (wy < MAT_XY[1] + MAT_SIZE[1] / 2 - border)
    fine = (np.abs(((wx * 100) + 0.5) % 1.0 - 0.5) < 0.04) | (np.abs(((wy * 100) + 0.5) % 1.0 - 0.5) < 0.04)
    bold = (np.abs(((wx * 20) + 0.5) % 1.0 - 0.5) < 0.012) | (np.abs(((wy * 20) + 0.5) % 1.0 - 0.5) < 0.012)
    diag = np.abs(((wx - wy) * 100 / math.sqrt(2) + 0.5) % 10.0 - 0.5) < 0.05
    line = np.array([0.78, 0.84, 0.80])
    for mask, a in ((fine & inner, 0.55), (diag & inner, 0.35), (bold & inner, 0.85)):
        rgb[mask] = rgb[mask] * (1 - a) + line * a
    edge = ~inner & ((np.abs(wx - (MAT_XY[0] - MAT_SIZE[0] / 2 + border)) < 0.0006) |
                     (np.abs(wy - (MAT_XY[1] - MAT_SIZE[1] / 2 + border)) < 0.0006))
    rgb[edge] = line * 0.9
    for _ in range(40):  # healed knife cuts
        p = np.array([RNG.uniform(0, w), RNG.uniform(0, h)])
        a = RNG.uniform(0, math.pi)
        n = int(RNG.uniform(80, 500))
        t = np.arange(n)
        xs = np.clip((p[0] + t * math.cos(a)).astype(int), 0, w - 1)
        ys = np.clip((p[1] + t * math.sin(a)).astype(int), 0, h - 1)
        rgb[ys, xs] *= 0.86
    return _save("mat_top", rgb)


def alu_texture():
    """Clear-anodised aluminium, brushed along the profile (u along it)."""
    w, h = 2048, 256
    streaks = 0.92 + 0.10 * _noise(h, w, 1)[:, :1] * np.ones((1, w)) + 0.03 * _noise(h, w, 40)
    rgb = np.array([0.80, 0.82, 0.84])[None, None, :] * streaks[..., None]
    return _save("alu_brushed", rgb)


# -- geometry --------------------------------------------------------------------------------------------------------


class Look:
    def __init__(self, name, origin, size=None):
        self.name, self.origin, self.size = name, Vector(origin), size
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
    c, s = (Vector(lo) + Vector(hi)) / 2, Vector(hi) - Vector(lo)
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
    """A polygon extruded along an axis. ``pts`` are (u, v) in the plane across the axis: (x, z) for Y."""
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


def _uv_along_y(bm, y0, y1, around=0.18):
    """UVs for the brushed texture: u along the profile, v around its section."""
    layer = bm.loops.layers.uv.new("UVMap")
    for f in bm.faces:
        n = f.normal
        for loop in f.loops:
            co = loop.vert.co
            v = (co.x if abs(n.z) > abs(n.x) else co.z) / around
            loop[layer].uv = ((co.y - y0) / (y1 - y0), v)


def slot_cutter(opening, lip, inside, depth, y0, y1, turn):
    """A T-slot's hollow, from a face of the section inward, as a prism along y. ``turn`` (quarter turns) picks the
    face: 0 top, 1 far (+x), 2 bottom, 3 near (-x). Coordinates (x, z) from the section's centre."""
    h = PW / 2
    pts = [(-opening / 2, h + 0.002), (-opening / 2, h - lip), (-inside / 2, h - lip),
           (-inside / 2 + 0.002, h - lip - depth), (inside / 2 - 0.002, h - lip - depth),
           (inside / 2, h - lip), (opening / 2, h - lip), (opening / 2, h + 0.002)]
    a = -turn * math.pi / 2
    c, s = math.cos(a), math.sin(a)
    world = [(PX + c * u - s * v, PZ + s * u + c * v) for u, v in pts]
    return bm_prism(world, y0, y1)


def build_profile():
    L = Look("profile", CORE_C)
    alu = material("aluminium", (0.62, 0.64, 0.66), 0.38, 0.85, image=alu_texture())
    h = PW / 2
    ch = 0.0025  # corner chamfer
    outer = [(PX - h + ch, PZ - h), (PX + h - ch, PZ - h), (PX + h, PZ - h + ch), (PX + h, PZ + h - ch),
             (PX + h - ch, PZ + h), (PX - h + ch, PZ + h), (PX - h, PZ + h - ch), (PX - h, PZ - h + ch)]
    bm = bm_prism(outer, PY0, PY1)
    body = L.mesh("profile_body", bm, alu)
    cutters = []
    # the top slot: exactly the colliders' channel (flat floor at FLOOR)
    top = [(PX - SLOT_OPEN / 2, PTOP + 0.002), (PX - SLOT_OPEN / 2, LIP_BOTTOM), (PX - SLOT_IN / 2, LIP_BOTTOM),
           (PX - SLOT_IN / 2, FLOOR), (PX + SLOT_IN / 2, FLOOR), (PX + SLOT_IN / 2, LIP_BOTTOM),
           (PX + SLOT_OPEN / 2, LIP_BOTTOM), (PX + SLOT_OPEN / 2, PTOP + 0.002)]
    cutters.append(L.cutter("slot_top", bm_prism(top, PY0 - 0.001, PY1 + 0.001)))
    for turn, name in ((1, "far"), (2, "bottom"), (3, "near")):  # the drawn-only slots, smaller
        cutters.append(L.cutter(f"slot_{name}", slot_cutter(0.008, 0.004, 0.016, 0.005, PY0 - 0.001, PY1 + 0.001,
                                                            turn)))
    cutters.append(L.cutter("bore", bm_cyl((PX, (PY0 + PY1) / 2, PZ), 0.0035, PLEN + 0.004, "Y", 32)))
    L.cut(body, cutters)
    # UVs after the cut are not needed for the drawn slots; the outside faces get the brushed texture
    me = body.data
    bm = bmesh.new()
    bm.from_mesh(me)
    bm.normal_update()
    _uv_along_y(bm, PY0, PY1)
    bm.to_mesh(me)
    bm.free()
    bev = body.modifiers.new("bevel", "BEVEL")
    bev.width, bev.segments, bev.limit_method = 0.0004, 1, "ANGLE"
    # end caps: black plastic, the section's size, a little domed
    cap = material("end_cap", (0.012, 0.012, 0.013), 0.55)
    for name, y0, y1 in (("cap_right", PY0 - CAP_T, PY0), ("cap_left", PY1, PY1 + CAP_T)):
        L.mesh(name, bm_box((PX - h, y0, PZ - h), (PX + h, y1, PZ + h)), cap, bevel=0.0012)
    # the red tape mark: on both lips' tops and down the near face
    red = material("tape_red", (0.70, 0.03, 0.02), 0.6)
    t = 0.0003
    m0, m1 = MARK_Y - MARK_W / 2, MARK_Y + MARK_W / 2
    L.mesh("mark_near", bm_box((PX - h - t, m0, PTOP - 0.0005), (PX - SLOT_OPEN / 2 - 0.0005, m1, PTOP + t)), red)
    L.mesh("mark_far", bm_box((PX + SLOT_OPEN / 2 + 0.0005, m0, PTOP - 0.0005), (PX + h + t, m1, PTOP + t)), red)
    L.mesh("mark_side", bm_box((PX - h - t, m0, PTOP - 0.018), (PX - h + 0.0002, m1, PTOP)), red)
    return L


def build_tnut():
    L = Look("tnut", TNUT_C, size=TNUT_SIZE)
    zinc = material("yellow_zinc", (0.78, 0.50, 0.07), 0.30, 0.85)
    cx, cy, cz = TNUT_C
    z0 = cz - TNUT_SIZE[2] / 2  # the nut's bottom
    lx, wy, t = NUT[0] / 2, NUT[1] / 2, NUT[2]
    c = 0.0015  # the drop-in nut's chamfered top edges, along its length
    section = [(-wy, 0.0), (wy, 0.0), (wy, t - c), (wy - c, t), (-wy + c, t), (-wy, t - c)]
    bm = bm_prism([(cy + u, z0 + v) for u, v in section], cx - lx, cx + lx, axis="X")
    nut = L.mesh("nut", bm, zinc, bevel=0.0004)
    hole = L.cutter("nut_hole", bm_cyl((cx, cy, z0 + t / 2), 0.0034, t + 0.002, "Z", 32))
    L.cut(nut, [hole])
    # the stud: an M8 thread, 1.25 mm pitch, as rings of cones, a chamfered top
    s0 = z0 + t - 0.0015  # it reaches into the nut
    s1 = cz + TNUT_SIZE[2] / 2
    pitch = 0.00125
    bm = bmesh.new()
    z = s0
    while z < s1 - 0.0012:
        for a, b in ((STUD_R * 0.90, STUD_R), (STUD_R, STUD_R * 0.90)):
            seg = bm_cyl((cx, cy, z + pitch / 4), a, pitch / 2, "Z", 24, r2=b)
            me = bpy.data.meshes.new(P + "tmp")
            seg.to_mesh(me)
            seg.free()
            bm.from_mesh(me)
            bpy.data.meshes.remove(me)
            z += pitch / 2
    tip = bm_cyl((cx, cy, (z + s1) / 2), STUD_R * 0.92, s1 - z, "Z", 24, r2=STUD_R * 0.70)
    me = bpy.data.meshes.new(P + "tmp")
    tip.to_mesh(me)
    tip.free()
    bm.from_mesh(me)
    bpy.data.meshes.remove(me)
    L.mesh("stud", bm, zinc, smooth=True)
    return L


def build_mat():
    L = Look("mat", (*MAT_XY, TOP + MAT_T / 2))
    x0, y0 = MAT_XY[0] - MAT_SIZE[0] / 2, MAT_XY[1] - MAT_SIZE[1] / 2
    x1, y1 = x0 + MAT_SIZE[0], y0 + MAT_SIZE[1]
    L.mesh("mat", bm_box((x0, y0, TOP), (x1, y1, MAT_TOP), uv=(x0, y0, x1, y1)),
           material("cutting_mat", (0.03, 0.15, 0.06), 0.8, image=mat_texture()), bevel=0.0008)
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
    if look.size is not None:
        half = Vector(look.size) / 2
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
    """Remove what an earlier run of this script made (everything named with its prefix), nothing else."""
    for coll in list(bpy.data.collections):
        if coll.name.startswith(P):
            for ob in list(coll.objects):
                bpy.data.objects.remove(ob)
    for coll in list(bpy.data.collections):
        if coll.name.startswith(P):
            bpy.data.collections.remove(coll)
    for block in (bpy.data.objects, bpy.data.meshes, bpy.data.materials, bpy.data.images, bpy.data.curves):
        for item in list(block):
            if item.name.startswith(P):
                block.remove(item)
    _MATS.clear()


def build():
    clear()
    looks = [build_profile(), build_tnut(), build_mat()]
    return {look.name: export(look) for look in looks}


if __name__ == "__main__":
    print(build())
