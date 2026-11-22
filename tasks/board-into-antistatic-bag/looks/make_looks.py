"""make_looks.py: model the scene's visual meshes in Blender and export them as USD next to this file.

    blender -b -P looks/make_looks.py          (Blender 4.2 or later, from the task folder)

The physics stays the task file's boxes (the bag's walls are boxes too, built by ``envelope.py``). Each look is a
render-only mesh that ``looks.py`` puts on one of those boxes, whose plain cuboid is then hidden but still sets its
bounds. Every look is built in its box's own frame, around the box's centre (metres, z up), and checked to stay inside
the box when it is exported.

- ``bag.usdc``: the pink antistatic bag, 7.7 x 15.2 x 1.2 cm, its mouth toward -y: two sheets, heat-sealed sides and
  end, the cavity open at the mouth, the thumb notch in the top sheet and a printed ESD mark.
- ``board.usdc``: the bare circuit board, 6.5 x 14 cm, 1.6 mm thick, with its parts up to 2 mm tall.
- ``mat.usdc``: the dark grey anti-static mat with its ground snap.

Textures are drawn here with numpy and saved in ``textures/``. Nothing is downloaded.
"""

import math
import os

import bmesh
import bpy
import numpy as np
from mathutils import Matrix, Vector

HERE = os.path.dirname(os.path.abspath(__file__))
TEX = os.path.join(HERE, "textures")
RNG = np.random.default_rng(5)

# -- the task file's sizes (board_into_antistatic_bag.py), metres -----------------------------------------------------
BAG_SIZE = (0.077, 0.152, 0.012)
WALL, SEAM, END, NOTCH = 0.002, 0.002, 0.004, (0.03, 0.015)
BOARD_SIZE = (0.065, 0.14, 0.0036)
PCB_T = 0.0016  # the board itself, under its 2 mm of parts
MAT_SIZE = (0.24, 0.282, 0.005)

# -- materials (linear colours) --------------------------------------------------------------------------------------
_MATS = {}


def material(name, color=(0.5, 0.5, 0.5), rough=0.5, metal=0.0, image=None):
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
    old = bpy.data.images.get(name)
    if old:
        bpy.data.images.remove(old)
    img = bpy.data.images.new(name, w, h, alpha=False)
    rgba = np.concatenate([np.clip(rgb, 0, 1), np.ones((h, w, 1))], axis=2).astype(np.float32)
    img.pixels.foreach_set(rgba.ravel())
    img.filepath_raw = path
    img.file_format = "JPEG"
    bpy.context.scene.render.image_settings.quality = 90
    img.save()
    bpy.data.images.remove(img)
    return bpy.data.images.load(path)


class Canvas:
    """A texture over a world rectangle, drawn in world coordinates."""

    def __init__(self, x0, y0, sx, sy, w):
        self.x0, self.y0, self.k = x0, y0, w / sx
        self.w, self.h = w, int(round(sy * self.k))

    def px(self, x, y):
        return (x - self.x0) * self.k, (y - self.y0) * self.k

    def rect(self, mask, x0, y0, x1, y1):
        a, b = self.px(min(x0, x1), min(y0, y1))
        c, d = self.px(max(x0, x1), max(y0, y1))
        mask[max(int(b), 0):max(int(math.ceil(d)), 0), max(int(a), 0):max(int(math.ceil(c)), 0)] = True

    def outline(self, mask, x0, y0, x1, y1, t=0.0003):
        self.rect(mask, x0, y0, x1, y0 + t)
        self.rect(mask, x0, y1 - t, x1, y1)
        self.rect(mask, x0, y0, x0 + t, y1)
        self.rect(mask, x1 - t, y0, x1, y1)

    def line_px(self, mask, a, b, width):
        n = int(max(abs(b[0] - a[0]), abs(b[1] - a[1])) * 2) + 2
        xs = np.linspace(a[0], b[0], n).round().astype(int)
        ys = np.linspace(a[1], b[1], n).round().astype(int)
        r = width // 2
        for ox in range(-r, width - r):
            for oy in range(-r, width - r):
                X, Y = np.clip(xs + ox, 0, self.w - 1), np.clip(ys + oy, 0, self.h - 1)
                mask[Y, X] = True

    def disc_px(self, mask, cx, cy, r):
        y, x = np.ogrid[: self.h, : self.w]
        mask[(x - cx) ** 2 + (y - cy) ** 2 <= r * r] = True


DIRS = [np.array([math.cos(a), math.sin(a)]) for a in np.arange(8) * math.pi / 4]


# -- geometry --------------------------------------------------------------------------------------------------------


class Look:
    def __init__(self, name, origin, size=None):
        self.name, self.origin, self.size = name, Vector(origin), size
        self.coll = bpy.data.collections.new("look_" + name)
        bpy.context.scene.collection.children.link(self.coll)
        self.root = bpy.data.objects.new(name, None)
        self.root.location = self.origin
        self.coll.objects.link(self.root)
        self.cutters = bpy.data.collections.new("cutters_" + name)
        bpy.context.scene.collection.children.link(self.cutters)

    def mesh(self, name, bm, mat, bevel=0.0, smooth=False, to=None):
        bmesh.ops.translate(bm, vec=-self.origin, verts=bm.verts)
        me = bpy.data.meshes.new(name)
        bm.to_mesh(me)
        bm.free()
        if smooth:
            me.shade_smooth()
        ob = bpy.data.objects.new(name, me)
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
        group = bpy.data.collections.new(f"cut_{ob.name}")
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
        _uv_planar(bm, *uv)
    return bm


def _uv_planar(bm, x0, y0, x1, y1):
    layer = bm.loops.layers.uv.new("UVMap")
    for f in bm.faces:
        for loop in f.loops:
            co = loop.vert.co
            loop[layer].uv = ((co.x - x0) / (x1 - x0), (co.y - y0) / (y1 - y0))


def _axis_matrix(axis):
    return {"Z": Matrix.Identity(4), "Y": Matrix.Rotation(math.pi / 2, 4, "X"), "X": Matrix.Rotation(math.pi / 2, 4, "Y")}[axis]


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


def bm_ring(center, r_in, r_out, h, axis="Z", segs=48):
    bm = bmesh.new()
    rings = []
    for r, z in ((r_out, -h / 2), (r_out, h / 2), (r_in, h / 2), (r_in, -h / 2)):
        rings.append([bm.verts.new((r * math.cos(2 * math.pi * i / segs), r * math.sin(2 * math.pi * i / segs), z))
                      for i in range(segs)])
    for a, b in zip(rings, rings[1:] + rings[:1]):
        for i in range(segs):
            j = (i + 1) % segs
            bm.faces.new((a[i], a[j], b[j], b[i]))
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    bm.transform(Matrix.Translation(Vector(center)) @ _axis_matrix(axis))
    return bm


def rounded_rect(x0, y0, x1, y1, r, n=4):
    pts = []
    for cx, cy, a0 in ((x1 - r, y0 + r, -90), (x1 - r, y1 - r, 0), (x0 + r, y1 - r, 90), (x0 + r, y0 + r, 180)):
        for i in range(n + 1):
            a = math.radians(a0 + 90 * i / n)
            pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
    return pts


def pipe(look, name, pts, r, mat):
    """A tube along a polyline, its corners rounded."""
    cu = bpy.data.curves.new(name, "CURVE")
    cu.dimensions, cu.bevel_depth, cu.bevel_resolution, cu.use_fill_caps = "3D", r, 3, True
    sp = cu.splines.new("POLY")
    pts = _fillet([Vector(p) for p in pts], 2.2 * r)
    sp.points.add(len(pts) - 1)
    for p, co in zip(sp.points, pts):
        p.co = (*co, 1.0)
    tmp = bpy.data.objects.new(name + "_curve", cu)
    bpy.context.scene.collection.objects.link(tmp)
    bpy.context.view_layer.update()
    me = bpy.data.meshes.new_from_object(tmp.evaluated_get(bpy.context.evaluated_depsgraph_get()))
    bpy.data.objects.remove(tmp)
    bpy.data.curves.remove(cu)
    bm = bmesh.new()
    bm.from_mesh(me)
    bpy.data.meshes.remove(me)
    return look.mesh(name, bm, mat, smooth=True)


def _fillet(pts, radius, n=6):
    out = [pts[0]]
    for a, b, c in zip(pts, pts[1:], pts[2:]):
        u, v = (a - b).normalized(), (c - b).normalized()
        p0, p1 = b + u * radius, b + v * radius
        for i in range(n + 1):  # a quadratic Bezier through the corner
            t = i / n
            out.append(p0 * (1 - t) ** 2 + b * 2 * t * (1 - t) + p1 * t * t)
    out.append(pts[-1])
    return out


# -- materials and textures -------------------------------------------------------------------------------------------


def materials():
    M = {}
    M["pcb_edge"] = material("pcb_edge", (0.30, 0.27, 0.12), 0.6)  # the glass-epoxy core seen on the board's edge
    M["chip"] = material("chip", (0.025, 0.025, 0.027), 0.5)
    M["tin"] = material("tin", (0.70, 0.70, 0.68), 0.30, 1.0)
    M["steel"] = material("steel", (0.66, 0.66, 0.68), 0.30, 1.0)
    M["cap"] = material("ceramic_cap", (0.40, 0.30, 0.18), 0.45)
    M["led"] = material("led_amber", (0.80, 0.60, 0.20), 0.25)
    M["seal"] = material("bag_seal", (0.80, 0.25, 0.44), 0.30)  # a little darker and glossier: heat-pressed
    M["bag_inside"] = material("bag_inside", (0.55, 0.13, 0.27), 0.5)
    return M


def bag_texture():
    """The bag's top sheet: pink film with a soft sheen, heat-seal bands on three sides and a printed ESD mark."""
    w, length = BAG_SIZE[0], BAG_SIZE[1]
    cv = Canvas(-w / 2, -length / 2, w, length, 600)
    H, W = cv.h, cv.w
    shade = 0.95 + 0.07 * _noise(H, W, 90) + 0.02 * _noise(H, W, 3)
    rgb = np.array([0.93, 0.45, 0.66])[None, None, :] * shade[..., None]
    seal = np.zeros((H, W), bool)
    for x0, y0, x1, y1 in ((-w / 2, -length / 2, -w / 2 + 0.004, length / 2), (w / 2 - 0.004, -length / 2, w / 2,
                           length / 2), (-w / 2, length / 2 - 0.006, w / 2, length / 2)):
        cv.rect(seal, x0, y0, x1, y1)
    lines = np.zeros((H, W), bool)
    for k in range(4):  # the sealing bar's ribs
        d = 0.0008 + k * 0.0009
        cv.rect(lines, -w / 2 + d, -length / 2, -w / 2 + d + 0.0003, length / 2)
        cv.rect(lines, w / 2 - d - 0.0003, -length / 2, w / 2 - d, length / 2)
        cv.rect(lines, -w / 2, length / 2 - d - 0.0003, w / 2, length / 2 - d)
    rgb[seal] *= 0.88
    rgb[lines] *= 0.78
    # the ESD mark: a yellow triangle with a black border and a black hand-and-arc sign, 2.2 cm across
    y, x = np.mgrid[:H, :W]
    cx, cy = cv.px(0.0, 0.018)
    s = 0.022 * cv.k

    def tri(scale):
        h = s * scale * math.sqrt(3) / 2
        top = cy + 2 * h / 3
        inside = (y <= top) & (y >= top - h)
        half = (top - y) / math.sqrt(3)
        return inside & (np.abs(x - cx) <= half)

    outer, inner = tri(1.0), tri(0.78)
    rgb[outer] = (0.05, 0.05, 0.05)
    rgb[inner] = (0.98, 0.80, 0.10)
    r = np.hypot(x - cx, y - cy + 0.02 * s)
    arc = inner & (np.abs(r - 0.20 * s) < 0.035 * s) & (y > cy - 0.05 * s)
    hand = inner & (np.abs(x - cx) < 0.05 * s) & (y < cy + 0.12 * s) & (y > cy - 0.18 * s)
    slash = inner & (np.abs((x - cx) + (y - cy + 0.02 * s)) < 0.03 * s) & (r < 0.28 * s)
    rgb[arc | hand | slash] = (0.05, 0.05, 0.05)
    # three lines of small print under the mark, as glyph-sized blocks
    m = np.zeros((H, W), bool)
    for row, n in enumerate((9, 13, 11)):
        yy = -0.002 - row * 0.0032
        for i in range(n):
            xx = -0.0011 * n + i * 0.0022
            cv.rect(m, xx, yy, xx + 0.0015, yy + 0.0019)
    rgb[m] = (0.15, 0.05, 0.10)
    return _save("bag_top", rgb)


def pcb_texture(parts, holes):
    """The board's top: green solder mask, copper traces under it, tinned pads, white silkscreen outlines."""
    w, length = BOARD_SIZE[0], BOARD_SIZE[1]
    cv = Canvas(-w / 2, -length / 2, w, length, 1300)
    H, W = cv.h, cv.w
    trace, tin, silk, hole, gold = (np.zeros((H, W), bool) for _ in range(5))
    margin = 20
    for _ in range(60):  # buses
        n, pitch = int(RNG.integers(2, 8)), 6
        d = int(RNG.choice([0, 2, 4, 6]))
        p = np.array([RNG.uniform(margin, W - margin), RNG.uniform(margin, H - margin)])
        pts = [p]
        for _ in range(int(RNG.integers(2, 4))):
            pts.append(pts[-1] + DIRS[d] * RNG.uniform(30, 260))
            d = (d + int(RNG.choice([-1, 1]))) % 8
        perp = np.array([-DIRS[d][1], DIRS[d][0]])
        for i in range(n):
            for a, b in zip(pts, pts[1:]):
                cv.line_px(trace, a + perp * pitch * i, b + perp * pitch * i, 2)
    for _ in range(160):  # single traces ending in vias
        d = int(RNG.integers(0, 8))
        p = np.array([RNG.uniform(margin, W - margin), RNG.uniform(margin, H - margin)])
        q = p + DIRS[d] * RNG.uniform(20, 120)
        cv.line_px(trace, p, q, int(RNG.choice([2, 3, 4])))
        cv.disc_px(tin, q[0], q[1], 4)
        cv.disc_px(hole, q[0], q[1], 1.5)
    for x0, y0, x1, y1, kind in parts:
        cv.outline(silk, x0 - 0.0008, y0 - 0.0008, x1 + 0.0008, y1 + 0.0008, 0.00025)
        if kind == "pads":
            cv.rect(tin, x0 - 0.0004, y0, x0, y1)
            cv.rect(tin, x1, y0, x1 + 0.0004, y1)
    for x, y in holes:
        cx, cy = cv.px(x, y)
        cv.disc_px(tin, cx, cy, 0.0028 * cv.k)
        cv.disc_px(hole, cx, cy, 0.0015 * cv.k)
    for i in range(10):  # a test-point row near the far end
        cv.rect(gold, -0.018 + i * 0.004, length / 2 - 0.006, -0.0165 + i * 0.004, length / 2 - 0.004)
    rgb = np.array([0.06, 0.32, 0.15])[None, None, :] * (0.92 + 0.16 * _noise(H, W, 48)[..., None])
    rgb[trace] = np.array([0.10, 0.42, 0.19])
    rgb[tin] = np.array([0.78, 0.78, 0.74])
    rgb[gold] = np.array([0.86, 0.68, 0.34])
    rgb[silk] = np.array([0.92, 0.92, 0.88])
    rgb[hole] = np.array([0.04, 0.04, 0.035])
    return _save("board_top", rgb)


def mat_texture():
    cv = Canvas(-MAT_SIZE[0] / 2, -MAT_SIZE[1] / 2, MAT_SIZE[0], MAT_SIZE[1], 1000)
    H, W = cv.h, cv.w
    shade = 0.94 + 0.08 * _noise(H, W, 160) + 0.05 * _noise(H, W, 2)
    rgb = np.array([0.24, 0.26, 0.29])[None, None, :] * shade[..., None]
    return _save("mat_top", rgb)


# -- the bag ---------------------------------------------------------------------------------------------------------


def build_bag(M):
    """The walls as ``envelope.py`` builds the colliders, rounded a little, the top sheet printed."""
    L = Look("bag", (0.0, 0.0, 0.0), BAG_SIZE)
    w, length, h = BAG_SIZE
    cw, ch = w - 2 * SEAM, h - 2 * WALL
    mouth = -length / 2
    pink = material("bag_film", (0.80, 0.19, 0.40), 0.32, image=bag_texture())
    uv = (-w / 2, -length / 2, w / 2, length / 2)
    L.mesh("bottom_sheet", bm_box((-w / 2, -length / 2, -h / 2), (w / 2, length / 2, -h / 2 + WALL), uv=uv), pink,
           bevel=0.0006)
    top = L.mesh("top_sheet", bm_box((-w / 2, -length / 2, h / 2 - WALL), (w / 2, length / 2, h / 2), uv=uv), pink,
                 bevel=0.0006)
    L.cut(top, [L.cutter("thumb_notch", bm_box((-NOTCH[0] / 2, mouth - 0.001, h / 2 - WALL - 0.001),
                                               (NOTCH[0] / 2, mouth + NOTCH[1], h / 2 + 0.001)))])
    for name, x0, x1 in (("seam_left", cw / 2, w / 2), ("seam_right", -w / 2, -cw / 2)):
        L.mesh(name, bm_box((x0, -length / 2, -ch / 2), (x1, length / 2, ch / 2)), M["seal"], bevel=0.0004)
    L.mesh("end_seal", bm_box((-cw / 2, length / 2 - END, -ch / 2), (cw / 2, length / 2, ch / 2)), M["seal"],
           bevel=0.0004)
    # the inside: a darker lining, seen through the mouth and the notch
    L.mesh("lining_floor", bm_box((-cw / 2, mouth + 0.0005, -ch / 2), (cw / 2, length / 2 - END, -ch / 2 + 0.0002)),
           M["bag_inside"])
    L.mesh("lining_roof", bm_box((-cw / 2, mouth + 0.0005, ch / 2 - 0.0002), (cw / 2, length / 2 - END, ch / 2)),
           M["bag_inside"])
    return L


# -- the board -------------------------------------------------------------------------------------------------------


def build_board(M):
    L = Look("board", (0.0, 0.0, 0.0), BOARD_SIZE)
    w, length, t = BOARD_SIZE
    z0 = -t / 2 + PCB_T  # the board's top face
    parts, holes = [], []

    def part(name, x0, y0, x1, y1, height, mat, kind="outline", bevel=0.0002):
        parts.append((x0, y0, x1, y1, kind))
        return L.mesh(name, bm_box((x0, y0, z0), (x1, y1, z0 + height)), mat, bevel=bevel)

    # microcontroller (a 10 mm QFP with its leads), two SOIC chips, a crystal, small parts, LEDs, a button
    part("mcu", -0.005, 0.004, 0.005, 0.014, 0.0012, M["chip"], bevel=0.0003)
    for side in (-1, 1):
        for i in range(12):
            a = -0.0045 + i * 0.0008
            L.mesh(f"lead_x{side}_{i}", bm_box((side * 0.005 if side > 0 else -0.0058, 0.0042 + 0.0008 * i + 0.0002,
                                                z0), (0.0058 if side > 0 else -0.005, 0.0042 + 0.0008 * i + 0.0005,
                                                      z0 + 0.0003)), M["tin"])
            L.mesh(f"lead_y{side}_{i}", bm_box((a + 0.0002, 0.0140 if side > 0 else 0.0032, z0),
                                               (a + 0.0005, 0.0148 if side > 0 else 0.0040, z0 + 0.0003)), M["tin"])
    part("soic_a", -0.022, -0.012, -0.012, -0.006, 0.0015, M["chip"], "pads")
    part("soic_b", 0.010, -0.025, 0.020, -0.019, 0.0015, M["chip"], "pads")
    part("crystal", 0.011, 0.006, 0.016, 0.0092, 0.0011, M["steel"], bevel=0.0004)
    part("regulator", -0.020, 0.030, -0.012, 0.036, 0.0019, M["chip"], "pads")
    L.mesh("regulator_tab", bm_box((-0.0195, 0.0362, z0), (-0.0125, 0.0395, z0 + 0.0003)), M["tin"])
    part("button", 0.012, 0.030, 0.018, 0.036, 0.0019, M["steel"], bevel=0.0004)
    L.mesh("button_cap", bm_cyl((0.015, 0.033, z0 + 0.0019), 0.0018, 0.0001, "Z", 20), M["chip"], smooth=True)
    for i in range(3):
        part(f"led_{i}", -0.004 + i * 0.004, 0.044, -0.0024 + i * 0.004, 0.0448, 0.0007, M["led"])
    for k in range(26):
        x, y = RNG.uniform(-0.026, 0.026), RNG.uniform(-0.045, 0.048)
        if abs(x) < 0.008 and 0.002 < y < 0.016:
            continue
        if RNG.random() < 0.5:
            part(f"chip_part_{k}", x, y, x + 0.0016, y + 0.0008, 0.0005, M["cap" if k % 2 else "chip"], "pads")
        else:
            part(f"chip_part_{k}", x, y, x + 0.0008, y + 0.0016, 0.0005, M["cap" if k % 2 else "chip"], "pads")
    for x in (-w / 2 + 0.004, w / 2 - 0.004):
        for y in (-length / 2 + 0.004, length / 2 - 0.004):
            holes.append((x, y))
    top = material("solder_mask", (0.06, 0.30, 0.13), 0.35, image=pcb_texture(parts, holes))
    L.mesh("pcb", bm_box((-w / 2, -length / 2, -t / 2), (w / 2, length / 2, z0), uv=(-w / 2, -length / 2, w / 2,
                                                                                     length / 2)), top, bevel=0.0003)
    return L


# -- the mat ---------------------------------------------------------------------------------------------------------


def build_mat(M):
    L = Look("mat", (0.0, 0.0, 0.0))
    x0, y0, t = -MAT_SIZE[0] / 2, -MAT_SIZE[1] / 2, MAT_SIZE[2]
    L.mesh("mat", bm_box((x0, y0, -t / 2), (-x0, -y0, t / 2), uv=(x0, y0, -x0, -y0)),
           material("esd_mat", (0.10, 0.11, 0.13), 0.85, image=mat_texture()), bevel=0.0012)
    L.mesh("ground_snap", bm_cyl((-x0 - 0.015, -y0 - 0.015, t / 2), 0.0045, 0.0018, "Z", 32), M["steel"],
           smooth=True, bevel=0.0006)
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
        over = max(max(lo[i] * -1 - half[i], hi[i] - half[i]) for i in range(3))
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
    return {"file": os.path.basename(path), "kB": round(os.path.getsize(path) / 1024), "low": tuple(round(v, 4) for v in lo),
            "high": tuple(round(v, 4) for v in hi)}


def clear():
    for coll in list(bpy.data.collections):
        if coll.name.startswith(("look_", "cutters_", "cut_")):
            for ob in list(coll.objects):
                bpy.data.objects.remove(ob)
            bpy.data.collections.remove(coll)
    for block in (bpy.data.meshes, bpy.data.curves, bpy.data.materials):
        for item in list(block):
            if item.users == 0:
                block.remove(item)
    _MATS.clear()


def build():
    clear()
    M = materials()
    looks = [build_bag(M), build_board(M), build_mat(M)]
    return {look.name: export(look) for look in looks}


if __name__ == "__main__":
    print(build())
