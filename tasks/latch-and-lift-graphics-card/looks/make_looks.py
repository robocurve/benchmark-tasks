"""make_looks.py: model the scene's visual meshes in Blender and export them as USD next to this file.

    blender -b -P looks/make_looks.py          (Blender 4.2 or later, from the task folder)

The physics stays the task file's boxes. Each look is a render-only mesh that ``looks.py`` puts on one of those
boxes, whose plain cuboid is then hidden but still collides. Every look is built in the task file's board frame
(u along the slot as x, v across it as y, metres, z up) around an empty at its box's centre, and exported with that
empty at the origin. The task file turns the board's boxes, and so their looks, a quarter turn onto the table.

- ``card.usdc``: the graphics card, inside the card's 18 x 2 x 11 cm box (checked when exported).
- ``latch.usdc``: the orange latch button, inside its 2 x 1.2 x 1.5 cm box (checked).
- ``mainboard.usdc``: the board, its slot, the slot's rails, the latch housing and the board's parts.
- ``mat.usdc`` and ``tray.usdc``: the anti-static mat, and the tray with its foam floor.

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
RNG = np.random.default_rng(11)

# -- the task file's layout (latch_and_lift_graphics_card.py), board frame, metres -----------------------------------
TOP = 0.7475
MAT_C, MAT_SIZE = (0.33, -0.01, TOP + 0.0015), (0.34, 0.30, 0.003)
MAT_TOP = TOP + MAT_SIZE[2]
BOARD_C, BOARD_SIZE = (0.33, -0.01, MAT_TOP + 0.001), (0.30, 0.24, 0.002)
BT = MAT_TOP + BOARD_SIZE[2]  # the board's top
SLOT_X0, SLOT_LEN = 0.26, 0.09
CARD_SIZE = (0.18, 0.02, 0.11)
CARD_C = (SLOT_X0 + CARD_SIZE[0] / 2, 0.0, BT + CARD_SIZE[2] / 2)
RAIL_SIZE, RAIL_Y = (0.09, 0.004, 0.012), 0.014
LATCH_X, LATCH_SIZE = 0.195, (0.02, 0.012, 0.015)
LATCH_C = (LATCH_X, 0.0, BT + 0.005 + LATCH_SIZE[2] / 2)
HOUSING_H, HOUSING_T, HOUSING_IN = 0.008, 0.002, (0.011, 0.007)
TRAY_C, TRAY_OUTER, TRAY_WALL, TRAY_WALL_H = (0.23, 0.23, TOP + 0.005), (0.25, 0.15), 0.01, 0.03
PCB_Y = -0.0055  # the card's circuit board, in the card's frame (the cooler is on +y, the backplate on -y)

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


def materials():
    M = {}
    M["shroud"] = material("shroud", (0.022, 0.022, 0.026), 0.42)
    M["accent"] = material("accent_gunmetal", (0.20, 0.21, 0.23), 0.32, 0.85)
    M["blade"] = material("fan_blade", (0.012, 0.012, 0.014), 0.55)
    M["hub"] = material("fan_hub", (0.06, 0.06, 0.065), 0.35, 0.3)
    M["alu"] = material("aluminium", (0.78, 0.79, 0.80), 0.32, 1.0)
    M["copper"] = material("copper", (0.93, 0.52, 0.33), 0.28, 1.0)
    M["gold"] = material("gold", (1.0, 0.75, 0.33), 0.22, 1.0)
    M["steel"] = material("steel", (0.66, 0.66, 0.68), 0.30, 1.0)
    M["backplate"] = material("backplate", (0.07, 0.07, 0.075), 0.45, 0.7)
    M["pcb"] = material("pcb_black", (0.012, 0.015, 0.013), 0.38)
    M["plastic"] = material("black_plastic", (0.018, 0.018, 0.019), 0.40)
    M["port"] = material("port_dark", (0.004, 0.004, 0.004), 0.6)
    M["latch"] = material("latch_orange", (0.95, 0.22, 0.01), 0.40)  # the one saturated colour on the board
    M["grey_latch"] = material("clip_black", (0.03, 0.03, 0.032), 0.45)  # dark, so nothing else looks like the latch
    M["coin"] = material("coin_cell", (0.22, 0.22, 0.23), 0.40, 1.0)
    M["choke"] = material("choke", (0.13, 0.13, 0.135), 0.6, 0.2)
    M["cap"] = material("polymer_cap", (0.07, 0.05, 0.09), 0.3, 0.4)
    M["chip"] = material("chip", (0.025, 0.025, 0.027), 0.5)
    M["sink"] = material("heatsink_gunmetal", (0.12, 0.125, 0.135), 0.35, 0.9)
    M["audio_cap"] = material("audio_cap", (0.05, 0.04, 0.035), 0.3, 0.5)  # dark: orange is the latch's alone
    M["tray"] = material("tray_plastic", (0.02, 0.02, 0.022), 0.48)
    return M


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


def pcb_texture(footprints, holes):
    """The mainboard's top: green solder mask over copper traces, tinned pads, small parts and white silkscreen."""
    cv = Canvas(BOARD_C[0] - BOARD_SIZE[0] / 2, BOARD_C[1] - BOARD_SIZE[1] / 2, BOARD_SIZE[0], BOARD_SIZE[1], 2400)
    H, W = cv.h, cv.w
    trace, tin, gold, silk, hole = (np.zeros((H, W), bool) for _ in range(5))
    tan, black = np.zeros((H, W), bool), np.zeros((H, W), bool)
    margin = 12
    # buses of parallel traces, with 45 degree bends
    for _ in range(170):
        n, pitch = int(RNG.integers(3, 14)), 5
        d = int(RNG.choice([0, 2, 4, 6]))
        p = np.array([RNG.uniform(margin, W - margin), RNG.uniform(margin, H - margin)])
        pts = [p]
        for _ in range(int(RNG.integers(2, 5))):
            pts.append(pts[-1] + DIRS[d] * RNG.uniform(40, 420))
            d = (d + int(RNG.choice([-1, 1]))) % 8
        perp = np.array([-DIRS[d][1], DIRS[d][0]])
        for i in range(n):
            for a, b in zip(pts, pts[1:]):
                cv.line_px(trace, a + perp * pitch * i, b + perp * pitch * i, 2)
    # single traces ending in vias
    for _ in range(420):
        d = int(RNG.integers(0, 8))
        p = np.array([RNG.uniform(margin, W - margin), RNG.uniform(margin, H - margin)])
        q = p + DIRS[d] * RNG.uniform(20, 160)
        r = q + DIRS[(d + int(RNG.choice([-1, 1]))) % 8] * RNG.uniform(10, 120)
        width = int(RNG.choice([2, 2, 3, 5]))
        cv.line_px(trace, p, q, width)
        cv.line_px(trace, q, r, width)
        cv.disc_px(tin, r[0], r[1], 3.5)
        cv.disc_px(hole, r[0], r[1], 1.2)
    # small surface-mount parts: two tinned pads and a tan (capacitor) or black (resistor) body
    for _ in range(1300):
        x, y = RNG.uniform(margin, W - margin), RNG.uniform(margin, H - margin)
        L, Wd = (8, 4) if RNG.random() < 0.7 else (12, 6)
        if RNG.random() < 0.5:
            L, Wd = Wd, L
            tin[int(y - L / 2):int(y - L / 2) + 3, int(x - Wd / 2):int(x + Wd / 2)] = True
            tin[int(y + L / 2) - 3:int(y + L / 2), int(x - Wd / 2):int(x + Wd / 2)] = True
            body = (slice(int(y - L / 2) + 3, int(y + L / 2) - 3), slice(int(x - Wd / 2), int(x + Wd / 2)))
        else:
            tin[int(y - Wd / 2):int(y + Wd / 2), int(x - L / 2):int(x - L / 2) + 3] = True
            tin[int(y - Wd / 2):int(y + Wd / 2), int(x + L / 2) - 3:int(x + L / 2)] = True
            body = (slice(int(y - Wd / 2), int(y + Wd / 2)), slice(int(x - L / 2) + 3, int(x + L / 2) - 3))
        (tan if RNG.random() < 0.6 else black)[body] = True
    # pads, outlines and labels round the modelled parts
    for x0, y0, x1, y1, kind in footprints:
        if kind == "pins":
            n = max(int((x1 - x0) / 0.0008), 2)
            for i in range(n):
                xi = x0 + (i + 0.5) * (x1 - x0) / n
                cv.rect(tin, xi - 0.0002, y0 - 0.0012, xi + 0.0002, y0)
                cv.rect(tin, xi - 0.0002, y1, xi + 0.0002, y1 + 0.0012)
        cv.outline(silk, x0 - 0.0015, y0 - 0.0015, x1 + 0.0015, y1 + 0.0015)
        lx, ly = x0, y1 + 0.0022
        for i in range(int(RNG.integers(2, 5))):  # a label: a few glyph-sized blocks
            cv.rect(silk, lx + i * 0.0011, ly, lx + i * 0.0011 + 0.0007, ly + 0.0011)
    for x, y in holes:
        cx, cy = cv.px(x, y)
        cv.disc_px(tin, cx, cy, 0.0048 * cv.k)
        cv.disc_px(hole, cx, cy, 0.0018 * cv.k)
    # colour
    base = np.array([0.07, 0.30, 0.14])
    rgb = base[None, None, :] * (0.92 + 0.16 * _noise(H, W, 64)[..., None])
    rgb[trace] = np.array([0.11, 0.40, 0.19])
    rgb[tan] = np.array([0.62, 0.50, 0.34])
    rgb[black] = np.array([0.05, 0.05, 0.05])
    rgb[tin] = np.array([0.78, 0.78, 0.74])
    rgb[gold] = np.array([0.86, 0.68, 0.34])
    rgb[silk] = np.array([0.92, 0.92, 0.88])
    rgb[hole] = np.array([0.04, 0.04, 0.035])
    rgb *= (0.97 + 0.06 * _noise(H, W, 3))[..., None]
    return _save("mainboard_top", rgb)


def mat_texture():
    cv = Canvas(MAT_C[0] - MAT_SIZE[0] / 2, MAT_C[1] - MAT_SIZE[1] / 2, MAT_SIZE[0], MAT_SIZE[1], 1200)
    H, W = cv.h, cv.w
    shade = 0.94 + 0.08 * _noise(H, W, 160) + 0.05 * _noise(H, W, 2)
    rgb = np.array([0.34, 0.38, 0.45])[None, None, :] * shade[..., None]
    return _save("mat_top", rgb)


def foam_texture():
    """Pick-and-pluck foam: 12.5 mm squares cut into grey foam."""
    cv = Canvas(TRAY_C[0] - 0.125, TRAY_C[1] - 0.075, 0.25, 0.15, 1200)
    H, W = cv.h, cv.w
    y, x = np.mgrid[:H, :W]
    cell = 0.0125 * cv.k
    cut = ((x % cell) < 2.2) | ((y % cell) < 2.2)
    cells = RNG.random((int(H / cell) + 2, int(W / cell) + 2))
    tone = 0.95 + 0.1 * cells[(y / cell).astype(int), (x / cell).astype(int)]
    pores = 0.9 + 0.2 * _noise(H, W, 2)
    rgb = np.array([0.20, 0.20, 0.215])[None, None, :] * (tone * pores)[..., None]
    rgb[cut] *= 0.35
    return _save("foam_top", rgb)


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


# -- the graphics card -----------------------------------------------------------------------------------------------


def build_card(M):
    L = Look("card", CARD_C, CARD_SIZE)
    cx, cz = CARD_C[0], CARD_C[2]

    def X(x):
        return cx + x

    def Z(z):
        return cz + z

    yp0, yp1 = PCB_Y - 0.0008, PCB_Y + 0.0008
    # circuit board, and its edge connector: 89 mm of tab with the key notch 11.65 mm from the bracket end
    L.mesh("pcb", bm_box((X(-0.087), yp0, Z(-0.044)), (X(0.0875), yp1, Z(0.050))), M["pcb"], bevel=0.0003)
    notch = (-0.0136, -0.0117)
    for i, (a, b) in enumerate(((-0.0895, notch[0]), (notch[1], -0.0005))):
        L.mesh(f"pcb_tab_{i}", bm_box((X(a), yp0, Z(-0.0545)), (X(b), yp1, Z(-0.044))), M["pcb"], bevel=0.0002)
        for side, y in (("a", yp0 - 0.00004), ("b", yp1 + 0.00004)):
            fingers = L.mesh(f"fingers_{i}{side}", bm_box((X(a + 0.0003), y - 0.00002, Z(-0.0538)),
                                                         (X(a + 0.0009), y + 0.00002, Z(-0.0462))), M["gold"])
            arr = fingers.modifiers.new("array", "ARRAY")
            arr.use_relative_offset, arr.use_constant_offset = False, True
            arr.constant_offset_displace = (0.001, 0, 0)
            arr.count = int((b - a - 0.0006) / 0.001)
    # fin stack and heat pipes on the +y side of the board
    fin = L.mesh("fins", bm_box((X(-0.080), yp1 + 0.0002, Z(-0.042)), (X(-0.0797), 0.0049, Z(0.0495))), M["alu"])
    arr = fin.modifiers.new("array", "ARRAY")
    arr.use_relative_offset, arr.use_constant_offset = False, True
    arr.constant_offset_displace, arr.count = (0.0019, 0, 0), 84
    for i, (y, z) in enumerate(((-0.0016, 0.0462), (0.0026, 0.0442))):
        pipe(L, f"heatpipe_{i}", [(X(-0.083), y, Z(-0.030)), (X(-0.083), y, Z(z)), (X(0.078), y, Z(z)),
                                  (X(0.078), y, Z(-0.024))], 0.0021, M["copper"])
    # shroud: a faceted plate with two fan openings, gunmetal accents
    sh = [(-0.088, -0.034), (-0.079, -0.043), (0.071, -0.043), (0.080, -0.034), (0.080, 0.036), (0.072, 0.044),
          (-0.080, 0.044), (-0.088, 0.036)]
    shroud = L.mesh("shroud", bm_prism([(X(u), Z(v)) for u, v in sh], 0.0050, 0.0094), M["shroud"], bevel=0.0006)
    fans = ((-0.044, 0.0005), (0.036, 0.0005))
    L.cut(shroud, [L.cutter(f"fan_hole_{i}", bm_cyl((X(fx), 0.007, Z(fz)), 0.0385, 0.01, "Y", 64))
                   for i, (fx, fz) in enumerate(fans)])
    for i, (fx, fz) in enumerate(fans):
        L.mesh(f"fan_ring_{i}", bm_ring((X(fx), 0.0094, Z(fz)), 0.0385, 0.0405, 0.0008, "Y", 64), M["accent"],
               smooth=True)
        build_fan(L, M, i, X(fx), Z(fz))
    for i, pts in enumerate(([(-0.086, 0.037), (-0.079, 0.044), (-0.052, 0.044), (-0.058, 0.040), (-0.077, 0.040)],
                             [(0.078, -0.035), (0.071, -0.043), (0.046, -0.043), (0.052, -0.039), (0.070, -0.039)],
                             [(-0.004, 0.044), (0.004, 0.044), (0.001, 0.036), (-0.001, 0.036)])):
        L.mesh(f"accent_{i}", bm_prism([(X(u), Z(v)) for u, v in pts], 0.0094, 0.0099), M["accent"])
    # backplate with screws round the processor
    L.mesh("backplate", bm_box((X(-0.0865), -0.0089, Z(-0.042)), (X(0.0855), -0.0079, Z(0.0495))), M["backplate"],
           bevel=0.0005)
    for i, (sx, sz) in enumerate(((-0.020, -0.012), (0.010, -0.012), (-0.020, 0.018), (0.010, 0.018))):
        L.mesh(f"screw_{i}", bm_cyl((X(sx), -0.0093, Z(sz)), 0.0022, 0.0008, "Y", 24), M["steel"], smooth=True)
    for i, (u0, v0, u1, v1) in enumerate(((-0.075, -0.036, -0.035, -0.030), (0.035, 0.036, 0.075, 0.042))):
        L.mesh(f"backplate_rib_{i}", bm_box((X(u0), -0.0094, Z(v0)), (X(u1), -0.0089, Z(v1))), M["accent"],
               bevel=0.0002)
    # 8-pin power socket on the top edge
    pw = L.mesh("power_socket", bm_box((X(0.050), -0.0046, Z(0.0465)), (X(0.0665), 0.0042, Z(0.0549))), M["plastic"],
                bevel=0.0003)
    holes = [L.cutter(f"power_pin_{i}{j}", bm_box((X(0.0515 + i * 0.0039), -0.0036 + j * 0.0041, Z(0.049)),
                                                  (X(0.0546 + i * 0.0039), -0.0036 + j * 0.0041 + 0.0032, Z(0.056))))
             for i in range(4) for j in range(2)]
    L.cut(pw, holes)
    build_bracket(L, M, X, Z)
    return L


def build_fan(L, M, i, fx, fz):
    L.mesh(f"fan_hub_{i}", bm_cyl((fx, 0.0073, fz), 0.0135, 0.0036, "Y", 48), M["hub"], smooth=True, bevel=0.0008)
    L.mesh(f"fan_cap_{i}", bm_cyl((fx, 0.00925, fz), 0.0105, 0.0003, "Y", 48), M["accent"], smooth=True)
    blades, nr, na = 11, 6, 5
    bm = bmesh.new()
    for k in range(blades):
        a0 = 2 * math.pi * k / blades + i * 0.3
        grid = []
        for ir in range(nr):
            r = 0.0137 + (0.0374 - 0.0137) * ir / (nr - 1)
            sweep = 0.38 * ir / (nr - 1)
            width = 0.42 + 0.12 * ir / (nr - 1)
            row = []
            for ia in range(na):
                t = ia / (na - 1)
                a = a0 + sweep + width * t
                y = 0.0060 + 0.0026 * t + 0.0003 * math.sin(math.pi * t)
                row.append(bm.verts.new((fx + r * math.cos(a), y, fz + r * math.sin(a))))
            grid.append(row)
        for ir in range(nr - 1):
            for ia in range(na - 1):
                bm.faces.new((grid[ir][ia], grid[ir][ia + 1], grid[ir + 1][ia + 1], grid[ir + 1][ia]))
    ob = L.mesh(f"fan_blades_{i}", bm, M["blade"], smooth=True)
    sol = ob.modifiers.new("solid", "SOLIDIFY")
    sol.thickness, sol.offset = 0.0005, 0.0


def build_bracket(L, M, X, Z):
    x0, x1 = X(0.0888), X(0.0898)
    plate = L.mesh("bracket", bm_box((x0, -0.0099, Z(-0.050)), (x1, 0.0099, Z(0.0545))), M["steel"], bevel=0.0002)
    cut = []
    ports = [("dp", 0.031), ("dp", 0.019), ("dp", 0.007), ("hdmi", -0.006)]
    for k, (kind, zc) in enumerate(ports):
        if kind == "dp":  # DisplayPort: 16 x 5 mm with one corner cut
            pts = [(-0.0080, -0.0024), (0.0068, -0.0024), (0.0080, -0.0012), (0.0080, 0.0024), (-0.0080, 0.0024)]
        else:  # HDMI: a 14 x 4.5 mm trapezoid
            pts = [(-0.0072, 0.0023), (0.0072, 0.0023), (0.0072, -0.0005), (0.0060, -0.0023), (-0.0060, -0.0023),
                   (-0.0072, -0.0005)]
        cut.append(L.cutter(f"port_hole_{k}", bm_prism([(u, Z(zc + v)) for u, v in pts], x0 - 0.001, x1 + 0.001,
                                                        "X")))
        L.mesh(f"port_{k}", bm_box((X(0.080), -0.0089, Z(zc - 0.0032)), (x0 - 0.00005, 0.0089, Z(zc + 0.0032))),
               M["port"])
        L.mesh(f"port_tongue_{k}", bm_box((X(0.083), -0.0055, Z(zc - 0.0006)), (x0 + 0.0003, 0.0055, Z(zc + 0.0002))),
               M["plastic"])
    for row in range(10):
        zc = -0.044 + row * 0.0027
        for col, yc in enumerate((-0.0045, 0.0045)):
            cut.append(L.cutter(f"vent_{row}_{col}", bm_box((x0 - 0.001, yc - 0.0033, Z(zc - 0.0007)),
                                                            (x1 + 0.001, yc + 0.0033, Z(zc + 0.0007)))))
    L.cut(plate, cut)


# -- the latch -------------------------------------------------------------------------------------------------------


def build_latch(M):
    L = Look("latch", LATCH_C, LATCH_SIZE)
    cx, cy, cz = LATCH_C

    def P(x, y, z):
        return (cx + x, cy + y, cz + z)

    L.mesh("button", bm_box(P(-0.0095, -0.0055, -0.0075), P(0.0085, 0.0055, 0.0058)), M["latch"], bevel=0.0007)
    L.mesh("hook", bm_box(P(0.0080, -0.0045, -0.0075), P(0.0099, 0.0045, -0.0036)), M["latch"], bevel=0.0004)
    for i, x in enumerate((-0.0062, -0.0021, 0.0020)):
        L.mesh(f"ridge_{i}", bm_box(P(x - 0.0007, -0.0047, 0.0050), P(x + 0.0007, 0.0047, 0.0073)), M["latch"],
               bevel=0.0004)
    L.mesh("arrow", bm_prism([(cx + 0.0048, cy - 0.0030), (cx + 0.0080, cy), (cx + 0.0048, cy + 0.0030)],
                             cz + 0.0056, cz + 0.0060, "Z"), M["latch"])
    return L


# -- the mainboard, its slots, the rails and the latch housing --------------------------------------------------------


def build_board(M):
    L = Look("mainboard", BOARD_C)
    footprints, holes = [], []
    z0 = BT

    def part(name, lo, hi, mat, bevel=0.0003, kind="outline"):
        footprints.append((lo[0], lo[1], hi[0], hi[1], kind))
        return L.mesh(name, bm_box((lo[0], lo[1], z0), (hi[0], hi[1], z0 + hi[2])), mat, bevel=bevel)

    def slot(name, x0, x1, yc, latch_x=None):
        """A PCIe slot: black body 7.5 mm wide, 11 mm tall, a 1.8 mm slit with gold contacts and the key."""
        body = part(name, (x0, yc - 0.00375), (x1, yc + 0.00375, 0.011), M["plastic"], bevel=0.0004)
        key = x1 - 0.0005 - 0.01165
        cuts = [L.cutter(f"{name}_slit_a", bm_box((x0 + 0.0015, yc - 0.0009, z0 + 0.003), (key - 0.0019, yc + 0.0009,
                                                                                            z0 + 0.012))),
                L.cutter(f"{name}_slit_b", bm_box((key, yc - 0.0009, z0 + 0.003), (x1 - 0.0015, yc + 0.0009,
                                                                                   z0 + 0.012)))]
        L.cut(body, cuts)
        for side in (-1, 1):
            pins = L.mesh(f"{name}_pins_{side}", bm_box((x0 + 0.0017, yc + side * 0.00088 - 0.00004, z0 + 0.004),
                                                        (x0 + 0.0022, yc + side * 0.00088 + 0.00004, z0 + 0.0095)),
                          M["gold"])
            arr = pins.modifiers.new("array", "ARRAY")
            arr.use_relative_offset, arr.use_constant_offset = False, True
            arr.constant_offset_displace, arr.count = (0.001, 0, 0), int((x1 - x0 - 0.004) / 0.001)
        if latch_x is not None:
            L.mesh(f"{name}_latch", bm_box((latch_x - 0.006, yc - 0.0032, z0), (latch_x, yc + 0.0032, z0 + 0.0115)),
                   M["grey_latch"], bevel=0.0005)

    # the board itself, textured on top
    bx0, by0 = BOARD_C[0] - BOARD_SIZE[0] / 2, BOARD_C[1] - BOARD_SIZE[1] / 2
    bx1, by1 = bx0 + BOARD_SIZE[0], by0 + BOARD_SIZE[1]
    board = L.mesh("board", bm_box((bx0, by0, BT - BOARD_SIZE[2]), (bx1, by1, BT), uv=(bx0, by0, bx1, by1)),
                   M["plastic"], bevel=0.0004)

    # the slot that holds the card (under the card's circuit board), its two rails and the latch housing
    slot("pcie_x16", SLOT_X0 + 0.0005, SLOT_X0 + SLOT_LEN - 0.0005, PCB_Y)
    for side, s in (("left", 1), ("right", -1)):
        y_in, y_out = s * (RAIL_Y - RAIL_SIZE[1] / 2), s * (RAIL_Y + RAIL_SIZE[1] / 2)
        lo_y, hi_y = min(y_in, y_out - s * 0.0004), max(y_in, y_out - s * 0.0004)
        L.mesh(f"rail_{side}", bm_box((SLOT_X0, lo_y, z0), (SLOT_X0 + SLOT_LEN, hi_y, z0 + RAIL_SIZE[2])),
               M["plastic"], bevel=0.0004)
        a, b = sorted((y_out - s * 0.0004, y_out))
        L.mesh(f"rail_{side}_armor", bm_box((SLOT_X0 + 0.002, a, z0 + 0.0015), (SLOT_X0 + SLOT_LEN - 0.002, b,
                                                                               z0 + 0.0112)), M["steel"])
    hx, hy = HOUSING_IN
    t, h = HOUSING_T, HOUSING_H
    for name, lo, hi in (("housing_near", (LATCH_X - hx - t, -hy - t), (LATCH_X - hx, hy + t)),
                         ("housing_far", (LATCH_X + hx, -hy - t), (LATCH_X + hx + t, hy + t)),
                         ("housing_left", (LATCH_X - hx, hy), (LATCH_X + hx, hy + t)),
                         ("housing_right", (LATCH_X - hx, -hy - t), (LATCH_X + hx, -hy))):
        L.mesh(name, bm_box((lo[0], lo[1], z0), (hi[0], hi[1], z0 + h)), M["plastic"], bevel=0.0003)
    footprints.append((LATCH_X - hx - t, -hy - t, LATCH_X + hx + t, hy + t, "outline"))

    # a second x16 slot and an x1 slot, empty
    slot("pcie_x16_2", SLOT_X0 + 0.0005, SLOT_X0 + SLOT_LEN - 0.0005, 0.061, latch_x=SLOT_X0 + 0.0005)
    slot("pcie_x1", SLOT_X0 + 0.0005, SLOT_X0 + 0.0255, 0.095)

    # processor socket: black base, steel load frame and lever, the gold contact field
    sx0, sy0, sx1, sy1 = 0.2865, -0.0945, 0.3240, -0.0495
    part("socket_base", (sx0, sy0), (sx1, sy1, 0.0022), M["plastic"])
    L.mesh("socket_contacts", bm_box((sx0 + 0.003, sy0 + 0.003, z0), (sx1 - 0.003, sy1 - 0.003, z0 + 0.0024)),
           material("contacts", (0.62, 0.48, 0.26), 0.45, 1.0))
    frame = L.mesh("socket_frame", bm_box((sx0 - 0.002, sy0 - 0.002, z0 + 0.0022), (sx1 + 0.002, sy1 + 0.002,
                                                                                   z0 + 0.0030)), M["steel"])
    L.cut(frame, [L.cutter("socket_window", bm_box((sx0 + 0.0035, sy0 + 0.0035, z0), (sx1 - 0.0035, sy1 - 0.0035,
                                                                                     z0 + 0.01)))])
    pipe(L, "socket_lever", [(sx1 + 0.0042, sy0 - 0.004, z0 + 0.0028), (sx1 + 0.0042, sy1 + 0.002, z0 + 0.0028),
                             (sx1 + 0.0010, sy1 + 0.002, z0 + 0.0028)], 0.0007, M["steel"])
    # power stage: chokes and polymer capacitors round the socket
    for i in range(7):
        y = -0.0915 + i * 0.0065
        part(f"choke_a{i}", (0.2700, y - 0.00275), (0.2755, y + 0.00275, 0.0045), M["choke"], bevel=0.0004)
        L.mesh(f"cap_a{i}", bm_cyl((0.2625, y, z0 + 0.0033), 0.0028, 0.0066, "Z", 24), M["cap"], smooth=True,
               bevel=0.0004)
    for i in range(5):
        x = 0.2890 + i * 0.0075
        part(f"choke_b{i}", (x - 0.00275, -0.0428), (x + 0.00275, -0.0373, 0.0045), M["choke"], bevel=0.0004)
    # memory slots, with grey clips at both ends
    for i, yc in enumerate((-0.1085, -0.1190)):
        body = part(f"dimm_{i}", (0.347, yc - 0.003), (0.474, yc + 0.003, 0.0065), M["plastic"], bevel=0.0004)
        L.cut(body, [L.cutter(f"dimm_{i}_slit", bm_box((0.349, yc - 0.0008, z0 + 0.002), (0.472, yc + 0.0008,
                                                                                          z0 + 0.01)))])
        for x in (0.3430, 0.4745):
            L.mesh(f"dimm_{i}_clip_{int(x * 1000)}", bm_box((x, yc - 0.0032, z0), (x + 0.0040, yc + 0.0032, z0 + 0.0085)),
                   M["grey_latch"], bevel=0.0005)
    # 24-pin power connector at the edge
    atx = part("atx_24pin", (0.4680, -0.0980), (0.4780, -0.0460, 0.0095), M["plastic"], bevel=0.0004)
    L.cut(atx, [L.cutter(f"atx_pin_{i}{j}", bm_box((0.4690 + j * 0.0045, -0.0972 + i * 0.0042, z0 + 0.003),
                                                   (0.4725 + j * 0.0045, -0.0972 + i * 0.0042 + 0.0034, z0 + 0.012)))
                for i in range(12) for j in range(2)])
    # chipset and M.2 heatsinks
    part("chipset_sink", (0.3950, 0.0400), (0.4350, 0.0800, 0.0045), M["sink"], bevel=0.0006)
    for i in range(6):
        y = 0.0440 + i * 0.0064
        L.mesh(f"chipset_fin_{i}", bm_box((0.3975, y, z0 + 0.0045), (0.4325, y + 0.0028, z0 + 0.0060)), M["sink"],
               bevel=0.0003)
    part("m2_sink", (0.2700, 0.0280), (0.3500, 0.0480, 0.0035), M["sink"], bevel=0.0006)
    for i in range(3):
        x = 0.2850 + i * 0.022
        L.mesh(f"m2_groove_{i}", bm_box((x, 0.0290, z0 + 0.0035), (x + 0.012, 0.0470, z0 + 0.0039)), M["accent"])
    # coin cell in its holder
    footprints.append((0.2035, 0.0485, 0.2265, 0.0715, "outline"))
    L.mesh("coin_holder", bm_cyl((0.215, 0.060, z0 + 0.0011), 0.0115, 0.0022, "Z", 48), M["plastic"], smooth=True)
    L.mesh("coin_cell", bm_cyl((0.215, 0.060, z0 + 0.0026), 0.0100, 0.0032, "Z", 48), M["coin"], smooth=True,
           bevel=0.0004)
    # chips: super I/O, audio codec, network, firmware
    for name, lo, hi in (("superio", (0.1960, -0.0760), (0.2100, -0.0620)), ("codec", (0.1960, -0.1000), (0.2030,
                                                                                                          -0.0930)),
                         ("lan", (0.4420, 0.0150), (0.4500, 0.0230)), ("bios", (0.2330, 0.0820), (0.2390, 0.0870))):
        footprints.append((lo[0], lo[1], hi[0], hi[1], "pins"))
        L.mesh(f"chip_{name}", bm_box((lo[0], lo[1], z0), (hi[0], hi[1], z0 + 0.0012)), M["chip"], bevel=0.0002)
    for i in range(3):
        L.mesh(f"audio_cap_{i}", bm_cyl((0.1920 + i * 0.0072, -0.1120, z0 + 0.0035), 0.0030, 0.0070, "Z", 24),
               M["audio_cap"], smooth=True, bevel=0.0004)
    # SATA ports at the near edge
    for i in range(4):
        y = 0.0300 + i * 0.0080
        sata = part(f"sata_{i}", (0.1830, y), (0.1920, y + 0.0062, 0.0060), M["plastic"], bevel=0.0003)
        L.cut(sata, [L.cutter(f"sata_{i}_mouth", bm_box((0.1820, y + 0.0012, z0 + 0.0018), (0.1900, y + 0.0050,
                                                                                            z0 + 0.0045)))])
    # front-panel header
    L.mesh("header_base", bm_box((0.2000, -0.1220, z0), (0.2130, -0.1170, z0 + 0.0025)), M["plastic"])
    for i in range(5):
        for j in range(2):
            L.mesh(f"header_pin_{i}{j}", bm_box((0.2012 + i * 0.00254 - 0.0003, -0.1207 + j * 0.00254 - 0.0003,
                                                 z0 + 0.0025), (0.2012 + i * 0.00254 + 0.0003,
                                                                -0.1207 + j * 0.00254 + 0.0003, z0 + 0.0080)),
                   M["gold"])
    # mounting screws
    for i, (x, y) in enumerate(((0.1890, -0.1210), (0.1890, 0.1010), (0.3350, 0.1010), (0.4710, 0.1010),
                                (0.4710, -0.0340), (0.3450, -0.0350))):
        holes.append((x, y))
        L.mesh(f"screw_{i}", bm_cyl((x, y, z0 + 0.0008), 0.0036, 0.0016, "Z", 32), M["steel"], smooth=True,
               bevel=0.0005)

    board.data.materials[0] = material("mainboard", (0.06, 0.25, 0.10), 0.35, image=pcb_texture(footprints, holes))
    return L


# -- the mat and the tray --------------------------------------------------------------------------------------------


def build_mat(M):
    L = Look("mat", MAT_C)  # its ground snap stands 1 mm proud
    x0, y0 = MAT_C[0] - MAT_SIZE[0] / 2, MAT_C[1] - MAT_SIZE[1] / 2
    x1, y1 = x0 + MAT_SIZE[0], y0 + MAT_SIZE[1]
    L.mesh("mat", bm_box((x0, y0, TOP), (x1, y1, MAT_TOP), uv=(x0, y0, x1, y1)),
           material("esd_mat", (0.12, 0.14, 0.19), 0.85, image=mat_texture()), bevel=0.0012)
    L.mesh("ground_snap", bm_cyl((x0 + 0.015, y1 - 0.015, MAT_TOP), 0.0045, 0.0018, "Z", 32), M["steel"],
           smooth=True, bevel=0.0006)
    return L


def build_tray(M):
    L = Look("tray", TRAY_C)
    tx, ty = TRAY_C[0], TRAY_C[1]
    ox, oy = TRAY_OUTER[0] / 2, TRAY_OUTER[1] / 2
    w = TRAY_WALL
    fx0, fy0, fx1, fy1 = tx - ox + w, ty - oy + w, tx + ox - w, ty + oy - w
    L.mesh("foam", bm_box((fx0, fy0, TOP), (fx1, fy1, TOP + 0.01), uv=(tx - ox, ty - oy, tx + ox, ty + oy)),
           material("foam", (0.03, 0.03, 0.035), 0.95, image=foam_texture()), bevel=0.0008)
    wall = (TOP, TOP + TRAY_WALL_H)
    for name, lo, hi in (("wall_near", (tx - ox, ty - oy), (tx - ox + w, ty + oy)),
                         ("wall_far", (tx + ox - w, ty - oy), (tx + ox, ty + oy)),
                         ("wall_right", (tx - ox + w, ty - oy), (tx + ox - w, ty - oy + w)),
                         ("wall_left", (tx - ox + w, ty + oy - w), (tx + ox - w, ty + oy))):
        L.mesh(name, bm_box((lo[0], lo[1], wall[0]), (hi[0], hi[1], wall[1])), M["tray"], bevel=0.0015)
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
    looks = [build_card(M), build_latch(M), build_board(M), build_mat(M), build_tray(M)]
    return {look.name: export(look) for look in looks}


if __name__ == "__main__":
    print(build())
