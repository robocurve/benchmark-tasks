"""make_looks.py: model the scene's visual meshes in Blender and export them as USD next to this file.

    blender -b -P looks/make_looks.py          (Blender 4.2 or later, from the task folder)

The physics stays the task file's boxes. Each look is a render-only mesh that ``looks.py`` puts on one of those
boxes, whose plain cuboid is then hidden but still collides. Every look is built in the task file's frame (metres,
x forward, y to the robot's left, z up) around an empty at its box's centre, and exported with that empty at the
origin. No box is turned.

- ``tool.usdc``: the discharge tool, inside the tool's 22 x 1.5 x 1.5 cm box (checked when exported).
- ``capacitor.usdc``: the blue capacitor and its two pads, inside its 3 x 3 x 4 cm box (checked).
- ``board.usdc``: the circuit board, inside its 10 x 14 x 0.2 cm box (checked).
- ``heatsink.usdc``: the finned heatsink, and the board's other parts (transformer, small capacitors, coil), drawn
  only.
- ``case.usdc``: the steel case, its rails, the fan grille and the mains inlet.
- ``box.usdc``: the cardboard box.

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
RNG = np.random.default_rng(7)

# -- the task file's layout (discharge_psu_capacitor.py), metres --------------------------------------------------
TOP = 0.7475
CASE_FRONT, CASE_SIZE, WALL = 0.29, (0.14, 0.15, 0.086), 0.003
CASE_BACK = CASE_FRONT + CASE_SIZE[0]
CASE_C = (CASE_FRONT + CASE_SIZE[0] / 2, 0.0, TOP + WALL / 2)  # the floor's box
INNER_Y = CASE_SIZE[1] / 2 - WALL
RAIL_X0, RAIL_LEN, RAIL_W = CASE_FRONT + 0.002, 0.10, 0.012
BOARD_SIZE = (0.10, 0.132, 0.002)
BOARD_BOTTOM = TOP + 0.012
BOARD_TOP = BOARD_BOTTOM + BOARD_SIZE[2]
BOARD_C = (RAIL_X0 + BOARD_SIZE[0] / 2, 0.0, BOARD_BOTTOM + BOARD_SIZE[2] / 2)
LIP_Z0 = BOARD_TOP + 0.003
CAP_SIZE = (0.03, 0.03, 0.04)
CAP_C = (BOARD_C[0] + 0.025, -0.038, BOARD_TOP + CAP_SIZE[2] / 2)
PAD_GAP = 0.01
SINK_SIZE = (0.06, 0.012, 0.04)
SINK_C = (BOARD_C[0] - 0.01, 0.012, BOARD_TOP + SINK_SIZE[2] / 2)
TOOL_SIZE = (0.22, 0.015, 0.015)
TOOL_C = (0.25, -0.30, TOP + TOOL_SIZE[2] / 2)
BOX_C2, BOX_OUTER, BOX_WALL = (0.25, 0.20), (0.30, 0.20, 0.10), 0.006
BOX_C = (*BOX_C2, TOP + BOX_WALL / 2)

# -- materials (linear colours) --------------------------------------------------------------------------------------
_MATS = {}


def material(name, color=(0.5, 0.5, 0.5), rough=0.5, metal=0.0, image=None):
    if name in _MATS:
        return _MATS[name]
    m = bpy.data.materials.get(name) or bpy.data.materials.new(name)
    try:
        m.use_nodes = True
    except AttributeError:  # Blender 5: always on
        pass
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
    M["red"] = material("tool_red", (0.70, 0.012, 0.008), 0.35)  # the one red thing on the table
    M["head"] = material("tool_head", (0.015, 0.015, 0.016), 0.45)
    M["pin"] = material("pin_steel", (0.80, 0.80, 0.82), 0.22, 1.0)
    M["sleeve"] = material("cap_sleeve", (0.012, 0.06, 0.55), 0.30)  # the one blue thing
    M["sleeve_band"] = material("cap_band", (0.25, 0.45, 0.85), 0.35)
    M["can"] = material("cap_can", (0.75, 0.76, 0.78), 0.25, 1.0)
    M["vent"] = material("cap_vent", (0.20, 0.20, 0.21), 0.4, 1.0)
    M["tin"] = material("pad_tin", (0.88, 0.88, 0.86), 0.18, 1.0)
    M["rubber"] = material("rubber", (0.02, 0.02, 0.02), 0.7)
    M["alu"] = material("aluminium", (0.70, 0.71, 0.73), 0.35, 1.0)
    M["black"] = material("black_plastic", (0.018, 0.018, 0.019), 0.45)
    M["ecap"] = material("small_cap", (0.03, 0.03, 0.035), 0.35)
    M["ferrite"] = material("ferrite", (0.06, 0.06, 0.065), 0.6)
    M["tape"] = material("transformer_tape", (0.80, 0.62, 0.05), 0.5)
    M["copper"] = material("copper_wire", (0.80, 0.40, 0.18), 0.3, 1.0)
    M["fr4"] = material("fr4_edge", (0.45, 0.42, 0.25), 0.6)
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

    def outline(self, mask, x0, y0, x1, y1, t=0.0004):
        self.rect(mask, x0, y0, x1, y0 + t)
        self.rect(mask, x0, y1 - t, x1, y1)
        self.rect(mask, x0, y0, x0 + t, y1)
        self.rect(mask, x1 - t, y0, x1, y1)

    def disc(self, mask, x, y, r):
        cx, cy = self.px(x, y)
        yy, xx = np.ogrid[: self.h, : self.w]
        mask[(xx - cx) ** 2 + (yy - cy) ** 2 <= (r * self.k) ** 2] = True

    def ring(self, mask, x, y, r, t=0.0004):
        cx, cy = self.px(x, y)
        yy, xx = np.ogrid[: self.h, : self.w]
        d = np.sqrt((xx - cx) ** 2 + (yy - cy) ** 2)
        mask[np.abs(d - r * self.k) <= t * self.k / 2] = True

    def line(self, mask, pts, width):
        for a, b in zip(pts, pts[1:]):
            (ax, ay), (bx, by) = self.px(*a), self.px(*b)
            n = int(max(abs(bx - ax), abs(by - ay))) + 2
            xs, ys = np.linspace(ax, bx, n).round().astype(int), np.linspace(ay, by, n).round().astype(int)
            r = max(1, int(width * self.k / 2))
            for ox in range(-r, r + 1):
                for oy in range(-r, r + 1):
                    mask[np.clip(ys + oy, 0, self.h - 1), np.clip(xs + ox, 0, self.w - 1)] = True


def pcb_texture():
    """The power supply board's top: green solder mask, wide copper pours and traces, tinned pads, white outlines."""
    x0, y0 = BOARD_C[0] - BOARD_SIZE[0] / 2, BOARD_C[1] - BOARD_SIZE[1] / 2
    cv = Canvas(x0, y0, BOARD_SIZE[0], BOARD_SIZE[1], 1000)
    H, W = cv.h, cv.w
    copper, tin, silk = (np.zeros((H, W), bool) for _ in range(3))
    # wide high-current traces
    for _ in range(26):
        p = np.array([RNG.uniform(x0 + 0.005, x0 + 0.095), RNG.uniform(y0 + 0.012, y0 + 0.128)])
        pts = [tuple(p)]
        for _ in range(int(RNG.integers(1, 4))):
            d = [np.array(v) for v in ((1, 0), (0, 1), (0.7, 0.7), (-0.7, 0.7))][int(RNG.integers(0, 4))]
            p = p + d * RNG.uniform(0.008, 0.035) * RNG.choice([-1, 1])
            p = np.clip(p, (x0 + 0.004, y0 + 0.011), (x0 + 0.096, y0 + 0.129))
            pts.append(tuple(p))
        cv.line(copper, pts, RNG.choice([0.0008, 0.0012, 0.002]))
        cv.disc(tin, *pts[-1], 0.0011)
    # the parts' outlines and pads, matching where the heatsink's look puts them
    cv.ring(silk, CAP_C[0], CAP_C[1], 0.0158, 0.0006)
    for s in (-1, 1):
        cv.disc(tin, CAP_C[0] + 0.0095, CAP_C[1] + s * PAD_GAP / 2, 0.0014)
    for x, y in ((BOARD_C[0] - 0.04, 0.045), (BOARD_C[0] - 0.04, 0.03), (BOARD_C[0] - 0.04, -0.005)):
        cv.ring(silk, x, y, 0.0045, 0.0004)
    tx, ty = BOARD_C[0] - 0.03, -0.035
    cv.outline(silk, tx - 0.012, ty - 0.011, tx + 0.012, ty + 0.011, 0.0005)
    sx0, sx1 = SINK_C[0] - SINK_SIZE[0] / 2 - 0.001, SINK_C[0] + SINK_SIZE[0] / 2 + 0.001
    cv.outline(silk, sx0, SINK_C[1] - 0.008, sx1, SINK_C[1] + 0.008, 0.0005)
    for i in range(12):  # rows of small pads: resistors and diodes
        x, y = x0 + 0.012 + 0.006 * (i % 6), y0 + 0.016 + 0.005 * (i // 6)
        cv.rect(tin, x, y, x + 0.0012, y + 0.0012)
        cv.rect(tin, x + 0.0035, y, x + 0.0047, y + 0.0012)
    mask = 0.9 + 0.1 * _noise(H, W, 40)
    rgb = np.array([0.05, 0.30, 0.13])[None, None, :] * mask[..., None]
    rgb[copper] = np.array([0.09, 0.42, 0.17])
    rgb[tin] = np.array([0.80, 0.80, 0.78])
    rgb[silk] = np.array([0.92, 0.92, 0.88])
    return _save("board_top", rgb), (x0, y0, x0 + BOARD_SIZE[0], y0 + BOARD_SIZE[1])


def steel_texture():
    """Galvanised steel: a faint spangle of grey patches."""
    H = W = 512
    spangle = _noise(H, W, 24)
    rgb = np.array([0.66, 0.67, 0.69])[None, None, :] * (0.9 + 0.12 * spangle + 0.03 * _noise(H, W, 2))[..., None]
    return _save("steel", rgb)


def cardboard_texture():
    """Brown corrugated cardboard: fibres and faint flutes along x."""
    H, W = 400, 600
    y = np.arange(H)[:, None]
    flutes = 0.97 + 0.03 * np.sin(2 * math.pi * y / 9.0)
    fibres = 0.92 + 0.10 * _noise(H, W, 3) + 0.05 * _noise(H, W, 60)
    rgb = np.array([0.66, 0.50, 0.33])[None, None, :] * (flutes * fibres)[..., None]
    return _save("cardboard", rgb)


# -- geometry --------------------------------------------------------------------------------------------------------


class Look:
    def __init__(self, name, origin, size=None):
        self.name, self.origin, self.size = name, Vector(origin), size
        self.coll = bpy.data.collections.new("look_" + name)
        bpy.context.scene.collection.children.link(self.coll)
        self.root = bpy.data.objects.new(name, None)
        self.root.location = self.origin
        self.coll.objects.link(self.root)

    def mesh(self, name, bm, mat, bevel=0.0, smooth=False):
        bmesh.ops.translate(bm, vec=-self.origin, verts=bm.verts)
        me = bpy.data.meshes.new(name)
        bm.to_mesh(me)
        bm.free()
        if smooth:
            me.shade_smooth()
        ob = bpy.data.objects.new(name, me)
        self.coll.objects.link(ob)
        ob.parent = self.root
        me.materials.append(mat)
        if bevel:
            m = ob.modifiers.new("bevel", "BEVEL")
            m.width, m.segments, m.limit_method = bevel, 2, "ANGLE"
        return ob


def bm_box(lo, hi, uv=None):
    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=1.0)
    c, s = (Vector(lo) + Vector(hi)) / 2, Vector(hi) - Vector(lo)
    for v in bm.verts:
        v.co = c + Vector((v.co.x * s.x, v.co.y * s.y, v.co.z * s.z))
    if uv:
        _uv_planar(bm, *uv)
    return bm


def _uv_planar(bm, x0, y0, x1, y1, axes=(0, 1)):
    layer = bm.loops.layers.uv.new("UVMap")
    for f in bm.faces:
        for loop in f.loops:
            co = loop.vert.co
            loop[layer].uv = ((co[axes[0]] - x0) / (x1 - x0), (co[axes[1]] - y0) / (y1 - y0))


def _axis_matrix(axis):
    return {"Z": Matrix.Identity(4), "Y": Matrix.Rotation(math.pi / 2, 4, "X"),
            "X": Matrix.Rotation(math.pi / 2, 4, "Y")}[axis]


def bm_cyl(center, r, h, axis="Z", segs=32, r2=None):
    bm = bmesh.new()
    bmesh.ops.create_cone(bm, cap_ends=True, segments=segs, radius1=r, radius2=r if r2 is None else r2, depth=h,
                          matrix=Matrix.Translation(Vector(center)) @ _axis_matrix(axis))
    return bm


def bm_ring(center, r_in, r_out, h, axis="Z", segs=48, a0=0.0, a1=2 * math.pi):
    """A tube (or a sector of one, from angle a0 to a1) around ``axis``."""
    bm = bmesh.new()
    closed = abs(a1 - a0 - 2 * math.pi) < 1e-9
    n = segs if closed else segs + 1
    angles = [a0 + (a1 - a0) * i / segs for i in range(n)]
    rings = [[bm.verts.new((r * math.cos(a), r * math.sin(a), z)) for a in angles]
             for r, z in ((r_out, -h / 2), (r_out, h / 2), (r_in, h / 2), (r_in, -h / 2))]
    for a, b in zip(rings, rings[1:] + rings[:1]):
        for i in range(n if closed else n - 1):
            j = (i + 1) % n
            bm.faces.new((a[i], a[j], b[j], b[i]))
    if not closed:
        for k in (0, n - 1):
            bm.faces.new([r[k] for r in rings])
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    bm.transform(Matrix.Translation(Vector(center)) @ _axis_matrix(axis))
    return bm


def bm_torus(center, R, r, segs=32, ring=12):
    bm = bmesh.new()
    vs = [[bm.verts.new((center[0] + (R + r * math.cos(b)) * math.cos(a), center[1] + (R + r * math.cos(b)) * math.sin(a),
                         center[2] + r * math.sin(b))) for b in np.linspace(0, 2 * math.pi, ring, endpoint=False)]
          for a in np.linspace(0, 2 * math.pi, segs, endpoint=False)]
    for i in range(segs):
        for j in range(ring):
            bm.faces.new((vs[i][j], vs[(i + 1) % segs][j], vs[(i + 1) % segs][(j + 1) % ring], vs[i][(j + 1) % ring]))
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    return bm


# -- the discharge tool ----------------------------------------------------------------------------------------------


def build_tool(M):
    L = Look("tool", TOOL_C, TOOL_SIZE)
    cx, cy, cz = TOOL_C
    x0 = cx - TOOL_SIZE[0] / 2  # the pins' end
    r = TOOL_SIZE[1] / 2
    rod0, rod1 = x0 + 0.020, cx + TOOL_SIZE[0] / 2 - 0.004
    L.mesh("rod", bm_cyl(((rod0 + rod1) / 2, cy, cz), r - 0.0002, rod1 - rod0, "X", 40), M["red"], smooth=True)
    L.mesh("rod_end", bm_cyl((rod1 + 0.002, cy, cz), r - 0.0002, 0.004, "X", 40, r2=r - 0.003), M["red"], smooth=True)
    for i, x in enumerate(np.arange(rod1 - 0.07, rod1 - 0.005, 0.006)):  # grip rings on the handle
        L.mesh(f"grip_{i}", bm_ring((x, cy, cz), r - 0.0012, r, 0.002, "X", 40), M["red"], smooth=True)
    L.mesh("guard", bm_cyl((rod0 + 0.002, cy, cz), r, 0.004, "X", 40), M["head"], smooth=True)
    L.mesh("head", bm_cyl((x0 + 0.0135, cy, cz), r - 0.0015, 0.009, "X", 40, r2=r - 0.0005), M["head"], smooth=True)
    for s in (-1, 1):  # the two steel pins, their tips on the box's end face
        L.mesh(f"pin_{s}", bm_cyl((x0 + 0.0045, cy + s * 0.005, cz), 0.0011, 0.009, "X", 16, r2=0.0011), M["pin"],
               smooth=True)
        L.mesh(f"tip_{s}", bm_cyl((x0 + 0.00025, cy + s * 0.005, cz), 0.0011, 0.0005, "X", 16, r2=0.0008), M["pin"],
               smooth=True)
    return L


# -- the capacitor -------------------------------------------------------------------------------------------------


def build_capacitor(M):
    L = Look("capacitor", CAP_C, CAP_SIZE)
    cx, cy, cz = CAP_C
    z0, z1 = cz - CAP_SIZE[2] / 2, cz + CAP_SIZE[2] / 2
    r = 0.0148
    L.mesh("foot", bm_cyl((cx, cy, z0 + 0.0008), r - 0.0006, 0.0016, "Z", 48), M["rubber"], smooth=True)
    L.mesh("sleeve", bm_cyl((cx, cy, (z0 + 0.0016 + z1 - 0.0016) / 2), r, z1 - z0 - 0.0032, "Z", 64), M["sleeve"],
           smooth=True)
    L.mesh("groove", bm_ring((cx, cy, z1 - 0.005), r - 0.0008, r + 0.0001, 0.0012, "Z", 64), M["vent"], smooth=True)
    band = bm_ring((cx, cy, (z0 + z1) / 2 - 0.001), r - 0.0002, r + 0.0002, z1 - z0 - 0.012, "Z", 16,
                   a0=-0.35, a1=0.35)  # the polarity band, on the side away from the robot
    L.mesh("band", band, M["sleeve_band"], smooth=True)
    L.mesh("top_lip", bm_ring((cx, cy, z1 - 0.0011), r - 0.0018, r, 0.001, "Z", 64), M["sleeve"], smooth=True)
    L.mesh("can_top", bm_cyl((cx, cy, z1 - 0.0012), r - 0.0018, 0.0008, "Z", 64), M["can"], smooth=True)
    for i, a in enumerate((0.0, math.pi / 2)):  # the scored vent cross
        bm = bm_box((cx - 0.009, cy - 0.0003, z1 - 0.0009), (cx + 0.009, cy + 0.0003, z1 - 0.0006))
        bm.transform(Matrix.Translation(Vector((cx, cy, 0))) @ Matrix.Rotation(a + math.pi / 4, 4, "Z")
                     @ Matrix.Translation(Vector((-cx, -cy, 0))))
        L.mesh(f"vent_{i}", bm, M["vent"])
    for s in (-1, 1):  # the two bare terminal pads, 5 mm across, 1 cm apart along y
        L.mesh(f"pad_base_{s}", bm_cyl((cx, cy + s * PAD_GAP / 2, z1 - 0.0006), 0.0028, 0.0004, "Z", 32), M["black"],
               smooth=True)
        L.mesh(f"pad_{s}", bm_cyl((cx, cy + s * PAD_GAP / 2, z1 - 0.0002), 0.0025, 0.0004, "Z", 32, r2=0.0023),
               M["tin"], smooth=True)
    return L


# -- the board, and the heatsink with the board's other parts -------------------------------------------------------


def build_board(M):
    L = Look("board", BOARD_C, BOARD_SIZE)
    img, uv = pcb_texture()
    x0, y0 = BOARD_C[0] - BOARD_SIZE[0] / 2, BOARD_C[1] - BOARD_SIZE[1] / 2
    L.mesh("pcb", bm_box((x0 + 0.0001, y0 + 0.0001, BOARD_BOTTOM + 0.0002), (x0 + BOARD_SIZE[0] - 0.0001,
                                                                            y0 + BOARD_SIZE[1] - 0.0001, BOARD_TOP),
                         uv=uv), material("pcb", (0.05, 0.30, 0.13), 0.35, image=img), bevel=0.0003)
    return L


def build_heatsink(M):
    L = Look("heatsink", SINK_C)  # its own box, and the board's other parts drawn on the board around it
    cx, cy, cz = SINK_C
    x0, x1 = cx - SINK_SIZE[0] / 2, cx + SINK_SIZE[0] / 2
    z0, z1 = cz - SINK_SIZE[2] / 2, cz + SINK_SIZE[2] / 2
    hy = SINK_SIZE[1] / 2
    L.mesh("plate", bm_box((x0, cy - 0.0015, z0), (x1, cy + 0.0015, z1)), M["alu"], bevel=0.0003)
    for i, z in enumerate(np.linspace(z0 + 0.006, z1 - 0.002, 6)):  # fins along x on both faces
        L.mesh(f"fin_{i}", bm_box((x0, cy - hy, z - 0.0009), (x1, cy + hy, z + 0.0009)), M["alu"], bevel=0.0003)
    L.mesh("foot", bm_box((x0, cy - hy, z0), (x1, cy + hy, z0 + 0.002)), M["alu"], bevel=0.0003)
    bz = BOARD_TOP
    # the transformer: a yellow-taped winding on a black ferrite core, front right
    tx, ty = BOARD_C[0] - 0.03, -0.035
    L.mesh("bobbin", bm_box((tx - 0.011, ty - 0.008, bz), (tx + 0.011, ty + 0.008, bz + 0.004)), M["black"],
           bevel=0.0005)
    L.mesh("winding", bm_box((tx - 0.0095, ty - 0.0105, bz + 0.003), (tx + 0.0095, ty + 0.0105, bz + 0.016)),
           M["tape"], bevel=0.0015)
    for s in (-1, 1):
        L.mesh(f"core_{s}", bm_box((tx - 0.012, ty + s * 0.0035 - 0.0025, bz + 0.002),
                                   (tx + 0.012, ty + s * 0.0035 + 0.0025, bz + 0.018)), M["ferrite"], bevel=0.0006)
    # small electrolytic capacitors along the front edge, left
    for i, (x, y) in enumerate(((BOARD_C[0] - 0.04, 0.045), (BOARD_C[0] - 0.04, 0.03), (BOARD_C[0] - 0.04, -0.005))):
        L.mesh(f"ecap_{i}", bm_cyl((x, y, bz + 0.006), 0.0042, 0.012, "Z", 32), M["ecap"], smooth=True)
        L.mesh(f"ecap_top_{i}", bm_cyl((x, y, bz + 0.0121), 0.0036, 0.0003, "Z", 32), M["can"], smooth=True)
    # the output coil: copper wire on a dark toroid, back left
    L.mesh("toroid", bm_torus((BOARD_C[0] + 0.03, 0.045, bz + 0.009), 0.0075, 0.0035), M["ferrite"])
    for i, a in enumerate(np.linspace(0, 2 * math.pi, 18, endpoint=False)):
        bm = bm_ring((0, 0, 0), 0.0034, 0.0042, 0.0012, "Y", 12)
        bm.transform(Matrix.Translation(Vector((BOARD_C[0] + 0.03, 0.045, bz + 0.009))) @ Matrix.Rotation(a, 4, "Z")
                     @ Matrix.Translation(Vector((0.0075, 0, 0))) @ Matrix.Rotation(math.pi / 2, 4, "Z"))
        L.mesh(f"turn_{i}", bm, M["copper"], smooth=True)
    # the bridge rectifier at the front right corner, inside the rails' lips
    L.mesh("bridge", bm_box((BOARD_C[0] - 0.045, -0.057, bz), (BOARD_C[0] - 0.025, -0.049, bz + 0.012)), M["black"],
           bevel=0.0008)
    return L


# -- the case ----------------------------------------------------------------------------------------------------------


def build_case(M):
    L = Look("case", CASE_C)
    steel = material("galvanised", (0.66, 0.67, 0.69), 0.32, 0.9, image=steel_texture())
    x0, x1 = CASE_FRONT, CASE_BACK
    y0, y1 = -CASE_SIZE[1] / 2, CASE_SIZE[1] / 2
    zt = TOP + CASE_SIZE[2]

    def box(name, lo, hi, mat=steel, bevel=0.0004):
        return L.mesh(name, bm_box(lo, hi, uv=(lo[0] / 0.1, lo[1] / 0.1, lo[0] / 0.1 + 1, lo[1] / 0.1 + 1)), mat,
                      bevel=bevel)

    box("floor", (x0, y0, TOP), (x1, y1, TOP + WALL))
    for s, name in ((1, "left"), (-1, "right")):
        ya, yb = sorted((s * (CASE_SIZE[1] / 2), s * (CASE_SIZE[1] / 2 - WALL)))
        box(f"wall_{name}", (x0, ya, TOP + WALL), (x1, yb, zt))
        box(f"hem_{name}", (x0, ya, zt - 0.004), (x1, yb + (0.001 if s < 0 else -0.001), zt))
        for i in range(7):  # vent slots, dark, pressed into the wall's outside
            x = x0 + 0.03 + 0.012 * i
            yo = s * (CASE_SIZE[1] / 2 + 0.0002)
            box(f"vent_{name}_{i}", (x, min(yo, yo - s * 0.0004), TOP + 0.045), (x + 0.004, max(yo, yo - s * 0.0004),
                                                                                TOP + 0.075), M["black"], 0.0)
        rail_y = sorted((s * INNER_Y, s * (INNER_Y - RAIL_W)))
        box(f"rail_{name}", (RAIL_X0, rail_y[0], TOP + WALL), (RAIL_X0 + RAIL_LEN, rail_y[1], BOARD_BOTTOM),
            M["black"], 0.0005)
        box(f"lip_{name}", (RAIL_X0, rail_y[0], LIP_Z0), (RAIL_X0 + RAIL_LEN, rail_y[1], LIP_Z0 + 0.002), M["black"],
            0.0003)
    box("back", (x1 - WALL, y0 + WALL, TOP + WALL), (x1, y1 - WALL, zt))
    # the fan grille on the back panel: a dark opening behind wire rings
    gy, gz, gr = -0.02, TOP + 0.045, 0.034
    L.mesh("fan_hole", bm_cyl((x1 - WALL - 0.0002, gy, gz), gr, 0.0006, "X", 64), M["black"], smooth=True)
    for k, r in enumerate(np.arange(0.006, gr, 0.0065)):
        bm = bm_ring((0, 0, 0), r - 0.0006, r + 0.0006, 0.0012, "X", 48)
        bm.transform(Matrix.Translation(Vector((x1 - WALL - 0.0008, gy, gz))))
        L.mesh(f"grille_{k}", bm, M["alu"], smooth=True)
    for a in (0.0, math.pi / 2):
        bm = bm_box((x1 - WALL - 0.0014, gy - gr, gz - 0.0006), (x1 - WALL - 0.0002, gy + gr, gz + 0.0006))
        bm.transform(Matrix.Translation(Vector((0, gy, gz))) @ Matrix.Rotation(a, 4, "X")
                     @ Matrix.Translation(Vector((0, -gy, -gz))))
        L.mesh(f"grille_bar_{int(a * 10)}", bm, M["alu"])
    # the mains inlet on the back panel, left
    L.mesh("inlet", bm_box((x1 - WALL - 0.008, 0.035, TOP + 0.03), (x1 - WALL, 0.062, TOP + 0.05)), M["black"],
           bevel=0.001)
    return L


# -- the box -------------------------------------------------------------------------------------------------------


def build_box(M):
    L = Look("box", BOX_C)
    card = material("cardboard", (0.62, 0.46, 0.30), 0.85, image=cardboard_texture())
    edge = material("cardboard_edge", (0.50, 0.36, 0.22), 0.9)
    bx, by = BOX_C2
    ox, oy, h, w = BOX_OUTER[0] / 2, BOX_OUTER[1] / 2, BOX_OUTER[2], BOX_WALL

    def panel(name, lo, hi, axes):
        bm = bm_box(lo, hi)
        a, b = axes
        _uv_planar(bm, lo[a], lo[b], lo[a] + 0.3, lo[b] + 0.2, axes)
        return L.mesh(name, bm, card, bevel=0.0008)

    panel("floor", (bx - ox, by - oy, TOP), (bx + ox, by + oy, TOP + w), (0, 1))
    panel("wall_near", (bx - ox, by - oy, TOP + w), (bx - ox + w, by + oy, TOP + h), (1, 2))
    panel("wall_far", (bx + ox - w, by - oy, TOP + w), (bx + ox, by + oy, TOP + h), (1, 2))
    panel("wall_right", (bx - ox + w, by - oy, TOP + w), (bx + ox - w, by - oy + w, TOP + h), (0, 2))
    panel("wall_left", (bx - ox + w, by + oy - w, TOP + w), (bx + ox - w, by + oy, TOP + h), (0, 2))
    for i, (lo, hi) in enumerate((((bx - ox, by - oy, TOP + h - 0.0012), (bx + ox, by - oy + w, TOP + h)),
                                  ((bx - ox, by + oy - w, TOP + h - 0.0012), (bx + ox, by + oy, TOP + h)),
                                  ((bx - ox, by - oy, TOP + h - 0.0012), (bx - ox + w, by + oy, TOP + h)),
                                  ((bx + ox - w, by - oy, TOP + h - 0.0012), (bx + ox, by + oy, TOP + h)))):
        L.mesh(f"edge_{i}", bm_box(lo, hi), edge)  # the cut edges show the flutes' darker brown
    L.mesh("tape", bm_box((bx - 0.025, by - oy, TOP - 0.0002), (bx + 0.025, by + oy, TOP + 0.0002)),
           material("packing_tape", (0.55, 0.42, 0.25), 0.25))
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
    bpy.ops.wm.read_factory_settings(use_empty=True)
    _MATS.clear()


def build():
    clear()
    M = materials()
    looks = [build_tool(M), build_capacitor(M), build_board(M), build_heatsink(M), build_case(M), build_box(M)]
    return {look.name: export(look) for look in looks}


if __name__ == "__main__":
    for name, info in build().items():
        print(name, info)
