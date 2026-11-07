"""make_looks.py: model the scene's visual meshes in Blender and export them as USD next to this file.

    blender -b -P looks/make_looks.py          (Blender 4.2 or later, from the task folder)

The physics stays the task file's boxes. Each look is a render-only mesh that ``looks.py`` puts on one of those
boxes, whose plain cuboid is then hidden but still collides. Every look is built in its box's own frame (metres,
centred on the box, before the box's turn: x along the box, y the box's own +y, z up), so it turns with the box.

- ``led.usdc``: the red 5 mm LED and its two legs, inside the LED's box (checked when exported).
- ``iron.usdc``: the soldering iron (blue handle, steel barrel, tinned tip), inside the iron's box (checked).
- ``board.usdc``: the circuit board, both sides textured, with its resistor, capacitor, pin header and chip on the
  components side and their solder joints on the solder side.
- ``holder.usdc``: the board holder: a base plate and two aluminium jaws with a thumb screw.
- ``stand.usdc``: the iron's stand: a cast base, a chrome spiral, the handle's rest and a damp sponge.

Every datablock is named with the task folder's prefix, so the script can run in a shared Blender session: it removes
only its own datablocks and touches no scene settings. Textures are drawn here with numpy and saved in ``textures/``.
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
PREFIX = "hold-led-solder-pad__"
RNG = np.random.default_rng(11)

# -- the task file's layout (hold_led_solder_pad.py), metres, in the boxes' own frames ---------------------------------
BOARD_SIZE = (0.08, 0.0016, 0.06)
BT = BOARD_SIZE[1]
LED_U, LED_V = -0.015, 0.015  # the LED on the board (from its centre: along it, up)
LED_BODY = (0.0058, 0.0086, 0.0058)
LED_LEGS = 0.0046
LED_LEN = LED_BODY[1] + LED_LEGS
PAD_PITCH = 0.00254
PARTS = {  # body (u, v), size (along the board, out of it, up), and joints (u, v)
    "resistor": ((0.017, -0.009), (0.0063, 0.0025, 0.0025), [(0.0119, -0.009), (0.0221, -0.009)]),
    "capacitor": ((-0.020, -0.008), (0.005, 0.011, 0.005), [(-0.021, -0.008), (-0.019, -0.008)]),
    "header": ((0.028, 0.017), (0.0051, 0.0085, 0.0025), [(0.0267, 0.017), (0.0293, 0.017)]),
    "chip": ((0.015, 0.010), (0.0094, 0.0040, 0.0064),
             [(0.015 + k * 0.00254, 0.010 + s * 0.0038) for k in (-1.5, -0.5, 0.5, 1.5) for s in (-1, 1)]),
}
IRON_SIZE = (0.02, 0.22, 0.02)
HOLDER_BASE = (0.12, 0.07, 0.012)
JAW = (0.10, 0.008, 0.022)
BOARD_IN_JAW = 0.012
STAND_BASE = (0.07, 0.19, 0.012)
BASE_LY = -0.005  # the base's centre along the iron, from the iron's centre
IRON_ABOVE_BASE = 0.055 - 0.006  # the iron's axis above the base box's centre
COIL_Y, COIL_R, WIRE = (0.01, 0.12), 0.012, 0.003
REST_Y = (-0.095, -0.075)

# -- naming and cleaning -------------------------------------------------------------------------------------------


def nm(name):
    return PREFIX + name


def clear():
    """Remove every datablock this script made (its prefix), and nothing else."""
    for coll in (bpy.data.objects, bpy.data.meshes, bpy.data.curves, bpy.data.materials, bpy.data.images,
                 bpy.data.collections):
        for block in [b for b in coll if b.name.startswith(PREFIX)]:
            coll.remove(block)
    _MATS.clear()


# -- materials (linear colours) --------------------------------------------------------------------------------------
_MATS = {}


def material(name, color=(0.5, 0.5, 0.5), rough=0.5, metal=0.0, image=None, alpha=1.0, emit=0.0):
    if name in _MATS:
        return _MATS[name]
    m = bpy.data.materials.new(nm(name))
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
    if emit:
        bsdf.inputs["Emission Color"].default_value = (*color, 1.0)
        bsdf.inputs["Emission Strength"].default_value = emit
    if image is not None:
        tex = nodes.new("ShaderNodeTexImage")
        tex.image = image
        tex.extension = "EXTEND"
        links.new(tex.outputs["Color"], bsdf.inputs["Base Color"])
    m.diffuse_color = (*color, 1.0)
    _MATS[name] = m
    return m


def materials():
    M = {}
    M["led"] = material("led_red", (0.80, 0.015, 0.01), 0.12, emit=0.15)  # the one red thing on the table
    M["led_dark"] = material("led_red_dark", (0.35, 0.01, 0.008), 0.2)
    M["lead"] = material("lead_tin", (0.78, 0.78, 0.76), 0.25, 1.0)
    M["tin"] = material("solder_tin", (0.80, 0.80, 0.78), 0.18, 1.0)
    M["fr4"] = material("fr4_edge", (0.55, 0.50, 0.28), 0.6)
    M["black"] = material("black_plastic", (0.02, 0.02, 0.022), 0.45)
    M["gold"] = material("gold_pin", (0.85, 0.62, 0.22), 0.25, 1.0)
    M["res"] = material("resistor_body", (0.62, 0.52, 0.34), 0.45)
    M["band_brown"] = material("band_brown", (0.18, 0.07, 0.02), 0.5)
    M["band_black"] = material("band_black", (0.015, 0.015, 0.015), 0.5)
    M["band_orange"] = material("band_orange", (0.85, 0.30, 0.02), 0.5)
    M["band_gold"] = material("band_gold", (0.70, 0.52, 0.18), 0.3, 1.0)
    M["cap"] = material("cap_sleeve", (0.025, 0.025, 0.03), 0.35)
    M["cap_stripe"] = material("cap_stripe", (0.45, 0.45, 0.48), 0.4)
    M["alu"] = material("aluminium", (0.70, 0.71, 0.73), 0.3, 1.0)
    M["chrome"] = material("chrome", (0.85, 0.86, 0.88), 0.12, 1.0)
    M["steel"] = material("steel", (0.62, 0.63, 0.65), 0.25, 1.0)
    M["handle"] = material("iron_handle", (0.02, 0.12, 0.55), 0.55)  # the one blue thing
    M["handle_dark"] = material("iron_handle_dark", (0.015, 0.015, 0.02), 0.5)
    M["tip"] = material("iron_tip", (0.72, 0.72, 0.70), 0.2, 1.0)
    M["cast"] = material("cast_base", (0.035, 0.035, 0.04), 0.6, 0.3)
    M["anodised"] = material("anodised_black", (0.03, 0.03, 0.035), 0.35, 0.6)
    M["rubber"] = material("rubber", (0.02, 0.02, 0.02), 0.8)
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
    img = bpy.data.images.new(nm(name + "_draw"), w, h, alpha=False)
    rgba = np.concatenate([np.clip(rgb, 0, 1), np.ones((h, w, 1))], axis=2).astype(np.float32)
    img.pixels.foreach_set(rgba.ravel())
    img.file_format = "JPEG"
    img.save(filepath=path, quality=92)
    bpy.data.images.remove(img)
    loaded = bpy.data.images.load(path)
    loaded.name = nm(name)
    return loaded


class Canvas:
    """A texture over a rectangle of the board (u along it, v up), drawn in board coordinates."""

    def __init__(self, u0, v0, su, sv, w):
        self.u0, self.v0, self.k = u0, v0, w / su
        self.w, self.h = w, int(round(sv * self.k))
        self.yy, self.xx = np.mgrid[: self.h, : self.w]

    def px(self, u, v):
        return (u - self.u0) * self.k, (v - self.v0) * self.k

    def rect(self, mask, u0, v0, u1, v1):
        a, b = self.px(min(u0, u1), min(v0, v1))
        c, d = self.px(max(u0, u1), max(v0, v1))
        mask[max(int(b), 0):max(int(math.ceil(d)), 0), max(int(a), 0):max(int(math.ceil(c)), 0)] = True

    def outline(self, mask, u0, v0, u1, v1, t=0.0003):
        self.rect(mask, u0, v0, u1, v0 + t)
        self.rect(mask, u0, v1 - t, u1, v1)
        self.rect(mask, u0, v0, u0 + t, v1)
        self.rect(mask, u1 - t, v0, u1, v1)

    def disc(self, mask, u, v, r):
        cu, cv = self.px(u, v)
        mask[(self.xx - cu) ** 2 + (self.yy - cv) ** 2 <= (r * self.k) ** 2] = True

    def ring(self, mask, u, v, r, t=0.0003):
        cu, cv = self.px(u, v)
        d = np.sqrt((self.xx - cu) ** 2 + (self.yy - cv) ** 2)
        mask[np.abs(d - r * self.k) <= t * self.k / 2] = True

    def line(self, mask, pts, width):
        r = width * self.k / 2
        for (au, av), (bu, bv) in zip(pts, pts[1:]):
            (ax, ay), (bx, by) = self.px(au, av), self.px(bu, bv)
            dx, dy = bx - ax, by - ay
            L2 = max(dx * dx + dy * dy, 1e-9)
            t = np.clip(((self.xx - ax) * dx + (self.yy - ay) * dy) / L2, 0, 1)
            mask[(self.xx - ax - t * dx) ** 2 + (self.yy - ay - t * dy) ** 2 <= r * r] = True


def _led_pads():
    return [(LED_U + s * PAD_PITCH / 2, LED_V) for s in (-1, 1)]


def _all_joints():
    return [j for _, _, joints in PARTS.values() for j in joints]


def board_textures():
    """The board's two sides: green solder mask, copper under it, tinned pads, white outlines on the components side."""
    su, sv = BOARD_SIZE[0], BOARD_SIZE[2]
    out = {}
    for side in ("front", "back"):
        cv = Canvas(-su / 2, -sv / 2, su, sv, 1600)
        H, W = cv.h, cv.w
        copper, tin, silk, hole = (np.zeros((H, W), bool) for _ in range(4))
        joints = _all_joints()
        pads = _led_pads()
        if side == "back":  # traces between the joints, the LED's pads to the header and the chip
            nets = [[pads[0], (pads[0][0], 0.024), (0.0267, 0.024), (0.0267, 0.017)],
                    [pads[1], (pads[1][0] + 0.004, 0.006), (0.0112, 0.006)],
                    [(0.0221, -0.009), (0.0221, 0.0062), (0.0188, 0.0062)],
                    [(-0.021, -0.008), (-0.021, -0.016), (0.0119, -0.016), (0.0119, -0.009)],
                    [(-0.019, -0.008), (-0.006, 0.002), (0.0112, 0.002), (0.0112, 0.0062)],
                    [(0.0293, 0.017), (0.034, 0.012), (0.034, -0.02)],
                    [(0.0188, 0.0138), (0.024, 0.0138), (0.024, 0.020), (0.0293, 0.020)]]
            for net in nets:
                cv.line(copper, net, 0.0009)
            cv.rect(copper, -0.036, -0.027, 0.036, -0.022)  # a ground pour along the bottom
        for u, v in joints:
            cv.disc(tin, u, v, 0.0009)
            cv.disc(hole, u, v, 0.0004)
        for u, v in pads:  # the LED's two pads, 2 mm, the square one the anode
            cv.disc(tin, u, v, 0.001)
            cv.disc(hole, u, v, 0.0005)
        cv.rect(tin, pads[0][0] - 0.001, pads[0][1] - 0.001, pads[0][0] + 0.001, pads[0][1] + 0.001)
        cv.disc(hole, *pads[0], 0.0005)
        for u, v in ((-0.036, 0.026), (0.036, 0.026), (-0.036, -0.018), (0.036, -0.018)):  # mounting holes
            cv.ring(tin, u, v, 0.0018, 0.0008)
            cv.disc(hole, u, v, 0.0014)
        if side == "front":
            cv.ring(silk, LED_U, LED_V, 0.0034, 0.0003)  # the LED's outline, its flat on the cathode side
            cv.rect(silk, LED_U + 0.0026, LED_V - 0.0022, LED_U + 0.0029, LED_V + 0.0022)
            cv.line(silk, [(LED_U - 0.0058, LED_V + 0.0045), (LED_U - 0.0042, LED_V + 0.0045)], 0.0003)
            cv.line(silk, [(LED_U - 0.005, LED_V + 0.0037), (LED_U - 0.005, LED_V + 0.0053)], 0.0003)
            for (cu, cvv), size, _ in PARTS.values():
                cv.outline(silk, cu - size[0] / 2 - 0.0008, cvv - size[2] / 2 - 0.0008, cu + size[0] / 2 + 0.0008,
                           cvv + size[2] / 2 + 0.0008)
            cv.disc(silk, 0.015 - 0.0055, 0.010 + 0.0028, 0.0004)  # the chip's pin-1 dot
            for k in range(5):  # a row of test-point marks along the top
                cv.ring(silk, -0.006 + 0.004 * k, 0.025, 0.0006, 0.0002)
        mask = 0.9 + 0.1 * _noise(H, W, 60)
        rgb = np.array([0.05, 0.30, 0.13])[None, None, :] * mask[..., None]
        rgb[copper] = np.array([0.09, 0.42, 0.17])
        rgb[silk] = np.array([0.93, 0.93, 0.90])
        rgb[tin] = np.array([0.82, 0.82, 0.80])
        rgb[hole] = np.array([0.10, 0.10, 0.08])
        out[side] = _save(f"board_{side}", rgb)
    return out


def sponge_texture():
    H, W = 256, 256
    pores = _noise(H, W, 3) * 0.6 + _noise(H, W, 9) * 0.4
    rgb = np.array([0.80, 0.62, 0.30])[None, None, :] * (0.65 + 0.45 * pores)[..., None]
    return _save("sponge", rgb)


def cast_texture():
    H, W = 256, 512
    g = 0.85 + 0.2 * _noise(H, W, 2) + 0.08 * _noise(H, W, 30)
    rgb = np.array([0.20, 0.20, 0.22])[None, None, :] * g[..., None]
    return _save("cast", rgb)


# -- geometry --------------------------------------------------------------------------------------------------------


class Look:
    """One exported file: a collection with an empty at the origin (the box's centre) and its meshes."""

    def __init__(self, name, size=None):
        self.name, self.size = name, size
        self.coll = bpy.data.collections.new(nm("look_" + name))
        bpy.context.scene.collection.children.link(self.coll)
        self.root = bpy.data.objects.new(nm(name), None)
        self.coll.objects.link(self.root)
        self.n = 0

    def mesh(self, name, bm, mat, bevel=0.0, smooth=False):
        self.n += 1
        me = bpy.data.meshes.new(nm(f"{self.name}_{name}"))
        bm.to_mesh(me)
        bm.free()
        if smooth:
            me.shade_smooth()
        ob = bpy.data.objects.new(nm(f"{self.name}_{name}"), me)
        self.coll.objects.link(ob)
        ob.parent = self.root
        me.materials.append(mat)
        if bevel:
            m = ob.modifiers.new("bevel", "BEVEL")
            m.width, m.segments, m.limit_method = bevel, 2, "ANGLE"
        return ob


def bm_box(lo, hi):
    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=1.0)
    c, s = (Vector(lo) + Vector(hi)) / 2, Vector(hi) - Vector(lo)
    for v in bm.verts:
        v.co = c + Vector((v.co.x * s.x, v.co.y * s.y, v.co.z * s.z))
    return bm


def bm_quad(corners, uvs):
    bm = bmesh.new()
    vs = [bm.verts.new(c) for c in corners]
    f = bm.faces.new(vs)
    layer = bm.loops.layers.uv.new("UVMap")
    for loop, uv in zip(f.loops, uvs):
        loop[layer].uv = uv
    return bm


def _axis_matrix(axis):
    return {"Z": Matrix.Identity(4), "Y": Matrix.Rotation(-math.pi / 2, 4, "X"),
            "X": Matrix.Rotation(math.pi / 2, 4, "Y")}[axis]


def bm_cyl(center, r, h, axis="Z", segs=32, r2=None):
    """A cylinder (or cone, to ``r2``) of length ``h`` along ``axis``: ``r`` at its - end, ``r2`` at its + end."""
    bm = bmesh.new()
    bmesh.ops.create_cone(bm, cap_ends=True, segments=segs, radius1=r, radius2=r if r2 is None else r2, depth=h,
                          matrix=Matrix.Translation(Vector(center)) @ _axis_matrix(axis))
    return bm


def bm_sphere(center, r, segs=24):
    bm = bmesh.new()
    bmesh.ops.create_uvsphere(bm, u_segments=segs, v_segments=segs // 2, radius=r,
                              matrix=Matrix.Translation(Vector(center)))
    return bm


def bm_lathe(profile, axis="Y", segs=40):
    """A surface of revolution: ``profile`` is [(t along the axis, radius), ...] in order, capped at its ends unless
    it ends where it starts (a ring)."""
    bm = bmesh.new()
    rings = []
    for t, r in profile:
        if r <= 1e-7:
            rings.append([bm.verts.new((0.0, 0.0, t))])
        else:
            rings.append([bm.verts.new((r * math.cos(a), r * math.sin(a), t))
                          for a in np.linspace(0, 2 * math.pi, segs, endpoint=False)])
    for a, b in zip(rings, rings[1:]):
        if len(a) == 1 and len(b) == 1:
            continue
        if len(a) == 1:
            for i in range(segs):
                bm.faces.new((a[0], b[i], b[(i + 1) % segs]))
        elif len(b) == 1:
            for i in range(segs):
                bm.faces.new((a[i], b[0], a[(i + 1) % segs]))
        else:
            for i in range(segs):
                j = (i + 1) % segs
                bm.faces.new((a[i], b[i], b[j], a[j]))
    if tuple(profile[0]) == tuple(profile[-1]):  # a closed profile: a ring, no caps
        bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=1e-9)
    else:
        for ring in (rings[0], rings[-1]):
            if len(ring) > 2:
                bm.faces.new(ring)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    bm.transform(_axis_matrix(axis))
    return bm


def bm_helix(c, r_coil, r_wire, y0, y1, pitch, steps_per_turn=36, ring=10):
    """A wire coiled round an axis along y through ``c`` (x, z), from y0 to y1."""
    bm = bmesh.new()
    turns = (y1 - y0) / pitch
    n = int(turns * steps_per_turn) + 1
    rings = []
    for i in range(n):
        a = 2 * math.pi * i / steps_per_turn
        y = y0 + (y1 - y0) * i / (n - 1)
        centre = Vector((c[0] + r_coil * math.cos(a), y, c[1] + r_coil * math.sin(a)))
        radial = Vector((math.cos(a), 0.0, math.sin(a)))
        along = Vector((0.0, 1.0, 0.0))
        rings.append([bm.verts.new(centre + r_wire * (math.cos(b) * radial + math.sin(b) * along))
                      for b in np.linspace(0, 2 * math.pi, ring, endpoint=False)])
    for a, b in zip(rings, rings[1:]):
        for i in range(ring):
            j = (i + 1) % ring
            bm.faces.new((a[i], a[j], b[j], b[i]))
    bm.faces.new(rings[0])
    bm.faces.new(rings[-1])
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    return bm


# -- the LED -----------------------------------------------------------------------------------------------------------


def build_led(M):
    L = Look("led", (LED_BODY[0], LED_LEN, LED_BODY[2]))
    y_back = LED_LEN / 2 - LED_BODY[1]  # the body's back, on the board when flush
    y_end = LED_LEN / 2
    L.mesh("flange", bm_cyl((0, y_back + 0.0005, 0), 0.0029, 0.001, "Y", 40), M["led_dark"], smooth=True)
    body = bm_lathe([(y_back + 0.001, 0.0025), (y_end - 0.0025, 0.0025)]
                    + [(y_end - 0.0025 + 0.0025 * math.sin(a), 0.0025 * math.cos(a))
                       for a in np.linspace(0.15, math.pi / 2, 8)], "Y", 40)
    L.mesh("dome", body, M["led"], smooth=True)
    for s, name in ((-1, "anode"), (1, "cathode")):  # the legs, 0.5 mm square, through the board
        L.mesh(f"leg_{name}", bm_box((s * PAD_PITCH / 2 - 0.00025, -y_end, -0.00025),
                                     (s * PAD_PITCH / 2 + 0.00025, y_back, 0.00025)), M["lead"])
    return L


# -- the soldering iron ------------------------------------------------------------------------------------------------


def build_iron(M):
    L = Look("iron", IRON_SIZE)
    y0 = -IRON_SIZE[1] / 2
    grip = bm_lathe([(y0, 0.0), (y0, 0.0062), (y0 + 0.004, 0.0075), (y0 + 0.012, 0.0085), (y0 + 0.03, 0.0088),
                     (y0 + 0.07, 0.0099), (y0 + 0.09, 0.0099), (y0 + 0.104, 0.0088), (y0 + 0.11, 0.0072),
                     (y0 + 0.11, 0.0)], "Y", 48)
    L.mesh("grip", grip, M["handle"], smooth=True)
    for i, y in enumerate(np.arange(y0 + 0.074, y0 + 0.102, 0.004)):  # finger rings at the front of the grip
        L.mesh(f"ring_{i}", bm_cyl((0, y, 0), 0.00995, 0.0012, "Y", 48), M["handle_dark"], smooth=True)
    L.mesh("cap", bm_cyl((0, y0 + 0.003, 0), 0.0064, 0.006, "Y", 40, r2=0.0076), M["handle_dark"], smooth=True)
    L.mesh("nut", bm_cyl((0, 0.004, 0), 0.0058, 0.008, "Y", 6), M["steel"], bevel=0.0004)
    L.mesh("barrel", bm_cyl((0, 0.0365, 0), 0.0036, 0.057, "Y", 40), M["steel"], smooth=True)
    L.mesh("collar", bm_cyl((0, 0.066, 0), 0.0038, 0.002, "Y", 40), M["steel"], smooth=True)
    tip = bm_lathe([(0.066, 0.0), (0.066, 0.0019), (0.096, 0.0019), (0.106, 0.0011), (0.1095, 0.0006),
                    (0.11, 0.0)], "Y", 32)
    L.mesh("tip", tip, M["tip"], smooth=True)
    return L


# -- the board -------------------------------------------------------------------------------------------------------


def build_board(M):
    L = Look("board")
    su, sv = BOARD_SIZE[0], BOARD_SIZE[2]
    tex = board_textures()
    L.mesh("core", bm_box((-su / 2, -BT / 2 + 0.0001, -sv / 2), (su / 2, BT / 2 - 0.0001, sv / 2)), M["fr4"])
    u0, u1, v0, v1 = -su / 2, su / 2, -sv / 2, sv / 2
    yf, yb = BT / 2, -BT / 2
    L.mesh("front", bm_quad([(u0, yf, v0), (u0, yf, v1), (u1, yf, v1), (u1, yf, v0)],
                            [(0, 0), (0, 1), (1, 1), (1, 0)]),
           material("board_front", (0.05, 0.30, 0.13), 0.35, image=tex["front"]))
    L.mesh("back", bm_quad([(u0, yb, v0), (u1, yb, v0), (u1, yb, v1), (u0, yb, v1)],
                           [(0, 0), (1, 0), (1, 1), (0, 1)]),
           material("board_back", (0.05, 0.30, 0.13), 0.35, image=tex["back"]))
    # solder joints on the back: a tinned cone round each lead's clipped end
    for i, (u, v) in enumerate(_all_joints()):
        L.mesh(f"joint_{i}", bm_cyl((u, yb - 0.0006, v), 0.0003, 0.0012, "Y", 20, r2=0.0009), M["tin"], smooth=True)
    # the resistor, lying on the board along u, its leads bent into its joints
    (cu, cv), size, joints = PARTS["resistor"]
    r, y = size[2] / 2, yf + size[2] / 2 + 0.0002
    L.mesh("res_body", bm_lathe([(cu - 0.0031, 0.0), (cu - 0.0031, 0.0009), (cu - 0.0027, 0.00125),
                                 (cu - 0.0018, 0.00125), (cu - 0.0013, 0.0011), (cu + 0.0013, 0.0011),
                                 (cu + 0.0018, 0.00125), (cu + 0.0027, 0.00125), (cu + 0.0031, 0.0009),
                                 (cu + 0.0031, 0.0)], "X", 24), M["res"], smooth=True).location = (0, y, cv)
    for k, (du, band) in enumerate(((-0.0022, "band_brown"), (-0.0009, "band_black"), (0.0003, "band_orange"),
                                    (0.0022, "band_gold"))):
        ob = L.mesh(f"res_band_{k}", bm_cyl((cu + du, 0, 0), 0.00118 if abs(du) < 0.0015 else 0.0013, 0.0006, "X",
                                            24), M[band], smooth=True)
        ob.location = (0, y, cv)
    for k, (ju, jv) in enumerate(joints):
        L.mesh(f"res_lead_{k}", bm_box((min(ju, cu + (k - 0.5) * 0.0062), y - 0.0002, jv - 0.0002),
                                       (max(ju, cu + (k - 0.5) * 0.0062), y + 0.0002, jv + 0.0002)), M["lead"])
        L.mesh(f"res_leg_{k}", bm_box((ju - 0.0002, yf, jv - 0.0002), (ju + 0.0002, y + 0.0002, jv + 0.0002)),
               M["lead"])
    # the electrolytic capacitor, standing out of the board
    (cu, cv), size, _ = PARTS["capacitor"]
    h = size[1]
    L.mesh("cap_can", bm_cyl((cu, yf + h / 2, cv), 0.0025, h - 0.0004, "Y", 40), M["cap"], smooth=True)
    L.mesh("cap_top", bm_cyl((cu, yf + h - 0.0001, cv), 0.0022, 0.0002, "Y", 40), M["alu"], smooth=True)
    band = bm_cyl((0, 0, 0), 0.00252, h - 0.003, "Y", 12)
    ob = L.mesh("cap_stripe", band, M["cap_stripe"])
    ob.scale = (0.35, 1.0, 0.35)
    ob.location = (cu - 0.0017, yf + h / 2, cv)
    # the 2-pin header: a black block and two gold pins
    (cu, cv), size, joints = PARTS["header"]
    L.mesh("hdr_body", bm_box((cu - size[0] / 2, yf, cv - size[2] / 2), (cu + size[0] / 2, yf + 0.0025, cv + size[2] / 2)),
           M["black"], bevel=0.0002)
    for k, (ju, jv) in enumerate(joints):
        L.mesh(f"hdr_pin_{k}", bm_box((ju - 0.00032, yf, jv - 0.00032), (ju + 0.00032, yf + size[1], jv + 0.00032)),
               M["gold"])
    # the 8-pin chip
    (cu, cv), size, joints = PARTS["chip"]
    L.mesh("chip_body", bm_box((cu - size[0] / 2, yf + 0.0007, cv - 0.0032), (cu + size[0] / 2, yf + size[1], cv + 0.0032)),
           M["black"], bevel=0.0003)
    L.mesh("chip_notch", bm_cyl((cu - size[0] / 2 + 0.0001, yf + size[1], cv), 0.0007, 0.0002, "Y", 16),
           material("chip_notch", (0.06, 0.06, 0.065), 0.5))
    for k, (ju, jv) in enumerate(joints):
        s = 1 if jv > cv else -1
        L.mesh(f"chip_pin_{k}", bm_box((ju - 0.0003, yf, jv - 0.0002 * s), (ju + 0.0003, yf + 0.0022, cv + s * 0.0035)),
               M["lead"])
    return L


# -- the holder ----------------------------------------------------------------------------------------------------


def build_holder(M):
    L = Look("holder")
    bx, by, bz = HOLDER_BASE
    L.mesh("plate", bm_box((-bx / 2, -by / 2, -bz / 2), (bx / 2, by / 2, bz / 2)), M["anodised"], bevel=0.002)
    for i, (sx, sy) in enumerate(((-1, -1), (-1, 1), (1, -1), (1, 1))):
        L.mesh(f"foot_{i}", bm_cyl((sx * (bx / 2 - 0.01), sy * (by / 2 - 0.01), -bz / 2 - 0.0005), 0.005, 0.001),
               M["rubber"], smooth=True)
    z0, z1 = bz / 2, bz / 2 + JAW[2]
    for s, name in ((1, "front"), (-1, "back")):
        y0, y1 = sorted((s * BT / 2, s * (BT / 2 + JAW[1])))
        L.mesh(f"jaw_{name}", bm_box((-JAW[0] / 2, y0, z0), (JAW[0] / 2, y1, z1)), M["alu"], bevel=0.0008)
        L.mesh(f"jaw_pad_{name}", bm_box((-JAW[0] / 2 + 0.004, y0 + (0.0 if s > 0 else 0.0007),
                                          z1 - 0.008), (JAW[0] / 2 - 0.004, y0 + (0.0007 if s > 0 else 0.0), z1 - 0.001)),
               M["rubber"])
    for k, u in enumerate((-0.035, 0.035)):  # thumb screws through the back jaw
        L.mesh(f"screw_{k}", bm_cyl((u, -BT / 2 - JAW[1] - 0.004, z0 + 0.011), 0.0015, 0.008, "Y", 16), M["steel"],
               smooth=True)
        L.mesh(f"knob_{k}", bm_cyl((u, -BT / 2 - JAW[1] - 0.009, z0 + 0.011), 0.005, 0.004, "Y", 24),
               M["black"], bevel=0.0004)
    return L


# -- the stand -------------------------------------------------------------------------------------------------------


def build_stand(M):
    L = Look("stand")
    bx, by, bz = STAND_BASE
    za = IRON_ABOVE_BASE  # the iron's axis, from the base box's centre

    def ly(v):  # a position along the iron, in the base box's frame
        return v - BASE_LY

    cast = material("cast_tex", (0.2, 0.2, 0.22), 0.6, 0.3, image=cast_texture())
    base = bm_box((-bx / 2, -by / 2, -bz / 2), (bx / 2, by / 2, bz / 2))
    layer = base.loops.layers.uv.new("UVMap")
    for f in base.faces:
        for loop in f.loops:
            loop[layer].uv = ((loop.vert.co.x + loop.vert.co.z) / bx, loop.vert.co.y / by + 0.5)
    L.mesh("base", base, cast, bevel=0.003)
    # the sponge in its tray, beside the spiral
    sx0, sx1, sy0, sy1 = 0.019, 0.034, ly(0.015), ly(0.075)
    L.mesh("tray", bm_box((sx0 - 0.001, sy0 - 0.001, bz / 2), (sx1 + 0.001, sy1 + 0.001, bz / 2 + 0.004)),
           M["black"], bevel=0.0006)
    sp = bm_box((sx0, sy0, bz / 2 + 0.001), (sx1, sy1, bz / 2 + 0.008))
    layer = sp.loops.layers.uv.new("UVMap")
    for f in sp.faces:
        for loop in f.loops:
            loop[layer].uv = (loop.vert.co.x * 30, loop.vert.co.y * 30 + loop.vert.co.z * 30)
    L.mesh("sponge", sp, material("sponge", (0.8, 0.62, 0.3), 0.9, image=sponge_texture()), bevel=0.001)
    # the rest: a post and a chrome cradle under the handle
    rest_top = za - 0.01
    ry0, ry1 = ly(REST_Y[0]), ly(REST_Y[1])
    L.mesh("rest_post", bm_box((-0.006, ry0 + 0.003, bz / 2), (0.006, ry1 - 0.003, rest_top - 0.002)), M["chrome"],
           bevel=0.001)
    L.mesh("rest_saddle", bm_box((-0.008, ry0, rest_top - 0.003), (0.008, ry1, rest_top)), M["chrome"], bevel=0.0008)
    for s in (-1, 1):
        xa, xb = sorted((s * 0.013, s * 0.017))
        L.mesh(f"rest_lip_{s}", bm_box((xa, ry0, bz / 2), (xb, ry1, rest_top + 0.005)), M["chrome"], bevel=0.001)
    # the spiral: a chrome wire coiled round the tip and barrel, open at the handle's end, on a bracket
    axis_z = za - 0.004 + COIL_R
    cy0, cy1 = ly(COIL_Y[0]), ly(COIL_Y[1])
    L.mesh("coil", bm_helix((0.0, axis_z), COIL_R + WIRE / 2, 0.0011, cy0 + 0.002, cy1 - 0.002, 0.009), M["chrome"],
           smooth=True)
    for k, y in enumerate((cy0 + 0.0015, cy1 - 0.0015)):  # the end rings
        ring = bm_lathe([(y - 0.0012, COIL_R + 0.0004), (y - 0.0012, COIL_R + WIRE), (y + 0.0012, COIL_R + WIRE),
                         (y + 0.0012, COIL_R + 0.0004), (y - 0.0012, COIL_R + 0.0004)], "Y", 40)
        ring.transform(Matrix.Translation(Vector((0.0, 0.0, axis_z))))
        L.mesh(f"end_ring_{k}", ring, M["chrome"], smooth=True)
    for k, x in enumerate((-0.0012, 0.0012)):  # two straight wires along the spiral's bottom
        L.mesh(f"spine_{k}", bm_cyl((x * 6, (cy0 + cy1) / 2, axis_z - COIL_R - WIRE / 2), 0.0011, cy1 - cy0, "Y", 12),
               M["chrome"], smooth=True)
    L.mesh("bracket", bm_box((-0.004, ly(0.045), bz / 2), (0.004, ly(0.055), axis_z - COIL_R - WIRE)), M["chrome"],
           bevel=0.001)
    return L


# -- export ----------------------------------------------------------------------------------------------------------


def _bounds(look):
    dg = bpy.context.evaluated_depsgraph_get()
    lo, hi = Vector((1e9,) * 3), Vector((-1e9,) * 3)
    for ob in look.coll.objects:
        if ob.type != "MESH":
            continue
        ev = ob.evaluated_get(dg)
        me = ev.to_mesh()
        for v in me.vertices:
            p = ob.matrix_world @ v.co
            lo = Vector(map(min, lo, p))
            hi = Vector(map(max, hi, p))
        ev.to_mesh_clear()
    return lo, hi


def export(look):
    bpy.context.view_layer.update()
    lo, hi = _bounds(look)
    if look.size is not None:
        half = Vector(look.size) / 2
        over = max(max(-lo[i] - half[i], hi[i] - half[i]) for i in range(3))
        if over > 1e-5:
            raise ValueError(f"{look.name}: the look sticks {over * 1000:.2f} mm out of its box ({lo}, {hi})")
    path = os.path.join(HERE, look.name + ".usdc")
    bpy.ops.wm.usd_export(filepath=path, collection=look.coll.name, export_materials=True,
                          generate_preview_surface=True, export_textures_mode="KEEP", relative_paths=True,
                          root_prim_path="/look", export_lights=False, export_cameras=False,
                          convert_world_material=False, export_custom_properties=False, author_blender_name=False,
                          evaluation_mode="RENDER", export_uvmaps=True, export_normals=True)
    return {"file": os.path.basename(path), "kB": round(os.path.getsize(path) / 1024),
            "low": tuple(round(v, 4) for v in lo), "high": tuple(round(v, 4) for v in hi)}


def build(keep=False):
    """Model and export every look. ``keep`` leaves the models in the scene to look at; otherwise they are removed."""
    clear()
    M = materials()
    looks = [build_led(M), build_iron(M), build_board(M), build_holder(M), build_stand(M)]
    out = {look.name: export(look) for look in looks}
    if not keep:
        clear()
    return out


if __name__ == "__main__":
    for name, info in build().items():
        print(name, info)
