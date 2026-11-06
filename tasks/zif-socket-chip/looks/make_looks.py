"""make_looks.py: model the scene's visual meshes in Blender and export them as USD next to this file.

    blender -b -P looks/make_looks.py          (Blender 4.2 or later, from the task folder)

The physics stays the task file's boxes. Each look is a render-only mesh that ``looks.py`` puts on one of those
boxes, whose plain cuboid is then hidden but still collides. Every look is built in its box's own frame (metres,
the box's centre at the origin, z up), so it turns and moves with the box.

- ``chip.usdc``: the 40-pin DIP chip, notch at +y, inside the chip's 15 x 52 x 8 mm box (checked when exported).
- ``lever.usdc``: the socket's lever, a steel rod with a red tip, inside its 4 x 4 x 45 mm box (checked).
- ``socket.usdc``: the blue ZIF socket with its pin slots, contacts, guide boss, cam block and hinge pin.
- ``board.usdc``: the programmer board and its parts. ``mat.usdc``: the anti-static mat.

Everything this script makes in Blender is named with the prefix ``zif-socket-chip__``, and only those datablocks
are removed when it runs again (the Blender session may be shared). Textures are drawn here with numpy and saved in
``textures/``. Nothing is downloaded.
"""

import math
import os

import bmesh
import bpy
import numpy as np
from mathutils import Vector

HERE = os.path.dirname(os.path.abspath(__file__))
TEX = os.path.join(HERE, "textures")
P = "zif-socket-chip__"
RNG = np.random.default_rng(7)

# -- the task file's sizes (zif_socket_chip.py), metres -------------------------------------------------------------
CHIP_SIZE = (0.015, 0.052, 0.008)  # body 5 mm over 3 mm pins
CHIP_BODY_Z = (-0.001, 0.004)
NOTCH = (0.0065, 0.0075)
PIN_X = 0.0065
LEVER_SIZE = (0.004, 0.004, 0.045)
# The socket in its own frame (x across, y along, z up from the board), and its box's centre in that frame.
SOCKET_W, SOCKET_TOP, SLOT_FLOOR = 0.025, 0.012, 0.0085
SLOT_IN, SLOT_OUT, SLOT_HALF = 0.0035, 0.0095, 0.0265
END_Y, CAM_Y, CAM_H = 0.0315, 0.046, 0.015
BOSS_Y, BOSS = 0.02225, (0.002, 0.0015, 0.002)
SOCKET_MID = (0.0, (-END_Y + CAM_Y) / 2, CAM_H / 2)
SOCKET_SIZE = (SOCKET_W, END_Y + CAM_Y, CAM_H)
LEVER_X, PIVOT_Y, PIVOT_Z = -0.0205, 0.040, SOCKET_TOP - 0.002  # from the socket's centre and the board
# The board's box: its centre from the socket's centre, and its size. The mat's size.
BOARD_OFF, BOARD_SIZE = (0.005, 0.008), (0.085, 0.125, 0.0016)
MAT_SIZE = (0.34, 0.40, 0.003)

# -- materials (linear colours) --------------------------------------------------------------------------------------
_MATS = {}


def material(name, color=(0.5, 0.5, 0.5), rough=0.5, metal=0.0, image=None, emit=0.0):
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
    if emit:
        bsdf.inputs["Emission Color"].default_value = (*color, 1.0)
        bsdf.inputs["Emission Strength"].default_value = emit
    if image is not None:
        tex = nodes.new("ShaderNodeTexImage")
        tex.image = image
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
    for img in [i for i in bpy.data.images if i.name.startswith(P + name)]:
        bpy.data.images.remove(img)
    img = bpy.data.images.new(P + name + "_draw", w, h, alpha=False)
    rgba = np.concatenate([np.clip(rgb, 0, 1), np.ones((h, w, 1))], axis=2).astype(np.float32)
    img.pixels.foreach_set(rgba.ravel())
    img.filepath_raw = path
    img.file_format = "JPEG"
    img.save()
    bpy.data.images.remove(img)
    loaded = bpy.data.images.load(path)
    loaded.name = P + name
    return loaded


class Canvas:
    """A texture over a rectangle (x0, y0, size), drawn in that rectangle's metres. Rows from the bottom."""

    def __init__(self, x0, y0, sx, sy, w):
        self.x0, self.y0, self.k = x0, y0, w / sx
        self.w, self.h = w, int(round(sy * self.k))

    def mask(self):
        return np.zeros((self.h, self.w), bool)

    def rect(self, m, x0, y0, x1, y1):
        a, b = (min(x0, x1) - self.x0) * self.k, (min(y0, y1) - self.y0) * self.k
        c, d = (max(x0, x1) - self.x0) * self.k, (max(y0, y1) - self.y0) * self.k
        m[max(int(b), 0):max(int(math.ceil(d)), 0), max(int(a), 0):max(int(math.ceil(c)), 0)] = True

    def disc(self, m, x, y, r):
        yy, xx = np.ogrid[: self.h, : self.w]
        cx, cy = (x - self.x0) * self.k, (y - self.y0) * self.k
        m[(xx - cx) ** 2 + (yy - cy) ** 2 <= (r * self.k) ** 2] = True

    def outline(self, m, x0, y0, x1, y1, t=0.0004):
        self.rect(m, x0, y0, x1, y0 + t)
        self.rect(m, x0, y1 - t, x1, y1)
        self.rect(m, x0, y0, x0 + t, y1)
        self.rect(m, x1 - t, y0, x1, y1)

    def label(self, m, x, y, n, size=0.0012, vertical=False):
        """A row of glyph-sized blocks, like printed text."""
        for i in range(n):
            if RNG.random() < 0.15:
                continue
            gx = x + i * size * 0.85
            hh = size * (0.7 + 0.3 * RNG.random())
            if vertical:
                self.rect(m, y, gx, y + hh, gx + size * 0.6)
            else:
                self.rect(m, gx, y, gx + size * 0.6, y + hh)


def chip_texture():
    """The chip's top: black epoxy, a pin-1 dot by the notch and grey part markings."""
    w, l = CHIP_SIZE[0], CHIP_SIZE[1]
    cv = Canvas(-w / 2, -l / 2, w, l, 300)
    rgb = np.array([0.045, 0.045, 0.05])[None, None, :] * (0.85 + 0.3 * _noise(cv.h, cv.w, 4))[..., None]
    dot, text = cv.mask(), cv.mask()
    cv.disc(dot, -0.0045, l / 2 - 0.0045, 0.0009)
    for row, n in ((0.006, 9), (0.0035, 7), (-0.0005, 8), (-0.006, 5)):  # the marking reads along the chip
        cv.label(text, row - 0.0005, -0.012, n, 0.0016, vertical=True)
    rgb[dot] = np.array([0.80, 0.80, 0.78])
    rgb[text] = np.array([0.55, 0.55, 0.55])
    return _save("chip_top", rgb)


def board_texture():
    """The programmer board's top: green solder mask, traces, pads, white silkscreen."""
    bx, by = BOARD_SIZE[0], BOARD_SIZE[1]
    cv = Canvas(-bx / 2, -by / 2, bx, by, 1400)
    H, W = cv.h, cv.w
    trace, tin, silk = cv.mask(), cv.mask(), cv.mask()
    for _ in range(140):  # traces: straight runs with one bend
        x, y = RNG.uniform(-bx / 2, bx / 2), RNG.uniform(-by / 2, by / 2)
        L1, L2 = RNG.uniform(0.004, 0.03), RNG.uniform(0.002, 0.02)
        t = RNG.choice([0.00025, 0.0004, 0.0008])
        if RNG.random() < 0.5:
            cv.rect(trace, x, y, x + L1, y + t)
            cv.rect(trace, x + L1 - t, y, x + L1, y + L2)
        else:
            cv.rect(trace, x, y, x + t, y + L1)
            cv.rect(trace, x, y + L1 - t, x + L2, y + L1)
        cv.disc(tin, x, y, 0.0005)
    # the socket's 40 solder pads, in two rows under it (socket centre at -BOARD_OFF)
    sx, sy = -BOARD_OFF[0], -BOARD_OFF[1]
    for i in range(20):
        yy = sy + (i - 9.5) * 0.00254
        for s in (-1, 1):
            cv.disc(tin, sx + s * 0.0076, yy, 0.0008)
    cv.outline(silk, sx - 0.0145, sy - 0.0325, sx + 0.0145, sy + 0.048)
    # pin-1 mark by the lever end, and labels round the board's edge
    cv.disc(silk, sx - 0.0165, sy + 0.025, 0.0007)
    cv.label(silk, -bx / 2 + 0.004, by / 2 - 0.006, 10, 0.0018)
    cv.label(silk, 0.012, -by / 2 + 0.004, 8, 0.0015)
    cv.label(silk, 0.016, 0.022, 6, 0.0012)
    for x, y in ((-bx / 2 + 0.004, -by / 2 + 0.004), (bx / 2 - 0.004, -by / 2 + 0.004),
                 (-bx / 2 + 0.004, by / 2 - 0.004), (bx / 2 - 0.004, by / 2 - 0.004)):
        cv.disc(tin, x, y, 0.0025)
    base = np.array([0.06, 0.28, 0.13])
    rgb = base[None, None, :] * (0.9 + 0.2 * _noise(H, W, 48))[..., None]
    rgb[trace] = np.array([0.09, 0.36, 0.17])
    rgb[tin] = np.array([0.78, 0.78, 0.74])
    rgb[silk] = np.array([0.92, 0.92, 0.88])
    for x, y in ((-bx / 2 + 0.004, -by / 2 + 0.004), (bx / 2 - 0.004, -by / 2 + 0.004),
                 (-bx / 2 + 0.004, by / 2 - 0.004), (bx / 2 - 0.004, by / 2 - 0.004)):
        hole = cv.mask()
        cv.disc(hole, x, y, 0.0015)
        rgb[hole] = np.array([0.03, 0.03, 0.03])
    return _save("board_top", rgb)


def mat_texture():
    cv = Canvas(-MAT_SIZE[0] / 2, -MAT_SIZE[1] / 2, MAT_SIZE[0], MAT_SIZE[1], 1024)
    shade = 0.92 + 0.10 * _noise(cv.h, cv.w, 140) + 0.05 * _noise(cv.h, cv.w, 2)
    rgb = np.array([0.64, 0.67, 0.70])[None, None, :] * shade[..., None]
    snap = cv.mask()  # the grounding snap in a corner
    cv.disc(snap, MAT_SIZE[0] / 2 - 0.02, MAT_SIZE[1] / 2 - 0.02, 0.006)
    rgb[snap] = np.array([0.7, 0.7, 0.68])
    return _save("mat_top", rgb)


# -- geometry --------------------------------------------------------------------------------------------------------


class Look:
    """A look's objects, parented to an empty at its box's centre (the origin of the box's frame)."""

    def __init__(self, name, size=None):
        self.name, self.size = name, size
        self.coll = bpy.data.collections.new(P + "look_" + name)
        bpy.context.scene.collection.children.link(self.coll)
        self.root = bpy.data.objects.new(P + name, None)
        self.coll.objects.link(self.root)

    def mesh(self, name, bm, mat, bevel=0.0, smooth=False):
        me = bpy.data.meshes.new(P + self.name + "_" + name)
        bm.to_mesh(me)
        bm.free()
        if smooth:
            me.shade_smooth()
        ob = bpy.data.objects.new(P + self.name + "_" + name, me)
        self.coll.objects.link(ob)
        ob.parent = self.root
        me.materials.append(mat)
        if bevel:
            m = ob.modifiers.new("bevel", "BEVEL")
            m.width, m.segments, m.limit_method = bevel, 2, "ANGLE"
        return ob

    def box(self, name, lo, hi, mat, bevel=0.0, uv=None):
        return self.mesh(name, bm_box(lo, hi, uv), mat, bevel)


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


def bm_cyl(center, r, h, axis="Z", segs=24):
    bm = bmesh.new()
    bmesh.ops.create_cone(bm, cap_ends=True, segments=segs, radius1=r, radius2=r, depth=h)
    rot = {"Z": None, "X": (0, math.pi / 2, 0), "Y": (math.pi / 2, 0, 0)}[axis]
    if rot:
        from mathutils import Euler

        bmesh.ops.rotate(bm, verts=bm.verts, matrix=Euler(rot).to_matrix())
    bmesh.ops.translate(bm, verts=bm.verts, vec=Vector(center))
    return bm


def bm_union(*bms):
    out = bmesh.new()
    for bm in bms:
        me = bpy.data.meshes.new(P + "tmp")
        bm.to_mesh(me)
        bm.free()
        out.from_mesh(me)
        bpy.data.meshes.remove(me)
    return out


# -- the looks -------------------------------------------------------------------------------------------------------


def build_chip():
    L = Look("chip", CHIP_SIZE)
    w, l = CHIP_SIZE[0] / 2, CHIP_SIZE[1] / 2
    z0, z1 = CHIP_BODY_Z
    epoxy = material("chip_epoxy", (0.03, 0.03, 0.034), 0.55, image=chip_texture())
    uv = (-w, -l, w, l)
    nw, nd = NOTCH[0] / 2, NOTCH[1]
    L.box("body", (-w, -l, z0), (w, l - nd, z1), epoxy, bevel=0.0003, uv=uv)
    L.box("cheek_l", (-w, l - nd, z0), (-nw, l, z1), epoxy, bevel=0.0003, uv=uv)
    L.box("cheek_r", (nw, l - nd, z0), (w, l, z1), epoxy, bevel=0.0003, uv=uv)
    tin = material("pin_tin", (0.80, 0.80, 0.78), 0.3, 1.0)
    legs = []
    for i in range(20):
        y = (i - 9.5) * 0.00254
        for s in (-1, 1):
            legs.append(bm_box((s * 0.0075 if s < 0 else 0.0060, y - 0.0007, z0 - 0.0012),
                               (-0.0060 if s < 0 else 0.0075, y + 0.0007, z0)))
            legs.append(bm_box((s * PIN_X - 0.00035, y - 0.00028, -CHIP_SIZE[2] / 2),
                               (s * PIN_X + 0.00035, y + 0.00028, z0 - 0.0012)))
    L.mesh("pins", bm_union(*legs), tin)
    return L


def build_lever():
    L = Look("lever", LEVER_SIZE)
    h = LEVER_SIZE[2] / 2
    steel = material("steel", (0.70, 0.70, 0.72), 0.28, 1.0)
    red = material("lever_red", (0.80, 0.04, 0.03), 0.4)
    L.mesh("rod", bm_cyl((0, 0, -h + (0.033) / 2), 0.0016, 0.033, segs=16), steel, smooth=True)
    L.box("crank", (-0.002, -0.002, -h), (0.002, 0.002, -h + 0.004), steel, bevel=0.0004)
    L.box("tip", (-0.002, -0.002, -h + 0.031), (0.002, 0.002, h), red, bevel=0.0008)
    return L


def build_socket():
    L = Look("socket")
    mx, my, mz = SOCKET_MID

    def S(x, y, z):
        return (x - mx, y - my, z - mz)

    blue = material("socket_blue", (0.04, 0.12, 0.55), 0.45)
    dark = material("slot_dark", (0.01, 0.01, 0.015), 0.7)
    gold = material("contact_gold", (1.0, 0.72, 0.30), 0.25, 1.0)
    steel = material("steel", (0.70, 0.70, 0.72), 0.28, 1.0)
    white = material("print_white", (0.85, 0.85, 0.82), 0.5)
    W = SOCKET_W / 2
    L.box("base", S(-W, -END_Y, 0), S(W, END_Y, SLOT_FLOOR), blue, bevel=0.0004)
    L.box("rail_l", S(-W, -END_Y, SLOT_FLOOR), S(-SLOT_OUT, END_Y, SOCKET_TOP), blue, bevel=0.0003)
    L.box("rail_r", S(SLOT_OUT, -END_Y, SLOT_FLOOR), S(W, END_Y, SOCKET_TOP), blue, bevel=0.0003)
    L.box("ridge", S(-SLOT_IN, -END_Y, SLOT_FLOOR), S(SLOT_IN, END_Y, SOCKET_TOP), blue, bevel=0.0003)
    L.box("end_n", S(-SLOT_OUT, -END_Y, SLOT_FLOOR), S(SLOT_OUT, -SLOT_HALF, SOCKET_TOP), blue)
    L.box("end_f", S(-SLOT_OUT, SLOT_HALF, SLOT_FLOOR), S(SLOT_OUT, END_Y, SOCKET_TOP), blue)
    L.box("floor", S(-SLOT_OUT, -SLOT_HALF, SLOT_FLOOR), S(SLOT_OUT, SLOT_HALF, SLOT_FLOOR + 0.0002), dark)
    contacts = []
    for i in range(20):
        y = (i - 9.5) * 0.00254
        for s in (-1, 1):
            contacts.append(bm_box(S(s * PIN_X - 0.0012, y - 0.0005, SLOT_FLOOR),
                                   S(s * PIN_X + 0.0012, y + 0.0005, SLOT_FLOOR + 0.0004)))
    L.mesh("contacts", bm_union(*contacts), gold)
    L.box("boss", S(-BOSS[0] / 2, BOSS_Y - BOSS[1] / 2, SOCKET_TOP), S(BOSS[0] / 2, BOSS_Y + BOSS[1] / 2, SOCKET_TOP + BOSS[2]),
          blue, bevel=0.0002)
    L.box("cam", S(-W, END_Y, 0), S(W, CAM_Y, CAM_H), blue, bevel=0.0012)
    # a white arrow on the cam block's top, pointing at the lever end: the side pin 1 goes
    L.box("mark", S(-0.004, CAM_Y - 0.007, CAM_H), S(0.004, CAM_Y - 0.005, CAM_H + 0.0002), white)
    L.box("mark2", S(-0.001, END_Y + 0.002, CAM_H), S(0.001, CAM_Y - 0.007, CAM_H + 0.0002), white)
    # the hinge pin, from the cam block out to the lever
    L.mesh("hinge_pin", bm_cyl(S((-W + LEVER_X + 0.002) / 2, PIVOT_Y, PIVOT_Z), 0.0013, (-W) - (LEVER_X + 0.002),
                               axis="X", segs=16), steel, smooth=True)
    return L


def build_board():
    L = Look("board")
    bx, by, bt = BOARD_SIZE[0] / 2, BOARD_SIZE[1] / 2, BOARD_SIZE[2] / 2
    pcb = material("pcb_green", (0.06, 0.28, 0.13), 0.4, image=board_texture())
    edge = material("pcb_edge", (0.30, 0.32, 0.18), 0.6)
    L.box("pcb", (-bx, -by, -bt), (bx, by, bt), pcb, uv=(-bx, -by, bx, by))
    L.box("pcb_edge", (-bx - 0.0001, -by - 0.0001, -bt), (bx + 0.0001, by + 0.0001, bt - 0.0001), edge)
    black = material("ic_black", (0.025, 0.025, 0.028), 0.5)
    tin = material("pin_tin", (0.80, 0.80, 0.78), 0.3, 1.0)
    silver = material("usb_silver", (0.72, 0.72, 0.74), 0.25, 1.0)
    cap = material("cap_dark", (0.05, 0.06, 0.12), 0.35, 0.3)
    header = material("header_black", (0.02, 0.02, 0.02), 0.6)
    top = bt
    # the programmer's controller: a 28-pin chip on the far side
    L.box("mcu", (0.022, -0.030, top), (0.030, 0.006, top + 0.0035), black, bevel=0.0003)
    legs = []
    for i in range(14):
        y = -0.0285 + i * 0.00254
        for x0, x1 in ((0.0205, 0.0222), (0.0298, 0.0315)):
            legs.append(bm_box((x0, y - 0.0003, top), (x1, y + 0.0003, top + 0.0022)))
    L.mesh("mcu_pins", bm_union(*legs), tin)
    # USB socket at the far edge, a crystal, capacitors, LEDs and a pin header
    L.box("usb", (0.028, 0.030, top), (bx + 0.002, 0.042, top + 0.010), silver, bevel=0.0005)
    L.box("crystal", (0.022, 0.010, top), (0.0255, 0.0215, top + 0.0035), silver, bevel=0.0012)
    for i, (x, y) in enumerate(((0.035, -0.045), (0.035, -0.035), (0.015, -0.052))):
        L.mesh(f"cap{i}", bm_cyl((x, y, top + 0.004), 0.003, 0.008, segs=20), cap, smooth=True)
    for i, (color, x) in enumerate((((0.05, 0.65, 0.10), -0.012), ((0.85, 0.65, 0.02), -0.004),
                                    ((0.05, 0.20, 0.85), 0.004))):
        led = material(f"led{i}", color, 0.2, emit=0.6 if i == 0 else 0.0)
        L.mesh(f"led{i}", bm_cyl((x, by - 0.008, top + 0.002), 0.0015, 0.004, segs=16), led, smooth=True)
    L.box("header", (-0.030, -by + 0.003, top), (-0.006, -by + 0.0055, top + 0.0025), header)
    for i in range(10):
        x = -0.029 + i * 0.00254
        L.box(f"hpin{i}", (x - 0.0003, -by + 0.0039, top + 0.0025), (x + 0.0003, -by + 0.0045, top + 0.008), tin)
    return L


def build_mat():
    L = Look("mat")
    x, y, z = (s / 2 for s in MAT_SIZE)
    m = material("mat_grey", (0.37, 0.41, 0.45), 0.85, image=mat_texture())
    L.box("mat", (-x, -y, -z), (x, y, z), m, bevel=0.001, uv=(-x, -y, x, y))
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


def clear():
    """Remove everything an earlier run of this script made (names with the prefix), and nothing else."""
    for coll in [c for c in bpy.data.collections if c.name.startswith(P)]:
        for ob in list(coll.objects):
            bpy.data.objects.remove(ob)
        bpy.data.collections.remove(coll)
    for ob in [o for o in bpy.data.objects if o.name.startswith(P)]:
        bpy.data.objects.remove(ob)
    for block in (bpy.data.meshes, bpy.data.materials, bpy.data.images):
        for item in [i for i in block if i.name.startswith(P)]:
            block.remove(item)
    _MATS.clear()


def build():
    clear()
    looks = [build_chip(), build_lever(), build_socket(), build_board(), build_mat()]
    return {look.name: export(look) for look in looks}


if __name__ == "__main__":
    print(build())
