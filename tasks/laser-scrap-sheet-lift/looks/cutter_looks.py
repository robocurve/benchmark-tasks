"""cutter_looks: Blender helpers and the desktop laser cutter's meshes, for make_looks.py (Blender 4.2 or later).

The same file is in the laser-scrap-sheet-lift and open-laser-cutter-lid tasks. ``make_looks.py`` calls
:func:`setup` with its folder's prefix, then the builders it needs, then :func:`export` on each look. Sizes come from
``cutter.py`` in the task folder. Every look is built in its carrier box's own frame (metres, z up, around the box's
centre). Textures are drawn here with numpy and saved as JPEG in ``textures/``. Nothing is downloaded. Everything made
in Blender is named with the folder's prefix, and only those are ever removed.
"""

import math
import os

import bmesh
import bpy
import numpy as np
from mathutils import Matrix, Vector

S = {"prefix": "cutter__", "here": os.path.dirname(os.path.abspath(__file__))}
RNG = np.random.default_rng(13)
_MATS = {}


def setup(prefix: str, here: str) -> None:
    S["prefix"], S["here"] = prefix, here
    _MATS.clear()


def lin(c):
    """sRGB 0..1 to linear."""
    return tuple(v / 12.92 if v <= 0.04045 else ((v + 0.055) / 1.055) ** 2.4 for v in c)


# -- materials and textures --------------------------------------------------------------------------------------------


def material(name, color=(0.5, 0.5, 0.5), rough=0.5, metal=0.0, image=None, alpha=1.0, emit=0.0):
    """A Principled material; ``color`` is sRGB."""
    full = S["prefix"] + name
    if full in _MATS:
        return _MATS[full]
    m = bpy.data.materials.get(full) or bpy.data.materials.new(full)
    m.use_nodes = True
    nodes, links = m.node_tree.nodes, m.node_tree.links
    nodes.clear()
    out = nodes.new("ShaderNodeOutputMaterial")
    bsdf = nodes.new("ShaderNodeBsdfPrincipled")
    links.new(bsdf.outputs["BSDF"], out.inputs["Surface"])
    c = lin(color)
    bsdf.inputs["Base Color"].default_value = (*c, 1.0)
    bsdf.inputs["Roughness"].default_value = rough
    bsdf.inputs["Metallic"].default_value = metal
    bsdf.inputs["Alpha"].default_value = alpha
    if emit > 0:
        bsdf.inputs["Emission Color"].default_value = (*c, 1.0)
        bsdf.inputs["Emission Strength"].default_value = emit
    if image is not None:
        tex = nodes.new("ShaderNodeTexImage")
        tex.image = image
        tex.extension = "REPEAT"
        links.new(tex.outputs["Color"], bsdf.inputs["Base Color"])
    m.diffuse_color = (*c, alpha)
    _MATS[full] = m
    return m


def save_texture(name, rgb):
    """Save an sRGB image (rows from the bottom, as Blender stores them) as JPEG in textures/ and load it."""
    tex = os.path.join(S["here"], "textures")
    os.makedirs(tex, exist_ok=True)
    path = os.path.join(tex, name + ".jpg")
    full = S["prefix"] + name
    for old in [i for i in bpy.data.images if i.name.startswith(full)]:
        bpy.data.images.remove(old)
    h, w, _ = rgb.shape
    img = bpy.data.images.new(full + "_draw", w, h, alpha=False)
    rgba = np.concatenate([np.clip(rgb, 0, 1), np.ones((h, w, 1))], axis=2).astype(np.float32)
    img.pixels.foreach_set(rgba.ravel())
    img.filepath_raw = path
    img.file_format = "JPEG"
    img.save()
    bpy.data.images.remove(img)
    loaded = bpy.data.images.load(path)
    loaded.name = full
    return loaded


def honeycomb_texture(sx, sy, cell=0.0064, px=1400):
    """Aluminium honeycomb seen from above: hexagonal cells ``cell`` across, thin bright walls, dark inside."""
    w, h = px, int(round(px * sy / sx))
    xs = (np.arange(w) + 0.5) / w * sx
    ys = (np.arange(h) + 0.5) / h * sy
    X, Y = np.meshgrid(xs, ys)
    row = cell * math.sqrt(3) / 2
    best = None
    for ox, oy in ((0.0, 0.0), (cell / 2, row)):
        cx = np.round((X - ox) / cell) * cell + ox
        cy = np.round((Y - oy) / (2 * row)) * 2 * row + oy
        dx, dy = X - cx, Y - cy
        d = np.hypot(dx, dy)
        if best is None:
            best, bx, by = d, dx, dy
        else:
            closer = d < best
            best, bx, by = np.where(closer, d, best), np.where(closer, dx, bx), np.where(closer, dy, by)
    hexd = np.maximum(np.abs(bx), np.abs(bx) / 2 + np.abs(by) * math.sqrt(3) / 2)
    wall = hexd > cell / 2 * 0.86
    base = 0.10 + 0.04 * RNG.random(X.shape)
    img = np.repeat(base[:, :, None], 3, axis=2)
    img[wall] = (0.62, 0.63, 0.64)
    burn = RNG.random(X.shape) < 0.002  # a few scorch marks
    img[burn] = (0.05, 0.04, 0.03)
    return save_texture("honeycomb", img)


def vent_texture(px=512):
    """A row of dark slots on the body's charcoal paint, for the side panels."""
    img = np.full((px // 4, px, 3), 0.13) + 0.01 * RNG.random((px // 4, px, 1))
    for k in range(18):
        x0 = int(px * (0.08 + k * 0.047))
        img[px // 16: px // 4 - px // 16, x0: x0 + px // 70] = 0.03
    return save_texture("vents", img)


def screen_texture(px=256):
    """The control panel's small display: a dark blue screen with a few light lines of text and a progress bar."""
    h = px // 2
    img = np.zeros((h, px, 3))
    img[:] = (0.04, 0.10, 0.22)
    for k, length in enumerate((0.7, 0.5, 0.6)):
        y0 = int(h * (0.72 - k * 0.2))
        img[y0: y0 + h // 14, int(px * 0.08): int(px * (0.08 + length))] = (0.55, 0.75, 0.95)
    img[int(h * 0.12): int(h * 0.2), int(px * 0.08): int(px * 0.92)] = (0.15, 0.25, 0.35)
    img[int(h * 0.12): int(h * 0.2), int(px * 0.08): int(px * 0.62)] = (0.35, 0.85, 0.45)
    return save_texture("screen", img)


# -- geometry ----------------------------------------------------------------------------------------------------------


class Look:
    """A look: a collection with an empty at the carrier box's centre and meshes under it. ``size`` (or None) is the
    box the look must stay inside. ``frame`` maps machine coordinates to the look's (a Matrix)."""

    def __init__(self, name, size=None, frame=None):
        self.name, self.size = name, size
        self.frame = frame if frame is not None else Matrix.Identity(4)
        self.coll = bpy.data.collections.new(S["prefix"] + "look_" + name)
        bpy.context.scene.collection.children.link(self.coll)
        self.root = bpy.data.objects.new(S["prefix"] + name, None)
        self.coll.objects.link(self.root)

    def mesh(self, name, bm, mats, bevel=0.0, place=None):
        """Add ``bm`` (built in machine coordinates, or the look's own with ``place`` False) as an object."""
        me = bpy.data.meshes.new(S["prefix"] + self.name + "_" + name)
        bm.transform(self.frame if place is None else place)
        bm.to_mesh(me)
        bm.free()
        for m in mats if isinstance(mats, (list, tuple)) else [mats]:
            me.materials.append(m)
        ob = bpy.data.objects.new(S["prefix"] + self.name + "_" + name, me)
        self.coll.objects.link(ob)
        ob.parent = self.root
        if bevel > 0:
            mod = ob.modifiers.new("bevel", "BEVEL")
            mod.width = bevel
            mod.segments = 2
            mod.limit_method = "ANGLE"
        return ob


def bm_box(lo, hi):
    bm = bmesh.new()
    (x0, y0, z0), (x1, y1, z1) = lo, hi
    v = [bm.verts.new(p) for p in [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),
                                   (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)]]
    for idx in [(0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4), (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)]:
        bm.faces.new([v[i] for i in idx])
    return bm


def bm_prism(pts, z0, z1, top_mat=None, side_mat=None):
    """A prism over the counter-clockwise outline ``pts`` from ``z0`` to ``z1``."""
    bm = bmesh.new()
    lo = [bm.verts.new((x, y, z0)) for x, y in pts]
    hi = [bm.verts.new((x, y, z1)) for x, y in pts]
    bm.faces.new(list(reversed(lo)))
    top = bm.faces.new(hi)
    n = len(pts)
    for i in range(n):
        f = bm.faces.new([lo[i], lo[(i + 1) % n], hi[(i + 1) % n], hi[i]])
        if side_mat is not None:
            f.material_index = side_mat
    if top_mat is not None:
        top.material_index = top_mat
    return bm


def bm_cylinder(p0, p1, r, n=24):
    """A closed cylinder of radius ``r`` from point ``p0`` to ``p1``."""
    bm = bmesh.new()
    a, b = Vector(p0), Vector(p1)
    axis = (b - a).normalized()
    ref = Vector((0, 0, 1)) if abs(axis.z) < 0.9 else Vector((1, 0, 0))
    e1 = axis.cross(ref).normalized()
    e2 = axis.cross(e1)
    ring0, ring1 = [], []
    for k in range(n):
        t = 2 * math.pi * k / n
        off = (e1 * math.cos(t) + e2 * math.sin(t)) * r
        ring0.append(bm.verts.new(a + off))
        ring1.append(bm.verts.new(b + off))
    bm.faces.new(list(reversed(ring0)))
    bm.faces.new(ring1)
    for k in range(n):
        bm.faces.new([ring0[k], ring0[(k + 1) % n], ring1[(k + 1) % n], ring1[k]])
    return bm


def circle(cx, cy, r, n=72):
    return [(cx + r * math.cos(2 * math.pi * k / n), cy + r * math.sin(2 * math.pi * k / n)) for k in range(n)]


def uv_planar(ob, x0, y0, x1, y1, axes=(0, 1)):
    me = ob.data
    uv = me.uv_layers.new(name="UVMap")
    for loop in me.loops:
        co = me.vertices[loop.vertex_index].co
        uv.data[loop.index].uv = ((co[axes[0]] - x0) / (x1 - x0), (co[axes[1]] - y0) / (y1 - y0))


def boolean_cut(look, ob, cutter_bms):
    """``ob`` minus the meshes in ``cutter_bms`` (in the look's own frame), applied into a new mesh."""
    tmp = bpy.data.collections.new(S["prefix"] + "cutters_" + look.name)
    bpy.context.scene.collection.children.link(tmp)
    cutters = []
    for k, bm in enumerate(cutter_bms):
        me = bpy.data.meshes.new(S["prefix"] + f"cutter_{look.name}_{k}")
        bm.to_mesh(me)
        bm.free()
        c = bpy.data.objects.new(S["prefix"] + f"cutter_{look.name}_{k}", me)
        tmp.objects.link(c)
        cutters.append(c)
        mod = ob.modifiers.new(f"cut{k}", "BOOLEAN")
        mod.operation = "DIFFERENCE"
        mod.solver = "EXACT"
        mod.object = c
    bpy.context.view_layer.update()
    dg = bpy.context.evaluated_depsgraph_get()
    me = bpy.data.meshes.new_from_object(ob.evaluated_get(dg))
    old = ob.data
    ob.modifiers.clear()
    ob.data = me
    me.name = old.name
    bpy.data.meshes.remove(old)
    for c in cutters:
        m = c.data
        bpy.data.objects.remove(c)
        bpy.data.meshes.remove(m)
    bpy.data.collections.remove(tmp)


def shift(c):
    """The matrix that moves machine coordinates to a carrier box centred at machine point ``c``."""
    return Matrix.Translation(-Vector(c))


def lid_turn(C, deg):
    """The lid turned ``deg`` open about its hinge, in machine coordinates (front edge up: about +v)."""
    h = Vector((C.HINGE_U, 0.0, C.HINGE_W))
    return Matrix.Translation(h) @ Matrix.Rotation(math.radians(deg), 4, "Y") @ Matrix.Translation(-h)


# -- the machine -------------------------------------------------------------------------------------------------------

CHARCOAL = (0.13, 0.14, 0.15)


def _mats():
    return {
        "body": material("cutter_body", CHARCOAL, 0.5),
        "inner": material("cutter_inner", (0.20, 0.21, 0.22), 0.6),
        "vents": material("cutter_vents", CHARCOAL, 0.55, image=vent_texture()),
        "alu": material("cutter_alu", (0.72, 0.73, 0.75), 0.3, metal=0.85),
        "steel": material("cutter_steel", (0.45, 0.46, 0.48), 0.35, metal=0.9),
        "black": material("cutter_black", (0.04, 0.04, 0.045), 0.45),
        "rubber": material("cutter_rubber", (0.06, 0.06, 0.06), 0.85),
        "window": material("cutter_window", (0.32, 0.24, 0.16), 0.05, alpha=0.45),
        "screen": material("cutter_screen", (0.3, 0.5, 0.8), 0.2, image=screen_texture(), emit=0.6),
        "green": material("cutter_green", (0.10, 0.65, 0.25), 0.4, emit=0.4),
        "red": material("cutter_red", (0.80, 0.08, 0.06), 0.4),
        "yellow": material("cutter_yellow", (0.95, 0.80, 0.10), 0.5),
        "duct": material("cutter_duct", (0.68, 0.69, 0.70), 0.35, metal=0.9),
        "trim": material("cutter_trim", (0.55, 0.56, 0.58), 0.3, metal=0.8),
    }


def build_body(C, carrier_centre, name="cutter_body"):
    """The fixed machine: walls, back block with its control panel and exhaust duct, the bed and the gantry. Built
    in machine coordinates, put in the frame of the carrier box centred at machine point ``carrier_centre``."""
    look = Look(name, None, shift(carrier_centre))
    m = _mats()
    s, t = C.W / 2, C.WALL
    for (bname, (cu, cv, cw), (su, sv, sw)) in C.body_boxes():
        lo, hi = (cu - su / 2, cv - sv / 2, cw - sw / 2), (cu + su / 2, cv + sv / 2, cw + sw / 2)
        if bname in ("cutter_front", "cutter_back"):
            look.mesh(bname, bm_box(lo, hi), m["body"], bevel=0.003)
        elif bname in ("cutter_right", "cutter_left"):
            ob = look.mesh(bname, bm_box(lo, hi), [m["vents"]], bevel=0.003)
            a = look.frame @ Vector((lo[0] + 0.02, 0.0, lo[2] + 0.04))
            b = look.frame @ Vector((hi[0] - 0.02, 0.0, lo[2] + 0.10))
            uv_planar(ob, a.x, a.z, b.x, b.z, axes=(0, 2))
    # inside: a floor and a lighter liner, so the inside reads as a cavity
    look.mesh("floor", bm_box((t, -s + t, 0.0), (C.HINGE_U, s - t, 0.012)), m["inner"])
    # the bed: a dark support, an aluminium frame and the honeycomb
    u0, u1 = C.BED_U
    bv = C.BED_V
    top = C.BED_TOP_W
    look.mesh("bed_support", bm_box((u0 + 0.01, -bv + 0.01, 0.012), (u1 - 0.01, bv - 0.01, top - 0.008)), m["black"])
    rim = 0.008
    for rname, lo, hi in [
        ("bed_near", (u0, -bv, top - 0.008), (u0 + rim, bv, top)),
        ("bed_far", (u1 - rim, -bv, top - 0.008), (u1, bv, top)),
        ("bed_right", (u0 + rim, -bv, top - 0.008), (u1 - rim, -bv + rim, top)),
        ("bed_left", (u0 + rim, bv - rim, top - 0.008), (u1 - rim, bv, top)),
    ]:
        look.mesh(rname, bm_box(lo, hi), m["alu"], bevel=0.001)
    comb = material("honeycomb", (0.4, 0.4, 0.4), 0.45, metal=0.6,
                    image=honeycomb_texture(u1 - u0 - 2 * rim, 2 * (bv - rim)))
    panel = look.mesh("honeycomb", bm_box((u0 + rim, -bv + rim, top - 0.008), (u1 - rim, bv - rim, top - 0.0002)),
                      comb)
    uv_planar(panel, *(look.frame @ Vector((u0 + rim, -bv + rim, 0)))[:2],
              *(look.frame @ Vector((u1 - rim, bv - rim, 0)))[:2])
    # the gantry: two side rails, the beam across, the head with its nozzle and a belt
    for (bname, (cu, cv, cw), (su, sv, sw)) in C.body_boxes():
        lo, hi = (cu - su / 2, cv - sv / 2, cw - sw / 2), (cu + su / 2, cv + sv / 2, cw + sw / 2)
        if bname.startswith("cutter_rail"):
            look.mesh(bname, bm_box(lo, hi), m["alu"], bevel=0.002)
        elif bname == "cutter_beam":
            look.mesh(bname, bm_box(lo, hi), m["alu"], bevel=0.002)
            look.mesh("belt", bm_box((lo[0] - 0.002, lo[1] + 0.01, hi[2] - 0.012), (lo[0], hi[1] - 0.01, hi[2] - 0.004)),
                      m["black"])
        elif bname == "cutter_head":
            look.mesh("head_block", bm_box((lo[0], lo[1], lo[2] + 0.02), hi), m["steel"], bevel=0.002)
            cu_, cv_ = (lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2
            look.mesh("head_tube", bm_cylinder((cu_, cv_, lo[2] + 0.02), (cu_, cv_, lo[2] + 0.004), 0.009), m["black"])
            look.mesh("nozzle", bm_cylinder((cu_, cv_, lo[2] + 0.006), (cu_, cv_, lo[2]), 0.004), m["trim"])
            look.mesh("air_hose", bm_cylinder((hi[0], cv_ + 0.01, hi[2] - 0.01), (hi[0] + 0.02, cv_ + 0.01, hi[2] - 0.01),
                                              0.003), m["black"])
    # the back block: a top with the control panel, and a warning label
    bu0 = C.HINGE_U
    look.mesh("panel_plate", bm_box((bu0 + 0.012, -s + 0.04, C.H), (C.D - 0.012, -s + 0.20, C.H + 0.004)), m["black"],
              bevel=0.001)
    look.mesh("screen", bm_box((bu0 + 0.018, -s + 0.05, C.H + 0.004), (bu0 + 0.048, -s + 0.11, C.H + 0.0055)),
              m["screen"])
    for k, mat in enumerate(("green", "black", "black")):
        cv_ = -s + 0.125 + k * 0.02
        look.mesh(f"button{k}", bm_cylinder((bu0 + 0.035, cv_, C.H + 0.004), (bu0 + 0.035, cv_, C.H + 0.009), 0.006),
                  m[mat])
    look.mesh("estop_base", bm_cylinder((bu0 + 0.035, -s + 0.185, C.H + 0.004), (bu0 + 0.035, -s + 0.185, C.H + 0.010),
                                       0.011), m["yellow"])
    look.mesh("estop", bm_cylinder((bu0 + 0.035, -s + 0.185, C.H + 0.010), (bu0 + 0.035, -s + 0.185, C.H + 0.020),
                                  0.009), m["red"])
    # the exhaust: a round aluminium duct out of the back, ribbed
    dv, dw = s - 0.12, 0.11
    look.mesh("duct_flange", bm_cylinder((C.D, dv, dw), (C.D + 0.008, dv, dw), 0.06), m["steel"])
    look.mesh("duct", bm_cylinder((C.D + 0.008, dv, dw), (C.D + 0.14, dv, dw), 0.05), m["duct"])
    for k in range(6):
        uu = C.D + 0.02 + k * 0.02
        look.mesh(f"duct_rib{k}", bm_cylinder((uu, dv, dw), (uu + 0.005, dv, dw), 0.052), m["duct"])
    # a thin trim along the front wall's top, and rubber feet
    look.mesh("front_trim", bm_box((-0.001, -s, C.FRONT_H - 0.006), (0.0, s, C.FRONT_H - 0.002)), m["trim"])
    for fu in (0.03, C.D - 0.03):
        for fv in (-s + 0.03, s - 0.03):
            look.mesh(f"foot_{fu:.2f}_{fv:.2f}", bm_box((fu - 0.015, fv - 0.015, -0.0005), (fu + 0.015, fv + 0.015, 0.0)),
                      m["rubber"])
    return look


def _lid_meshes(look, C, place, m):
    """The lid's meshes in machine coordinates, closed, moved by ``place`` (a Matrix) into the look."""
    s, t = C.W / 2, C.WALL
    u1 = C.LID_U[1]
    frame_w = 0.035
    top0 = C.H - t
    for nm, lo, hi in [
        ("frame_near", (0.0, -s, top0), (frame_w, s, C.H)),
        ("frame_far", (u1 - frame_w, -s, top0), (u1, s, C.H)),
        ("frame_right", (frame_w, -s, top0), (u1 - frame_w, -s + frame_w, C.H)),
        ("frame_left", (frame_w, s - frame_w, top0), (u1 - frame_w, s, C.H)),
    ]:
        look.mesh(nm, bm_box(lo, hi), m["body"], bevel=0.002, place=place)
    look.mesh("window", bm_box((frame_w, -s + frame_w, top0 + 0.003), (u1 - frame_w, s - frame_w, C.H - 0.003)),
              m["window"], place=place)
    look.mesh("flap", bm_box((0.0, -s, C.FLAP_W), (t, s, top0)), m["body"], bevel=0.002, place=place)
    look.mesh("flap_trim", bm_box((-0.001, -s + 0.01, C.FLAP_W + 0.006), (0.0, s - 0.01, C.FLAP_W + 0.010)), m["trim"],
              place=place)
    hu, hw, ht = C.HANDLE_U, C.HANDLE_W, C.HANDLE_T
    look.mesh("handle_bar", bm_cylinder((hu, -C.HANDLE_V + 0.002, hw), (hu, C.HANDLE_V - 0.002, hw), ht / 2, n=20),
              m["black"], place=place)
    for k, vv in enumerate((-C.HANDLE_V + ht / 2, C.HANDLE_V - ht / 2)):
        look.mesh(f"handle_post{k}", bm_box((hu - ht / 2 + 0.002, vv - ht / 2, hw - ht / 2),
                                            (-0.0005, vv + ht / 2, hw + ht / 2)), m["trim"], bevel=0.002, place=place)
    # two hinge barrels at the back
    for k, vv in enumerate((-s + 0.06, s - 0.06)):
        look.mesh(f"hinge{k}", bm_cylinder((u1 - 0.007, vv - 0.03, C.H - 0.007), (u1 - 0.007, vv + 0.03, C.H - 0.007),
                                           0.006), m["steel"], place=place)


def build_lid(C):
    """The lid, closed, in the frame of the lid's own box (centred on C.LID_CENTRE): it must stay inside that box."""
    look = Look("cutter_lid", C.LID_SIZE, shift(C.LID_CENTRE))
    _lid_meshes(look, C, look.frame, _mats())
    return look


def build_open_lid(C, deg, carrier_centre, name="cutter_lid_open"):
    """The lid standing open ``deg`` degrees with its two gas struts, in the frame of a fixed box centred at machine
    point ``carrier_centre``."""
    look = Look(name, None, shift(carrier_centre))
    m = _mats()
    _lid_meshes(look, C, look.frame @ lid_turn(C, deg), m)
    # the struts: from the body's side walls, low at the front, up to the lid's sides
    turned = lid_turn(C, deg)
    s = C.W / 2
    for k, vv in enumerate((-s - 0.008, s + 0.008)):
        a = Vector((C.HINGE_U - 0.10, vv, C.H - 0.09))
        b = turned @ Vector((C.HINGE_U - 0.16, vv, C.H - 0.01))
        mid = a + (b - a) * 0.55
        look.mesh(f"strut_body{k}", bm_cylinder(a, mid, 0.009), m["black"])
        look.mesh(f"strut_rod{k}", bm_cylinder(mid, b, 0.004), m["steel"])
    return look


# -- export --------------------------------------------------------------------------------------------------------


def _local_bounds(look):
    lo = Vector((1e9, 1e9, 1e9))
    hi = -lo
    dg = bpy.context.evaluated_depsgraph_get()
    for ob in look.coll.objects:
        if ob.type != "MESH":
            continue
        me = ob.evaluated_get(dg).to_mesh()
        for v in me.vertices:
            p = ob.matrix_world @ v.co
            lo = Vector(map(min, lo, p))
            hi = Vector(map(max, hi, p))
        ob.evaluated_get(dg).to_mesh_clear()
    return lo, hi


def export(look, file_name=None):
    bpy.context.view_layer.update()
    lo, hi = _local_bounds(look)
    if look.size is not None:
        half = Vector(look.size) / 2
        over = max(max(-lo[i] - half[i], hi[i] - half[i]) for i in range(3))
        if over > 1e-5:
            raise ValueError(f"{look.name}: the look sticks {over * 1000:.2f} mm out of its box ({lo}, {hi})")
    path = os.path.join(S["here"], (file_name or look.name) + ".usdc")
    bpy.ops.wm.usd_export(filepath=path, collection=look.coll.name, export_materials=True,
                          generate_preview_surface=True, export_textures_mode="KEEP", relative_paths=True,
                          root_prim_path="/look", export_lights=False, export_cameras=False,
                          convert_world_material=False, export_custom_properties=False, author_blender_name=False,
                          evaluation_mode="RENDER", export_uvmaps=True, export_normals=True)
    _opacity(path)
    return {"file": os.path.basename(path), "kB": round(os.path.getsize(path) / 1024),
            "low": tuple(round(v, 4) for v in lo), "high": tuple(round(v, 4) for v in hi)}


def _opacity(path):
    """Blender's exporter writes every material opaque: write each see-through material's alpha as its opacity."""
    from pxr import Sdf, Usd, UsdShade

    alphas = {m.name.replace("-", "_"): m.node_tree.nodes["Principled BSDF"].inputs["Alpha"].default_value
              for m in _MATS.values()}
    stage = Usd.Stage.Open(path)
    changed = False
    for prim in stage.Traverse():
        if not prim.IsA(UsdShade.Material):
            continue
        alpha = alphas.get(prim.GetName(), 1.0)
        if alpha >= 1.0:
            continue
        for child in prim.GetChildren():
            shader = UsdShade.Shader(child)
            if shader and shader.GetIdAttr().Get() == "UsdPreviewSurface":
                shader.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(float(alpha))
                shader.CreateInput("opacityThreshold", Sdf.ValueTypeNames.Float).Set(0.0)
                changed = True
    if changed:
        stage.GetRootLayer().Save()


def clear():
    """Remove everything made before with this prefix, and nothing else."""
    p = S["prefix"]
    for coll in [c for c in bpy.data.collections if c.name.startswith(p)]:
        for ob in list(coll.objects):
            bpy.data.objects.remove(ob)
        bpy.data.collections.remove(coll)
    for ob in [o for o in bpy.data.objects if o.name.startswith(p)]:
        bpy.data.objects.remove(ob)
    for block in (bpy.data.meshes, bpy.data.materials, bpy.data.images):
        for item in [i for i in block if i.name.startswith(p)]:
            block.remove(item)
    _MATS.clear()
