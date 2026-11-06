"""make_looks.py: model the SD card, the card reader and the bench shelf in Blender and export them as USD next to this
file.

    blender -b -P looks/make_looks.py          (Blender 4.2 or later, from the task folder)

The physics stays the task file's boxes. Each look is a render-only mesh that ``looks.py`` puts on one of those boxes,
whose plain cuboid (and its collider parts) is then hidden but still collides. Each look is built around the origin,
in its box's own frame (x toward the back of the shelf, y to the robot's left, z up). The card's and the reader's
looks are checked to fit inside their boxes when exported.

- ``card.usdc``: the SD card, 32 x 24 x 2.1 mm, its front right corner cut 5 mm at 45 degrees: black body, a red
  label on top, nine gold contacts under its front edge.
- ``reader.usdc``: the card reader and USB hub, 45 x 140 x 14 mm: black body with rounded corners, the SD slot
  (25 x 3 mm, 24 mm deep) in a silver frame at its right end, a microSD slot beside it, two USB-A ports at its left
  end, a blue light on top.
- ``shelf.usdc``: the bench shelf, a 56 x 23.5 cm oak top 18 mm thick on two end panels, and the reader's flat USB
  cable lying on it to the back edge.

Textures are drawn here with numpy and saved as JPEG in ``textures/``. Nothing is downloaded. Everything this script
makes in Blender is named with the ``sd-card-insert__`` prefix, and only that is removed when it runs again, so it can
run in a Blender session that holds other work.
"""

import math
import os

import bmesh
import bpy
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
TEX = os.path.join(HERE, "textures")
PREFIX = "sd-card-insert__"
RNG = np.random.default_rng(20)
INSET = 0.0001  # m: each scored look sits this far inside its box

# -- the task file's numbers (sd_card_insert.py) ----------------------------------------------------------------------
CARD_SIZE = (0.032, 0.024, 0.0021)
CHAMFER = 0.005
READER_SIZE = (0.045, 0.14, 0.014)
SLOT_W, SLOT_H, SLOT_FLOOR, SLOT_DEPTH, RIGHT_WALL = 0.025, 0.003, 0.005, 0.024, 0.008
SHELF_SIZE = (0.235, 0.56, 0.018)  # its top board
SHELF_H = 0.13  # its top above the table
SHELF_CENTRE = (0.375 + 0.235 / 2, -0.02)  # world x, y
READER_CENTRE = (0.375 + 0.0225, -0.085 - 0.0125 - 0.008 + 0.07)  # world x, y


def n(name):
    return PREFIX + name


def clear():
    for coll in [c for c in bpy.data.collections if c.name.startswith(PREFIX)]:
        for ob in list(coll.objects):
            bpy.data.objects.remove(ob, do_unlink=True)
        bpy.data.collections.remove(coll)
    for blocks in (bpy.data.objects, bpy.data.meshes, bpy.data.curves, bpy.data.materials, bpy.data.images):
        for b in [b for b in blocks if b.name.startswith(PREFIX)]:
            blocks.remove(b)


# -- textures --------------------------------------------------------------------------------------------------------


def _noise(h, w, cy, cx):
    gh, gw = h // cy + 2, w // cx + 2
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
    img = bpy.data.images.new(n(name + "_draw"), w, h, alpha=False)
    rgba = np.concatenate([np.clip(rgb, 0, 1), np.ones((h, w, 1))], axis=2).astype(np.float32)
    img.pixels.foreach_set(rgba.ravel())
    img.filepath_raw = path
    img.file_format = "JPEG"
    img.save()
    bpy.data.images.remove(img)
    loaded = bpy.data.images.load(path)
    loaded.name = n(name)
    return loaded


def card_label():
    """The label, seen from above: u along the card (back to contacts), v across it. Red, a white band near the
    back with dark print, a white capacity block and small print. No brand."""
    h, w = 240, 320  # v rows, u columns (24 x 32 mm at 10 px/mm)
    u = np.arange(w)[None, :] / 10.0  # mm from the back edge
    v = np.arange(h)[:, None] / 10.0  # mm from the right edge
    rgb = np.empty((h, w, 3))
    rgb[:] = (0.78, 0.06, 0.05)
    rgb *= (0.94 + 0.06 * _noise(h, w, 8, 8))[..., None]
    band = np.broadcast_to((u > 3) & (u < 9.5), (h, w))
    rgb[band] = (0.93, 0.93, 0.90)
    # dark print in the band: two lines of glyph blocks
    for row in range(2):
        for i in range(9 - 3 * row):
            v_lo = 4.0 + i * 1.6
            m = (u > 4.0 + row * 2.8) & (u < 5.6 + row * 2.8) & (v > v_lo) & (v < v_lo + 1.1)
            rgb[np.broadcast_to(m, (h, w))] = (0.10, 0.10, 0.12)
    # a white block with the capacity, in the label's middle
    block = (u > 14) & (u < 21) & (v > 7) & (v < 17)
    rgb[np.broadcast_to(block, (h, w))] = (0.96, 0.96, 0.94)
    for i in range(3):
        m = (u > 15.5) & (u < 19.5) & (v > 8.2 + i * 2.9) & (v < 10.4 + i * 2.9)
        rgb[np.broadcast_to(m, (h, w))] = (0.78, 0.06, 0.05)
    # small white print near the contacts
    for i in range(10):
        m = (u > 25) & (u < 26.2) & (v > 3 + i * 1.8) & (v < 4.1 + i * 1.8)
        rgb[np.broadcast_to(m, (h, w))] = (0.95, 0.92, 0.90)
    return _save("card_label", rgb)


def oak():
    """Oak veneer for the shelf's top: grain along the image's width (the shelf's length)."""
    h, w = 512, 1536
    y = np.arange(h)[:, None] / h
    x = np.arange(w)[None, :] / w
    wave = 0.02 * np.sin(2 * math.pi * (x * 2.0 + 0.3)) + 0.02 * _noise(h, w, 64, 256)
    grain = np.sin(2 * math.pi * (y + wave) * 60.0) * 0.5 + 0.5
    grain = grain ** 5 * 0.55 + _noise(h, w, 3, 90) * 0.45
    pores = (_noise(h, w, 1, 12) > 0.82) * 0.25
    base, dark = np.array([0.74, 0.56, 0.36]), np.array([0.52, 0.36, 0.20])
    t = np.clip(0.55 * grain + 0.2 * _noise(h, w, 48, 160) + pores, 0, 1)[..., None]
    return _save("oak", base * (1 - t) + dark * t)


# -- materials -------------------------------------------------------------------------------------------------------


def material(name, color, rough=0.5, metal=0.0, image=None, emit=None):
    m = bpy.data.materials.new(n(name))
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
    if emit is not None:
        key = "Emission Color" if "Emission Color" in bsdf.inputs else "Emission"
        bsdf.inputs[key].default_value = (*emit, 1.0)
        if "Emission Strength" in bsdf.inputs:
            bsdf.inputs["Emission Strength"].default_value = 4.0
    if image is not None:
        tex = nodes.new("ShaderNodeTexImage")
        tex.image = image
        tex.extension = "EXTEND"
        links.new(tex.outputs["Color"], bsdf.inputs["Base Color"])
    m.diffuse_color = (*color, 1.0)
    return m


# -- meshes ----------------------------------------------------------------------------------------------------------


class Look:
    """A collection holding one look's meshes, built around the origin in its box's frame."""

    def __init__(self, name, size=None):
        self.name, self.size = name, size
        self.coll = bpy.data.collections.new(n("look_" + name))
        bpy.context.scene.collection.children.link(self.coll)

    def mesh(self, name, bm, mats, smooth=False):
        me = bpy.data.meshes.new(n(name))
        bm.normal_update()
        bm.to_mesh(me)
        bm.free()
        if smooth:
            me.shade_smooth()
        for m in mats:
            me.materials.append(m)
        ob = bpy.data.objects.new(n(name), me)
        self.coll.objects.link(ob)
        return ob


def _cutters():
    name = n("cutters")
    coll = bpy.data.collections.get(name)
    if coll is None:
        coll = bpy.data.collections.new(name)
        bpy.context.scene.collection.children.link(coll)
    return coll


def box(lo, hi):
    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=1.0)
    for v in bm.verts:
        v.co = [lo[i] + (v.co[i] + 0.5) * (hi[i] - lo[i]) for i in range(3)]
    return bm


def prism(pts, z0, z1):
    """A polygon in the xy plane extruded from z0 to z1."""
    bm = bmesh.new()
    lo = [bm.verts.new((x, y, z0)) for x, y in pts]
    hi = [bm.verts.new((x, y, z1)) for x, y in pts]
    bm.faces.new(list(reversed(lo)))
    bm.faces.new(hi)
    k = len(pts)
    for i in range(k):
        bm.faces.new((lo[i], lo[(i + 1) % k], hi[(i + 1) % k], hi[i]))
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    return bm


def cylinder(centre, r, h, segs=24):
    bm = bmesh.new()
    bmesh.ops.create_cone(bm, cap_ends=True, segments=segs, radius1=r, radius2=r, depth=h)
    for v in bm.verts:
        v.co = (v.co[0] + centre[0], v.co[1] + centre[1], v.co[2] + centre[2])
    return bm


def planar_uv(bm, lo, hi, axes=(0, 1)):
    """UVs from two coordinates over [lo, hi]."""
    uv = bm.loops.layers.uv.new("UVMap")
    a, b = axes
    for f in bm.faces:
        for loop in f.loops:
            p = loop.vert.co
            loop[uv].uv = ((p[a] - lo[0]) / (hi[0] - lo[0]), (p[b] - lo[1]) / (hi[1] - lo[1]))


def build_card(M):
    look = Look("card", CARD_SIZE)
    cx, cy, cz = (v / 2 - INSET for v in CARD_SIZE)
    c = CHAMFER
    pad_t = 0.00008
    outline = [(-cx, -cy), (cx - c, -cy), (cx, -cy + c), (cx, cy), (-cx, cy)]
    bm = prism(outline, -cz + pad_t, cz)
    planar_uv(bm, (-cx, cy), (cx, -cy))  # u along the card, v from its right edge (texture rows from the right)
    uv = bm.loops.layers.uv.active
    for f in bm.faces:
        f.material_index = 1 if f.normal.z > 0.5 else 0
        if f.material_index == 1:
            for loop in f.loops:
                p = loop.vert.co
                loop[uv].uv = ((p.x + cx) / (2 * cx), (p.y + cy) / (2 * cy))
    look.mesh("card_body", bm, [M["card_black"], M["label"]])
    # nine contacts under the front edge, 2 mm from it
    pitch = (2 * cy - c - 0.001 - 0.0015) / 8
    for i in range(9):
        y = -cy + c + 0.0005 + i * pitch
        look.mesh(f"contact_{i}", box((cx - 0.0062, y, -cz), (cx - 0.0018, y + 0.0015, -cz + pad_t)), [M["gold"]])
    # the write-protect switch on the left edge, a white tab in a groove
    look.mesh("lock_switch", box((-0.004, cy - 0.0004, -0.0005), (0.0005, cy + 0.00009, 0.0005)), [M["white"]])
    return look


def build_reader(M):
    look = Look("reader", READER_SIZE)
    rx, ry, rz = (v / 2 - INSET for v in READER_SIZE)
    hx, hy, hz = (v / 2 for v in READER_SIZE)
    slot_y = -hy + RIGHT_WALL + SLOT_W / 2
    z0 = -hz + SLOT_FLOOR
    # the body: a box with rounded corners (top view) and softened top edges
    bm = box((-rx, -ry, -rz), (rx, ry, rz))
    vert = [e for e in bm.edges if abs(e.verts[0].co.z - e.verts[1].co.z) > 1e-6]
    bmesh.ops.bevel(bm, geom=vert, offset=0.004, segments=4, affect="EDGES", profile=0.5)
    top = [e for e in bm.edges if all(v.co.z > rz - 1e-6 for v in e.verts)]
    bmesh.ops.bevel(bm, geom=top, offset=0.0012, segments=2, affect="EDGES", profile=0.5)
    body = look.mesh("reader_body", bm, [M["reader_black"]], smooth=False)
    cutters = []
    for name, lo, hi in (
        ("sd_slot", (-hx - 0.001, slot_y - SLOT_W / 2, z0), (-hx + SLOT_DEPTH, slot_y + SLOT_W / 2, z0 + SLOT_H)),
        ("sd_lip", (-hx - 0.001, slot_y - SLOT_W / 2 - 0.0005, z0 - 0.0006), (-hx + 0.0012, slot_y + SLOT_W / 2 + 0.0005,
                                                                            z0 + SLOT_H + 0.0006)),
        ("micro_slot", (-hx - 0.001, slot_y + SLOT_W / 2 + 0.008, z0 + 0.0007),
         (-hx + 0.004, slot_y + SLOT_W / 2 + 0.019, z0 + 0.0017)),
        *((f"usb_{i}", (-hx - 0.001, hy - 0.012 - i * 0.02 - 0.0125, z0 - 0.0005),
           (-hx + 0.012, hy - 0.012 - i * 0.02, z0 + 0.0045)) for i in range(2)),
    ):
        me = bpy.data.meshes.new(n("cut_" + name))
        b = box(lo, hi)
        b.to_mesh(me)
        b.free()
        ob = bpy.data.objects.new(n("cut_" + name), me)
        _cutters().objects.link(ob)
        ob.hide_render = True
        ob.display_type = "WIRE"
        cutters.append(ob)
    for ob in cutters:
        mod = body.modifiers.new(n("cut"), "BOOLEAN")
        mod.operation, mod.object, mod.solver = "DIFFERENCE", ob, "EXACT"
    # inside the slot: dark contact springs on the roof at the back
    look.mesh("slot_back", box((-hx + SLOT_DEPTH - 0.0004, slot_y - SLOT_W / 2, z0),
                                (-hx + SLOT_DEPTH, slot_y + SLOT_W / 2, z0 + SLOT_H)), [M["slot_dark"]])
    # the USB ports' tongues
    for i in range(2):
        y1 = hy - 0.012 - i * 0.02
        look.mesh(f"usb_tongue_{i}", box((-hx + 0.002, y1 - 0.0115, z0 + 0.0022), (-hx + 0.012, y1 - 0.001, z0 + 0.0040)),
                  [M["usb_blue"]])
    # a brushed silver frame round the SD slot's mouth
    f0, f1 = -rx, -hx + 0.0004
    w = 0.0012
    ylo, yhi = slot_y - SLOT_W / 2 - w - 0.0005, slot_y + SLOT_W / 2 + w + 0.0005
    zlo, zhi = z0 - 0.0006 - w, z0 + SLOT_H + 0.0006 + w
    for name, lo, hi in (
        ("frame_bottom", (f0, ylo, zlo), (f1, yhi, z0 - 0.0006)),
        ("frame_top", (f0, ylo, z0 + SLOT_H + 0.0006), (f1, yhi, zhi)),
        ("frame_right", (f0, ylo, z0 - 0.0006), (f1, slot_y - SLOT_W / 2 - 0.0005, z0 + SLOT_H + 0.0006)),
        ("frame_left", (f0, slot_y + SLOT_W / 2 + 0.0005, z0 - 0.0006), (f1, yhi, z0 + SLOT_H + 0.0006)),
    ):
        look.mesh(name, box(lo, hi), [M["silver"]])
    # the grey top panel and the blue light
    panel = box((-rx + 0.006, -ry + 0.006, rz - 0.0002), (rx - 0.006, ry - 0.006, rz))
    look.mesh("top_panel", panel, [M["reader_grey"]])
    look.mesh("light", cylinder((-rx + 0.004, slot_y, rz - 0.0003), 0.0012, 0.0006), [M["blue_light"]], smooth=True)
    return look


def build_shelf(M):
    """The shelf's top board, its end panels and the cable, around the top board's centre."""
    look = Look("shelf")
    dx, wy, t = SHELF_SIZE
    hx, hy, hz = dx / 2, wy / 2, t / 2
    bm = box((-hx, -hy, -hz), (hx, hy, hz))
    front = [e for e in bm.edges if all(v.co.x < -hx + 1e-6 for v in e.verts) and
             abs(e.verts[0].co.y - e.verts[1].co.y) > 1e-6]
    bmesh.ops.bevel(bm, geom=front, offset=0.003, segments=3, affect="EDGES", profile=0.5)
    planar_uv(bm, (-hy, -hx), (hy, hx), axes=(1, 0))
    look.mesh("shelf_top", bm, [M["oak"]])
    leg_h = SHELF_H - t
    for side, s in (("right", -1), ("left", 1)):
        y0, y1 = (s * hy - t, s * hy) if s > 0 else (s * hy, s * hy + t)
        leg = box((-hx, y0, -hz - leg_h), (hx, y1, -hz))
        planar_uv(leg, (-hx, -hz - leg_h), (hx, -hz), axes=(0, 2))
        look.mesh(f"shelf_leg_{side}", leg, [M["oak"]])
    # the reader's flat cable: out of its back, along the shelf to the back edge and down behind it
    rx = READER_CENTRE[0] - SHELF_CENTRE[0] + 0.0225
    ry = READER_CENTRE[1] - SHELF_CENTRE[1] + 0.03
    pts = [(rx, ry), (rx + 0.03, ry + 0.004), (rx + 0.08, ry + 0.03), (hx - 0.01, ry + 0.06), (hx + 0.004, ry + 0.065)]
    for i, ((x0, y0), (x1, y1)) in enumerate(zip(pts, pts[1:])):
        length = math.hypot(x1 - x0, y1 - y0)
        seg = box((-length / 2 - 0.001, -0.0018, hz), (length / 2 + 0.001, 0.0018, hz + 0.0025))
        ang = math.atan2(y1 - y0, x1 - x0)
        bmesh.ops.rotate(seg, verts=seg.verts, cent=(0, 0, 0), matrix=_rz(ang))
        bmesh.ops.translate(seg, verts=seg.verts, vec=((x0 + x1) / 2, (y0 + y1) / 2, 0.0))
        look.mesh(f"cable_{i}", seg, [M["cable"]])
    drop = box((hx + 0.001, pts[-1][1] - 0.0018, -hz - 0.10), (hx + 0.0035, pts[-1][1] + 0.0018, hz + 0.0025))
    look.mesh("cable_drop", drop, [M["cable"]])
    plug = box((rx - 0.0005, ry - 0.005, hz), (rx + 0.008, ry + 0.005, hz + 0.004))
    look.mesh("cable_plug", plug, [M["cable"]])
    return look


def _rz(a):
    from mathutils import Matrix

    return Matrix.Rotation(a, 3, "Z")


# -- export ----------------------------------------------------------------------------------------------------------


def _bounds(look):
    dg = bpy.context.evaluated_depsgraph_get()
    pts = []
    for ob in look.coll.objects:
        if ob.type != "MESH":
            continue
        ev = ob.evaluated_get(dg)
        me = ev.to_mesh()
        pts += [tuple(ob.matrix_world @ v.co) for v in me.vertices]
        ev.to_mesh_clear()
    p = np.asarray(pts)
    return p.min(0), p.max(0)


def export(look):
    bpy.context.view_layer.update()
    lo, hi = _bounds(look)
    if look.size is not None:
        half = np.asarray(look.size) / 2
        over = float(max(np.max(-lo - half), np.max(hi - half)))
        if over > 1e-6:
            raise ValueError(f"{look.name}: the look sticks {over * 1000:.2f} mm out of its box ({lo}, {hi})")
    path = os.path.join(HERE, look.name + ".usdc")
    bpy.ops.wm.usd_export(filepath=path, collection=look.coll.name, export_materials=True,
                          generate_preview_surface=True, export_textures_mode="KEEP", relative_paths=True,
                          root_prim_path="/look", export_lights=False, export_cameras=False,
                          convert_world_material=False, export_custom_properties=False, author_blender_name=False,
                          evaluation_mode="RENDER", export_uvmaps=True, export_normals=True)
    return {"file": os.path.basename(path), "kB": round(os.path.getsize(path) / 1024),
            "low": tuple(round(float(v), 4) for v in lo), "high": tuple(round(float(v), 4) for v in hi)}


def build():
    clear()
    M = {
        "card_black": material("card_black", (0.025, 0.025, 0.028), 0.45),
        "label": material("card_label", (0.78, 0.06, 0.05), 0.4, image=card_label()),
        "gold": material("gold", (1.0, 0.76, 0.34), 0.25, 1.0),
        "white": material("white", (0.9, 0.9, 0.88), 0.5),
        "reader_black": material("reader_black", (0.02, 0.02, 0.022), 0.35),
        "reader_grey": material("reader_grey", (0.10, 0.10, 0.11), 0.25, 0.3),
        "slot_dark": material("slot_dark", (0.004, 0.004, 0.004), 0.7),
        "silver": material("silver", (0.75, 0.76, 0.78), 0.3, 1.0),
        "blue_light": material("blue_light", (0.1, 0.3, 1.0), 0.3, emit=(0.15, 0.4, 1.0)),
        "oak": material("oak", (0.66, 0.49, 0.31), 0.55, image=oak()),
        "cable": material("cable", (0.04, 0.04, 0.045), 0.6),
        "usb_blue": material("usb_blue", (0.05, 0.12, 0.45), 0.4),
    }
    looks = [build_card(M), build_reader(M), build_shelf(M)]
    return {look.name: export(look) for look in looks}


if __name__ == "__main__":
    for name, info in build().items():
        print(name, info)
