"""make_looks.py: model a print-in-place hinge and the protractor card under it in Blender and export them as USD next to
this file.

    blender -b -P looks/make_looks.py          (Blender 4.2 or later, from the task folder)

The physics stays the task file's boxes. Each look is a render-only mesh that ``looks.py`` puts on one of those boxes, whose
plain cuboid is then hidden but still collides. Every look is built in its box's own frame (metres, the box's centre at the
origin, z up) and must fit inside the box (checked when exported).

- ``leaf.usdc``: the orange moving leaf: a 7.5 x 1 x 4 cm plate in PLA with fine layer lines, two screw holes, rounded edges and
  a neck cut round the hinge, and the knuckle, a 13 mm barrel 24 mm tall between the base leaf's two barrels.
- ``base.usdc``: the dark teal base leaf: a 5 x 1 x 4 cm plate with two screw holes, two bridges and the two outer barrels
  of the hinge, and the steel pin through all three with its domed head on top.
- ``card.usdc``: a 28 x 17 cm card, light grey, printed with a protractor round the hinge: ticks every 5 degrees, the
  numbers 0 to 180 every 30, and the part past 60 and past 120 degrees tinted amber and green.

Textures are drawn here with numpy and saved in ``textures/``. Nothing is downloaded. Everything this script makes in
Blender is named with the prefix ``<task folder>__``, and only those datablocks are removed when it runs again.
"""

import math
import os

import bmesh
import bpy
import numpy as np
from mathutils import Matrix, Vector

try:
    HERE = os.path.dirname(os.path.abspath(__file__))
except NameError:  # pasted into a console
    HERE = os.getcwd()
TEX = os.path.join(HERE, "textures")
FOLDER = os.path.basename(os.path.dirname(HERE))
P = FOLDER + "__"
RNG = np.random.default_rng(31)

# -- the boxes (the task file): sizes, and where the hinge axis is in each box's frame -----------------------------------
PLATE_T, PLATE_H = 0.010, 0.040
LEAF = (0.089, 0.014, PLATE_H)  # the plate (7.5 cm, 7 mm off the axis) and the knuckle round the axis
PLATE_LEN = 0.075
KNUCKLE = (0.014, 0.014, 0.024)
BASE = (0.069, 0.013, PLATE_H)
CARD = (0.28, 0.17, 0.002)
BASE_LEN = 0.05
HINGE_IN_BASE = (BASE[0] / 2 - KNUCKLE[0] / 2, 0.0)  # the hinge axis, 7 mm from the base box's right end
CARD_HINGE = (0.0, -0.04)  # the hinge axis in the card's frame
BARREL_R = 0.0065
GAP = 0.0005  # print-in-place clearance between the barrels
ORANGE = (0.95, 0.42, 0.04)
TEAL = (0.06, 0.27, 0.31)


def srgb(c):
    return tuple(((v + 0.055) / 1.055) ** 2.4 if v > 0.04045 else v / 12.92 for v in c)


# -- materials -------------------------------------------------------------------------------------------------------


def material(name, color, rough=0.5, metal=0.0, image=None):
    full = P + name
    m = bpy.data.materials.get(full) or bpy.data.materials.new(full)
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
    return m


# -- textures --------------------------------------------------------------------------------------------------------


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


def layer_texture(name, color, h=500):
    """PLA in 0.2 mm layers (one texture repeat is 1 cm tall, 50 layers)."""
    w = 64
    base = np.array(srgb(color)) ** (1 / 2.2)
    rows = np.arange(h) % 10
    ridge = 1.0 - 0.10 * (rows >= 8) + 0.04 * (rows <= 1)
    rgb = np.ones((h, w, 3)) * base * ridge[:, None, None]
    rgb *= 0.98 + 0.04 * RNG.random((h, w, 1))
    return _save(name, rgb)


DIGITS = {  # seven segments: top, top-left, top-right, middle, bottom-left, bottom-right, bottom
    "0": "1110111", "1": "0010010", "2": "1011101", "3": "1011011", "4": "0111010", "5": "1101011", "6": "1101111",
    "7": "1010010", "8": "1111111", "9": "1111011",
}


def card_texture():
    """The card: light grey paper with a protractor round the hinge axis (CARD_HINGE), 0 degrees along +x, the swing
    counter-clockwise to 180. Rows run up the card's +y, columns along +x."""
    w, h = 1536, 1024
    px = CARD[0] / w
    ys, xs = np.mgrid[:h, :w]
    x = -CARD[0] / 2 + (xs + 0.5) * px - CARD_HINGE[0]
    y = -CARD[1] / 2 + (ys + 0.5) * px - CARD_HINGE[1]
    r = np.hypot(x, y)
    ang = np.degrees(np.arctan2(y, x))  # 0 along +x, 90 along +y, +-180 along -x
    rgb = np.ones((h, w, 3)) * np.array([0.90, 0.90, 0.88]) * (0.985 + 0.03 * RNG.random((h, w, 1)))
    half = ang >= 0.0  # the swing's side
    zone = np.zeros((h, w, 3))
    ring = (r > 0.062) & (r < 0.100) & half
    for lo, hi, col in ((0, 60, (0.90, 0.90, 0.88)), (60, 120, (0.98, 0.86, 0.50)), (120, 180.1, (0.62, 0.85, 0.58))):
        m = ring & (ang >= lo) & (ang < hi)
        rgb[m] = col
    ink = np.array([0.06, 0.06, 0.07])

    def draw(mask):
        rgb[mask] = ink

    draw((np.abs(r - 0.100) < 0.00045) & half)
    draw((np.abs(r - 0.062) < 0.00035) & half)
    draw((np.abs(y) < 0.0003) & (np.abs(x) < 0.110) & (np.abs(x) > 0.062))  # the 0 and 180 degree line
    draw((np.abs(x) < 0.0003) & (y > 0.062) & (y < 0.112))  # the 90 degree line
    draw((r < 0.0016))  # the hinge axis
    for deg in range(0, 181, 5):
        long = deg % 30 == 0
        mid = deg % 10 == 0
        r0, r1 = (0.100, 0.110) if long else (0.100, 0.106) if mid else (0.100, 0.1035)
        draw((r > r0) & (r < r1) & (np.abs(np.radians(ang - deg)) * r < (0.0006 if long else 0.0004)) & half)
    # numbers 0..180 every 30 degrees, 5 x 8 mm, in seven segments
    seg_boxes = [(0.0, 1.0, 0.85, 1.0), (0.0, 0.2, 0.5, 1.0), (0.8, 1.0, 0.5, 1.0), (0.0, 1.0, 0.425, 0.575),
                 (0.0, 0.2, 0.0, 0.5), (0.8, 1.0, 0.0, 0.5), (0.0, 1.0, 0.0, 0.15)]

    def glyph(ch, cx, cy, gw=0.0038, gh=0.0062):
        for on, (x0, x1, y0, y1) in zip(DIGITS[ch], seg_boxes):
            if on == "1":
                gx, gy = x - (cx - gw / 2), y - (cy - gh / 2)
                draw((gx >= x0 * gw) & (gx <= x1 * gw) & (gy >= y0 * gh) & (gy <= y1 * gh))

    for deg in range(0, 181, 30):
        text = str(deg)
        a = math.radians(deg)
        cx, cy = 0.1165 * math.cos(a), 0.1165 * math.sin(a)
        pitch = 0.0052
        for i, ch in enumerate(text):
            glyph(ch, cx + (i - (len(text) - 1) / 2) * pitch, cy)
    return _save("card_protractor", rgb)


# -- building --------------------------------------------------------------------------------------------------------


class Look:
    def __init__(self, name, size):
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

    def box(self, name, a, b, mat, bevel=0.0, uv=None):
        bm = cuboid(a, b)
        if uv:
            uv_box(bm, *uv)
        return self.mesh(name, bm, mat, bevel)

    def cyl_z(self, name, x, y, z0, z1, r, mat, n=32, bevel=0.0, uv=None):
        bm = bmesh.new()
        bmesh.ops.create_cone(bm, cap_ends=True, segments=n, radius1=r, radius2=r, depth=z1 - z0)
        bmesh.ops.translate(bm, vec=(x, y, (z0 + z1) / 2), verts=bm.verts)
        if uv:
            uv_cyl(bm, *uv)
        return self.mesh(name, bm, mat, bevel, smooth=True)

    def holes(self, ob, centres, r, depth):
        """Cut round holes through ``ob`` along y (boolean difference, applied when exported)."""
        cutters = _cutter_collection()
        for i, (x, z) in enumerate(centres):
            bm = bmesh.new()
            bmesh.ops.create_cone(bm, cap_ends=True, segments=24, radius1=r, radius2=r, depth=depth)
            bmesh.ops.transform(bm, matrix=Matrix.Rotation(math.pi / 2, 4, "X"), verts=bm.verts)
            bmesh.ops.translate(bm, vec=(x, 0.0, z), verts=bm.verts)
            me = bpy.data.meshes.new(P + self.name + f"_cut{i}")
            bm.to_mesh(me)
            bm.free()
            cut = bpy.data.objects.new(P + self.name + f"_cut{i}", me)
            cutters.objects.link(cut)
            mod = ob.modifiers.new(f"hole{i}", "BOOLEAN")
            mod.operation, mod.object, mod.solver = "DIFFERENCE", cut, "EXACT"
            cut.hide_render = True


def _cutter_collection():
    cutters = bpy.data.collections.get(P + "cutters") or bpy.data.collections.new(P + "cutters")
    if cutters.name not in bpy.context.scene.collection.children:
        bpy.context.scene.collection.children.link(cutters)
    return cutters


def cut_boxes(look, ob, boxes):
    """Cut boxes ((x0, y0, z0), (x1, y1, z1)) out of ``ob`` (boolean difference, applied when exported)."""
    for i, (a, b) in enumerate(boxes):
        me = bpy.data.meshes.new(P + look.name + f"_boxcut{i}")
        bm = cuboid(a, b)
        bm.to_mesh(me)
        bm.free()
        cut = bpy.data.objects.new(P + look.name + f"_boxcut{i}", me)
        _cutter_collection().objects.link(cut)
        mod = ob.modifiers.new(f"notch{i}", "BOOLEAN")
        mod.operation, mod.object, mod.solver = "DIFFERENCE", cut, "EXACT"
        cut.hide_render = True


def cuboid(a, b):
    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=1.0)
    s = [b[i] - a[i] for i in range(3)]
    c = [(a[i] + b[i]) / 2 for i in range(3)]
    bmesh.ops.scale(bm, vec=s, verts=bm.verts)
    bmesh.ops.translate(bm, vec=c, verts=bm.verts)
    return bm


def uv_box(bm, su, sv):
    """Box mapping: each face projected on its dominant plane, ``su`` / ``sv`` texture repeats per metre."""
    layer = bm.loops.layers.uv.new("UVMap")
    for f in bm.faces:
        n = f.normal
        ax = max(range(3), key=lambda i: abs(n[i]))
        u_i, v_i = [(1, 2), (0, 2), (0, 1)][ax]
        for loop in f.loops:
            co = loop.vert.co
            loop[layer].uv = (co[u_i] * su, co[v_i] * sv)


def uv_cyl(bm, su, sv):
    layer = bm.loops.layers.uv.new("UVMap")
    for f in bm.faces:
        for loop in f.loops:
            co = loop.vert.co
            loop[layer].uv = (math.atan2(co.y, co.x) * su, co.z * sv)


def uv_planar(bm, size):
    layer = bm.loops.layers.uv.new("UVMap")
    for f in bm.faces:
        for loop in f.loops:
            co = loop.vert.co
            loop[layer].uv = (co.x / size[0] + 0.5, co.y / size[1] + 0.5)


# -- the looks -------------------------------------------------------------------------------------------------------


def build_leaf():
    L = Look("leaf", LEAF)
    pla = material("leaf_pla", srgb(ORANGE), 0.42, image=layer_texture("leaf_layers", ORANGE))
    hx = -LEAF[0] / 2 + KNUCKLE[0] / 2  # the hinge axis in the leaf's frame
    x0 = hx + KNUCKLE[0] / 2  # the plate starts at the knuckle's face
    plate = L.box("plate", (x0, -PLATE_T / 2, -PLATE_H / 2), (x0 + PLATE_LEN, PLATE_T / 2, PLATE_H / 2), pla,
                  bevel=0.0012, uv=(20.0, 100.0))
    L.holes(plate, [(x0 + 0.0085, 0.0), (x0 + 0.0635, 0.0)], 0.0028, 0.03)
    # the neck: the plate's top and bottom are cut away round the hinge, where the base leaf's barrels and bridges reach in
    band = (KNUCKLE[2] / 2, PLATE_H / 2 + 0.001)
    cut_boxes(L, plate, [((x0 - 0.001, -0.01, band[0]), (x0 + 0.0185, 0.01, band[1])),
                         ((x0 - 0.001, -0.01, -band[1]), (x0 + 0.0185, 0.01, -band[0]))])
    L.cyl_z("knuckle", hx, 0.0, -KNUCKLE[2] / 2, KNUCKLE[2] / 2, BARREL_R, pla, bevel=0.0008, uv=(40.0, 100.0))
    return L


def build_base():
    L = Look("base", BASE)
    pla = material("base_pla", srgb(TEAL), 0.42, image=layer_texture("base_layers", TEAL))
    steel = material("hinge_pin", srgb((0.74, 0.75, 0.78)), 0.25, 1.0)
    hx, hy = HINGE_IN_BASE
    x0 = -BASE[0] / 2
    plate = L.box("plate", (x0, -PLATE_T / 2, -PLATE_H / 2), (x0 + BASE_LEN, PLATE_T / 2, PLATE_H / 2), pla,
                  bevel=0.0012, uv=(20.0, 100.0))
    L.holes(plate, [(-0.012, 0.0), (0.0, 0.0)], 0.0028, 0.03)
    top0, top1 = KNUCKLE[2] / 2 + GAP, PLATE_H / 2
    for tag, z0, z1 in (("top", top0, top1), ("bottom", -top1, -top0)):
        L.box(f"bridge_{tag}", (x0 + BASE_LEN - 0.002, -PLATE_T / 2, z0), (hx, PLATE_T / 2, z1), pla, bevel=0.0008,
              uv=(20.0, 100.0))
        L.cyl_z(f"barrel_{tag}", hx, hy, z0, z1, BARREL_R, pla, bevel=0.0006, uv=(40.0, 100.0))
    # the pin: a 3 mm steel rod through the three barrels, its head a low dome on the top one
    L.cyl_z("pin", hx, hy, -PLATE_H / 2 + 0.0005, PLATE_H / 2 - 0.0006, 0.0015, steel)
    bm = bmesh.new()
    bmesh.ops.create_uvsphere(bm, u_segments=24, v_segments=8, radius=0.0032)
    bmesh.ops.scale(bm, vec=(1, 1, 0.25), verts=bm.verts)
    bmesh.ops.translate(bm, vec=(hx, hy, PLATE_H / 2 - 0.0008), verts=bm.verts)
    L.mesh("pin_head", bm, steel, smooth=True)
    return L


def build_card():
    L = Look("card", CARD)
    paper = material("card_paper", (0.9, 0.9, 0.88), 0.7, image=card_texture())
    bm = cuboid((-CARD[0] / 2, -CARD[1] / 2, -CARD[2] / 2), (CARD[0] / 2, CARD[1] / 2, CARD[2] / 2))
    uv_planar(bm, CARD)
    L.mesh("card", bm, paper, bevel=0.0004)
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
    """Remove every datablock this script made (named with the prefix), and nothing else."""
    for coll in [c for c in bpy.data.collections if c.name.startswith(P)]:
        for ob in list(coll.objects):
            if ob.name.startswith(P):
                bpy.data.objects.remove(ob, do_unlink=True)
        bpy.data.collections.remove(coll)
    for ob in [o for o in bpy.data.objects if o.name.startswith(P)]:
        bpy.data.objects.remove(ob, do_unlink=True)
    for store in (bpy.data.meshes, bpy.data.materials, bpy.data.images):
        for block in [b for b in store if b.name.startswith(P)]:
            store.remove(block)


BUILDERS = {"leaf": build_leaf, "base": build_base, "card": build_card}


def build(names=None, hide=True):
    clear()
    looks = [BUILDERS[n]() for n in (names or list(BUILDERS))]
    out = {look.name: export(look) for look in looks}
    cutters = bpy.data.collections.get(P + "cutters")
    if hide:  # keep the session tidy for the others using it
        for look in looks:
            bpy.context.view_layer.layer_collection.children[look.coll.name].exclude = True
        if cutters:
            bpy.context.view_layer.layer_collection.children[cutters.name].exclude = True
    return out


if __name__ == "__main__":
    print(build())
