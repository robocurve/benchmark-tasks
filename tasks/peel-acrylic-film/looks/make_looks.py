"""make_looks.py: model the scene's visual meshes in Blender and export them as USD next to this file.

    blender -b -P looks/make_looks.py          (Blender 4.2 or later, from the task folder)

The physics stays the task file's boxes. Each look is a render-only mesh that ``looks.py`` puts on one of those boxes,
whose plain cuboid is then hidden but still collides. Every look is built in the task file's frame (metres, z up)
around an empty at its box's centre, and exported with that empty at the origin. Each scored look fills its box and
stays inside it (checked when exported).

- ``mat.usdc``: a green self-healing cutting mat with its printed grid.
- ``sheet.usdc``: the opal white acrylic sheet, glossy, with rounded edges.
- ``strip_1.usdc`` to ``strip_4.usdc``: the brown paper film, printed as one piece across the four strips.
- ``tab.usdc``: the folded paper tab, with a printed arrow.

Textures are drawn here with numpy and saved in ``textures/``. Nothing is downloaded. Every datablock this script makes
is named with the ``peel-acrylic-film__`` prefix, and only those are ever removed.
"""

import math
import os

import bmesh
import bpy
import numpy as np
from mathutils import Vector

HERE = os.path.dirname(os.path.abspath(__file__))
TEX = os.path.join(HERE, "textures")
PREFIX = "peel-acrylic-film__"
RNG = np.random.default_rng(14)

# -- the task file's layout (peel_acrylic_film.py), metres -------------------------------------------------------------
TOP = 0.7475
MAT_C, MAT_SIZE = (0.33, -0.05, TOP + 0.0015), (0.22, 0.24, 0.003)
MAT_TOP = TOP + MAT_SIZE[2]
SHEET_SIZE = (0.15, 0.20, 0.003)
SHEET_C = (0.33, -0.05, MAT_TOP + SHEET_SIZE[2] / 2)
SHEET_TOP = MAT_TOP + SHEET_SIZE[2]
SHEET_FAR = SHEET_C[0] + SHEET_SIZE[0] / 2
STRIPS, STRIP_W, FILM_LEN, STRIP_T = 4, SHEET_SIZE[0] / 4, 0.10, 0.0015
FILM_Y = SHEET_C[1] - SHEET_SIZE[1] / 2 + FILM_LEN / 2
STRIP_TOP = SHEET_TOP + STRIP_T
TAB_SIZE = (0.003, 0.03, 0.025)
TAB_C = (SHEET_FAR - TAB_SIZE[0] / 2, FILM_Y, STRIP_TOP + TAB_SIZE[2] / 2)
FILM_X0, FILM_Y0 = SHEET_FAR - STRIPS * STRIP_W, FILM_Y - FILM_LEN / 2  # the film's corner, for its texture


def strip_c(i):
    return (SHEET_FAR - (i - 0.5) * STRIP_W, FILM_Y, SHEET_TOP + STRIP_T / 2)


# -- materials (linear colours) ---------------------------------------------------------------------------------------
_MATS = {}


def material(name, color=(0.5, 0.5, 0.5), rough=0.5, metal=0.0, image=None, alpha=1.0):
    name = PREFIX + name
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
    bsdf.inputs["Alpha"].default_value = alpha
    if image is not None:
        tex = nodes.new("ShaderNodeTexImage")
        tex.image = image
        tex.extension = "EXTEND"
        links.new(tex.outputs["Color"], bsdf.inputs["Base Color"])
    m.diffuse_color = (*color, alpha)
    _MATS[name] = m
    return m


# -- textures ---------------------------------------------------------------------------------------------------------


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
    full = PREFIX + name
    for old in [i for i in bpy.data.images if i.name.startswith(full)]:
        bpy.data.images.remove(old)
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


def mat_texture():
    """A green self-healing cutting mat: 1 cm grid, a bolder line every 5 cm, 45 degree guides, knife scores."""
    k = 4000  # pixels per metre
    w, h = int(MAT_SIZE[0] * k), int(MAT_SIZE[1] * k)
    x0, y0 = MAT_C[0] - MAT_SIZE[0] / 2, MAT_C[1] - MAT_SIZE[1] / 2
    ys, xs = np.mgrid[0:h, 0:w]
    X, Y = xs / k + x0 - 0.21, ys / k + y0 + 0.17  # grid origin off the mat's corner, like a cut-down mat
    base = np.array([0.07, 0.30, 0.17])
    rgb = base[None, None, :] * (0.93 + 0.12 * _noise(h, w, 90) + 0.04 * _noise(h, w, 3))[..., None]

    def lines(v, pitch, width):
        d = np.abs((v / pitch + 0.5) % 1.0 - 0.5) * pitch
        return d < width / 2

    minor = lines(X, 0.01, 0.0004) | lines(Y, 0.01, 0.0004)
    major = lines(X, 0.05, 0.0009) | lines(Y, 0.05, 0.0009)
    diag = lines((X + Y) / math.sqrt(2), 0.0707, 0.0004) & ~major
    line = np.array([0.80, 0.86, 0.72])
    rgb[minor] = rgb[minor] * 0.4 + line * 0.6 * 0.75
    rgb[diag] = rgb[diag] * 0.6 + line * 0.4 * 0.7
    rgb[major] = line * 0.85
    # knife scores: thin pale cuts
    for _ in range(40):
        a = RNG.uniform(0, math.pi)
        cx, cy = RNG.uniform(0, w), RNG.uniform(0, h)
        n = int(RNG.uniform(60, 500))
        t = np.linspace(-n / 2, n / 2, n)
        px = np.clip((cx + t * math.cos(a)).astype(int), 0, w - 1)
        py = np.clip((cy + t * math.sin(a)).astype(int), 0, h - 1)
        rgb[py, px] = rgb[py, px] * 0.75 + 0.25 * np.array([0.55, 0.65, 0.55])
    return _save("mat_top", rgb)


GLYPHS = {  # 3 x 5 block letters, rows from the top
    "P": ["111", "101", "111", "100", "100"], "E": ["111", "100", "110", "100", "111"],
    "L": ["100", "100", "100", "100", "111"], "R": ["110", "101", "110", "101", "101"],
    "O": ["111", "101", "101", "101", "111"], "T": ["111", "010", "010", "010", "010"],
    "C": ["111", "100", "100", "100", "111"], "I": ["111", "010", "010", "010", "111"],
    "V": ["101", "101", "101", "101", "010"], "F": ["111", "100", "110", "100", "100"],
    "M": ["101", "111", "111", "101", "101"], "A": ["010", "101", "111", "101", "101"],
    " ": ["000"] * 5,
}


def _text(mask, text, x, y, px, angle=0.0):
    """Stamp block letters into ``mask`` at pixel (x, y) (bottom-left), ``px`` pixels per block, turned ``angle``."""
    ca, sa = math.cos(angle), math.sin(angle)
    h, w = mask.shape
    for n, ch in enumerate(text):
        rows = GLYPHS.get(ch, GLYPHS[" "])
        for r, row in enumerate(rows):
            for c, bit in enumerate(row):
                if bit != "1":
                    continue
                u, v = (n * 4 + c) * px, (4 - r) * px
                for du in range(px):
                    for dv in range(px):
                        X = int(x + (u + du) * ca - (v + dv) * sa)
                        Y = int(y + (u + du) * sa + (v + dv) * ca)
                        if 0 <= X < w and 0 <= Y < h:
                            mask[Y, X] = True


def film_texture():
    """Brown kraft paper over the whole film (u along x from the near edge, v along y), printed with rows of slanted
    lettering, as acrylic masking paper is."""
    k = 8000
    w, h = int(STRIPS * STRIP_W * k), int(FILM_LEN * k)
    fibre = 0.9 + 0.08 * _noise(h, w, 40) + 0.06 * _noise(h, w, 4) + 0.04 * RNG.random((h, w))
    base = np.array([0.66, 0.47, 0.28])
    rgb = base[None, None, :] * fibre[..., None]
    ink = np.zeros((h, w), bool)
    a = math.radians(30)
    for row in range(-6, 14):
        for col in range(-2, 4):
            x = col * 520 + row * 90 + (row % 2) * 260
            y = row * 120
            _text(ink, "PROTECTIVE FILM", x, y, 4, a)
    rgb[ink] = rgb[ink] * 0.55 + np.array([0.20, 0.12, 0.06]) * 0.45
    return _save("film_top", rgb)


def tab_texture():
    """The tab's face: kraft paper with a printed arrow pointing up (pull here)."""
    k = 10000
    w, h = int(TAB_SIZE[1] * k), int(TAB_SIZE[2] * k)
    fibre = 0.9 + 0.08 * _noise(h, w, 30) + 0.05 * RNG.random((h, w))
    rgb = np.array([0.66, 0.47, 0.28])[None, None, :] * fibre[..., None]
    ys, xs = np.mgrid[0:h, 0:w]
    cx = w / 2
    shaft = (np.abs(xs - cx) < w * 0.07) & (ys > h * 0.15) & (ys < h * 0.55)
    head = (ys >= h * 0.55) & (ys < h * 0.85) & (np.abs(xs - cx) < (h * 0.85 - ys) * 0.9)
    rgb[shaft | head] = np.array([0.30, 0.17, 0.08])
    return _save("tab_face", rgb)


# -- geometry ---------------------------------------------------------------------------------------------------------


class Look:
    def __init__(self, name, origin, size=None):
        self.name, self.origin, self.size = name, Vector(origin), size
        self.coll = bpy.data.collections.new(PREFIX + "look_" + name)
        bpy.context.scene.collection.children.link(self.coll)
        self.root = bpy.data.objects.new(PREFIX + name, None)
        self.root.location = self.origin
        self.coll.objects.link(self.root)

    def mesh(self, name, bm, mats, bevel=0.0, smooth=False):
        bmesh.ops.translate(bm, vec=-self.origin, verts=bm.verts)
        me = bpy.data.meshes.new(PREFIX + self.name + "_" + name)
        bm.to_mesh(me)
        bm.free()
        if smooth:
            me.shade_smooth()
        ob = bpy.data.objects.new(PREFIX + self.name + "_" + name, me)
        self.coll.objects.link(ob)
        ob.parent = self.root
        for m in mats if isinstance(mats, (list, tuple)) else [mats]:
            me.materials.append(m)
        if bevel:
            mod = ob.modifiers.new("bevel", "BEVEL")
            mod.width, mod.segments, mod.limit_method = bevel, 2, "ANGLE"
        return ob


def bm_box(lo, hi, uv=None, top_only_mat=False):
    """A box from ``lo`` to ``hi``. ``uv`` = (x0, y0, x1, y1) maps x, y planar onto the faces. With
    ``top_only_mat``, the top face gets material slot 0 and the others slot 1."""
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
    if top_only_mat:
        bm.normal_update()
        for f in bm.faces:
            f.material_index = 0 if f.normal.z > 0.9 else 1
    return bm


def _lohi(c, size, shrink=0.0):
    return (tuple(c[i] - size[i] / 2 + shrink for i in range(3)), tuple(c[i] + size[i] / 2 - shrink for i in range(3)))


def build_mat():
    L = Look("mat", MAT_C)
    lo, hi = _lohi(MAT_C, MAT_SIZE)
    top = material("cutting_mat", (0.05, 0.25, 0.13), 0.8, image=mat_texture())
    edge = material("cutting_mat_core", (0.03, 0.03, 0.035), 0.8)
    L.mesh("mat", bm_box(lo, hi, uv=(lo[0], lo[1], hi[0], hi[1]), top_only_mat=True), [top, edge], bevel=0.001)
    return L


def build_sheet():
    L = Look("sheet", SHEET_C, SHEET_SIZE)
    lo, hi = _lohi(SHEET_C, SHEET_SIZE)
    opal = material("acrylic_opal", (0.86, 0.87, 0.86), 0.06)  # glossy opal white cast acrylic
    L.mesh("acrylic", bm_box(lo, hi), opal, bevel=0.0006)
    return L


def build_strip(i, film, paper_edge):
    c = strip_c(i)
    size = (STRIP_W, FILM_LEN, STRIP_T)
    L = Look(f"strip_{i}", c, size)
    lo, hi = _lohi(c, size)
    lo = (lo[0] + 0.0002, lo[1], lo[2])  # a hairline gap between strips shows the cuts between them
    hi = (hi[0] - 0.0002, hi[1], hi[2])
    uv = (FILM_X0, FILM_Y0, FILM_X0 + STRIPS * STRIP_W, FILM_Y0 + FILM_LEN)
    L.mesh("paper", bm_box(lo, hi, uv=uv, top_only_mat=True), [film, paper_edge])
    return L


def build_tab(paper_edge):
    L = Look("tab", TAB_C, TAB_SIZE)
    lo, hi = _lohi(TAB_C, TAB_SIZE)
    face = material("tab_face", (0.66, 0.47, 0.28), 0.85, image=tab_texture())
    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=1.0)
    c, s = Vector(TAB_C), Vector(hi) - Vector(lo)
    for v in bm.verts:
        v.co = c + Vector((v.co.x * s.x, v.co.y * s.y, v.co.z * s.z))
    layer = bm.loops.layers.uv.new("UVMap")
    bm.normal_update()
    for f in bm.faces:
        f.material_index = 0 if abs(f.normal.x) > 0.9 else 1
        for loop in f.loops:
            co = loop.vert.co
            loop[layer].uv = ((co.y - lo[1]) / s.y, (co.z - lo[2]) / s.z)
    L.mesh("paper", bm, [face, paper_edge], bevel=0.0004)
    return L


# -- export -----------------------------------------------------------------------------------------------------------


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
    """Remove everything this script made before (named with PREFIX), and nothing else."""
    for coll in [c for c in bpy.data.collections if c.name.startswith(PREFIX)]:
        for ob in list(coll.objects):
            bpy.data.objects.remove(ob)
        bpy.data.collections.remove(coll)
    for block in (bpy.data.objects, bpy.data.meshes, bpy.data.materials, bpy.data.images):
        for item in [i for i in block if i.name.startswith(PREFIX)]:
            block.remove(item)
    _MATS.clear()


def build():
    clear()
    film = material("film_print", (0.66, 0.47, 0.28), 0.9, image=film_texture())
    paper_edge = material("paper_edge", (0.50, 0.34, 0.19), 0.9)
    looks = [build_mat(), build_sheet(), *(build_strip(i, film, paper_edge) for i in range(1, STRIPS + 1)),
             build_tab(paper_edge)]
    return {look.name: export(look) for look in looks}


if __name__ == "__main__":
    print(build())
