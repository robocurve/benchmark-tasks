"""make_looks.py: model the scene's visual meshes in Blender and export them as USD next to this file.

    blender -b -P looks/make_looks.py          (Blender 4.2 or later, from the task folder)

The physics stays the task file's boxes. Each look is a render-only mesh that ``looks.py`` puts on one of those boxes,
whose plain cuboid is then hidden but still sets its bounds. Every look is built in its box's own frame, around the
box's centre (metres, z up), and checked to stay inside the box when it is exported. (The clamp is drawn as its own
collision boxes, coloured, by ``bonds.py``.)

- ``board_right.usdc``, ``board_left.usdc``: pine boards, 20 x 6 x 1.8 cm, with their grain and end grain.
- ``mat.usdc``: the dark grey rubber work mat with a faint 1 cm grid.

Textures are drawn here with numpy and saved in ``textures/``. Nothing is downloaded. Everything this script makes in
Blender is named with the ``bar-clamp-boards__`` prefix, and only such data is removed.
"""

import math
import os

import bmesh
import bpy
import numpy as np
from mathutils import Matrix, Vector

HERE = os.path.dirname(os.path.abspath(__file__))
TEX = os.path.join(HERE, "textures")
P = "bar-clamp-boards__"

# -- the task file's sizes (bar_clamp_boards.py), metres ---------------------------------------------------------------
BOARD = (0.20, 0.060, 0.018)
MAT = (0.40, 0.46, 0.003)


def linear(c):
    return tuple(v / 12.92 if v <= 0.04045 else ((v + 0.055) / 1.055) ** 2.4 for v in c)


# -- materials (sRGB colours in, linear in Blender) -------------------------------------------------------------------
_MATS = {}


def material(name, color=(0.5, 0.5, 0.5), rough=0.5, metal=0.0, image=None):
    key = P + name
    if key in _MATS:
        return _MATS[key]
    m = bpy.data.materials.get(key) or bpy.data.materials.new(key)
    m.use_nodes = True
    nodes, links = m.node_tree.nodes, m.node_tree.links
    nodes.clear()
    out = nodes.new("ShaderNodeOutputMaterial")
    bsdf = nodes.new("ShaderNodeBsdfPrincipled")
    links.new(bsdf.outputs["BSDF"], out.inputs["Surface"])
    lin = linear(color)
    bsdf.inputs["Base Color"].default_value = (*lin, 1.0)
    bsdf.inputs["Roughness"].default_value = rough
    bsdf.inputs["Metallic"].default_value = metal
    if image is not None:
        tex = nodes.new("ShaderNodeTexImage")
        tex.image = image
        tex.extension = "REPEAT"
        links.new(tex.outputs["Color"], bsdf.inputs["Base Color"])
    m.diffuse_color = (*lin, 1.0)
    _MATS[key] = m
    return m


# -- textures ----------------------------------------------------------------------------------------------------------


def _smooth_noise(rng, h, w, ch, cw):
    gh, gw = h // ch + 2, w // cw + 2
    g = rng.random((gh, gw))
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
    img = bpy.data.images.new(P + name, w, h, alpha=False)
    rgba = np.concatenate([np.clip(rgb, 0, 1), np.ones((h, w, 1))], axis=2).astype(np.float32)
    img.pixels.foreach_set(rgba.ravel())
    img.filepath_raw = path
    img.file_format = "JPEG"
    img.save()
    bpy.data.images.remove(img)
    loaded = bpy.data.images.load(path)
    loaded.name = P + name
    return loaded


def wood_texture(name, seed, light, dark, w=1024, h=512, rings=34.0):
    """Flat-sawn grain along the image's width: wavy rings, a few knots-free streaks, fine pores."""
    rng = np.random.default_rng(seed)
    y = np.linspace(0, 1, h)[:, None]
    x = np.linspace(0, 1, w)[None, :]
    warp = 0.06 * _smooth_noise(rng, h, w, h // 3, w // 4) + 0.015 * _smooth_noise(rng, h, w, h // 12, w // 16)
    ring = np.sin(2 * math.pi * (rings * (y + warp) + 0.8 * x * _smooth_noise(rng, h, w, h, w // 2)))
    late = np.clip((ring - 0.55) / 0.45, 0, 1) ** 1.5  # the dark late wood of each ring
    streak = 0.08 * (_smooth_noise(rng, h, w, 3, w // 2) - 0.5)
    pores = 0.05 * (rng.random((h, w)) - 0.5)
    t = np.clip(0.75 * late + streak + pores, 0, 1)[..., None]
    lt, dk = np.asarray(light)[None, None, :], np.asarray(dark)[None, None, :]
    return _save(name, lt * (1 - t) + dk * t)


def mat_texture():
    """Dark grey rubber with faint lighter lines every centimetre and stronger ones every 5 cm."""
    rng = np.random.default_rng(3)
    k = 1000  # pixels per metre
    w, h = int(MAT[0] * k), int(MAT[1] * k)
    base = np.full((h, w, 3), 0.15) + 0.015 * (rng.random((h, w, 1)) - 0.5)
    yy, xx = np.mgrid[0:h, 0:w]
    fine = ((xx % 10) == 0) | ((yy % 10) == 0)
    bold = ((xx % 50) == 0) | ((yy % 50) == 0)
    base[fine] += 0.035
    base[bold] += 0.05
    return _save("mat", base)


def materials():
    return {
        "pine_a": material("pine_a", (0.86, 0.70, 0.46), 0.7,
                           image=wood_texture("pine_a", 11, (0.90, 0.75, 0.52), (0.72, 0.50, 0.28))),
        "pine_b": material("pine_b", (0.86, 0.70, 0.46), 0.7,
                           image=wood_texture("pine_b", 23, (0.89, 0.74, 0.50), (0.70, 0.48, 0.26))),
        "mat": material("mat", (0.15, 0.15, 0.15), 0.9, image=mat_texture()),
    }


# -- geometry ----------------------------------------------------------------------------------------------------------


class Look:
    """A look's meshes, drawn in some frame and exported around ``origin`` (the box's centre in that frame)."""

    def __init__(self, name, origin, size):
        self.name, self.origin, self.size = name, Vector(origin), size
        self.coll = bpy.data.collections.new(P + "look_" + name)
        bpy.context.scene.collection.children.link(self.coll)
        self.root = bpy.data.objects.new(P + name, None)
        self.root.location = self.origin
        self.coll.objects.link(self.root)

    def mesh(self, name, bm, mat, bevel=0.0, smooth=False):
        bmesh.ops.translate(bm, vec=-self.origin, verts=bm.verts)
        me = bpy.data.meshes.new(P + name)
        bm.to_mesh(me)
        bm.free()
        if smooth:
            me.shade_smooth()
        ob = bpy.data.objects.new(P + name, me)
        self.coll.objects.link(ob)
        ob.parent = self.root
        me.materials.append(mat)
        if bevel:
            m = ob.modifiers.new("bevel", "BEVEL")
            m.width, m.segments, m.limit_method = bevel, 2, "ANGLE"
        return ob


def bm_box(lo, hi, uv_scale=None):
    """A box from ``lo`` to ``hi``. With ``uv_scale`` (metres per texture width), each face is mapped flat onto the
    plane it faces, the texture's width along the box's longest side."""
    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=1.0)
    c, s = (Vector(lo) + Vector(hi)) / 2, Vector(hi) - Vector(lo)
    for v in bm.verts:
        v.co = c + Vector((v.co.x * s.x, v.co.y * s.y, v.co.z * s.z))
    if uv_scale:
        layer = bm.loops.layers.uv.new("UVMap")
        long_axis = max(range(3), key=lambda i: s[i])
        for f in bm.faces:
            n = [abs(v) for v in f.normal]
            normal = n.index(max(n))
            axes = [i for i in range(3) if i != normal]
            u_axis = long_axis if long_axis in axes else axes[0]
            v_axis = [i for i in axes if i != u_axis][0]
            for loop in f.loops:
                co = loop.vert.co
                loop[layer].uv = (co[u_axis] / uv_scale, co[v_axis] / uv_scale * 2.0)
    return bm


# -- the looks ---------------------------------------------------------------------------------------------------------


def build_board(M, name, mat):
    L = Look(name, (0.0, 0.0, 0.0), BOARD)
    h = [s / 2 for s in BOARD]
    L.mesh("board_" + name, bm_box([-v for v in h], h, uv_scale=0.21), mat, bevel=0.0012)
    return L


def build_mat(M):
    L = Look("mat", (0.0, 0.0, 0.0), MAT)
    h = [s / 2 for s in MAT]
    bm = bm_box([-v for v in h], h)
    layer = bm.loops.layers.uv.new("UVMap")
    for f in bm.faces:
        for loop in f.loops:
            co = loop.vert.co
            loop[layer].uv = (co.x / MAT[0] + 0.5, co.y / MAT[1] + 0.5)
    L.mesh("mat", bm, M["mat"], bevel=0.001)
    return L


# -- export ------------------------------------------------------------------------------------------------------------


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
    """Remove everything this script made before (names with the prefix), nothing else."""
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


def build(keep=False):
    """Build and export every look. ``keep`` leaves them in the Blender scene to look at, otherwise they are removed."""
    clear()
    M = materials()
    looks = [build_board(M, "board_right", M["pine_a"]), build_board(M, "board_left", M["pine_b"]), build_mat(M)]
    out = {look.name: export(look) for look in looks}
    if not keep:
        clear()
    return out


if __name__ == "__main__":
    print(build())
