"""make_looks.py: model the workpiece and the sanding block in Blender and export them as USD next to this file.

    blender -b -P looks/make_looks.py          (Blender 4.2 or later, from the task folder)

The physics stays the task file's boxes. Each look is a render-only mesh that ``looks.py`` puts on one of those
boxes, whose plain cuboid is then hidden but still collides. Each look is built around the origin, in its box's own
frame (x across, y along, z up), and checked to fit inside its box when exported.

- ``workpiece.usdc``: 3 mm birch plywood, 3 x 200 x 70 mm: pale grained faces, its edges showing the three plies,
  a pencil line along one face 5 mm under the top edge.
- ``block.usdc``: the sanding block, 70 x 100 x 30 mm: orange foam with rounded top edges, grey 80-grit sandpaper on its underside.

Textures are drawn here with numpy and saved as JPEG in ``textures/``. Nothing is downloaded. Everything this script
makes in Blender is named with the ``sand-edge-five-passes__`` prefix, and only that is removed when it runs again,
so it can run in a Blender session that holds other work.
"""

import math
import os

import bmesh
import bpy
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
TEX = os.path.join(HERE, "textures")
PREFIX = "sand-edge-five-passes__"
RNG = np.random.default_rng(17)

PIECE_SIZE = (0.003, 0.20, 0.07)
BLOCK_SIZE = (0.07, 0.10, 0.03)
INSET = 0.0001  # m: each look sits this far inside its box


def n(name):
    return PREFIX + name


# -- clean up what an earlier run made --------------------------------------------------------------------------------


def clear():
    for coll in [c for c in bpy.data.collections if c.name.startswith(PREFIX)]:
        for ob in list(coll.objects):
            bpy.data.objects.remove(ob, do_unlink=True)
        bpy.data.collections.remove(coll)
    for blocks in (bpy.data.objects, bpy.data.meshes, bpy.data.materials, bpy.data.images):
        for b in [b for b in blocks if b.name.startswith(PREFIX)]:
            blocks.remove(b)


# -- textures --------------------------------------------------------------------------------------------------------


def _smooth_noise(h, w, cy, cx):
    """Value noise, cells of cy x cx pixels, bilinear."""
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


def birch_face():
    """Rotary-cut birch veneer, the grain running along the piece (the image's width): pale, soft wavy lines."""
    h, w = 256, 1024
    y = np.arange(h)[:, None] / h
    x = np.arange(w)[None, :] / w
    wave = 0.04 * np.sin(2 * math.pi * (x * 1.5 + 0.2)) + 0.03 * _smooth_noise(h, w, 64, 256)
    grain = np.sin(2 * math.pi * (y + wave) * 38.0) * 0.5 + 0.5
    grain = grain ** 6 * 0.5 + _smooth_noise(h, w, 4, 64) * 0.5
    base = np.array([0.90, 0.80, 0.62])
    dark = np.array([0.72, 0.56, 0.36])
    t = (0.55 * grain + 0.25 * _smooth_noise(h, w, 32, 128))[..., None]
    rgb = base * (1 - t) + dark * t
    # the pencil line along the face, 5 mm under the top edge (the image's top is the edge)
    line = (np.abs(y - (1 - 0.005 / PIECE_SIZE[2])) < 0.4 / 70 / 2) & (_smooth_noise(h, w, 2, 6) > 0.25)
    rgb[np.broadcast_to(line, (h, w))] = (0.30, 0.30, 0.32)
    return _save("birch_face", rgb)


def ply_edge():
    """The plywood's edge: three plies across its thickness (the image's height), the middle one end grain."""
    h, w = 48, 1024
    y = np.arange(h)[:, None] / h
    rgb = np.empty((h, w, 3))
    light, mid = np.array([0.88, 0.76, 0.56]), np.array([0.70, 0.54, 0.34])
    ply = (y > 0.36) & (y < 0.64)
    rgb[:] = light
    rgb[np.broadcast_to(ply, (h, w))] = mid
    glue = (np.abs(y - 0.36) < 0.03) | (np.abs(y - 0.64) < 0.03)
    rgb[np.broadcast_to(glue, (h, w))] = (0.45, 0.34, 0.22)
    rgb *= (0.88 + 0.24 * _smooth_noise(h, w, 2, 3))[..., None]
    return _save("ply_edge", rgb)


def foam():
    h, w = 256, 256
    rgb = np.array([0.98, 0.45, 0.06]) * (0.90 + 0.10 * _smooth_noise(h, w, 2, 2))[..., None]
    return _save("foam", rgb)


def sandpaper():
    h, w = 256, 256
    g = RNG.random((h, w))
    v = 0.30 + 0.10 * _smooth_noise(h, w, 2, 2) + 0.25 * (g > 0.93) - 0.12 * (g < 0.05)
    return _save("sandpaper", np.repeat(v[..., None], 3, axis=2) * np.array([1.0, 0.98, 0.95]))


# -- materials -------------------------------------------------------------------------------------------------------


def material(name, color, rough=0.6, image=None):
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
    if image is not None:
        tex = nodes.new("ShaderNodeTexImage")
        tex.image = image
        tex.extension = "REPEAT"
        links.new(tex.outputs["Color"], bsdf.inputs["Base Color"])
    m.diffuse_color = (*color, 1.0)
    return m


# -- meshes ----------------------------------------------------------------------------------------------------------


class Look:
    """A collection holding one look's meshes, built around the origin."""

    def __init__(self, name, size):
        self.name, self.size = name, size
        self.coll = bpy.data.collections.new(n("look_" + name))
        bpy.context.scene.collection.children.link(self.coll)

    def mesh(self, name, bm, mats):
        me = bpy.data.meshes.new(n(name))
        bm.to_mesh(me)
        bm.free()
        for m in mats:
            me.materials.append(m)
        ob = bpy.data.objects.new(n(name), me)
        self.coll.objects.link(ob)
        return ob


def _uv_box(bm, lo, hi, faces_to_mat):
    """Planar UVs per face, from the face's normal: x faces map (y, z), y faces (x, z), z faces (y, x), each over
    the box's extent. ``faces_to_mat(normal)`` picks the face's material index."""
    uv = bm.loops.layers.uv.new("UVMap")
    lo, hi = np.asarray(lo), np.asarray(hi)
    span = np.maximum(hi - lo, 1e-9)
    for f in bm.faces:
        nrm = np.asarray(f.normal)
        axis = int(np.argmax(np.abs(nrm)))
        u_ax, v_ax = {0: (1, 2), 1: (0, 2), 2: (1, 0)}[axis]
        f.material_index = faces_to_mat(nrm)
        for loop in f.loops:
            p = np.asarray(loop.vert.co)
            loop[uv].uv = ((p[u_ax] - lo[u_ax]) / span[u_ax], (p[v_ax] - lo[v_ax]) / span[v_ax])


def _box(lo, hi):
    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=1.0)
    for v in bm.verts:
        v.co = [lo[i] + (v.co[i] + 0.5) * (hi[i] - lo[i]) for i in range(3)]
    bm.normal_update()
    return bm


def build_workpiece(M):
    look = Look("workpiece", PIECE_SIZE)
    half = np.asarray(PIECE_SIZE) / 2 - INSET
    bm = _box(-half, half)
    # faces (normal along x) get the veneer, the edges the plies
    _uv_box(bm, -half, half, lambda nrm: 0 if abs(nrm[0]) > 0.5 else 1)
    # the plies run across the thickness: on the top and bottom edges, v must be x; on the ends, too
    uv = bm.loops.layers.uv.active
    for f in bm.faces:
        nrm = np.asarray(f.normal)
        if abs(nrm[0]) > 0.5:
            continue
        for loop in f.loops:
            p = np.asarray(loop.vert.co)
            along = p[1] if abs(nrm[2]) > 0.5 else p[2]
            span = 2 * half[1] if abs(nrm[2]) > 0.5 else 2 * half[2]
            loop[uv].uv = ((along + span / 2) / span, (p[0] + half[0]) / (2 * half[0]))
    look.mesh("workpiece", bm, [M["birch"], M["ply"]])
    return look


def build_block(M):
    look = Look("block", BLOCK_SIZE)
    hx, hy, hz = np.asarray(BLOCK_SIZE) / 2 - INSET
    paper_t = 0.0008
    # the foam body, its top edges rounded
    bm = _box((-hx + 0.0005, -hy + paper_t, -hz + paper_t), (hx - 0.0005, hy - paper_t, hz))
    edges = [e for e in bm.edges if all(v.co[2] > hz - 1e-6 for v in e.verts)]
    bmesh.ops.bevel(bm, geom=edges, offset=0.005, segments=4, affect="EDGES", profile=0.5)
    bm.normal_update()
    _uv_box(bm, (-hx, -hy, -hz), (hx, hy, hz), lambda nrm: 0)
    look.mesh("block_foam", bm, [M["foam"]])
    # the sandpaper sheet under it, wrapped 3 mm up each end
    paper = _box((-hx, -hy, -hz), (hx, hy, -hz + paper_t))
    _uv_box(paper, (-hx, -hy, -hz), (hx, hy, hz), lambda nrm: 0)
    look.mesh("block_paper", paper, [M["paper"]])
    for s in (-1, 1):
        wrap = _box((-hx, s * hy - (paper_t if s > 0 else 0), -hz), (hx, s * hy + (0 if s > 0 else paper_t), -hz + 0.004))
        _uv_box(wrap, (-hx, -hy, -hz), (hx, hy, hz), lambda nrm: 0)
        look.mesh(f"block_wrap_{'left' if s > 0 else 'right'}", wrap, [M["paper"]])
    return look


# -- export ----------------------------------------------------------------------------------------------------------


def _bounds(look):
    pts = []
    for ob in look.coll.objects:
        if ob.type == "MESH":
            pts += [tuple(ob.matrix_world @ v.co) for v in ob.data.vertices]
    p = np.asarray(pts)
    return p.min(0), p.max(0)


def export(look):
    bpy.context.view_layer.update()
    lo, hi = _bounds(look)
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
        "birch": material("birch", (0.78, 0.62, 0.40), 0.7, birch_face()),
        "ply": material("ply", (0.72, 0.58, 0.38), 0.8, ply_edge()),
        "foam": material("foam", (0.95, 0.32, 0.01), 0.85, foam()),
        "paper": material("sandpaper", (0.09, 0.09, 0.09), 0.95, sandpaper()),
    }
    looks = [build_workpiece(M), build_block(M)]
    out = {look.name: export(look) for look in looks}
    return out


if __name__ == "__main__":
    for name, info in build().items():
        print(name, info)
