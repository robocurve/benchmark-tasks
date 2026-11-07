"""make_looks.py: model the scene's visual meshes in Blender and export them as USD next to this file.

    blender -b -P looks/make_looks.py          (Blender 4.2 or later, from the task folder)

Or, in a running Blender: ``exec(compile(open(p).read(), p, "exec"), {"__file__": p, "__name__": "make_looks"})``
and then ``build()``.

The physics stays the task file's boxes. Each look is a render-only mesh that ``looks.py`` puts on one of those boxes,
whose plain cuboid is then hidden but still sets the box's bounds. Every look is built around its box's centre, in the
box's own frame (metres, z up), and exported with that centre at the origin.

- ``m8.usdc``: an ISO 4017 M8 x 30 hex bolt, bright zinc plated, lying on a flat of its head along x (head toward -x),
  inside the bolt's 35.3 x 14.5 x 13 mm box (checked when exported). Rolled thread, 1.25 mm pitch.
- ``m4.usdc``: an M4 x 20 hex bolt, black oxide, inside its 22.8 x 7.7 x 7 mm box (checked). 0.7 mm pitch.
- ``organizer.usdc``: a blue two-compartment parts organizer with light grey floor liners, on its 3 mm floor box.
- ``mat.usdc``: a green self-healing cutting mat with a 1 cm grid, on its 2 mm box.

Everything this script makes in Blender is named with the prefix ``sort-m4-m8-bolts__`` and only those datablocks are
removed when it runs again. Every mesh is triangulated on export. Textures are drawn here with numpy and saved in
``textures/``. Nothing is downloaded.
"""

import math
import os

import bmesh
import bpy
import numpy as np
from mathutils import Matrix, Vector

HERE = os.path.dirname(os.path.abspath(__file__))
TEX = os.path.join(HERE, "textures")
P = "sort-m4-m8-bolts__"  # namespace of every datablock this script makes
RNG = np.random.default_rng(18)

# -- the task file's sizes (sort_m4_m8_bolts.py), metres ---------------------------------------------------------------
BOLTS = {
    # name: thread d, pitch, head height k, across flats s, across corners e, shank length l
    "m8": dict(d=0.008, pitch=0.00125, k=0.0053, s=0.013, e=0.0144, l=0.030),
    "m4": dict(d=0.004, pitch=0.0007, k=0.0028, s=0.007, e=0.0076, l=0.020),
}
ORG_OUT = (0.098, 0.192)  # organizer outside, x by y
ORG_WALL, ORG_FLOOR, ORG_H, ORG_CELL = 0.004, 0.003, 0.030, 0.090
MAT_SIZE = (0.22, 0.30, 0.002)


def bolt_box(b):
    """The bolt's box: length along x, across corners along y, across flats along z."""
    return (b["k"] + b["l"], b["e"] + 0.0001, b["s"])


# -- materials (linear colours) --------------------------------------------------------------------------------------
_MATS = {}


def material(name, color=(0.5, 0.5, 0.5), rough=0.5, metal=0.0, image=None):
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
    if image is not None:
        tex = nodes.new("ShaderNodeTexImage")
        tex.image = image
        tex.extension = "REPEAT"
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


def mat_texture():
    """A green cutting mat seen from above (u along x, v along y): faint 1 cm grid, 5 cm lines a little stronger,
    a ruler scale along two edges."""
    sx, sy = MAT_SIZE[0], MAT_SIZE[1]
    k = 4000  # pixels per metre
    W, H = int(sx * k), int(sy * k)  # image x is the mat's x, image rows its y
    shade = 0.95 + 0.07 * _noise(H, W, 120) + 0.04 * _noise(H, W, 2)
    rgb = np.array([0.10, 0.40, 0.27])[None, None, :] * shade[..., None]
    y, x = np.mgrid[:H, :W]
    grid, major, tick = np.zeros((H, W), bool), np.zeros((H, W), bool), np.zeros((H, W), bool)
    edge = 0.012 * k
    inner = (x > edge) & (x < W - edge) & (y > edge) & (y < H - edge)
    cm = 0.01 * k
    for i in range(int(sx / 0.01) + 1):
        c = int(round(edge + i * cm))
        if c < W - edge + 1:
            (major if i % 5 == 0 else grid)[:, max(c - 1, 0):c + 1] = True
    for j in range(int(sy / 0.01) + 1):
        c = int(round(edge + j * cm))
        if c < H - edge + 1:
            (major if j % 5 == 0 else grid)[max(c - 1, 0):c + 1, :] = True
    grid &= inner
    major &= inner
    for i in range(int((sx - 2 * 0.012) / 0.002) + 1):  # 2 mm ticks along the near edge
        c = int(round(edge + i * 0.002 * k))
        length = 0.006 if i % 5 == 0 else 0.003
        tick[: int(edge - 0.002 * k + length * k * 0.5), c:c + 2] = (y[: int(edge - 0.002 * k + length * k * 0.5), c:c + 2]
                                                                    > edge - length * k)
    rgb[grid] = rgb[grid] * 0.8 + np.array([0.55, 0.72, 0.62]) * 0.2 * 0.8
    rgb[major] = rgb[major] * 0.6 + np.array([0.55, 0.72, 0.62]) * 0.4
    rgb[tick] = np.array([0.80, 0.86, 0.82])
    return _save("mat_top", rgb)


# -- geometry --------------------------------------------------------------------------------------------------------


class Look:
    def __init__(self, name, size=None):
        self.name, self.size = name, size
        self.coll = bpy.data.collections.new(P + "look_" + name)
        bpy.context.scene.collection.children.link(self.coll)
        self.root = bpy.data.objects.new(P + name, None)
        self.coll.objects.link(self.root)
        self.cutters = bpy.data.collections.new(P + "cutters_" + name)
        bpy.context.scene.collection.children.link(self.cutters)
        self.cutters.hide_render = True

    def mesh(self, name, bm, mat, bevel=0.0, smooth=False, to=None, segments=2):
        me = bpy.data.meshes.new(P + name)
        bm.to_mesh(me)
        bm.free()
        if smooth:
            me.shade_smooth()
        ob = bpy.data.objects.new(P + name, me)
        (to or self.coll).objects.link(ob)
        ob.parent = self.root
        me.materials.append(mat)
        if bevel:
            m = ob.modifiers.new("bevel", "BEVEL")
            m.width, m.segments, m.limit_method = bevel, segments, "ANGLE"
        return ob

    def cut(self, ob, bms):
        group = bpy.data.collections.new(P + f"cut_{ob.name[len(P):]}")
        self.cutters.children.link(group)
        for i, bm in enumerate(bms):
            c = self.mesh(f"{ob.name[len(P):]}_cutter{i}", bm, material("cutter"), to=group)
            c.display_type, c.hide_render = "WIRE", True
        m = ob.modifiers.new("cut", "BOOLEAN")
        m.operation, m.operand_type, m.collection, m.solver = "DIFFERENCE", "COLLECTION", group, "EXACT"
        ob.modifiers.move(len(ob.modifiers) - 1, 0)  # cut before bevelling


def bm_box(lo, hi):
    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=1.0)
    c, s = (Vector(lo) + Vector(hi)) / 2, Vector(hi) - Vector(lo)
    for v in bm.verts:
        v.co = c + Vector((v.co.x * s.x, v.co.y * s.y, v.co.z * s.z))
    return bm


def bm_prism(pts, a0, a1, axis="Z"):
    """A polygon extruded along an axis. ``pts`` are (u, v) in the plane across it: (y, z) for X, (x, y) for Z."""
    def co(u, v, a):
        return {"X": (a, u, v), "Z": (u, v, a)}[axis]
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


def rounded_rect(x0, y0, x1, y1, r, n=5):
    pts = []
    for cx, cy, a0 in ((x1 - r, y0 + r, -90), (x1 - r, y1 - r, 0), (x0 + r, y1 - r, 90), (x0 + r, y0 + r, 180)):
        for i in range(n + 1):
            a = math.radians(a0 + 90 * i / n)
            pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
    return pts


def bm_thread(x0, x1, d, pitch, segs=40, per_pitch=8):
    """A rolled metric thread along +x from x0 to x1: a 60 degree profile (depth 0.54 pitch) swept along a helix,
    the crests and roots flattened, a 45 degree chamfer at the tip and a flat end."""
    r_maj = d / 2
    depth = 0.54 * pitch
    r_min = r_maj - depth
    n_u = max(int((x1 - x0) / pitch * per_pitch), 8)
    us = np.linspace(x0, x1, n_u + 1)
    th = np.linspace(0, 2 * math.pi, segs, endpoint=False)
    bm = bmesh.new()
    rows = []
    for u in us:
        row = []
        for t in th:
            f = ((u - x0) / pitch - t / (2 * math.pi)) % 1.0
            tri = 1.0 - abs(2.0 * f - 1.0)  # 0 at the root, 1 at the crest
            r = r_min + depth * min(1.0, max(0.0, (tri - 0.08) / 0.8))
            r = min(r, r_min * 0.92 + (x1 - u))  # chamfered tip
            row.append(bm.verts.new((u, r * math.cos(t), r * math.sin(t))))
        rows.append(row)
    for a, b in zip(rows, rows[1:]):
        for i in range(segs):
            j = (i + 1) % segs
            bm.faces.new((a[i], a[j], b[j], b[i]))
    bm.faces.new(list(reversed(rows[0])))
    bm.faces.new(rows[-1])
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    return bm


def bm_cyl_x(x0, x1, r, segs=40):
    bm = bmesh.new()
    bmesh.ops.create_cone(bm, cap_ends=True, segments=segs, radius1=r, radius2=r, depth=x1 - x0,
                          matrix=Matrix.Translation(((x0 + x1) / 2, 0, 0)) @ Matrix.Rotation(math.pi / 2, 4, "Y"))
    return bm


def build_bolt(name, b, mats):
    """An ISO 4017 hex bolt along +x, centred in its box: the head from -L/2, a flat of the hex facing down."""
    head_mat, shank_mat = mats
    L = b["k"] + b["l"]
    x_head, x_neck = -L / 2, -L / 2 + b["k"]
    look = Look(name, bolt_box(b))
    e2, s2 = b["e"] / 2, b["s"] / 2
    hexagon = [(e2 * math.cos(math.radians(a)), e2 * math.sin(math.radians(a))) for a in range(0, 360, 60)]
    hexagon = [(u, max(-s2, min(s2, v))) for u, v in hexagon]
    head = look.mesh(f"{name}_head", bm_prism(hexagon, x_head, x_neck - 0.04 * b["k"], "X"), head_mat,
                     bevel=0.06 * b["k"])
    # the 30 degree chamfer of the head's top: everything outside a cone round the axis is cut off the corners
    depth = 0.01 * math.tan(math.radians(30))
    cone = bmesh.new()
    bmesh.ops.create_cone(cone, cap_ends=True, segments=48, radius1=b["s"] * 0.49, radius2=b["s"] * 0.49 + 0.01,
                          depth=depth, matrix=Matrix.Translation((x_head + depth / 2, 0, 0))
                          @ Matrix.Rotation(math.pi / 2, 4, "Y"))
    ring = bm_cyl_x(x_head - 0.001, x_head + depth - 0.0001, b["e"])
    look.cut(head, [_ring_minus(ring, cone)])
    look.mesh(f"{name}_washer_face", bm_cyl_x(x_neck - 0.05 * b["k"], x_neck, b["s"] * 0.46, 48), head_mat)
    look.mesh(f"{name}_thread", bm_thread(x_neck, L / 2, b["d"], b["pitch"]), shank_mat, smooth=True)
    return look


def _ring_minus(ring, cone):
    """The space round the chamfer cone: a cylinder with the cone's inside carved out, as one cutter mesh."""
    ring_me, cone_me = bpy.data.meshes.new(P + "tmp_ring"), bpy.data.meshes.new(P + "tmp_cone")
    ring.to_mesh(ring_me)
    cone.to_mesh(cone_me)
    ring.free()
    cone.free()
    a = bpy.data.objects.new(P + "tmp_ring", ring_me)
    c = bpy.data.objects.new(P + "tmp_cone", cone_me)
    bpy.context.scene.collection.objects.link(a)
    bpy.context.scene.collection.objects.link(c)
    m = a.modifiers.new("cut", "BOOLEAN")
    m.operation, m.object, m.solver = "DIFFERENCE", c, "EXACT"
    bpy.context.view_layer.update()
    me = bpy.data.meshes.new_from_object(a.evaluated_get(bpy.context.evaluated_depsgraph_get()))
    bm = bmesh.new()
    bm.from_mesh(me)
    for ob in (a, c):
        bpy.data.objects.remove(ob)
    for mesh in (ring_me, cone_me, me):
        bpy.data.meshes.remove(mesh)
    return bm


def build_organizer(M):
    """Blue polypropylene organizer: rounded outside, two square compartments side by side along y, a 3 mm floor.
    The look's origin is the centre of the floor box (3 mm thick), so the floor's bottom is at z = -1.5 mm."""
    ox, oy = ORG_OUT[0] / 2, ORG_OUT[1] / 2
    z0, z1 = -ORG_FLOOR / 2, -ORG_FLOOR / 2 + ORG_H
    look = Look("organizer")
    body = look.mesh("organizer_body", bm_prism(rounded_rect(-ox, -oy, ox, oy, 0.006), z0, z1), M["blue"],
                     bevel=0.0008)
    cells = []
    for side in (-1, 1):
        cy = side * (ORG_WALL / 2 + ORG_CELL / 2)
        h = ORG_CELL / 2
        cells.append(bm_prism(rounded_rect(-h, cy - h, h, cy + h, 0.003), z0 + ORG_FLOOR, z1 + 0.01))
    look.cut(body, cells)
    # light grey liners on the compartment floors, so the two compartments read apart from above
    for side in (-1, 1):
        cy = side * (ORG_WALL / 2 + ORG_CELL / 2)
        h = ORG_CELL / 2 - 0.0004
        look.mesh(f"organizer_liner_{'left' if side > 0 else 'right'}",
                  bm_prism(rounded_rect(-h, cy - h, h, cy + h, 0.0026), z0 + ORG_FLOOR, z0 + ORG_FLOOR + 0.0004),
                  M["liner"])
    # a lip round the top: a slightly darker rim, so the compartments read from above
    rim = look.mesh("organizer_rim", bm_prism(rounded_rect(-ox - 0.0006, -oy - 0.0006, ox + 0.0006, oy + 0.0006,
                                                             0.0066), z1 - 0.003, z1 - 0.0005), M["blue_dark"])
    look.cut(rim, [bm_prism(rounded_rect(-ox + 0.0015, -oy + 0.0015, ox - 0.0015, oy - 0.0015, 0.005),
                            z1 - 0.01, z1 + 0.01)])
    return look


def build_mat(M):
    sx, sy, sz = MAT_SIZE
    look = Look("mat", MAT_SIZE)
    bm = bm_box((-sx / 2, -sy / 2, -sz / 2), (sx / 2, sy / 2, sz / 2))
    layer = bm.loops.layers.uv.new("UVMap")
    for f in bm.faces:
        for loop in f.loops:
            co = loop.vert.co
            loop[layer].uv = ((co.x + sx / 2) / sx, (co.y + sy / 2) / sy)
    look.mesh("mat_sheet", bm, M["mat"], bevel=0.0004)
    return look


def materials():
    M = {}
    M["zinc"] = material("zinc_bright", (0.72, 0.74, 0.78), 0.28, 0.85)
    M["zinc_head"] = material("zinc_head", (0.66, 0.68, 0.72), 0.34, 0.85)
    M["oxide"] = material("black_oxide", (0.018, 0.018, 0.020), 0.42, 0.55)
    M["oxide_head"] = material("black_oxide_head", (0.022, 0.022, 0.024), 0.48, 0.5)
    M["blue"] = material("organizer_blue", (0.02, 0.12, 0.55), 0.45)
    M["blue_dark"] = material("organizer_rim", (0.012, 0.07, 0.36), 0.45)
    M["liner"] = material("organizer_liner", (0.50, 0.52, 0.55), 0.6)
    M["mat"] = material("cutting_mat", (0.06, 0.30, 0.20), 0.7, image=mat_texture())
    return M


# -- export ----------------------------------------------------------------------------------------------------------


def _local_bounds(look):
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
    # Triangulate last: the boolean cuts leave concave n-gons (the organizer's top round its compartments), which
    # Isaac Sim's renderer fans into triangles that cover the holes
    for ob in look.coll.objects:
        if ob.type == "MESH" and "triangulate" not in ob.modifiers:
            ob.modifiers.new("triangulate", "TRIANGULATE")
    bpy.context.view_layer.update()
    lo, hi = _local_bounds(look)
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
    """Remove what an earlier run of this script made (only datablocks named with the prefix)."""
    for coll in [c for c in bpy.data.collections if c.name.startswith(P)]:
        for ob in list(coll.objects):
            bpy.data.objects.remove(ob)
    for coll in [c for c in bpy.data.collections if c.name.startswith(P)]:
        bpy.data.collections.remove(coll)
    for block in (bpy.data.objects, bpy.data.meshes, bpy.data.curves, bpy.data.materials, bpy.data.images):
        for item in [i for i in block if i.name.startswith(P)]:
            block.remove(item)
    _MATS.clear()


def build():
    clear()
    M = materials()
    looks = [
        build_bolt("m8", BOLTS["m8"], (M["zinc_head"], M["zinc"])),
        build_bolt("m4", BOLTS["m4"], (M["oxide_head"], M["oxide"])),
        build_organizer(M),
        build_mat(M),
    ]
    return {look.name: export(look) for look in looks}


if __name__ == "__main__":
    print(build())
