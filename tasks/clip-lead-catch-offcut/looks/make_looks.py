"""make_looks.py: model the scene's visual meshes in Blender and export them as USD next to this file.

    blender -b -P looks/make_looks.py          (Blender 4.2 or later, from the task folder)

The physics stays the task file's boxes. Each look is a render-only mesh that ``looks.py`` puts on one of those
boxes, whose plain cuboid is then hidden but still collides. Every look is built in its box's own frame (metres,
centred on the box, before the box's turn: x across the box, y along it, z up), so it turns with the box.

- ``cutter_f.usdc``: the nippers' fixed half: a black grip, a steel neck and the anvil, a flat foot that rests on the
  board, inside the fixed half's box (checked when exported).
- ``cutter_m.usdc``: the moving half: a red grip, a steel neck and the blade, a bevelled foot above the anvil, inside
  the moving half's box (checked).
- ``board.usdc``: the circuit board seen from the solder side (pads, traces, a clipped lead's solder domes) and from
  below.
- ``capacitor.usdc``: the blue electrolytic capacitor that hangs under the board.

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
PREFIX = "clip-lead-catch-offcut__"
RNG = np.random.default_rng(23)
MM = 0.001

# -- the task file's layout (clip_lead_catch_offcut.py), metres ---------------------------------------------------------
BOARD_SIZE = (0.08, 0.07, 0.0016)
CAP_SIZE = (0.012, 0.012, 0.010)
# the nippers' boxes, in the cutter frame (x across, y along the jaws, z up, origin on the fixed half's bottom plane)
PLATE_F_X, PLATE_M_X = (-22 * MM, -12 * MM), (12 * MM, 22 * MM)
PLATE_Y, FOOT_Y, PLATE_H = (-170 * MM, -8 * MM), (-12 * MM, 22 * MM), 16 * MM
ANVIL_X, BLADE_X = -1 * MM, 9 * MM
ANVIL_TOP, BLADE_BOTTOM, BLADE_TOP = 0.8 * MM, 1.1 * MM, 3.4 * MM
F_BOX = ((PLATE_F_X[0], ANVIL_X), (PLATE_Y[0], FOOT_Y[1]), (0.0, PLATE_H))
M_BOX = ((BLADE_X, PLATE_M_X[1]), (PLATE_Y[0], FOOT_Y[1]), (0.0, PLATE_H))


def centre(box):
    return Vector([(a + b) / 2 for a, b in box])


def size_of(box):
    return tuple(b - a for a, b in box)

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
    M["lead"] = material("lead_tin", (0.78, 0.78, 0.76), 0.25, 1.0)
    M["tin"] = material("solder_tin", (0.80, 0.80, 0.78), 0.18, 1.0)
    M["fr4"] = material("fr4_edge", (0.55, 0.50, 0.28), 0.6)
    M["black"] = material("black_plastic", (0.02, 0.02, 0.022), 0.45)
    M["grip_black"] = material("grip_black", (0.012, 0.012, 0.014), 0.7)
    M["grip_red"] = material("grip_red", (0.70, 0.03, 0.02), 0.6)  # the one red thing on the table
    M["steel"] = material("steel", (0.62, 0.63, 0.65), 0.25, 1.0)
    M["steel_dark"] = material("steel_dark", (0.30, 0.31, 0.33), 0.3, 1.0)
    M["blade"] = material("blade_edge", (0.78, 0.79, 0.81), 0.15, 1.0)
    M["cap_blue"] = material("cap_blue", (0.02, 0.07, 0.55), 0.35)
    M["cap_stripe"] = material("cap_stripe", (0.55, 0.57, 0.62), 0.4)
    M["alu"] = material("aluminium", (0.70, 0.71, 0.73), 0.3, 1.0)
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


class Board:
    """The board's drawing, in board coordinates (u along x, v along y, from its centre)."""

    PADS = [(0.0, 0.0)]  # the pad the lead leaves through
    # a row of clipped leads of other parts, already soldered: (u, v)
    DONE = [(-0.030 + k * 0.00254, 0.020) for k in range(8)] + [(-0.030 + k * 0.00254, 0.0145) for k in range(8)]
    DONE += [(0.026, -0.018), (0.0286, -0.018), (0.026, 0.020), (0.0286, 0.020)]


def board_textures():
    """The board's two sides: green solder mask, copper under it, tinned pads, white outlines on the components side."""
    su, sv = BOARD_SIZE[0], BOARD_SIZE[1]
    out = {}
    for side in ("top", "bottom"):
        cv = Canvas(-su / 2, -sv / 2, su, sv, 1800)
        H, W = cv.h, cv.w
        copper, tin, silk, hole = (np.zeros((H, W), bool) for _ in range(4))
        if side == "top":  # solder side: traces between the pads, a ground pour
            nets = [[Board.PADS[0], (0.0, -0.012), (-0.020, -0.012), (-0.020, 0.0145)],
                    [Board.PADS[0], (0.012, 0.0), (0.012, 0.012), (0.026, 0.012), (0.026, 0.020)],
                    [(-0.030, 0.020), (-0.036, 0.020), (-0.036, -0.014), (0.0, -0.014)],
                    [(-0.0272, 0.0145), (-0.0272, 0.008), (0.0286, 0.008), (0.0286, -0.018)],
                    [(0.026, -0.018), (0.026, -0.026), (-0.010, -0.026)]]
            for net in nets:
                cv.line(copper, net, 0.0009)
            cv.rect(copper, 0.028, -0.030, 0.038, 0.030)
        for u, v in Board.DONE:
            cv.disc(tin, u, v, 0.0009)
            cv.disc(hole, u, v, 0.0003)
        for u, v in Board.PADS:  # the lead's pad: a square tinned land, 3.4 mm, round hole
            cv.rect(tin, u - 0.0017, v - 0.0017, u + 0.0017, v + 0.0017)
            cv.disc(hole, u, v, 0.0006)
        for u, v in ((-0.036, 0.031), (0.036, 0.031), (-0.036, -0.031), (0.036, -0.031)):  # mounting holes
            cv.ring(tin, u, v, 0.0022, 0.0009)
            cv.disc(hole, u, v, 0.0017)
        if side == "bottom":  # components side: the capacitor's outline over the lead, silk lines
            cv.ring(silk, 0.0, 0.0, 0.0072, 0.0004)
            cv.rect(silk, -0.0072, -0.0005, 0.0, 0.0005)
            cv.line(silk, [(-0.0095, 0.0075), (-0.0095, 0.0095)], 0.0004)
            cv.line(silk, [(-0.0105, 0.0085), (-0.0085, 0.0085)], 0.0004)
            for k in range(5):
                cv.ring(silk, -0.012 + 0.006 * k, 0.027, 0.0007, 0.0002)
        mask = 0.9 + 0.1 * _noise(H, W, 60)
        rgb = np.array([0.05, 0.30, 0.13])[None, None, :] * mask[..., None]
        rgb[copper] = np.array([0.09, 0.42, 0.17])
        rgb[silk] = np.array([0.93, 0.93, 0.90])
        rgb[tin] = np.array([0.82, 0.82, 0.80])
        rgb[hole] = np.array([0.10, 0.10, 0.08])
        out[side] = _save(f"board_{side}", rgb)
    return out


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



# -- the looks ----------------------------------------------------------------------------------------------------------


def build_board(M):
    L = Look("board")  # not scored: its solder domes stand 0.7 mm over the box
    su, sv, t = BOARD_SIZE
    tex = board_textures()
    L.mesh("core", bm_box((-su / 2, -sv / 2, -t / 2 + 0.0001), (su / 2, sv / 2, t / 2 - 0.0001)), M["fr4"])
    u0, u1, v0, v1 = -su / 2, su / 2, -sv / 2, sv / 2
    L.mesh("top", bm_quad([(u0, v0, t / 2), (u1, v0, t / 2), (u1, v1, t / 2), (u0, v1, t / 2)],
                          [(0, 0), (1, 0), (1, 1), (0, 1)]),
           material("board_top", (0.05, 0.30, 0.13), 0.35, image=tex["top"]))
    L.mesh("bottom", bm_quad([(u0, v0, -t / 2), (u0, v1, -t / 2), (u1, v1, -t / 2), (u1, v0, -t / 2)],
                             [(0, 0), (0, 1), (1, 1), (1, 0)]),
           material("board_bottom", (0.05, 0.30, 0.13), 0.35, image=tex["bottom"]))
    for i, (u, v) in enumerate(Board.DONE):  # the clipped leads of other parts: a solder dome on each
        L.mesh(f"dome_{i}", bm_cyl((u, v, t / 2 + 0.0003), 0.0004, 0.0008, "Z", 16, r2=0.0009), M["tin"], smooth=True)
    return L


def build_capacitor(M):
    L = Look("capacitor", CAP_SIZE)
    r, h = 0.0055, CAP_SIZE[2]
    L.mesh("can", bm_cyl((0, 0, 0), r, h - 0.0006, "Z", 40), M["cap_blue"], smooth=True)
    L.mesh("end", bm_cyl((0, 0, -h / 2 + 0.0002), r - 0.0004, 0.0004, "Z", 40), M["alu"], smooth=True)
    stripe = bm_cyl((0, 0, 0), r + 0.00005, h - 0.002, "Z", 12)
    ob = L.mesh("stripe", stripe, M["cap_stripe"])
    ob.scale = (0.35, 0.35, 1.0)
    ob.location = (-0.0038, 0.0, 0.0)
    return L


def build_cutter(name, box, plate_x, foot_x, foot_z, grip_mat, face_x, M):
    """One half of the nippers, in its box's frame (centred on the box). ``plate_x`` the handle plate's (lo, hi),
    ``foot_x`` the foot's, ``foot_z`` the foot's (bottom, top), ``face_x`` the working face's x."""
    L = Look(name, size_of(box))
    c = centre(box)

    def b(x, y, z):
        return (Vector((x[0], y[0], z[0])) - c, Vector((x[1], y[1], z[1])) - c)

    grip_y = (PLATE_Y[0], -34 * MM)
    neck_y = (-36 * MM, PLATE_Y[1])
    outer_m = plate_x[0] > 0  # the moving half's outer face is +x, the fixed half's -x
    inset = (plate_x[0], plate_x[1] - 0.0005) if outer_m else (plate_x[0] + 0.0005, plate_x[1])
    L.mesh("grip", bm_box(*b(inset, grip_y, (0.0, PLATE_H))), grip_mat, bevel=0.0018)
    ridge_mat = M["grip_black"] if grip_mat is M["grip_red"] else M["grip_red"]
    for k in range(7):  # finger ridges on the grip's outer face, flush with the plate
        y = PLATE_Y[0] + 0.012 + k * 0.011
        out = (plate_x[1] - 0.0008, plate_x[1]) if outer_m else (plate_x[0], plate_x[0] + 0.0008)
        L.mesh(f"ridge_{k}", bm_box(*b(out, (y, y + 0.0035), (0.003, PLATE_H - 0.003))), ridge_mat, bevel=0.0003)
    L.mesh("neck", bm_box(*b(plate_x, neck_y, (0.0, PLATE_H))), M["steel"], bevel=0.001)
    L.mesh("rivet", bm_cyl(Vector((sum(plate_x) / 2, -22 * MM, PLATE_H / 2)) - c, 0.0045, plate_x[1] - plate_x[0] - 0.0004,
                           "X", 24), M["steel_dark"], smooth=True)
    L.mesh("foot", bm_box(*b(foot_x, FOOT_Y, foot_z)), M["blade"], bevel=0.0002)
    return L


def build_cutter_f(M):
    return build_cutter("cutter_f", F_BOX, PLATE_F_X, (PLATE_F_X[0], ANVIL_X), (0.0, ANVIL_TOP), M["grip_black"], ANVIL_X, M)


def build_cutter_m(M):
    return build_cutter("cutter_m", M_BOX, PLATE_M_X, (BLADE_X, PLATE_M_X[1]), (BLADE_BOTTOM, BLADE_TOP), M["grip_red"],
                        BLADE_X, M)


def build(keep=False):
    """Model and export every look. ``keep`` leaves the models in the scene to look at; otherwise they are removed."""
    clear()
    M = materials()
    looks = [build_board(M), build_capacitor(M), build_cutter_f(M), build_cutter_m(M)]
    out = {look.name: export(look) for look in looks}
    if not keep:
        clear()
    return out


if __name__ == "__main__":
    for name, info in build().items():
        print(name, info)
