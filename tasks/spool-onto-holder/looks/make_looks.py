"""make_looks.py: model the scene's visual meshes in Blender and export them as USD next to this file.

    blender -b -P looks/make_looks.py          (Blender 4.2 or later, from the task folder)

The physics stays the task file's boxes and the colliders ``spool.py`` adds. Each look is a render-only mesh that
``looks.py`` puts on one of the task file's boxes, whose plain cuboid is then hidden but still collides. Every look
is built in the station frame (metres, x forward, y left, z up) around an empty at its box's centre, and exported with
that empty at the origin.

- ``spool.usdc``: the filament spool lying flat (its axis along z), inside the spool's 20 x 20 x 7 cm box (checked
  when exported): two black flanges with windows, the grey hub round the 52 mm bore, the orange filament wound
  13 cm across, a blank white label.
- ``holder.usdc``: the spool holder on the post's box: the grooved aluminium post, the foot plate, the steel rod and
  the black stop collar, at the places of their colliders.
- ``carton.usdc``: the cardboard parts box the spool lies on.

Everything this script makes in Blender is named with the prefix ``spool-onto-holder__``, and only such datablocks are
removed when it runs again. Textures are drawn here with numpy and saved in ``textures/``. Nothing is downloaded.
"""

import math
import os

import bmesh
import bpy
import numpy as np
from mathutils import Matrix, Vector

HERE = os.path.dirname(os.path.abspath(__file__))
TEX = os.path.join(HERE, "textures")
P = "spool-onto-holder__"
RNG = np.random.default_rng(7)

# -- the task file's layout (spool_onto_holder.py), metres ------------------------------------------------------------
TOP = 0.7475
SPOOL_C, SPOOL_R, SPOOL_W = (0.21, 0.0, TOP + 0.10 + 0.035), 0.10, 0.07
BORE_R, CORE_R, FLANGE_T, HUB_R = 0.026, 0.065, 0.006, 0.040
POST_C, POST_SIZE = (0.47, 0.0, TOP + 0.145), (0.04, 0.04, 0.29)
POST_FACE_X = 0.45
ROD_Z, ROD_R, ROD_TIP_X = TOP + 0.25, 0.0225, 0.33
COLLAR_R, STOP_X = 0.035, 0.44
FOOT_C, FOOT_SIZE = (0.49, 0.0, TOP + 0.006), (0.14, 0.16, 0.012)
CARTON_C, CARTON_SIZE = (0.21, 0.0, TOP + 0.05), (0.22, 0.11, 0.10)


# -- materials (linear colours) --------------------------------------------------------------------------------------
_MATS = {}


def material(name, color=(0.5, 0.5, 0.5), rough=0.5, metal=0.0, image=None):
    full = P + name
    if full in _MATS:
        return _MATS[full]
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
    _MATS[full] = m
    return m


def materials():
    M = {}
    M["flange"] = material("flange_black", (0.016, 0.016, 0.018), 0.38)
    M["hub"] = material("hub_grey", (0.12, 0.12, 0.13), 0.45)
    M["filament"] = material("filament_orange", (0.90, 0.18, 0.006), 0.30, image=filament_texture())
    M["label"] = material("label_white", (0.80, 0.80, 0.78), 0.6, image=label_texture())
    M["alu"] = material("post_aluminium", (0.55, 0.57, 0.60), 0.35, 0.9)
    M["slot"] = material("post_slot", (0.05, 0.05, 0.055), 0.5, 0.5)
    M["steel"] = material("rod_steel", (0.72, 0.73, 0.75), 0.22, 1.0)
    M["collar"] = material("collar_black", (0.02, 0.02, 0.022), 0.35, 0.6)
    M["foot"] = material("foot_grey", (0.10, 0.105, 0.11), 0.45, 0.3)
    M["screw"] = material("screw_steel", (0.45, 0.45, 0.47), 0.3, 1.0)
    M["carton"] = material("carton", (0.42, 0.27, 0.13), 0.8, image=carton_texture())
    return M


# -- textures --------------------------------------------------------------------------------------------------------


def _save(name, rgb):
    """Save an sRGB image (rows from the bottom, as Blender stores them) as JPEG in textures/ and load it."""
    os.makedirs(TEX, exist_ok=True)
    path = os.path.join(TEX, name + ".jpg")
    h, w, _ = rgb.shape
    full = P + name
    old = bpy.data.images.get(full)
    if old:
        bpy.data.images.remove(old)
    img = bpy.data.images.new(full, w, h, alpha=False)
    rgba = np.concatenate([np.clip(rgb, 0, 1), np.ones((h, w, 1))], axis=2).astype(np.float32)
    img.pixels.foreach_set(rgba.ravel())
    img.filepath_raw = path
    img.file_format = "JPEG"
    img.save()
    bpy.data.images.remove(img)
    image = bpy.data.images.load(path)
    image.name = full
    return image


def filament_texture():
    """Wound filament seen from outside: fine strands across the spool's width (v), slightly uneven in shade."""
    h, w = 256, 512  # v across the width, u round the spool
    v = np.arange(h)[:, None]
    strands = 0.80 + 0.20 * np.abs(np.sin(v * math.pi / 3.0))  # one strand every 3 pixels (1.75 mm)
    wobble = 0.94 + 0.06 * np.sin(np.arange(w)[None, :] * 2 * math.pi / w * 3 + v * 0.05)
    shade = strands * wobble * (0.97 + 0.03 * RNG.random((h, w)))
    base = np.array([0.98, 0.47, 0.07])
    return _save("filament", shade[..., None] * base)


def label_texture():
    """A blank spool label: white, a coloured band and grey bars where print would be (no words)."""
    h, w = 128, 256
    rgb = np.full((h, w, 3), 0.93)
    rgb[h - 30:h - 8, 8:w - 8] = (0.98, 0.47, 0.07)
    for i, (y, length) in enumerate(((h - 50, 0.7), (h - 66, 0.5), (h - 82, 0.6), (h - 98, 0.35))):
        rgb[y - 4:y, 14:14 + int((w - 28) * length)] = 0.55 if i else 0.3
    return _save("label", rgb)


def carton_texture():
    """Brown corrugated cardboard (the box's top, 22 x 11 cm): faint flutes, a strip of packing tape along it and a
    blank white shipping label with grey bars (no words)."""
    h, w = 128, 256  # v across the box (y), u along it (x)
    y, x = np.mgrid[0:h, 0:w]
    flutes = 0.96 + 0.04 * np.sin(x * 2 * math.pi / 6)
    rgb = (flutes * (0.97 + 0.03 * RNG.random((h, w))))[..., None] * np.array([0.72, 0.53, 0.33])
    tape = np.abs(y - h / 2) < 12
    rgb[tape] = rgb[tape] * 0.85 + np.array([0.80, 0.62, 0.40]) * 0.25
    rgb[18:52, 20:90] = 0.92
    for k, length in enumerate((0.8, 0.55, 0.65)):
        rgb[24 + 9 * k:28 + 9 * k, 26:26 + int(58 * length)] = 0.5
    return _save("carton", rgb)


# -- geometry --------------------------------------------------------------------------------------------------------


class Look:
    def __init__(self, name, origin, size=None):
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


def _axis_matrix(axis):
    return {"Z": Matrix.Identity(4), "Y": Matrix.Rotation(math.pi / 2, 4, "X"),
            "X": Matrix.Rotation(math.pi / 2, 4, "Y")}[axis]


def bm_cyl(center, r, h, axis="Z", segs=48, r2=None):
    bm = bmesh.new()
    bmesh.ops.create_cone(bm, cap_ends=True, segments=segs, radius1=r, radius2=r if r2 is None else r2, depth=h,
                          matrix=Matrix.Translation(Vector(center)) @ _axis_matrix(axis))
    return bm


def bm_ring(center, r_in, r_out, h, axis="Z", segs=96, uv_outer=False):
    """A tube from ``r_in`` to ``r_out``, ``h`` long, its axis along ``axis``. With ``uv_outer`` its outside is
    mapped u round it, v along it."""
    bm = bmesh.new()
    rings = []
    for r, z in ((r_out, -h / 2), (r_out, h / 2), (r_in, h / 2), (r_in, -h / 2)):
        rings.append([bm.verts.new((r * math.cos(2 * math.pi * i / segs), r * math.sin(2 * math.pi * i / segs), z))
                      for i in range(segs)])
    layer = bm.loops.layers.uv.new("UVMap") if uv_outer else None
    for k, (a, b) in enumerate(zip(rings, rings[1:] + rings[:1])):
        for i in range(segs):
            j = (i + 1) % segs
            f = bm.faces.new((a[i], a[j], b[j], b[i]))
            if layer is not None and k == 0:  # the outside, from -h/2 to h/2
                for loop, (u, v) in zip(f.loops, ((i, 0), (i + 1, 0), (i + 1, 1), (i, 1))):
                    loop[layer].uv = (u / segs * 4, v)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    bm.transform(Matrix.Translation(Vector(center)) @ _axis_matrix(axis))
    return bm


def bm_flange(center, z, windows=6, segs=96):
    """A spool flange: a disc from the bore to the rim, ``FLANGE_T`` thick, with ``windows`` openings between the hub
    and the filament's surface. Its face is a grid of quads between radii, the windows' quads left out, extruded
    through the thickness (which makes the rim, the bore and the windows' sides)."""
    bm = bmesh.new()
    radii = [BORE_R, HUB_R + 0.002, 0.047, 0.055, 0.061, CORE_R - 0.002, SPOOL_R]
    band = (1, 4)  # between radii[1] and radii[4]: the windows
    per = segs // windows
    verts = {(ri, i): bm.verts.new((r * math.cos(2 * math.pi * i / segs), r * math.sin(2 * math.pi * i / segs),
                                    FLANGE_T / 2))
             for ri, r in enumerate(radii) for i in range(segs)}
    faces = []
    for ri in range(len(radii) - 1):
        for i in range(segs):
            if band[0] <= ri < band[1] and 0.12 < (i % per) / per < 0.78:
                continue
            j = (i + 1) % segs
            faces.append(bm.faces.new((verts[ri, i], verts[ri + 1, i], verts[ri + 1, j], verts[ri, j])))
    out = bmesh.ops.extrude_face_region(bm, geom=faces, use_keep_orig=True)
    moved = [g for g in out["geom"] if isinstance(g, bmesh.types.BMVert)]
    bmesh.ops.translate(bm, vec=Vector((0, 0, -FLANGE_T)), verts=moved)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    bm.transform(Matrix.Translation(Vector(center) + Vector((0, 0, z))))
    return bm


# -- the spool -------------------------------------------------------------------------------------------------------


def build_spool(M):
    L = Look("spool", SPOOL_C, (2 * SPOOL_R, 2 * SPOOL_R, SPOOL_W))
    c = Vector(SPOOL_C)
    inner_w = SPOOL_W - 2 * FLANGE_T
    for name, z in (("flange_top", SPOOL_W / 2 - FLANGE_T / 2), ("flange_bottom", -SPOOL_W / 2 + FLANGE_T / 2)):
        L.mesh(name, bm_flange(c, z), M["flange"], bevel=0.0008)
    L.mesh("hub", bm_ring(c, BORE_R, HUB_R, inner_w + 0.0004), M["hub"], smooth=True)
    L.mesh("filament", bm_ring(c, HUB_R - 0.0005, CORE_R, inner_w - 0.0006, uv_outer=True), M["filament"],
           smooth=True)
    # A blank label on the top flange, between two windows' rims (no words).
    a = math.radians(90)
    lc = c + Vector((0.084 * math.cos(a), 0.084 * math.sin(a), SPOOL_W / 2 + 0.0))
    label = bm_box((-0.022, -0.009, -0.0004), (0.022, 0.009, 0.0), uv=(-0.022, -0.009, 0.022, 0.009))
    bmesh.ops.translate(label, vec=lc, verts=label.verts)
    L.mesh("label", label, M["label"])
    return L


# -- the holder ------------------------------------------------------------------------------------------------------


def build_holder(M):
    L = Look("holder", POST_C)
    hx, hy, hz = (s / 2 for s in POST_SIZE)
    px, py, pz = POST_C
    L.mesh("post", bm_box((px - hx, py - hy, pz - hz), (px + hx, py + hy, pz + hz)), M["alu"], bevel=0.0015)
    # T-slots: a dark groove down the middle of each face (drawn on the face, 6 mm wide).
    for name, lo, hi in (
        ("slot_front", (px - hx - 0.0004, -0.003, TOP + 0.015), (px - hx + 0.0004, 0.003, pz + hz - 0.005)),
        ("slot_back", (px + hx - 0.0004, -0.003, TOP + 0.015), (px + hx + 0.0004, 0.003, pz + hz - 0.005)),
        ("slot_left", (px - 0.003, hy - 0.0004, TOP + 0.015), (px + 0.003, hy + 0.0004, pz + hz - 0.005)),
        ("slot_right", (px - 0.003, -hy - 0.0004, TOP + 0.015), (px + 0.003, -hy + 0.0004, pz + hz - 0.005)),
    ):
        L.mesh(name, bm_box(lo, hi), M["slot"])
    L.mesh("cap", bm_box((px - hx, -hy, pz + hz - 0.003), (px + hx, hy, pz + hz + 0.001)), M["collar"], bevel=0.001)
    fx, fy, fz = FOOT_C
    sx, sy, sz = (s / 2 for s in FOOT_SIZE)
    L.mesh("foot", bm_box((fx - sx, fy - sy, fz - sz), (fx + sx, fy + sy, fz + sz)), M["foot"], bevel=0.002)
    for i, (dx, dy) in enumerate(((-1, -1), (-1, 1), (1, -1), (1, 1))):
        L.mesh(f"screw_{i}", bm_cyl((fx + dx * (sx - 0.012), fy + dy * (sy - 0.012), fz + sz + 0.0015), 0.0045,
                                    0.003, segs=24), M["screw"], smooth=True)
    # The rod: steel, its last 2 cm chamfered to a 28 mm nose; the collar, black, with a set screw on top.
    L.mesh("rod", bm_cyl(((ROD_TIP_X + 0.02 + POST_FACE_X) / 2, 0.0, ROD_Z), ROD_R, POST_FACE_X - ROD_TIP_X - 0.02,
                         axis="X"), M["steel"], smooth=True)
    L.mesh("rod_nose", bm_cyl((ROD_TIP_X + 0.01, 0.0, ROD_Z), 0.014, 0.02, axis="X", r2=ROD_R), M["steel"],
           smooth=True)
    L.mesh("collar", bm_cyl(((STOP_X + POST_FACE_X) / 2, 0.0, ROD_Z), COLLAR_R, POST_FACE_X - STOP_X, axis="X"),
           M["collar"], bevel=0.0008)
    L.mesh("set_screw", bm_cyl(((STOP_X + POST_FACE_X) / 2, 0.0, ROD_Z + COLLAR_R + 0.001), 0.003, 0.003, segs=16),
           M["screw"], smooth=True)
    return L


def build_carton(M):
    L = Look("carton", CARTON_C, CARTON_SIZE)
    cx, cy, cz = CARTON_C
    hx, hy, hz = (s / 2 for s in CARTON_SIZE)
    L.mesh("carton", bm_box((cx - hx, cy - hy, cz - hz), (cx + hx, cy + hy, cz + hz),
                            uv=(cx - hx, cy - hy, cx + hx, cy + hy)), M["carton"], bevel=0.002)
    return L


# -- export ----------------------------------------------------------------------------------------------------------


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
    """Remove everything this script made before (and only that: names with the prefix)."""
    for coll in list(bpy.data.collections):
        if coll.name.startswith(P):
            for ob in list(coll.objects):
                if ob.name.startswith(P):
                    bpy.data.objects.remove(ob)
            bpy.data.collections.remove(coll)
    for block in (bpy.data.objects, bpy.data.meshes, bpy.data.materials, bpy.data.images):
        for item in list(block):
            if item.name.startswith(P) and item.users == 0:
                block.remove(item)
    _MATS.clear()


def build():
    clear()
    M = materials()
    looks = [build_spool(M), build_holder(M), build_carton(M)]
    return {look.name: export(look) for look in looks}


if __name__ == "__main__":
    print(build())
