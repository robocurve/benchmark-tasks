"""make_looks.py: model the scene's visual meshes in Blender and export them as USD next to this file.

    blender -b -P looks/make_looks.py          (Blender 4.2 or later, from the task folder)

The physics stays the task file's boxes. Each look is a render-only mesh that ``looks.py`` puts on one of those
boxes, whose plain cuboid is then hidden but still collides. Every look is built in its box's own frame (metres,
x along the box's length, z up, the origin at the box's centre). The scene starts from an empty file, and nothing
is downloaded: there are no image textures.

- ``phone_1.usdc``, ``phone_2.usdc``, ``phone_3.usdc``: a 15 x 7.5 x 0.8 cm phone lying screen up, its top end
  toward +x, inside its box (checked when exported). Each has a metal frame in its own colour, a black glass front,
  a dark screen (``screen_off``) and, 0.1 mm above it, a white glowing screen (``screen_on``) that the task file
  shows once the phone has been switched on. The power button is a separate box of the task file.
- ``tray_green.usdc``, ``tray_grey.usdc``: an 18 x 12 x 3 cm plastic tray with 6 mm walls and a ribbed floor, put
  on the tray's floor box (its walls are hidden boxes of their own).
"""

import math
import os

import bmesh
import bpy
from mathutils import Matrix, Vector

HERE = os.path.dirname(os.path.abspath(__file__))

# -- the task file's sizes (phone_power_test_sort.py), metres -------------------------------------------------------
PHONE = (0.15, 0.075, 0.008)
TRAY_OUTER = (0.18, 0.12)
TRAY_H = 0.03
TRAY_WALL = 0.006
TRAY_FLOOR_T = 0.004
FRAMES = {"phone_1": (0.05, 0.05, 0.055), "phone_2": (0.62, 0.63, 0.66), "phone_3": (0.04, 0.07, 0.16)}
TRAYS = {"tray_green": (0.05, 0.42, 0.08), "tray_grey": (0.30, 0.31, 0.32)}

# -- materials (linear colours) --------------------------------------------------------------------------------------
_MATS = {}


def material(name, color=(0.5, 0.5, 0.5), rough=0.5, metal=0.0, emit=0.0):
    if name in _MATS:
        return _MATS[name]
    m = bpy.data.materials.new(name)
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
    m.diffuse_color = (*color, 1.0)
    _MATS[name] = m
    return m


# -- geometry --------------------------------------------------------------------------------------------------------


class Look:
    """A look's objects, parented to an empty at the box's centre (the origin), exported as one USD file."""

    def __init__(self, name, size=None):
        self.name, self.size = name, size
        self.coll = bpy.data.collections.new("look_" + name)
        bpy.context.scene.collection.children.link(self.coll)
        self.root = bpy.data.objects.new(name, None)
        self.coll.objects.link(self.root)

    def mesh(self, name, bm, mat, bevel=0.0, smooth=False):
        me = bpy.data.meshes.new(name)
        bm.to_mesh(me)
        bm.free()
        if smooth:
            me.shade_smooth()
        ob = bpy.data.objects.new(name, me)
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


def rounded_rect(hx, hy, r, n=6, cx=0.0, cy=0.0):
    pts = []
    for qx, qy, a0 in ((hx - r, -hy + r, -90), (hx - r, hy - r, 0), (-hx + r, hy - r, 90), (-hx + r, -hy + r, 180)):
        for i in range(n + 1):
            a = math.radians(a0 + 90 * i / n)
            pts.append((cx + qx + r * math.cos(a), cy + qy + r * math.sin(a)))
    return pts


def bm_slab(pts, z0, z1):
    """A polygon in the xy plane extruded from z0 to z1."""
    bm = bmesh.new()
    lo = [bm.verts.new((x, y, z0)) for x, y in pts]
    hi = [bm.verts.new((x, y, z1)) for x, y in pts]
    bm.faces.new(lo)
    bm.faces.new(hi)
    n = len(pts)
    for i in range(n):
        bm.faces.new((lo[i], lo[(i + 1) % n], hi[(i + 1) % n], hi[i]))
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    return bm


def bm_ring(outer, inner, z0, z1):
    """The band between two outlines with as many points, from z0 to z1: a tray's walls, open in the middle."""
    bm = bmesh.new()
    n = len(outer)
    ob = [bm.verts.new((x, y, z0)) for x, y in outer]
    ot = [bm.verts.new((x, y, z1)) for x, y in outer]
    ib = [bm.verts.new((x, y, z0)) for x, y in inner]
    it = [bm.verts.new((x, y, z1)) for x, y in inner]
    for i in range(n):
        j = (i + 1) % n
        bm.faces.new((ob[i], ob[j], ot[j], ot[i]))
        bm.faces.new((it[i], it[j], ib[j], ib[i]))
        bm.faces.new((ot[i], ot[j], it[j], it[i]))
        bm.faces.new((ib[i], ib[j], ob[j], ob[i]))
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    return bm


def bm_disc(cx, cy, r, z0, z1, segs=24):
    return bm_slab([(cx + r * math.cos(2 * math.pi * i / segs), cy + r * math.sin(2 * math.pi * i / segs))
                    for i in range(segs)], z0, z1)


# -- the phones ------------------------------------------------------------------------------------------------------


def build_phone(name, frame):
    L = Look(name, PHONE)
    hx, hy, hz = PHONE[0] / 2, PHONE[1] / 2, PHONE[2] / 2
    metal = material(f"{name}_frame", frame, 0.3, 0.8)
    glass = material("front_glass", (0.004, 0.004, 0.005), 0.08)
    off = material("screen_off", (0.010, 0.012, 0.016), 0.12)
    on = material("screen_on", (1.0, 1.0, 1.0), 0.4, emit=3.0)
    lens = material("lens", (0.0, 0.0, 0.0), 0.05)
    # body: the frame and back, its corners rounded 9 mm
    L.mesh("body", bm_slab(rounded_rect(hx, hy, 0.009), -hz, hz - 0.0008), metal, bevel=0.0006, smooth=True)
    # the front glass, 0.4 mm in from the frame, and the screen under it
    L.mesh("glass", bm_slab(rounded_rect(hx - 0.0004, hy - 0.0004, 0.0086), hz - 0.0008, hz - 0.0003), glass)
    sx, sy = hx - 0.0035, hy - 0.0030  # 3 mm bezels
    L.mesh("screen_off", bm_slab(rounded_rect(sx, sy, 0.0065), hz - 0.0003, hz - 0.0002), off)
    L.mesh("screen_on", bm_slab(rounded_rect(sx, sy, 0.0065), hz - 0.0002, hz - 0.0001), on)
    # the front camera, a punch hole near the top end (+x), drawn over both screens
    L.mesh("front_camera", bm_disc(sx - 0.006, 0.0, 0.0018, hz - 0.0001, hz), lens)
    # the earpiece slit in the top bezel
    L.mesh("earpiece", bm_box((hx - 0.0024, -0.006, hz - 0.0003), (hx - 0.0014, 0.006, hz)), lens)
    return L


# -- the trays -------------------------------------------------------------------------------------------------------


def build_tray(name, color):
    """The tray's floor box is 4 mm thick: its centre, the look's origin, is 2 mm above the table."""
    L = Look(name)
    plastic = material(name + "_plastic", color, 0.55)
    hx, hy, w = TRAY_OUTER[0] / 2, TRAY_OUTER[1] / 2, TRAY_WALL
    z0, zf, zt = -TRAY_FLOOR_T / 2, TRAY_FLOOR_T / 2, TRAY_H - TRAY_FLOOR_T / 2
    L.mesh("floor", bm_slab(rounded_rect(hx, hy, 0.008), z0, zf), plastic, bevel=0.0008)
    L.mesh("walls", bm_ring(rounded_rect(hx, hy, 0.008), rounded_rect(hx - w, hy - w, 0.004), zf, zt), plastic,
           bevel=0.001)
    # a lip round the top, 2 mm proud of the walls outside
    L.mesh("lip", bm_ring(rounded_rect(hx + 0.002, hy + 0.002, 0.009), rounded_rect(hx - w, hy - w, 0.004),
                          zt - 0.003, zt + 0.0005), plastic, bevel=0.0008)
    # ribs along the floor, 1 mm high
    for i in range(5):
        y = -0.04 + 0.02 * i
        L.mesh(f"rib_{i}", bm_box((-hx + w + 0.006, y - 0.0015, zf), (hx - w - 0.006, y + 0.0015, zf + 0.001)), plastic)
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
            lo, hi = Vector(map(min, lo, p)), Vector(map(max, hi, p))
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
                          generate_preview_surface=True, relative_paths=True, root_prim_path="/look",
                          export_lights=False, export_cameras=False, convert_world_material=False,
                          export_custom_properties=False, author_blender_name=False, evaluation_mode="RENDER",
                          export_uvmaps=False, export_normals=True)
    return {"file": os.path.basename(path), "kB": round(os.path.getsize(path) / 1024),
            "low": tuple(round(v, 4) for v in lo), "high": tuple(round(v, 4) for v in hi)}


def remove(look):
    """Delete a look's objects and meshes once exported, so the next look's parts keep their plain names."""
    for ob in list(look.coll.objects):
        data = ob.data
        bpy.data.objects.remove(ob)
        if data is not None and data.users == 0:
            bpy.data.meshes.remove(data)
    bpy.data.collections.remove(look.coll)


def build():
    bpy.ops.wm.read_factory_settings(use_empty=True)
    _MATS.clear()
    makers = [(build_phone, n, c) for n, c in FRAMES.items()] + [(build_tray, n, c) for n, c in TRAYS.items()]
    out = {}
    for make, name, color in makers:
        look = make(name, color)
        out[name] = export(look)
        remove(look)
    return out


if __name__ == "__main__":
    for name, info in build().items():
        print(name, info)
