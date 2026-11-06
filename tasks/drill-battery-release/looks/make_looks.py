"""make_looks.py: model the battery pack and the tray in Blender and export them as USD next to this file.

    blender -b -P looks/make_looks.py          (Blender 4.2 or later, from the task folder)

The physics stays the task file's boxes. Each look is a render-only mesh that ``looks.py`` puts on one of those
boxes, whose plain cuboid is then hidden but still collides. Each look is built in its box's own frame (metres,
origin at the box's centre, z up) and exported with that origin.

- ``battery.usdc``: the 18 V battery pack, inside the battery's 5 x 7 x 5.8 cm box (checked when exported). x runs
  along the slide (+x faces the drill's handle), y across the two release buttons, z is the pack's thickness (the
  drill lies on its side, so one wide face is up).
- ``tray.usdc``: the parts tray, on its floor box (20 x 15 cm, 6 mm thick), with the walls the task file's wall
  boxes stand for.

The release buttons are task-file boxes of their own and need no mesh. Nothing is downloaded.
"""

import math
import os

import bmesh
import bpy
from mathutils import Matrix, Vector

HERE = os.path.dirname(os.path.abspath(__file__))

# -- the task file's sizes (drill_battery_release.py), metres --------------------------------------------------------
BATTERY = (0.05, 0.07, 0.058)  # along the slide, across the buttons, thickness
BUTTON = (0.01, 0.01)  # the release buttons' face (along x, along z)
TRAY_OUTER = (0.20, 0.15)
TRAY_WALL = 0.008
TRAY_WALL_H = 0.04
TRAY_FLOOR_T = 0.006

# -- materials (linear colours) --------------------------------------------------------------------------------------
_MATS = {}


def material(name, color=(0.5, 0.5, 0.5), rough=0.5, metal=0.0):
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
    m.diffuse_color = (*color, 1.0)
    _MATS[name] = m
    return m


# -- building blocks -------------------------------------------------------------------------------------------------


class Look:
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
            m.width, m.segments, m.limit_method = bevel, 3, "ANGLE"
        return ob

    def text(self, name, body, size, mat, at, depth):
        """Flat lettering lying in the xy plane, its base at height ``at[2]``, raised ``depth``."""
        curve = bpy.data.curves.new(name, "FONT")
        curve.body, curve.size, curve.extrude = body, size, depth / 2
        curve.align_x, curve.align_y = "CENTER", "CENTER"
        ob = bpy.data.objects.new(name + "_curve", curve)
        bpy.context.scene.collection.objects.link(ob)
        bpy.context.view_layer.update()
        me = bpy.data.meshes.new_from_object(ob.evaluated_get(bpy.context.evaluated_depsgraph_get()))
        bpy.data.objects.remove(ob)
        bpy.data.curves.remove(curve)
        me.transform(Matrix.Translation(Vector((at[0], at[1], at[2] + depth / 2))))
        tob = bpy.data.objects.new(name, me)
        self.coll.objects.link(tob)
        tob.parent = self.root
        me.materials.append(mat)
        return tob


def bm_box(lo, hi):
    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=1.0)
    c, s = (Vector(lo) + Vector(hi)) / 2, Vector(hi) - Vector(lo)
    for v in bm.verts:
        v.co = c + Vector((v.co.x * s.x, v.co.y * s.y, v.co.z * s.z))
    return bm


def rounded_rect(x0, y0, x1, y1, r, n=6):
    pts = []
    for cx, cy, a0 in ((x1 - r, y0 + r, -90), (x1 - r, y1 - r, 0), (x0 + r, y1 - r, 90), (x0 + r, y0 + r, 180)):
        for i in range(n + 1):
            a = math.radians(a0 + 90 * i / n)
            pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
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
    """The wall between two loops of the same length (outer and inner), from z0 to z1."""
    bm = bmesh.new()
    ol = [bm.verts.new((x, y, z0)) for x, y in outer]
    oh = [bm.verts.new((x, y, z1)) for x, y in outer]
    il = [bm.verts.new((x, y, z0)) for x, y in inner]
    ih = [bm.verts.new((x, y, z1)) for x, y in inner]
    n = len(outer)
    for i in range(n):
        j = (i + 1) % n
        bm.faces.new((ol[i], ol[j], oh[j], oh[i]))
        bm.faces.new((ih[i], ih[j], il[j], il[i]))
        bm.faces.new((oh[i], oh[j], ih[j], ih[i]))
        bm.faces.new((il[i], il[j], ol[j], ol[i]))
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    return bm


def turned(bm, axis, angle):
    bm.transform(Matrix.Rotation(angle, 4, axis))
    return bm


# -- the battery pack ------------------------------------------------------------------------------------------------


def build_battery():
    L = Look("battery", BATTERY)
    hx, hy, hz = (s / 2 for s in BATTERY)
    orange = material("battery_orange", (0.80, 0.17, 0.008), 0.45)
    black = material("battery_black", (0.012, 0.012, 0.013), 0.7)
    grey = material("battery_grey", (0.06, 0.062, 0.068), 0.5)
    brass = material("terminal_brass", (0.75, 0.55, 0.22), 0.3, 1.0)
    white = material("label_white", (0.85, 0.85, 0.85), 0.5)
    green = material("gauge_green", (0.05, 0.6, 0.08), 0.35)

    # The base: a black rubber cap at the end away from the drill.
    base_x = -hx + 0.009
    L.mesh("base_cap", bm_box((-hx, -hy, -hz), (base_x, hy, hz)), black, bevel=0.0025)
    # The casing: orange, a hair inside the cap's section.
    top_x = hx - 0.006
    L.mesh("casing", bm_box((base_x - 0.001, -hy + 0.001, -hz + 0.0008), (top_x, hy - 0.001, hz - 0.0008)), orange,
           bevel=0.002)
    # The stem plate that slides on the handle's foot, with its two terminal slots and the rail grooves.
    L.mesh("stem", bm_box((top_x - 0.001, -0.027, -0.021), (hx - 0.0006, 0.027, 0.021)), grey, bevel=0.0012)
    for s in (-1, 1):
        L.mesh(f"slot_{'lr'[s > 0]}", bm_box((hx - 0.0012, s * 0.009 - 0.0035, -0.012), (hx, s * 0.009 + 0.0035, 0.012)),
               black, bevel=0.0003)
        L.mesh(f"terminal_{'lr'[s > 0]}", bm_box((hx - 0.0011, s * 0.009 - 0.0007, -0.009), (hx - 0.0001, s * 0.009 + 0.0007, 0.009)),
               brass)
        L.mesh(f"rail_{'lr'[s > 0]}", bm_box((top_x - 0.0005, -0.029, s * 0.023 - 0.0015), (hx - 0.002, 0.029, s * 0.023 + 0.0015)),
               black, bevel=0.0004)
    # A black bezel round each release button, on the faces across the pack.
    bx, bz = (b / 2 + 0.003 for b in BUTTON)
    for s in (-1, 1):
        y0, y1 = sorted((s * (hy - 0.0012), s * hy))
        L.mesh(f"bezel_{'lr'[s > 0]}", bm_box((-bx, y0, -bz), (bx, y1, bz)), black, bevel=0.0004)
    # Grip ribs and a label on both wide faces (the pack lies on one of them).
    for s in (-1, 1):
        z_face = s * (hz - 0.0008)
        z_out = s * hz
        zl, zh = sorted((z_face, z_out))
        for i in range(5):
            x = base_x + 0.004 + i * 0.0035
            L.mesh(f"rib_{s}_{i}", bm_box((x, -0.026, zl), (x + 0.0016, 0.026, zh)), black, bevel=0.0003)
        panel_lo, panel_hi = sorted((s * (hz - 0.0008), s * (hz - 0.0003)))
        L.mesh(f"panel_{s}", bm_slab(rounded_rect(0.0005, -0.022, top_x - 0.004, 0.022, 0.003), panel_lo, panel_hi), black)
        if s > 0:
            L.text("label_18v", "18V", 0.014, white, (0.0105, 0.0, hz - 0.0003), 0.0002)
            for i in range(3):
                y = -0.008 + i * 0.008
                L.mesh(f"gauge_{i}", bm_box((-0.0045, y - 0.003, hz - 0.0003), (-0.0015, y + 0.003, hz - 0.0001)), green)
    return L


# -- the tray --------------------------------------------------------------------------------------------------------


def build_tray():
    L = Look("tray")
    ox, oy = TRAY_OUTER[0] / 2, TRAY_OUTER[1] / 2
    w = TRAY_WALL
    z0 = -TRAY_FLOOR_T / 2  # the floor box's bottom, on the table
    z1 = z0 + TRAY_WALL_H
    plastic = material("tray_plastic", (0.02, 0.05, 0.14), 0.55)
    floor = material("tray_floor", (0.03, 0.07, 0.18), 0.65)
    outer = rounded_rect(-ox, -oy, ox, oy, 0.014)
    inner = rounded_rect(-ox + w, -oy + w, ox - w, oy - w, 0.014 - w)
    L.mesh("walls", bm_ring(outer, inner, z0, z1), plastic, bevel=0.0012)
    L.mesh("floor", bm_slab(inner, z0, -z0), floor)
    lip = rounded_rect(-ox - 0.002, -oy - 0.002, ox + 0.002, oy + 0.002, 0.016)
    L.mesh("lip", bm_ring(lip, outer, z1 - 0.004, z1), plastic, bevel=0.0008)
    for i, x in enumerate((-0.05, 0.0, 0.05)):  # moulded ribs across the floor
        L.mesh(f"floor_rib_{i}", bm_box((x - 0.002, -oy + w, -z0), (x + 0.002, oy - w, -z0 + 0.0015)), floor, bevel=0.0006)
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
    path = os.path.join(HERE, look.name + ".usdc")
    bpy.ops.wm.usd_export(filepath=path, collection=look.coll.name, export_materials=True,
                          generate_preview_surface=True, export_textures_mode="KEEP", relative_paths=True,
                          root_prim_path="/look", export_lights=False, export_cameras=False,
                          convert_world_material=False, export_custom_properties=False, author_blender_name=False,
                          evaluation_mode="RENDER", export_uvmaps=True, export_normals=True)
    return {"file": os.path.basename(path), "kB": round(os.path.getsize(path) / 1024),
            "low": tuple(round(v, 4) for v in lo), "high": tuple(round(v, 4) for v in hi)}


def clear():
    """Start from an empty scene, whatever the file Blender opened."""
    for ob in list(bpy.data.objects):
        bpy.data.objects.remove(ob)
    for coll in list(bpy.data.collections):
        bpy.data.collections.remove(coll)
    for block in (bpy.data.meshes, bpy.data.curves, bpy.data.materials):
        for item in list(block):
            if item.users == 0:
                block.remove(item)
    _MATS.clear()


def build():
    clear()
    looks = [build_battery(), build_tray()]
    return {look.name: export(look) for look in looks}


if __name__ == "__main__":
    print(build())
