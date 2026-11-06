"""make_looks.py: model the scene's visual meshes in Blender and export them as USD next to this file.

    blender -b -P looks/make_looks.py          (Blender 4.2 or later)

The physics stays the task file's boxes. Each look is a render-only mesh that ``looks.py`` puts on one of those boxes,
whose plain cuboid is then hidden but still sets its bounds. Every look is built in its box's own frame, around the
box's centre (metres, z up), and checked to stay inside its bounds when it is exported.

- ``power_supply.usdc``: a bench power supply, 24 x 20 x 10 cm, its front panel toward -x: two displays, two knobs,
  a power switch and three brass output posts.
- ``black_lead.usdc``, ``red_lead.usdc``: a test lead along x, pin toward -x: a 4 mm nickel pin with a pointed tip,
  a shrouded plug body 1 cm across with a strain relief, and a 10 cm stub of silicone cable. It stays inside the
  bounds of the lead's colliders (``leads.py``): x from -38 mm to 120 mm, 5 mm either side of the axis.

Everything this script makes in Blender is named with the task folder's prefix, and only that is removed again, so
it can run in a Blender session that holds other work. Textures are drawn here with numpy and saved in
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
PREFIX = "multimeter-test-leads__"
RNG = np.random.default_rng(6)

# -- the task file's sizes (multimeter_test_leads.py, leads.py), metres ----------------------------------------------
PSU_SIZE = (0.24, 0.20, 0.10)
LEAD_LO = (-0.038, -0.005, -0.005)  # the lead's colliders' bounds around the plug body's centre
LEAD_HI = (0.120, 0.005, 0.005)


def n(name):
    return PREFIX + name


# -- materials (linear colours) --------------------------------------------------------------------------------------
_MATS = {}


def material(name, color=(0.5, 0.5, 0.5), rough=0.5, metal=0.0, image=None, emit=None):
    if name in _MATS:
        return _MATS[name]
    m = bpy.data.materials.get(n(name)) or bpy.data.materials.new(n(name))
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
        tex.extension = "EXTEND"
        links.new(tex.outputs["Color"], bsdf.inputs["Base Color"])
    if emit is not None:
        bsdf.inputs["Emission Color"].default_value = (*emit, 1.0)
        bsdf.inputs["Emission Strength"].default_value = 1.0
    m.diffuse_color = (*color, 1.0)
    _MATS[name] = m
    return m


def _save(name, rgb):
    """Save an sRGB image (rows from the bottom, as Blender stores them) as JPEG in textures/ and load it."""
    os.makedirs(TEX, exist_ok=True)
    path = os.path.join(TEX, name + ".jpg")
    h, w, _ = rgb.shape
    for old in [i for i in bpy.data.images if i.name.startswith(n(name))]:
        bpy.data.images.remove(old)
    img = bpy.data.images.new(n(name) + "_draw", w, h, alpha=False)
    rgba = np.concatenate([np.clip(rgb, 0, 1), np.ones((h, w, 1))], axis=2).astype(np.float32)
    img.pixels.foreach_set(rgba.ravel())
    img.filepath_raw = path
    img.file_format = "JPEG"
    img.save()
    bpy.data.images.remove(img)
    loaded = bpy.data.images.load(path)
    loaded.name = n(name)
    return loaded


# -- the power supply's front panel ----------------------------------------------------------------------------------
# Drawn over the panel's face as seen from the front: u along -y (the robot's left to right is +y to -y, so u runs
# from the panel's +y edge), v up. 1 px = 0.25 mm.
PANEL_Y = (0.094, -0.094)  # the panel's left and right edges (y), seen from the front
PANEL_Z = (-0.040, 0.044)
PX = 4000.0  # pixels per metre

SEGMENTS = {  # seven-segment digits: a b c d e f g
    "0": "abcdef", "1": "bc", "2": "abdeg", "3": "abcdg", "4": "bcfg", "5": "acdfg", "6": "acdefg", "7": "abc",
    "8": "abcdefg", "9": "abcdfg",
}


def _uv(y, z):
    """Panel point (y, z) to texture pixels (column, row from the bottom)."""
    return (PANEL_Y[0] - y) * PX, (z - PANEL_Z[0]) * PX


def _fill(img, y0, z0, y1, z1, color):
    c0, r0 = _uv(max(y0, y1), min(z0, z1))
    c1, r1 = _uv(min(y0, y1), max(z0, z1))
    img[int(r0):int(math.ceil(r1)), int(c0):int(math.ceil(c1))] = color


def _digits(img, text, y_right, z0, height, color):
    """Seven-segment digits, right-aligned at ``y_right`` (the display reads from +y to -y)."""
    w, t = height * 0.5, height * 0.12
    cells = []
    for ch in text:
        if ch == ".":
            cells[-1] = (cells[-1][0], True)
        else:
            cells.append((ch, False))
    y = y_right + (len(cells) - 1) * (w + 0.0025) + w  # the leftmost digit's left edge
    for ch, dot in cells:
        l, r = y, y - w
        mid = z0 + height / 2
        seg = {
            "a": (l, z0 + height - t, r, z0 + height), "d": (l, z0, r, z0 + t), "g": (l, mid - t / 2, r, mid + t / 2),
            "f": (l, mid, l - t, z0 + height), "b": (r + t, mid, r, z0 + height),
            "e": (l, z0, l - t, mid), "c": (r + t, z0, r, mid),
        }
        for s in SEGMENTS[ch]:
            _fill(img, *seg[s], color)
        if dot:
            _fill(img, r - 0.0004, z0, r - 0.0004 - t, z0 + t, color)
        y -= w + 0.0025


def panel_texture():
    w = int(round((PANEL_Y[0] - PANEL_Y[1]) * PX))
    h = int(round((PANEL_Z[1] - PANEL_Z[0]) * PX))
    base = np.array([0.20, 0.21, 0.22])
    noise = RNG.normal(0, 0.008, (h, w, 1))
    img = np.broadcast_to(base, (h, w, 3)) + noise
    img = np.array(img)
    lcd, lit, label = np.array([0.03, 0.06, 0.04]), np.array([0.35, 0.95, 0.45]), np.array([0.72, 0.74, 0.76])
    for y_left, text, unit in ((0.084, "12.00", "V"), (0.034, "0.250", "A")):
        _fill(img, y_left, 0.012, y_left - 0.044, 0.036, lcd)  # the display window
        _digits(img, text, y_left - 0.040, 0.016, 0.016, lit)
        _fill(img, y_left, 0.006, y_left - 0.044, 0.0072, label)  # a label bar under it
        _fill(img, y_left - 0.040, 0.038, y_left - 0.044, 0.042, label)  # the unit mark
    for y_c in (0.062, 0.012):  # rings printed around the knobs (the knobs are meshes)
        c, r = _uv(y_c, -0.016)
        yy, xx = np.ogrid[:h, :w]
        d = np.hypot(xx - c, yy - r)
        img[(d > 0.0115 * PX) & (d < 0.0123 * PX)] = label
        for k in range(9):  # scale ticks
            a = math.radians(-135 + 270 * k / 8)
            for s in np.linspace(0.0128, 0.0140, 6):
                cc, rr = int(c + s * PX * math.sin(a)), int(r + s * PX * math.cos(a))
                img[max(rr - 1, 0):rr + 1, max(cc - 1, 0):cc + 1] = label
    _fill(img, -0.020, -0.036, -0.088, -0.030, label)  # the label strip over the output posts
    _fill(img, 0.090, -0.038, 0.072, -0.034, label)  # under the power switch
    return _save("psu_panel", img)  # rows from the bottom, as drawn


def case_texture():
    h, w = 256, 256
    img = np.full((h, w, 3), 0.11) + RNG.normal(0, 0.012, (h, w, 1))
    return _save("psu_case", img)


# -- geometry --------------------------------------------------------------------------------------------------------


class Look:
    def __init__(self, name, lo, hi):
        self.name, self.lo, self.hi = name, Vector(lo), Vector(hi)
        self.coll = bpy.data.collections.new(n("look_" + name))
        bpy.context.scene.collection.children.link(self.coll)
        self.root = bpy.data.objects.new(n(name), None)
        self.coll.objects.link(self.root)

    def mesh(self, name, bm, mat, bevel=0.0, smooth=False):
        me = bpy.data.meshes.new(n(f"{self.name}_{name}"))
        bm.to_mesh(me)
        bm.free()
        if smooth:
            me.shade_smooth()
        ob = bpy.data.objects.new(n(f"{self.name}_{name}"), me)
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
        uv(bm)
    return bm


def _axis_matrix(axis):
    return {"Z": Matrix.Identity(4), "Y": Matrix.Rotation(math.pi / 2, 4, "X"),
            "X": Matrix.Rotation(math.pi / 2, 4, "Y")}[axis]


def bm_cyl(center, r, h, axis="Z", segs=32, r2=None):
    bm = bmesh.new()
    bmesh.ops.create_cone(bm, cap_ends=True, segments=segs, radius1=r, radius2=r if r2 is None else r2, depth=h,
                          matrix=Matrix.Translation(Vector(center)) @ _axis_matrix(axis))
    return bm


def bm_along_x(x0, x1, r0, r1=None, segs=32):
    """A cylinder (or cone) on the x axis from x0 (radius r0) to x1 (radius r1)."""
    r1 = r0 if r1 is None else r1
    bm = bmesh.new()
    place = Matrix.Translation(Vector(((x0 + x1) / 2, 0, 0))) @ Matrix.Rotation(math.pi / 2, 4, "Y")
    bmesh.ops.create_cone(bm, cap_ends=True, segments=segs, radius1=r0, radius2=r1, depth=x1 - x0, matrix=place)
    return bm


def _panel_uv(bm):
    layer = bm.loops.layers.uv.new("UVMap")
    for f in bm.faces:
        for loop in f.loops:
            co = loop.vert.co
            loop[layer].uv = ((PANEL_Y[0] - co.y) / (PANEL_Y[0] - PANEL_Y[1]),
                              (co.z - PANEL_Z[0]) / (PANEL_Z[1] - PANEL_Z[0]))


def _box_uv(bm, scale=6.0):
    layer = bm.loops.layers.uv.new("UVMap")
    for f in bm.faces:
        nrm = f.normal
        for loop in f.loops:
            co = loop.vert.co
            if abs(nrm.z) > 0.5:
                uv = (co.x, co.y)
            elif abs(nrm.x) > 0.5:
                uv = (co.y, co.z)
            else:
                uv = (co.x, co.z)
            loop[layer].uv = (uv[0] * scale, uv[1] * scale)


def build_power_supply():
    sx, sy, sz = (s / 2 for s in PSU_SIZE)
    L = Look("power_supply", (-sx, -sy, -sz), (sx, sy, sz))
    case = material("psu_case", (0.012, 0.013, 0.014), 0.55, image=case_texture())
    panel = material("psu_panel", (0.03, 0.03, 0.03), 0.6, image=panel_texture())
    knob = material("psu_knob", (0.008, 0.008, 0.009), 0.35)
    brass = material("psu_brass", (0.55, 0.40, 0.12), 0.3, metal=1.0)
    grey = material("psu_switch", (0.30, 0.31, 0.32), 0.4)
    rubber = material("psu_feet", (0.01, 0.01, 0.01), 0.9)
    foot = 0.006
    front = -sx + 0.014  # the panel's face, recessed 14 mm behind the case's front lip
    L.mesh("case", bm_box((front + 0.002, -sy, -sz + foot), (sx, sy, sz), uv=_box_uv), case, bevel=0.003)
    # the front frame around the recessed panel
    lip = front + 0.003
    for lo, hi in (((-sx, -sy, sz - 0.006), (lip, sy, sz)), ((-sx, -sy, -sz + foot), (lip, sy, -sz + foot + 0.004)),
                   ((-sx, -sy, -sz + foot), (lip, -sy + 0.006, sz)), ((-sx, sy - 0.006, -sz + foot), (lip, sy, sz))):
        L.mesh("frame", bm_box(lo, hi, uv=_box_uv), case, bevel=0.0015)
    L.mesh("panel", bm_box((front, PANEL_Y[1], PANEL_Z[0]), (front + 0.002, PANEL_Y[0], PANEL_Z[1]), uv=_panel_uv),
           panel)
    for y in (0.062, 0.012):  # knobs
        L.mesh("knob_skirt", bm_cyl((front - 0.002, y, -0.016), 0.0105, 0.004, "X", 48), knob, smooth=True)
        L.mesh("knob", bm_cyl((front - 0.0075, y, -0.016), 0.0085, 0.011, "X", 48, r2=0.0080), knob, smooth=True,
               bevel=0.001)
        L.mesh("knob_mark", bm_box((front - 0.0132, y - 0.0008, -0.016), (front - 0.0128, y + 0.0008, -0.009)), grey)
    for y in (-0.030, -0.054, -0.078):  # output posts
        L.mesh("post_nut", bm_cyl((front - 0.003, y, -0.022), 0.0055, 0.006, "X", 6), brass, bevel=0.0005)
        L.mesh("post", bm_cyl((front - 0.008, y, -0.022), 0.0038, 0.008, "X", 32), brass, smooth=True, bevel=0.0006)
    L.mesh("switch", bm_box((front - 0.004, 0.072, -0.031), (front, 0.090, -0.022)), grey, bevel=0.001)
    for x in (-sx + 0.03, sx - 0.03):
        for y in (-sy + 0.03, sy - 0.03):
            L.mesh("foot", bm_cyl((x, y, -sz + foot / 2), 0.008, foot, "Z", 24), rubber, smooth=True)
    for i in range(9):  # vent slots on the top
        y = -0.06 + 0.015 * i
        L.mesh("vent", bm_box((0.02, y - 0.002, sz - 0.0004), (0.10, y + 0.002, sz + 0.0)), rubber)
    return L


def build_lead(name, plastic, cable_color):
    L = Look(name, LEAD_LO, LEAD_HI)
    body = material(f"{name}_plastic", plastic, 0.45)
    cable = material(f"{name}_cable", cable_color, 0.6)
    nickel = material("lead_nickel", (0.62, 0.62, 0.60), 0.25, metal=1.0)
    L.mesh("shroud", bm_along_x(-0.020, -0.012, 0.0040, 0.0044, 40), body, smooth=True)
    L.mesh("body", bm_along_x(-0.012, 0.012, 0.0048, 0.0048, 40), body, smooth=True, bevel=0.0006)
    for x in np.linspace(-0.006, 0.008, 6):  # grip rings
        L.mesh("ring", bm_along_x(x - 0.0006, x + 0.0006, 0.0050, 0.0050, 40), body, smooth=True)
    L.mesh("strain_relief", bm_along_x(0.012, 0.028, 0.0045, 0.0026, 40), body, smooth=True)
    L.mesh("cable", bm_along_x(0.028, LEAD_HI[0] - 0.0005, 0.0021, 0.0021, 24), cable, smooth=True)
    L.mesh("pin", bm_along_x(-0.0365, -0.0198, 0.00195, 0.00195, 32), nickel, smooth=True)
    L.mesh("pin_spring", bm_along_x(-0.031, -0.025, 0.00195, 0.00205, 32), nickel, smooth=True)
    L.mesh("pin_tip", bm_along_x(LEAD_LO[0] + 0.0002, -0.0365, 0.0008, 0.00195, 32), nickel, smooth=True)
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
    over = max(max(look.lo[i] - lo[i], hi[i] - look.hi[i]) for i in range(3))
    if over > 1e-5:
        raise ValueError(f"{look.name}: the look sticks {over * 1000:.2f} mm out of its bounds ({lo}, {hi})")
    path = os.path.join(HERE, look.name + ".usdc")
    bpy.ops.wm.usd_export(filepath=path, collection=look.coll.name, export_materials=True,
                          generate_preview_surface=True, export_textures_mode="KEEP", relative_paths=True,
                          root_prim_path="/look", export_lights=False, export_cameras=False,
                          convert_world_material=False, export_custom_properties=False, author_blender_name=False,
                          evaluation_mode="RENDER", export_uvmaps=True, export_normals=True)
    return {"file": os.path.basename(path), "kB": round(os.path.getsize(path) / 1024),
            "low": tuple(round(v, 4) for v in lo), "high": tuple(round(v, 4) for v in hi)}


def clear():
    """Remove everything this script made before (only names with its prefix)."""
    for coll in [c for c in bpy.data.collections if c.name.startswith(PREFIX)]:
        for ob in list(coll.objects):
            bpy.data.objects.remove(ob)
        bpy.data.collections.remove(coll)
    for block in (bpy.data.objects, bpy.data.meshes, bpy.data.materials, bpy.data.images):
        for item in [i for i in block if i.name.startswith(PREFIX)]:
            block.remove(item)
    _MATS.clear()


def build(keep=False):
    clear()
    looks = [
        build_power_supply(),
        build_lead("black_lead", (0.012, 0.012, 0.013), (0.015, 0.015, 0.016)),
        build_lead("red_lead", (0.62, 0.012, 0.010), (0.55, 0.010, 0.008)),
    ]
    out = {look.name: export(look) for look in looks}
    if not keep:
        clear()
    return out


if __name__ == "__main__":
    print(build())
