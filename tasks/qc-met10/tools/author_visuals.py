"""Author the qc-met10 visual meshes in Blender (headless) and export one USD per physics body.

blender -b --factory-startup --python tools/author_visuals.py -- <package dir>
(then each .usda is re-written as a binary .usdc crate with pxr Sdf.Layer.Export; content checked identical)

Every mesh is built in its body's frame (metres, z up) from the same numbers as qc_met10.py, so the visual sits on the
invisible collision boxes. Materials are UsdPreviewSurface (base colour / roughness / normal / emissive textures from
tools/make_textures.py, metallic per material)."""
import math
import pathlib
import sys

import bmesh
import bpy

PKG = pathlib.Path(sys.argv[sys.argv.index("--") + 1]).resolve()
OUT = PKG / "assets" / "qc_met10"
TEX = OUT / "textures"

# -- geometry copied from qc_met10.py (metres, part frames) ----------------------------------------------------------
AXIS_Z, SPEC_T = 0.076, 0.0032
FLOOR_Z = AXIS_Z - SPEC_T / 2
BED_TOP = 0.040
GAUGE_Y, CROSSHEAD_Y = 0.205, 0.160
XH_BACK = 0.035  # crosshead start: grip B backed off 35 mm along -y
GRIP_FACE, STOP_Y = 0.0575, 0.0905
JAW_W, HOUSING_D, HOUSING_A, HOUSING_B = 0.0191, 0.0381, 0.0495, 0.0622  # maker drawing C, D, A, B
HOUSING_Y0 = GRIP_FACE + 0.003
HOUSING_TOP = AXIS_Z + HOUSING_A / 2
HOUSING_BOT = AXIS_Z - HOUSING_A / 2
JAW_OPEN, JAW_H = 0.0064, 0.008
JAW_TOP = FLOOR_Z + JAW_H
SLOT_CEIL = FLOOR_Z + JAW_OPEN + JAW_H + 0.0006
STEM = (0.006, 0.010)
FOLLOWER_Z, FOLLOWER = HOUSING_TOP + 0.005, (0.006, 0.026, 0.004)
GAP_LIFTED, RAMP_RUN, RAMP_Y = 0.0062, 0.016, 0.009
LEAD_IN = 0.003
JAW_EDGE = 0.0015  # 45 deg chamfer on each lower jaw's entry edge at the jaw face
RAMP_LOW = FOLLOWER_Z - 0.001
PADDLE_X, PADDLE = 0.026, (0.006, 0.026, 0.030)
PADDLE_BOT = HOUSING_TOP + 0.001
MAT_H = 0.003
REST_H = 0.012
REST_Y, STOP_X = 0.074, 0.0205  # tray rests under the tab ends; back stops (front face from the tray centre)
STOP_TOP = REST_H - MAT_H + 0.005
PANEL_SIZE, PANEL_TOP, KEY_ROW, KEY_SIZE = (0.110, 0.060, 0.016), 0.085, 0.010, (0.016, 0.016, 0.004)


def reset():
    bpy.ops.wm.read_factory_settings(use_empty=True)


_mats = {}


def img(name, srgb=True):
    im = bpy.data.images.load(str(TEX / f"{name}.png"), check_existing=True)
    im.colorspace_settings.name = "sRGB" if srgb else "Non-Color"
    return im


def mat(name, albedo=None, color=(0.5, 0.5, 0.5), rough=0.5, rough_map=None, metal=0.0, normal=None, emissive=None,
        coat=None):
    key = name
    if key in _mats:
        return _mats[key]
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    nt = m.node_tree
    bsdf = next(n for n in nt.nodes if n.type == "BSDF_PRINCIPLED")
    bsdf.inputs["Metallic"].default_value = metal
    bsdf.inputs["Roughness"].default_value = rough
    bsdf.inputs["Base Color"].default_value = (*color, 1.0)
    if albedo:
        t = nt.nodes.new("ShaderNodeTexImage")
        t.image = img(albedo)
        nt.links.new(t.outputs["Color"], bsdf.inputs["Base Color"])
    if rough_map:
        t = nt.nodes.new("ShaderNodeTexImage")
        t.image = img(rough_map, srgb=False)
        nt.links.new(t.outputs["Color"], bsdf.inputs["Roughness"])
    if normal:
        t = nt.nodes.new("ShaderNodeTexImage")
        t.image = img(normal, srgb=False)
        nm = nt.nodes.new("ShaderNodeNormalMap")
        nt.links.new(t.outputs["Color"], nm.inputs["Color"])
        nt.links.new(nm.outputs["Normal"], bsdf.inputs["Normal"])
    if emissive:
        t = nt.nodes.new("ShaderNodeTexImage")
        t.image = img(emissive)
        nt.links.new(t.outputs["Color"], bsdf.inputs["Emission Color"])
        bsdf.inputs["Emission Strength"].default_value = 1.0
    if coat is not None:
        bsdf.inputs["Coat Weight"].default_value = coat
    _mats[key] = m
    return m


def M():  # material library (real finishes)
    return {
        "steel": mat("brushed_stainless", "steel_albedo", rough_map="steel_rough", metal=1.0),
        "serr": mat("serrated_jaw_face", "serration_albedo", rough=0.35, metal=1.0, normal="serration_normal"),
        "alu": mat("bead_blasted_aluminium", "alu_albedo", rough_map="alu_rough", metal=1.0),
        "powder": mat("powder_coat_grey", "powder_albedo", rough_map="powder_rough"),
        "black": mat("black_anodised", "anod_black_albedo", rough_map="anod_black_rough", metal=0.6),
        "blue": mat("blue_anodised", "anod_blue_albedo", rough=0.35, metal=0.6),
        "abs": mat("gauge_abs_housing", color=(0.10, 0.105, 0.115), rough=0.45),
        "red": mat("red_lever_coating", color=(0.62, 0.035, 0.03), rough=0.32, coat=0.4),
        "rubber": mat("black_rubber", color=(0.02, 0.02, 0.022), rough=0.85),
        "esd": mat("esd_mat_rubber", "esd_albedo", rough_map="esd_rough"),
        "pc": mat("natural_polycarbonate", color=(0.86, 0.83, 0.72), rough=0.18, coat=0.6),
        "ink": mat("marker_ink", color=(0.02, 0.02, 0.03), rough=0.5),
        "lcd": mat("gauge_lcd", "gauge_lcd", rough=0.1, emissive="gauge_lcd"),
        "keys": mat("gauge_keypad", "gauge_keys", rough=0.55),
        "panel": mat("stand_membrane_panel", "stand_panel", rough=0.5),
        "led": mat("travel_led", "travel_led", rough=0.1, emissive="travel_led"),
        "scale": mat("engraved_scale", "scale_rail", rough=0.4, metal=0.8),
        "label": mat("specimen_label", "specimen_id", rough=0.3),
    }


def finish(ob, m, bevel=0.0, segs=3, tile=0.064):
    """Box-project UVs (texel size uniform at ``tile`` metres per texture), bevel the edges, smooth by angle."""
    me = ob.data
    bm = bmesh.new()
    bm.from_mesh(me)
    uv = bm.loops.layers.uv.verify()
    for f in bm.faces:
        n = f.normal
        ax = max(range(3), key=lambda i: abs(n[i]))
        a, b = [i for i in range(3) if i != ax]
        for lp in f.loops:
            co = ob.matrix_world @ lp.vert.co
            lp[uv].uv = (co[b] / tile, co[a] / tile) if ax == 2 else (co[a] / tile, co[b] / tile)
    bm.to_mesh(me)
    bm.free()
    me.materials.append(m)
    if bevel > 0:
        md = ob.modifiers.new("bevel", "BEVEL")
        md.width, md.segments, md.limit_method = bevel, segs, "ANGLE"
        md.harden_normals = True
    bpy.context.view_layer.objects.active = ob
    ob.select_set(True)
    bpy.ops.object.shade_smooth_by_angle(angle=math.radians(35))
    return ob


def box(name, c, s, m, bevel=0.0, segs=3, tile=0.064):
    bpy.ops.mesh.primitive_cube_add(size=1.0, location=c)
    ob = bpy.context.active_object
    ob.name = name
    ob.scale = s
    bpy.ops.object.transform_apply(scale=True)
    return finish(ob, m, bevel=min(bevel, min(s) * 0.45) if bevel else 0.0, segs=segs, tile=tile)


def box_pitched(name, c, s, pitch, m, bevel=0.0, yaw=0.0):
    """Box turned about y by pitch, then about z by yaw (the labkit/mj_check (yaw, pitch) convention)."""
    bpy.ops.mesh.primitive_cube_add(size=1.0, location=c)
    ob = bpy.context.active_object
    ob.name = name
    ob.scale = s
    ob.rotation_mode = "ZYX"  # applied as Ry then Rz, as rz(yaw) @ ry(pitch)
    ob.rotation_euler = (0.0, pitch, yaw)
    bpy.ops.object.transform_apply(scale=True, rotation=True)
    return finish(ob, m, bevel=min(bevel, min(s) * 0.45) if bevel else 0.0)


def cyl(name, c, r, h, m, axis="Z", verts=48, bevel=0.0):
    rot = {"X": (0, math.pi / 2, 0), "Y": (math.pi / 2, 0, 0), "Z": (0, 0, 0)}[axis]
    bpy.ops.mesh.primitive_cylinder_add(vertices=verts, radius=r, depth=h, location=c, rotation=rot)
    ob = bpy.context.active_object
    ob.name = name
    bpy.ops.object.transform_apply(rotation=True)
    return finish(ob, m, bevel=bevel)


def decal(name, c, size, facing, m):
    """Textured rectangle with 0..1 UVs (the whole image) facing -x (front face) or +z (top)."""
    bpy.ops.mesh.primitive_plane_add(size=1.0, location=c)
    ob = bpy.context.active_object
    ob.name = name
    if facing == "-x":  # plane local x -> world y (reading left to right from the operator), local y -> world z
        ob.rotation_euler = (math.pi / 2, 0, -math.pi / 2)
        ob.scale = (size[0], size[1], 1)
    else:  # top, reading along world -y... local x -> world -y, local y -> world -x
        ob.rotation_euler = (0, 0, -math.pi / 2)
        ob.scale = (size[0], size[1], 1)
    bpy.ops.object.transform_apply(rotation=True, scale=True)
    ob.data.materials.append(m)
    return ob


def prism(name, pts, z0, z1, m, bevel=0.0, tile=0.064):
    """Extrude a closed x-y outline from z0 to z1 (concave outlines welcome)."""
    me = bpy.data.meshes.new(name)
    bm = bmesh.new()
    lo = [bm.verts.new((x, y, z0)) for x, y in pts]
    hi = [bm.verts.new((x, y, z1)) for x, y in pts]
    bm.faces.new(list(reversed(lo)))
    bm.faces.new(hi)
    n = len(pts)
    for i in range(n):
        j = (i + 1) % n
        bm.faces.new((lo[i], lo[j], hi[j], hi[i]))
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    bm.to_mesh(me)
    bm.free()
    ob = bpy.data.objects.new(name, me)
    bpy.context.collection.objects.link(ob)
    for o in bpy.context.selected_objects:
        o.select_set(False)
    return finish(ob, m, bevel=bevel, tile=tile)


def rrect(cx, cy, w, h, r, n=8):
    pts = []
    for qx, qy, a0 in ((1, 1, 0), (-1, 1, 90), (-1, -1, 180), (1, -1, 270)):
        ox, oy = cx + qx * (w / 2 - r), cy + qy * (h / 2 - r)
        for k in range(n + 1):
            a = math.radians(a0 + 90 * k / n)
            pts.append((ox + r * math.cos(a), oy + r * math.sin(a)))
    return pts


def export(name):
    path = OUT / f"{name}.usda"
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.wm.usd_export(filepath=str(path), selected_objects_only=True, export_materials=True,
                          generate_preview_surface=True, generate_materialx_network=False, export_textures_mode="KEEP",
                          relative_paths=True, triangulate_meshes=True, export_uvmaps=True, export_normals=True,
                          convert_orientation=False, root_prim_path="/root", export_lights=False,
                          export_cameras=False, evaluation_mode="RENDER", author_blender_name=False)
    print("exported", path)


# -- bodies ------------------------------------------------------------------------------------------------------------
def grip(k, s, dy, rod):
    """Wedge grip at the maker drawing size: housing 38.1 (x) x 62.2 (y) x 49.5 (z) with the thru slot open front and
    back (x); serrated lower jaw 19.1 mm wide; the upper jaw is its own body (upper_jaw_*.usda). dy shifts it along y
    (grip B at the crosshead's start); rod = the tie rod's far end (gauge or crosshead face)."""
    y_end = HOUSING_Y0 + HOUSING_B
    yh = s * (HOUSING_Y0 + HOUSING_B / 2) + dy
    box(f"grip_mount_{s}", (0, yh, (BED_TOP + HOUSING_BOT) / 2), (0.024, HOUSING_B - 0.010, HOUSING_BOT - BED_TOP),
        k["alu"], bevel=0.0015)
    box(f"housing_low_{s}", (LEAD_IN / 2, yh, (HOUSING_BOT + FLOOR_Z - 0.0002) / 2),
        (HOUSING_D - LEAD_IN, HOUSING_B, FLOOR_Z - 0.0002 - HOUSING_BOT), k["black"], bevel=0.0008)
    box_pitched(f"housing_lead_in_{s}", (-HOUSING_D / 2 + LEAD_IN / 2 + 0.004 * math.sqrt(0.5), yh,
                FLOOR_Z - 0.0002 - LEAD_IN / 2 - 0.004 * math.sqrt(0.5)), (LEAD_IN * math.sqrt(2), HOUSING_B, 0.008),
                -math.pi / 4, k["black"], bevel=0.0004)
    cheek = HOUSING_D / 2 - STEM[0] / 2 - 0.001
    for sx in (-1, 1):
        box(f"housing_up_{s}_{sx}", (sx * (HOUSING_D / 2 - cheek / 2), yh, (SLOT_CEIL + HOUSING_TOP) / 2),
            (cheek, HOUSING_B, HOUSING_TOP - SLOT_CEIL), k["black"], bevel=0.002)
    box(f"slot_end_{s}", (0, s * (STOP_Y + y_end) / 2 + dy, (FLOOR_Z + SLOT_CEIL) / 2),
        (HOUSING_D, y_end - STOP_Y, SLOT_CEIL - FLOOR_Z + 0.001), k["black"], bevel=0.0015)
    e = JAW_EDGE
    yj = s * (GRIP_FACE + e + STOP_Y) / 2 + dy
    box(f"lower_jaw_{s}", (0, yj, (FLOOR_Z - 0.006 + FLOOR_Z) / 2 - 0.00025), (JAW_W, STOP_Y - GRIP_FACE - e, 0.0055),
        k["steel"], bevel=0.001)
    box(f"lower_jaw_face_{s}", (0, yj, FLOOR_Z - 0.00025), (JAW_W - 0.001, STOP_Y - GRIP_FACE - e - 0.001, 0.0005),
        k["serr"], tile=0.064)
    box_pitched(f"lower_jaw_edge_{s}", (0, s * (GRIP_FACE + e / 2 + 0.0015 * math.sqrt(0.5)) + dy,
                FLOOR_Z - e / 2 - 0.0015 * math.sqrt(0.5)), (e * math.sqrt(2), JAW_W, 0.003), -s * math.pi / 4,
                k["steel"], bevel=0.0003, yaw=math.pi / 2)
    for dz in (-0.017, 0.017):  # socket-head cap screws on the housing front
        for ds in (-0.020, 0.020):
            cyl(f"screw_{s}_{dz}_{ds}", (-HOUSING_D / 2 - 0.0005, yh + ds, AXIS_Z + dz), 0.0028, 0.0012,
                k["steel"], axis="X", bevel=0.0004)
    box(f"tie_rod_{s}", (0, s * (y_end + rod) / 2 + dy, (BED_TOP + AXIS_Z) / 2), (0.024, rod - y_end, AXIS_Z - BED_TOP),
        k["alu"], bevel=0.002)


def stand():
    reset(); _mats.clear(); k = M()
    box("base", (0, 0, BED_TOP / 2), (0.130, 0.460, BED_TOP), k["powder"], bevel=0.004)
    box("base_top_plate", (0.008, 0, BED_TOP + 0.0006), (0.100, 0.440, 0.0012), k["alu"], bevel=0.0005)
    for y in (-0.205, 0.205):
        for x in (-0.055, 0.055):
            cyl(f"foot_{x}_{y}", (x, y, 0.003), 0.008, 0.006, k["rubber"], bevel=0.001)
    box("rail", (-0.050, 0, BED_TOP + 0.002), (0.012, 0.300, 0.004), k["alu"], bevel=0.0008)
    decal("rail_scale", (-0.050, 0, BED_TOP + 0.0041), (0.300, 0.010), "top", k["scale"])
    # force gauge (load-cell end): rounded ABS housing, LCD + keypad on the operator face, load shaft to the grip
    prism("gauge", rrect(0, GAUGE_Y, 0.090, 0.070, 0.012), BED_TOP, BED_TOP + 0.100, k["abs"], bevel=0.004)
    box("gauge_bezel", (-0.0452, GAUGE_Y, 0.098), (0.002, 0.056, 0.064), k["black"], bevel=0.0008)
    decal("gauge_lcd", (-0.0464, GAUGE_Y, 0.108), (0.050, 0.030), "-x", k["lcd"])
    decal("gauge_keys", (-0.0464, GAUGE_Y, 0.079), (0.050, 0.011), "-x", k["keys"])
    cyl("load_shaft", (0, 0.140, AXIS_Z - 0.004), 0.0045, 0.060, k["steel"], axis="Y")
    cyl("drive_screw", (0.035, -0.140, AXIS_Z - 0.004), 0.0045, 0.060, k["steel"], axis="Y")
    grip(k, +1, 0.0, 0.170)
    export("test_stand")


def crosshead():
    """Crosshead (stand joint, slide y) at its start, backed off XH_BACK, carrying grip B (-y)."""
    reset(); _mats.clear(); k = M()
    y = -CROSSHEAD_Y - XH_BACK
    box("crosshead", (0, y, BED_TOP + 0.050), (0.100, 0.040, 0.100), k["powder"], bevel=0.005)
    box("crosshead_cap", (0, y, BED_TOP + 0.1005), (0.096, 0.036, 0.003), k["black"], bevel=0.001)
    grip(k, -1, -XH_BACK, CROSSHEAD_Y - 0.020)
    export("crosshead")


def panel():
    """Stand control panel on its bracket at the +y frame end, keys up (the two jog keys are their own bodies)."""
    reset(); _mats.clear(); k = M()
    f = PANEL_TOP
    box("panel", (0, 0, f - PANEL_SIZE[2] / 2), PANEL_SIZE, k["abs"], bevel=0.003)
    box("bracket", (0, -0.034, 0.0225), (0.030, 0.022, 0.025), k["powder"], bevel=0.002)
    box("bracket_riser", (0, -0.026, (0.035 + f - PANEL_SIZE[2]) / 2), (0.030, 0.008, f - PANEL_SIZE[2] - 0.035),
        k["powder"], bevel=0.002)
    decal("panel_lcd", (0, -0.016, f + 0.0009), (0.050, 0.016), "top", k["lcd"])
    box("stop_key", (0, KEY_ROW, f + 0.0005), (KEY_SIZE[0], KEY_SIZE[1], 0.001), k["red"], bevel=0.0004)
    export("control_panel")


def lever(side, name, dy=0.0):
    """Release lever (stand joint, slide x): red handle behind the housing, two wedge rails beside the jaw stem."""
    reset(); _mats.clear(); k = M()
    yc = side * (GRIP_FACE + STOP_Y) / 2 + dy
    rise, t, xa = GAP_LIFTED + 0.001, 0.003, -0.001
    th = math.atan2(rise, RAMP_RUN)
    xm, zm = xa - RAMP_RUN / 2, RAMP_LOW + rise / 2
    box("lever", (PADDLE_X, yc, PADDLE_BOT + 0.011), (PADDLE[0], PADDLE[1] - 0.004, 0.022), k["red"], bevel=0.0018)
    box("lever_grip", (PADDLE_X, yc, PADDLE_BOT + 0.026), (PADDLE[0] + 0.0004, PADDLE[1], 0.008), k["rubber"],
        bevel=0.0025)
    for dy in (-RAMP_Y, RAMP_Y):
        y = yc + dy
        box_pitched(f"wedge_{dy}", (xm - math.sin(th) * t / 2, y, zm - math.cos(th) * t / 2),
                    (math.hypot(RAMP_RUN, rise), 0.005, t), th, k["steel"], bevel=0.0004)
        box(f"land_{dy}", (xa - RAMP_RUN - 0.005, y, RAMP_LOW + rise - t / 2), (0.010, 0.005, t), k["steel"],
            bevel=0.0004)
        box(f"rail_{dy}", ((xa + PADDLE_X) / 2, y, RAMP_LOW - 0.003), (PADDLE_X - xa, 0.005, 0.002), k["black"],
            bevel=0.0004)
    export(name)


def upper_jaw(side, name, dy=0.0):
    """Upper serrated jaw with its stem and the follower bar that rides the lever's wedges (jaw part, slide z)."""
    reset(); _mats.clear(); k = M()
    yc = side * (GRIP_FACE + STOP_Y) / 2 + dy
    bite = STOP_Y - GRIP_FACE - 0.001
    box("upper_jaw", (0, yc, FLOOR_Z + JAW_H / 2), (JAW_W, bite, JAW_H), k["steel"], bevel=0.0010)
    box("upper_jaw_face", (0, yc, FLOOR_Z + 0.00025), (JAW_W - 0.001, bite - 0.001, 0.0005), k["serr"], tile=0.064)
    box("stem", (0, yc, (JAW_TOP + FOLLOWER_Z) / 2), (*STEM, FOLLOWER_Z - JAW_TOP), k["steel"], bevel=0.0006)
    box("follower", (0, yc, FOLLOWER_Z + FOLLOWER[2] / 2), FOLLOWER, k["steel"], bevel=0.0008)
    export(name)


def dogbone():
    reset(); _mats.clear(); k = M()
    # ASTM D638 Type I: 165 overall, 19 tab, 13 x 57 narrow section, R76 fillets (tangent), 3.2 thick
    half = []
    ys = [0.0]
    yf = math.sqrt(0.076 ** 2 - (0.076 - 0.003) ** 2)  # fillet run 21.1 mm
    for i in range(1, 25):
        ys.append(0.0285 + yf * i / 24)
    for y in ys:
        d = y - 0.0285
        w = 0.0065 if d <= 0 else 0.0065 + 0.076 - math.sqrt(0.076 ** 2 - d ** 2)
        half.append((min(w, 0.0095), y))
    half += [(0.0095, 0.0805), (0.0089, 0.0820), (0.0075, 0.0825)]  # tab corner radius ~2 mm
    right = [(w, y) for w, y in half]
    outline = right + [(w, -y) for w, y in reversed(right)]
    # outline as (x, y): x = half width, mirror across x for the other side
    pts = [(w, y) for w, y in reversed(right)] + [(w, -y) for w, y in right[1:]]
    pts = pts + [(-x, y) for x, y in reversed(pts)]
    del outline
    prism("dogbone", pts, -SPEC_T / 2, SPEC_T / 2, k["pc"], bevel=0.0005)
    for y in (-0.025, 0.025):  # gauge-length marks (50 mm) in marker ink
        box(f"gauge_mark_{y}", (0, y, SPEC_T / 2 + 0.00006), (0.0125, 0.0007, 0.0001), k["ink"])
    decal("specimen_id", (0, -0.068, SPEC_T / 2 + 0.00008), (0.024, 0.0048), "top", k["label"])
    export("dogbone")


def tray():
    reset(); _mats.clear(); k = M()
    prism("plate", rrect(0.00225, 0, 0.0445, 0.200, 0.004), 0.0, 0.003, k["blue"], bevel=0.0006)
    for s in (1, -1):
        prism(f"rest_{s}", rrect(0, s * REST_Y, 0.030, 0.022, 0.003), 0.003, REST_H - MAT_H, k["blue"], bevel=0.0008)
        box(f"stop_{s}", (STOP_X + 0.002, s * (REST_Y - 0.005), (0.003 + STOP_TOP) / 2), (0.004, 0.010, STOP_TOP - 0.003),
            k["blue"], bevel=0.0006)
    export("specimen_tray")


def bench_mat():
    reset(); _mats.clear(); k = M()
    prism("mat", rrect(0, 0, 0.420, 0.600, 0.020), -MAT_H / 2, MAT_H / 2, k["esd"], bevel=0.001, tile=0.2)
    cyl("ground_snap", (0.185, 0.275, MAT_H / 2 + 0.001), 0.006, 0.002, k["steel"], bevel=0.0006)
    export("bench_mat")


stand()
crosshead()
panel()
lever(+1, "lever_l")
lever(-1, "lever_r", -XH_BACK)
upper_jaw(+1, "upper_jaw_l")
upper_jaw(-1, "upper_jaw_r", -XH_BACK)
dogbone()
tray()
bench_mat()
