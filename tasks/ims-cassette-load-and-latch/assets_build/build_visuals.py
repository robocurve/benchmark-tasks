"""Author the visual meshes of the sterilization cassette, its lid and clip, the holder, the tray and the instruments in
Blender; one USD per rigid body.

Run: blender -b --factory-startup --python assets_build/build_visuals.py -- <pkg_dir>

Every mesh is in its body's part frame (metres, Z up): cassette.usdc (holder, perforated base, blue silicone rails),
lid.usdc (perforated lid with its blue press pads, at the closed position), clip.usdc (red latch clip, open), tray.usdc,
and one handle mesh per instrument (inst_*.usdc, centred on the handle, axis along Y). Dimensions follow the frozen
constants of ims_cassette_load_and_latch.py (76 x 203 x 32 mm closed, catalog); no maker marks are applied.
"""
import math
import sys
from pathlib import Path

import bpy

argv = sys.argv[sys.argv.index("--") + 1:]
PKG = Path(argv[0]).resolve()
OUT = PKG / "assets" / "visual"
OUT.mkdir(parents=True, exist_ok=True)
MM = 0.001

# frozen task constants (mm)
HOLD, CX, CY, BASE_H, LID_H, WALL = 6.0, 76.0, 203.0, 17.0, 15.0, 1.5
PITCH, N = 12.7, 5
RAIL_Y, RAIL_T, NOTCH_W, FLOOR_Z, TOOTH_Z = 60.0, 8.0, 10.5, 4.0, 16.0
TOOTH_W = PITCH - NOTCH_W
HR, HL, TR, TL = 4.75, 130.0, 1.5, 17.5
PAD_Z, PAD_Y = 14.3, 40.0
CLIP_L, CLIP_W, CLIP_T, CLIP_OUT = 23.0, 12.0, 2.0, 2.5
TAB = (1.5, 1.2)
JAW_R, JAW_IN, JAW_Z = 1.0, 2.5, 14.6
WORK = (0, 1, 2, 3, 4)  # every notch has its sprung lips
TRAY_W, TRAY_L = 64.0, 190.0

bpy.ops.wm.read_factory_settings(use_empty=True)
scene = bpy.context.scene


def srgb(c):
    return tuple(((x / 255.0 + 0.055) / 1.055) ** 2.4 for x in c)


def material(name, color, rough, metal=0.0):
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    bsdf = next(n for n in m.node_tree.nodes if n.type == "BSDF_PRINCIPLED")
    bsdf.inputs["Base Color"].default_value = (*color, 1.0)
    bsdf.inputs["Roughness"].default_value = rough
    bsdf.inputs["Metallic"].default_value = metal
    return m


SS = material("brushed_stainless", srgb((190, 192, 195)), 0.32, metal=1.0)
SS_DARK = material("stainless_shadow", srgb((70, 72, 76)), 0.5, metal=0.8)
SILICONE = material("blue_silicone", srgb((38, 92, 190)), 0.55)
RED = material("red_latch_clip", srgb((196, 40, 34)), 0.4)
HOLDER = material("black_anodised_holder", srgb((28, 28, 30)), 0.45, metal=0.6)
TRAY = material("white_resin_tray", srgb((226, 228, 230)), 0.6)
HANDLE = material("satin_stainless_handle", srgb((172, 175, 180)), 0.38, metal=1.0)
COLOR = {1: srgb((60, 150, 80)), 2: srgb((210, 170, 40)), 3: srgb((170, 60, 160)), 4: srgb((40, 150, 200)), 5: srgb((220, 110, 40))}

GROUPS: dict[str, list] = {k: [] for k in ("cassette", "lid", "clip", "tray", "inst1", "inst2", "inst3", "inst4", "inst5",
                                          *(f"jaw{i}{s}" for i in range(1, 6) for s in "lr"))}


def finish(obj, group, mat, bevel=0.0, smooth=False):
    obj.data.materials.clear()
    obj.data.materials.append(mat)
    if bevel:
        mod = obj.modifiers.new("bevel", "BEVEL")
        mod.width = bevel * MM
        mod.segments = 2
        mod.limit_method = "ANGLE"
        bpy.context.view_layer.objects.active = obj
        bpy.ops.object.modifier_apply(modifier=mod.name)
    for p in obj.data.polygons:
        p.use_smooth = smooth
    GROUPS[group].append(obj)
    return obj


def box(name, center, size, group, mat, bevel=0.3):
    bpy.ops.mesh.primitive_cube_add(size=1.0, location=tuple(c * MM for c in center))
    obj = bpy.context.active_object
    obj.name = name
    obj.scale = tuple(s * MM for s in size)
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    return finish(obj, group, mat, bevel)


def cyl(name, center, r, h, group, mat, axis="Z", seg=32, bevel=0.0):
    bpy.ops.mesh.primitive_cylinder_add(vertices=seg, radius=r * MM, depth=h * MM, location=tuple(c * MM for c in center))
    obj = bpy.context.active_object
    obj.name = name
    if axis == "X":
        obj.rotation_euler[1] = math.pi / 2
    elif axis == "Y":
        obj.rotation_euler[0] = math.pi / 2
    bpy.ops.object.transform_apply(location=False, rotation=True, scale=False)
    return finish(obj, group, mat, bevel, smooth=True)


def perforate(target, axis, centers, r=1.6):
    """Cut a pattern of round holes through a sheet (stamped perforation)."""
    cutters = []
    for c in centers:
        bpy.ops.mesh.primitive_cylinder_add(vertices=12, radius=r * MM, depth=6 * MM, location=tuple(v * MM for v in c))
        o = bpy.context.active_object
        if axis == "X":
            o.rotation_euler[1] = math.pi / 2
        elif axis == "Y":
            o.rotation_euler[0] = math.pi / 2
        cutters.append(o)
    bpy.ops.object.select_all(action="DESELECT")
    for o in cutters:
        o.select_set(True)
    bpy.context.view_layer.objects.active = cutters[0]
    bpy.ops.object.join()
    cutter = bpy.context.active_object
    mod = target.modifiers.new("perf", "BOOLEAN")
    mod.operation = "DIFFERENCE"
    mod.object = cutter
    bpy.context.view_layer.objects.active = target
    bpy.ops.object.modifier_apply(modifier=mod.name)
    bpy.data.objects.remove(cutter, do_unlink=True)


def grid(u0, u1, v0, v1, step):
    us = [u0 + step / 2 + k * step for k in range(int((u1 - u0) // step))]
    vs = [v0 + step / 2 + k * step for k in range(int((v1 - v0) // step))]
    return [(u, v) for u in us for v in vs]


# -- cassette body (frame: z = 0 at the cassette floor underside, holder below) -----------------------------------------
hx, hy = CX / 2, CY / 2
box("holder_plate", (0, 0, -HOLD / 2), (CX + 24, CY + 24, HOLD), "cassette", HOLDER, 1.0)
for sx in (1, -1):
    for sy in (1, -1):
        box(f"holder_corner_{sx}{sy}", (sx * (hx + 4), sy * (hy + 4), 4), (6, 6, 8), "cassette", HOLDER, 0.6)
floor = box("floor", (0, 0, 0.5), (CX, CY, 1.0), "cassette", SS, 0.2)
perforate(floor, "Z", [(u, v, 0.5) for u, v in grid(-hx + 4, hx - 4, -hy + 6, hy - 6, 7.0)
                       if abs(abs(v) - RAIL_Y) > 7])
for sx in (1, -1):
    w = box(f"wall_x{sx}", (sx * (hx - WALL / 2), 0, BASE_H / 2), (WALL, CY, BASE_H), "cassette", SS, 0.3)
    perforate(w, "X", [(sx * (hx - WALL / 2), v, u) for u, v in grid(3, BASE_H - 3, -hy + 8, hy - 8, 6.0)])
for sy in (1, -1):
    box(f"wall_y{sy}", (0, sy * (hy - WALL / 2), BASE_H / 2), (CX, WALL, BASE_H), "cassette", SS, 0.3)
    box(f"rim_y{sy}", (0, sy * (hy - WALL / 2), BASE_H - 0.4), (CX + 0.6, WALL + 0.6, 0.8), "cassette", SS, 0.2)
box("latch_lip", (-hx - 0.6, 0, BASE_H - 4), (1.2, 24, 1.2), "cassette", SS, 0.2)
for k in (-1, 1):  # hinge knuckles on the back edge
    cyl(f"hinge_{k}", (hx + 0.8, k * 70, BASE_H), 1.6, 18, "cassette", SS, axis="Y", seg=20)
edges = [-(N / 2) * PITCH + k * PITCH for k in range(N + 1)]
for sy in (1, -1):
    y = sy * RAIL_Y
    box(f"rail_base_{sy}", (0, y, FLOOR_Z / 2 + 0.5), (CX - 2 * WALL, RAIL_T, FLOOR_Z - 1), "cassette", SILICONE, 0.6)
    jaw_teeth = {round((i - 2) * PITCH + PITCH / 2, 3) for i in WORK}
    for xc in edges:
        zt = (JAW_Z - JAW_R - 0.2 - TOOTH_W / 2) if round(xc, 3) in jaw_teeth else (TOOTH_Z - TOOTH_W / 2)  # cut down under its jaw
        box(f"tooth_{sy}_{xc:+.1f}", (xc, y, (FLOOR_Z + zt) / 2), (TOOTH_W, RAIL_T, zt - FLOOR_Z), "cassette", SILICONE, 0.0)
        cyl(f"tooth_top_{sy}_{xc:+.1f}", (xc, y, zt), TOOTH_W / 2, RAIL_T, "cassette", SILICONE, axis="Y", seg=16)
    for i in range(N):  # sprung silicone grip lip at every notch mouth
        xj = (i - 2) * PITCH + NOTCH_W / 2 - JAW_IN + JAW_R
        g = f"jaw{i + 1}{'l' if sy > 0 else 'r'}" if i in WORK else "cassette"
        cyl(f"jaw_{sy}_{i}", (xj, y, JAW_Z), JAW_R, RAIL_T, g, SILICONE, axis="Y", seg=16)
    for xo in (-1, 1):
        x0, x1 = xo * (N / 2 * PITCH + TOOTH_W / 2), xo * (hx - WALL)
        box(f"rail_end_{sy}{xo}", ((x0 + x1) / 2, y, (FLOOR_Z + TOOTH_Z) / 2), (abs(x1 - x0), RAIL_T, TOOTH_Z - FLOOR_Z),
            "cassette", SILICONE, 0.6)

# -- lid (closed position) --------------------------------------------------------------------------------------------
zt = BASE_H + LID_H
top = box("lid_top", (0, 0, zt - 0.5), (CX, CY, 1.0), "lid", SS, 0.3)
perforate(top, "Z", [(u, v, zt - 0.5) for u, v in grid(-hx + 4, hx - 4, -hy + 6, hy - 6, 7.0)])
for sx in (1, -1):
    w = box(f"lid_wall_x{sx}", (sx * (hx - WALL / 2), 0, BASE_H + LID_H / 2 + 0.2), (WALL, CY, LID_H - 0.4), "lid", SS, 0.3)
    perforate(w, "X", [(sx * (hx - WALL / 2), v, u) for u, v in grid(BASE_H + 3, zt - 3, -hy + 8, hy - 8, 6.0)])
for sy in (1, -1):
    box(f"lid_wall_y{sy}", (0, sy * (hy - WALL / 2), BASE_H + LID_H / 2 + 0.2), (CX, WALL, LID_H - 0.4), "lid", SS, 0.3)
box("lid_pad", (0, 0, (PAD_Z + zt - 1) / 2), (CX - 2 * WALL - 2, 2 * PAD_Y, zt - 1 - PAD_Z), "lid", SILICONE, 1.0)
box("clip_mount", (-hx - CLIP_OUT / 2, 0, zt + 0.2), (CLIP_OUT + 1.0, CLIP_W + 4, 2.0), "lid", SS_DARK, 0.3)

# -- clip (open) ------------------------------------------------------------------------------------------------------
xp, zp = -hx - CLIP_OUT, zt + 1.0  # pivot on the lid's top front edge; clip drawn at joint 0 (horizontal, pointing -X)
box("clip", (xp - CLIP_L / 2, 0, zp), (CLIP_L, CLIP_W, CLIP_T), "clip", RED, 0.5)
box("clip_tab", (xp - CLIP_L + TAB[0] / 2, 0, zp - CLIP_T / 2 - TAB[1] / 2), (TAB[0], CLIP_W, TAB[1]), "clip", RED, 0.3)
cyl("clip_pin", (xp, 0, zp), 1.1, CLIP_W + 2, "clip", SS, axis="Y", seg=16)

# -- tray (frame z = 0 on the bench) ----------------------------------------------------------------------------------
box("tray", (0, 0, HOLD / 2), (TRAY_W, TRAY_L, HOLD), "tray", TRAY, 1.5)
box("tray_lip", (TRAY_W / 2, 0, HOLD + 2), (3, TRAY_L, 4), "tray", TRAY, 1.0)
for _k, _s in enumerate((18.0, -18.0)):  # cradle ridges, tray frame (spares at cassette x -60 / -96, tray centre -78)
    for _sx in (-1, 1):
        for _sy in (-1, 1):
            box(f"tray_cradle_{_k}_{_sx}_{_sy}", (_s + _sx * (4.75 + 5.5 + 0.75), _sy * 60.0, HOLD + 1.5), (1.5, 24.0, 3.0), "tray", TRAY, 0.3)

# -- instruments (frame: handle centre, axis Y) -----------------------------------------------------------------------
for i in range(1, 6):
    g = f"inst{i}"
    cyl(f"{g}_handle", (0, 0, 0), HR, HL, g, HANDLE, axis="Y", seg=24)
    for k in range(-5, 6):  # knurled grip rings
        cyl(f"{g}_ring{k}", (0, k * 6.0, 0), HR + 0.15, 1.2, g, SS_DARK, axis="Y", seg=24)
    cyl(f"{g}_band", (0, HL / 2 - 6, 0), HR + 0.2, 3.0, g, material(f"band{i}", COLOR[i], 0.5), axis="Y", seg=24)
    for s in (-1, 1):
        cyl(f"{g}_cone{s}", (0, s * (HL / 2 + 4), 0), 2.6, 8, g, HANDLE, axis="Y", seg=20)
        cyl(f"{g}_shank{s}", (0, s * (HL / 2 + 8 + (TL - 8) / 2), 0), TR, TL - 8, g, HANDLE, axis="Y", seg=12)
    if i == 2:  # mouth mirror head on one end
        cyl(f"{g}_mirror", (0, HL / 2 + TL + 2, 0), 11.0, 1.5, g, material("mirror_face", (0.9, 0.92, 0.95), 0.05, metal=1.0),
            axis="Z", seg=32)
    elif i == 4:  # explorer hook
        cyl(f"{g}_hook", (0, HL / 2 + TL + 3, 2.5), 0.6, 7, g, HANDLE, axis="Z", seg=10)

# -- export -------------------------------------------------------------------------------------------------------
for group, objs in GROUPS.items():
    bpy.ops.object.select_all(action="DESELECT")
    for o in objs:
        o.select_set(True)
    bpy.ops.wm.usd_export(filepath=str(OUT / f"{group}.usdc"), selected_objects_only=True, export_materials=True,
                          generate_preview_surface=True, relative_paths=True, export_uvmaps=True, export_normals=True,
                          root_prim_path="/visual", convert_orientation=False, export_animation=False)
    print("exported", group, len(objs))
