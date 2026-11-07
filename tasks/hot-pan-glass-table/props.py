"""The props of the hot-pan task, as USD text: the hob with its control panel, the small steel pan with two loop handles, the glass table and the cork pad.
Each `*_usda()` returns one file. Sizes are in metres; each prop's origin is noted on its function.

Sizes follow common products: a 14 cm stainless casserole (6 cm walls, 1 kg), a 24 cm single-zone ceramic hob, a small glass-topped stand with
10 mm tempered glass, and a 16 cm trivet in 20 mm cork.
"""

from __future__ import annotations

import numpy as np
from kitchenkit import Look, Mesh, _merge, box_mesh, col_box, col_cylinder, flat_disc, flat_rect, lathe, mesh_usda, physics_material, srgb, swept, usda_file

STEEL = Look("steel", metallic=1.0, roughness=1.0, color_map="steel_color.jpg", rough_map="steel_rough.jpg", normal_map="steel_normal.jpg", tint=(1.05, 1.05, 1.07))
# Steel darkened by heat: its colour dimmed and less mirror-like, so that the room's light does not wash out the glow.
HOT_OUT = Look("hot_out", metallic=0.5, roughness=0.55, color_map="steel_color.jpg", normal_map="steel_normal.jpg", tint=(0.42, 0.39, 0.37), emissive_map="pan_glow_up.jpg", emissive_gain=1.6)
HOT_IN = Look("hot_in", metallic=0.5, roughness=0.55, color_map="steel_color.jpg", normal_map="steel_normal.jpg", tint=(0.42, 0.39, 0.37), emissive_map="pan_glow_in.jpg", emissive_gain=1.4)
DISPLAY_ON = Look("display_on", color=(0.0, 0.0, 0.0), roughness=0.25, emissive_map="display_on.jpg", emissive_gain=1.6)
DISPLAY_HOT = Look("display_hot", color=(0.0, 0.0, 0.0), roughness=0.25, emissive_map="display_hot.jpg", emissive_gain=1.6)
BLACK_GLASS = Look("black_glass", color=srgb(0.03, 0.03, 0.035), roughness=0.08)
RUBBER = Look("rubber", color=srgb(0.05, 0.05, 0.055), roughness=0.8)
DARK = Look("dark", color=srgb(0.04, 0.04, 0.04), roughness=0.6)
GLOW_ON = Look("glow_on", color=(0.2, 0.01, 0.0), roughness=0.5, emissive=(1.3, 0.07, 0.004))
GLASS = Look("glass", color=srgb(0.93, 0.97, 0.96), roughness=0.02, opacity=0.08)
GLASS_EDGE = Look("glass_edge", color=srgb(0.55, 0.78, 0.70), roughness=0.08)
CRACK = Look("crack", color=srgb(0.95, 0.97, 0.98), roughness=0.3, emissive=(0.5, 0.5, 0.5))
CORK = Look("cork", roughness=1.0, color_map="cork_color.jpg", rough_map="cork_rough.jpg", normal_map="cork_normal.jpg")
WOOD = Look("wood", roughness=1.0, color_map="wood_color.jpg", rough_map="wood_rough.jpg", normal_map="wood_normal.jpg")

HOB = (0.24, 0.24, 0.03)
RING = (0.078, 0.088)
# The control panel on the hob's front: a wedge, in the hob's frame. x from the front edge to the hob, z at both edges, half its width across y.
CONSOLE_X, CONSOLE_Z, CONSOLE_HALF_W = (-0.17, -0.12), (0.012, 0.045), 0.08
SCREEN_W, SCREEN_H, SCREEN_Y = 0.104, 0.04, 0.015  # the LED screen on the panel's slope, its middle this far to the left of the panel's
PAN_R, PAN_H, PAN_WALL, PAN_FLOOR = 0.07, 0.06, 0.005, 0.006
PAN_BASE_R = 0.065  # the flat of the pan's underside
LOOP_Z, LOOP_H = 0.045, 0.012  # the handles' middle above the underside, and their height
LOOP_W, LOOP_DEPTH, LOOP_BAR = 0.06, 0.03, 0.008  # across x, out from the wall, the thickness of the strap
GRIP_Y = PAN_R + 0.015  # a TCP this far from the pan's axis lies over the handle's middle
TABLE = (0.32, 0.30, 0.01)  # the glass top: x, y, thickness
TABLE_LEG = 0.04
TABLE_TOP_Z = TABLE_LEG + TABLE[2]
PAD = 0.16
LUG = (0.025, 0.04, 0.03)  # along x (out from the pad's edge), y, z
CORK_T = 0.02
CRACK_LINES = 8


# -- the hob -------------------------------------------------------------------------------


def _quad(name: str, look: str, corners, tile: float | None = None, visible: bool = True) -> Mesh:
    """One flat quad, given anticlockwise as seen from its front. Its texture runs corner to corner, or at `tile` metres per repeat."""
    P = np.asarray(corners, dtype=float)
    n = np.cross(P[1] - P[0], P[3] - P[0])
    n /= np.linalg.norm(n)
    if tile is None:
        U = np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]])
    else:
        e0, e1 = (P[1] - P[0]) / np.linalg.norm(P[1] - P[0]), n
        U = np.stack([(P - P[0]) @ e0, (P - P[0]) @ np.cross(e1, e0)], axis=-1) / tile
    return _merge(name, look, [(P, np.tile(n, (4, 1)), U, np.array([[0, 1, 2, 3]]))], visible)


def _console() -> tuple[list[Mesh], np.ndarray]:
    """The control panel's steel wedge, its LED screen in both states and two buttons. Also the wedge's eight corners, for its collider."""
    (xf, xb), (zf, zb), w = CONSOLE_X, CONSOLE_Z, CONSOLE_HALF_W
    fl, fr, bl, br = (xf, w, 0.0), (xf, -w, 0.0), (xb, w, 0.0), (xb, -w, 0.0)
    tfl, tfr, tbl, tbr = (xf, w, zf), (xf, -w, zf), (xb, w, zb), (xb, -w, zb)
    meshes = [_quad("panel_slope", "steel", [tfl, tfr, tbr, tbl], tile=0.25), _quad("panel_front", "steel", [fl, fr, tfr, tfl], tile=0.25),
              _quad("panel_back", "steel", [br, bl, tbl, tbr], tile=0.25), _quad("panel_left", "steel", [bl, fl, tfl, tbl], tile=0.25),
              _quad("panel_right", "steel", [fr, br, tbr, tfr], tile=0.25)]
    along = np.array([xb - xf, 0.0, zb - zf])
    length = float(np.linalg.norm(along))
    along /= length
    normal = np.array([-along[2], 0.0, along[0]])
    mid = np.array([xf, 0.0, zf]) + 0.5 * length * along

    def on_slope(name, look, y, half_w, half_h, lift, visible=True):
        c = mid + np.array([0.0, y, 0.0]) + lift * normal
        across = np.array([0.0, 1.0, 0.0])
        return _quad(name, look, [c + half_w * across - half_h * along, c - half_w * across - half_h * along, c - half_w * across + half_h * along, c + half_w * across + half_h * along], visible=visible)

    meshes += [on_slope("bezel", "dark", SCREEN_Y, 0.5 * SCREEN_W + 0.004, 0.5 * SCREEN_H + 0.004, 0.0003),
               on_slope("display_on", "display_on", SCREEN_Y, 0.5 * SCREEN_W, 0.5 * SCREEN_H, 0.0006),
               on_slope("display_hot", "display_hot", SCREEN_Y, 0.5 * SCREEN_W, 0.5 * SCREEN_H, 0.0006, visible=False)]
    for k, s in enumerate((-0.012, 0.012)):
        c = mid + np.array([0.0, -0.058, 0.0]) + s * along
        meshes.append(swept(f"panel_button_{k}", "dark", [c, c + 0.002 * normal], 0.008, 0.008, side=(0.0, 1.0, 0.0), n=20, power=2.0))
    return meshes, np.array([fl, fr, bl, br, tfl, tfr, tbl, tbr])


def _col_hull(name: str, corners: np.ndarray, material: str) -> str:
    """A box-like solid as one convex collider, from its eight corners in the order of `_console`."""
    points = ",".join(f"({x:.6f},{y:.6f},{z:.6f})" for x, y, z in corners)
    faces = [0, 1, 3, 2, 4, 5, 7, 6, 0, 1, 5, 4, 2, 3, 7, 6, 0, 2, 6, 4, 1, 3, 7, 5]
    return f"""
        def Mesh "{name}" (
            prepend apiSchemas = ["PhysicsCollisionAPI", "PhysicsMeshCollisionAPI", "MaterialBindingAPI"]
        )
        {{
            rel material:binding:physics = <{material}>
            token visibility = "invisible"
            uniform token physics:approximation = "convexHull"
            uniform token subdivisionScheme = "none"
            int[] faceVertexCounts = [4, 4, 4, 4, 4, 4]
            int[] faceVertexIndices = [{",".join(map(str, faces))}]
            point3f[] points = [{points}]
        }}"""


def stove_usda() -> str:
    """Origin: the hob's centre on the table."""
    root = "Stove"
    w, d, h = HOB
    visuals = [
        box_mesh("frame", "steel", (0, 0, 0.5 * (h - 0.001)), (w, d, h - 0.001), tile=0.25),
        box_mesh("ceramic", "black_glass", (0, 0, h - 0.0005), (w - 0.02, d - 0.02, 0.001), tile=0.25),
        flat_disc("glow_on", "glow_on", RING[0], RING[1], h + 0.0003),
    ]
    for sx in (-1, 1):
        for sy in (-1, 1):
            visuals.append(lathe(f"foot_{sx + 1}{sy + 1}", "rubber", [[(0.012, 0.0), (0.012, 0.002)]], n=16, center=(sx * 0.10, sy * 0.10, 0.0)))
    console, corners = _console()
    mat = f"/{root}/PhysicsMaterial"
    body = (physics_material(root, (0.9, 0.8)) + col_box("hob_body", (0, 0, 0.5 * h), HOB, material=mat) + _col_hull("panel_body", corners, mat)
            + "".join(mesh_usda(m, root) for m in visuals + console))
    return usda_file(root, body, [STEEL, BLACK_GLASS, GLOW_ON, RUBBER, DARK, DISPLAY_ON, DISPLAY_HOT])


# -- the pan ---------------------------------------------------------------------------------


def _loop(name: str, sign: float) -> list[Mesh]:
    """One loop handle: a strap out of the wall, across and back, on the +y side for sign 1."""
    w, dp, t, z, h = 0.5 * LOOP_W, LOOP_DEPTH, 0.5 * LOOP_BAR, LOOP_Z, 0.5 * LOOP_H
    y0 = np.sqrt(PAN_R ** 2 - (w - t) ** 2) - 0.002
    y1 = PAN_R + dp
    bar = y1 - t  # the strap's middle, matching its collider
    path = [(-w + t, sign * y0, z), (-w + t, sign * (bar - 0.008), z), (-w + t + 0.002, sign * (bar - 0.002), z), (-w + t + 0.008, sign * bar, z), (-0.012, sign * bar, z), (0.012, sign * bar, z),
            (w - t - 0.008, sign * bar, z), (w - t - 0.002, sign * (bar - 0.002), z), (w - t, sign * (bar - 0.008), z), (w - t, sign * y0, z)]
    strap = _strap(name, path, t, h)
    rivets = [swept(f"{name}_rivet_{k}", "steel", [(sx * (w - t), sign * (PAN_R - PAN_WALL + 0.0015), z), (sx * (w - t), sign * (PAN_R - PAN_WALL), z)], 0.0035, 0.0035, n=12, tile=0.09, power=2.0) for k, sx in enumerate((-1, 1))]
    return [strap, *rivets]


def _strap(name: str, path, half_t: float, half_h: float) -> Mesh:
    """A flat strap along a path in a horizontal plane: its thickness in the plane, its height along z."""
    return swept(name, "steel", path, half_h, half_t, side=(0.0, 0.0, 1.0), n=16, tile=0.09, power=3.0)


def _profile_length(points) -> float:
    return float(np.linalg.norm(np.diff(np.asarray(points), axis=0), axis=1).sum())


def pan_usda() -> str:
    """Origin: the middle of the pan's underside."""
    root = "Pan"
    r, h, t, fl = PAN_R, PAN_H, PAN_WALL, PAN_FLOOR
    roll = 0.002
    # The steel glows with heat: from the base up the wall outside, and from the rim down to the floor inside. The rim and the handles stay plain steel.
    body = lathe("pan_body", "steel", [
        [(1e-4, 0.0), (PAN_BASE_R, 0.0), (r - 0.001, 0.002)],
        [(r, h - roll), (r + 0.0012, h - 0.8 * roll), (r + 0.0015, h - 0.2 * roll), (r + 0.0005, h), (r - 0.001, h), (r - t + 0.002, h - 0.001), (r - t, h - 0.004)],
    ], n=96, tile=0.09)
    wall_out = [(r - 0.001, 0.002), (r, 0.007), (r, h - roll)]
    wall_in = [(r - t, h - 0.004), (r - t, fl + 0.004), (r - t - 0.004, fl), (1e-4, fl)]
    hot_out = lathe("pan_hot_out", "hot_out", [wall_out], n=96, tile=_profile_length(wall_out))
    hot_in = lathe("pan_hot_in", "hot_in", [wall_in], n=96, tile=_profile_length(wall_in))
    visuals = [body, hot_out, hot_in, *_loop("loop_left", 1.0), *_loop("loop_right", -1.0)]
    cols = col_cylinder("floor", (0, 0, 0.5 * fl), PAN_BASE_R + 0.001, fl)
    n = 20
    arc = 2 * r * np.tan(np.pi / n) + 0.001
    for k in range(n):
        a = 2 * np.pi * k / n
        c = (r - 0.5 * t) * np.array([np.cos(a), np.sin(a)])
        cols += col_box(f"wall_{k:02d}", (c[0], c[1], 0.5 * (fl + h)), (t, arc, h - fl), orient_wxyz=(np.cos(a / 2), 0.0, 0.0, np.sin(a / 2)))
    w, y1 = 0.5 * LOOP_W, r + LOOP_DEPTH
    for sign, tag in ((1.0, "l"), (-1.0, "r")):
        y0 = np.sqrt(r ** 2 - (w - 0.5 * LOOP_BAR) ** 2) - 0.002
        ym = 0.5 * (y0 + y1)
        cols += col_box(f"loop_{tag}_a", (-w + 0.5 * LOOP_BAR, sign * ym, LOOP_Z), (LOOP_BAR, y1 - y0, LOOP_H))
        cols += col_box(f"loop_{tag}_b", (w - 0.5 * LOOP_BAR, sign * ym, LOOP_Z), (LOOP_BAR, y1 - y0, LOOP_H))
        cols += col_box(f"loop_{tag}_c", (0.0, sign * (y1 - 0.5 * LOOP_BAR), LOOP_Z), (LOOP_W, LOOP_BAR, LOOP_H))
    return usda_file(root, cols + "".join(mesh_usda(m, root) for m in visuals), [STEEL, HOT_OUT, HOT_IN], root_api='"PhysicsRigidBodyAPI", "PhysicsMassAPI"')


# -- the glass table -------------------------------------------------------------------------


def _crack_lines() -> list[Mesh]:
    """Cracks fanning out from the origin in the plane z = 0, as thin flat strips; hidden until the pan touches the glass."""
    rng = np.random.default_rng(7)
    out = []
    for k, a in enumerate(np.linspace(0.0, 2 * np.pi, CRACK_LINES + 1)[:-1] + rng.uniform(-0.2, 0.2, CRACK_LINES)):
        length = rng.uniform(0.09, 0.15)
        pts = [np.zeros(3)]
        ang = a
        steps = 7
        for _ in range(steps):
            ang += rng.uniform(-0.28, 0.28)
            pts.append(pts[-1] + length / steps * np.array([np.cos(ang), np.sin(ang), 0.0]))
        pts = np.array(pts)
        w = np.linspace(0.0013, 0.0005, len(pts))
        out.append(swept(f"crack_{k}", "crack", pts, w, 0.0002, side=(-np.sin(a), np.cos(a), 0.0), n=8, power=2.0))
    return out


def glass_table_usda() -> str:
    """Origin: the centre of the glass top's footprint on the table."""
    root = "GlassTable"
    x, y, t = TABLE
    z0, z1 = TABLE_LEG, TABLE_LEG + t
    zm = 0.5 * (z0 + z1)
    e = 0.0015
    visuals = [
        flat_rect("top", "glass", (0, 0, z1), (x, y)),
        box_mesh("edge_x0", "glass_edge", (-0.5 * x + 0.5 * e, 0, zm), (e, y, t)),
        box_mesh("edge_x1", "glass_edge", (0.5 * x - 0.5 * e, 0, zm), (e, y, t)),
        box_mesh("edge_y0", "glass_edge", (0, -0.5 * y + 0.5 * e, zm), (x, e, t)),
        box_mesh("edge_y1", "glass_edge", (0, 0.5 * y - 0.5 * e, zm), (x, e, t)),
    ]
    legs, cols = [], ""
    mat = f"/{root}/PhysicsMaterial"
    for sx in (-1, 1):
        for sy in (-1, 1):
            cx, cy = sx * (0.5 * x - 0.025), sy * (0.5 * y - 0.025)
            legs.append(lathe(f"leg_{sx + 1}{sy + 1}", "steel", [[(0.009, 0.0), (0.009, z0)]], n=20, tile=0.09, center=(cx, cy, 0.0)))
            legs.append(lathe(f"pad_{sx + 1}{sy + 1}", "dark", [[(0.012, z0 - 0.002), (0.012, z0)], [(0.012, z0), (1e-4, z0)]], n=20, center=(cx, cy, 0.0)))
            cols += col_cylinder(f"leg_col_{sx + 1}{sy + 1}", (cx, cy, 0.5 * z0), 0.009, z0)
    cols += col_box("top_collider", (0, 0, zm), TABLE, material=mat)
    crack = "".join(mesh_usda(m, root, indent="            ") for m in _crack_lines())
    body = physics_material(root, (0.7, 0.6)) + cols + "".join(mesh_usda(m, root) for m in visuals + legs) + f"""
        def Xform "crack" (
        )
        {{
            token visibility = "invisible"
            double3 xformOp:translate = (0, 0, {z1 + 0.0006:.6f})
            uniform token[] xformOpOrder = ["xformOp:translate"]
{crack}
        }}"""
    return usda_file(root, body, [GLASS, GLASS_EDGE, STEEL, DARK, CRACK])


# -- the pad -----------------------------------------------------------------------------------


def _pad(root: str, look: Look, thickness: float, tile: float) -> str:
    """Origin: the middle of the pad's underside; its lug stands on its -x edge."""
    lx, ly, lz = LUG
    lug_c = (-0.5 * PAD - 0.5 * lx, 0.0, 0.5 * lz)
    visuals = [box_mesh("pad", look.name, (0, 0, 0.5 * thickness), (PAD, PAD, thickness), tile=tile), box_mesh("lug", "wood", lug_c, LUG, tile=0.12)]
    cols = col_box("pad_collider", (0, 0, 0.5 * thickness), (PAD, PAD, thickness)) + col_box("lug_collider", lug_c, LUG)
    return usda_file(root, cols + "".join(mesh_usda(m, root) for m in visuals), [look, WOOD], root_api='"PhysicsRigidBodyAPI", "PhysicsMassAPI"')


def cork_usda() -> str:
    return _pad("CorkPad", CORK, CORK_T, 0.10)
