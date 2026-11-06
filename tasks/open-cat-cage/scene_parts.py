"""The scene's visual parts, as meshes. Pure Python and NumPy: nothing here needs the simulator, so the layout can be checked offline.

All lengths in metres. World frame: x forward, y left, the table's top at z = TOP.
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np

from usdkit import Look, Mesh, box, cylinder, join, lathe, tube

HERE = Path(__file__).resolve().parent
TOP = 0.7475

# The toy cat, crouched in its cage just behind the shut door, aimed at the birdcage.
CAT_XY = (0.66, 0.30)
CAT_YAW = math.radians(220.0)  # its nose points this way, from +x
CAT_SCALE = 0.75

# The bird's cage: a glass box with small air holes, like a glass terrarium. Its door is a glass pane in a brass frame on the near face, sliding along y in rails like the cat's door. It starts open.
CAGE_XY = (0.24, 0.0)
CAGE_D = 0.20  # along x
CAGE_W = 0.34  # along y
TRAY_H = 0.035
WALL_H = 0.15
GLASS_T = 0.003
WIRE_R = 0.0018
DOOR_W = 0.12
DOOR_X = -CAGE_D / 2 - 0.006  # the door's plane, from the cage's middle
DOOR_Z0 = TRAY_H + 0.006  # the door's bottom edge
DOOR_H = 0.134
OPENING_HALF = 0.0575  # the doorway, along y from the cage's middle
DOOR_START_Y = -0.115  # the door's middle at the start, parked in front of the fixed glass
DOOR_TRAVEL = -DOOR_START_Y  # shut with its middle at y 0, its leading edge against a stop
KNOB_D = 0.02  # a small brass knob on a stem, for a straight-down pinch
KNOB_H = 0.026
LUG = (KNOB_D, KNOB_D, KNOB_H)
LUG_X = DOOR_X - 0.028  # the knob's middle, far enough out that the wrist camera, 7 cm ahead of the fingers, passes over the glass top
LUG_Z = 0.137
PERCH_Z = TRAY_H + 0.02
PERCH_R = 0.006
BIRD_XY = (CAGE_XY[0] + 0.01, CAGE_XY[1] - 0.005)
BIRD_YAW = math.radians(145.0)
BIRD_SCALE = 1.2  # a 15 cm canary

# The lock: a thick solid column on its own foot beside the doorway, the door closing against it. Its keyway is in a brass plug on the column's top, and the key starts in it.
COLUMN_X = (-0.1575, -0.1025)  # from the cage's middle
COLUMN_Y = (0.0625, 0.1175)
COLUMN_TOP = 0.145
PIER_X = (-0.109, -0.095)  # the column's upper part, up to the cage's top, behind the fingers
LOCK_XY = (-0.13, 0.09)  # the keyway's axis, from the cage's middle
LOCK_TOP = COLUMN_TOP
LOCK_FLOOR = 0.105
HOLE_HALF = 0.008
PLUG_R = 0.012
BEZEL_R = 0.0165
KEY_HEAD = (0.03, 0.008, 0.026)  # along x, thick, tall: a red plastic head, pinched on its faces
KEY_SHAFT_R = 0.004
KEY_SHAFT_L = 0.035
KEY_MASS = 0.025
BOLT_Z = 0.12
BOLT_TRAVEL = 0.012

# The cat's cage: a roofed wire cage on the far left of the table. Its door slides along y, in rails on the near face. It starts shut, and when open it stands in front of the fixed part of that face.
CRATE_X = (0.485, 0.88)
CRATE_Y = (0.02, 0.64)
CRATE_H = 0.26
CRATE_OPENING = (0.02, 0.30)  # the doorway, along y, on the near face
CRATE_BAR = 0.045
CRATE_DOOR_LEN = 0.30
CRATE_DOOR_X = CRATE_X[0] - 0.015  # the door's plane
CRATE_DOOR_OPEN_Y = 0.46  # the door's middle when it stands open
CRATE_DOOR_SHUT_Y = 0.16
# The door's pull: one bent length of the cage's own wire, welded to an upright bar of the door, with a dark plastic sleeve on its upright part, as on a real wire cage.
HANDLE_Y = -0.075  # along y from the door's middle: on the door's third bar from its leading edge
HANDLE_OUT = 0.09  # how far the sleeve's axis stands off the door, on the robot's side: the wrist camera, 7 cm ahead of the fingers, must clear the door
HANDLE_Z = 0.085  # the sleeve's middle
SLEEVE_D = 0.022
SLEEVE_H = 0.05
PULL_R = 0.004  # the pull's wire

_CRATE_WIRE = (0.78, 0.79, 0.80)  # white-coated wire: the cat stays readable through it at 320 x 176

LOOKS = {
    "glass": Look(mdl_glass=(0.97, 0.99, 0.98), no_shadow=True),  # the renderer's clear glass, casting no shadow, so that the bird stays lit and shows at 320 x 176
    "frame": Look(color=(0.018, 0.019, 0.021), metallic=0.3, roughness=0.42),
    "air_hole": Look(color=(0.008, 0.008, 0.009), roughness=0.6),
    "key_cap": Look(color=(0.62, 0.035, 0.03), roughness=0.4),
    "lock": Look(color=(0.06, 0.06, 0.065), metallic=0.6, roughness=0.35),
    "icon": Look(color=(0.85, 0.85, 0.82), roughness=0.5),
    "bolt": Look(color=(0.72, 0.72, 0.74), metallic=1.0, roughness=0.28),
    "slot": Look(color=(0.004, 0.004, 0.005), roughness=0.8),
    "tray": Look(color=(0.015, 0.11, 0.20), roughness=0.38),
    "liner": Look(color=(0.05, 0.16, 0.25), roughness=0.75),
    "brass": Look(color=(0.78, 0.55, 0.16), metallic=1.0, roughness=0.3),
    "wood": Look(color=(0.42, 0.24, 0.10), roughness=0.7),
    "cup": Look(color=(0.82, 0.82, 0.80), roughness=0.3),
    "crate": Look(color=_CRATE_WIRE, metallic=0.2, roughness=0.42),
    "crate_door": Look(color=_CRATE_WIRE, metallic=0.2, roughness=0.42),
    "rail": Look(color=(0.32, 0.33, 0.35), metallic=0.9, roughness=0.4),
    "sleeve": Look(color=(0.03, 0.03, 0.033), roughness=0.55),
    "cat": Look(texture="cat_color.jpg", roughness=0.78),
    "cat_tail": Look(texture="cat_tail_color.jpg", roughness=0.78),
    "bird": Look(texture="bird_color.jpg", roughness=0.6),
    "cat_eye": Look(color=(0.75, 0.56, 0.02), roughness=0.08),
    "bird_eye": Look(color=(0.005, 0.005, 0.005), roughness=0.08),
    "pupil": Look(color=(0.002, 0.002, 0.002), roughness=0.06),
}


def wire(a, b, r: float = WIRE_R, sides: int = 6) -> Mesh:
    return tube(np.array([a, b], float), r, sides)


def _rot_z(deg_or_rad: float) -> np.ndarray:
    c, s = math.cos(deg_or_rad), math.sin(deg_or_rad)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def skin(name: str, look: str, scale: float, yaw: float) -> list:
    """A baked skin and its eyes, scaled and turned about the vertical through its origin. Returns parts, (name, Mesh, look)."""
    d = np.load(HERE / "animals" / f"{name}.npz")
    r = _rot_z(yaw)
    v, f, uv, n = d["v"].astype(float), d["f"].astype(int), d["uv"].astype(float), d["n"].astype(float)
    body = Mesh(scale * v @ r.T, f, n=n @ r.T, uv=uv)
    parts = [(name, body, look)]
    eye_r = float(d["eye_r"]) * scale
    eye = lathe([(0.0, -eye_r)] + [(eye_r * math.cos(a), eye_r * math.sin(a)) for a in np.linspace(-1.3, 1.3, 8)] + [(0.0, eye_r)], 16)
    eyes, pupils = [], []
    for c in d["eyes"]:
        at = scale * np.asarray(c, float) @ r.T
        eyes.append(eye.moved(at))
        if name == "cat":  # a wide hunting pupil, looking along the head
            gaze = r @ np.array([1.0, 0.0, 0.0])
            pupils.append(eye.scaled((0.42, 0.62, 0.92)).turned(math.degrees(yaw), "z").moved(at + gaze * eye_r * 0.66))
    if eyes:
        parts.append((f"{name}_eyes", join(*eyes), f"{name}_eye"))
    if pupils:
        parts.append((f"{name}_pupils", join(*pupils), "pupil"))
    return parts


def cat_parts() -> list:
    return skin("cat", "cat", CAT_SCALE, CAT_YAW)


def tail_tip_parts() -> list:
    """The tail tip, in the same frame as the cat: scaled and turned, about the cat's origin."""
    return skin("cat_tail", "cat_tail", CAT_SCALE, CAT_YAW)[:1]


def tail_pivot() -> tuple[float, float, float]:
    """The point the tail tip turns about, in the cat's placed frame."""
    p = np.load(HERE / "animals" / "cat_tail.npz")["pivot"].astype(float)
    return tuple(float(v) for v in (CAT_SCALE * p @ _rot_z(CAT_YAW).T))


def bird_parts() -> list:
    return skin("bird", "bird", BIRD_SCALE, BIRD_YAW)


def knob_y(slid: float) -> float:
    """The knob's y from the cage's middle, with the door slid that far from its start."""
    return DOOR_START_Y + slid


def door_parts() -> list:
    """The bird's door at its start, its origin under the cage's middle: a glass pane in a brass frame with a cross bar, and a brass knob on a stem. Joint zero is this pose."""
    x, y = DOOR_X, DOOR_START_Y
    z0, z1 = DOOR_Z0, DOOR_Z0 + DOOR_H
    y0, y1 = y - DOOR_W / 2, y + DOOR_W / 2
    pane = box((GLASS_T, DOOR_W, DOOR_H), (x, y, (z0 + z1) / 2))
    bars = [wire((x, y0, z0), (x, y1, z0), 0.003, 8), wire((x, y0, z1), (x, y1, z1), 0.003, 8), wire((x, y0, z0), (x, y0, z1), 0.003, 8), wire((x, y1, z0), (x, y1, z1), 0.003, 8), wire((x, y0, LUG_Z - 0.01), (x, y1, LUG_Z - 0.01), 0.0025, 8)]
    bars.append(tube(np.array([(x, y, LUG_Z - 0.01), (LUG_X + 0.006, y, LUG_Z - 0.01), (LUG_X, y, LUG_Z - 0.012)]), 0.0025, 8))
    h, r = KNOB_H / 2, KNOB_D / 2
    knob = lathe([(0.0, -h), (r - 0.0015, -h), (r, -h + 0.0015), (r, h - 0.003), (r - 0.003, h), (0.0, h)], 16).moved((LUG_X, y, LUG_Z))
    return [("pane", pane, "glass"), ("door_frame", join(*bars), "brass"), ("knob", knob, "brass")]


def door_hulls() -> list:
    """The door's colliders, (name, Mesh), each for a convex hull: the pane and the round knob."""
    x, y = DOOR_X, DOOR_START_Y
    pane = box((0.004, DOOR_W, DOOR_H), (x, y, DOOR_Z0 + DOOR_H / 2))
    h, r = KNOB_H / 2, KNOB_D / 2
    knob = lathe([(0.0, -h), (r, -h), (r, h), (0.0, h)], 16).moved((LUG_X, y, LUG_Z))
    return [("c_pane", pane), ("c_knob", knob)]


def _column_boxes() -> list:
    """The lock column's colliders, (name, size, centre): solid, with the keyway's square hole down from its top."""
    x0, x1 = COLUMN_X
    y0, y1 = COLUMN_Y
    lx, ly = LOCK_XY
    h = HOLE_HALF
    top, floor = LOCK_TOP, LOCK_FLOOR
    zm, dz = (floor + top) / 2, top - floor
    return [
        ("c_col_low", (x1 - x0, y1 - y0, floor), ((x0 + x1) / 2, (y0 + y1) / 2, floor / 2)),
        ("c_col_a", ((lx - h) - x0, y1 - y0, dz), ((x0 + lx - h) / 2, (y0 + y1) / 2, zm)),
        ("c_col_b", (x1 - (lx + h), y1 - y0, dz), ((x1 + lx + h) / 2, (y0 + y1) / 2, zm)),
        ("c_col_c", (2 * h, (ly - h) - y0, dz), (lx, (y0 + ly - h) / 2, zm)),
        ("c_col_d", (2 * h, y1 - (ly + h), dz), (lx, (y1 + ly + h) / 2, zm)),
        ("c_pier", (PIER_X[1] - PIER_X[0], y1 - y0, TRAY_H + WALL_H - top), ((PIER_X[0] + PIER_X[1]) / 2, (y0 + y1) / 2, (top + TRAY_H + WALL_H) / 2)),
    ]


def cage_boxes() -> list:
    """The glass cage's fixed colliders, (name, size, centre): glass walls and top, with the doorway clear, and the lock column."""
    hd, hw = CAGE_D / 2, CAGE_W / 2
    z0, h = TRAY_H, WALL_H
    zm = z0 + h / 2
    side = hw - OPENING_HALF
    above = DOOR_Z0 + DOOR_H + 0.002
    return [
        ("c_tray", (CAGE_D + 0.016, CAGE_W + 0.016, TRAY_H), (0.0, 0.0, TRAY_H / 2)),
        ("c_far", (0.004, CAGE_W, h), (hd, 0.0, zm)),
        ("c_left", (CAGE_D, 0.004, h), (0.0, hw, zm)),
        ("c_right", (CAGE_D, 0.004, h), (0.0, -hw, zm)),
        ("c_near_left", (0.004, side, h), (-hd, hw - side / 2, zm)),
        ("c_near_right", (0.004, side, h), (-hd, -hw + side / 2, zm)),
        ("c_lintel", (0.004, 2 * OPENING_HALF, z0 + h - above), (-hd, 0.0, (above + z0 + h) / 2)),
        ("c_top", (CAGE_D, CAGE_W, 0.004), (0.0, 0.0, z0 + h)),
        *_column_boxes(),
    ]


def key_parts() -> list:
    """The key, its origin in the middle of its head: a red plastic head with a hole for a ring, a brass collar and a flat brass blade with cuts. Its head and blade lie along x at the start."""
    hx, ht, hh = KEY_HEAD
    head = box((hx, ht, hh), (0.0, 0.0, 0.0), bevel=0.0012, round_xy=0.003)
    ring_hole = cylinder(0.0035, ht + 0.0006, (0.0, 0.0, hh * 0.22), axis="y", segments=12)
    z = -hh / 2
    collar = cylinder(0.0065, 0.004, (0.0, 0.0, z - 0.002), segments=12)
    blade = box((0.012, 0.0028, KEY_SHAFT_L), (0.0, 0.0, z - KEY_SHAFT_L / 2))
    cuts = [box((0.003, 0.0034, 0.004), (0.0052, 0.0, z - 0.012 - k * 0.008)) for k in range(3)]
    return [("key_head", head, "key_cap"), ("key_ring_hole", ring_hole, "slot"), ("key_shaft", join(collar, blade), "brass"), ("key_cuts", join(*cuts), "slot")]


def key_shaft_hull() -> Mesh:
    """The blade's collider, a round shaft with a pointed tip: it fits the keyway's hole with 4 mm to spare, and the key cannot stand on its tip when set down."""
    z = -KEY_HEAD[2] / 2
    return lathe([(0.0, z - KEY_SHAFT_L), (KEY_SHAFT_R, z - KEY_SHAFT_L + 0.008), (KEY_SHAFT_R, z), (0.0, z)], 10)


def plug_parts() -> list:
    """The lock's plug, which turns with the key, its origin under the cage's middle: a brass disc with a keyway slot along x. Joint zero is this pose."""
    lx, ly = LOCK_XY
    disc = cylinder(PLUG_R, 0.0024, (lx, ly, LOCK_TOP + 0.0012), segments=20)
    slot = join(box((0.015, 0.0036, 0.0005), (lx, ly, LOCK_TOP + 0.00255)), cylinder(0.0028, 0.0005, (lx, ly, LOCK_TOP + 0.00255), segments=10))
    return [("plug", disc, "brass"), ("keyway", slot, "slot")]


def bolt_parts() -> list:
    """The lock's bolt, drawn back into the column, its origin under the cage's middle. It slides toward the door when the key turns. Joint zero is this pose."""
    y = COLUMN_Y[0]
    return [("bolt", box((0.008, BOLT_TRAVEL + 0.004, 0.01), (DOOR_X, y + (BOLT_TRAVEL + 0.004) / 2, BOLT_Z), bevel=0.0008), "bolt")]


def _padlock(c, size: float, axis: str, closed: bool) -> Mesh:
    """A padlock symbol, drawn on the face across `axis` (x or z) at c: a body and a shackle, the shackle lifted free on one side when open."""
    c = np.asarray(c, float)
    w, h = size, size * 0.72
    t = 0.0012
    r = w * 0.3
    tube_r = w * 0.09
    lift = 0.0 if closed else w * 0.22
    arc = [(r * math.cos(a), h / 2 + w * 0.12 + lift + r * math.sin(a)) for a in np.linspace(0.0, math.pi, 9)]
    pts2 = [(r, h / 2 + lift)] + arc + ([(-r, h / 2)] if closed else [])
    if axis == "x":  # up the column's near face, its width along y
        body = box((t, w, h), (c[0], c[1], c[2]))
        path = np.array([(c[0] - t / 2, c[1] + p[0], c[2] + p[1]) for p in pts2])
    else:  # on the column's top, its width along y and its height along +x
        body = box((h, w, t), (c[0], c[1], c[2]))
        path = np.array([(c[0] + p[1], c[1] + p[0], c[2] + t / 2) for p in pts2])
    return join(body, tube(path, tube_r, 6))


def column_parts() -> list:
    """The lock column, in the cage's frame: solid and thick, on a foot, with its upper part rising to the cage's top. Its keyway has a brass bezel, with a padlock symbol at each position of the key."""
    x0, x1 = COLUMN_X
    y0, y1 = COLUMN_Y
    lx, ly = LOCK_XY
    z1 = TRAY_H + WALL_H
    low = box((x1 - x0, y1 - y0, COLUMN_TOP), ((x0 + x1) / 2, (y0 + y1) / 2, COLUMN_TOP / 2), bevel=0.0015, round_xy=0.003)
    pier = box((PIER_X[1] - PIER_X[0], y1 - y0, z1 - COLUMN_TOP + 0.002), ((PIER_X[0] + PIER_X[1]) / 2, (y0 + y1) / 2, (COLUMN_TOP + z1) / 2), bevel=0.001)
    foot = box((x1 - x0 + 0.02, y1 - y0 + 0.02, 0.006), ((x0 + x1) / 2, (y0 + y1) / 2, 0.003), bevel=0.001)
    bezel = lathe([(PLUG_R, LOCK_TOP), (BEZEL_R, LOCK_TOP), (BEZEL_R, LOCK_TOP + 0.002), (BEZEL_R - 0.0015, LOCK_TOP + 0.003), (PLUG_R, LOCK_TOP + 0.003), (PLUG_R, LOCK_TOP)], 24).moved((lx, ly, 0.0))
    screws = join(*[cylinder(0.0025, 0.0012, (cx, cy, COLUMN_TOP + 0.0006), segments=8) for cx, cy in ((x0 + 0.005, y0 + 0.005), (x0 + 0.005, y1 - 0.005))])
    marks = join(
        _padlock((lx - BEZEL_R - 0.0055, ly, LOCK_TOP + 0.0007), 0.008, "z", False),  # at the key's starting position: unlocked
        _padlock((lx, ly + BEZEL_R + 0.0055, LOCK_TOP + 0.0007), 0.008, "z", True),  # a quarter turn on: locked
        _padlock((x0 - 0.0007, (y0 + y1) / 2, 0.082), 0.03, "x", True),  # the large symbol on the near face
    )
    return [("column", join(low, pier), "lock"), ("column_foot", foot, "lock"), ("bezel", bezel, "brass"), ("screws", screws, "bolt"), ("lock_marks", marks, "icon")]


def _ring(c, axis: str, r: float = 0.005) -> Mesh:
    """A small dark ring on a pane: one air hole, in the plane across `axis`."""
    c = np.asarray(c, float)
    t = np.linspace(0.0, 2 * math.pi, 17)
    u, v = {"x": ((0, 1, 0), (0, 0, 1)), "y": ((1, 0, 0), (0, 0, 1)), "z": ((1, 0, 0), (0, 1, 0))}[axis]
    pts = c + r * (np.outer(np.cos(t), u) + np.outer(np.sin(t), v))
    return tube(pts, 0.0009, 4)


def cage_parts() -> list:
    """The glass cage, with its origin under the middle of the tray."""
    hd, hw = CAGE_D / 2, CAGE_W / 2
    z0, z1 = TRAY_H, TRAY_H + WALL_H
    zm = (z0 + z1) / 2
    h = WALL_H
    side = hw - OPENING_HALF
    above = DOOR_Z0 + DOOR_H + 0.002
    glass = join(
        box((GLASS_T, CAGE_W, h), (hd, 0.0, zm)),
        box((CAGE_D, GLASS_T, h), (0.0, hw, zm)),
        box((CAGE_D, GLASS_T, h), (0.0, -hw, zm)),
        box((GLASS_T, hw - COLUMN_Y[1], h), (-hd, (hw + COLUMN_Y[1]) / 2, zm)),
        box((GLASS_T, side, h), (-hd, -hw + side / 2, zm)),
        box((GLASS_T, 2 * OPENING_HALF, z1 - above), (-hd, 0.0, (above + z1) / 2)),
        box((CAGE_D, CAGE_W, GLASS_T), (0.0, 0.0, z1)),
    )
    # A black frame along the box's edges.
    corners = [(-hd, -hw), (hd, -hw), (hd, hw), (-hd, hw)]
    edges = []
    for (xa, ya), (xb, yb) in zip(corners, corners[1:] + corners[:1]):
        edges.append(wire((xa, ya, z1), (xb, yb, z1), 0.004, 8))
    for xa, ya in corners:
        edges.append(wire((xa, ya, z0), (xa, ya, z1), 0.004, 8))
    # The doorway's frame in brass, so that the gap shows.
    x = -hd - 0.001
    opening = [wire((x, s * OPENING_HALF, z0), (x, s * OPENING_HALF, above), 0.003, 8) for s in (-1.0, 1.0)]
    opening += [wire((x, -OPENING_HALF, above), (x, OPENING_HALF, above), 0.003, 8), wire((x, -OPENING_HALF, z0 + 0.003), (x, OPENING_HALF, z0 + 0.003), 0.003, 8)]
    # Small air holes, in rows along the top and round the walls' upper part.
    holes = []
    for gx in np.linspace(-hd * 0.8, hd * 0.8, 5):
        for gy in np.linspace(-hw * 0.85, hw * 0.85, 7):
            holes.append(_ring((gx, gy, z1 + 0.002), "z"))
    for gx in np.linspace(-hd * 0.7, hd * 0.7, 4):
        for s in (-1.0, 1.0):
            holes.append(_ring((gx, s * (hw + 0.002), z1 - 0.022), "y"))
    for gy in np.linspace(-hw * 0.85, hw * 0.85, 7):
        holes.append(_ring((hd + 0.002, gy, z1 - 0.022), "x"))
    # The door's rails.
    ylo, yhi = DOOR_START_Y - DOOR_W / 2 - 0.012, COLUMN_Y[0]
    rails = join(box((0.012, yhi - ylo, 0.005), (DOOR_X, (ylo + yhi) / 2, z0 + 0.0025), bevel=0.0005), wire((DOOR_X, ylo, DOOR_Z0 + DOOR_H + 0.004), (DOOR_X, yhi, DOOR_Z0 + DOOR_H + 0.004), 0.003, 8),
                 *[box((0.012, 0.006, 0.004), ((DOOR_X - hd) / 2, yb, DOOR_Z0 + DOOR_H + 0.004)) for yb in (ylo + 0.02, yhi - 0.015)])
    tray = box((CAGE_D + 0.016, CAGE_W + 0.016, TRAY_H), (0.0, 0.0, TRAY_H / 2), bevel=0.003, round_xy=0.012)
    liner = box((CAGE_D - 0.002, CAGE_W - 0.002, 0.002), (0.0, 0.0, TRAY_H + 0.001))
    perch = cylinder(PERCH_R, CAGE_W, (0.01, 0.0, PERCH_Z), axis="y", segments=12)
    cups = join(*[lathe([(0.0, 0.0), (0.014, 0.0), (0.018, 0.022), (0.016, 0.022), (0.012, 0.004), (0.0, 0.004)], 16).moved((0.075, s * (hw - 0.025), TRAY_H + 0.002)) for s in (-1.0, 1.0)])
    return [
        ("tray", tray, "tray"),
        ("liner", liner, "liner"),
        ("glass", glass, "glass"),
        ("edges", join(*edges), "frame"),
        ("opening", join(*opening), "brass"),
        ("air_holes", join(*holes), "air_hole"),
        ("rails", rails, "rail"),
        ("perch", perch, "wood"),
        ("cups", cups, "cup"),
    ]


def toy_hull(name: str, scale: float, yaw: float, keep_x: float | None = None) -> Mesh:
    """A toy's skin as a point cloud for a convex-hull collider. `keep_x` leaves out everything behind that x, in the toy's own frame (the cat's tail)."""
    v = np.load(HERE / "animals" / f"{name}.npz")["v"].astype(float)
    if keep_x is not None:
        v = v[v[:, 0] > keep_x]
    v = v[:: max(len(v) // 1500, 1)]
    v = scale * v @ _rot_z(yaw).T
    return Mesh(v, np.array([(0, 1, 2)]))


CAT_BOX = ((-0.245, -0.105, 0.0), (0.325, 0.115, 0.225))  # the cat without its tail, in its own frame at scale 1
BIRD_BOX = ((-0.105, -0.032, 0.0), (0.062, 0.032, 0.098))  # the bird, likewise


def _crate_face(a, b, height: float, step: float, frame_r: float = 0.0045, bar_r: float = 0.002) -> list[Mesh]:
    """A vertical wire wall from point a to point b on the table: a frame, a middle rail and upright bars."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    ax, ay, bx, by = a[0], a[1], b[0], b[1]
    m = [wire((ax, ay, 0.006), (bx, by, 0.006), frame_r, 8), wire((ax, ay, height), (bx, by, height), frame_r, 8), wire((ax, ay, 0.0), (ax, ay, height), frame_r, 8), wire((bx, by, 0.0), (bx, by, height), frame_r, 8), wire((ax, ay, height * 0.5), (bx, by, height * 0.5), bar_r, 6)]
    n = max(int(round(np.linalg.norm(b - a) / step)), 1)
    for k in range(1, n):
        p = a + (b - a) * k / n
        m.append(wire((p[0], p[1], 0.006), (p[0], p[1], height), bar_r))
    return m


def crate_parts() -> list:
    """The cat's cage, in world xy with z from the table: three walls, the fixed part of the near face, the roof, and the door's rails."""
    x0, x1 = CRATE_X
    y0, y1 = CRATE_Y
    h = CRATE_H
    m = []
    m += _crate_face((x0, y0), (x1, y0), h, CRATE_BAR)
    m += _crate_face((x1, y0), (x1, y1), h, CRATE_BAR)
    m += _crate_face((x1, y1), (x0, y1), h, CRATE_BAR)
    m += _crate_face((x0, y1), (x0, CRATE_OPENING[1]), h, CRATE_BAR)
    m.append(wire((x0, y0, h), (x0, CRATE_OPENING[1], h), 0.0045, 8))  # the lintel across the doorway
    for y in np.linspace(y0, y1, int(round((y1 - y0) / CRATE_BAR)) + 1)[1:-1]:  # the roof: bars along x, two rails along y
        m.append(wire((x0, y, h), (x1, y, h)))
    for x in (x0 + (x1 - x0) / 3, x0 + 2 * (x1 - x0) / 3):
        m.append(wire((x, y0, h), (x, y1, h)))
    xr = CRATE_DOOR_X
    ylo, yhi = y0 - 0.012, y1
    floor_rail = box((0.022, yhi - ylo, 0.006), (xr, (ylo + yhi) / 2, 0.003), bevel=0.001)
    top_rail = wire((xr, ylo, h + 0.002), (xr, yhi, h + 0.002), 0.004, 8)  # thin, so that it does not cut across the cat in the top camera
    brackets = join(box((0.03, 0.008, 0.005), ((xr + x0) / 2, ylo + 0.03, h + 0.002)), box((0.03, 0.008, 0.005), ((xr + x0) / 2, yhi - 0.03, h + 0.002)))
    return [("crate_wires", join(*m), "crate"), ("floor_rail", floor_rail, "rail"), ("top_rail", join(top_rail, brackets), "crate")]



def crate_door_parts() -> list:
    """The cat's cage's door, origin at the middle of its bottom edge: a wire panel, and a brass handle on an arm on the robot's side."""
    h = CRATE_DOOR_LEN / 2
    panel = _crate_face((0.0, -h), (0.0, h), CRATE_H - 0.006, CRATE_BAR * 0.8, 0.005, 0.002)
    y, x, z0, z1 = HANDLE_Y, -HANDLE_OUT, HANDLE_Z - SLEEVE_H / 2, HANDLE_Z + SLEEVE_H / 2
    bend = 0.012
    path = [(0.0, y, z0 - bend)] + [(x * f, y, z0 - bend) for f in (0.5, 0.8)] + [(x, y, z0 - bend * 0.3), (x, y, z0 + 0.005), (x, y, z1 - 0.005), (x, y, z1 + bend * 0.3)] + [(x * f, y, z1 + bend) for f in (0.8, 0.5)] + [(0.0, y, z1 + bend)]
    pull = tube(np.array(path), PULL_R, 8)
    sleeve = lathe([(0.0, z0), (SLEEVE_D / 2 - 0.001, z0), (SLEEVE_D / 2, z0 + 0.001), (SLEEVE_D / 2, z1 - 0.001), (SLEEVE_D / 2 - 0.001, z1), (0.0, z1)], 16).moved((x, y, 0.0))
    return [("door_panel", join(*panel), "crate_door"), ("door_pull", pull, "crate_door"), ("door_sleeve", sleeve, "sleeve")]


def crate_boxes() -> list:
    """The cat's cage's fixed colliders, (name, size, centre) in world xy with z from the table: thin solid walls and roof, and the doorway clear."""
    x0, x1 = CRATE_X
    y0, y1 = CRATE_Y
    h = CRATE_H
    yd = CRATE_OPENING[1]
    return [
        ("c_right", (x1 - x0, 0.006, h), ((x0 + x1) / 2, y0, h / 2)),
        ("c_left", (x1 - x0, 0.006, h), ((x0 + x1) / 2, y1, h / 2)),
        ("c_back", (0.006, y1 - y0, h), (x1, (y0 + y1) / 2, h / 2)),
        ("c_near", (0.006, y1 - yd, h), (x0, (y1 + yd) / 2, h / 2)),
        ("c_roof", (x1 - x0, y1 - y0, 0.006), ((x0 + x1) / 2, (y0 + y1) / 2, h)),
    ]


def crate_door_boxes() -> list:
    """The cat's door's colliders, (name, size, centre), origin at the middle of its bottom edge."""
    return [
        ("c_panel", (0.006, CRATE_DOOR_LEN, CRATE_H - 0.018), (0.0, 0.0, (CRATE_H - 0.006 + 0.012) / 2)),
        ("c_grip", (SLEEVE_D, SLEEVE_D, SLEEVE_H), (-HANDLE_OUT, HANDLE_Y, HANDLE_Z)),
        ("c_prong_low", (HANDLE_OUT, 2 * PULL_R, 2 * PULL_R), (-HANDLE_OUT / 2, HANDLE_Y, HANDLE_Z - SLEEVE_H / 2 - 0.012)),
        ("c_prong_high", (HANDLE_OUT, 2 * PULL_R, 2 * PULL_R), (-HANDLE_OUT / 2, HANDLE_Y, HANDLE_Z + SLEEVE_H / 2 + 0.012)),
    ]
