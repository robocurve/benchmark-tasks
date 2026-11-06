"""On the station table, between the arms: a dark grey access frame fixed to the table, holding a flat grey cover
that sits flush with the frame's top. The gap round the cover is about 2 mm. The cover has two narrow slotted
keyholes, one towards each arm. Nearer the robot lie two blue bearers, parallel bars running across the robot's
view. Beside each arm stands a lifting key in a small dark holder: a thin steel shaft with a yellow handle on top
and a short crossbar at its foot.
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
import parts  # noqa: E402
from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

FRAME_C, COVER_C, BLUE_C = (0.22, 0.23, 0.25), (0.62, 0.64, 0.66), (0.15, 0.33, 0.75)
STEEL_C, HANDLE_C, HOLDER_C = (0.72, 0.73, 0.75), (0.95, 0.78, 0.12), (0.18, 0.18, 0.20)

# Frame (fixed): 170 (x) x 294 (y) x 40 mm outside, 15 mm walls, opening 140 x 264 mm; a 5 mm ledge inside the walls,
# 10 mm deep, top 30 mm above the table. Under the cover there is only the table.
FRAME_XY = (0.380, 0.0)
FRAME_OUT = (0.170, 0.294, 0.040)
FRAME_WALL, LEDGE_W, LEDGE_TOP, LEDGE_H = 0.015, 0.005, 0.030, 0.010
OPEN = (FRAME_OUT[0] - 2 * FRAME_WALL, FRAME_OUT[1] - 2 * FRAME_WALL)  # 0.140 x 0.264


def _frame() -> parts.Part:
    hx, hy, h = FRAME_OUT[0] / 2, FRAME_OUT[1] / 2, FRAME_OUT[2]
    ox, oy = OPEN[0] / 2, OPEN[1] / 2
    lz = LEDGE_TOP - LEDGE_H / 2
    return parts.Part("frame", pos=(*FRAME_XY, TOP), shapes=[
        parts.box((FRAME_WALL, FRAME_OUT[1], h), at=(hx - FRAME_WALL / 2, 0.0, h / 2), color=FRAME_C),
        parts.box((FRAME_WALL, FRAME_OUT[1], h), at=(-hx + FRAME_WALL / 2, 0.0, h / 2), color=FRAME_C),
        parts.box((OPEN[0], FRAME_WALL, h), at=(0.0, hy - FRAME_WALL / 2, h / 2), color=FRAME_C),
        parts.box((OPEN[0], FRAME_WALL, h), at=(0.0, -hy + FRAME_WALL / 2, h / 2), color=FRAME_C),
        parts.box((LEDGE_W, OPEN[1], LEDGE_H), at=(ox - LEDGE_W / 2, 0.0, lz), color=FRAME_C),
        parts.box((LEDGE_W, OPEN[1], LEDGE_H), at=(-ox + LEDGE_W / 2, 0.0, lz), color=FRAME_C),
        parts.box((OPEN[0] - 2 * LEDGE_W, LEDGE_W, LEDGE_H), at=(0.0, oy - LEDGE_W / 2, lz), color=FRAME_C),
        parts.box((OPEN[0] - 2 * LEDGE_W, LEDGE_W, LEDGE_H), at=(0.0, -oy + LEDGE_W / 2, lz), color=FRAME_C),
    ])


# Cover (loose, glass-fibre composite): 136 x 260 x 10 mm, 0.6 kg, 2 mm gap all round in the opening. Body origin at
# the centre of its underside. Two cross-shaped keyholes through it, centred 100 mm either side of its centre (keys any
# closer together bring the two forearms into contact): a 14 mm square centre that takes the 10 mm shaft with 2 mm of
# play, and two narrow wings, 8 mm wide (x), which together make a 30 mm (y) slot that only the 4 mm crossbar fits.
# The shaft cannot enter a wing, so an unturned crossbar always lies within the slot and cannot hook the cover. A
# finger tip (about 15.5 x 4 mm) fits only singly and on the diagonal of the square; the closed pair (15.5 x 8 mm)
# fits nowhere.
COVER = (0.136, 0.260, 0.010)
COVER_MASS = 0.6
HOLE_SQ = 0.014
WING = (0.008, 0.030)  # x width, total y length of the crossbar slot
HOLE_Y = 0.100


def _cover_shapes():
    hx, hy, t = COVER[0] / 2, COVER[1] / 2, COVER[2]
    sq, wx, wy = HOLE_SQ / 2, WING[0] / 2, WING[1] / 2
    shapes = [parts.box((COVER[0], 2 * (HOLE_Y - wy), t), at=(0.0, 0.0, t / 2), color=COVER_C)]

    def band(y0, y1, gap):
        """Solid across the cover between y0 and y1 (one side), except |x| < gap."""
        for sy in (1, -1):
            for sx in (1, -1):
                shapes.append(parts.box((hx - gap, y1 - y0, t), at=(sx * (gap + hx) / 2, sy * (y0 + y1) / 2, t / 2),
                                        color=COVER_C))

    band(HOLE_Y - wy, HOLE_Y - sq, wx)  # wing nearer the centre
    band(HOLE_Y - sq, HOLE_Y + sq, sq)  # square centre
    band(HOLE_Y + sq, HOLE_Y + wy, wx)  # outer wing
    for sy in (1, -1):
        shapes.append(parts.box((COVER[0], hy - HOLE_Y - wy, t), at=(0.0, sy * (hy + HOLE_Y + wy) / 2, t / 2),
                                color=COVER_C))
    return shapes


COVER_PART = parts.Part("cover", pos=(*FRAME_XY, TOP + LEDGE_TOP + 0.0005), mass=COVER_MASS, shapes=_cover_shapes())

# Bearers (fixed): two blue bars 20 (x) x 270 (y) x 20 mm, 100 mm apart centre to centre, where the cover is laid so
# the key crossbars have room beneath it.
BEARER = (0.020, 0.270, 0.020)
BEARER_XY = (0.225, 0.0)
BEARER_GAP = 0.100
BEARERS = [Box(f"bearer_{n}", pos=(BEARER_XY[0] + dx, BEARER_XY[1], TOP + BEARER[2] / 2), size=BEARER, color=BLUE_C)
           for n, dx in (("near", -BEARER_GAP / 2), ("far", BEARER_GAP / 2))]

# Lifting keys (loose, 0.06 kg): steel crossbar 4 (x) x 26 (y) x 6 mm at the foot, 10 mm round steel shaft 70 mm long,
# 20 mm plastic cube handle on top (96 mm tall overall). Body origin at the centre of the crossbar's underside.
CROSS = (0.004, 0.026, 0.006)
SHAFT_R, SHAFT_L, HANDLE = 0.0050, 0.070, 0.020
KEY_H = CROSS[2] + SHAFT_L + HANDLE
KEY_MASS = 0.06
KEY_SHAPES = [
    parts.box(CROSS, at=(0.0, 0.0, CROSS[2] / 2), color=STEEL_C),
    parts.cylinder(SHAFT_R, SHAFT_L, at=(0.0, 0.0, CROSS[2] + SHAFT_L / 2), color=STEEL_C, axis="z"),
    parts.box((HANDLE, HANDLE, HANDLE), at=(0.0, 0.0, CROSS[2] + SHAFT_L + HANDLE / 2), color=HANDLE_C),
]
HOLDER_XY = {"left": (0.300, 0.195), "right": (0.300, -0.195)}
KEYS = {side: parts.Part(f"key_{side}", pos=(*xy, TOP + 0.0005), mass=KEY_MASS, shapes=KEY_SHAPES)
        for side, xy in HOLDER_XY.items()}

# Key holders (fixed): an open-topped dark sleeve 30 mm tall, 14 (x) x 32 (y) mm inside, 4 mm walls.
HOLDER_IN, HOLDER_WALL, HOLDER_H = (0.014, 0.032), 0.004, 0.030


def _holder(side: str) -> parts.Part:
    ix, iy, w, h = HOLDER_IN[0] / 2, HOLDER_IN[1] / 2, HOLDER_WALL, HOLDER_H
    return parts.Part(f"holder_{side}", pos=(*HOLDER_XY[side], TOP), shapes=[
        parts.box((w, 2 * (iy + w), h), at=(ix + w / 2, 0.0, h / 2), color=HOLDER_C),
        parts.box((w, 2 * (iy + w), h), at=(-ix - w / 2, 0.0, h / 2), color=HOLDER_C),
        parts.box((2 * ix, w, h), at=(0.0, iy + w / 2, h / 2), color=HOLDER_C),
        parts.box((2 * ix, w, h), at=(0.0, -iy - w / 2, h / 2), color=HOLDER_C),
    ])


OBJECTS = [_frame(), COVER_PART, *BEARERS, _holder("left"), _holder("right"), *KEYS.values()]

# -- scoring --------------------------------------------------------------------------------------------------------
FLAT_TOL = 0.0015  # every corner of the cover's underside within 1.5 mm of the surface it lies on
FLAT_DEG = 3.0
ON_BEARERS_X, ON_BEARERS_Y = 0.025, 0.030  # cover centre within 25 mm (x) / 30 mm (y) of the bearers' midpoint
DROP_SPEED = 0.75  # m/s downward: a free fall of about 29 mm
DROP_FALL = 0.025  # or more than 25 mm lower than two control steps before
RESET_TOL, RESET_JUMP = 0.0005, 0.020
_HALF = np.array(COVER) / 2
_CORNERS = np.array([[sx * _HALF[0], sy * _HALF[1], z] for sx in (1, -1) for sy in (1, -1) for z in (0.0, COVER[2])])


def _rot(q) -> np.ndarray:
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def corners(st) -> np.ndarray:
    """World positions (8 x 3, table-relative z) of the cover's corners, from its pose and true size."""
    p = np.asarray(st.pos, float)[:, None] + _rot(st.quat_xyzw) @ _CORNERS.T
    out = p.T.copy()
    out[:, 2] -= TOP
    return out


def lying_flat_at(st, height: float) -> bool:
    """Flat (a face within 3 degrees of level) with its lowest four corners within 1.5 mm of `height`, at rest."""
    c = corners(st)
    low4 = np.sort(c[:, 2])[:4]
    level = abs(_rot(st.quat_xyzw)[2, 2]) >= math.cos(math.radians(FLAT_DEG))
    return bool(level and np.all(np.abs(low4 - height) <= FLAT_TOL + 1e-6) and st.resting())


def key_in_cover(key, cover) -> bool:
    """A key still passes through the cover: the key's axis, from its crossbar foot to its handle top, crosses the
    plane of the cover's upper face inside the cover's outline (from the key's and the cover's poses)."""
    rc, rk = _rot(cover.quat_xyzw), _rot(key.quat_xyzw)
    n, a = rc[:, 2], rk[:, 2]
    p, base = np.asarray(key.pos, float), np.asarray(cover.pos, float)
    c = max((base, base + n * COVER[2]), key=lambda v: v[2])  # the upper face, whichever way up the cover lies
    denom = float(n @ a)
    if abs(denom) < 1e-6:  # key lying parallel to the cover
        return False
    t = float(n @ (c - p)) / denom
    local = rc.T @ (p + t * a - c)
    return bool(0.0 <= t <= KEY_H and abs(local[0]) < _HALF[0] and abs(local[1]) < _HALF[1])


def _keys_out(objects) -> bool:
    return not any(key_in_cover(objects[f"key_{s}"], objects["cover"]) for s in KEYS)


def on_bearers(st) -> bool:
    d = np.asarray(st.pos[:2], float) - BEARER_XY
    return bool(lying_flat_at(st, BEARER[2]) and abs(d[0]) <= ON_BEARERS_X + 1e-6 and abs(d[1]) <= ON_BEARERS_Y + 1e-6)


# Trial history: whether the cover was ever dropped (moving down faster than 0.75 m/s, or more than 25 mm lower than
# two control steps before). Cleared only at a new trial: every loose object back within 0.5 mm of its start, at
# rest, after a jump of more than 20 mm in one step.
_START = {o.name: np.asarray(o.pos, float) for o in OBJECTS if o.free}
_HISTORY = {"dropped": False, "prev": None, "z": []}


def _update(objects) -> None:
    now = {n: np.asarray(objects[n].pos, float) for n in _START}
    home = all(np.linalg.norm(now[n] - p) < RESET_TOL and objects[n].resting() for n, p in _START.items())
    prev = _HISTORY["prev"]
    jumped = prev is None or any(np.linalg.norm(now[n] - prev[n]) > RESET_JUMP for n in _START)
    _HISTORY["prev"] = now
    z = float(now["cover"][2])
    if home and jumped:  # a new trial: forget the old one, including its last heights
        _HISTORY["dropped"] = False
        _HISTORY["z"] = [z]
        return
    hist = _HISTORY["z"]
    if -float(objects["cover"].lin_vel[2]) > DROP_SPEED or any(h - z > DROP_FALL for h in hist[-2:]):
        _HISTORY["dropped"] = True
    _HISTORY["z"] = (hist + [z])[-2:]


def score(objects):
    """Pass/fail: 1.0 when the cover lies flat across both bearers with both keys out of it, and it was never
    dropped; 0 otherwise."""
    _update(objects)
    cover = objects["cover"]
    if _HISTORY["dropped"] or not _keys_out(objects):
        return 0.0
    return 1.0 if on_bearers(cover) else 0.0


def success(objects):
    return score(objects) == 1.0


# -- scripted demo --------------------------------------------------------------------------------------------------
# Each arm pinches its key's handle (fingers along y), lifts it out of the holder, carries it over its keyhole with
# the crossbar lined up with the slot, lowers it through, turns it a quarter turn (left -90, right +90 degrees: within
# each wrist's range), and both arms lift the cover together, carry it over the bearers, lower it onto them, turn the
# keys back and draw them out. Both plans have the same timing.
GRIP_BELOW_TOP = 0.007
FINGER_HALF = 0.45 * FINGER_OPEN  # opened about 21 mm a side: enough for the 20 mm handle, clear of the other arm
LIFT_SAG = 0.012  # extra lift commanded while the cover hangs on the keys (to be measured)
SAG_Y = {"left": 0.0002, "right": -0.0016}  # measured at the keyholes (third GPU trace)
SAG_Z = 0.004
# First GPU trace: lowering the key at the keyhole, both arms came about 3.5 mm short in x.
SAG_X_HOLE = 0.003
# Pinching the handle turned both keys about 4.2 degrees (counter-clockwise from above) in the fingers; the wrist
# turns back by that much once the key is out of its holder, so the crossbar meets its slot square.
TWIST = -0.073


def _tcp(foot: float) -> float:
    """TCP height (absolute) with the key's foot `foot` above the table."""
    return TOP + foot + KEY_H - GRIP_BELOW_TOP


def _plan(side: str):
    s = 1.0 if side == "left" else -1.0
    hx, hy = HOLDER_XY[side]
    kx, ky = FRAME_XY[0] + SAG_X_HOLE, s * HOLE_Y - SAG_Y[side]
    px, py = BEARER_XY[0], ky
    turn = -s * math.pi / 2 + TWIST
    t0 = TWIST
    cover_bottom = LEDGE_TOP
    under = cover_bottom - 0.004 - CROSS[2]  # crossbar top 4 mm below the cover's underside
    lifted = 0.052  # cover underside 52 mm up: 12 mm over the frame
    on_b = BEARER[2]
    return [
        Waypoint(1.5, (hx, hy, _tcp(0.0) + 0.04), FINGER_HALF, "over the key", 0.0),
        Waypoint(1.2, (hx, hy, _tcp(0.0)), FINGER_HALF, "descend", 0.0),
        Waypoint(0.8, (hx, hy, _tcp(0.0)), FINGER_CLOSED, "pinch the handle", 0.0),
        Waypoint(1.2, (hx, hy, _tcp(0.045) + SAG_Z), FINGER_CLOSED, "draw it out", 0.0),
        Waypoint(1.8, (kx, ky, _tcp(LEDGE_TOP + COVER[2] + 0.010) + SAG_Z), FINGER_CLOSED, "over the keyhole", t0),
        Waypoint(1.2, (kx, ky, _tcp(under) + SAG_Z), FINGER_CLOSED, "through the keyhole", t0),
        Waypoint(1.0, (kx, ky, _tcp(under) + SAG_Z), FINGER_CLOSED, "quarter turn", turn),
        Waypoint(2.0, (kx, ky, _tcp(lifted - CROSS[2]) + SAG_Z + LIFT_SAG), FINGER_CLOSED, "lift the cover", turn),
        Waypoint(2.5, (px, py, _tcp(lifted - CROSS[2]) + SAG_Z + LIFT_SAG), FINGER_CLOSED, "carry", turn),
        Waypoint(2.0, (px, py, _tcp(on_b - CROSS[2] + 0.003) + SAG_Z + LIFT_SAG), FINGER_CLOSED, "lower", turn),
        Waypoint(1.0, (px, py, _tcp(on_b - 0.004 - CROSS[2]) + SAG_Z), FINGER_CLOSED, "set down", turn),
        Waypoint(1.0, (px, py, _tcp(on_b - 0.004 - CROSS[2]) + SAG_Z), FINGER_CLOSED, "turn back", t0),
        Waypoint(1.5, (px, py, _tcp(on_b + COVER[2] + 0.020) + SAG_Z), FINGER_CLOSED, "draw the keys out", t0),
    ]


def demo():
    return {"left": _plan("left"), "right": _plan("right")}
