"""On the station table, a two-level serving pass in front of the arms: a 12 cm high wooden shelf with a second shelf
above it (its underside 21 cm up), both 20-24 cm wide and held by a back panel, open at the front and the sides. On the
lower shelf rests a rimmed serving tray (36 x 20 cm), wider than the pass so its two side rims stick out to the left
and right. In the middle of the tray, under the upper shelf, stands a soup bowl with a small side handle on its right,
holding the soup: 150 ml of water in a real trial, here 15 orange beads (8 mm), each standing for 10 ml, with the
rest of the soup's mass in the bowl. To the right, a round placemat lies on the table. The tray, the bowl and the
beads are loose; the pass and the placemat are fixed.

Task: unload the tray from the pass without spilling the soup, leaving the tray lying flat on the table and the bowl
standing on the blue placemat. The bowl cannot be reached under the upper shelf.

Run it:  inspect-robots run --embodiment yam_arena -E task=unload_a_tray_from_a_pass_shelf.py \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import Body, BoxShape, CylShape, SphereShape, on_reset, plate, rotation  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box, Cylinder  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- layout (world frame: x forward from the arm bases, y left, z up; metres) ----------------------------------------
SHELF_X0, SHELF_X1, SHELF_W, SHELF_H = 0.25, 0.37, 0.24, 0.12  # lower shelf: front and back edge, width, top height
UPPER_X0, UPPER_X1, UPPER_W, UPPER_Z, UPPER_T = 0.275, 0.41, 0.20, 0.21, 0.010  # upper shelf: its underside at UPPER_Z
# (its front edge leaves the front of the bowl in view but the bowl's handle under it)
BACK_X, BACK_T = 0.405, 0.010

TRAY_D, TRAY_W, TRAY_FLOOR, RIM_H, RIM_T = 0.20, 0.36, 0.006, 0.016, 0.005  # depth (x), width (y), floor, rims
TRAY_XY = (0.30, 0.0)
TRAY_MASS = 0.35

BOWL_R_I, BOWL_T, BOWL_H, BOWL_FLOOR = 0.045, 0.003, 0.050, 0.004
LUG_LEN, LUG_T, LUG_H, LUG_Z = 0.035, 0.008, 0.020, 0.040  # the handle: a fin out to -y, thin in x
BOWL_MASS = 0.25

BEAD_R, BEAD_MASS, N_BEADS = 0.004, 0.0015, 15
# the real soup: 150 ml of water (1 g per ml), about 2.4 cm deep in the bowl; each bead stands for 10 ml of it, and the
# soup's mass that the beads do not carry rides in the bowl's body, so the tray and the bowl carry the real load
SOUP_ML, BEAD_ML = 150, 10
SOUP_LOAD = SOUP_ML / 1000.0 - N_BEADS * BEAD_MASS  # kg, added to the bowl: 0.1275

MAT_XY, MAT_R = (0.32, -0.26), 0.065

WOOD, TRAY_C, BOWL_C, SOUP, MAT = (0.55, 0.38, 0.22), (0.20, 0.22, 0.25), (0.93, 0.93, 0.90), (0.93, 0.45, 0.10), (0.25, 0.45, 0.70)

# -- scoring --------------------------------------------------------------------------------------------------------
SPILL_FULL_ML, SPILL_HALF_ML = 10, 30  # at most this much soup spilled for 1.0, and for 0.5 or 0.25
SPILL_FULL, SPILL_HALF = SPILL_FULL_ML // BEAD_ML, SPILL_HALF_ML // BEAD_ML  # in beads out of the bowl: 1 and 3
ON_MAT = 0.04  # the bowl's centre within this of the placemat's centre
UPRIGHT, TIPPED = math.radians(15), math.radians(45)
SUCCESS_HOLD_S = 1.0


def _tray() -> Body:
    """Origin at the centre of the tray's underside."""
    hd, hw = TRAY_D / 2, TRAY_W / 2
    z = TRAY_FLOOR + RIM_H / 2
    shapes = [BoxShape((0.0, 0.0, TRAY_FLOOR / 2), (TRAY_D, TRAY_W, TRAY_FLOOR), TRAY_C),
              BoxShape((0.0, hw - RIM_T / 2, z), (TRAY_D, RIM_T, RIM_H), TRAY_C),
              BoxShape((0.0, -hw + RIM_T / 2, z), (TRAY_D, RIM_T, RIM_H), TRAY_C),
              BoxShape((hd - RIM_T / 2, 0.0, z), (RIM_T, TRAY_W, RIM_H), TRAY_C),
              BoxShape((-hd + RIM_T / 2, 0.0, z), (RIM_T, TRAY_W, RIM_H), TRAY_C)]
    return Body.build("tray", pos=(*TRAY_XY, TOP + SHELF_H + 0.0005), mass=TRAY_MASS, shapes=shapes, friction=(0.5, 0.4),
                      color=TRAY_C)


BOWL_Z0 = TOP + SHELF_H + 0.0005 + TRAY_FLOOR + 0.0005  # the bowl's underside, on the tray


def _bowl() -> Body:
    """Origin at the centre of the bowl's underside."""
    n, r = 20, BOWL_R_I + BOWL_T / 2
    shapes = [CylShape((0.0, 0.0, BOWL_FLOOR / 2), BOWL_R_I + BOWL_T, BOWL_FLOOR, BOWL_C)]
    for k in range(n):
        a = 2 * math.pi * k / n
        u = (math.cos(a), math.sin(a), 0.0)
        shapes.append(plate((r * u[0], r * u[1], BOWL_FLOOR), (r * u[0], r * u[1], BOWL_FLOOR + BOWL_H),
                            2 * r * math.tan(math.pi / n) + 0.001, BOWL_T, u, BOWL_C))
    shapes.append(BoxShape((0.0, -(BOWL_R_I + BOWL_T + LUG_LEN / 2 - 0.001), LUG_Z), (LUG_T, LUG_LEN, LUG_H), BOWL_C))
    return Body.build("bowl", pos=(TRAY_XY[0], TRAY_XY[1], BOWL_Z0), mass=BOWL_MASS + SOUP_LOAD, shapes=shapes,
                      friction=(0.6, 0.5),
                      color=BOWL_C)


def _bead_spots() -> list[tuple[float, float, float]]:
    d = 2 * BEAD_R + 0.0008
    rmax = BOWL_R_I - BEAD_R - 0.0015
    z = BOWL_FLOOR + BEAD_R + 0.0005
    pts = [((i + 0.5 * (j % 2)) * d, j * d * math.sqrt(3) / 2, z) for j in range(-6, 7) for i in range(-6, 7)]
    pts = sorted([p for p in pts if math.hypot(p[0], p[1]) <= rmax], key=lambda p: (round(math.hypot(p[0], p[1]), 6), p))
    return pts[:N_BEADS]


def _beads() -> list[Body]:
    bx, by = TRAY_XY
    return [Body.build(f"soup_{i:02d}", pos=(bx + x, by + y, BOWL_Z0 + z), mass=BEAD_MASS, color=SOUP,
                       friction=(0.3, 0.25), shapes=[SphereShape((0.0, 0.0, 0.0), BEAD_R, SOUP)], damping=(0.05, 0.3))
            for i, (x, y, z) in enumerate(_bead_spots())]


def _pass() -> list[Box]:
    return [
        Box("pass_shelf", pos=((SHELF_X0 + SHELF_X1) / 2, 0.0, TOP + SHELF_H / 2),
            size=(SHELF_X1 - SHELF_X0, SHELF_W, SHELF_H), color=WOOD),
        Box("pass_back", pos=(BACK_X, 0.0, TOP + (UPPER_Z + UPPER_T) / 2), size=(BACK_T, UPPER_W, UPPER_Z + UPPER_T),
            color=WOOD),
        Box("pass_upper", pos=((UPPER_X0 + UPPER_X1) / 2, 0.0, TOP + UPPER_Z + UPPER_T / 2),
            size=(UPPER_X1 - UPPER_X0, UPPER_W, UPPER_T), color=WOOD),
    ]


TRAY = _tray()
BOWL = _bowl()
BEADS = _beads()
OBJECTS = [*_pass(), Cylinder("placemat", pos=(*MAT_XY, TOP + 0.001), radius=MAT_R, height=0.002, color=MAT),
           TRAY, BOWL, *BEADS]
BEAD_NAMES = [b.name for b in BEADS]


# -- scoring --------------------------------------------------------------------------------------------------------

_EPISODE = {"bowl_tipped": False}


@on_reset
def _new_episode() -> None:
    _EPISODE.update(bowl_tipped=False)


def _tilt(rot) -> float:
    return float(math.acos(max(-1.0, min(1.0, rot[2, 2]))))


def _tally(objects) -> dict:
    tray, bowl = objects["tray"], objects["bowl"]
    t_rot, b_rot = rotation(tray.quat_xyzw), rotation(bowl.quat_xyzw)
    in_bowl = 0
    for name in BEAD_NAMES:
        p = b_rot.T @ (np.asarray(objects[name].pos, float) - bowl.pos)
        if math.hypot(p[0], p[1]) < BOWL_R_I and BOWL_FLOOR - 0.002 < p[2] < BOWL_FLOOR + BOWL_H:
            in_bowl += 1
    bowl_tilt = _tilt(b_rot)
    if bowl_tilt >= TIPPED:
        _EPISODE["bowl_tipped"] = True
    tray_flat_on_table = abs(tray.pos[2] - TOP) < 0.01 and _tilt(t_rot) < math.radians(10)
    tray_out = tray.pos[0] + TRAY_D / 2 < SHELF_X0  # wholly in front of the lower shelf
    bowl_on_mat = (math.hypot(bowl.pos[0] - MAT_XY[0], bowl.pos[1] - MAT_XY[1]) <= ON_MAT
                   and abs(bowl.pos[2] - (TOP + 0.002)) < 0.01)
    return {"in_bowl": in_bowl, "spilled": N_BEADS - in_bowl, "bowl_tilt": bowl_tilt, "bowl_up": bowl_tilt < UPRIGHT,
            "tray_on_table": tray_flat_on_table, "tray_out": tray_out, "bowl_on_mat": bowl_on_mat, **_EPISODE}


def _points(t) -> float:
    if t["bowl_tipped"] or not t["bowl_up"]:
        return 0.0
    if t["bowl_on_mat"] and t["tray_on_table"] and t["spilled"] <= SPILL_FULL:
        return 1.0
    if t["tray_on_table"] and t["spilled"] <= SPILL_HALF:
        return 0.5
    if t["tray_out"] and t["spilled"] <= SPILL_HALF:
        return 0.25
    return 0.0


_DEBUG = {"on": bool(os.environ.get("TRAY_DEBUG")), "calls": 0}


def score(objects) -> float:
    """0 if the bowl is ever tipped 45 degrees or ends tilted 15 degrees or more. Otherwise 1.0 with the bowl on the
    placemat, the tray flat on the table and at most 1 of the 15 beads (10 ml) out of the bowl; 0.5 with the tray flat on
    the table and at most 3 beads (30 ml) out, whether or not the bowl is on the placemat; 0.25 with the tray pulled
    wholly out in front of the shelf and at most 3 beads out; else 0."""
    t = _tally(objects)
    if _DEBUG["on"]:  # TRAY_DEBUG=1: one line a second (at 20 Hz) on stderr
        _DEBUG["calls"] += 1
        if _DEBUG["calls"] % 20 == 0:
            tray, bowl = objects["tray"], objects["bowl"]
            print(f"[tray] t={_DEBUG['calls'] / 20:5.1f}s in_bowl={t['in_bowl']} tray=({tray.pos[0]:.3f},{tray.pos[1]:.3f},"
                  f"{tray.pos[2] - TOP:.3f}) bowl=({bowl.pos[0]:.3f},{bowl.pos[1]:.3f},{bowl.pos[2] - TOP:.3f}) "
                  f"bowl_tilt={math.degrees(t['bowl_tilt']):.0f}deg out={t['tray_out']} on_table={t['tray_on_table']} "
                  f"on_mat={t['bowl_on_mat']} score={_points(t)}", file=sys.stderr, flush=True)
    return _points(t)


def success(objects) -> bool:
    """Full score with the tray, the bowl and every bead still."""
    if _points(_tally(objects)) < 1.0:
        return False
    return all(objects[n].resting() for n in ("tray", "bowl", *BEAD_NAMES))


# -- scripted demo --------------------------------------------------------------------------------------------------
# Both grips come straight down on the tray's side rims (fingers closing across each rim), lift it 1 cm off the shelf,
# draw it straight back until it is clear of the shelf, lower it to the table and let go. Then the right arm takes the
# bowl by its handle (fingers closing front and back across the fin) and sets it on the placemat.

RIM_Y = TRAY_W / 2 - RIM_T / 2
GRIP_DOWN = PAD_TIP_PAST_TCP + RIM_H - 0.004  # TCP above the tray's floor top: pad tips 4 mm above the floor
CLEAR = 0.01  # lift off the shelf before drawing back
OUT_X = SHELF_X0 - TRAY_D / 2 - 0.012  # tray centre once it is clear of the shelf


def _w(t, xyz, finger, label, yaw=0.0) -> Waypoint:
    return Waypoint(t, tuple(float(v) for v in xyz), finger, label, yaw)


def _tray_grip(x: float, floor_z: float, side: float):
    return (x, side * RIM_Y, floor_z + TRAY_FLOOR + GRIP_DOWN)


def _arm(arm: str) -> list[Waypoint]:
    side = 1.0 if arm == "left" else -1.0
    shelf_floor = TOP + SHELF_H + 0.0005
    table_floor = TOP + 0.002
    x0 = TRAY_XY[0]
    out = [
        _w(2.5, _tray_grip(x0, shelf_floor + 0.035, side), FINGER_OPEN, "over the tray's rim"),
        _w(1.2, _tray_grip(x0, shelf_floor, side), FINGER_OPEN, "down round the rim"),
        _w(0.8, _tray_grip(x0, shelf_floor, side), FINGER_CLOSED, "grip the rim"),
        _w(1.0, _tray_grip(x0, shelf_floor + CLEAR, side), FINGER_CLOSED, "lift off the shelf"),
        _w(3.0, _tray_grip(OUT_X, shelf_floor + CLEAR, side), FINGER_CLOSED, "draw it out"),
        _w(3.0, _tray_grip(OUT_X, table_floor, side), FINGER_CLOSED, "lower it to the table"),
        _w(0.8, _tray_grip(OUT_X, table_floor, side), FINGER_OPEN, "let go"),
        _w(1.0, _tray_grip(OUT_X, table_floor + 0.05, side), FINGER_OPEN, "up"),
    ]
    if arm == "left":
        return out
    # the bowl now stands at the middle of the tray, on the table
    bx, by = OUT_X, TRAY_XY[1]
    lug_y = by - (BOWL_R_I + BOWL_T + LUG_LEN / 2)
    bowl_floor = table_floor + TRAY_FLOOR + 0.0005
    lug_z = bowl_floor + LUG_Z + 0.004
    carry = lug_z + 0.05
    mx, my = MAT_XY[0], MAT_XY[1] - (BOWL_R_I + BOWL_T + LUG_LEN / 2)
    set_z = TOP + 0.002 + LUG_Z + 0.004 + 0.003
    yaw = math.pi / 2  # fingers close along x, across the fin
    out += [
        _w(1.5, (bx, lug_y, carry), FINGER_OPEN, "over the bowl's handle", yaw),
        _w(1.0, (bx, lug_y, lug_z), FINGER_OPEN, "down round the handle", yaw),
        _w(0.8, (bx, lug_y, lug_z), FINGER_CLOSED, "grip the handle", yaw),
        _w(1.2, (bx, lug_y, carry), FINGER_CLOSED, "lift the bowl", yaw),
        _w(2.5, (mx, my, carry), FINGER_CLOSED, "carry it to the placemat", yaw),
        _w(1.2, (mx, my, set_z), FINGER_CLOSED, "set it down", yaw),
        _w(0.8, (mx, my, set_z), FINGER_OPEN, "let go", yaw),
        _w(1.0, (mx, my, set_z + 0.05), FINGER_OPEN, "up", yaw),
    ]
    return out


def demo():
    return {"left": _arm("left"), "right": _arm("right")}
