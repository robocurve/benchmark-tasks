"""A commercial-kitchen mock; the table top is the counter. Nearer the arms on the robot's right is a small
handwashing sink (stainless open-top basin of fixed walls, 12 x 12 cm inside, 13 cm tall) with a blue soap
dispenser (4 x 4 x 17 cm block) standing against its back wall and a white paper-towel dispenser beside it.
Farther back on the robot's left is the dishwashing sink (stainless basin, 20 x 18 cm inside, 15 cm tall).
A dirty coffee mug (a white ceramic cylinder, 7 cm across, 9 cm tall, 300 g, no handle in sim) stands on the
counter in front of the arms.
"""

import numpy as np

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
INSTRUCTION = "Put the dirty mug in the sink to soak."

T = 0.01
STEEL = (0.75, 0.76, 0.78)
SINKS = {  # name: (center x, center y, inside x, inside y, wall height)
    "hand_sink": (0.22, -0.15, 0.12, 0.12, 0.13),
    "dish_sink": (0.33, 0.11, 0.20, 0.18, 0.15),
}


def _basin(name, cx, cy, ix, iy, H):
    hx, hy = ix / 2 + T / 2, iy / 2 + T / 2
    z = TOP + H / 2
    return [
        Box(f"{name}_front", pos=(cx - hx, cy, z), size=(T, iy + 2 * T, H), color=STEEL),
        Box(f"{name}_back", pos=(cx + hx, cy, z), size=(T, iy + 2 * T, H), color=STEEL),
        Box(f"{name}_left", pos=(cx, cy + hy, z), size=(ix, T, H), color=STEEL),
        Box(f"{name}_right", pos=(cx, cy - hy, z), size=(ix, T, H), color=STEEL),
    ]


HX, HYC = SINKS["hand_sink"][:2]
SOAP = Box("soap_dispenser", pos=(HX + 0.06 + T + 0.02, HYC + 0.03, TOP + 0.085), size=(0.04, 0.04, 0.17),
           color=(0.15, 0.40, 0.90))
TOWELS = Box("towel_dispenser", pos=(HX + 0.06 + T + 0.02, HYC - 0.035, TOP + 0.085), size=(0.04, 0.05, 0.17),
             color=(0.97, 0.97, 0.97))
MUG_H = 0.09
MUG = Cylinder("mug", pos=(0.14, 0.0, TOP + MUG_H / 2), radius=0.035, height=MUG_H, mass=0.3, color=(0.95, 0.95, 0.93))
OBJECTS = [*(p for n, spec in SINKS.items() for p in _basin(n, *spec)), SOAP, TOWELS, MUG]


def _upright(s, max_tilt_deg=20.0):
    qx, qy = s.quat_xyzw[0], s.quat_xyzw[1]
    return 1 - 2 * (qx * qx + qy * qy) > np.cos(np.radians(max_tilt_deg))


def _in(s, sink):
    """Still, its center inside the sink's walls and its bottom on the sink floor."""
    cx, cy, ix, iy, _h = SINKS[sink]
    return bool(s.resting() and abs(s.pos[0] - cx) < ix / 2 and abs(s.pos[1] - cy) < iy / 2
                and s.low[2] < TOP + 0.015)


def success(objects):
    """1.0: the mug stands upright on the dishwashing sink's floor."""
    p = objects["mug"]
    return _in(p, "dish_sink") and _upright(p)


def score(objects):
    """1.0 upright in the dishwashing sink; 0.5 in the dishwashing sink but on its side (it cannot hold soaking
    water); 0 in the handwashing sink, still on the counter, on a rim, or held."""
    p = objects["mug"]
    if not _in(p, "dish_sink"):
        return 0.0
    return 1.0 if _upright(p) else 0.5


def demo():
    """Left arm: grip the mug 2 cm below its rim, carry it over the counter and set it down upright in the
    middle of the dishwashing sink, released 1 cm above the sink floor."""
    travel = TOP + 0.27  # mug bottom clears the 15 cm walls
    grip = (MUG.pos[0], MUG.pos[1], TOP + MUG_H - 0.02 + 0.013)
    cx, cy = SINKS["dish_sink"][:2]
    place = (cx, cy, TOP + 0.01 + MUG_H - 0.02 + 0.013)
    left = pick_place_waypoints(grip, place, travel_z=travel)
    left.append(Waypoint(1.5, (0.15, 0.25, TOP + 0.25), FINGER_OPEN, "park"))
    return {"left": left}
