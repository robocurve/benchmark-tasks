"""On the station table: a fixed plastic expansion adapter stands in a clamp block with its 24/29 socket facing up.
In front of the arms, in a row running left to right, four loose plastic (PTFE) expansion adapters stand upright in
grey collars, each with a cone at the bottom and a socket at the top:

    14/23 cone -> 19/26 socket
    19/26 cone -> 29/32 socket
    24/29 cone -> 29/32 socket   (the only one that mates with the fixed 24/29 socket)
    29/32 cone -> 34/35 socket

Only one adapter has a 24/29 cone, and every upper end is a socket, so no other part can be seated in the 24/29 socket
either way up. The 29/32 cone is too wide to enter; the 14/23 and 19/26 cones drop in loose (a leaking joint) and
score 0.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \\
           -E cam_width=320 -E cam_height=176 --instruction "<INSTRUCTION below>" --scorer task_score --max-steps 2400
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from partkit import part  # noqa: E402
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

INSTRUCTION = "pick the adapter with the 24/29 cone and seat it in the 24/29 socket"

PTFE_WHITE = (0.93, 0.93, 0.90)   # the loose adapters and the fixed one: all the same plastic, so colour is no cue
COLLAR_GREY = (0.55, 0.56, 0.58)
CLAMP_GREY = (0.30, 0.30, 0.32)
PLASTIC_FRICTION = 0.4            # plastic on plastic; enough for the gripper (the parts weigh 30-60 g)

# World frame: x forward from the arm bases, y to the robot's left, z up, metres. Arm bases at x 0, y +-0.305.

# -- the fixed adapter: a 24/29 socket on a clamp block ------------------------------------------------------------------
SOCKET_XY = (0.30, 0.0)
CLAMP = (0.050, 0.050, 0.060)
FLOOR_Z = TOP + CLAMP[2]                    # the socket's floor
BORE, WALL, DEPTH = 0.027, 0.005, 0.029     # a 24 mm cone + 1.5 mm per side; 29 mm deep (a 24/29 joint)
RIM_Z = FLOOR_Z + DEPTH

# -- the loose adapters. Origin at the centre of the cone's lower end. Cone (d, L), a 15 mm neck, then the upper socket,
# solid in the sim, 6 mm wider outside than its joint size ----------------------------------------------------------------
NECK_R, NECK_H = 0.009, 0.015
ROW_X = 0.17
ADAPTERS = {
    # name: (y in the row, cone diameter, cone length, upper socket outer diameter, upper socket length, mass kg)
    "adapter_29_32": (0.15, 0.029, 0.032, 0.040, 0.035, 0.060),   # 29/32 cone -> 34/35 socket
    "adapter_14_23": (0.05, 0.014, 0.023, 0.025, 0.026, 0.030),   # 14/23 cone -> 19/26 socket
    "adapter_24_29": (-0.05, 0.024, 0.029, 0.035, 0.032, 0.045),  # 24/29 cone -> 29/32 socket: the right one
    "adapter_19_26": (-0.15, 0.019, 0.026, 0.035, 0.032, 0.040),  # 19/26 cone -> 29/32 socket
}
TARGET = "adapter_24_29"
COLLAR_PLAY, COLLAR_WALL, COLLAR_H = 0.002, 0.004, 0.020   # each cone stands in a collar 4 mm wider than it


def _adapter(name, y, d, cone_l, top_d, top_l, kg):
    return part(
        name, pos=(ROW_X, y, TOP + 0.0005), mass=kg, color=PTFE_WHITE, friction=PLASTIC_FRICTION,
        boxes=[((0.0, 0.0, cone_l / 2), (0.006, 0.006, 0.006), PTFE_WHITE)],  # partkit needs one box; hidden inside
        cylinders=[
            ((0.0, 0.0, cone_l / 2), d / 2, cone_l, PTFE_WHITE),
            ((0.0, 0.0, cone_l + NECK_H / 2), NECK_R, NECK_H, PTFE_WHITE),
            ((0.0, 0.0, cone_l + NECK_H + top_l / 2), top_d / 2, top_l, PTFE_WHITE),
        ],
    )


def _cell(prefix, centre_xy, inner, wall, z0, z1, color):
    """Four fixed walls around a square opening, sides along the world axes."""
    cx, cy = centre_xy
    off, outer, z, hh = inner / 2 + wall / 2, inner + 2 * wall, (z0 + z1) / 2, z1 - z0
    return [Box(f"{prefix}_n", pos=(cx, cy + off, z), size=(outer, wall, hh), color=color),
            Box(f"{prefix}_s", pos=(cx, cy - off, z), size=(outer, wall, hh), color=color),
            Box(f"{prefix}_e", pos=(cx + off, cy, z), size=(wall, inner, hh), color=color),
            Box(f"{prefix}_w", pos=(cx - off, cy, z), size=(wall, inner, hh), color=color)]


OBJECTS = [
    Box("clamp_block", pos=(*SOCKET_XY, TOP + CLAMP[2] / 2), size=CLAMP, color=CLAMP_GREY),
    *_cell("socket", SOCKET_XY, BORE, WALL, FLOOR_Z, RIM_Z, PTFE_WHITE),
    *[w for n, (y, d, *_r) in ADAPTERS.items()
      for w in _cell(f"collar_{n}", (ROW_X, y), d + 2 * COLLAR_PLAY, COLLAR_WALL, TOP, TOP + COLLAR_H, COLLAR_GREY)],
    *[_adapter(n, *spec) for n, spec in ADAPTERS.items()],
]

# -- scoring -----------------------------------------------------------------------------------------------------------
SEAT_Z_TOL = 0.003      # the cone's lower end within 3 mm of the socket floor
SEAT_XY_TOL = 0.003     # 1.5 mm bore play, plus margin
SEAT_TILT_DEG = 4.0
ENTER_DEPTH = 0.010     # partial credit: the cone's lower end at least 10 mm below the rim
ENTER_TILT_DEG = 20.0


def _tilt_deg(state) -> float:
    return math.degrees(math.acos(max(-1.0, min(1.0, abs(float(state.rotation()[2, 2]))))))


def _off_axis(state) -> float:
    return math.hypot(float(state.pos[0]) - SOCKET_XY[0], float(state.pos[1]) - SOCKET_XY[1])


def _in_socket(state) -> bool:
    """The adapter's lower end is down inside the bore: its centre more than 2 mm below the rim and inside the bore,
    seen from above."""
    return float(state.pos[2]) < RIM_Z - 0.002 and _off_axis(state) <= BORE / 2


def seated(state) -> bool:
    return (abs(float(state.pos[2]) - FLOOR_Z) <= SEAT_Z_TOL and _off_axis(state) <= SEAT_XY_TOL
            and _tilt_deg(state) <= SEAT_TILT_DEG)


def entered(state) -> bool:
    return (float(state.pos[2]) <= RIM_Z - ENTER_DEPTH and _off_axis(state) <= BORE / 2
            and _tilt_deg(state) <= ENTER_TILT_DEG)


def _wrong_in_socket(objects) -> bool:
    return any(_in_socket(objects[n]) for n in ADAPTERS if n != TARGET)


def success(objects) -> bool:
    """The 24/29 adapter seated, nothing else in the socket; must hold for 1 s."""
    return seated(objects[TARGET]) and not _wrong_in_socket(objects)


def score(objects) -> float:
    """0 if any other adapter is down in the socket (only the 14/23 and 19/26 cones can get there). Otherwise 1.0 for
    the 24/29 adapter seated, 0.5 for its cone at least 10 mm into the socket, else 0."""
    if _wrong_in_socket(objects):
        return 0.0
    target = objects[TARGET]
    if seated(target):
        return 1.0
    return 0.5 if entered(target) else 0.0


def distance(objects) -> float:
    t = objects[TARGET]
    return math.hypot(_off_axis(t), float(t.pos[2]) - FLOOR_Z)


# -- scripted demo: the right arm alone (the 24/29 adapter is on the robot's right) -------------------------------------
SQUEEZE = 0.008
TRAVEL = TOP + 0.20


def demo():
    """The right arm grips the 24/29 adapter by its upper socket (a neck under a wider top can't be reached from
    above), lifts it out of its collar, carries it over the fixed socket, lowers it to 5 mm above the rim, pushes it
    slowly to the floor and lets go."""
    y, d, cone_l, top_d, top_l, _ = ADAPTERS[TARGET]
    grip_h = cone_l + NECK_H + top_l - 0.015      # TCP 15 mm below the top; pads 13 mm lower, on the upper socket
    g = -(top_d - SQUEEZE) / 2
    sx, sy = SOCKET_XY
    above_rim = RIM_Z + 0.005 + grip_h
    home = FLOOR_Z + grip_h + 0.001
    right = [
        Waypoint(1.5, (ROW_X, y, TRAVEL), FINGER_OPEN, "over the 24/29 adapter"),
        Waypoint(1.5, (ROW_X, y, TOP + grip_h), FINGER_OPEN, "descend"),
        Waypoint(0.6, (ROW_X, y, TOP + grip_h), FINGER_OPEN, "settle"),
        Waypoint(1.5, (ROW_X, y, TOP + grip_h), g, "close"),
        Waypoint(1.5, (ROW_X, y, TRAVEL), g, "lift out of the collar"),
        Waypoint(2.0, (sx, sy, TRAVEL), g, "carry over the socket"),
        Waypoint(0.6, (sx, sy, TRAVEL), g, "settle"),
        Waypoint(1.2, (sx, sy, above_rim), g, "lower to the rim"),
        Waypoint(0.8, (sx, sy, above_rim), g, "settle at the rim"),
        Waypoint(2.5, (sx, sy, home), g, "seat"),
        Waypoint(0.8, (sx, sy, home), g, "hold"),
        Waypoint(1.5, (sx, sy, home), FINGER_OPEN, "release"),
        Waypoint(1.2, (sx, sy, TRAVEL), FINGER_OPEN, "retreat"),
    ]
    return {"right": right}
