"""On the station table: a car body (100 mm front to back, 70 mm side to side, 40 mm tall, fixed) stands on the
midline in front of the arms, its front (two yellow headlights) facing away from the arms. Its four wheels are off and
lie flat, one on each corner's dark pad: each wheel is a 44 mm disc, 22 mm thick, tagged by colour (front left red,
front right blue, rear left green, rear right yellow). The table around is free to set a wheel down on.

Goal: the wheels rotated in the rearward-cross pattern: rear wheels straight forward on the same side, front wheels
crossed to the opposite rear corner. Every wheel moves, and the four form one cycle, so one must be set aside first.

Run it:  inspect-robots run --embodiment yam_arena -E task=tire_rotation_cross.py -E object_jitter=0 \\
           --instruction "Rotate the car's wheels in the rearward-cross pattern: move each rear wheel straight forward to \\
           the front pad on the same side, and move each front wheel to the rear pad on the opposite side. Leave every \\
           wheel lying flat on its new pad." --scorer task_score ...
"""

import math

import numpy as np

from yam_arena.constants import FINGER_OPEN, PAD_TIP_PAST_TCP
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the car and its corner pads (fixed) ---------------------------------------------------------------------------
CAR_X, CAR_SIZE = 0.33, (0.100, 0.070, 0.040)
PAD_DX, PAD_Y, PAD_SIZE = 0.065, 0.075, (0.050, 0.050, 0.003)
PAD_TOP = TOP + PAD_SIZE[2]
CORNERS = {
    "front_left": (CAR_X + PAD_DX, PAD_Y), "front_right": (CAR_X + PAD_DX, -PAD_Y),
    "rear_left": (CAR_X - PAD_DX, PAD_Y), "rear_right": (CAR_X - PAD_DX, -PAD_Y),
}
OBJECTS = [
    Box("car_body", pos=(CAR_X, 0.0, TOP + CAR_SIZE[2] / 2), size=CAR_SIZE, color=(0.70, 0.72, 0.75)),
    *[Box(f"headlight_{s}", pos=(CAR_X + CAR_SIZE[0] / 2 + 0.003, d * 0.022, TOP + 0.028), size=(0.006, 0.014, 0.008),
          color=(1.0, 0.85, 0.2)) for s, d in (("left", 1), ("right", -1))],
    *[Box(f"pad_{c}", pos=(x, y, TOP + PAD_SIZE[2] / 2), size=PAD_SIZE, color=(0.22, 0.22, 0.24))
      for c, (x, y) in CORNERS.items()],
]

# -- the wheels: start corner -> rearward-cross target corner --------------------------------------------------------
WHEEL_R, WHEEL_H, WHEEL_MASS = 0.022, 0.022, 0.06
WHEELS = {
    "wheel_red": {"start": "front_left", "goal": "rear_right", "color": (0.80, 0.12, 0.10)},
    "wheel_blue": {"start": "front_right", "goal": "rear_left", "color": (0.12, 0.25, 0.80)},
    "wheel_green": {"start": "rear_left", "goal": "front_left", "color": (0.10, 0.60, 0.20)},
    "wheel_yellow": {"start": "rear_right", "goal": "front_right", "color": (0.95, 0.80, 0.10)},
}
OBJECTS += [Cylinder(n, pos=(*CORNERS[w["start"]], PAD_TOP + WHEEL_H / 2), radius=WHEEL_R, height=WHEEL_H,
                     mass=WHEEL_MASS, color=w["color"]) for n, w in WHEELS.items()]

# -- scoring -------------------------------------------------------------------------------------------------------
ON_PAD_MM, FLAT_DZ, FLAT_TILT_DEG = 12.0, 0.004, 8.0


def _tilt_deg(q):
    x, y, _, _ = (float(v) for v in q)
    return math.degrees(math.acos(min(1.0, abs(1 - 2 * (x * x + y * y)))))


def on_pad(st, corner):
    """Lying flat on that corner's pad: centre within 12 mm of the pad centre (side to side and front to back), within
    4 mm of its resting height, axis tilted under 8 degrees."""
    cx, cy = CORNERS[corner]
    near = math.hypot(st.pos[0] - cx, st.pos[1] - cy) * 1000 <= ON_PAD_MM
    return near and abs(st.pos[2] - (PAD_TOP + WHEEL_H / 2)) < FLAT_DZ and _tilt_deg(st.quat_xyzw) < FLAT_TILT_DEG


def score(objects):
    """0.25 per wheel lying flat on its rearward-cross corner."""
    return 0.25 * sum(on_pad(objects[n], w["goal"]) for n, w in WHEELS.items())


def success(objects):
    return score(objects) >= 0.999 and all(objects[n].resting() for n in WHEELS)


def distance(objects):
    return float(sum(math.hypot(objects[n].pos[0] - CORNERS[w["goal"]][0], objects[n].pos[1] - CORNERS[w["goal"]][1])
                     for n, w in WHEELS.items()))


# -- scripted demo -------------------------------------------------------------------------------------------------
TRAVEL_Z = TOP + 0.09
GRIP_ABOVE_BOTTOM = PAD_TIP_PAST_TCP + 0.008
RELEASE_GAP = 0.003
SETTLE_S = 0.8
OPEN_W = -(WHEEL_R + 0.012)
PARK = {"left": (0.22, 0.17), "right": (0.22, -0.17)}  # table spots to set a wheel aside
HANDOFF = (0.22, 0.0)  # table spot in front of the car, reachable by both arms
YAW = {"left": -math.pi / 2, "right": math.pi / 2}


def _move(src, dst, src_z, dst_z, yaw, label):
    pick = src_z + GRIP_ABOVE_BOTTOM
    drop = dst_z + RELEASE_GAP + GRIP_ABOVE_BOTTOM
    return [
        Waypoint(1.5, (*src, TRAVEL_Z), OPEN_W, label, yaw),
        Waypoint(1.2, (*src, pick), OPEN_W, "descend", yaw),
        Waypoint(0.8, (*src, pick), 0.0, "grip", yaw),
        Waypoint(1.0, (*src, TRAVEL_Z), 0.0, "lift", yaw),
        Waypoint(1.6, (*dst, TRAVEL_Z), 0.0, "carry", yaw),
        Waypoint(0.9, (*dst, drop + 0.012), 0.0, "lower", yaw),
        Waypoint(SETTLE_S, (*dst, drop + 0.012), 0.0, "settle", yaw),
        Waypoint(0.6, (*dst, drop), 0.0, "down", yaw),
        Waypoint(SETTLE_S, (*dst, drop), 0.0, "settle", yaw),
        Waypoint(0.5, (*dst, drop), OPEN_W, "let go", yaw),
        Waypoint(0.9, (*dst, TRAVEL_Z), OPEN_W, "up", yaw),
    ]


C = CORNERS


def _plan():
    """Steps as (arm, source, destination, source surface, destination surface, label)."""
    pad, tab = PAD_TOP, TOP
    return [
        ("left", C["front_left"], PARK["left"], pad, tab, "red aside"),
        ("left", C["rear_left"], C["front_left"], pad, pad, "green forward"),
        ("right", C["front_right"], HANDOFF, pad, tab, "blue to the handoff spot"),
        ("right", C["rear_right"], C["front_right"], pad, pad, "yellow forward"),
        ("left", HANDOFF, C["rear_left"], tab, pad, "blue to rear left"),
        ("left", PARK["left"], HANDOFF, tab, tab, "red to the handoff spot"),
        ("right", HANDOFF, C["rear_right"], tab, pad, "red to rear right"),
    ]


def demo():
    """Seven moves, one arm at a time; the idle arm waits clear at its side. Red is set aside, green and yellow move
    forward, blue and red cross to the rear through a handoff spot on the table in front of the car."""
    out = {"left": [], "right": []}
    t = {"left": 0.0, "right": 0.0}
    rest = {"left": (0.20, 0.25), "right": (0.20, -0.25)}
    for arm, src, dst, sz, dz, label in _plan():
        start = max(t["left"], t["right"])
        if start > t[arm]:
            out[arm].append(Waypoint(start - t[arm], (*rest[arm], TRAVEL_Z), FINGER_OPEN, "wait", YAW[arm]))
        seg = _move(src, dst, sz, dz, YAW[arm], label) + [
            Waypoint(1.2, (*rest[arm], TRAVEL_Z), FINGER_OPEN, "clear", YAW[arm])]
        out[arm] += seg
        t[arm] = start + sum(w.duration for w in seg)
    return out
