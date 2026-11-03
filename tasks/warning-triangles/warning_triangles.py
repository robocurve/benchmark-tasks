"""On the station table: a 1:100 scale straight, two-lane rural road, fixed, runs left to right across the arms'
reach, 18 to 29 cm out: the lane nearer the arms, a yellow centre line, the far lane, a white edge line and a grey
shoulder. A fixed box truck (a white body 60 mm long, a blue cab 20 mm long at its left end, toward the left arm)
stands on the shoulder, centred on the midline. Three red warning-triangle cases (24 x 24 x 18 mm) lie on the table
in front of the road, 13 cm out.
"""

import itertools

import numpy as np

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

SCALE = 100
FT = 0.3048 / SCALE  # one foot at scale (m)

# The road runs along y. Its strips, out from the arms (x, m): each strip's near and far edge.
ROAD_H = 0.004
ROAD_TOP = TOP + ROAD_H
ROAD_LEN = 1.20
GREY, LINE_WHITE, LINE_YELLOW = (0.33, 0.33, 0.35), (0.95, 0.95, 0.95), (0.95, 0.80, 0.10)
STRIPS = [  # name, near x, far x, colour
    ("near_lane", 0.180, 0.216, GREY),  # 3.6 m lanes
    ("centre_line", 0.216, 0.222, LINE_YELLOW),
    ("far_lane", 0.222, 0.258, GREY),
    ("edge_line", 0.258, 0.262, LINE_WHITE),
    ("shoulder", 0.262, 0.292, (0.55, 0.55, 0.55)),  # 3 m
]
ROAD = [Box(name, pos=((a + b) / 2, 0.0, TOP + ROAD_H / 2), size=(b - a, ROAD_LEN, ROAD_H), color=c)
        for name, a, b, c in STRIPS]
SHOULDER_X = (STRIPS[4][1] + STRIPS[4][2]) / 2

# A box truck (about 8 m long, 2.6 m wide) on the shoulder, heading +y: the cab is at its +y end.
TRUCK_W = 0.026
BODY = (0.060, 0.035)  # length, height
CAB = (0.020, 0.028)
REAR_Y = -0.040
FRONT_Y = REAR_Y + BODY[0] + CAB[0]
TRUCK = [
    Box("truck_body", pos=(SHOULDER_X, REAR_Y + BODY[0] / 2, ROAD_TOP + BODY[1] / 2),
        size=(TRUCK_W, BODY[0], BODY[1]), color=(0.92, 0.92, 0.90)),
    Box("truck_cab", pos=(SHOULDER_X, REAR_Y + BODY[0] + CAB[0] / 2, ROAD_TOP + CAB[1] / 2),
        size=(TRUCK_W, CAB[0], CAB[1]), color=(0.15, 0.30, 0.65)),
]
TRAFFIC_FACE_X = SHOULDER_X - TRUCK_W / 2  # the truck's side toward the traffic lane

# WHY: oversize for 1:100 (a real case is about 45 cm), so the grippers can hold them.
CASE = (0.024, 0.024, 0.018)
CASE_START = {"triangle_a": (0.13, -0.10), "triangle_b": (0.13, -0.02), "triangle_c": (0.13, 0.10)}
TRIANGLES = [Box(n, pos=(x, y, TOP + CASE[2] / 2 + 0.001), size=CASE, mass=0.05, color=(0.85, 0.10, 0.10))
             for n, (x, y) in CASE_START.items()]
OBJECTS = [*ROAD, *TRUCK, *TRIANGLES]

# 49 CFR 392.22(b)(1), for a two-way road: one on the traffic side, 10 ft behind; one 100 ft behind and one 100 ft
# ahead, both in the centre of the lane or shoulder the truck is on. "Behind" is toward approaching traffic.
NEAR_FT = (5.0, 15.0)  # the rule's "4 paces (approximately 3 meters or 10 feet)"
FAR_FT = (75.0, 125.0)  # "40 paces (approximately 30 meters or 100 feet)"
# WHY: beside the truck's traffic-side face: the case's centre from one case width out into the far lane to the
# truck's side line.
TRAFFIC_SIDE_X = (TRAFFIC_FACE_X - CASE[0], TRAFFIC_FACE_X)
CENTRE_MM = 10.0  # within this of the shoulder's centre line
ON_ROAD_MM = 2.0  # a case's bottom within this of the road
FLAT_DEG = 10.0


def _flat(case) -> bool:
    """Lying on any of its faces, within FLAT_DEG."""
    x, y, z, w = (float(v) for v in case.quat_xyzw)
    up = (2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y))  # world z of each of the case's axes
    return max(abs(u) for u in up) >= np.cos(np.radians(FLAT_DEG))


def _on_road(case) -> bool:
    return abs(float(case.low[2]) - ROAD_TOP) <= ON_ROAD_MM / 1000 and _flat(case)


def _behind_ft(case) -> float:
    """How far behind the truck's rear the case's centre is (ft at scale); negative if it isn't behind."""
    return (REAR_Y - float(case.pos[1])) / FT


def _ahead_ft(case) -> float:
    return (float(case.pos[1]) - FRONT_Y) / FT


def _on_centre(case) -> bool:
    return abs(float(case.pos[0]) - SHOULDER_X) <= CENTRE_MM / 1000


SPOTS = {
    "10 ft behind, traffic side": lambda c: (NEAR_FT[0] <= _behind_ft(c) <= NEAR_FT[1]
                                             and TRAFFIC_SIDE_X[0] <= float(c.pos[0]) <= TRAFFIC_SIDE_X[1]),
    "100 ft behind": lambda c: FAR_FT[0] <= _behind_ft(c) <= FAR_FT[1] and _on_centre(c),
    "100 ft ahead": lambda c: FAR_FT[0] <= _ahead_ft(c) <= FAR_FT[1] and _on_centre(c),
}


def _filled(objects) -> int:
    """The most spots that hold a different case each, lying flat on the road."""
    cases = [objects[t.name] for t in TRIANGLES if _on_road(objects[t.name])]
    orders = itertools.permutations(SPOTS.values())
    return max(sum(check(case) for case, check in zip(cases, order)) for order in orders)


def success(objects):
    """All three spots filled and every case still; holds 1 s."""
    return _filled(objects) == 3 and all(objects[t.name].resting() for t in TRIANGLES)


def distance(objects):
    """Spots still empty."""
    return 3 - _filled(objects)


def score(objects):
    """Partial credit (success scores 1.0): a third for each spot holding a case."""
    return _filled(objects) / 3


TRAVEL_Z = TOP + 0.08
GRIP_Z = TOP + 0.017  # pad tips 13 mm below the TCP: 4 mm above the table, gripping the 18 mm case
PLACE_Z = ROAD_TOP + 0.019  # set down from 2 mm above the road
PLACE = {
    "triangle_a": (SHOULDER_X, REAR_Y - 100 * FT),
    "triangle_b": (TRAFFIC_FACE_X - CASE[0] / 2 + 0.002, REAR_Y - 10 * FT),
    "triangle_c": (SHOULDER_X, FRONT_Y + 100 * FT),
}


def _move(name):
    (px, py), (qx, qy) = CASE_START[name], PLACE[name]
    return pick_place_waypoints((px, py, GRIP_Z), (qx, qy, PLACE_Z), travel_z=TRAVEL_Z)


def demo():
    """The right arm sets the two triangles behind the truck, the far one first; the left arm sets the one ahead."""
    right = [*_move("triangle_a"), *_move("triangle_b")]
    left = [*_move("triangle_c"), Waypoint(1.0, (0.20, 0.20, TRAVEL_Z), FINGER_OPEN, "clear")]
    right.append(Waypoint(1.0, (0.20, -0.20, TRAVEL_Z), FINGER_OPEN, "clear"))
    return {"left": left, "right": right}
