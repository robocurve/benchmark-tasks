"""On the station table, in front of the left arm: a dark grey silicon wafer, a disc 80 mm across and 6 mm thick,
lies flat on the tips of three thin steel lift pins 30 mm tall. Further out and nearer the midline stands a fixed,
open-topped, single-slot wafer cassette: two white side walls 60 mm tall with 92 mm between their inner faces, a
narrow pale ledge 30 mm high running along the foot of each wall, open toward the robot, and two dark stop posts
at its far end with a gap between them. Nothing else is on the table.
"""

import math

import numpy as np

from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN, Waypoint
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# World frame: x away from the arm bases, y to the robot's left, z up (m).
STEEL, DARK = (0.62, 0.64, 0.67), (0.18, 0.19, 0.21)
WALL_C, LEDGE_C = (0.92, 0.92, 0.90), (0.80, 0.78, 0.66)
SILICON = (0.27, 0.29, 0.34)

# Wafer, scaled: 80 mm across and 6 mm thick (a real 200 mm wafer is 0.725 mm thick).
R, H = 0.040, 0.006
WAFER_MASS = 0.030

# Lift pins: three steel pins on a 25 mm radius around the wafer's start centre.
PIN_R, PIN_H = 0.004, 0.030
PIN_RING = 0.025
PIN_XY = (0.20, 0.22)
PIN_ANGLES_DEG = (90.0, 210.0, 330.0)
PINS = [
    Cylinder(f"lift_pin_{i + 1}", pos=(PIN_XY[0] + PIN_RING * math.cos(math.radians(a)),
                                        PIN_XY[1] + PIN_RING * math.sin(math.radians(a)), TOP + PIN_H / 2),
             radius=PIN_R, height=PIN_H, color=STEEL)
    for i, a in enumerate(PIN_ANGLES_DEG)
]
WAFER_START = (PIN_XY[0], PIN_XY[1], TOP + PIN_H + H / 2)
WAFER = Cylinder("wafer", pos=WAFER_START, radius=R, height=H, mass=WAFER_MASS, color=SILICON)

# Cassette: one slot along x, open toward the robot and at the top.
YC = 0.12  # slot centre line
STOP_X = 0.38  # near face of the two stop posts
FRONT_X = 0.26  # near end of the walls and ledges
HALF_GAP = 0.046  # wall inner faces at YC +- 46 mm
WALL_T, WALL_H = 0.010, 0.060
LEDGE_W, LEDGE_H = 0.014, 0.030  # ledges from |y - YC| = 32 to 46 mm, top 30 mm above the table
LEDGE_TOP = TOP + LEDGE_H
POST = (0.010, 0.010, 0.045)  # stop posts: 10 x 10 mm, 45 mm tall
POST_INNER = 0.026  # their inner faces at YC +- 26 mm: 52 mm between them
WALL_X1 = STOP_X + POST[0]

_wall_len = WALL_X1 - FRONT_X
_ledge_len = STOP_X - FRONT_X
CASSETTE = []
for side, s in (("left", 1.0), ("right", -1.0)):
    CASSETTE += [
        Box(f"cassette_wall_{side}", pos=((FRONT_X + WALL_X1) / 2, YC + s * (HALF_GAP + WALL_T / 2), TOP + WALL_H / 2),
            size=(_wall_len, WALL_T, WALL_H), color=WALL_C),
        Box(f"cassette_ledge_{side}", pos=((FRONT_X + STOP_X) / 2, YC + s * (HALF_GAP - LEDGE_W / 2), TOP + LEDGE_H / 2),
            size=(_ledge_len, LEDGE_W, LEDGE_H), color=LEDGE_C),
        Box(f"stop_post_{side}", pos=(STOP_X + POST[0] / 2, YC + s * (POST_INNER + POST[1] / 2), TOP + POST[2] / 2),
            size=POST, color=DARK),
    ]
POSTS = [o for o in CASSETTE if o.name.startswith("stop_post")]

OBJECTS = [*PINS, WAFER, *CASSETTE]

# Seated: the wafer, centred on the slot, touches both stop posts at their inner near corners.
SEAT_X = STOP_X - math.sqrt(R**2 - POST_INNER**2)  # 0.3496

EPS = 1e-6  # every limit below is inclusive
FLAT_DEG = 2.0  # lying flat: tilt at most 2 degrees
ON_TOL = 0.001  # its underside within 1 mm of the ledge tops
FRONT_MARGIN = 0.020  # centre at least 20 mm inside the ledges' near ends
STOP_GAP = 0.002  # at the stop: within 2 mm of each post
DROP_SPEED = 0.5  # m/s: what a fall of about 13 mm gives
# At rest is judged from the wafer's pose, not the simulator's velocity: lying still on the two ledges the simulator
# reports a steady 2.4 cm/s and 1.1 rad/s for it although its pose does not change (measured on the L40S).
REST_STEPS = 10  # the last 10 control steps (0.5 s at 20 Hz)
REST_MOVE = 0.0005  # m: the centre moved at most 0.5 mm over them
REST_TURN_DEG = 0.5  # and the wafer turned at most 0.5 degrees
RESET_NEAR = 0.0005  # scene reset: back within 0.5 mm of its start ...
RESET_JUMP = 0.020  # ... by a jump of more than 20 mm in one control step

# History over the trial, kept between calls: the wafer's recent poses and whether it was ever dropped.
_hist = {"poses": [], "dropped": False, "last": None}


def _tilt_deg(st) -> float:
    """Angle between the wafer's face and the horizontal; either face up counts the same (the disc has no
    visible top side)."""
    x, y, _, _ = (float(v) for v in st.quat_xyzw)
    return float(np.degrees(np.arccos(np.clip(abs(1 - 2 * (x * x + y * y)), 0.0, 1.0))))


def _turn_deg(q1, q2) -> float:
    d = abs(float(np.dot(q1, q2)))
    return float(np.degrees(2 * np.arccos(np.clip(d, 0.0, 1.0))))


def _track(objects) -> None:
    """Record this step's wafer state once (success() and score() see the same state each step)."""
    w = objects["wafer"]
    if _hist["last"] is w:
        return
    prev = _hist["poses"][-1][0] if _hist["poses"] else None
    near_start = np.linalg.norm(w.pos - np.asarray(WAFER_START)) <= RESET_NEAR + EPS
    if near_start and (prev is None or np.linalg.norm(w.pos - prev) > RESET_JUMP):
        _hist["dropped"] = False  # a scene reset, which the robot cannot produce
        _hist["poses"] = []
    if float(np.linalg.norm(w.lin_vel)) > DROP_SPEED:
        _hist["dropped"] = True
    _hist["poses"] = (_hist["poses"] + [(np.array(w.pos, float), np.array(w.quat_xyzw, float))])[-REST_STEPS:]
    _hist["last"] = w


def still() -> bool:
    """At rest: over the last 10 control steps the centre moved at most 0.5 mm and the wafer turned at most 0.5
    degrees."""
    poses = _hist["poses"]
    if len(poses) < REST_STEPS:
        return False
    p0, q0 = poses[-1]
    return all(np.linalg.norm(p - p0) <= REST_MOVE + EPS and _turn_deg(q, q0) <= REST_TURN_DEG + EPS for p, q in poses)


def post_gaps(st) -> list[float]:
    """Horizontal distance from the wafer's edge to each stop post (m): a feeler gauge between edge and post."""
    c = np.asarray(st.pos[:2], float)
    out = []
    for p in POSTS:
        lo = np.array([p.pos[0] - p.size[0] / 2, p.pos[1] - p.size[1] / 2])
        hi = np.array([p.pos[0] + p.size[0] / 2, p.pos[1] + p.size[1] / 2])
        out.append(float(np.linalg.norm(c - np.clip(c, lo, hi)) - R))
    return out


def on_ledges(st) -> bool:
    """Lying flat on both ledges, inside the slot, at rest. (The side walls keep the centre within 6 mm of the
    slot centre line, so every flat wafer between them reaches both ledges.)"""
    return bool(
        _tilt_deg(st) <= FLAT_DEG + EPS
        and abs(st.low[2] - LEDGE_TOP) <= ON_TOL + EPS
        and abs(st.pos[1] - YC) <= HALF_GAP - R + EPS
        and FRONT_X + FRONT_MARGIN - EPS <= st.pos[0] <= STOP_X + EPS
        and still()
    )


def seated(st) -> bool:
    return on_ledges(st) and max(post_gaps(st)) <= STOP_GAP + EPS


def success(objects):
    """On both ledges, flat, within 2 mm of both stop posts, at rest, and never dropped."""
    _track(objects)
    return bool(not _hist["dropped"] and seated(objects["wafer"]))


def distance(objects):
    return float(np.linalg.norm(objects["wafer"].pos[:2] - np.array((SEAT_X, YC))))


def score(objects):
    """1.0 seated at the stop; 0.5 flat on both ledges but more than 2 mm from a stop post; 0 otherwise, and 0
    whatever the end state if the wafer moved faster than 0.5 m/s at any time in the trial."""
    _track(objects)
    if _hist["dropped"]:
        return 0.0
    w = objects["wafer"]
    if seated(w):
        return 1.0
    return 0.5 if on_ledges(w) else 0.0


# ---- scripted demo (left arm) -------------------------------------------------------------------------------
# Grip the wafer by its edge across x (front and back), so the fingers clear the cassette's side walls and the
# back finger passes between the stop posts. The finger tips go 2 mm below the wafer's underside.
YAW = math.pi / 2  # fingers close along world x
TIP_PAST_TCP = 0.013
GRIP_BELOW = 0.002
TRAVEL_Z = TOP + 0.11
PICK_Z = TOP + PIN_H - GRIP_BELOW + TIP_PAST_TCP
RELEASE_UP = 0.007  # underside 7 mm above the ledges when let go
PLACE_Z = LEDGE_TOP + RELEASE_UP - GRIP_BELOW + TIP_PAST_TCP
# Aim offset (TCP minus wanted wafer pose) for the left arm's tracking error, from traces: carried, the wafer sits
# about 3.5 mm below the commanded height, so the TCP aims 3 mm high and the wafer is let go about 7 mm up.
AIM = (0.0, 0.0, 0.003)
DEMO_TOUCHES = ()


def demo():
    px, py, _ = WAFER_START
    qx, qy, qz = SEAT_X + AIM[0], YC + AIM[1], PLACE_Z + AIM[2]
    w = [
        Waypoint(1.5, (px, py, TRAVEL_Z), FINGER_OPEN, "over the wafer", YAW),
        Waypoint(1.5, (px, py, PICK_Z), FINGER_OPEN, "descend beside the edge", YAW),
        Waypoint(0.8, (px, py, PICK_Z), FINGER_CLOSED, "grip the edge", YAW),
        Waypoint(1.2, (px, py, TRAVEL_Z), FINGER_CLOSED, "lift off the pins", YAW),
        Waypoint(2.0, (qx - 0.004, qy, TRAVEL_Z), FINGER_CLOSED, "carry over the slot", YAW),
        Waypoint(1.5, (qx - 0.004, qy, qz), FINGER_CLOSED, "lower between the walls", YAW),
        Waypoint(1.0, (qx + 0.001, qy, qz), FINGER_CLOSED, "slide back to the stop", YAW),
        Waypoint(0.6, (qx + 0.001, qy, qz), FINGER_OPEN, "release", YAW),
        Waypoint(1.0, (qx, qy, TRAVEL_Z), FINGER_OPEN, "up", YAW),
    ]
    return {"left": w}
