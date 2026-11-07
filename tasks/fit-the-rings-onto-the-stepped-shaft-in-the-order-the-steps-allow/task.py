"""On the station table, between the arms: a steel base plate with a stepped steel shaft standing on it, a thick
lower section with a shoulder and a thin upper section above it. In front of the left arm lies an orange square
ring; in front of the right arm lies a blue square ring of the same outside size. Both rings are loose.
"""

import json
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import parts  # noqa: E402
from yam_arena.demo import FINGER_OPEN, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box, Cylinder  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
EPS = 1e-6

# World frame: x away from the arm bases, y to the robot's left, z up (m).
POST_XY = (0.30, 0.0)
PLATE = (0.12, 0.12, 0.010)
PLATE_TOP = TOP + PLATE[2]
LOWER_R, LOWER_H = 0.015, 0.035  # lower section: 30 mm across, 35 mm tall
UPPER_R = 0.009  # upper section: 18 mm across
SHOULDER = PLATE_TOP + LOWER_H  # top of the lower section
POST_TOP = PLATE_TOP + 0.075

STEEL, DARK_STEEL = (0.62, 0.64, 0.67), (0.42, 0.44, 0.47)
BLUE, ORANGE = (0.15, 0.32, 0.85), (0.95, 0.50, 0.10)

FIXTURE = [
    Box("base_plate", pos=(*POST_XY, TOP + PLATE[2] / 2), size=PLATE, color=DARK_STEEL),
    Cylinder("shaft_lower", pos=(*POST_XY, PLATE_TOP + LOWER_H / 2), radius=LOWER_R, height=LOWER_H, color=STEEL),
    Cylinder("shaft_upper", pos=(*POST_XY, (SHOULDER + POST_TOP) / 2), radius=UPPER_R, height=POST_TOP - SHOULDER,
             color=STEEL),
]

RING_H = 0.025


def _ring_shapes(outside, bore, color):
    """A square ring from four boxes, centred on its own origin (the middle of its height)."""
    w = (outside - bore) / 2
    off = (bore + w) / 2
    return [parts.box((outside, w, RING_H), at=(0.0, off, 0.0), color=color),
            parts.box((outside, w, RING_H), at=(0.0, -off, 0.0), color=color),
            parts.box((w, bore, RING_H), at=(off, 0.0, 0.0), color=color),
            parts.box((w, bore, RING_H), at=(-off, 0.0, 0.0), color=color)]


# Both rings are 60 mm square outside, so size, side and arm give nothing away.
# Blue (in front of the right arm): 38 mm bore: passes both sections, so its seat is the plate.
# Orange (in front of the left arm): 26 mm bore: passes only the upper section, so its seat is the shoulder.
BLUE_OUT, BLUE_BORE = 0.060, 0.038
ORANGE_OUT, ORANGE_BORE = 0.060, 0.026
BLUE_START = (0.19, -0.17, TOP + RING_H / 2)
ORANGE_START = (0.19, 0.17, TOP + RING_H / 2)
BLUE_RING = parts.Part("blue_ring", pos=BLUE_START, mass=0.07, friction=(0.6, 0.5),
                       shapes=_ring_shapes(BLUE_OUT, BLUE_BORE, BLUE))
ORANGE_RING = parts.Part("orange_ring", pos=ORANGE_START, mass=0.10, friction=(0.6, 0.5),
                         shapes=_ring_shapes(ORANGE_OUT, ORANGE_BORE, ORANGE))

OBJECTS = [*FIXTURE, BLUE_RING, ORANGE_RING]

# The offline checker models the fingers fully closed, so it sees the finger tips inside the shaft while a ring is
# lowered over it. With the fingers at the ring's width (60 mm) or open, they stay at least 8 mm clear of the
# shaft's bounding box along the whole demo (checked with the real finger openings).
DEMO_TOUCHES = ("shaft_lower", "shaft_upper")

# ---- scoring ---------------------------------------------------------------------------------------------------
BLUE_TOL = 0.001  # blue underside within 1 mm of the plate top
ORANGE_TOL = 0.0015  # orange underside within 1.5 mm of the shoulder top
CENTRE_TOL = 0.0065  # ring centre within 6.5 mm of the shaft axis (around the shaft: the bore allows 5.7 mm at most)
TILT_MAX = 3.0  # degrees


def tilt_deg(state) -> float:
    """Angle between the ring's own vertical axis and world vertical."""
    r = parts._quat_to_matrix(state.quat_xyzw)
    return float(np.degrees(np.arccos(np.clip(r[2, 2], -1.0, 1.0))))


def _seated(state, seat_z, tol) -> bool:
    p = np.asarray(state.pos, float)
    return bool(
        abs(float(state.low[2]) - seat_z) <= tol + EPS
        and np.hypot(p[0] - POST_XY[0], p[1] - POST_XY[1]) <= CENTRE_TOL + EPS
        and tilt_deg(state) <= TILT_MAX + EPS
        and state.resting()
    )


def blue_seated(objects) -> bool:
    """Blue around the shaft, its underside within 1 mm of the plate, flat within 3 degrees, at rest."""
    return _seated(objects["blue_ring"], PLATE_TOP, BLUE_TOL)


def orange_seated(objects) -> bool:
    """Orange around the shaft, its underside within 1.5 mm of the shoulder, flat within 3 degrees, at rest."""
    return _seated(objects["orange_ring"], SHOULDER, ORANGE_TOL)


def success(objects):
    return bool(blue_seated(objects) and orange_seated(objects))


def distance(objects):
    b, o = objects["blue_ring"], objects["orange_ring"]
    gb = np.array((*POST_XY, PLATE_TOP + RING_H / 2))
    go = np.array((*POST_XY, SHOULDER + RING_H / 2))
    return float(np.linalg.norm(np.asarray(b.pos) - gb) + np.linalg.norm(np.asarray(o.pos) - go))


def score(objects):
    """Pass/fail: 1.0 when both rings are seated, else 0 (including orange on the shoulder with blue stuck on top)."""
    return 1.0 if success(objects) else 0.0


# ---- demo: fixed waypoints, straight-down grips across each ring's outside ---------------------------------------
TIP = 0.013  # finger tips below the TCP
GRIP_UP = 0.005  # finger tips this far above the ring's underside
TRAVEL_Z = TOP + 0.135  # carried ring's underside about 30 mm above the shaft top
DROP = 0.008  # release with the underside this far above its seat (the arm sags about 4 mm carrying a part)
LEFT_PARK = (0.18, 0.30, TRAVEL_Z)
RIGHT_PARK = (0.18, -0.30, TRAVEL_Z)


def _grip_z(underside):
    return underside + GRIP_UP + TIP


SQUEEZE = 0.003  # each finger's target this far inside the ring's outside face


def _fit(arm, start, seat_z, width):
    """Pick a ring up across its outside and lower it over the shaft. The fingers are told to close only 3 mm past
    the ring's faces: told to close fully, one finger overpowers the other and pushes the ring off centre in the
    grip, as far as the other finger's open stop allows (seen in the first GPU trace)."""
    sx, sy, _ = start
    hold = -(width / 2 - SQUEEZE)
    gz = _grip_z(TOP)
    px, py = POST_XY
    rz = _grip_z(seat_z + DROP)
    return [
        Waypoint(1.5, (sx, sy, TRAVEL_Z), FINGER_OPEN, "over the ring"),
        Waypoint(1.5, (sx, sy, gz), FINGER_OPEN, "down"),
        Waypoint(0.8, (sx, sy, gz), hold, "grip across the outside"),
        Waypoint(1.5, (sx, sy, TRAVEL_Z), hold, "lift"),
        Waypoint(2.0, (px, py, TRAVEL_Z), hold, "over the shaft"),
        Waypoint(1.0, (px, py, TRAVEL_Z), hold, "settle"),
        Waypoint(2.0, (px, py, rz), hold, "lower over the shaft"),
        Waypoint(0.8, (px, py, rz), FINGER_OPEN, "let go"),
        Waypoint(1.5, (px, py, TRAVEL_Z), FINGER_OPEN, "up clear of the shaft"),
    ]


def _then(first, second):
    """Two arms one after the other: the second waits at its park until the first has parked."""
    wait = sum(w.duration for w in first)
    return [Waypoint(wait, second[0].pos, FINGER_OPEN, "wait for the other arm"), *second[1:]]


def demo():
    """The right arm fits blue (the larger bore) first; it drops past the shoulder to the plate. Then the left arm
    fits orange, which stops on the shoulder. RING_DEMO=wrong replays the wrong order (left arm and orange first) for
    a GPU trace; it should score 0."""
    right = [Waypoint(1.0, RIGHT_PARK, FINGER_OPEN, "start")] + _fit("right", BLUE_START, PLATE_TOP, BLUE_OUT) + [
        Waypoint(1.5, RIGHT_PARK, FINGER_OPEN, "park")]
    left = [Waypoint(1.0, LEFT_PARK, FINGER_OPEN, "start")] + _fit("left", ORANGE_START, SHOULDER, ORANGE_OUT) + [
        Waypoint(1.5, LEFT_PARK, FINGER_OPEN, "park")]
    if os.environ.get("RING_DEMO") == "wrong":
        return {"left": left, "right": _then(left, right) + [Waypoint(3.0, RIGHT_PARK, FINGER_OPEN, "wait")]}
    return {"right": right, "left": _then(right, left) + [Waypoint(3.0, LEFT_PARK, FINGER_OPEN, "wait")]}


# ---- debugging only: with RING_DEBUG=<file>, log the finger-tip positions every 5 steps ------------------------
if os.environ.get("RING_DEBUG"):
    _DBG = {"k": 0}

    @parts.on_step
    def _log_tips(sim):
        _DBG["k"] += 1
        if _DBG["k"] % 5:
            return
        base = sim.env.unwrapped
        origin = parts._np(base.scene.env_origins[0])
        row = {"k": _DBG["k"]}
        for arm in ("left_robot", "right_robot"):
            asset = base.scene[arm]
            poses = parts._np(asset.data.body_link_pose_w)[0]
            for i, n in enumerate(asset.body_names):
                if n in ("tip_left", "tip_right", "gripper"):
                    row[f"{arm[0]}_{n}"] = [round(float(v), 4) for v in poses[i, :3] - origin]
        with open(os.environ["RING_DEBUG"], "a") as f:
            f.write(json.dumps(row) + "\n")
