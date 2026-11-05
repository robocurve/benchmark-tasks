"""On the station table, in front of the left arm: a loose brown shipping carton (120 mm front to back, 80 mm
across, 60 mm tall, 250 g, standing on one of its two 120 x 80 mm faces) and, nearer the middle of the table, a
fixed grey drop-test pad 200 mm square and 2 mm thick.
"""

import numpy as np

from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN, Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]
G = 9.81

# World frame: x away from the arm bases, y to the robot's left, z up (m).
PAD = Box("drop_pad", pos=(0.300, 0.030, TOP + 0.001), size=(0.200, 0.200, 0.002), color=(0.55, 0.56, 0.58))
PAD_TOP = TOP + 0.002
C_SIZE = (0.120, 0.080, 0.060)
CARTON = Box("carton", pos=(0.170, 0.250, TOP + C_SIZE[2] / 2), size=C_SIZE, mass=0.250, color=(0.66, 0.50, 0.32))
OBJECTS = [PAD, CARTON]

ON_TOL = 0.001  # resting on the pad: lowest point within 1 mm of its top
FLAT_DEG = 5.0  # landed flat: the 60 mm axis within 5 degrees of vertical
DROP_FULL = (0.075, 0.150)  # drop height (m) for full credit
DROP_HALF = (0.050, 0.250)  # for half credit (outside the full band)
LEAVE_MM, LEAVE_DEG = 0.005, 5.0  # after it lands it may not move further than this
EPS = 1e-6

# Free fall. A control step is a free-fall step when the carton is off the table and pad and moving down faster
# than 0.1 m/s, and its downward speed grew by at least 85 % of g over the last step and over the last two steps
# together (0.05 s and 0.1 s at the submitted control_hz=20). A carton still in the gripper cannot speed up like
# that. A fall counts as a drop only if the very next step after a free-fall step finds the carton at the pad
# (lowest point within 10 mm of it) or already rebounding: it arrived straight from the fall, without slowing.
DT = 0.05
FF = 0.85 * G
AIRBORNE = 0.004  # lowest point more than 4 mm above the table: off the table and the pad
ARRIVED = 0.010
HELD_STEPS = 6  # still in the air (under 0.1 m/s vertically) for 6 steps, 0.3 s: held, a new attempt
RESET_MM, RESET_JUMP = 0.0005, 0.020

# Trial history, kept here because the scorer sees one step at a time. drop: the largest drop height that landed
# since the last new attempt; fall: the drop height of the fall in progress; landed: the carton's pose when it
# first came to rest on the pad; disturbed: it moved after that. Cleared by an episode reset (the carton back
# within 0.5 mm of its start and at rest, reached by a jump of more than 20 mm between calls, which only a reset
# produces); drop and landing are also cleared when the carton is held still in the air for 0.3 s.
_H = {"last_pos": None, "vz": [0.0, 0.0], "drop": 0.0, "fall": None, "was_ff": False, "held": 0,
      "landed": None, "disturbed": False}


def _rot(st) -> np.ndarray:
    x, y, z, w = np.asarray(st.quat_xyzw, float) / np.linalg.norm(st.quat_xyzw)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _tilt_deg(st) -> float:
    return float(np.degrees(np.arccos(min(1.0, abs(_rot(st)[2, 2])))))


def _on_pad(st) -> bool:
    """Lying flat on a 120 x 80 face, resting on the pad, its whole footprint on the pad, at rest."""
    if not (abs(st.low[2] - PAD_TOP) <= ON_TOL + EPS and _tilt_deg(st) <= FLAT_DEG + EPS and st.resting()):
        return False
    r = _rot(st)
    down = -np.sign(r[2, 2]) or -1.0  # the local z side that faces down
    px, py, _ = PAD.pos
    for sx in (-1, 1):
        for sy in (-1, 1):  # the four corners of the face resting on the pad
            corner = st.pos + r @ np.array([sx * C_SIZE[0] / 2, sy * C_SIZE[1] / 2, down * C_SIZE[2] / 2])
            if abs(corner[0] - px) > PAD.size[0] / 2 + EPS or abs(corner[1] - py) > PAD.size[1] / 2 + EPS:
                return False
    return True


def _update(objects) -> None:
    st = objects["carton"]
    pos = np.asarray(st.pos, float).copy()
    last = _H["last_pos"]
    if last is not None and np.array_equal(pos, last):
        return  # the same control step seen again
    jumped = last is None or np.linalg.norm(pos - last) > RESET_JUMP
    if jumped and np.linalg.norm(pos - np.asarray(CARTON.pos)) <= RESET_MM and st.resting():
        _H.update(drop=0.0, fall=None, was_ff=False, held=0, landed=None, disturbed=False)
    vz = float(st.lin_vel[2])
    v1, v2 = _H["vz"]  # the previous step's and the one before's vertical speeds
    height = float(st.low[2] - PAD_TOP)
    airborne = st.low[2] > TOP + AIRBORNE
    ff = (last is not None and airborne and vz < -0.1
          and vz - v1 <= -FF * DT + EPS and vz - v2 <= -FF * 2 * DT + EPS)
    if ff:
        h = height + vz * vz / (2 * G)
        _H["fall"] = h if _H["fall"] is None else max(_H["fall"], h)
    elif _H["was_ff"]:
        # The step after a free-fall step: a drop only if it is now at the pad or rebounding.
        if height <= ARRIVED + EPS or vz > 0:
            _H["drop"] = max(_H["drop"], _H["fall"])
        _H["fall"] = None
    elif not airborne:
        _H["fall"] = None
    _H["was_ff"] = ff
    _H["held"] = _H["held"] + 1 if (airborne and abs(vz) < 0.1) else 0
    if _H["held"] >= HELD_STEPS:  # held still in the air: a new attempt starts
        _H.update(drop=0.0, fall=None, landed=None, disturbed=False)
    if jumped and _H["landed"] is not None:
        # Moved more than 20 mm in one step after landing (a fast pick-up): a new attempt, and nothing has landed.
        _H.update(drop=0.0, fall=None, landed=None, disturbed=False)
    if _on_pad(st):
        if _H["landed"] is None:
            _H["landed"] = (pos, np.asarray(st.quat_xyzw, float).copy())
    if _H["landed"] is not None:
        p0, q0 = _H["landed"]
        dq = abs(float(np.dot(q0 / np.linalg.norm(q0), np.asarray(st.quat_xyzw) / np.linalg.norm(st.quat_xyzw))))
        turned = np.degrees(2 * np.arccos(min(1.0, dq)))
        if np.linalg.norm(pos - p0) > LEAVE_MM + EPS or turned > LEAVE_DEG + EPS:
            _H["disturbed"] = True
    _H["last_pos"], _H["vz"] = pos, [vz, v1]


def drop_height() -> float:
    return _H["drop"]


def score(objects):
    """1.0: dropped from 75 to 150 mm (both inclusive) and landed flat, wholly on the pad, left where it landed.
    0.5: the same landing from 50 to 250 mm outside that band. 0.2: the same landing, set down or dropped from under
    50 mm. 0 otherwise."""
    _update(objects)
    st = objects["carton"]
    if _H["disturbed"] or not _on_pad(st):
        return 0.0
    h = _H["drop"]
    if DROP_FULL[0] - EPS <= h <= DROP_FULL[1] + EPS:
        return 1.0
    if DROP_HALF[0] - EPS <= h <= DROP_HALF[1] + EPS:
        return 0.5
    if h < DROP_HALF[0] - EPS:
        return 0.2
    return 0.0


def success(objects):
    return score(objects) >= 1.0 - EPS


def distance(objects):
    c = objects["carton"].pos
    return float(np.hypot(c[0] - PAD.pos[0], c[1] - PAD.pos[1]))


# Demo: the left arm grips the carton across its 80 mm width near the bottom, carries it over the pad, holds it
# still with its underside 100 mm above the pad, and opens.
TIP_PAST_TCP = 0.013
TIP_ABOVE_BOTTOM = 0.010
GRIP_DZ = TIP_PAST_TCP + TIP_ABOVE_BOTTOM
DROP_AT = 0.106  # commanded; held, the carton hangs about 6 mm lower (measured 93.5 mm at 0.100)


def demo():
    cx, cy, _ = CARTON.pos
    px, py, _ = PAD.pos
    grip = TOP + GRIP_DZ
    hold = PAD_TOP + DROP_AT + GRIP_DZ
    travel = hold + 0.01
    w = [
        Waypoint(1.5, (cx, cy, travel), FINGER_OPEN, "over the carton"),
        Waypoint(1.5, (cx, cy, grip), FINGER_OPEN, "descend"),
        Waypoint(0.8, (cx, cy, grip), FINGER_CLOSED, "grip"),
        Waypoint(1.2, (cx, cy, travel), FINGER_CLOSED, "lift"),
        Waypoint(1.8, (px, py, travel), FINGER_CLOSED, "carry"),
        Waypoint(0.8, (px, py, hold), FINGER_CLOSED, "settle at drop height"),
        Waypoint(0.8, (px, py, hold), FINGER_CLOSED, "hold still"),
        Waypoint(0.3, (px, py, hold), FINGER_OPEN, "let go"),
        Waypoint(1.0, (px, py, hold), FINGER_OPEN, "wait"),
        Waypoint(1.0, (px, py, hold + 0.04), FINGER_OPEN, "up"),
    ]
    return {"left": w}
