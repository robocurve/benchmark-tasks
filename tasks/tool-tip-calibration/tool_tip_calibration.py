"""On the station table, in front of the left arm: a steel pin (a 10 mm rod, 10 cm long, 62 g, blue) stands upright in
a fixed grey holder, a block with a 20 mm square hole 3 cm deep. To its right, 21 cm away, is a fixed reference post:
a dark grey block 6 x 6 cm and 5 cm tall, with an orange frame on top that leaves an 18 mm square pocket, 7 mm deep,
in its centre. Nothing else is near the post.

Calibrating a tool tip means touching one fixed point with the tip from several tool angles. Here the fixed point is
the pocket: the pin's lower end goes into it, and the pin is tilted to different angles while its lower end stays in.
"""

import math

import numpy as np

import yam_arena.demo as _demo
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# World frame: x forward from the arms, y to the left, z up (m); the left arm's base stands at x 0, y 0.305.

PIN_R, PIN_H, PIN_KG = 0.005, 0.10, 0.062  # steel, 7,850 kg/m^3
HOLDER_XY = (0.20, 0.42)
HOLDER_HOLE, HOLDER_WALL, HOLDER_H = 0.020, 0.012, 0.03

POST_XY = (0.27, 0.22)
POST_SIZE, POST_H = 0.06, 0.05
POCKET, POCKET_DEPTH = 0.018, 0.007  # the square opening in the orange frame, and the frame's height
FLOOR_Z = TOP + POST_H  # the pocket's floor: the post's top face

BLUE, ORANGE, GREY, DARK = (0.10, 0.35, 0.90), (0.95, 0.45, 0.05), (0.55, 0.55, 0.57), (0.22, 0.22, 0.24)


def _frame(name: str, cx: float, cy: float, hole: float, wall: float, z0: float, h: float, color) -> list[Box]:
    """Four fixed boxes around a square hole ``hole`` wide, from height z0 up h."""
    outer = hole + 2 * wall
    zc = z0 + h / 2
    return [
        Box(f"{name}_front", pos=(cx + (hole + wall) / 2, cy, zc), size=(wall, outer, h), color=color),
        Box(f"{name}_back", pos=(cx - (hole + wall) / 2, cy, zc), size=(wall, outer, h), color=color),
        Box(f"{name}_left", pos=(cx, cy + (hole + wall) / 2, zc), size=(hole, wall, h), color=color),
        Box(f"{name}_right", pos=(cx, cy - (hole + wall) / 2, zc), size=(hole, wall, h), color=color),
    ]


PIN = Cylinder("pin", pos=(*HOLDER_XY, TOP + PIN_H / 2 + 0.001), radius=PIN_R, height=PIN_H, mass=PIN_KG, color=BLUE)
POST = Box("post", pos=(*POST_XY, TOP + POST_H / 2), size=(POST_SIZE, POST_SIZE, POST_H), color=DARK)
FRAME = _frame("frame", *POST_XY, POCKET, (POST_SIZE - POCKET) / 2, FLOOR_Z, POCKET_DEPTH, ORANGE)
HOLDER = _frame("holder", *HOLDER_XY, HOLDER_HOLE, HOLDER_WALL, TOP, HOLDER_H, GREY)
OBJECTS = [PIN, POST, *FRAME, *HOLDER]

# A pose counts when, for 20 control steps (1 s at 20 Hz) in a row: the centre of the pin's lower end is inside the
# pocket (within 9 mm of its centre along x and y, and between 1 mm below the floor and the frame's top); the pin
# leans between 10 and 35 degrees from vertical; and the pin holds still: through the whole second its lower end stays within
# 2 mm, and its axis within 2 degrees, of where they were when the second began. A pose is credited only if its axis
# is at least 15 degrees from every pose credited before. Stillness is judged by drift over the second, not by speed:
# a gripped pin jitters at a few cm/s in the sim while its position holds to under 1 mm (trace, 2026-11-05).
HOLD_STEPS = 20
MIN_TILT_DEG, MAX_TILT_DEG, MIN_SPREAD_DEG = 10.0, 35.0, 15.0
# The 10 degree floor: a free pin (10 mm, flat end) stands by itself up to about 6 degrees of lean, so an upright pin
# let go in the pocket would otherwise earn a pose with no grip.
DRIFT_M, DRIFT_DEG = 0.002, 2.0
POSES_NEEDED = 4
# One fixed point: each credited pose's tip must be within 4 mm of the first credited pose's tip. The pocket is wider
# (18 mm) only so a leaning pin fits; in the demo the four tips landed within 2.4 mm of each other (trace, 2026-11-05).
SAME_POINT_M = 0.004
SCORE_OVER = "best"  # the score only rises; returning the pin to its start must not wipe earned poses


def _axis_up(pin) -> np.ndarray:
    """The pin's axis as a unit vector, pointing from its lower end to its upper end."""
    x, y, z, w = (float(v) for v in pin.quat_xyzw)
    axis = np.array([2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)])
    return axis if axis[2] >= 0 else -axis


def _tip(pin) -> np.ndarray:
    """The centre of the pin's lower end face."""
    return np.asarray(pin.pos, dtype=float) - _axis_up(pin) * PIN_H / 2


def _tilt_deg(pin) -> float:
    return math.degrees(math.acos(max(-1.0, min(1.0, float(_axis_up(pin)[2])))))


def _seated(pin) -> bool:
    """The lower end's centre is inside the pocket, and the pin leans MIN_TILT_DEG to MAX_TILT_DEG."""
    tip = _tip(pin)
    half = POCKET / 2
    return (abs(tip[0] - POST_XY[0]) <= half and abs(tip[1] - POST_XY[1]) <= half
            and FLOOR_Z - 0.001 <= tip[2] <= FLOOR_Z + POCKET_DEPTH and MIN_TILT_DEG <= _tilt_deg(pin) <= MAX_TILT_DEG)


def _angle_deg(a: np.ndarray, b: np.ndarray) -> float:
    return math.degrees(math.acos(max(-1.0, min(1.0, float(a @ b)))))


def _at_start(pin) -> bool:
    """The pin stands where a reset puts it (the runs use -E object_jitter=0)."""
    pos = np.asarray(pin.pos, dtype=float)
    return (math.hypot(pos[0] - PIN.pos[0], pos[1] - PIN.pos[1]) <= 0.001 and abs(pos[2] - PIN.pos[2]) <= 0.002
            and _tilt_deg(pin) < 0.5)


# The trial's state: where the current hold began (the lower end and the axis), how many steps it has lasted, and the
# axes credited so far. success() and score()
# both get the same states object on a step (success() first), so the state advances once per new object; it clears
# when the pin is back at its start, which is how the next trial begins.
_last = None
_anchor: tuple[np.ndarray, np.ndarray] | None = None
_held = 0
_credited: list[np.ndarray] = []
_tips: list[np.ndarray] = []  # the tip of each credited pose


def _update(objects) -> int:
    global _last, _anchor, _held
    if objects is _last:
        return len(_credited)
    _last = objects
    pin = objects["pin"]
    if _at_start(pin):
        _anchor, _held = None, 0
        _credited.clear()
        _tips.clear()
        return 0
    if not _seated(pin):
        _anchor, _held = None, 0
        return len(_credited)
    tip, axis = _tip(pin), _axis_up(pin)
    if (_anchor is None or float(np.linalg.norm(tip - _anchor[0])) > DRIFT_M
            or _angle_deg(axis, _anchor[1]) > DRIFT_DEG):
        _anchor, _held = (tip, axis), 1  # a new hold starts here
    else:
        _held += 1
    if _held >= HOLD_STEPS and len(_credited) < POSES_NEEDED:
        same_point = not _tips or float(np.linalg.norm(_anchor[0] - _tips[0])) <= SAME_POINT_M
        if same_point and all(_angle_deg(_anchor[1], c) >= MIN_SPREAD_DEG for c in _credited):
            _credited.append(_anchor[1])
            _tips.append(_anchor[0])
    return len(_credited)


def success(objects):
    """Four poses credited; must hold for 1 s."""
    return _update(objects) >= POSES_NEEDED


def score(objects):
    """0.25 per credited pose: 0, 0.25, 0.5, 0.75 or 1.0."""
    return _update(objects) / POSES_NEEDED


def distance(objects):
    """From the centre of the pin's lower end to the centre of the pocket's floor (m)."""
    return float(np.linalg.norm(_tip(objects["pin"]) - np.array([*POST_XY, FLOOR_Z])))


# --- The demo -------------------------------------------------------------------------------------------------------
# The stock demo grips straight down, turned only about the vertical. Calibration needs a tilted grip, so a waypoint
# here may carry a full TCP rotation inside its yaw: a float subclass with a ``rot`` matrix. The patched rotation
# function returns that matrix and otherwise behaves exactly as the stock one, so other tasks are unaffected.


class TiltedYaw(float):
    """A yaw value (0.0) that also carries a TCP rotation matrix for the demo."""

    def __new__(cls, rot):
        obj = super().__new__(cls, 0.0)
        obj.rot = np.asarray(rot, dtype=float)
        return obj


if not getattr(_demo.grip_rotation, "accepts_tilt", False):
    _stock_grip_rotation = _demo.grip_rotation

    def _grip_rotation(yaw: float = 0.0) -> np.ndarray:
        rot = getattr(yaw, "rot", None)
        return rot if rot is not None else _stock_grip_rotation(yaw)

    _grip_rotation.accepts_tilt = True
    _demo.grip_rotation = _grip_rotation

DOWN = _demo.grip_rotation(0.0)
GRIP_BELOW_TOP = 0.012  # the TCP sits 12 mm below the pin's top; the pad tips reach 25 mm down
TIP_BELOW_TCP = PIN_H - GRIP_BELOW_TOP
TIP_IN_TCP = DOWN.T @ np.array([0.0, 0.0, -TIP_BELOW_TCP])  # the pin's lower end in the TCP frame
PIVOT = np.array([*POST_XY, FLOOR_Z + 0.0035])  # the lower end's centre, 3.5 mm up: a tilted rim clears the floor

DEMO_TILT_DEG = 25.0  # 30 was commanded first; the pin turned 4 degrees further in the grip (34), too near 35
DEMO_AZIMUTHS_DEG = (90.0, 150.0, 210.0, 270.0)  # the pin's top leans left, then round toward the arm, to the right; 24 deg apart


def _axis_angle(axis, angle: float) -> np.ndarray:
    k = np.asarray(axis, dtype=float) / np.linalg.norm(axis)
    kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(angle) * kx + (1 - math.cos(angle)) * kx @ kx


def tilted(azimuth_deg: float, tilt_deg: float) -> np.ndarray:
    """The TCP rotation that leans the pin's top ``tilt_deg`` toward ``azimuth_deg`` (0 = +x, 90 = +y)."""
    az = math.radians(azimuth_deg)
    return _axis_angle((-math.sin(az), math.cos(az), 0.0), math.radians(tilt_deg)) @ DOWN


def _slerp(a: np.ndarray, b: np.ndarray, s: float) -> np.ndarray:
    rel = a.T @ b
    angle = math.acos(max(-1.0, min(1.0, (np.trace(rel) - 1) / 2)))
    if angle < 1e-9:
        return b
    axis = np.array([rel[2, 1] - rel[1, 2], rel[0, 2] - rel[2, 0], rel[1, 0] - rel[0, 1]]) / (2 * math.sin(angle))
    return a @ _axis_angle(axis, s * angle)


def _about_tip(rot: np.ndarray, duration: float, label: str, tip=PIVOT) -> Waypoint:
    """A waypoint that puts the pin's lower end at ``tip`` with the TCP turned to ``rot``."""
    return Waypoint(duration, tuple(np.asarray(tip) - rot @ TIP_IN_TCP), FINGER_CLOSED, label, TiltedYaw(rot))


def _pivot_to(start: np.ndarray, end: np.ndarray, duration: float, label: str, steps: int = 6) -> list[Waypoint]:
    """Turn from ``start`` to ``end`` about the lower end, in short steps so it stays in the pocket."""
    return [_about_tip(_slerp(start, end, i / steps), duration / steps, label) for i in range(1, steps + 1)]


def demo():
    """The left arm takes the pin from its holder, lowers its end into the pocket, then leans it 25 degrees
    in four directions (left, two toward the arm, right), holding each for 2.5 s. The right arm doesn't move."""
    hx, hy = HOLDER_XY
    grip_z = TOP + PIN_H - GRIP_BELOW_TOP
    lift_z = grip_z + 0.07  # the pin's lower end 7 cm up, clear of the holder's walls
    above = PIVOT + np.array([0.0, 0.0, 0.03])
    plan = [
        Waypoint(1.8, (hx, hy, grip_z + 0.08), FINGER_OPEN, "over the pin"),
        Waypoint(1.5, (hx, hy, grip_z), FINGER_OPEN, "down"),
        Waypoint(0.8, (hx, hy, grip_z), FINGER_CLOSED, "grip"),
        Waypoint(1.5, (hx, hy, lift_z), FINGER_CLOSED, "lift out"),
        _about_tip(DOWN, 2.0, "over the pocket", tip=above),
        _about_tip(DOWN, 1.5, "lower into the pocket"),
    ]
    rot = DOWN
    for az in DEMO_AZIMUTHS_DEG:
        nxt = tilted(az, DEMO_TILT_DEG)
        plan += _pivot_to(rot, nxt, 1.5, f"lean toward {az:.0f} deg")
        plan.append(_about_tip(nxt, 2.5, f"hold at {az:.0f} deg"))
        rot = nxt
    return {"left": plan}
