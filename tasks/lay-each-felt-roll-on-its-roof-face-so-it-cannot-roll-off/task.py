"""On the station table: a model of two pitched roof faces, grey boards on supports, each with a dark-red ridge strip
along its high edge, one on the robot's left and one on its right. Two black rolls of roofing felt lie on the table in
front of the arms, their long axes running left to right, each between two low wooden battens.
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402

import parts  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

GREY, DARK = (0.60, 0.61, 0.63), (0.38, 0.38, 0.40)
RIDGE, BLACK = (0.55, 0.08, 0.08), (0.08, 0.08, 0.09)

SLOPE = math.radians(22.0)  # pitch of both faces
FACE_LEN, FACE_W, FACE_T = 0.20, 0.16, 0.01  # along the slope, across it, thickness
PIVOT_Z = 0.125  # high edge (board mid-plane) above the table; the low edge ends 5 cm up
# Face A (left): high edge along y at x XA_HI, falls toward the robot (-x).
XA_HI, YA = 0.46, 0.15
# Face B (right): high edge along x at y YB_HI, falls toward the robot's right (-y).
XB, YB_HI = 0.32, -0.03

FACE_A = parts.Hinge(
    "face_a",
    shapes=[
        parts.box((FACE_LEN, FACE_W, FACE_T), at=(XA_HI - FACE_LEN / 2, YA, PIVOT_Z), color=GREY),
        parts.box((0.012, FACE_W, 0.006), at=(XA_HI - 0.006, YA, PIVOT_Z + FACE_T / 2 + 0.003), color=RIDGE),
    ],
    pivot=(XA_HI, YA, PIVOT_Z),
    axis=(0.0, -1.0, 0.0),  # positive lowers the near (-x) edge; it rests on its upper stop
    lower=0.0,
    upper=SLOPE,
    mass=0.3,
    damping=0.05,
)
FACE_B = parts.Hinge(
    "face_b",
    shapes=[
        parts.box((FACE_W, FACE_LEN, FACE_T), at=(XB, YB_HI - FACE_LEN / 2, PIVOT_Z), color=GREY),
        parts.box((FACE_W, 0.012, 0.006), at=(XB, YB_HI - 0.006, PIVOT_Z + FACE_T / 2 + 0.003), color=RIDGE),
    ],
    pivot=(XB, YB_HI, PIVOT_Z),
    axis=(1.0, 0.0, 0.0),  # positive lowers the -y edge
    lower=0.0,
    upper=SLOPE,
    mass=0.3,
    damping=0.05,
)
_RUN = FACE_LEN * math.cos(SLOPE)  # horizontal run of a face
ROOF = parts.Part(
    "roof",
    pos=(0.0, 0.0, TOP),
    shapes=[  # supports: a post under each high edge, a low wall under each low edge (clear of the face)
        parts.box((0.02, FACE_W, PIVOT_Z - 0.012), at=(XA_HI - 0.005, YA, (PIVOT_Z - 0.012) / 2), color=DARK),
        parts.box((0.015, FACE_W, 0.035), at=(XA_HI - _RUN + 0.02, YA, 0.0175), color=DARK),
        parts.box((FACE_W, 0.02, PIVOT_Z - 0.012), at=(XB, YB_HI - 0.005, (PIVOT_Z - 0.012) / 2), color=DARK),
        parts.box((FACE_W, 0.015, 0.035), at=(XB, YB_HI - _RUN + 0.02, 0.0175), color=DARK),
    ],
    joints=[FACE_A, FACE_B],
)

R, L = 0.025, 0.14  # felt roll radius and length
SEAM = 0.0368  # square core whose four edges stand 1 mm proud of the round surface: the roll's lap edges


def _roll_shapes():
    """A felt roll: a cylinder along y with four lap edges along its length (a square core whose corners stand 1 mm
    proud of the round surface)."""
    return [parts.cylinder(R, L, at=(0.0, 0.0, 0.0), color=BLACK, axis="y"),
            parts.box((SEAM, L, SEAM), at=(0.0, 0.0, 0.0), color=BLACK)]
ROLL_MASS = 0.25
ROLL_A = parts.Part("roll_left", pos=(0.18, 0.15, TOP + R + 0.0005), mass=ROLL_MASS,
                    shapes=_roll_shapes())
ROLL_B = parts.Part("roll_right", pos=(0.18, -0.15, TOP + R + 0.0005), mass=ROLL_MASS,
                    shapes=_roll_shapes())
WOOD = (0.62, 0.45, 0.28)
BATTEN = (0.01, L, 0.008)  # a low fixed batten each side of each roll keeps it from rolling away on the table
BATTENS = [
    Box(f"batten_{side}_{k}", pos=(roll.pos[0] + dx, roll.pos[1], TOP + BATTEN[2] / 2), size=BATTEN, color=WOOD)
    for side, roll in (("left", ROLL_A), ("right", ROLL_B))
    for k, dx in (("near", -0.025), ("far", 0.025))
]
OBJECTS = [ROOF, ROLL_A, ROLL_B, *BATTENS]
ROLLS = ("roll_left", "roll_right")

SUCCESS_HOLD_S = 3.0  # a roll must stay put for 3 s
EPS = 1e-6
ON_FACE = 0.003  # roll surface within 3 mm of the face
LYING = math.radians(5.0)  # roll axis within 5 deg of the face plane
ALONG = math.radians(20.0)  # roll axis within 20 deg of the face's fall line (no roll is still across a slope unless held)
AT_STOP = math.radians(1.0)  # face within 1 deg of its pitch
TABLE_TOUCH = 0.002  # roll's lowest point within 2 mm of the table

# Face frames (in the hinge link frame): u along the slope from the high edge (0) to the low edge (-FACE_LEN),
# v across, w the outward normal; the face's top is at w = FACE_T / 2.
_FACES = {
    "face_a": dict(u=np.array([1.0, 0.0, 0.0]), v=np.array([0.0, 1.0, 0.0])),
    "face_b": dict(u=np.array([0.0, 1.0, 0.0]), v=np.array([-1.0, 0.0, 0.0])),
}

_history = {"on_face": set(), "spoiled": set(), "last": None}


def _quat_matrix(q):
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _roll_axis(st):
    return _quat_matrix(st.quat_xyzw) @ np.array([0.0, 1.0, 0.0])


def _on(objects, roll: str, face: str, any_direction: bool = False) -> bool:
    """The roll lies on the face: its whole outline within the board's edges, touching it, its axis in the face's
    plane and (unless any_direction) within 20 degrees of the fall line."""
    roof, st = objects["roof"], objects[roll]
    if face not in roof.links:
        return False
    p, q = roof.links[face]
    rot = _quat_matrix(q)  # link frame -> world
    f = _FACES[face]
    u, v = f["u"], f["v"]
    w = np.cross(u, v)
    c = rot.T @ (np.asarray(st.pos) - np.asarray(p))  # roll centre in the link frame (origin at the pivot)
    a = rot.T @ _roll_axis(st)
    if abs(a @ w) > math.sin(LYING) + EPS:
        return False
    height = c @ w - FACE_T / 2 - R
    if abs(height) > ON_FACE + EPS:
        return False
    ain = a - (a @ w) * w
    ain /= np.linalg.norm(ain)
    if not any_direction and abs(ain @ u) < math.cos(ALONG) - EPS:  # must point down the slope
        return False
    n = np.cross(w, ain)
    base = c - (c @ w) * w
    for s1 in (-1, 1):
        for s2 in (-1, 1):
            k = base + s1 * ain * L / 2 + s2 * n * R
            if not (-FACE_LEN - EPS <= k @ u <= EPS and abs(k @ v) <= FACE_W / 2 + EPS):
                return False
    return True


def _at_start(objects) -> bool:
    for name, spec in (("roll_left", ROLL_A), ("roll_right", ROLL_B)):
        st = objects[name]
        if np.linalg.norm(np.asarray(st.pos) - np.asarray(spec.pos)) > 0.0005 or not st.resting():
            return False
    return True


def _update(objects) -> None:
    """Whole-trial record: which rolls have been on a face (outline over the board and touching it, in any direction),
    and which then touched the table. Cleared only by a scene
    reset: both rolls back within 0.5 mm of their start, at rest, after a jump of more than 20 mm in one step (or at
    the first step)."""
    now = {r: np.asarray(objects[r].pos, float) for r in ROLLS}
    last = _history["last"]
    jump = last is None or max(np.linalg.norm(now[r] - last[r]) for r in ROLLS) > 0.02
    if jump and _at_start(objects):
        _history["on_face"].clear()
        _history["spoiled"].clear()
    _history["last"] = now
    for r in ROLLS:
        if any(_on(objects, r, f, any_direction=True) for f in _FACES):
            _history["on_face"].add(r)
        if r in _history["on_face"] and objects[r].low[2] <= TOP + TABLE_TOUCH + EPS:
            _history["spoiled"].add(r)


def _grade(objects) -> float:
    _update(objects)
    roof = objects["roof"]
    for face in _FACES:  # pass/fail: both faces must hold a correctly laid roll
        if roof.joints.get(face, 0.0) < SLOPE - AT_STOP - EPS:
            return 0.0
        if not any(r not in _history["spoiled"] and objects[r].resting() and _on(objects, r, face) for r in ROLLS):
            return 0.0
    return 1.0


def success(objects):
    """Each face holds a roll lying on it, still, that never fell off a face onto the table."""
    return _grade(objects) == 1.0


def score(objects):
    """Pass/fail: 1.0 when both faces hold such a roll, else 0."""
    return _grade(objects)


# -- scripted demo ----------------------------------------------------------------------------------------------

TIP = 0.013
HABIT = False  # True: lay the left roll on face A as it starts, across the slope (a probe of the habit)


def _face_point(face: str, u: float):
    """World point on the face's top surface, u metres down the slope from the high edge, on its centre line."""
    drop, run = u * math.sin(SLOPE), u * math.cos(SLOPE)
    top = PIVOT_Z + FACE_T / 2 / math.cos(SLOPE) - drop
    if face == "face_a":
        return np.array([XA_HI - run, YA, TOP + top])
    return np.array([XB, YB_HI - run, TOP + top])


def _place_tcp(face: str, along_fall_line: bool):
    """TCP over the face's middle holding the roll level, its lowest point 5 mm above the face."""
    mid = _face_point(face, 0.11)
    reach = L / 2 if along_fall_line else R  # how far up the slope the roll's lowest point is
    z = mid[2] + reach * math.tan(SLOPE) + R + 0.005
    if not along_fall_line:
        z = mid[2] + R / math.cos(SLOPE) + 0.005
    return (float(mid[0]), float(mid[1]), float(z + TIP))


def _carry(roll, place, yaw_pick, yaw_place, wait=0.0):
    up = TOP + 0.17
    x, y, _ = roll.pos
    grip_z = TOP + R + TIP
    return [
        Waypoint(1.5 + wait, (x, y, TOP + 0.10), FINGER_OPEN, "over the roll", yaw_pick),
        Waypoint(1.2, (x, y, grip_z), FINGER_OPEN, "down", yaw_pick),
        Waypoint(0.8, (x, y, grip_z), FINGER_CLOSED, "grip", yaw_pick),
        Waypoint(1.2, (x, y, up), FINGER_CLOSED, "lift", yaw_pick),
        Waypoint(2.0, (place[0], place[1], up), FINGER_CLOSED, "carry", yaw_place),
        Waypoint(1.5, place, FINGER_CLOSED, "lower", yaw_place),
        Waypoint(1.5, place, FINGER_CLOSED, "hold still", yaw_place),  # the arm lags; a moving roll keeps rolling
        Waypoint(0.8, place, FINGER_OPEN, "release", yaw_place),
        Waypoint(1.0, (place[0], place[1], up), FINGER_OPEN, "up", yaw_place),
    ]


def demo():
    """Each arm grips its roll across the middle and lays it on its face along the fall line: the left arm turns its
    roll a quarter turn so it points up the left face; the right roll already points down the right face."""
    q = math.pi / 2
    left = _carry(ROLL_A, _place_tcp("face_a", not HABIT), -q, -q if HABIT else 0.0)
    right = _carry(ROLL_B, _place_tcp("face_b", True), q, q, wait=1.0)
    return {"left": left, "right": right}
