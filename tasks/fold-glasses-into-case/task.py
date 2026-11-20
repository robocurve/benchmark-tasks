"""Put your glasses away. On the station table, in the middle, a pair of adult eyeglasses (black acetate, frame size
52-18-140: 139 mm across, temples 140 mm, 25 g) stands open the way people put them down: on the bottom of its rims
and the tips of its two earpieces, lenses toward the robot, temples pointing away. On the right lies an open hard
clamshell glasses case (160 x 60 x 38 mm, 90 g) with a spring hinge along its far side; its lid stands open at
105 degrees.

The task: fold both temples, put the glasses in the case and shut the lid. A temple folds about a small hinge with
real friction, so it has to be pushed round while the frame is held still (pushed alone, the 25 g frame just slides).
The case is snug: its lid shuts flush only over glasses lying flat inside with both temples folded to within about
10 degrees; a temple left open, or glasses half outside, hold the lid open.

Requires -E object_jitter=0 (one fixed layout).

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \\
           --instruction "Fold both arms of the glasses, put them in their case and snap the case shut." \\
           --scorer task_score ...
Demo:    python demo.py task.py demo_out
"""

from __future__ import annotations

import dataclasses
import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402

import mechkit  # noqa: E402
from mechkit import Joint, Mech, box, on_reset, on_step  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]

INSTRUCTION = "Fold both arms of the glasses, put them in their case and snap the case shut."

ACETATE = (0.07, 0.07, 0.08)
LENS = (0.62, 0.72, 0.80)
PAD = (0.85, 0.85, 0.80)
SHELL = (0.55, 0.12, 0.16)     # the case's outside (burgundy leatherette)
FELT = (0.25, 0.25, 0.28)      # its lining (grey felt)
STEEL = (0.70, 0.70, 0.72)


def _ry(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _rz(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _wxyz(r: np.ndarray) -> tuple[float, float, float, float]:
    """(w, x, y, z) of a rotation matrix."""
    r = np.asarray(r, float)
    w = math.sqrt(max(0.0, 1.0 + r[0, 0] + r[1, 1] + r[2, 2])) / 2
    if w > 1e-6:
        return (w, (r[2, 1] - r[1, 2]) / (4 * w), (r[0, 2] - r[2, 0]) / (4 * w), (r[1, 0] - r[0, 1]) / (4 * w))
    x = math.sqrt(max(0.0, 1.0 + r[0, 0] - r[1, 1] - r[2, 2])) / 2
    return (0.0, x, (r[0, 1] + r[1, 0]) / (4 * x), (r[0, 2] + r[2, 0]) / (4 * x))


# ---------------------------------------------------------------------------------------------------------------------
# The glasses: frame size 52-18-140 (52 mm lenses, an 18 mm bridge, 140 mm temples), black acetate, 25 g.
# Glasses frame: origin on the back face of the front, at its middle and its bottom edge; x from the lenses toward the
# temples, y to the robot's left, z up. The front is 5 mm thick (x -5 to 0), 139 mm across, 42 mm tall.
FRONT_T = 0.005
LENS_W, LENS_H, LENS_T = 0.052, 0.035, 0.002
BRIDGE = 0.018
RIM = 0.0035
LENS_Y = BRIDGE / 2 + LENS_W / 2                  # 0.035: each lens's centre
FRONT_H = LENS_H + 2 * RIM                        # 0.042
FRONT_HALF = LENS_Y + LENS_W / 2 + RIM + 0.005    # 0.0695: 139 mm across
HINGE_X, HINGE_Y, HINGE_Z = 0.0035, 0.067, 0.026  # hinge axes (vertical): 3.5 mm behind the front, 16 mm under its top
TEMPLE_L, TEMPLE_T, TEMPLE_H = 0.100, 0.004, 0.007  # straight part, hinge to bend
EAR_L, EAR_T, EAR_H = 0.040, 0.0035, 0.005          # earpiece (100 + 40 = 140 mm along the temple)
EAR_DROP = math.radians(38.5)                        # earpiece bent down 38.5 degrees: its tip reaches the table
FOLD = math.radians(90.0)                            # fully folded: the temple lies along the back of the front
HINGE_HOLD = 0.010                                   # N m, hinge friction (estimate, to be measured)
TEMPLE_MASS = 0.0055
FRONT_MASS = 0.014

GLASS_BOXES: list = []   # (body, centre, size, rotation) of every glasses collider, for the scorer


def _gbox(body, centre, size, color, rot=None):
    rot = np.eye(3) if rot is None else np.asarray(rot, float)
    GLASS_BOXES.append((body, np.asarray(centre, float), np.asarray(size, float), rot))
    return box(tuple(float(v) for v in centre), tuple(float(v) for v in size), color, tilt=_wxyz(rot))


def _look(shape):
    return dataclasses.replace(shape, collide=False)


def _front_shapes() -> list:
    xc = -FRONT_T / 2
    zc = RIM + LENS_H / 2
    out = []
    for s in (-1, 1):
        y = s * LENS_Y
        out += [_gbox("base", (xc, y, FRONT_H - RIM / 2), (FRONT_T, LENS_W + 2 * RIM, RIM), ACETATE),   # top rim
                _gbox("base", (xc, y, RIM / 2), (FRONT_T, LENS_W + 2 * RIM, RIM), ACETATE),             # bottom rim
                _gbox("base", (xc, s * (LENS_Y + LENS_W / 2 + RIM / 2), zc), (FRONT_T, RIM, LENS_H), ACETATE),
                _gbox("base", (xc, s * (LENS_Y - LENS_W / 2 - RIM / 2), zc), (FRONT_T, RIM, LENS_H), ACETATE),
                _gbox("base", (xc, y, zc), (LENS_T, LENS_W, LENS_H), LENS)]                             # lens
        # endpiece: from the rim's outer edge back to the hinge
        y0 = LENS_Y + LENS_W / 2 + RIM
        out.append(_gbox("base", ((-FRONT_T + HINGE_X + 0.002) / 2, s * (y0 + FRONT_HALF) / 2, HINGE_Z),
                         (FRONT_T + HINGE_X + 0.002, FRONT_HALF - y0, 0.009), ACETATE))
        out.append(_look(box((0.006, s * 0.009, 0.020), (0.002, 0.006, 0.010), PAD)))                  # nose pad
    out.append(_gbox("base", (xc, 0.0, 0.034), (FRONT_T, BRIDGE - 2 * RIM, 0.006), ACETATE))          # bridge
    return out


def _temple_shapes(body: str) -> list:
    """A temple in its hinge frame, open (pointing +x)."""
    re = _ry(EAR_DROP)
    e = re @ np.array([1.0, 0.0, 0.0])
    bend = np.array([TEMPLE_L, 0.0, 0.0])
    return [_gbox(body, (TEMPLE_L / 2, 0.0, 0.0), (TEMPLE_L, TEMPLE_T, TEMPLE_H), ACETATE),
            _gbox(body, bend + e * EAR_L / 2 + re @ np.array([0.0, 0.0, (TEMPLE_H - EAR_H) / 2]),
                  (EAR_L, EAR_T, EAR_H), ACETATE, re)]


# The left temple (+y) folds clockwise seen from above (negative turn), the right one (-y) counterclockwise.
LEFT_T = Joint("left_temple", "hinge", axis="z", at=(HINGE_X, HINGE_Y, HINGE_Z), low=-FOLD, high=0.0,
               mass=TEMPLE_MASS, hold=HINGE_HOLD, damping=0.0005, friction=0.5, shapes=_temple_shapes("left_temple"))
RIGHT_T = Joint("right_temple", "hinge", axis="z", at=(HINGE_X, -HINGE_Y, HINGE_Z), low=0.0, high=FOLD,
                mass=TEMPLE_MASS, hold=HINGE_HOLD, damping=0.0005, friction=0.5,
                shapes=_temple_shapes("right_temple"))
_JOINT_AT = {"base": np.zeros(3), "left_temple": np.array(LEFT_T.at), "right_temple": np.array(RIGHT_T.at)}


def _low_local() -> float:
    return min(float((_JOINT_AT[b] + c + r @ (np.array([sx, sy, sz]) * size))[2])
               for b, c, size, r in GLASS_BOXES for sx in (-0.5, 0.5) for sy in (-0.5, 0.5) for sz in (-0.5, 0.5))


GLASSES_X, GLASSES_Y = 0.240, 0.020    # the back face of the front, at its middle
_FRONT = _front_shapes()
GLASSES = Mech("glasses", pos=(GLASSES_X, GLASSES_Y, TOP - _low_local() + 0.0005), mass=FRONT_MASS, friction=0.5,
               shapes=_FRONT, joints=[LEFT_T, RIGHT_T])

# ---------------------------------------------------------------------------------------------------------------------
# The case: a hard clamshell glasses case, 160 x 60 x 38 mm closed (base 20 mm, lid 18 mm), 2 mm shell, felt lined,
# 90 g (base 60 g, lid 30 g), with a spring hinge along its far long side. Case frame: origin at the middle of its
# bottom, x toward the hinge (away from the robot), y along its length, z up.
CASE_L, CASE_W = 0.160, 0.060
BASE_H, LID_H, WALL = 0.020, 0.018, 0.002
IN_X, IN_Y = CASE_W / 2 - WALL, CASE_L / 2 - WALL          # interior half sizes: 28 x 78 mm
IN_Z = (WALL, BASE_H + LID_H - WALL)                        # interior floor to lid underside: 2 to 36 mm
CASE_HINGE = (CASE_W / 2, 0.0, BASE_H)                      # the hinge line: the base's far top edge
LID_DEPTH = CASE_W                                          # hinge to the lid's front edge
OPEN = math.radians(105.0)                                  # the lid starts open, its spring holding it there
FLIP = math.radians(10.0)                                   # past this far closed from open, the spring snaps it shut
LID_SPRING = 0.05                                           # N m per rad (estimate)
PRELOAD = math.radians(20.0)                                # shut, the spring still presses it down by this much
LID_DAMPING = 0.01                                          # N m s per rad: the snap takes about 0.3 s
CASE_X, CASE_Y = 0.300, -0.170
CASE_MASS, LID_MASS = 0.060, 0.030


def _base_shapes() -> list:
    h = BASE_H - WALL
    return [box((0.0, 0.0, WALL / 2), (CASE_W, CASE_L, WALL), SHELL),
            _look(box((0.0, 0.0, WALL + 0.0002), (2 * IN_X, 2 * IN_Y, 0.0004), FELT)),
            box((-(CASE_W - WALL) / 2, 0.0, WALL + h / 2), (WALL, CASE_L, h), SHELL),
            box(((CASE_W - WALL) / 2, 0.0, WALL + h / 2), (WALL, CASE_L, h), SHELL),
            box((0.0, -(CASE_L - WALL) / 2, WALL + h / 2), (CASE_W - 2 * WALL, WALL, h), SHELL),
            box((0.0, (CASE_L - WALL) / 2, WALL + h / 2), (CASE_W - 2 * WALL, WALL, h), SHELL),
            _look(box((CASE_W / 2 + 0.002, 0.0, BASE_H - 0.002), (0.004, 0.050, 0.004), STEEL))]   # hinge barrel


def _lid_closed() -> list:
    """The lid's boxes (centre, size, colour) in the case frame, shut."""
    h = LID_H - WALL
    top = BASE_H + LID_H
    return [((0.0, 0.0, top - WALL / 2), (CASE_W, CASE_L, WALL), SHELL),
            ((-(CASE_W - WALL) / 2, 0.0, BASE_H + h / 2), (WALL, CASE_L, h), SHELL),
            (((CASE_W - WALL) / 2, 0.0, BASE_H + h / 2), (WALL, CASE_L, h), SHELL),
            ((0.0, -(CASE_L - WALL) / 2, BASE_H + h / 2), (CASE_W - 2 * WALL, WALL, h), SHELL),
            ((0.0, (CASE_L - WALL) / 2, BASE_H + h / 2), (CASE_W - 2 * WALL, WALL, h), SHELL)]


def _lid_shapes() -> list:
    """The lid in its hinge frame, open at OPEN (its rest)."""
    r = _ry(OPEN)
    hinge = np.array(CASE_HINGE)
    out = [box(tuple(r @ (np.array(c) - hinge)), s, col, tilt=_wxyz(r)) for c, s, col in _lid_closed()]
    out.append(_look(box(tuple(r @ (np.array((0.0, 0.0, BASE_H + LID_H - WALL - 0.0002)) - hinge)),
                         (2 * IN_X, 2 * IN_Y, 0.0004), FELT, tilt=_wxyz(r))))
    return out


# Lid joint value: 0 open (105 degrees), -OPEN shut. Its spring rest is set every control step by the over-centre rule.
LID = Joint("lid", "hinge", axis="y", at=CASE_HINGE, low=-OPEN, high=0.0, mass=LID_MASS, spring=LID_SPRING,
            damping=LID_DAMPING, friction=0.6, shapes=_lid_shapes())
CASE = Mech("glasses_case", pos=(CASE_X, CASE_Y, TOP + 0.0005), mass=CASE_MASS, friction=0.6,
            shapes=_base_shapes(), joints=[LID])

OBJECTS = [GLASSES, CASE]


def lid_rest(q: float) -> float:
    """The over-centre spring: within FLIP of open its rest is open (it holds the lid open); past that, its rest is
    PRELOAD beyond shut (it snaps the lid shut and keeps pressing it down on the rim)."""
    return 0.0 if q > -FLIP else -OPEN - PRELOAD


_LID: dict = {"rest": 0.0}


@on_reset
def _lid_reset(env=None) -> None:
    _LID["rest"] = 0.0
    if env is not None:
        mechkit.set_rest(env, "glasses_case", "lid", 0.0)


@on_step
def _lid_spring(env) -> None:
    q = float(mechkit._mech_state(env, CASE).joints.get("lid", 0.0))
    rest = lid_rest(q)
    if rest != _LID["rest"]:
        _LID["rest"] = rest
        mechkit.set_rest(env, "glasses_case", "lid", rest)


# ---------------------------------------------------------------------------------------------------------------------
# Scoring
FOLDED_DEG = 75.0        # a temple counts as folded within 15 degrees of fully folded
LID_GAP = 0.001          # shut: the lid's front edge within 1 mm of the rim
INSIDE_TOL = 0.001       # every point of the glasses inside the case's interior (to 1 mm)
CLEAR = 0.03             # let go: every finger pad point of both arms 3 cm from the case and the glasses
UPRIGHT_DEG = 10.0
LIN_STILL, ANG_STILL = 0.003, math.radians(3.0)
HOLD_STEPS = 20          # 1 s at 20 Hz
OFF_TABLE = 0.01         # a part whose lowest point is 1 cm below the table top has fallen off it

OFFLINE_RELEASED = False  # offline there are no fingers: the tests set this


def fold_deg(gs) -> tuple[float, float]:
    """How far (degrees) the left and right temples are folded from open (90 = fully folded)."""
    return (-math.degrees(float(gs.joints.get("left_temple", 0.0))),
            math.degrees(float(gs.joints.get("right_temple", 0.0))))


def lid_gap(cs) -> float:
    """Height (m) of the lid's front edge over the rim: 0 when shut."""
    shut = float(cs.joints.get("lid", 0.0)) + OPEN
    return LID_DEPTH * math.sin(max(0.0, shut))


def glasses_points(gs) -> np.ndarray:
    """Every corner of every glasses collider, in the world."""
    rw, ow = gs.rotation(), np.asarray(gs.origin, float)
    bodies = mechkit.body_poses(GLASSES, gs)
    pts = []
    for body, c, size, r in GLASS_BOXES:
        rb, ob = (rw, ow) if body == "base" else bodies[body]
        for sx in (-0.5, 0.5):
            for sy in (-0.5, 0.5):
                for sz in (-0.5, 0.5):
                    pts.append(ob + rb @ (c + r @ (np.array([sx, sy, sz]) * size)))
    return np.array(pts)


def in_case(gs, cs) -> bool:
    """Every point of the glasses inside the case's interior (floor to the lid's underside), to 1 mm."""
    local = (glasses_points(gs) - np.asarray(cs.origin, float)) @ cs.rotation()
    t = INSIDE_TOL
    return bool(np.all((np.abs(local[:, 0]) <= IN_X + t) & (np.abs(local[:, 1]) <= IN_Y + t)
                       & (local[:, 2] >= IN_Z[0] - t) & (local[:, 2] <= IN_Z[1] + t)))


def upright_deg(cs) -> float:
    return math.degrees(math.acos(max(-1.0, min(1.0, float(cs.rotation()[2, 2])))))


def still(s) -> bool:
    return float(np.linalg.norm(s.lin_vel)) < LIN_STILL and float(np.linalg.norm(s.ang_vel)) < ANG_STILL


def _released(gs, cs) -> bool:
    if mechkit._ENV.get("env") is None:
        return bool(OFFLINE_RELEASED)
    return bool(mechkit.released(cs, clearance=CLEAR) and mechkit.released(gs, clearance=CLEAR))


def tier(gs, cs) -> float:
    """The partial score of a state: 0.75 both temples folded and the glasses inside the case; 0.5 both folded;
    0.25 one folded; else 0."""
    n = sum(f >= FOLDED_DEG for f in fold_deg(gs))
    if n == 2 and in_case(gs, cs):
        return 0.75
    return 0.25 * n


def complete(gs, cs) -> bool:
    """Everything that full score needs at one step (the hold is counted outside)."""
    return bool(tier(gs, cs) >= 0.75 and lid_gap(cs) <= LID_GAP and upright_deg(cs) <= UPRIGHT_DEG
                and still(gs) and still(cs) and _released(gs, cs))


_ST: dict = {}


@on_reset
def _reset(env=None) -> None:
    _ST.clear()
    _ST.update(last=None, fell=False, held=0, score=0.0, info={})


_reset()


def _update(objects) -> None:
    """Advance once per control step (success() and score() get the same dict each step)."""
    if objects is _ST["last"]:
        return
    _ST["last"] = objects
    gs, cs = objects["glasses"], objects["glasses_case"]
    if float(gs.low[2]) < TOP - OFF_TABLE or float(cs.low[2]) < TOP - OFF_TABLE:
        _ST["fell"] = True
    done = complete(gs, cs)
    _ST["held"] = _ST["held"] + 1 if done else 0
    if _ST["fell"]:
        s = 0.0
    elif _ST["held"] >= HOLD_STEPS:
        s = 1.0
    else:
        s = tier(gs, cs)
    _ST["score"] = s
    _ST["info"] = {"fold": fold_deg(gs), "lid_gap_mm": 1000 * lid_gap(cs), "done": done, "held": _ST["held"]}
    if os.environ.get("GLASSES_DEBUG"):
        print(f"[glasses] {_ST['info']} score {s}", file=sys.stderr, flush=True)


def score(objects) -> float:
    """1.0 once the glasses have lain in the case with both temples folded (within 15 degrees), the lid shut (its
    front edge within 1 mm of the rim), the case upright, both grippers 3 cm clear and everything still, for 1 s
    (20 steps). Otherwise, from the state now: 0.75 both temples folded and the glasses inside the case; 0.5 both
    folded; 0.25 one folded; 0. A part that falls off the table scores 0 for the rest of the run."""
    _update(objects)
    return float(_ST["score"])


def success(objects) -> bool:
    """True only on steps where score() is 1.0."""
    return score(objects) >= 1.0
