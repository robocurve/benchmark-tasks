"""On the station table, in front of the arms: a short flanged pipe joint opened up for a new gasket. A grey steel
pipe stands upright on the table with a flange near its top, four bolt holes near the flange's corners, two on the far
side and two on the side nearer the robot. A matching flange on a second short length of pipe sits level a short way
above it, its holes over the lower flange's holes. It rests on two loose steel spacer blocks set between the flanges
at their left and right edges, which hold the joint open. On a block in front of the gap lies a flat black ring gasket
with a long handle tab pointing toward the robot, at the height of the gap's floor. Four loose steel bolts stand
upright on the table, two to the robot's left of the joint and two to its right.
"""

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import parts  # noqa: E402
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SUCCESS_HOLD_S = 3.0  # the trial ends at 1.0 once success() has held this long: the arms are clear by then
TOP = TABLE_TOP[SCENE]

STEEL, DARK, RUBBER, BLOCK_C, BOLT_C = (0.55, 0.57, 0.60), (0.35, 0.36, 0.38), (0.08, 0.08, 0.09), \
    (0.80, 0.62, 0.30), (0.72, 0.73, 0.75)

# World frame: x away from the arm bases, y to the robot's left, z up (m). Joint coordinates are relative to the
# pipe axis on the table (CX, CY, TOP).
CX, CY = 0.32, 0.0
HALF = 0.080  # flanges are 160 mm square
LOW_Z0, LOW_H = 0.020, 0.015  # lower flange: 20 to 35 mm, on the lower pipe
LOW_TOP = LOW_Z0 + LOW_H
GAP = 0.016  # 35 to 51 mm, held open by the spacer blocks
UP_Z0, UP_H = LOW_TOP + GAP, 0.010  # upper flange: 51 to 61 mm
UP_TOP = UP_Z0 + UP_H
BORE = 0.030  # half-width of the square bore; the pipe (60 mm square tube, 5 mm wall) fills it
PIPE_IN = 0.025  # half-width of the pipe's inside
UP_PIPE = 0.040  # the upper pipe stands 40 mm above the upper flange
UP_MASS = 2.1  # upper flange and its pipe, steel
SPACER = (0.040, 0.012, GAP)  # two loose steel spacer blocks (the spreader) between the flanges
SPACER_Y = 0.072  # their centres, left and right of the axis
SPACER_MASS = 0.060  # steel
BOLT_CIRCLE = 0.065
HOLE_UP = 0.0065  # half-width of the upper flange's square bolt holes (13 mm): they locate the bolts
HOLE_LOW = 0.008  # the lower flange's holes are clearance holes (16 mm)
H45 = BOLT_CIRCLE / math.sqrt(2)
HOLES = {"far_left": (H45, H45), "far_right": (H45, -H45), "near_left": (-H45, H45), "near_right": (-H45, -H45)}
FAR, NEAR = ("far_left", "far_right"), ("near_left", "near_right")

# Bolt: a 10 mm shank 45 mm long under a 20 mm head 10 mm tall (M10 x 45 size). Seated, the head rests on the upper
# flange and the shank runs through both flanges, its end 4 mm below the lower flange: with the spacers out and the
# flanges closed on the gasket, 16 mm would stand below the lower flange for a nut.
SHANK_R, SHANK_L, HEAD_R, HEAD_H = 0.005, 0.045, 0.010, 0.010
BOLT_MASS = 0.052  # steel
SEATED_Z = UP_TOP - SHANK_L  # bolt origin (bottom of the shank) when seated, relative to the table

# Ring gasket: an octagonal band whose flats face the bolts, 4 mm thick, with a handle tab toward the robot.
RING_OUT = BOLT_CIRCLE - SHANK_R - 0.0005  # outer radius: 0.5 mm inside the bolts' inner edges
RING_IN = 0.042
RING_T = 0.004
TAB_LEN, TAB_W, TAB_H = 0.085, 0.020, 0.010
RING_MASS = 0.050  # rubber, about 41 cm3


def _plate(z0, h, color, hole):
    """A square plate from boxes, leaving the bore and the four bolt holes open (part frame)."""
    xs = sorted({-HALF, HALF, -BORE, BORE, *[s * (H45 + d) for s in (-1, 1) for d in (-hole, hole)]})
    holes = [(hx - hole, hx + hole, hy - hole, hy + hole) for hx, hy in HOLES.values()]
    holes.append((-BORE, BORE, -BORE, BORE))
    out = []
    for i in range(len(xs) - 1):
        x0, x1 = xs[i], xs[i + 1]
        run = None
        for j in range(len(xs) - 1):
            y0, y1 = xs[j], xs[j + 1]
            mx, my = (x0 + x1) / 2, (y0 + y1) / 2
            open_ = any(a < mx < b and c < my < d for a, b, c, d in holes)
            if not open_:
                run = (run[0], y1) if run else (y0, y1)
            if run and (open_ or j == len(xs) - 2):
                ya, yb = run
                out.append(parts.box((x1 - x0, yb - ya, h), at=((x0 + x1) / 2, (ya + yb) / 2, z0 + h / 2), color=color))
                run = None
    return out


def _pipe(z0, z1):
    """A square steel tube round the pipe axis, 60 mm outside and 50 mm inside, from z0 to z1 (part frame)."""
    h, w, c = z1 - z0, BORE - PIPE_IN, (z0 + z1) / 2
    return [parts.box((2 * BORE, w, h), at=(0.0, s * (BORE - w / 2), c), color=STEEL) for s in (-1, 1)] + \
        [parts.box((w, 2 * PIPE_IN, h), at=(s * (BORE - w / 2), 0.0, c), color=STEEL) for s in (-1, 1)]


BLOCK_X = (-HALF - 0.135, -HALF)  # support block in front of the gap, its top level with the lower flange
LOWER = parts.Part(  # fixed: the lower pipe standing on the table, its flange and the gasket's support block
    "lower_flange", pos=(CX, CY, TOP),
    shapes=[*_plate(LOW_Z0, LOW_H, STEEL, HOLE_LOW), *_pipe(0.0, LOW_TOP),
            parts.box((BLOCK_X[1] - BLOCK_X[0], 0.140, LOW_TOP), at=(sum(BLOCK_X) / 2, 0.0, LOW_TOP / 2),
                      color=BLOCK_C)],
)
UPPER = parts.Part(  # loose: the upper flange and its pipe, resting on the spacer blocks; origin at its underside
    "upper_flange", pos=(CX, CY, TOP + UP_Z0), mass=UP_MASS,
    shapes=[*_plate(0.0, UP_H, STEEL, HOLE_UP), *_pipe(0.0, UP_H + UP_PIPE)],
)
SPACERS = [
    parts.Part(f"spacer_{i + 1}", pos=(CX, CY + s * SPACER_Y, TOP + LOW_TOP), mass=SPACER_MASS,
               shapes=[parts.box(SPACER, at=(0.0, 0.0, GAP / 2), color=DARK)])
    for i, s in enumerate((1, -1))
]

RING_START_X = -HALF - 0.065  # ring centre at the start, relative to the pipe axis
RING_MID = 0.050  # the band is 48 overlapping discs round this radius: a round ring, its edges scalloped by 0.6 mm
N_DISC = 48
DISC_R = RING_OUT - RING_MID
RING = parts.Part(
    "gasket", pos=(CX + RING_START_X, CY, TOP + LOW_TOP), mass=RING_MASS,
    shapes=[
        *[parts.cylinder(DISC_R, RING_T, at=(RING_MID * math.cos(a), RING_MID * math.sin(a), RING_T / 2), color=RUBBER)
          for a in (2 * math.pi * k / N_DISC for k in range(N_DISC))],
        parts.box((TAB_LEN + 0.010, TAB_W, TAB_H), at=(-RING_OUT - TAB_LEN / 2 + 0.005, 0.0, TAB_H / 2), color=RUBBER),
    ],
)

BOLT_SPOTS = [(0.20, 0.15), (0.20, 0.21), (0.20, -0.15), (0.20, -0.21)]  # two each side


def _bolt(i, xy):
    return parts.Part(
        f"bolt_{i}", pos=(*xy, TOP), mass=BOLT_MASS,
        shapes=[parts.cylinder(SHANK_R, SHANK_L, at=(0.0, 0.0, SHANK_L / 2), color=BOLT_C),
                parts.cylinder(HEAD_R, HEAD_H, at=(0.0, 0.0, SHANK_L + HEAD_H / 2), color=BOLT_C)],
    )


BOLTS = [_bolt(i + 1, xy) for i, xy in enumerate(BOLT_SPOTS)]
BOLT_NAMES = [b.name for b in BOLTS]
OBJECTS = [LOWER, UPPER, *SPACERS, RING, *BOLTS]

CENTRE_TOL = 0.004
FLAT_TOL = 3.0
SEAT_TOL_Z = 0.0015  # ring underside to the lower flange's top
UPRIGHT_TOL = 8.0
HEAD_TOL = 0.002  # bolt head underside to the upper flange's top
IN_HOLE = 0.003  # bolt head centre within 3 mm of the upper flange's hole centre, seen from above
RELEASE_GAP = 0.005


def _rot(q):
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _tilt(st) -> float:
    return math.degrees(math.acos(float(np.clip(_rot(st.quat_xyzw)[2, 2], -1.0, 1.0))))


# -- release check: no gripper link within 5 mm of the gasket or a counted bolt ------------------------------------
# In the simulator an on_step hook measures, every step, each part's distance to both grippers. A part that has not
# been measured yet (no step since the reset) is not credited: the check fails closed. Offline (near_misses.py sets
# OFFLINE_FLAG; no simulator, no robot) each case stands for a state with the grippers away.
_GAP: dict[str, float] = {}  # object -> closest gripper distance at the last step (m)
OFFLINE_FLAG = "GASKET_TASK_OFFLINE_CHECK"
_HAND_LINKS = ("gripper", "tip_left", "tip_right")
_SPECS = {o.name: o for o in OBJECTS}


@parts.on_reset
def _clear(sim):
    _GAP.clear()


@parts.on_step
def _record_gaps(sim):
    from yam_arena import analysis

    base = sim.env.unwrapped
    origin = base.scene.env_origins[0].cpu().numpy()
    states = sim.states()
    hands = []
    for arm in ("left_robot", "right_robot"):
        rob = base.scene[arm]
        poses = rob.data.body_link_pose_w
        poses = (poses.torch if hasattr(poses, "torch") else poses)[0].cpu().numpy().astype(float)
        links = {}
        for i, n in enumerate(rob.body_names):
            if n in _HAND_LINKS:
                t = np.eye(4)
                t[:3, :3] = _rot(poses[i, 3:])
                t[:3, 3] = poses[i, :3] - origin
                links[n] = t
        hands += analysis.world_hulls(links)
    for name in ("gasket", *BOLT_NAMES):
        st = states[name]
        boxes = _SPECS[name].world_boxes({"base": (np.asarray(st.pos, float), _rot(st.quat_xyzw))})
        _GAP[name] = min(float(analysis.hulls_to_box(hands, c, r, sz)[0]) for _, c, r, sz in boxes)


def _offline() -> bool:
    import os

    return os.environ.get(OFFLINE_FLAG) == "1"


def _released(name) -> bool:
    if name not in _GAP:
        return _offline()
    return _GAP[name] >= RELEASE_GAP - 1e-6


def _ring_in(objects) -> bool:
    """Between the flanges: lying flat on the lower flange, centred on the pipe axis, still and let go."""
    g = objects["gasket"]
    p = np.asarray(g.pos, float)
    return bool(
        abs(p[2] - (TOP + LOW_TOP)) <= SEAT_TOL_Z + 1e-6
        and _tilt(g) < FLAT_TOL
        and math.hypot(p[0] - CX, p[1] - CY) <= CENTRE_TOL + 1e-6
        and g.resting()
        and _released("gasket")
    )


def _bolt_in(st, upper, hole) -> bool:
    """Upright, head on the upper flange's top and centred over one of its holes, still. Measured in the upper
    flange's own frame, wherever it is (it is loose on its spacers)."""
    hx, hy = HOLES[hole]
    ru, pu = _rot(upper.quat_xyzw), np.asarray(upper.pos, float)
    rb, p = _rot(st.quat_xyzw), np.asarray(st.pos, float)
    under = ru.T @ (p + rb @ np.array([0.0, 0.0, SHANK_L]) - pu)  # centre of the head's underside
    head = ru.T @ (p + rb @ np.array([0.0, 0.0, SHANK_L + HEAD_H / 2]) - pu)  # centre of the head
    return bool(
        _tilt(st) <= UPRIGHT_TOL + 1e-6
        and abs(under[2] - UP_H) <= HEAD_TOL + 1e-6
        and math.hypot(head[0] - hx, head[1] - hy) <= IN_HOLE + 1e-6
        and st.resting()
    )


def _filled(objects, hole) -> bool:
    return any(_bolt_in(objects[n], objects["upper_flange"], hole) and _released(n) for n in BOLT_NAMES)


def success(objects):
    """The gasket is centred between the flanges and all four bolts are in."""
    return bool(_ring_in(objects) and all(_filled(objects, h) for h in HOLES))


def score(objects):
    """With the gasket centred: 0.2, plus 0.2 for each hole with a bolt in, in any order (1.0 with all four).
    Without it: 0."""
    if not _ring_in(objects):
        return 0.0
    return (1 + sum(_filled(objects, h) for h in HOLES)) / 5


# -- scripted demo: fixed waypoints, no state read ---------------------------------------------------------------
TIP_PAST_TCP = 0.013
TRAVEL_Z = TOP + 0.13
FINGER_BOLT = -(HEAD_R - 0.005)  # close 5 mm past the head's faces
FINGER_TAB = -(TAB_W / 2 - 0.003)
FINGER_NEAR = -(HEAD_R + 0.007)  # part open: 7 mm clear of a head, narrow enough to miss the next bolt
COMP = {}  # hole -> (dx, dy) aim offsets measured on the GPU


def _bolt_moves(spot, hole):
    bx, by = spot
    hx, hy = HOLES[hole]
    dx, dy = COMP.get(hole, (0.0, 0.0))
    tx, ty = CX + hx + dx, CY + hy + dy
    grip = TOP + SHANK_L + HEAD_H - 0.008 + TIP_PAST_TCP
    above = TOP + UP_TOP + 0.015 + (grip - TOP)  # shank end 15 mm above the upper flange
    drop = TOP + SEATED_Z + 0.003 + (grip - TOP)  # head 3 mm above its seat
    pre = TOP + UP_TOP + 0.002 + (grip - TOP)  # shank end 2 mm above the upper flange
    return [
        Waypoint(1.2, (bx, by, TRAVEL_Z), FINGER_NEAR, "over a bolt"),
        Waypoint(1.0, (bx, by, grip), FINGER_NEAR, "down to its head"),
        Waypoint(0.6, (bx, by, grip), FINGER_BOLT, "grip the head"),
        Waypoint(1.0, (bx, by, above + 0.01), FINGER_BOLT, "lift"),
        Waypoint(2.5, (tx, ty, above), FINGER_BOLT, f"over the {hole} hole"),
        Waypoint(1.5, (tx, ty, pre), FINGER_BOLT, "lower to just above the hole"),
        Waypoint(1.0, (tx, ty, pre), FINGER_BOLT, "settle"),
        Waypoint(1.2, (tx, ty, drop), FINGER_BOLT, "lower it in"),
        Waypoint(0.5, (tx, ty, drop), FINGER_NEAR, "let go"),
        Waypoint(0.8, (tx, ty, above + 0.005), FINGER_NEAR, "up"),
    ]


PARK_L = (0.16, 0.24, TOP + 0.11)
PARK_R = (0.16, -0.24, TOP + 0.11)


def _wait(w, until, park):
    """Hold at the park pose until time ``until`` (s)."""
    t = sum(x.duration for x in w)
    if until > t + 1e-9:
        w.append(Waypoint(until - t, park, FINGER_NEAR, "wait"))


def _t(w):
    return sum(x.duration for x in w)


def demo():
    """Left arm: far-left bolt. Right arm: far-right bolt, then the gasket. Left arm: near-left bolt. Right arm:
    near-right bolt. The arms take turns, so their grippers never work near each other."""
    left = [Waypoint(0.5, PARK_L, FINGER_NEAR, "start")]
    right = [Waypoint(0.5, PARK_R, FINGER_NEAR, "start")]
    left += _bolt_moves(BOLT_SPOTS[0], "far_left")
    left.append(Waypoint(1.0, PARK_L, FINGER_NEAR, "park"))
    _wait(right, _t(left) - 1.0, PARK_R)
    right += _bolt_moves(BOLT_SPOTS[2], "far_right")
    # Grip the tab 20 mm from its end and slide the gasket in to 3 mm past touching the far bolts, let go, back out.
    grip_off = RING_OUT + TAB_LEN - 0.020  # grip point from the ring centre
    tz = TOP + LOW_TOP + 0.002 + TIP_PAST_TCP  # finger tips 2 mm above the tab's underside
    x0 = CX + RING_START_X - grip_off
    contact = math.sqrt(2) * (BOLT_CIRCLE - SHANK_R - RING_OUT)  # ring centre past the axis when it touches
    x1 = CX + contact + 0.003 - grip_off
    right += [
        Waypoint(1.5, (x0, CY, tz + 0.04), FINGER_NEAR, "over the tab"),
        Waypoint(1.0, (x0, CY, tz), FINGER_NEAR, "down round the tab"),
        Waypoint(0.6, (x0, CY, tz), FINGER_TAB, "grip the tab"),
        Waypoint(3.5, (x1, CY, tz), FINGER_TAB, "slide the gasket in against the far bolts"),
        Waypoint(0.8, (x1, CY, tz), FINGER_TAB, "hold"),
        Waypoint(0.5, (x1, CY, tz), FINGER_NEAR, "let go"),
        Waypoint(1.0, (x1 - 0.03, CY, tz + 0.04), FINGER_NEAR, "back out"),
        Waypoint(1.0, PARK_R, FINGER_NEAR, "park"),
    ]
    _wait(left, _t(right) - 1.0, PARK_L)
    left += _bolt_moves(BOLT_SPOTS[1], "near_left")
    left.append(Waypoint(1.0, PARK_L, FINGER_NEAR, "park"))
    _wait(right, _t(left) - 1.0, PARK_R)
    right += _bolt_moves(BOLT_SPOTS[3], "near_right")
    right.append(Waypoint(1.0, PARK_R, FINGER_NEAR, "park"))
    right.append(Waypoint(3.0, PARK_R, FINGER_NEAR, "stay clear while the joint is judged"))
    _wait(left, _t(right), PARK_L)
    return {"left": left, "right": right}
