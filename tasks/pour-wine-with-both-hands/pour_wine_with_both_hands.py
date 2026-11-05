"""On the station table, a slim dark green bottle of wine (27.5 cm tall, 6 cm across, sloping shoulders, 0.8 kg) stands
upright in front of the arms, open, on the centre line. Inside it is the wine: 10 dark red beads (5 mm) on its floor. In
front of it stands a stemmed wine glass (bowl 6 cm across inside and 5 cm deep, 7.8 cm tall). All three are loose.

Task: pour a glass of wine with both hands, one hand on the bottle's neck and the other on its body: at least 8 of the
10 beads in the glass, no more than 1 spilled, and the glass left standing. While the bottle is tipped more than 45 degrees from
upright both grippers must be holding it, the neck may never point more than 60 degrees below level, and the glass
must not be knocked over.

Run it:  inspect-robots run --embodiment yam_arena -E task=pour_wine_with_both_hands.py \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import (  # noqa: E402
    Body, CylShape, SphereShape, allow_matrix_grips, as_numpy, on_reset, on_step, plate, rotation,
)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the bottle, in its own frame: origin at the centre of its underside, the neck up (+z) ---------------------------
R_O, WALL = 0.030, 0.0035
R_I = R_O - WALL
BASE_T = 0.005
BODY_END, SHOULDER_END, MOUTH = 0.160, 0.225, 0.275  # body, sloping shoulder (inside slope about 15 degrees), neck
NECK_R_O = 0.0125
NECK_R_I = NECK_R_O - WALL
BOTTLE_MASS = 0.8

# -- layout (world frame: x forward from the arm bases, y left, z up; metres) ----------------------------------------
BOTTLE_XY = (0.22, 0.0)
GLASS_XY = (0.34, 0.0)
G_BASE_R, G_BASE_T, STEM_R, STEM_H = 0.032, 0.004, 0.0045, 0.020
BOWL_R_I, BOWL_T, BOWL_H, BOWL_FLOOR_T = 0.030, 0.003, 0.050, 0.004
BOWL_FLOOR_Z = G_BASE_T + STEM_H
GLASS_H = BOWL_FLOOR_Z + BOWL_FLOOR_T + BOWL_H
GLASS_MASS = 0.15

BEAD_R, BEAD_MASS, N_BEADS = 0.0025, 0.0006, 10

GREEN, LABEL, WINE, GLASS = (0.10, 0.24, 0.13), (0.92, 0.88, 0.76), (0.42, 0.03, 0.08), (0.80, 0.86, 0.92)

# -- scoring --------------------------------------------------------------------------------------------------------
FULL_IN, FULL_SPILL = 8, 1
HALF_IN, HALF_SPILL = 5, 3
SOME_IN = 2
TIPPED = math.radians(45)  # tipped further than this from upright, the bottle must be in both grippers
STEEP = math.radians(60)  # the neck may never point more than this far below level
KNOCKED = math.radians(30)  # the glass tilted this far at any time has been knocked over
UPRIGHT = math.radians(10)  # and must end standing within this
SUCCESS_HOLD_S = 1.0


def _chord(r: float, n: int) -> float:
    return 2 * r * math.tan(math.pi / n) + 0.001


def _ring(n: int, r0: float, z0: float, z1: float, width: float, color, r1: float | None = None) -> list:
    """Wall panels of a tube (or a cone, if r1 differs) around the bottle's axis, from z0 to z1."""
    out = []
    for k in range(n):
        a = 2 * math.pi * k / n
        u = np.array([math.cos(a), math.sin(a), 0.0])
        end = r0 if r1 is None else r1
        out.append(plate(tuple(u * r0 + [0, 0, z0]), tuple(u * end + [0, 0, z1]), width, WALL, tuple(u), color))
    return out


def _bottle() -> Body:
    rb, rn = R_O - WALL / 2, NECK_R_O - WALL / 2
    shapes = [CylShape((0.0, 0.0, BASE_T / 2), R_O, BASE_T, GREEN)]
    shapes += _ring(16, rb, 0.0, BODY_END, _chord(rb, 16), GREEN)
    # shoulder panels as wide as the shoulder's widest ring, so no slot opens between them there (a bead wedged in a
    # slot narrower than itself gets squeezed out at absurd speed); they simply overlap toward the neck
    shapes += _ring(16, rb, BODY_END, SHOULDER_END, _chord(rb, 16), GREEN, r1=rn)
    shapes += _ring(12, rn, SHOULDER_END, MOUTH, _chord(rn, 12), GREEN)
    shapes.append(CylShape((0.0, 0.0, 0.075), R_O + 0.0025, 0.08, LABEL, collide=False))  # the label (visual only)
    return Body.build("bottle", pos=(*BOTTLE_XY, TOP + 0.0005), mass=BOTTLE_MASS, shapes=shapes, friction=(0.8, 0.7),
                      color=GREEN)


def _bead_spots() -> list[tuple[float, float, float]]:
    """Bottle-frame centres: hexagonal layers on the floor, the second in the hollows of the first."""
    d = 2 * BEAD_R + 0.0005
    rmax = R_I - BEAD_R - 0.0015
    spots = []
    for layer in range(4):
        z = BASE_T + BEAD_R + 0.0005 + layer * d * math.sqrt(2 / 3)
        ox, oy = (d / 2, d / (2 * math.sqrt(3))) if layer % 2 else (0.0, 0.0)
        pts = [(ox + (i + 0.5 * (j % 2)) * d, oy + j * d * math.sqrt(3) / 2, z) for j in range(-9, 10) for i in range(-9, 10)]
        pts = sorted([p for p in pts if math.hypot(p[0], p[1]) <= rmax], key=lambda p: (round(math.hypot(p[0], p[1]), 6), p))
        spots += pts
        if len(spots) >= N_BEADS:
            break
    return spots[:N_BEADS]


def _beads() -> list[Body]:
    bx, by = BOTTLE_XY
    return [Body.build(f"wine_{i:02d}", pos=(bx + x, by + y, TOP + 0.0005 + z), mass=BEAD_MASS, color=WINE,
                       friction=(0.3, 0.25), shapes=[SphereShape((0.0, 0.0, 0.0), BEAD_R, WINE)], damping=(0.05, 0.3))
            for i, (x, y, z) in enumerate(_bead_spots())]


def _glass() -> Body:
    """Origin at the centre of the foot's underside."""
    shapes = [CylShape((0.0, 0.0, G_BASE_T / 2), G_BASE_R, G_BASE_T, GLASS),
              CylShape((0.0, 0.0, G_BASE_T + STEM_H / 2), STEM_R, STEM_H + 0.001, GLASS),
              CylShape((0.0, 0.0, BOWL_FLOOR_Z + BOWL_FLOOR_T / 2), BOWL_R_I + BOWL_T, BOWL_FLOOR_T, GLASS)]
    r, z0 = BOWL_R_I + BOWL_T / 2, BOWL_FLOOR_Z + BOWL_FLOOR_T
    for k in range(16):
        a = 2 * math.pi * k / 16
        u = (math.cos(a), math.sin(a), 0.0)
        shapes.append(plate((r * u[0], r * u[1], z0), (r * u[0], r * u[1], z0 + BOWL_H), _chord(r, 16), BOWL_T, u, GLASS))
    return Body.build("glass", pos=(*GLASS_XY, TOP + 0.0005), mass=GLASS_MASS, shapes=shapes, friction=(0.6, 0.5),
                      color=GLASS)


BOTTLE = _bottle()
BEADS = _beads()
GLASS_BODY = _glass()
OBJECTS = [BOTTLE, *BEADS, GLASS_BODY]
BEAD_NAMES = [b.name for b in BEADS]


# -- scoring --------------------------------------------------------------------------------------------------------

_EPISODE = {"one_hand": False, "steep": False, "knocked": False}


@on_reset
def _new_episode() -> None:
    _EPISODE.update(one_hand=False, steep=False, knocked=False)


# Is a gripper holding the bottle? Its fingers are closed past 80 % of open, and the midpoint between its pad tips is
# within 1.5 cm of the bottle's surface, between the bottle's base and mouth.
TIP_IN_GRIPPER = np.array([0.0, 0.0, TCP_OFFSET[2] + PAD_TIP_PAST_TCP])
_ARM_IDS: dict = {}


def _radius_at(z: float) -> float:
    if z <= BODY_END:
        return R_O
    if z >= SHOULDER_END:
        return NECK_R_O
    return R_O + (NECK_R_O - R_O) * (z - BODY_END) / (SHOULDER_END - BODY_END)


@on_step
def _watch_hands(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    bottle = as_numpy(base.scene["bottle"].data.root_link_pose_w)[0]
    b_rot, b_pos = rotation(bottle[3:7]), bottle[:3] - origin
    if math.acos(max(-1.0, min(1.0, b_rot[2, 2]))) <= TIPPED:
        return
    holding = 0
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        if arm not in _ARM_IDS:
            names, joints = list(robot.body_names), list(robot.joint_names)
            _ARM_IDS[arm] = (next(i for i, n in enumerate(names) if n == "gripper" or n.endswith("/gripper")),
                             joints.index("joint7"))
        body_i, finger_j = _ARM_IDS[arm]
        g = as_numpy(robot.data.body_link_pose_w)[0][body_i]
        finger = float(as_numpy(robot.data.joint_pos)[0][finger_j])
        tip = b_rot.T @ (g[:3] - origin + rotation(g[3:7]) @ TIP_IN_GRIPPER - b_pos)  # pad tips, bottle frame
        closed = finger > FINGER_OPEN * 0.8  # FINGER_OPEN is negative: closed is toward 0
        if closed and 0.0 <= tip[2] <= MOUTH and math.hypot(tip[0], tip[1]) <= _radius_at(tip[2]) + 0.015:
            holding += 1
    if holding < 2:
        _EPISODE["one_hand"] = True


def _tally(objects) -> dict:
    bottle, glass = objects["bottle"], objects["glass"]
    b_rot, g_rot = rotation(bottle.quat_xyzw), rotation(glass.quat_xyzw)
    in_glass = in_bottle = 0
    stray = []
    for name in BEAD_NAMES:
        w = np.asarray(objects[name].pos, float)
        g = g_rot.T @ (w - glass.pos)
        b = b_rot.T @ (w - bottle.pos)
        if math.hypot(g[0], g[1]) < BOWL_R_I and BOWL_FLOOR_Z < g[2] < GLASS_H:
            in_glass += 1
        elif 0.0 < b[2] < MOUTH and math.hypot(b[0], b[1]) < R_I:
            in_bottle += 1
        else:
            stray.append(np.round(g * 100, 1).tolist())
    neck = b_rot[:, 2]
    mouth_g = g_rot.T @ (bottle.pos + b_rot @ np.array([0.0, 0.0, MOUTH]) - glass.pos)
    below = math.asin(max(-1.0, min(1.0, -neck[2])))
    glass_tilt = float(math.acos(max(-1.0, min(1.0, g_rot[2, 2]))))
    if below > STEEP:
        _EPISODE["steep"] = True
    if glass_tilt >= KNOCKED:
        _EPISODE["knocked"] = True
    return {"in_glass": in_glass, "in_bottle": in_bottle, "spilled": N_BEADS - in_glass - in_bottle, "below": below,
            "tipped": float(math.acos(max(-1.0, min(1.0, neck[2])))), "glass_tilt": glass_tilt,
            "glass_down": abs(glass.pos[2] - TOP) < 0.01, "mouth_in_glass_frame": mouth_g, "stray": stray, **_EPISODE}


def _points(t) -> float:
    if t["one_hand"] or t["steep"] or t["knocked"] or t["glass_tilt"] >= UPRIGHT or not t["glass_down"]:
        return 0.0
    if t["in_glass"] >= FULL_IN and t["spilled"] <= FULL_SPILL:
        return 1.0
    if t["in_glass"] >= HALF_IN and t["spilled"] <= HALF_SPILL:
        return 0.5
    return 0.25 if t["in_glass"] >= SOME_IN else 0.0


_DEBUG = {"on": bool(os.environ.get("WINE_DEBUG")), "calls": 0, "fast": set()}


def _report_fast(objects) -> None:
    """WINE_DEBUG: the first time a bead moves faster than 2 m/s, print where it is in the bottle's and glass's frames."""
    bottle, glass = objects["bottle"], objects["glass"]
    b_rot, g_rot = rotation(bottle.quat_xyzw), rotation(glass.quat_xyzw)
    for name in BEAD_NAMES:
        o = objects[name]
        v = float(np.linalg.norm(o.lin_vel))
        if v > 2.0 and name not in _DEBUG["fast"]:
            _DEBUG["fast"].add(name)
            w = np.asarray(o.pos, float)
            print(f"[wine] FAST {name} at t={_DEBUG['calls'] / 20:.2f}s: {v:.1f} m/s; bottle frame (cm) "
                  f"{np.round(b_rot.T @ (w - bottle.pos) * 100, 1).tolist()}, glass frame (cm) "
                  f"{np.round(g_rot.T @ (w - glass.pos) * 100, 1).tolist()}", file=sys.stderr, flush=True)


def score(objects) -> float:
    """0 if, at any time, the bottle is tipped more than 45 degrees from upright without both grippers holding it, its
    neck points more than 60 degrees below level, or the glass is tilted 30 degrees (knocked over); or if the glass does
    not end standing on the table. Otherwise 1.0 with at least 8 of the 10 beads in the glass and at most 1 spilled
    (outside both the glass and the bottle); 0.5 with at least 5 in the glass and at most 3 spilled; 0.25 with at least
    2 in the glass; else 0."""
    t = _tally(objects)
    if _DEBUG["on"]:  # WINE_DEBUG=1: one line a second (at 20 Hz) on stderr
        _DEBUG["calls"] += 1
        _report_fast(objects)
        if _DEBUG["calls"] % 20 == 0:
            print(f"[wine] t={_DEBUG['calls'] / 20:5.1f}s glass={t['in_glass']} bottle={t['in_bottle']} "
                  f"spilled={t['spilled']} tipped={math.degrees(t['tipped']):.0f}deg "
                  f"neck_below={math.degrees(t['below']):.0f}deg glass_tilt={math.degrees(t['glass_tilt']):.0f}deg "
                  f"one_hand={t['one_hand']} knocked={t['knocked']} mouth_xyz_from_glass_cm="
                  f"{np.round(t['mouth_in_glass_frame'] * 100, 1).tolist()} score={_points(t)}",
                  file=sys.stderr, flush=True)
            if t["stray"] and _DEBUG["calls"] % 100 == 0:
                print(f"[wine] spilled beads, glass frame (cm): {t['stray'][:8]}", file=sys.stderr, flush=True)
    return _points(t)


def success(objects) -> bool:
    """Full score with the glass, the bottle and every bead still."""
    if _points(_tally(objects)) < 1.0:
        return False
    return all(objects[n].resting() for n in ("glass", "bottle", *BEAD_NAMES))


# -- scripted demo --------------------------------------------------------------------------------------------------
# Both grips reach the upright bottle level from their own sides, fingers closing front and back across it: the left
# on the body, the right on the neck, both centred on the bottle's axis. Each waits OUT to its own side, slides in with
# its fingers open (they are 9.4 cm apart, the bottle 6 cm across), closes, and from then on the bottle follows one
# shared path, given as the neck grip's position and how far the bottle is tipped forward (about world y). Both wrists
# roll with it, so the grips stay locked on the bottle.


def _rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


SIDE_GRIP = {  # level grips reaching in from each arm's own side, fingers closing along x
    "left": _rx(-math.pi / 2) @ grip_rotation(-math.pi / 2),  # pointing -y
    "right": _rx(math.pi / 2) @ grip_rotation(math.pi / 2),  # pointing +y
}
NECK_GRIP, BODY_GRIP = 0.250, 0.145  # heights on the bottle: the right hand on the neck, the left on the body
# (lower on the body, a level grip from the side cannot reach the centre line)
POUR_TIP = math.radians(110)  # the neck 20 degrees below level
MID_TIP = math.radians(60)
OUT = 0.07  # the grips wait this far out to their own sides of the bottle's axis (pad tips 2.7 cm off its side)
LEAD_S, LEAD_STEPS = 3.0, 15


START =np.array([BOTTLE_XY[0], BOTTLE_XY[1], TOP + NECK_GRIP])


def _pour_pivot() -> np.ndarray:
    """Neck grip position for pouring: the mouth's centre 3 mm past the glass's middle and 2.5 cm above its rim. The beads
    roll out over the lowest point of the lip, a few millimetres behind the mouth's centre, and drop almost straight
    down, so this puts them into the middle of the bowl."""
    mouth_from_grip = _ry(POUR_TIP) @ np.array([0.0, 0.0, MOUTH - NECK_GRIP])
    over = np.array([GLASS_XY[0] + 0.003, GLASS_XY[1], TOP + GLASS_H + 0.025])
    return over - mouth_from_grip


POUR = _pour_pivot()
READY_TIP = math.radians(95)  # tipped this far nothing runs out yet (the shoulder still slopes up): get over the glass
# first, then tip on to pour
LIFTED = START + [0.0, 0.0, 0.04]
SWING = np.array([(START[0] + POUR[0]) / 2, START[1], LIFTED[2] - 0.01])  # partway, tipped MID_TIP: the body clears the glass

# (seconds, neck-grip position, tip, fingers, label, out): "out" waypoints hold each grip OUT to its own side
PATH = [
    (1.5, START, 0.0, "open", "slide in round neck and body", False),
    (0.8, START, 0.0, "closed", "grip neck and body", False),
    (1.5, LIFTED, 0.0, "closed", "lift it", False),
    (2.5, SWING, MID_TIP, "closed", "tip it toward the glass", False),
    (2.0, POUR, READY_TIP, "closed", "over the glass, not yet running", False),
    (3.5, POUR, POUR_TIP, "closed", "tip on slowly and pour", False),
    (3.5, POUR, POUR_TIP, "closed", "let it run", False),
    (2.0, SWING, MID_TIP, "closed", "tip it back", False),
    (2.5, LIFTED, 0.0, "closed", "upright again", False),
    (1.5, START + [0.0, 0.0, 0.003], 0.0, "closed", "set it down", False),
    (0.8, START + [0.0, 0.0, 0.003], 0.0, "open", "let go", False),
    (1.2, START + [0.0, 0.0, 0.003], 0.0, "open", "slide back out", True),
]


def _grip_pose(arm: str, neck_at, tip: float):
    rot = _ry(tip)
    pos = np.asarray(neck_at, float)
    if arm == "left":
        pos = pos + rot @ np.array([0.0, 0.0, BODY_GRIP - NECK_GRIP])
    return pos, rot @ SIDE_GRIP[arm]


def _good_ik(arm: str, pos, rot) -> np.ndarray:
    import itertools

    from yam_arena import ik, station

    best = None
    for seed in itertools.product((-0.5, 0.0, 0.5), (0.8, 1.3), (0.4, 1.0), (-0.8, 0.0, 0.8), (-1.4, 0.0, 1.4), (0.0,)):
        q, pe, re = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(rot), np.array(seed), base_pos=station.BASES[arm],
                             iters=200)
        if pe < 0.001 and re < math.radians(0.5):
            room = float(np.minimum(q - ik.LOWER, ik.UPPER - q).min())
            if best is None or room > best[1]:
                best = (q, room)
    if best is None:
        raise ValueError(f"no IK solution for the {arm} grip at {pos}")
    return best[0]


def _arm(arm: str) -> list[Waypoint]:
    from yam_arena import ik, station
    from yam_arena.constants import READY

    side = np.array([0.0, OUT if arm == "left" else -OUT, 0.0])
    wait, rot0 = _grip_pose(arm, START, 0.0)
    q0, q1 = np.array(READY, float), _good_ik(arm, wait + side, rot0)
    out = []
    for k in range(1, LEAD_STEPS + 1):
        pos, rot = ik.tcp_pose(q0 + (q1 - q0) * k / LEAD_STEPS, station.BASES[arm])
        out.append(Waypoint(LEAD_S / LEAD_STEPS, tuple(float(v) for v in pos), FINGER_OPEN, "level, beside the bottle", rot))
    for dt, neck_at, tip, fingers, label, outside in PATH:
        pos, rot = _grip_pose(arm, neck_at, tip)
        if outside:
            pos = pos + side
        out.append(Waypoint(dt, tuple(float(v) for v in pos), FINGER_OPEN if fingers == "open" else FINGER_CLOSED, label,
                            rot))
    return out


def demo():
    return {"left": _arm("left"), "right": _arm("right")}
