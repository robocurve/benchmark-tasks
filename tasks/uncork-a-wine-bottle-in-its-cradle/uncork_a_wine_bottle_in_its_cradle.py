"""On the station table, a corked bottle of wine lies in a wooden pouring cradle, tipped up 15 degrees with its neck toward
the right arm; a T-handled corkscrew stands in a holder to the right.

Task: holding the bottle's neck with one hand, screw the corkscrew into the cork with the other and pull the cork out.
The thread drags and the cork is tight: turned or pulled with nothing holding the bottle, the bottle turns in the
cradle or slides out of it instead.

Run it:  inspect-robots run --embodiment yam_arena -E task=uncork_a_wine_bottle_in_its_cradle.py \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import (Body, BoxShape, CylShape, allow_matrix_grips, as_numpy, on_reset, on_step, plate,  # noqa: E402
                     rotation, segment, set_slide_target, set_thread_on, thread)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the bottle. Its frame: origin on its axis at the body's middle, the neck along -y; it lies tipped up 15 degrees
# (about x), its neck toward the right arm --------------------------------------------------------------------------
TILT = math.radians(15.0)
BODY_R, BODY_L = 0.0375, 0.200
NECK_IN, NECK_OUT, NECK_Y = 0.012, 0.0155, (-0.130, -0.220)  # the neck: bore, outside, from and to (y)
LIP_Y, LIP_R, LIP_L = NECK_Y[1], 0.0175, 0.006
BOTTLE_MASS = 1.2
LIP = np.array([0.27, 0.0, TOP + 0.125])  # the middle of the bottle's mouth, in the world

# -- the cork: in the neck, its outer face flush with the lip; frame at its middle, axes as the bottle's ------------
CORK_R, CORK_L = 0.0115, 0.040
CORK_Y0 = LIP_Y + CORK_L / 2
CORK_K, CORK_SET, CORK_TURN = 4000.0, 0.003, 5.0  # spring toward where it was last pulled to (N/m): it gives way at
# 12 N (3 mm), then stays where it is pulled; it hardly turns in the neck (N m s/rad)
CORK_OUT = CORK_L + 0.005  # out of the bottle: pulled this far (its inner face 5 mm past the lip)

# -- the corkscrew: a round wooden knob, a steel shank and worm; frame at the knob's middle, the worm along -z ---------
KNOB_R, KNOB_L = 0.010, 0.030
SHANK_R, SHANK_L, WORM_R, WORM_L, POINT_L = 0.0035, 0.015, 0.0035, 0.040, 0.005
TIP_Z = -(KNOB_L / 2 + SHANK_L + WORM_L + POINT_L)  # the worm's point
CORKSCREW_MASS = 0.06
HOLDER_XY, HOLDER_H, HOLDER_BASE, TUBE_W, TUBE_BORE = (0.25, -0.18), 0.070, (0.040, 0.040, 0.010), 0.012, 0.009
# the corkscrew stands in a stand: a base block and a slim tube the worm hangs in (narrower than the handle, so that
# fingers closing on the handle pass either side of it)

# -- the thread of the worm in the cork ------------------------------------------------------------------------------
PITCH = 0.012  # m per turn
CATCH = math.radians(90.0)  # catches after a quarter turn with the point in the cork, lined up
ENGAGE_TILT, ENGAGE_RADIAL, ENGAGE_DEPTH = math.radians(12.0), 0.006, (0.001, 0.012)
THREAD_DRAG = 0.08  # N m s/rad: turning the worm in the cork drags
WORM_K, WORM_D, WORM_MAX = 3000.0, 20.0, 30.0  # the cork drawn along the worm as it turns (N/m, N s/m, N)
DEPTH_MAX = WORM_L - 0.002

WOOD, GLASS, LABEL, CORK_C, STEEL, DARK = (0.55, 0.36, 0.20), (0.10, 0.25, 0.13), (0.92, 0.88, 0.78), \
    (0.72, 0.55, 0.36), (0.70, 0.71, 0.74), (0.18, 0.18, 0.20)

# -- scoring --------------------------------------------------------------------------------------------------------
TURNS_IN = 1.0
IN_CRADLE_MOVE, IN_CRADLE_TILT = 0.02, math.radians(5.0)
SUCCESS_HOLD_S = 1.0


def _quat_x(a: float) -> tuple[float, float, float, float]:
    return (math.sin(a / 2), 0.0, 0.0, math.cos(a / 2))


BOTTLE_Q = _quat_x(-TILT)
BOTTLE_R = rotation(BOTTLE_Q)
BOTTLE_POS = LIP - BOTTLE_R @ np.array([0.0, LIP_Y, 0.0])
OUTWARD = BOTTLE_R @ np.array([0.0, -1.0, 0.0])  # out of the bottle's mouth


def _neck_ring(r_in, r_out, y0, y1, color, n=12) -> list:
    out, r = [], (r_in + r_out) / 2
    chord = 2 * r * math.tan(math.pi / n) * 1.05
    for k in range(n):
        a = 2 * math.pi * k / n
        q = (0.0, math.sin(a / 2), 0.0, math.cos(a / 2))  # about y: the boxes round the neck's axis
        out.append(BoxShape((r * math.sin(a), (y0 + y1) / 2, r * math.cos(a)), (chord, abs(y1 - y0), r_out - r_in),
                            color, quat_xyzw=q))
    return out


def _bottle() -> Body:
    shapes = [CylShape((0.0, 0.0, 0.0), BODY_R, BODY_L, GLASS, axis="Y"),
              CylShape((0.0, -0.1075, 0.0), 0.028, 0.015, GLASS, axis="Y"),  # the shoulder
              CylShape((0.0, -0.1225, 0.0), 0.020, 0.015, GLASS, axis="Y")]
    shapes += _neck_ring(NECK_IN, NECK_OUT, NECK_Y[0], NECK_Y[1], GLASS)
    shapes += _neck_ring(NECK_IN, LIP_R, LIP_Y + LIP_L, LIP_Y, GLASS)
    shapes.append(BoxShape((0.0, 0.0, BODY_R + 0.0002), (0.040, 0.090, 0.0004), LABEL, collide=False))  # the label
    return Body.build("bottle", pos=tuple(BOTTLE_POS.tolist()), mass=BOTTLE_MASS, shapes=shapes, friction=(0.7, 0.6),
                      color=GLASS, start_xyzw=BOTTLE_Q, com=(0.0, 0.01, 0.0))


def _cradle() -> Body:
    """A wooden pouring cradle, in the bottle's (tipped) frame: two slats under the body, posts and a base board."""
    shapes, t, w = [], 0.008, 0.035
    for s in (-1.0, 1.0):
        d = np.array([s * math.sin(math.radians(40.0)), 0.0, -math.cos(math.radians(40.0))])
        m = (BODY_R + t / 2 + 0.0005) * d
        shapes.append(plate(m + [0.0, -0.085, 0.0], m + [0.0, 0.085, 0.0], w, t, d, WOOD))
    up = BOTTLE_R.T @ np.array([0.0, 0.0, 1.0])  # the world's vertical, in the bottle's frame
    for y in (-0.07, 0.07):
        top = np.array([0.0, y, -(BODY_R + 0.020)])
        foot_world = BOTTLE_POS + BOTTLE_R @ top
        foot = top - up * (foot_world[2] - TOP - 0.006)
        shapes.append(segment(top, foot, 0.016, WOOD))
    base_centre = BOTTLE_R.T @ (np.array([BOTTLE_POS[0], BOTTLE_POS[1], TOP + 0.003]) - BOTTLE_POS)
    shapes.append(BoxShape(tuple(base_centre.tolist()), (0.09, 0.22, 0.006), WOOD, quat_xyzw=_quat_x(TILT)))
    return Body.build("cradle", pos=tuple(BOTTLE_POS.tolist()), mass=1.0, shapes=shapes, kinematic=True,
                      friction=(0.6, 0.5), color=WOOD, start_xyzw=BOTTLE_Q)


def _cork() -> Body:
    pos = BOTTLE_POS + BOTTLE_R @ np.array([0.0, CORK_Y0, 0.0])
    return Body.build("cork", pos=tuple(pos.tolist()), mass=0.004,
                      shapes=[CylShape((0.0, 0.0, 0.0), CORK_R, CORK_L, CORK_C, axis="Y")], friction=(0.8, 0.7),
                      color=CORK_C, start_xyzw=BOTTLE_Q, ignore=("bottle", "corkscrew"))


def _corkscrew() -> Body:
    shapes = [CylShape((0.0, 0.0, 0.0), KNOB_R, KNOB_L, WOOD),
              CylShape((0.0, 0.0, -KNOB_L / 2 - SHANK_L / 2), SHANK_R, SHANK_L, STEEL),
              CylShape((0.0, 0.0, -KNOB_L / 2 - SHANK_L - WORM_L / 2), WORM_R, WORM_L, STEEL),
              CylShape((0.0, 0.0, TIP_Z + POINT_L / 2), 0.0015, POINT_L, STEEL)]
    pos = (HOLDER_XY[0], HOLDER_XY[1], TOP + HOLDER_H + KNOB_L / 2 + 0.0005)
    return Body.build("corkscrew", pos=pos, mass=CORKSCREW_MASS, shapes=shapes, friction=(1.2, 1.0), color=STEEL)


def _holder() -> Body:
    """The corkscrew's stand, frame at its foot: a base block and a slim square tube up to the handle."""
    bx, by, bz = HOLDER_BASE
    t, h = (TUBE_W - TUBE_BORE) / 2, HOLDER_H - bz
    shapes = [BoxShape((0.0, 0.0, bz / 2 + 0.0005), HOLDER_BASE, DARK)]
    for s in (-1.0, 1.0):
        shapes.append(BoxShape((s * (TUBE_W - t) / 2, 0.0, bz + h / 2 + 0.0005), (t, TUBE_W, h), DARK))
        shapes.append(BoxShape((0.0, s * (TUBE_W - t) / 2, bz + h / 2 + 0.0005), (TUBE_BORE, t, h), DARK))
    return Body.build("holder", pos=(HOLDER_XY[0], HOLDER_XY[1], TOP), mass=0.2, shapes=shapes, kinematic=True,
                      color=DARK)


CRADLE, HOLDER, BOTTLE, CORKSCREW, CORK = _cradle(), _holder(), _bottle(), _corkscrew(), _cork()
OBJECTS = [CRADLE, HOLDER, BOTTLE, CORKSCREW, CORK]
# the cork in the neck: it slides out only when pulled hard (the task file moves the spring's target), hardly turns
thread("neck", "bottle", "cork", anchor0=(0.0, CORK_Y0, 0.0), axis0=(0.0, -1.0, 0.0), anchor1=(0.0, 0.0, 0.0),
       axis1=(0.0, -1.0, 0.0), slide_limits=(0.0, 0.08), turn_damping=CORK_TURN, slide_stiffness=CORK_K,
       slide_damping=50.0, slide_max_force=1.0e3, always_on=True)
# the worm in the cork, switched on once it catches: the cork's outer face this deep along the worm from its point
thread("worm", "corkscrew", "cork", anchor0=(0.0, 0.0, TIP_Z), axis0=(0.0, 0.0, 1.0), anchor1=(0.0, -CORK_L / 2, 0.0),
       axis1=(0.0, -1.0, 0.0), slide_limits=(0.0, DEPTH_MAX), turn_damping=THREAD_DRAG, slide_stiffness=WORM_K,
       slide_damping=WORM_D, slide_max_force=WORM_MAX)


# -- followed after every control step: the thread, the cork in the neck, the grippers ------------------------------

def _fresh() -> dict:
    return {"pre": 0.0, "caught": False, "x0": 0.0, "turns": 0.0, "phi": None, "out_set": 0.0, "out": 0.0,
            "in_neck": True, "tcps": None}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("CORK_DEBUG")), "steps": 0}


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())
    _DEBUG["steps"] = 0
    set_slide_target("neck", 0.0)
    set_thread_on("neck", True)  # back in the neck if it was pulled out


def _tcps(base, origin) -> list[np.ndarray]:
    out = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        i = next(j for j, n in enumerate(robot.body_names) if n == "gripper" or n.endswith("/gripper"))
        g = as_numpy(robot.data.body_link_pose_w)[0][i]
        out.append(g[:3] - origin + rotation(g[3:7]) @ np.array(TCP_OFFSET))
    return out


def _geometry(c_pos, c_rot, k_pos, k_rot) -> dict:
    """The worm against the cork: how deep its point is in, how far off the cork's axis, and their turn."""
    d = c_rot @ np.array([0.0, 0.0, -1.0])  # the worm, handle to point
    tip = np.asarray(c_pos, float) + c_rot @ np.array([0.0, 0.0, TIP_Z])
    n = k_rot @ np.array([0.0, 1.0, 0.0])  # into the cork
    face = np.asarray(k_pos, float) - (CORK_L / 2) * n
    v = tip - face
    depth = float(v @ n)
    u = k_rot @ np.array([1.0, 0.0, 0.0])
    v1 = c_rot @ np.array([1.0, 0.0, 0.0])
    v2 = np.cross(d, v1)
    return {"depth": depth, "radial": float(np.linalg.norm(v - depth * n)), "cos": float(d @ n),
            "phi": math.atan2(float(u @ v2), float(u @ v1))}


def _cork_out(b_pos, b_rot, k_pos) -> float:
    """How far the cork has come out of the neck."""
    start = np.asarray(b_pos, float) + b_rot @ np.array([0.0, CORK_Y0, 0.0])
    return float((np.asarray(k_pos, float) - start) @ (b_rot @ np.array([0.0, -1.0, 0.0])))


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    c = as_numpy(base.scene["corkscrew"].data.root_link_pose_w)[0]
    k = as_numpy(base.scene["cork"].data.root_link_pose_w)[0]
    b = as_numpy(base.scene["bottle"].data.root_link_pose_w)[0]
    cr, kr, br = rotation(c[3:7]), rotation(k[3:7]), rotation(b[3:7])
    g = _geometry(c[:3] - origin, cr, k[:3] - origin, kr)
    _EP["tcps"] = _tcps(base, origin)
    tighten = 0.0
    if _EP["phi"] is not None:
        tighten = -((g["phi"] - _EP["phi"] + math.pi) % (2 * math.pi) - math.pi)
    _EP["phi"] = g["phi"]
    if _EP["caught"]:
        _EP["turns"] += tighten / (2 * math.pi)
        target = _EP["x0"] + PITCH * _EP["turns"]
        if target < -PITCH / 4:  # unscrewed off the point
            set_thread_on("worm", False)
            _EP.update(caught=False, pre=0.0, turns=0.0)
        else:
            set_slide_target("worm", min(max(target, 0.0), DEPTH_MAX))
    else:
        engaged = (g["cos"] >= math.cos(ENGAGE_TILT) and g["radial"] <= ENGAGE_RADIAL
                   and ENGAGE_DEPTH[0] <= g["depth"] <= ENGAGE_DEPTH[1])
        _EP["pre"] = _EP["pre"] + tighten if engaged else 0.0
        if _EP["pre"] >= CATCH:
            set_slide_target("worm", g["depth"])
            set_thread_on("worm", True)
            _EP.update(caught=True, x0=g["depth"], turns=0.0)
    out = _cork_out(b[:3] - origin, br, k[:3] - origin)
    _EP["out"] = max(_EP["out"], out)
    if _EP["in_neck"]:
        if out > CORK_OUT:  # out of the bottle
            set_thread_on("neck", False)
            _EP["in_neck"] = False
        elif out > _EP["out_set"] + CORK_SET:  # pulled hard: it gives way, and stays where it is pulled to
            _EP["out_set"] = out - CORK_SET
            set_slide_target("neck", _EP["out_set"])
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 10 == 0:
            st = _status_from(b[:3] - origin, br, k[:3] - origin)
            tl, tr = _EP["tcps"]
            neck = b[:3] - origin + br @ np.array([0.0, LIP_Y + GRIP_FROM_LIP, 0.0])
            print(f"[cork] bottle={np.round((b[:3] - origin) * 1000).astype(int).tolist()} cork={np.round((k[:3] - origin) * 1000).astype(int).tolist()} lip={np.round((b[:3] - origin + br @ np.array([0.0, LIP_Y, 0.0])) * 1000).astype(int).tolist()} "
                  f"t={_DEBUG['steps'] / 20:5.1f}s depth={g['depth'] * 1000:6.1f}mm radial={g['radial'] * 1000:4.1f} "
                  f"tilt={math.degrees(math.acos(max(-1.0, min(1.0, g['cos'])))):4.1f} pre={math.degrees(_EP['pre']):6.1f} "
                  f"caught={_EP['caught']} turns={_EP['turns']:5.2f} out={out * 1000:5.1f}mm in_neck={_EP['in_neck']} "
                  f"bottle moved={st['moved'] * 1000:4.1f}mm turned={math.degrees(st['turned']):4.1f} "
                  f"left TCP-neck={np.round((tl - neck) * 1000).astype(int).tolist()} "
                  f"right TCP-handle={np.round((tr - (c[:3] - origin)) * 1000).astype(int).tolist()} score={_points(st)}",
                  file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------


def _status_from(b_pos, b_rot, k_pos) -> dict:
    moved = float(np.linalg.norm(np.asarray(b_pos, float) - BOTTLE_POS))
    turned = float(math.acos(max(-1.0, min(1.0, float(np.trace(BOTTLE_R.T @ b_rot) - 1.0) / 2))))
    out = _cork_out(b_pos, b_rot, k_pos)
    return {"caught": _EP["caught"], "turns": _EP["turns"], "out": out >= CORK_OUT, "moved": moved, "turned": turned,
            "in_cradle": moved <= IN_CRADLE_MOVE and turned <= IN_CRADLE_TILT}


def _status(objects) -> dict:
    b, k = objects["bottle"], objects["cork"]
    return _status_from(b.pos, rotation(b.quat_xyzw), k.pos)


def _points(st) -> float:
    if st["caught"] and st["out"] and st["in_cradle"]:
        return 1.0
    if st["caught"] and st["turns"] >= TURNS_IN:
        return 0.5
    return 0.25 if st["caught"] else 0.0


def score(objects) -> float:
    """1.0 with the cork pulled out of the bottle on the corkscrew and the bottle still in its cradle (moved no more
    than 2 cm, turned no more than 5 degrees); 0.5 with the corkscrew caught in the cork and turned at least 1 turn
    in; 0.25 with it caught (a quarter turn in); else 0.0."""
    return _points(_status(objects))


def success(objects) -> bool:
    """Full score with the bottle still."""
    return _points(_status(objects)) >= 1.0 and objects["bottle"].resting()


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The left hand comes straight down on the bottle's neck and holds it. The right hand comes straight down on the
# corkscrew's handle, lifts the corkscrew out of its holder and turns it to point down the bottle's neck, sets the point
# on the middle of the cork and screws it in, in strokes of a third of a turn (the wrist turns only so far): turn, let
# go, turn back open, take the handle again, following the worm in. Then it pulls straight back along the neck,
# drawing the cork out, and holds it.


def _rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


DOWN_L, DOWN_R = grip_rotation(-math.pi / 2), grip_rotation(math.pi / 2)  # straight down, fingers closing along x
SIDE_R = _rx(math.pi / 2) @ DOWN_R  # pointing +y


def _aim(roll: float) -> np.ndarray:
    """The right hand pointing down the bottle's neck (into the bottle), turned ``roll`` about that."""
    return _rx(-TILT) @ SIDE_R @ _rz(roll)


GRIP_FROM_LIP = 0.045  # the left hand holds the neck this far in from the lip
BAR_IN_HAND = 0.003  # the right TCP this far above the knob's middle (as it stands in the holder)
R_FIX = np.array([0.0, 0.0, 0.0])  # where the right hand misses by, holding the corkscrew: aim the other way
L_FIX = np.array([0.0, 0.0, 0.0])
ROLL0, STROKE, STROKES = math.radians(-40.0), math.radians(120.0), 7
START_DEPTH, READY_GAP, PULL = 0.004, 0.02, 0.065
LEAD_S, LEAD_STEPS = 3.0, 15
PICK = [1.0, 0.8, 0.3]
TURN_S, RELEASE_S, BACK_S, REGRIP_S = 1.6, 0.4, 0.8, 0.4
ABOVE = 0.05


def _tcp_for_tip(depth: float) -> np.ndarray:
    """The right TCP with the worm's point ``depth`` into the cork (negative: short of it)."""
    tip = LIP - OUTWARD * depth
    return tip + OUTWARD * (BAR_IN_HAND - TIP_Z) + R_FIX


def _depth_after(turned: float) -> float:
    return START_DEPTH + PITCH * max(0.0, turned - CATCH) / (2 * math.pi)


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


def _lead_in(arm: str, pos, rot, label: str) -> list[Waypoint]:
    from yam_arena import ik, station
    from yam_arena.constants import READY

    q0, q1 = np.array(READY, float), _good_ik(arm, pos, rot)
    out = []
    for k in range(1, LEAD_STEPS + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / LEAD_STEPS, station.BASES[arm])
        out.append(Waypoint(LEAD_S / LEAD_STEPS, tuple(float(v) for v in p), FINGER_OPEN, label, r))
    return out


def _w(t, pos, finger, label, rot) -> Waypoint:
    return Waypoint(t, tuple(float(v) for v in pos), finger, label, rot)


def _right() -> list[Waypoint]:
    pick = np.array(CORKSCREW.pos) + [0.0, 0.0, BAR_IN_HAND]
    out = _lead_in("right", pick + [0.0, 0.0, ABOVE], DOWN_R, "over the corkscrew")
    out += [_w(PICK[0], pick, FINGER_OPEN, "down round its handle", DOWN_R),
            _w(PICK[1], pick, FINGER_CLOSED, "grip it", DOWN_R),
            _w(PICK[2], pick, FINGER_CLOSED, "settle", DOWN_R),
            _w(1.2, pick + [0.0, 0.0, 0.08], FINGER_CLOSED, "lift it out of the holder", DOWN_R),
            _w(1.2, _tcp_for_tip(-READY_GAP) + [0.0, -0.03, 0.03], FINGER_CLOSED, "turn it toward the bottle",
               _rx(-TILT / 2) @ _rx(math.pi / 4) @ DOWN_R),
            _w(1.2, _tcp_for_tip(-READY_GAP), FINGER_CLOSED, "point it down the neck, at the cork", _aim(ROLL0)),
            _w(1.0, _tcp_for_tip(START_DEPTH), FINGER_CLOSED, "set the point in the cork", _aim(ROLL0))]
    turned = 0.0
    for k in range(STROKES):
        for j in range(1, 5):  # a quarter of the stroke at a time (slerp takes the short way round)
            done = turned + STROKE * j / 4
            out.append(_w(TURN_S / 4, _tcp_for_tip(_depth_after(done)), FINGER_CLOSED, f"screw it in ({k + 1})",
                          _aim(ROLL0 + STROKE * j / 4)))
        turned += STROKE
        at = _tcp_for_tip(_depth_after(turned))
        if k < STROKES - 1:
            out.append(_w(RELEASE_S, at, FINGER_OPEN, "let go", _aim(ROLL0 + STROKE)))
            for j in range(1, 5):
                out.append(_w(BACK_S / 4, at, FINGER_OPEN, "turn back, open", _aim(ROLL0 + STROKE * (1 - j / 4))))
            out.append(_w(REGRIP_S, at, FINGER_CLOSED, "take the handle again", _aim(ROLL0)))
    end = _tcp_for_tip(_depth_after(turned))
    out += [_w(0.5, end, FINGER_CLOSED, "hold", _aim(ROLL0 + STROKE)),
            _w(2.5, end + OUTWARD * PULL, FINGER_CLOSED, "pull the cork out along the neck", _aim(ROLL0 + STROKE)),
            _w(4.0, end + OUTWARD * PULL, FINGER_CLOSED, "hold it", _aim(ROLL0 + STROKE))]
    return out


def _left(total: float) -> list[Waypoint]:
    neck = BOTTLE_POS + BOTTLE_R @ np.array([0.0, LIP_Y + GRIP_FROM_LIP, 0.0]) + L_FIX
    out = _lead_in("left", neck + [0.0, 0.0, ABOVE], DOWN_L, "over the bottle's neck")
    out += [_w(PICK[0], neck, FINGER_OPEN, "down round the neck", DOWN_L),
            _w(PICK[1], neck, FINGER_CLOSED, "grip it", DOWN_L)]
    t = LEAD_S + PICK[0] + PICK[1]
    out.append(_w(max(total - t, 0.5), neck, FINGER_CLOSED, "hold the bottle still", DOWN_L))
    return out


def demo():
    right = _right()
    total = sum(w.duration for w in right)
    return {"left": _left(total), "right": right}
