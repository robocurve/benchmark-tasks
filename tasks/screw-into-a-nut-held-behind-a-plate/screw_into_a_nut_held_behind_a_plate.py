"""On the station table, a steel plate stands upright between the two arms, square to the line between them, with a
hole through it. Ahead of the left arm, a long hex nut stands on a peg; ahead of the right arm, a hex-head screw stands
head up in a holder.

Task: hold the nut against the plate over the hole with one hand, put the screw through the hole from the other side
with the other hand and turn it into the nut until it is snug. The thread drags: a nut that is not held turns with the
screw and the screw does not go in.

Run it:  inspect-robots run --embodiment yam_arena -E task=screw_into_a_nut_held_behind_a_plate.py \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import (Body, BoxShape, CylShape, allow_matrix_grips, as_numpy, on_reset, on_step, rotation,  # noqa: E402
                     set_slide_target, set_thread_on, thread)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the plate (a fixed bracket: an upright plate on a foot), square to the y axis, centred on the centre line -------
HOLE_X, HOLE_Z = 0.26, TOP + 0.20  # the hole's centre
PLATE_T, PLATE_W, PLATE_H = 0.008, 0.10, 0.25  # thick (along y), wide (along x), tall
HOLE = 0.012  # a square hole, 12 mm: an M10 clearance hole (1 mm round the 10 mm shank)

# -- the screw (origin at the centre of the head's underside; at the start it stands head up, so its shank runs along
# its -z axis) and its holder ---------------------------------------------------------------------------------------
HEAD_AF, HEAD_H = 0.016, 0.0064  # M10 hex head (ISO 4017): across flats, tall
SHANK_R, SHANK_L = 0.005, 0.020  # M10 x 20: its 10 mm major diameter
POINT_R, POINT_L = 0.004, 0.003  # the last 3 mm of the shank narrower (a chamfered dog point, 8 mm)
SCREW_MASS = 0.025
HOLDER_XY, HOLDER_SIZE, HOLDER_BORE = (0.24, -0.12), (0.030, 0.030, 0.025), 0.011

# -- the nut (origin at its centre; its axis is its z) and its stand. Nut and head are under 2 cm across (wider, both
# fingers push at their 50 N cap and no longer centre the part), and the nut is long, so that a hand holds it round
# its middle ---------------------------------------------------
NUT_AF, NUT_T, NUT_BORE_R = 0.017, 0.030, 0.0055  # an M10 coupling nut (DIN 6334): across flats, long, the plain bore's inscribed radius (11 mm, round the 10 mm shank)
NUT_SINK_R, NUT_SINK_D = 0.0075, 0.003  # a wider mouth (counterbore) 3 mm deep on each face: a lead-in for the screw
NUT_MASS = 0.040
STAND_XY, STAND_SIZE = (0.24, 0.12), (0.030, 0.030, 0.025)  # a block with a peg through the nut's bore
PEG_R, PEG_H = 0.0025, 0.003  # short and loose: it keeps the nut from being pushed far, never jams it

# -- the thread -----------------------------------------------------------------------------------------------------
PITCH = 0.005  # m per turn (coarse, so that two turns draw the nut 1 cm)
CATCH = math.radians(90.0)  # the thread catches after a quarter turn with the screw's tip in the nut, lined up
ENGAGE_TILT, ENGAGE_RADIAL, ENGAGE_DEPTH = math.radians(10.0), 0.004, 0.001
TURN_DAMPING = 0.15  # N m s/rad: thread friction
SLIDE_K, SLIDE_D, SLIDE_MAX = 2000.0, 20.0, 20.0  # the nut is pulled along the screw as it turns (N/m, N s/m, N)
X_MIN, X_MAX = NUT_T / 2, SHANK_L + NUT_T / 2 - ENGAGE_DEPTH  # the nut's centre from the head's underside, on the screw

STEEL, ZINC, DARK, PLATE_C, MARK = (0.30, 0.30, 0.33), (0.80, 0.80, 0.83), (0.18, 0.18, 0.20), (0.62, 0.64, 0.68), \
    (0.95, 0.75, 0.10)

# -- scoring --------------------------------------------------------------------------------------------------------
HOLE_TOL = 0.004  # the screw's axis crosses the plate within 4 mm of the hole's centre
TURNS_FULL, TURNS_HALF = 1.5, 0.5
PLAY_MAX = 0.002  # head and nut together no more than 2 mm off the plate's faces
LET_GO = 0.04  # both grippers' TCPs at least this far from the screw's head and from the nut
SUCCESS_HOLD_S = 1.0


def _hex_solid(cz: float, af: float, h: float, color) -> list:
    """A solid hexagonal prism about the z axis, flats facing +-x."""
    side = af / math.sqrt(3.0)
    return [BoxShape((0.0, 0.0, cz), (af, side, h), color, yaw=k * math.pi / 3) for k in range(3)]


def _hex_ring(af: float, r_in: float, h: float, color, cz: float = 0.0) -> list:
    """A hex ring about the z axis, flats facing +-x, its bore a hexagon of inscribed radius r_in."""
    side, depth = af / math.sqrt(3.0), af / 2 - r_in
    out = []
    for k in range(6):
        a = k * math.pi / 3
        r = r_in + depth / 2
        out.append(BoxShape((r * math.cos(a), r * math.sin(a), cz), (depth, side, h), color, yaw=a))
    return out


def _screw() -> Body:
    shapes = _hex_solid(HEAD_H / 2, HEAD_AF, HEAD_H, STEEL)
    shapes.append(CylShape((0.0, 0.0, -(SHANK_L - POINT_L) / 2), SHANK_R, SHANK_L - POINT_L, STEEL))
    shapes.append(CylShape((0.0, 0.0, -SHANK_L + POINT_L / 2), POINT_R, POINT_L, STEEL))
    shapes.append(BoxShape((0.0, 0.0, HEAD_H + 0.0002), (0.012, 0.0015, 0.0004), MARK, collide=False))  # a mark on top
    pos = (HOLDER_XY[0], HOLDER_XY[1], TOP + HOLDER_SIZE[2] + 0.0005)
    return Body.build("screw", pos=pos, mass=SCREW_MASS, shapes=shapes, friction=(0.8, 0.7), color=STEEL)


def _nut() -> Body:
    core = NUT_T - 2 * NUT_SINK_D
    shapes = _hex_ring(NUT_AF, NUT_BORE_R, core, ZINC)
    for s in (-1.0, 1.0):
        shapes += _hex_ring(NUT_AF, NUT_SINK_R, NUT_SINK_D, ZINC, cz=s * (core + NUT_SINK_D) / 2)
    pos = (STAND_XY[0], STAND_XY[1], TOP + STAND_SIZE[2] + NUT_T / 2 + 0.0005)
    return Body.build("nut", pos=pos, mass=NUT_MASS, shapes=shapes, friction=(0.8, 0.7), color=ZINC)


def _bracket() -> Body:
    """The plate on its foot, in a frame at (HOLE_X, 0, TOP)."""
    hz, h, w, t = HOLE_Z - TOP, HOLE / 2, PLATE_W / 2, PLATE_T

    def box(x0, x1, z0, z1, y=0.0, ty=t, color=PLATE_C, collide=True):
        return BoxShape(((x0 + x1) / 2, y, (z0 + z1) / 2), (x1 - x0, ty, z1 - z0), color, collide=collide)

    shapes = [box(-w, w, 0.0005, hz - h), box(-w, w, hz + h, PLATE_H),  # below and above the hole
              box(-w, -h, hz - h, hz + h), box(h, w, hz - h, hz + h),  # either side of it
              box(-w, w, 0.0005, 0.0125, ty=0.08, color=DARK)]  # the foot
    m, line = 0.006, 0.0015  # a printed square round the hole, on both faces
    for s in (-1.0, 1.0):
        y = s * (t / 2 + 0.0002)
        for z0, z1 in ((hz - h - m, hz - h - m + line), (hz + h + m - line, hz + h + m)):
            shapes.append(box(-h - m, h + m, z0, z1, y=y, ty=0.0004, color=MARK, collide=False))
        for x0, x1 in ((-h - m, -h - m + line), (h + m - line, h + m)):
            shapes.append(box(x0, x1, hz - h - m, hz + h + m, y=y, ty=0.0004, color=MARK, collide=False))
    return Body.build("bracket", pos=(HOLE_X, 0.0, TOP), mass=1.0, shapes=shapes, kinematic=True, friction=(0.5, 0.4),
                      color=PLATE_C)


def _holder() -> Body:
    """A block with a square bore the screw's shank hangs in, frame at the block's foot."""
    sx, sy, sz = HOLDER_SIZE
    b, g = HOLDER_BORE / 2, (sx - HOLDER_BORE) / 2
    shapes = [BoxShape((0.0, (b + sy / 2) / 2 * s, sz / 2 + 0.0005), (sx, sy / 2 - b, sz), DARK) for s in (-1.0, 1.0)]
    shapes += [BoxShape(((b + sx / 2) / 2 * s, 0.0, sz / 2 + 0.0005), (g, HOLDER_BORE, sz), DARK) for s in (-1.0, 1.0)]
    return Body.build("holder", pos=(HOLDER_XY[0], HOLDER_XY[1], TOP), mass=0.2, shapes=shapes, kinematic=True,
                      color=DARK)


def _stand() -> Body:
    """The nut's stand: a block with a peg the nut sits on, frame at the block's foot."""
    sx, sy, sz = STAND_SIZE
    shapes = [BoxShape((0.0, 0.0, sz / 2 + 0.0005), STAND_SIZE, DARK),
              CylShape((0.0, 0.0, sz + 0.0005 + PEG_H / 2), PEG_R, PEG_H, DARK)]
    return Body.build("stand", pos=(STAND_XY[0], STAND_XY[1], TOP), mass=0.2, shapes=shapes, kinematic=True, color=DARK)


BRACKET, HOLDER, STAND, SCREW, NUT = _bracket(), _holder(), _stand(), _screw(), _nut()
for _name, _axis in (("thread_fwd", (0.0, 0.0, 1.0)), ("thread_rev", (0.0, 0.0, -1.0))):  # the nut either way round
    thread(_name, "screw", "nut", anchor0=(0.0, 0.0, 0.0), axis0=(0.0, 0.0, -1.0), anchor1=(0.0, 0.0, 0.0),
           axis1=_axis, slide_limits=(X_MIN, X_MAX), turn_damping=TURN_DAMPING, slide_stiffness=SLIDE_K,
           slide_damping=SLIDE_D, slide_max_force=SLIDE_MAX)
OBJECTS = [BRACKET, HOLDER, STAND, SCREW, NUT]


# -- the thread, followed after every control step ------------------------------------------------------------------
# The screw's turn against the nut about its own axis (from the head toward the tip) is followed every control step.
# With the screw's tip in the nut's bore and the two lined up, a quarter turn (tightening, clockwise seen from the
# head) catches the thread: the joint between them is switched on, so that the nut stays on the screw's axis, drags
# against the screw's turning, and from then on is drawn along the screw by PITCH for every turn (back again when the
# screw is turned the other way, off the end when it is turned back far enough).


def _fresh() -> dict:
    return {"pre": 0.0, "caught": None, "x0": 0.0, "turns": 0.0, "phi": None, "tcps": None}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("SCREW_DEBUG")), "steps": 0}


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())
    _DEBUG["steps"] = 0


def _tcps(base, origin) -> list[np.ndarray]:
    out = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        i = next(j for j, n in enumerate(robot.body_names) if n == "gripper" or n.endswith("/gripper"))
        g = as_numpy(robot.data.body_link_pose_w)[0][i]
        out.append(g[:3] - origin + rotation(g[3:7]) @ np.array(TCP_OFFSET))
    return out


def _geometry(s_pos, s_rot, n_pos, n_rot) -> dict:
    """Where the nut is on the screw's axis."""
    a = s_rot @ np.array([0.0, 0.0, -1.0])  # the screw's axis, head to tip
    d = np.asarray(n_pos, float) - np.asarray(s_pos, float)
    x = float(d @ a)
    u = n_rot @ np.array([1.0, 0.0, 0.0])
    v1 = s_rot @ np.array([1.0, 0.0, 0.0])
    v2 = np.cross(a, v1)
    return {"a": a, "x": x, "radial": float(np.linalg.norm(d - x * a)), "cos": float(a @ (n_rot @ np.array([0, 0, 1.0]))),
            "phi": math.atan2(float(u @ v2), float(u @ v1))}


@on_step
def _follow_thread(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    s = as_numpy(base.scene["screw"].data.root_link_pose_w)[0]
    n = as_numpy(base.scene["nut"].data.root_link_pose_w)[0]
    g = _geometry(s[:3] - origin, rotation(s[3:7]), n[:3] - origin, rotation(n[3:7]))
    _EP["tcps"] = _tcps(base, origin)
    tighten = 0.0
    if _EP["phi"] is not None:
        tighten = -((g["phi"] - _EP["phi"] + math.pi) % (2 * math.pi) - math.pi)  # the screw's turn on the nut, this step
    _EP["phi"] = g["phi"]
    if _EP["caught"]:
        _EP["turns"] += tighten / (2 * math.pi)
        target = _EP["x0"] - PITCH * _EP["turns"]
        if target > X_MAX + PITCH / 4:  # unscrewed off the end
            set_thread_on(_EP["caught"], False)
            _EP.update(caught=None, pre=0.0, turns=0.0)
        else:
            set_slide_target(_EP["caught"], min(max(target, X_MIN), X_MAX))
    else:
        engaged = (abs(g["cos"]) >= math.cos(ENGAGE_TILT) and g["radial"] <= ENGAGE_RADIAL
                   and g["x"] - NUT_T / 2 >= -0.0005 and SHANK_L - (g["x"] - NUT_T / 2) >= ENGAGE_DEPTH)
        _EP["pre"] = _EP["pre"] + tighten if engaged else 0.0
        if _EP["pre"] >= CATCH:
            name = "thread_fwd" if g["cos"] > 0 else "thread_rev"
            set_slide_target(name, g["x"])
            set_thread_on(name, True)
            _EP.update(caught=name, x0=g["x"], turns=0.0)
    if _DEBUG["on"]:  # SCREW_DEBUG=1: twice a second (at 20 Hz) on stderr
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 10 == 0:
            st = _status_from(s[:3] - origin, rotation(s[3:7]), n[:3] - origin, rotation(n[3:7]))
            tl, tr = _EP["tcps"]
            print(f"[screw] t={_DEBUG['steps'] / 20:5.1f}s nut={np.round((n[:3] - origin) * 1000).astype(int).tolist()} "
                  f"left TCP-nut={np.round((tl - (n[:3] - origin)) * 1000).astype(int).tolist()} "
                  f"right TCP-head={np.round((tr - (s[:3] - origin)) * 1000).astype(int).tolist()} "
                  f"axis={np.round(g['a'], 2).tolist()} x={g['x'] * 1000:6.1f}mm radial={g['radial'] * 1000:4.1f}mm "
                  f"tilt={math.degrees(math.acos(min(1.0, abs(g['cos'])))):4.1f} pre={math.degrees(_EP['pre']):6.1f}deg "
                  f"caught={_EP['caught']} turns={_EP['turns']:5.2f} through={st['through']} "
                  f"play={st['play'] * 1000:5.1f}mm let_go={st['let_go']} score={_points(st)}", file=sys.stderr,
                  flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------


def _status_from(s_pos, s_rot, n_pos, n_rot) -> dict:
    g = _geometry(s_pos, s_rot, n_pos, n_rot)
    a, head = g["a"], np.asarray(s_pos, float)
    tip = head + SHANK_L * a
    through, far_side = False, False
    if abs(a[1]) > 0.5:
        cross = head + (-head[1] / a[1]) * a  # where the axis crosses the plate's middle
        side = -1.0 if head[1] < 0 else 1.0
        through = (abs(cross[0] - HOLE_X) <= HOLE_TOL and abs(cross[2] - HOLE_Z) <= HOLE_TOL
                   and side * head[1] >= PLATE_T / 2 - 0.0005 and -side * tip[1] >= PLATE_T / 2)
        far_side = -side * n_pos[1] >= PLATE_T / 2
    play = g["x"] - (PLATE_T + NUT_T / 2)
    head_mid = head - (HEAD_H / 2) * a
    tcps = _EP["tcps"]
    let_go = tcps is None or all(min(np.linalg.norm(t - head_mid), np.linalg.norm(t - n_pos)) >= LET_GO for t in tcps)
    return {"through": through, "fastened": bool(_EP["caught"]) and far_side and through, "turns": _EP["turns"],
            "play": play, "let_go": let_go}


def _status(objects) -> dict:
    s, n = objects["screw"], objects["nut"]
    return _status_from(s.pos, rotation(s.quat_xyzw), n.pos, rotation(n.quat_xyzw))


def _points(st) -> float:
    if st["fastened"] and st["turns"] >= TURNS_FULL and st["play"] <= PLAY_MAX and st["let_go"]:
        return 1.0
    if st["fastened"] and st["turns"] >= TURNS_HALF:
        return 0.5
    return 0.25 if st["through"] else 0.0


def score(objects) -> float:
    """1.0 with the screw through the plate's hole and turned at least 1.5 turns into the nut on the far side, head and
    nut snug against the plate (no more than 2 mm of play), both grippers let go; 0.5 with it through and caught in
    the nut, turned at least half a turn; 0.25 with the screw through the hole; else 0.0."""
    return _points(_status(objects))


def success(objects) -> bool:
    """Full score with the screw and the nut still."""
    return _points(_status(objects)) >= 1.0 and objects["screw"].resting() and objects["nut"].resting()


# -- scripted demo --------------------------------------------------------------------------------------------------
# Both hands come straight down, fingers closing front and back: the left one round the middle of the nut, the right
# one on the screw's head. Each lifts its part and turns its gripper to point at the plate: the nut is now upright, its bore
# along the hole's axis, and the screw level, pointing at the hole. The left hand presses the nut flat on the plate
# over the hole; the right hand pushes the screw through the hole into the nut and turns it in strokes of half a turn
# (the wrist turns only so far): turn, let go, turn back open, take it again. Each turn draws the screw in by the pitch,
# and the right hand follows it in. Then both let go and back away.


def _rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


DOWN_L, DOWN_R = grip_rotation(-math.pi / 2), grip_rotation(math.pi / 2)  # straight down, fingers closing along x
SIDE_L = _rx(-math.pi / 2) @ DOWN_L  # pointing -y, at the plate's left face
SIDE_R = _rx(math.pi / 2) @ DOWN_R  # pointing +y, at its right face


def _side_r(roll: float) -> np.ndarray:
    """The right hand pointing at the plate, turned ``roll`` about its pointing (+y): turning the screw it holds."""
    return SIDE_R @ _rz(roll)


NUT_IN_HAND = 0.0  # the nut's centre at the TCP: gripped round its middle, its end face 2 mm past the fingertips
HEAD_IN_HAND = 0.015  # the screw's head underside this far past the TCP (the 6.4 mm head sits 8.6..15 mm past it)
L_PICK = np.array(NUT.pos) + [0.0, 0.0, NUT_IN_HAND]
R_PICK = np.array(SCREW.pos) + [0.0, 0.0, HEAD_IN_HAND]
L_HOLD = np.array([HOLE_X, PLATE_T / 2 + NUT_T / 2 + NUT_IN_HAND, HOLE_Z])  # the nut flat on the plate's left face
PRESS = 0.002  # the left hand aims this far into the plate, to keep the nut flat on it
L_FIX = np.array([0.0, 0.0, 0.0])  # where the left hand misses by, holding the nut (sag): aim the other way
R_FIX = np.array([0.0, 0.0, 0.0])  # ... and the right hand, holding the screw
ROLL0, STROKE, STROKES = -math.pi / 2, math.pi, 6
GAP_IN = 0.009  # pushed in, the head's underside stands this far off the plate (the tip 3 mm into the nut)
OFF, ABOVE, BACK = 0.012, 0.05, 0.05
LEAD_S, LEAD_STEPS = 3.0, 15
PICK = [1.2, 0.8, 0.3, 1.0]  # down, grip, settle, lift
CARRY_L, CARRY_R, PRESS_S, INSERT_S = 2.5, 2.5, 0.8, 1.5
TURN_S, RELEASE_S, BACK_S, REGRIP_S = 2.0, 0.5, 1.0, 0.6


def _r_tcp(gap: float) -> np.ndarray:
    """The right TCP with the head's underside ``gap`` off the plate's right face."""
    return np.array([HOLE_X, -(PLATE_T / 2 + gap) - HEAD_IN_HAND, HOLE_Z]) + R_FIX


def _gap_after(turned: float) -> float:
    """Where the head should be once the screw has turned ``turned`` (rad) since it went in (catching at a quarter)."""
    return max(GAP_IN - PITCH * max(0.0, turned - CATCH) / (2 * math.pi), 0.0005)


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


def _turning() -> tuple[list, float]:
    """The right hand's strokes, from the screw pushed in: (waypoints, seconds)."""
    out, turned = [], 0.0
    for k in range(STROKES):
        for j in range(1, 5):  # a quarter of the stroke at a time (slerp takes the short way round)
            done = turned + STROKE * j / 4
            out.append(_w(TURN_S / 4, _r_tcp(_gap_after(done)), FINGER_CLOSED, f"turn the screw ({k + 1})",
                          _side_r(ROLL0 + STROKE * j / 4)))
        turned += STROKE
        at = _r_tcp(_gap_after(turned))
        out.append(_w(RELEASE_S, at, FINGER_OPEN, "let go", _side_r(ROLL0 + STROKE)))
        if k < STROKES - 1:
            for j in range(1, 5):
                out.append(_w(BACK_S / 4, at, FINGER_OPEN, "turn back, open", _side_r(ROLL0 + STROKE * (1 - j / 4))))
            out.append(_w(REGRIP_S, at, FINGER_CLOSED, "take the head again", _side_r(ROLL0)))
    return out, sum(w.duration for w in out)


TURNING, TURNING_S = _turning()
L_READY = LEAD_S + sum(PICK) + CARRY_L + PRESS_S  # the left hand holds the nut on the plate
R_AT_PLATE = LEAD_S + sum(PICK) + CARRY_R  # the right hand stands off the hole with the screw


def _left() -> list[Waypoint]:
    hold = L_HOLD - [0.0, PRESS, 0.0] + L_FIX
    out = _lead_in("left", L_PICK + [0.0, 0.0, ABOVE], DOWN_L, "over the nut")
    out += [_w(PICK[0], L_PICK, FINGER_OPEN, "down round its flats", DOWN_L),
            _w(PICK[1], L_PICK, FINGER_CLOSED, "grip the nut", DOWN_L),
            _w(PICK[2], L_PICK, FINGER_CLOSED, "settle", DOWN_L),
            _w(PICK[3], L_PICK + [0.0, 0.0, 0.04], FINGER_CLOSED, "lift it off its peg", DOWN_L),
            _w(CARRY_L, hold + [0.0, OFF + 0.01, 0.0], FINGER_CLOSED, "turn it upright, beside the hole", SIDE_L),
            _w(PRESS_S, hold, FINGER_CLOSED, "hold it flat on the plate over the hole", SIDE_L),
            _w(max(R_AT_PLATE, L_READY) + 0.4 + INSERT_S + TURNING_S - L_READY, hold, FINGER_CLOSED,
               "hold the nut still while the screw turns into it", SIDE_L),
            _w(0.6, hold, FINGER_OPEN, "let go", SIDE_L),
            _w(1.2, hold + [0.0, BACK, 0.0], FINGER_OPEN, "back away", SIDE_L),
            _w(4.0, hold + [0.0, BACK, 0.0], FINGER_OPEN, "wait", SIDE_L)]
    return out


def _right() -> list[Waypoint]:
    stand = _r_tcp(GAP_IN + OFF + 0.012)  # the tip 1.2 cm off the plate
    out = _lead_in("right", R_PICK + [0.0, 0.0, ABOVE], DOWN_R, "over the screw")
    out += [_w(PICK[0], R_PICK, FINGER_OPEN, "down round its head", DOWN_R),
            _w(PICK[1], R_PICK, FINGER_CLOSED, "grip the head", DOWN_R),
            _w(PICK[2], R_PICK, FINGER_CLOSED, "settle", DOWN_R),
            _w(PICK[3], R_PICK + [0.0, 0.0, 0.04], FINGER_CLOSED, "lift it out of the holder", DOWN_R),
            _w(CARRY_R, stand, FINGER_CLOSED, "point it at the hole", _side_r(ROLL0)),
            _w(max(R_AT_PLATE, L_READY) + 0.4 - R_AT_PLATE, stand, FINGER_CLOSED, "wait for the nut", _side_r(ROLL0)),
            _w(INSERT_S, _r_tcp(GAP_IN), FINGER_CLOSED, "push it through the hole into the nut", _side_r(ROLL0))]
    out += TURNING
    last = _r_tcp(_gap_after(STROKES * STROKE))
    out += [_w(0.6, last, FINGER_OPEN, "wait for the left hand", _side_r(ROLL0 + STROKE)),
            _w(1.2, last - [0.0, BACK, 0.0], FINGER_OPEN, "back away", _side_r(ROLL0 + STROKE)),
            _w(4.0, last - [0.0, BACK, 0.0], FINGER_OPEN, "wait", _side_r(ROLL0 + STROKE))]
    return out


def demo():
    return {"left": _left(), "right": _right()}
