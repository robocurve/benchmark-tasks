"""On the station table a solved 3x3 puzzle cube sits loose on a display stand (a slim post with a small pad on top).

Task: turn the cube's right layer a quarter turn. The cube is loose and light, so a layer turned on its own just turns
the whole cube: one hand holds the left two layers, the other grips the right layer and turns it 90 degrees until it
clicks into line. Then both hands let go.

Run it:  inspect-robots run --embodiment yam_arena -E task=turn_a_puzzle_cube_layer_a_quarter_turn.py \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import (Body, BoxShape, CylShape, allow_matrix_grips, as_numpy, hinge, on_reset, on_step, rotation,  # noqa: E402
                     set_hinge_drive)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the cube: 6 cm, three layers of 2 cm across y. Two bodies, both with their origin at the cube's centre: "cube" (the
# left two layers, y from -1 to +3 cm) and "layer" (the right one, y from -3 to -1 cm), on a hinge along y -------------
SIDE = 0.06
PITCH = SIDE / 3
STICKER, STICKER_T = 0.017, 0.0008
CX, CY = 0.32, 0.0
# -- the stand: a base disc, a slim post and a small square pad, fixed; the cube sits on the pad --------------------------
BASE_R, BASE_H = 0.035, 0.006
POST_R = 0.006
PAD, PAD_H = 0.016, 0.004
PAD_TOP = TOP + 0.10
CZ0 = PAD_TOP + SIDE / 2 + STICKER_T  # the cube's centre, resting on the pad (the stickers are not solid)
CUBE_MASS, LAYER_MASS = 0.06, 0.03
BLACK = (0.08, 0.08, 0.09)  # the plastic between the stickers
WHITE, YELLOW, GREEN, BLUE = (0.93, 0.93, 0.92), (0.95, 0.82, 0.10), (0.10, 0.60, 0.25), (0.10, 0.30, 0.75)
RED, ORANGE = (0.80, 0.10, 0.10), (0.95, 0.45, 0.08)
FACES = {  # outward axis, sign: colour (top white, front - toward the arms' bases - green, right red)
    (2, 1): WHITE, (2, -1): YELLOW, (0, -1): GREEN, (0, 1): BLUE, (1, -1): RED, (1, 1): ORANGE}

# -- the turn: a spring toward the nearest quarter turn (it clicks into line past 45 degrees) ---------------------------
DETENT_PEAK = 0.25  # N m, at 45 degrees from a quarter
DETENT = DETENT_PEAK / 45.0  # N m per degree
DETENT_DAMPING = 1.0e-4  # N m s per degree

# -- scoring ----------------------------------------------------------------------------------------------------------
QUARTER_TOL = 10.0  # degrees
TURNED = 20.0  # degrees, for partial credit
LET_GO = 0.06  # both TCPs at least this far from the cube's centre
SUCCESS_HOLD_S = 1.0


def _stickers(cols: tuple[float, ...]) -> list[BoxShape]:
    """The stickers on the cubies whose centres (y) are in ``cols``."""
    out = []
    grid = (-PITCH, 0.0, PITCH)
    for (ax, sign), color in FACES.items():
        for a in grid:
            for b in grid:
                c = [0.0, 0.0, 0.0]
                c[ax] = sign * (SIDE / 2 + STICKER_T / 2 - 0.0002)
                others = [i for i in range(3) if i != ax]
                c[others[0]], c[others[1]] = a, b
                if ax != 1 and c[1] not in cols:
                    continue
                if ax == 1 and ((sign < 0) != (min(cols) < -PITCH / 2)):
                    continue
                size = [STICKER, STICKER, STICKER]
                size[ax] = STICKER_T
                out.append(BoxShape(tuple(c), tuple(size), color, collide=False))
    return out


def _cube() -> Body:
    shapes = [BoxShape((0.0, PITCH / 2, 0.0), (SIDE, 2 * PITCH, SIDE), BLACK)]
    shapes += _stickers((0.0, PITCH))
    return Body.build("cube", pos=(CX, CY, CZ0), mass=CUBE_MASS, shapes=shapes, friction=(1.0, 0.9),
                      color=BLACK, com=(0.0, PITCH / 2, 0.0))


def _layer() -> Body:
    shapes = [BoxShape((0.0, -PITCH, 0.0), (SIDE, PITCH, SIDE), BLACK)]
    shapes += _stickers((-PITCH,))
    return Body.build("layer", pos=(CX, CY, CZ0), mass=LAYER_MASS, shapes=shapes, friction=(1.0, 0.9),
                      color=BLACK, com=(0.0, -PITCH, 0.0))


def _stand() -> Body:
    grey = (0.30, 0.31, 0.33)
    post_h = PAD_TOP - PAD_H - TOP - BASE_H
    shapes = [CylShape((0.0, 0.0, BASE_H / 2), BASE_R, BASE_H, grey),
              CylShape((0.0, 0.0, BASE_H + post_h / 2), POST_R, post_h, grey),
              BoxShape((0.0, 0.0, PAD_TOP - TOP - PAD_H / 2), (PAD, PAD, PAD_H), grey)]
    return Body.build("stand", pos=(CX, CY, TOP), mass=0.2, shapes=shapes, kinematic=True, friction=(0.6, 0.5),
                      color=grey)


STAND, CUBE, LAYER = _stand(), _cube(), _layer()
OBJECTS = [STAND, CUBE, LAYER]
hinge("r_turn", "cube", "layer", anchor=(CX, CY, CZ0), axis="Y", spring=DETENT, damping=DETENT_DAMPING,
      target_deg=0.0)


# -- followed after every control step: the detent and the turn --------------------------------------------------------

def _fresh() -> dict:
    return {"best": 0.0, "tcps": None, "target": 0.0}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("CUBE_DEBUG")), "steps": 0}


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


def _turn(c_rot, l_rot) -> float:
    """The right layer's turn against the rest of the cube, degrees in (-180, 180]."""
    r = c_rot.T @ l_rot
    return math.degrees(math.atan2(r[0, 2], r[0, 0]))


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    c = as_numpy(base.scene["cube"].data.root_link_pose_w)[0]
    lay = as_numpy(base.scene["layer"].data.root_link_pose_w)[0]
    c_pos, c_rot, l_rot = c[:3] - origin, rotation(c[3:7]), rotation(lay[3:7])
    turn = _turn(c_rot, l_rot)
    target = 90.0 * round(turn / 90.0)  # the nearest quarter turn: the spring clicks the layer into line there
    if target != _EP["target"]:
        _EP["target"] = target
        set_hinge_drive("r_turn", target_deg=target)
    _EP["best"] = max(_EP["best"], abs(turn))
    _EP["tcps"] = _tcps(base, origin)
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 10 == 0:
            st = _status_from(c_pos, turn)
            tl, tr = _EP["tcps"]
            up = c_rot @ np.array([0.0, 0.0, 1.0])
            print(f"[cube] t={_DEBUG['steps'] / 20:5.1f}s cube={np.round(c_pos * 1000).astype(int).tolist()} "
                  f"tilt={math.degrees(math.acos(max(-1.0, min(1.0, up[2])))):5.1f} turn={turn:7.1f} best={_EP['best']:6.1f} "
                  f"left TCP-cube={np.round((tl - c_pos) * 1000).astype(int).tolist()} "
                  f"right TCP-cube={np.round((tr - c_pos) * 1000).astype(int).tolist()} "
                  f"let_go={st['let_go']} score={_points(st)}", file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def _status_from(c_pos, turn) -> dict:
    let_go = _EP["tcps"] is None or all(float(np.linalg.norm(t - np.asarray(c_pos))) >= LET_GO for t in _EP["tcps"])
    return {"quarter": abs(abs(turn) - 90.0) <= QUARTER_TOL, "let_go": let_go, "turned": _EP["best"] >= TURNED}


def _status(objects) -> dict:
    c, lay = objects["cube"], objects["layer"]
    return _status_from(np.asarray(c.pos, float), _turn(rotation(c.quat_xyzw), rotation(lay.quat_xyzw)))


def _points(st) -> float:
    if st["quarter"] and st["let_go"]:
        return 1.0
    if st["quarter"]:
        return 0.5
    return 0.25 if st["turned"] else 0.0


def score(objects) -> float:
    """1.0 with the right layer a quarter turn (90 +- 10 degrees, either way) from the rest of the cube and the cube let
    go (both TCPs 6 cm from its centre); 0.5 with it a quarter turn; 0.25 if it was turned at least 20 degrees at some
    point; else 0.0."""
    return _points(_status(objects))


def success(objects) -> bool:
    """Full score with the cube still."""
    return score(objects) >= 1.0 and objects["cube"].resting() and objects["layer"].resting()


# -- scripted demo ----------------------------------------------------------------------------------------------------
# Both hands come in level from their own side. The left one, fingers closing on the cube's top and bottom, grips its
# left two layers; the right one, fingers closing on the right layer's front and back, grips that layer and rolls the
# wrist a quarter turn: the layer clicks into line. Then the right hand lets go and backs away, and so does the left.


def _rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


SIDE_L = _ry(math.pi / 2) @ _rx(-math.pi / 2) @ grip_rotation(-math.pi / 2)  # level, pointing -y, fingers closing along z
SIDE_R = _rx(math.pi / 2) @ grip_rotation(math.pi / 2)  # level, pointing +y, fingers closing along x
L_DY = 0.006  # the left TCP from the cube's centre: its fingertips 3 mm short of the right layer
R_DY = -0.026  # the right TCP from the cube's centre: 4 mm inside the right face, its fingertips 3 mm short of the rest
OUT = 0.08
ROLL = math.radians(90.0)
ROLL_STEPS, ROLL_S = 6, 2.4
LEAD_S, LEAD_STEPS = 3.0, 15


def _good_ik(arm: str, pos, rot) -> np.ndarray:
    import itertools

    from yam_arena import ik, station

    best = None
    for seed in itertools.product((-0.5, 0.0, 0.5), (0.8, 1.3), (0.4, 1.0), (-1.4, -0.8, 0.0, 0.8, 1.4), (-1.4, 0.0, 1.4),
                                  (0.0,)):
        q, pe, re = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(rot), np.array(seed), base_pos=station.BASES[arm],
                             iters=200)
        if pe < 0.001 and re < math.radians(0.5):
            room = float(np.minimum(q - ik.LOWER, ik.UPPER - q).min())
            if best is None or room > best[1]:
                best = (q, room)
    if best is None:
        raise ValueError(f"no IK solution for the {arm} grip at {pos}")
    return best[0]


def _lead_in(arm: str, pos, rot, label: str, finger) -> list[Waypoint]:
    from yam_arena import ik, station
    from yam_arena.constants import READY

    q0, q1 = np.array(READY, float), _good_ik(arm, pos, rot)
    out = []
    for k in range(1, LEAD_STEPS + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / LEAD_STEPS, station.BASES[arm])
        out.append(Waypoint(LEAD_S / LEAD_STEPS, tuple(float(v) for v in p), finger, label, r))
    return out


def _w(t, pos, finger, label, rot) -> Waypoint:
    return Waypoint(t, tuple(float(v) for v in pos), finger, label, rot)


L_GRIP = np.array([CX, CY + L_DY, CZ0])
R_GRIP = np.array([CX, CY + R_DY, CZ0])
HELD_AT = LEAD_S + 1.2 + 0.8


def _left(release_at: float, total: float) -> list[Waypoint]:
    out_pos = L_GRIP + [0.0, OUT, 0.0]
    out = _lead_in("left", out_pos, SIDE_L, "level, to the left of the cube", FINGER_OPEN)
    out += [_w(1.2, L_GRIP, FINGER_OPEN, "in round its left two layers", SIDE_L),
            _w(0.8, L_GRIP, FINGER_CLOSED, "grip them", SIDE_L)]
    t = sum(w.duration for w in out)
    out += [_w(max(release_at - t, 0.3), L_GRIP, FINGER_CLOSED, "hold them while the right layer is turned", SIDE_L),
            _w(0.6, L_GRIP, FINGER_OPEN, "let go", SIDE_L),
            _w(1.0, out_pos, FINGER_OPEN, "back away", SIDE_L)]
    t = sum(w.duration for w in out)
    out.append(_w(max(total - t, 0.5), out_pos, FINGER_OPEN,
                  "hold still: the trial ends once the full score has held 1 s", SIDE_L))
    return out


def _right() -> tuple[list[Waypoint], float]:
    out_pos = R_GRIP + [0.0, -OUT, 0.0]
    out = _lead_in("right", out_pos, SIDE_R, "level, to the right of the cube", FINGER_OPEN)
    t = sum(w.duration for w in out)
    out += [_w(max(HELD_AT - t, 0.2) + 0.3, out_pos, FINGER_OPEN, "wait for the left hand to hold the cube", SIDE_R),
            _w(1.2, R_GRIP, FINGER_OPEN, "in round the right layer", SIDE_R),
            _w(0.8, R_GRIP, FINGER_CLOSED, "grip it", SIDE_R)]
    for k in range(1, ROLL_STEPS + 1):
        out.append(_w(ROLL_S / ROLL_STEPS, R_GRIP, FINGER_CLOSED, "turn it a quarter turn",
                      _ry(ROLL * k / ROLL_STEPS) @ SIDE_R))
    turned = _ry(ROLL) @ SIDE_R
    out += [_w(0.6, R_GRIP, FINGER_OPEN, "let go", turned),
            _w(1.0, out_pos, FINGER_OPEN, "back away", turned)]
    return out, sum(w.duration for w in out)


def demo():
    right, t_right = _right()
    total = t_right + 4.0
    right.append(_w(4.0, right[-1].pos, FINGER_OPEN, "wait", right[-1].yaw))
    return {"left": _left(t_right, total), "right": right}
