"""On the station table lies a utility knife (a box cutter), spine up, along the left-right line, its blade out 3 cm at
its right end; its blade's slider button is on the spine. A tool drawer (an open tray, fixed) stands to the left.

Task: retract the blade (push the slider button back until it clicks home) and put the knife away in the drawer. A
knife put in the drawer with its blade out scores 0, whatever else was done. The button needs a firm push: pushed on a
loose knife, it shoves the whole knife along the table, so one hand holds the knife while the other pushes the button.

Run it:  inspect-robots run --embodiment yam_arena -E task=retract_a_utility_knife_blade_before_putting_it_away.py \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import (Body, BoxShape, allow_matrix_grips, as_numpy, on_reset, on_step, rotation,  # noqa: E402
                     set_slider_drive, slider)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the knife. Its frame: origin at the middle of its underside, +y toward its rear (left), z up; it lies spine up ----
KX, KY = 0.30, -0.010
BODY = (0.018, 0.170, 0.028)  # across (x), long (y), tall (z)
KNIFE_MASS = 0.08
SLOT_FRONT = -0.050  # the button's middle along the knife with the blade out; pushed back (+y) 3 cm, the blade is in
TRAVEL = 0.030
BUTTON = (0.012, 0.010, 0.008)  # on the spine
BLADE_OUT = 0.030  # with the button forward, the blade sticks out this far past the knife's front end
YELLOW, DARK, STEEL, GREY = (0.95, 0.75, 0.10), (0.12, 0.12, 0.13), (0.80, 0.81, 0.83), (0.62, 0.63, 0.66)

# -- the button's detents: a spring toward the nearer of the two, so it clicks out or home and stays -------------------
DETENT_K = 300.0  # N/m: about 4.5 N to push it over the middle

# -- the drawer (fixed): an open tray to the left --------------------------------------------------------------------
DX, DY = 0.30, 0.25
D_IN, D_WALL, D_H = (0.12, 0.23), 0.006, 0.035  # inside (x, y), wall, height

# -- scoring ----------------------------------------------------------------------------------------------------------
HOME = 0.025  # the button at least this far back: the blade is in (less than 5 mm out)
LET_GO = 0.03  # both grippers' TCPs at least this far from the knife, end to end
SUCCESS_HOLD_S = 1.0


def _knife() -> Body:
    shapes = [BoxShape((0.0, 0.0, BODY[2] / 2), BODY, YELLOW),
              BoxShape((0.0, BODY[1] / 2 - 0.035, BODY[2] / 2), (BODY[0] + 0.001, 0.06, BODY[2] * 0.7), DARK,
                       collide=False),  # the rubber grip at the rear
              BoxShape((0.0, SLOT_FRONT + TRAVEL / 2, BODY[2] + 0.0002), (0.004, TRAVEL + BUTTON[1], 0.0004), DARK,
                       collide=False)]  # the slot in the spine
    return Body.build("knife", pos=(KX, KY, TOP + 0.0005), mass=KNIFE_MASS, shapes=shapes, friction=(0.6, 0.5),
                      color=YELLOW)


def _blade() -> Body:
    """The button with the blade carrier under it. Its frame: origin at the button's foot on the spine."""
    front = -BODY[1] / 2 - BLADE_OUT - SLOT_FRONT  # the blade's tip, from the button
    shapes = [BoxShape((0.0, 0.0, BUTTON[2] / 2), BUTTON, DARK),
              BoxShape((0.0, (front + 0.0) / 2, -BODY[2] / 2), (0.0008, -front, 0.012), STEEL, collide=False)]
    return Body.build("blade", pos=(KX, KY + SLOT_FRONT, TOP + 0.0005 + BODY[2]), mass=0.01, shapes=shapes,
                      friction=(1.0, 0.9), color=DARK, ignore=("knife",))


def _drawer() -> Body:
    ox, oy = D_IN[0] + 2 * D_WALL, D_IN[1] + 2 * D_WALL
    shapes = [BoxShape((0.0, 0.0, D_WALL / 2), (ox, oy, D_WALL), GREY)]
    for s in (-1.0, 1.0):
        shapes += [BoxShape((s * (D_IN[0] / 2 + D_WALL / 2), 0.0, D_H / 2), (D_WALL, oy, D_H), GREY),
                   BoxShape((0.0, s * (D_IN[1] / 2 + D_WALL / 2), D_H / 2), (ox, D_WALL, D_H), GREY)]
    return Body.build("drawer", pos=(DX, DY, TOP), mass=2.0, shapes=shapes, kinematic=True, friction=(0.6, 0.5),
                      color=GREY)


DRAWER, KNIFE, BLADE = _drawer(), _knife(), _blade()
OBJECTS = [DRAWER, KNIFE, BLADE]
slider("button", "knife", "blade", anchor=BLADE.pos, axis="Y", limits=(0.0, TRAVEL), stiffness=DETENT_K, damping=3.0,
       target=0.0, max_force=50.0)


# -- followed after every control step: the button's detents, the drawer --------------------------------------------------

def _fresh() -> dict:
    return {"home": False, "exposed_in_drawer": False, "best_back": 0.0, "tcps": None}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("KNIFE_DEBUG")), "steps": 0}


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


def _back(k_pos, k_rot, b_pos) -> float:
    """How far the button is pushed back from its forward detent."""
    return float((k_rot.T @ (np.asarray(b_pos, float) - np.asarray(k_pos, float)))[1] - SLOT_FRONT)


def _in_drawer(k_pos) -> bool:
    return (abs(k_pos[0] - DX) < D_IN[0] / 2 and abs(k_pos[1] - DY) < D_IN[1] / 2 and k_pos[2] < TOP + D_H)


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    kn = as_numpy(base.scene["knife"].data.root_link_pose_w)[0]
    bl = as_numpy(base.scene["blade"].data.root_link_pose_w)[0]
    k_pos, k_rot, b_pos = kn[:3] - origin, rotation(kn[3:7]), bl[:3] - origin
    back = _back(k_pos, k_rot, b_pos)
    _EP["tcps"] = _tcps(base, origin)
    _EP["best_back"] = max(_EP["best_back"], back)
    home = back >= TRAVEL / 2  # past the middle: it clicks home (else it clicks back out)
    if home != _EP["home"]:
        set_slider_drive("button", target=TRAVEL if home else 0.0)
        _EP["home"] = home
    if _in_drawer(k_pos) and back < HOME:  # into the drawer with the blade out: never allowed
        _EP["exposed_in_drawer"] = True
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 10 == 0:
            st = _status_from(k_pos, back, k_rot)
            tl, tr = _EP["tcps"]
            button = b_pos + np.array([0.0, 0.0, BUTTON[2] / 2])
            print(f"[knife] t={_DEBUG['steps'] / 20:5.1f}s knife={np.round(k_pos * 1000).astype(int).tolist()} "
                  f"button_back={back * 1000:5.1f}mm home={_EP['home']} in_drawer={_in_drawer(k_pos)} "
                  f"exposed_in_drawer={_EP['exposed_in_drawer']} let_go={st['let_go']} "
                  f"left TCP-knife={np.round((tl - k_pos) * 1000).astype(int).tolist()} "
                  f"right TCP-button={np.round((tr - button) * 1000).astype(int).tolist()} score={_points(st)}",
                  file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def _to_knife(t, k_pos, k_rot) -> float:
    """How far point t is from the knife (its body's centre line, end to end)."""
    v = k_rot.T @ (np.asarray(t, float) - np.asarray(k_pos, float)) - np.array([0.0, 0.0, BODY[2] / 2])
    v[1] = max(0.0, abs(v[1]) - BODY[1] / 2)
    return float(np.linalg.norm(v))


def _status_from(k_pos, back, k_rot=np.eye(3)) -> dict:
    let_go = _EP["tcps"] is None or all(_to_knife(t, k_pos, k_rot) >= LET_GO for t in _EP["tcps"])
    return {"retracted": back >= HOME, "in_drawer": _in_drawer(k_pos), "let_go": let_go,
            "bad": _EP["exposed_in_drawer"], "tried": _EP["best_back"] >= 0.010}


def _status(objects) -> dict:
    k, b = objects["knife"], objects["blade"]
    k_pos, k_rot = np.asarray(k.pos, float), rotation(k.quat_xyzw)
    return _status_from(k_pos, _back(k_pos, k_rot, b.pos), k_rot)


def _points(st) -> float:
    if st["bad"]:
        return 0.0
    if st["retracted"] and st["in_drawer"] and st["let_go"]:
        return 1.0
    if st["retracted"]:
        return 0.5
    return 0.25 if st["tried"] else 0.0


def score(objects) -> float:
    """0.0 if the knife was ever in the drawer with its blade out. Otherwise 1.0 with the blade in and the knife in the
    drawer, both grippers let go (3 cm clear of it); 0.5 with the blade in; 0.25 if the button was pushed back at least 1 cm at some point;
    else 0.0."""
    return _points(_status(objects))


def success(objects) -> bool:
    """Full score with the knife still."""
    return _points(_status(objects)) >= 1.0 and objects["knife"].resting()


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The left hand comes straight down on the knife's rear end, fingers closing across it, and holds it. The right hand
# comes in over the knife's front, tipped 45 degrees toward the left (its wrist stays clear of the other one), and pushes
# the button back with its fingertips until it clicks home. Then it rises out of the way, and the left hand lifts the
# knife, carries it over the drawer, lowers it in and lets go.


def _rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


HOLD = grip_rotation(-math.pi / 2)  # straight down, fingers closing along x (across the knife)
HOLD_AT = BODY[1] / 2 - 0.025  # the left hand holds the knife this far behind its middle
HOLD_UP = 0.004  # the left TCP this far above the knife's middle height
PUSH_TILT = math.radians(45.0)
PUSH = _rx(PUSH_TILT) @ grip_rotation(0.0)  # pointing down and to the left (+y), fingers closing in that plane
TIP = 0.013  # the fingertips this far past the TCP along its pointing
PUSH_OVER = 0.004  # push this far past the home detent (the slot's end stops it)
LEAD_S, LEAD_STEPS = 3.0, 15
GRIP_S = [1.0, 0.8, 0.3]


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


def _push_tcp(button_y: float, dz: float = 0.0) -> np.ndarray:
    """The right TCP with its fingertips against the button's right face, the button's middle at button_y."""
    point = PUSH @ np.array([0.0, 0.0, 1.0])
    tip = np.array([KX, button_y - BUTTON[1] / 2 - 0.004, TOP + 0.0005 + BODY[2] + BUTTON[2] / 2 + dz])
    return tip - TIP * point


PUSHED_AT = LEAD_S + 2.0 + 1.0 + 1.5  # wait, set the fingertips, push


def _right() -> list[Waypoint]:
    y0 = KY + SLOT_FRONT
    out = _lead_in("right", _push_tcp(y0 - 0.02, 0.03), PUSH, "over the knife's front, tipped toward the left", FINGER_CLOSED)
    out += [_w(2.0, _push_tcp(y0 - 0.02, 0.03), FINGER_CLOSED, "wait for the knife to be held", PUSH),
            _w(1.0, _push_tcp(y0), FINGER_CLOSED, "fingertips against the button", PUSH),
            _w(1.5, _push_tcp(y0 + TRAVEL + PUSH_OVER), FINGER_CLOSED, "push the button back: it clicks home", PUSH),
            _w(1.0, _push_tcp(y0 + TRAVEL, 0.06) - [0.0, 0.04, 0.0], FINGER_CLOSED, "rise out of the way", PUSH)]
    return out


def _left(total: float) -> list[Waypoint]:
    grip = np.array([KX, KY + HOLD_AT, TOP + 0.0005 + BODY[2] / 2 + HOLD_UP])
    lifted = grip + [0.0, 0.0, 0.06]
    over = np.array([DX, DY + HOLD_AT, lifted[2]])
    down = np.array([DX, DY + HOLD_AT, grip[2] + D_WALL + 0.004])
    out = _lead_in("left", grip + [0.0, 0.0, 0.05], HOLD, "over the knife's rear end", FINGER_OPEN)
    out += [_w(GRIP_S[0], grip, FINGER_OPEN, "down round the rear end", HOLD),
            _w(GRIP_S[1], grip, FINGER_CLOSED, "hold the knife", HOLD)]
    t = sum(w.duration for w in out)
    out.append(_w(max(PUSHED_AT + 1.0 - t, 0.3), grip, FINGER_CLOSED, "hold it while the button is pushed back", HOLD))
    out += [_w(1.0, lifted, FINGER_CLOSED, "lift the knife (blade in)", HOLD),
            _w(2.0, over, FINGER_CLOSED, "carry it over the drawer", HOLD),
            _w(1.2, down, FINGER_CLOSED, "lower it in", HOLD),
            _w(0.5, down, FINGER_OPEN, "let go", HOLD),
            _w(1.0, down + [0.0, 0.0, 0.06], FINGER_OPEN, "rise", HOLD)]
    t = sum(w.duration for w in out)
    out.append(_w(max(total - t, 2.0), down + [0.0, 0.0, 0.06], FINGER_OPEN, "wait", HOLD))
    return out


def demo():
    right = _right()
    left = _left(0.0)
    total = sum(w.duration for w in left)
    right.append(_w(max(total - sum(w.duration for w in right), 0.5), right[-1].pos, FINGER_CLOSED, "wait", right[-1].yaw))
    return {"left": left, "right": right}
