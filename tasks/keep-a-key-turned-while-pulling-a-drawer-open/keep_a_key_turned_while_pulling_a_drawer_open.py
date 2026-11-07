"""On the station table stands a small cabinet with a locked drawer. A key sticks out of the lock on the drawer front's
left part; a pull handle stands on its right part.

Task: open the drawer. The key's spring turns it back as soon as it is let go, and the lock's bolt catches again unless
the drawer is already on its way out. One hand turns the key a quarter turn clockwise and keeps it turned; the other
pulls the drawer open by its handle at the same time. Turning the key, letting it go and then pulling does not open it.

Run it:  inspect-robots run --embodiment yam_arena -E task=keep_a_key_turned_while_pulling_a_drawer_open.py \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import (Body, BoxShape, CylShape, allow_matrix_grips, as_numpy, glue, hinge, on_reset, on_step,  # noqa: E402
                     rotation, set_glue_on, slider)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the cabinet (fixed) and its drawer, which slides out toward the arms (-x) -----------------------------------------
FACE_X = 0.27  # the cabinet's front face
CAB = (0.20, 0.34, 0.20)  # deep (x), wide (y), tall (z)
FRONT = (0.015, 0.30, 0.12)  # the drawer's front panel: thick, wide, tall
FRONT_Z0 = TOP + 0.05  # its bottom edge
DRAWER_DEEP = 0.17
OUT_MAX = 0.14
DRAWER_MASS = 0.5
DRAG = 4.0  # N s/m: the runners' drag

# -- the lock: a key in the drawer front; a quarter turn clockwise (seen from the front) draws the bolt; its spring turns
# it back when let go. While it is turned less than OPEN_AT and the drawer is shut, the bolt holds the drawer shut ----
KEY_Y, KEY_Z = 0.08, TOP + 0.155
TURN_DEG, OPEN_AT = 90.0, 60.0
KEY_K = 0.0004  # N m per degree, back toward 0
BOW = (0.020, 0.007, 0.024)  # the key's bow (its grip): out from the front (x), thick (y), tall (z)
SHANK_L = 0.010
RELOCK = 0.003  # the drawer back within this of shut, key let back: the bolt catches again
CAUGHT = 0.008  # the drawer this far out: the bolt is past the strike, it no longer catches

# -- the pull: a bar standing up from an arm on the drawer front's right part ------------------------------------------
PULL_Y = -0.08
PULL_OFF = 0.045  # the bar's middle this far out from the front
PULL_W = 0.012
PULL_Z = (TOP + 0.075, TOP + 0.145)  # the bar, bottom to top (its arm at the bottom)

WOOD, DARK, BRASS, STEEL = (0.52, 0.34, 0.20), (0.16, 0.12, 0.10), (0.80, 0.66, 0.30), (0.70, 0.71, 0.74)

# -- scoring ----------------------------------------------------------------------------------------------------------
OPEN_FULL = 0.05
OPEN_SOME = 0.01
KEY_BACK = 10.0  # the key turned back to within this of upright: let go
SUCCESS_HOLD_S = 1.0


def _cabinet() -> Body:
    cx = FACE_X + CAB[0] / 2
    return Body.build("cabinet", pos=(cx, 0.0, TOP), mass=10.0, kinematic=True, color=WOOD,
                      shapes=[BoxShape((0.0, 0.0, CAB[2] / 2), CAB, WOOD),
                              BoxShape((-CAB[0] / 2 - 0.0005, 0.0, FRONT_Z0 - TOP + FRONT[2] / 2),
                                       (0.001, FRONT[1] + 0.006, FRONT[2] + 0.006), DARK, collide=False)])  # the gap


def _drawer() -> Body:
    """Origin at the middle of the front panel's back face (on the cabinet's face when shut)."""
    h = FRONT[2] - 0.02
    shapes = [BoxShape((-FRONT[0] / 2, 0.0, 0.0), FRONT, WOOD),
              BoxShape((DRAWER_DEEP / 2, 0.0, -h / 2 + 0.004), (DRAWER_DEEP, FRONT[1] - 0.02, 0.008), WOOD),  # bottom
              *[BoxShape((DRAWER_DEEP / 2, s * (FRONT[1] / 2 - 0.014), -0.01), (DRAWER_DEEP, 0.008, h - 0.02), WOOD)
                for s in (-1.0, 1.0)],
              BoxShape((DRAWER_DEEP - 0.004, 0.0, -0.01), (0.008, FRONT[1] - 0.02, h - 0.02), WOOD),
              CylShape((-FRONT[0] - 0.0003, KEY_Y, KEY_Z - (FRONT_Z0 + FRONT[2] / 2)), 0.008, 0.0006, BRASS,
                       collide=False, axis="X"),  # the lock's face
              # the pull: an arm out from the front, and the bar standing up from its end
              BoxShape((-FRONT[0] - PULL_OFF / 2, PULL_Y, PULL_Z[0] - (FRONT_Z0 + FRONT[2] / 2) + 0.005),
                       (PULL_OFF + PULL_W / 2, PULL_W, 0.010), STEEL),
              BoxShape((-FRONT[0] - PULL_OFF, PULL_Y, (PULL_Z[0] + PULL_Z[1]) / 2 - (FRONT_Z0 + FRONT[2] / 2)),
                       (PULL_W, PULL_W, PULL_Z[1] - PULL_Z[0]), STEEL)]
    return Body.build("drawer", pos=(FACE_X, 0.0, FRONT_Z0 + FRONT[2] / 2), mass=DRAWER_MASS, shapes=shapes,
                      friction=(1.0, 0.9), color=WOOD, com=(DRAWER_DEEP / 2, 0.0, -0.02))


def _key() -> Body:
    """Origin on the lock's axis at the drawer front's face; the key sticks out toward -x, its bow upright."""
    shapes = [CylShape((-SHANK_L / 2, 0.0, 0.0), 0.003, SHANK_L, BRASS, axis="X"),
              BoxShape((-SHANK_L - BOW[0] / 2, 0.0, 0.0), BOW, BRASS)]
    return Body.build("key", pos=(FACE_X - FRONT[0], KEY_Y, KEY_Z), mass=0.01, shapes=shapes, friction=(1.2, 1.1),
                      color=BRASS)


OBJECTS = [_cabinet(), _drawer(), _key()]
slider("runners", "cabinet", "drawer", anchor=(FACE_X, 0.0, FRONT_Z0 + FRONT[2] / 2), axis="X", limits=(-OUT_MAX, 0.0),
       damping=DRAG)
hinge("key_turn", "drawer", "key", anchor=(FACE_X - FRONT[0], KEY_Y, KEY_Z), axis="X", limits_deg=(0.0, TURN_DEG),
      spring=KEY_K, damping=0.00002, target_deg=-3.0)  # the target just past upright: it rests against its stop
glue("bolt", "cabinet", "drawer", anchor=(FACE_X, 0.0, FRONT_Z0 + FRONT[2] / 2))  # on: the drawer is locked


# -- followed after every control step: the lock ------------------------------------------------------------------------

def _fresh() -> dict:
    return {"locked": True, "ever_unlocked": False, "most_out": 0.0, "tcps": None, "s": {}}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("KEY_DEBUG")), "steps": 0}


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())
    _DEBUG["steps"] = 0
    set_glue_on("bolt", True)


def _tcps(base, origin) -> list[np.ndarray]:
    out = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        i = next(j for j, n in enumerate(robot.body_names) if n == "gripper" or n.endswith("/gripper"))
        g = as_numpy(robot.data.body_link_pose_w)[0][i]
        out.append(g[:3] - origin + rotation(g[3:7]) @ np.array(TCP_OFFSET))
    return out


def _pose(base, origin, name) -> tuple[np.ndarray, np.ndarray]:
    p = as_numpy(base.scene[name].data.root_link_pose_w)[0]
    return p[:3] - origin, rotation(p[3:7])


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    d_pos, d_rot = _pose(base, origin, "drawer")
    k_pos, k_rot = _pose(base, origin, "key")
    out = float(FACE_X - d_pos[0])
    rel = d_rot.T @ k_rot
    turn = math.degrees(math.atan2(float(rel[2, 1]), float(rel[1, 1])))  # clockwise seen from the front
    _EP["tcps"] = _tcps(base, origin)
    _EP["most_out"] = max(_EP["most_out"], out)
    if _EP["locked"] and turn >= OPEN_AT:  # the key draws the bolt
        _EP["locked"] = False
        _EP["ever_unlocked"] = True
        set_glue_on("bolt", False)
    elif not _EP["locked"] and turn < OPEN_AT and out < (RELOCK if _EP["most_out"] >= CAUGHT else CAUGHT):
        # let go of the key before the drawer was out past the strike, or pushed back shut: the bolt catches
        _EP["locked"] = True
        _EP["most_out"] = out
        set_glue_on("bolt", True)
    _EP["s"] = {"out": out, "turn": turn, "key": k_pos}
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 10 == 0:
            tl, tr = _EP["tcps"]
            bow = k_pos + k_rot @ np.array([-SHANK_L - BOW[0] / 2, 0.0, 0.0])
            bar = d_pos + d_rot @ np.array([-FRONT[0] - PULL_OFF, PULL_Y, (PULL_Z[0] + PULL_Z[1]) / 2 - (FRONT_Z0 + FRONT[2] / 2)])
            print(f"[key] t={_DEBUG['steps'] / 20:5.1f}s out={out * 1000:6.1f}mm turn={turn:6.1f} locked={_EP['locked']} "
                  f"ever_unlocked={_EP['ever_unlocked']} most_out={_EP['most_out'] * 1000:6.1f}mm "
                  f"leftTCP-bow={np.round((tl - bow) * 1000).astype(int).tolist()} "
                  f"rightTCP-bar={np.round((tr - bar) * 1000).astype(int).tolist()} score={_points(_status())}",
                  file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def _status() -> dict:
    st = _EP["s"]
    return {"out": st.get("out", 0.0), "turn": st.get("turn", 0.0), "ever_unlocked": _EP["ever_unlocked"]}


def _points(s) -> float:
    if s["out"] >= OPEN_FULL and s["turn"] <= KEY_BACK:
        return 1.0
    if s["out"] >= OPEN_SOME:
        return 0.5
    return 0.25 if s["ever_unlocked"] else 0.0


def score(objects) -> float:
    """1.0 with the drawer open at least 5 cm and the key let go (turned back to within 10 degrees of upright); 0.5
    with the drawer open at least 1 cm; 0.25 with the key turned far enough to draw the bolt at some point; else 0.0."""
    return _points(_status())


def success(objects) -> bool:
    """Full score with the drawer and the key still: the trial ends once it has held 1 s."""
    return score(objects) >= 1.0 and objects["drawer"].resting() and objects["key"].resting()


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The left hand comes level from the front onto the key's bow, fingers closing across it, and rolls its wrist a quarter
# turn clockwise, turning the key, and holds it there. The right hand comes straight down on the pull's bar, fingers
# closing across it, and pulls the drawer straight out 9 cm. The left keeps the key turned, following it out, until the
# drawer is 2.5 cm out (the bolt past the strike), then lets it go (it turns back) and rises out of the drawer's way.
# The right lets go of the pull and rises.


def _rx(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _ry(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


KEY_GRIP = _ry(-math.pi / 2) @ grip_rotation(0.0)  # level, pointing +x (into the drawer front), closing along y
PULL_GRIP = grip_rotation(0.0)  # straight down, fingers closing along y, across the bar
BOW_IN_HAND = 0.004  # the left TCP this far into the bow from its outer end
BAR_IN_HAND = 0.012  # the right TCP this far below the bar's top
PULL_TO = 0.09
PULL_EXTRA = 0.01  # aimed further: the arm gives
KEY_CLEAR_S = 1.6  # the pull pauses this long once the bolt is past the strike, while the key hand lets go and clears
LEAD_S, LEAD_STEPS = 3.0, 15


def _good_ik(arm: str, pos, rot) -> np.ndarray:
    import itertools

    from yam_arena import ik, station

    best = None
    for seed in itertools.product((-0.5, 0.0, 0.5), (0.8, 1.3), (0.4, 1.0), (-0.8, 0.0, 0.8), (-1.4, 0.0, 1.4), (0.0,)):
        q, pe, re = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(rot), np.array(seed),
                             base_pos=station.BASES[arm], iters=200)
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


def _left(release_at: float, total: float) -> tuple[list[Waypoint], float]:
    bow = np.array([FACE_X - FRONT[0] - SHANK_L - BOW[0] + BOW_IN_HAND, KEY_Y, KEY_Z])
    back = bow - [0.05, 0.0, 0.0]
    out = _lead_in("left", back, KEY_GRIP, "level, in front of the key", FINGER_OPEN)
    out += [_w(1.0, bow, FINGER_OPEN, "onto its bow", KEY_GRIP),
            _w(0.6, bow, FINGER_CLOSED, "grip it", KEY_GRIP)]
    for k in range(1, 5):
        out.append(_w(0.4, bow, FINGER_CLOSED, "turn it a quarter turn clockwise", _rx(math.radians(TURN_DEG + 3.0) * k / 4) @ KEY_GRIP))
    turned_at = sum(w.duration for w in out)
    turned = _rx(math.radians(TURN_DEG + 3.0)) @ KEY_GRIP
    step = (PULL_TO + PULL_EXTRA) / 4  # the drawer's first quarter of the pull
    out += [_w(max(release_at - turned_at, 0.3), bow, FINGER_CLOSED, "keep it turned while the drawer is pulled",
               turned),
            _w(0.6, bow - [step, 0.0, 0.0], FINGER_CLOSED, "keep it turned as the drawer starts out", turned),
            _w(0.4, bow - [step + 0.01, 0.0, 0.0], FINGER_OPEN, "let it go, the bolt past the strike: it turns back",
               turned),
            _w(0.5, bow - [step + 0.07, 0.0, 0.0], FINGER_OPEN, "back off the key's bow", turned),
            _w(0.6, bow - [step + 0.07, 0.0, -0.07], FINGER_OPEN,
               "up out of the way, above the drawer's front (which comes out under it)", turned)]
    t = sum(w.duration for w in out)
    out.append(_w(max(total - t, 0.5), bow - [step + 0.07, 0.0, -0.07], FINGER_OPEN,
                  "hold still: the trial ends once the full score has held 1 s", turned))
    return out, turned_at


def _right(start: float) -> tuple[list[Waypoint], float]:
    bar = np.array([FACE_X - FRONT[0] - PULL_OFF, PULL_Y, PULL_Z[1] - BAR_IN_HAND])
    out = _lead_in("right", bar + [0.0, 0.0, 0.04], PULL_GRIP, "over the pull's bar", FINGER_OPEN)
    out += [_w(max(start - LEAD_S, 0.3), bar, FINGER_OPEN, "down round it", PULL_GRIP),
            _w(0.6, bar, FINGER_CLOSED, "grip it", PULL_GRIP)]
    pull0 = sum(w.duration for w in out)
    for k in range(1, 5):
        out.append(_w(0.6, bar - [(PULL_TO + PULL_EXTRA) * k / 4, 0.0, 0.0], FINGER_CLOSED,
                      "pull the drawer straight out", PULL_GRIP))
        if k == 1:  # the bolt is past the strike: wait while the other hand lets go of the key and clears the way
            out.append(_w(KEY_CLEAR_S, out[-1].pos, FINGER_CLOSED, "wait for the key hand to clear", PULL_GRIP))
    end = bar - [PULL_TO + PULL_EXTRA, 0.0, 0.0]
    out += [_w(1.2, end, FINGER_CLOSED, "hold it out", PULL_GRIP),
            _w(0.5, end, FINGER_OPEN, "let go", PULL_GRIP),
            _w(1.0, end + [0.0, 0.0, 0.05], FINGER_OPEN, "rise", PULL_GRIP)]
    return out, pull0


def demo():
    _, turned_at = _left(0.0, 0.0)
    right, pull0 = _right(turned_at)
    t_r = sum(w.duration for w in right)
    total = t_r + 3.0
    right.append(_w(3.0, right[-1].pos, FINGER_OPEN, "hold still", PULL_GRIP))
    left, _ = _left(pull0, total)
    return {"left": left, "right": right}
