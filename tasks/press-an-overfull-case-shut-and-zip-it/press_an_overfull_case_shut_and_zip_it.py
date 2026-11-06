"""On the station table lies an overfull carry case, fixed: its lid, hinged along its left side, is pushed up about 6
degrees by what is packed inside. The case closes with a zip along a ledge on its right side, running front to back;
the zip's slider, with a tall pull ring, waits at the front end.

Task: press the lid shut and pull the zip from the front end all the way to the back. A zip only runs where its two
sides meet: with the lid up, the slider jams. One hand holds the lid down while the other pulls the zip; once about a
third of it is zipped, the zip holds the lid shut by itself.

Run it:  inspect-robots run --embodiment yam_arena -E task=press_an_overfull_case_shut_and_zip_it.py \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import (Body, BoxShape, allow_matrix_grips, as_numpy, hinge, on_reset, on_step, rotation,  # noqa: E402
                     set_hinge_drive, set_slider_drive, slider)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the case (fixed): its body, the zip's ledge along its right side ----------------------------------------------------
CX, CY = 0.28, 0.005  # its middle
CASE_L, CASE_W, BODY_H = 0.26, 0.30, 0.060  # along x, along y, the body's height
Y_RIGHT, Y_LEFT = CY - CASE_W / 2, CY + CASE_W / 2
BODY_TOP = TOP + BODY_H
LEDGE = 0.060  # the zip runs along a ledge this wide on the right side; the lid closes beside it
ZIP_Y = Y_RIGHT + 0.012  # the zip, near the ledge's outer edge: 4.8 cm from the lid (a hand on the pull ring cannot
# reach the lid; the fingers reach 3.4 cm to the side)
ZIP_X0, ZIP_LEN = CX - CASE_L / 2 + 0.02, CASE_L - 0.04  # the zip runs from its front end (where the slider starts) back
FABRIC, TRIM, TEETH, METAL, RED = (0.14, 0.22, 0.40), (0.08, 0.10, 0.16), (0.10, 0.10, 0.10), (0.75, 0.76, 0.78), \
    (0.80, 0.12, 0.10)

# -- the lid: hinged along the case's left top edge; its frame: origin on the hinge line, middle of the case -----------
LID_W, LID_T = CASE_W - LEDGE, 0.020  # across (y), thick
LID_MASS = 0.4
SPRING, BULGE_DEG = 0.10, 12.0  # N m per degree toward BULGE_DEG up: it rests ~7 degrees up
OPEN_MAX = 25.0
SHUT_GAP = 0.003  # the zip runs only where the lid's front edge is within this of shut

# -- the zip's slider: on a slide along the ledge; its frame: on the ledge at the start, the pull ring standing up ----
SLIDER = (0.020, 0.014, 0.008)  # along x, along y, tall
RING = (0.018, 0.008, 0.035)  # the pull ring, standing up (gripped across x, pulled along x)
HOLD_FRACTION = 0.30  # zipped this far, the zip holds the lid shut by itself
JAM_K, RUN_DAMPING = 2.0e4, 8.0  # N/m holding a jammed slider where it is; N s/m running

# -- scoring ----------------------------------------------------------------------------------------------------------
DONE = 0.95  # the zip pulled at least this far along: closed
SUCCESS_HOLD_S = 1.0


def _case() -> Body:
    shapes = [BoxShape((0.0, 0.0, BODY_H / 2), (CASE_L, CASE_W, BODY_H), FABRIC),
              BoxShape((0.0, ZIP_Y - CY, BODY_H + 0.001), (ZIP_LEN + 0.02, 0.006, 0.002), TEETH, collide=False),
              BoxShape((-CASE_L / 2 - 0.002, 0.0, BODY_H * 0.5), (0.004, CASE_W - 0.04, 0.012), TRIM, collide=False)]
    return Body.build("case", pos=(CX, CY, TOP), mass=5.0, shapes=shapes, kinematic=True, friction=(0.8, 0.7),
                      color=FABRIC)


def _lid() -> Body:
    """The lid; its origin on the hinge (the left top edge), so it can start turned up."""
    shapes = [BoxShape((0.0, -LID_W / 2, LID_T / 2), (CASE_L, LID_W, LID_T), FABRIC),
              BoxShape((0.0, -LID_W + 0.004, LID_T / 2), (CASE_L, 0.008, LID_T + 0.002), TRIM)]  # its free edge
    return Body.build("lid", pos=(CX, Y_LEFT, BODY_TOP), mass=LID_MASS, shapes=shapes, friction=(0.9, 0.8),
                      color=FABRIC, com=(0.0, -LID_W / 2, LID_T / 2), start_xyzw=_quat_x(-math.radians(7.0)),
                      ignore=("case",))


def _quat_x(a: float) -> tuple[float, float, float, float]:
    return (math.sin(a / 2), 0.0, 0.0, math.cos(a / 2))


def _zip() -> Body:
    shapes = [BoxShape((0.0, 0.0, SLIDER[2] / 2), SLIDER, METAL),
              BoxShape((0.0, 0.0, SLIDER[2] + RING[2] / 2), RING, RED)]
    return Body.build("zip", pos=(ZIP_X0, ZIP_Y, BODY_TOP), mass=0.02, shapes=shapes, friction=(1.2, 1.0), color=METAL,
                      ignore=("case", "lid"))


CASE, LID, ZIP = _case(), _lid(), _zip()
OBJECTS = [CASE, LID, ZIP]
hinge("lid_hinge", "case", "lid", anchor=LID.pos, axis="X", limits_deg=(-OPEN_MAX, 0.0), spring=SPRING,
      target_deg=-BULGE_DEG, damping=0.02)  # turned about -x, the lid's free (right) edge goes up
slider("zip_track", "case", "zip", anchor=ZIP.pos, axis="X", limits=(0.0, ZIP_LEN), stiffness=JAM_K, damping=RUN_DAMPING,
       target=0.0, max_force=400.0)


# -- followed after every control step: the lid, the zip ------------------------------------------------------------------

def _fresh() -> dict:
    return {"jammed": True, "held": False, "best": 0.0, "tcps": None}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("ZIP_DEBUG")), "steps": 0}


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


def _lid_up(rot) -> float:
    """How far the lid is up (degrees): its free edge (its -y) tipped up."""
    return math.degrees(math.asin(max(-1.0, min(1.0, -float(rot[2, 1])))))


def _gap(up_deg: float) -> float:
    return LID_W * math.sin(math.radians(max(up_deg, 0.0)))


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    lid = as_numpy(base.scene["lid"].data.root_link_pose_w)[0]
    zp = as_numpy(base.scene["zip"].data.root_link_pose_w)[0]
    up = _lid_up(rotation(lid[3:7]))
    s = float(zp[0] - origin[0] - ZIP_X0)  # how far the slider has been pulled
    _EP["tcps"] = _tcps(base, origin)
    _EP["best"] = max(_EP["best"], s)
    jam = _gap(up) > SHUT_GAP and not _EP["held"]
    if jam != _EP["jammed"]:  # jammed: held where it is; running: free, against a little drag
        set_slider_drive("zip_track", target=s, stiffness=JAM_K if jam else 0.0)
        _EP["jammed"] = jam
    held = s >= HOLD_FRACTION * ZIP_LEN
    if held != _EP["held"]:  # zipped a third of the way: the zip holds the lid shut
        set_hinge_drive("lid_hinge", target_deg=0.0 if held else -BULGE_DEG, spring=5.0 if held else SPRING)
        _EP["held"] = held
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 10 == 0:
            tl, tr = _EP["tcps"]
            ring = zp[:3] - origin + np.array([0.0, 0.0, SLIDER[2] + RING[2] / 2])
            print(f"[zip] t={_DEBUG['steps'] / 20:5.1f}s lid_up={up:5.2f}deg gap={_gap(up) * 1000:5.1f}mm "
                  f"zip={s * 1000:6.1f}mm ({s / ZIP_LEN * 100:4.0f}%) jammed={_EP['jammed']} held={_EP['held']} "
                  f"left TCP={np.round(tl * 1000).astype(int).tolist()} right TCP-ring={np.round((tr - ring) * 1000).astype(int).tolist()} "
                  f"score={_points(s, up)}", file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def _points(s: float, up: float) -> float:
    if s >= DONE * ZIP_LEN and up <= 1.0:
        return 1.0
    if s >= 0.5 * ZIP_LEN:
        return 0.5
    return 0.25 if s >= 0.1 * ZIP_LEN else 0.0


def _now(objects) -> tuple[float, float]:
    return float(objects["zip"].pos[0] - ZIP_X0), _lid_up(rotation(objects["lid"].quat_xyzw))


def score(objects) -> float:
    """1.0 with the zip pulled all the way (at least 95%) and the lid shut; 0.5 with it pulled at least half way; 0.25
    with it pulled at least a tenth of the way; else 0.0."""
    return _points(*_now(objects))


def success(objects) -> bool:
    """Full score with the zip still."""
    return score(objects) >= 1.0 and objects["zip"].resting()


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The left hand comes straight down on the lid, 17 cm left of the zip, fingers closed, and presses it shut. The right
# hand comes straight down on the zip's pull ring at the front end, grips it and pulls it along to the back end. Once
# the zip is a good way along (it holds the lid shut by then), the left hand rises and moves out of the way.

GRIP = grip_rotation(0.0)  # straight down, fingers closing along y (the left hand, pressing)
PULL_GRIP = grip_rotation(math.pi / 2)  # straight down, fingers closing along x (across the ring; the pull is along x)
PRESS_AT = (CX, ZIP_Y + 0.17)  # where the left hand presses the lid
PRESS = 0.012  # aim the fingertips this far below the shut lid's top
RING_IN_HAND = 0.006  # the right TCP this far below the ring's top
LEAD_S, LEAD_STEPS = 3.0, 15
PRESS_S, PULL_S = 2.0, 5.0
LEFT_OFF_AT = 0.45  # the left hand lets go once the zip is this far along (it holds the lid from a third)


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


def _ring_tcp(s: float) -> np.ndarray:
    return np.array([ZIP_X0 + s, ZIP_Y, BODY_TOP + SLIDER[2] + RING[2] - RING_IN_HAND])


PRESSED_AT = LEAD_S + 1.0 + PRESS_S


def _right() -> tuple[list[Waypoint], float]:
    start = _ring_tcp(0.0)
    out = _lead_in("right", start + [0.0, 0.0, 0.05], PULL_GRIP, "over the zip's pull ring", FINGER_OPEN)
    t = LEAD_S
    out.append(_w(max(PRESSED_AT - t, 0.2), start + [0.0, 0.0, 0.05], FINGER_OPEN, "wait for the lid to be pressed shut",
                  PULL_GRIP))
    out += [_w(1.0, start, FINGER_OPEN, "down round the pull ring", PULL_GRIP),
            _w(0.6, start, FINGER_CLOSED, "grip it", PULL_GRIP)]
    pull_from = sum(w.duration for w in out)
    for k in range(1, 9):  # pull it along, an eighth at a time
        out.append(_w(PULL_S / 8, _ring_tcp(ZIP_LEN * k / 8 + 0.004), FINGER_CLOSED, "pull the zip along to the back end",
                      PULL_GRIP))
    end = _ring_tcp(ZIP_LEN)
    out += [_w(0.5, end, FINGER_OPEN, "let go", PULL_GRIP),
            _w(1.0, end + [0.0, 0.0, 0.05], FINGER_OPEN, "rise", PULL_GRIP)]
    return out, pull_from + PULL_S * LEFT_OFF_AT


def _left(release_at: float, total: float) -> list[Waypoint]:
    lid_top = BODY_TOP + LID_T
    press = np.array([PRESS_AT[0], PRESS_AT[1], lid_top + 0.013 - PRESS])
    up = (Y_LEFT - PRESS_AT[1]) * math.sin(math.radians(7.0)) + 0.01  # over the lid as it stands up
    out = _lead_in("left", press + [0.0, 0.0, up + 0.02], GRIP, "over the lid, left of the zip", FINGER_CLOSED)
    out += [_w(1.0, press + [0.0, 0.0, up], FINGER_CLOSED, "onto the lid", GRIP),
            _w(PRESS_S, press, FINGER_CLOSED, "press it shut", GRIP)]
    t = sum(w.duration for w in out)
    out.append(_w(max(release_at - t, 0.3), press, FINGER_CLOSED, "hold it shut while the zip is pulled", GRIP))
    away = press + [-0.08, 0.08, 0.08]
    out.append(_w(1.2, away, FINGER_CLOSED, "the zip holds it now: rise and move out of the way", GRIP))
    t = sum(w.duration for w in out)
    out.append(_w(max(total - t, 0.5), away, FINGER_CLOSED, "wait", GRIP))
    return out


def demo():
    right, release_at = _right()
    total = sum(w.duration for w in right) + 3.0
    right.append(_w(3.0, right[-1].pos, FINGER_OPEN, "wait", right[-1].yaw))
    return {"left": _left(release_at, total), "right": right}
