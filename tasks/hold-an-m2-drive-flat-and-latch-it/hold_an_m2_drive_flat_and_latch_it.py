"""On the station table lies a motherboard, fixed. An M.2 drive (22 x 110 mm) sits in its socket, pointing right, but
sprung up 20 degrees, as a drive does once its edge is pushed into the socket. At its far end is a standoff
with a tool-less quarter-turn latch: a knob on a short stem, with an arm that turns over the drive's end.

Task: hold the drive down flat in its socket with one hand while the other turns the latch's knob a quarter turn
counterclockwise, so its arm comes over the drive's end and holds it down; then let go with both hands. The latch's arm
turns at a fixed height just over the flat drive: it cannot press the drive down, only hold it once it is down, and
let go, the drive springs back up at once. So one hand has to hold it flat while the other turns the latch.

Run it:  inspect-robots run --embodiment yam_arena -E task=hold_an_m2_drive_flat_and_latch_it.py \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import (Body, BoxShape, CylShape, allow_matrix_grips, as_numpy, hinge, on_reset, on_step,  # noqa: E402
                     rotation)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the motherboard (fixed): origin on the table under the middle of its board ---------------------------------------
MB_X, MB_Y = 0.28, 0.035
MB_L, MB_W, MB_T = 0.18, 0.22, 0.012  # along x, along y, thick
PCB_TOP = TOP + MB_T
SOCKET_H = 0.0035

# -- the drive: an M.2 22110 card. Its frame: origin on the hinge line in the socket (the card's mid-plane), the card
# along -y (pointing right), z up (when flat) ---------------------------------------------------------------------------
CX = 0.28  # the card's centre line
HY = 0.09  # the hinge line's y (the socket)
CARD_L, CARD_W, CARD_T = 0.110, 0.022, 0.0015
NOTCH_HALF, NOTCH_DEPTH = 0.003, 0.005  # the slot at the card's far end the latch's hub goes through
Z_HINGE = PCB_TOP + 0.0025
CARD_TOP = Z_HINGE + CARD_T / 2  # with the card flat
CARD_MASS = 0.008
SPRUNG_DEG = 20.0  # it rests sprung up this far
SPRING = 0.003  # N m per degree, toward -30 degrees: 0.03 N m holds it at 20 up, 0.09 N m pressing it flat
POST_Y = HY - CARD_L + NOTCH_DEPTH / 2  # the standoff and the latch's axis

# -- the latch: origin on its axis at the board's top. Its arm points -y (off the card's end) at the start and turns
# counterclockwise (seen from above) a quarter turn to point +x (away from the arms), over the card's end -------------
HUB_R = 0.0018
ARM_L, ARM_W, ARM_T = 0.013, 0.006, 0.005
ARM_GAP = 0.001  # the arm's underside this far over the flat card's top
ARM_Z0 = CARD_TOP + ARM_GAP - PCB_TOP  # the arm's underside, in the latch's frame
STEM_R, STEM_H = 0.003, 0.040  # a tall stem: the hand on the knob stays clear of the drive and the other hand
KNOB_R, KNOB_H = 0.007, 0.012
KNOB_Z0 = ARM_Z0 + ARM_T + STEM_H  # the knob's underside, in the latch's frame
LATCH_MASS = 0.010
LATCH_DAMPING = 0.0003  # N m per degree/s: a stiff latch that stays where it is turned

PCB_C, DARK, GOLD, BRASS, CHIP, LABEL, SILVER, SOCKET_C = (0.10, 0.32, 0.18), (0.08, 0.08, 0.09), (0.85, 0.68, 0.25), \
    (0.75, 0.62, 0.30), (0.12, 0.12, 0.13), (0.92, 0.92, 0.90), (0.72, 0.73, 0.75), (0.15, 0.15, 0.16)

# -- scoring --------------------------------------------------------------------------------------------------------
FLAT_DEG = 2.5  # the card within this of flat
LATCHED_DEG = 70.0  # the latch turned at least this far (the arm over the card's end)
HELD_FLAT_S = 1.0
CLEAR = 0.03  # let go: each gripper's TCP at least this far from the card and the knob
SUCCESS_HOLD_S = 1.0


def _quat_x(a: float) -> tuple[float, float, float, float]:
    return (math.sin(a / 2), 0.0, 0.0, math.cos(a / 2))


def _motherboard() -> Body:
    shapes = [BoxShape((0.0, 0.0, MB_T / 2), (MB_L, MB_W, MB_T), PCB_C),
              BoxShape((CX - MB_X, HY - MB_Y + 0.001, MB_T + SOCKET_H / 2), (CARD_W + 0.006, 0.008, SOCKET_H), SOCKET_C),
              CylShape((CX - MB_X, POST_Y - MB_Y, MB_T + 0.0008), 0.0035, 0.0016, BRASS),  # the standoff's shoulder
              # decoration, all low (under 1 cm) and away from the drive: a CPU socket, two memory slots, chips
              BoxShape((-0.055, 0.060, MB_T + 0.002), (0.045, 0.045, 0.004), SILVER),
              BoxShape((0.062, 0.0, MB_T + 0.004), (0.007, 0.170, 0.008), DARK),
              BoxShape((0.075, 0.0, MB_T + 0.004), (0.007, 0.170, 0.008), DARK),
              BoxShape((-0.060, -0.060, MB_T + 0.0015), (0.020, 0.020, 0.003), CHIP),
              BoxShape((0.030, -0.085, MB_T + 0.0015), (0.015, 0.015, 0.003), CHIP)]
    return Body.build("motherboard", pos=(MB_X, MB_Y, TOP), mass=1.0, shapes=shapes, kinematic=True,
                      friction=(0.6, 0.5), color=PCB_C)


def _drive() -> Body:
    t = CARD_T
    body_l = CARD_L - NOTCH_DEPTH
    prong_w = CARD_W / 2 - NOTCH_HALF
    shapes = [BoxShape((0.0, -body_l / 2, 0.0), (CARD_W, body_l, t), DARK),
              *[BoxShape((s * (NOTCH_HALF + prong_w / 2), -(body_l + NOTCH_DEPTH / 2), 0.0), (prong_w, NOTCH_DEPTH, t), DARK)
                for s in (-1.0, 1.0)],
              BoxShape((0.0, -0.0035, 0.0), (CARD_W - 0.002, 0.004, t + 0.0002), GOLD, collide=False),  # the contacts
              BoxShape((0.0, -0.045, t / 2 + 0.0006), (0.012, 0.012, 0.0012), CHIP),  # controller
              BoxShape((0.0, -0.068, t / 2 + 0.0006), (0.016, 0.016, 0.0012), CHIP),  # flash
              BoxShape((0.0, -0.088, t / 2 + 0.0006), (0.016, 0.016, 0.0012), CHIP),
              BoxShape((0.0, -0.066, t / 2 + 0.0013), (0.018, 0.060, 0.0002), LABEL, collide=False)]
    return Body.build("drive", pos=(CX, HY, Z_HINGE), mass=CARD_MASS, shapes=shapes, friction=(0.8, 0.7), color=DARK,
                      start_xyzw=_quat_x(-math.radians(SPRUNG_DEG)), ignore=("motherboard",))


def _latch() -> Body:
    z_arm = ARM_Z0 + ARM_T / 2
    shapes = [CylShape((0.0, 0.0, (0.001 + KNOB_Z0) / 2), HUB_R, KNOB_Z0 - 0.001, SILVER),
              BoxShape((0.0, -ARM_L / 2, z_arm), (ARM_W, ARM_L, ARM_T), DARK),
              CylShape((0.0, 0.0, ARM_Z0 + ARM_T + STEM_H / 2), STEM_R, STEM_H, DARK),
              CylShape((0.0, 0.0, KNOB_Z0 + KNOB_H / 2), KNOB_R, KNOB_H, DARK)]
    return Body.build("latch", pos=(CX, POST_Y, PCB_TOP), mass=LATCH_MASS, shapes=shapes, friction=(1.2, 1.0),
                      color=DARK, ignore=("motherboard",))


MOTHERBOARD = _motherboard()
DRIVE = _drive()
LATCH = _latch()
OBJECTS = [MOTHERBOARD, DRIVE, LATCH]
hinge("slot", "motherboard", "drive", anchor=(CX, HY, Z_HINGE), axis="X", limits_deg=(-SPRUNG_DEG, 0.0),
      spring=SPRING, target_deg=-30.0, damping=0.0001)
hinge("turn", "motherboard", "latch", anchor=(CX, POST_Y, PCB_TOP), axis="Z", limits_deg=(0.0, 90.0),
      damping=LATCH_DAMPING)


# -- followed after every control step: how long the card has been held flat, where the grippers are ------------------

def _fresh() -> dict:
    return {"tcps": None, "flat_steps": 0, "held_flat": False}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("M2_DEBUG")), "steps": 0}


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


def _card_up(rot) -> float:
    """How far the card is sprung up from flat (degrees): its far end's rise along it."""
    return math.degrees(math.asin(max(-1.0, min(1.0, -float(rot[2, 1])))))


def _latch_turn(rot) -> float:
    """How far the latch is turned counterclockwise from its start (degrees): its arm from -y toward +x."""
    d = rot[:, 1] * -1.0
    return math.degrees(math.atan2(float(d[0]), float(-d[1])))


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    _EP["tcps"] = _tcps(base, origin)
    d = as_numpy(base.scene["drive"].data.root_link_pose_w)[0]
    up = _card_up(rotation(d[3:7]))
    _EP["flat_steps"] = _EP["flat_steps"] + 1 if up <= FLAT_DEG else 0
    if _EP["flat_steps"] >= round(HELD_FLAT_S * 20):
        _EP["held_flat"] = True
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 10 == 0:
            la = as_numpy(base.scene["latch"].data.root_link_pose_w)[0]
            st = _status_from(d[:3] - origin, rotation(d[3:7]), rotation(la[3:7]))
            tl, tr = _EP["tcps"]
            press = np.array([CX, HY - PRESS_S, CARD_TOP]) - CORNER * (_rx(-TILT) @ np.array([0.0, 1.0, 0.0]))
            knob = np.array([CX, POST_Y, PCB_TOP + KNOB_Z0 + KNOB_H / 2])
            print(f"[m2] t={_DEBUG['steps'] / 20:5.1f}s card_up={up:5.2f} latch={st['turn']:5.1f} "
                  f"held_flat={_EP['held_flat']} latched={st['latched']} clear={st['clear']} "
                  f"left TCP-press={np.round((tl - press) * 1000).astype(int).tolist()} "
                  f"right TCP-knob={np.round((tr - knob) * 1000).astype(int).tolist()} score={_points(st)}",
                  file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def _seg_dist(p, a, b) -> float:
    ab = b - a
    s = float(np.clip((p - a) @ ab / (ab @ ab), 0.0, 1.0))
    return float(np.linalg.norm(p - (a + s * ab)))


def _status_from(card_pos, card_rot, latch_rot) -> dict:
    up = _card_up(card_rot)
    turn = _latch_turn(latch_rot)
    latched = up <= FLAT_DEG and turn >= LATCHED_DEG
    clear = True
    tcps = _EP["tcps"]
    if tcps is not None:
        a = np.asarray(card_pos, float)
        b = a + card_rot @ np.array([0.0, -CARD_L, 0.0])
        knob = np.array([CX, POST_Y, PCB_TOP + KNOB_Z0 + KNOB_H / 2])
        for t in tcps:
            clear &= _seg_dist(np.asarray(t, float), a, b) >= CLEAR and float(np.linalg.norm(t - knob)) >= CLEAR
    return {"up": up, "turn": turn, "latched": latched, "clear": clear, "held_flat": _EP["held_flat"]}


def _status(objects) -> dict:
    d, la = objects["drive"], objects["latch"]
    return _status_from(d.pos, rotation(d.quat_xyzw), rotation(la.quat_xyzw))


def _points(st) -> float:
    if st["latched"] and st["clear"]:
        return 1.0
    if st["latched"]:
        return 0.5
    return 0.25 if st["held_flat"] else 0.0


def score(objects) -> float:
    """1.0 with the drive flat (within 2.5 degrees) under the latch's arm (turned at least 70 degrees) and both grippers
    let go (3 cm clear of the drive and the knob); 0.5 with it flat and latched; 0.25 if it was held flat for 1 s at
    some point; else 0.0."""
    return _points(_status(objects))


def success(objects) -> bool:
    """Full score with the drive and the latch still."""
    return _points(_status(objects)) >= 1.0 and objects["drive"].resting() and objects["latch"].resting()


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The left hand comes down on the drive near its socket, fingers closed and closing left-right, tipped 40 degrees toward
# the right (its wrist then stays clear of the other one). Tipped so, the lowest part of its fingers is the left
# finger's back corner, 3.5 cm behind the fingertips and 1.2 cm lower: that presses the drive flat, 4 cm from its
# socket. The right hand comes straight down on the latch's knob, grips it and turns it a quarter turn
# counterclockwise. Then the right hand lets go and rises, and the left hand rises and backs away: the drive springs
# up against the latch's arm.


def _rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


PRESS_S = 0.040  # where along the card the left hand presses (from the hinge line)
TILT = math.radians(40.0)  # the left grip tipped this far from straight down, toward -y
CORNER = 0.035  # the left finger's lowest corner this far from the TCP along the closing axis
PRESS = 0.014  # aim it this far below the flat card's top (the hinge stops the card flat; the arm gives)
PRESS_L = _rx(-TILT) @ grip_rotation(0.0)  # fingers closing along y, tipped toward -y
KNOB_IN_HAND = 0.004  # the right TCP this far above the knob's middle
YAW0 = 0.0  # the right grip's yaw on the knob before the turn
LEAD_S, LEAD_STEPS = 3.0, 15
ABOVE = 0.05


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
        out.append(Waypoint(LEAD_S / LEAD_STEPS, tuple(float(v) for v in p), FINGER_CLOSED if arm == "left" else FINGER_OPEN,
                            label, r))
    return out


def _w(t, pos, finger, label, rot) -> Waypoint:
    return Waypoint(t, tuple(float(v) for v in pos), finger, label, rot)


LEFT_IN = [1.5, 1.0]  # slide in over the drive, press it down
TURN_S, TURN_STEPS = 2.0, 5
PRESSED_AT = LEAD_S + sum(LEFT_IN)


def _right() -> tuple[list[Waypoint], float]:
    knob = np.array([CX, POST_Y, PCB_TOP + KNOB_Z0 + KNOB_H / 2 + KNOB_IN_HAND])
    rot0 = grip_rotation(YAW0)
    out = _lead_in("right", knob + [0.0, 0.0, ABOVE], rot0, "over the latch's knob")
    t = LEAD_S
    out.append(_w(max(PRESSED_AT - t, 0.2) + 0.5, knob + [0.0, 0.0, ABOVE], FINGER_OPEN,
                  "wait for the drive to be held flat (sprung up, its end is over the knob)", rot0))
    out += [_w(1.0, knob, FINGER_OPEN, "down round the knob", rot0),
            _w(0.6, knob, FINGER_CLOSED, "grip it", rot0)]
    for k in range(1, TURN_STEPS + 1):
        out.append(_w(TURN_S / TURN_STEPS, knob, FINGER_CLOSED, "turn the latch a quarter turn counterclockwise",
                      _rz(math.pi / 2 * k / TURN_STEPS) @ rot0))
    end = _rz(math.pi / 2) @ rot0
    out += [_w(0.5, knob, FINGER_OPEN, "let go", end),
            _w(1.0, knob + [0.0, 0.0, ABOVE], FINGER_OPEN, "rise", end)]
    return out, sum(w.duration for w in out)


RIGHT, RIGHT_S = _right()


def _left() -> list[Waypoint]:
    corner = CORNER * (_rx(-TILT) @ np.array([0.0, 1.0, 0.0]))  # from the TCP: +y and down
    press = np.array([CX, HY - PRESS_S, CARD_TOP - PRESS]) - corner
    lifted = np.array([CX, HY - PRESS_S, Z_HINGE + PRESS_S * math.sin(math.radians(SPRUNG_DEG)) + 0.015]) - corner
    out = _lead_in("left", lifted + [0.0, 0.0, 0.03], PRESS_L, "over the drive near its socket, tipped toward the right")
    out += [_w(LEFT_IN[0], lifted, FINGER_CLOSED, "down to just over the sprung-up drive", PRESS_L),
            _w(LEFT_IN[1], press, FINGER_CLOSED, "press it down flat", PRESS_L)]
    t = PRESSED_AT
    out.append(_w(max(RIGHT_S - 1.0 - t, 0.5), press, FINGER_CLOSED, "hold it flat while the latch is turned", PRESS_L))
    out += [_w(1.0, press + [0.0, 0.0, 0.06], FINGER_CLOSED, "rise: the drive springs up against the latch's arm", PRESS_L),
            _w(4.0, press + [0.0, 0.0, 0.06], FINGER_CLOSED, "wait", PRESS_L)]
    return out


def demo():
    left = _left()
    total = sum(w.duration for w in left)
    right = list(RIGHT) + [_w(max(total - RIGHT_S, 0.5), RIGHT[-1].pos, FINGER_OPEN, "wait", RIGHT[-1].yaw)]
    return {"left": left, "right": right}
