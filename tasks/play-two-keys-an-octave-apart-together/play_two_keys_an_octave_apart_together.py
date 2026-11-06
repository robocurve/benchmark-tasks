"""On the station table lies a toy keyboard, fixed, its keys running away from the arms: fifteen white keys, 3 cm wide,
from G to G, each on a sprung hinge at its back end, with the black keys between them. The two C keys, an octave apart
(21 cm), carry red dots.

Task: play the two marked keys together, as one chord: press both down at the same moment (within 0.1 s), hold them
down together for 1 s, and press no other key. 21 cm is far wider than one gripper opens: one hand on each key.

Run it:  inspect-robots run --embodiment yam_arena -E task=play_two_keys_an_octave_apart_together.py \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import Body, BoxShape, CylShape, allow_matrix_grips, as_numpy, hinge, on_reset, on_step, rotation  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the keyboard (fixed): its keys run along x, away from the arms; key i's middle at y = Y0 - i * PITCH ------------
NAMES = "G A B C D E F G A B C D E F G".split()
N_KEYS = len(NAMES)
PITCH = 0.030  # white keys 3 cm apart (29 mm wide, a 1 mm gap)
KEY_W, KEY_L, KEY_T = 0.029, 0.150, 0.014
X_FRONT = 0.235  # the white keys' front edge
X_HINGE = X_FRONT + KEY_L  # their hinges, at their back ends
TARGETS = (3, 10)  # the two C keys, an octave apart
Y0 = (TARGETS[1] - TARGETS[0]) * PITCH / 2 + TARGETS[0] * PITCH  # the targets end up at +-10.5 cm
KEY_TOP = TOP + 0.045  # the white keys' top face, at rest
TRAVEL = 0.008  # a key's front goes down at most this far
BLACK_L = KEY_L * 0.55  # the black keys cover the back 55% of the white keys
DOT_X = 0.020  # the red dots this far back from the keys' front edge
TRAVEL_DEG = math.degrees(math.asin(TRAVEL / KEY_L))
SPRING, PRELOAD_DEG = 0.02, 5.0  # N m per degree, toward PRELOAD_DEG up: ~0.7 N at the front to start, ~1.1 N fully down
CASE_C, WHITE, BLACK, RED, PANEL = (0.15, 0.16, 0.18), (0.96, 0.96, 0.94), (0.08, 0.08, 0.09), (0.85, 0.12, 0.10), \
    (0.25, 0.26, 0.28)

# -- scoring --------------------------------------------------------------------------------------------------------
DOWN = 0.005  # a key is down with its front pressed at least 5 mm
TOGETHER_S = 0.10  # the two marked keys must go down within this of each other
HOLD_S = 1.0
SUCCESS_HOLD_S = 1.0


def _key_y(i: int) -> float:
    return Y0 - i * PITCH


def _black_after(i: int) -> bool:
    """Is there a black key between white key i and i+1? (none after B or E)"""
    return i < N_KEYS - 1 and NAMES[i] not in ("B", "E")


def _case() -> Body:
    """The keyboard's body: a key bed under the keys, a raised back with the black keys and two knobs."""
    y_mid = (_key_y(0) + _key_y(N_KEYS - 1)) / 2
    span = N_KEYS * PITCH + 0.03
    bed_top = KEY_TOP - KEY_T - TRAVEL - 0.002
    x_mid = (X_FRONT - 0.01 + X_HINGE + 0.06) / 2
    shapes = [BoxShape((x_mid - X_HINGE, 0.0, (bed_top - TOP) / 2), (X_HINGE + 0.06 - X_FRONT + 0.01, span, bed_top - TOP),
                       CASE_C),
              BoxShape((0.035, 0.0, (KEY_TOP + 0.012 - TOP) / 2), (0.05, span, KEY_TOP + 0.012 - TOP), PANEL),  # back
              *[BoxShape((0.035, s * (span / 2 - 0.006), (KEY_TOP + 0.012 - TOP) / 2), (0.05, 0.012, KEY_TOP + 0.012 - TOP),
                         CASE_C) for s in (-1.0, 1.0)],
              *[CylShape((0.035, s * 0.08, KEY_TOP + 0.012 - TOP + 0.005), 0.008, 0.010, BLACK) for s in (-1.0, 1.0)]]
    # the side cheeks, left and right of the keys
    for s in (-1.0, 1.0):
        shapes.append(BoxShape((x_mid - X_HINGE, s * (span / 2 - 0.006), (KEY_TOP + 0.004 - TOP) / 2),
                               (X_HINGE + 0.06 - X_FRONT + 0.01, 0.012, KEY_TOP + 0.004 - TOP), CASE_C))
    # the black keys, fixed, over the back of the white keys
    for i in range(N_KEYS):
        if _black_after(i):
            y = (_key_y(i) + _key_y(i + 1)) / 2 - y_mid
            shapes.append(BoxShape((-BLACK_L / 2, y, KEY_TOP + 0.010 - TOP - 0.008), (BLACK_L, 0.015, 0.016), BLACK))
    return Body.build("keyboard", pos=(X_HINGE, y_mid, TOP), mass=2.0, shapes=shapes, kinematic=True,
                      friction=(0.6, 0.5), color=CASE_C)


def _key(i: int) -> Body:
    """White key i. Its frame: origin on its hinge line (the middle of its back end, mid-thickness), -x toward its
    front."""
    shapes = [BoxShape((-KEY_L / 2, 0.0, 0.0), (KEY_L, KEY_W, KEY_T), WHITE)]
    if i in TARGETS:  # a red dot near its front
        shapes.append(CylShape((-KEY_L + DOT_X, 0.0, KEY_T / 2 + 0.0003), 0.007, 0.0008, RED, collide=False))
    return Body.build(f"key_{i:02d}", pos=(X_HINGE, _key_y(i), KEY_TOP - KEY_T / 2), mass=0.03, shapes=shapes,
                      friction=(0.8, 0.7), color=WHITE, ignore=("keyboard",))


KEYBOARD = _case()
KEYS = [_key(i) for i in range(N_KEYS)]
OBJECTS = [KEYBOARD, *KEYS]
KEY_NAMES = [k.name for k in KEYS]
for i, k in enumerate(KEYS):
    hinge(f"pivot_{i:02d}", "keyboard", k.name, anchor=k.pos, axis="Y", limits_deg=(-TRAVEL_DEG, 0.0), spring=SPRING,
          target_deg=PRELOAD_DEG, damping=0.0005)


# -- followed after every control step: which keys are down, since when ------------------------------------------------

def _fresh() -> dict:
    return {"step": 0, "since": [None] * N_KEYS, "wrong": False, "both": 0, "level": 0.0}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("KEYS_DEBUG"))}


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())


def _depth(rot) -> float:
    """How far a key's front is pressed down (m), from its rotation."""
    return KEY_L * float(rot[2, 0])  # pressed, the key turns about +y by a negative angle: its x axis tips up


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    _EP["step"] += 1
    depths = []
    for i, n in enumerate(KEY_NAMES):
        r = rotation(as_numpy(base.scene[n].data.root_link_pose_w)[0][3:7])
        d = _depth(r)
        depths.append(d)
        down = d >= DOWN
        if down and _EP["since"][i] is None:
            _EP["since"][i] = _EP["step"]
        elif not down:
            _EP["since"][i] = None
        if down and i not in TARGETS:
            _EP["wrong"] = True
    a, b = (_EP["since"][t] for t in TARGETS)
    level = 0.0
    if a is not None or b is not None:
        held = [(_EP["step"] - s) for s in (a, b) if s is not None]
        if max(held) >= round(HOLD_S * 20):
            level = 0.25
    if a is not None and b is not None:
        _EP["both"] += 1
        if _EP["both"] >= round(HOLD_S * 20):
            level = 1.0 if abs(a - b) <= round(TOGETHER_S * 20) else 0.5
    else:
        _EP["both"] = 0
    _EP["level"] = max(_EP["level"], level)
    if _EP["wrong"]:  # another key was pressed: 0.25 at most
        _EP["level"] = min(_EP["level"], 0.25)
    if _DEBUG["on"] and _EP["step"] % 5 == 0:
        print(f"[keys] t={_EP['step'] / 20:5.2f}s depth(mm)={[round(d * 1000, 1) for d in depths]} "
              f"since={[_EP['since'][t] for t in TARGETS]} both={_EP['both']} wrong={_EP['wrong']} "
              f"level={_EP['level']}", file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def score(objects) -> float:
    """1.0 once both marked keys went down within 0.1 s of each other and were held down together for 1 s; 0.5 if they
    were held down together for 1 s but went down further apart; 0.25 if one of them was held down for 1 s; at most
    0.25 if any other key was ever pressed down; else 0.0."""
    return float(_EP["level"])


def success(objects) -> bool:
    """The chord has been played (full score)."""
    return _EP["level"] >= 1.0


# -- scripted demo ----------------------------------------------------------------------------------------------------
# Each hand comes over its marked key, fingers closed and closing across the keys (the closed fingertips are narrower
# than a key), a little above it near its front. Both press down together, hold the chord, and rise.

PRESS_X = X_FRONT + DOT_X  # press on the red dot, well in front of the black keys
TIP = 0.013  # the fingertips this far below the TCP
PRESS = 0.012  # aim the fingertips this far below the key's top (the key stops 8 mm down; the arm gives)
GRIP = grip_rotation(0.0)  # straight down, fingers closing along y (across the keys)
LEAD_S, LEAD_STEPS = 3.0, 15


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
        out.append(Waypoint(LEAD_S / LEAD_STEPS, tuple(float(v) for v in p), FINGER_CLOSED, label, r))
    return out


def _w(t, pos, finger, label, rot) -> Waypoint:
    return Waypoint(t, tuple(float(v) for v in pos), finger, label, rot)


def _hand(key: int, side: str) -> list[Waypoint]:
    over = np.array([PRESS_X, _key_y(key), KEY_TOP + TIP + 0.02])
    down = np.array([PRESS_X, _key_y(key), KEY_TOP + TIP - PRESS])
    out = _lead_in(side, over + [0.0, 0.0, 0.03], GRIP, f"over the marked key ({side})")
    out += [_w(1.5, over, FINGER_CLOSED, "just over the key", GRIP),
            _w(0.5, over, FINGER_CLOSED, "ready", GRIP),
            _w(0.4, down, FINGER_CLOSED, "press it down, together with the other hand", GRIP),
            _w(2.0, down, FINGER_CLOSED, "hold the chord", GRIP),
            _w(0.5, over, FINGER_CLOSED, "let the key up", GRIP),
            _w(2.0, over, FINGER_CLOSED, "wait", GRIP)]
    return out


def demo():
    return {"left": _hand(TARGETS[0], "left"), "right": _hand(TARGETS[1], "right")}
