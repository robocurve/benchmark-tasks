"""On the station table: three small cabinet units side by side, 34 cm in front of the arms, each with a handleless
push-to-open flap door (18 cm wide, 22 cm tall, hinged at its bottom edge). A push-to-open latch works both ways:
pressing an open door shut latches it, and pressing a latched door pops it open again. Some doors, drawn at random,
stand ajar (tilted 6 degrees open, 2.3 cm at the top); the others are latched shut.

The robot must close every door that is open and leave the closed ones alone: pressing a closed door opens it.

Simulation: AJAR=0,2 picks the open doors (0 left, 1 middle, 2 right); unset, one or two are drawn at random.
"""

from __future__ import annotations

import math
import os
import pathlib
import random
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from jointkit import Hinge, on_physics, on_reset, part, rot_z, set_hinge_target  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]
INSTRUCTION = "Close any cabinet door that is open. Do not press the doors that are already closed."

WOOD, DOOR = (0.85, 0.82, 0.76), (0.93, 0.93, 0.93)
X0 = 0.34                         # the doors' outer faces (x)
UY = (0.20, 0.0, -0.20)           # the units' centres (y), left to right
NAMES = ("door_left", "door_middle", "door_right")
W, H, T = 0.18, 0.22, 0.018       # door width, height, thickness
BOT = 0.02                        # door bottom edge above the table (on the unit's plinth)
DEPTH = 0.28

AJAR_ANGLE = -math.radians(6)     # negative = top toward the robot
POP_ANGLE = -math.radians(20)     # where a popped door swings to
PRESS = math.radians(1.2)         # pressed this far past shut, the latch acts
RELEASED = math.radians(0.6)      # and must come back under this before it can act again

_pick = os.environ.get("AJAR")
AJAR = (tuple(int(i) for i in _pick.split(",")) if _pick
        else tuple(sorted(random.SystemRandom().sample(range(3), random.SystemRandom().choice((1, 2))))))


def unit(name, y):
    # origin: the door's hinge line (outer face, bottom edge) at the unit's centre; the cabinet body is behind it
    boxes = [((T + DEPTH / 2, 0.0, (BOT + H + 0.01) / 2 - BOT), (DEPTH, W + 0.02, BOT + H + 0.01), WOOD)]
    door = Hinge(name=f"{name}_hinge", anchor=(0.0, 0.0, 0.0), yaw=math.pi / 2,
                 boxes=(((T / 2, 0.0, H / 2), (T, W, H), DOOR), ((-0.001, 0.0, H - 0.01), (0.002, W * 0.6, 0.004), (0.6, 0.6, 0.6))),
                 lower=POP_ANGLE - 0.05, upper=math.radians(3.0), stiffness=8.0, damping=0.5, mass=0.6)
    return part(name, pos=(X0, y, TOP + BOT), boxes=boxes, mass=25.0, color=WOOD, hinge=door, friction=1.0)


OBJECTS = [unit(n, y) for n, y in zip(NAMES, UY)]

# -- the latches (per physics step) -------------------------------------------------------------------------------
_STATE: dict = {}
_TICK = [0]


def _target(i):
    return 0.0 if _STATE["latched"][i] else (AJAR_ANGLE if not _STATE["popped"][i] else POP_ANGLE)


def _reset_state(environment=None) -> None:
    _STATE.update(latched=[i not in AJAR for i in range(3)], armed=[True] * 3, popped=[False] * 3,
                  ever_popped=[False] * 3)
    if environment is not None:
        for i, n in enumerate(NAMES):
            set_hinge_target(environment, n, _target(i))
        print(f"[cab] ajar doors: {[NAMES[i] for i in AJAR]}", flush=True)


_reset_state()
on_reset(_reset_state)


@on_physics
def _latches(environment) -> None:
    from yam_arena.scene import object_states

    s = object_states(environment, list(NAMES))
    if os.environ.get("CAB_DEBUG"):
        _TICK[0] += 1
        if _TICK[0] % 60 == 0:
            print(f"[cab] t {_TICK[0] / 240:5.2f} angles {[round(math.degrees(s[n].hinge), 2) for n in NAMES]} "
                  f"latched {_STATE['latched']} popped {_STATE['popped']}", flush=True)
    for i, n in enumerate(NAMES):
        a = s[n].hinge
        if a < RELEASED:
            _STATE["armed"][i] = True
        if a > PRESS and _STATE["armed"][i]:
            _STATE["armed"][i] = False
            if _STATE["latched"][i]:              # a latched door pressed: it pops open
                _STATE["latched"][i] = False
                _STATE["popped"][i] = _STATE["ever_popped"][i] = True
            else:                                 # an open door pressed shut: it latches
                _STATE["latched"][i] = True
            set_hinge_target(environment, n, _target(i))


def score(objects) -> float:
    """0 if a door that started closed was ever popped open. Otherwise the share of the open doors that are now
    latched shut: 1.0 when all are (success, held 1 s)."""
    if any(_STATE["ever_popped"][i] for i in range(3) if i not in AJAR):
        return 0.0
    return sum(_STATE["latched"][i] for i in AJAR) / len(AJAR)


def success(objects) -> bool:
    return score(objects) == 1.0


# -- scripted demo: press each open door shut with the closed fingertips; never touch the closed ones -------------
def frame(z, x) -> np.ndarray:
    z = np.asarray(z, float) / np.linalg.norm(z)
    x = np.asarray(x, float) - z * np.dot(x, z)
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


P = math.radians(60)                               # fingers pointing forward and 30 degrees down
PUSH = frame((math.sin(P), 0.0, -math.cos(P)), (0.0, -1.0, 0.0))
Z_PUSH = TOP + BOT + 0.16                          # press 16 cm above the hinge
TIP = 0.013 + 0.012                                # TCP to the closed fingertips, plus their rounding
CLOSED = 0.0


def _press(y):
    tcp_at_face = X0 - TIP * math.sin(P)
    return [(1.5, (tcp_at_face - 0.08, y, Z_PUSH + 0.02), PUSH, CLOSED, "in front of the door"),
            (1.5, (tcp_at_face - 0.03, y, Z_PUSH), PUSH, CLOSED, "up to the door"),
            (1.5, (tcp_at_face + 0.04, y, Z_PUSH), PUSH, CLOSED, "press it shut"),
            (0.6, (tcp_at_face + 0.04, y, Z_PUSH), PUSH, CLOSED, "hold"),
            (1.0, (tcp_at_face - 0.08, y, Z_PUSH + 0.02), PUSH, CLOSED, "let it go")]


def plan(ajar=AJAR) -> dict[str, list]:
    """Each arm closes its own door, in turn (left first, then right): the left arm the left door, the right arm the right one; the
    middle door goes to whichever arm has no door of its own to close (the right arm, after its own, if both do)."""
    left, right = [], []
    home_l, home_r = (0.18, 0.28, TOP + 0.20), (0.18, -0.28, TOP + 0.20)
    if 0 in ajar:
        left += _press(UY[0]) + [(1.5, home_l, PUSH, CLOSED, "away")]
    if 2 in ajar:
        right += _press(UY[2]) + [(1.5, home_r, PUSH, CLOSED, "away")]
    if 1 in ajar:
        if os.environ.get("DEMO_ALL"):  # contrast: the right arm presses the middle door first
            right = _press(UY[1]) + [(1.5, home_r, PUSH, CLOSED, "away")] + right
        elif not left:
            left += _press(UY[1]) + [(1.5, home_l, PUSH, CLOSED, "away")]
        else:
            right += _press(UY[1]) + [(1.5, home_r, PUSH, CLOSED, "away")]
    if left and right and not os.environ.get("DEMO_ALL"):  # take turns: the right arm starts once the left is done
        right = [(sum(w[0] for w in left), home_r, PUSH, CLOSED, "wait for the left arm")] + right
    return {k: v for k, v in (("left", left), ("right", right)) if v}


def demo():
    """Presses only the doors that stand open, each arm its own door, left first, then right (see plan())."""
    from yam_arena.demo import Waypoint

    route = plan((0, 1, 2)) if os.environ.get("DEMO_ALL") else plan()  # DEMO_ALL: press every door, closed ones too
    return {arm: [Waypoint(d, tuple(p), f, label, r) for d, p, r, f, label in wps] for arm, wps in route.items()}
