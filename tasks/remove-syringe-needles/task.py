"""On the station table: a syringe stand holding three filled 3 mL Luer-lock syringes, each with a capped 21 G x 1 in
needle, lying side by side with its needle pointing right. The stand is two cradles across the table, one under each
end of the barrels, with a notch for each syringe; between them the barrels are clear all round.

The needles have to come off the syringes (as when the needle used to draw up a dose is taken off before a fresh
one is fitted). A Luer-lock needle is screwed onto the syringe's collar: it comes off only after it has been turned
half a turn anticlockwise (seen from its tip), and then pulls straight off. Turning it is done by its cap, which is
keyed to the hub by ribs. The cap itself is only a friction fit on the hub: pulled before the needle is unscrewed,
it is the cap that comes off, and the needle is left bare.

The thread drags on the needle as it turns (0.008 N m s/rad), and a syringe lying loose in its notches turns with
it, so the syringe has to be held still while its needle is unscrewed.

Run it:    inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score ...
Demo:      python demo.py task.py out_dir
Second layout: set the environment variable SYRINGE_LOT=B before the command (default A).
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import syringekit as K  # noqa: E402
from syringekit import Fixed, Syringe, finger_points, on_reset, on_step, write_joint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]

INSTRUCTION = "Take the needles off the three syringes on the stand."

STAND_COLOR = (0.82, 0.84, 0.86)
LIQUIDS = ((0.80, 0.88, 0.98), (0.98, 0.93, 0.70), (0.85, 0.95, 0.85))

# -- layout ---------------------------------------------------------------------------------------------------------
LOT = os.environ.get("SYRINGE_LOT", "A")
LOTS = {"A": dict(xs=(0.280, 0.330, 0.380), flange_y=0.108),
        "B": dict(xs=(0.270, 0.320, 0.370), flange_y=0.116)}
if LOT not in LOTS:
    raise ValueError(f"SYRINGE_LOT must be one of {sorted(LOTS)}, got {LOT!r}")
XS = LOTS[LOT]["xs"]
FLANGE_Y = LOTS[LOT]["flange_y"]        # the flange's outer face; the needle points toward -y from there
AXIS_Z = 0.105                          # the syringes' axis above the table
SYRINGE_QUAT = (math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5))   # syringe frame: +z (the needle) along world -y
NAMES = tuple(f"syringe_{i + 1}" for i in range(len(XS)))
N = len(NAMES)
SYRINGE_MASS = 0.0072                   # empty 3 mL syringe 5.7 g, plus 1.5 mL of dose

# -- stand: two cradles across the table, one under each end of the barrels, each with a notch per syringe -----------
CHANNEL_W, CHANNEL_WALL, CHANNEL_H = 0.012, 0.003, 0.005
CRADLES = ((0.044, 0.052), (0.004, 0.012))           # along each syringe's axis from its flange: near the collar, the flange
CRADLE_Y = tuple((FLANGE_Y - b, FLANGE_Y - a) for a, b in CRADLES)   # world y spans; the 32 mm between is open
STAND_TOP = AXIS_Z - K.BARREL_R - 0.0003              # the barrels rest on it
STAND_X = (XS[0] - 0.030, XS[-1] + 0.030)
FRICTION = dict(static_friction=0.5, dynamic_friction=0.4, combine="average")

# -- the mechanism ---------------------------------------------------------------------------------------------------
FREE_TURN = math.pi                     # half a turn anticlockwise (seen from the tip) and the needle is free
SPIN_DRAG = 0.008                       # N m s/rad: the thread's drag while the needle is on it
SPIN_FREE = 0.0005                      # N m s/rad: once free
SEATED = 0.002                          # the needle lifted less than this sits on the thread
OFF = 0.010                             # lifted this far, the hub is clear of the collar: the needle is off
LIFT_MAX = 0.025                        # how far a freed needle can be drawn along the syringe's axis
CAP_SPRING = 2000.0                     # N/m: the cap's friction fit holds up to 4 N ...
CAP_POP = 0.002                         # ... and lets go once it has slid this far
EXPOSED = 0.008                         # slid this far, the cap is off the hub and the needle is bare
MOVED = 0.03                            # a syringe whose flange ends this far from its start was knocked off its place


def flange_point(i: int) -> np.ndarray:
    return np.array([XS[i], FLANGE_Y, TOP + AXIS_Z])


def on_axis(i: int, z_local: float) -> np.ndarray:
    """World position (at the start) of the point ``z_local`` along syringe i's axis from its flange."""
    return flange_point(i) + np.array([0.0, -z_local, 0.0])


def _syringe(i: int) -> Syringe:
    p = flange_point(i)
    return Syringe(NAMES[i], pos=tuple(float(v) for v in p), quat=SYRINGE_QUAT, size=(0.022, 0.15, 0.022), mass=SYRINGE_MASS,
                   color=(0.9, 0.9, 0.9), liquid=LIQUIDS[i], spin_damping=SPIN_DRAG, cap_spring=CAP_SPRING)


def _stand() -> list:
    def fixed(name, x0, x1, y0, y1, z0, z1):
        return Fixed(name, pos=((x0 + x1) / 2, (y0 + y1) / 2, TOP + (z0 + z1) / 2),
                     size=(x1 - x0, y1 - y0, z1 - z0), color=STAND_COLOR, **FRICTION)

    parts = []
    for k, ys in enumerate(CRADLE_Y):
        parts.append(fixed(f"cradle_{k + 1}", *STAND_X, *ys, 0.0, STAND_TOP))
        for i, x in enumerate(XS):
            for side in (-1, 1):
                c = x + side * (CHANNEL_W / 2 + CHANNEL_WALL / 2)
                parts.append(fixed(f"notch_{k + 1}_{i + 1}_{'ab'[side > 0]}", c - CHANNEL_WALL / 2,
                                   c + CHANNEL_WALL / 2, *ys, STAND_TOP, STAND_TOP + CHANNEL_H))
    return parts


OBJECTS = [*_stand(), *(_syringe(i) for i in range(N))]


# -- what happened during the trial ---------------------------------------------------------------------------------
_STATE: dict = {}


def _clear() -> None:
    _STATE.update({n: dict(unscrewed=0.0, last_spin=0.0, free=False, off=False, popped=False, exposed=False,
                           dragging=True, knocked=False) for n in NAMES})


_clear()


@on_reset
def _reset(environment) -> None:
    _clear()
    for n in NAMES:
        for j in K.JOINTS:
            write_joint(environment, n, j, position=0.0)
        write_joint(environment, n, "lift", limits=(0.0, 0.0))
        write_joint(environment, n, "spin", damping=SPIN_DRAG)
        write_joint(environment, n, "cap", stiffness=CAP_SPRING, limits=(0.0, 0.05))


@on_step
def _watch(environment) -> None:
    """Runs after every control step: the thread, the needle coming off, and the cap's friction fit."""
    from yam_arena.scene import object_states   # importable only once the simulator has started

    states = object_states(environment, list(NAMES))
    for i, n in enumerate(NAMES):
        s, st = _STATE[n], states[n]
        j = st.joints
        turned = math.remainder(j["spin"] - s["last_spin"], 2 * math.pi)   # anticlockwise from the tip is positive
        s["last_spin"] = j["spin"]
        seated = j["lift"] < SEATED
        if seated and not s["off"]:
            s["unscrewed"] = min(FREE_TURN, max(0.0, s["unscrewed"] + turned))   # clockwise screws it back on
            free = s["unscrewed"] >= FREE_TURN
            if free != s["free"]:
                s["free"] = free
                write_joint(environment, n, "lift", limits=(0.0, LIFT_MAX if free else 0.0))
        dragging = seated and not s["free"]
        if dragging != s["dragging"]:
            s["dragging"] = dragging
            write_joint(environment, n, "spin", damping=SPIN_DRAG if dragging else SPIN_FREE)
        if s["free"] and not s["off"] and j["lift"] >= OFF:
            s["off"] = True                                            # clear of the collar: it stays off
            write_joint(environment, n, "lift", limits=(OFF, LIFT_MAX))
        if not s["popped"] and j["cap"] > CAP_POP:
            s["popped"] = True                                         # the friction fit has let go
            write_joint(environment, n, "cap", stiffness=0.0)
        s["exposed"] = s["exposed"] or j["cap"] >= EXPOSED              # at any time
        s["knocked"] = s["knocked"] or float(np.linalg.norm(np.asarray(st.pos) - flange_point(i))) > MOVED


# -- scoring --------------------------------------------------------------------------------------------------------
def needle_score(name: str) -> float:
    s = _STATE[name]
    if s["knocked"]:
        return 0.0
    if s["off"]:
        return 1.0
    return 0.5 if s["free"] else 0.0


def score(objects) -> float:
    """0 if any needle was ever bared (its cap slid off its hub). Else per syringe: 1 when its needle is off (turned
    free and lifted 1 cm clear of the collar), 0.5 when it is turned free but still on the collar, 0 otherwise or if
    the syringe was knocked off its place; the score is their mean."""
    if any(_STATE[n]["exposed"] for n in NAMES):
        return 0.0
    return sum(needle_score(n) for n in NAMES) / N


def success(objects) -> bool:
    return score(objects) >= 1.0


def roll_deg(state) -> float:
    """How far a syringe has turned about its own axis from its start (its flange's long side starts along world x)."""
    x = state.rotation()[:, 0]
    return math.degrees(math.atan2(float(x[2]), float(x[0])))


def report(objects) -> dict:
    out = {}
    for i, n in enumerate(NAMES):
        s, st = _STATE[n], objects[n]
        out[n] = {"unscrewed_deg": round(math.degrees(s["unscrewed"]), 1), "free": s["free"], "off": s["off"],
                  "cap_mm": round(1000 * st.joints["cap"], 1), "lift_mm": round(1000 * st.joints["lift"], 1),
                  "exposed": s["exposed"], "knocked": s["knocked"],
                  "moved_cm": round(100 * float(np.linalg.norm(np.asarray(st.pos) - flange_point(i))), 2),
                  "roll_deg": round(roll_deg(st), 1)}
    return out


def demo() -> dict:
    """yam_scripted plays straight-down pick-and-place only; the two-handed demo is demo.py. Returning no waypoints
    holds both arms still, which is a check of the scene by itself."""
    return {}
