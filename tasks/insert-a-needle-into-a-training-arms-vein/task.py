"""On the station table: an adult venipuncture training arm, strapped down palm up, and a phlebotomy cradle in which
a single-use tube holder lies with a 21 gauge, 1 inch multi-sample flashback needle on it, uncapped, bevel up.

The job is the needle stick of a routine blood draw: draw the skin below the site toward the wrist with one hand and
keep it drawn, and with the other put the needle into the vein at the elbow crease, bevel up, at a shallow angle,
stop once the tip is in the vein, and keep it still.

The needle meets resistance: from the moment its point reaches the skin it slides on its own line against a force the
physics solver applies (1 N until the skin is pierced, 0.1 N of shaft friction after, 0.4 N at a wall of the vein).
The skin below the site is a patch on a sprung slide, moved by the friction of a fingertip pressed on it. A vein whose
skin is not drawn when the needle reaches it moves aside: that last is a stated rule, not physics.

The arm's and the cradle's places change with ``VEIN_LOT`` (A or B).

Run it:    inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score ...
Demo:      python demo.py task.py out_dir
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import veinkit as K  # noqa: E402
from veinkit import Cradle, Holder, SkinPatch, Stick, TrainArm, on_reset, on_step  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]

INSTRUCTION = ("Put the needle into the training arm's vein at the elbow crease for a blood draw: with your other hand "
               "draw the skin below the site toward the wrist and keep it drawn, go in bevel up at a shallow angle, "
               "and stop when the needle is in the vein. Keep it still there.")

# -- layout (m; x away from the robot, y left, heights above the table) ---------------------------------------------------
LOT = os.environ.get("VEIN_LOT", "A")
SITE = np.array({"A": (0.250, 0.060), "B": (0.240, 0.075)}[LOT])        # the middle of the elbow crease
CRADLE_AT = np.array({"A": (0.400, 0.170), "B": (0.410, 0.150)}[LOT])   # under the middle of the holder's barrel
ARM, CRADLE, HOLDER, PATCH = "training_arm", "cradle", "holder", "skin_patch"
BEVEL_START = os.environ.get("VEIN_BEVEL", "up")       # "down" starts the holder turned over (for a control run)

# -- scoring -----------------------------------------------------------------------------------------------------------
TOUCH = 0.0015                    # the needle within this of something touches it
STICK = 0.015                     # the needle within this of the other gripper: a needlestick
HELD = 0.040                      # a gripper's grasp point within this of the barrel's middle holds the holder
STILL = 0.001                     # the holder's flange (its back end) has moved less than this over the last second ...
STILL_DEG = 2.0                   # ... and the holder has turned less than this
STILL_STEPS = 20
ANGLE_END = (5.0, 30.0)           # the needle's angle to the skin at the end


def arm_pos() -> np.ndarray:
    return np.array([SITE[0], SITE[1], TOP])


def holder_start() -> np.ndarray:
    return np.array([CRADLE_AT[0], CRADLE_AT[1], TOP + K.AXIS_H])


OBJECTS = [TrainArm(ARM, pos=tuple(float(v) for v in arm_pos()), size=(0.10, 0.78, 0.10)),
           Cradle(CRADLE, pos=(float(CRADLE_AT[0]), float(CRADLE_AT[1]), TOP), size=(0.04, 0.03, 0.03)),
           Holder(HOLDER, pos=tuple(float(v) for v in holder_start()), size=(0.034, 0.085, 0.024),
                  quat=(0.0, 1.0, 0.0, 0.0) if BEVEL_START == "down" else (0.0, 0.0, 0.0, 1.0)),
           SkinPatch(PATCH, pos=tuple(float(v) for v in arm_pos()), size=(0.06, 0.07, 0.01), arm=ARM)]

# -- what the trial looks like now ------------------------------------------------------------------------------------
_STICK = Stick()
_STATE: dict = {}


def _clear() -> None:
    _STICK.reset()
    _STATE.update(steps=0, void=None, holder=None, traction=0.0, drawn=False, backs=[], still=False, near=1.0,
                  tip=(0.0, 0.0, 0.0), resist=0.0)


_clear()


def score_from(void, in_skin: bool, in_vein: bool, in_hand: bool, still: bool, angle_ok: bool, kept: bool) -> float:
    """Pure scoring rule. ``void``: a needlestick, the needle touched something before the skin at the site, a bad
    stick (away from the site, outside 10 to 30 degrees, bevel not up), or the tip went through the vein. ``in_skin``:
    the needle is in the skin after a good stick. ``in_vein``: its tip is in the vein. ``in_hand``: a gripper holds
    the holder. ``still``: the holder's back end has not moved 1 mm, nor the holder turned 2 degrees, in the last
    second. ``angle_ok``: the needle is 5 to 30 degrees to the skin now. ``kept``: the skin below the site was drawn
    5 mm toward the wrist when the tip reached the vein and has been kept drawn ever since."""
    if void or not in_skin:
        return 0.0
    if not in_vein:
        return 0.25
    if not (in_hand and still and angle_ok):
        return 0.5
    return 1.0 if kept else 0.75


def _score() -> float:
    st, s = _STATE, _STICK
    return score_from(st["void"], s.state == "in" and s.good, s.in_vein, st["holder"] is not None, st["still"],
                      ANGLE_END[0] <= s.angle <= ANGLE_END[1], s.held and s.kept)


def _segment_gap(points, a, b) -> float:
    p = np.atleast_2d(np.asarray(points, float))
    ab = b - a
    t = np.clip((p - a) @ ab / max(float(ab @ ab), 1e-12), 0.0, 1.0)
    return float(np.linalg.norm(p - (a + t[:, None] * ab), axis=1).min())


@on_reset
def _reset(environment) -> None:
    _clear()


def still_from(backs, dirs) -> bool:
    """Whether the holder has been still: over the last ``STILL_STEPS`` control steps its back end (the middle of its
    flange) stayed within ``STILL`` of where it is now and its axis within ``STILL_DEG`` of where it points now."""
    if len(backs) < STILL_STEPS:
        return False
    b, d = np.asarray(backs, float), np.asarray(dirs, float)
    moved = float(np.linalg.norm(b - b[-1], axis=1).max())
    turned = math.degrees(math.acos(max(-1.0, min(1.0, float((d @ d[-1]).min())))))
    return moved < STILL and turned < STILL_DEG


@on_step
def _watch(environment) -> None:
    from yam_arena.scene import object_states   # importable only once the simulator has started

    st, s = _STATE, _STICK
    st["steps"] += 1
    states = object_states(environment, [HOLDER, PATCH])
    h = states[HOLDER]
    nd = K.needle(h.pos, h.quat_xyzw)
    origin = arm_pos()
    tip, base = nd["tip"] - origin, nd["base"] - origin          # the arm's frame (the arm is not turned)
    grasp = K.finger_points(environment)                         # left, right
    hands = [np.array(p) for p in K.gripper_points(environment)]
    holding = [i for i in (0, 1) if float(np.linalg.norm(np.asarray(grasp[i]) - h.pos)) <= HELD]
    st["holder"] = ("left", "right")[holding[0]] if holding else None
    others = [i for i in (0, 1) if i not in holding]
    st["traction"] = K.traction(states[PATCH].pos, origin)
    st["drawn"] = bool(st["traction"] >= K.TRACTION)
    was = s.state
    s.step(tip, nd["dir"], nd["up"], st["drawn"])
    # what the needle meets: its joint to the arm goes on when its point reaches the skin, and off when it is out again
    if not K.engaged():
        if s.over and s.depth >= -K.ENGAGE:
            entry = K.skin_cross(tip, nd["dir"])
            K.engage(ARM, HOLDER, tip if entry is None else entry, h.quat_xyzw, K.F_SKIN)
    elif not s.over or s.depth < -0.001:
        K.release()
    else:
        K.set_resistance(K.resistance(tip, s.vein_x))
    st["resist"] = float(K._NEEDLE["force"]) if K.engaged() else 0.0
    # what voids the trial
    if not st["void"]:
        for i in others:
            gap = _segment_gap(hands[i] - origin, base, tip)
            st["near"] = min(st["near"], gap)
            if gap < STICK:
                st["void"] = f"needle {1000 * gap:.0f} mm from the {('left', 'right')[i]} gripper at step {st['steps']}"
        if was == "out" and s.state == "out":
            mid = (tip + base) / 2
            at_site = abs(tip[0]) <= K.SITE_X + 0.004 and abs(tip[1]) <= K.SITE_Y + 0.004     # the tip, just past where it crosses
            if nd["tip"][2] < TOP + TOUCH or (nd["tip"][2] + nd["base"][2]) / 2 < TOP + TOUCH:
                st["void"] = f"needle touched the table at step {st['steps']}"
            elif (float(K.arm_distance(tip)[0]) < TOUCH and not at_site) or float(K.arm_distance(mid)[0]) < TOUCH:
                st["void"] = f"needle touched the arm away from the site at step {st['steps']}"
        if s.state == "in" and not s.good:
            st["void"] = f"bad stick ({s.why}) at step {st['steps']}"
        elif s.through:
            st["void"] = f"needle went through the vein at step {st['steps']}"
    st["tip"] = tuple(float(v) for v in tip)
    back = K.place(h.pos, h.quat_xyzw, (0.0, K.BARREL_Y[0], 0.0)) - origin
    st["backs"] = (st["backs"] + [(back, nd["dir"].copy())])[-STILL_STEPS:]
    st["still"] = still_from([b for b, _ in st["backs"]], [d for _, d in st["backs"]])


# -- scoring --------------------------------------------------------------------------------------------------------
def score(objects) -> float:
    """Scored at the end of the trial. 0 unless the needle is in the skin after a good stick: it crossed the skin
    within 8 mm of the vein's line and 12 mm of the elbow crease, 10 to 30 degrees to the skin, bevel facing away
    from the skin within 30 degrees. 0.25: it is. 0.5: and its tip is inside the vein (4 mm across, its middle 3.5 mm
    under the skin), which needs the vein to have been held when the tip reached it: the skin below the site drawn at
    least 5 mm toward the wrist. 0.75: and a gripper holds the holder, the holder's back end has not moved 1 mm nor
    the holder turned 2 degrees in the last second, and the needle is 5 to 30 degrees to the skin. 1.0: and the skin
    has been kept drawn ever since the tip reached the vein. The whole trial scores 0 if the needle came within 15 mm
    of the gripper that is not holding it, touched the table or the arm away from the site (within 1.5 mm), made a bad
    stick (away from the site, outside 10 to 30 degrees, or bevel not up), or went through the vein's far wall."""
    return _score()


def success(objects) -> bool:
    return score(objects) >= 1.0


def report(objects) -> dict:
    st, s = _STATE, _STICK
    return {"score": _score(), "lot": LOT, "void": st["void"], "state": s.state, "sticks": s.sticks, "good": s.good,
            "why": s.why, "entry_angle": None if s.entry_angle is None else round(s.entry_angle, 1),
            "entry_bevel": None if s.entry_bevel is None else round(s.entry_bevel, 1),
            "entry_mm": None if s.entry is None else [round(1000 * float(v), 1) for v in s.entry],
            "tip_mm": [round(1000 * v, 1) for v in st["tip"]], "depth_mm": round(1000 * s.depth, 2),
            "angle": round(s.angle, 1), "bevel": round(s.bevel, 1), "reached": s.reached, "held": s.held, "kept": s.kept,
            "vein_x_mm": round(1000 * s.vein_x, 1), "in_vein": s.in_vein, "through": s.through,
            "holder": st["holder"], "traction_mm": round(1000 * st["traction"], 1), "drawn": st["drawn"],
            "resist_N": round(st["resist"], 2), "still": st["still"],
            "nearest_other_gripper_mm": round(1000 * st["near"], 1)}


def demo(objects=None, seed=None):
    return {}
