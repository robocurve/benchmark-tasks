"""On the station table (standing in for a clinic's treatment-room counter): a Mayo instrument tray under a sterile
drape (the sterile field being set up for a minor procedure), and beside it a sterile supply pack that has been
opened flat, its wrapper's inside up, with three things on it: iris scissors, a 1 mL syringe and a stainless medicine
cup. A pair of sterile thumb dressing forceps lies across the near edge of the field: working end on the field, handle
sticking out over the edge.

The job is the one a nurse or medical assistant does with bare (unsterile) hands while setting up: use the sterile
forceps to move the three items from the opened pack onto the sterile field, then lay the forceps back down.

The robot's grippers are not sterile. Only the working end of the forceps may touch the items, the field and the
inside of the pack. The outer 2.5 cm of the field and of the pack's wrapper count as unsterile, and so does the table.
No part of either arm may pass over a sterile area. While they are held, the forceps' tips must not be raised above
their handle. Anything sterile that touches something unsterile is contaminated, and passes that on to what it touches
next.

The items' places on the pack change with ``XFER_LOT`` (A or B).

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

import xferkit as K  # noqa: E402
from xferkit import Blade, Cup, Forceps, Scissors, Syringe, on_reset, on_step  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]

INSTRUCTION = ("Using the sterile forceps, move the scissors, the syringe and the medicine cup from the opened sterile "
               "pack onto the sterile field without contaminating anything, then lay the forceps back across the "
               "field's edge and let go.")

# -- layout (m; x away from the robot, y left, heights above the table) ---------------------------------------------------
LOT = os.environ.get("XFER_LOT", "A")
BORDER = 0.025                                   # the outer 2.5 cm (1 in) of a sterile field counts as unsterile
NEAR = 0.330                                     # x of the near edge of both the field and the pack
TRAY = (0.325, 0.490, 0.019)                     # the Mayo tray: depth (x), width (y), height
DRAPE_T = 0.001                                  # the drape over it
FIELD_Y = (-0.170, 0.320)                        # the tray's edges in y
FIELD_TOP = TRAY[2] + DRAPE_T                    # the field's surface above the table
PACK = (0.305, 0.305, 0.002)                     # the opened wrapper: 12 in square, lying flat
PACK_Y = (-0.515, -0.210)
PACK_TOP = PACK[2]
# the sterile part of each: its top surface inside the border (x0, x1, y0, y1)
FIELD_IN = (NEAR + BORDER, NEAR + TRAY[0] - BORDER, FIELD_Y[0] + BORDER, FIELD_Y[1] - BORDER)
PACK_IN = (NEAR + BORDER, NEAR + PACK[0] - BORDER, PACK_Y[0] + BORDER, PACK_Y[1] - BORDER)
# On the pack. The scissors lie flat, rings toward the robot, points away and 15 degrees to the left (their left shank
# then points straight away): (x, y) of their screw. The syringe lies along x, thumb rest toward the robot: (x of the
# thumb rest's end, y of its axis). The cup: its middle.
SCISSORS_AT = {"A": (0.442, -0.277), "B": (0.445, -0.270)}[LOT]
SCISSORS_YAW = K.shank_yaw()                     # 15.1 degrees
SYRINGE_AT = {"A": (0.368, -0.445), "B": (0.372, -0.380)}[LOT]
CUP_AT = {"A": (0.385, -0.375), "B": (0.390, -0.440)}[LOT]
FORCEPS_Y = {"A": -0.125, "B": -0.120}[LOT]      # the forceps lie along x at this y
OVERHANG = 0.050                                 # their handle sticks out this far over the field's near edge
BLUE, WRAP = (0.20, 0.42, 0.66), (0.36, 0.60, 0.74)

FORCEPS, BLADE = "forceps", "blade"
ITEMS = ("scissors", "syringe", "cup")           # each item's name is also its kind in the kit
AREAS = {"field": (FIELD_IN, FIELD_TOP), "pack": (PACK_IN, PACK_TOP)}

# -- scoring -----------------------------------------------------------------------------------------------------------
TOUCH = 0.0015           # within this of a surface or of another thing counts as touching it
NEAR_ARM = 0.004         # ... and within this of any point of an arm
TIPS_UP = 0.005          # while the forceps are held, their tips may not be more than this above the handle's end
UPRIGHT = math.radians(10.0)   # the cup stands if its axis is within this of upright
LYING = 0.025            # the syringe and the scissors lie on the field if no part of them is higher than this above it
CLEAR = 0.030            # both arms this far from a thing for it to be let go
TIP_CLEAR = 0.020        # ... and the forceps' tips this far from an item
STILL = 0.02             # m/s


def tail_start() -> np.ndarray:
    """Where the forceps' tail is at the start (world): on edge, the blades' lower edges on the field."""
    return np.array([NEAR - OVERHANG, FORCEPS_Y, TOP + FIELD_TOP + K.half_width(0.05) + 0.0005])


def item_start(name: str) -> tuple:
    """Where an item is at the start: (position of its own origin in the world, quaternion xyzw)."""
    z = TOP + PACK_TOP + 0.0005
    if name == "cup":
        return np.array([CUP_AT[0], CUP_AT[1], z]), (0.0, 0.0, 0.0, 1.0)
    if name == "syringe":
        pitch, height = K.syringe_rest()
        back = -float(K.syringe_pieces()[4][0][:, 0].min())           # from the thumb rest's end to the centre of mass
        return np.array([SYRINGE_AT[0] + back, SYRINGE_AT[1], z + height]), K.turn_y(pitch)
    quat = K.turn_z(SCISSORS_YAW)
    screw = K.item_local("scissors", (K.X_PIVOT_AT, 0.0, 0.0))        # the screw, on the underside, in their own frame
    at = np.array([SCISSORS_AT[0], SCISSORS_AT[1], z]) - K.place((0.0, 0.0, 0.0), quat, screw)
    return at, quat


def _objects() -> list:
    half = K.F_ANGLE / 2
    t = tail_start()
    tray = Box("field", pos=(NEAR + TRAY[0] / 2, sum(FIELD_Y) / 2, TOP + FIELD_TOP / 2), size=(TRAY[0], TRAY[1], FIELD_TOP),
               color=BLUE)
    pack = Box("pack", pos=(NEAR + PACK[0] / 2, sum(PACK_Y) / 2, TOP + PACK_TOP / 2), size=PACK, color=WRAP)
    fa = Forceps(FORCEPS, pos=tuple(float(v) for v in t), size=(0.14, 0.01, 0.012), quat=K.turn_z(half), blade=BLADE)
    fb = Blade(BLADE, pos=tuple(float(v) for v in t), size=(0.14, 0.01, 0.012), quat=K.turn_z(-half))
    kinds = {"cup": (Cup, (0.052, 0.052, 0.045)), "syringe": (Syringe, (0.093, 0.020, 0.011)),
             "scissors": (Scissors, (0.115, 0.048, 0.004))}
    items = []
    for n in ITEMS:
        pos, quat = item_start(n)
        cls, size = kinds[n]
        items.append(cls(n, pos=tuple(float(v) for v in pos), size=size, quat=tuple(float(v) for v in quat)))
    return [tray, pack, fa, fb, *items]


OBJECTS = _objects()


# -- where things are, and what touches what (pure) ----------------------------------------------------------------------

def _inside(points, rect) -> np.ndarray:
    p = np.atleast_2d(points)
    return (p[:, 0] >= rect[0]) & (p[:, 0] <= rect[1]) & (p[:, 1] >= rect[2]) & (p[:, 1] <= rect[3])


_FIELD_RECT = (NEAR, NEAR + TRAY[0], FIELD_Y[0], FIELD_Y[1])
_PACK_RECT = (NEAR, NEAR + PACK[0], PACK_Y[0], PACK_Y[1])


def surface_height(points) -> np.ndarray:
    """The height (above the table) of the surface under each point: the field's top, the pack's, or the table."""
    p = np.atleast_2d(points)
    h = np.zeros(len(p))
    h[_inside(p, _PACK_RECT)] = PACK_TOP
    h[_inside(p, _FIELD_RECT)] = FIELD_TOP
    return h


def over(points, area: str) -> np.ndarray:
    """For each point: is it above the sterile part of ``area`` ("field" or "pack"), at any height?"""
    return _inside(points, AREAS[area][0])


def touches(points, area: str) -> bool:
    """Does any point touch the sterile part of ``area``?"""
    p = np.atleast_2d(points)
    return bool(np.any(over(p, area) & (p[:, 2] - TOP - AREAS[area][1] < TOUCH)))


def touches_unsterile(points) -> bool:
    """Does any point touch the table, a border, or the side of the tray?"""
    p = np.atleast_2d(points)
    z = p[:, 2] - TOP
    sterile = over(p, "field") | over(p, "pack")
    if np.any(~sterile & (z - surface_height(p) < TOUCH)):
        return True
    grown = (_FIELD_RECT[0] - TOUCH, _FIELD_RECT[1] + TOUCH, _FIELD_RECT[2] - TOUCH, _FIELD_RECT[3] + TOUCH)
    return bool(np.any(_inside(p, grown) & ~_inside(p, _FIELD_RECT) & (z < FIELD_TOP)))


def _apart(a, b) -> float:
    """The least distance between two sets of points."""
    a, b = np.atleast_2d(a), np.atleast_2d(b)
    best = math.inf
    for i in range(0, len(a), 256):
        d = np.linalg.norm(a[i:i + 256, None, :] - b[None, :, :], axis=2)
        best = min(best, float(d.min()))
    return best


def new_state() -> dict:
    s = {"steps": 0, "dirty": {n: False for n in ("forceps", "field", "pack", *ITEMS)}, "why": {},
         "items_on": 0, "on": {n: False for n in ITEMS}, "down": False, "ready": False, "tips_up_mm": 0.0,
         "gap_mm": 0.0, "arm_over": False}
    return s


def _soil(s: dict, name: str, why: str) -> None:
    if not s["dirty"][name]:
        s["dirty"][name] = True
        s["why"][name] = f"step {s['steps']}: {why}"


def advance(s: dict, arm, blades, items, speeds) -> dict:
    """One control step of the record. ``arm``: points on both arms (world). ``blades``: ((pos, quat), (pos, quat))
    of the two blades. ``items``: {name: (pos, quat)}. ``speeds``: {name: m/s} for "forceps" and each item."""
    s["steps"] += 1
    arm = np.atleast_2d(arm)
    f = K.forceps_points(*blades[0], *blades[1])
    work, handle = f["work"], f["handle"]
    s["tips_up_mm"], s["gap_mm"] = round(1000 * f["up"], 1), round(1000 * f["gap"], 1)
    s["arm_over"] = False
    # an arm above a sterile area contaminates it
    for area in AREAS:
        if np.any(over(arm, area)):
            s["arm_over"] = True
            _soil(s, area, "an arm passed over it")
    # the forceps
    held = _apart(arm, np.vstack([work, handle])) < CLEAR
    if held and f["up"] > TIPS_UP:
        _soil(s, "forceps", "tips raised above the handle")
    if _apart(arm, work) < NEAR_ARM:
        _soil(s, "forceps", "an arm touched the working end")
    if touches_unsterile(work):
        _soil(s, "forceps", "the working end touched an unsterile surface")
    pts = {n: K.item_points(n, *items[n]) for n in ITEMS}
    near = lambda n, points, margin: bool(np.any(K.near_item(n, points, *items[n], margin)))   # noqa: E731
    for n in ITEMS:
        if near(n, arm, NEAR_ARM):
            _soil(s, n, "an arm touched it")
        if touches_unsterile(pts[n]):
            _soil(s, n, "it touched an unsterile surface")
        if near(n, handle, TOUCH):
            _soil(s, n, "the forceps' handle touched it")
    for area in AREAS:
        if touches(handle, area):
            _soil(s, area, "the forceps' handle touched it")
    # contamination passes between sterile things that touch
    on_area = {(n, area): touches(pts[n], area) for n in ITEMS for area in AREAS}
    work_on = {area: touches(work, area) for area in AREAS}
    work_at = {n: near(n, work, TOUCH) for n in ITEMS}
    pairs = [(a, b) for i, a in enumerate(ITEMS) for b in ITEMS[i + 1:]
             if near(b, pts[a], TOUCH) or near(a, pts[b], TOUCH)]
    for _ in range(3):
        for area in AREAS:
            if work_on[area]:
                if s["dirty"][area]:
                    _soil(s, "forceps", f"the working end touched the contaminated {area}")
                elif s["dirty"]["forceps"]:
                    _soil(s, area, "contaminated forceps touched it")
            for n in ITEMS:
                if on_area[n, area]:
                    if s["dirty"][area]:
                        _soil(s, n, f"it lay on the contaminated {area}")
                    elif s["dirty"][n]:
                        _soil(s, area, f"the contaminated {n} touched it")
        for n in ITEMS:
            if work_at[n]:
                if s["dirty"]["forceps"]:
                    _soil(s, n, "contaminated forceps touched it")
                elif s["dirty"][n]:
                    _soil(s, "forceps", f"the working end touched the contaminated {n}")
        for a, b in pairs:
            if s["dirty"][a] != s["dirty"][b]:
                _soil(s, a if s["dirty"][b] else b, f"the contaminated {b if s['dirty'][b] else a} touched it")
    # where things have ended up
    rect, top = AREAS["field"]
    for n in ITEMS:
        pos, quat = items[n]
        z = pts[n][:, 2] - TOP - top
        if n == "cup":
            resting = bool(K.cup_tilt(quat) < UPRIGHT and abs(pos[2] - TOP - top) < 0.003)
        else:
            resting = bool(z.min() < 0.003 and z.max() <= LYING)
        inside = bool(np.all(_inside(pts[n], rect)))
        let_go = not near(n, arm, CLEAR) and not near(n, f["tip"][None, :], TIP_CLEAR)
        s["on"][n] = bool(resting and inside and let_go and speeds[n] < STILL and not s["dirty"][n])
    s["items_on"] = sum(s["on"].values())
    s["down"] = bool(not held and speeds["forceps"] < STILL)
    resting = any(bool(np.all(over(work, area))) and touches(work, area) and not s["dirty"][area] for area in AREAS)
    s["ready"] = bool(s["down"] and resting and not s["dirty"]["forceps"])
    return s


def score_from(field_clean: bool, items_on: int, ready: bool) -> float:
    """Pure scoring rule. ``field_clean``: the sterile field has not been contaminated. ``items_on``: how many of the
    three items rest on the field's sterile part, uncontaminated, let go and at rest. ``ready``: the forceps have been
    put down and let go, are at rest and uncontaminated, with their whole working end resting on a sterile area."""
    if not field_clean or items_on <= 0:
        return 0.0
    if items_on < len(ITEMS):
        return 0.25 * items_on
    return 1.0 if ready else 0.75


# -- what the trial looks like now ------------------------------------------------------------------------------------
_STATE: dict = new_state()


def _clear() -> None:
    _STATE.clear()
    _STATE.update(new_state())


@on_reset
def _reset(environment) -> None:
    _clear()


@on_step
def _watch(environment) -> None:
    from yam_arena.scene import object_states   # importable only once the simulator has started

    st = object_states(environment, [FORCEPS, BLADE, *ITEMS])
    blades = tuple((st[n].pos, st[n].quat_xyzw) for n in (FORCEPS, BLADE))
    items = {n: (st[n].pos, st[n].quat_xyzw) for n in ITEMS}
    speeds = {n: float(np.linalg.norm(st[n].lin_vel)) for n in ITEMS}
    speeds["forceps"] = max(float(np.linalg.norm(st[n].lin_vel)) for n in (FORCEPS, BLADE))
    advance(_STATE, K.arm_points(environment), blades, items, speeds)


# -- scoring --------------------------------------------------------------------------------------------------------
def score(objects) -> float:
    """Scored at the end of the trial from what was touched during it. Touching means within 1.5 mm (4 mm for any
    part of an arm). The sterile things are the scissors, the syringe, the cup, the forceps' working end (the 5 cm
    from the tips) and the tops of the field and of the pack's wrapper more than 2.5 cm from their edges. A sterile
    thing is contaminated when it touches the table, a 2.5 cm border, the tray's side, an arm, the forceps' handle
    (the 6.5 cm from the tail) or anything already contaminated; the field or the pack also when any part of an arm
    is above its sterile part, at any height; the forceps also when, while they are held (an arm within 3 cm of
    them), their tips are more than 5 mm higher than the end of their handle.
    0 if the field is contaminated. Otherwise 0.25 for one item, 0.5 for two and 0.75 for all three that rest on the
    field's sterile part (every part of the item inside the border; the cup upright within 10 degrees; the syringe
    and the scissors lying, no part more than 2.5 cm above the field), uncontaminated, let go (both arms 3 cm away,
    the forceps' tips 2 cm away) and at rest (under 2 cm/s). 1.0: all three, and the forceps put down, let go (both
    arms 3 cm away), at rest and uncontaminated, with their whole working end resting on the sterile part of the
    field or of the pack."""
    s = _STATE
    return score_from(not s["dirty"]["field"], s["items_on"], s["ready"])


def success(objects) -> bool:
    return score(objects) >= 1.0


def report(objects) -> dict:
    s = _STATE
    return {"score": score(objects), "lot": LOT, "dirty": dict(s["dirty"]), "why": dict(s["why"]),
            "items_on": s["items_on"], "on": dict(s["on"]), "forceps_down": s["down"], "forceps_ready": s["ready"],
            "tips_up_mm": s["tips_up_mm"], "tip_gap_mm": s["gap_mm"], "arm_over": s["arm_over"]}


def demo(objects=None, seed=None):
    return {}
