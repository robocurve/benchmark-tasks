"""Straddling the middle of the bench: two round bowls bolted down, 220 mm across, their rims about
60 mm above the bench and 250 mm apart, their midpoint the centre of the bench. The nearer one holds
24 amber glass beads 18 mm across; the far one is empty.

On a rack to one side, held clear of the bench, lie two tools pointing the same way: a stainless
scoop -- an 80 x 70 mm pan with 26 mm walls on three sides, an open lip, and a 90 x 24 x 18 mm
handle -- and a flat spatula with no walls at all, which beads roll straight off.

Score (task_score): 1.0 once at least 22 of the 24 beads are resting in the far bowl; otherwise the
best the trial reached -- 0.5 at 14 beads, 0.25 at 7, 0 below that.

Run it:  inspect-robots run --embodiment yam_arena -E task=scoop_beads.py \\
           --instruction "Move more than four beads into the empty bowl." --scorer task_score ...
"""

import math
import importlib.util
import os
import pathlib
import sys

# The embodiment imports this file by path, so its folder is not on sys.path.
for _name, _file in (("simkit_scoop", "simkit.py"), ("bench_scoop", "bench.py")):
    if _name not in sys.modules:
        _spec = importlib.util.spec_from_file_location(_name, pathlib.Path(__file__).with_name(_file))
        _module = importlib.util.module_from_spec(_spec)
        sys.modules[_name] = _module
        if _name == "bench_scoop":
            sys.modules["simkit"] = sys.modules["simkit_scoop"]  # bench.py does `import simkit`
        _spec.loader.exec_module(_module)

simkit = sys.modules["simkit_scoop"]
bench = sys.modules["bench_scoop"]

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP

simkit.install()
DOWN = simkit.patch_demo()  # 3x3 rotations in Waypoint.yaw, and ArmPlan gliding along a spline

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "best"

OBJECTS = [
    *bench.bowl_objects(TOP),
    *bench.bead_objects(TOP),
    *bench.rack_objects(TOP),
    bench.scoop_object(TOP),
    bench.spatula_object(TOP),
]

_TRACE = {"on": bool(os.environ.get("SCOOP_TRACE")), "n": 0}

# -- thresholds: fractions of the beads ---------------------------------------------------

# The job is more than four beads out of a full tray, not the whole tray.
FULL = 5
HALF = 3
QUARTER = 1


def delivered(objects) -> int:
    return bench.beads_in(objects, bench.TARGET_XY, TOP)


def success(objects) -> bool:
    """Full score: more than four beads resting in the far bowl.

    ``SUCCESS_HOLD_S`` (1 s) means this must hold for a second, and every bead must have settled, so
    beads still bouncing on a rim do not count until they have come to rest somewhere.
    """
    return bool(delivered(objects) >= FULL and bench.beads_still(objects))


def distance(objects) -> float:
    """Fraction of the beads still to be moved, as a 0..1 'distance' from the goal."""
    return float(max(0, FULL - delivered(objects)) / FULL)


def score(objects) -> float:
    """Partial credit by bead count (``SCORE_OVER = "best"`` keeps the most ever delivered).

    A judge counts the beads in the far bowl; nothing else is needed. ``score()`` stops at 0.5, so
    1.0 comes only from ``success()`` holding for a second with every bead settled -- a scoopful in
    mid-air over the bowl cannot latch it.
    """
    n = delivered(objects)
    value = 0.5 if n >= HALF else 0.25 if n >= QUARTER else 0.0
    if _TRACE["on"]:
        _TRACE["n"] += 1
        if _TRACE["n"] % 20 == 0:
            src = bench.beads_in(objects, bench.SOURCE_XY, TOP)
            sc = objects["scoop"]
            up = sum(1 for i in range(bench.BEAD_COUNT)
                     if objects[f"bead{i:02d}"].pos[2] > TOP + 0.030)
            spread = [
                (objects[f"bead{i:02d}"].pos[0] - bench.SOURCE_XY[0]) * U[0]
                + (objects[f"bead{i:02d}"].pos[1] - bench.SOURCE_XY[1]) * U[1]
                for i in range(bench.BEAD_COUNT)
            ]
            # Where every bead that is in neither vessel actually is: how far it sits from the pan
            # it is supposed to be riding in, and how far from the bowl it is supposed to land in.
            loose_at = []
            for i in range(bench.BEAD_COUNT):
                q = objects[f"bead{i:02d}"].pos
                if (math.dist(q[:2], bench.SOURCE_XY) <= bench.SOURCE_INNER_R
                        or math.dist(q[:2], bench.TARGET_XY) <= bench.TARGET_INNER_R):
                    continue
                # Signed, along and across the pour's own heading, so a miss says which way to aim.
                dx = q[0] - bench.TARGET_XY[0]
                dy = q[1] - bench.TARGET_XY[1]
                along = dx * math.cos(POUR_PSI) + dy * math.sin(POUR_PSI)
                side = -dx * math.sin(POUR_PSI) + dy * math.cos(POUR_PSI)
                loose_at.append(f"(along{along * 1000:+.0f},side{side * 1000:+.0f}"
                                f",z{(q[2] - TOP) * 1000:.0f})")
            print("  loose: " + (" ".join(loose_at) or "none"), flush=True)
            print(
                f"[scoop {_TRACE['n']:4d}] target={n:2d} source={src:2d}"
                f" loose={bench.BEAD_COUNT - n - src:2d} lifted={up:2d}"
                f" scoop={sc.pos.round(3)} tilt={bench.body_tilt_deg(sc):5.1f}"
                f" low={(sc.low[2] - TOP) * 1000:6.1f}mm"
                f" pile=[{min(spread) * 1000:+.0f},{max(spread) * 1000:+.0f}]mm"
                f" frombowl={math.dist(sc.pos[:2], bench.SOURCE_XY) * 1000:4.0f}mm"
                f" score={value}",
                flush=True,
            )
    return value


# -- demo ---------------------------------------------------------------------------------

GRIP_XY = bench.scoop_grip_xy()
# Pads straddling the centre of mass exactly. Closing 13 mm above it, as the old web grip did,
# leaves a lever that works on the scoop every time the arm accelerates.
GRIP_Z = bench.scoop_grip_z(TOP)
GRIP_FINGER = bench.finger_for(bench.HANDLE_W - bench.GRIP_SQUEEZE)

DIG_TILT = 0.45  # lip down, to get under the beads; also what lifts the handle over the wall
# Enough to keep the lip on the floor while it is pushing, and no more.
DRAW_PRESS = 0.008
# The draw is where the handle is closest to the tray's wall, so it cannot be run flat: at 7
# degrees the handle was 15 mm under the wall it has to come back out over.
DRAW_TILT = 0.30
# Hovering over the wall before dropping in.
ENTRY_TILT = 0.25
LEVEL_TILT = 0.14  # nearly flat, so a loaded pan does not spill -- but still nose-down
# enough that the handle rides over the dish's rim rather than onto it Once the pan has beads in
# it, nose-down is the wrong way up: the pan is open at the lip, which is the low end, and 8
CARRY_TILT = -0.12
# Scanned over every heading with both bowls in place: the dig and the carry both solve at the
# scoop's own heading, and only the pour wants a different one -- the arm cannot tip the pan out
CARRY_PSI = math.radians(0.0)
POUR_PSI = math.radians(60.0)
# 37 degrees, and poured at its own heading.
POUR_TILT = 0.96

SRC_RIM_Z = bench.source_rim_z(TOP)
TGT_RIM_Z = bench.target_rim_z(TOP)
# Just enough to clear both rims, no more.
CLEAR_Z = 0.015  # hovering over a vessel, before dropping in and after lifting out
# Crossing the bench between the two vessels rather than out over either of them.
CARRY_XY = (0.300, 0.050)
CARRY_Z = TOP + 0.085
POUR_Z = TGT_RIM_Z + 0.030
# How far short of the bowl the pan is placed.
POUR_BACK = 0.070


def _pour_xy():
    return (bench.TARGET_XY[0] - POUR_BACK * math.cos(POUR_PSI),
            bench.TARGET_XY[1] - POUR_BACK * math.sin(POUR_PSI))


PSI = bench.SCOOP_YAW  # the scoop keeps one heading from pick-up to pour
U = (math.cos(PSI), math.sin(PSI))  # the way the lip leads when the pan is drawn through the beads
V = (-math.sin(PSI), math.cos(PSI))  # across the draw, for spacing the passes out


def _pan(bowl, along: float, across: float, z: float):
    """A pan position ``along`` the draw and ``across`` it, measured from a bowl's centre."""
    return (bowl[0] + along * U[0] + across * V[0], bowl[1] + along * U[1] + across * V[1], z)


def _wp(seconds, pan, tilt, label, finger=None, psi=None):
    """A waypoint that places the scoop's pan at ``pan`` with the scoop tipped ``tilt``.

    ``psi`` is the scoop's heading. It is the same for the dig and the pour, but the carry between
    them turns the scoop about the vertical: at the digging heading the arm cannot hold a loaded
    pan nose-up anywhere over the middle of the bench, and nose-down it empties on the way. Turning
    the scoop costs nothing now that the grip is on its own axis -- the pan simply swivels under the
    hand -- and nose-up holds the beads against the back wall whichever way the lip is pointing.
    """
    psi = PSI if psi is None else psi
    return Waypoint(
        seconds,
        bench.scoop_tcp(pan, tilt, psi),
        GRIP_FINGER if finger is None else finger,
        label,
        bench.scoop_rotation(tilt, DOWN, psi),
    )


# The pan is 70 mm wide against a pile 57 mm across, so one pass already spans it; the offsets
# only vary where the lip bites, pass to pass.
DIG_PASSES = (0.0, 0.0, 0.0, 0.0)


def dig_pass(across: float):
    """One pass through the beads as (seconds, pan position, tilt, label).

    The lip goes down on *clear floor beyond the pile*, then drives through it. Dropping the pan
    onto the middle of the pile instead simply rests it on top of the beads -- they are 18 mm
    across and the lip is 2 mm thick, so it never gets underneath, and the scoop rode 30 mm high
    through every dig while the beads sat untouched.

    bench.dig_pan_z puts the leading lip on the dish's *measured* floor at whatever tilt is held,
    and the whole sweep stays inside SOURCE_FLAT_R, where that floor is flat: further out the lip
    meets the curved wall instead.
    """
    src = bench.SOURCE_XY
    # Where this track meets the tray's wall: the flat floor is a circle, so an off-centre track
    # runs out of it sooner. Carrying the lip past that point drives the pan into the wall.
    span = math.sqrt(max(bench.SOURCE_FLAT_R ** 2 - (abs(across) + bench.PAN_W / 2) ** 2, 0.0))
    end = span - bench.PAN_L / 2 - 0.004
    start = -min(0.045, end - 0.010)
    return [
        (2.2, _pan(src, start - 0.003, across, SRC_RIM_Z + 0.006), ENTRY_TILT, "over the bowl"),
        # Six seconds on the floor, not two, and held there by a repeated waypoint.
        (2.8, _pan(src, start, across, bench.dig_pan_z(TOP, DIG_TILT)), DIG_TILT, "lip onto the bottom"),
        (2.2, _pan(src, start, across, bench.dig_pan_z(TOP, DIG_TILT)), DIG_TILT, "settle on the floor"),
        # The draw runs the beads up against the dish's own wall.
        (3.0, _pan(src, (start + end) / 2, across, bench.dig_pan_z(TOP, DRAW_TILT, DRAW_PRESS)), DRAW_TILT,
         "draw it through"),
        # ...and it has to be carried all the way to that wall.
        (2.4, _pan(src, end, across, bench.dig_pan_z(TOP, DRAW_TILT, DRAW_PRESS)), DRAW_TILT,
         "run them up the wall"),
        # Straight up from where the draw ended, not back from it.
        (1.0, _pan(src, end, across, TOP + 0.052), CARRY_TILT, "lift them off the wall",
         None, CARRY_PSI),
        # And only now roll nose-up, which is what stops an open-lipped pan pouring itself out.
        (1.6, _pan(src, end, across, TOP + 0.062), CARRY_TILT, "come level", None, CARRY_PSI),
                # Out of the bowl and turned to the carrying heading in one move: nose-up is what
                # keeps the load in, and the arm can only hold nose-up once the scoop has
        (1.6, _pan(src, end - 0.007, across, TOP + 0.078), CARRY_TILT, "lift out", None, CARRY_PSI),
    ]


def pour_pass():
    """Carrying a loaded pan to the far bowl and tipping it in."""
    tgt = bench.TARGET_XY
    return [
        (2.4, (*CARRY_XY, CARRY_Z), CARRY_TILT, "carry across", None, CARRY_PSI),
        # Square over the bowl before any of the tilt goes on.
        (1.6, (0.270, -0.069, TOP + 0.105), CARRY_TILT, "in close", None, CARRY_PSI),
        (1.8, (*_pour_xy(), POUR_Z), POUR_TILT, "tip it out", None, POUR_PSI),
        (1.4, (*_pour_xy(), POUR_Z), POUR_TILT, "let it run out", None, POUR_PSI),
        (1.4, (*CARRY_XY, CARRY_Z), CARRY_TILT, "come back level", None, CARRY_PSI),
    ]


def demo():
    """Left arm only. Take the scoop by its handle; draw the pan across the near bowl's floor, lip
    leading, coming level as it fills; lift it out, carry it to the far bowl and tip it in. Three
    passes, because one does not lift 90% of the beads.

    The wrist holds one heading throughout -- see bench.SCOOP_YAW -- so nothing rolls while loaded.
    """
    # Come down onto the shaft from directly above.
    level = bench.scoop_rotation(0.0, DOWN, PSI)
    plan = [
        Waypoint(2.4, (*GRIP_XY, GRIP_Z + 0.045), FINGER_OPEN, "above the scoop", level),
        Waypoint(1.4, (*GRIP_XY, GRIP_Z), FINGER_OPEN, "down onto the handle", level),
        Waypoint(0.9, (*GRIP_XY, GRIP_Z), GRIP_FINGER, "take the handle", level),
    ]
    for i, across in enumerate(DIG_PASSES):
        for leg in dig_pass(across) + pour_pass():
            seconds, pan, tilt, label = leg[:4]
            finger, psi = (leg[4:] + (None, None))[:2] if len(leg) > 4 else (None, None)
            plan.append(_wp(seconds, pan, tilt, f"{label} {i + 1}", finger, psi))
    # Stand clear back over the rack the scoop came from.
    plan.append(_wp(2.2, (*bench.SCOOP_XY, CARRY_Z), LEVEL_TILT, "stand clear"))
    return {"left": plan}
