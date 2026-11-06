"""Standing in the middle of the bench, bolted down, a bottle: a 124 mm body under a shoulder that
closes to a 44 mm mouth, 97 mm to the top of its neck. Beside it, an open jug holding 20 amber
plastic beads 18 mm across. Standing beside them, a funnel: a 150 mm cone over a 44 mm spout, on
three short legs.

The bottle cannot be moved and cannot be filled by eye: a jug tipped over a 44 mm mouth puts most
of its beads on the bench.

Score (task_score): 1.0 once at least 18 of the 20 beads are resting in the bottle's body;
otherwise the best the trial reached -- 0.5 at 12 beads, 0.25 at 6, 0 below that.

Run it:  inspect-robots run --embodiment yam_arena -E task=funnel_beads.py \\
           --instruction "Fill the bottle with beads." --scorer task_score ...
"""

import importlib.util
import math
import os
import pathlib
import sys

import numpy as np

# The embodiment imports this file by path, so its folder is not on sys.path and a plain ``import
# bench`` would fail.
for _name, _file in (("simkit_funnel", "simkit.py"), ("bench_funnel", "bench.py")):
    if _name not in sys.modules:
        _spec = importlib.util.spec_from_file_location(_name, pathlib.Path(__file__).with_name(_file))
        _module = importlib.util.module_from_spec(_spec)
        sys.modules[_name] = _module
        if _name == "bench_funnel":
            sys.modules["simkit"] = sys.modules["simkit_funnel"]  # bench.py does `import simkit`
        _spec.loader.exec_module(_module)

simkit = sys.modules["simkit_funnel"]
bench = sys.modules["bench_funnel"]

from yam_arena.constants import FINGER_OPEN, PAD_TIP_PAST_TCP as PAD_TIP
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP

simkit.install()
DOWN = simkit.patch_demo()  # 3x3 rotations in Waypoint.yaw, and ArmPlan gliding along a spline

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "best"

OBJECTS = [
    bench.bottle_object(TOP),
    bench.jug_object(TOP),
    *bench.bead_objects(TOP),
    bench.funnel_object(TOP),
]

_TRACE = {"on": bool(os.environ.get("FUNNEL_TRACE")), "n": 0}

# -- thresholds: fractions of the beads ---------------------------------------------------

FULL = round(0.90 * bench.BEAD_COUNT)
HALF = round(0.60 * bench.BEAD_COUNT)
QUARTER = round(0.30 * bench.BEAD_COUNT)


def delivered(objects) -> int:
    return bench.beads_in_bottle(objects, TOP)


def success(objects) -> bool:
    """Full score: at least 90% of the beads resting in the bottle's body.

    ``SUCCESS_HOLD_S`` (1 s) means this must hold for a second, and every bead must have settled, so
    beads still rattling down the neck do not count until they have come to rest.
    """
    return bool(delivered(objects) >= FULL and bench.beads_still(objects))


def distance(objects) -> float:
    """Fraction of the beads still to be delivered, as a 0..1 'distance' from the goal."""
    return float(max(0, FULL - delivered(objects)) / FULL)


def _in_bottle(objects, i):
    """Whether bead ``i`` counts as delivered: below the bottle's shoulder, inside its body."""
    q = objects[f"bead{i:02d}"].pos
    return (math.dist(q[:2], bench.BOTTLE_XY) < bench.BODY_R
            and q[2] < bench.bottle_shoulder_z(TOP))


def _in_frame(points, origin, quat_xyzw):
    """``points`` expressed in the body's own frame, so a tipped jug reads as a cup, not a heap."""
    x, y, z, w = (float(v) for v in quat_xyzw)
    rot = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
    return (np.asarray(points) - np.asarray(origin)) @ rot


def score(objects) -> float:
    """Partial credit by bead count (``SCORE_OVER = "best"`` keeps the most ever delivered).

    A judge counts the beads in the bottle; nothing else is needed. ``score()`` stops at 0.5, so 1.0
    comes only from ``success()`` holding for a second with every bead settled -- a jug held over
    the funnel cannot latch it.
    """
    n = delivered(objects)
    value = 0.5 if n >= HALF else 0.25 if n >= QUARTER else 0.0
    if _TRACE["on"]:
        _TRACE["n"] += 1
        if _TRACE["n"] % 20 == 0:
            f = objects["funnel"]
            pos = np.array([objects[f"bead{i:02d}"].pos for i in range(bench.BEAD_COUNT)])
            fx, fy = f.pos[0], f.pos[1]
            cone = sum(1 for q in pos
                       if math.dist(q[:2], (fx, fy)) < bench.CONE_TOP_R
                       and q[2] > f.pos[2] - bench.CONE_H / 2)
            throat = sum(1 for q in pos
                         if math.dist(q[:2], (fx, fy)) < bench.SPOUT_R + 0.004
                         and q[2] < f.pos[2] - bench.CONE_H / 2)
            on_bench = sum(1 for q in pos if q[2] < TOP + 0.020)
            stuck = [(math.hypot(q[0] - fx, q[1] - fy), q[2] - TOP)
                     for i, q in enumerate(pos) if not _in_bottle(objects, i)]
            print("  beads: " + (" ".join(f"(r{r*1000:.0f},z{z*1000:.0f})" for r, z in stuck)
                                 or "all in the bottle"), flush=True)
            print(
                f"[funnel {_TRACE['n']:4d}] bottle={n:2d} loose={bench.BEAD_COUNT - n:2d}"
                f" funnel={f.pos.round(3)} tilt={bench.body_tilt_deg(f):5.1f}"
                f" jug={objects['jug'].pos.round(3)}"
                f" jugtilt={bench.body_tilt_deg(objects['jug']):5.1f} score={value}",
                flush=True,
            )
    return value


# -- demo ---------------------------------------------------------------------------------

# The wrist works at a different heading for each half of the job, and turning about the vertical
# between them spills nothing.
SEAT_PSI = 3 * math.pi / 2
# The right arm's own pouring direction.
POUR_PSI = math.radians(285.0)
# Where the jug is held to pour.
POUR_XY = (0.330, 0.040)
FUNNEL_GRIP = bench.funnel_grip_xy()
JUG_GRIP = bench.jug_grip_xy()
FUNNEL_FINGER = bench.finger_for(bench.funnel_grip_opening())
JUG_FINGER = bench.finger_for(bench.jug_grip_opening())

MOUTH_Z = bench.bottle_mouth_z(TOP)
# Where the funnel's spout ends up: inside the neck, standing clear of the shoulder so beads have
# somewhere to fall.
SEAT_BODY_Z = bench.funnel_seat_body_z(TOP)
# The spout's tip, on the way in.
LIFT_Z = bench.bottle_rim_z(TOP) + 0.015
FUNNEL_RIM_Z = bench.funnel_seat_body_z(TOP) + bench.funnel_rim_above_body()
POUR_Z = (FUNNEL_RIM_Z
          + 0.075)  # see check_layout: high enough that the jug's rim misses the funnel as it
                    # goes over, low enough that the beads do not bounce back out of the cone
# caught the funnel and shoved it off the bottle before anything came out
CARRY_Z = TOP + 0.120

# Right over, not just tipped.
POUR_TILT = 1.396
LEVEL_TILT = 0.08
CARRY_TILT = LEVEL_TILT  # the jug is brought across nearly level


def _tool_tcp(body, tilt: float, psi: float, reach: float, rise: float):
    """TCP that puts a tool's body centre at ``body``, pointing along ``psi`` and tipped ``tilt``."""
    from yam_arena.constants import PAD_TIP_PAST_TCP

    c, s = math.cos(tilt), math.sin(tilt)
    d = reach
    along = -(d * c - rise * s) + PAD_TIP_PAST_TCP * s
    return (
        body[0] + along * math.cos(psi),
        body[1] + along * math.sin(psi),
        body[2] + d * s + rise * c + PAD_TIP_PAST_TCP * c,
    )


def _rot(tilt: float, psi: float):
    """The TCP rotation that points a tool along ``psi`` and tips it by ``tilt``."""
    return simkit.rot_z(psi - math.pi / 2) @ _tip(tilt) @ simkit.rot_z(math.pi / 2) @ DOWN


def _tip(tilt: float):
    import numpy as np

    c, s = math.cos(tilt), math.sin(tilt)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, s], [0.0, -s, c]])


def demo():
    """Two arms, each on its own side, neither of them turning about the vertical.

    Left: take the funnel, carry it over the bottle, lower it until its spout is down the neck and
    its legs are on the shoulder, and let go. Right: take the jug -- by the body, not by a handle --
    carry it over the funnel and tip it out. The right arm waits while the left one works, because
    there is no funnel to pour into until it has.
    """
    bottle = bench.BOTTLE_XY
    seat_body = (bottle[0], bottle[1], SEAT_BODY_Z - 0.002)
    over_body = (bottle[0], bottle[1], LIFT_Z + bench.SPOUT_H / 2)

    def funnel_wp(seconds, body, label, finger=FUNNEL_FINGER):
        """The jaws close round the funnel's post, out beside its cone."""
        rise = bench.funnel_grip_z(TOP) - bench.funnel_rest_body_z(TOP) - PAD_TIP
        return Waypoint(seconds, _tool_tcp(body, 0.0, SEAT_PSI, bench.POST_R, rise),
                        finger, label, _rot(0.0, SEAT_PSI))

    def jug_wp(seconds, body, tilt, label, finger=JUG_FINGER):
        """The jaws close round the jug's body, so the TCP sits on its axis -- no lever either."""
        rise = bench.jug_grip_z(TOP) - (TOP + 0.004) - PAD_TIP
        return Waypoint(seconds, _tool_tcp(body, tilt, POUR_PSI, 0.0, rise),
                        finger, label, _rot(tilt, POUR_PSI))

    funnel_rest = (bench.FUNNEL_XY[0], bench.FUNNEL_XY[1], bench.funnel_rest_body_z(TOP))
    funnel_high = (funnel_rest[0], funnel_rest[1], funnel_rest[2] + 0.055)
    jug_rest = (bench.JUG_XY[0], bench.JUG_XY[1], TOP + 0.004)
    # The jug crosses the bench below the pour and rises into it.
    approach_z = max(POUR_Z - 0.095, FUNNEL_RIM_Z + 0.035)
    jug_high = (jug_rest[0], jug_rest[1], jug_rest[2] + 0.100)

    left = [
        funnel_wp(2.6, funnel_high, "above the funnel", FINGER_OPEN),
        funnel_wp(1.6, funnel_rest, "down round its collar", FINGER_OPEN),
        funnel_wp(1.0, funnel_rest, "take the funnel"),
        funnel_wp(1.8, (funnel_rest[0], funnel_rest[1], CARRY_Z - 0.020), "lift it clear"),
        funnel_wp(3.0, over_body, "carry it over the bottle"),
        funnel_wp(1.2, over_body, "let it stop swinging"),
        funnel_wp(2.2, (bottle[0], bottle[1], SEAT_BODY_Z + 0.020), "spout into the neck"),
        funnel_wp(2.4, seat_body, "cone down on the lip"),
        funnel_wp(1.6, seat_body, "let it settle"),
        funnel_wp(1.2, seat_body, "let go", FINGER_OPEN),
        funnel_wp(2.0, over_body, "back off", FINGER_OPEN),
        funnel_wp(2.0, funnel_high, "stand clear", FINGER_OPEN),
    ]
    waiting = sum(w.duration for w in left)
    right = [
        # Nothing to pour into until the left arm has seated the funnel.
        jug_wp(waiting, jug_high, 0.0, "wait for the funnel", FINGER_OPEN),
        jug_wp(1.8, jug_rest, 0.0, "down round the jug", FINGER_OPEN),
        jug_wp(1.2, jug_rest, 0.0, "take the jug"),
        jug_wp(2.4, (jug_rest[0], jug_rest[1], CARRY_Z), LEVEL_TILT, "lift it clear"),
        jug_wp(3.0, (POUR_XY[0], -0.160, approach_z), LEVEL_TILT, "over to the bottle"),
        # Square over the funnel before any of the tilt goes on.
        jug_wp(2.6, (*POUR_XY, POUR_Z), LEVEL_TILT, "square over the funnel"),
        # Slowly.
        jug_wp(4.4, (*POUR_XY, POUR_Z), POUR_TILT, "tip it out"),
        jug_wp(3.6, (*POUR_XY, POUR_Z), POUR_TILT, "let it all run in"),
        jug_wp(1.8, (POUR_XY[0], -0.160, approach_z), LEVEL_TILT, "come level"),
        jug_wp(2.4, (jug_rest[0], jug_rest[1], CARRY_Z), LEVEL_TILT, "stand clear"),
    ]
    return {"left": left, "right": right}
