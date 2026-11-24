"""On the station table, 30 cm out: a section of kitchen worktop, 30 cm deep, running left to right and ending in an
open edge on the robot's right. On its left end a sloping steel drainboard. A brown glass beer bottle has just been
knocked over on the drainboard: it lies on its side, across the worktop, and is rolling down towards the worktop and
along it towards the open edge on the right. Don't let it fall.
"""

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
import mechanism as mech  # noqa: E402
from mechanism import Joint, Link, Shape, mechanism  # noqa: E402
from parts import Part, Piece  # noqa: E402
from yam_arena.constants import PAD_TIP_PAST_TCP  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

WORKTOP_GREY, EDGE_GREY, STEEL, BROWN_GLASS, NECK_GLASS = (0.82, 0.80, 0.76), (0.55, 0.54, 0.52), \
    (0.72, 0.73, 0.75), (0.36, 0.20, 0.07), (0.30, 0.17, 0.06)
MATERIALS = {"default": (1.2, 1.0)}


def _m(v):
    return tuple(x / 1000 for x in v)


# -- the worktop: fixed. A Mechanism with no moving links, so its reset hook can clear the "fell" record ------------
WORKTOP_XY = (0.30, 0.05)
WORKTOP = (300.0, 550.0, 60.0)  # x, y, height: its top 6 cm above the table, its open end at y -22.5 cm
WORKTOP_TOP = TOP + WORKTOP[2] / 1000
OPEN_END_Y = WORKTOP_XY[1] - WORKTOP[1] / 2000

# WHY the edge trim on a stiff joint: a Mechanism with no joints fails to build in Isaac Lab ("resolve joint names",
# official sim, 2026-11-14), and the reset hook that clears the "fell" record needs a Mechanism.
# WHY the top 6 mm as a fixed box over the mechanism's slab: a capsule rolling on an articulation link here gained
# speed on the level, about 0.06 m/s per second (a stopped bottle rolled off again); on a fixed box it keeps its speed
# (0.297 to 0.292 m/s over the worktop with no rolling resistance; official sim, 2026-11-17)
SLAB_BELOW = 6.0  # mm
TRIM = (300.0, 4.0, 59.5)  # 0.5 mm below the worktop's top, so it is no lip
COUNTER = mechanism(
    "worktop", (*WORKTOP_XY, TOP),
    base=Link("worktop_slab", 20.0, (
        Shape("box", (0.0, 0.0, (WORKTOP[2] - SLAB_BELOW) / 2000),
              _m((WORKTOP[0], WORKTOP[1], WORKTOP[2] - SLAB_BELOW)), WORKTOP_GREY, contact_offset=0.002),
    )),
    links=(Link("edge_trim", 0.05, (
        Shape("box", (0.0, -WORKTOP[1] / 2000 + TRIM[1] / 2000, TRIM[2] / 2000), _m(TRIM), EDGE_GREY,
              contact_offset=0.002),)),),
    joints=(Joint("trim_mount", "prismatic", "edge_trim", (0.0, -WORKTOP[1] / 2000, 0.0), "Z", 0.0, 0.0001,
                  stiffness=1e5, damping=100.0),),
    materials=MATERIALS)
# the worktop's top 6 mm, which everything rests and rolls on: a fixed box
WORKTOP_SURFACE = Box("worktop_top", pos=(WORKTOP_XY[0], WORKTOP_XY[1], WORKTOP_TOP - SLAB_BELOW / 2000),
                      size=(WORKTOP[0] / 1000, WORKTOP[1] / 1000, SLAB_BELOW / 1000), color=WORKTOP_GREY)

# -- the drainboard: a heavy steel tray resting on the worktop's left end, sloping down towards the right ------------
# WHY 2.2 degrees, the bottle let go 13 cm up it: a 5 mm drop, which sends a real full 330 ml bottle onto the worktop
# at about 0.24-0.27 m/s (from its energy: glass rolling, beer mostly sliding). At 1.2 degrees from 12 cm (2.5 mm) a
# real bottle reaches at most about 0.20 m/s, so the earlier setup could not be repeated for real (review of TK-BM9BK5).
# At 3 degrees the bottle was over the edge 1.0 s into the trial, before an arm could get there (official sim,
# 2026-11-14).
DRAIN_DEG = 2.2
DRAIN_LOW_Y, DRAIN_HIGH_Y = 0.18, 0.32  # where it meets the worktop, and its high end
DRAIN_T = 3.0  # mm
DRAIN_KG = 15.0  # heavy enough that nothing in the task shifts it


def _drain_part():
    th = math.radians(DRAIN_DEG)
    half = (DRAIN_HIGH_Y - DRAIN_LOW_Y) / 2
    centre_y = (DRAIN_LOW_Y + DRAIN_HIGH_Y) / 2
    # the slab's underside at its low end rests on the worktop
    centre_z = WORKTOP_TOP + half * math.sin(th) + DRAIN_T / 2000 / math.cos(th)
    slab = Piece((0.0, 0.0, 0.0), (WORKTOP[0] / 1000, 2 * half / math.cos(th), DRAIN_T / 1000), rot=(th, 0.0, 0.0))
    # a block under the high end holds it at its slope
    y_support = half - 0.008
    underside = y_support * math.tan(th) - DRAIN_T / 2000 / math.cos(th)  # in the part's frame
    support_h = underside + (centre_z - WORKTOP_TOP)
    support = Piece((0.0, y_support, underside - support_h / 2), (WORKTOP[0] / 1000, 0.012, support_h))
    return Part("drainboard", pos=(WORKTOP_XY[0], centre_y, centre_z), size=(WORKTOP[0] / 1000, 2 * half, 0.012),
                mass=DRAIN_KG, color=STEEL, pieces=(slab, support))


DRAINBOARD = _drain_part()


def drain_top(y: float) -> float:
    """The drainboard's top surface height over the worktop at world y."""
    th = math.radians(DRAIN_DEG)
    return WORKTOP_TOP + DRAIN_T / 1000 / math.cos(th) + (y - DRAIN_LOW_Y) * math.tan(th)


# -- the bottle: a 330 ml glass beer bottle, full, lying across the worktop (its axis along x) ------------------------
BOTTLE_R, BODY_LEN = 0.0305, 0.140  # a capsule: 61 mm across, 201 mm long with its rounded ends
NECK_R, NECK_LEN = 0.012, 0.040
BOTTLE_KG = 0.53  # about 200 g of glass and 330 g of beer
# WHY a rolling resistance: PhysX has none, and a real bottle on a worktop slows by about 0.02-0.05 m/s each second.
# 0.05 m/s^2 (a 0.026 N push against its motion) slows it by about 0.038 m/s each second on the worktop (official sim,
# 2026-11-17: 0.04 gave 0.031, 0.06 gave 0.045). No PhysX damping: angular damping 0.3 made the stopped bottle bounce
# off the fingers (official sim, 2026-11-14).
BOTTLE_DAMPING = None
BOTTLE_ROLLING_RESISTANCE = 0.05
# Let go at rest 13 cm up the drainboard (its middle over y 0.31): in a real trial it is held against a straightedge
# laid across the drainboard just below it, and the straightedge is lifted away.
RELEASE_RUN = 0.13
BOTTLE_START = (0.29, DRAIN_LOW_Y + RELEASE_RUN)


def _bottle():
    body = Piece((0.0, 0.0, 0.0), (BOTTLE_R, BODY_LEN), kind="capsule", axis="X")
    neck = Piece((BODY_LEN / 2 + BOTTLE_R + NECK_LEN / 2 - 0.004, 0.0, 0.0), (NECK_R, NECK_LEN), NECK_GLASS,
                 kind="capsule", axis="X")
    x, y = BOTTLE_START
    return Part("bottle", pos=(x, y, drain_top(y) + BOTTLE_R + 0.001),
                size=(BODY_LEN + 2 * BOTTLE_R + NECK_LEN, 2 * BOTTLE_R, 2 * BOTTLE_R), mass=BOTTLE_KG,
                color=BROWN_GLASS, pieces=(body, neck), angular_damping=BOTTLE_DAMPING,
                rolling_resistance=BOTTLE_ROLLING_RESISTANCE)


BOTTLE = _bottle()
OBJECTS = [COUNTER, DRAINBOARD, BOTTLE, WORKTOP_SURFACE]

# -- scoring ----------------------------------------------------------------------------------------------------------
# The bottle may end lying, standing (stood up by the robot) or anywhere in between, on the worktop or still on the
# drainboard: any of those, still and never over the edge, is the bottle stopped.
SPAN_HALF = 0.1245  # half the bottle's whole length (body caps to neck tip); the reported box's centre is its middle
REST_TOL = 0.010  # resting on the surface under it: its lowest point within 1 cm of it (the estimate below is rough
# where the bottle leans)
FELL_BELOW = 0.020  # its lowest point this far below the worktop's top: it has gone over an edge
COMPLETE_SCORE = 0.9
_RECORD = {"fell": False}


@mech.on_reset
def _new_trial():
    """Every reset puts the bottle back on the drainboard: a new trial, nothing has fallen yet."""
    _RECORD["fell"] = False


def _rot(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


# WHY from the centre and the tilt, not the reported bounds: the bounds the simulator gives a lying capsule sit about
# 1 cm low and jitter (official sim, 2026-11-14). Lying, the box's centre is on the body's axis (the lowest point a
# radius below); standing, it is the middle of the bottle's length (half its length below).
def lowest_point(bottle) -> float:
    upright = min(1.0, abs(float(_rot(bottle.quat_xyzw)[2, 0])))  # the axis's vertical part: 0 lying, 1 standing
    return float(bottle.pos[2]) - upright * SPAN_HALF - math.sqrt(1.0 - upright * upright) * BOTTLE_R


def surface_under(bottle) -> float:
    y = float(bottle.pos[1])
    return drain_top(min(y, DRAIN_HIGH_Y)) if y >= DRAIN_LOW_Y else WORKTOP_TOP


def over_the_top(bottle) -> bool:
    """Its middle over the worktop (or the drainboard on it), not past an edge."""
    x, y = float(bottle.pos[0]), float(bottle.pos[1])
    half_x = WORKTOP[0] / 2000
    return abs(x - WORKTOP_XY[0]) < half_x and OPEN_END_Y < y < DRAIN_HIGH_Y


def on_top(bottle) -> bool:
    """Resting on the worktop or the drainboard (not held up off it), its middle over it."""
    return over_the_top(bottle) and abs(lowest_point(bottle) - surface_under(bottle)) <= REST_TOL


def _note(bottle) -> None:
    if lowest_point(bottle) < WORKTOP_TOP - FELL_BELOW:
        _RECORD["fell"] = True  # once over an edge it counts as broken, whatever happens after


def _grade(objects) -> float:
    bottle = objects["bottle"]
    _note(bottle)
    if _RECORD["fell"]:
        return 0.0
    # WHY no credit for "on it but still rolling": at the start the bottle is already that, and a capsule rolling on
    # this simulator's level worktop doesn't slow down, so it is not a part-stopped bottle
    return 1.0 if on_top(bottle) and bottle.resting() else 0.0


def score(objects):
    """0.9 for the bottle stopped and still on the worktop or drainboard, lying or standing, never having gone over an
    edge (1.0 once success() has held 1 s); 0 otherwise: still rolling, over an edge at any point (even if put back),
    held up, or anywhere else."""
    return min(_grade(objects), COMPLETE_SCORE)


def success(objects):
    """The bottle still on the worktop or drainboard, never having gone over an edge. Must hold for 1 s."""
    return _grade(objects) == 1.0


# -- the scripted demo: the right arm puts its closed fingers in the bottle's path -----------------------------------
# Open loop, from the fixed start: the right arm comes down with its fingers closed (their 26 mm width across the
# bottle's path) and stands them on the worktop ahead of the bottle; it rolls into them and stops, and the fingers stay
# there until the trial ends on success.
# where the fingers' near face stands (the bottle's middle stops a radius short of it); 4 cm from the open end
BLOCK_Y = -0.185
FINGERS_HALF_T = 0.0063  # closed fingertips: 12.6 mm thick along y
TIPS_ABOVE = 0.004  # the fingertips this far above the worktop
DEMO_AIM_SHIFT_MM = 0.0  # the block this far off (+x and +y), for margin runs
ARRIVE_S, WAIT_S = 1.1, 4.0


def block_tcp():
    s = DEMO_AIM_SHIFT_MM / 1000
    return (BOTTLE_START[0] + s, BLOCK_Y - FINGERS_HALF_T + s, WORKTOP_TOP + TIPS_ABOVE + PAD_TIP_PAST_TCP)


def _above(p, dz):
    return (p[0], p[1], p[2] + dz)


def demo():
    block = block_tcp()
    right = [
        Waypoint(ARRIVE_S, _above(block, 0.03), 0.0, "fingers closed, down ahead of the bottle"),
        Waypoint(0.5, block, 0.0, "stand them on the worktop in its path"),
        Waypoint(WAIT_S, block, 0.0, "let it roll in and stop; keep the fingers there"),
    ]
    left = [Waypoint(1.0, (0.22, 0.30, TOP + 0.20), 0.0, "stay clear")]
    return {"left": left, "right": right}
