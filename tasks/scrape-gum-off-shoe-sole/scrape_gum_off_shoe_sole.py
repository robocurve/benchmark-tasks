"""A sneaker lies sole-up on the table with a flat piece of pink chewing gum stuck to the ball of its sole. A plastic
putty knife lies to the right. Get the gum off the sole.

The gum is a trodden wad 4 cm across on the smooth disc of the sole, domed: 3.5 mm thick in the middle, 2 mm at
its rim. It peels: it is eight strips lying across the sole, hinged to each other, so it bends and lifts as one sheet.
Fingers can't shift it, and neither can a blade pushed against it or dragged over it: a strip only lets go once the
knife's edge is down on the sole and slid under it, laid low (gum.py has the rule). The shoe weighs 350 g and slides
if it isn't held, so one arm steadies it while the other scrapes.
"""

from __future__ import annotations

import math

import numpy as np
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))  # labkit.py and assets/ sit next to this file
from labkit import asset_model  # noqa: E402

import gum
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]
INSTRUCTION = "Scrape the chewing gum off the sole of the shoe with the putty knife."

# -- the shoe: a men's size 9 sneaker, sole-up, heel toward the left arm ---------------------------------------------
SHOE_SIZE = (0.285, 0.0999, 0.098)      # length, width, height as it lies (asset sneaker-sole-up)
SHOE_XY, SHOE_YAW = (0.36, 0.06), -math.pi / 2
SOLE_LOCAL_Z = SHOE_SIZE[2] / 2          # the sole's surface, in the shoe's frame
PIVOT_LOCAL = (0.0456, -0.0003)          # centre of the smooth disc on the ball of the sole, in the shoe's frame
SHOE = asset_model("shoe", "sneaker-sole-up", pos=(*SHOE_XY, TOP + SHOE_SIZE[2] / 2), yaw=SHOE_YAW, sha256="9d97ab3a5d8ef3632ac8b1046429a834cfe1849404bcdba32b1b599c647c604a")

# -- the putty knife: 19.5 cm, 5 cm plastic blade toward the shoe ----------------------------------------------------
KNIFE_SIZE = (0.195, 0.050, 0.016)
KNIFE = asset_model("knife", "putty-knife", pos=(0.30, -0.27, TOP + KNIFE_SIZE[2] / 2), yaw=math.pi / 2, sha256="abeca7c71ef899443d02e902809f4bedfcf6d7ccc1e38320198b2d9f42570b9b")

# -- the gum: a trodden wad, 4 cm across and domed, on the smooth disc of the sole ------------------------------------
GUM = dict(
    piece_size=0.005,        # m: each strip is 5 mm wide (along the shoe)
    radius=0.020,            # m: the wad is 4 cm across (the smooth disc under it is 4.4 cm)
    rim_thickness=0.0015,    # m: how thick the wad is at its rim ...
    dome=0.002,              # m: ... and how much thicker at its very centre
    mass=0.001,              # kg per strip (a real 4 g wad is 0.5 g a strip; 1 g keeps the solver steady)
    color=(0.95, 0.45, 0.65),
    gap=0.001,               # m: the wad's underside stands this far off the sole (the give of real gum at its edge),
    #                          so a blade laid on the sole gets under it; one that rides higher only bumps into it
    # an unlocked strip is tied to its place by a spring and lets go once drawn `let_go` from it: gum stretches first
    tether_k=800.0,          # N/m (not calibrated against real gum): about 1.6 N a strip at the let-go stretch
    tether_c=4.0,            # N s/m
    tether_kr=0.0005,        # N m/deg: it barely resists tilting up the blade
    tether_cr=0.00002,       # N m s/deg
    let_go=0.002,            # m
    cohesion_force=20.0,     # N: what tears two neighbouring strips apart at their hinge
    cohesion_torque=1.0,     # N m
    hinge_axis="Y",          # the strips' long axis, in their own frame: the wad curls about it as it lifts
    hinge_limit_deg=80.0,
    # when the knife counts as getting under a strip (gum.engaged)
    under=0.0022,            # m: the edge is no higher above the sole than this (the gap, and a little)
    max_angle_deg=35.0,      # blade to sole: steeper than this digs in or skips over
    reach=0.007,             # m: edge to strip centre, along the sole
    edge_slack=0.0015,       # m: how far above the strip's top the edge may ride (the `under` limit is the tighter)
    # the knife's working edge and blade direction, in the knife's frame (from the model)
    edge=((0.0975, -0.025, -0.0076), (0.0975, 0.025, -0.0076)),
    blade_dir=(1.0, 0.0, 0.0),
    sole_z=SOLE_LOCAL_Z,
)
GUM_STRIPS = 8               # strips across the wad, each lying across the shoe


def _shoe_to_world(local_xy, z_local):
    c, s = math.cos(SHOE_YAW), math.sin(SHOE_YAW)
    x, y = local_xy
    return (SHOE_XY[0] + c * x - s * y, SHOE_XY[1] + s * x + c * y, TOP + SHOE_SIZE[2] / 2 + z_local)


def gum_thickness(r: float) -> float:
    """How thick the wad is `r` m from its centre: a low dome, rim_thickness at the rim."""
    return round(GUM["rim_thickness"] + GUM["dome"] * max(0.0, 1.0 - (r / GUM["radius"]) ** 2), 5)


GUM_PIECES, GUM_LOCAL = [], {}
for _k in range(GUM_STRIPS):
    _dx = (_k - (GUM_STRIPS - 1) / 2) * GUM["piece_size"]          # along the shoe
    _chord = round(2.0 * math.sqrt(GUM["radius"] ** 2 - _dx ** 2), 4)   # the round wad's width at this strip
    _t = gum_thickness(abs(_dx))
    _name = f"gum_{_k}"
    _local = (PIVOT_LOCAL[0] + _dx, PIVOT_LOCAL[1])
    GUM_LOCAL[_name] = (*_local, SOLE_LOCAL_Z + GUM["gap"] + _t / 2)
    GUM_PIECES.append(Box(_name, pos=_shoe_to_world(_local, SOLE_LOCAL_Z + GUM["gap"] + _t / 2),
                          size=(GUM["piece_size"], _chord, _t), mass=GUM["mass"], color=GUM["color"], yaw=SHOE_YAW))
GUM["neighbours"] = tuple((f"gum_{k}", f"gum_{k + 1}") for k in range(GUM_STRIPS - 1))
gum.setup(GUM, GUM_LOCAL)

OBJECTS = [SHOE, KNIFE, *GUM_PIECES]

# -- scoring ----------------------------------------------------------------------------------------------------------
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
OFF_SOLE = 0.006             # m: a piece this far above the sole's surface, or past its edge, is off the sole
TABLE_X, TABLE_Y = (-0.19, 0.91), (-0.65, 0.65)
ON_TABLE_Z = 0.006


def on_sole(objects, name) -> bool:
    """Gum still on the sole: the piece's centre is over the sole and within OFF_SOLE of its surface, wherever on
    the sole it has been pushed to (smeared along the sole is not removed). The shoe is solid under its sole, so gum
    lower than that has dropped off the side onto the table."""
    local = gum.to_local(objects["shoe"], objects[name].pos)
    over = abs(local[0]) <= SHOE_SIZE[0] / 2 and abs(local[1]) <= SHOE_SIZE[1] / 2
    return bool(over and abs(local[2] - SOLE_LOCAL_Z) <= OFF_SOLE)


def removed_count(objects) -> int:
    return sum(not on_sole(objects, p.name) for p in GUM_PIECES)


def on_table(state) -> bool:
    return bool(abs(state.low[2] - TOP) <= ON_TABLE_Z and TABLE_X[0] <= state.low[0] and state.high[0] <= TABLE_X[1]
                and TABLE_Y[0] <= state.low[1] and state.high[1] <= TABLE_Y[1])


def score(objects) -> float:
    """Nine tenths shared over the wad's eight strips, by how many are off the sole (0.9 with all eight off), 0 if
    the shoe has left the table or anything but the blade moved gum (gum.fouled)."""
    gum.update(objects)
    if not on_table(objects["shoe"]) or gum.fouled():
        return 0.0
    return math.floor(9 * removed_count(objects) / len(GUM_PIECES)) / 10


def success(objects) -> bool:
    """Full marks: no gum on the sole, only ever moved by the blade, the shoe and the knife both back on the table,
    let go and still."""
    gum.update(objects)
    return bool(not gum.fouled() and removed_count(objects) == len(GUM_PIECES)
                and on_table(objects["shoe"]) and on_table(objects["knife"])
                and objects["shoe"].resting() and objects["knife"].resting())


# -- the scripted demo --------------------------------------------------------------------------------------------------
DEMO_EDGE_FROM_TCP = (-0.0137, 0.1394, -0.0152)   # m: the edge's middle from the TCP, knife held flat by its handle
DEMO_TILT_DEG = 6.0                                # blade to sole in the stroke: all the arm reaches at this height
DEMO_PRESS = 0.003                                 # m: the stroke aims the edge this far into the sole, so it stays down
DEMO_YAW_DEG = -15.0                               # the blade turned about the vertical in the stroke (negative: its tip away
#                                                    from the robot), so the peeled wad slides off the near side of the sole


def demo():
    """Right arm: take the putty knife by its handle, lay its edge on the sole short of the gum, push it under the
    wad in one stroke, draw back, lay the knife down where it was and let go. Left arm: its closed gripper goes up
    against the far end of the shoe before the stroke, as a stop to scrape against, and leaves when the stroke is
    done."""
    from labkit import Pose
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
    from yam_arena.demo import Waypoint, grip_rotation
    from yam_arena.urdf import axis_angle_matrix

    turn = math.pi / 2                       # fingers close along world x, across the handle's 28 mm width
    hx, hy = KNIFE.pos[0], KNIFE.pos[1] - 0.045   # the handle's centre (the knife lies along y, blade toward +y)
    grip_z, up = TOP + 0.015, TOP + 0.15
    ex, ey, ez = DEMO_EDGE_FROM_TCP
    tilt = math.radians(DEMO_TILT_DEG)
    turned = (axis_angle_matrix(np.array([0.0, 0.0, 1.0]), math.radians(DEMO_YAW_DEG))
              @ axis_angle_matrix(np.array([1.0, 0.0, 0.0]), -tilt))                   # handle up, edge down, then the yaw
    rot = turned @ grip_rotation(turn)
    ox, oy, oz = turned @ np.array([ex, ey, ez])                                      # the edge from the TCP, so held
    gum_x, gum_y, _ = _shoe_to_world(PIVOT_LOCAL, SOLE_LOCAL_Z)
    sole = TOP + SHOE_SIZE[2]
    low, clear = sole - DEMO_PRESS, sole + 0.012
    y0, y1 = gum_y - 0.05, gum_y + 0.045     # the edge's run: 5 cm short of the wad's middle to 4.5 cm past it

    def at(edge_y, edge_height, secs, label):
        return Pose(secs, (gum_x - ox, edge_y - oy, edge_height - oz), FINGER_CLOSED, label, rot=rot)

    right = [
        Waypoint(2.0, (hx, hy, TOP + 0.12), FINGER_OPEN, "over the knife's handle", turn),
        Waypoint(2.0, (hx, hy, grip_z), FINGER_OPEN, "down onto the handle", turn),
        Waypoint(1.0, (hx, hy, grip_z), FINGER_CLOSED, "grip", turn),
        Waypoint(2.0, (hx, hy, up), FINGER_CLOSED, "lift the knife", turn),
        Waypoint(3.0, (gum_x, y0 - ey, up), FINGER_CLOSED, "carry it to just short of the gum", turn),
        Waypoint(2.0, (gum_x, y0 - ey, up), FINGER_CLOSED, "wait for the left hand", turn),
        at(y0, clear, 2.5, "tilt the blade, edge above the sole"),
        at(y0, low, 2.0, "set the edge on the sole"),
        at(y1, low, 7.0, "push the edge under the gum"),
        at(y1, low, 1.0, "pause at the end of the stroke"),
        at(y0, low + 0.007, 4.0, "draw the knife back"),
        at(y0, clear, 2.0, "lift the knife away"),
        Waypoint(3.0, (hx, hy, up), FINGER_CLOSED, "carry the knife back", turn),
        Waypoint(2.5, (hx, hy, grip_z + 0.003), FINGER_CLOSED, "lay it on the table", turn),
        Waypoint(1.0, (hx, hy, grip_z + 0.003), FINGER_OPEN, "let go", turn),
        Waypoint(2.0, (hx, hy, TOP + 0.12), FINGER_OPEN, "right hand clear", turn),
        Waypoint(3.0, (hx, hy, TOP + 0.12), FINGER_OPEN, "hold", turn),
    ]
    toe_y = SHOE_XY[1] + SHOE_SIZE[0] / 2
    stop = (SHOE_XY[0], toe_y + 0.010, TOP + 0.05)
    back = (stop[0], stop[1] + 0.06)
    left = [
        Waypoint(3.0, (*back, TOP + 0.12), FINGER_CLOSED, "left hand closed, over the far end of the shoe"),
        Waypoint(2.5, (*back, stop[2]), FINGER_CLOSED, "down behind the shoe's far end"),
        Waypoint(2.5, stop, FINGER_CLOSED, "up against the shoe: a stop to scrape against"),
        Waypoint(24.0, stop, FINGER_CLOSED, "hold the shoe"),
        Waypoint(2.0, (*back, stop[2]), FINGER_CLOSED, "left hand off the shoe"),
        Waypoint(2.0, (*back, TOP + 0.12), FINGER_CLOSED, "left hand clear"),
        Waypoint(6.0, (*back, TOP + 0.12), FINGER_CLOSED, "hold"),
    ]
    return {"right": right, "left": left}
