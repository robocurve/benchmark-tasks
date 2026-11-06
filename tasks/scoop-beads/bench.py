"""Geometry of the bead-transfer bench for ``scoop_beads.py``: the two bowls, the beads, the scoop
and the one wrong tool. Pure Python (NumPy only): no Isaac Sim, so the task file stays importable
before the simulator starts.

The bowls are real models from Arena's object library, fixed to the bench. That is a measured
choice, not a convenience: a probe dropped nine beads into each of three library bowls and all nine
settled inside every time, so their colliders really are concave shells rather than filled hulls.
Everything else is a :class:`simkit.Part`, which is how the scoop can be one rigid body welded from
five pieces -- a stock task file allows one primitive collider per object.

Frame: x forward from the arms, y left, z up, metres; ``top`` is the bench surface.

The layout, and why
-------------------
* **Both bowls are bolted down**, so neither can be picked up and tipped: the beads have to be
  carried. They straddle the middle of the bench, 250 mm apart about its centre, and each has a
  60 mm rim, so beads cannot be swept from one to the other either.
* **The tools sit on a rack nearer the arms**, held clear of the bench so the jaws have air to close
  in. A handle lying flat on a surface cannot be gripped: the pads reach down beside it, hit the
  bench and shove the tool away instead of holding it -- measured at 9 mm of shove and no grip.
* **The scoop is the only tool that holds beads.** Its pan has walls on three sides and an open lip,
  and a handle the gripper can close on. The flat spatula has no walls at all, so beads roll off it.
* **The scoop is held on one heading throughout, ``SCOOP_YAW``.** The wrist has only 11 to 20
  degrees of tilt about one horizontal axis at pouring height but 69 to 89 about the other
  (measured by ramping the tilt through warm-started IK), so which way the scoop points decides
  whether it can be emptied at all -- and, at a bowl across the midline, whether the dig pose can be
  held. 225 degrees is the one heading that serves both bowls, so the wrist never rolls while the
  scoop is loaded.
"""

from __future__ import annotations

import math

import numpy as np

from yam_arena.constants import PAD_TIP_PAST_TCP
from yam_arena.task_file import Asset

import simkit
from simkit import Lifted, Part, Piece

# -- look -------------------------------------------------------------------------------

RACK_GREY = (0.33, 0.35, 0.38)
SCOOP_STEEL = (0.78, 0.80, 0.82)
DISH_CREAM = (0.86, 0.84, 0.78)  # the source tray, pale against the amber beads
BEAD_COLOUR = (0.90, 0.52, 0.06)  # amber, so the beads read against the pale bowls

# -- bowls ------------------------------------------------------------------------------
# Both from Arena's object library, fixed (no mass). Different models so the two read apart.

# The beads start on a wide, shallow **dish** and have to end up in a deep **bowl**.
TARGET_BOWL = "bowl_ycb_robolab"
# 170 mm, not 220. The pair has to come closer together than 300 mm -- see the separation below --
# and two vessels 115 and 110 mm in radius will not fit any closer with bench left between them.
TARGET_EXTENT = 0.170
_YCB = TARGET_EXTENT / 0.220  # measured at 220 mm wide and scaled with the asset
TARGET_RIM = 0.0759 * _YCB
TARGET_FLOOR = 0.0029 * _YCB
TARGET_INNER_R = 0.085 * _YCB

SOURCE_R = 0.110  # a 220 mm floor: room to dig, and the same footprint as the bowl it feeds
SOURCE_WALL = 0.005
# Taller than a bead, so the scoop corners them; low enough that the handle rides out over it.
SOURCE_WALL_H = 0.024
SOURCE_STAVES = 48
SOURCE_RIM = SOURCE_WALL_H
SOURCE_FLOOR = 0.004  # inner floor of the source tray, above the bench
SOURCE_INNER_R = SOURCE_R
SOURCE_FLAT_R = SOURCE_R - 0.010  # flat right out to the wall, being a built floor

# 240 mm apart, not 300: the pour has to be aimed short of the bowl, and at 300 the arm could
# not hold such a pose on any heading.
SOURCE_XY = (0.390, 0.104)
TARGET_XY = (0.270, -0.104)

# -- beads ------------------------------------------------------------------------------

BEAD_R = 0.008  # 16 mm: the tray's wall has to beat this, and the handle has to clear the wall
BEAD_MASS = 0.00536  # soda-lime glass, 2500 kg/m3
# A full tray. The charge is load-bearing: a scoop cannot lift a sphere off open floor, so what
# loads the pan is the heap behind the lip wedging the beads in front of it up into it.
BEAD_COUNT = 24
BEAD_GRID = (4, 6, 1)
BEAD_PITCH = 0.018  # just clear of touching, so the charge settles into a heap
BEAD_OFFSET = 0.034  # the pile sits this far along the draw, leaving clear floor to start the dig on

# -- the scoop --------------------------------------------------------------------------

# The pan has to fit inside SOURCE_FLAT_R with room left to be drawn through the beads: with the
# 80 x 70 mm pan this started with, a corner reached 80 mm from the bowl's centre by the end of
PAN_L = 0.070  # along the handle
PAN_W = 0.080
PAN_FLOOR = 0.002  # a thin blade: a thick leading edge ploughs beads along instead of lifting them
SCOOP_LIP = 0.040  # wall height on the three closed sides, enough to hold a 30 mm bead
# A bevelled lip was tried here and is deliberately not fitted.
SCOOP_WALL = 0.004
# The scoop is held by a straight handle running back from the pan, in the plane of the pads.
HANDLE_L = 0.290
HANDLE_W = 0.024  # what the jaws actually close on
HANDLE_T = 0.018
SCOOP_MASS = 0.150  # a little heavier than a bare pressing, as a weighted-handle ladle is
# Both tools stand on the arm's own side, clear of the bowls: the bench's middle is taken up by
# the two of them and their 110 mm radii leave no room in front.
SCOOP_XY = (0.200, 0.180)

# The scoop points along SCOOP_YAW -- the world direction from its grip to its pan -- and is
# tipped about the horizontal axis square to that.
SCOOP_YAW = math.radians(45.0)
# The handle carries a tall, narrow web where the jaws close.
GRIP_WEB_L = 0.070  # along the handle
GRIP_WEB_T = 0.048  # deep: this is what resists pitch
PAD_LINE = 0.014  # where the pad tips land up the web, above the handle's underside

HANDLE_DY = -PAN_L / 2 - HANDLE_L / 2 + 0.010
HANDLE_REACH = -HANDLE_DY

# No crank: the handle lies in the pan's plane, so the wrist's tilt *is* the pan's tilt.
CRANK = 0.0

GRIP_TO_PAN = HANDLE_REACH
PAN_BELOW_HANDLE = PAD_LINE - PAN_FLOOR / 2

# -- the wrong tool ------------------------------------------------------------------------

SPATULA_ASSET = "spatula_01_vomp_robolab"  # a real flat spatula: no walls, so beads roll off
SPATULA_EXTENT = 0.240
SPATULA_MASS = 0.080
SPATULA_XY = (0.156, 0.282)  # alongside the scoop, clear of it and of the arm's own base
SPATULA_YAW = SCOOP_YAW  # laid the same way round as the scoop, not across it

# -- the rack both tools sit on ------------------------------------------------------------

RACK_TOP = 0.040  # post tops above the bench
POST = 0.020

FINGER_PER_M = 0.04695 / 0.094  # finger-joint travel per metre of jaw opening
GRIP_SQUEEZE = 0.005
# PhysX generates contacts this far out. Arena's default is 5 mm, fine for finger pads on chunky
# objects but it means a 2 mm blade can never come closer than 5 mm to a bowl's floor.
TOUCH_OFFSET = 0.001


def finger_for(opening: float) -> float:
    """The ``Waypoint.finger`` target that holds the jaws ``opening`` metres apart."""
    return -max(opening, 0.0) * FINGER_PER_M


# -- builders ---------------------------------------------------------------------------


def bowl_objects(top: float):
    """The two vessels, fixed where they stand.

    The target is a library bowl -- deep, which is what keeps the beads once they land in it. The
    source is a round tray built here, because nothing in the library is the shape this end needs:
    its wall has to be taller than a bead, or the scoop rolls them over it, and low enough that the
    gripper clears it, and every bowl in the library is about a third as deep as it is wide. The
    tray is filled, not sparsely loaded, and that is load-bearing rather than decorative. A scoop
    cannot lift a sphere off open floor -- the lip pushes below its centre and it simply rolls --
    so what loads the pan is the beads behind the lip wedging the ones in front of it up the ramp.
    A full tray does that. Four beads alone, with nothing behind them, were picked up by nothing at
    all: square tray, round tray, bevelled lip, every one of them came out empty or spilled.
    """
    floor_t = SOURCE_FLOOR
    return [
        Part(
            "source_bowl",
            pos=(SOURCE_XY[0], SOURCE_XY[1], top + floor_t / 2),
            size=(SOURCE_R + SOURCE_WALL, floor_t, SOURCE_R + SOURCE_WALL),
            shape="cylinder",
            exact_z=True,
            color=DISH_CREAM,
            collision="convexHull",
            contact_offset=TOUCH_OFFSET,
            pieces=tuple(_ring(SOURCE_STAVES, SOURCE_R, SOURCE_WALL_H,
                               SOURCE_WALL_H / 2, wall=SOURCE_WALL)),
        ),
        Asset("target_bowl", asset=TARGET_BOWL, xy=TARGET_XY, extent=TARGET_EXTENT),
    ]


def bead_objects(top: float) -> list[Part]:
    """``BEAD_COUNT`` beads in one layer on the dish, in a fixed grid so the layout never varies.

    The grid is laid out along the scoop's own heading rather than the world axes, so the pile is
    square to the draw and a pass sweeps a whole row of it.
    """
    nx, ny, _ = BEAD_GRID
    beads = []
    for i in range(BEAD_COUNT):
        layer, rem = divmod(i, nx * ny)
        row, col = divmod(rem, nx)
        along = BEAD_OFFSET + (col - (nx - 1) / 2) * BEAD_PITCH
        across = (row - (ny - 1) / 2) * BEAD_PITCH
        x, y = _along(SOURCE_XY, along, SCOOP_YAW)
        x -= across * math.sin(SCOOP_YAW)
        y += across * math.cos(SCOOP_YAW)
        beads.append(
            Part(
                f"bead{i:02d}",
                pos=(
                    x,
                    y,
                    # Resting on the dish's measured floor, not dropped from above it: spawned high
                    # they start the trial outside the dish's own rim, and bounce on landing.
                    source_floor_z(top) + BEAD_R + 0.001 + layer * BEAD_PITCH,
                ),
                size=(BEAD_R,) * 3,
                shape="sphere",
                mass=BEAD_MASS,
                color=BEAD_COLOUR,
                friction=(0.35, 0.30),  # glass on glass, glass on ceramic
                restitution=0.08,  # they have to stay in the vessel they land in
                contact_offset=TOUCH_OFFSET,
                collision="convexHull",
            )
        )
    return beads


def scoop_object(top: float) -> Part:
    """The scoop: one rigid body welded from a pan floor, three walls and a handle."""
    cx, cy = SCOOP_XY
    lip_z = PAN_FLOOR / 2 + SCOOP_LIP / 2
    return Part(
        "scoop",
        pos=(cx, cy, scoop_body_z(top)),
        size=(PAN_L, PAN_W, PAN_FLOOR),
        mass=SCOOP_MASS,
        color=SCOOP_STEEL,
        material="steel",
        friction=simkit.FRICTION["steel_on_steel"],
        restitution=0.20,
        contact_offset=TOUCH_OFFSET,
        collision="convexHull",
        yaw=SCOOP_YAW,
        pieces=(
            # Back wall and two side walls; the front (local +x, the way SCOOP_YAW points) is the lip.
            Piece("box", (SCOOP_WALL, PAN_W, SCOOP_LIP), (-PAN_L / 2 + SCOOP_WALL / 2, 0.0, lip_z)),
            Piece("box", (PAN_L, SCOOP_WALL, SCOOP_LIP), (0.0, PAN_W / 2 - SCOOP_WALL / 2, lip_z)),
            Piece("box", (PAN_L, SCOOP_WALL, SCOOP_LIP), (0.0, -PAN_W / 2 + SCOOP_WALL / 2, lip_z)),
            Piece(
                "box",
                (HANDLE_L, HANDLE_W, HANDLE_T),
                (HANDLE_DY, 0.0, HANDLE_T / 2 - PAN_FLOOR / 2),
                color=(0.26, 0.28, 0.31),
                # A grip, so a grippy surface.
                friction=simkit.FRICTION["rubber"],
            ),
            Piece(
                "box",
                (GRIP_WEB_L, HANDLE_W, GRIP_WEB_T),
                (HANDLE_DY, 0.0, GRIP_WEB_T / 2 - PAN_FLOOR / 2),
                color=(0.26, 0.28, 0.31),
                friction=simkit.FRICTION["rubber"],
            ),
        ),
        # Balanced on the grip point, by mass rather than by bulk.
        com=(HANDLE_DY, 0.0, 0.0),
    )


def spatula_object(top: float) -> Lifted:
    """The spatula, raised onto its rack.

    A plain ``Asset`` is always settled onto the bench -- Arena solves its height from the model's
    own bounds -- so it ended up lying *under* its own posts. ``Lifted`` carries the rack height.
    """
    return Lifted(
        "spatula",
        asset=SPATULA_ASSET,
        xy=SPATULA_XY,
        extent=SPATULA_EXTENT,
        mass=SPATULA_MASS,
        yaw=SPATULA_YAW,
        lift=RACK_TOP,
    )


def _ring(count: int, radius: float, height: float, z: float, wall: float,
          colour=None) -> list[Piece]:
    """A hollow cylinder as ``count`` tangential staves at ``radius`` (inner), centred on ``z``."""
    mid = radius + wall / 2
    chord = 2 * math.pi * (radius + wall) / count
    return [
        Piece(
            "box",
            (wall, chord, height),
            (mid * math.cos(2 * math.pi * i / count), mid * math.sin(2 * math.pi * i / count), z),
            rot=(0.0, 0.0, 2 * math.pi * i / count),
            color=colour,
        )
        for i in range(count)
    ]


def _along(xy, distance: float, yaw: float) -> tuple[float, float]:
    """``distance`` metres from ``xy`` along a tool's own heading."""
    return (xy[0] + distance * math.cos(yaw), xy[1] + distance * math.sin(yaw))


def rack_objects(top: float) -> list[Part]:
    """Posts under each tool: two under the scoop's handle with a gap for the jaws, one under its
    pan, and two under the spatula.

    Each tool's posts run along its own heading. Posts laid out in a fixed world direction sit
    across a tool that points somewhere else, and hold nothing.
    """
    places = [
        _along(SCOOP_XY, HANDLE_DY - 0.032, SCOOP_YAW),
        _along(SCOOP_XY, HANDLE_DY + 0.032, SCOOP_YAW),
        _along(SCOOP_XY, PAN_L / 2 - 0.010, SCOOP_YAW),
        _along(SCOOP_XY, HANDLE_DY - HANDLE_L / 2 + 0.020, SCOOP_YAW),  # under the handle's tail
        _along(SPATULA_XY, -0.080, SPATULA_YAW),
        _along(SPATULA_XY, 0.080, SPATULA_YAW),
    ]
    return [
        Part(
            f"rack{i}",
            pos=(x, y, top + RACK_TOP / 2),
            size=(POST, POST, RACK_TOP),
            color=RACK_GREY,
            material="steel_painted",
            friction=simkit.FRICTION["steel_on_steel"],
            restitution=simkit.RESTITUTION["steel"],
            collision="convexHull",
        )
        for i, (x, y) in enumerate(places)
    ]


# -- where things are ---------------------------------------------------------------------


def scoop_body_z(top: float) -> float:
    """Height of the scoop's origin (its pan floor's centre) while it rests on its rack."""
    return top + RACK_TOP + PAN_FLOOR / 2


def scoop_grip_xy() -> tuple[float, float]:
    """Where to close the jaws on the handle: its midpoint, behind the pan along the heading."""
    return _along(SCOOP_XY, HANDLE_DY, SCOOP_YAW)


def scoop_grip_z(top: float) -> float:
    """Height of the handle's midline while the scoop rests on its rack."""
    return scoop_body_z(top) + PAN_BELOW_HANDLE


def _rot_z(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def scoop_tcp(pan_centre, tilt: float, psi: float = SCOOP_YAW) -> tuple[float, float, float]:
    """TCP that puts the scoop's pan floor centre at ``pan_centre``, tipped ``tilt`` rad.

    ``psi`` is the world direction from the grip to the pan -- the way the scoop is pointing. The
    grip is unchanged by it, so the arm may turn the scoop about the vertical freely between
    waypoints, and the tilt axis turns with it.
    """
    c, s = math.cos(tilt), math.sin(tilt)
    # The grip, in the pan's frame, turned into the world by the pan's own tilt.
    along = -(GRIP_TO_PAN * c - PAN_BELOW_HANDLE * s)
    up = GRIP_TO_PAN * s + PAN_BELOW_HANDLE * c
    # The pads reach down the *gripper's* axis, which the crank holds at a different angle from the
    # pan's: at a 34-degree crank, taking this from the pan's tilt instead misplaces them by 26 mm.
    g = tilt + CRANK
    along += PAD_TIP_PAST_TCP * math.sin(g)
    up += PAD_TIP_PAST_TCP * math.cos(g)
    return (
        pan_centre[0] + along * math.cos(psi),
        pan_centre[1] + along * math.sin(psi),
        pan_centre[2] + up,
    )


def pad_tips(pan_centre, tilt: float, psi: float = SCOOP_YAW) -> tuple[float, float, float]:
    """Where the finger pad tips sit when the pan is at ``pan_centre`` -- the lowest part of the hand.

    This is what has to stay outside a bowl while the pan is inside it.
    """
    c, s = math.cos(tilt), math.sin(tilt)
    back = -(GRIP_TO_PAN * c - PAN_BELOW_HANDLE * s)
    return (
        pan_centre[0] + back * math.cos(psi),
        pan_centre[1] + back * math.sin(psi),
        pan_centre[2] + GRIP_TO_PAN * s + PAN_BELOW_HANDLE * c,
    )


def scoop_rotation(tilt: float, down, psi: float = SCOOP_YAW):
    """The TCP rotation that points the scoop along ``psi`` and tips its *pan* by ``tilt``.

    The wrist is held a crank's worth steeper than the pan, which is what keeps the pads flat on
    the handle they are closed around.
    """
    c, s = math.cos(tilt + CRANK), math.sin(tilt + CRANK)
    tip = np.array([[1.0, 0.0, 0.0], [0.0, c, s], [0.0, -s, c]])  # about -x, the axis at psi = 90
    return _rot_z(psi - math.pi / 2) @ tip @ _rot_z(math.pi / 2) @ np.asarray(down)


def scoop_lip_rise(tilt: float) -> float:
    """How much higher the pan's centre sits than its leading lip, at a given tilt."""
    return PAN_L / 2 * math.sin(tilt)


def source_floor_z(top: float) -> float:
    """Inner floor of the source bowl: the height the scoop's blade has to run along."""
    return top + SOURCE_FLOOR


# What is left of the scoop sagging in the jaws once the grip web has done its work.
GRIP_SAG = 0.006
# Commanded below the floor, so the lip arrives on it within the seconds the dig has.
DIG_PRESS = 0.010


def dig_pan_z(top: float, tilt: float, press: float = DIG_PRESS) -> float:
    """Height for the pan floor's *centre* that puts its leading lip on the source bowl's floor.

    ``scoop_tcp`` positions the centre, and tipping nose-down drops the lip half a pan-length ahead
    of it, so the centre has to sit that much higher for the lip to reach the bottom of the bowl.

    ``press`` is how far below that the lip is commanded. A firm press is what gets the lip down
    onto the floor in the first place, but it is not something the arm can then drag: commanded to
    pull a pan pressed 10 mm into the dish across 143 mm of floor, the arm stalled outright and the
    pan did not move at all. The draw therefore eases the press off once the lip is down.
    """
    return source_floor_z(top) + PAN_FLOOR / 2 + scoop_lip_rise(tilt) - GRIP_SAG - press


def source_rim_z(top: float) -> float:
    return top + SOURCE_RIM


def target_rim_z(top: float) -> float:
    return top + TARGET_RIM


# -- measurements the scorer takes ------------------------------------------------------


def body_tilt_deg(state) -> float:
    """How far a body's own up axis has tipped from vertical, in degrees."""
    x, y, z, w = (float(v) for v in state.quat_xyzw)
    up_z = 1.0 - 2.0 * (x * x + y * y)
    return math.degrees(math.acos(max(-1.0, min(1.0, up_z))))


def beads_in(objects, xy, top: float) -> int:
    """How many bead centres are resting in a vessel: inside it, and no higher than its rim.

    The rim test is what stops a scoopful held over the bowl from counting before it is tipped in.
    """
    cx, cy = xy
    source = xy == SOURCE_XY
    inner = SOURCE_INNER_R if source else TARGET_INNER_R
    rim = top + (SOURCE_RIM if source else TARGET_RIM)
    n = 0
    for i in range(BEAD_COUNT):
        p = objects[f"bead{i:02d}"].pos
        if (p[0] - cx) ** 2 + (p[1] - cy) ** 2 <= inner**2 and top <= p[2] <= rim:
            n += 1
    return n


def beads_still(objects, max_speed: float = 0.02) -> bool:
    """Whether every bead has settled."""
    return all(
        float(np.linalg.norm(objects[f"bead{i:02d}"].lin_vel)) < max_speed for i in range(BEAD_COUNT)
    )
