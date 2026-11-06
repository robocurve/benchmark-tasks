"""Geometry of the bottling bench for ``funnel_beads.py``: the bottle, the beads and their jug, the
funnel and the two wrong tools. Pure Python (NumPy only): no Isaac Sim, so the task file stays
importable before the simulator starts.

Frame: x forward from the arms, y left, z up, metres; ``top`` is the bench surface.

Everything that has to be hollow is built here from primitives, because a stock task file offers one
convex collider per object and a convex hull of a bottle is a solid lump. A ring of thin staves is a
hollow cylinder; a ring of sloping plates is a cone. ``simkit.Piece`` carries a rotation for exactly
this, and ``simkit.Part`` welds the lot into one rigid body.

Why the task needs a funnel
---------------------------
* **The bottle is bolted down and its mouth is narrow** -- a 40 mm bore at the top of a 35 mm
  neck, against 9 mm beads. It cannot be picked up and it cannot be filled by eye: a jug poured at
  that mouth, aimed as well as the arm can aim it, puts the beads on the bench.
* **The funnel is the only thing that turns a wide stream into a narrow one.** Its cone is 110 mm
  across at the rim and tapers to a 29 mm spout that drops into the neck.
* **Nothing else on the bench will do.** The jug is the only other thing that can be picked up, and
  it is where the beads already are.
"""

from __future__ import annotations

import dataclasses
import math

import numpy as np

import simkit
from simkit import Part, Piece

# -- look -------------------------------------------------------------------------------

# Light colours and a real brushed-metal MDL on the funnel. Isaac's darker MDLs render almost
# black from the top camera, which is why the beads and the vessels keep plain surfaces.
GLASS_GREEN = (0.42, 0.74, 0.52)  # the bottle
BOTTLE_OPACITY = 0.22  # see-through, so the beads can be watched arriving
STEEL = (0.86, 0.88, 0.90)  # the jug
COPPER = (0.88, 0.52, 0.26)  # the funnel -- the one tool that works
HANDLE = (0.22, 0.24, 0.28)
BEAD_COLOUR = (0.07, 0.20, 0.80)  # deep blue: amber beads vanished against the orange funnel

# -- the bottle -------------------------------------------------------------------------

BODY_R = 0.100  # inner radius: a 208 mm body
WALL = 0.004
BODY_H = 0.055
SHOULDER_T = 0.005
NECK_R = 0.020  # a 40 mm bore
NECK_H = 0.035  # long enough to guide the spout straight
BODY_STAVES = 56
SHOULDER_SPOKES = 56  # matches BODY_STAVES, so the cap lines up with the staves
NECK_STAVES = 24
# A lip flared out of the top of the bore, so a spout arriving off-centre is guided in.
MOUTH_LEAD_R = 0.028
MOUTH_LEAD_H = 0.014
# Dead centre of the bench.
BOTTLE_XY = (0.330, 0.000)

# -- beads ------------------------------------------------------------------------------

BEAD_R = 0.0045  # 9 mm: the whole bottle is sized backwards from this
# Plastic, not glass: a glass charge put enough torque on the pads to twist the jug out of them.
BEAD_MASS = 0.0018  # polypropylene, 900 kg/m3
BEAD_COUNT = 24
BEAD_GRID = (4, 3, 2)  # two layers: the jug is narrow now
BEAD_PITCH = 0.010
# Slippery, so a pile of them runs rather than arching across the funnel's throat.
BEAD_FRICTION = (0.06, 0.04)

# -- the jug the beads start in ----------------------------------------------------------

JUG_R = 0.040  # 88 mm outside, inside a 94 mm jaw
JUG_H = 0.045  # short, so the mouth stays close to the grip when the jug goes over
JUG_MASS = 0.050
JUG_STAVES = 24
JUG_XY = (0.330, -0.250)
JUG_WALL = 0.004
# The jug is a tapered cup: cylindrical outside for the jaws, a cone inside. A straight-sided cup
# does not empty -- at 90 degrees it is a horizontal tube and the beads have nowhere to run.
JUG_BORE = 0.026  # the cone's radius where it meets the floor: wide enough to stand the beads on
JUG_PLATES = 20
# Slick, and only on the lining. The outside keeps the grippy default the jaws need.
JUG_LINING = (0.04, 0.03)
# The funnel wants the same: a grippy spout catches on the rim and seats askew, and a grippy cone
# holds the beads on its wall instead of running them to the throat.
FUNNEL_SLIDE = (0.06, 0.04)
HANDLE_GRIP = simkit.FRICTION["rubber"]
JUG_OPACITY = 0.22  # see-through, so the beads inside are visible on camera

# -- the funnel ---------------------------------------------------------------------------

CONE_TOP_R = 0.055  # a 110 mm mouth: the whole point of the tool
CONE_BOT_R = 0.018  # wider than the spout below it, so nothing can perch on the join
CONE_H = 0.036
CONE_PLATES = 16  # more than this and they overlap into ledges near the throat
SPOUT_R = 0.0145  # a 29 mm bore: about three bead diameters, so they cannot bridge it
SPOUT_H = 0.055  # right through the neck and on into the bottle, without touching its floor
SPOUT_STAVES = 20
SPOUT_WALL = 0.002  # thin, so the neck can close down round it and still leave the bore
PLUG = 0.001  # the root bead that carries the body, on the axis and out of the way
# The handle: a post beside the cone, with an arm joining it to the funnel.
POST_SIDE = 0.024  # what the jaws close on
POST_H = 0.060
POST_R = 0.081
POST_FOOT = 0.010
POST_ARM_T = 0.006  # the arm joining the post to the cone, so the handle reads as part of it
FUNNEL_XY = (0.420, 0.205)  # on the left arm's side, clear of the bottle
# The post is placed at +y in the funnel's own geometry rather than by turning the whole part:
# a Part's yaw rotates its body, and reading the rest pose back showed the pieces had not followed.
FUNNEL_YAW = 0.0
FUNNEL_MASS = 0.028  # a thin pressing
# What keeps the tall, narrow tools upright before they are picked up.
STAND_HOLD = simkit.Anchor(break_force=1.5, break_torque=8.0)

FINGER_PER_M = 0.04695 / 0.094
GRIP_SQUEEZE = 0.009  # the funnel -- the same bite that holds the jug
JUG_SQUEEZE = 0.009
TOUCH_OFFSET = 0.001
# A rubbery handle, not bare steel: the pads have to hold a jug of beads out at arm's length.
JUG_GRIP = (1.6, 1.4)


def finger_for(opening: float) -> float:
    """The ``Waypoint.finger`` target that holds the jaws ``opening`` metres apart."""
    return -max(opening, 0.0) * FINGER_PER_M


def _ring(count: int, radius: float, height: float, z: float, wall: float = WALL,
          overlap: float = 1.0, colour=None) -> list[Piece]:
    """A hollow cylinder as ``count`` tangential staves at ``radius`` (inner), centred on ``z``."""
    mid = radius + wall / 2
    chord = 2 * math.pi * (radius + wall) / count * overlap
    out = []
    for i in range(count):
        theta = 2 * math.pi * i / count
        out.append(
            Piece(
                "box",
                (wall, chord, height),
                (mid * math.cos(theta), mid * math.sin(theta), z),
                rot=(0.0, 0.0, theta),
                color=colour,
            )
        )
    return out


def _spokes(count: int, inner: float, outer: float, thickness: float, z: float,
            overlap: float = 1.0, colour=None) -> list[Piece]:
    """A flat annulus as ``count`` radial plates: the bottle's shoulder, closing it down to a neck."""
    mid = (inner + outer) / 2
    chord = 2 * math.pi * mid / count * overlap
    out = []
    for i in range(count):
        theta = 2 * math.pi * i / count
        out.append(
            Piece(
                "box",
                (outer - inner, chord, thickness),
                (mid * math.cos(theta), mid * math.sin(theta), z),
                rot=(0.0, 0.0, theta),
                color=colour,
            )
        )
    return out


def _cone(count: int, top_r: float, bottom_r: float, height: float, z: float,
          thickness: float = 0.003, overlap: float = 1.0, colour=None) -> list[Piece]:
    """A hollow cone as ``count`` sloping plates, its mouth at ``z + height/2``.

    Each plate lies along the cone's slant. Pitching a plate by 90 degrees plus the cone's own
    half-angle turns its long axis from horizontal onto that slant; yawing then carries it round.
    """
    slant = math.hypot(top_r - bottom_r, height)
    half_angle = math.atan2(top_r - bottom_r, height)
    mid = (top_r + bottom_r) / 2
    chord = 2 * math.pi * top_r / count * overlap
    out = []
    for i in range(count):
        theta = 2 * math.pi * i / count
        out.append(
            Piece(
                "box",
                (slant, chord, thickness),
                (mid * math.cos(theta), mid * math.sin(theta), z),
                rot=(0.0, math.pi / 2 + half_angle, theta),
                color=colour,
            )
        )
    return out


def spout_outer_r() -> float:
    """The spout's outside radius. Its wall is thinner than the rest, so the neck can close down
    round it: everything about the bottle's mouth is measured from this."""
    return SPOUT_R + SPOUT_WALL


def funnel_grip_z(top: float) -> float:
    """TCP for a grip round the funnel's post: the pads land on it, the wrist is upright."""
    from yam_arena.constants import PAD_TIP_PAST_TCP

    # Pad tips just above the post's foot, so the pads cover its lower third.
    return funnel_post_bottom(top) + 0.012 + PAD_TIP_PAST_TCP


def funnel_grip_opening() -> float:
    """How far apart the jaws close on the funnel's post."""
    return POST_SIDE - GRIP_SQUEEZE


def jug_grip_z(top: float) -> float:
    """TCP for a grip round the jug's body, pads landing on the lower half of its wall."""
    from yam_arena.constants import PAD_TIP_PAST_TCP

    return top + 0.004 + JUG_H * 0.35 + PAD_TIP_PAST_TCP


def jug_grip_opening() -> float:
    """How far apart the jaws close on the jug's body."""
    return 2 * (JUG_R + JUG_WALL) - JUG_SQUEEZE





def funnel_grip_xy() -> tuple[float, float]:
    """Where the jaws close: on the post, just outboard of the cone's rim."""
    return (FUNNEL_XY[0], FUNNEL_XY[1] + POST_R)


def jug_grip_xy() -> tuple[float, float]:
    """Straight over the jug: the jaws close round its body, so there is no offset at all."""
    return JUG_XY





# -- builders ---------------------------------------------------------------------------


def funnel_rest_body_z(top: float) -> float:
    """The funnel standing on the bench, held upright by its anchor."""
    return top + SPOUT_H / 2


def funnel_post_bottom(top: float) -> float:
    """Where the post's foot sits when the funnel is standing on the bench."""
    return funnel_rest_body_z(top) + POST_FOOT


def bottle_object(top: float) -> Part:
    """The bottle: a hollow body, a floor, a shoulder closing down to a neck, and the neck.

    No mass, so it is a fixed collider -- it cannot be picked up, tipped, or carried to the beads.
    """
    floor_t = 0.004
    body_z = floor_t + BODY_H / 2
    shoulder_z = floor_t + BODY_H + SHOULDER_T / 2
    bore_h = NECK_H - MOUTH_LEAD_H
    neck_z = floor_t + BODY_H + SHOULDER_T + bore_h / 2
    lead_z = floor_t + BODY_H + SHOULDER_T + bore_h + MOUTH_LEAD_H / 2
    return Part(
        "bottle",
        pos=(BOTTLE_XY[0], BOTTLE_XY[1], top + floor_t / 2),
        size=(BODY_R + WALL, floor_t, BODY_R + WALL),
        exact_z=True,
        shape="cylinder",
        color=GLASS_GREEN,
        material="acrylic",
        opacity=BOTTLE_OPACITY,
        collision="convexHull",
        contact_offset=TOUCH_OFFSET,
        pieces=(
            *_ring(BODY_STAVES, BODY_R, BODY_H, body_z - floor_t / 2),
            *_ring(SHOULDER_SPOKES, NECK_R, SHOULDER_T, shoulder_z - floor_t / 2,
                   wall=BODY_R + WALL + 0.003 - NECK_R),
            *_ring(NECK_STAVES, NECK_R, bore_h, neck_z - floor_t / 2),
            *_cone(NECK_STAVES, MOUTH_LEAD_R, NECK_R, MOUTH_LEAD_H, lead_z - floor_t / 2),
        ),
    )


def jug_object(top: float) -> Part:
    """The open jug the beads start in: a plain hollow cup, gripped round its body."""
    floor_t = 0.004
    return Part(
        "jug",
        pos=(JUG_XY[0], JUG_XY[1], top + floor_t / 2),
        size=(JUG_R + JUG_WALL, floor_t, JUG_R + JUG_WALL),
        shape="cylinder",
        exact_z=True,
        mass=JUG_MASS,
        color=STEEL,
        friction=JUG_GRIP,
        restitution=0.05,
        collision="convexHull",
        contact_offset=TOUCH_OFFSET,
        opacity=JUG_OPACITY,
        pieces=tuple(_ring(JUG_STAVES, JUG_R, JUG_H, JUG_H / 2, wall=JUG_WALL))
        + tuple(
            dataclasses.replace(piece, friction=JUG_LINING)
            for piece in _cone(JUG_PLATES, JUG_R, JUG_BORE, JUG_H, JUG_H / 2)
        ),
    )


def funnel_object(top: float) -> Part:
    """The funnel: a cone over a spout, with a post beside it for the jaws."""
    spout_z = SPOUT_H / 2
    cone_z = SPOUT_H + CONE_H / 2
    return Part(
        "funnel",
        pos=(FUNNEL_XY[0], FUNNEL_XY[1], funnel_rest_body_z(top)),
        size=(PLUG, PLUG, PLUG),
        exact_z=True,
        shape="sphere",
        mass=FUNNEL_MASS,
        color=COPPER,
        material="aluminium_brushed",
        friction=simkit.FRICTION["steel_on_steel"],
        restitution=0.05,
        collision="convexHull",
        contact_offset=TOUCH_OFFSET,
        pieces=(
            # The root cylinder is solid, so the spout is a ring of staves around it instead and the
            # root is only what carries the body.
            *(dataclasses.replace(piece, friction=FUNNEL_SLIDE)
              for piece in _ring(SPOUT_STAVES, SPOUT_R, SPOUT_H, 0.0, wall=SPOUT_WALL)),
            *(dataclasses.replace(piece, friction=FUNNEL_SLIDE)
              for piece in _cone(CONE_PLATES, CONE_TOP_R, CONE_BOT_R, CONE_H, cone_z - spout_z)),
            # An arm out to the post, and a collar where it meets the cone. Without them the post
            # reads as a loose block standing beside the funnel rather than a handle on it.
            Piece(
                "box",
                (POST_SIDE * 0.6, POST_R - CONE_BOT_R, POST_ARM_T),
                (0.0, (CONE_BOT_R + POST_R) / 2, POST_FOOT + POST_ARM_T / 2),
                color=HANDLE,
            ),
            Piece(
                "box",
                (POST_SIDE, POST_SIDE, POST_H),
                (0.0, POST_R, POST_FOOT + POST_H / 2),
                color=HANDLE,
                friction=HANDLE_GRIP,
            ),
        ),
        anchor=STAND_HOLD,
    )


def bead_objects(top: float) -> list[Part]:
    """``BEAD_COUNT`` beads resting on the jug's floor, in a fixed grid so the layout never varies."""
    nx, ny, _ = BEAD_GRID
    cx, cy = JUG_XY
    beads = []
    for i in range(BEAD_COUNT):
        layer, rem = divmod(i, nx * ny)
        row, col = divmod(rem, nx)
        beads.append(
            Part(
                f"bead{i:02d}",
                pos=(
                    cx + (col - (nx - 1) / 2) * BEAD_PITCH,
                    cy + (row - (ny - 1) / 2) * BEAD_PITCH,
                    top + 0.004 + BEAD_R + 0.001 + layer * BEAD_PITCH,
                ),
                size=(BEAD_R,) * 3,
                shape="sphere",
                exact_z=True,
                mass=BEAD_MASS,
                # No MDL here: Isaac's plastic renders near-black at this lighting and the beads
                # stopped reading through the jug, which is the one thing they have to do.
                color=BEAD_COLOUR,
                friction=BEAD_FRICTION,
                restitution=0.05,  # plastic, and they have to stay in the cone they land in
                contact_offset=TOUCH_OFFSET,
                collision="convexHull",
            )
        )
    return beads


# -- where things are ---------------------------------------------------------------------


def bottle_floor_z(top: float) -> float:
    return top + 0.004


def bottle_rim_z(top: float) -> float:
    """Top of the flared lip: the ring the funnel's cone comes to rest on. The flare is cut into
    the neck, so this is the top of the neck -- the mouth is no higher for having a lip."""
    return bottle_mouth_z(top)


def funnel_seat_body_z(top: float) -> float:
    """Where the funnel comes to rest: the cone jams on the bottle's lip, spout down the neck."""
    up_the_cone = (MOUTH_LEAD_R - CONE_BOT_R) / (CONE_TOP_R - CONE_BOT_R) * CONE_H
    return bottle_rim_z(top) - SPOUT_H / 2 - up_the_cone


def funnel_spout_tip_z(top: float) -> float:
    return funnel_seat_body_z(top) - SPOUT_H / 2


def bottle_mouth_z(top: float) -> float:
    """Top of the neck: where the funnel's spout has to go in."""
    return top + 0.004 + BODY_H + SHOULDER_T + NECK_H


def bottle_shoulder_z(top: float) -> float:
    return top + 0.004 + BODY_H


def jug_rim_z(top: float) -> float:
    return top + 0.004 + JUG_H


def funnel_rim_above_body() -> float:
    """Height of the funnel's rim above its own origin."""
    return SPOUT_H / 2 + CONE_H


def beads_in_bottle(objects, top: float) -> int:
    """Beads resting inside the bottle: within its inner radius and below its shoulder.

    Below the *shoulder*, not the mouth, so a bead still in the neck or in the funnel's spout does
    not count until it has actually dropped into the body.
    """
    cx, cy = BOTTLE_XY
    lid = bottle_shoulder_z(top)
    n = 0
    for i in range(BEAD_COUNT):
        p = objects[f"bead{i:02d}"].pos
        if (p[0] - cx) ** 2 + (p[1] - cy) ** 2 <= BODY_R**2 and top <= p[2] <= lid:
            n += 1
    return n


def beads_still(objects, max_speed: float = 0.02) -> bool:
    """Whether every bead has settled."""
    return all(
        float(np.linalg.norm(objects[f"bead{i:02d}"].lin_vel)) < max_speed for i in range(BEAD_COUNT)
    )


def body_tilt_deg(state) -> float:
    """How far a body's own up axis has tipped from vertical, in degrees."""
    x, y, z, w = (float(v) for v in state.quat_xyzw)
    up_z = 1.0 - 2.0 * (x * x + y * y)
    return math.degrees(math.acos(max(-1.0, min(1.0, up_z))))
