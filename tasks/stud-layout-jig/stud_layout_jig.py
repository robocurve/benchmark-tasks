"""On the station table: a framing layout jig and four loose 2x4 cripple studs.

The jig is a 3/4-inch plywood base with a 2x4 end fence standing on edge across its far side (away from the
arms). Four slots are laid out 12 inches on centre across the jig. Each slot is a pair of 3/4 x 1-1/2-inch
cleats, 6 inches long, standing at the fence. The gap between each pair is 44.1 mm, so a 2x4 (38.1 mm thick)
fits with 3 mm to spare on each side. The top of each cleat is rounded over toward its slot with a
3/4-inch (19 mm) radius, so a board set down on a cleat's edge can slide off it into the slot.

The four studs are surfaced 2x4 lumber (1-1/2 x 3-1/2 inches actual, cut to 14-1/2 inches, about 0.56 kg).
They start standing on edge and a little off square. Two are staged on the plywood in the open strip between
the two middle slots, and one stands on the table beside each side of the jig.

Goal: seat every stud on edge in its own slot, flat on the plywood, and tight against the end fence.

Run the scripted demo:
    inspect-robots run --embodiment yam_arena --policy yam_scripted \\
        -E task=stud_layout_jig.py \\
        -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
        --instruction "seat the four 2x4 studs on edge in the layout jig, tight against the end fence" \\
        --scorer task_score --max-steps 1000 --store-frames \\
        --log-dir ~/yam-eval/stud-layout-jig -E headless=false

The scripted plan is open loop and uses only the fixed constants in this file, so run it with
``-E object_jitter=0``. Each arm seats the two slots on its own side. For each stud, the arm grips the top of the
stud across its 38 mm thickness, turns it square to the slot, and carries it over the slot about 10 mm short of
the fence. It lowers the stud until it is just above the cleats, slides it forward until its end presses against
the fence, and lets go. The stud drops into the slot, the rounded cleat tops steer it in, and its end slides down
the fence face. Landing can nudge the stud a few millimetres back, so the arm then comes down behind the stud's
near end with the fingers closed and gives it a short push until it stops against the fence.

Fidelity notes. The lumber, plywood, cleats and fence are boxes at their real dimensions. Lumber density is taken
as about 450 kg/m^3 (spruce-pine-fir). The platform has no tilted or curved shapes, so each cleat's rounded top is
built from a row of overlapping spheres on a lower box. The spheres also bulge about 19 mm past each cleat's outer
face, away from its slot. Left out: wood grain, the eased (rounded) edges of real dimensional lumber, and nailing.
This task covers layout only; fastening is a separate step. Every free object has the platform's fixed friction
(about 1.2), which is higher than wood on wood (about 0.4-0.5); a board only slides on the steepest part of the
rounded cleat tops. The slots are spaced at 12 inches on centre rather than 16, because four slots at 16 inches
do not fit the table. 12 inches is also a standard framing spacing.
"""

import math

import numpy as np

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box, Sphere

SCENE = "station"
TOP = TABLE_TOP[SCENE]
INCH = 0.0254

# -- lumber (ALSC PS 20 dressed sizes) --------------------------------------------------
STUD_THICK = 1.5 * INCH  # 38.1 mm, across the slot (y when seated)
STUD_DEPTH = 3.5 * INCH  # 88.9 mm, vertical when on edge
STUD_LENGTH = 14.5 * INCH  # 368.3 mm, along the slot (x when seated)
WOOD_DENSITY = 450.0  # kg/m^3, spruce-pine-fir near 12 % moisture
STUD_MASS = WOOD_DENSITY * STUD_THICK * STUD_DEPTH * STUD_LENGTH  # ~0.56 kg
STUD_COLOR = (0.87, 0.76, 0.56)

# -- jig -------------------------------------------------------------------------------
PLY_THICK = 0.75 * INCH  # 19.05 mm
CLEAT_W = 0.75 * INCH  # 19.05 mm across y
CLEAT_H = 1.5 * INCH  # 38.1 mm tall, to the top of the rounded cap
CLEAT_L = 6.0 * INCH  # 152.4 mm along x
ROUND_R = CLEAT_W  # roundover radius: the cap curves from the cleat's outer edge down to its slot-side face
CLEAT_BOX_H = CLEAT_H - ROUND_R  # the square lower part of the cleat, 19.05 mm
ROUND_SPHERES = 16  # spheres per cleat; ~9 mm apart, so the ridges between them are ~0.5 mm
# 3 mm per side: the fallback agreed before building. At 1.5 mm a stud cocked more than ~1.1 degrees touched both
# cleats and wedged when pushed; at 3 mm that takes ~2.3 degrees.
SLOT_CLEARANCE = 0.003  # per side
SLOT_W = STUD_THICK + 2 * SLOT_CLEARANCE  # 44.1 mm
ON_CENTRE = 12.0 * INCH  # 304.8 mm
SLOT_Y = (-1.5 * ON_CENTRE, -0.5 * ON_CENTRE, 0.5 * ON_CENTRE, 1.5 * ON_CENTRE)

BASE_TOP = TOP + PLY_THICK
FENCE_FACE_X = 0.56  # the fence face the stud ends butt against (keeps every grip within the arms' reach)
FENCE_T = STUD_THICK  # a 2x4 on edge: 38.1 mm along x
FENCE_H = STUD_DEPTH  # 88.9 mm tall
CLEAT_NEAR_X = FENCE_FACE_X - CLEAT_L
JIG_NEAR_X = 0.12
JIG_FAR_X = FENCE_FACE_X + FENCE_T
JIG_HALF_Y = max(abs(y) for y in SLOT_Y) + SLOT_W / 2 + 2 * CLEAT_W + 0.015
PLY_COLOR = (0.80, 0.66, 0.44)
CLEAT_COLOR = (0.62, 0.47, 0.28)
FENCE_COLOR = (0.70, 0.56, 0.36)

SEATED_X = FENCE_FACE_X - STUD_LENGTH / 2
SEATED_Z = BASE_TOP + STUD_DEPTH / 2


def _jig():
    parts = [
        Box(
            "jig_base",
            pos=((JIG_NEAR_X + JIG_FAR_X) / 2, 0.0, TOP + PLY_THICK / 2),
            size=(JIG_FAR_X - JIG_NEAR_X, 2 * JIG_HALF_Y, PLY_THICK),
            color=PLY_COLOR,
        ),
        Box(
            "end_fence",
            pos=(FENCE_FACE_X + FENCE_T / 2, 0.0, BASE_TOP + FENCE_H / 2),
            size=(FENCE_T, 2 * JIG_HALF_Y, FENCE_H),
            color=FENCE_COLOR,
        ),
    ]
    cleat_x = FENCE_FACE_X - CLEAT_L / 2
    box_offset = SLOT_W / 2 + CLEAT_W / 2
    # Sphere centres run along the cleat's outer top edge at the top of the square part, so each sphere meets the
    # slot-side face tangentially at that height and the slot keeps its full width all the way down.
    round_offset = SLOT_W / 2 + CLEAT_W
    round_z = BASE_TOP + CLEAT_BOX_H
    round_xs = np.linspace(CLEAT_NEAR_X, FENCE_FACE_X - ROUND_R, ROUND_SPHERES)
    for i, y in enumerate(SLOT_Y, start=1):
        for side, sign in (("a", -1.0), ("b", 1.0)):
            parts.append(
                Box(
                    f"cleat_{i}{side}",
                    pos=(cleat_x, y + sign * box_offset, BASE_TOP + CLEAT_BOX_H / 2),
                    size=(CLEAT_L, CLEAT_W, CLEAT_BOX_H),
                    color=CLEAT_COLOR,
                )
            )
            for k, rx in enumerate(round_xs):
                parts.append(
                    Sphere(
                        f"cleat_{i}{side}_round_{k:02d}",
                        pos=(float(rx), y + sign * round_offset, round_z),
                        radius=ROUND_R,
                        color=CLEAT_COLOR,
                    )
                )
    return parts


# Starting layout, all on edge and turned a little off square (x, y, yaw, bottom height). Two studs are staged
# on the plywood in the open strip between the inner slots. One stands on the table beside each side of the jig.
START = {
    "stud_1": (0.30, 0.585, -0.10, TOP),
    "stud_2": (0.36, 0.05, 0.08, BASE_TOP),
    "stud_3": (0.36, -0.05, -0.06, BASE_TOP),
    "stud_4": (0.30, -0.585, 0.12, TOP),
}
STUDS = tuple(
    Box(
        name,
        pos=(x, y, bottom + STUD_DEPTH / 2),
        size=(STUD_LENGTH, STUD_THICK, STUD_DEPTH),
        mass=STUD_MASS,
        color=STUD_COLOR,
        yaw=yaw,
    )
    for name, (x, y, yaw, bottom) in START.items()
)
OBJECTS = [*_jig(), *STUDS]

# -- scoring -------------------------------------------------------------------------------
SEAT_CREDIT = 0.15
FLUSH_CREDIT = 0.10
HEIGHT_TOLERANCE = 0.003  # bottom on the plywood
ANGLE_TOLERANCE = math.radians(3)  # upright on edge, and square to the fence
FLUSH_TOLERANCE = 0.002  # end against the fence face
MIN_ENGAGEMENT = CLEAT_L / 2  # the stud runs at least this far between its cleats
SLOT_SLACK = 0.001  # allowance past the cleat faces
QUAT_NORM_TOLERANCE = 1e-3
OUTLINE_TOLERANCE = 0.002  # built box vs the spec, per axis
STUD_HALF = np.array([STUD_LENGTH, STUD_THICK, STUD_DEPTH]) / 2


def _finite(state) -> bool:
    fields = (state.pos, state.quat_xyzw, state.lin_vel, state.ang_vel, state.low, state.high)
    if not all(np.all(np.isfinite(f)) for f in fields):
        return False
    return abs(float(np.linalg.norm(state.quat_xyzw)) - 1.0) <= QUAT_NORM_TOLERANCE


def _rotation(state) -> np.ndarray:
    x, y, z, w = (float(v) for v in state.quat_xyzw)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ]
    )


def _valid(objects) -> bool:
    """Fail-safe: every stud is present with finite values and a unit quaternion, and its built outline (the
    world-aligned box the simulator reports) matches a 1-1/2 x 3-1/2 x 14-1/2-inch box at its pose."""
    for stud in STUDS:
        state = objects.get(stud.name)
        if state is None or not _finite(state):
            return False
        expected = 2 * np.abs(_rotation(state)) @ STUD_HALF
        built = np.asarray(state.high, dtype=float) - np.asarray(state.low, dtype=float)
        if np.any(np.abs(built - expected) > OUTLINE_TOLERANCE):
            return False
    return True


def _axes(state):
    """The stud's long (local x) and depth (local z) axes in the world frame."""
    rot = _rotation(state)
    return rot[:, 0], rot[:, 2]


def _slot_of(state):
    """The index of the slot this stud is seated in, or None."""
    long_axis, depth_axis = _axes(state)
    upright = abs(depth_axis[2]) >= math.cos(ANGLE_TOLERANCE)
    square = abs(long_axis[0]) >= math.cos(ANGLE_TOLERANCE)
    on_base = abs(float(state.low[2]) - BASE_TOP) <= HEIGHT_TOLERANCE
    engaged = float(state.high[0]) >= FENCE_FACE_X - CLEAT_L + MIN_ENGAGEMENT
    if not (upright and square and on_base and engaged and state.resting()):
        return None
    # Where the stud's centreline crosses the middle of the cleats; it must lie between that slot's cleat faces.
    cleat_mid_x = FENCE_FACE_X - CLEAT_L / 2
    centre_y = float(state.pos[1]) + long_axis[1] / long_axis[0] * (cleat_mid_x - float(state.pos[0]))
    for index, slot_y in enumerate(SLOT_Y):
        if abs(centre_y - slot_y) <= SLOT_CLEARANCE + SLOT_SLACK:
            return index
    return None


def _flush(state) -> bool:
    return abs(float(state.high[0]) - FENCE_FACE_X) <= FLUSH_TOLERANCE


def _results(objects):
    """(seated, flush) per stud, each slot credited to at most one stud."""
    used = set()
    out = []
    for stud in STUDS:
        state = objects[stud.name]
        slot = _slot_of(state)
        if slot is None or slot in used:
            out.append((False, False))
            continue
        used.add(slot)
        out.append((True, _flush(state)))
    return out


def score(objects):
    """0.15 per stud seated on edge in its own slot, plus 0.10 more when that stud is also tight to the fence."""
    if not _valid(objects):
        return 0.0
    return float(sum(SEAT_CREDIT * seated + FLUSH_CREDIT * flush for seated, flush in _results(objects)))


def success(objects):
    """All four studs are seated on edge in separate slots and tight against the end fence."""
    return _valid(objects) and all(seated and flush for seated, flush in _results(objects))


def distance(objects):
    """Mean distance from each stud's centre to the nearest seated position, in metres."""
    if not _valid(objects):
        return float("inf")
    targets = [np.array([SEATED_X, y, SEATED_Z]) for y in SLOT_Y]
    return float(
        np.mean([min(np.linalg.norm(objects[s.name].pos - t) for t in targets) for s in STUDS])
    )


# -- scripted demo ------------------------------------------------------------------------
GRIP_DEPTH = 0.035  # pad tips this far below the stud's top edge (16 mm above the cleats when seated)
TRAVEL_Z = TOP + 0.15  # a carried stud's bottom clears the cleat tops by about 26 mm
SHORT_OF_FENCE = 0.010  # the stud is lowered over its slot this far back from the fence
DROP_GAP = 0.005  # the stud is let go with its bottom this far above the cleat tops
# Before letting go, the held stud (still above the cleats, so nothing drags on it) is slid forward until its far
# end meets the fence. The slide is commanded this far past the fence so the springy arm ends pressing the stud
# against it. A stud pushed a long way along the slot after the drop can bind between the cleats instead of sliding.
FENCE_PRESS = 0.020  # the springy arm lagged up to ~13 mm short of an 8 mm press in physics
# Landing on the rounded cleat tops can nudge the stud a few millimetres back off the fence, so after the drop the
# closed fingers give its near end a short push. The stud is already square in its slot by then; a long, hard push
# on a stud cocked more than ~1.1 degrees wedges it between the cleats instead.
PUSH_BEHIND = 0.040  # the closed fingers come down this far behind the seated near end
PUSH_PAST = 0.030  # and push to this far past it; the fence stops the stud and the springy arm presses it there
PUSH_TIP_HEIGHT = 0.035  # finger tips this far above the plywood while pushing
ASSIGNMENT = {  # stud -> (arm, slot index), in order per arm; the arms work the centre strip at different times
    "stud_1": ("left", 3),
    "stud_2": ("left", 2),
    "stud_3": ("right", 1),
    "stud_4": ("right", 0),
}


def _tcp_z(bottom_z: float) -> float:
    return bottom_z + STUD_DEPTH - GRIP_DEPTH + PAD_TIP_PAST_TCP


LOWER_X = SEATED_X - SHORT_OF_FENCE
FENCE_X = SEATED_X + FENCE_PRESS
SEATED_NEAR_END = FENCE_FACE_X - STUD_LENGTH


def _seat_waypoints(stud: Box, slot_y: float) -> list[Waypoint]:
    sx, sy, sz = stud.pos
    pick_z = _tcp_z(sz - STUD_DEPTH / 2)
    drop_z = _tcp_z(BASE_TOP + CLEAT_H + DROP_GAP)
    behind_x = SEATED_NEAR_END - PUSH_BEHIND
    pushed_x = SEATED_NEAR_END + PUSH_PAST
    push_z = BASE_TOP + PUSH_TIP_HEIGHT + PAD_TIP_PAST_TCP
    # yaw 0 closes the fingers along world y; a stud with yaw psi needs its thickness axis across the fingers.
    grip_yaw = stud.yaw
    return [
        Waypoint(1.5, (sx, sy, TRAVEL_Z), FINGER_OPEN, "over the stud", grip_yaw),
        Waypoint(1.5, (sx, sy, pick_z), FINGER_OPEN, "descend", grip_yaw),
        Waypoint(0.8, (sx, sy, pick_z), FINGER_CLOSED, "close", grip_yaw),
        Waypoint(1.5, (sx, sy, TRAVEL_Z), FINGER_CLOSED, "lift", grip_yaw),
        Waypoint(2.0, (LOWER_X, slot_y, TRAVEL_Z), FINGER_CLOSED, "carry and square up", 0.0),
        Waypoint(1.5, (LOWER_X, slot_y, drop_z), FINGER_CLOSED, "just above the cleats", 0.0),
        Waypoint(1.2, (FENCE_X, slot_y, drop_z), FINGER_CLOSED, "slide up against the fence", 0.0),
        Waypoint(0.5, (FENCE_X, slot_y, drop_z), FINGER_CLOSED, "hold against the fence", 0.0),
        Waypoint(0.5, (FENCE_X, slot_y, drop_z), FINGER_OPEN, "drop it into the slot", 0.0),
        Waypoint(1.0, (FENCE_X, slot_y, TRAVEL_Z), FINGER_OPEN, "up", 0.0),
        Waypoint(1.2, (behind_x, slot_y, TRAVEL_Z), FINGER_CLOSED, "behind the near end", 0.0),
        Waypoint(1.0, (behind_x, slot_y, push_z), FINGER_CLOSED, "down to push height", 0.0),
        Waypoint(3.0, (pushed_x, slot_y, push_z), FINGER_CLOSED, "nudge it to the fence", 0.0),
        Waypoint(0.5, (pushed_x, slot_y, push_z), FINGER_CLOSED, "hold against the fence", 0.0),
        # Back straight off at push height before lifting, so the fingers do not drag up the stud's end face.
        Waypoint(1.0, (behind_x, slot_y, push_z), FINGER_CLOSED, "back off", 0.0),
        Waypoint(1.0, (behind_x, slot_y, TRAVEL_Z), FINGER_CLOSED, "retreat", 0.0),
    ]


def demo():
    """Each arm seats the two slots on its own side: the left arm outer slot first, the right arm inner slot
    first, so the two arms never work the centre strip at the same time."""
    plans = {"left": [], "right": []}
    by_name = {s.name: s for s in STUDS}
    for name, (arm, slot) in ASSIGNMENT.items():
        plans[arm].extend(_seat_waypoints(by_name[name], SLOT_Y[slot]))
    return plans
