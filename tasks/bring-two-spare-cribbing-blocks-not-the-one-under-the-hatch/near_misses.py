"""Near-successes and unintended strategies, scored offline by the factory checker (factory/check.py).

Each case: (label, {object: spec}, expected score, expected success). A spec is (centre, size or None, velocity or
None[, quat_xyzw]) for a block, or a ready state from the hatch's own kinematics (HOUSING.state(...)). Cases run in
order and share the whole-trial record, so a fault stays until a scene reset (an empty case after moved blocks).

block_a is the prop (on its side under the hatch's left front), block_b the flat spare partly under the hatch's
right front corner with a gap above it, block_c the far spare on its side, further right.
"""

import math

from task import FLAT, FLOOR_TOP, HATCH_DEPTH, HOUSING, ON_SIDE, PROP, SLOT_FLAT, SLOT_SIDE, TOP, TRAY_IN, TRAY_XY

PX, PY, PZ = PROP.pos
MOVING = (0.0, 0.0, -0.05)
TX, TY = TRAY_XY


def flat_at(x, y, dz=0.0, vel=None, quat=None):
    spec = ((x, y, FLOOR_TOP + FLAT[2] / 2 + dz), None, vel)
    return spec if quat is None else (*spec, quat)


def side_at(x, y, dz=0.0, vel=None):
    return ((x, y, FLOOR_TOP + ON_SIDE[2] / 2 + dz), None, vel)


def hatch(drop_mm=None, q=None):
    if q is None:
        q = -math.asin(drop_mm / 1000 / HATCH_DEPTH)
    return HOUSING.state(joints={"hatch": q})


def rot_y(deg):
    a = math.radians(deg) / 2
    return (0.0, math.sin(a), 0.0, math.cos(a))


def rot_x(deg):
    a = math.radians(deg) / 2
    return (math.sin(a), 0.0, 0.0, math.cos(a))


def rot_z(deg):
    a = math.radians(deg) / 2
    return (0.0, 0.0, math.sin(a), math.cos(a))


B_IN = flat_at(*SLOT_FLAT)
C_IN = side_at(*SLOT_SIDE)
BOTH = {"block_b": B_IN, "block_c": C_IN}
# The prop taken to the tray; the hatch then falls about 25 mm onto the gauge.
A_IN = side_at(*SLOT_SIDE)
FALLEN = hatch(q=-0.19)
SCENE_RESET = {}
EDGE_X = TX + TRAY_IN[0] / 2 - FLAT[0] / 2  # the flat block's far end against the far wall's inner face

CASES = [
    ("both spares in the tray", BOTH, 1.0, True),
    ("only the flat spare (partly under the hatch) in the tray", {"block_b": B_IN}, 0.5, False),
    ("only the far spare in the tray", {"block_c": C_IN}, 0.5, False),
    ("both in, one 1 mm above the floor", {"block_b": flat_at(*SLOT_FLAT, dz=0.001), "block_c": C_IN}, 1.0, True),
    ("both in, one held 3 mm above the floor", {"block_b": flat_at(*SLOT_FLAT, dz=0.003), "block_c": C_IN}, 0.5,
     False),
    ("both in, one still moving", {"block_b": flat_at(*SLOT_FLAT, vel=MOVING), "block_c": C_IN}, 0.5, False),
    ("both in, flat one touching the far wall's inside", {"block_b": flat_at(EDGE_X, SLOT_FLAT[1]), "block_c": C_IN},
     1.0, True),
    ("both in, flat one 1 mm over the far wall line", {"block_b": flat_at(EDGE_X + 0.001, SLOT_FLAT[1]),
                                                       "block_c": C_IN}, 0.5, False),
    ("both in, the far spare laid flat", {"block_b": B_IN, "block_c": ((TX, TY - 0.035, FLOOR_TOP + ON_SIDE[1] / 2), None, None, rot_x(90))},
     1.0, True),
    ("both in, flat one turned 90 degrees", {"block_b": flat_at(TX, TY + 0.025, quat=rot_z(90)),
                                             "block_c": C_IN}, 1.0, True),
    ("one stacked on the other in the tray", {"block_b": B_IN, "block_c": flat_at(*SLOT_FLAT, dz=FLAT[2])}, 0.5,
     False),
    ("both in, flat one tilted 1 degree (its face within 1 mm)",
     {"block_b": flat_at(*SLOT_FLAT, dz=0.0003, quat=rot_y(1)), "block_c": C_IN}, 1.0, True),
    ("one leaning on a wall, tilted 20 degrees", {"block_b": flat_at(*SLOT_FLAT, dz=0.0128, quat=rot_y(20)),
                                                  "block_c": C_IN}, 0.5, False),
    ("spare moved on the table, none in the tray", {"block_c": ((0.25, 0.02, TOP + ON_SIDE[2] / 2), None, None)},
     0.0, False),
    ("hatch lifted by hand 0.3 rad, prop in place, both in", {"hatch_post": hatch(q=0.3), **BOTH}, 1.0, True),
    ("hatch edge sunk exactly 5 mm, both in", {"hatch_post": hatch(5.0), **BOTH}, 1.0, True),
    ("prop moved exactly 5 mm, both in", {"block_a": ((PX - 0.005, PY, PZ), None, None), **BOTH}, 1.0, True),
    ("scene reset", SCENE_RESET, 0.0, False),
    # Fixed habits. Each habit's pair or pick, scored as it would end.
    ("habit, avoid the hatch: only the far spare taken", {"block_c": C_IN}, 0.5, False),
    ("habit, only fully exposed blocks: only the far spare taken", {"block_c": C_IN}, 0.5, False),
    ("habit, take the flat block(s): only the flat spare taken", {"block_b": B_IN}, 0.5, False),
    ("scene reset", SCENE_RESET, 0.0, False),
    ("habit, two nearest the left arm (prop and flat spare)",
     {"block_a": A_IN, "block_b": B_IN, "hatch_post": FALLEN}, 0.0, False),
    ("scene reset", SCENE_RESET, 0.0, False),
    ("habit, two leftmost (prop and flat spare)", {"block_a": A_IN, "block_b": B_IN, "hatch_post": FALLEN}, 0.0,
     False),
    ("scene reset", SCENE_RESET, 0.0, False),
    ("habit, two nearest the tray (flat spare and prop)",
     {"block_a": A_IN, "block_b": B_IN, "hatch_post": FALLEN}, 0.0, False),
    ("scene reset", SCENE_RESET, 0.0, False),
    ("habit, two farthest from the tray (prop and far spare)",
     {"block_a": A_IN, "block_c": side_at(TX, TY + 0.03), "hatch_post": FALLEN}, 0.0, False),
    ("scene reset", SCENE_RESET, 0.0, False),
    ("habit, the two blocks on their sides (prop and far spare)",
     {"block_a": A_IN, "block_c": side_at(TX, TY + 0.03), "hatch_post": FALLEN}, 0.0, False),
    ("scene reset", SCENE_RESET, 0.0, False),
    ("habit, the closest pair of blocks (prop and flat spare)",
     {"block_a": A_IN, "block_b": B_IN, "hatch_post": FALLEN}, 0.0, False),
    ("scene reset", SCENE_RESET, 0.0, False),
    # Other faults; each is followed by a scene reset.
    ("hatch edge sunk 6 mm", {"hatch_post": hatch(6.0)}, 0.0, False),
    ("then the hatch lifted back and both delivered: still 0", BOTH, 0.0, False),
    ("scene reset", SCENE_RESET, 0.0, False),
    ("after the reset, both delivered: 1.0", BOTH, 1.0, True),
    ("scene reset", SCENE_RESET, 0.0, False),
    ("prop pulled 6 mm toward the robot", {"block_a": ((PX - 0.006, PY, PZ), None, None)}, 0.0, False),
    ("then put back exactly and both delivered: still 0", BOTH, 0.0, False),
    ("scene reset", SCENE_RESET, 0.0, False),
    ("swap: prop in the tray, the flat spare stood under the hatch in its place",
     {"block_a": A_IN, "block_b": ((PX, PY, PZ), ON_SIDE, None), "block_c": side_at(TX, TY + 0.03)}, 0.0, False),
    ("scene reset", SCENE_RESET, 0.0, False),
    ("hatch held up while the prop is pulled out 30 mm",
     {"hatch_post": hatch(q=0.1), "block_a": ((PX - 0.03, PY, PZ), None, None)}, 0.0, False),
    ("then the prop pushed back and both delivered: still 0", BOTH, 0.0, False),
    ("scene reset", SCENE_RESET, 0.0, False),
    ("prop knocked over beside the hatch", {"block_a": ((PX - 0.06, PY, TOP + 0.015), None, None, rot_y(90))},
     0.0, False),
]
