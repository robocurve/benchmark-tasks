"""On the station table, right of the midline: a fixed green PCIe riser board (277 x 80 mm) lying flat, its long side
running out from the robot, with a black x16 slot (89 mm long) on it and a steel bracket gap at the slot's near end
and a steel support channel at the far end. Beside it, toward the right arm, a dual-slot graphics card (254 x 35 mm,
111 mm tall, 350 g) stands on its gold edge connector between two blue foam strips, parallel to the riser.

The card goes into the slot: its gold edge connector (1.6 mm thick, split by a key notch) into the black slot (2.0 mm
wide, its key rib in the notch), its steel end bracket into the 2.9 mm gap at the slot's near end and its far end into
the support channel (36 mm wide), pressed straight down until the board edge rests on the slot. The slot is keyed, so
the card goes in only with its bracket toward the robot. Seated, the card's top edge is about 120 mm above the table.
"""

import math
from pathlib import Path

import numpy as np

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Asset, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]
HERE = Path(__file__).resolve().parent


def _register():
    """Add the card and the riser (USD files next to this one, written by make_meshes.py) to Arena's asset
    registry, so ``Asset`` can name them. Outside the simulator (tests, checks) there is nothing to register."""
    try:
        from isaaclab_arena.assets.registries import AssetRegistry
    except ImportError:
        return
    reg = AssetRegistry()
    for name in ("seat_gpu_card", "seat_gpu_riser"):
        if not reg.is_registered(name, ensure_loaded=False):
            reg.register(type(name, (), {"name": name, "tags": ["object"], "usd_path": str(HERE / f"{name}.usda")}), name)


_register()

# Frames (make_meshes.py): x along the card from its bracket end, y across it (board plane at 0), z up from the
# bottom of the gold fingers. The riser's frame is the seated card's frame dropped to the table.
REST = 0.001  # assets spawn this far above the table; the fixed riser stays there
CARD_BOX_C, CARD_EXTENT = (0.125, 0.0155), 0.258   # centre and widest side of the card's footprint, its frame
RISER_BOX_C, RISER_EXTENT = (0.1265, 0.010), 0.277
FLOOR = 0.00875        # slot floor (= board edge on the slot's top) above the riser's frame origin
CARD_TOP = 0.11115
MASS = 0.35

RISER_O = np.array([0.18, -0.14, TOP + REST])   # riser frame origin, world
CARD_O = np.array([0.18, -0.32, TOP])           # card frame origin at the start (fingers on the table)
SEATED_O = RISER_O + np.array([0.0, 0.0, FLOOR])
# The card's reported ``pos`` sits at this point of its frame (measured in the sim, identical in two runs)
POS_OFF = np.array([0.125, 0.0155, 0.060])

FOAM = (0.2, 0.3, 0.8)
OBJECTS = [
    Asset("riser", "seat_gpu_riser", xy=tuple(RISER_O[:2] + RISER_BOX_C), extent=RISER_EXTENT),
    Asset("card", "seat_gpu_card", xy=tuple(CARD_O[:2] + CARD_BOX_C), extent=CARD_EXTENT, mass=MASS),
    # foam stand: two strips 1 mm clear of the card's faces and a block under its far end
    # WHY: with 4.5 mm of play the closing grip turned the card 2.6 deg in the stand, enough to miss the far guide
    Box("foam_left", size=(0.21, 0.006, 0.025), pos=(CARD_O[0] + 0.125, CARD_O[1] + 0.037, TOP + 0.0125), color=FOAM),
    Box("foam_right", size=(0.21, 0.006, 0.025), pos=(CARD_O[0] + 0.125, CARD_O[1] - 0.006, TOP + 0.0125), color=FOAM),
    Box("foam_block", size=(0.09, 0.030, 0.0085), pos=(CARD_O[0] + 0.195, CARD_O[1] + 0.0155, TOP + 0.00425), color=FOAM),
]

# Tolerances
SEAT_Z_TOL = 0.001
SEAT_X_TOL, SEAT_Y_TOL = 0.0015, 0.0010   # the slot allows 0.5 mm along it and 0.2 mm across
MAX_TILT, MAX_YAW = 2.0, 2.0              # degrees
IN_SLOT_Z = 0.0080                        # fingers below the slot's top (they are 8.5 mm tall)
LINED_XY, LINED_YAW, LINED_Z = 0.003, 3.0, 0.040
# Card-frame points (make_meshes.py): the bottoms of the gold fingers at the connector's two ends, and the body's
# bottom edge at the middle of the support channel
CONNECTOR_ENDS = (np.array([0.040, 0.0, 0.0]), np.array([0.040 + 0.01165 + 0.0020 + 0.07165, 0.0, 0.0]))
FAR_BOTTOM = np.array([0.225, 0.0155, 0.0085])
GUIDE_TOP = 0.065   # support channel's top above the riser's frame origin (make_meshes.GUIDE_TOP)


def _rot(q):
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _frame(o):
    """The card's rotation matrix and its frame origin (world)."""
    r = _rot(o.quat_xyzw)
    return r, np.asarray(o.pos, dtype=float) - r @ POS_OFF


def card_pose(o):
    """(error of the card's frame origin from its seated place (m, world), tilt from upright (deg), yaw (deg))."""
    r, origin = _frame(o)
    tilt = math.degrees(math.acos(float(np.clip(r[2, 2], -1.0, 1.0))))
    yaw = math.degrees(math.atan2(r[1, 0], r[0, 0]))
    return origin - SEATED_O, tilt, yaw


def connector_heights(o):
    """Heights (m) of the gold fingers' bottoms at the connector's two ends above the slot floor."""
    r, origin = _frame(o)
    return [float((origin + r @ p)[2] - SEATED_O[2]) for p in CONNECTOR_ENDS]


def far_end_in_channel(o):
    """The card's bottom edge at the support channel is below the channel's top."""
    r, origin = _frame(o)
    return float((origin + r @ FAR_BOTTOM)[2]) < RISER_O[2] + GUIDE_TOP


def lined_up(objects):
    """Bracket end toward the robot, over the slot and low enough that the far end is in the support channel."""
    c = objects["card"]
    e, tilt, yaw = card_pose(c)
    return (abs(yaw) <= LINED_YAW and tilt <= 5.0 and max(abs(e[0]), abs(e[1])) <= LINED_XY
            and -SEAT_Z_TOL <= min(connector_heights(c)) and e[2] <= LINED_Z and far_end_in_channel(c))


def in_slot(objects):
    """Gold fingers started into the slot along the whole connector, not all the way down."""
    e, tilt, yaw = card_pose(objects["card"])
    return (lined_up(objects) and max(connector_heights(objects["card"])) <= IN_SLOT_Z
            and abs(e[0]) <= SEAT_X_TOL and abs(e[1]) <= SEAT_X_TOL)


def seated(objects):
    """Both ends of the connector within 1 mm of the slot floor (so pitch along the card is under about 0.7 deg),
    centred, and upright and square."""
    e, tilt, yaw = card_pose(objects["card"])
    return (all(abs(h) <= SEAT_Z_TOL for h in connector_heights(objects["card"]))
            and abs(e[0]) <= SEAT_X_TOL and abs(e[1]) <= SEAT_Y_TOL
            and tilt <= MAX_TILT and abs(yaw) <= MAX_YAW)


def success(objects):
    """The card seated and still. Holds for 1 s."""
    return seated(objects) and objects["card"].resting()


def distance(objects):
    e, _, _ = card_pose(objects["card"])
    return float(np.linalg.norm(e))


def score(objects):
    """Partial credit (success scores 1.0): 0.8 seated but not still, 0.5 with the fingers started into the slot, 0.2
    with the card lined up over it, bracket toward the robot and its far end in the support channel."""
    if seated(objects):
        # WHY: credit needs the card still (robocurve#34 was sent back for a missing stillness test); 1.0 only when
        # success() holds
        return 1.0 if objects["card"].resting() else 0.8
    if in_slot(objects):
        return 0.5
    if lined_up(objects):
        return 0.2
    return 0.0


GRIP_DZ = 0.030  # TCP below the card's top edge: the whole pad length on the card
# WHY: each sim finger is its own joint, capped at 50 N (yam_arena.constants.GRIPPER). Closed all the way on a 35 mm
# card both fingers hit the cap, nothing centres the card and it drifts up to 13 mm along the fingers. Aiming the
# pads 19 mm apart presses each 8 mm into the card at 40 N, under the cap, so the grip centres itself.
GRIP_FINGER = -(0.035 - 0.016) / 2
PAD_TIP = 0.013   # pad tips past the TCP (yam_arena.constants.PAD_TIP_PAST_TCP)
PUSH = 0.003      # the closed pads aim this far below the seated card's top edge
PRESS_X = 0.083   # over the middle of the connector, along the card
# WHY: carried at speed, the held card turns about 3.2 deg (same in three runs), which puts its far end 4 to 7 mm off the
# support channel; the carry is turned back by that much
CARRY_YAW = -0.056


def demo(delay=0.0):
    """The right arm lifts the card out of the foam by its sides, carries it over the riser, lowers it into the
    support channel and bracket gap and on into the slot, lets go, and presses the top edge straight down over the
    middle of the connector with the closed fingers."""
    hi = TOP + 0.19
    pick = tuple(CARD_O[:2] + CARD_BOX_C)
    put = tuple(SEATED_O[:2] + CARD_BOX_C)
    grip_z = CARD_O[2] + CARD_TOP - GRIP_DZ
    seat_z = SEATED_O[2] + CARD_TOP - GRIP_DZ
    g = GRIP_FINGER
    w = [Waypoint(2.0 + delay, (*pick, hi), FINGER_OPEN, "to card"),
         Waypoint(1.5, (*pick, grip_z), FINGER_OPEN, "down"),
         Waypoint(1.0, (*pick, grip_z), FINGER_OPEN, "settle"),
         Waypoint(0.8, (*pick, grip_z), g, "grip"),
         Waypoint(2.0, (*pick, hi), g, "lift out"),
         Waypoint(2.5, (*put, hi), g, "over riser", CARRY_YAW),
         # WHY: the arm lags up to 5 mm sideways on a fast descent; stop above the guide, let it settle, go in slowly
         Waypoint(2.5, (*put, seat_z + 0.060), g, "over the guide", CARRY_YAW),
         Waypoint(1.5, (*put, seat_z + 0.060), g, "settle", CARRY_YAW),
         Waypoint(4.0, (*put, seat_z + 0.030), g, "into the channel", CARRY_YAW),
         Waypoint(1.0, (*put, seat_z + 0.030), g, "settle", CARRY_YAW),
         Waypoint(3.0, (*put, seat_z + 0.010), g, "bracket in", CARRY_YAW),
         # WHY: let go 2 mm short of seated (tried 2026-11-07), the card caught on the slot's mouth 5 mm up at a 0.05 s
         # start delay and wedged with its bracket end 3 mm up; three presses aimed 15 mm past seated moved it 0.3 mm.
         # So the card is pushed down while held; success fires at the end of the push or as the fingers open (step 536)
         Waypoint(3.0, (*put, seat_z - 0.001), g, "into the slot", CARRY_YAW),
         Waypoint(0.8, (*put, seat_z - 0.001), g, "hold", CARRY_YAW),
         Waypoint(0.6, (*put, seat_z - 0.001), FINGER_OPEN, "let go", CARRY_YAW),
         Waypoint(1.0, (*put, seat_z + 0.060), FINGER_OPEN, "up"),
         Waypoint(0.6, (*put, seat_z + 0.060), FINGER_CLOSED, "close")]
    top = SEATED_O[2] + CARD_TOP
    x, y = SEATED_O[0] + PRESS_X, SEATED_O[1] + CARD_BOX_C[1]
    w += [Waypoint(1.0, (x, y, top + PAD_TIP + 0.015), FINGER_CLOSED, "over the edge"),
          Waypoint(1.2, (x, y, top + PAD_TIP - PUSH), FINGER_CLOSED, "press"),
          Waypoint(0.8, (x, y, top + PAD_TIP - PUSH), FINGER_CLOSED, "hold"),
          Waypoint(1.0, (x, y, hi), FINGER_CLOSED, "up")]
    return {"right": w}
