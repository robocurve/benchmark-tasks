"""On the station table, a kitchen counter at home: a weekly calendar blister card (seven cells in a row, Monday at
the green tab nearest the arms) lies dome-up in the rimmed base of a blister popper. Monday's and Tuesday's cells are
already empty; Wednesday's tablet was missed and is still in; Thursday to Sunday are full. Beside it: a medication
record with ticks for Monday and Tuesday, a glass of water, reading glasses, a fruit bowl and a juice carton.
"""

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from partkit import Slider, on_reset, on_step, part  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Asset, Box, Cylinder  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

FOIL, DOME, TORN, GREEN = (0.80, 0.81, 0.84), (0.95, 0.72, 0.20), (0.18, 0.18, 0.20), (0.10, 0.70, 0.25)
POPPER, PAPER, INK = (0.22, 0.42, 0.70), (0.96, 0.95, 0.90), (0.20, 0.55, 0.25)

DAYS = ["monday", "tuesday", "wednesday", "thursday", "friday", "saturday", "sunday"]
TAKEN = {"monday", "tuesday"}
TODAY = "thursday"
PITCH = 0.03
CARD_X0, CARD_Y = 0.20, -0.20
CARD_LEN, CARD_W, CARD_T = 0.225, 0.05, 0.003
DOME_SIDE, DOME_H, TRAVEL, POPPED = 0.014, 0.010, 0.0095, -0.004
BASE_T = 0.01
CARD_Z = TOP + BASE_T + CARD_T / 2


def cell_x(day):
    return CARD_X0 + 0.0225 + PITCH * DAYS.index(day)


def _local_x(day):
    return cell_x(day) - (CARD_X0 + CARD_LEN / 2)


DOMES = tuple(
    Slider(day, rest=(_local_x(day), 0, 0), travel=TRAVEL, rate=200.0, damping=5.0,
           boxes=(((_local_x(day), 0, CARD_T / 2 + DOME_H / 2), (DOME_SIDE, DOME_SIDE, DOME_H), DOME),))
    for day in DAYS if day not in TAKEN
)
CARD = part(
    "blister_card", pos=(CARD_X0 + CARD_LEN / 2, CARD_Y, CARD_Z), mass=0.4, color=FOIL, friction=1.0,
    boxes=[((0, 0, 0), (CARD_LEN, CARD_W, CARD_T), FOIL),
           ((-CARD_LEN / 2 + 0.0075, 0, CARD_T / 2 + 0.0005), (0.015, CARD_W, 0.001), GREEN),
           *[((_local_x(d), 0, CARD_T / 2 + 0.0005), (DOME_SIDE - 0.001, DOME_SIDE - 0.001, 0.001), TORN) for d in DAYS]],
    sliders=DOMES,
)

RIM = 0.008
BASE_X, BASE_LEN, BASE_W = CARD_X0 + CARD_LEN / 2, CARD_LEN + 2 * RIM + 0.004, CARD_W + 2 * RIM + 0.004
FLOOR_T, LEDGE_W = 0.002, 0.01
POPPER_BASE = [
    Box("popper_base", pos=(BASE_X, CARD_Y, TOP + FLOOR_T / 2), size=(BASE_LEN, BASE_W, FLOOR_T), color=POPPER),
    *[Box(f"popper_ledge_{side}", pos=(BASE_X, CARD_Y + sign * (CARD_W - LEDGE_W) / 2, TOP + BASE_T / 2),
          size=(CARD_LEN, LEDGE_W, BASE_T), color=POPPER) for side, sign in (("left", 1), ("right", -1))],
    Box("popper_rim_near", pos=(BASE_X - (BASE_LEN - RIM) / 2, CARD_Y, TOP + BASE_T + 0.003),
        size=(RIM, BASE_W, 0.006), color=POPPER),
    Box("popper_rim_far", pos=(BASE_X + (BASE_LEN - RIM) / 2, CARD_Y, TOP + BASE_T + 0.003),
        size=(RIM, BASE_W, 0.006), color=POPPER),
    Box("popper_rim_left", pos=(BASE_X, CARD_Y + (BASE_W - RIM) / 2, TOP + BASE_T + 0.003),
        size=(BASE_LEN, RIM, 0.006), color=POPPER),
    Box("popper_rim_right", pos=(BASE_X, CARD_Y - (BASE_W - RIM) / 2, TOP + BASE_T + 0.003),
        size=(BASE_LEN, RIM, 0.006), color=POPPER),
]

RECORD_X, RECORD_Y = 0.30, -0.02
RECORD = [
    Box("medication_record", pos=(RECORD_X, RECORD_Y, TOP + 0.001), size=(0.21, 0.10, 0.002), color=PAPER),
    *[Box(f"record_tick_{d}", pos=(RECORD_X - 0.075 + 0.025 * i, RECORD_Y - 0.025, TOP + 0.0025),
          size=(0.012, 0.012, 0.001), color=INK) for i, d in enumerate(DAYS) if d in TAKEN],
    *[Box(f"record_box_{d}", pos=(RECORD_X - 0.075 + 0.025 * i, RECORD_Y - 0.025, TOP + 0.0022),
          size=(0.016, 0.016, 0.0004), color=(0.30, 0.30, 0.32)) for i, d in enumerate(DAYS)],
    Box("record_header", pos=(RECORD_X, RECORD_Y + 0.03, TOP + 0.0022), size=(0.17, 0.012, 0.0004),
        color=(0.30, 0.30, 0.32)),
]

DRESSING = [
    Cylinder("water_glass", pos=(0.52, -0.30, TOP + 0.055), radius=0.035, height=0.11, color=(0.70, 0.85, 0.95)),
    Asset("reading_glasses", "glasses_hot3d_robolab", xy=(0.50, -0.06), extent=0.14, yaw=0.4),
    Asset("fruit_bowl", "wooden_bowl_hot3d_robolab", xy=(0.62, 0.18), extent=0.18),
    Asset("juice", "orange_juice_carton_hope_robolab", xy=(0.60, 0.36), extent=0.075),
    Box("tea_towel", pos=(0.40, 0.30, TOP + 0.002), size=(0.16, 0.12, 0.004), color=(0.80, 0.30, 0.25)),
]

OBJECTS = [*POPPER_BASE, CARD, *RECORD, *DRESSING]

_S = {"pressed": set()}


def _hold(env, day, z):
    """Set the spring rest of a dome: 0 for an intact dome, fully down for a crushed one."""
    card = env.unwrapped.scene[CARD.name]
    target = card.data.joint_pos_target.clone()
    target[:, list(card.joint_names).index(f"{day}_slide")] = z
    card.set_joint_position_target(target)


@on_reset
def _reset(env=None):
    _S["pressed"] = set()
    if env is not None:
        for s in DOMES:
            _hold(env, s.name, 0.0)


@on_step
def _crush(env=None):
    """A dome pressed down 4 mm sinks below the foil and stays there, showing an empty cell like a crushed blister, and
    its tablet counts as pressed out."""
    card = env.unwrapped.scene[CARD.name]
    names = list(card.joint_names)
    pos = card.data.joint_pos[0]
    for s in DOMES:
        if s.name not in _S["pressed"] and float(pos[names.index(f"{s.name}_slide")]) < POPPED:
            _S["pressed"].add(s.name)
            _hold(env, s.name, -TRAVEL)


def score(objects):
    """1.0 if Thursday's tablet is pressed out and no other; 0 if any other cell is pressed (Wednesday's missed dose
    taken late, or a wrong day), or if nothing is pressed."""
    return 1.0 if _S["pressed"] == {TODAY} else 0.0


def demo():
    """The right arm presses Thursday's dome (the fourth cell from the green tab) with closed fingers, then lifts."""
    x = cell_x(TODAY)
    dome_top = CARD_Z + CARD_T / 2 + DOME_H
    right = [
        Waypoint(2.0, (x, CARD_Y, dome_top + 0.08), FINGER_CLOSED, "above Thursday's cell"),
        Waypoint(1.5, (x, CARD_Y, dome_top + 0.015), FINGER_CLOSED, "touch the dome"),
        Waypoint(1.5, (x, CARD_Y, dome_top - 0.012), FINGER_CLOSED, "press it flat"),
        Waypoint(0.5, (x, CARD_Y, dome_top - 0.012), FINGER_CLOSED, "hold"),
        Waypoint(1.5, (x, CARD_Y, dome_top + 0.10), FINGER_CLOSED, "lift"),
        Waypoint(1.5, (0.12, -0.42, TOP + 0.20), FINGER_OPEN, "move clear of the camera's view"),
    ]
    return {"right": right}
