"""On the station table: a dark liquid handler deck in front of the left arm, with six numbered positions in two columns,
and a white setup sheet in front of the right arm.

Five coloured plates sit on the deck. The sheet shows the right layout: one large colour block per position, drawn the
same way round as the deck, with a grey block, like a bare pad, for the position that stays empty. Four plates are right.
One is on the position the sheet shows empty, and its own position stands empty. Colour does the work because printed
labels would not read in the 320 x 176 top camera.

Run it:  inspect-robots run --embodiment yam_arena -E task=fix_the_deck_to_match_the_setup_sheet.py --scorer task_score ...
"""

from __future__ import annotations

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from yam_arena.demo import Waypoint, pick_place_waypoints  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]

INSTRUCTION = "One plate on the deck is in the wrong position. Move it so the deck matches the setup sheet."

# -- the deck: numbered as read from the arms, far pair first, odd numbers on the left ----------------------------------
DECK_H = 0.004
PAD_H = 0.001
PAD_TOP = TOP + DECK_H + PAD_H
PAD = (0.112, 0.080)  # a position's outline: a plate plus 8 mm all round
COLUMN_Y = {"left": 0.175, "right": 0.075}  # 10 cm apart: open fingers clear the next plate by 1.7 cm
ROW_X = {"far": 0.395, "middle": 0.273, "near": 0.151}  # 12.2 cm apart; past x 41 the arm can't point straight down
POSITIONS = {
    1: (ROW_X["far"], COLUMN_Y["left"]),
    2: (ROW_X["far"], COLUMN_Y["right"]),
    3: (ROW_X["middle"], COLUMN_Y["left"]),
    4: (ROW_X["middle"], COLUMN_Y["right"]),
    5: (ROW_X["near"], COLUMN_Y["left"]),
    6: (ROW_X["near"], COLUMN_Y["right"]),
}
DECK_CENTRE = (0.273, 0.125)
DECK_SIZE = (0.378, 0.200)  # long enough for the near row's number squares

# -- position numbers: white squares on the arm side of each pad, one per unit. A person reads these, not the camera -----
PIP = (0.008, 0.008, 0.001)
PIP_PITCH = 0.012
PIP_BEHIND = PAD[0] / 2 + 0.005  # the middle of the 10 mm gap between one pad and the next

# -- the plates: long side along x, so the fingers close across the 6.4 cm width. Three-quarter size, like a real plate's
# shape, because a full-size plate is wider than the gripper can take with room to spare ----------------------------------
PLATE = (0.096, 0.064, 0.024)
PLATE_MASS = 0.08

DECK_DARK = (0.17, 0.18, 0.20)
PAD_GREY = (0.62, 0.63, 0.65)
PIP_WHITE = (0.95, 0.95, 0.95)
SHEET_WHITE = (0.96, 0.96, 0.94)

# The station's lamp and tone-mapping pale everything, so these start darker and purer than they'll look. Yellow, the plate
# that matters, would otherwise fade towards the sheet's white and the table. See the tests for the estimate.
COLOURS = {
    "blue": (0.08, 0.30, 0.85),
    "green": (0.05, 0.50, 0.20),
    "orange": (0.85, 0.25, 0.02),
    "pink": (0.88, 0.25, 0.62),
    "yellow": (0.85, 0.62, 0.0),
}

# -- the layout: what the sheet shows, and where the trial starts ----------------------------------------------------------
SHEET_LAYOUT = {1: None, 2: "blue", 3: "green", 4: "orange", 5: "pink", 6: "yellow"}
START_LAYOUT = {1: "yellow", 2: "blue", 3: "green", 4: "orange", 5: "pink", 6: None}
WRONG = "yellow"  # far left to near right: the plate furthest from the arms, and it can't slide home past the others
WRONG_FROM = 1
OTHERS = tuple(c for c in COLOURS if c != WRONG)
OWN_POSITION = {colour: pos for pos, colour in SHEET_LAYOUT.items() if colour}


def plate_name(colour: str) -> str:
    return f"{colour}_plate"


# -- the sheet: the deck's layout drawn at 0.6 scale, on the same bearing. Each block is about 9 x 10 px in the top view ------
SHEET_XY = (0.273, -0.060)  # kept near the middle: the idle right arm's shadow starts around y -13 cm
SHEET_SIZE = (0.245, 0.130)
SHEET_H = 0.002
SHEET_SCALE = 0.6
BLOCK = (0.062, 0.046)  # leaves a gap of about 2 px between blocks in the top view
BLOCK_H = 0.001


def sheet_cell(position: int) -> tuple[float, float]:
    x, y = POSITIONS[position]
    return (SHEET_XY[0] + SHEET_SCALE * (x - DECK_CENTRE[0]), SHEET_XY[1] + SHEET_SCALE * (y - DECK_CENTRE[1]))


def _pips(position: int) -> list[Box]:
    x, y = POSITIONS[position]
    first = y - PIP_PITCH * (position - 1) / 2
    return [Box(f"pip{position}_{i}", pos=(x - PIP_BEHIND, first + i * PIP_PITCH, TOP + DECK_H + PIP[2] / 2), size=PIP,
                color=PIP_WHITE) for i in range(position)]


PLATES = {
    colour: Box(plate_name(colour), pos=(*POSITIONS[pos], PAD_TOP + PLATE[2] / 2), size=PLATE, mass=PLATE_MASS,
                color=COLOURS[colour])
    for pos, colour in START_LAYOUT.items() if colour
}

OBJECTS = [
    Box("deck", pos=(*DECK_CENTRE, TOP + DECK_H / 2), size=(*DECK_SIZE, DECK_H), color=DECK_DARK),
    *(Box(f"pad{n}", pos=(x, y, TOP + DECK_H + PAD_H / 2), size=(*PAD, PAD_H), color=PAD_GREY)
      for n, (x, y) in POSITIONS.items()),
    *(pip for n in POSITIONS for pip in _pips(n)),
    Box("setup_sheet", pos=(*SHEET_XY, TOP + SHEET_H / 2), size=(*SHEET_SIZE, SHEET_H), color=SHEET_WHITE),
    *(Box(f"sheet_block{n}", pos=(*sheet_cell(n), TOP + SHEET_H + BLOCK_H / 2), size=(*BLOCK, BLOCK_H),
          color=COLOURS[colour] if colour else PAD_GREY) for n, colour in SHEET_LAYOUT.items()),
    *PLATES.values(),
]

# -- scoring -------------------------------------------------------------------------------------------------------
IN_PLACE = 0.020  # the centre within this of the pad's centre, along x and along y
SEATED = 0.005  # bottom under this above the pad: a plate at rest is on it, a plate still being held is not
DOWN_ON_DECK = 0.030  # under this above the pad the plate is on the position; a plate on another plate is 2.4 cm up
ON_TABLE = 0.010  # bottom under this above the table: set down, not being carried
BESIDE = 0.100  # centre under this from position 6's centre: a near miss. A shove from position 1 can't get this close
FLAT_DEG = 10.0
SQUARE_DEG = 15.0
OFF_TABLE = 0.020  # a plate this far below the table top has gone over the edge

FIXED = 1.0
ROUGH = 0.7  # on the right position, flat, but not in place
MISSED = 0.3  # the right plate carried to its position and set down beside it
NUDGED = 0.5  # the multiplier when any of the four correct plates has been knocked out of place


def _tilt_deg(state) -> float:
    x, y, z, w = state.quat_xyzw
    return float(np.degrees(np.arccos(np.clip(1 - 2 * (x * x + y * y), -1.0, 1.0))))


def _turn_deg(state) -> float:
    """How far the plate's long side is from x. A plate looks the same turned half a circle."""
    x, y, z, w = state.quat_xyzw
    yaw = np.degrees(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
    return float(abs((yaw + 90.0) % 180.0 - 90.0))


def on_position(state, position: int) -> bool:
    """The plate's centre is inside the pad and it is down on the deck, whether square or not."""
    x, y = POSITIONS[position]
    return bool(abs(state.pos[0] - x) <= PAD[0] / 2 and abs(state.pos[1] - y) <= PAD[1] / 2
                and state.low[2] < PAD_TOP + DOWN_ON_DECK)


def is_flat(state) -> bool:
    return _tilt_deg(state) <= FLAT_DEG


def in_place(state, position: int) -> bool:
    """Sitting where the position says: centred within 2 cm, down on the pad, flat and square."""
    x, y = POSITIONS[position]
    return bool(abs(state.pos[0] - x) <= IN_PLACE and abs(state.pos[1] - y) <= IN_PLACE
                and state.low[2] < PAD_TOP + SEATED and is_flat(state) and _turn_deg(state) <= SQUARE_DEG)


def _deck_broken(objects) -> bool:
    """A plate on a position that isn't its own, a correct plate off its position, or a plate off the table."""
    for colour in COLOURS:
        state = objects[plate_name(colour)]
        if state.low[2] < TOP - OFF_TABLE:
            return True
        if any(on_position(state, n) for n in POSITIONS if n != OWN_POSITION[colour]):
            return True
    return not all(on_position(objects[plate_name(c)], OWN_POSITION[c]) for c in OTHERS)


def down_on(state, position: int) -> bool:
    """On the position and resting there, however rough."""
    return bool(on_position(state, position) and is_flat(state) and state.low[2] < PAD_TOP + SEATED)


def _beside(state, position: int) -> bool:
    """Set down flat on the table or deck, close to the position and on no position at all."""
    near = float(np.linalg.norm(state.pos[:2] - np.array(POSITIONS[position]))) < BESIDE
    return bool(near and state.low[2] < TOP + ON_TABLE and is_flat(state)
                and not any(on_position(state, n) for n in POSITIONS))


def grade(objects) -> float:
    """The score as the outcome table states it."""
    if _deck_broken(objects):
        return 0.0
    plate, home = objects[plate_name(WRONG)], OWN_POSITION[WRONG]
    if in_place(plate, home):
        value = FIXED
    elif down_on(plate, home):
        value = ROUGH
    elif _beside(plate, home):
        value = MISSED
    else:
        return 0.0
    if not all(in_place(objects[plate_name(c)], OWN_POSITION[c]) for c in OTHERS):
        value *= NUDGED
    return round(value, 6)


def success(objects) -> bool:
    return grade(objects) >= 1.0 and all(objects[plate_name(c)].resting(max_speed=0.02, max_spin=1.0) for c in COLOURS)


def score(objects) -> float:
    return grade(objects)


def distance(objects) -> float:
    """Metres from the yellow plate to its own position."""
    return float(np.linalg.norm(objects[plate_name(WRONG)].pos[:2] - np.array(POSITIONS[OWN_POSITION[WRONG]])))


# -- the scripted demo: the left arm makes the one move; the right arm holds its ready pose --------------------------------
GRIP_ABOVE_BOTTOM = 0.020  # the TCP above the plate's bottom face: pad tips 7 mm above it
DROP = 0.003
TRAVEL_Z = TOP + 0.120
SPEED = 0.7
SETTLE = 0.6
TRIM = {"pick": (0.0, 0.0), "place": (0.0, 0.0), "grip_z": 0.0}


def pick_point(position: int) -> tuple[float, float, float]:
    x, y = POSITIONS[position]
    px, py = TRIM["pick"]
    return (x + px, y + py, PAD_TOP + GRIP_ABOVE_BOTTOM + TRIM["grip_z"])


def place_point(position: int) -> tuple[float, float, float]:
    x, y = POSITIONS[position]
    qx, qy = TRIM["place"]
    return (x + qx, y + qy, PAD_TOP + GRIP_ABOVE_BOTTOM + DROP + TRIM["grip_z"])


def move_plate(start: int, end: int) -> list[Waypoint]:
    """One pick and place, pausing above the plate and above the position so the arm settles before the precise moves."""
    plan: list[Waypoint] = []
    for w in pick_place_waypoints(pick_point(start), place_point(end), travel_z=TRAVEL_Z, speed=SPEED):
        plan.append(w)
        if w.label in ("to the object", "carry"):
            plan.append(Waypoint(SETTLE, w.pos, w.finger, "settle", w.yaw))
    return plan


def demo():
    """Left arm: the yellow plate from position 1 to position 6. Nothing else moves."""
    return {"left": move_plate(WRONG_FROM, OWN_POSITION[WRONG])}
