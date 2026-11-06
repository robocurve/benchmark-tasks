"""On the station table, a pharmacy dispensing bench: four identical white medicine cartons stand in a row in a low
tote, all turned with their label faces away from the arms, so no camera can see a label. Each label carries a
strength band: red (10 mg) on one carton, green (5 mg) on the other three. A dispensing basket stands in front of the
right arm.
"""

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from partkit import on_reset, part  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box, Cylinder  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

CARD, LABEL, PRINT, RED, GREEN = (0.97, 0.97, 0.96), (0.90, 0.92, 0.94), (0.35, 0.37, 0.40), (0.85, 0.12, 0.12), \
    (0.15, 0.62, 0.30)
TOTE_C, BASKET_C, BENCH = (0.25, 0.45, 0.70), (0.85, 0.85, 0.82), (0.92, 0.92, 0.90)

DEPTH, WIDTH, HEIGHT = 0.04, 0.05, 0.10
ROW_X, FLOOR = 0.33, TOP + 0.004
CARTONS = {"carton_1": (0.19, GREEN), "carton_2": (0.065, GREEN), "carton_3": (-0.065, RED), "carton_4": (-0.19, GREEN)}
TEN_MG = "carton_3"


def _carton(name, y, band):
    """Label face on the carton's +x side, turned away from the arms at the start."""
    face = DEPTH / 2 + 0.0004
    return part(name, pos=(ROW_X, y, FLOOR + HEIGHT / 2), mass=0.06, color=CARD, friction=1.0, boxes=[
        ((0, 0, 0), (DEPTH, WIDTH, HEIGHT), CARD),
        ((face, 0, 0.005), (0.0008, WIDTH - 0.006, 0.07), LABEL),
        ((face + 0.0006, 0, 0.0225), (0.0008, WIDTH - 0.006, 0.035), band),
        ((face + 0.0006, 0, -0.012), (0.0008, WIDTH - 0.016, 0.004), PRINT),
        ((face + 0.0006, 0, -0.022), (0.0008, WIDTH - 0.024, 0.003), PRINT),
        ((-face, 0, -0.03), (0.0008, 0.02, 0.012), PRINT),
    ])


TOTE_X0, TOTE_X1, TOTE_HALF_Y, TOTE_WALL, TOTE_H = 0.29, 0.37, 0.25, 0.005, 0.03
TOTE = [
    Box("tote_floor", pos=((TOTE_X0 + TOTE_X1) / 2, 0, TOP + 0.002), size=(TOTE_X1 - TOTE_X0, 2 * TOTE_HALF_Y, 0.004),
        color=TOTE_C),
    Box("tote_front", pos=(TOTE_X0, 0, TOP + TOTE_H / 2), size=(TOTE_WALL, 2 * TOTE_HALF_Y, TOTE_H), color=TOTE_C),
    Box("tote_back", pos=(TOTE_X1, 0, TOP + TOTE_H / 2), size=(TOTE_WALL, 2 * TOTE_HALF_Y, TOTE_H), color=TOTE_C),
    Box("tote_left", pos=((TOTE_X0 + TOTE_X1) / 2, TOTE_HALF_Y, TOP + TOTE_H / 2),
        size=(TOTE_X1 - TOTE_X0, TOTE_WALL, TOTE_H), color=TOTE_C),
    Box("tote_right", pos=((TOTE_X0 + TOTE_X1) / 2, -TOTE_HALF_Y, TOP + TOTE_H / 2),
        size=(TOTE_X1 - TOTE_X0, TOTE_WALL, TOTE_H), color=TOTE_C),
]

BASKET_XY, BASKET_INSIDE, BASKET_H = (0.19, -0.30), (0.10, 0.10), 0.05
BASKET = [
    Box("basket_floor", pos=(*BASKET_XY, TOP + 0.002), size=(0.11, 0.11, 0.004), color=BASKET_C),
    *[Box(f"basket_wall_{i}", pos=(BASKET_XY[0] + dx, BASKET_XY[1] + dy, TOP + BASKET_H / 2), size=size,
          color=BASKET_C)
      for i, (dx, dy, size) in enumerate([(0.0525, 0, (0.005, 0.11, BASKET_H)), (-0.0525, 0, (0.005, 0.11, BASKET_H)),
                                          (0, 0.0525, (0.11, 0.005, BASKET_H)), (0, -0.0525, (0.11, 0.005, BASKET_H))])],
]

DRESSING = [
    Box("bench_mat", pos=(0.34, 0.0, TOP + 0.0005), size=(0.30, 0.62, 0.001), color=BENCH),
    Box("prescription", pos=(0.18, 0.12, TOP + 0.0012), size=(0.10, 0.07, 0.001), color=(0.98, 0.98, 0.95)),
    Box("prescription_lines", pos=(0.18, 0.12, TOP + 0.0018), size=(0.06, 0.04, 0.0004), color=PRINT),
    Box("counting_tray", pos=(0.58, 0.20, TOP + 0.006), size=(0.14, 0.08, 0.012), color=(0.20, 0.55, 0.75)),
    Box("pharmacy_bag", pos=(0.60, -0.22, TOP + 0.05), size=(0.09, 0.05, 0.10), color=(0.95, 0.95, 0.92)),
    Cylinder("tablet_bottle", pos=(0.58, -0.02, TOP + 0.04), radius=0.025, height=0.08, color=(0.65, 0.40, 0.15)),
]

OBJECTS = [*TOTE, *BASKET, *DRESSING, *[_carton(n, y, band) for n, (y, band) in CARTONS.items()]]


_S = {"checked": False}


@on_reset
def _reset(environment=None):
    _S["checked"] = False


def _label_towards_arms(state):
    return float(state.rotation()[0, 0]) < -0.7


def _in_basket(state):
    dx, dy = np.abs(state.pos[:2] - BASKET_XY)
    return bool(dx < BASKET_INSIDE[0] / 2 and dy < BASKET_INSIDE[1] / 2 and state.pos[2] < TOP + BASKET_H + 0.02)


def score(objects):
    """0 if a 5 mg carton is in the basket. Otherwise 1.0 for the 10 mg carton in the basket after its label was turned
    towards the arms, 0.5 for it in the basket unchecked, 0.25 for it turned but not in the basket, 0 otherwise."""
    ten = objects[TEN_MG]
    if _label_towards_arms(ten) and not _in_basket(ten):
        _S["checked"] = True
    if any(_in_basket(objects[c]) for c in CARTONS if c != TEN_MG):
        return 0.0
    if _in_basket(ten):
        return 1.0 if _S["checked"] else 0.5
    return 0.25 if _S["checked"] else 0.0


GRIP = 0.07 + 0.008
LIFT = 0.13


def _quarter(y):
    grip, lifted = FLOOR + GRIP, FLOOR + LIFT
    return [
        (1.5, (ROW_X, y, TOP + 0.20), FINGER_OPEN, "above", 0.0),
        (1.5, (ROW_X, y, grip), FINGER_OPEN, "descend", 0.0),
        (0.6, (ROW_X, y, grip), FINGER_OPEN, "settle", 0.0),
        (0.8, (ROW_X, y, grip), FINGER_CLOSED, "close", 0.0),
        (1.0, (ROW_X, y, lifted), FINGER_CLOSED, "lift clear of the tote", 0.0),
        (1.5, (ROW_X, y, lifted), FINGER_CLOSED, "turn a quarter", math.pi / 2),
        (1.2, (ROW_X, y, grip + 0.004), FINGER_CLOSED, "lower", math.pi / 2),
        (0.6, (ROW_X, y, grip + 0.004), FINGER_OPEN, "release", math.pi / 2),
    ]


def _turn(y):
    """Lift a carton clear of the tote and turn it a half turn, in two quarter turns with a fresh grip between, so its
    label faces the arms; set it back down."""
    steps = [*_quarter(y), *_quarter(y), (1.0, (ROW_X, y, TOP + 0.20), FINGER_OPEN, "retreat", math.pi / 2)]
    return [Waypoint(*s) for s in steps]


def demo():
    """Each arm turns its two cartons so their labels face the arms; the third carton shows the red band, so the right
    arm lifts it into the basket. The green-band cartons stay in the tote."""
    y = CARTONS[TEN_MG][0]
    into = [
        (1.5, (ROW_X, y, TOP + 0.20), FINGER_OPEN, "above the red-band carton", 0.0),
        (1.5, (ROW_X, y, FLOOR + GRIP), FINGER_OPEN, "descend", 0.0),
        (0.6, (ROW_X, y, FLOOR + GRIP), FINGER_OPEN, "settle", 0.0),
        (0.8, (ROW_X, y, FLOOR + GRIP), FINGER_CLOSED, "close", 0.0),
        (1.0, (ROW_X, y, TOP + 0.20), FINGER_CLOSED, "lift", 0.0),
        (2.0, (*BASKET_XY, TOP + 0.20), FINGER_CLOSED, "carry to the basket", 0.0),
        (1.5, (*BASKET_XY, FLOOR + GRIP + 0.006), FINGER_CLOSED, "lower into the basket", 0.0),
        (0.6, (*BASKET_XY, FLOOR + GRIP + 0.006), FINGER_OPEN, "release", 0.0),
        (1.0, (*BASKET_XY, TOP + 0.20), FINGER_OPEN, "retreat", 0.0),
        (1.5, (0.12, -0.42, TOP + 0.20), FINGER_OPEN, "move clear", 0.0),
    ]
    left = [*_turn(CARTONS["carton_1"][0]), *_turn(CARTONS["carton_2"][0]),
            Waypoint(1.5, (0.12, 0.42, TOP + 0.20), FINGER_OPEN, "move clear")]
    turn_time = sum(w.duration for w in _turn(y))
    right = [*_turn(CARTONS["carton_4"][0]), Waypoint(turn_time, (0.20, -0.30, TOP + 0.20), FINGER_OPEN, "wait"),
             *_turn(y), *[Waypoint(*s) for s in into]]
    return {"left": left, "right": right}
