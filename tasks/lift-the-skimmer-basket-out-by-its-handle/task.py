"""On the station table, in front of the left arm: a block of grey pool deck with a round skimmer well in it. A
white skimmer basket full of leaves sits in the well, its rim below the deck surface, with a close fit all round.
A wire handle is hinged to the basket on a front-to-back axis across its middle and lies folded flat over the top
of the leaves. On the deck beside the well, toward the middle of the table, is a shallow blue tray. The basket is
the only loose object.
"""

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import parts  # noqa: E402
from yam_arena.constants import FINGER_CLOSED  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

DECK_C = (0.72, 0.71, 0.68)
WELL_C = (0.55, 0.56, 0.58)
TRAY_C = (0.20, 0.42, 0.80)
BASKET_C = (0.90, 0.91, 0.92)
LEAF_C = (0.36, 0.33, 0.14)
WIRE_C = (0.12, 0.12, 0.13)

# World frame: x away from the arm bases, y to the robot's left, z up (m).
WELL = (0.27, 0.22)  # well centre
TRAY = (0.27, 0.06)  # tray centre

# -- basket (loose; origin at the centre of its base) ---------------------------------------------------------------
BASKET_R, BASKET_H, BASKET_MASS = 0.050, 0.045, 0.30
RIM_BELOW_DECK = 0.030
DECK_H = BASKET_H + RIM_BELOW_DECK  # 75 mm deck above the table; the well floor is the table
PIVOT_Z = BASKET_H + 0.005  # handle hinge axis above the basket's base
BAIL_L = 0.026  # pivot axis to grip-bar axis
BAR_R, BAR_LEN = 0.005, 0.084
LEG_X = 0.040
HANDLE_STOP = math.radians(95.0)  # the handle swings up to a stop 5 deg past upright and stays there

BASKET = parts.Part(
    "skimmer_basket", pos=(WELL[0], WELL[1], TOP), mass=BASKET_MASS,
    shapes=[
        parts.cylinder(BASKET_R, BASKET_H, at=(0.0, 0.0, BASKET_H / 2), color=BASKET_C),
        parts.cylinder(BASKET_R - 0.004, 0.002, at=(0.0, 0.0, BASKET_H - 0.0009), color=LEAF_C),
    ],
    joints=[parts.Hinge(
        "handle", pivot=(0.0, 0.0, PIVOT_Z), axis=(1.0, 0.0, 0.0), lower=0.0, upper=HANDLE_STOP, mass=0.02,
        damping=0.005,
        shapes=[
            *[parts.box((0.004, BAIL_L + 0.006, 0.006), at=(s * LEG_X, BAIL_L / 2, PIVOT_Z), color=WIRE_C)
              for s in (-1, 1)],
            parts.cylinder(BAR_R, BAR_LEN, at=(0.0, BAIL_L, PIVOT_Z), color=WIRE_C, axis="x"),
        ])],
)

# -- the well: twelve wall blocks whose inner faces make a regular 12-sided bore -----------------------------------
WELL_APOTHEM = 0.054  # 4 mm clearance to the basket wall at every face
WALL_T, WALL_W = 0.030, 0.046
WALLS = []
for k in range(12):
    a = k * math.pi / 6
    r = WELL_APOTHEM + WALL_T / 2
    WALLS.append(Box(f"well_wall_{k}", pos=(WELL[0] + r * math.cos(a), WELL[1] + r * math.sin(a), TOP + DECK_H / 2),
                     size=(WALL_T, WALL_W, DECK_H), yaw=a, color=WELL_C))

# -- the deck slab around the well (a square hole the wall blocks fill) ----------------------------------------------
DECK_X, DECK_Y = (0.17, 0.37), (-0.02, 0.32)
HOLE = 0.056  # half-side of the slab's square hole


def _slab(name, x, y):
    return Box(name, pos=((x[0] + x[1]) / 2, (y[0] + y[1]) / 2, TOP + DECK_H / 2),
               size=(x[1] - x[0], y[1] - y[0], DECK_H), color=DECK_C)


DECK = [
    _slab("deck_left", DECK_X, (WELL[1] + HOLE, DECK_Y[1])),
    _slab("deck_right", DECK_X, (DECK_Y[0], WELL[1] - HOLE)),
    _slab("deck_far", (WELL[0] + HOLE, DECK_X[1]), (WELL[1] - HOLE, WELL[1] + HOLE)),
    _slab("deck_near", (DECK_X[0], WELL[0] - HOLE), (WELL[1] - HOLE, WELL[1] + HOLE)),
]

# -- the tray on the deck ------------------------------------------------------------------------------------------
TRAY_IN = 0.130  # inside length and width
TRAY_FLOOR_T, TRAY_LIP_H, TRAY_WALL_T = 0.003, 0.006, 0.005
TRAY_FLOOR = TOP + DECK_H + TRAY_FLOOR_T  # the tray's floor surface
_tz = TOP + DECK_H + (TRAY_FLOOR_T + TRAY_LIP_H) / 2
_th = TRAY_FLOOR_T + TRAY_LIP_H
_to = TRAY_IN / 2 + TRAY_WALL_T / 2
_tl = TRAY_IN + 2 * TRAY_WALL_T
TRAY_PARTS = [
    Box("tray_floor", pos=(*TRAY, TOP + DECK_H + TRAY_FLOOR_T / 2), size=(TRAY_IN, TRAY_IN, TRAY_FLOOR_T),
        color=TRAY_C),
    Box("tray_wall_far", pos=(TRAY[0] + _to, TRAY[1], _tz), size=(TRAY_WALL_T, _tl, _th), color=TRAY_C),
    Box("tray_wall_near", pos=(TRAY[0] - _to, TRAY[1], _tz), size=(TRAY_WALL_T, _tl, _th), color=TRAY_C),
    Box("tray_wall_left", pos=(TRAY[0], TRAY[1] + _to, _tz), size=(TRAY_IN, TRAY_WALL_T, _th), color=TRAY_C),
    Box("tray_wall_right", pos=(TRAY[0], TRAY[1] - _to, _tz), size=(TRAY_IN, TRAY_WALL_T, _th), color=TRAY_C),
]

OBJECTS = [*DECK, *WALLS, *TRAY_PARTS, BASKET]

# -- scoring ---------------------------------------------------------------------------------------------------------
UPRIGHT_DEG = 5.0  # standing upright on the tray
SPILL_DEG = 45.0  # tipped further than this at any time: the leaves spill
REST_TOL = 0.001  # base within this of the tray floor
CENTRE_TOL = TRAY_IN / 2 - BASKET_R  # 15 mm: the whole basket inside the tray

# Whole-trial record, cleared only by a scene reset (on_reset), which the robot cannot cause.
_HIST = {"tilt": 0.0}


@parts.on_reset
def _clear(sim):
    _HIST["tilt"] = 0.0


def tilt_deg(quat_xyzw) -> float:
    """Angle between the basket's axis and the vertical."""
    x, y, z, w = np.asarray(quat_xyzw, float) / np.linalg.norm(quat_xyzw)
    up_z = 1 - 2 * (x * x + y * y)
    return math.degrees(math.acos(float(np.clip(up_z, -1.0, 1.0))))


def _record(objects):
    _HIST["tilt"] = max(_HIST["tilt"], tilt_deg(objects["skimmer_basket"].quat_xyzw))


def _on_tray(b) -> bool:
    return bool(
        tilt_deg(b.quat_xyzw) <= UPRIGHT_DEG + 1e-6
        and abs(b.pos[2] - TRAY_FLOOR) <= REST_TOL + 1e-6
        and abs(b.pos[0] - TRAY[0]) <= CENTRE_TOL + 1e-6
        and abs(b.pos[1] - TRAY[1]) <= CENTRE_TOL + 1e-6
        and b.resting()
    )


def success(objects) -> bool:
    """Standing upright on the tray floor, wholly inside the tray, at rest; never tipped past 45 deg."""
    _record(objects)
    return bool(_HIST["tilt"] <= SPILL_DEG + 1e-6 and _on_tray(objects["skimmer_basket"]))


def distance(objects) -> float:
    b = objects["skimmer_basket"].pos
    return float(np.linalg.norm(b - np.array((TRAY[0], TRAY[1], TRAY_FLOOR))))


def score(objects) -> float:
    """Pass/fail: 1.0 exactly when success() holds."""
    return 1.0 if success(objects) else 0.0


# -- demo: the left arm raises the folded handle, then lifts by it ---------------------------------------------------
# Fixed waypoints only. Fingers close along world y, across the grip bar.
TCP_ABOVE_BAR = 0.011  # TCP above the bar's axis: pad tips 2 mm below it, clear of the leaves
STRADDLE = -0.010  # finger target while straddling the bar: pads 20 mm apart
LIFT_TCP_ABOVE_BAR = 0.004  # once upright: pad tips 9 mm below the bar's axis, so it cannot slip out
PINCH = -0.0035  # light pinch: the bar can turn between the pads as the handle swings up
SAG = 0.006  # measured: the hanging basket rides about 6 mm below the commanded TCP
CARRY_BOTTOM = 0.017  # basket base above the deck while carrying (the tray lip is 9 mm)
DROP = 0.007  # base above the tray floor when the fingers open


def _bar(phi):
    """Grip-bar axis (world) with the handle raised phi rad from folded."""
    return (WELL[0], WELL[1] + BAIL_L * math.cos(phi), TOP + PIVOT_Z + BAIL_L * math.sin(phi))


def demo():
    wx, wy = WELL
    tx, ty = TRAY
    bx, by, bz = _bar(0.0)
    up = TCP_ABOVE_BAR
    hang = PIVOT_Z + BAIL_L + LIFT_TCP_ABOVE_BAR  # TCP above the basket's base when it hangs from the upright handle
    carry_z = TOP + DECK_H + CARRY_BOTTOM + hang + SAG
    set_z = TRAY_FLOOR + DROP + hang + SAG
    plan = [
        Waypoint(1.5, (bx, by, TOP + DECK_H + 0.06), STRADDLE, "above the folded bar"),
        Waypoint(1.5, (bx, by, bz + up), STRADDLE, "down into the well, straddling the bar"),
        Waypoint(0.8, (bx, by, bz + up), PINCH, "pinch the bar lightly"),
    ]
    # Push the handle over to its stop just past upright (it lags the arc by about 15 deg), let go, and grip
    # it again lower down, where the bar cannot slip out of the pads.
    for deg in (25, 50, 75, 100, 120):
        x, y, z = _bar(math.radians(deg))
        plan.append(Waypoint(0.8, (x, y, z + up), PINCH, f"swing the handle up toward {deg} deg"))
    plan.append(Waypoint(0.5, plan[-1].pos, STRADDLE, "let go: the handle rests on its stop"))
    x, y, z = _bar(HANDLE_STOP)
    plan += [
        Waypoint(0.8, (x, y, z + 0.03), STRADDLE, "above the standing handle"),
        Waypoint(1.0, (x, y, z + LIFT_TCP_ABOVE_BAR), STRADDLE, "straddle the bar"),
        Waypoint(0.6, (x, y, z + LIFT_TCP_ABOVE_BAR), FINGER_CLOSED, "close on the bar"),
        Waypoint(3.0, (wx, wy, carry_z), FINGER_CLOSED, "lift straight out of the well"),
        Waypoint(1.0, (wx, wy, carry_z), FINGER_CLOSED, "steady"),
        Waypoint(3.0, (tx, ty, carry_z), FINGER_CLOSED, "carry over the tray"),
        Waypoint(2.0, (tx, ty, carry_z), FINGER_CLOSED, "let the swing die"),
        Waypoint(1.5, (tx, ty, set_z), FINGER_CLOSED, "lower onto the tray"),
        Waypoint(0.8, (tx, ty, set_z), STRADDLE, "let go"),
        Waypoint(1.5, (tx, ty, carry_z + 0.01), STRADDLE, "up"),
        Waypoint(1.5, (0.22, 0.25, carry_z + 0.01), STRADDLE, "clear"),
    ]
    return {"left": plan}
