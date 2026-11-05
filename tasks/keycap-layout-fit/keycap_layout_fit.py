"""On the station table: a loose dark-grey keyboard plate section (300 x 45 x 15 mm) lies flat in front of the
arms, its long side running left to right. Four grey key switches stand in a row 27 mm apart on the plate's
left end. Just beyond the plate, a white layout card shows four colored squares lined up with the switches:
red, green, blue, yellow, from left to right. Four loose keycaps (18 x 18 x 10 mm) stand on the table in front
of the left arm, from left to right: blue, yellow, red, green.

Goal: each keycap seated on the switch whose card square has its color, while the plate stays where it is.

Run it:  inspect-robots run --embodiment yam_arena -E task=keycap_layout_fit.py -E object_jitter=0 \\
           --instruction "Fit the four keycaps onto the four switches in the color order shown on the white \\
           card, pressing each one fully down, and keep the keyboard plate where it is." --scorer task_score ...
"""

import math
import os

import numpy as np

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

PITCH = 0.027  # switch spacing: the YAM fingertips reach ~13 mm sideways, so a 19.05 mm keyboard pitch leaves
# no room for them beside a neighbouring keycap
PLATE_SIZE = (0.045, 0.30, 0.015)
PLATE_XY = (0.33, 0.03)
KEYS_Y = 0.125  # centre of the four-switch group; keeps the two hands ~18 cm apart
SWITCH_SIZE = (0.014, 0.014, 0.006)
CAP_SIZE = (0.018, 0.018, 0.010)
CARD_X = 0.40

COLOURS = {"red": (0.80, 0.15, 0.12), "green": (0.15, 0.60, 0.25), "blue": (0.15, 0.35, 0.80), "yellow": (0.95, 0.80, 0.10)}
ORDER = ("red", "green", "blue", "yellow")  # the card, left (+y) to right
TRAY_ORDER = ("blue", "yellow", "red", "green")  # where the keycaps start, left to right
SWITCH_Y = tuple(KEYS_Y + k * PITCH for k in (1.5, 0.5, -0.5, -1.5))
SWITCH_TOP = TOP + PLATE_SIZE[2] + SWITCH_SIZE[2]
SEATED_Z = SWITCH_TOP + CAP_SIZE[2] / 2  # a keycap's centre when it sits flat on its switch

PLATE = Box("plate", pos=(*PLATE_XY, TOP + PLATE_SIZE[2] / 2), size=PLATE_SIZE, mass=0.25, color=(0.20, 0.20, 0.22))
SWITCHES = [
    Box(f"switch_{i}", pos=(PLATE_XY[0], y, TOP + PLATE_SIZE[2] + SWITCH_SIZE[2] / 2), size=SWITCH_SIZE,
        color=(0.35, 0.35, 0.38))
    for i, y in enumerate(SWITCH_Y)
]
CARD = Box("layout_card", pos=(CARD_X, KEYS_Y, TOP + 0.001), size=(0.03, 0.115, 0.002), color=(0.95, 0.95, 0.95))
CARD_SQUARES = [
    Box(f"card_{c}", pos=(CARD_X, y, TOP + 0.0025), size=(0.012, 0.012, 0.001), color=COLOURS[c])
    for c, y in zip(ORDER, SWITCH_Y)
]
CAP_MASS = 0.03  # a real ABS keycap is ~4 g; lighter caps make the simulated finger contacts unstable
CAP_START = {c: (0.25, 0.20 - 0.035 * i, TOP + CAP_SIZE[2] / 2) for i, c in enumerate(TRAY_ORDER)}
KEYCAPS = [Box(f"keycap_{c}", pos=CAP_START[c], size=CAP_SIZE, mass=CAP_MASS, color=COLOURS[c]) for c in TRAY_ORDER]
OBJECTS = [PLATE, *SWITCHES, CARD, *CARD_SQUARES, *KEYCAPS]


SEAT_DZ = (SWITCH_SIZE[2] + CAP_SIZE[2]) / 2  # keycap centre above its switch's centre when seated


def _offsets(objects, i, colour):
    """(sideways offset, height error) of a keycap from seated on switch i, from the two bodies' centres (m)."""
    cap, sw = objects[f"keycap_{colour}"].pos, objects[f"switch_{i}"].pos
    return math.hypot(cap[0] - sw[0], cap[1] - sw[1]), cap[2] - sw[2] - SEAT_DZ


def _keycap(objects, i, colour):
    """0.25 seated (centred within 3 mm and flat on its switch), 0.1 resting on its switch but not seated, else 0."""
    off, dz = _offsets(objects, i, colour)
    on_switch = off < 0.006 and abs(dz) < 0.003
    if on_switch and off < 0.003 and abs(dz) < 0.0015:
        return 0.25
    return 0.1 if on_switch else 0.0


_DEBUG = os.environ.get("KEYCAP_DEBUG") == "1"
_calls = [0]


def _debug(objects):
    """With KEYCAP_DEBUG=1, print each keycap's offsets every 2 s of robot time (score() runs every step at 20 Hz)."""
    _calls[0] += 1
    if _calls[0] % 10:
        return
    rows = []
    for i, c in enumerate(ORDER):
        cap, sw = objects[f"keycap_{c}"].pos, objects[f"switch_{i}"].pos
        rows.append(f"{c} dx {(cap[0] - sw[0]) * 1000:+.1f} dy {(cap[1] - sw[1]) * 1000:+.1f} dz {(cap[2] - sw[2] - SEAT_DZ) * 1000:+.1f}")
    p = objects["plate"].pos
    print(f"KEYCAP step {_calls[0]}: " + " | ".join(rows) + f" | plate {(p[0] - PLATE_XY[0]) * 1000:+.1f},{(p[1] - PLATE_XY[1]) * 1000:+.1f}", flush=True)


def _plate_moved(objects):
    return math.hypot(objects["plate"].pos[0] - PLATE_XY[0], objects["plate"].pos[1] - PLATE_XY[1]) > 0.01


def score(objects):
    """0.25 per keycap seated on the switch matching its card colour, 0.1 if it only rests on that switch;
    capped at 0.5 once the plate has moved more than 10 mm."""
    if _DEBUG:
        _debug(objects)
    s = sum(_keycap(objects, i, c) for i, c in enumerate(ORDER))
    return min(s, 0.5) if _plate_moved(objects) else s


def success(objects):
    return (
        all(_keycap(objects, i, c) == 0.25 for i, c in enumerate(ORDER))
        and not _plate_moved(objects)
        and all(objects[f"keycap_{c}"].resting() for c in ORDER)
    )


def distance(objects):
    """Summed distance of each keycap from its seated pose (m)."""
    return float(sum(
        np.linalg.norm(objects[f"keycap_{c}"].pos - np.array([PLATE_XY[0], SWITCH_Y[i], SEATED_Z]))
        for i, c in enumerate(ORDER)
    ))


# -- scripted demo --------------------------------------------------------------------------------------------
TRAVEL_Z = TOP + 0.10
ACROSS_X = -math.pi / 2  # grip yaw: fingers close along world x, so they never reach toward a neighbouring switch
CAP_OPEN = -0.015  # finger target: pads 30 mm apart, so open fingers clear the neighbouring keycaps on the table
GRIP_TCP_ABOVE_BOTTOM = PAD_TIP_PAST_TCP + 0.004  # pad tips 4 mm above the keycap's underside
RELEASE_GAP = 0.002  # let go with the keycap's underside this far above its switch
CAP_PRESS = 0.0005  # closed pad tips pushed this far into the keycap's top
PLATE_PRESS = 0.0015  # closed pad tips pushed this far into the plate's top
LET_GO_S = 0.6  # seconds to open the fingers over the switch
HOVER = 0.012  # stop this far above the release point while the soft wrist catches up
SETTLE_S = 0.8  # hold still this long before going on
# The keycaps stand only 1 mm apart, and in Isaac the hand trails its target by ~5 mm while moving, leaning
# toward +y. Placing from the -y end first keeps that lean away from the keycaps already in place.
PLACE_ORDER = (3, 2, 1, 0)
# Per-keycap correction of the release point (m), measured in Isaac: the soft wrist joints sag, so the open-loop
# hand lands a few mm off its IK target.
PLACE_FIX = {"red": (0.0, 0.0), "green": (0.0, 0.0), "blue": (0.0, 0.0), "yellow": (0.0, 0.0)}
HOLD_XY = (PLATE_XY[0], PLATE_XY[1] - PLATE_SIZE[1] / 2 + 0.02)  # right hand ~17 cm from the nearest switch:
# the YAM wrist housings stick out about 3 cm toward each other


def _fit(colour, i):
    x, y, _ = CAP_START[colour]
    pick_z = TOP + GRIP_TCP_ABOVE_BOTTOM
    fx, fy = PLACE_FIX.get(colour, (0.0, 0.0))
    sx, sy = PLATE_XY[0] + fx, SWITCH_Y[i] + fy
    release_z = SWITCH_TOP + RELEASE_GAP + GRIP_TCP_ABOVE_BOTTOM
    press_z = SWITCH_TOP + CAP_SIZE[2] + PAD_TIP_PAST_TCP - CAP_PRESS
    return [
        Waypoint(1.5, (x, y, TRAVEL_Z), CAP_OPEN, f"to the {colour} keycap", ACROSS_X),
        Waypoint(1.2, (x, y, pick_z), CAP_OPEN, "descend", ACROSS_X),
        Waypoint(0.8, (x, y, pick_z), FINGER_CLOSED, "grip", ACROSS_X),
        Waypoint(1.0, (x, y, TRAVEL_Z), FINGER_CLOSED, "lift", ACROSS_X),
        Waypoint(1.5, (sx, sy, TRAVEL_Z), FINGER_CLOSED, "carry", ACROSS_X),
        Waypoint(1.2, (sx, sy, release_z + HOVER), FINGER_CLOSED, "lower to hover", ACROSS_X),
        Waypoint(SETTLE_S, (sx, sy, release_z + HOVER), FINGER_CLOSED, "settle", ACROSS_X),
        Waypoint(1.0, (sx, sy, release_z), FINGER_CLOSED, "lower onto the switch", ACROSS_X),
        Waypoint(SETTLE_S, (sx, sy, release_z), FINGER_CLOSED, "settle", ACROSS_X),
        Waypoint(LET_GO_S, (sx, sy, release_z), CAP_OPEN, "let go", ACROSS_X),
        Waypoint(0.5, (sx, sy, release_z + 0.02), CAP_OPEN, "back up", ACROSS_X),
        Waypoint(0.4, (sx, sy, release_z + 0.02), FINGER_CLOSED, "close", ACROSS_X),
        Waypoint(0.8, (sx, sy, press_z), FINGER_CLOSED, "press down", ACROSS_X),
        Waypoint(0.6, (sx, sy, TRAVEL_Z), FINGER_CLOSED, "up", ACROSS_X),
    ]


def demo():
    """Right arm: closed fingers press the plate's right end onto the table for the whole task. Left arm: each
    keycap from the -y end of the row, gripped across x, settled above its switch, released just above its switch, then pressed down."""
    left = [w for i in PLACE_ORDER for w in _fit(ORDER[i], i)]
    left.append(Waypoint(1.5, (0.20, 0.25, TRAVEL_Z), FINGER_OPEN, "clear"))
    press_z = TOP + PLATE_SIZE[2] + PAD_TIP_PAST_TCP - PLATE_PRESS
    duration = sum(w.duration for w in left)
    right = [
        Waypoint(1.5, (*HOLD_XY, TRAVEL_Z), FINGER_CLOSED, "to the plate"),
        Waypoint(1.2, (*HOLD_XY, press_z), FINGER_CLOSED, "press the plate down"),
        Waypoint(duration - 2.7, (*HOLD_XY, press_z), FINGER_CLOSED, "hold it"),
        Waypoint(1.0, (*HOLD_XY, TRAVEL_Z), FINGER_CLOSED, "let go"),
    ]
    return {"left": left, "right": right}
