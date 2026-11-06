"""On the station table, a hospital bedside: the patient's forearm rests on a pillow on the sheet with the index finger
extended past its edge. The pulse oximeter's finger probe has slipped off and lies on the sheet beside the hand. The
bedside monitor's red alarm lamp is lit; its yellow silence button is on top, beside the lamps.
"""

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from partkit import Slider, env, on_reset, part  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box, Cylinder  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

SHEET_C, PILLOW_C, SKIN, NAIL = (0.62, 0.78, 0.88), (0.96, 0.96, 0.94), (0.86, 0.66, 0.52), (0.93, 0.80, 0.76)
PROBE_C, LED, CABLE = (0.20, 0.32, 0.55), (0.95, 0.15, 0.10), (0.30, 0.30, 0.32)
CASE, SCREEN, WAVE = (0.80, 0.80, 0.78), (0.08, 0.10, 0.12), (0.20, 0.85, 0.40)
RED, GREEN, YELLOW = (0.95, 0.10, 0.08), (0.10, 0.85, 0.25), (0.98, 0.82, 0.10)

SHEET_T = 0.01
SHEET = TOP + SHEET_T
PILLOW_H = 0.035
PILLOW = SHEET + PILLOW_H
HAND_Y = -0.20

FINGER_Y, FINGER_TIP, FINGER_LEN, FINGER_W = HAND_Y + 0.025, 0.245, 0.065, 0.016
FINGER_Z = PILLOW + 0.0125
FINGER_TOP = FINGER_Z + FINGER_W / 2

PATIENT = [
    Box("bed_sheet", pos=(0.42, -0.10, TOP + SHEET_T / 2), size=(0.56, 0.46, SHEET_T), color=SHEET_C),
    Box("pillow", pos=(0.43, HAND_Y, SHEET + PILLOW_H / 2), size=(0.14, 0.13, PILLOW_H), color=PILLOW_C),
    Box("forearm", pos=(0.51, HAND_Y, PILLOW + 0.02), size=(0.22, 0.065, 0.04), color=SKIN),
    Box("palm", pos=(0.355, HAND_Y, PILLOW + 0.0125), size=(0.09, 0.075, 0.025), color=SKIN),
    Box("index_finger", pos=(FINGER_TIP + FINGER_LEN / 2, FINGER_Y, FINGER_Z), size=(FINGER_LEN, FINGER_W, FINGER_W),
        color=SKIN),
    Box("index_nail", pos=(FINGER_TIP + 0.007, FINGER_Y, FINGER_TOP + 0.0005), size=(0.01, 0.012, 0.001),
        color=NAIL),
    Box("curled_fingers", pos=(0.305, HAND_Y - 0.012, PILLOW + 0.011), size=(0.022, 0.045, 0.02), color=SKIN),
    Box("thumb", pos=(0.335, HAND_Y + 0.05, PILLOW + 0.01), size=(0.04, 0.018, 0.018), color=SKIN),
    Box("id_wristband", pos=(0.43, HAND_Y, PILLOW + 0.02), size=(0.014, 0.069, 0.043), color=(0.97, 0.97, 0.97)),
]

PROBE_L, PROBE_W, WALL, BLOCK_H, CAVITY_H = 0.035, 0.030, 0.004, 0.018, 0.022
PROBE_START = (0.22, -0.30)
PROBE_START_YAW = 0.3
PROBE = part(
    "finger_probe", pos=(*PROBE_START, SHEET + CAVITY_H), yaw=PROBE_START_YAW, mass=0.03, color=PROBE_C,
    boxes=[((0, 0, BLOCK_H / 2), (PROBE_L, PROBE_W, BLOCK_H), PROBE_C),
           ((0, PROBE_W / 2 - WALL / 2, -CAVITY_H / 2), (PROBE_L, WALL, CAVITY_H), PROBE_C),
           ((0, -PROBE_W / 2 + WALL / 2, -CAVITY_H / 2), (PROBE_L, WALL, CAVITY_H), PROBE_C),
           ((-PROBE_L / 2 + WALL / 2, 0, -CAVITY_H / 2), (WALL, PROBE_W - 2 * WALL, CAVITY_H), PROBE_C),
           ((-0.004, 0, BLOCK_H + 0.0005), (0.008, 0.008, 0.001), LED),
           ((PROBE_L / 2 + 0.015, 0, 0.012), (0.03, 0.006, 0.006), CABLE)],
)

MON_X, MON_Y, MON_H = 0.44, 0.24, 0.10
LAMP_TRAVEL, PRESSED = 0.013, -0.004
LAMPS = {"red_lamp": ((0.035, -0.03), RED), "green_lamp": ((0.035, 0.0), GREEN)}
SILENCE_AT = (-0.03, -0.025)


def _top_slider(name, at, color, size, travel, rate):
    return Slider(name, rest=(*at, MON_H / 2), travel=travel, rate=rate, damping=20.0,
                  boxes=(((*at, MON_H / 2 + 0.006), (size, size, 0.012), color),))


MONITOR = part(
    "monitor", pos=(MON_X, MON_Y, TOP + MON_H / 2), mass=8.0, color=CASE,
    boxes=[((0, 0, 0), (0.16, 0.10, MON_H), CASE),
           ((-0.0805, 0, 0.005), (0.001, 0.085, 0.065), SCREEN),
           ((-0.0812, 0, 0.012), (0.001, 0.07, 0.003), WAVE),
           ((-0.0812, 0, -0.012), (0.001, 0.05, 0.003), (0.30, 0.70, 0.95))],
    sliders=(*[_top_slider(n, at, c, 0.02, LAMP_TRAVEL, 3000.0) for n, (at, c) in LAMPS.items()],
             _top_slider("silence", SILENCE_AT, YELLOW, 0.025, 0.008, 200.0)),
)

DRESSING = [
    Cylinder("water_cup", pos=(0.58, 0.06, TOP + 0.045), radius=0.03, height=0.09, color=(0.85, 0.90, 0.95)),
    Box("call_bell", pos=(0.60, -0.36, TOP + 0.012), size=(0.06, 0.04, 0.024), color=(0.85, 0.20, 0.20)),
    Box("tissue_box", pos=(0.62, 0.30, TOP + 0.03), size=(0.10, 0.06, 0.06), color=(0.55, 0.75, 0.60)),
]

OBJECTS = [*PATIENT, PROBE, MONITOR, *DRESSING]

_S = {"early": False, "silenced": False, "lamps": None}


@on_reset
def _reset(environment=None):
    _S.update(early=False, silenced=False, lamps=None)


def _tilt_deg(state):
    x, y, _, _ = state.quat_xyzw
    return float(np.degrees(np.arccos(np.clip(1 - 2 * (x * x + y * y), -1.0, 1.0))))


def depth(probe):
    """How far (m) the fingertip is inside the probe, or 0 if the probe isn't sitting level over the finger."""
    tip = probe.to_local((FINGER_TIP, FINGER_Y, FINGER_Z))
    on_finger = abs(tip[1]) < PROBE_W / 2 - WALL and abs(probe.pos[2] - FINGER_TOP) < 0.004
    if not on_finger or _tilt_deg(probe) > 15:
        return 0.0
    return float(max(PROBE_L / 2 - tip[0], 0.0))


def _show(red, green):
    if _S["lamps"] == (red, green):
        return
    _S["lamps"] = (red, green)
    monitor = env().unwrapped.scene[MONITOR.name]
    target = monitor.data.joint_pos_target.clone()
    names = list(monitor.joint_names)
    target[:, names.index("red_lamp_slide")] = 0.0 if red else -LAMP_TRAVEL
    target[:, names.index("green_lamp_slide")] = 0.0 if green else -LAMP_TRAVEL
    monitor.set_joint_position_target(target)


def score(objects):
    """0 if the silence button was pressed while the probe was off the finger. Otherwise 1.0 for the probe sitting
    level on the finger with the fingertip at least 1.5 cm inside it, and 0 for anything else."""
    reading = depth(objects[PROBE.name]) >= 0.015
    if objects[MONITOR.name].sliders["silence"] < PRESSED:
        _S["early"] = _S["early"] or not reading
        _S["silenced"] = True
    _show(red=not reading and not _S["silenced"], green=reading)
    return 1.0 if reading and not _S["early"] else 0.0


def demo():
    """The right arm picks the probe up by its body, turns it square to the finger and lowers it over the fingertip,
    then moves clear. The alarm clears by itself; the silence button is not touched."""
    grip = 0.008
    travel = TOP + 0.17
    x0, y0 = PROBE_START
    pick = (x0, y0, SHEET + CAVITY_H + grip)
    seat_x = FINGER_TIP + PROBE_L / 2 - WALL - 0.002
    hover = (seat_x, FINGER_Y, FINGER_TOP + grip + 0.012)
    seat = (seat_x, FINGER_Y, FINGER_TOP + grip + 0.002)
    steps = [
        (1.5, (x0, y0, travel), FINGER_OPEN, "above the probe", PROBE_START_YAW),
        (1.5, pick, FINGER_OPEN, "descend", PROBE_START_YAW),
        (0.6, pick, FINGER_OPEN, "settle", PROBE_START_YAW),
        (0.8, pick, FINGER_CLOSED, "close", PROBE_START_YAW),
        (1.0, (x0, y0, travel), FINGER_CLOSED, "lift", PROBE_START_YAW),
        (2.0, (seat_x, FINGER_Y, travel), FINGER_CLOSED, "carry over the fingertip", 0.0),
        (1.5, hover, FINGER_CLOSED, "lower to just above the finger", 0.0),
        (0.8, hover, FINGER_CLOSED, "steady", 0.0),
        (1.5, seat, FINGER_CLOSED, "lower onto the finger", 0.0),
        (0.6, seat, FINGER_OPEN, "release", 0.0),
        (1.0, (seat_x, FINGER_Y, travel), FINGER_OPEN, "retreat", 0.0),
        (1.5, (0.12, -0.42, TOP + 0.20), FINGER_OPEN, "move clear of the camera's view", 0.0),
    ]
    return {"right": [Waypoint(*step) for step in steps]}
