"""On the station table, a care-home bedside: the resident's place is a green placemat at the far side with a light
blue tray in front of it. A peak flow meter (a white tube with its mouthpiece towards the arms) lies in front of the left
arm; its red marker still sits high up the scale from the last reading.
"""

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from partkit import part  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box, Cylinder  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

TUBE_C, MOUTH_C, MARK_C = (0.94, 0.95, 0.96), (0.45, 0.47, 0.50), (0.85, 0.12, 0.12)
SCALE_C, TICK_C = (0.98, 0.98, 0.95), (0.35, 0.35, 0.38)
MAT_C, TRAY_C, CLOTH = (0.45, 0.70, 0.45), (0.55, 0.75, 0.92), (0.86, 0.82, 0.75)

TUBE_L, TUBE_W, TUBE_H = 0.15, 0.04, 0.035
SLOT_W, SLOT_D, SLOT_HALF_L = 0.015, 0.01, 0.065      # the marker's slot, sunk into the top of the tube
FLOOR = TUBE_H / 2 - SLOT_D                            # slot floor height in the meter frame
MARK = (0.025, 0.012, 0.02)
ZERO_X = -SLOT_HALF_L + MARK[0] / 2                    # marker centre against the zero end of the slot
START_X = 0.04                                         # where the last reading left it
WALL_W = (TUBE_W - SLOT_W) / 2
METER_XY = (0.22, 0.20)
METER = part(
    "peak_flow_meter", pos=(*METER_XY, TOP + TUBE_H / 2), mass=0.15, color=TUBE_C, friction=1.2,
    boxes=[((0, 0, (FLOOR - TUBE_H / 2) / 2), (TUBE_L, TUBE_W, FLOOR + TUBE_H / 2), TUBE_C),
           *[((0, s * (SLOT_W + WALL_W) / 2, FLOOR + SLOT_D / 2), (TUBE_L, WALL_W, SLOT_D), TUBE_C) for s in (1, -1)],
           *[((s * (SLOT_HALF_L + (TUBE_L / 2 - SLOT_HALF_L) / 2), 0, FLOOR + SLOT_D / 2),
              (TUBE_L / 2 - SLOT_HALF_L, SLOT_W, SLOT_D), TUBE_C) for s in (1, -1)],
           ((-TUBE_L / 2 - 0.015, 0, -0.004), (0.03, 0.028, 0.022), MOUTH_C),
           ((0, (SLOT_W + WALL_W) / 2, TUBE_H / 2 + 0.0005), (2 * SLOT_HALF_L, WALL_W - 0.003, 0.001), SCALE_C),
           *[((x, (SLOT_W + WALL_W) / 2, TUBE_H / 2 + 0.0011), (0.0015, 0.006, 0.0004), TICK_C)
             for x in np.linspace(-0.058, 0.058, 12).tolist()]],
)
MARKER = part("marker", pos=(METER_XY[0] + START_X, METER_XY[1], TOP + TUBE_H / 2 + FLOOR + MARK[2] / 2 + 0.001),
              mass=0.004, color=MARK_C, friction=0.55, boxes=[((0, 0, 0), MARK, MARK_C)])

PATIENT_XY = (0.60, 0.20)
TRAY_XY, TRAY_INSIDE = (0.42, 0.20), (0.21, 0.09)
SCENE_OBJECTS = [
    Box("bedside_cloth", pos=(0.42, 0.12, TOP + 0.0005), size=(0.46, 0.46, 0.001), color=CLOTH),
    Box("resident_placemat", pos=(*PATIENT_XY, TOP + 0.0015), size=(0.12, 0.24, 0.002), color=MAT_C),
    Cylinder("resident_cup", pos=(0.62, 0.29, TOP + 0.045), radius=0.03, height=0.09, color=(0.90, 0.90, 0.95)),
    Box("tray_floor", pos=(*TRAY_XY, TOP + 0.002), size=(0.22, 0.10, 0.004), color=TRAY_C),
    *[Box(f"tray_rim_{i}", pos=(TRAY_XY[0] + dx, TRAY_XY[1] + dy, TOP + 0.007), size=size, color=TRAY_C)
      for i, (dx, dy, size) in enumerate([(0.1125, 0, (0.005, 0.10, 0.01)), (-0.1125, 0, (0.005, 0.10, 0.01)),
                                          (0, 0.0475, (0.22, 0.005, 0.01)), (0, -0.0475, (0.22, 0.005, 0.01))])],
    Box("reading_diary", pos=(0.25, 0.02, TOP + 0.003), size=(0.10, 0.14, 0.006), color=(0.30, 0.45, 0.70)),
    Box("diary_page", pos=(0.25, 0.02, TOP + 0.0062), size=(0.085, 0.125, 0.0004), color=(0.97, 0.97, 0.94)),
    Box("tissue_box", pos=(0.62, -0.12, TOP + 0.03), size=(0.11, 0.06, 0.06), color=(0.60, 0.75, 0.85)),
    Cylinder("water_jug", pos=(0.60, -0.30, TOP + 0.07), radius=0.04, height=0.14, color=(0.82, 0.90, 0.95)),
]

OBJECTS = [*SCENE_OBJECTS, METER, MARKER]


def _tilt_deg(state):
    return float(np.degrees(np.arccos(np.clip(state.rotation()[2, 2], -1.0, 1.0))))


def marker_offset(objects):
    """The marker's position along the scale in the meter frame (m), or None if it is out of its groove."""
    local = objects[METER.name].to_local(objects[MARKER.name].pos)
    in_groove = abs(local[1]) < 0.005 and abs(local[2] - (FLOOR + MARK[2] / 2)) < 0.006
    return float(local[0]) if in_groove else None


def _reset(objects):
    x = marker_offset(objects)
    return x is not None and x < ZERO_X + 0.01


def _on_tray(state):
    dx, dy = np.abs(state.pos[:2] - TRAY_XY)
    return bool(dx < TRAY_INSIDE[0] / 2 and dy < TRAY_INSIDE[1] / 2 and state.pos[2] < TOP + TUBE_H / 2 + 0.012
                and _tilt_deg(state) < 15 and state.resting())


def _mouthpiece_to_resident(state):
    mouth = -state.rotation()[:2, 0]
    to_patient = np.array(PATIENT_XY) - state.pos[:2]
    return bool(mouth @ to_patient / (np.linalg.norm(mouth) * np.linalg.norm(to_patient) + 1e-9) > 0.5)


def score(objects):
    """1.0 for the marker back at zero and the meter on the tray, mouthpiece towards the resident; 0.75 the same with
    the mouthpiece turned away; 0.5 for the marker reset but the meter not on the tray; 0.25 for the meter on the tray
    with the marker not reset; 0 otherwise."""
    meter = objects[METER.name]
    reset, on_tray = _reset(objects), _on_tray(meter)
    if reset and on_tray:
        return 1.0 if _mouthpiece_to_resident(meter) else 0.75
    return 0.5 if reset else 0.25 if on_tray else 0.0


GRIP = TUBE_H / 2 + 0.008
TRAVEL = TOP + 0.20


def _quarter(xy, start_yaw):
    x, y = xy
    return [
        (1.5, (x, y, TRAVEL), FINGER_OPEN, "above the meter", start_yaw),
        (1.5, (x, y, TOP + GRIP), FINGER_OPEN, "descend", start_yaw),
        (0.6, (x, y, TOP + GRIP), FINGER_OPEN, "settle", start_yaw),
        (0.8, (x, y, TOP + GRIP), FINGER_CLOSED, "close", start_yaw),
        (1.0, (x, y, TOP + GRIP + 0.04), FINGER_CLOSED, "lift", start_yaw),
        (1.5, (x, y, TOP + GRIP + 0.04), FINGER_CLOSED, "turn a quarter", start_yaw - math.pi / 2),
        (1.0, (x, y, TOP + GRIP + 0.003), FINGER_CLOSED, "set down", start_yaw - math.pi / 2),
        (0.6, (x, y, TOP + GRIP + 0.003), FINGER_OPEN, "release", start_yaw - math.pi / 2),
        (1.0, (x, y, TRAVEL), FINGER_OPEN, "up", start_yaw - math.pi / 2),
    ]


def demo():
    """The left arm turns the meter half a turn, in two quarter turns, so its mouthpiece faces the resident, carries it
    to the tray, then pushes the marker back along its slot to the zero stop."""
    x0, y0 = METER_XY
    grip_xy = (x0 - 0.02, y0)
    carry = [
        (1.5, (*grip_xy, TRAVEL), FINGER_OPEN, "above the meter", 0.0),
        (1.5, (*grip_xy, TOP + GRIP), FINGER_OPEN, "descend", 0.0),
        (0.6, (*grip_xy, TOP + GRIP), FINGER_OPEN, "settle", 0.0),
        (0.8, (*grip_xy, TOP + GRIP), FINGER_CLOSED, "close", 0.0),
        (1.5, (*grip_xy, TOP + 0.10), FINGER_CLOSED, "lift", 0.0),
        (3.0, (*TRAY_XY, TOP + 0.10), FINGER_CLOSED, "carry to the tray", 0.0),
        (1.5, (*TRAY_XY, TOP + 0.004 + GRIP + 0.004), FINGER_CLOSED, "lower onto the tray", 0.0),
        (0.6, (*TRAY_XY, TOP + 0.004 + GRIP + 0.004), FINGER_OPEN, "release", 0.0),
        (1.0, (*TRAY_XY, TRAVEL), FINGER_OPEN, "retreat", 0.0),
    ]
    centre_x = TRAY_XY[0] - 0.02
    marker_x = centre_x - START_X
    behind = marker_x - 0.035
    tip = TOP + 0.004 + TUBE_H + 0.005 + 0.008
    zero = centre_x - ZERO_X - MARK[0] / 2 + 0.002
    push = [
        (1.5, (behind, y0, TRAVEL), FINGER_CLOSED, "above, behind the marker", 0.0),
        (1.5, (behind, y0, tip), FINGER_CLOSED, "lower behind the marker", 0.0),
        (3.0, (zero, y0, tip), FINGER_CLOSED, "push the marker back to zero", 0.0),
        (1.0, (zero, y0, TRAVEL), FINGER_CLOSED, "lift clear", 0.0),
        (1.5, (0.12, 0.42, TOP + 0.20), FINGER_OPEN, "move clear", 0.0),
    ]
    steps = [*_quarter(grip_xy, 0.0), *_quarter(grip_xy, math.pi / 2), *carry, *push]
    return {"left": [Waypoint(*s) for s in steps]}
