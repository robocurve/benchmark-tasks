"""Drill a shallow hole at the centre of the red target on a secured wood block.
The right arm holds the drill; the left arm pulls its spring-loaded trigger.
Feed the rotating 6 mm bit down to make a 10 to 13 mm deep hole, then withdraw
the bit and release the trigger. Maintain alignment while pushing against the
wood's resistance. The reduced cutting model and its limits are in the README.

Run with this folder's run.py, which adds the physical trigger joint and robot-contact
measurements to the standard yam_arena task-file interface.
"""

import numpy as np

from yam_arena.task_file import Asset, Box, Cylinder, TABLE_TOP
from yam_arena.urdf import axis_angle_matrix, quat_xyzw_to_matrix

SCENE = "station"
TOP = TABLE_TOP[SCENE]
TARGET_XY = np.array([0.26, -0.08])
WOOD_SIZE = (0.18, 0.14, 0.03)
SURFACE_Z = TOP + WOOD_SIZE[2]
TARGET_RADIUS = 0.010
BIT_TIP_LOCAL = np.array([-0.127, -0.0695, 0.0])
BIT_AXIS_LOCAL = np.array([-1.0, 0.0, 0.0])
TRIGGER_LOCAL = np.array([-0.017, -0.017, 0.0])
TRIGGER_SIZE = (0.014, 0.020, 0.040)
TRIGGER_TRAVEL = 0.005
TRIGGER_THRESHOLD = 0.003
MAX_TILT_DEG = 12.0
CONTACT_Z_TOLERANCE = 0.004
SUCCESS_HOLD_S = 0.5
HOLE_POSITION_TOLERANCE = 0.003
HOLE_MIN_DEPTH = 0.010
HOLE_MAX_DEPTH = 0.013
WITHDRAW_CLEARANCE = 0.015

# Match the original asset's horizontal extent exactly, keeping model coordinates in metres.
DRILL = Asset("drill", asset="power_drill", xy=(0.25, -0.30), extent=0.18683595,
              mass=0.5, yaw=np.pi / 2)
ASSET_CENTRE = np.array([0.000000195, 0.000000435, 0.00000032])
# Adding the bit shifts the spawned drill's bounding-box centre by 17.41 mm in -X.
MODEL_CENTRE = np.array([-0.017409625, 0.000000435, 0.00000032])
MODEL_HALF_Z = 0.02859581
INITIAL_DRILL_ORIGIN = np.array([*DRILL.xy, TOP + MODEL_HALF_Z + 0.001]) - ASSET_CENTRE
INITIAL_DRILL_ROTATION = axis_angle_matrix(np.array([0., 0., 1.]), DRILL.yaw)
INITIAL_TRIGGER_POS = INITIAL_DRILL_ORIGIN + INITIAL_DRILL_ROTATION @ TRIGGER_LOCAL

WOOD = Box("wood", pos=(float(TARGET_XY[0]), float(TARGET_XY[1]), TOP + WOOD_SIZE[2] / 2), size=WOOD_SIZE,
           color=(0.78, 0.58, 0.32))
TARGET = Cylinder("target", pos=(*TARGET_XY, SURFACE_Z + 0.0002), radius=TARGET_RADIUS,
                  height=0.0004, color=(0.9, 0.04, 0.03))
TRIGGER = Box("trigger", pos=tuple(float(v) for v in INITIAL_TRIGGER_POS), size=TRIGGER_SIZE,
              mass=0.015, yaw=DRILL.yaw, color=(0.9, 0.04, 0.03))
OBJECTS = [WOOD, TARGET, DRILL, TRIGGER]


def holding_handle(tcp, handle, gap, finger_forces):
    """Both gripper pads must contact the drill; nearby single-finger contact is insufficient."""
    forces = np.asarray(finger_forces, dtype=float)
    return bool(
        forces.shape == (2,)
        and np.all(np.isfinite(forces))
        and np.linalg.norm(np.asarray(tcp) - np.asarray(handle)) < 0.055
        and 0.0 <= gap < 0.075
        and np.min(forces) > 0.05
    )


def measure(objects, robot=None):
    """Pure geometry plus measured robot contacts; no scripted timing enters the score."""
    drill, trigger = objects["drill"], objects["trigger"]
    rotation = quat_xyzw_to_matrix(np.asarray(drill.quat_xyzw))
    origin = np.asarray(drill.pos) - rotation @ MODEL_CENTRE
    tip = origin + rotation @ BIT_TIP_LOCAL
    axis = rotation @ BIT_AXIS_LOCAL
    trigger_offset = rotation.T @ (np.asarray(trigger.pos) - origin) - TRIGGER_LOCAL
    tilt = float(np.degrees(np.arccos(np.clip(-axis[2], -1., 1.))))
    xy_error = float(np.linalg.norm(tip[:2] - TARGET_XY))
    z_error = float(tip[2] - SURFACE_Z)
    travel = float(trigger_offset[0])
    attached = bool(np.linalg.norm(trigger_offset[1:]) < 0.003 and -0.002 <= travel <= TRIGGER_TRAVEL + 0.002)
    robot = robot or {}
    right_holding = bool(robot.get("right_holding", False))
    left_pressing = bool(robot.get("left_trigger_contact", False))
    wood_contact = bool(xy_error <= TARGET_RADIUS and abs(z_error) <= CONTACT_Z_TOLERANCE
                        and robot.get("bit_wood_contact", False))
    vertical = tilt <= MAX_TILT_DEG
    pressed = bool(attached and travel >= TRIGGER_THRESHOLD and left_pressing)
    depth = float(robot.get('cut_depth_m',0.))
    qualified_depth = float(robot.get('qualified_depth_m',0.))
    entry = robot.get('hole_entry_m')
    bore_axis = robot.get('bore_axis')
    entry_error = None if entry is None else float(np.linalg.norm(np.asarray(entry)[:2]-TARGET_XY))
    bore_tilt = None if bore_axis is None else float(np.degrees(np.arccos(np.clip(-np.asarray(bore_axis)[2],-1,1))))
    hole_position_ok = entry_error is not None and entry_error <= HOLE_POSITION_TOLERANCE
    hole_direction_ok = bore_tilt is not None and bore_tilt <= MAX_TILT_DEG
    damaged = depth > HOLE_MAX_DEPTH or (depth >= .002 and not (hole_position_ok and hole_direction_ok))
    hole_complete = bool(hole_position_ok and hole_direction_ok and HOLE_MIN_DEPTH <= depth <= HOLE_MAX_DEPTH
                         and qualified_depth >= HOLE_MIN_DEPTH)
    released = bool(attached and travel <= .001)
    withdrawn = bool(z_error >= WITHDRAW_CLEARANCE)
    complete = bool(right_holding and hole_complete and withdrawn and released and not damaged)
    partial = 0.0
    if right_holding and not damaged:
        partial = 0.25
        if depth >= .002 and qualified_depth >= .002 and hole_position_ok and hole_direction_ok:
            partial = 0.5
            if hole_complete:
                partial = 0.75
    return {
        "tip_m": tip.tolist(), "drill_origin_m": origin.tolist(),
        "tilt_deg": tilt, "target_error_m": xy_error, "surface_error_m": z_error,
        "trigger_travel_m": travel, "trigger_attached": attached,
        "right_holding": right_holding, "left_trigger_contact": left_pressing,
        "tip_on_target": wood_contact, "vertical": vertical,
        "trigger_pressed": pressed, "success_now": complete, "partial_score": partial,
        "hole_entry_error_m": entry_error, "hole_tilt_deg": bore_tilt,
        "hole_complete": hole_complete, "damaged_workpiece": bool(damaged),
        "bit_withdrawn": withdrawn, "trigger_released": released,
        **{k: v for k, v in robot.items() if k not in ("right_holding", "left_trigger_contact")},
    }


_robot = {}


def success(objects):
    return bool(measure(objects, _robot)["success_now"])


def score(objects):
    return float(measure(objects, _robot)["partial_score"])


def distance(objects):
    m = measure(objects, _robot)
    return float(max(0.,HOLE_MIN_DEPTH-float(m.get('qualified_depth_m',0.))))
