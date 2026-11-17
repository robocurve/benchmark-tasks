"""A secured 180 x 140 x 30 mm wood block sits on the table with a red target on its top.
A cordless drill with a 35 mm blunt bit lies beside it. The right arm must pick up the
drill and place its bit vertically on the red target; the left arm must pull the red
spring-loaded trigger while the bit stays on the target. No cutting or hole is simulated.

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
    complete = right_holding and wood_contact and vertical and pressed
    # Partial credit is conditional on the right arm actually holding the tool.
    partial = 0.0
    if right_holding:
        partial = 0.25
        if vertical:
            partial = 0.5
            if wood_contact:
                partial = 0.75
    return {
        "tip_m": tip.tolist(), "drill_origin_m": origin.tolist(),
        "tilt_deg": tilt, "target_error_m": xy_error, "surface_error_m": z_error,
        "trigger_travel_m": travel, "trigger_attached": attached,
        "right_holding": right_holding, "left_trigger_contact": left_pressing,
        "tip_on_target": wood_contact, "vertical": vertical,
        "trigger_pressed": pressed, "success_now": complete, "partial_score": partial,
        **{k: v for k, v in robot.items() if k not in ("right_holding", "left_trigger_contact")},
    }


_robot = {}


def success(objects):
    return bool(measure(objects, _robot)["success_now"])


def score(objects):
    return float(measure(objects, _robot)["partial_score"])


def distance(objects):
    m = measure(objects, _robot)
    return float(np.hypot(m["target_error_m"], m["surface_error_m"]))
