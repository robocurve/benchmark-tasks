"""On the station table: a cordless power drill lying on its side in front of the right arm, and a small
red target dot on the table. Pick up the drill and hold it with the chuck pointing straight down at the dot."""

import json
import os
import time
from pathlib import Path

import numpy as np

from yam_arena import ik
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint, grip_rotation
from yam_arena.task_file import TABLE_TOP, Asset, Cylinder
from yam_arena.urdf import axis_angle_matrix, quat_xyzw_to_matrix

SCENE = "station"
TOP = TABLE_TOP[SCENE]
TARGET_XY = np.array([0.30, -0.15])

DRILL = Asset("drill", asset="power_drill", xy=(0.25, -0.30), extent=0.187, mass=0.5, yaw=np.pi / 2)
TARGET = Cylinder("target", pos=(*TARGET_XY, TOP + 0.0005), radius=0.015, height=0.001, color=(0.9, 0.1, 0.1))
OBJECTS = [DRILL, TARGET]

# From inspecting power_drill_physics.usd (model frame, metres): the chuck end and the bit direction.
TIP_LOCAL = np.array([-0.092, -0.0695, 0.0])
AXIS_LOCAL = np.array([-1.0, 0.0, 0.0])
# Grasp adjustment in the drill's model XY frame (m): negative Y goes up the handle toward the head.
GRASP_OFFSET_XY = np.array([0.0, -0.020])

LIFT_CLEARANCE_M = 0.020
MAX_TILT_DEG = 15.0
SUCCESS_AIM_M = 0.040
PARTIAL_AIM_M = 0.050
SUCCESS_HOLD_S = 1.0

# Directional extrema of these support vertices match the source USD mesh. This avoids treating the
# empty corners of the drill's L-shaped bounding box as physical parts of the drill.
_GEOMETRY = json.loads(Path(__file__).with_name("drill_geometry.json").read_text())
if _GEOMETRY["asset"] != DRILL.asset:
    raise ValueError("drill_geometry.json must describe the configured drill asset")
_MODEL_CENTRE = (np.asarray(_GEOMETRY["bounds_min_m"]) + np.asarray(_GEOMETRY["bounds_max_m"])) / 2
_MODEL_SCALE = DRILL.extent / _GEOMETRY["reference_horizontal_extent_m"]
# ObjectState.pos is the transformed local geometry centre, so offsets must use that same origin.
_DRILL_POINTS_LOCAL = (np.asarray(_GEOMETRY["vertices_m"]) - _MODEL_CENTRE) * _MODEL_SCALE
_TIP_OFFSET_LOCAL = (TIP_LOCAL - _MODEL_CENTRE) * _MODEL_SCALE

_diagnostic_calls = 0
_last_diagnostic_flags = None


def _rotation_matrix(q_xyzw):
    q = np.asarray(q_xyzw, dtype=float)
    norm = np.linalg.norm(q)
    if q.shape != (4,) or not np.all(np.isfinite(q)) or not np.isfinite(norm) or norm == 0:
        raise ValueError("quat_xyzw must contain four finite values with a nonzero finite norm")
    return quat_xyzw_to_matrix(q)


def _rotate(q_xyzw, v):
    """Rotate vector v by quaternion (x, y, z, w)."""
    return np.asarray(v, dtype=float) @ _rotation_matrix(q_xyzw).T


def _measure(drill):
    """Measure the physical drill geometry in world coordinates, independently of the box approximation."""
    rotation = _rotation_matrix(drill.quat_xyzw)
    pos = np.asarray(drill.pos, dtype=float)
    if pos.shape != (3,) or not np.all(np.isfinite(pos)):
        raise ValueError("drill position must contain three finite values")
    world_points = pos + _DRILL_POINTS_LOCAL @ rotation.T
    clearance = float(world_points[:, 2].min() - TOP)
    axis = rotation @ AXIS_LOCAL
    tip = pos + rotation @ _TIP_OFFSET_LOCAL
    tilt = float(np.degrees(np.arccos(np.clip(-axis[2], -1.0, 1.0))))
    miss = 1.0
    if axis[2] < -0.1:
        travel = (TOP - tip[2]) / axis[2]
        if travel >= 0.0:  # A forward ray must reach the table; ignore intersections behind the chuck.
            hit = tip + travel * axis
            miss = float(np.linalg.norm(hit[:2] - TARGET_XY))

    speed = float(np.linalg.norm(drill.lin_vel))
    spin = float(np.linalg.norm(drill.ang_vel))
    lifted = clearance > LIFT_CLEARANCE_M
    down = tilt < MAX_TILT_DEG
    near = miss < PARTIAL_AIM_M
    exact = miss < SUCCESS_AIM_M
    partial = 0.25 * lifted + 0.25 * (lifted and down) + 0.25 * (lifted and down and near)
    return {
        "clearance_m": clearance,
        "bbox_clearance_m": float(drill.low[2] - TOP),
        "tilt_deg": tilt,
        "aim_error_m": miss,
        "linear_speed_m_s": speed,
        "angular_speed_rad_s": spin,
        "tip_world_m": tip.tolist(),
        "axis_world": axis.tolist(),
        "lifted": lifted,
        "pointing_down": down,
        "aim_near": near,
        "aim_exact": exact,
        "success_now": lifted and down and exact,
        "partial_score": float(partial),
    }


def _aim(drill):
    """(tilt from straight down in degrees, forward chuck-ray miss distance in metres)."""
    measured = _measure(drill)
    return measured["tilt_deg"], measured["aim_error_m"]


def _report(measured):
    """Show score gates periodically; DRILL_SCORING_LOG optionally saves every step as JSONL."""
    global _diagnostic_calls, _last_diagnostic_flags
    _diagnostic_calls += 1
    flags = tuple(measured[key] for key in ("lifted", "pointing_down", "aim_near", "aim_exact"))
    if _diagnostic_calls % 20 == 1 or flags != _last_diagnostic_flags:
        print(
            f"[drill score] step={_diagnostic_calls} partial={measured['partial_score']:.2f} "
            f"clearance={measured['clearance_m'] * 100:.2f}cm "
            f"box={measured['bbox_clearance_m'] * 100:.2f}cm "
            f"tilt={measured['tilt_deg']:.2f}deg aim={measured['aim_error_m'] * 100:.2f}cm "
            f"speed={measured['linear_speed_m_s']:.4f}m/s spin={measured['angular_speed_rad_s']:.4f}rad/s "
            f"lifted={measured['lifted']} success_now={measured['success_now']}",
            flush=True,
        )
    _last_diagnostic_flags = flags
    log_path = os.environ.get("DRILL_SCORING_LOG")
    if log_path:
        path = Path(log_path).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)
        row = {"step": _diagnostic_calls, "wall_time_s": time.time(), **measured}
        with path.open("a") as stream:
            stream.write(json.dumps(row, allow_nan=False) + "\n")


def success(objects):
    """Lifted off the table, within 15 degrees of straight down, and aimed within 4 cm of the dot."""
    return bool(_measure(objects["drill"])["success_now"])


def distance(objects):
    """Metres from the target to the forward chuck-axis intersection with the tabletop."""
    return _measure(objects["drill"])["aim_error_m"]


def score(objects):
    """Partial credit: 0.25 for lifting it, 0.25 more for pointing it down, 0.25 more for aiming near the dot."""
    measured = _measure(objects["drill"])
    _report(measured)
    return measured["partial_score"]


def demo():
    """Right arm: grasp the handle, lift the drill, point its chuck down over the dot, and hold its aim.

    Uses the fixed starting layout (-E object_jitter=0) and keeps the gripper closed after pickup.
    """
    # Shift the nominal handle centre toward the head before rotating it into world coordinates.
    handle_xy = np.array([0.040, 0.025]) + GRASP_OFFSET_XY
    c, s = np.cos(DRILL.yaw), np.sin(DRILL.yaw)
    x, y = np.asarray(DRILL.xy) + np.array([[c, -s], [s, c]]) @ handle_xy
    grasp = (float(x), float(y), TOP + 0.030)
    raised = (float(x), float(y), TOP + 0.140)
    # Close across the handle's width (model x); at the drill's 90-degree yaw, this is world y.
    yaw = DRILL.yaw - np.pi / 2

    # The drill's chuck points along local -X. This desired drill rotation puts local -X along world -Z.
    drill_start = axis_angle_matrix(np.array([0.0, 0.0, 1.0]), DRILL.yaw)
    drill_down = axis_angle_matrix(np.array([0.0, 1.0, 0.0]), -np.pi / 2)
    # Preserve the drill-to-gripper orientation established by the grasp.
    gripper_down = drill_down @ drill_start.T @ grip_rotation(yaw)
    aim_quat = tuple(float(v) for v in ik.matrix_to_quat_xyzw(gripper_down))

    # Put the chuck 5.5 cm above the dot, accounting for its offset from the handle's grasp point.
    tip_from_handle = _TIP_OFFSET_LOCAL - np.array([*handle_xy, 0.0])
    tip_goal = np.array([*TARGET_XY, TOP + 0.055])
    aim = tuple(float(v) for v in tip_goal - drill_down @ tip_from_handle)
    return {
        "right": [
            Waypoint(3.0, raised, FINGER_OPEN, "approach handle", yaw),
            Waypoint(1.0, raised, FINGER_OPEN, "settle above handle", yaw),
            Waypoint(2.5, grasp, FINGER_OPEN, "descend around handle", yaw),
            Waypoint(2.0, grasp, FINGER_CLOSED, "grasp handle", yaw),
            Waypoint(1.0, grasp, FINGER_CLOSED, "settle grasp", yaw),
            Waypoint(3.0, raised, FINGER_CLOSED, "lift drill", yaw),
            Waypoint(3.0, raised, FINGER_CLOSED, "hold drill", yaw),
            Waypoint(6.0, aim, FINGER_CLOSED, "turn and aim over dot", quat_xyzw=aim_quat),
            Waypoint(3.0, aim, FINGER_CLOSED, "hold on target", quat_xyzw=aim_quat),
        ]
    }
