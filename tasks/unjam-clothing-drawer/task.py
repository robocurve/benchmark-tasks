"""DEVELOPMENT PROTOTYPE: open an EMPTY tabletop drawer.

The drawer slides towards the arms (negative world x). Its blue rectangular
pull knob can be grasped from above. The cabinet is fixed to the table.
There is no clothing or jam yet. A score of 1 refers ONLY to empty-drawer opening.
"""
from __future__ import annotations

import os
from pathlib import Path

import numpy as np
import yam_arena
from yam_arena.task_file import Box, TABLE_TOP

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
TRAVEL = 0.16
FULL_OPEN = 0.15
DRAWER_ORIGIN = (0.46, 0.04, TOP + 0.065)
HANDLE = (0.28, 0.04, TOP + 0.069)
DRAWER_CLOSED_BACK = 0.58
DRAWER_BOTTOM = TOP + 0.018
DRAWER_WIDTH = 0.24
DRAWER_BOUND_DEPTH = 0.313
HERE = Path(__file__).resolve().parent
ASSET = HERE / "assets" / "empty_drawer.usda"

_scene_source = Path(yam_arena.__file__).resolve().parent / "scene.py"
if "# ROBOCURVE_CUSTOM_OBJECTS_V1" not in _scene_source.read_text():
    raise RuntimeError("Install the optional drawer scene hook first: python tasks/unjam-clothing-drawer/install_scene_hook.py")

# The drawer Box is a declaration for the existing loader/state reader ONLY.
# build_scene_objects() below substitutes the hollow, physically constrained USD.
DRAWER = Box("drawer", pos=DRAWER_ORIGIN, size=(0.24, 0.24, 0.082), mass=0.35)
WOOD = (0.63, 0.43, 0.25)
OBJECTS = [
    Box("cabinet_floor", pos=(0.474, 0.04, TOP + 0.006), size=(0.268, 0.286, 0.012), color=WOOD),
    Box("cabinet_left", pos=(0.474, 0.177, TOP + 0.071), size=(0.268, 0.012, 0.118), color=WOOD),
    Box("cabinet_right", pos=(0.474, -0.097, TOP + 0.071), size=(0.268, 0.012, 0.118), color=WOOD),
    Box("cabinet_back", pos=(0.602, 0.04, TOP + 0.071), size=(0.012, 0.262, 0.118), color=WOOD),
    Box("cabinet_top", pos=(0.474, 0.04, TOP + 0.124), size=(0.268, 0.286, 0.012), color=WOOD),
    DRAWER,
]


def build_scene_objects():
    """Called by our optional scene hook AFTER Isaac Sim has started."""
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose
    from yam_arena.scene import task_object
    if not ASSET.is_file():
        raise FileNotFoundError(f"Missing bundled asset: {ASSET}")
    objects = [task_object(spec) for spec in OBJECTS if spec.name != "drawer"]
    objects.append(Object(
        name="drawer",
        object_type=ObjectType.RIGID,
        usd_path=str(ASSET),
        initial_pose=Pose(position_xyz=DRAWER_ORIGIN, rotation_xyzw=(0.0, 0.0, 0.0, 1.0)),
    ))
    return objects


def measurement(objects):
    state = objects["drawer"]
    values = np.r_[state.low, state.high, state.lin_vel, state.ang_vel]
    if not np.isfinite(values).all():
        return 0.0, False, False
    opening = DRAWER_CLOSED_BACK - float(state.high[0])
    valid = bool(
        -0.002 <= opening <= TRAVEL + 0.002
        and abs((state.high[1] + state.low[1]) / 2 - DRAWER_ORIGIN[1]) <= 0.004
        and abs(state.low[2] - DRAWER_BOTTOM) <= 0.004
        and abs(state.high[1] - state.low[1] - DRAWER_WIDTH) <= 0.006
        and abs(state.high[0] - state.low[0] - DRAWER_BOUND_DEPTH) <= 0.006
    )
    still = bool(np.linalg.norm(state.lin_vel) < 0.01 and np.linalg.norm(state.ang_vel) < 0.1)
    return opening, valid, still


def success(objects):
    opening, valid, still = measurement(objects)
    return bool(valid and still and opening >= FULL_OPEN)


def distance(objects):
    opening, valid, _ = measurement(objects)
    return max(0.0, FULL_OPEN - opening) if valid else FULL_OPEN


def score(objects):
    opening, valid, _ = measurement(objects)
    return 0.5 if valid and opening >= 0.08 else 0.0


def demo():
    """Robot actions only: grasp the knob and pull. The joint is NEVER driven."""
    from yam_arena.constants import FINGER_OPEN, FINGER_CLOSED
    from yam_arena.demo import Waypoint
    if os.environ.get("DRAWER_PREVIEW") == "1":
        return {}  # yam_scripted holds both arms in their ready pose
    x, y, z = HANDLE
    end = (x - TRAVEL, y, z)
    hover = (x, y, TOP + 0.16)
    return {"left": [
        Waypoint(2.0, hover, FINGER_OPEN, "approach handle"),
        Waypoint(2.0, HANDLE, FINGER_OPEN, "lower onto handle"),
        Waypoint(1.0, HANDLE, FINGER_CLOSED, "grasp handle"),
        Waypoint(5.0, end, FINGER_CLOSED, "pull drawer"),
        Waypoint(4.0, end, FINGER_CLOSED, "hold open"),
        Waypoint(1.0, end, FINGER_OPEN, "release handle"),
        Waypoint(2.0, (end[0], y, TOP + 0.16), FINGER_OPEN, "withdraw"),
    ]}
