"""Empty drawer with detailed oak finish and a blue metal handle.

Visual variant of the user-validated empty-drawer prototype. No garment yet.
Uses the same physical dimensions, passive rail, robot plan and scorer.
"""
from pathlib import Path
import importlib.util
import os
import sys

HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location("_drawer_validated_baseline",HERE/"task.py")
base=importlib.util.module_from_spec(spec)
sys.modules[spec.name]=base
spec.loader.exec_module(base)

SCENE=base.SCENE
OBJECTS=base.OBJECTS
SUCCESS_HOLD_S=base.SUCCESS_HOLD_S
SCORE_OVER=base.SCORE_OVER
success=base.success
score=base.score
distance=base.distance
demo=base.demo


def build_scene_objects():
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose
    from yam_arena import scene as scene_module
    assets=HERE/"assets"/"high_fidelity"
    if os.environ.get("DRAWER_VIEW","front") == "front":
        # Debug viewport only: model camera poses and calibration are unchanged.
        scene_module.WINDOW_EYE=(-0.60,0.04,1.95)
        scene_module.WINDOW_LOOKAT=(0.36,0.04,0.82)
    result=[]
    for item in OBJECTS:
        path=assets/("drawer.usda" if item.name=="drawer" else item.name+".usda")
        if not path.is_file():raise FileNotFoundError(path)
        result.append(Object(name=item.name,
            object_type=ObjectType.RIGID if item.name=="drawer" else ObjectType.BASE,
            usd_path=str(path),initial_pose=Pose(position_xyz=item.pos,rotation_xyzw=(0.,0.,0.,1.))))
    return result

# ROBOCURVE_DRAWER_NO_STAND_V1
from yam_arena import station as _drawer_station

if not hasattr(_drawer_station, "_drawer_original_camera_stand"):
    _drawer_station._drawer_original_camera_stand = _drawer_station.camera_stand

def _drawer_camera_stand(top):
    parts = _drawer_station._drawer_original_camera_stand(top)
    if os.environ.get("DRAWER_SHOW_STAND") == "1":
        return parts
    return {name: part for name, part in parts.items()
            if name not in {"stand_foot", "stand_mast", "stand_arm"}}

_drawer_station.camera_stand = _drawer_camera_stand
