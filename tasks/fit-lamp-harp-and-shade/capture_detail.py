"""Record the same stock YAM run from a side view of the saddle and shade post."""

from dataclasses import replace
import os

import numpy as np
from inspect_robots.cli import main
from yam_arena import ik, inspect_embodiment, station


def review_camera(camera):
    view = os.environ.get("HARP_DETAIL","saddle")
    if view == "thread":
        position, target = [.512,-.190,1.198], [.303,0.,1.056]
    elif view == "overview":
        position, target = [.802,-.449,1.264], [.280,0.,.920]
    else:
        position, target = [.563,-.182,1.008], [.290,0.,.875]
    back = np.subtract(position,target)
    back /= np.linalg.norm(back)
    right = np.cross([0.,0.,1.],back)
    right /= np.linalg.norm(right)
    up = np.cross(back,right)
    rotation = ik.matrix_to_quat_xyzw(np.column_stack([right,up,back]))
    return replace(camera,pos=tuple(position),rot_xyzw=tuple(rotation))


def use_review_camera():
    stock_rig = station.rig
    stock_stand = station.camera_stand

    def rig(variant="calibrated"):
        calibrated = stock_rig(variant)
        return replace(calibrated,top_cam=review_camera(calibrated.top_cam))

    station.rig = rig
    station.camera_stand = lambda _camera: stock_stand(station.top_camera())


if __name__ == "__main__":
    use_review_camera()
    inspect_embodiment.RESET_RENDERS = 96
    raise SystemExit(main())
