"""Fixed station and gripper-relative optical mounts for the lathe fixture."""
from dataclasses import replace

import numpy as np
from yam_arena.ik import matrix_to_quat_xyzw
from yam_arena.urdf import quat_xyzw_to_matrix

from fixture import WORLD


def optical_axes(forward, up):
    back = -np.asarray(forward, dtype=float)
    back /= np.linalg.norm(back)
    right = np.cross(up, back)
    right /= np.linalg.norm(right)
    return np.column_stack((right, np.cross(back, right), back))


def camera_rig(base):
    if base.name.endswith("-lathe"):
        return base
    position = np.array((-.37, -.38, .68))
    target = np.array((-.064, .025, .090))
    top = replace(base.top_cam, pos=tuple(position + WORLD),
                  rot_xyzw=tuple(matrix_to_quat_xyzw(optical_axes(target-position, (0, 0, 1)))))
    directions = {"left": (.38, .02, -.93), "right": (-.67, .21, -.71)}
    wrist = {}
    for side, camera in base.wrist_cams.items():
        mount = quat_xyzw_to_matrix(np.array(camera.rot_xyzw))
        axes = mount @ optical_axes(directions[side], (0, 1, 0))
        lens = replace(camera.lens, fx=camera.lens.fx*.90, fy=camera.lens.fy*.90)
        wrist[side] = replace(camera, rot_xyzw=tuple(matrix_to_quat_xyzw(axes)), lens=lens)
    return replace(base, name=base.name+"-lathe", top_cam=top, wrist_cams=wrist)
