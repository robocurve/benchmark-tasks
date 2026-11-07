"""The camera that takes the photo on the card (studio mode only). Pure Python: yam_arena's rig types, no Isaac Sim.

The photo is taken from in front of the model, between the arms, 25 degrees down, so it shows how tall the
cylinders are as well as each part's shape and colour. In studio mode (CONVEYOR_STUDIO=1) the station's overhead
camera is moved here for the one rendered step; trials never use this camera.
"""
from __future__ import annotations

from dataclasses import replace

import numpy as np

from layout import GATE_X, GATE_Y, POST_TOP, TOP

EYE = (GATE_X - 0.36, GATE_Y, TOP + 0.215)  # 0.36 m in front of the model, 0.215 m above the table
TARGET = (GATE_X, GATE_Y, (TOP + POST_TOP) / 2 + 0.03)  # the middle of the model


def camera(lens=None):
    """A Camera (OpenGL convention: looking along -z, y up) at EYE looking at TARGET, with the overhead lens."""
    from yam_arena import ik, station
    from yam_arena.rig import Camera

    if lens is None:
        lens = getattr(station.rig, 'original', station.rig)().top_cam.lens
    forward = np.asarray(TARGET) - np.asarray(EYE)
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, (0.0, 0.0, 1.0))
    right /= np.linalg.norm(right)
    up = np.cross(right, forward)
    rot = np.column_stack([right, up, -forward])
    quat = tuple(float(v) for v in ik.matrix_to_quat_xyzw(rot))
    return Camera(tuple(float(v) for v in EYE), quat, 'opengl', lens)


def use_studio_camera() -> None:
    """Make yam_arena's station rig put its overhead camera at the studio camera (and its mock stand behind it)."""
    from yam_arena import station

    if hasattr(station.rig, 'original'):
        return
    original = station.rig

    def rig(top_cam: str = 'calibrated'):
        base = original(top_cam)
        return replace(base, top_cam=camera(base.top_cam.lens))

    rig.original = original
    station.rig = rig
