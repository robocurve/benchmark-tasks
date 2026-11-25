"""Record the stock overhead view and a fixed close-up of the two locks."""

from dataclasses import replace
from importlib.abc import MetaPathFinder
from importlib.machinery import PathFinder
import sys

import numpy as np
from inspect_robots.cli import main
from yam_arena import ik, inspect_embodiment, station


def detail_camera(camera):
    position = np.array([.120, -.055, .944])
    target = np.array([.310, .025, .788])
    back = position - target
    back /= np.linalg.norm(back)
    right = np.cross([0., 0., 1.], back)
    right /= np.linalg.norm(right)
    up = np.cross(back, right)
    rotation = ik.matrix_to_quat_xyzw(np.column_stack([right, up, back]))
    lens = replace(camera.lens, fx=1.25 * camera.lens.fx, fy=1.25 * camera.lens.fy)
    return replace(camera, pos=tuple(position), rot_xyzw=tuple(rotation), lens=lens)


def install_detail(module):
    cls = module.BimanualYamEmbodiment
    stock = cls._wrist_cam_cfg

    def camera(self, side, height, width):
        config = stock(self, side, height, width)
        if side == "right":
            detail = detail_camera(station.top_camera())
            config.prim_path = "{ENV_REGEX_NS}/DetailCam"
            config.offset = module._offset(detail)
            config.spawn = module._pinhole(detail, height, width, near=.02)
        return config

    cls._wrist_cam_cfg = camera


class DetailCameraImport(MetaPathFinder):
    def find_spec(self, name, path=None, target=None):
        # This module requires Kit and must load after the stock launcher.
        if name != "yam_arena.embodiment":
            return None
        spec = PathFinder.find_spec(name, path, target)
        if spec is None or spec.loader is None:
            return spec
        load = spec.loader.exec_module

        def configure(module):
            load(module)
            install_detail(module)

        spec.loader.exec_module = configure
        return spec


if __name__ == "__main__":
    sys.meta_path.insert(0, DetailCameraImport())
    inspect_embodiment.RESET_RENDERS = 64
    raise SystemExit(main())
