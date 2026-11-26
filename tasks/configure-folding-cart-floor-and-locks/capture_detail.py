"""Record both cart sides and retain the stock overhead stream."""

from dataclasses import replace
from importlib.abc import MetaPathFinder
from importlib.machinery import PathFinder
import sys

import numpy as np
from inspect_robots.cli import main
from yam_arena import ik, inspect_embodiment, station


def detail_camera(camera, side):
    sign = 1 if side == "left" else -1
    position = np.array([.50, sign*.65, 1.19])
    target = np.array([.238, sign*.138, .805])
    back = position - target
    back /= np.linalg.norm(back)
    right = np.cross([0., 0., 1.], back)
    right /= np.linalg.norm(right)
    up = np.cross(back, right)
    rotation = ik.matrix_to_quat_xyzw(np.column_stack([right, up, back]))
    return replace(camera, pos=tuple(position), rot_xyzw=tuple(rotation))


def install_detail(module):
    cls = module.BimanualYamEmbodiment
    stock = cls._wrist_cam_cfg

    def camera(self, side, height, width):
        config = stock(self, side, height, width)
        detail = detail_camera(station.top_camera(),side)
        config.prim_path = f"{{ENV_REGEX_NS}}/{side.title()}DetailCam"
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
    inspect_embodiment.RESET_RENDERS = 80
    raise SystemExit(main())
