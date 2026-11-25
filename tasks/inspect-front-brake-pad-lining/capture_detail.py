"""Record the overhead, measuring gap, and inspection card views."""

from dataclasses import replace
from importlib.abc import MetaPathFinder
from importlib.machinery import PathFinder
import sys

import numpy as np
from inspect_robots.cli import main
from yam_arena import ik, inspect_embodiment, station


def detail_camera(camera, *, gap_view=False):
    position = np.array([.225, .204, .993])
    target = np.array([.313, .204, .754])
    if gap_view:
        position = np.array([.43675, -.08713, 1.0025])
        target = np.array([.50126, -.05887, 1.0105])
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
        detail = detail_camera(station.top_camera(), gap_view=side == "left")
        config.prim_path = f"{{ENV_REGEX_NS}}/Inspection{side.title()}Cam"
        config.offset = module._offset(detail)
        config.spawn = module._pinhole(detail, height, width, near=.02)
        return config

    cls._wrist_cam_cfg = camera


def install_fill_light():
    cls = inspect_embodiment.YamArenaEmbodiment
    stock = cls._ensure_sim

    def ensure(self, goal):
        stock(self, goal)
        import isaaclab.sim as sim
        from isaaclab.sim.utils.stage import get_current_stage

        path = "/World/InspectionFill"
        if not get_current_stage().GetPrimAtPath(path).IsValid():
            light = sim.SphereLightCfg(intensity=1.6e4, radius=.04)
            light.func(path, light, translation=(.259, .150, 1.045))
            inspection = sim.SphereLightCfg(intensity=2600., radius=.012)
            inspection.func("/World/ProbeLight", inspection, translation=(.433, -.091, 1.027))

    cls._ensure_sim = ensure


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
    install_fill_light()
    inspect_embodiment.RESET_RENDERS = 64
    raise SystemExit(main())
