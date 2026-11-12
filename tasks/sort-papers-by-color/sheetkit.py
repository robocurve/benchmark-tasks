"""sheetkit: task-file boxes with their own friction, for thin sheets and the smooth surfaces they slide on.

    import pathlib, sys
    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
    from sheetkit import Slick

    SHEET = Slick("sheet_1", pos=(0.24, 0.0, 0.81), size=(0.148, 0.105, 0.0005), mass=0.006, color=(1, 0.8, 0.1))
    OBJECTS = [SHEET, ...]

yam_arena gives every task-file object one rubber-like material (friction 1.2), so a stack of sheets would stick
together harder than a finger pad can drag the top one. A ``Slick`` is an ordinary task-file ``Box`` plus a friction
pair and a PhysX combine mode. It is spawned through yam_arena's own cuboid spawner, so the simulator reports its
state exactly as it does for a plain ``Box``.

Importing this module hooks ``yam_arena.scene.task_object`` without modifying yam_arena. If the hook is not
installed, a ``Slick`` still loads as a plain ``Box`` with yam_arena's default friction.

Pure Python at import time; Isaac Sim is reached only through ``yam_arena.scene``, when the scene is built.
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import sys
from dataclasses import dataclass

from yam_arena.task_file import Box

__all__ = ["Slick", "install"]

COMBINE_MODES = ("average", "min", "multiply", "max")


@dataclass(frozen=True)
class Slick(Box):
    """A ``Box`` with its own friction. With ``combine="max"`` a contact uses the larger of the two bodies'
    coefficients: sheet on sheet gives the sheet's own value, finger pad on sheet gives the pad's."""

    static_friction: float = 0.4
    dynamic_friction: float = 0.3
    combine: str = "max"
    contact_offset: float = 0.003      # m; contacts are generated this far out, which stops thin sheets tunnelling
    linear_damping: float | None = None   # 1/s; air drag on a light sheet
    angular_damping: float | None = None
    roughness: float = 0.6             # look only

    def _check(self) -> None:
        super()._check()
        if self.static_friction < 0 or self.dynamic_friction < 0:
            raise ValueError(f"{self.name}: friction must be >= 0")
        if self.dynamic_friction > self.static_friction:
            raise ValueError(f"{self.name}: dynamic friction must not exceed static friction")
        if self.combine not in COMBINE_MODES:
            raise ValueError(f"{self.name}.combine must be one of {COMBINE_MODES}, got {self.combine!r}")
        if not self.contact_offset > 0:
            raise ValueError(f"{self.name}.contact_offset must be positive")


def _material(sim_utils, spec: Slick):
    try:
        return sim_utils.RigidBodyMaterialCfg(static_friction=spec.static_friction,
                                              dynamic_friction=spec.dynamic_friction,
                                              friction_combine_mode=spec.combine)
    except TypeError:  # a build without the combine-mode field: keep the coefficients
        return sim_utils.RigidBodyMaterialCfg(static_friction=spec.static_friction,
                                              dynamic_friction=spec.dynamic_friction)


def _rigid_props(scene, spec: Slick):
    try:
        return scene.PhysxRigidBodyPropertiesCfg(solver_position_iteration_count=32, max_depenetration_velocity=1.0,
                                                 linear_damping=spec.linear_damping,
                                                 angular_damping=spec.angular_damping)
    except TypeError:
        return scene.PhysxRigidBodyPropertiesCfg()


def _slick_object(scene, spec: Slick):
    """The Arena object for a Slick: yam_arena's ``task_object`` for a Box, with this material and these props."""
    sim_utils = scene.sim_utils
    look = sim_utils.PreviewSurfaceCfg(diffuse_color=scene.linear_rgb(spec.color), roughness=spec.roughness)
    if spec.free:
        body = dict(
            rigid_props=_rigid_props(scene, spec),
            mass_props=sim_utils.MassPropertiesCfg(mass=spec.mass),
            collision_props=scene.PhysxCollisionPropertiesCfg(contact_offset=spec.contact_offset, rest_offset=0.0),
        )
    else:
        body = dict(collision_props=scene.PhysxCollisionPropertiesCfg())
    cfg = sim_utils.CuboidCfg(size=tuple(spec.size), physics_material=_material(sim_utils, spec),
                              visual_material=look, **body)
    return scene.Object(
        name=spec.name,
        object_type=scene.ObjectType.RIGID if spec.free else scene.ObjectType.BASE,
        spawner_cfg=cfg,
        initial_pose=scene.Pose(position_xyz=tuple(spec.pos), rotation_xyzw=scene._yaw_xyzw(spec.yaw)),
    )


def _patch_scene(scene) -> None:
    if getattr(scene, "_sheetkit_patched", False):
        return
    stock = scene.task_object

    def task_object(spec):
        if isinstance(spec, Slick):
            return _slick_object(scene, spec)
        return stock(spec)

    scene.task_object = task_object
    scene._sheetkit_patched = True


class _Finder(importlib.abc.MetaPathFinder):
    """Patches ``yam_arena.scene`` as it is first imported (it can only be imported once Isaac Sim has started)."""

    target = "yam_arena.scene"

    def find_spec(self, fullname, path, target=None):
        if fullname != self.target:
            return None
        sys.meta_path.remove(self)
        try:
            spec = importlib.util.find_spec(fullname)
        finally:
            sys.meta_path.insert(0, self)
        if spec is None or spec.loader is None:
            return None
        loader = spec.loader
        stock_exec = loader.exec_module

        def exec_module(module):
            stock_exec(module)
            _patch_scene(module)

        loader.exec_module = exec_module
        return spec


def install() -> None:
    """Hook yam_arena.scene: now if it is already imported, otherwise when it is. Idempotent."""
    if _Finder.target in sys.modules:
        _patch_scene(sys.modules[_Finder.target])
    if not any(isinstance(f, _Finder) for f in sys.meta_path):
        sys.meta_path.insert(0, _Finder())


install()
