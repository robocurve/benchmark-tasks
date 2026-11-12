"""sheetkit: thin sheets for yam_arena task files: boxes with their own friction, and pages with marks printed on them.

    import pathlib, sys
    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
    from sheetkit import Page, Slick

    SHELF = Slick("shelf", pos=(0.36, 0.18, 0.82), size=(0.5, 0.2, 0.15), static_friction=0.02, dynamic_friction=0.02)
    PAGE = Page("page_1", pos=(0.2, 0.1, 0.9), size=(0.148, 0.105, 0.0005), mass=0.006,
                marks=((0.0, 0.0, 0.09, 0.013),))          # one stroke: centre x, y and size x, y on the page (m)
    OBJECTS = [SHELF, PAGE]

yam_arena gives every task-file object one rubber-like material (friction 1.2) and one color. A ``Slick`` is an
ordinary task-file ``Box`` plus a friction pair and a PhysX combine mode, spawned through yam_arena's own cuboid
spawner. A ``Page`` is a loose sheet with dark rectangles drawn on its top face (numerals, built from strokes); the
marks are looks only and are not felt.

Importing this module hooks ``yam_arena.scene.task_object`` and ``yam_arena.scene.object_states`` without modifying
yam_arena. A ``Page`` reports its own pose (its centre and orientation) and a bounding box computed from its size.

Pure Python at import time; Isaac Sim is reached only when the scene is built.
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import sys
from dataclasses import dataclass

import numpy as np
from yam_arena.task_file import Box, ObjectState

__all__ = ["Page", "Slick", "install"]

COMBINE_MODES = ("average", "min", "multiply", "max")
MARK_T = 0.0001          # how far a mark stands above the page (m): enough to show, hidden when a page lies on top


@dataclass(frozen=True)
class Slick(Box):
    """A ``Box`` with its own friction. PhysX combines the two bodies' coefficients by ``combine`` (the default,
    "average", is also what bodies without a material use)."""

    static_friction: float = 0.4
    dynamic_friction: float = 0.3
    combine: str = "average"
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


@dataclass(frozen=True)
class Page(Slick):
    """A loose sheet with ``marks`` on its top face: rectangles (centre x, centre y, size x, size y) in the page's own
    frame, in metres, drawn in ``ink``."""

    marks: tuple = ()
    ink: tuple = (0.05, 0.05, 0.06)

    def _check(self) -> None:
        super()._check()
        if self.mass is None:
            raise ValueError(f"{self.name}: a Page must be loose (give it a mass)")
        for m in self.marks:
            if len(m) != 4 or not all(math.isfinite(float(v)) for v in m) or m[2] <= 0 or m[3] <= 0:
                raise ValueError(f"{self.name}: each mark is (centre x, centre y, size x, size y), got {m!r}")
            if abs(m[0]) + m[2] / 2 > self.size[0] / 2 + 1e-9 or abs(m[1]) + m[3] / 2 > self.size[1] / 2 + 1e-9:
                raise ValueError(f"{self.name}: mark {m!r} reaches past the page's edge")


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


def _yaw_xyzw(yaw: float) -> tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


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
        initial_pose=scene.Pose(position_xyz=tuple(spec.pos), rotation_xyzw=_yaw_xyzw(spec.yaw)),
    )


# -- pages --------------------------------------------------------------------------------------------------------

_PAGES: dict[str, Page] = {}
_SPAWNER_CFG = None


def _spawn_page(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    """Author the page under ``prim_path``: one rigid body holding the sheet (which collides) and its marks (which
    are only drawn)."""
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics

    from yam_arena.station import linear_rgb

    spec: Page = _PAGES[cfg.page_name]
    stage = get_current_stage()
    double = UsdGeom.XformOp.PrecisionDouble
    root = UsdGeom.Xform.Define(stage, prim_path)
    root.AddTranslateOp(precision=double).Set(Gf.Vec3d(*(translation or (0.0, 0.0, 0.0))))
    x, y, z, w = orientation if orientation is not None else (0.0, 0.0, 0.0, 1.0)
    root.AddOrientOp(precision=double).Set(Gf.Quatd(w, x, y, z))

    phys = f"{prim_path}/Looks/physics"
    pm = _material(sim_utils, spec)
    pm.func(phys, pm)

    def look(name: str, color, roughness: float) -> str:
        path = f"{prim_path}/Looks/{name}"
        vis = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=roughness)
        vis.func(path, vis)
        return path

    paper, ink = look("paper", spec.color, spec.roughness), look("ink", spec.ink, 0.9)

    body_path = f"{prim_path}/body"
    body = UsdGeom.Xform.Define(stage, body_path)
    body.AddTranslateOp(precision=double).Set(Gf.Vec3d(0.0, 0.0, 0.0))
    body.AddOrientOp(precision=double).Set(Gf.Quatd(1, 0, 0, 0))
    prim = body.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(prim)
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(spec.mass)
    rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
    rb.CreateMaxDepenetrationVelocityAttr(1.0)
    rb.CreateSolverPositionIterationCountAttr(32)
    if spec.linear_damping is not None:
        rb.CreateLinearDampingAttr(spec.linear_damping)
    if spec.angular_damping is not None:
        rb.CreateAngularDampingAttr(spec.angular_damping)

    plate = UsdGeom.Cube.Define(stage, f"{body_path}/plate")
    plate.CreateSizeAttr(1.0)
    plate.AddScaleOp().Set(Gf.Vec3f(*spec.size))
    UsdPhysics.CollisionAPI.Apply(plate.GetPrim())
    cp = PhysxSchema.PhysxCollisionAPI.Apply(plate.GetPrim())
    cp.CreateContactOffsetAttr(spec.contact_offset)
    cp.CreateRestOffsetAttr(0.0)
    bind_physics_material(str(plate.GetPath()), phys, stage=stage)
    bind_visual_material(str(plate.GetPath()), paper, stage=stage)

    top = spec.size[2] / 2
    for i, (cx, cy, wx, wy) in enumerate(spec.marks):
        mark = UsdGeom.Cube.Define(stage, f"{body_path}/mark_{i}")
        mark.CreateSizeAttr(1.0)
        mark.AddTranslateOp().Set(Gf.Vec3d(float(cx), float(cy), top + MARK_T / 2))
        mark.AddScaleOp().Set(Gf.Vec3f(float(wx), float(wy), MARK_T))
        bind_visual_material(str(mark.GetPath()), ink, stage=stage)
    return root.GetPrim()


def _page_object(spec: Page):
    from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    global _SPAWNER_CFG
    if _SPAWNER_CFG is None:
        @configclass
        class PageSpawnerCfg(SpawnerCfg):
            func: object = clone(_spawn_page)
            page_name: str = ""

        _SPAWNER_CFG = PageSpawnerCfg
    return Object(
        name=spec.name,
        object_type=ObjectType.RIGID,
        spawner_cfg=_SPAWNER_CFG(page_name=spec.name),
        initial_pose=Pose(position_xyz=tuple(spec.pos), rotation_xyzw=_yaw_xyzw(spec.yaw)),
    )


def _np(x) -> np.ndarray:
    return getattr(x, "torch", x).detach().cpu().numpy().astype(np.float64)


def _rotation(q) -> np.ndarray:
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def _page_state(environment, spec: Page) -> ObjectState:
    """The page's centre, orientation and velocities as the simulator has them, and its world-aligned bounds."""
    base = environment.unwrapped
    asset = base.scene[spec.name]
    origin = _np(base.scene.env_origins[0])
    pose = _np(asset.data.root_link_pose_w)[0]
    vel = _np(asset.data.root_vel_w)[0] if hasattr(asset.data, "root_vel_w") else np.zeros(6)
    pos, quat = pose[:3] - origin, pose[3:7]
    half = np.array(spec.size) / 2
    corners = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * half
    pts = corners @ _rotation(quat).T + pos
    return ObjectState(pos=pos, quat_xyzw=quat, lin_vel=vel[:3], ang_vel=vel[3:6], low=pts.min(0), high=pts.max(0))


# -- hooking yam_arena --------------------------------------------------------------------------------------------

def _patch_scene(scene) -> None:
    if getattr(scene, "_sheetkit_patched", False):
        return
    stock_object, stock_states = scene.task_object, scene.object_states

    def task_object(spec):
        if isinstance(spec, Page):
            _PAGES[spec.name] = spec
            return _page_object(spec)
        if isinstance(spec, Slick):
            return _slick_object(scene, spec)
        return stock_object(spec)

    def object_states(environment, names):
        plain = [n for n in names if n not in _PAGES]
        out = stock_states(environment, plain) if plain else {}
        for n in names:
            if n in _PAGES:
                out[n] = _page_state(environment, _PAGES[n])
        return {n: out[n] for n in names}

    scene.task_object = task_object
    scene.object_states = object_states
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
