"""Task-local embodiment: the yam_arena embodiment with this task's object shapes and materials.

Nothing installed is modified. While the scene is built, ``yam_arena.scene.task_object`` is wrapped so that

- the tiles and spacers collide as the convex hulls in ``geometry.py`` (eased tile edges, chamfered spacer
  edges) instead of sharp boxes, with their own friction;
- the backer board gets the friction of fibre-cement under a tile.

After every control step the grippers' TCPs are handed to the task file's scorer, which uses them only for
the final hands-clear condition, and the scorer's measurements are appended to ``measurements.jsonl``.
No object pose is ever written by this code after the reset.
"""

from __future__ import annotations

import atexit
import json
import os

import numpy as np

from yam_arena import inspect_embodiment as adapter
from yam_arena.inspect_embodiment import YamArenaEmbodiment

# Friction (static, dynamic), estimated; PhysX averages the two bodies' values at a contact. The YAM's finger
# pads keep PhysX's default 0.5 / 0.5.
TILE_FRICTION = (0.7, 0.6)  # fired ceramic back on fibre-cement board
BOARD_FRICTION = (0.7, 0.6)
SPACER_FRICTION = (0.5, 0.4)  # moulded polypropylene
SOLVER_POSITION_ITERATIONS = 16  # thin spacer squeezed between two tiles: more iterations than the default 4
DIAGNOSTIC_EVERY = 5  # control steps between measurement records


def _kind(name: str) -> str | None:
    if name.startswith("tile_"):
        return "tile"
    if name.startswith("spacer_"):
        return "spacer"
    if name == "board":
        return "board"
    return None


def _spawn_hull(prim_path, cfg, translation=None, orientation=None, **kwargs):
    """A rigid body whose one collider and visual mesh are the convex hull of ``cfg.points``."""
    import trimesh
    from isaaclab.sim.spawners.meshes.meshes import _spawn_mesh_geom_from_mesh
    from isaaclab.sim.utils import get_current_stage

    mesh = trimesh.convex.convex_hull(np.asarray(cfg.points, dtype=float))
    mesh.unmerge_vertices()  # flat shading: no normals smoothed across the chamfers
    stage = get_current_stage()
    _spawn_mesh_geom_from_mesh(prim_path, cfg, mesh, translation, orientation, None, stage=stage)
    return stage.GetPrimAtPath(prim_path)


def _hull_cfg(spec, points, friction):
    """The spawner config of a free object collided as the convex hull of ``points``."""
    import isaaclab.sim as sim_utils
    from isaaclab.sim.spawners.meshes.meshes_cfg import MeshCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_physx.sim.schemas import PhysxCollisionPropertiesCfg, PhysxRigidBodyPropertiesCfg

    from yam_arena.station import linear_rgb

    @configclass
    class ConvexHullCfg(MeshCfg):  # any name but Mesh{Sphere,Cuboid}Cfg makes the collider a convex hull
        points: tuple = ()

    return ConvexHullCfg(
        func=clone(_spawn_hull),
        points=tuple(tuple(float(v) for v in p) for p in points),
        rigid_props=PhysxRigidBodyPropertiesCfg(
            solver_position_iteration_count=SOLVER_POSITION_ITERATIONS, solver_velocity_iteration_count=1
        ),
        mass_props=sim_utils.MassPropertiesCfg(mass=spec.mass),
        collision_props=PhysxCollisionPropertiesCfg(contact_offset=0.005, rest_offset=0.0),
        physics_material=sim_utils.RigidBodyMaterialCfg(static_friction=friction[0], dynamic_friction=friction[1]),
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(spec.color), roughness=0.35),
    )


class TileSpacerEmbodiment(YamArenaEmbodiment):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.diagnostics_path = None
        self._steps = 0

    def _ensure_sim(self, goal):
        if self._env is not None:
            return
        if adapter._APP is None:  # start Kit first: the scene modules below import Isaac Sim
            from isaaclab.app import AppLauncher

            os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")
            options = {"headless": self.headless, "device": self.device, "enable_cameras": True}
            if not self.headless:
                options["visualizer"] = ["kit"]
                os.environ["DISPLAY"] = adapter.window_display(os.environ)
            adapter._APP = AppLauncher(**options).app
            atexit.register(adapter._close_app_at_exit)

        import isaaclab.sim as sim_utils

        from geometry import spacer_points, tile_points
        from yam_arena import scene

        module = self.task.module
        original = scene.task_object

        def make_object(spec):
            obj = original(spec)
            kind = _kind(spec.name)
            if kind == "tile":
                obj.spawner_cfg = _hull_cfg(spec, tile_points(spec.size, module.TILE_CHAMFER), TILE_FRICTION)
            elif kind == "spacer":
                obj.spawner_cfg = _hull_cfg(spec, spacer_points(spec.size, module.SPACER_CHAMFER), SPACER_FRICTION)
            elif kind == "board":
                obj.spawner_cfg.physics_material = sim_utils.RigidBodyMaterialCfg(
                    static_friction=BOARD_FRICTION[0], dynamic_friction=BOARD_FRICTION[1]
                )
            else:
                return obj
            obj.object_cfg = obj._init_object_cfg()
            return obj

        scene.task_object = make_object
        try:
            super()._ensure_sim(goal)
        finally:
            scene.task_object = original

    def reset(self, scene, *, seed=None):
        self.task.module._robot.clear()
        self._steps = 0
        return super().reset(scene, seed=seed)

    def _task_step(self):
        from yam_arena.runtime import tcp_world
        from yam_arena.scene import object_states

        module = self.task.module
        robot = {"tcp": {arm: tcp_world(self._base.scene[f"{arm}_robot"]).tolist() for arm in ("left", "right")}}
        module._robot.clear()
        module._robot.update(robot)
        result = super()._task_step()
        self._steps += 1
        if self.diagnostics_path is not None and (self._steps % DIAGNOSTIC_EVERY == 0 or result.terminated):
            states = object_states(self._env, self.task.names)
            record = {"step": self._steps, "time_s": round(self._t, 4), **robot, **module.measure(states, robot)}
            record["objects"] = {
                n: {"pos": states[n].pos.round(6).tolist(), "quat_xyzw": states[n].quat_xyzw.round(6).tolist()}
                for n in (*module.TILES, *module.SPACERS)
            }
            with open(self.diagnostics_path, "a") as out:
                out.write(json.dumps(record, default=float) + "\n")
        return result
