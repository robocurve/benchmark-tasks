"""Task-local embodiment: the yam_arena embodiment with this task's cover built from its parts.

Nothing installed is modified. While the scene is built, ``yam_arena.scene.task_object`` is wrapped so that

- the cover is one rigid body with four convex colliders (plate, handle, two chamfered cleats) instead of
  the single box the task file declares for it;
- the wrench gets the friction of steel on wood and the cover that of painted plywood.

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
COVER_FRICTION = (0.5, 0.45)  # painted plywood on plywood
WRENCH_FRICTION = (0.4, 0.35)  # chrome-plated steel on a wooden subfloor
SOLVER_POSITION_ITERATIONS = 8
DIAGNOSTIC_EVERY = 5  # control steps between measurement records


def _spawn_cover(prim_path, cfg, translation=None, orientation=None, **kwargs):
    """A rigid body whose colliders are the convex hulls of ``cfg.parts``: (name, points) in its frame."""
    import trimesh
    from pxr import UsdPhysics
    from isaaclab.sim import schemas
    from isaaclab.sim.spawners.meshes.meshes import _spawn_mesh_geom_from_mesh
    from isaaclab.sim.utils import bind_visual_material, create_prim, get_current_stage

    def hull_mesh(points):
        mesh = trimesh.convex.convex_hull(np.asarray(points, dtype=float))
        mesh.unmerge_vertices()  # flat shading
        return mesh

    stage = get_current_stage()
    (first_name, first_points), *rest = cfg.parts
    # The first part goes through Isaac Lab's mesh spawner, which also applies the body's rigid, mass,
    # collision and material properties; the others are added as sibling collider meshes of the same body.
    _spawn_mesh_geom_from_mesh(prim_path, cfg, hull_mesh(first_points), translation, orientation, None, stage=stage)
    geometry = f"{prim_path}/geometry"
    for name, points in rest:
        mesh = hull_mesh(points)
        path = f"{geometry}/{name}"
        prim = create_prim(
            path,
            prim_type="Mesh",
            attributes={
                "points": mesh.vertices,
                "faceVertexIndices": mesh.faces.flatten(),
                "faceVertexCounts": np.asarray([3] * len(mesh.faces)),
                "subdivisionScheme": "bilinear",
            },
            stage=stage,
        )
        UsdPhysics.MeshCollisionAPI.Apply(prim).GetApproximationAttr().Set("convexHull")
        schemas.define_collision_properties(path, cfg.collision_props, stage=stage)
        bind_visual_material(path, f"{geometry}/{cfg.visual_material_path}", stage=stage)
    return stage.GetPrimAtPath(prim_path)


def _cover_cfg(spec, parts, friction):
    import isaaclab.sim as sim_utils
    from isaaclab.sim.spawners.meshes.meshes_cfg import MeshCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_physx.sim.schemas import PhysxCollisionPropertiesCfg, PhysxRigidBodyPropertiesCfg

    from yam_arena.station import linear_rgb

    @configclass
    class PartsCfg(MeshCfg):  # any name but Mesh{Sphere,Cuboid}Cfg makes each collider a convex hull
        parts: tuple = ()

    return PartsCfg(
        func=clone(_spawn_cover),
        parts=tuple((n, tuple(tuple(float(v) for v in p) for p in pts)) for n, pts in parts),
        rigid_props=PhysxRigidBodyPropertiesCfg(
            solver_position_iteration_count=SOLVER_POSITION_ITERATIONS, solver_velocity_iteration_count=1
        ),
        mass_props=sim_utils.MassPropertiesCfg(mass=spec.mass),
        collision_props=PhysxCollisionPropertiesCfg(contact_offset=0.005, rest_offset=0.0),
        physics_material=sim_utils.RigidBodyMaterialCfg(static_friction=friction[0], dynamic_friction=friction[1]),
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(spec.color), roughness=0.5),
    )


class HoleCoverEmbodiment(YamArenaEmbodiment):
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

        from geometry import cover_part_points
        from yam_arena import scene

        original = scene.task_object

        def make_object(spec):
            obj = original(spec)
            if spec.name == "cover":
                obj.spawner_cfg = _cover_cfg(spec, cover_part_points(), COVER_FRICTION)
            elif spec.name == "wrench":
                obj.spawner_cfg.physics_material = sim_utils.RigidBodyMaterialCfg(
                    static_friction=WRENCH_FRICTION[0], dynamic_friction=WRENCH_FRICTION[1]
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
                n: {"pos": states[n].pos.round(6).tolist(), "quat_xyzw": states[n].quat_xyzw.round(6).tolist(),
                    "low": states[n].low.round(6).tolist(), "high": states[n].high.round(6).tolist()}
                for n in module.FREE
            }  # fmt: skip
            with open(self.diagnostics_path, "a") as out:
                out.write(json.dumps(record, default=float) + "\n")
        return result
