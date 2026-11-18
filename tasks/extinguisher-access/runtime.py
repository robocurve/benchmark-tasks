"""Task-local embodiment: the yam_arena embodiment with this task's toolbox and extinguisher built from parts.

Nothing installed is modified. While the scene is built, ``yam_arena.scene.task_object`` is wrapped so that

- the toolbox is one rigid body with a convex collider per part (the box and lid, the handle's two posts and
  its bar, with room under the bar for the fingers) instead of the single box the task file declares;
- the extinguisher is one rigid body with a convex collider per part (round body, domed shoulder, neck, valve,
  carry handle, squeeze lever and nozzle) instead of a box.

The handle, valve and levers are drawn black. After every control step the grippers' TCPs are handed to the
task file's scorer, which uses them only for the release check, and the scorer's measurements are appended to
``measurements.jsonl``. No object pose is ever written by this code after the reset.
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
TOOLBOX_FRICTION = (0.45, 0.40)  # moulded polypropylene on painted plywood
EXTINGUISHER_FRICTION = (0.40, 0.35)  # painted steel on powder-coated steel
ACCENT_COLOR = (0.08, 0.08, 0.09)  # the toolbox's handle, the extinguisher's valve, handle, lever and nozzle
SOLVER_POSITION_ITERATIONS = 16
CONTACT_OFFSET = 0.002
DIAGNOSTIC_EVERY = 5  # control steps between measurement records


def _spawn_parts(prim_path, cfg, translation=None, orientation=None, **kwargs):
    """A body whose colliders are the convex hulls of ``cfg.parts``: (name, points) in its frame. Parts named
    in ``cfg.accent_parts`` get the accent look; the rest the body's own."""
    import isaaclab.sim as sim_utils
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
    accent = sim_utils.PreviewSurfaceCfg(diffuse_color=cfg.accent_color, metallic=0.3, roughness=0.45)
    accent.func(f"{geometry}/accent_material", accent)
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
        material = "accent_material" if name in cfg.accent_parts else cfg.visual_material_path
        bind_visual_material(path, f"{geometry}/{material}", stage=stage)
    return stage.GetPrimAtPath(prim_path)


def _parts_cfg(spec, parts, accent_parts, friction):
    import isaaclab.sim as sim_utils
    from isaaclab.sim.spawners.meshes.meshes_cfg import MeshCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_physx.sim.schemas import PhysxCollisionPropertiesCfg, PhysxRigidBodyPropertiesCfg

    from yam_arena.station import linear_rgb

    @configclass
    class PartsCfg(MeshCfg):  # any name but Mesh{Sphere,Cuboid}Cfg makes each collider a convex hull
        parts: tuple = ()
        accent_parts: tuple = ()
        accent_color: tuple = (0.0, 0.0, 0.0)

    if parts[0][0] in accent_parts:
        raise ValueError("the first part takes the body's own look")
    return PartsCfg(
        func=clone(_spawn_parts),
        parts=tuple((n, tuple(tuple(float(v) for v in p) for p in pts)) for n, pts in parts),
        accent_parts=tuple(accent_parts),
        accent_color=linear_rgb(ACCENT_COLOR),
        rigid_props=PhysxRigidBodyPropertiesCfg(
            solver_position_iteration_count=SOLVER_POSITION_ITERATIONS, solver_velocity_iteration_count=1
        ),
        mass_props=sim_utils.MassPropertiesCfg(mass=spec.mass),
        collision_props=PhysxCollisionPropertiesCfg(contact_offset=CONTACT_OFFSET, rest_offset=0.0),
        physics_material=sim_utils.RigidBodyMaterialCfg(static_friction=friction[0], dynamic_friction=friction[1]),
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(spec.color), roughness=0.5),
    )


class ExtinguisherAccessEmbodiment(YamArenaEmbodiment):
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

        import geometry as G
        from yam_arena import scene

        original = scene.task_object

        def make_object(spec):
            obj = original(spec)
            if spec.name == "toolbox":
                parts = G.toolbox_part_points()
                accent = [n for n, _ in parts if n != "body"]
                obj.spawner_cfg = _parts_cfg(spec, parts, accent, TOOLBOX_FRICTION)
            elif spec.name == "extinguisher":
                parts = G.extinguisher_part_points()
                accent = [n for n, _ in parts if G.EXT_PART_COLORS[n] == "black"]
                obj.spawner_cfg = _parts_cfg(spec, parts, accent, EXTINGUISHER_FRICTION)
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
