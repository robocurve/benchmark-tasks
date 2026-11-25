"""Task-local embodiment: the yam_arena embodiment with this task's nailed board and wood materials.

Nothing installed is modified. While the scene is built, ``yam_arena.scene.task_object`` is wrapped so that

- the board named in ``task.py`` as the nailed one is spawned as one rigid body with two colliders: the board
  itself and the nail standing out of its top face (a cylinder with a head), so the nail is a solid part of
  the board for the physics and the cameras alike;
- every board and the pallet get sawn-softwood friction.

After every control step the grippers' TCPs are handed to the task file's scorer, which uses them only for
the final hands-clear condition, and the scorer's measurements are appended to ``measurements.jsonl``.
No object pose is ever written by this code after the reset.
"""

from __future__ import annotations

import atexit
import json
import os

from yam_arena import inspect_embodiment as adapter
from yam_arena.inspect_embodiment import YamArenaEmbodiment

# Friction (static, dynamic), estimated; PhysX averages the two bodies' values at a contact. The YAM's finger
# pads keep PhysX's default 0.5 / 0.5.
WOOD_FRICTION = (0.5, 0.4)  # rough-sawn softwood on softwood
DIAGNOSTIC_EVERY = 5  # control steps between measurement records


def _spawn_nailed_board(prim_path, cfg, translation=None, orientation=None, **kwargs):
    """The board as a cuboid rigid body, plus the nail as a second collider under the same body."""
    import isaaclab.sim as sim_utils
    from isaaclab.sim import schemas
    from isaaclab.sim.utils import bind_visual_material, create_prim, get_current_stage
    from isaaclab_physx.sim.schemas import PhysxCollisionPropertiesCfg

    stage = get_current_stage()
    board = sim_utils.spawn_cuboid(prim_path, cfg, translation, orientation)
    geometry = f"{prim_path}/geometry"
    # Steel look for the nail; the shank and head are plain cylinders along the board's local z.
    look = sim_utils.PreviewSurfaceCfg(diffuse_color=cfg.nail_color, metallic=0.8, roughness=0.3)
    look.func(f"{geometry}/nail_material", look)
    for name, (radius, height, centre) in cfg.nail_parts.items():
        path = f"{geometry}/{name}"
        create_prim(path, "Cylinder", translation=centre, attributes={"radius": radius, "height": height, "axis": "Z"},
                    stage=stage)  # fmt: skip
        schemas.define_collision_properties(path, PhysxCollisionPropertiesCfg(contact_offset=0.002, rest_offset=0.0),
                                            stage=stage)  # fmt: skip
        bind_visual_material(path, f"{geometry}/nail_material", stage=stage)
    return board


def _nailed_cfg(spec, module):
    """The spawner config of the nailed board: the task file's box with the nail parts attached."""
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_physx.sim.schemas import PhysxCollisionPropertiesCfg, PhysxRigidBodyPropertiesCfg

    from yam_arena.station import linear_rgb

    @configclass
    class NailedCuboidCfg(sim_utils.CuboidCfg):
        nail_parts: dict = {}
        nail_color: tuple = (0.8, 0.8, 0.8)

    half = spec.size[2] / 2
    shank_h = module.NAIL_PROUD + module.NAIL_DRIVEN
    parts = {
        "nail_shank": (module.NAIL_DIAMETER / 2, shank_h, (module.NAIL_X, module.NAIL_Y, half - module.NAIL_DRIVEN + shank_h / 2)),
        "nail_head": (module.NAIL_HEAD_DIAMETER / 2, module.NAIL_HEAD_THICKNESS,
                      (module.NAIL_X, module.NAIL_Y, half + module.NAIL_PROUD - module.NAIL_HEAD_THICKNESS / 2)),
    }  # fmt: skip
    return NailedCuboidCfg(
        func=clone(_spawn_nailed_board),
        size=tuple(float(s) for s in spec.size),
        nail_parts={k: (float(r), float(h), tuple(float(v) for v in c)) for k, (r, h, c) in parts.items()},
        nail_color=linear_rgb(module.NAIL_COLOR),
        rigid_props=PhysxRigidBodyPropertiesCfg(),
        mass_props=sim_utils.MassPropertiesCfg(mass=spec.mass),
        collision_props=PhysxCollisionPropertiesCfg(contact_offset=0.005, rest_offset=0.0),
        physics_material=sim_utils.RigidBodyMaterialCfg(static_friction=WOOD_FRICTION[0], dynamic_friction=WOOD_FRICTION[1]),
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(spec.color), roughness=0.6),
    )


class NailBoardEmbodiment(YamArenaEmbodiment):
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

        from yam_arena import scene

        module = self.task.module
        original = scene.task_object

        def make_object(spec):
            obj = original(spec)
            if spec.name == module.NAILED:
                obj.spawner_cfg = _nailed_cfg(spec, module)
            elif spec.name in module.BOARDS or spec.name.startswith("pallet_"):
                obj.spawner_cfg.physics_material = sim_utils.RigidBodyMaterialCfg(
                    static_friction=WOOD_FRICTION[0], dynamic_friction=WOOD_FRICTION[1]
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
                for n in module.BOARDS
            }  # fmt: skip
            with open(self.diagnostics_path, "a") as out:
                out.write(json.dumps(record, default=float) + "\n")
        return result
