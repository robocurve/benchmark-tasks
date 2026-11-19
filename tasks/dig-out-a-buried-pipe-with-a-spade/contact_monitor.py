"""Latch physical contacts at every physics substep, independently of actions."""

import bodykit
import digplan as d
import numpy as np
import soilkit

pipe_touched = False
robot_soil_touched = False
peak_pipe_force = 0.0
peak_robot_soil_force = 0.0
peak_any_soil_force = 0.0
_views = None
_base = None
grip_forces = {"left": 0.0, "right": 0.0}


def _author(stage, path, spec):
    from pxr import PhysxSchema

    if spec.name in ("spade", "pipe"):
        PhysxSchema.PhysxContactReportAPI.Apply(
            stage.GetPrimAtPath(path + "/body")
        ).CreateThresholdAttr(0.0)


bodykit.EXTRA_AUTHORS.append(_author)


@bodykit.on_reset
def reset():
    global \
        pipe_touched, \
        robot_soil_touched, \
        peak_pipe_force, \
        peak_robot_soil_force, \
        peak_any_soil_force
    pipe_touched = robot_soil_touched = False
    peak_pipe_force = peak_robot_soil_force = 0.0
    peak_any_soil_force = 0.0


def _read():
    global \
        pipe_touched, \
        robot_soil_touched, \
        peak_pipe_force, \
        peak_robot_soil_force, \
        peak_any_soil_force
    if _views is None:
        return
    pipe_force = float(
        np.linalg.norm(
            _views[0]
            .get_contact_force_matrix(dt=_base.physics_dt)
            .numpy()
            .reshape(-1, 3),
            axis=1,
        ).sum()
    )
    contacts = _views[1].get_contact_force_matrix(dt=_base.physics_dt).numpy()
    per_grain = np.linalg.norm(contacts, axis=-1).sum(axis=-1).reshape(-1)
    # A grain brushing a wrist after being lifted is not bare-hand excavation.
    # Count every robot link's contact with material still inside the fixture,
    # including contact at its rim. Query physics poses at the same substep.
    pos = soilkit._view("soil").get_transforms().numpy()[:, :3]
    radii = soilkit.grains("soil")[1]
    uv = d.to_trench(pos)
    bed = (
        (abs(uv[:, 0]) < d.IN_U / 2 + radii)
        & (abs(uv[:, 1]) < d.IN_V / 2 + radii)
        & (pos[:, 2] < d.TOP + d.WALL_H + radii)
    )
    bed_force = float(per_grain[bed].sum())
    peak_any_soil_force = max(peak_any_soil_force, float(per_grain.sum()))
    pads = _views[2].get_contact_force_matrix(dt=_base.physics_dt).numpy().reshape(4, 3)
    for i, arm in enumerate(("left", "right")):
        grip_forces[arm] = float(np.linalg.norm(pads[i * 2 : i * 2 + 2], axis=1).min())
    peak_pipe_force = max(peak_pipe_force, pipe_force)
    peak_robot_soil_force = max(peak_robot_soil_force, bed_force)
    pipe_touched |= pipe_force > 0.01
    robot_soil_touched |= bed_force > 0.01


@bodykit.on_step
def observe(env):
    global _views, _base
    if _views is None:
        import carb
        from isaaclab_physx.physics import PhysxManager
        from pxr import PhysxSchema
        from yam_arena.robot import asset_link_paths

        _base = env.unwrapped
        root = "/World/envs/env_0"
        links = [
            root + "/" + side + "Arm" + p
            for side in ("Left", "Right")
            for p in asset_link_paths().values()
        ]
        carb.settings.get_settings().set_bool(
            "/physics/disableContactProcessing", False
        )
        for path in links:
            prim = _base.sim.stage.GetPrimAtPath(path)
            if prim.IsValid():
                PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.0)
        sim = PhysxManager.get_physics_sim_view()
        pads = [
            root + "/" + side + "Arm" + asset_link_paths()[pad]
            for side in ("Left", "Right")
            for pad in ("tip_left", "tip_right")
        ]
        _views = [
            sim.create_rigid_contact_view(
                root + "/pipe/body", filter_patterns=links + [root + "/spade/body"]
            ),
            sim.create_rigid_contact_view(root + "/soil/g*", filter_patterns=links),
            sim.create_rigid_contact_view(root + "/spade/body", filter_patterns=pads),
        ]
        original = _base.sim.step

        def step(*args, **kwargs):
            result = original(*args, **kwargs)
            _read()
            return result

        _base.sim.step = step
    _read()
