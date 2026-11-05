"""Run the compound depth-stop fixture on the Robocurve Isaac Lab-Arena station."""

import argparse
import atexit
import json
import os
from pathlib import Path
import sys

import scoring
import task as fixture


def install_scene():
    from isaaclab.sim.utils import clone
    from isaaclab_arena.assets.object import Object
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics, UsdShade
    from yam_arena import scene

    original = scene.task_object

    def task_object(spec):
        obj = original(spec)
        if spec.name not in fixture.PARTS:
            return obj

        @clone
        def spawn(path, cfg, translation=None, orientation=None, **kwargs):
            import omni.usd

            stage = omni.usd.get_context().get_stage()
            root = UsdGeom.Xform.Define(stage, path)
            origin = translation if translation is not None else spec.pos
            root.AddTranslateOp().Set(Gf.Vec3d(*origin))
            prim = root.GetPrim()
            if spec.free:
                UsdPhysics.RigidBodyAPI.Apply(prim)
                UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(spec.mass)
                physics = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
                physics.CreateDisableGravityAttr(True)
                physics.CreateSolverPositionIterationCountAttr(32)
                physics.CreateSolverVelocityIterationCountAttr(8)
                physics.CreateSleepThresholdAttr(0.0)

            mat = UsdShade.Material.Define(stage, f"{path}/contact_material")
            physics = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
            physics.CreateStaticFrictionAttr(0.4)
            physics.CreateDynamicFrictionAttr(0.25)
            for part in fixture.PARTS[spec.name]:
                geom = UsdGeom.Cube.Define(stage, f"{path}/{part.name}")
                geom.CreateSizeAttr(1.0)
                geom.AddTranslateOp().Set(Gf.Vec3d(*part.center))
                geom.AddScaleOp().Set(Gf.Vec3f(*part.size))
                geom.CreateDisplayColorPrimvar().Set([Gf.Vec3f(*scene.linear_rgb(part.color))])
                if part.collides:
                    shape = geom.GetPrim()
                    UsdPhysics.CollisionAPI.Apply(shape)
                    collision = PhysxSchema.PhysxCollisionAPI.Apply(shape)
                    collision.CreateContactOffsetAttr(0.0003)
                    collision.CreateRestOffsetAttr(0.0)
                    UsdShade.MaterialBindingAPI.Apply(shape).Bind(mat, materialPurpose="physics")

            guide = fixture.GUIDES.get(spec.name)
            if guide is not None:
                joint = UsdPhysics.PrismaticJoint.Define(stage, f"{path}/guide")
                joint.CreateBody1Rel().SetTargets([root.GetPath()])
                joint.CreateLocalPos0Attr(Gf.Vec3f(*origin))
                joint.CreateAxisAttr(guide.axis)
                joint.CreateLowerLimitAttr(guide.limits[0])
                joint.CreateUpperLimitAttr(guide.limits[1])
                stiffness, damping, max_force = guide.spring
                drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "linear")
                drive.CreateTypeAttr("force")
                drive.CreateStiffnessAttr(stiffness)
                drive.CreateDampingAttr(damping)
                drive.CreateMaxForceAttr(max_force)
                drive.CreateTargetPositionAttr(0.0)
            return prim

        obj.spawner_cfg.func = spawn
        return Object(
            name=spec.name,
            object_type=obj.object_type,
            spawner_cfg=obj.spawner_cfg,
            initial_pose=obj.initial_pose,
        )

    scene.task_object = task_object


def install(trace_dir):
    from inspect_robots.types import StepResult
    from yam_arena import inspect_embodiment as arena
    from yam_arena.runtime import tcp_world

    cls = arena.YamArenaEmbodiment
    original_ensure = cls._ensure_sim
    original_reset = cls.reset
    scene_installed = False

    def ensure(self, goal):
        nonlocal scene_installed
        if arena._APP is None:
            from isaaclab.app import AppLauncher

            os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")
            opts = dict(headless=self.headless, device=self.device, enable_cameras=True)
            if not self.headless:
                os.environ["DISPLAY"] = arena.window_display(os.environ)
                opts["visualizer"] = ["kit"]
            arena._APP = AppLauncher(**opts).app
            atexit.register(arena._close_app_at_exit)
        if not scene_installed:
            install_scene()
            scene_installed = True
        return original_ensure(self, goal)

    def reset(self, scene, *, seed=None):
        self.proof = scoring.Proof()
        self.trace_index = getattr(self, "trace_index", 0) + 1
        self._ensure_sim(None)
        base = self._base
        if not getattr(base.scene, "_depth_stop_drive", False):
            import warp as wp

            spindle = base.scene["spindle"]
            original_write = base.scene.write_data_to_sim
            force = self._torch.zeros((1, 3), dtype=self._torch.float32, device=base.device)
            torque = self._torch.zeros_like(force)
            force_wp, torque_wp = wp.from_torch(force), wp.from_torch(torque)

            def write():
                original_write()
                # This is a remote spring-load fixture with a bounded downfeed load.
                # The stop and pawl are native contact bodies, and only they set depth.
                force[0, 2] = -self.proof.force
                spindle.root_view.apply_forces_and_torques_at_position(
                    force_data=force_wp,
                    torque_data=torque_wp,
                    position_data=None,
                    indices=spindle._ALL_INDICES,
                    is_global=True,
                )

            base.scene.write_data_to_sim = write
            base.scene._depth_stop_drive = True
        return original_reset(self, scene, seed=seed)

    def step(self):
        base = self._base
        positions = {}
        for name in fixture.GUIDES:
            positions[name] = (
                base.scene[name].data.root_link_pose_w.torch[0, :3] - base.scene.env_origins[0]
            ).tolist()
        tcp = {arm: list(tcp_world(base.scene[arm])) for arm in arena.ARMS}
        clear = not any(
            0.18 <= x <= 0.47 and -0.193 <= y <= 0.231 and z <= fixture.TOP + 0.210
            for x, y, z in tcp.values()
        )
        travel = fixture.QUILL[2] - positions["spindle"][2]
        stop_travel = fixture.STOP[2] - positions["depth_stop"][2]
        catch_travel = positions["catch"][0] - fixture.CATCH[0]
        speed = base.scene["spindle"].data.root_link_vel_w.torch[0, 2].item()
        sample = scoring.Sample(
            travel=travel,
            depth=travel - fixture.DATUM_GAP,
            stop_gap=fixture.STOP_GAP + stop_travel - travel,
            speed=speed,
            latch=abs(catch_travel) <= 0.002,
            clear=clear,
            pressed=fixture.BUTTON[2] - positions["test_button"][2] >= 0.005,
        )
        value = self.proof.step(self._t, sample)
        success = value == 1.0
        info = {
            "score": value,
            "success": success,
            "depth_m": sample.depth,
            "stop_gap_m": sample.stop_gap,
            "hands_clear": sample.clear,
            "latch_engaged": sample.latch,
            "test_button_pressed": sample.pressed,
            "proof_force_n": self.proof.force,
            "proof_depths_m": self.proof.readings.copy(),
            "proof_peaks_m": self.proof.peaks.copy(),
        }
        if trace_dir is not None:
            trace_dir.mkdir(parents=True, exist_ok=True)
            with (trace_dir / f"proof-{self.trace_index}.jsonl").open("a") as stream:
                print(json.dumps({
                    **info,
                    "time_s": self._t,
                    "tcp_m": {arm: [float(v) for v in xyz] for arm, xyz in tcp.items()},
                    "body_m": positions,
                    "spindle_speed_m_s": speed,
                    "proof_start_s": self.proof.start,
                    "proof_valid": self.proof.valid,
                }), file=stream)
        return StepResult(
            observation=self._observation(),
            terminated=success or self.proof.invalid,
            termination_reason="success" if success else "invalid" if self.proof.invalid else None,
            info=info,
        )

    cls._ensure_sim, cls.reset, cls._task_step = ensure, reset, step


if __name__ == "__main__":
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--log-dir", type=Path)
    args, rest = parser.parse_known_args()
    sys.argv = [sys.argv[0], *rest]
    if args.log_dir:
        sys.argv += ["--log-dir", str(args.log_dir)]
    install(args.log_dir)

    from inspect_robots.cli import main

    main()
