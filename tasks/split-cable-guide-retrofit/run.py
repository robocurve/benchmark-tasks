"""Run the task in yam_arena with compound colliders and constrained fixture parts."""

import argparse
import atexit
import json
import math
import os
from pathlib import Path

import numpy as np
import task as fixture


def install_scene():
    from isaaclab.sim.utils import clone
    from isaaclab_arena.assets.object import Object
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics, UsdShade
    from yam_arena import scene

    original = scene.task_object
    specs = {spec.name: spec for spec in fixture.OBJECTS}

    def local_anchor(anchor, spec):
        if spec is None:
            return Gf.Vec3f(*anchor)
        x, y, z = np.subtract(anchor, spec.pos)
        c, s = math.cos(spec.yaw), math.sin(spec.yaw)
        return Gf.Vec3f(c * x + s * y, -s * x + c * y, z)

    def author_joint(stage, root, spec, index, rule):
        schema = {
            "slide": UsdPhysics.PrismaticJoint,
            "fixed": UsdPhysics.FixedJoint,
            "ball": UsdPhysics.SphericalJoint,
        }[rule.kind]
        joint = schema.Define(stage, f"{root}/constraint_{index}")
        joint.CreateBody1Rel().SetTargets([root])
        if rule.parent:
            joint.CreateBody0Rel().SetTargets(
                [root.GetParentPath().AppendChild(rule.parent)]
            )
        joint.CreateLocalPos0Attr(local_anchor(rule.anchor, specs.get(rule.parent)))
        joint.CreateLocalPos1Attr(local_anchor(rule.anchor, spec))
        joint.CreateCollisionEnabledAttr(False)
        if rule.kind == "slide":
            joint.CreateAxisAttr(rule.axis)
            joint.CreateLowerLimitAttr(rule.limits[0])
            joint.CreateUpperLimitAttr(rule.limits[1])
            if rule.spring:
                stiffness, damping, max_force = rule.spring
                drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "linear")
                drive.CreateTypeAttr("force")
                drive.CreateStiffnessAttr(stiffness)
                drive.CreateDampingAttr(damping)
                drive.CreateMaxForceAttr(max_force)
                drive.CreateTargetPositionAttr(0.0)

    def task_object(spec):
        obj = original(spec)
        if spec.name not in specs:
            return obj

        @clone
        def spawn(path, cfg, translation=None, orientation=None, **kwargs):
            import omni.usd

            stage = omni.usd.get_context().get_stage()
            root = UsdGeom.Xform.Define(stage, path)
            root.AddTranslateOp().Set(
                Gf.Vec3d(*(translation if translation is not None else spec.pos))
            )
            x, y, z, w = orientation if orientation is not None else (0, 0, 0, 1)
            root.AddOrientOp().Set(Gf.Quatf(w, Gf.Vec3f(x, y, z)))
            prim = root.GetPrim()
            if spec.free:
                UsdPhysics.RigidBodyAPI.Apply(prim)
                UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(spec.mass)
                phys = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
                phys.CreateSolverPositionIterationCountAttr(32)
                phys.CreateSolverVelocityIterationCountAttr(8)
                phys.CreateSleepThresholdAttr(0.0)
            mat = UsdShade.Material.Define(stage, f"{path}/contact_material")
            physics = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
            slippery = spec.name in fixture.LINKS or spec.name in {
                "cable_run",
                "loop_tray",
                "drawer_rail",
            }
            physics.CreateStaticFrictionAttr(0.10 if slippery else 0.8)
            physics.CreateDynamicFrictionAttr(0.07 if slippery else 0.6)
            material = PhysxSchema.PhysxMaterialAPI.Apply(mat.GetPrim())
            material.CreateFrictionCombineModeAttr("min" if slippery else "average")
            for part in fixture.SHAPES[spec.name]:
                part_path = f"{path}/{part.name}"
                if part.cylinder:
                    geom = UsdGeom.Cylinder.Define(stage, part_path)
                    geom.CreateAxisAttr("Y")
                    geom.CreateRadiusAttr(part.size[0])
                    geom.CreateHeightAttr(part.size[1])
                else:
                    geom = UsdGeom.Cube.Define(stage, part_path)
                    geom.CreateSizeAttr(1.0)
                geom.AddTranslateOp().Set(Gf.Vec3d(*part.pos))
                if not part.cylinder:
                    geom.AddScaleOp().Set(Gf.Vec3f(*part.size))
                geom.CreateDisplayColorPrimvar().Set(
                    [Gf.Vec3f(*scene.linear_rgb(part.color))]
                )
                shape = geom.GetPrim()
                UsdPhysics.CollisionAPI.Apply(shape)
                collision = PhysxSchema.PhysxCollisionAPI.Apply(shape)
                collision.CreateContactOffsetAttr(0.0004)
                collision.CreateRestOffsetAttr(0.0)
                UsdShade.MaterialBindingAPI.Apply(shape).Bind(
                    mat, materialPurpose="physics"
                )
            for index, rule in enumerate(fixture.JOINTS[spec.name]):
                author_joint(stage, root.GetPath(), spec, index, rule)
            return prim

        spawner = obj.spawner_cfg
        spawner.func = spawn
        return Object(
            name=spec.name,
            object_type=obj.object_type,
            spawner_cfg=spawner,
            initial_pose=obj.initial_pose,
        )

    scene.task_object = task_object


def install(*, trace=None):
    from inspect_robots.types import StepResult
    from yam_arena import inspect_embodiment as ie
    from yam_arena.runtime import tcp_world

    cls = ie.YamArenaEmbodiment
    original_ensure, original_reset = cls._ensure_sim, cls.reset
    scene_installed = False

    def ensure(self, goal):
        nonlocal scene_installed
        os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")
        if ie._APP is None:
            from isaaclab.app import AppLauncher

            opts = {
                "headless": self.headless,
                "device": self.device,
                "enable_cameras": True,
            }
            if not self.headless:
                os.environ["DISPLAY"] = ie.window_display(os.environ)
                opts["visualizer"] = ["kit"]
            ie._APP = AppLauncher(**opts).app
            atexit.register(ie._close_app_at_exit)
        if not scene_installed:
            install_scene()
            scene_installed = True
        return original_ensure(self, goal)

    def root(base, name):
        return (
            base.scene[name].data.root_link_pose_w.torch[0, :3]
            - base.scene.env_origins[0]
        )

    def reset(self, scene, *, seed=None):
        self.fixture_trial = fixture.Trial()
        self.fixture_armed = False
        self.fixture_unsafe = False
        self.fixture_max_gap = 0.0
        self.fixture_trace_index = getattr(self, "fixture_trace_index", 0) + 1
        self._ensure_sim(None)
        base = self._base
        if not getattr(base.scene, "_guide_drive_installed", False):
            import warp as wp

            original_write = base.scene.write_data_to_sim
            original_update = base.scene.update
            drawer = base.scene["drawer"]
            force = self._torch.zeros(
                (1, 3), dtype=self._torch.float32, device=base.device
            )
            torque = self._torch.zeros_like(force)

            def write():
                original_write()
                delta = root(base, "drawer")[1] - fixture.DRAWER_POS[1]
                velocity = drawer.data.root_link_vel_w.torch[0, 1]
                if self.fixture_trial.start is None:
                    # Closed-position latch. It releases only after physical button travel.
                    force[0, 1] = (-4500.0 * delta - 40.0 * velocity).clamp(-30.0, 30.0)
                else:
                    # Preloaded opening spring and damper; the joint supplies the physical end stop.
                    force[0, 1] = (1.45 - 8.0 * delta - 4.0 * velocity).clamp(-1.6, 1.6)
                drawer.root_view.apply_forces_and_torques_at_position(
                    force_data=wp.from_torch(force),
                    torque_data=wp.from_torch(torque),
                    position_data=None,
                    indices=drawer._ALL_INDICES,
                    is_global=True,
                )

            def update(dt):
                original_update(dt)
                if self.fixture_armed:
                    # Preserve brief disconnects even when the plug returns before a camera frame.
                    delta = root(base, "drawer")[1] - fixture.DRAWER_POS[1]
                    fixed_gap = (
                        root(base, "fixed_plug")[1] - fixture.FIXED_PLUG_POS[1]
                    ).abs()
                    moving_gap = (
                        root(base, "moving_plug")[1]
                        - fixture.MOVING_PLUG_POS[1]
                        - delta
                    ).abs()
                    self.fixture_max_gap = max(
                        self.fixture_max_gap, fixed_gap.item(), moving_gap.item()
                    )
                    self.fixture_unsafe |= self.fixture_max_gap > 0.002

            base.scene.write_data_to_sim = write
            base.scene.update = update
            base.scene._guide_drive_installed = True
        self.task.module.TRIAL = self.fixture_trial
        obs = original_reset(self, scene, seed=seed)
        self.fixture_armed = True
        return obs

    def step(self):
        base = self._base
        states = {}
        names = (
            *fixture.GUIDES,
            "keeper",
            "drawer",
            "cable_run",
            "release",
            "fixed_plug",
            "moving_plug",
        )
        for name in names:
            states[name] = root(base, name).detach().cpu().numpy()
        travel = float(states["drawer"][1] - fixture.DRAWER_POS[1])
        clear = True
        tcp_positions = {}
        for arm in ie.ARMS:
            tcp = np.asarray(tcp_world(base.scene[arm]))
            tcp_positions[arm] = tcp.tolist()
            # The release lies outside this region. The bounds include clearance for
            # the fingers around the guide, drawer, and exposed travel section.
            over_stroke = 0.182 <= tcp[0] <= 0.328 and -0.105 <= tcp[1] <= 0.258
            over_loop = 0.105 <= tcp[0] <= 0.285 and -0.251 <= tcp[1] <= -0.078
            if (over_stroke or over_loop) and fixture.TOP <= tcp[
                2
            ] <= fixture.Z + 0.095:
                clear = False
        seated = None
        for name in fixture.GUIDES:
            delta = states[name] - (fixture.X, 0.0, fixture.Z)
            q = (
                base.scene[name]
                .data.root_link_pose_w.torch[0, 3:]
                .detach()
                .cpu()
                .numpy()
            )
            # Isaac Lab 3 reports root quaternions in XYZW order. The halves' top/bottom and tabs are keyed.
            if np.all(np.abs(delta) <= (0.0020, 0.0025, 0.0020)) and abs(
                q[3]
            ) >= math.cos(math.radians(4) / 2):
                seated = name
        press = bool(states["release"][2] < fixture.RELEASE_POS[2] - 0.005)
        keeper_closed = bool(
            states["keeper"][1] >= fixture.KEEPER_POS[1] + fixture.KEEPER_TRAVEL - 0.002
        )
        unsafe = self.fixture_unsafe or not all(
            np.isfinite(p).all() for p in states.values()
        )
        unsafe |= any(states[name][2] < fixture.TOP - 0.010 for name in fixture.GUIDES)
        sleeve_clear = bool(
            states["cable_run"][1] + fixture.SLEEVE_Y - fixture.SLEEVE_LENGTH / 2
            > fixture.GUIDE_DEPTH / 2
        )
        trial = self.fixture_trial
        sample = fixture.Sample(
            guide=seated,
            keeper_closed=keeper_closed,
            hands_clear=clear,
            press=press,
            travel=travel,
            sleeve_clear=sleeve_clear,
            damaged=unsafe,
            assisted=not clear,
        )
        trial.step(self._t, sample)
        info = {
            "score": trial.score,
            "success": trial.score == 1.0,
            "guide": seated,
            "drawer_travel_m": travel,
            "keeper_closed": keeper_closed,
            "release": press,
            "sleeve_clear": sleeve_clear,
            "hands_clear": clear,
            "invalid": trial.invalid,
            "proof_started": trial.start,
            "max_plug_gap_m": self.fixture_max_gap,
            "time_s": self._t,
        }
        terminated = trial.invalid or trial.score == 1.0
        if trace is not None:
            trace.mkdir(parents=True, exist_ok=True)
            record = {
                **info,
                "tcp_m": tcp_positions,
                "objects": {n: p.tolist() for n, p in states.items()},
            }
            with (trace / f"proof-{self.fixture_trace_index}.jsonl").open(
                "a"
            ) as output:
                print(json.dumps(record), file=output)
        return StepResult(
            observation=self._observation(),
            terminated=terminated,
            termination_reason=(
                "unsafe" if trial.invalid else "success" if terminated else None
            ),
            info=info,
        )

    cls._ensure_sim, cls.reset, cls._task_step = ensure, reset, step


if __name__ == "__main__":
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--log-dir", type=Path)
    args, rest = parser.parse_known_args()
    import sys

    sys.argv = [sys.argv[0], *rest]
    if args.log_dir:
        sys.argv += ["--log-dir", str(args.log_dir)]
    install(trace=args.log_dir)
    from inspect_robots.cli import main

    main()
