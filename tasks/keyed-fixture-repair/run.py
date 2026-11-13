"""Run the keyed fixture in the official Isaac Lab-Arena YAM embodiment."""

import argparse
import atexit
import json
import os
import sys
from pathlib import Path

import numpy as np
from yam_arena.urdf import quat_xyzw_to_matrix

import scoring
import task


def install_scene():
    from isaaclab.sim.utils import clone
    from isaaclab_arena.assets.object import Object
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics, UsdShade
    from yam_arena import scene

    original = scene.task_object

    def task_object(spec):
        obj = original(spec)
        if spec.name not in task.PARTS:
            return obj

        @clone
        def spawn(path, cfg, translation=None, orientation=None, **kwargs):
            import omni.usd

            stage = omni.usd.get_context().get_stage()
            root = UsdGeom.Xform.Define(stage, path)
            root.AddTranslateOp().Set(Gf.Vec3d(*(translation if translation is not None else spec.pos)))
            # Isaac Lab 3 spawners receive xyzw. The Arena scene writes each
            # dynamic object's initial pose (including its yaw) on reset.
            x, y, z, w = orientation if orientation is not None else (0., 0., 0., 1.)
            root.AddOrientOp().Set(Gf.Quatf(w, Gf.Vec3f(x, y, z)))
            prim = root.GetPrim()
            if spec.free:
                UsdPhysics.RigidBodyAPI.Apply(prim)
                UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(spec.mass)
                phys = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
                phys.CreateSolverPositionIterationCountAttr(32)
                phys.CreateSolverVelocityIterationCountAttr(8)
                phys.CreateEnableCCDAttr(True)
            mat = UsdShade.Material.Define(stage, f"{path}/contact_material")
            physics = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
            # The lock pin is a ground steel pin in steel bushings; the rest is polymer or aluminium.
            pin = spec.name == "lock_pin"
            physics.CreateStaticFrictionAttr(.15 if pin else .45)
            physics.CreateDynamicFrictionAttr(.10 if pin else .30)
            for part in task.PARTS[spec.name]:
                path_to_part = f"{path}/{part.name}"
                if part.vertices:
                    geom = UsdGeom.Mesh.Define(stage, path_to_part)
                    # A convex prism: one ring of n vertices, then the matching ring.
                    points = [Gf.Vec3f(*xyz) for xyz in part.vertices]
                    n = len(part.vertices) // 2
                    faces = [list(range(n))[::-1], list(range(n, 2 * n))] + [
                        [i, (i + 1) % n, n + (i + 1) % n, n + i] for i in range(n)]
                    vertices = np.asarray(part.vertices)
                    if np.dot(np.cross(vertices[1] - vertices[0], vertices[2] - vertices[0]),
                              vertices[n] - vertices[0]) < 0:
                        faces = [f[::-1] for f in faces]
                    geom.CreatePointsAttr(points)
                    geom.CreateFaceVertexCountsAttr([len(f) for f in faces])
                    geom.CreateFaceVertexIndicesAttr([i for f in faces for i in f])
                    geom.CreateSubdivisionSchemeAttr("none")
                else:
                    geom = UsdGeom.Cube.Define(stage, path_to_part)
                    geom.CreateSizeAttr(1.)
                    geom.AddTranslateOp().Set(Gf.Vec3d(*part.pos))
                    if part.yaw:
                        geom.AddRotateZOp().Set(float(np.degrees(part.yaw)))
                    geom.AddScaleOp().Set(Gf.Vec3f(*part.size))
                geom.CreateDisplayColorPrimvar().Set([Gf.Vec3f(*scene.linear_rgb(part.color))])
                if part.collision:
                    shape = geom.GetPrim()
                    UsdPhysics.CollisionAPI.Apply(shape)
                    if part.vertices:
                        UsdPhysics.MeshCollisionAPI.Apply(shape).CreateApproximationAttr("convexHull")
                    collision = PhysxSchema.PhysxCollisionAPI.Apply(shape)
                    collision.CreateContactOffsetAttr(.00035)
                    collision.CreateRestOffsetAttr(0.)
                    UsdShade.MaterialBindingAPI.Apply(shape).Bind(mat, materialPurpose="physics")
            return prim

        obj.spawner_cfg.func = spawn
        return Object(name=spec.name, object_type=obj.object_type,
                      spawner_cfg=obj.spawner_cfg, initial_pose=obj.initial_pose)

    scene.task_object = task_object


def measure(base, name):
    obj = base.scene[name].data
    raw = obj.root_link_pose_w.torch[0].cpu().numpy()
    velocity = obj.root_link_vel_w.torch[0].cpu().numpy()
    return scoring.Pose(raw[:3] - base.scene.env_origins[0].cpu().numpy(),
                        quat_xyzw_to_matrix(raw[3:]),
                        float(np.linalg.norm(velocity[:3])), float(np.linalg.norm(velocity[3:])))


def install(trace_dir=None, *, servo=False):
    from inspect_robots.types import StepResult
    from yam_arena import inspect_embodiment as arena
    from yam_arena.runtime import link_poses, tcp_world

    cls = arena.YamArenaEmbodiment
    original_ensure, original_reset = cls._ensure_sim, cls.reset
    scene_installed = False
    active = None
    if servo:
        import servo_demo

        servo_demo.install(lambda: active, measure)

    def ensure(self, goal):
        nonlocal scene_installed
        if arena._APP is None:
            from isaaclab.app import AppLauncher

            os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")
            opts = {"headless": self.headless, "device": self.device, "enable_cameras": True}
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
        nonlocal active
        active = self
        self.repair = scoring.Repair(max(1, round(self.info.control_hz)))
        self.trace_index = getattr(self, "trace_index", 0) + 1
        return original_reset(self, scene, seed=seed)

    def step(self):
        base = self._base
        poses = {name: measure(base, name) for name in scoring.ALL_FREE}
        tcp = {arm: np.asarray(tcp_world(base.scene[arm])) for arm in arena.ARMS}

        def clear(xy, top):
            # With the actual YAM fingers at most about 2 cm below TCP, the
            # extra standoff also leaves a visible gap during a proof.
            return not any(np.linalg.norm(p[:2] - xy) <= .105 and p[2] < task.TOP + top
                           for p in tcp.values())

        hands_clear = {"fixture": clear(task.FIX, .132),
                       "cradle": clear(task.CRADLE, .120),
                       "tray": clear(task.REJECT, .102)}
        value = self.repair.update(poses, hands_clear)
        success = value == 1.0
        part = poses["test_part"]
        info = {
            "score": value,
            "success": success,
            "installed": self.repair.current,
            "fault_proven": self.repair.fault_proven,
            "wrong_spare_tried": self.repair.wrong_tried,
            "pin_home": bool(scoring.pin_home(poses["lock_pin"])),
            "replaced": self.repair.replaced,
            "reading": self.repair.previous,
            "reading_hold_steps": self.repair.count,
            "verified": sorted(self.repair.verified),
            "finish_hold_steps": self.repair.finish_count,
            "height_above_seat_mm": float((part.pos[2] - task.TOP - task.SEAT) * 1000),
            "hands_clear": hands_clear,
        }
        if trace_dir is not None:
            trace_dir.mkdir(parents=True, exist_ok=True)
            with (trace_dir / f"proof-{self.trace_index}.jsonl").open("a") as stream:
                print(json.dumps({
                    **info,
                    "t": self._t,
                    "tcp": {k: v.tolist() for k, v in tcp.items()},
                    "tips": {k: v[:3, 3].tolist() for k, v in link_poses(base.scene["right_robot"]).items()
                             if k.startswith("tip")},
                    "poses": {k: [p.pos.tolist(), scoring.yaw_error(p.rot, 0),
                                  p.linear, p.angular, bool(p.upright)] for k, p in poses.items()},
                }), file=stream)
        return StepResult(observation=self._observation(),
                          terminated=success or self.repair.invalid,
                          termination_reason="success" if success else "object_fell" if self.repair.invalid else None,
                          info=info)

    cls._ensure_sim, cls.reset, cls._task_step = ensure, reset, step


if __name__ == "__main__":
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--log-dir", type=Path)
    args, rest = parser.parse_known_args()
    sys.argv = [sys.argv[0], *rest]
    if args.log_dir:
        sys.argv += ["--log-dir", str(args.log_dir)]
    install(args.log_dir, servo="--policy" in rest and rest[rest.index("--policy") + 1] == "yam_scripted")

    from inspect_robots.cli import main

    main()
