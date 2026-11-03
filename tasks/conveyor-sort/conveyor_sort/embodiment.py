"""YAM station with a PhysX contact-velocity conveyor and visual stripes."""
from yam_arena.inspect_embodiment import YamArenaEmbodiment

BELT_SPEED = 0.025  # metres per second in world +y


def spawn_belt(prim_path, cfg, **kwargs):
    from isaaclab.sim.spawners.shapes.shapes import spawn_cuboid
    from pxr import Gf, PhysxSchema

    prim = spawn_cuboid(prim_path, cfg, **kwargs)
    surface = PhysxSchema.PhysxSurfaceVelocityAPI.Apply(prim)
    surface.CreateSurfaceVelocityEnabledAttr().Set(True)
    surface.CreateSurfaceVelocityLocalSpaceAttr().Set(False)
    surface.CreateSurfaceVelocityAttr().Set(Gf.Vec3f(0.0, BELT_SPEED, 0.0))
    return prim


class ConveyorEmbodiment(YamArenaEmbodiment):
    def __init__(self, view="raised", **kwargs):
        if view not in ("raised", "original"):
            raise ValueError("view must be 'raised' or 'original'")
        self.view = view
        kwargs.setdefault("device", "cpu")
        if kwargs["device"] != "cpu":
            raise ValueError("Use CPU PhysX: GPU physics did not maintain conveyor contacts in validation")
        super().__init__(**kwargs)

    def _ensure_sim(self, goal):
        # Task-file shapes normally make fixed static colliders. Replace just
        # the belt with a kinematic collider, and make the stripes visual-only.
        import atexit
        import os
        from yam_arena import inspect_embodiment as base_module
        if base_module._APP is None:
            from isaaclab.app import AppLauncher
            os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")
            kwargs = {"headless": self.headless, "device": self.device, "enable_cameras": True}
            if not self.headless:
                kwargs["visualizer"] = ["kit"]
                os.environ["DISPLAY"] = base_module.window_display(os.environ)
            base_module._APP = AppLauncher(**kwargs).app
            atexit.register(base_module._close_app_at_exit)
        from yam_arena import scene
        from isaaclab.sim import PhysxRigidBodyPropertiesCfg

        original = scene.task_object

        def make_object(spec):
            obj = original(spec)
            if spec.name == "belt":
                obj.spawner_cfg.rigid_props = PhysxRigidBodyPropertiesCfg(kinematic_enabled=True)
                obj.spawner_cfg.func = spawn_belt
            elif spec.name.startswith("stripe_"):
                obj.spawner_cfg.collision_props = None
            if spec.name == "belt" or spec.name.startswith("stripe_"):
                obj.object_cfg = obj._init_object_cfg()
            return obj

        scene.task_object = make_object
        try:
            return super()._ensure_sim(goal)
        finally:
            scene.task_object = original

    def _place_block(self, name, position):
        obj = self._base.scene[name]
        pose = obj.data.default_root_pose.torch.clone()
        pose[:, :3] = self._torch.tensor(position, device=pose.device)
        pose[:, :3] += self._base.scene.env_origins
        obj.write_root_pose_to_sim_index(root_pose=pose)
        obj.write_root_velocity_to_sim_index(root_velocity=self._torch.zeros_like(obj.data.default_root_vel.torch))

    def reset(self, scene, *, seed=None):
        super().reset(scene, seed=seed)
        self._released = set()
        self._discarded = set()
        # A higher overhead view reveals the entire loading shelf and belt.
        # Wrist cameras and the physical station remain unchanged.
        if self.view == "raised":
            camera = self._base.scene["top_cam"]
            camera.set_world_poses_from_view([[0.15, 0, 2.20]], [[0.36, 0, 0.75]])
            for _ in range(8):
                self._base.sim.render()
                camera.update(0.05, force_recompute=True)
        return self._observation()

    def _feed_due(self):
        # Kinematically slide one visible block at a time down the fixed-height
        # loading shelf. Once it reaches the belt, PhysX controls it normally.
        task = self.task.module
        for i, (name, release) in enumerate(task.STARTS):
            if name in self._released or self._t < release:
                continue
            x = max(0.34, task.FEED_X + i*task.FEED_SPACING - task.FEED_SPEED*(self._t-release))
            self._place_block(name, (x, task.FEED_Y, task.BELT_TOP + 0.02))
            if x <= 0.34:
                self._released.add(name)

    def _task_step(self):
        # A closed gripper close to the cube is required for lift credit.
        # Positional elevation alone could reward fallen blocks stacked at the exit.
        from yam_arena import ik, station
        from yam_arena.constants import FINGER_OPEN
        from yam_arena.scene import object_states
        result = super()._task_step()
        hands = []
        for side in ("left", "right"):
            robot = self._base.scene[f"{side}_robot"]
            arm_ids, finger_ids = self._ids[f"{side}_robot"]
            q = robot.data.joint_pos.torch[0]
            pos, _ = ik.tcp_pose(q[arm_ids].cpu().numpy(), station.BASES[side])
            closed = float(q[finger_ids].mean().item() / FINGER_OPEN) < 0.8
            hands.append((pos, closed))
        states = object_states(self._env, self.task.names)
        result.info["score"] = float(self.task.score(states, hands))
        return result

    def step(self, action):
        from yam_arena.scene import object_states
        # Missed blocks are removed at the end of the belt; they cannot pile up
        # and feed again or score in a destination bin.
        states = object_states(self._env, tuple(self._released - self._discarded))
        for name, state in states.items():
            if state.pos[1] > 0.60 and state.pos[0] > 0.27:
                self._place_block(name, (1.7, 0.1 * len(self._discarded), 0.03))
                self._discarded.add(name)
        self._feed_due()
        from pxr import Gf, UsdGeom
        import omni.usd

        stage = omni.usd.get_context().get_stage()
        for i in range(10):
            prim = stage.GetPrimAtPath(f"/World/envs/env_0/stripe_{i}")
            if not prim.IsValid():
                raise RuntimeError(f"Missing conveyor stripe {i}")
            for op in UsdGeom.Xformable(prim).GetOrderedXformOps():
                if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                    y = ((-0.54+i*0.12 + self._t*BELT_SPEED + 0.60) % 1.20) - 0.60
                    op.Set(Gf.Vec3d(0.34, y, 0.7678))
                    break
        return super().step(action)


def make_embodiment(**kwargs):
    return ConveyorEmbodiment(**kwargs)
