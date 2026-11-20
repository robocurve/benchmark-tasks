"""Load the passive zipper mechanism into the calibrated bimanual YAM station."""

import os
import time
from dataclasses import asdict, replace

import numpy as np
from inspect_robots.embodiment import RESETTABLE, SEEDABLE
from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS, READY
from yam_arena.inspect_embodiment import (
    ARMS, DESKTOP_DISPLAY, DESKTOP_SIZE, JOINT_GUIDE, YamArenaEmbodiment, window_display,
)

from .jacket_fabric import FABRIC_NAME, jacket_spawner, initial_vertices
from .mechanism import (
    BRACE_POINT, CONTROL_HZ, INITIAL_PIN, LENGTH, ORIGIN_XY, PIN_GRASP, PIN_LIMITS, SLIDER_HANDLE,
    author_mechanism, pin_seated,
)
from .scoring import Reading
from .visual import ZipperVisual


def make_station(initial_slider, *, rig, wrist_cam_mount, device, auxiliary_grips):
    from isaaclab.assets import ArticulationCfg
    from isaaclab.managers import EventTermCfg
    from isaaclab.sim import CuboidCfg
    from isaaclab.sim.utils import clone, get_current_stage
    from isaaclab_arena.assets.deformable_object import DeformableObject
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.environments.arena_env_builder import ArenaEnvBuilder
    from isaaclab_arena.environments.arena_env_builder_cfg import ArenaEnvBuilderCfg
    from isaaclab_arena.environments.isaaclab_arena_environment import IsaacLabArenaEnvironment
    from isaaclab_arena.scene.scene import Scene
    from isaaclab_arena.terms.events import set_object_pose
    from isaaclab_arena.utils.pose import Pose
    from yam_arena.embodiment import BimanualYamEmbodiment
    from yam_arena.scene import env_cfg_callback, task_room
    from yam_arena.station import TABLE_TOP

    @clone
    def spawn(prim_path, cfg, translation=None, orientation=None, **kwargs):
        return author_mechanism(
            get_current_stage(), prim_path, translation, orientation, initial_slider, auxiliary_grips=auxiliary_grips,
        )

    pose = Pose(position_xyz=(*ORIGIN_XY, TABLE_TOP + 0.001))
    zipper = Object(
        name="zipper",
        object_type=ObjectType.ARTICULATION,
        spawner_cfg=CuboidCfg(size=(0.22, 0.19, 0.05), func=spawn),
        initial_pose=pose,
        asset_cfg_addon={
            "init_state": ArticulationCfg.InitialStateCfg(
                joint_pos={"slider_travel": 0.0, "pin_axial": INITIAL_PIN[0], "pin_lateral": INITIAL_PIN[1], "pin_height": INITIAL_PIN[2]}
            ),
        },
    )
    fabric = DeformableObject(name=FABRIC_NAME, spawner_cfg=jacket_spawner(), initial_pose=pose)
    robot = BimanualYamEmbodiment(
        enable_cameras=True, wrist_cam_mount=wrist_cam_mount, contact_sensors=False, rig=rig,
    )
    visual = None

    def update_zipper(env, env_ids):
        nonlocal visual
        asset = env.scene["zipper"]
        names = ("slider_travel", "pin_axial", "pin_lateral", "pin_height")
        joint_ids = [asset.joint_names.index(name) for name in names]
        positions = asset.data.joint_pos.torch[0, joint_ids].cpu().numpy()
        limits = asset.data.default_joint_pos_limits.torch.clone()
        if not hasattr(env, "zipper_engaged"):
            env.zipper_engaged = False
        if not hasattr(env, "zipper_locked"):
            env.zipper_locked = False
        if not env.zipper_locked:
            env.zipper_engaged = bool(pin_seated(positions[1:]) and (env.zipper_engaged or positions[0] < 0.003))
        if env.zipper_engaged and positions[0] > 0.006:
            env.zipper_locked = True
        limits[:, joint_ids[0], 1] = LENGTH if env.zipper_engaged else 0.002
        if env.zipper_locked:
            for joint_id in joint_ids[1:]:
                limits[:, joint_id, 0] = -0.0004
                limits[:, joint_id, 1] = 0.0004
        asset.write_joint_position_limit_to_sim_index(limits=limits)
        if visual is None:
            visual = ZipperVisual(env.scene.stage, f"{env.scene.env_prim_paths[0]}/zipper")
        visual.update(positions[0], positions[1:], env.zipper_engaged)

    def reset_zipper(env, env_ids, asset_cfg, pose, velocity=None):
        set_object_pose(env, env_ids, asset_cfg, pose, velocity)
        asset = env.scene[asset_cfg.name]
        env.zipper_engaged = False
        env.zipper_locked = False
        asset.write_joint_position_limit_to_sim_index(limits=asset.data.default_joint_pos_limits.torch.clone())
        asset.write_joint_state_to_sim_index(
            position=asset.data.default_joint_pos.torch[env_ids],
            velocity=asset.data.default_joint_vel.torch[env_ids],
            env_ids=env_ids,
        )

    def reset_fabric(env, env_ids, asset_cfg):
        asset = env.scene[asset_cfg.name]
        # The custom mesh origin is the zipper rail, so resetting by centroid would move the seam.
        state = asset.data.default_nodal_state_w.torch[env_ids].clone()
        rest = initial_vertices() + (*ORIGIN_XY, TABLE_TOP + 0.001)
        state[:, :, :3] = env.scene.env_origins[env_ids, None, :] + env.scene["zipper"].data.default_root_pose.torch.new_tensor(rest)
        state[:, :, 3:] = 0
        asset.write_nodal_state_to_sim_index(state, env_ids=env_ids)
        asset.reset(env_ids=env_ids)

    def configure(env_cfg):
        env_cfg = env_cfg_callback(CONTROL_HZ)(env_cfg)
        env_cfg.events.zipper.func = reset_zipper
        env_cfg.events.animate_zipper = EventTermCfg(
            func=update_zipper, mode="interval", interval_range_s=(0.0, 0.0), is_global_time=True,
        )
        event = getattr(env_cfg.events, FABRIC_NAME)
        event.func = reset_fabric
        event.params = {"asset_cfg": event.params["asset_cfg"]}
        return env_cfg

    arena = IsaacLabArenaEnvironment(
        name="textilebench_zipper_station",
        scene=Scene([*task_room("station", rig.top_cam), zipper, fabric]),
        embodiment=robot,
        env_cfg_callback=configure,
    )
    config = ArenaEnvBuilderCfg(num_envs=1, solve_relations=False, device=device)
    return ArenaEnvBuilder(arena, config).make_registered()


class ZipperEmbodiment(YamArenaEmbodiment):
    def __init__(self, task_name, instruction, initial_slider, *, live, auxiliary_grips=True):
        super().__init__(
            scene="station", object_jitter=0, control_hz=CONTROL_HZ,
            cam_width=320, cam_height=176, headless=not live,
        )
        self.initial_slider = initial_slider
        self.auxiliary_grips = auxiliary_grips
        self.app = None
        self.info = replace(
            self.info,
            capabilities=frozenset({SEEDABLE, RESETTABLE}),
            supported_target_kinds=frozenset(),
            docs=f"{JOINT_GUIDE}\n\n{instruction}",
            environment_id=f"{self.info.environment_id}-{task_name}",
        )

    def _ensure_sim(self, goal):
        if self._env is not None:
            return
        from isaaclab.app import AppLauncher

        options = dict(headless=self.headless, device=self.device, enable_cameras=True)
        if not self.headless:
            display = window_display(os.environ)
            os.environ["DISPLAY"] = display
            options["visualizer"] = ["kit"]
            if display == DESKTOP_DISPLAY:
                options["window_width"], options["window_height"] = DESKTOP_SIZE
        self.app = AppLauncher(**options).app

        import torch

        self._env = make_station(
            self.initial_slider, rig=self.rig, wrist_cam_mount=self.wrist_cam_mount, device=self.device,
            auxiliary_grips=self.auxiliary_grips,
        )
        self._base = self._env.unwrapped
        self._torch = torch
        self._ready = torch.tensor([[*READY, 1.0, *READY, 1.0]], dtype=torch.float32, device=self._base.device)
        self._ids = {
            name: (
                self._base.scene[name].find_joints(ARM_JOINTS, preserve_order=True)[0],
                self._base.scene[name].find_joints(FINGER_JOINTS, preserve_order=True)[0],
            )
            for name in ARMS
        }
        self.jacket = self._base.scene["zipper"]
        self.slider_joint = self.jacket.joint_names.index("slider_travel")
        self.pin_joints = [self.jacket.joint_names.index(name) for name in ("pin_axial", "pin_lateral", "pin_height")]
        self.handle_bodies = [self.jacket.body_names.index(name) for name in ("Base", "Slider", "PinSide")]
        self.handle_offsets = np.array([BRACE_POINT, SLIDER_HANDLE[0], PIN_GRASP])

    def reset(self, scene, *, seed=None):
        observation = super().reset(scene, seed=seed)
        if not self.headless:
            self._base.sim.set_camera_view((0.65, -0.47, 1.38), (0.23, 0.0, 0.76))
        return observation

    def handle_positions(self):
        poses = self.jacket.data.body_link_pose_w.torch[0, self.handle_bodies].cpu().numpy()
        if not np.isfinite(poses).all():
            raise RuntimeError("The zipper articulation returned a non-finite state")
        # Rotate each link-local handle center by the body's (x, y, z, w) quaternion.
        xyz = poses[:, 3:6]
        twice_cross = 2 * np.cross(xyz, self.handle_offsets)
        return poses[:, :3] + self.handle_offsets + poses[:, 6:7] * twice_cross + np.cross(xyz, twice_cross)

    def reading(self):
        from yam_arena.ik import tcp_pose
        from yam_arena.station import BASES

        joint_pos = self.jacket.data.joint_pos.torch[0].cpu().numpy()
        if not np.isfinite(joint_pos).all():
            raise RuntimeError("The zipper articulation returned a non-finite state")
        centers = self.handle_positions()
        if not self.auxiliary_grips:
            fabric = self._base.scene[FABRIC_NAME].data.nodal_pos_w.torch[0].cpu().numpy()
            centers = np.concatenate([centers[1:], fabric])
        hands = []
        for side in ("left", "right"):
            name = f"{side}_robot"
            arm_joints, _ = self._ids[name]
            angles = self._base.scene[name].data.joint_pos.torch[0, arm_joints].cpu().numpy()
            hands.append(tcp_pose(angles, BASES[side])[0])
        clearance = np.linalg.norm(centers[:, None, :] - hands, axis=2).min()
        return Reading(float(joint_pos[self.slider_joint]), *map(float, joint_pos[self.pin_joints]),
                       bool(getattr(self._base, "zipper_engaged", False)), bool(clearance > 0.04))

    def step(self, action):
        result = super().step(action)
        return replace(result, info={**result.info, "zipper": asdict(self.reading())})

    def preview(self, scene, *, hold_open):
        from yam_arena.station import TABLE_TOP

        self.reset(scene, seed=0)
        for _ in range(40):
            self._env.step(self._ready)
        final = self.reading()
        pose = self.jacket.data.root_link_pose_w.torch[0].cpu().numpy()
        if np.linalg.norm(pose[:2] - ORIGIN_XY) > 0.020 or np.linalg.norm(pose[3:5]) > 0.15:
            raise RuntimeError(f"The jacket moved or tipped during settling: {pose}")
        if not (abs(final.slider_m) < 0.004 and final.pin_y_m < -0.012
                and -0.005 <= final.pin_z_m <= 0.021 and not final.engaged):
            raise RuntimeError(f"Zipper did not retain its initial state: {final}")
        nodes = self._base.scene[FABRIC_NAME].data.nodal_pos_w.torch[0]
        heights = nodes[:, 2] - TABLE_TOP
        if not self._torch.isfinite(nodes).all() or float(heights.min()) < -0.005:
            raise RuntimeError("The joined jacket fabric did not settle on the table")
        print(f"{FABRIC_NAME}: {len(nodes)} simulated nodes, {float(heights.max()) * 1000:.1f} mm above table",
              flush=True)
        print(f"Initial state verified: {final}", flush=True)
        self.hold_view(hold_open)

    def hold_view(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline and self._base.sim.is_headless_or_exist_active_visualizer():
            self._base.sim.render()
            time.sleep(0.1)
