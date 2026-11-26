"""Connect the articulated prop and recorder to the stock YAM task-file entry point."""

from dataclasses import replace
from functools import wraps
import importlib.abc
import importlib.util
import math
import sys

from geometry import (ACTION_LIMIT, ARM_BASES, CONTROL_HZ, DESK_CENTER, DESK_SIZE, HINGE_FRICTION,
                      PHYSICS_HZ, RESET_ANGLE, RESET_HINGE)


def active(task):
    return task is not None and getattr(task.module, 'LAPTOP_TENT_TASK', False)


def reset_hinge(env, env_ids):
    import torch

    laptop = env.scene['laptop']
    angle = laptop.data.default_joint_pos.torch[env_ids]
    laptop.write_joint_state_to_sim_index(
        position=angle, velocity=laptop.data.default_joint_vel.torch[env_ids], env_ids=env_ids,
    )
    static, dynamic, viscous = (torch.full_like(angle, value) for value in HINGE_FRICTION)
    laptop.write_joint_friction_coefficient_to_sim_index(
        joint_friction_coeff=static, joint_dynamic_friction_coeff=dynamic,
        joint_viscous_friction_coeff=viscous, env_ids=env_ids,
    )


def build_environment(embodiment, task, control_hz):
    from isaaclab.assets import ArticulationCfg
    from isaaclab.managers import EventTermCfg, RecorderTermCfg
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.environments.isaaclab_arena_environment import IsaacLabArenaEnvironment
    from isaaclab_arena.scene.scene import Scene
    from isaaclab_arena.utils.pose import Pose
    from isaaclab_physx.sim.schemas import PhysxCollisionPropertiesCfg
    from yam_arena import scene

    from physics_monitor import TentRecorder

    if control_hz != CONTROL_HZ:
        raise ValueError(f'Use -E control_hz={CONTROL_HZ} for laptop tent mode')
    for side in ('left_robot', 'right_robot'):
        spawn = getattr(embodiment.scene_config, side).spawn
        spawn.collision_props = PhysxCollisionPropertiesCfg(contact_offset=0.0005, rest_offset=0.)
        spawn.make_uninstanceable = True

    def configure(cfg):
        cfg = scene.env_cfg_callback(control_hz)(cfg)
        cfg.sim.dt = 1 / PHYSICS_HZ
        cfg.decimation = PHYSICS_HZ // CONTROL_HZ
        cfg.sim.render_interval = cfg.decimation
        cfg.events.reset_laptop_hinge = EventTermCfg(func=reset_hinge, mode='reset')
        cfg.recorders.laptop_tent = RecorderTermCfg(class_type=TentRecorder)
        return cfg

    laptop = Object(
        name='laptop', object_type=ObjectType.ARTICULATION,
        usd_path=str(task.path.parent / 'assets' / 'laptop.usda'),
        initial_pose=Pose(position_xyz=RESET_HINGE, rotation_xyzw=(0., 0., 0., 1.)),
        asset_cfg_addon=dict(init_state=ArticulationCfg.InitialStateCfg(
            joint_pos={'Hinge': math.radians(RESET_ANGLE - 180)}, joint_vel={'Hinge': 0.},
        )),
    )
    room = [prop for prop in scene.scene_assets(objects=False, d435_collider=False) if prop.name != 'table']
    desk = scene._box('table', DESK_SIZE, DESK_CENTER, (0.55, 0.42, 0.30))
    arm_mount = scene._box('left_mount', (0.18, 0.18, 0.04),
                           (ARM_BASES[0][0], ARM_BASES[0][1], DESK_CENTER[2]), (0.23, 0.25, 0.27))
    return IsaacLabArenaEnvironment(name='yam_laptop_tent_mode', scene=Scene([*room, desk, arm_mount, laptop]),
                                    embodiment=embodiment, env_cfg_callback=configure)


def patch_scene(module):
    original = module.build_task_environment

    @wraps(original)
    def build(embodiment, task, control_hz=None):
        if active(task):
            return build_environment(embodiment, task, control_hz)
        return original(embodiment, task, control_hz)

    module.build_task_environment = build


class SceneImport(importlib.abc.MetaPathFinder):
    """Install the task builder once Isaac Sim has started."""

    def find_spec(self, fullname, path=None, target=None):
        if fullname != 'yam_arena.scene':
            return None
        sys.meta_path.remove(self)
        spec = importlib.util.find_spec(fullname)
        original = spec.loader.exec_module

        def load(module):
            original(module)
            patch_scene(module)

        spec.loader.exec_module = load
        return spec


def side_camera():
    from scipy.spatial.transform import Rotation
    from yam_arena.rig import Camera, Lens

    eye = (0.605, 0.665, 1.055)
    goal = (0.347, -0.040, 0.839)
    dx, dy, dz = (b - a for a, b in zip(eye, goal, strict=True))
    rotation = Rotation.from_euler('ZY', (math.atan2(dy, dx), -math.atan2(dz, math.hypot(dx, dy))))
    return Camera(eye, tuple(rotation.as_quat()), 'world', Lens.from_focal(22.0))


def install():
    from inspect_robots.types import StepResult
    from yam_arena.inspect_embodiment import JOINT_GUIDE, YamArenaEmbodiment
    from yam_arena.scripted import YamScriptedPolicy

    if getattr(YamArenaEmbodiment, '_laptop_tent_adapter', False):
        return
    if 'yam_arena.scene' in sys.modules:
        patch_scene(sys.modules['yam_arena.scene'])
    else:
        sys.meta_path.insert(0, SceneImport())

    reset, step, task_step = YamArenaEmbodiment.reset, YamArenaEmbodiment.step, YamArenaEmbodiment._task_step
    policy_reset, policy_act = YamScriptedPolicy.reset, YamScriptedPolicy.act

    @wraps(reset)
    def reset_task(self, scene, *, seed=None):
        if not active(self.task):
            return reset(self, scene, seed=seed)
        if self.object_jitter or not self.device.startswith('cuda'):
            raise ValueError('Use -E object_jitter=0 and GPU PhysX for the fixed laptop reset')
        self.rig = replace(self.rig, top_cam=side_camera(), bases=dict(zip(('left', 'right'), ARM_BASES, strict=True)))
        self.info = replace(self.info, docs=f'{JOINT_GUIDE}\n\n{self.task.docs}')
        if self._base is not None:
            self._base.tent_recorder.active = False
        obs = reset(self, scene, seed=seed)
        self._base.tent_recorder.start(self.task.module.CONTROL)
        return obs

    @wraps(step)
    def step_task(self, action):
        if active(self.task):
            self._base.tent_recorder.stage_label = action.meta.get('stage', '')
        return step(self, action)

    @wraps(task_step)
    def outcome(self):
        if not active(self.task):
            return task_step(self)
        recorder = self._base.tent_recorder
        obs = self._observation()
        recorder.record_action(obs)
        score = recorder.result.score
        success = score == 1.
        last_action = recorder.actions >= ACTION_LIMIT
        return StepResult(
            observation=obs, terminated=success, truncated=last_action and not success,
            termination_reason='success' if success else 'action_limit' if last_action else None,
            info={'score': score, 'success': success},
        )

    @wraps(policy_reset)
    def reset_policy(self, scene):
        if active(self._task):
            self._laptop_controller = self._task.demo()
            return None
        return policy_reset(self, scene)

    @wraps(policy_act)
    def act_policy(self, observation):
        if active(self._task):
            return self._laptop_controller.act(observation)
        return policy_act(self, observation)

    YamArenaEmbodiment.reset, YamArenaEmbodiment.step, YamArenaEmbodiment._task_step = reset_task, step_task, outcome
    YamScriptedPolicy.reset, YamScriptedPolicy.act = reset_policy, act_policy
    YamArenaEmbodiment._laptop_tent_adapter = True
