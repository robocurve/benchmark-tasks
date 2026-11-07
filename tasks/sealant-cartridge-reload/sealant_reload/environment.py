"""Native passive mechanism; policies can actuate only the ordinary 14 robot targets."""
from __future__ import annotations

import atexit
from dataclasses import replace
import hashlib
import importlib.util
import json
import os
import tempfile

import numpy as np
from inspect_robots.embodiment import PRIVILEGED_SUCCESS
from inspect_robots.types import StepResult
from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS, READY
from yam_arena.inspect_embodiment import ARMS, JOINT_GUIDE, ROOM_DOCS, YamArenaEmbodiment

from . import TASK_ROOT
from .config import CONTROL_HZ, FRAME_POSITION, OLD_POSITION, NEW_POSITION
from .scoring import Scorer
from .geometry import local_solids, world_points, robot_clearance

_APP = None


def numpy(value):
    if hasattr(value, 'torch'):
        value = value.torch
    if hasattr(value, 'detach'):
        return value.detach().cpu().numpy().astype(float)
    if hasattr(value, 'numpy'):
        return value.numpy().astype(float)
    return np.asarray(value, dtype=float)


def source_fingerprint():
    digest = hashlib.sha256()
    for p in sorted(TASK_ROOT.rglob('*')):
        if not p.is_file() or any(s in p.parts for s in ('__pycache__', '.pytest_cache', '.ruff_cache')):
            continue
        if '.egg-info' in str(p):
            continue
        if p.suffix == '.py' or p.name == 'pyproject.toml':
            digest.update(p.relative_to(TASK_ROOT).as_posix().encode()+b'\0')
            digest.update(hashlib.sha256(p.read_bytes()).digest())
    return digest.hexdigest()


def close_app():
    if _APP is not None:
        _APP.close()


class SealantReloadEnvironment(YamArenaEmbodiment):
    def __init__(self, *, object_jitter=0, control_hz=CONTROL_HZ, **kwargs):
        if float(object_jitter) != 0 or float(control_hz) != CONTROL_HZ:
            raise ValueError('Use the fixed layout at 20 Hz')
        super().__init__(scene='station', object_jitter=0, control_hz=CONTROL_HZ, **kwargs)
        self.info = replace(self.info, name='sealant_reload', supported_target_kinds=frozenset(),
            capabilities=self.info.capabilities | {PRIVILEGED_SUCCESS},
            docs='\n\n'.join((JOINT_GUIDE, ROOM_DOCS['station'],
                'An unpowered ratchet-rod dispenser holds an empty orange cartridge. A prepared blue '
                'cartridge and a collection tray are beside it. The cradle supports the free tool. '
                'The cartridges are inert; only replacement is requested.')),
            environment_id='sealant-reload-'+source_fingerprint()[:16],
            environment_revision=source_fingerprint()[:12])
        self.task = self
        self.scorer = Scorer()
        self.metrics = {}
        self._callback = None
        self._temp = None
        self._audit_active = False
        self._audit_error = None

    def _ensure_sim(self, goal=None):
        global _APP
        if self._env is not None:
            return
        if os.environ.get('OMNI_KIT_ACCEPT_EULA', '').upper() not in {'YES', 'Y', 'TRUE', '1'}:
            raise RuntimeError('The operator must accept NVIDIA\'s license before setting OMNI_KIT_ACCEPT_EULA')
        if _APP is None:
            from isaaclab.app import AppLauncher
            _APP = AppLauncher(headless=self.headless, device=self.device, enable_cameras=True, multi_gpu=False).app
            atexit.register(close_app)
        import torch
        from isaaclab_arena.assets.object import Object, ObjectType
        from isaaclab_arena.environments.arena_env_builder import ArenaEnvBuilder
        from isaaclab_arena.environments.arena_env_builder_cfg import ArenaEnvBuilderCfg
        from isaaclab_arena.environments.isaaclab_arena_environment import IsaacLabArenaEnvironment
        from isaaclab_arena.scene.scene import Scene
        from isaaclab_arena.utils.pose import Pose
        from isaaclab_physx.physics.physx_manager import IsaacEvents
        from yam_arena.embodiment import BimanualYamEmbodiment
        from yam_arena.scene import env_cfg_callback, task_room

        spec = importlib.util.spec_from_file_location('sealant_asset_builder', TASK_ROOT/'build_scene.py')
        builder = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(builder)
        self._temp = tempfile.TemporaryDirectory(prefix='sealant-reload-assets-')
        assets = builder.create_assets(self._temp.name)
        self._local = local_solids(json.loads((assets['fixtures'].parent/'geometry-inventory.json').read_text()))
        robot = BimanualYamEmbodiment(enable_cameras=True, rig=self.rig, contact_sensors=False,
                                     robot_contact_offset=.001)
        apparatus = [
            Object(name='dispenser', object_type=ObjectType.ARTICULATION, usd_path=str(assets['dispenser']),
                   initial_pose=Pose(position_xyz=tuple(map(float, FRAME_POSITION)))),
            Object(name='used', object_type=ObjectType.RIGID, usd_path=str(assets['used']),
                   initial_pose=Pose(position_xyz=tuple(map(float, OLD_POSITION)))),
            Object(name='replacement', object_type=ObjectType.RIGID, usd_path=str(assets['replacement']),
                   initial_pose=Pose(position_xyz=tuple(map(float, NEW_POSITION)))),
            Object(name='fixtures', object_type=ObjectType.BASE, usd_path=str(assets['fixtures']),
                   initial_pose=Pose(position_xyz=(0., 0., 0.))),
        ]
        arena = IsaacLabArenaEnvironment(name=self.info.environment_id,
            scene=Scene([*task_room('station', self.rig.top_cam), *apparatus]),
            embodiment=robot, env_cfg_callback=env_cfg_callback(CONTROL_HZ))
        self._env = ArenaEnvBuilder(arena, ArenaEnvBuilderCfg(
            num_envs=1, solve_relations=False, device=self.device)).make_registered()
        self._base, self._torch = self._env.unwrapped, torch
        self._ready = torch.tensor([[*READY, 1., *READY, 1.]], dtype=torch.float32, device=self._base.device)
        self._ids = {name: (
            self._base.scene[name].find_joints(ARM_JOINTS, preserve_order=True)[0],
            self._base.scene[name].find_joints(FINGER_JOINTS, preserve_order=True)[0]) for name in ARMS}
        self._callback = self._base.sim.physics_manager.register_callback(
            self._observe_physics, IsaacEvents.POST_PHYSICS_STEP, order=100,
            name='sealant_reload_validity', wrap_weak_ref=False)

    def _observe_physics(self, dt):
        if not self._audit_active:
            return
        try:
            gun = self._base.scene['dispenser']
            transforms = numpy(gun.root_view.get_link_transforms())[0]
            poses = {'dispenser:'+n: p for n, p in zip(gun.body_names, transforms) if 'dispenser:'+n in self._local}
            poses.update({k+':Body': numpy(self._base.scene[k].root_view.get_transforms())[0]
                          for k in ('used', 'replacement')})
            self.scorer.observe_physics(world_points(self._local, poses))
        except Exception as exc:
            self._audit_error = exc

    def reset(self, scene, *, seed=None):
        self._ensure_sim()
        self._audit_active = False
        self._env.reset(seed=seed)
        gun = self._base.scene['dispenser']
        q = gun.data.joint_pos
        q = q.torch if hasattr(q, 'torch') else q
        values = self._torch.zeros(q.shape, dtype=q.dtype, device=q.device)
        gun.write_joint_state_to_sim_index(position=values, velocity=values)
        self._instruction = scene.instruction
        for _ in range(10):
            self._env.step(self._ready)
        for _ in range(4):
            self._base.sim.render()
        self.scorer.reset()
        self._audit_error = None
        self._t = 0.
        self._audit_active = True
        self.metrics = self.scorer.evaluate(self.state())
        if self.metrics['score'] != 0 or self.metrics['violations']:
            raise RuntimeError(f'Invalid reset: {self.metrics}')
        if np.linalg.norm(np.array(self.state()['used']['pose'][:3])-OLD_POSITION) > .01:
            raise RuntimeError('Used cartridge did not settle into its declared starting layout')
        return self._observation()

    def state(self):
        from yam_arena.runtime import tcp_world
        gun = self._base.scene['dispenser']
        poses = numpy(gun.data.body_link_pose_w)[0]
        velocity = numpy(gun.data.body_com_vel_w)[0]
        result = {'time_s': self._t,
            'mechanism': {n: float(v) for n, v in zip(gun.joint_names, numpy(gun.data.joint_pos)[0])},
            'bodies': {n: {'pose': p.tolist(), 'velocity': v.tolist()}
                       for n, p, v in zip(gun.body_names, poses, velocity)},
            'hands': {}}
        for k in ('used', 'replacement'):
            item = self._base.scene[k]
            result[k] = {'pose': numpy(item.data.root_link_pose_w)[0].tolist(),
                         'velocity': numpy(item.data.root_com_vel_w)[0].tolist()}
        for k in ARMS:
            robot = self._base.scene[k]
            q = numpy(robot.data.joint_pos)[0]
            result['hands'][k] = {'tcp': tcp_world(robot).tolist(),
                'joint_positions': q.tolist(), 'opening_m': float(-q[self._ids[k][1]].sum()),
                'poses': numpy(robot.data.body_link_pose_w)[0].tolist(), 'names': robot.body_names}
        points = world_points(self._local, {
            **{'dispenser:'+n: v['pose'] for n, v in result['bodies'].items() if 'dispenser:'+n in self._local},
            **{k+':Body': result[k]['pose'] for k in ('used', 'replacement')},
        })
        result['bounds'] = {k: [v.min(axis=0).tolist(), v.max(axis=0).tolist()] for k, v in points.items()}
        result['robot_clearance_m'] = robot_clearance(points, result['hands'])
        return result

    def _task_step(self):
        if self._audit_error is not None:
            raise RuntimeError('Native physics audit failed') from self._audit_error
        if self.scorer.physics_samples != round(self._t*240):
            raise RuntimeError('Incomplete per-physics-step audit')
        self.metrics = self.scorer.evaluate(self.state())
        return StepResult(observation=self._observation(), terminated=self.metrics['success'],
            termination_reason='success' if self.metrics['success'] else None, info=self.metrics.copy())

    def diagnostics(self):
        return self.state()

    def close(self):
        self._audit_active = False
        if self._callback is not None:
            self._callback.deregister()
            self._callback = None
        if self._env is not None:
            self._env.close()
            self._env = self._base = None
        if self._temp is not None:
            self._temp.cleanup()


def sealant_reload_embodiment(**kwargs):
    return SealantReloadEnvironment(**kwargs)
