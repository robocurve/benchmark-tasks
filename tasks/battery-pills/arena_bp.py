"""Bridge between task.py and the pinned YAM/Arena loader (yam-isaac-lab-arena 0059d02a), on the pattern of our
ring-on-finger build (arena_ring.py): one ``Box`` subclass in task.OBJECTS for every body, and an import hook wrapping
``yam_arena.scene.task_object`` / ``object_states`` so that each body is one dynamic rigid body authored by
scene.author_body (convex pieces). The trial's draw is applied by authoring the bodies at their drawn start poses
(ROBOCURVE_BP_SEED, set by run.py before the embodiment loads task.py). Nothing here writes a pose after the scene is
built, and nothing here reaches a policy: the scorer and diagnostics read it.
"""
import importlib.abc
import importlib.util
import os
import sys
from dataclasses import dataclass, field

import numpy as np
from yam_arena.task_file import Box, ObjectState

import geometry as G

SPECS = {}
TRIAL = dict(seed=int(os.environ.get('ROBOCURVE_BP_SEED', '0')), tracker=None, audit={})


@dataclass(frozen=True)
class BPBody(Box):
    @property
    def free(self):
        return False   # no yam jitter: the draw is authored


@dataclass(frozen=True)
class BodyState(ObjectState):
    rot: np.ndarray = field(default_factory=lambda: np.eye(3))


def _arr(value):
    return getattr(value, 'torch', value).detach().cpu().numpy()


def _base(env):
    return env.unwrapped if hasattr(env, 'unwrapped') else env


def body_asset(spec):
    from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    def spawn(prim_path, cfg, translation=None, orientation=None, **kwargs):
        from isaaclab.sim.utils.stage import get_current_stage
        from scene import author_body
        stage = get_current_stage()
        if orientation is not None and not (np.allclose(orientation, (0, 0, 0, 1)) or np.allclose(orientation, (1, 0, 0, 0))):
            raise ValueError('battery-pills bodies start unrotated')
        t = tuple(spec.pos) if translation is None else tuple(float(v) for v in translation)
        TRIAL['audit'].setdefault('spawn_translation', {})[spec.name] = list(t)
        return author_body(stage, str(prim_path), t, spec.name)

    @configclass
    class SpawnCfg(SpawnerCfg):
        func: object = clone(spawn)

    return Object(name=spec.name, object_type=ObjectType.RIGID, spawner_cfg=SpawnCfg(),
                  initial_pose=Pose(position_xyz=tuple(float(v) for v in spec.pos), rotation_xyzw=(0., 0., 0., 1.)))


# -- state readers (scorer and diagnostics only) ------------------------------------------------------------
def body_pose(env, name):
    """(position (3,), rotation (3, 3)) of a rigid body's frame origin, world minus the env origin."""
    base = _base(env)
    obj = base.scene[name]
    origin = _arr(base.scene.env_origins)[0]
    pose = _arr(obj.data.root_link_pose_w)[0].astype(float)
    return pose[:3] - origin, G.quat_xyzw_to_matrix(pose[3:7])


def body_velocity(env, name):
    base = _base(env)
    obj = base.scene[name]
    v = _arr(obj.data.root_link_vel_w)[0].astype(float) if hasattr(obj.data, 'root_link_vel_w') else np.zeros(6)
    return v[:3], v[3:6]


def _rigid_state(env, name):
    p, r = body_pose(env, name)
    v, w = body_velocity(env, name)
    qw, qx, qy, qz = G.matrix_to_quat_wxyz(r)
    return BodyState(pos=p, quat_xyzw=np.array([qx, qy, qz, qw]), lin_vel=v, ang_vel=w, low=p - .01, high=p + .01, rot=r)


def gripper_hulls(env):
    """Collision hulls of both arms' gripper parts (gripper link and both fingers) at the simulator's link poses:
    {'left': [(link, Hull)], 'right': [...]}."""
    from yam_arena.analysis import world_hulls
    base = _base(env)
    origin = _arr(base.scene.env_origins)[0]
    out = {}
    for side, arm in (('left', 'left_robot'), ('right', 'right_robot')):
        robot = base.scene[arm]
        poses = _arr(robot.data.body_link_pose_w)[0]
        mats = {}
        for i, name in enumerate(robot.body_names):
            if name not in ('gripper', 'tip_left', 'tip_right'):
                continue
            t = np.eye(4)
            t[:3, :3] = G.quat_xyzw_to_matrix(poses[i, 3:7])
            t[:3, 3] = poses[i, :3] - origin
            mats[name] = t
        out[side] = world_hulls(mats)
    return out


# -- patches ------------------------------------------------------------------------------------------------
def _patch_scene(scene):
    if getattr(scene, '_bp_installed', False):
        return
    original_object, original_states = scene.task_object, scene.object_states

    def task_object(spec):
        if isinstance(spec, BPBody):
            spec._check()
            SPECS[spec.name] = spec
            return body_asset(spec)
        return original_object(spec)

    def object_states(env, names):
        ordinary = [n for n in names if n not in SPECS]
        result = original_states(env, ordinary) if ordinary else {}
        for n in names:
            if n in SPECS:
                result[n] = _rigid_state(env, n)
        return {n: result[n] for n in names}

    scene.task_object, scene.object_states = task_object, object_states
    scene._bp_installed = True


def _patch_embodiment(module):
    cls = module.YamArenaEmbodiment
    if getattr(cls, '_bp_installed', False):
        return
    original_reset = cls.reset

    def reset(self, scene, *, seed=None):
        obs = original_reset(self, scene, seed=seed)
        TRIAL['tracker'] = None
        return obs

    cls.reset = reset
    cls._bp_installed = True


class _Patcher(importlib.abc.MetaPathFinder):
    patches = {'yam_arena.scene': _patch_scene, 'yam_arena.inspect_embodiment': _patch_embodiment}

    def find_spec(self, fullname, path=None, target=None):
        if fullname not in self.patches:
            return None
        sys.meta_path.remove(self)
        try:
            spec = importlib.util.find_spec(fullname)
        finally:
            sys.meta_path.insert(0, self)
        if spec is None or spec.loader is None:
            return None
        original, patch = spec.loader.exec_module, self.patches[fullname]

        def execute(module):
            original(module)
            patch(module)

        spec.loader.exec_module = execute
        return spec


def install():
    for name, patch in _Patcher.patches.items():
        if name in sys.modules:
            patch(sys.modules[name])
    if not any(isinstance(f, _Patcher) for f in sys.meta_path):
        sys.meta_path.insert(0, _Patcher())


install()
