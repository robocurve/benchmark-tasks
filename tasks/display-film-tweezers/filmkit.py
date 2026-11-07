"""Task-local YAM integration. Imports Isaac only after the simulator starts.

Cohesive hinge model: each bonded strip resists opening with an angular
spring. Only the current peel front releases, after opening upward by 60 degrees.
Compression and in-plane shear cannot break the attachment. Release is irreversible
within a trial; reset restores every attachment. Updated at the 20 Hz control rate.
"""
import importlib.abc
import importlib.machinery
import sys
from functools import wraps
import numpy as np
from geometry import (FILM_NAMES, N, N_BONDED, BONDED_FROM, CARRIER_POS, SIZES, STRIP_SIZE, TAB_SEAM,
                      TAB_QUAT, TAB_SIZE, rotation, strip_pos)
from scoring import Trial

trial = Trial()
_paths = {}
_bonds = []
_installed = False
_task_path = None
_assembling = False
OPEN_ANGLE = np.deg2rad(60)
# N m/rad per strip; provisional, requires force-displacement calibration. With 5 mm strips,
# 0.015 (0.03 at 10 mm, scaled with length) kept the release force but halved the sideways peel
# load, so an unheld carrier slid only 10 mm and a one-arm peel scored 1.0. At 0.025 the unheld
# carrier is dragged 295 mm, while the held demo carrier moves 0.1 mm at a 3.0 N peak tab load.
STIFFNESS = .025


def _joint(stage, name, body0, body1, p0, p1, *, stiffness=0., target=0.,
           damping=.0001, max_torque=None):
    from pxr import Gf, UsdPhysics
    joint = UsdPhysics.RevoluteJoint.Define(stage, f'{body1}/{name}')
    joint.CreateBody0Rel().SetTargets([body0])
    joint.CreateBody1Rel().SetTargets([body1])
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*map(float, p0)))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(*map(float, p1)))
    joint.CreateAxisAttr().Set('X')
    joint.CreateLowerLimitAttr().Set(-175.)
    joint.CreateUpperLimitAttr().Set(175.)
    joint.CreateCollisionEnabledAttr().Set(False)
    if stiffness or damping:
        drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), 'angular')
        drive.CreateTypeAttr().Set('force')
        drive.CreateTargetPositionAttr().Set(float(target))
        # USD angular drive stiffness is torque per degree.
        drive.CreateStiffnessAttr().Set(float(stiffness * np.pi / 180))
        drive.CreateDampingAttr().Set(float(damping * np.pi / 180))
        if max_torque is not None:
            drive.CreateMaxForceAttr().Set(float(max_torque))
    return joint


def _patch_scene(module):
    original = module.task_object
    original_build = module.build_task_environment

    @wraps(original_build)
    def build(embodiment, task, control_hz=None):
        global _assembling
        _assembling = task.path == _task_path
        if _assembling:
            _paths.clear()
            _bonds.clear()
        try:
            return original_build(embodiment, task, control_hz)
        finally:
            _assembling = False

    module.build_task_environment = build

    @wraps(original)
    def task_object(spec):
        if not _assembling or spec.name not in SIZES:
            return original(spec)
        from isaaclab.sim.spawners.shapes import spawn_cuboid
        from isaaclab.sim.utils import clone
        from isaaclab_arena.utils.pose import Pose

        obj = original(spec)
        cfg = obj.spawner_cfg
        cfg.rigid_props.solver_position_iteration_count = 128
        cfg.rigid_props.solver_velocity_iteration_count = 16
        cfg.collision_props.contact_offset = .0002
        cfg.collision_props.rest_offset = 0.
        if spec.name == 'carrier':
            cfg.physics_material.static_friction = .4
            cfg.physics_material.dynamic_friction = .4
            cfg.physics_material.friction_combine_mode = 'min'
        pose = obj.initial_pose
        if spec.name == 'pull_tab':
            pose = Pose(position_xyz=tuple(spec.pos), rotation_xyzw=tuple(map(float, TAB_QUAT)))
            cfg.activate_contact_sensors = True

        @clone
        def spawn(path, config, *args, **kwargs):
            prim = spawn_cuboid.__wrapped__(path, config, *args, **kwargs)
            stage = prim.GetStage()
            _paths[spec.name] = str(path)
            if spec.name in ('carrier','pull_tab'):
                from fixture_geometry import carrier_parts,tab_parts
                from fixturekit import replace_mesh
                replace_mesh(prim,config,carrier_parts() if spec.name=='carrier' else tab_parts())
            if spec.name.startswith('film_'):
                i = int(spec.name.split('_')[1])
                prev = 'pull_tab' if i == 0 else f'film_{i-1:02d}'
                prev_anchor = TAB_SEAM if i == 0 else (0.,-STRIP_SIZE[1]/2,0.)
                _joint(stage, 'hinge', _paths[prev], str(path),
                       prev_anchor, (0., STRIP_SIZE[1]/2, 0.),
                       stiffness=.01 if i == 0 else 0.,
                       target=0.,
                       damping=.0001 if i == 0 else .00002,
                       max_torque=.0002 if i == 0 else None)
                if i < BONDED_FROM:
                    return prim  # starter zone: hinged to its neighbours, no adhesive
                anchor = strip_pos(i) - CARRIER_POS + np.array([0., -STRIP_SIZE[1]/2, 0.])
                bond = _joint(stage, 'adhesion', _paths['carrier'], str(path),
                              anchor, (0., -STRIP_SIZE[1]/2, 0.), stiffness=STIFFNESS)
                _bonds.append(bond)
            return prim

        cfg.func = spawn
        # Object caches both its spawn config and reset event in __init__.
        # Rebuild after customization so all changes apply on spawn AND reset.
        return module.Object(name=spec.name, object_type=obj.object_type,
                             spawner_cfg=cfg, initial_pose=pose)

    module.task_object = task_object
    from toolkit import patch
    patch(module)


class _SceneFinder(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path, target=None):
        if fullname != 'yam_arena.scene':
            return None
        spec = importlib.machinery.PathFinder.find_spec(fullname, path)
        if spec is None or spec.loader is None:
            return None
        loader = spec.loader

        class Loader(importlib.abc.Loader):
            def create_module(self, module_spec):
                return loader.create_module(module_spec)

            def exec_module(self, module):
                loader.exec_module(module)
                _patch_scene(module)

        spec.loader = Loader()
        return spec


def advance(objects, hands_clear):
    """Advance a single cohesive front; geometry is measured in the carrier frame."""
    trial.observe(objects, hands_clear=hands_clear)
    if trial.peeled >= N_BONDED:
        return
    # A scoring failure must not change subsequent material mechanics. Continue
    # physical peeling after carrier excursions; only malformed state prevents an update.
    try:
        plate = objects['carrier']
        s = objects[FILM_NAMES[trial.peeled + BONDED_FROM + 1]]
        r = rotation(plate.quat_xyzw).T @ rotation(s.quat_xyzw)
    except (KeyError, ValueError):
        return
    angle = np.arctan2(r[2, 1], r[1, 1])
    if angle >= OPEN_ANGLE:
        # No force-magnitude break threshold: pushing down cannot release a strip.
        _bonds[trial.peeled].GetJointEnabledAttr().Set(False)
        trial.peeled += 1


def validate_integrity(objects):
    """Reject numerical separation of an unbreakable film seam (>3 mm).

    This is a failure gate, not an alternate mechanism for releasing adhesive bonds.
    """
    for i in range(N):
        prev, cur = objects[FILM_NAMES[i]], objects[FILM_NAMES[i+1]]
        anchor = TAB_SEAM if i == 0 else np.array([0.,-STRIP_SIZE[1]/2,0.])
        a = prev.pos + rotation(prev.quat_xyzw) @ anchor
        b = cur.pos + rotation(cur.quat_xyzw) @ np.array([0., STRIP_SIZE[1]/2, 0.])
        if np.linalg.norm(a-b) > .003:
            trial.invalid = True


def _hands_clear(emb, objects):
    """Both TCPs have withdrawn at least 80 mm from every task object.

    Uses measured joints, never policy commands. The conservative distance includes
    the pads extending beyond the TCP; requires the film to rest flat separately.
    """
    from yam_arena.ik import tcp_pose
    for name, base in (('left_robot', emb.rig.bases['left']), ('right_robot', emb.rig.bases['right'])):
        arm = emb._base.scene[name]
        ids, _ = emb._ids[name]
        q = arm.data.joint_pos.torch[0, ids].detach().cpu().numpy()
        if not np.all(np.isfinite(q)):
            return False
        tcp, _ = tcp_pose(q, np.asarray(base))
        for piece in objects:
            low, high = objects[piece].low, objects[piece].high
            if np.linalg.norm(tcp - np.clip(tcp, low, high)) < .08:
                return False
    return True


def install(task_path):
    global _installed, _task_path
    _task_path = task_path
    if _installed:
        return
    _installed = True
    if 'yam_arena.scene' in sys.modules:
        _patch_scene(sys.modules['yam_arena.scene'])
    else:
        sys.meta_path.insert(0, _SceneFinder())
    from yam_arena.inspect_embodiment import YamArenaEmbodiment
    original_reset = YamArenaEmbodiment.reset
    original_step = YamArenaEmbodiment._task_step

    def ours(emb):
        return emb.task is not None and emb.task.path == _task_path

    @wraps(original_reset)
    def reset(emb, scene, *, seed=None):
        if ours(emb):
            if emb.object_jitter != 0:
                raise ValueError('Connected film requires -E object_jitter=0 for every policy.')
            emb._ensure_sim(None)
            if len(_bonds) != N_BONDED:
                raise RuntimeError(f'Expected {N_BONDED} adhesive bonds, created {len(_bonds)}')
            trial.reset()
            for bond in _bonds:
                bond.GetJointEnabledAttr().Set(True)
        return original_reset(emb, scene, seed=seed)

    @wraps(original_step)
    def step(emb):
        if ours(emb):
            from yam_arena.scene import object_states
            objects = object_states(emb._env, emb.task.names)
            trial.observe(objects, hands_clear=False)
            if not trial.invalid:
                validate_integrity(objects)
            advance(objects, _hands_clear(emb, objects))
            trial.finish_step(objects, emb._base.step_dt)
        result = original_step(emb)
        if ours(emb):
            result.info.update(peeled_strips=trial.peeled, invalid_trial=trial.invalid)
        return result

    YamArenaEmbodiment.reset = reset
    YamArenaEmbodiment._task_step = step
