"""Native wheel assembly bridge for YAM. No proxy rim or commanded nipple poses.

The authored assembly contains independently simulated bodies. Reset writes are
confined to the reset event; the material callback only applies measured-angle
thread shortening and reaction torques. The central-distance-spring candidate
uses ordinary proportional external forces; the older once-per-step reference
configuration is retained only for comparison and is not validated for truing.
"""
from dataclasses import dataclass, field
import importlib.abc
import importlib.util
import sys
import numpy as np
from yam_arena.task_file import Box, ObjectState


@dataclass(frozen=True)
class WheelAssembly(Box):
    @property
    def free(self):
        return False


@dataclass(frozen=True)
class WheelState(ObjectState):
    body_names: tuple = ()
    body_poses: np.ndarray = field(default_factory=lambda: np.empty((0, 7)))
    body_velocities: np.ndarray = field(default_factory=lambda: np.empty((0, 6)))


BODY_NAMES = tuple([f'wheel/rim_{i:02d}' for i in range(32)] + ['wheel/hub'] +
                   [f'wheel/spoke_{i:02d}_{half}' for i in range(32) for half in ('inner', 'outer')] +
                   [f'wheel/nipple_{i:02d}' for i in range(32)] + ['wrench'])
ASSEMBLIES = {}
# Explicit diagnostic overrides are set before environment construction only.
RESET_PROFILE_OVERRIDE = None
SOLVER_ITERATIONS_OVERRIDE = None
CLOCKING_RAD_OVERRIDE = 0.
FLAT_PHASE_RAD_OVERRIDE = 0.
CONTACT_AUDIT_DIAGNOSTIC = False
NO_REWIND_DIAGNOSTIC = False
FROZEN_MATERIAL_DIAGNOSTIC = False
PRELOADED_RESET_CANDIDATE = False
CENTRAL_SPRING_CANDIDATE = False
DISTANCE_TOLERANCE_COMPENSATION_M = 0.
BATCH_SDF_NOTICES = False
MATERIAL_STRIDE = 1
SOLVER_TYPE = 'TGS'
FUNCTIONAL_MODEL = False
FIXTURE_LAYOUT = 'reference'
RESET_DIAGNOSTICS = []


def _spawn(path, cfg, translation=None, orientation=None, **kwargs):
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import UsdGeom
    from wheel_scene import build_scene
    from thread_coupling import rotation
    stage = get_current_stage()
    name = cfg.assembly_name
    # Paths contain nested wheel/rim entries; strip the complete known suffix.
    root = str(path)[:-len(cfg.body_name)-1]
    if name not in ASSEMBLIES:
        metadata = build_scene(stage, root=root, reset_profile=RESET_PROFILE_OVERRIDE or 'two-defect-candidate',
            wheel_clocking_rad=CLOCKING_RAD_OVERRIDE,nipple_flat_phase_rad=FLAT_PHASE_RAD_OVERRIDE,
            central_spring_candidate=CENTRAL_SPRING_CANDIDATE,
            distance_spring_tolerance_m=DISTANCE_TOLERANCE_COMPENSATION_M,
            fixture_layout=FIXTURE_LAYOUT)
        if FUNCTIONAL_MODEL:
            from functional_wheel import configure
            configure(stage,metadata)
        if PRELOADED_RESET_CANDIDATE:
            from preloaded_reset import author_preloaded_configuration
            author_preloaded_configuration(stage,metadata)
        if SOLVER_ITERATIONS_OVERRIDE is not None:
            from pxr import PhysxSchema
            from thread_coupling import body_paths
            for body_path in body_paths(metadata):
                PhysxSchema.PhysxRigidBodyAPI.Get(stage,body_path).CreateSolverPositionIterationCountAttr(SOLVER_ITERATIONS_OVERRIDE)
        if CONTACT_AUDIT_DIAGNOSTIC:
            from pxr import PhysxSchema
            from thread_coupling import body_paths
            for body_path in body_paths(metadata):
                PhysxSchema.PhysxContactReportAPI.Apply(stage.GetPrimAtPath(body_path)).CreateThresholdAttr(0.)
        cache = UsdGeom.XformCache()
        poses = []
        for relative in BODY_NAMES:
            matrix = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(root+'/'+relative))
            transform = matrix.ExtractRotationQuat()
            v = transform.GetImaginary()
            poses.append([*map(float, matrix.ExtractTranslation()), *map(float, v), float(transform.GetReal())])
        ASSEMBLIES[name] = dict(root=root, metadata=metadata, reset_poses=np.asarray(poses), coupling=None)
    prim = stage.GetPrimAtPath(str(path))
    if not prim:
        raise RuntimeError('Native wheel body missing: '+str(path))
    return prim


def reset_assembly(env, env_ids, assembly_name):
    import torch
    from thread_coupling import NativeThreadCoupler
    base = env.unwrapped if hasattr(env, 'unwrapped') else env
    obj = base.scene[assembly_name]
    origins = getattr(base.scene.env_origins, 'torch', base.scene.env_origins)
    ids = torch.arange(len(origins), device=origins.device, dtype=torch.long) if env_ids is None else env_ids
    if len(origins) != 1:
        raise ValueError('The validated target is one deterministic wheel environment')
    item = ASSEMBLIES[assembly_name]
    RESET_DIAGNOSTICS.append({'assembly':assembly_name,'native_pose_rewind':not NO_REWIND_DIAGNOSTIC,
                              'material_installed':item['coupling'] is not None})
    if NO_REWIND_DIAGNOSTIC:
        # Isolate constructor warm-start state. This is not a task reset: all
        # native constructor evolution is retained and reported by the probe.
        return
    pose = torch.as_tensor(item['reset_poses'], device=origins.device, dtype=origins.dtype)[None].repeat(len(ids), 1, 1)
    pose[:, :, :3] += origins[ids, None, :]
    obj.write_body_link_pose_to_sim_index(body_poses=pose, env_ids=ids)
    obj.write_body_link_velocity_to_sim_index(body_velocities=torch.zeros((len(ids), len(BODY_NAMES), 6),
         device=origins.device, dtype=origins.dtype), env_ids=ids)
    if item['coupling'] is not None:
        from integrity import RimDomainGuard
        item['coupling'].integrity=RimDomainGuard()
        item['coupling'].previous[:] = 0.
        item['coupling'].unwrapped[:] = 0.
        item['coupling'].friction.rest[:] = 0.
        item['coupling'].friction.dissipated[:] = 0.
        item['coupling'].parameters.invalidate()
        item['coupling']._material_step=0
        item['coupling']._held_torques=None
        item['coupling'].last = []
        from mechanics import WheelSpec
        spec=item['coupling'].spec
        for spoke in item['metadata']['spokes']:
            spoke['rest_target_attribute'].Set(spoke['nominal_rest_m']-
                spoke['initial_turn_offset']*spec.pitch_m_per_turn-spoke['rest_target_origin_m']-
                spoke.get('native_rest_bias_m',0.))
            spoke['friction_drive'].GetTargetPositionAttr().Set(0.)
            spoke['friction_drive'].GetMaxForceAttr().Set(spec.friction_coefficient*
                WheelSpec().baseline_tension_n*spec.thread_mean_radius_m)


def asset(spec):
    from isaaclab.assets import RigidObjectCfg, RigidObjectCollectionCfg
    from isaaclab.managers import EventTermCfg
    from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.asset import Asset
    @configclass
    class SpawnCfg(SpawnerCfg):
        func: object = clone(_spawn)
        assembly_name: str = ''
        body_name: str = ''
    class AssemblyAsset(Asset):
        def __init__(self):
            super().__init__(spec.name, tags=['wheel', 'native', 'compliant-rim'])
            self.cfg = RigidObjectCollectionCfg(rigid_objects={relative.replace('/', '__'): RigidObjectCfg(
                prim_path='{ENV_REGEX_NS}/'+spec.name+'/'+relative,
                spawn=SpawnCfg(assembly_name=spec.name, body_name=relative)) for relative in BODY_NAMES})
        def get_object_cfg(self):
            return self.name, self.cfg
        def get_event_cfg(self):
            return self.name, EventTermCfg(func=reset_assembly, mode='reset', params={'assembly_name': self.name})
        def validate_simulation_cfg(self, sim_cfg):
            from isaaclab_physx.physics import PhysxCfg
            if sim_cfg.physics is not None and not isinstance(sim_cfg.physics, PhysxCfg):
                raise ValueError('Wheel material requires PhysX')
    return AssemblyAsset()


def state(env, name):
    base = env.unwrapped if hasattr(env, 'unwrapped') else env
    obj = base.scene[name]
    names = list(getattr(obj, 'body_names', getattr(obj, 'object_names', ())))
    indices = [names.index(relative.replace('/', '__')) for relative in BODY_NAMES]
    def read(*attributes):
        for attribute in attributes:
            if hasattr(obj.data, attribute):
                data = getattr(obj.data, attribute)
                return getattr(data, 'torch', data)[0, indices].detach().cpu().numpy().astype(float)
        raise AttributeError('Missing wheel telemetry '+repr(attributes))
    p = read('body_link_pos_w', 'object_link_pos_w')
    q = read('body_link_quat_w', 'object_link_quat_w')
    v = read('body_link_lin_vel_w', 'object_link_lin_vel_w')
    w = read('body_link_ang_vel_w', 'object_link_ang_vel_w')
    if not all(np.isfinite(x).all() for x in (p, q, v, w)):
        raise RuntimeError('Nonfinite wheel body state')
    low, high = p.min(0)-.03, p.max(0)+.03
    return WheelState(pos=(low+high)/2, quat_xyzw=q[32], lin_vel=v.mean(0), ang_vel=w.mean(0),
        low=low, high=high, body_names=BODY_NAMES, body_poses=np.column_stack((p,q)), body_velocities=np.column_stack((v,w)))


def install_material_hook(base, stage):
    from pxr import UsdUtils
    from thread_coupling import NativeThreadCoupler, TensorNativeIO, body_paths
    import omni.physics.tensors as tensors
    cache = UsdUtils.StageCache.Get()
    stage_id = cache.GetId(stage).ToLongInt()
    if stage_id < 0:
        cache.Insert(stage)
        stage_id = cache.GetId(stage).ToLongInt()
    tensor_view=tensors.create_simulation_view('torch',stage_id)
    tensor_view.set_subspace_roots('/')
    for item in ASSEMBLIES.values():
        item['tensor_view']=tensor_view
        io=TensorNativeIO(tensor_view,body_paths(item['metadata']))
        item['coupling'] = NativeThreadCoupler(stage_id, item['metadata'],native_io=io,
            batch_sdf_notices=BATCH_SDF_NOTICES,material_stride=MATERIAL_STRIDE)
    original = base.scene.write_data_to_sim
    def before_substep(*args, **kwargs):
        result = original(*args, **kwargs)
        for item in ASSEMBLIES.values():
            item['coupling'].before_step(frozen_material_diagnostic=FROZEN_MATERIAL_DIAGNOSTIC)
        return result
    base.scene.write_data_to_sim = before_substep
    return original


def _patch(scene):
    if getattr(scene, '_native_wheel_installed', False):
        return
    original_object, original_states = scene.task_object, scene.object_states
    original_configuration = scene.env_cfg_callback
    def configure_callback(*args, **kwargs):
        configure = original_configuration(*args, **kwargs)
        def configure_wheel(cfg):
            cfg = configure(cfg)
            from isaaclab_physx.physics import PhysxCfg
            if cfg.sim.physics is None:
                cfg.sim.physics = PhysxCfg()
            if not isinstance(cfg.sim.physics, PhysxCfg):
                raise ValueError('Wheel assembly requires native PhysX')
            # The059 isolated constitutive test passes only with ordinary
            # proportional forces and compensated central DistanceJoint. The
            # older D6 reference retains its historical comparison setting.
            cfg.sim.physics.solver_type = 0 if SOLVER_TYPE=='PGS' else 1
            cfg.sim.physics.enable_external_forces_every_iteration = CENTRAL_SPRING_CANDIDATE and SOLVER_TYPE=='TGS'
            return cfg
        return configure_wheel
    scene.env_cfg_callback = configure_callback
    def task_object(spec):
        return asset(spec) if isinstance(spec, WheelAssembly) else original_object(spec)
    def object_states(env, names):
        normal = [name for name in names if name not in ASSEMBLIES]
        result = original_states(env, normal) if normal else {}
        result.update({name: state(env, name) for name in names if name in ASSEMBLIES})
        return {name: result[name] for name in names}
    scene.task_object, scene.object_states = task_object, object_states
    scene._native_wheel_installed = True


class Finder(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname != 'yam_arena.scene':
            return None
        sys.meta_path.remove(self)
        try:
            spec = importlib.util.find_spec(fullname)
        finally:
            sys.meta_path.insert(0, self)
        original = spec.loader.exec_module
        def execute(module):
            original(module)
            _patch(module)
        spec.loader.exec_module = execute
        return spec


if 'yam_arena.scene' in sys.modules:
    _patch(sys.modules['yam_arena.scene'])
elif not any(isinstance(item, Finder) for item in sys.meta_path):
    sys.meta_path.insert(0, Finder())
