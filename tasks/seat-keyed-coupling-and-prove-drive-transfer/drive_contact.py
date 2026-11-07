"""Measure direct robot contact with the driven rotor during the drive test."""
import math
import numpy as np

_base = None
_view = None
_failed = False
_error = None
_polls = 0


def reset():
    global _base, _view, _failed, _error, _polls
    _base = None
    _view = None
    _failed = False
    _error = None
    _polls = 0


def array(value):
    if hasattr(value, 'detach'):
        return value.detach().cpu().numpy()
    return np.asarray(value.numpy() if hasattr(value, 'numpy') else value)


def poll(manager):
    global _failed, _error, _polls
    if _base is None or _view is None:
        return
    try:
        rotor = _base.scene['hub_fixture']
        index = rotor.find_joints('rotor_angle')[0][0]
        angle = float(rotor.data.joint_pos.torch[0, index].item())
        _, _, _, separation, counts, starts = _view.get_contact_data(dt=manager.get_physics_dt())
        _polls += 1
        if abs(angle) < math.radians(5):
            return
        counts = array(counts).reshape(-1)
        starts = array(starts).reshape(-1)
        separation = array(separation).reshape(-1)
        for count, start in zip(counts, starts):
            if count and np.min(separation[int(start):int(start + count)]) <= .001:
                _failed = True
    except Exception as error:
        _error = repr(error)
        _failed = True


def bind(base):
    global _base, _view
    if _base is base and _view is not None:
        return
    import carb
    from isaaclab.sim.utils.stage import get_current_stage
    from isaaclab_physx.physics.physx_manager import PhysxManager
    from pxr import UsdPhysics

    carb.settings.get_settings().set_bool('/physics/disableContactProcessing', False)
    sensors, targets = [], []
    for prim in get_current_stage().Traverse():
        path = prim.GetPath().pathString
        if '/env_0/' not in path:
            continue
        if any(name in path for name in ('/LeftArm/', '/RightArm/')) and 'PhysxContactReportAPI' in prim.GetAppliedSchemas():
            sensors.append(path)
        elif '/hub_fixture/' in path and path.endswith('/rotor') and prim.HasAPI(UsdPhysics.RigidBodyAPI):
            targets.append(path)
    if not sensors or len(targets) != 1:
        raise RuntimeError(f'Drive contact monitor found {len(sensors)} robot bodies and {len(targets)} rotors')
    _view = PhysxManager.get_physics_sim_view().create_rigid_contact_view(
        sensors, filter_patterns=[targets] * len(sensors), max_contact_data_count=128 * len(sensors))
    _base = base
    if not getattr(PhysxManager.step, '_drive_contact_hook', False):
        original = PhysxManager.step.__func__

        def step(cls):
            original(cls)
            poll(cls)

        step._drive_contact_hook = True
        PhysxManager.step = classmethod(step)
    print(f'Drive contact monitor ready with {len(sensors)} robot bodies', flush=True)


def failed():
    if _error:
        raise RuntimeError('Drive contact monitor failed: ' + _error)
    return _failed
