"""Check that the robots have released the clamping parts."""
import numpy as np

_base=None
_view=None
_touching=False
_error=None


def reset():
    global _base,_view,_touching,_error
    _base=None;_view=None;_touching=False;_error=None


def array(value):
    if hasattr(value,'detach'):return value.detach().cpu().numpy()
    return np.asarray(value.numpy() if hasattr(value,'numpy') else value)


def poll(manager):
    global _touching,_error
    if _base is None or _view is None:return
    try:
        _,_,_,separation,counts,starts=_view.get_contact_data(dt=manager.get_physics_dt())
        separation=array(separation).reshape(-1)
        counts=array(counts).reshape(-1);starts=array(starts).reshape(-1)
        _touching=any(count and np.min(separation[int(start):int(start+count)])<=.001 for count,start in zip(counts,starts))
    except Exception as error:
        _error=repr(error)


def bind(base):
    global _base,_view
    if _base is base and _view is not None:return
    import carb
    from isaaclab.sim.utils.stage import get_current_stage
    from isaaclab_physx.physics.physx_manager import PhysxManager
    from pxr import UsdPhysics
    carb.settings.get_settings().set_bool('/physics/disableContactProcessing',False)
    sensors=[];targets=[]
    for prim in get_current_stage().Traverse():
        path=prim.GetPath().pathString
        if '/env_0/' not in path:continue
        if any(name in path for name in ('/LeftArm/','/RightArm/')) and 'PhysxContactReportAPI' in prim.GetAppliedSchemas():sensors.append(path)
        elif prim.HasAPI(UsdPhysics.RigidBodyAPI):
            if 'blank' in path.split('/') or ('/cam_fixture/' in path and path.rsplit('/',1)[-1] in ('cam','compliant_stop')):targets.append(path)
    if not sensors or len(targets)!=3:raise RuntimeError(f'Clamp release monitor found {len(sensors)} robot bodies and {len(targets)} clamp bodies')
    _view=PhysxManager.get_physics_sim_view().create_rigid_contact_view(sensors,filter_patterns=[targets]*len(sensors),max_contact_data_count=128*len(sensors)*len(targets))
    _base=base
    if not getattr(PhysxManager.step,'_clamp_release_hook',False):
        original=PhysxManager.step.__func__
        def step(cls):
            original(cls);poll(cls)
        step._clamp_release_hook=True
        PhysxManager.step=classmethod(step)
    print(f'Clamp release monitor ready with {len(sensors)} robot bodies',flush=True)


def clear():
    if _error:raise RuntimeError('Clamp release monitor failed: '+_error)
    return not _touching
