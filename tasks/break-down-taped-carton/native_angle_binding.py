"""Pinned Kit PhysX read-only bottom-angle observer. No physics state writes."""
import hashlib
import inspect
from pathlib import Path
import numpy as np
from angle_tracking import ContinuousBottomAngles,BODIES,PHYSICS_DT,SUBSTEPS
from angle_step_observer import AngleStepObserver
from capture import array

PINS={
 'simulation':'c3c17f084bc21ddfd80649f8179a6aa916d857a1dcaa1cc1c7f2b7c3945936b2',
 'arena':'27f9b4986aa7500bf3a2a4a719a5e1324300ad7da6576ad075971b35b14127de',
 'rl_environment':'15341168a31adc2fa10785ebde13e7e2a8b5a6cef5be113f0c4c88d9a5895562',
 'environment':'f914dd1306033bc2f2cb1482f96d67080db6b725a4041437cfd349f9d964cc10',
 'physics':'a20f98c841ea031329d0d083303e1fb4c84cb13a7ba3f4cc6bbeee028302bd35',
 'rigid_object':'0fd17152d133805412776d8cc90eebe85159a383a2e052dafc2eafdd14e54390',
}


def verify_source(obj,key):
    path=Path(inspect.getsourcefile(obj))
    if hashlib.sha256(path.read_bytes()).hexdigest()!=PINS[key]:
        raise RuntimeError('unverified native angle binding source: '+key)


def install(base):
    from pxr import PhysxSchema,Usd
    from isaaclab.envs import ManagerBasedEnv,ManagerBasedRLEnv
    verify_source(type(base),'arena')
    verify_source(ManagerBasedEnv,'environment')
    verify_source(ManagerBasedRLEnv,'rl_environment')
    verify_source(base.sim.physics_manager.step,'physics')
    verify_source(base.sim.step,'simulation')
    verify_source(type(base.scene['north']),'rigid_object')
    if base._physics_handles_decimation or base.cfg.decimation!=SUBSTEPS or base.physics_dt!=PHYSICS_DT:
        raise RuntimeError('unverified native substep configuration')
    if base._sim_step_counter!=0:raise RuntimeError('native angle install requires untouched reset')
    stage=base.scene.stage;views={};caps={}
    # Read schema fallback on a private stage; never author limits on live bodies.
    scratch=Usd.Stage.CreateInMemory();prim=scratch.DefinePrim('/Fallback','Xform')
    fallback=float(PhysxSchema.PhysxRigidBodyAPI.Apply(prim).GetMaxAngularVelocityAttr().Get())
    for n in BODIES:
        view=base.scene[n].root_view
        if view.count!=1 or len(view.prim_paths)!=1:raise RuntimeError('ambiguous native body: '+n)
        path=view.prim_paths[0]
        if path!='/World/envs/env_0/'+n:raise RuntimeError('unexpected native body path: '+path)
        attr=PhysxSchema.PhysxRigidBodyAPI(stage.GetPrimAtPath(path)).GetMaxAngularVelocityAttr()
        value=attr.Get() if attr else None
        caps[n]=fallback if value is None else float(value);views[n]=view
    def read():
        poses={};velocities={}
        for n,v in views.items():
            p=np.asarray(array(v.get_transforms()),dtype=float).copy()
            w=np.asarray(array(v.get_velocities()),dtype=float).copy()
            if p.shape!=(1,7) or w.shape!=(1,6):raise RuntimeError('native raw-view shape changed')
            poses[n]=p[0].tolist();velocities[n]=w[0].tolist()
        return poses,velocities
    tracker=ContinuousBottomAngles(caps)
    native_origin=int(base.sim.get_physics_step_count())
    def counter():
        native=int(base.sim.get_physics_step_count())-native_origin
        if native!=int(base._sim_step_counter):raise RuntimeError('environment/native physics counters disagree')
        return native
    observer=AngleStepObserver(tracker,base.sim.step,read,
        counter,lambda:float(counter())*PHYSICS_DT)
    if not observer.initialize():raise RuntimeError(tracker.failure)
    observer.initial_state['native_counter_origin']=native_origin
    base.sim.step=observer.step
    base._carton_angle_observer=observer
    return observer
