"""Surface-paper bridge for the pinned YAM/Arena loader.

Pattern follows the team's accepted rope task: lazy Box subclass dispatch.
The actual body is a native DeformableObject, never a rigid paper proxy. Nodal
state is exposed only to the assessor, not to any demonstration policy.
"""
from dataclasses import dataclass,field
import importlib.abc
import importlib.util
import sys
import numpy as np
from yam_arena.task_file import Box,ObjectState
from paper import PaperMaterial


@dataclass(frozen=True)
class Paper(Box):
    mesh: object = None
    initial: object = None
    material: PaperMaterial = PaperMaterial()
    vertex_colors: object = None

    @property
    def free(self):
        return False  # suppress the adapter's rigid-root jitter, not dynamics

    def _check(self):
        super()._check()
        self.mesh.audit()
        if np.asarray(self.initial).shape != self.mesh.rest.shape:
            raise ValueError('incomplete initial paper pose')
        if self.vertex_colors is not None and np.asarray(self.vertex_colors).shape!=self.mesh.rest.shape:
            raise ValueError('incomplete paper colour map')


@dataclass(frozen=True)
class PaperState(ObjectState):
    nodes: np.ndarray = field(default_factory=lambda:np.empty((0,3)))
    velocities: np.ndarray = field(default_factory=lambda:np.empty((0,3)))


SPECS={}


def _spawn(path,cfg,translation=None,orientation=None,**kwargs):
    from isaaclab.sim.utils.stage import get_current_stage
    from native_paper import author_paper
    from pxr import UsdPhysics,PhysxSchema
    spec=SPECS[cfg.paper_name]
    stage=get_current_stage()
    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.Scene):
            api=PhysxSchema.PhysxSceneAPI.Apply(prim);api.CreateEnableGPUDynamicsAttr(True);api.CreateBroadphaseTypeAttr("GPU");api.CreateEnableCCDAttr(False)
    # The only supported starting orientation is identity; paper points are
    # explicit. Clone offsets are applied by the normal scene cloner.
    if orientation is not None and not np.allclose(orientation,(0.,0.,0.,1.)):
        raise ValueError('author the initial paper orientation in its vertices')
    points=np.asarray(spec.initial)+np.asarray(translation or (0.,0.,0.))
    center=points[0]-spec.mesh.rest[0]
    if not np.allclose(points-spec.mesh.rest,center):raise ValueError("Initial paper must be an unstrained translated sheet")
    prim=author_paper(stage,str(path),spec.mesh,center,material=spec.material).GetPrim()
    import runtime_config
    controls={}
    if runtime_config.REFINED_CONTACT:
        controls.update({'physxDeformableBody:enableSpeculativeCCD':True,
            'physxDeformableBody:collisionIterationMultiplier':4,
            'physxDeformableBody:collisionPairUpdateFrequency':4})
    if runtime_config.MAX_DEPENETRATION_M_S is not None:
        controls['physxDeformableBody:maxDepenetrationVelocity']=runtime_config.MAX_DEPENETRATION_M_S
    for name,value in controls.items():
        attr=prim.GetAttribute(name)
        if not attr or not attr.Set(value):raise RuntimeError('Missing native paper contact control: '+name)
    return prim


def reset_paper(env,env_ids,paper_name):
    """Reset-only initial state write. No control callback calls this function."""
    import torch
    base=env.unwrapped if hasattr(env,'unwrapped') else env
    obj=base.scene[paper_name]
    origins=getattr(base.scene.env_origins,'torch',base.scene.env_origins)
    ids=torch.arange(len(origins),device=origins.device) if env_ids is None else env_ids
    points=torch.as_tensor(SPECS[paper_name].initial,device=origins.device,dtype=origins.dtype)
    state=torch.zeros((len(ids),len(points),6),device=origins.device,dtype=origins.dtype)
    state[:,:,:3]=points[None,:,:]+origins[ids,None,:]
    obj.write_nodal_state_to_sim_index(state,env_ids=ids)
    obj.reset(env_ids=ids)


def asset(spec):
    from isaaclab.assets import DeformableObjectCfg
    from isaaclab.managers import EventTermCfg
    from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.asset import Asset
    @configclass
    class SpawnCfg(SpawnerCfg):
        func: object=clone(_spawn)
        paper_name: str=''
    class SurfaceAsset(Asset):
        def __init__(self):
            super().__init__(spec.name,tags=['surface-deformable','paper','physx'])
        def get_object_cfg(self):
            return self.name,DeformableObjectCfg(prim_path='{ENV_REGEX_NS}/'+spec.name,
                  spawn=SpawnCfg(paper_name=spec.name))
        def get_event_cfg(self):
            return self.name,EventTermCfg(func=reset_paper,mode='reset',params={'paper_name':self.name})
        def validate_simulation_cfg(self,sim_cfg):
            from isaaclab_physx.physics import PhysxCfg
            if sim_cfg.physics is not None and not isinstance(sim_cfg.physics,PhysxCfg):
                raise ValueError('This scene requires native PhysX surface FEM')
    return SurfaceAsset()


def state(env,name):
    base=env.unwrapped if hasattr(env,'unwrapped') else env
    obj=base.scene[name]
    def arr(value):
        return getattr(value,'torch',value).detach().cpu().numpy()
    n=len(SPECS[name].mesh.rest)
    points=arr(obj.data.nodal_pos_w)[0,:n].copy()-arr(base.scene.env_origins)[0]
    velocity=arr(obj.data.nodal_vel_w)[0,:n].copy()
    if points.shape != (n,3) or not np.isfinite(points).all() or not np.isfinite(velocity).all():
        raise ValueError('invalid paper telemetry')
    low,high=points.min(0),points.max(0)
    return PaperState(pos=(low+high)/2,quat_xyzw=np.array((0.,0.,0.,1.)),
                      lin_vel=velocity.mean(0),ang_vel=np.zeros(3),low=low,high=high,
                      nodes=points,velocities=velocity)


def _patch(scene):
    if getattr(scene,'_native_surface_paper_installed',False):
        return
    original_object,original_states=scene.task_object,scene.object_states
    def task_object(spec):
        from arena_hardware import StapleHardware,hardware_asset
        if isinstance(spec,StapleHardware):
            return hardware_asset(spec)
        if isinstance(spec,Paper):
            spec._check()
            SPECS[spec.name]=spec
            return asset(spec)
        return original_object(spec)
    def object_states(env,names):
        from arena_hardware import ASSEMBLIES,hardware_state
        ordinary=[n for n in names if n not in SPECS and n not in ASSEMBLIES]
        result=original_states(env,ordinary) if ordinary else {}
        result.update({n:state(env,n) for n in names if n in SPECS})
        result.update({n:hardware_state(env,n) for n in names if n in ASSEMBLIES})
        return {n:result[n] for n in names}
    scene.task_object,scene.object_states=task_object,object_states
    scene._native_surface_paper_installed=True


class Finder(importlib.abc.MetaPathFinder):
    def find_spec(self,fullname,path=None,target=None):
        if fullname!='yam_arena.scene':
            return None
        sys.meta_path.remove(self)
        try:
            spec=importlib.util.find_spec(fullname)
        finally:
            sys.meta_path.insert(0,self)
        original=spec.loader.exec_module
        def execute(module):
            original(module)
            _patch(module)
        spec.loader.exec_module=execute
        return spec


if 'yam_arena.scene' in sys.modules:
    _patch(sys.modules['yam_arena.scene'])
elif not any(isinstance(x,Finder) for x in sys.meta_path):
    sys.meta_path.insert(0,Finder())
