"""Native dynamic thread collections for the YAM scene, with reset-only writes."""
from dataclasses import dataclass,field
import numpy as np
from yam_arena.task_file import Box,ObjectState
from yam_arena import scene
from native_stitches import author_thread,x_quat


@dataclass(frozen=True)
class Thread(Box):
    nodes: object=None
    radius_m: float=.0002
    bending: object=None
    density_kg_m3: float=1400.
    @property
    def free(self):return False


@dataclass(frozen=True)
class ThreadState(ObjectState):
    centres: np.ndarray=field(default_factory=lambda:np.empty((0,3)))


SPECS={}
SPAWNED={}


def initial(spec):
    p=np.asarray(spec.nodes,float);d=np.roll(p,-1,axis=0)-p
    if spec.bending is not None:
        from thread_bending import planar_frames
        q=planar_frames(p)
    else:q=np.asarray([x_quat(v) for v in d])
    return np.column_stack(((p+np.roll(p,-1,axis=0))/2,q))


def spawn(path,cfg,**kwargs):
    from isaaclab.sim.utils.stage import get_current_stage
    stage=get_current_stage();root=str(path).rsplit('/',1)[0]
    spec=SPECS[cfg.thread_name]
    if not stage.GetPrimAtPath(str(path)):
        item=author_thread(stage,root,np.asarray(spec.nodes),radius=spec.radius_m,
                           density=spec.density_kg_m3,
                           joint_kind='d6' if spec.bending is not None else 'spherical',bending=spec.bending)
        SPAWNED[spec.name]=item
        from cpu_scene import configure_native
        configure_native(stage,item['bodies'])
    return stage.GetPrimAtPath(str(path))


def reset(env,env_ids,thread_name):
    import torch
    from native_stitches import restore_thread_joints
    import omni.usd
    base=env.unwrapped if hasattr(env,'unwrapped') else env
    origins=getattr(base.scene.env_origins,'torch',base.scene.env_origins)
    ids=torch.arange(len(origins),device=origins.device) if env_ids is None else env_ids
    pose=torch.as_tensor(initial(SPECS[thread_name]),device=origins.device,dtype=origins.dtype)
    pose=pose[None,:,:].repeat(len(ids),1,1);pose[:,:,:3]+=origins[ids,None,:]
    obj=base.scene[thread_name]
    obj.write_body_link_pose_to_sim_index(body_poses=pose,env_ids=ids)
    obj.write_body_link_velocity_to_sim_index(body_velocities=torch.zeros((len(ids),pose.shape[1],6),device=origins.device),env_ids=ids)
    restore_thread_joints(omni.usd.get_context().get_stage(),SPAWNED[thread_name])


def asset(spec):
    from isaaclab.assets import RigidObjectCfg,RigidObjectCollectionCfg
    from isaaclab.managers import EventTermCfg
    from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.asset import Asset
    @configclass
    class SpawnCfg(SpawnerCfg):
        func: object=clone(spawn)
        thread_name: str=''
    class ThreadAsset(Asset):
        def __init__(self):super().__init__(spec.name,tags=['native-thread','contact','physx'])
        def get_object_cfg(self):
            states=initial(spec)
            bodies={f'link_{i:03d}':RigidObjectCfg(prim_path='{ENV_REGEX_NS}/'+spec.name+f'/link_{i:03d}',
                    spawn=SpawnCfg(thread_name=spec.name),
                    init_state=RigidObjectCfg.InitialStateCfg(pos=tuple(s[:3]),rot=tuple(s[3:])))
                    for i,s in enumerate(states)}
            return self.name,RigidObjectCollectionCfg(rigid_objects=bodies)
        def get_event_cfg(self):return self.name,EventTermCfg(func=reset,mode='reset',params={'thread_name':self.name})
        def validate_simulation_cfg(self,sim_cfg):
            from isaaclab_physx.physics import PhysxCfg
            if sim_cfg.physics is not None and not isinstance(sim_cfg.physics,PhysxCfg):
                raise ValueError('This native thread collection requires PhysX')
    return ThreadAsset()


original_object,original_states=scene.task_object,scene.object_states


def task_object(spec):
    if not isinstance(spec,Thread):return original_object(spec)
    SPECS[spec.name]=spec
    return asset(spec)


def object_states(env,names):
    plain=[n for n in names if n not in SPECS]
    out=original_states(env,plain) if plain else {}
    base=env.unwrapped if hasattr(env,'unwrapped') else env
    def arr(v):return getattr(v,'torch',v).detach().cpu().numpy()
    def field(data,body_name,object_name):
        return getattr(data,body_name) if hasattr(data,body_name) else getattr(data,object_name)
    for n in names:
        if n not in SPECS:continue
        data=base.scene[n].data
        pts=arr(field(data,'body_link_pos_w','object_link_pos_w'))[0]-arr(base.scene.env_origins)[0]
        lo,hi=pts.min(0)-SPECS[n].radius_m,pts.max(0)+SPECS[n].radius_m
        out[n]=ThreadState(pos=(lo+hi)/2,quat_xyzw=np.array((0.,0.,0.,1.)),
                           lin_vel=arr(field(data,'body_link_lin_vel_w','object_link_lin_vel_w'))[0].mean(0),ang_vel=np.zeros(3),
                           low=lo,high=hi,centres=pts)
    return {n:out[n] for n in names}


scene.task_object,scene.object_states=task_object,object_states
