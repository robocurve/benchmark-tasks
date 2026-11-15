"""Native passive-peg and fixed-fixture registration for the YAM task loader."""
from dataclasses import dataclass,field
import numpy as np
from yam_arena.task_file import Box,ObjectState


@dataclass(frozen=True)
class SpringPeg(Box):
    base_angle: float=0.
    open_angle: float=0.
    @property
    def free(self):
        return False  # reset the actual dynamic jaws instead of a phantom root

    def _check(self):
        super()._check()
        if self.yaw != 0.:
            raise ValueError('Peg yaw is fixed; the explicit initial pitch is authored on both jaws')
        if not np.isfinite((self.base_angle,self.open_angle)).all():
            raise ValueError('Peg initial angles must be finite')


@dataclass(frozen=True)
class Fixture(Box):
    kind: str=''


@dataclass(frozen=True)
class PegState(ObjectState):
    body_poses: np.ndarray=field(default_factory=lambda:np.empty((0,7)))
    body_velocities: np.ndarray=field(default_factory=lambda:np.empty((0,6)))


PEG_SPECS={}
FIXTURE_SPECS={}


def _peg_spawn(path,cfg,translation=None,orientation=None,**kwargs):
    from isaaclab.sim.utils.stage import get_current_stage
    from native_cloth import author_peg_jaw,author_peg_hinge,ensure_gpu_scene
    stage=get_current_stage()
    ensure_gpu_scene(stage)
    expected=(0.,np.sin(cfg.angle/2),0.,np.cos(cfg.angle/2))
    if orientation is not None and not np.allclose(orientation,expected):
        raise ValueError('Peg initial rotation differs from the authored jaw angle')
    position=translation or (0.,0.,0.)
    prim=author_peg_jaw(stage,str(path),position,upper=cfg.upper,angle=cfg.angle)
    if cfg.upper:
        # The lower body is first in the collection, so both jaws exist now.
        author_peg_hinge(stage,str(path).rsplit('/',1)[0],position,
                         fixed_lower=False,spring=True)
    import os
    if cfg.upper and os.environ.get('ROBOCURVE_SHIRT_FUNCTIONAL_PEGS')=='1':
        from functional_pegs import configure
        configure(stage,str(path).rsplit('/',1)[0])
    return prim


def reset_peg(env,env_ids,peg_name):
    import torch
    base=env.unwrapped if hasattr(env,'unwrapped') else env
    obj=base.scene[peg_name]
    origins=getattr(base.scene.env_origins,'torch',base.scene.env_origins)
    ids=torch.arange(len(origins),device=origins.device,dtype=torch.long) if env_ids is None else env_ids
    pose=torch.zeros((len(ids),2,7),device=origins.device,dtype=origins.dtype)
    pose[:,:,:3]=torch.as_tensor(PEG_SPECS[peg_name].pos,device=origins.device,dtype=origins.dtype)+origins[ids,None,:]
    spec=PEG_SPECS[peg_name]
    angles=torch.as_tensor((spec.base_angle,spec.base_angle+spec.open_angle),device=origins.device,dtype=origins.dtype)
    pose[:,:,4]=torch.sin(angles/2)
    pose[:,:,6]=torch.cos(angles/2)
    obj.write_body_link_pose_to_sim_index(body_poses=pose,env_ids=ids)
    obj.write_body_link_velocity_to_sim_index(
        body_velocities=torch.zeros((len(ids),2,6),device=origins.device,dtype=origins.dtype),env_ids=ids)


def peg_asset(spec):
    from isaaclab.assets import RigidObjectCfg,RigidObjectCollectionCfg
    from isaaclab.managers import EventTermCfg
    from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.asset import Asset
    spec._check()
    PEG_SPECS[spec.name]=spec
    @configclass
    class SpawnCfg(SpawnerCfg):
        func: object=clone(_peg_spawn)
        upper: bool=False
        angle: float=0.
    class PegAsset(Asset):
        def __init__(self):
            super().__init__(spec.name,tags=['spring-peg','passive','physx'])
            bodies={}
            for name,upper in (('lower',False),('upper',True)):
                angle=spec.base_angle+(spec.open_angle if upper else 0.)
                rotation=(0.,float(np.sin(angle/2)),0.,float(np.cos(angle/2)))
                bodies[name]=RigidObjectCfg(prim_path='{ENV_REGEX_NS}/'+spec.name+'/'+name,
                    spawn=SpawnCfg(upper=upper,angle=angle),
                    init_state=RigidObjectCfg.InitialStateCfg(pos=tuple(spec.pos),rot=rotation))
            self.cfg=RigidObjectCollectionCfg(rigid_objects=bodies)
        def get_object_cfg(self):
            return self.name,self.cfg
        def get_event_cfg(self):
            return self.name,EventTermCfg(func=reset_peg,mode='reset',params={'peg_name':self.name})
        def validate_simulation_cfg(self,sim_cfg):
            from isaaclab_physx.physics import PhysxCfg
            if sim_cfg.physics is not None and not isinstance(sim_cfg.physics,PhysxCfg):
                raise ValueError('Native passive pegs require PhysX')
    return PegAsset()


def peg_state(env,name):
    base=env.unwrapped if hasattr(env,'unwrapped') else env
    obj=base.scene[name]
    names=list(getattr(obj,'body_names',getattr(obj,'object_names',())))
    indices=[names.index(x) for x in ('lower','upper')]
    def read(*attrs):
        for attr in attrs:
            if hasattr(obj.data,attr):
                value=getattr(obj.data,attr)
                return getattr(value,'torch',value)[0,indices].detach().cpu().numpy().astype(float)
        raise AttributeError('Missing passive jaw telemetry '+repr(attrs))
    pos=read('body_link_pos_w','object_link_pos_w')
    origin=getattr(base.scene.env_origins,'torch',base.scene.env_origins)[0].detach().cpu().numpy()
    pos-=origin
    quat=read('body_link_quat_w','object_link_quat_w')
    lin=read('body_link_lin_vel_w','object_link_lin_vel_w')
    ang=read('body_link_ang_vel_w','object_link_ang_vel_w')
    if not all(np.isfinite(v).all() for v in (pos,quat,lin,ang)):
        raise ValueError('Nonfinite peg telemetry')
    # Conservative assessor AABB. Jaw capture is judged from actual contact
    # evidence, never from this group bounding box.
    lo=pos.min(0)-.038
    hi=pos.max(0)+.038
    return PegState(pos=(lo+hi)/2,quat_xyzw=quat[0],lin_vel=lin.mean(0),ang_vel=ang.mean(0),
                    low=lo,high=hi,body_poses=np.column_stack((pos,quat)),
                    body_velocities=np.column_stack((lin,ang)))


def _fixture_spawn(path,cfg,translation=None,orientation=None,**kwargs):
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf,UsdGeom
    stage=get_current_stage()
    root=UsdGeom.Xform.Define(stage,str(path))
    if translation is not None:
        root.AddTranslateOp().Set(Gf.Vec3d(*translation))
    if orientation is not None and not np.allclose(orientation,(0.,0.,0.,1.)):
        raise ValueError('Fixed fixtures use the authored coordinate frame')
    spec=FIXTURE_SPECS[cfg.fixture_name]
    if spec.kind=='clothesline':
        from scene import author_rack
        author_rack(stage,str(path))
    elif spec.kind=='laundry_support':
        from scene import author_support
        author_support(stage,str(path)+'/Support')
    elif spec.kind=='peg_rests':
        from peg_rests import author
        author(stage,str(path))
    elif spec.kind=='collar':
        from layout import author_collar
        author_collar(stage,str(path))
    else:
        raise ValueError('Unknown fixture '+spec.kind)
    return root.GetPrim()


def fixture_asset(spec):
    from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose
    FIXTURE_SPECS[spec.name]=spec
    @configclass
    class SpawnCfg(SpawnerCfg):
        func: object=clone(_fixture_spawn)
        fixture_name: str=''
    return Object(name=spec.name,object_type=ObjectType.BASE,
                  spawner_cfg=SpawnCfg(fixture_name=spec.name),
                  initial_pose=Pose(position_xyz=(0.,0.,0.),rotation_xyzw=(0.,0.,0.,1.)))
