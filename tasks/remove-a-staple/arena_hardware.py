"""YAM bridge for the free segmented staple and ordinary lever remover.

Only reset writes rigid poses. The material hook changes steel plastic rest
angles from measured rotations and rejects unsupported paper states. This
integration is a development draft, awaiting the native contact gates.
"""
from dataclasses import dataclass,field
import numpy as np
from yam_arena.task_file import Box,ObjectState
from materials import clinched_staple_centerline


@dataclass(frozen=True)
class StapleHardware(Box):
    @property
    def free(self):
        return False


@dataclass(frozen=True)
class HardwareState(ObjectState):
    body_names: tuple=()
    body_poses: np.ndarray=field(default_factory=lambda:np.empty((0,7)))
    body_velocities: np.ndarray=field(default_factory=lambda:np.empty((0,6)))


BODY_NAMES=tuple(f'staple/wire_{i:03d}' for i in range(len(clinched_staple_centerline(sample_m=.0005))-1))+('remover',)
ASSEMBLIES={}
PANEL_CONTACT_CANDIDATE=False
PANEL_POSITION_ITERATIONS=128
FUNCTIONAL_MODEL=False


def _spawn(path,cfg,translation=None,orientation=None,**kwargs):
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf,UsdGeom,PhysxSchema
    from full_scene import build_scene
    import runtime_config
    stage=get_current_stage();root=str(path)[:-len(cfg.body_name)-1]
    if cfg.assembly_name not in ASSEMBLIES:
        metadata=build_scene(stage,root=root,include_table=False,include_papers=False,
            reset_profile=runtime_config.RESET_PROFILE,mesh_profile=runtime_config.MESH_PROFILE,
            paper_profile=runtime_config.PAPER_PROFILE)
        if FUNCTIONAL_MODEL:
            from functional_staple import configure
            configure(stage,metadata)
        if PANEL_CONTACT_CANDIDATE:
            from coordinate_frame import OFFSET_M
            UsdGeom.Xform.Define(stage,root).AddTranslateOp().Set(Gf.Vec3d(*OFFSET_M))
            for body_path in metadata['staple']['paths']:
                rigid=PhysxSchema.PhysxRigidBodyAPI.Get(stage,body_path)
                rigid.CreateSolverPositionIterationCountAttr(PANEL_POSITION_ITERATIONS)
                rigid.CreateSolverVelocityIterationCountAttr(4)
                rigid.CreateMaxDepenetrationVelocityAttr(.05)
        cache=UsdGeom.XformCache();poses=[]
        for name in BODY_NAMES:
            matrix=cache.GetLocalToWorldTransform(stage.GetPrimAtPath(root+'/'+name))
            q=matrix.ExtractRotationQuat()
            poses.append([*map(float,matrix.ExtractTranslation()),*map(float,q.GetImaginary()),float(q.GetReal())])
        ASSEMBLIES[cfg.assembly_name]={'root':root,'metadata':metadata,'reset_poses':np.asarray(poses),'monitor':None}
    prim=stage.GetPrimAtPath(str(path))
    if not prim:raise RuntimeError('Missing native hardware body '+str(path))
    return prim


def reset_hardware(env,env_ids,assembly_name):
    import torch
    base=env.unwrapped if hasattr(env,'unwrapped') else env
    obj=base.scene[assembly_name]
    origins=getattr(base.scene.env_origins,'torch',base.scene.env_origins)
    if len(origins)!=1:raise ValueError('Only a single fixed staple environment is supported')
    ids=torch.arange(len(origins),device=origins.device,dtype=torch.long) if env_ids is None else env_ids
    item=ASSEMBLIES[assembly_name]
    pose=torch.as_tensor(item['reset_poses'],device=origins.device,dtype=origins.dtype)[None].repeat(len(ids),1,1)
    pose[:,:,:3]+=origins[ids,None,:]
    obj.write_body_link_pose_to_sim_index(body_poses=pose,env_ids=ids)
    obj.write_body_link_velocity_to_sim_index(body_velocities=torch.zeros((len(ids),len(BODY_NAMES),6),device=origins.device,dtype=origins.dtype),env_ids=ids)
    for h in item['metadata']['staple']['hinges']:h['material'].reset()
    if item['monitor'] is not None:item['monitor'].reset()


def hardware_asset(spec):
    from isaaclab.assets import RigidObjectCfg,RigidObjectCollectionCfg
    from isaaclab.managers import EventTermCfg
    from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.asset import Asset
    @configclass
    class SpawnCfg(SpawnerCfg):
        func: object=clone(_spawn)
        assembly_name: str=''
        body_name: str=''
    class HardwareAsset(Asset):
        def __init__(self):
            super().__init__(spec.name,tags=['staple','plastic-wire','physx'])
            self.cfg=RigidObjectCollectionCfg(rigid_objects={name.replace('/','__'):RigidObjectCfg(
                prim_path='{ENV_REGEX_NS}/'+spec.name+'/'+name,
                spawn=SpawnCfg(assembly_name=spec.name,body_name=name)) for name in BODY_NAMES})
        def get_object_cfg(self):return self.name,self.cfg
        def get_event_cfg(self):return self.name,EventTermCfg(func=reset_hardware,mode='reset',params={'assembly_name':self.name})
        def validate_simulation_cfg(self,sim_cfg):
            from isaaclab_physx.physics import PhysxCfg
            if sim_cfg.physics is not None and not isinstance(sim_cfg.physics,PhysxCfg):raise ValueError('Native staple scene requires PhysX')
    return HardwareAsset()


def hardware_state(env,name):
    base=env.unwrapped if hasattr(env,'unwrapped') else env;obj=base.scene[name]
    names=list(getattr(obj,'body_names',getattr(obj,'object_names',())))
    indices=[names.index(n.replace('/','__')) for n in BODY_NAMES]
    def read(*attrs):
        for attr in attrs:
            if hasattr(obj.data,attr):
                x=getattr(obj.data,attr);return getattr(x,'torch',x)[0,indices].detach().cpu().numpy().astype(float)
        raise AttributeError('Missing hardware telemetry '+repr(attrs))
    p=read('body_link_pos_w','object_link_pos_w');q=read('body_link_quat_w','object_link_quat_w')
    v=read('body_link_lin_vel_w','object_link_lin_vel_w');w=read('body_link_ang_vel_w','object_link_ang_vel_w')
    p-=getattr(base.scene.env_origins,'torch',base.scene.env_origins)[0].detach().cpu().numpy()
    if not all(np.isfinite(x).all() for x in (p,q,v,w)):raise RuntimeError('Invalid hardware telemetry')
    lo,hi=p.min(0)-.06,p.max(0)+.06
    return HardwareState(pos=(lo+hi)/2,quat_xyzw=q[-1],lin_vel=v.mean(0),ang_vel=w.mean(0),low=lo,high=hi,
        body_names=BODY_NAMES,body_poses=np.column_stack((p,q)),body_velocities=np.column_stack((v,w)))


class MaterialDomainError(RuntimeError):pass


class NativeMonitor:
    def __init__(self,simulation,item,paper_specs):
        self.item=item;self.specs=paper_specs
        self.rigid=simulation.create_rigid_body_view(item['metadata']['staple']['paths']+[item['metadata']['remover_path']])
        env_root=item['root'].rsplit('/',1)[0]
        self.paper_views=[simulation.create_surface_deformable_body_view(env_root+'/'+s.name) for s in self.specs]
        if any(view.count!=1 for view in self.paper_views):raise RuntimeError('Native paper missing from monitor')
        self.reset()
    def reset(self):
        from paper import PaperIntegrityGuard
        from contact_guard import PaperWireCrossingGuard
        self.guards=[PaperIntegrityGuard(s.mesh,s.material) for s in self.specs]
        self.crossing=[PaperWireCrossingGuard(s.mesh.faces) for s in self.specs]
        self.steps=0;self.last=None;self.frames={}
    def get_rigidbody_transformation(self,path):
        row=self.frames.get(path)
        return {'ret_val':False} if row is None else {'ret_val':True,'position':row[:3],'rotation':row[3:]}
    def inspect(self,update_material=True):
        from native_paper import update_staple,measured_wire_segments
        self.frames=dict(zip(self.rigid.prim_paths,self.rigid.get_transforms().detach().cpu().numpy()))
        staple=self.item['metadata']['staple'];wire=measured_wire_segments(self,staple);values=[]
        for spec,view,guard,audit in zip(self.specs,self.paper_views,self.guards,self.crossing):
            nodes=view.get_simulation_nodal_positions().detach().cpu().numpy()[0,:len(spec.mesh.rest)].copy()
            result=guard.update(nodes);result['contact_audit']=audit.update(nodes,wire)
            result['valid']=result['valid'] and result['contact_audit']['valid'];values.append(result)
        self.steps+=1;self.last={'observations':self.steps,'paper':values,'valid':all(x['valid'] for x in values)}
        if not self.last['valid']:raise MaterialDomainError('Trial left the supported membrane/contact domain; no task score is valid')
        if update_material:self.last['steel']=update_staple(self,staple)
        return self.last


def install_material_hook(base,stage):
    from pxr import UsdUtils
    import omni.physics.tensors as tensors
    from arena_paper import SPECS
    cache=UsdUtils.StageCache.Get();stage_id=cache.GetId(stage).ToLongInt()
    if stage_id<0:cache.Insert(stage);stage_id=cache.GetId(stage).ToLongInt()
    simulation=tensors.create_simulation_view('torch',stage_id);simulation.set_subspace_roots('/')
    for item in ASSEMBLIES.values():
        item['simulation_view']=simulation
        item['monitor']=NativeMonitor(simulation,item,[SPECS['paper_0'],SPECS['paper_1']])
    original=base.scene.write_data_to_sim
    def before_substep(*args,**kwargs):
        result=original(*args,**kwargs)
        for item in ASSEMBLIES.values():item['monitor'].inspect()
        return result
    base.scene.write_data_to_sim=before_substep
    return original
