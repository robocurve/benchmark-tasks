"""Candidate YAM bridge for flexible native paper panels.

Each analytic100µm box is dynamic and edge-connected by a passive finite
bending hinge. Only reset writes poses. Inspection reconstructs paper vertices
and rejects broken/gaping material or staple crossings on every physics step.
Its two-action passive integration gate passes at100µs near the paper origin.
Loaded manipulation remains unverified.
"""
from dataclasses import dataclass,field
import importlib.abc
import importlib.util
import sys
import numpy as np
from yam_arena.task_file import Box,ObjectState
from facet_paper import author_shell,mechanics,measured_surface,ShellMaterial


@dataclass(frozen=True)
class PanelPaper(Box):
    mesh: object=None
    material: ShellMaterial=ShellMaterial()
    @property
    def free(self):return False


@dataclass(frozen=True)
class PanelState(ObjectState):
    nodes: np.ndarray=field(default_factory=lambda:np.empty((0,3)))
    facet_poses: np.ndarray=field(default_factory=lambda:np.empty((0,7)))
    facet_velocities: np.ndarray=field(default_factory=lambda:np.empty((0,6)))
    continuity: object=None


SPECS={}
SHELLS={}
POSITION_ITERATIONS=128
MAXIMUM_HINGE_GAP_M=.000025
GEOMETRIC_APERTURE_LIMIT_M=None
MINIMUM_PANEL_MASS_KG=0.


def _spawn(path,cfg,translation=None,orientation=None,**kwargs):
    from isaaclab.sim.utils.stage import get_current_stage
    spec=SPECS[cfg.paper_name];stage=get_current_stage()
    root=str(path).rsplit('/',1)[0]
    if cfg.paper_name not in SHELLS:
        from coordinate_frame import to_native
        from paper import PaperMesh
        mesh=PaperMesh(to_native(spec.mesh.rest),spec.mesh.faces,spec.mesh.holes)
        shell=author_shell(stage,root,mesh,spec.material,primitive_boxes=True,velocity_iterations=4,position_iterations=POSITION_ITERATIONS,minimum_panel_mass_kg=MINIMUM_PANEL_MASS_KG)
        model=shell['model'];shell['reset_poses']=np.c_[model['centres'],np.tile((0.,0.,0.,1.),(len(model['centres']),1))]
        SHELLS[cfg.paper_name]=shell
    prim=stage.GetPrimAtPath(str(path))
    if not prim:raise RuntimeError('Native paper panel missing: '+str(path))
    return prim


def reset_panels(env,env_ids,paper_name):
    import torch
    base=env.unwrapped if hasattr(env,'unwrapped') else env;obj=base.scene[paper_name]
    origins=getattr(base.scene.env_origins,'torch',base.scene.env_origins)
    if len(origins)!=1:raise ValueError('Only one fixed paper environment is supported')
    ids=torch.arange(len(origins),device=origins.device,dtype=torch.long) if env_ids is None else env_ids
    shell=SHELLS[paper_name];pose=torch.as_tensor(shell['reset_poses'],device=origins.device,dtype=origins.dtype)[None].repeat(len(ids),1,1)
    pose[:,:,:3]+=origins[ids,None,:]
    obj.write_body_link_pose_to_sim_index(body_poses=pose,env_ids=ids)
    obj.write_body_link_velocity_to_sim_index(body_velocities=torch.zeros((len(ids),len(pose[0]),6),device=origins.device,dtype=origins.dtype),env_ids=ids)


def asset(spec):
    from isaaclab.assets import RigidObjectCfg,RigidObjectCollectionCfg
    from isaaclab.managers import EventTermCfg
    from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.asset import Asset
    count=len(mechanics(spec.mesh,spec.material,primitive_boxes=True)['masses'])
    @configclass
    class SpawnCfg(SpawnerCfg):
        func: object=clone(_spawn)
        paper_name: str=''
    class PaperAsset(Asset):
        def __init__(self):
            super().__init__(spec.name,tags=['flexible-paper','passive-hinges','native-boxes'])
            self.cfg=RigidObjectCollectionCfg(rigid_objects={f'facet_{i:03d}':RigidObjectCfg(
                prim_path='{ENV_REGEX_NS}/'+spec.name+f'/facet_{i:03d}',
                spawn=SpawnCfg(paper_name=spec.name)) for i in range(count)})
        def get_object_cfg(self):return self.name,self.cfg
        def get_event_cfg(self):return self.name,EventTermCfg(func=reset_panels,mode='reset',params={'paper_name':self.name})
        def validate_simulation_cfg(self,sim_cfg):
            from isaaclab_physx.physics import PhysxCfg
            if sim_cfg.physics is not None and not isinstance(sim_cfg.physics,PhysxCfg):raise ValueError('Native panel paper requires PhysX')
    return PaperAsset()


def state(env,name):
    base=env.unwrapped if hasattr(env,'unwrapped') else env;obj=base.scene[name];shell=SHELLS[name]
    names=list(getattr(obj,'body_names',getattr(obj,'object_names',())))
    indices=[names.index(path.rsplit('/',1)[1]) for path in shell['bodies']]
    def read(*attrs):
        for attr in attrs:
            if hasattr(obj.data,attr):
                x=getattr(obj.data,attr);return getattr(x,'torch',x)[0,indices].detach().cpu().numpy().astype(float)
        raise AttributeError('Missing native panel telemetry '+repr(attrs))
    p=read('body_link_pos_w','object_link_pos_w');q=read('body_link_quat_w','object_link_quat_w')
    v=read('body_link_lin_vel_w','object_link_lin_vel_w');w=read('body_link_ang_vel_w','object_link_ang_vel_w')
    p-=getattr(base.scene.env_origins,'torch',base.scene.env_origins)[0].detach().cpu().numpy()
    poses=np.c_[p,q];nodes,continuity=measured_surface(shell,poses,MAXIMUM_HINGE_GAP_M,
        geometry_aperture_m=GEOMETRIC_APERTURE_LIMIT_M)
    if not all(np.isfinite(x).all() for x in (nodes,v,w)):raise RuntimeError('Invalid native paper state')
    low,high=nodes.min(0),nodes.max(0)
    return PanelState(pos=(low+high)/2,quat_xyzw=np.array((0.,0.,0.,1.)),lin_vel=v.mean(0),ang_vel=w.mean(0),
        low=low,high=high,nodes=nodes,facet_poses=poses,facet_velocities=np.c_[v,w],continuity=continuity)


class PanelMonitor:
    def __init__(self,simulation,item):
        self.item=item
        paths=item['metadata']['staple']['paths']+[item['metadata']['remover_path']]
        self.rigid=simulation.create_rigid_body_view(paths)
        self.shells=[SHELLS['paper_0'],SHELLS['paper_1']]
        self.views=[simulation.create_rigid_body_view(shell['bodies']) for shell in self.shells]
        self.indices=[[list(view.prim_paths).index(path) for path in shell['bodies']]
                      for view,shell in zip(self.views,self.shells)]
        if self.rigid.count!=len(paths) or any(v.count!=len(s['bodies']) for v,s in zip(self.views,self.shells)):
            raise RuntimeError('Incomplete native paper/hardware monitor')
        self.reset()
    def reset(self):
        from contact_guard import PaperWireCrossingGuard
        self.crossing=[PaperWireCrossingGuard(s['mesh'].faces) for s in self.shells]
        self.steps=0;self.last=None;self.frames={};self.failed=False
        self.failure_state=None
        self.peak_wire_gap_m=0.;self.peak_paper_gaps_m=[0.,0.]
    def get_rigidbody_transformation(self,path):
        row=self.frames.get(path)
        return {'ret_val':False} if row is None else {'ret_val':True,'position':row[:3],'rotation':row[3:]}
    def inspect(self,update_material=True):
        from native_paper import update_staple,measured_wire_segments
        from arena_hardware import MaterialDomainError
        self.frames=dict(zip(self.rigid.prim_paths,self.rigid.get_transforms().detach().cpu().numpy()))
        staple=self.item['metadata']['staple'];wire=measured_wire_segments(self,staple);values=[];paper_poses=[]
        from contact_guard import wire_chain_continuity
        chain=wire_chain_continuity(wire,poses_xyzw=np.asarray([self.frames[p] for p in staple['paths']]))
        self.peak_wire_gap_m=max(self.peak_wire_gap_m,chain['maximum_endpoint_gap_m'])
        for shell,view,ids,audit in zip(self.shells,self.views,self.indices,self.crossing):
            poses=view.get_transforms().detach().cpu().numpy()[ids]
            paper_poses.append(poses)
            nodes,continuity=measured_surface(shell,poses,MAXIMUM_HINGE_GAP_M,
                geometry_aperture_m=GEOMETRIC_APERTURE_LIMIT_M);crossing=audit.update(nodes,wire)
            values.append(dict(continuity=continuity,contact_audit=crossing,
                valid=continuity['valid'] and crossing['valid']))
        self.steps+=1;self.failed=self.failed or not chain['valid'] or not all(x['valid'] for x in values)
        self.peak_paper_gaps_m=[max(old,x['continuity']['maximum_hinge_endpoint_gap_m']) for old,x in zip(self.peak_paper_gaps_m,values)]
        self.last={'observations':self.steps,'paper':values,'wire_continuity':chain,
            'peak_wire_endpoint_gap_m':self.peak_wire_gap_m,'peak_paper_hinge_gaps_m':list(self.peak_paper_gaps_m),'valid':not self.failed}
        if self.failed:
            if self.failure_state is None:
                self.failure_state={f'paper_{i}_poses':p.copy() for i,p in enumerate(paper_poses)}
                self.failure_state['wire_segments']=wire.copy()
                self.failure_state['hardware_poses']=np.asarray([self.frames[p] for p in self.rigid.prim_paths]).copy()
            raise MaterialDomainError('Paper hinge continuity or staple crossing failed; no task score is valid')
        if update_material:self.last['steel']=update_staple(self,staple)
        return self.last


def install_material_hook(base,stage):
    from pxr import UsdUtils
    import omni.physics.tensors as tensors
    from arena_hardware import ASSEMBLIES
    cache=UsdUtils.StageCache.Get();sid=cache.GetId(stage).ToLongInt()
    if sid<0:cache.Insert(stage);sid=cache.GetId(stage).ToLongInt()
    simulation=tensors.create_simulation_view('torch',sid);simulation.set_subspace_roots('/')
    for item in ASSEMBLIES.values():
        item['simulation_view']=simulation;item['monitor']=PanelMonitor(simulation,item)
    original=base.scene.write_data_to_sim
    def before_substep(*args,**kwargs):
        result=original(*args,**kwargs)
        for item in ASSEMBLIES.values():item['monitor'].inspect()
        return result
    base.scene.write_data_to_sim=before_substep
    return original


def _patch(scene):
    if getattr(scene,'_native_panel_paper_installed',False):return
    original_object,original_states=scene.task_object,scene.object_states
    def task_object(spec):
        from arena_hardware import StapleHardware,hardware_asset
        if isinstance(spec,StapleHardware):return hardware_asset(spec)
        if isinstance(spec,PanelPaper):
            spec.mesh.audit();SPECS[spec.name]=spec;return asset(spec)
        return original_object(spec)
    def object_states(env,names):
        from arena_hardware import ASSEMBLIES,hardware_state
        normal=[n for n in names if n not in SPECS and n not in ASSEMBLIES]
        result=original_states(env,normal) if normal else {}
        result.update({n:state(env,n) for n in names if n in SPECS})
        result.update({n:hardware_state(env,n) for n in names if n in ASSEMBLIES})
        return {n:result[n] for n in names}
    scene.task_object,scene.object_states=task_object,object_states
    scene._native_panel_paper_installed=True


class Finder(importlib.abc.MetaPathFinder):
    def find_spec(self,fullname,path=None,target=None):
        if fullname!='yam_arena.scene':return None
        sys.meta_path.remove(self)
        try:spec=importlib.util.find_spec(fullname)
        finally:sys.meta_path.insert(0,self)
        original=spec.loader.exec_module
        def execute(module):original(module);_patch(module)
        spec.loader.exec_module=execute;return spec


if 'yam_arena.scene' in sys.modules:_patch(sys.modules['yam_arena.scene'])
elif not any(isinstance(x,Finder) for x in sys.meta_path):sys.meta_path.insert(0,Finder())
