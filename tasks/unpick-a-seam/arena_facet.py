"""Opt-in native rigid-facet cloth collection for the YAM loader.

Pose/velocity writes occur only in the ordinary episode reset. Controller
steps use native bodies and passive hinges; measured vertices feed assessment.
"""
from dataclasses import dataclass,field
import numpy as np
from yam_arena.task_file import Box,ObjectState
from yam_arena import scene
from native_cloth import Fabric
from facet_cloth import ShellMaterial,mechanics,author_shell,measured_surface


@dataclass(frozen=True)
class FacetCloth(Box):
    mesh: object=None
    initial: object=None
    fabric: Fabric=Fabric()
    contact_refinement: object=None
    primitive_boxes: bool=False
    @property
    def free(self):return False
    def _check(self):
        super()._check();self.mesh.audit()
        if not np.array_equal(self.initial,self.mesh.rest):
            raise ValueError('The facet candidate requires its explicit unstressed reset')


@dataclass(frozen=True)
class FacetState(ObjectState):
    centres: np.ndarray=field(default_factory=lambda:np.empty((0,3)))


SPECS={};SPAWNED={}


def material(spec):
    f=spec.fabric
    import os
    return ShellMaterial(f.thickness,f.density,f.bend_modulus,f.poissons_ratio,.15,f.dynamic_friction,
                        .000005 if os.environ.get('ROBOCURVE_SEAM_PRACTICAL_MATERIAL')=='1' else 0.)


def initial(spec):
    m=mechanics(spec.mesh,material(spec),spec.primitive_boxes)
    return np.column_stack((m['centres'],np.tile((0.,0.,0.,1.),(len(m['centres']),1))))


def spawn(path,cfg,**kwargs):
    from isaaclab.sim.utils.stage import get_current_stage
    stage=get_current_stage();root=str(path).rsplit('/',1)[0];spec=SPECS[cfg.facet_name]
    if not stage.GetPrimAtPath(str(path)):
        SPAWNED[spec.name]=author_shell(stage,root,spec.mesh,material(spec),spec.color,primitive_boxes=spec.primitive_boxes)
        from cpu_scene import configure_native
        configure_native(stage,SPAWNED[spec.name]['bodies'])
    return stage.GetPrimAtPath(str(path))


def reset(env,env_ids,facet_name):
    import torch
    base=env.unwrapped if hasattr(env,'unwrapped') else env
    origins=getattr(base.scene.env_origins,'torch',base.scene.env_origins)
    ids=torch.arange(len(origins),device=origins.device) if env_ids is None else env_ids
    pose=torch.as_tensor(initial(SPECS[facet_name]),device=origins.device,dtype=origins.dtype)
    pose=pose[None].repeat(len(ids),1,1);pose[:,:,:3]+=origins[ids,None,:]
    obj=base.scene[facet_name]
    obj.write_body_link_pose_to_sim_index(body_poses=pose,env_ids=ids)
    obj.write_body_link_velocity_to_sim_index(body_velocities=torch.zeros((len(ids),pose.shape[1],6),device=origins.device),env_ids=ids)


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
        facet_name: str=''
    class FacetAsset(Asset):
        def __init__(self):super().__init__(spec.name,tags=['native-facet-shell','contact','physx'])
        def get_object_cfg(self):
            bodies={f'facet_{i:03d}':RigidObjectCfg(prim_path='{ENV_REGEX_NS}/'+spec.name+f'/facet_{i:03d}',
                spawn=SpawnCfg(facet_name=spec.name),
                init_state=RigidObjectCfg.InitialStateCfg(pos=tuple(row[:3]),rot=tuple(row[3:])))
                for i,row in enumerate(initial(spec))}
            return self.name,RigidObjectCollectionCfg(rigid_objects=bodies)
        def get_event_cfg(self):return self.name,EventTermCfg(func=reset,mode='reset',params={'facet_name':self.name})
        def validate_simulation_cfg(self,sim_cfg):
            from isaaclab_physx.physics import PhysxCfg
            if sim_cfg.physics is not None and not isinstance(sim_cfg.physics,PhysxCfg):
                raise ValueError('The facet candidate requires native PhysX')
    return FacetAsset()


class FacetView:
    """Read-only surface adapter; preserve every-substep hinge-gap evidence."""
    count=1
    def __init__(self,simulation,shell,*,geometry_aperture_m=None,boundary_tolerance_m=0.,require_edge_overlap=True):
        self.shell=shell;self.view=simulation.create_rigid_body_view(shell['bodies'])
        self.geometry_aperture_m=geometry_aperture_m
        self.require_edge_overlap=require_edge_overlap
        if not 0.<=boundary_tolerance_m<=.0001:raise ValueError('Boundary tolerance outside0–0.1mm')
        self.boundary_tolerance_m=boundary_tolerance_m
        paths=list(self.view.prim_paths)
        self.order=np.array([paths.index(p) for p in shell['bodies']])
        if self.view.count!=len(self.order):raise RuntimeError('Missing facet body')
        self.reset_audit()
    def reset_audit(self):
        self.maximum_gap=0.;self.first_failure=None;self.updates=0
        self.first_endpoint_warning=None;self.last_check=None;self.last_poses=None
        self.first_aperture_warning=None
        self.first_solid_failure=None;self.solid_checks=0;self.maximum_thread_overlap=0.
        self.tolerated_boundary_intersections=0;self.maximum_tolerated_boundary_depth_m=0.
        self.geometry_maxima={}
    def poses(self):
        values=self.view.get_transforms()
        return getattr(values,'torch',values).detach().cpu().numpy()[self.order].copy()
    def get_simulation_nodal_positions(self):
        import torch
        self.last_poses=self.poses()
        points,check=measured_surface(self.shell,self.last_poses,
                                     geometry_aperture_m=self.geometry_aperture_m,
                                     require_edge_overlap=self.require_edge_overlap)
        self.last_check=check
        self.maximum_gap=max(self.maximum_gap,check['maximum_hinge_endpoint_gap_m'])
        if check['legacy_endpoint_warning'] and self.first_endpoint_warning is None:
            self.first_endpoint_warning=dict(update=self.updates,**check)
        if check.get('aperture_warning') and self.first_aperture_warning is None:
            self.first_aperture_warning=dict(update=self.updates,**check)
        for key,value in check.get('geometry',{}).items():
            if key.startswith('maximum_'):
                self.geometry_maxima[key]=max(value,self.geometry_maxima.get(key,0.))
        if not check['valid'] and self.first_failure is None:
            self.first_failure=dict(update=self.updates,**check)
        self.updates+=1
        return torch.as_tensor(points[None])
    def get_simulation_nodal_velocities(self):
        import torch
        from scipy.spatial.transform import Rotation
        poses=self.poses();values=self.view.get_velocities()
        velocity=getattr(values,'torch',values).detach().cpu().numpy()[self.order]
        mesh=self.shell['mesh'];model=self.shell['model']
        offset=np.einsum('fij,fvj->fvi',Rotation.from_quat(poses[:,3:]).as_matrix(),model['points']-model['centres'][:,None])
        corners=velocity[:,:3,None].transpose(0,2,1)+np.cross(velocity[:,None,3:],offset)
        out=np.zeros_like(mesh.rest);count=np.zeros(len(mesh.rest),int)
        ids=model['vertex_ids']
        np.add.at(out,ids.ravel(),corners.reshape(-1,3));np.add.at(count,ids.ravel(),1)
        return torch.as_tensor((out/count[:,None])[None])
    def summary(self):
        return dict(maximum_hinge_endpoint_gap_m=self.maximum_gap,first_failure=self.first_failure,
                    valid=self.first_failure is None and self.first_solid_failure is None,reads=self.updates,
                    legacy_endpoint_warning_m=.0001,
                    endpoint_distance_is_stop_condition=self.geometry_aperture_m is None,
                    maximum_allowed_empty_aperture_m=self.geometry_aperture_m,
                    first_endpoint_warning=self.first_endpoint_warning,
                    first_aperture_warning=self.first_aperture_warning,
                    native_thread_solid_audit=dict(checks=self.solid_checks,
                        boundary_tolerance_m=self.boundary_tolerance_m,
                        tolerated_boundary_intersections=self.tolerated_boundary_intersections,
                        maximum_tolerated_boundary_depth_m=self.maximum_tolerated_boundary_depth_m,
                        first_failure=self.first_solid_failure,
                        maximum_capsule_overlap_m=self.maximum_thread_overlap,
                        interpretation='Exact current capsule axes against actual box midplanes; finite overlap is reported separately. The independent swept mesh/junction guards remain active.'),
                    geometry_maxima=self.geometry_maxima,last_check=self.last_check)

    def audit_thread_solids(self,segments,radius,time_s):
        from panel_diagnostics import solid_contacts
        result=solid_contacts(self.shell['model'],self.last_poses,segments,
                              radius=radius,thickness=self.shell['material'].thickness_m,
                              boundary_tolerance_m=self.boundary_tolerance_m)
        tolerated=result.get('tolerated_boundary_intersections',[])
        self.tolerated_boundary_intersections+=len(tolerated)
        self.maximum_tolerated_boundary_depth_m=max(self.maximum_tolerated_boundary_depth_m,
            max((r['distance_to_actual_boundary_m'] for r in tolerated),default=0.))
        self.solid_checks+=1
        self.maximum_thread_overlap=max(self.maximum_thread_overlap,
            result['maximum_capsule_overlap_lower_bound_m'])
        if result['native_midplane_intersections'] and self.first_solid_failure is None:
            self.first_solid_failure=dict(time_s=float(time_s),**result)
        return self.first_solid_failure is not None

    def native_state(self):
        """Unaveraged readback for geometry reconstruction after any failure."""
        values=self.view.get_velocities()
        velocity=getattr(values,'torch',values).detach().cpu().numpy()[self.order].copy()
        return self.poses(),velocity


original_object,original_states=scene.task_object,scene.object_states
def task_object(spec):
    if not isinstance(spec,FacetCloth):return original_object(spec)
    spec._check();SPECS[spec.name]=spec;return asset(spec)
def object_states(env,names):
    plain=[name for name in names if name not in SPECS]
    result=original_states(env,plain) if plain else {}
    base=env.unwrapped if hasattr(env,'unwrapped') else env
    def arr(v):return getattr(v,'torch',v).detach().cpu().numpy()
    def field(data,old,new):return getattr(data,old) if hasattr(data,old) else getattr(data,new)
    for name in names:
        if name not in SPECS:continue
        data=base.scene[name].data
        pts=arr(field(data,'body_link_pos_w','object_link_pos_w'))[0]-arr(base.scene.env_origins)[0]
        lo,hi=pts.min(0),pts.max(0)
        result[name]=FacetState(pos=(lo+hi)/2,quat_xyzw=np.array((0.,0.,0.,1.)),
            lin_vel=arr(field(data,'body_link_lin_vel_w','object_link_lin_vel_w'))[0].mean(0),
            ang_vel=np.zeros(3),low=lo,high=hi,centres=pts)
    return {name:result[name] for name in names}
scene.task_object,scene.object_states=task_object,object_states
