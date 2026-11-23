"""Pinned IsaacLab contact sensors read after every native scene.update.

Import only after AppLauncher starts. This adapter deliberately rejects batched
control updates: force and dwell must resolve each physics substep. Dynamic USD
joint creation still requires a native pressure/retention validation run.
"""
import numpy as np
from config import CELLS

SENSOR_NAME='film_contacts'


def add_contact_sensor(embodiment, target_names):
    from isaaclab.sensors import ContactSensorCfg
    from isaaclab_arena.utils.configclass import make_configclass
    cfg=embodiment.scene_config
    # The scene builder enumerates annotated config fields. setattr alone is
    # silently ignored; recreate the config with all existing fields preserved.
    fields=[(name,type(getattr(cfg,name)),getattr(cfg,name)) for name in cfg.__class__.__annotations__]
    fields.append((SENSOR_NAME,ContactSensorCfg,ContactSensorCfg(
        prim_path='{ENV_REGEX_NS}/tape_.*', update_period=0., track_pose=False,
        filter_prim_paths_expr=['{ENV_REGEX_NS}/'+name for name in target_names])))
    embodiment.scene_config=make_configclass('PackingContactSceneCfg',fields)()


class ContactSubsteps:
    def __init__(self,base,target_sizes,enabled=True):
        from contact_native import NativeAdhesion
        from seal_geometry import BladeCuts
        self.base=base;self.targets=target_sizes;self.filters=[*target_sizes,'cutter'];self.enabled=enabled
        self.adhesion=NativeAdhesion(base.scene.stage,'/World/envs/env_0')
        self.blade=BladeCuts();self.steps=0;self.time=0.;self.last_forces={};self.max_force=0.;self.cut_coordinates=[]
        self.original_update=base.scene.update
        def after_update(dt,*args,**kwargs):
            result=self.original_update(dt,*args,**kwargs)
            self.update(float(dt))
            return result
        base.scene.update=after_update

    def close(self):self.base.scene.update=self.original_update

    @staticmethod
    def array(value):
        value=value.torch if hasattr(value,'torch') else value
        return value.detach().cpu().numpy() if hasattr(value,'detach') else np.asarray(value)

    def pose(self,name):
        from yam_arena.urdf import quat_xyzw_to_matrix
        p=self.array(self.base.arena_world.get_pose_w(name))[0]
        return {'pos':p[:3],'rotation':quat_xyzw_to_matrix(p[3:])}

    def update(self,dt):
        if abs(dt-float(self.base.sim.get_physics_dt()))>1e-9 or not 0<dt<=1/120:
            raise RuntimeError('contact adapter requires one scene.update per physics substep')
        sensor=self.base.scene[SENSOR_NAME]
        forces=self.array(sensor.data.force_matrix_w)
        names=list(sensor.body_names)
        if forces.shape!=(1,CELLS,len(self.filters),3) or set(names)!={f'tape_{i}' for i in range(CELLS)}:
            raise RuntimeError(f'unexpected native contact layout: {forces.shape}, {names}')
        if not np.isfinite(forces).all():raise RuntimeError('nonfinite native contact telemetry')
        self.last_forces={(name,target):forces[0,i,j].copy() for i,name in enumerate(names) for j,target in enumerate(self.filters)}
        self.max_force=max(self.max_force,float(np.linalg.norm(forces,axis=-1).max()))
        self.steps+=1;self.time+=dt
        if not self.enabled:return
        # Wound/free-flight material has no possible adhesion or blade action.
        # Avoid dozens of GPU pose readbacks when all filtered forces are zero.
        blade_contact=bool(np.any(np.linalg.norm(forces[0,:,-1],axis=-1)>=.05))
        pressure=bool(np.any(np.linalg.norm(forces[0,:,:-1],axis=-1)>=.5))
        if not blade_contact:self.blade.previous.clear()
        if not pressure and not self.adhesion.bonds and not blade_contact:return
        films={name:self.pose(name) for name in names}
        targets={name:dict(self.pose(name),size=size) for name,size in self.targets.items()}
        self.adhesion.step(dt,films,targets,self.last_forces)
        from pxr import UsdPhysics
        from config import CELL_LENGTH
        new_cuts=self.blade.step(films,self.pose('cutter'),self.last_forces) if blade_contact else []
        for index in new_cuts:
            joint=UsdPhysics.RevoluteJoint(self.base.scene.stage.GetPrimAtPath(f'/World/envs/env_0/tape_{index}/film_bend'))
            if not joint:raise RuntimeError('missing native material hinge at measured cut')
            joint.GetJointEnabledAttr().Set(False)
            self.cut_coordinates.append((CELLS-index)*CELL_LENGTH)

    def telemetry(self):
        return {'substeps':self.steps,'time_s':self.time,'max_filtered_force_n':self.max_force,
                'active_bonds':len(self.adhesion.active_patches()),'broken_bonds':len(self.adhesion.broken),
                'blade_cut_hinges':sorted(self.blade.cuts),'cut_coordinates_m':self.cut_coordinates,
                'force_samples':[{ 'film':f,'target':t,'force_w_n':v.tolist()} for (f,t),v in self.last_forces.items() if np.linalg.norm(v)>0.]}
