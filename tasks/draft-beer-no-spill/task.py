"""A primed hand-pull beer engine and an empty 500 mL handled glass on the station.

Serve at least 400 mL using the hand-pull engine without spilling outside the glass.
The fixed wooden counter frame supports the cylinder-equipped engine above the table.
"""
import gzip
import json
import tempfile
from pathlib import Path

import numpy as np
from resist_isaac.state import pose_state
from resist_isaac.usd_assets import create_asset
from .geometry import geometry, GLASS_START, ROTATION, NOZZLE
from .pump import add_loop_joint, measured_extension, StrokeMeter, CRANK_PIN, STROKE, PULL_ANGLE, EFFECTIVE_AREA
from .scoring import BeerScore

CONDITIONS = ('recoverable',)


def spawn_beer(prim_path, cfg, translation=None, orientation=None, **kwargs):
    """Enable GPU readback after context configuration, before native attach."""
    import carb
    import os
    import omni.usd
    from pxr import UsdPhysics, PhysxSchema
    from isaaclab.sim.spawners.from_files import spawn_from_usd
    settings=carb.settings.get_settings()
    settings.set('/physics/suppressReadback',False)
    settings.set('/physics/disableContactProcessing',False)
    settings.set('/physics/cudaDevice',int(os.environ.get('BEER_PHYSICS_GPU','0')))
    for prim in omni.usd.get_context().get_stage().Traverse():
        if prim.IsA(UsdPhysics.Scene):
            scene=PhysxSchema.PhysxSceneAPI.Apply(prim)
            scene.CreateEnableGPUDynamicsAttr(True)
            scene.CreateBroadphaseTypeAttr('GPU')
    for key in ('/physics/updateToUsd','/physics/updateParticlesToUsd',
                '/physics/updateVelocitiesToUsd'):
        settings.set(key,True)
    return spawn_from_usd(prim_path,cfg,translation=translation,
                          orientation=orientation,**kwargs)


class DraftBeer:
    physics_hz = 480
    hold_s = .05
    policy_actions = 1400
    retry_blank_camera_frames = True

    def __init__(self):
        import os
        self.app_device='cuda:'+os.environ.get('BEER_PHYSICS_GPU','0')
        self.descriptions = geometry()
        self.scorer = BeerScore()
        self.metrics = dict(score=0.,retained_ml=0.,spilled_ml=0.,emitted_ml=0.,
                            pending_particles=0,spill_latched=False)
        self.directory = None
        self.error = None
        self.subscriptions = []
        self.elapsed = 0.
        self.first_liquid_position = None
        self.liquid_motion_observed = False

    def set_evidence_directory(self, directory):
        self.directory = Path(directory)

    def build_scene(self, embodiment):
        from isaaclab_arena.assets.object import Object, ObjectType
        from isaaclab_arena.assets.object_reference import ObjectReference
        from isaaclab_arena.utils.pose import Pose
        from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema
        from .fluid import create_fluid
        self.temporary = tempfile.TemporaryDirectory(prefix='draft-beer-')
        root = Path(self.temporary.name)
        objects = []
        for name, description in self.descriptions.items():
            path = create_asset(description, root/(name+'.usda'),source_dir=Path(__file__).parent)
            asset_stage=Usd.Stage.Open(str(path))
            asset_root=str(asset_stage.GetDefaultPrim().GetPath())
            if name=='tap':add_loop_joint(asset_stage,asset_root)
            # Preserve the original CAD normals and material metalness.
            from pxr import Sdf, UsdShade, Vt
            for body in description['bodies']:
                for i,item in enumerate(body['shapes']):
                    shader=UsdShade.Shader.Get(asset_stage,
                        f"{asset_root}/Materials/{body['name']}_{i:03d}/Surface")
                    shader.CreateInput('metallic',Sdf.ValueTypeNames.Float).Set(float(item.get('metallic',0.)))
                    if item.get('mesh_file'):
                        mesh_path=Path(__file__).parent/item['mesh_file']
                        normals=[list(map(float,line.split()[1:])) for line in mesh_path.read_text().splitlines() if line.startswith('vn ')]
                        mesh=UsdGeom.Mesh.Get(asset_stage,f"{asset_root}/{body['name']}/shape_{i:03d}")
                        if len(normals)!=len(mesh.GetPointsAttr().Get()):raise RuntimeError('CAD normal count mismatch')
                        mesh.CreateNormalsAttr(Vt.Vec3fArray.FromNumpy(np.asarray(normals,dtype=np.float32)))
                        mesh.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
            for prim in asset_stage.Traverse():
                if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                    PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.)
            asset_stage.GetRootLayer().Save()
            obj = Object(name=name,object_type=ObjectType.BASE,usd_path=str(path),
                initial_pose=Pose(position_xyz=GLASS_START if name=='glass' else (0,0,0)))
            objects.append(obj)
            for body in description['bodies']:
                objects.append(ObjectReference(parent_asset=obj,name=name+'_'+body['name'],
                    prim_path=obj.get_prim_path()+'/'+body['name'],object_type=ObjectType.RIGID))
        stage=Usd.Stage.CreateNew(str(root/'beer.usda'))
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage,1.)
        parent=UsdGeom.Xform.Define(stage,'/Beer')
        stage.SetDefaultPrim(parent.GetPrim())
        create_fluid(stage,'/Beer')
        stage.GetRootLayer().Save()
        from isaaclab.sim import UsdFileCfg
        objects.append(Object(name='beer',object_type=ObjectType.BASE,
            spawner_cfg=UsdFileCfg(usd_path=str(root/'beer.usda'),func=spawn_beer),
            initial_pose=Pose(position_xyz=(0,0,0))))
        return objects

    def configure_native(self,cfg):
        cfg.sim.use_fabric=False
        cfg.scene.right_robot.init_state.joint_pos['joint2']=.8
        for side in ('left_robot','right_robot'):
            getattr(cfg.scene,side).actuators['gripper'].joint_effort_limit=40.

    def initial_action(self):
        from yam_arena.constants import READY
        right=np.asarray(READY,dtype=float).copy()
        right[1]=.8
        return np.r_[READY,1.,right,1.]

    def pose_state(self,base,name,part=None):
        return pose_state(base.scene[name+'_'+(part or name)])

    def reset(self,base):
        import carb
        for key in ('/physics/updateToUsd','/physics/updateParticlesToUsd',
                    '/physics/updateVelocitiesToUsd'):
            carb.settings.get_settings().set(key,True)

    def begin_episode(self,base):
        import carb
        import omni.usd
        from omni.physx import get_physx_interface
        from pxr import UsdGeom, UsdPhysics, PhysxSchema
        from .fluid import NativeBeerFluid
        carb.settings.get_settings().set('/physics/suppressReadback',False)
        self.physics_gpu=int(carb.settings.get_settings().get('/physics/cudaDevice'))
        if self.physics_gpu<0:raise RuntimeError('No native particle physics GPU selected')
        self.base=base
        self.contact_setting=base.sim.get_setting('/physics/disableContactProcessing')
        base.sim.set_setting('/physics/disableContactProcessing',False)
        self.native=get_physx_interface()
        self.stage=omni.usd.get_context().get_stage()
        scenes=[p for p in self.stage.Traverse() if p.IsA(UsdPhysics.Scene)]
        if len(scenes)!=1 or not PhysxSchema.PhysxSceneAPI(scenes[0]).GetEnableGPUDynamicsAttr().Get():
            raise RuntimeError('Native beer requires a verified GPU dynamics scene')
        points=[p for p in self.stage.Traverse() if p.IsA(UsdGeom.Points)
                and p.GetName()=='particles']
        if len(points)!=1: raise RuntimeError('Native beer particle set missing or ambiguous')
        self.fluid=NativeBeerFluid(self.stage,str(points[0].GetPath().GetParentPath()))
        from .contacts import NativeContacts
        self.contacts=NativeContacts(base,self.stage,self.directory) if self.directory else None
        state=self.pump_state()
        self.meter=StrokeMeter(state['piston_extension_m'])
        self.physics_step=base.sim.get_physics_step_count()
        self.records=(self.directory/'native-liquid.jsonl').open('w') if self.directory else None
        self.positions=gzip.open(self.directory/'native-liquid.npy.gz','wb',compresslevel=1) if self.directory else None
        self.subscriptions=[self.native.subscribe_physics_on_step_events(self.before_step,pre_step=True,order=0),
                            self.native.subscribe_physics_on_step_events(self.after_step,pre_step=False,order=0)]

    def native_body(self,name):
        from yam_arena.urdf import quat_xyzw_to_matrix
        transform=np.asarray(self.base.scene['tap_'+name].root_view.get_transforms().numpy())[0]
        return transform[:3],quat_xyzw_to_matrix(transform[3:])

    def pump_state(self):
        lever,lr=self.native_body('lever')
        cylinder,cr=self.native_body('cylinder')
        piston,_=self.native_body('piston')
        separation=piston-cylinder
        lateral=float(np.linalg.norm(separation-np.dot(separation,cr[:,2])*cr[:,2]))
        closure=float(np.linalg.norm(lever+lr@CRANK_PIN-piston))
        extension=measured_extension(cylinder,cr,piston)
        relative=ROTATION.T@lr
        angle=float(np.arctan2(relative[0,2],relative[2,2]))
        if (closure>.001 or lateral>.001 or not -.001<=extension<=STROKE+.001
                or not -PULL_ANGLE-.03<=angle<=.03):
            raise RuntimeError(f'Invalid native pump linkage: closure={closure}, lateral={lateral}, extension={extension}, angle={angle}')
        return dict(handle_angle_rad=angle,piston_extension_m=extension,
                    crank_closure_error_m=closure,slider_lateral_error_m=lateral)

    def before_step(self,dt):
        if self.error:return
        try:
            delivered=self.meter.update(self.pump_state()['piston_extension_m'])
            self.fluid.emit_volume(delivered,float(dt))
        except Exception as error:
            self.error=error

    def after_step(self,dt):
        if self.error: return
        try:
            step=self.base.sim.get_physics_step_count()
            if step!=self.physics_step+1 or abs(float(dt)-1/self.physics_hz)>1e-8:
                raise RuntimeError(f'Inconsistent native liquid step: {step}, expected {self.physics_step+1}, dt={float(dt)}')
            self.physics_step=step
            if self.contacts:self.contacts.sample(step)
            self.native.update_transformations(False,True,True,False)
            from yam_arena.urdf import quat_xyzw_to_matrix
            transform=np.asarray(self.base.scene['glass_glass'].root_view.get_transforms().numpy())[0]
            center=transform[:3]
            rotation=quat_xyzw_to_matrix(transform[3:])
            particles=self.fluid.positions()
            if len(particles):
                if self.first_liquid_position is None:
                    self.first_liquid_position=particles[0].copy()
                    self.first_liquid_time=self.elapsed
                self.liquid_motion_observed |= bool(np.linalg.norm(particles[0]-self.first_liquid_position)>1e-5)
                if self.elapsed-self.first_liquid_time>.1 and not self.liquid_motion_observed:
                    raise RuntimeError('Native liquid readback did not move after birth')
            self.metrics=self.scorer.update(particles,center,rotation)
            state=self.pump_state()
            self.metrics.update(state,native_liquid_motion_observed=self.liquid_motion_observed,pump_supplied_ml=self.meter.supplied_ml,
                accepted_piston_extension_m=self.meter.previous,
                next_step_displacement_ml=max(state['piston_extension_m']-self.meter.previous,0.)*EFFECTIVE_AREA*1e6)
            from .fluid import PARTICLE_ML
            self.metrics['inlet_pending_ml']=self.fluid._remainder*PARTICLE_ML
            if abs((self.fluid.count+self.fluid._remainder)*PARTICLE_ML-self.meter.supplied_ml)>1e-6:
                raise RuntimeError('Native pump supply/particle balance mismatch')
            self.elapsed+=float(dt)
            if self.positions:
                np.save(self.positions,particles.astype(np.float32),allow_pickle=False)
            if self.records:
                self.records.write(json.dumps(dict(time_s=self.elapsed,physics_step=step,particles=len(particles),
                    glass_position=center.tolist(),glass_rotation=rotation.tolist(),
                    **self.metrics))+'\n')
        except Exception as error:
            self.error=error

    def evaluate(self,base):
        if self.error: raise RuntimeError('Native liquid evidence is incomplete') from self.error
        # Full credit requires the fixed horizon and two-second observation tail.
        # Shorter mechanism diagnostics cannot establish task completion.
        score=float(self.metrics['score'])
        if self.elapsed < (self.policy_actions+40)/20 - 1e-6:
            score=min(score,np.nextafter(1.,0.))
        return dict(self.metrics,score=score,success=bool(score==1.),
                    time_s=self.elapsed,scene_gpu_dynamics=True,physics_gpu_index=self.physics_gpu)

    def validate_reset(self,base):
        position,_,_=self.pose_state(base,'glass')
        if np.linalg.norm(position.cpu().numpy()-GLASS_START)>.005 or abs(self.pump_state()['handle_angle_rad'])>.01:
            raise RuntimeError('Native glass/hand-pump reset does not match task layout')
        if self.metrics['emitted_ml']!=0:raise RuntimeError('Hand-pump reset emitted beer')

    def scripted_actions(self,base,policy,max_steps):
        from .controller import scripted_actions
        yield from scripted_actions(self,base,policy,max_steps)

    def close(self):
        self.subscriptions.clear()
        if getattr(self,'contacts',None):
            self.contacts.close()
            self.contacts=None
        if hasattr(self,'contact_setting'):
            self.base.sim.set_setting('/physics/disableContactProcessing',self.contact_setting)
        for file in (getattr(self,'records',None),getattr(self,'positions',None)):
            if file:file.close()
        self.records=self.positions=None


def make_task(condition):
    if condition not in CONDITIONS:raise ValueError(condition)
    return DraftBeer()
