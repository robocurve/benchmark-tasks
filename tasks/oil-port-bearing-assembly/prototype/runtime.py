"""Task-only scene authoring and private state; ordinary calibrated station inputs."""
import atexit,json
from pathlib import Path
import numpy as np
from . import layout as L
from .contact import Monitor
from .scoring import Evaluator
from .passage import check as live_passage
MONITOR=Monitor();TRACE=[];RAYS=[]
EVALUATOR=Evaluator()

def passage_check():
    """Legacy 7 mm CPU query diagnostic, NEVER a scoring authority.

    This stack missed moving bodies despite the static guard; the actual score
    uses the separate live-pose 10 mm convex-solid check below.
    """
    from omni.physx import get_physx_scene_query_interface
    query=get_physx_scene_query_interface();x,y,z=map(float,L.HOUSING)
    start=(x,y+L.HOUSING_OUT+.007,z);length=L.HOUSING_OUT+.007-L.R_IN+.005
    guard=query.sweep_sphere_closest(.0035,(x+.040,start[1],z),(0.,-1.,0.),float(length))
    valid=bool(guard.get('hit')) and '/housing/' in str(guard.get('collision',''))
    hit=query.sweep_sphere_closest(.0035,start,(0.,-1.,0.),float(length))
    return dict(valid=valid,clear=valid and not bool(hit.get('hit')),diameter_m=.007,
                start=list(start),length_m=float(length),blocked_by=str(hit.get('collision','')),
                calibration_collision=str(guard.get('collision','')))

def oil_rays():
    if not hasattr(L,'HOUSING'):return None
    from omni.physx import get_physx_scene_query_interface
    query=get_physx_scene_query_interface();out=[]
    x,y,z=L.HOUSING
    calibration=query.raycast_closest((float(x),float(y+L.HOUSING_OUT+.003),float(z-.027)),(0.,-1.,0.),.12)
    if not calibration['hit']:
        raise RuntimeError('Ray diagnostic invalid: known solid housing base not detected')
    for dx in np.linspace(-.0035,.0035,5):
        for dz in np.linspace(-.0035,.0035,5):
            if dx*dx+dz*dz>.0035**2:continue
            start=(float(x+dx),float(y+L.HOUSING_OUT+.003),float(z+dz))
            hit=query.raycast_closest(start,(0.,-1.,0.),float(L.HOUSING_OUT-L.R_IN+.004))
            out.append(dict(dx=float(dx),dz=float(dz),hit=bool(hit['hit']),
                            collision=str(hit.get('collision','')),rigid_body=str(hit.get('rigidBody',''))))
    return dict(calibration_hit=str(calibration.get('collision','')),clear=sum(not row['hit'] for row in out),total=len(out),rays=out,
                note='Physics scene rays test collider openings, NOT oil flow or policy perception.')

def install_scene(output):
    from yam_arena import scene
    from isaaclab.sim.utils import clone
    from isaaclab.sim.spawners.shapes.shapes import spawn_cuboid
    from isaaclab_physx.sim.schemas import PhysxCollisionPropertiesCfg
    from .geometry import decorate
    original_object,original_states=scene.task_object,scene.object_states
    original_cfg=scene.env_cfg_callback
    def cfg_callback(*args,**kwargs):
        callback=original_cfg(*args,**kwargs)
        def configure(cfg):
            cfg=callback(cfg)
            cfg.sim.enable_scene_query_support=True
            return cfg
        return configure
    scene.env_cfg_callback=cfg_callback
    names={o.name for o in L.OBJECTS}
    def task_object(spec):
        obj=original_object(spec)
        if spec.name in names:
            @clone
            def spawn(path,cfg,*args,**kwargs):
                import carb
                carb.settings.get_settings().set_bool('/physics/disableContactProcessing',False)
                prim=spawn_cuboid.__wrapped__(path,cfg,*args,**kwargs)
                decorate(prim,spec.name);return prim
            obj.spawner_cfg.func=spawn
            obj.spawner_cfg.collision_props=PhysxCollisionPropertiesCfg(contact_offset=.0002,rest_offset=0)
            obj.object_cfg.spawn=obj.spawner_cfg
        return obj
    def states(env,names):
        objects=original_states(env,names);base=env.unwrapped
        row=dict(step=len(TRACE),contact_samples=MONITOR.samples,objects={},joints={})
        for name in L.MOVING:
            obj=base.scene[name]
            row['objects'][name]=dict(pose_xyzw=obj.data.root_pose_w.torch[0].detach().cpu().tolist(),
                linear_velocity=obj.data.root_lin_vel_w.torch[0].detach().cpu().tolist(),
                angular_velocity=obj.data.root_ang_vel_w.torch[0].detach().cpu().tolist())
        for name in ('left_robot','right_robot'):
            obj=base.scene[name];row['joints'][name]=dict(names=obj.joint_names,q=obj.data.joint_pos.torch[0].detach().cpu().tolist())
        row['contact_interval']=MONITOR.consume_interval()
        # The stack's CPU scene queries missed a rotated/moved sleeve in a real
        # negative control. Keep them for diagnostics, never for the score.
        row['passage']=live_passage(row['objects']['sleeve']['pose_xyzw'])
        if len(TRACE)%20==0:row['cpu_scene_sweep_advisory']=passage_check()
        row['evaluation']=EVALUATOR.update(row)
        objects['__evaluation__']=row['evaluation']
        if len(TRACE)%20==0 and hasattr(L,'HOUSING'):
            row['oil_rays']=oil_rays();RAYS.append(dict(step=len(TRACE),**row['oil_rays']))
        TRACE.append(row);MONITOR.step=len(TRACE)
        if len(TRACE)%20==0:
            (output/'progress.json').write_text(json.dumps(dict(step=len(TRACE),samples=MONITOR.samples,
                monitor_valid=MONITOR.ready and MONITOR.error is None,robot_contact_reports=len(MONITOR.events),
                evaluation=row['evaluation']))+'\n')
        return objects
    scene.task_object=task_object;scene.object_states=states

def install(output):
    from yam_arena import inspect_embodiment as official
    original_ensure=official.YamArenaEmbodiment._ensure_sim;original_reset=official.YamArenaEmbodiment.reset
    installed=False;resets=0;output=Path(output)
    def ensure(self,goal):
        nonlocal installed
        if self.rig!=official.station.rig('calibrated') or (self.cam_width,self.cam_height)!=(320,176):
            raise RuntimeError('Original calibrated RGB required')
        if self.info.control_hz!=20 or self.object_jitter!=0:raise RuntimeError('Fixed 20 Hz required')
        if not installed:
            from isaaclab.app import AppLauncher
            assert official._APP is None
            official._APP=AppLauncher(headless=True,device=self.device,enable_cameras=True,multi_gpu=False).app
            atexit.register(official._close_app_at_exit);install_scene(output);installed=True
        original_ensure(self,goal)
        import carb,omni.usd
        carb.settings.get_settings().set_bool('/physics/disableContactProcessing',False)
        if not MONITOR.ready:MONITOR.bind(omni.usd.get_context().get_stage())
        (output/'contact_contract.json').write_text(json.dumps(MONITOR.contract,indent=2)+'\n')
    def reset(self,*args,**kwargs):
        nonlocal resets
        if resets:raise RuntimeError('Prototype requires fresh-process reset; joint rearming not validated yet')
        resets+=1;obs=original_reset(self,*args,**kwargs)
        import cv2
        for name,rgb in obs.images.items():
            np.save(output/f'setup_{name}.npy',rgb)
            cv2.imwrite(str(output/f'setup_{name}.png'),cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR))
        import omni.usd
        stage=omni.usd.get_context().get_stage()
        (output/'scene_authored.usda').write_text(stage.GetRootLayer().ExportToString())
        return obs
    official.YamArenaEmbodiment._ensure_sim=ensure;official.YamArenaEmbodiment.reset=reset
    def save():
        (output/'diagnostics.json').write_text(json.dumps(dict(frames=TRACE,contacts=MONITOR.report(),oil_rays=RAYS),indent=2)+'\n')
    return save
