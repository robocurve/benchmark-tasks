"""Prototype compound rigid body and private registration measurements."""
import atexit,json
from pathlib import Path
import numpy as np
from . import layout as L
from .contact import Monitor
from .scoring import Evaluator
HISTORY=Monitor();EVALUATOR=Evaluator();TRACE=[]
OUTPUT=None
FIXED_STATES=None
RECORD_UNTIL_S=None

def install_scene():
    from yam_arena import scene
    from isaaclab.sim.utils import clone
    from isaaclab.sim.spawners.shapes.shapes import spawn_cuboid
    from isaaclab_physx.sim.schemas import PhysxCollisionPropertiesCfg
    from .usd_helpers import cube
    original_object,original_states=scene.task_object,scene.object_states
    def task_object(spec):
        obj=original_object(spec)
        if spec.name=='goal_board':
            @clone
            def marked_board(path,cfg,*args,**kwargs):
                prim=spawn_cuboid.__wrapped__(path,cfg,*args,**kwargs)
                for i,(x,y) in enumerate(L.WINDOWS):
                    for axis,size in [('x',(.014,.0024,.0006)),('y',(.0024,.014,.0006))]:
                        cube(prim.GetStage(),str(prim.GetPath())+f'/cross_{i}_{axis}',
                             (x+L.GOAL[0]-L.BOARD_CENTER[0],y+L.GOAL[1]-L.BOARD_CENTER[1],.0103),
                             size,(.01,.65,.85),False,emission=(.002,.25,.40))
                return prim
            obj.spawner_cfg.func=marked_board
            obj.object_cfg.spawn=obj.spawner_cfg
        if spec.name=='plate':
            @clone
            def spawn(path,cfg,*args,**kwargs):
                import carb
                from pxr import Usd,UsdPhysics,UsdShade,Sdf
                carb.settings.get_settings().set_bool('/physics/disableContactProcessing',False)
                prim=spawn_cuboid.__wrapped__(path,cfg,*args,**kwargs)
                prim.AddAppliedSchema('PhysxContactReportAPI')
                prim.CreateAttribute('physxContactReport:threshold',Sdf.ValueTypeNames.Float).Set(0.)
                stage=prim.GetStage();root=str(prim.GetPath())
                for name,pos,size,color,collision in L.CHILDREN:
                    cube(stage,root+'/'+name,pos,size,color,collision)
                material=UsdShade.Material.Define(stage,root+'/grip_material')
                physics=UsdPhysics.MaterialAPI.Apply(material.GetPrim())
                physics.CreateStaticFrictionAttr(.9);physics.CreateDynamicFrictionAttr(.7)
                physics.CreateRestitutionAttr(0.)
                for p in Usd.PrimRange(prim):
                    if p.HasAPI(UsdPhysics.CollisionAPI):
                        p.AddAppliedSchema('PhysxCollisionAPI')
                        p.CreateAttribute('physxCollision:contactOffset',Sdf.ValueTypeNames.Float).Set(.001)
                        p.CreateAttribute('physxCollision:restOffset',Sdf.ValueTypeNames.Float).Set(0.)
                        UsdShade.MaterialBindingAPI.Apply(p).Bind(material,materialPurpose='physics')
                return prim
            obj.spawner_cfg.func=spawn
            obj.spawner_cfg.collision_props=PhysxCollisionPropertiesCfg(contact_offset=.001,rest_offset=0)
            obj.object_cfg.spawn=obj.spawner_cfg
        return obj
    def object_states(env,names):
        global FIXED_STATES
        # These objects are static physics colliders. Cache their unchanging private
        # scorer states; do not query every fixed guide segment across the GPU each tick.
        if FIXED_STATES is None:FIXED_STATES=original_states(env,[n for n in names if n!='plate'])
        states=dict(FIXED_STATES)
        if 'plate' in names:states.update(original_states(env,['plate']))
        if 'plate' not in names:return states
        base=env.unwrapped;obj=base.scene['plate']
        pose=obj.data.root_pose_w.torch[0].detach().cpu().numpy()
        linear=obj.data.root_lin_vel_w.torch[0].detach().cpu().numpy()
        angular=obj.data.root_ang_vel_w.torch[0].detach().cpu().numpy()
        robots={}
        for name in ('left_robot','right_robot'):
            robot=base.scene[name]
            robots[name]=dict(joint_names=robot.joint_names,joint_pos=robot.data.joint_pos.torch[0].detach().cpu().tolist(),
                             body_names=robot.body_names,body_pos=robot.data.body_pos_w.torch[0].detach().cpu().tolist())
        result=EVALUATOR.update(pose,HISTORY)
        states['__evaluation__']=result
        # Demo capture only: continue through withdrawal while retaining the real
        # score/contact history. Dream has no extra termination delay.
        states['__stop_ready__']=RECORD_UNTIL_S is None or (len(TRACE)+1)/20>=RECORD_UNTIL_S
        TRACE.append(dict(step=len(TRACE),**result,robots=robots,contact_samples=HISTORY.samples,
                          linear_velocity=linear.tolist(),angular_velocity=angular.tolist()))
        HISTORY.step=len(TRACE)
        if OUTPUT is not None and len(TRACE)%20==0:
            (OUTPUT/'progress.json').write_text(json.dumps(dict(step=len(TRACE),**result,
                contact_samples=HISTORY.samples,prohibited_reports=len(HISTORY.violations),
                peak_grasp_force_n=HISTORY.peak_grasp_force,
                recent_violation=HISTORY.violations[-1] if HISTORY.violations else None)))
        return states
    scene.task_object=task_object;scene.object_states=object_states

def install(output,record_until_s=None):
    global OUTPUT,RECORD_UNTIL_S
    from yam_arena import inspect_embodiment as official
    original_ensure=official.YamArenaEmbodiment._ensure_sim
    original_reset=official.YamArenaEmbodiment.reset
    installed=False;output=Path(output);output.mkdir(parents=True,exist_ok=True)
    OUTPUT=output;RECORD_UNTIL_S=record_until_s
    def ensure(self,goal):
        nonlocal installed
        if self.rig!=official.station.rig('calibrated') or (self.cam_width,self.cam_height)!=(320,176):
            raise RuntimeError('Original calibrated cameras required')
        if self.info.control_hz!=20 or self.object_jitter!=0:raise RuntimeError('Fixed 20Hz required')
        if not installed:
            from isaaclab.app import AppLauncher
            assert official._APP is None
            official._APP=AppLauncher(headless=True,device=self.device,enable_cameras=True,multi_gpu=False).app
            atexit.register(official._close_app_at_exit);install_scene();installed=True
        original_ensure(self,goal)
        import carb,omni.usd
        carb.settings.get_settings().set_bool('/physics/disableContactProcessing',False)
        if not HISTORY.ready:HISTORY.bind(omni.usd.get_context().get_stage())
        (output/'contact_contract.json').write_text(json.dumps(HISTORY.contract,indent=2))
    def reset(self,*args,**kwargs):
        global FIXED_STATES
        FIXED_STATES=None
        EVALUATOR.reset();HISTORY.reset();TRACE.clear()
        obs=original_reset(self,*args,**kwargs)
        import cv2
        for name,rgb in obs.images.items():
            np.save(output/f'setup_{name}.npy',rgb)
            cv2.imwrite(str(output/f'setup_{name}.png'),cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR))
        return obs
    official.YamArenaEmbodiment._ensure_sim=ensure;official.YamArenaEmbodiment.reset=reset
    def save():
        (output/'evaluation_trace.json').write_text(json.dumps(dict(steps=TRACE,contacts=HISTORY.events,
            violations=HISTORY.violations,contact_samples=HISTORY.samples,monitor_error=HISTORY.error,
            monitor_ready=HISTORY.ready,peak_grasp_force_n=HISTORY.peak_grasp_force),indent=2))
    return save
