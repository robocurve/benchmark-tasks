"""Compound contact geometry and trajectory telemetry for the official bimanual YAM rig."""
from dataclasses import dataclass
from pathlib import Path
import atexit
import json
import math
import os
import numpy as np
from yam_arena.task_file import Box

@dataclass(frozen=True)
class Compound(Box):
    parts: tuple = ()
    grasp_offset: tuple = (0.,0.,0.)

    def _check(self):
        super()._check()
        if not self.parts:
            raise ValueError('Compound requires physical parts')
        for p in self.parts:
            if len(p['pos'])!=3 or len(p['size'])!=3 or min(p['size'])<=0:
                raise ValueError('Invalid compound part')

def _usd(spec,directory):
    from pxr import Gf, Usd, UsdGeom, UsdPhysics, PhysxSchema
    directory.mkdir(parents=True,exist_ok=True)
    path=directory/(spec.name+'.usda')
    stage=Usd.Stage.CreateNew(str(path))
    UsdGeom.SetStageUpAxis(stage,UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage,1.)
    root=UsdGeom.Xform.Define(stage,'/part').GetPrim()
    stage.SetDefaultPrim(root)
    if spec.free:
        UsdPhysics.RigidBodyAPI.Apply(root).CreateRigidBodyEnabledAttr(True)
        UsdPhysics.MassAPI.Apply(root).CreateMassAttr(float(spec.mass))
        PhysxSchema.PhysxRigidBodyAPI.Apply(root).CreateEnableCCDAttr(True)
    for i,item in enumerate(spec.parts):
        g=UsdGeom.Cube.Define(stage,f'/part/p{i}')
        g.CreateSizeAttr(1.)
        g.AddTranslateOp().Set(Gf.Vec3d(*item['pos']))
        g.AddScaleOp().Set(Gf.Vec3f(*item['size']))
        g.CreateDisplayColorAttr([Gf.Vec3f(*item.get('color',spec.color))])
        UsdPhysics.CollisionAPI.Apply(g.GetPrim()).CreateCollisionEnabledAttr(True)
        c=PhysxSchema.PhysxCollisionAPI.Apply(g.GetPrim())
        c.CreateContactOffsetAttr(.001)
        c.CreateRestOffsetAttr(0.)
    stage.GetRootLayer().Save()
    return path

def _install_spawn():
    from yam_arena import scene
    if getattr(scene.task_object,'_industrial',False): return
    original=scene.task_object
    original_states=scene.object_states
    specs={}
    def spawn(spec):
        if not isinstance(spec,Compound): return original(spec)
        specs[spec.name]=spec
        from isaaclab_arena.assets.object import Object
        from isaaclab_arena.utils.pose import Pose
        # Use the enum already imported by the upstream scene module.
        path=_usd(spec,Path(os.environ.get('YAM_GEOMETRY_DIR','generated_geometry')))
        q=(0.,0.,math.sin(spec.yaw/2),math.cos(spec.yaw/2))
        return Object(name=spec.name,usd_path=str(path.resolve()),
                      object_type=scene.ObjectType.RIGID if spec.free else scene.ObjectType.BASE,
                      initial_pose=Pose(position_xyz=spec.pos,rotation_xyzw=q))
    spawn._industrial=True
    scene.task_object=spawn
    def object_states(env,names):
        from yam_arena.task_file import ObjectState
        from yam_arena.urdf import quat_xyzw_to_matrix
        out=original_states(env,names)
        base=env.unwrapped
        origin=base.scene.env_origins[0].detach().cpu().numpy()
        signs=np.array([[x,y,z] for x in [-1,1] for y in [-1,1] for z in [-1,1]])
        for name in names:
            if name not in specs:continue
            pose=base.arena_world.get_pose_w(name)[0].detach().cpu().numpy()
            rotation=quat_xyzw_to_matrix(pose[3:]);points=[]
            for part in specs[name].parts:
                vertices=signs*np.asarray(part['size'])/2+np.asarray(part['pos'])
                points.extend(vertices@rotation.T+pose[:3]-origin)
            vertices=np.asarray(points);state=out[name]
            # Retain Arena's attached reference centroid. Only bounds use exact authored boxes.
            out[name]=ObjectState(state.pos,state.quat_xyzw,state.lin_vel,state.ang_vel,vertices.min(axis=0),vertices.max(axis=0))
        return out
    scene.object_states=object_states

def embodiment(**kwargs):
    from yam_arena import inspect_embodiment as core
    class IndustrialYam(core.YamArenaEmbodiment):
        def _ensure_sim(self,goal):
            if core._APP is None:
                from isaaclab.app import AppLauncher
                os.environ.setdefault('OMNI_KIT_ACCEPT_EULA','YES')
                opts=dict(headless=self.headless,device=self.device,enable_cameras=True)
                if not self.headless:
                    opts['visualizer']=['kit']
                    os.environ['DISPLAY']=core.window_display(os.environ)
                core._APP=AppLauncher(**opts).app
                atexit.register(core._close_app_at_exit)
            _install_spawn()
            return super()._ensure_sim(goal)

        def reset(self,scene,*,seed=None):
            self._telemetry=None
            obs=super().reset(scene,seed=seed)
            if self.task and hasattr(self.task.module,'reset_history'):
                self.task.module.reset_history()
            destination=os.environ.get('YAM_TELEMETRY')
            if destination:
                self._telemetry=Path(destination)
                self._telemetry.parent.mkdir(parents=True,exist_ok=True)
                self._telemetry.write_text('')
            return obs

        def _task_step(self):
            from yam_arena.scene import object_states
            states=object_states(self._env,self.task.names)
            if hasattr(self.task.module,'observe'):
                self.task.module.observe(states,self._t,self._base)
            result=super()._task_step()
            if self._telemetry:
                record=dict(t=float(self._t),objects={n:dict(pos=s.pos.tolist(),quat=s.quat_xyzw.tolist(),
                            lin_vel=s.lin_vel.tolist(),ang_vel=s.ang_vel.tolist(),low=s.low.tolist(),high=s.high.tolist())
                            for n,s in states.items()},info=result.info)
                with self._telemetry.open('a') as f:
                    f.write(json.dumps(record,default=lambda x:x.item() if isinstance(x,np.generic) else str(x))+'\n')
            return result
    return IndustrialYam(**kwargs)
