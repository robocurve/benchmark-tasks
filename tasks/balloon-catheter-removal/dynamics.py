"""Arena adapter for the same native SceneRuntime used by manual collection."""
import itertools
import numpy as np
from mechanics import rotation
from scoring import INITIAL
from layout import VALVE_QUATERNION

# Conservative local geometry bounds from the original task-local builder.
BOUNDS={'catheter':((-.015,-.104,-.015),(.015,.172,.022)),
        'valve':((-.020,-.008,-.008),(.044,.028,.018)),
        'barrel':((-.049,-.0215,-.014),(.062,.0215,.014)),
        'plunger':((-.075,-.016,-.0125),(.039,.016,.0125))}

class Dynamics:
    def __init__(self,env):
        import omni.usd
        from scene_runtime import SceneRuntime
        self.env=env.unwrapped;self.stage=omni.usd.get_context().get_stage()
        roots=[str(p.GetPath()) for p in self.stage.Traverse() if str(p.GetPath()).endswith('/catheter_station')]
        if len(roots)!=1:raise RuntimeError('One native catheter station required')
        self.root=roots[0];self.runtime=SceneRuntime(self.stage,self.env.sim,self.root)

    def reset(self):
        # Standard between-trial reset only. No pose writes occur during a task.
        import warp as wp
        from pxr import UsdPhysics
        from mechanics import DrainState
        rt=self.runtime
        if rt.connection is not None:rt.connection.CreateJointEnabledAttr(False)
        rt.connection=None;rt.state=DrainState();rt._aligned_frames=0
        for key,view in rt.views.items():
            device=view.get_transforms().device
            view.set_transforms(wp.array(np.asarray([[*INITIAL[key],*(VALVE_QUATERNION if key=='valve' else (0,0,0,1))]],dtype=np.float32),device=device),
                                wp.array([0],dtype=wp.int32,device=device))
            view.set_velocities(wp.zeros((1,6),dtype=wp.float32,device=device),wp.array([0],dtype=wp.int32,device=device))
        rt.drive.CreateStiffnessAttr(0.);rt.drive.CreateDampingAttr(.5)
        UsdPhysics.CollisionAPI(rt.inflated.GetPrim()).CreateCollisionEnabledAttr(True);rt.inflated.CreateVisibilityAttr('inherited')
        UsdPhysics.CollisionAPI(rt.collapsed.GetPrim()).CreateCollisionEnabledAttr(False);rt.collapsed.CreateVisibilityAttr('invisible')
        rt.collapsed_mode=False;rt.balloon=rt.inflated

    def before_step(self,step,action):pass

    def after_step(self,step,objects,root_poses,robot):
        state=self.runtime.update(.05)
        forces={}
        for side in ('left','right'):
            fingers=[]
            for finger in ('tip_left','tip_right'):
                matrix=self.env.scene[side+'_'+finger+'_device'].data.force_matrix_w
                tensor=matrix.torch if hasattr(matrix,'torch') else matrix
                pair=tensor.detach().cpu().numpy().reshape(2,3)
                fingers.append(np.linalg.norm(pair,axis=1).tolist())
            forces[side]=fingers
        state['tip_device_forces_N']=forces
        return state

    def read_object_states(self):
        from yam_arena.task_file import ObjectState
        poses=self.runtime.poses();states={}
        for name,p in poses.items():
            lo,hi=BOUNDS[name];corners=np.array(list(itertools.product(*zip(lo,hi))))
            world=corners@rotation(p[3:]).T+p[:3]
            velocity=self.runtime.views[name].get_velocities().numpy()[0]
            states[name]=ObjectState(pos=(world.min(0)+world.max(0))/2,quat_xyzw=p[3:].copy(),
                lin_vel=velocity[:3].copy(),ang_vel=velocity[3:].copy(),low=world.min(0),high=world.max(0))
        return states,poses
