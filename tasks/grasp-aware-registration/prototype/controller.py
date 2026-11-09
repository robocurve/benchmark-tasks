"""Known-layout approach with optional RGB alignment; not autonomous grasp planning."""
import numpy as np
from yam_arena import ik,station
from yam_arena.constants import READY,FINGER_OPEN,TCP_OFFSET
from yam_arena.urdf import quat_xyzw_to_matrix
from yam_arena.demo import ArmPlan,Waypoint

from . import layout as L
MODE='setup'
INITIAL_GRASP='central'
LOG=[]

def waypoints(mode):
    if mode=='regrasp_visual':
        from .recovery import prefix
        return prefix(waypoints('central'))+waypoints('end_visual')
    grasp=mode.removesuffix('_visual')
    offset=np.array(L.GRASPS[grasp]);pick=np.array(L.HOME)+offset+[0,0,L.TCP_HEIGHT];goal=np.array(L.GOAL)+offset+[0,0,L.TCP_HEIGHT]
    hover=max(pick[2],goal[2])+.055
    points=[Waypoint(2.,(*pick[:2],hover),FINGER_OPEN,'approach chosen grasp'),
            Waypoint(2.,tuple(pick),FINGER_OPEN,'descend'),
            Waypoint(1.5,tuple(pick),-.006,'grasp'),
            Waypoint(2.,(*pick[:2],hover),-.006,'lift'),
            Waypoint(2.,(pick[0],goal[1],hover),-.006,'pass beside screen post'),
            Waypoint(2.,(*goal[:2],hover),-.006,'advance into registration area'),
            Waypoint(2.,tuple(goal+[0,0,.008]),-.006,'near-board inspection'),
            Waypoint(8.,tuple(goal+[0,0,.008]),-.006,'hold for view audit'),
            Waypoint(2.,tuple(goal),-.006,'seat'),
            Waypoint(1.,tuple(goal),FINGER_OPEN,'release'),
            Waypoint(2.,(*goal[:2],hover),FINGER_OPEN,'withdraw'),
            Waypoint(2.,(*goal[:2],hover),FINGER_OPEN,'rest')]
    if mode.endswith('_visual'):
        points.insert(9,Waypoint(3.,tuple(goal),-.006,'wait for observer withdrawal'))
    return points

def observer_action(t,observer,visual=False):
    # Inspect while the plate is stationary; leave before the manipulator withdraws.
    enter=ik.smoothstep((t-13.5)/3.)
    leave=ik.smoothstep((t-(23.5 if visual else 18.5))/3.)
    s=enter*(1-leave)
    return (1-s)*np.asarray(READY)+s*np.asarray(observer)

def recovery_offset(mode):
    if mode!='regrasp_visual':return 0.
    from .recovery import prefix
    return sum(p.duration for p in prefix(waypoints('central')))

def observer_for_mode(t,mode,observer):
    offset=recovery_offset(mode)
    if offset and t<offset:
        from .recovery import initial_observer
        return observer_action(t,initial_observer())
    return observer_action(t-offset,observer,visual=mode.endswith('_visual'))

def install(output,observer):
    from yam_arena.scripted import YamScriptedPolicy
    from inspect_robots.types import Action,ActionChunk
    from inspect_robots_yam.packing import pack
    original=YamScriptedPolicy.reset
    def reset(self,scene):
        original(self,scene);LOG.clear()
        self.path_time=0.;self.correction=np.zeros(3)
        self.active_mode=MODE
        self.branch=None
        if MODE=='adaptive_visual':
            from .view_decision import ViewDecision
            self.view_decision=ViewDecision()
            self.active_mode=INITIAL_GRASP+'_visual'
        if MODE not in ('setup','view'):self.plan=ArmPlan(waypoints(self.active_mode),station.BASES['right'])
    def act(self,observation):
        t=float(observation.state_time)+.05
        measurement=None;paused=False;decision=None;decision_pending=False
        if MODE=='setup':left=right=np.r_[READY,1.];label='setup'
        elif MODE=='view':
            s=ik.smoothstep(t/3.);left=np.r_[(1-s)*np.asarray(READY)+s*np.asarray(observer),1.]
            right=np.r_[READY,1.];label='inspect fixed targets without manipulating'
        else:
            visual=MODE.endswith('_visual')
            if visual:
                from .vision import alignment
                t=self.path_time+.05
                active_mode=getattr(self,'active_mode',MODE)
                if MODE=='adaptive_visual' and self.branch is None and t>=16.8:
                    decision=self.view_decision.update(observation.images.get('left_cam'))
                    measurement=decision['alignment']
                    if decision['choice']=='continue':self.branch='continue'
                    elif decision['choice']=='recover' and INITIAL_GRASP=='central':
                        q=self.plan.q.copy()
                        self.active_mode=active_mode='regrasp_visual'
                        self.plan=ArmPlan(waypoints(active_mode),station.BASES['right'])
                        self.plan.q=q
                        self.branch='recover'
                    else:
                        t=self.path_time;decision_pending=True
                local_t=t-recovery_offset(active_mode)
                if not decision_pending and 16.8<=local_t<=23.5:
                    measurement=alignment(observation.images.get('left_cam'))
                    if not measurement['valid']:t=self.path_time;paused=True
                    else:
                        joints=np.asarray(observation.state['joint_pos'],dtype=float)
                        tcp,grip_rot=ik.tcp_pose(joints[:6],station.BASES['left'])
                        cam=station.rig().wrist_cams['left']
                        cam_pos=tcp+grip_rot@(np.asarray(cam.pos)-TCP_OFFSET)
                        cam_rot=grip_rot@quat_xyzw_to_matrix(np.asarray(cam.rot_xyzw))
                        feature_z=self.plan.target(t)[0][2]-L.GRASPS['end'][2]-L.TCP_HEIGHT+.0093
                        depth=float(cam_pos[2]-feature_z)
                        error=np.asarray(measurement['error_px'])
                        ray=np.array([error[0]*depth/(cam.lens.fx*.5),-error[1]*depth/(cam.lens.fy*176/360),0.])
                        delta=.12*(cam_rot@ray);delta[2]=0.
                        refine=np.linalg.norm(error)>.25
                        if active_mode=='regrasp_visual' or MODE=='adaptive_visual':
                            from .vision import alignment_world_error
                            world_error=alignment_world_error(measurement,cam_pos,cam_rot,cam.lens,feature_z,L.BOARD_TOP+.0005)
                            measurement['world_error_m']=world_error.tolist()
                            delta=.12*world_error
                            refine=np.linalg.norm(world_error)>.0002
                        length=np.linalg.norm(delta)
                        if length>.00025:delta*=.00025/length
                        if refine:self.correction+=delta
                        self.correction[:2]=np.clip(self.correction[:2],-.006,.006)
                self.path_time=t
            right,label=self.plan.action7(t)
            if visual and np.linalg.norm(self.correction)>0:
                pos,rot,_,_=self.plan.target(t)
                q,pe,re=ik.solve(pos+self.correction,ik.matrix_to_quat_xyzw(rot),self.plan.q,base_pos=station.BASES['right'],iters=80)
                if pe>.001 or re>.02:raise RuntimeError('Visual correction exceeds checked IK envelope')
                right[:6]=q;self.plan.q=q
            active_mode=getattr(self,'active_mode',MODE)
            observer_pose=observer
            if active_mode=='central_visual':
                from .recovery import initial_observer
                observer_pose=initial_observer()
            left=np.r_[observer_for_mode(t,active_mode,observer_pose),1.]
            # Initial central view is recorded, but does not select this scripted recovery.
            if active_mode=='regrasp_visual' and 16.8<=t<=18.5:
                from .vision import alignment
                measurement=alignment(observation.images.get('left_cam'))
        LOG.append(dict(time_s=t,wall_time_s=float(observation.state_time)+.05,label=label,
                        privileged_known_layout=True,rgb_measurement=measurement,
                        view_decision=decision,decision_pending=decision_pending,
                        selected_branch=getattr(self,'branch',None),active_mode=getattr(self,'active_mode',MODE),
                        paused_for_missing_features=paused,visual_correction_m=self.correction.tolist()))
        return ActionChunk(actions=[Action(data=pack(left,right))],control_hz=20)
    YamScriptedPolicy.reset=reset;YamScriptedPolicy.act=act
