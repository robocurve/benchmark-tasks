"""Scripted pickup followed by measured-pose Cartesian control of the two arms."""

import numpy as np
from inspect_robots.policy import PolicyBase, PolicyInfo
from inspect_robots.types import Action, ActionChunk
from inspect_robots_yam.config import action_box

from yam_arena import ik, station
from yam_arena.constants import READY, FINGER_OPEN, FINGER_CLOSED
from yam_arena.demo import ArmPlan, Waypoint, grip_rotation
from yam_arena.urdf import axis_angle_matrix as rotation, quat_xyzw_to_matrix

import runtime


class WoodDrillDemo(PolicyBase):
    def __init__(self):
        low, high = np.r_[ik.LOWER, 0.], np.r_[ik.UPPER, 1.]
        self.info = PolicyInfo(name="wood_drill_demo", action_space=action_box(np.r_[low,low], np.r_[high,high]))
        self.stage = "pickup"
        self.contact_since = None
        self.stage_since = 0.
        self._right_q = np.array(READY)
        self._left_q = np.array(READY)
        self._left_pos = None
        self._right_pos = None
        self._right_rot = None
        self._left_rot = None
        self._bias = {side: np.zeros(6) for side in ('left','right')}
        self.last_ik = {}
        self._orientation_selected = False
        self._feed_z = None
        self._trigger_push = .014
        self._contact_z = None
        self._withdraw_z = None

    def bind(self, info):
        self.dt = 1. / info.control_hz

    def reset(self, scene):
        emb = runtime.ACTIVE
        self.task = emb.task.module
        top = self.task.TOP
        x, y = .245, -.260
        raised, grasp = (x,y,top+.14), (x,y,top+.03)
        self.pickup = ArmPlan([
            Waypoint(3.,raised,FINGER_OPEN,"approach"),
            Waypoint(1.,raised,FINGER_OPEN,"settle"),
            Waypoint(2.5,grasp,FINGER_OPEN,"descend"),
            Waypoint(2.,grasp,FINGER_CLOSED,"grasp"),
            Waypoint(1.,grasp,FINGER_CLOSED,"settle grasp"),
            Waypoint(3.,raised,FINGER_CLOSED,"lift"),
            Waypoint(2.,raised,FINGER_CLOSED,"hold"),
        ], station.BASES["right"])
        self.stage = "pickup"
        self.stage_since = 0.
        self.contact_since = None
        self._right_pos = self._right_rot = self._left_pos = self._left_rot = None
        self._bias = {side: np.zeros(6) for side in ('left','right')}
        self._orientation_selected = False
        self._feed_z = None
        self._trigger_push = .014
        self._contact_z = None
        self._withdraw_z = None
        self.left_rotation = rotation(np.array([1.,0,0]),np.pi/2) @ rotation(np.array([0.,0,1]),np.pi/2)
        self.drill_rotation = rotation(np.array([0.,1,0]),-np.pi/2)

    def _arm(self, side, pos, rot, current_q, gripper):
        q, pe, re = ik.solve(pos,ik.matrix_to_quat_xyzw(rot),current_q,
                             base_pos=station.BASES[side],iters=80)
        if pe > .003 or re > .04:
            alternative, ae, ar = ik.solve(pos,ik.matrix_to_quat_xyzw(rot),READY,
                                           base_pos=station.BASES[side],iters=180)
            if ae+.15*ar < pe+.15*re:
                if np.max(np.abs(alternative-q))>.7:
                    self._bias[side][:]=0.
                q,pe,re=alternative,ae,ar
        self.last_ik[side] = (pe, re)
        from yam_arena.runtime import arm_q
        measured = arm_q(runtime.ACTIVE._base.scene[side+'_robot'])
        self._bias[side] = np.clip(self._bias[side] + .035*(q-measured), -.5, .5)
        return q, np.r_[np.clip(q+self._bias[side],ik.LOWER,ik.UPPER),gripper]

    def act(self, observation):
        emb = runtime.ACTIVE
        t = float(observation.state_time) + self.dt
        left = np.r_[READY,1.]
        if self.stage == "pickup" and t <= self.pickup.duration:
            right, _ = self.pickup.action7(t)
            self._right_q = right[:6]
        else:
            if self.stage == "pickup":
                self.stage = "orient"
                self.stage_since = t
                print('[demo] rotate and position drill',flush=True)
            objects = emb.object_states()
            d = objects['drill']
            r = quat_xyzw_to_matrix(d.quat_xyzw)
            origin = np.asarray(d.pos) - r @ self.task.MODEL_CENTRE
            from yam_arena.runtime import arm_q, tcp_world, link_poses
            right_robot, left_robot = emb._base.scene['right_robot'],emb._base.scene['left_robot']
            rq = arm_q(right_robot)
            tcp, gr = ik.tcp_pose(rq,station.BASES['right'])
            local_tcp = r.T @ (tcp-origin)
            if not self._orientation_selected:
                choices=[]
                for yaw in np.linspace(-np.pi,np.pi,25):
                    dr=rotation(np.array([0.,0,1]),yaw) @ rotation(np.array([0.,1,0]),-np.pi/2)
                    gr_goal=dr @ r.T @ gr
                    quality=0.
                    for clearance in (-.014,0.,.060):
                        tip=np.array([*self.task.TARGET_XY,self.task.SURFACE_Z+clearance])
                        candidate=tip+dr@(local_tcp-self.task.BIT_TIP_LOCAL)
                        q,e,a=ik.solve(candidate,ik.matrix_to_quat_xyzw(gr_goal),rq,
                                       base_pos=station.BASES['right'],iters=160)
                        quality+=e+.15*a
                    trigger=np.array([*self.task.TARGET_XY,self.task.SURFACE_Z])+dr@(self.task.TRIGGER_LOCAL-self.task.BIT_TIP_LOCAL)
                    _,e,a=ik.solve(trigger+np.array([0.,.020,-.009]),ik.matrix_to_quat_xyzw(self.left_rotation),
                                   base_pos=station.BASES['left'],iters=160)
                    quality+=e+.15*a+.0001*abs(yaw)
                    choices.append((quality,yaw,dr))
                feasible=[item for item in choices if item[0]<.003]
                quality,yaw,self.drill_rotation=min(choices,key=lambda item:item[0])
                print('[demo] feasible tool yaws:',[round(np.degrees(item[1])) for item in feasible],flush=True)
                print(f'[demo] selected vertical tool yaw={np.degrees(yaw):.1f}deg, reach cost={quality:.5f}',flush=True)
                self._orientation_selected=True
            m = self.task.measure(objects,emb.robot_state())
            if self.stage == 'press' and m['trigger_pressed'] and m.get('spindle_rpm',0)>300:
                self.stage='recenter';self.stage_since=t
                print('[demo] centre bit while holding trigger at a controlled stroke',flush=True)
            if self.stage == 'recenter' and m['target_error_m']<.0012 and m['tilt_deg']<2 and t-self.stage_since>1.:
                self.stage='feed';self.stage_since=t
                self._feed_z=self.task.SURFACE_Z-.0007
                print('[demo] feed rotating bit into wood against resistance',flush=True)
            if self.stage == 'feed' and m.get('qualified_depth_m',0)>=.0105:
                self.stage='withdraw';self.stage_since=t
                self._withdraw_z=float(m['tip_m'][2])
                print('[demo] hole depth reached; withdraw rotating bit',flush=True)
            if self.stage == 'withdraw' and m['surface_error_m']>.026:
                self.stage='release';self.stage_since=t
                print('[demo] release trigger and leave hole visible',flush=True)
            desired_tip = np.array([*self.task.TARGET_XY,self.task.SURFACE_Z-.0007])
            if self._contact_z is None:
                self._contact_z=desired_tip[2]
            if self.stage in ('position','left_prepare','left_approach','press','recenter','feed'):
                target_force=min(13.5,7.+.5*(t-self.stage_since)) if self.stage=='feed' else 7.0
                delta=-.00035*(target_force-m.get('feed_force_n',0.))*self.dt
                self._contact_z=np.clip(self._contact_z+np.clip(delta,-.0001,.0001),
                                        self.task.SURFACE_Z-.025,self.task.SURFACE_Z+.001)
                desired_tip[2]=self._contact_z
            if self.stage == 'orient':
                desired_tip[2] += .060
            elif self.stage=='withdraw':
                self._withdraw_z=min(self.task.SURFACE_Z+.035,self._withdraw_z+.0025*self.dt)
                desired_tip[2]=self._withdraw_z
                entry=np.asarray(m['hole_entry_m'])
                bore=np.asarray(m['bore_axis'])
                desired_tip[:2]=entry[:2]+bore[:2]*((desired_tip[2]-entry[2])/bore[2])
            elif self.stage=='release':
                desired_tip[2]=self.task.SURFACE_Z+.045
                desired_tip[0]+=.050
            desired_tcp = desired_tip + self.drill_rotation @ (local_tcp-self.task.BIT_TIP_LOCAL)
            if self.stage in ('position','left_prepare','left_approach','press','recenter','feed','withdraw'):
                desired_tcp += .65*(desired_tip-np.asarray(m['tip_m']))
            desired_gr = self.drill_rotation @ r.T @ gr
            # Limit per-tick position and orientation increments; all tool motion comes from contact physics.
            displacement = desired_tcp-tcp
            if self._right_pos is None:
                self._right_pos, self._right_rot = tcp.copy(), gr.copy()
            displacement = desired_tcp-self._right_pos
            self._right_pos += displacement*min(1.,.003/max(np.linalg.norm(displacement),1e-9))
            angle = np.linalg.norm(ik._rot_error(desired_gr,self._right_rot))
            self._right_rot = ik.slerp_matrix(self._right_rot,desired_gr,min(1.,.035/max(angle,1e-9)))
            self._right_q,right=self._arm('right',self._right_pos,self._right_rot,self._right_q,0.)
            if round(t / self.dt) % 40 == 0:
                pe,re=self.last_ik['right']
                print(f'[demo] stage={self.stage} right IK={pe*1000:.1f}mm/{np.degrees(re):.1f}deg bias={np.round(self._bias["right"],3)}',flush=True)
            if self.stage == 'orient' and m['tilt_deg']<6 and m['target_error_m']<.008 and abs(m['surface_error_m']-.056)<.010:
                self.stage='position';self.stage_since=t
                print('[demo] lower bit onto wood target',flush=True)
            if self.stage == 'position' and m['tilt_deg']<2 and m['target_error_m']<.0012 and 6.<m.get('feed_force_n',0.)<14.:
                if self.contact_since is None: self.contact_since=t
                if t-self.contact_since>1.:
                    self.stage='left_prepare';self.stage_since=t
                    print('[demo] approach trigger with left arm',flush=True)
            elif self.stage == 'position': self.contact_since=None
            if self.stage in ('left_prepare','left_approach','press','recenter','feed','withdraw','release'):
                # Anchor to the unpressed switch position, rather than chasing
                # the moving slider and continuously increasing its load.
                trigger=origin+r@self.task.TRIGGER_LOCAL
                # Horizontal gripper from +Y, with its lower closed finger below the trigger.
                goal_left=trigger+np.array([0.,.020,-.023])
                if self.stage=='left_prepare':
                    goal_left=trigger+np.array([0.,.120,.090])
                elif self.stage=='left_approach':
                    goal_left[1]+=.070
                elif self.stage=='press':
                    fraction=min(1.,(t-self.stage_since)/4.)
                    goal_left[1]+=.070*(1.-fraction)
                    goal_left[2]+=.014*max(0.,min(1.,(t-self.stage_since-4.)/3.))
                elif self.stage in ('recenter','feed','withdraw'):
                    self._trigger_push=np.clip(self._trigger_push+.08*(.004-m['trigger_travel_m']),.008,.023)
                    if m['trigger_contact_force_n']>2.:
                        self._trigger_push-=.001*self.dt*(m['trigger_contact_force_n']-2.)
                    goal_left[2]+=self._trigger_push
                elif self.stage=='release':
                    goal_left[1]+=.140
                    goal_left[2]+=.014
                lq=arm_q(left_robot)
                lpos,lrot=ik.tcp_pose(lq,station.BASES['left'])
                reached=np.linalg.norm(lpos-goal_left)<.012 and np.linalg.norm(ik._rot_error(self.left_rotation,lrot))<.08
                if self.stage=='left_prepare' and reached:
                    self.stage='left_approach';self.stage_since=t
                    print('[demo] left arm aligned; lower beside trigger',flush=True)
                elif self.stage=='left_approach' and reached:
                    self.stage='press';self.stage_since=t
                    print('[demo] pull spring trigger upward',flush=True)
                if self._left_pos is None:
                    self._left_pos, self._left_rot = lpos.copy(), lrot.copy()
                delta=goal_left-self._left_pos
                self._left_pos+=delta*min(1.,.003/max(np.linalg.norm(delta),1e-9))
                langle=np.linalg.norm(ik._rot_error(self.left_rotation,self._left_rot))
                self._left_rot=ik.slerp_matrix(self._left_rot,self.left_rotation,min(1.,.04/max(langle,1e-9)))
                self._left_q,left=self._arm('left',self._left_pos,self._left_rot,self._left_q,0.)
                if round(t/self.dt)%40==0:
                    pe,re=self.last_ik['left']
                    print(f'[demo] left IK={pe*1000:.1f}mm/{np.degrees(re):.1f}deg target error={np.linalg.norm(lpos-goal_left)*1000:.1f}mm',flush=True)
        meta={"request_stop":True} if t>110. else {}
        return ActionChunk(actions=[Action(data=np.r_[left,right],meta=meta)],control_hz=1./self.dt)
