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
                    for clearance in (0.,.060):
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
            desired_tip = np.array([*self.task.TARGET_XY,self.task.SURFACE_Z-0.004])
            if self.stage == 'orient':
                desired_tip[2] += .060
            desired_tcp = desired_tip + self.drill_rotation @ (local_tcp-self.task.BIT_TIP_LOCAL)
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
            m = self.task.measure(objects,emb.robot_state())
            if round(t / self.dt) % 40 == 0:
                pe,re=self.last_ik['right']
                print(f'[demo] stage={self.stage} right IK={pe*1000:.1f}mm/{np.degrees(re):.1f}deg bias={np.round(self._bias["right"],3)}',flush=True)
            if self.stage == 'orient' and m['tilt_deg']<6 and m['target_error_m']<.008 and abs(m['surface_error_m']-.056)<.010:
                self.stage='position';self.stage_since=t
                print('[demo] lower bit onto wood target',flush=True)
            if self.stage == 'position' and m['tilt_deg']<8 and m['tip_on_target']:
                if self.contact_since is None: self.contact_since=t
                if t-self.contact_since>1.:
                    self.stage='left_prepare';self.stage_since=t
                    print('[demo] approach trigger with left arm',flush=True)
            elif self.stage == 'position': self.contact_since=None
            if self.stage in ('left_prepare','left_approach','press'):
                trigger=np.asarray(objects['trigger'].pos)
                # Horizontal gripper from +Y, with its lower closed finger below the trigger.
                goal_left=trigger+np.array([0.,.020,-.023])
                if self.stage=='left_prepare':
                    goal_left=trigger+np.array([0.,.120,.090])
                elif self.stage=='left_approach':
                    goal_left[1]+=.070
                else:
                    fraction=min(1.,(t-self.stage_since)/4.)
                    goal_left[1]+=.070*(1.-fraction)
                    goal_left[2]+=.014*max(0.,min(1.,(t-self.stage_since-4.)/3.))
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
        meta={"request_stop":True} if t>70. else {}
        return ActionChunk(actions=[Action(data=np.r_[left,right],meta=meta)],control_hz=1./self.dt)
