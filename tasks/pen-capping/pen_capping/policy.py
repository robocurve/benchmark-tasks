"""Privileged pose feedback commands robot joints; assembly motion stays physical."""
from dataclasses import dataclass
import math,json
import numpy as np
from inspect_robots_yam.packing import split,pack
from inspect_robots.types import Action,ActionChunk
from yam_arena import ik
from yam_arena.constants import FINGER_OPEN as O,FINGER_CLOSED as C,READY
from yam_arena.demo import ArmPlan,Waypoint,grip_rotation
from yam_arena.scripted import YamScriptedPolicy
from yam_arena.urdf import axis_angle_matrix
from pen_capping.recovery import ContactRecovery, PrecontactBias
from pen_capping.scoring import SEATED_X
# Controller travel target only; measured physical engagement criteria unchanged.
INSERT_TARGET_X=SEATED_X+.0015
from pen_capping.alignment import AxisServo, LateralDamping, rotation_vector, rotation_about_object, safe_depth, RetainedReleaseClock, follow_bias_update, InsertionAdvance
ASSEMBLY_AXIS=(-math.sqrt(.5),-math.sqrt(.5),0.)
SEATING_HOLD_LABELS={'align body','align cap','hold insertion','insert','hold seat','seat','hold reached','release supporting hand'}

@dataclass
class ToolWaypoint(Waypoint):
    object_name: str | None = None
    axis: tuple = ASSEMBLY_AXIS
    follow_pen: int | None = None
    depth: float | None = None
    release: bool = False
    pick_name: str | None = None
    pick_x: float = 0.
    pick_height: float = 0.
    pick_yaw_offset: float = 0.

def make_plan(top,pens,caps,colors):
    plans={'left':[],'right':[]};counts={'red':0,'blue':0}
    park={'left':(.13,.37,top+.20),'right':(.13,-.40,top+.20)}
    def tcp(d,p,f,l,yaw=0):return ToolWaypoint(d,tuple(p),f,l,yaw)
    def root(d,name,p,f,l,axis=ASSEMBLY_AXIS,follow=None,depth=None,release=False):return ToolWaypoint(d,tuple(p),f,l,0.,name,axis,follow,depth,release)
    for i,(pen,cap,color) in enumerate(zip(pens,caps,colors)):
        pn,cn=f'pen_{i}',f'cap_{i}';x,y,z,a=pen;cx,cy,cz,ca=cap
        pg=np.array([x+.055*math.cos(a),y+.055*math.sin(a),z+.010]);cg=np.array([cx,cy,cz+.010])
        right=plans['right'];left=plans['left']
        for points,g,yaw in ((right,pg,a+math.pi),(left,cg,ca+math.pi)):
            points += [tcp(2,g+[0,0,.10],O,'approach',yaw),tcp(2,g,O,'lower',yaw),tcp(1,g,C,'grasp',yaw),tcp(2,g+[0,0,.10],C,'lift',yaw)]
            for w,height in zip(points[-4:],(.110,.010,.010,.110)):
                w.pick_name=pn if points is right else cn;w.pick_x=.055 if points is right else -.008;w.pick_height=height;w.pick_yaw_offset=math.pi
        body=(.27,-.02,top+.10);cap_above=tuple(np.array(body)-.16*np.array(ASSEMBLY_AXIS))
        right += [root(4,pn,body,C,'carry body'),root(4,pn,body,C,'align body'),root(14,pn,body,C,'hold insertion'),root(14,pn,body,C,'hold seat')]
        left += [root(4,cn,cap_above,C,'carry cap'),root(4,cn,cap_above,C,'align cap',follow=i,depth=-.16),root(14,cn,cap_above,C,'insert',follow=i,depth=INSERT_TARGET_X),root(14,cn,cap_above,C,'seat',follow=i,depth=INSERT_TARGET_X)]
        placer='left' if color=='red' else 'right';other='right' if placer=='left' else 'left';held=cn if placer=='left' else pn;other_name=pn if placer=='left' else cn
        plans[other] += [root(1,other_name,body,O,'release supporting hand',release=True),tcp(2,park[other],O,'retreat supporting hand')]
        plans[placer] += [root(1,held,body,C,'hold assembled'),root(2,held,body,C,'hold assembled')]
        # The left arm holds the cap for red pens; the right holds the blue body.
        # Reuse the last reached pose in the handover hold, rather than moving it.
        for w in plans[placer][-2:]:w.label='hold reached'
        index=counts[color];counts[color]+=1;dest=np.array([.30,(.19+.08*index)*(1 if color=='red' else -1),top+.10]);axis=np.array([-1.,0.,0.])
        target=dest+(axis*SEATED_X if placer=='left' else 0)
        low=target.copy();low[2]=top+.017
        plans[placer] += [root(5,held,target,C,'carry to destination',tuple(axis)),root(3,held,low,C,'lower assembly',tuple(axis)),root(2,held,low,O,'release assembly',tuple(axis),release=True),root(1,held,low,O,'wait fully open',tuple(axis),release=True)]
        grip_offset=0. if placer=='left' else .055;hover=low+axis*grip_offset+[0,0,.13]
        if placer=='left':
            exit_point=low+axis*.11+[0,0,.010]
            plans[placer].append(tcp(2,exit_point,O,'withdraw through open mouth'))
            hover=exit_point+[0,0,.12]
        plans[placer] += [tcp(2,hover,O,'retreat vertically'),tcp(2,park[placer],O,'park'),tcp(2,park[placer],O,'rest')]
        plans[other] += [tcp(19 if placer=='left' else 17,park[other],O,'wait for placement')]
    return plans

class FeedbackPlan(ArmPlan):
    def __init__(self,points,base,side,dt=.05):
        super().__init__(points,base);self.side=side;self.actual_pose=ik.tcp_pose(np.asarray(READY),base);self.active=-1;self.follow_index=None;self.filtered_follow=None;self.follow_bias=np.zeros(3);self.axis_servo=AxisServo();self.lateral_damping=LateralDamping();self.insertion_advance=InsertionAdvance();self.recovery=ContactRecovery();self.precontact_bias=PrecontactBias();self.command_holds={};self.body_command_holds={};self.active_hold=None;self.actual_q=np.asarray(READY).copy();self.episode_time=0.;self._diagnostic_step=0;self.object_diagnostic={};self.dt=dt
    def target(self,t):
        from pen_capping.embodiment import FEEDBACK
        elapsed=t;index=0
        for index,w in enumerate(self.waypoints):
            if elapsed<w.duration:break
            elapsed-=w.duration
        else:return self.last
        self.object_diagnostic={}
        tcp,trot=self.actual_pose;poses=FEEDBACK['poses'];rots=FEEDBACK['rotations'];s=ik.smoothstep(elapsed/w.duration)
        if index!=self.active:
            print('PEN_PHASE',self.side,round(t,2),w.label,'object',w.object_name,flush=True)
            self.active=index;self.start_tcp=tcp.copy();self.start_rot=trot.copy();self.start_finger=self.finger_start if index==0 else self.waypoints[index-1].finger
            if w.pick_name:
                axis=rots[w.pick_name][:,0];self.pick_goal=poses[w.pick_name][:3]+axis*w.pick_x+np.array([0.,0.,w.pick_height]);yaw=math.atan2(axis[1],axis[0])+w.pick_yaw_offset;self.pick_rotation=grip_rotation(yaw)
                print('MEASURED_PICK',self.side,w.label,w.pick_name,'root',poses[w.pick_name][:3].tolist(),'target',self.pick_goal.tolist(),'yaw',math.degrees(yaw),flush=True)
            if w.object_name:
                self.start_root=poses[w.object_name][:3].copy()
                if w.follow_pen is None:self.axis_servo=AxisServo()
                if w.follow_pen is not None:
                    if self.follow_index!=w.follow_pen:self.follow_index=w.follow_pen;self.filtered_follow=None;self.follow_bias=np.zeros(3);self.axis_servo=AxisServo();self.lateral_damping=LateralDamping();self.insertion_advance=InsertionAdvance();self.recovery=ContactRecovery();self.precontact_bias=PrecontactBias()
                    pn=f'pen_{w.follow_pen}';self.start_relative=rots[pn].T@(poses[w.object_name][:3]-poses[pn][:3]);self.start_depth=float(self.start_relative[0])
        finger=self.start_finger+s*(w.finger-self.start_finger)
        pairs=FEEDBACK.get('pairs',{})
        retained_objects={name for i,j in pairs.items() for name in (f'pen_{i}',f'cap_{j}')}
        self.active_hold=None
        if w.object_name and w.object_name.startswith('pen_') and w.label=='hold insertion' and w.object_name not in self.body_command_holds:
            # Alignment has completed. Keep its last actuator command rather
            # than continually moving the reference while the cap approaches.
            pos0,rot0,_,_=self.last
            self.body_command_holds[w.object_name]=(np.asarray(pos0).copy(),np.asarray(rot0).copy(),self.q.copy())
            print('BODY_COMMAND_LATCH',json.dumps(dict(side=self.side,time_s=self.episode_time,plan_time_s=t,object_name=w.object_name,tcp_m=np.asarray(pos0).tolist(),rotation=np.asarray(rot0).tolist(),q_rad=self.q.tolist()),sort_keys=True),flush=True)
        if w.label in SEATING_HOLD_LABELS and w.object_name in retained_objects and w.object_name not in self.command_holds:
            # last/q generated the action whose physics step established the
            # accepted joint. Preserve that actuator command, not observed TCP.
            pos0,rot0,_,_=self.last
            self.command_holds[w.object_name]=(np.asarray(pos0).copy(),np.asarray(rot0).copy(),self.q.copy())
            print('ASSEMBLY_COMMAND_LATCH',json.dumps(dict(side=self.side,time_s=self.episode_time,plan_time_s=t,object_name=w.object_name,tcp_m=np.asarray(pos0).tolist(),rotation=np.asarray(rot0).tolist(),q_rad=self.q.tolist(),observed_tcp_m=tcp.tolist(),observed_rotation=trot.tolist()),sort_keys=True),flush=True)
        if w.label in SEATING_HOLD_LABELS and w.object_name in self.command_holds:
            pos,rot,self.active_hold=self.command_holds[w.object_name]
        elif w.label in {'hold insertion','hold seat'} and w.object_name in self.body_command_holds:
            pos,rot,self.active_hold=self.body_command_holds[w.object_name]
        elif w.release or w.label=='hold reached':pos=self.start_tcp;rot=self.start_rot
        elif w.object_name:
            goal_axis=np.array(w.axis);desired=self.start_root+s*(np.array(w.pos)-self.start_root);guard_blocked=False;requested_depth=None;recovery_state=None;target_lateral=np.zeros(2)
            if w.follow_pen is not None:
                pn=f'pen_{w.follow_pen}';goal_axis=rots[pn][:,0]
                depth=self.start_depth+s*(w.depth-self.start_depth);requested_depth=depth
                if w.label=='align cap':target_lateral=(1.-s)*self.start_relative[1:]
                relative=rots[pn].T@(poses[w.object_name][:3]-poses[pn][:3])
                if w.follow_pen not in FEEDBACK.get('pairs',{}):depth=safe_depth(depth,float(np.linalg.norm(rotation_vector(rots[w.object_name][:,0],goal_axis))),float(np.linalg.norm(relative[1:])),float(relative[0]))
                guard_blocked=depth<requested_depth-1e-9
                if w.label in {'insert','seat'} and w.follow_pen not in pairs:
                    recovery_state=self.recovery.step(self.episode_time,relative,rots[pn].T@rots[w.object_name],requested_depth,FEEDBACK.get('backing_contacts',{}).get(f'{w.follow_pen}_{w.follow_pen}'),self.dt)
                    if recovery_state['clear_bias']:self.follow_bias=np.zeros(3)
                    if recovery_state['transition']=='center':
                        self.follow_bias=self.precontact_bias.restore(goal_axis)
                        print('CAP_RECOVERY_BIAS_RESTORE',json.dumps(dict(time_s=self.episode_time,pair=w.follow_pen,source_time_s=self.precontact_bias.saved_time,first_report_interval_start_s=self.precontact_bias.first_interval,restored_bias_world_m=self.follow_bias.tolist()),sort_keys=True),flush=True)
                    if recovery_state['transition'] is not None:print('CAP_RECOVERY_TRANSITION',json.dumps(dict(time_s=self.episode_time,pair=w.follow_pen,**recovery_state),sort_keys=True),flush=True)
                    if recovery_state['failed']:
                        self.active_hold=self.q.copy();self.last=(self.last[0],self.last[1],finger,w.label);return self.last
                    if recovery_state['depth'] is not None:
                        depth=recovery_state['depth'];target_lateral=np.array(recovery_state['lateral']);guard_blocked=depth<requested_depth-1e-9
                    else:
                        ready=float(np.linalg.norm(relative[1:]))<.00065 and float(np.linalg.norm(rotation_vector(rots[w.object_name][:,0],goal_axis)))<math.radians(.75)
                        depth=self.insertion_advance.step(requested_depth,float(relative[0]),self.dt,ready)
                        guard_blocked=not ready
                desired=poses[pn][:3]+rots[pn]@np.r_[depth,target_lateral]
            actual=poses[w.object_name][:3];error=desired-actual
            self.object_diagnostic=dict(object_name=w.object_name,actual_root_m=actual.tolist(),desired_root_m=desired.tolist(),root_error_m=error.tolist(),axis_error_deg=math.degrees(float(np.linalg.norm(rotation_vector(rots[w.object_name][:,0],goal_axis)))))
            axis=rots[w.object_name][:,0]
            if w.follow_pen is not None and w.follow_pen not in FEEDBACK.get('pairs',{}):
                correction=self.axis_servo.correction(axis,goal_axis,self.dt)
            else:
                # Do not integrate against a joint that already preserves the
                # accepted relative orientation of the assembled pair.
                vector=rotation_vector(axis,goal_axis);angle=float(np.linalg.norm(vector))
                correction=np.eye(3) if angle<1e-8 else axis_angle_matrix(vector/angle,angle)
            pivot=rotation_about_object(tcp,actual,correction)
            if w.follow_pen is not None:
                previous_bias=self.follow_bias.copy()
                braking=self.lateral_damping.observe(relative,rots[pn],self.dt)
                damping_active=w.label in {'insert','seat'} and w.follow_pen not in pairs
                if not damping_active:braking=np.zeros(3)
                self.follow_bias=follow_bias_update(previous_bias,error,goal_axis,pivot+braking,guard_blocked)
                if guard_blocked and recovery_state is not None and recovery_state['state']=='normal' and float(relative[0])>=-.102:
                    unit_axis=goal_axis/np.linalg.norm(goal_axis)
                    self.follow_bias+=unit_axis*float((previous_bias-self.follow_bias)@unit_axis)
                self.precontact_bias.observe(self.episode_time,self.follow_bias,goal_axis,FEEDBACK.get('backing_contacts',{}).get(f'{w.follow_pen}_{w.follow_pen}'))
                raw=error+self.follow_bias+pivot+braking;limited=np.clip(raw,-.004,.004)
                unit_axis=goal_axis/np.linalg.norm(goal_axis)
                follow=dict(time_s=self.episode_time,plan_time_s=t,pen=w.follow_pen,cap=w.object_name,requested_depth_m=requested_depth,effective_depth_m=depth,depth_guard_blocked=guard_blocked,recovery=recovery_state,relative_to_pen_m=relative.tolist(),root_error_world_m=error.tolist(),bias_before_world_m=previous_bias.tolist(),bias_after_world_m=self.follow_bias.tolist(),bias_axial_m=float(self.follow_bias@unit_axis),bias_lateral_world_m=(self.follow_bias-unit_axis*float(self.follow_bias@unit_axis)).tolist(),pivot_world_m=pivot.tolist(),raw_command_delta_m=raw.tolist(),limited_command_delta_m=limited.tolist(),saturated_components=(np.abs(raw)>.004).tolist())
                follow.update(lateral_damping_active=damping_active,lateral_velocity_pen_m_s=self.lateral_damping.raw_velocity.tolist(),lateral_velocity_filtered_pen_m_s=self.lateral_damping.velocity.tolist(),lateral_damping_world_m=braking.tolist())
                self.object_diagnostic['follow']=follow
                if -.110<float(relative[0])<-.070 and np.linalg.norm(relative[1:])<.010:print('CAP_FOLLOW_CONTROL',json.dumps(follow,sort_keys=True),flush=True)
                pos=tcp+limited
            else:pos=tcp+np.clip(error+pivot,-.004,.004)
            rot=correction@trot
            if w.follow_pen is not None and w.follow_pen not in pairs and w.label in {'insert','seat'} and float(relative[0])>-.110:
                if self.filtered_follow is None:
                    last=getattr(self,'last',(tcp,trot))
                    self.filtered_follow=(np.asarray(last[0]).copy(),np.asarray(last[1]).copy())
                previous_pos,previous_rot=self.filtered_follow
                alpha=self.dt/(.150+self.dt)
                pos=previous_pos+alpha*(pos-previous_pos)
                rot=ik.slerp_matrix(previous_rot,rot,alpha)
                self.filtered_follow=(pos.copy(),rot.copy())
        else:
            goal=self.pick_goal if w.pick_name else np.array(w.pos);goal_rot=self.pick_rotation if w.pick_name else grip_rotation(w.yaw)
            pos=self.start_tcp+s*(goal-self.start_tcp);rot=ik.slerp_matrix(self.start_rot,goal_rot,s)
        self.last=(pos,rot,finger,w.label);return self.last

    def action7(self,t):
        pos,rot,finger,label=self.target(t)
        if self.active_hold is None:
            self.q,pe,re=ik.solve(pos,ik.matrix_to_quat_xyzw(rot),self.q,base_pos=self.base_pos,iters=60)
            self.worst_ik_mm=max(self.worst_ik_mm,pe*1000);self.worst_ik_deg=max(self.worst_ik_deg,math.degrees(re))
        else:self.q=self.active_hold.copy()
        grip=float(np.clip(finger/O,0.,1.)) if O!=C else 1.
        action=np.r_[self.q,grip]
        self._diagnostic_step+=1
        if self._diagnostic_step%10==0:
            actual_p,actual_r=self.actual_pose;desired_p,desired_r,_,_=self.last
            solved_p,solved_r=ik.tcp_pose(self.q,self.base_pos)
            row=dict(episode_time_s=self.episode_time,plan_time_s=t,side=self.side,label=label,
                actual_tcp_m=actual_p.tolist(),desired_tcp_m=np.asarray(desired_p).tolist(),solved_tcp_m=solved_p.tolist(),
                actual_tcp_rotation=actual_r.tolist(),desired_tcp_rotation=desired_r.tolist(),solved_tcp_rotation=solved_r.tolist(),
                actual_q_rad=self.actual_q.tolist(),command_q_rad=self.q.tolist(),
                joint_tracking_error_rad=(self.q-self.actual_q).tolist(),
                ik_position_error_m=float(np.linalg.norm(solved_p-desired_p)),
                ik_rotation_error_deg=math.degrees(float(np.linalg.norm(ik._rot_error(desired_r,solved_r)))),
                observed_tracking_position_error_m=float(np.linalg.norm(np.asarray(desired_p)-actual_p)),
                observed_tracking_rotation_error_deg=math.degrees(float(np.linalg.norm(ik._rot_error(desired_r,actual_r)))),
                min_joint_limit_margin_rad=float(min(np.min(self.q-ik.LOWER),np.min(ik.UPPER-self.q))),object_control=self.object_diagnostic)
            print('ARM_CONTROL',json.dumps(row,sort_keys=True),flush=True)
        return action,label

class PenPolicy(YamScriptedPolicy):
    def reset(self,scene):
        self._plans={side:FeedbackPlan(points,self._bases[side],side,self._dt) for side,points in self.plan_for(scene.instruction,scene.target).items()};self._end=max(p.duration for p in self._plans.values());self._seat_checks=[];self._seen_pairs={}
        elapsed=0.
        for w in self._plans['left'].waypoints:
            elapsed+=w.duration
            if w.label=='seat':self._seat_checks.append((elapsed,w.follow_pen))
        self._clock=RetainedReleaseClock(self._seat_checks);self._plan_time=0.
    def act(self,obs):
        for side,q in zip(('left','right'),split(obs.state['joint_pos'])):
            self._plans[side].actual_q=np.asarray(q[:6]).copy();self._plans[side].episode_time=float(obs.state_time)
            self._plans[side].actual_pose=ik.tcp_pose(q[:6],self._bases[side])
        from pen_capping.embodiment import FEEDBACK
        pairs=FEEDBACK.get('pairs',{})
        self._plan_time=self._clock.advance(self._plan_time,self._dt,pairs)
        per_arm=[self._plans[side].action7(self._plan_time)[0] for side in ('left','right')]
        meta={'request_stop':True} if self._plan_time>self._end+self.hold_s or self._clock.failed is not None else {}
        result=ActionChunk(actions=[Action(data=pack(*per_arm),meta=meta)],control_hz=1./self._dt)
        if self._plans['left'].recovery.state=='failed':
            result.actions[0].meta['request_stop']=True
            print('DEMO_STOP_RECOVERY',json.dumps(dict(reason=self._plans['left'].recovery.reason,time_s=float(obs.state_time)),sort_keys=True),flush=True)
        if self._clock.failed is not None:
            index=self._clock.failed
            print('DEMO_STOP_RETENTION_TIMEOUT',index,'waited_s',self._clock.waited[index],json.dumps(FEEDBACK.get('attempts',{}).get(f'{index}_{index}',{}),sort_keys=True),flush=True)
        lost=[i for i,j in self._seen_pairs.items() if pairs.get(i)!=j]
        self._seen_pairs.update(pairs)
        if lost:
            result.actions[0].meta['request_stop']=True
            print('DEMO_STOP_LOST_RETENTION',json.dumps({str(i):FEEDBACK.get('retention_losses',{}).get(str(i),{}) for i in lost},sort_keys=True),flush=True)
        return result
def make_policy(**kwargs):
    kwargs.setdefault('hold_s',5.)
    return PenPolicy(**kwargs)
