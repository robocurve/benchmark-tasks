"""Privileged development controller; only commands native YAM joints and fingers."""
import json
import math
import numpy as np
from yam_arena import ik,station
from yam_arena.constants import SHOULDER,WRIST
from yam_arena.runtime import arm_q,tcp_world
from yam_arena.urdf import axis_angle_matrix,quat_xyzw_to_matrix


def force_position_bias(q,force,base):
    """Encode a TCP force request using the existing native position actuators."""
    eps=1e-5
    jac=np.column_stack([(ik.tcp_pose(q+np.eye(6)[i]*eps,base)[0]
                         -ik.tcp_pose(q-np.eye(6)[i]*eps,base)[0])/(2*eps) for i in range(6)])
    stiffness=np.array([SHOULDER['stiffness']]*3+[WRIST['stiffness']]*3)
    return (jac.T@force)/stiffness


def insertion_spring_force(target_height, stiffness, rest):
    contact_fraction=np.clip((target_height+.012)/.004,0.,1.)
    compression=np.clip(target_height+.008,0.,.008)
    return float(contact_fraction*stiffness*(compression-rest))


def bayonet_turn_clear(p,r,shaft_p,shaft_r):
    origin=r.T@(shaft_p-p)
    axis=r.T@shaft_r[:,2]
    if axis[2]<.98:return False
    lo=max(-.005,(-.020-origin[2])/axis[2])
    hi=min(.032,(.020-origin[2])/axis[2])
    if lo>=hi:return False
    if any(np.linalg.norm((origin+axis*z)[:2])>=.0003 for z in [lo,hi]):return False
    return all(abs((r.T@(shaft_p+shaft_r@np.array([0.,y,0.])-p))[2])<.0014 for y in [.005,.010])


def shaft_frame_pose(shaft_p,shaft_r,start_p,start_r,goal_p,goal_r,fraction):
    return (shaft_p+shaft_r@(start_p+fraction*(goal_p-start_p)),
            shaft_r@ik.slerp_matrix(start_r,goal_r,fraction))


def bayonet_alignment_rotation(tool_r,shaft_r):
    """Align the bore axis while preserving the measured twist, without chasing shaft yaw."""
    relative=shaft_r.T@tool_r
    yaw=math.atan2(relative[1,0],relative[0,0])
    return shaft_r@axis_angle_matrix(np.array([0.,0.,1.]),yaw)


def beater_withdrawal(hand_p,hand_r,clear_r,fraction):
    """Unwind during a 120 mm retreat so a released wrist at its limit can exit."""
    return hand_p-.12*fraction*hand_r[:,2],ik.slerp_matrix(hand_r,clear_r,fraction)


def guard_withdrawal(hand_p,hand_r,fraction):
    clear=ik.smoothstep(min(2*fraction,1.))
    outboard=ik.smoothstep(max(2*fraction-1.,0.))
    return hand_p-.06*clear*hand_r[:,2]+outboard*np.array([0.,.20,0.])


def guard_goal_rotation(phase):
    # Point the open slot toward the tool during the diagonal approach.
    yaw=-math.pi/12 if phase in ['carry guard to front left','align guard in front of bowl'] else 0.
    return axis_angle_matrix(np.array([0.,0.,1.]),yaw)


class Controller:
    def __init__(self,task):
        self.task=task
        plans=json.loads((task.folder/'trajectory.json').read_text())
        self.stages={s['phase']:s for plan in plans.values() for s in plan}
        self.durations={name:round(stage['seconds']*20) for name,stage in self.stages.items()}
        self.guard_nominal=None
        self.sequence_phase=None
        self.sequence_active=False
        self.last_action=None
        self.phase=None
        self.relative=None
        self.bias=np.zeros(3)
        self.nominal=None
        self.rotation_bias=np.zeros(3)

    def action(self,action,phase,base):
        if phase in ['clear mixer before guard','approach guard tab','clear head before START']:self.sequence_active=True
        if self.sequence_active:
            if self.sequence_phase!=phase:
                self.sequence_phase=phase;self.sequence_tick=0
                self.sequence_hold=(self.last_action if self.last_action is not None else action).copy()
                self.sequence_start={}
                for side in ['left','right']:
                    robot=base.scene[side+'_robot']
                    self.sequence_start[side]=(np.asarray(tcp_world(robot)),ik.tcp_pose(arm_q(robot),station.BASES[side])[1])
            self.sequence_tick+=1
            f=ik.smoothstep(min(1.,self.sequence_tick/self.durations[phase]))
            action=self.sequence_hold.copy()
            for side,offset in [('left',0),('right',7)]:
                target=self.stages[phase].get(side)
                if target is None:continue
                p0,r0=self.sequence_start[side]
                p=p0+f*(np.asarray(target['position'])-p0)
                r=ik.slerp_matrix(r0,quat_xyzw_to_matrix(np.asarray(target['rotation_xyzw'])),f)
                q,pe,re=ik.solve(p,ik.matrix_to_quat_xyzw(r),arm_q(base.scene[side+'_robot']),station.BASES[side],iters=200)
                if pe>.003 or re>.04:raise RuntimeError(f'Unreachable transition {phase}: {pe}, {re}')
                action[offset:offset+6]=q
                action[offset+6]+=f*(target['grip']-action[offset+6])
        result,feedback=self.adaptive_action(action,phase,base)
        self.last_action=result.copy()
        return result,feedback

    def adaptive_action(self,action,phase,base):
        if phase in ['lift guard clear of rack','carry guard to front left','align guard in front of bowl',
                     'slide guard around tool','center guard above bowl','seat guard on rim','release guard','withdraw from guard','observe guard']:
            return self.guard_action(action,phase,base)
        if phase in ['raise mixer bowl','settle raised bowl','release lift handle','withdraw from lift handle','observe bowl lift']:
            return self.lift_action(action,phase,base)
        adaptive=['lift beater','carry to bowl front','lower into bowl','align below shaft',
                  'insert hub','settle inserted hub','turn bayonet','settle retaining pocket','release beater',
                  'withdraw from beater','observe retention']
        if phase not in adaptive:return action,{}
        state=self.task.states(base)
        p,r,v=state['beater'];sp,sr,sv=state['mixer/shaft']
        robot=base.scene['left_robot'];q=arm_q(robot)
        hp=np.asarray(tcp_world(robot));_,hr=ik.tcp_pose(q,station.BASES['left'])
        if phase!=self.phase:
            if phase=='settle retaining pocket':
                relative_r=sr.T@r
                yaw=math.atan2(relative_r[1,0],relative_r[0,0])
                error=math.atan2(math.sin(yaw-self.task.cfg['hub_yaw']),math.cos(yaw-self.task.cfg['hub_yaw']))
                if getattr(self,'turn_tick',0)<100 or abs(error)>math.radians(5):
                    raise RuntimeError('Bayonet turn did not complete with shaft and pin clearance')
            self.phase=phase;self.tick=0
            if phase in ['insert hub','settle inserted hub','turn bayonet','settle retaining pocket']:
                self.shaft_start_p=sr.T@(p-sp)
                self.shaft_start_r=sr.T@r
            if phase=='turn bayonet':
                self.turn_tick=0
                yaw=math.atan2(self.shaft_start_r[1,0],self.shaft_start_r[0,0])
                self.shaft_start_r=axis_angle_matrix(np.array([0.,0.,1.]),yaw)
            self.start_p,self.start_r=(p.copy(),r.copy()) if self.nominal is None else self.nominal
            self.hand_p=hp.copy();self.hand_r=hr.copy()
            if self.relative is None:self.relative=(r.T@(hp-p),r.T@hr)
        self.tick+=1
        f=ik.smoothstep(min(1.,self.tick/self.durations[phase]))
        grip=0.
        turn_clear=None
        precision=phase in ['align below shaft','insert hub','settle inserted hub','turn bayonet']
        if phase in ['release beater','withdraw from beater','observe retention']:
            position=self.hand_p.copy();rotation=self.hand_r.copy();grip=1.
            if phase=='withdraw from beater':
                clear_r=quat_xyzw_to_matrix(np.asarray(self.stages['approach beater']['left']['rotation_xyzw']))
                position,rotation=beater_withdrawal(self.hand_p,self.hand_r,clear_r,f)
        else:
            goal=self.start_p.copy();goal_r=self.start_r.copy()
            if phase=='lift beater':goal[2]=1.16
            elif phase=='carry to bowl front':goal=np.array([sp[0]-.10,sp[1],1.16]);goal_r=np.eye(3)
            elif phase=='lower into bowl':goal=np.array([sp[0]-.09,sp[1],sp[2]-.045]);goal_r=sr
            elif phase=='align below shaft':goal=sp+[0,0,-.045];goal_r=sr
            elif phase in ['insert hub','settle inserted hub']:goal=sp+[0,0,self.task.cfg['hub_insert_offset']];goal_r=sr
            elif phase=='turn bayonet':
                goal=sp+[0,0,self.task.cfg['hub_insert_offset']]
                goal_r=sr@axis_angle_matrix(np.array([0.,0.,1.]),self.task.cfg['hub_yaw'])
            elif phase=='settle retaining pocket':
                goal=sp+[0,0,self.task.cfg['hub_rest_offset']]
                goal_r=sr@axis_angle_matrix(np.array([0.,0.,1.]),self.task.cfg['hub_yaw'])
            target=self.start_p+f*(goal-self.start_p)
            rotation_object=ik.slerp_matrix(self.start_r,goal_r,f)
            if phase in ['insert hub','settle inserted hub','turn bayonet','settle retaining pocket']:
                offset=self.task.cfg['hub_rest_offset'] if phase=='settle retaining pocket' else self.task.cfg['hub_insert_offset']
                relative_goal=np.array([0.,0.,offset])
                relative_rotation=axis_angle_matrix(np.array([0.,0.,1.]),self.task.cfg['hub_yaw']) if phase in ['turn bayonet','settle retaining pocket'] else np.eye(3)
                relative_start=self.shaft_start_p
                fraction=f
                if phase=='turn bayonet':
                    relative_start=relative_goal
                    turn_clear=bayonet_turn_clear(p,r,sp,sr)
                    if turn_clear:self.turn_tick+=1
                    fraction=ik.smoothstep(min(self.turn_tick/100.,1.))
                target,rotation_object=shaft_frame_pose(sp,sr,relative_start,self.shaft_start_r,relative_goal,relative_rotation,fraction)
                if turn_clear is False:rotation_object=bayonet_alignment_rotation(r,sr)
            self.nominal=(target.copy(),rotation_object.copy())
            self.bias+=np.array([.24,.24,.08] if precision else [.08]*3)*(target-p)
            self.bias=np.clip(self.bias,[-.015,-.015,-.04],[.015,.015,.04])
            rotation_error=ik._rot_error(rotation_object,r)
            if turn_clear is False:
                axis=sr[:,2]
                rotation_error-=axis*np.dot(axis,rotation_error)
                self.rotation_bias-=axis*np.dot(axis,self.rotation_bias)
            self.rotation_bias+=(.12 if precision else .04)*rotation_error
            magnitude=np.linalg.norm(self.rotation_bias)
            self.rotation_bias*=min(1.,.15/max(magnitude,1e-12))
            magnitude=np.linalg.norm(self.rotation_bias)
            correction=axis_angle_matrix(self.rotation_bias/max(magnitude,1e-12),magnitude) if magnitude>1e-9 else np.eye(3)
            commanded_rotation=correction@rotation_object
            position=target+self.bias+commanded_rotation@self.relative[0]
            rotation=commanded_rotation@self.relative[1]
        solved,pe,re=ik.solve(position,ik.matrix_to_quat_xyzw(rotation),q,station.BASES['left'],iters=200)
        if pe>.003 or re>.04:
            raise RuntimeError(f'Unreachable native mixer target {phase}: {pe:.5f} m, {re:.5f} rad')
        spring_force=0.
        if phase in ['insert hub','settle inserted hub','turn bayonet']:
            collar=next(joint for joint in self.task.descriptions['mixer']['joints'] if joint['name']=='retaining_spring')
            spring_force=insertion_spring_force((sr.T@(target-sp))[2],collar['stiffness'],collar['target'])
        joint_bias=force_position_bias(solved,sr@np.array([0.,0.,spring_force]),station.BASES['left'])
        action=action.copy();action[:6]=np.clip(solved+joint_bias,ik.LOWER,ik.UPPER);action[6]=grip
        return action,dict(position_error_m=pe,rotation_error_rad=re,hub_command=position.tolist(),translation_bias=self.bias.tolist(),
                           rotation_bias=self.rotation_bias.tolist(),spring_compensation_N=float(spring_force),
                           bayonet_turn_controls=getattr(self,'turn_tick',0),bayonet_turn_clear=turn_clear)

    def lift_action(self,action,phase,base):
        robot=base.scene['right_robot'];q=arm_q(robot)
        hp=np.asarray(tcp_world(robot));_,hr=ik.tcp_pose(q,station.BASES['right'])
        if self.phase!=phase:
            self.phase=phase;self.tick=0
            self.hand_p=hp.copy();self.hand_r=hr.copy()
            if phase=='raise mixer bowl':
                self.handle_pivot=self.task.states(base)['mixer/lift_handle'][0].copy()
                self.handle_start_r=self.task.states(base)['mixer/lift_handle'][1].copy()
                self.handle_grip_r=self.hand_r.copy()
                self.knob_offset=self.handle_start_r@np.asarray(self.task.cfg['handle_knob_offset'])
                self.grip_tcp_offset=self.hand_r.T@(hp-self.handle_pivot-self.knob_offset)
                self.lift_bias=np.zeros(3)
                self.lift_rotation_bias=np.zeros(3)
        self.tick+=1
        f=ik.smoothstep(min(1.,self.tick/self.durations[phase]))
        grip=1.;position=self.hand_p.copy();requested_force=np.zeros(3)
        if phase in ['raise mixer bowl','settle raised bowl']:
            handle_r=self.task.states(base)['mixer/lift_handle'][1]@self.handle_start_r.T
            actual_angle=math.atan2(handle_r[0,2],handle_r[0,0])
            # Build lifting force without running the hand ahead of a stalled lever.
            angle=min((f if phase=='raise mixer bowl' else 1.)*math.pi/2,
                      max(0.,actual_angle)+math.radians(10))
            turn=axis_angle_matrix(np.array([0.,1.,0.]),angle)
            knob_goal=self.handle_pivot+turn@self.knob_offset
            yaw=min(max(actual_angle/(math.pi/3),0.),1.)*math.pi/6
            rotation=(axis_angle_matrix(np.array([0.,1.,0.]),actual_angle)
                      @axis_angle_matrix(np.array([0.,0.,1.]),yaw)@self.handle_grip_r)
            position=knob_goal+rotation@self.grip_tcp_offset
            self.lift_bias=np.clip(self.lift_bias+.06*(position-hp),-.01,.01)
            self.lift_rotation_bias+=.04*ik._rot_error(rotation,hr)
            magnitude=np.linalg.norm(self.lift_rotation_bias)
            self.lift_rotation_bias*=min(1.,.15/max(magnitude,1e-12))
            magnitude=np.linalg.norm(self.lift_rotation_bias)
            if magnitude>1e-9:rotation=axis_angle_matrix(self.lift_rotation_bias/magnitude,magnitude)@rotation
            position=knob_goal+rotation@self.grip_tcp_offset+self.lift_bias
            bowl=next(body for body in self.task.descriptions['mixer']['bodies'] if body['name']=='bowl')
            load=bowl['mass']
            if self.task.mechanical_state(self.task.states(base))['guard_seated']:
                load+=self.task.descriptions['guard']['bodies'][0]['mass']
            # Robot-applied preload offsets hinge friction and lifted weight;
            # the constant-ratio lift needs constant gravity torque at every angle.
            hinge=next(joint for joint in self.task.descriptions['mixer']['joints'] if joint['name']=='handle_hinge')
            effort=hinge['static_friction_effort']+.2+load*9.81*self.task.cfg['lift_travel']/(math.pi/2)
            if actual_angle<math.pi/2-math.radians(.5):
                requested_force=effort/np.linalg.norm(self.knob_offset)*np.array([math.sin(actual_angle),0.,math.cos(actual_angle)])
            grip=.12
        elif phase=='withdraw from lift handle':position-=f*.06*self.hand_r[:,2]
        if phase not in ['raise mixer bowl','settle raised bowl']:rotation=self.hand_r
        if phase=='withdraw from lift handle':
            rotation=axis_angle_matrix(np.array([0.,0.,1.]),-f*math.pi/6)@rotation
        solved,pe,re=ik.solve(position,ik.matrix_to_quat_xyzw(rotation),q,station.BASES['right'],iters=200)
        if pe>.003 or re>.04:raise RuntimeError(f'Unreachable lift target {phase}: {pe}, {re}')
        joint_bias=force_position_bias(solved,requested_force,station.BASES['right'])
        action=action.copy();action[7:13]=np.clip(solved+joint_bias,ik.LOWER,ik.UPPER);action[13]=grip
        return action,dict(lift_hand_target=position.tolist(),position_error_m=pe,rotation_error_rad=re,
                           lift_translation_bias=self.lift_bias.tolist(),requested_lift_force_N=requested_force.tolist(),
                           feedforward_joint_bias_rad=joint_bias.tolist())

    def guard_action(self,action,phase,base):
        state=self.task.states(base);p,r,v=state['guard'];bp=state['mixer/bowl'][0]
        entry_y=state['mixer/shaft'][0][1]
        robot=base.scene['left_robot'];q=arm_q(robot)
        hp=np.asarray(tcp_world(robot));_,hr=ik.tcp_pose(q,station.BASES['left'])
        if self.phase!=phase:
            self.phase=phase;self.tick=0
            if self.guard_nominal is None:
                self.bias[:]=0.
                self.guard_rotation_bias=np.zeros(3)
            self.start_p,self.start_r=(p.copy(),r.copy()) if self.guard_nominal is None else self.guard_nominal
            self.hand_p=hp.copy();self.hand_r=hr.copy()
            if self.guard_nominal is None:self.guard_relative=(r.T@(hp-p),r.T@hr)
        self.tick+=1;f=ik.smoothstep(min(1.,self.tick/self.durations[phase]))
        grip=.08 if phase=='lift guard clear of rack' else .08-.02*f if phase=='carry guard to front left' else .06
        if phase in ['release guard','withdraw from guard','observe guard']:
            position=self.hand_p.copy();rotation=self.hand_r;grip=1.
            if phase=='withdraw from guard':position=guard_withdrawal(self.hand_p,self.hand_r,f)
        else:
            goals={'lift guard clear of rack':[.35,.31,.92],
                   'carry guard to front left':[.20,.28,1.015],
                   'align guard in front of bowl':[.145,entry_y,1.00],
                   'slide guard around tool':[bp[0],entry_y,1.00],
                   'center guard above bowl':[bp[0],bp[1],1.00],
                   'seat guard on rim':[bp[0],bp[1],bp[2]+self.task.cfg['guard_height']]}
            target=self.start_p+f*(np.asarray(goals[phase])-self.start_p)
            rotation_object=ik.slerp_matrix(self.start_r,guard_goal_rotation(phase),f)
            self.guard_nominal=(target.copy(),rotation_object.copy())
            self.bias=np.clip(self.bias+.08*(target-p),-.015,.015)
            self.guard_rotation_bias+=.04*ik._rot_error(rotation_object,r)
            magnitude=np.linalg.norm(self.guard_rotation_bias)
            self.guard_rotation_bias*=min(1.,.15/max(magnitude,1e-12))
            magnitude=np.linalg.norm(self.guard_rotation_bias)
            correction=axis_angle_matrix(self.guard_rotation_bias/max(magnitude,1e-12),magnitude) if magnitude>1e-9 else np.eye(3)
            commanded_rotation=correction@rotation_object
            position=target+self.bias+commanded_rotation@self.guard_relative[0]
            rotation=commanded_rotation@self.guard_relative[1]
        solved,pe,re=ik.solve(position,ik.matrix_to_quat_xyzw(rotation),q,station.BASES['left'],iters=200)
        if pe>.003 or re>.04:raise RuntimeError(f'Unreachable guard target {phase}: {pe}, {re}')
        action=action.copy();action[:6]=solved;action[6]=grip
        return action,dict(guard_hand_target=position.tolist(),position_error_m=pe,rotation_error_rad=re)
