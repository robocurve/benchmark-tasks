"""Measured bimanual release/reinsert using ordinary bounded 14D commands.

The LEFT robot grasps the ordinary boot. The RIGHT robot presses with its
unmodified tip_left finger. Object poses and actual finger joints are read,
never written; a grasp transform is measured rather than welded.
"""
import json
from pathlib import Path
import numpy as np
from yam_arena import ik
from yam_arena.constants import FINGER_OPEN
from yam_arena.urdf import quat_xyzw_to_matrix,rotation_angle_deg,axis_angle_matrix as axis_rotation
from config import SERVICE,TARGET,CLIP_Z,CLIP_TRAVEL,RELEASE_TRAVEL,port,USED as USED_PORTS
from collision_geometry import FingerGeometry,minimum_separation
from feedback import snapshot,clearance,gripper_sweep,BootJawPredictor,BootJawContactCache,EndpointCorrection,JointEndpointCorrection,plug_fit,plug_hand_target,retaining_face,closing_opening,predicted_jaw_normal_forces,boot_contact_force_gate,mesh_pose_distance
from geometry import object_obstacles
from diagnostics import clip_joint_geometry
from trajectory_control import mesh_reference_step_allowance

_angle=np.radians(80.)
R_GRIP=np.array([[0.,np.cos(_angle),np.sin(_angle)],[0.,-np.sin(_angle),np.cos(_angle)],[1.,0.,0.]])
BOOT_LOCAL=np.array((-.038,-.0035,.00024))
GRIP_OPENING=.00304001
GRIP=GRIP_OPENING/abs(FINGER_OPEN)
ENTRY_GRIP=.0065/abs(FINGER_OPEN)
PRESS_JAW=-.028
PRESS_GRIP=.028/abs(FINGER_OPEN)
MAX_RELEASE_STEPS=2400
PREPRESS_DISTANCE=.002
PREPRESS_DURATION=3.


class GraspProbe:
    def __init__(self,env,plug):
        self.env,self.base,self.plug=env,env._base,np.array(plug)
        self.previous=env._ready.detach().cpu().numpy().reshape(-1).copy()
        self.starts=[];self.q=[];self.bias=[np.zeros(3),np.zeros(3)];self.cartesian_bias_enabled=False
        for side in ('left','right'):
            q,f,p,r=self.arm(side);self.q.append(q.copy());self.starts.append((p.copy(),r.copy()))
        self.phase='approach_boot';self.phase_start=0.;self.stable_frames=0
        self.failure=None;self.last_t=None;self.release=False;self.events=[]
        self.boot_goal=self.plug+BOOT_LOCAL;self.boot_rotation=R_GRIP;self.front_offset=np.array((-.040,0,0))
        self.geometry=FingerGeometry();self.hand_clearance=None
        self.hold_offset=None;self.hold_rotation=None;self.grasp_drift=0.
        self.last_fingers=None
        self.free_clearance=None;self.entry_clearance=None;self.boot_prediction=None;self.jaw_predictor=BootJawPredictor(self.geometry)
        self.jaw_contact_cache=BootJawContactCache(self.jaw_predictor);self.closing_stable_frames=0
        self.entry_endpoint_correction=EndpointCorrection();self.entry_joint_servo=JointEndpointCorrection();self.entry_latched_pose=None
        self.entry_reference_error=(0.,0.);self.entry_goal_drift=0.;self.last_left_joints=None;self.left_joint_velocity=None
        self.left_unbiased_q=None
        self.stationary_hold_clearance=None;self.stationary_hold_safe=True;self.boot_force_gate=None
        self.entry_low_velocity_frames=0;self.last_left_tcp=None;self.left_tcp_speed=None

    def arm(self,side):
        ids,fids=self.env._ids[side+'_robot']
        data=self.base.scene[side+'_robot'].data.joint_pos
        data=data.torch if hasattr(data,'torch') else data
        values=data.detach().cpu().numpy()[0]
        q,f=values[ids],values[fids]
        p,r=ik.tcp_pose(q,self.env.rig.bases[side])
        return q,f,p,r

    def native_joint_velocity(self,side):
        ids,_=self.env._ids[side+'_robot']
        data=self.base.scene[side+'_robot'].data.joint_vel
        data=data.torch if hasattr(data,'torch') else data
        velocity=np.asarray(data.detach().cpu().numpy()[0][ids],dtype=float)
        if velocity.shape!=(6,) or not np.isfinite(velocity).all():
            raise ValueError('native right joint velocity is missing or nonfinite')
        return velocity.copy()

    def object_pose(self,name):
        data=self.base.arena_world.get_pose_w(name)
        data=data.torch if hasattr(data,'torch') else data
        return data.detach().cpu().numpy()[0]

    def transition(self,phase,t):
        self.events.append({'time':float(t),'from':self.phase,'to':phase})
        self.phase,self.phase_start,self.stable_frames=phase,t,0
        if phase=='enter_boot':
            self.entry_endpoint_correction.reset();self.entry_joint_servo.reset();self.entry_latched_pose=None;self.bias[0][:]=0.;self.entry_low_velocity_frames=0
            self.left_unbiased_q=None

    @staticmethod
    def settled(measured,target,tolerance=.001,rotation_tolerance=1.):
        return np.linalg.norm(measured[0]-target[0])<tolerance and rotation_angle_deg(measured[1],target[1])<rotation_tolerance

    def gate(self,condition,age,duration,timeout,next_phase,t):
        self.stable_frames=self.stable_frames+1 if condition and age>=duration else 0
        if self.stable_frames>=5:self.transition(next_phase,t)
        elif age>duration+timeout:self.failure=self.phase+' did not satisfy measured settle gate'

    def capture_grasp(self,plug,rotation,measured):
        self.hold_offset=rotation.T@(measured[2]-plug[:3])
        self.hold_rotation=rotation.T@measured[3]
        self.held_origin=plug[:3].copy()
        self.boot_goal,self.boot_rotation=measured[2].copy(),measured[3].copy()

    def stationary_hold_safety(self,measured):
        # Contact with the controlled boot is intended. Other rigid geometry
        # and the second hand must remain clear; flexible brushing is allowed.
        if not all(np.isfinite(value).all() for arm in measured for value in arm):
            return {'rigid_gap_m':0.,'other_hand_gap_m':0.,'invalid':'nonfinite_hand_state'}
        plugs={i:self.object_pose(f'plug_{i}') for i in USED_PORTS}
        clips={i:self.object_pose(f'clip_{i}') for i in USED_PORTS}
        left=[a@measured[0][3].T+measured[0][2] for _,a in self.geometry.pieces(measured[0][1])]
        right=[a@measured[1][3].T+measured[1][2] for _,a in self.geometry.pieces(measured[1][1])]
        obstacles=object_obstacles(plugs,clips,include_wires=False)
        self.current_press_obstacles=obstacles;self.current_press_left=left
        rigid=[a for owner,name,a in obstacles if (owner,name)!=(f'plug_{SERVICE}','boot_bound')]
        result={'rigid_gap_m':minimum_separation(left,rigid),'other_hand_gap_m':minimum_separation(left,right)}
        if self.release and self.phase in ('approach_above_clip','prepress_endpoint','depress_clip','withdraw_with_clip_pressed'):
            non_target=[a for owner,name,a in obstacles if (owner,name) not in ((f'clip_{SERVICE}','tab'),(f'clip_{SERVICE}','hook'))]
            result['right_non_target_rigid_gap_m']=minimum_separation(right,non_target)
        return result

    def command(self,t):
        dt=.05 if self.last_t is None else float(np.clip(t-self.last_t,0.,.1));self.last_t=t
        self.command_dt=dt
        if self.release:self.current_press_obstacles=None;self.current_press_left=None
        measured=[self.arm(side) for side in ('left','right')]
        if self.release:
            self.right_native_joint_velocity=None
            try:self.right_native_joint_velocity=self.native_joint_velocity('right')
            except (AttributeError,ValueError,IndexError,TypeError):self.failure='native right joint velocity is missing or nonfinite'
            self.right_joint_velocity=None if self.last_right_joints is None else (measured[1][0]-self.last_right_joints)/max(dt,.001)
            if self.phase=='prepress_endpoint' and self.last_right_joints is not None:
                movement=float(measured[1][0][2]-self.last_right_joints[2])
                if abs(movement)>1e-5:self.prepress_last_q3_movement=movement
            self.last_right_joints=measured[1][0].copy()
        self.left_tcp_speed=None if self.last_left_tcp is None else float(np.linalg.norm(measured[0][2]-self.last_left_tcp)/max(dt,.001))
        self.last_left_tcp=measured[0][2].copy()
        self.left_joint_velocity=None if self.last_left_joints is None else ((measured[0][0]-self.last_left_joints)/max(dt,.001)).tolist()
        self.last_left_joints=measured[0][0].copy()
        plug=self.object_pose(f'plug_{SERVICE}');rotation=quat_xyzw_to_matrix(plug[3:])
        clip=self.object_pose(f'clip_{SERVICE}');self.measured_clip=clip.copy()
        if self.release:
            self.clip_constraints={str(i):clip_joint_geometry(plug if i==SERVICE else self.object_pose(f'plug_{i}'),
                                                            clip if i==SERVICE else self.object_pose(f'clip_{i}')) for i in USED_PORTS}
            self.clip_constraints_safe=all(np.isfinite(list(v.values())).all() and v['relative_angle_deg']<=.5
                                           and v['lateral_anchor_error_m']<=.0001 for v in self.clip_constraints.values())
            if t>=2. and not self.clip_constraints_safe:self.failure='controlled connector exceeded 0.5 degree / 0.1 mm slider constraint guard'
        travel=float(CLIP_Z-(rotation.T@(clip[:3]-plug[:3]))[2])
        age=t-self.phase_start;phase=self.phase
        if self.hold_offset is None and phase in ('approach_boot','narrow_in_front_of_boot','enter_boot','close_on_boot'):
            # Cable load settles the real plug at a pitch/yaw before contact.
            # Track its actual boot centreline, while keeping this reachable
            # wrist orientation fixed in the world. The pad reaches forward
            # from this rearward/outboard TCP onto the rear of the ordinary boot.
            self.boot_goal=plug[:3]+rotation@BOOT_LOCAL
            self.boot_rotation=R_GRIP
            self.front_offset=rotation@np.array((-.040,0,0))
        lp,lr,lg=self.boot_goal.copy(),self.boot_rotation,1. if getattr(self,'tracking_only',False) else GRIP
        rp,rr=self.starts[1];rg=1.
        left_actual=measured[0][2:];right_actual=measured[1][2:]
        if self.hold_offset is not None:
            actual_offset=rotation.T@(measured[0][2]-plug[:3])
            self.grasp_drift=float(np.linalg.norm(actual_offset-self.hold_offset))
            if self.grasp_drift>.006:self.failure='boot slipped more than 6 mm relative to measured grasp'
        if travel>CLIP_TRAVEL+.0002:self.failure='clip exceeded physical travel limit'
        # Only the certified front standoff uses a wider pose gate. Geometry
        # uses actual rotation/fingers and includes all 84 measured cables.
        if phase in ('approach_boot','narrow_in_front_of_boot') and age>=2.:
            front=self.boot_goal+self.front_offset
            if self.settled(left_actual,(front,self.boot_rotation),.003,3.):
                state=snapshot(self.object_pose)
                sweep=gripper_sweep(self.geometry,measured[0][1],measured[0][2],measured[0][3],end_fingers=[-.0065,-.0065])
                self.free_clearance=clearance(sweep,state)
        free_ok=self.free_clearance is not None and self.free_clearance['minimum_gap_m']>=.005
        stationary_phases=('close_on_boot','verified_boot_hold','prepare_press','approach_above_clip','prepress_endpoint','depress_clip','withdraw_with_clip_pressed')
        if phase in stationary_phases and (self.entry_joint_servo.reference is not None or self.release):
            self.stationary_hold_clearance=self.stationary_hold_safety(measured)
            self.stationary_hold_safe=(self.stationary_hold_clearance['rigid_gap_m']>=.0002
                                       and self.stationary_hold_clearance['other_hand_gap_m']>=.0002)
            # The reference remains fixed, but its bounded feedback must reject
            # changed load while both jaws make contact and while holding.
            if self.entry_joint_servo.reference is not None:
                if phase=='withdraw_with_clip_pressed':self.entry_joint_servo.freeze()
                else:self.entry_joint_servo.update(measured[0][0],dt,allowed=self.stationary_hold_safe and self.failure is None)
            if self.entry_joint_servo.invalid:self.failure=self.entry_joint_servo.invalid

        if phase=='approach_boot':
            p0,r0=self.starts[0];s=ik.smoothstep(min(1.,age/4.))
            front=self.boot_goal+self.front_offset
            lp,lr,lg=p0+s*(front-p0),ik.slerp_matrix(r0,self.boot_rotation,s),1.
            self.gate(free_ok and self.settled(left_actual,(front,self.boot_rotation),.002,2.),age,4.,5.,'narrow_in_front_of_boot',t)
        elif phase=='narrow_in_front_of_boot':
            lp=self.boot_goal+self.front_offset;lr=self.boot_rotation
            lg=1.+(ENTRY_GRIP-1.)*ik.smoothstep(min(1.,age/2.))
            jaw_ok=np.max(np.abs(measured[0][1]+.0065))<.001
            self.gate(free_ok and jaw_ok and self.settled(left_actual,(lp,lr),.002,2.),age,2.,5.,'enter_boot',t)
        elif phase=='enter_boot':
            lp=self.boot_goal+self.front_offset*(1-ik.smoothstep(min(1.,age/3.)));lg=ENTRY_GRIP
            entry_ok=False
            if age>=3. or np.linalg.norm(measured[0][2]-self.boot_goal)<.010:
                self.entry_clearance=clearance(gripper_sweep(self.geometry,measured[0][1],measured[0][2],measured[0][3]),snapshot(self.object_pose),wire_margin=.0005)
                # Flexible cable brushing is permitted. Only rigid socket,
                # own-boot and clip clearance blocks this entry stage.
                rigid_safe=self.entry_clearance['rigid_gap_m']>=.0005
                if age>=3.:
                    slow=self.left_tcp_speed is not None and self.left_tcp_speed<.002
                    self.entry_low_velocity_frames=self.entry_low_velocity_frames+1 if slow else 0
                    enabled=(self.entry_low_velocity_frames>=3 and rigid_safe
                             and np.linalg.norm(self.boot_goal-measured[0][2])<.003
                             and rotation_angle_deg(measured[0][3],lr)<2.)
                    if self.entry_joint_servo.reference is None and enabled:
                        # Latch ONE unbiased IK endpoint. Do not repeatedly
                        # invert a Cartesian target moved by the integrator.
                        reference,pe,re=ik.solve(self.boot_goal,ik.matrix_to_quat_xyzw(lr),measured[0][0],
                                                 base_pos=self.env.rig.bases['left'],iters=220,tol=1e-6)
                        self.entry_joint_servo.latch(reference);self.entry_reference_error=(float(pe),float(re))
                        self.entry_latched_pose=(self.boot_goal.copy(),lr.copy())
                    if self.entry_joint_servo.reference is not None:
                        self.entry_goal_drift=float(np.linalg.norm(self.boot_goal-self.entry_latched_pose[0]))
                        if self.entry_goal_drift>.0002:self.failure='fixed entry endpoint moved more than 0.2 mm; explicit replan required'
                        # Activation stays latched through its own corrective
                        # motion. Geometry/nonfinite guards still fail closed.
                        self.entry_joint_servo.update(measured[0][0],dt,allowed=rigid_safe and self.failure is None)
                        if self.entry_joint_servo.invalid:self.failure=self.entry_joint_servo.invalid
                entry_ok=(rigid_safe and self.entry_joint_servo.reference is not None and self.failure is None
                          and self.settled(left_actual,(self.boot_goal,lr),.0002,2.))
            if age<3.:
                self.entry_endpoint_correction.reset();self.entry_joint_servo.reset();self.entry_latched_pose=None
                self.bias[0][:]=0.;self.entry_low_velocity_frames=0
            # At 0.5/s, 8 s allows four integral time constants (~98%).
            # This is an endpoint-correction budget after the 3 s motion.
            self.gate(entry_ok,age,3.,8.,'close_on_boot',t)
        elif phase=='close_on_boot':
            lg=closing_opening(age,.0065,GRIP_OPENING)/abs(FINGER_OPEN)
            jaws=np.abs(measured[0][1]);jaw_ok=False
            jaw_stable=self.last_fingers is not None and np.max(np.abs(measured[0][1]-self.last_fingers))<.0001
            pose_stable=self.settled(left_actual,(lp,lr),.002,2.)
            self.closing_stable_frames=self.closing_stable_frames+1 if jaw_stable and pose_stable else 0
            command_opening=GRIP*abs(FINGER_OPEN)
            # The same 0.75 N obstruction margin must include gravity:
            # the upper jaw can support real contact with less raw PD error.
            # Current surfaces and both first-contact roots are still required.
            self.boot_force_gate=boot_contact_force_gate(jaws,command_opening,measured[0][3][2,0])
            blocked=self.boot_force_gate['blocked']
            if age>=4. and self.closing_stable_frames>=5 and blocked:
                self.boot_prediction=self.jaw_contact_cache.evaluate(plug,measured[0][2],measured[0][3],measured[0][1],t,
                                                                    command_opening,fresh=self.stable_frames>=4)
                expected=np.array(self.boot_prediction['opening_m'])
                jaw_ok=(all(self.boot_prediction['contact_possible']) and np.all(np.abs(jaws-expected)<=.0005)
                        and max(self.boot_prediction['actual_surface_gap_m'])<=.0006)
            self.gate(jaw_ok and jaw_stable and blocked and self.stationary_hold_safe,age,4.,5.,'prepare_press' if self.release else 'verified_boot_hold',t)
            if self.phase!=phase:
                self.capture_grasp(plug,rotation,measured[0])
                self.press_start=(measured[1][2].copy(),measured[1][3].copy())
        elif phase=='verified_boot_hold':
            pass
        elif self.release:
            lp,lr,lg,rp,rr,rg=self.release_targets(t,age,phase,plug,rotation,travel,measured,(lp,lr,lg,rp,rr,rg))
        else:raise RuntimeError('unknown grasp phase '+phase)

        self.last_fingers=measured[0][1].copy()
        targets=[(lp,lr,lg),(rp,rr,rg)];out=self.env._ready.detach().cpu().numpy().reshape(-1).copy();arms=[]
        contact=phase in ('enter_boot','close_on_boot','approach_above_clip','prepress_endpoint','depress_clip','withdraw_with_clip_pressed','finish_withdrawal','lift_press_finger','clear_press_finger','align_empty_target','insert_empty_target','retract_for_alignment','seat_and_release_clip','release_boot','retreat_hands','certify_open_clearance','retreat_partially_open','open_clear_hands','retention_rest')
        for i,side in enumerate(('left','right')):
            p,r,g=targets[i];actual,actualr=measured[i][2:];error=p-actual
            if self.cartesian_bias_enabled and not contact and np.linalg.norm(error)<.02 and rotation_angle_deg(actualr,r)<5.:
                cap=.0025 if i==0 and phase in ('approach_boot','narrow_in_front_of_boot') and free_ok else .001
                self.bias[i]=np.clip(self.bias[i]+.8*dt*error,-cap,cap)
            # Contact retains bounded learned gravity/stiction compensation;
            # there is no new integral and no decay that erases the preload.
            fixed_entry_phases=('enter_boot','close_on_boot','verified_boot_hold','prepare_press','approach_above_clip','prepress_endpoint','depress_clip')
            if i==0 and getattr(self,'tracking_only',False):
                self.q[i]=self.left_ready_joints.copy();pe=re=0.
            elif self.release and self.failure is not None:
                self.q[i]=self.previous[:6].copy() if i==0 else measured[i][0].copy();pe=re=0.
            elif i==1 and self.release and self.press_joint_servo.reference is not None and (
                    (phase=='approach_above_clip' and self.press_servo_stage=='HOVER') or
                    (phase=='prepress_endpoint' and self.press_servo_stage in ('PREPRESS','PREPRESS_MOTION')) or
                    (phase=='depress_clip' and self.press_servo_stage in ('CONTACT','CONTACT_MOTION'))):
                self.q[i]=self.press_joint_servo.command();pe,re=self.press_reference_error
            elif i==0 and self.entry_joint_servo.reference is not None and phase in fixed_entry_phases:
                self.q[i]=self.entry_joint_servo.command();pe,re=self.entry_reference_error
            elif i==0 and self.entry_joint_servo.reference is not None:
                self.entry_joint_servo.freeze()
                if not np.isfinite(self.entry_joint_servo.bias).all() or np.max(np.abs(self.entry_joint_servo.bias))>.020000000001:
                    self.failure='left transport bias is nonfinite or exceeds 20 mrad';self.q[i]=self.previous[:6].copy();pe=re=0.
                else:
                    if self.left_unbiased_q is None:self.left_unbiased_q=self.entry_joint_servo.reference.copy()
                    self.left_unbiased_q,pe,re=ik.solve(p,ik.matrix_to_quat_xyzw(r),self.left_unbiased_q,
                                                      base_pos=self.env.rig.bases[side],iters=110)
                    self.q[i]=self.left_unbiased_q+self.entry_joint_servo.bias
            else:
                self.q[i],pe,re=ik.solve(p+self.bias[i],ik.matrix_to_quat_xyzw(r),self.q[i],base_pos=self.env.rig.bases[side],iters=110)
                if i==1 and self.release and self.press_joint_servo.reference is not None:
                    if phase in ('prepress_endpoint','depress_clip','withdraw_with_clip_pressed'):self.q[i]=self.q[i]+self.press_joint_servo.bias
                    else:self.press_joint_servo.reset();self.press_servo_stage=None;self.press_latched_pose=None
            out[7*i:7*i+6],out[7*i+6]=self.q[i],g
            arms.append({'side':side,'tcp':actual.tolist(),'target':p.tolist(),'position_error':float(np.linalg.norm(error)),
                         'rotation_error_deg':rotation_angle_deg(actualr,r),'ik_position_error':float(pe),'ik_rotation_error':float(re),
                         'effective_ik_tcp_target':(p+self.bias[i]).tolist(),'actual_joints':measured[i][0].tolist(),'actual_finger_joints_m':measured[i][1].tolist(),'bias_m':self.bias[i].tolist()})
        delta=np.array([.04]*6+[.05]+[.04]*6+[.05]);out=np.clip(np.clip(out,self.previous-delta,self.previous+delta),self.env._low,self.env._high);self.previous=out
        for i,side in enumerate(('left','right')):
            command_p,command_r=ik.tcp_pose(out[7*i:7*i+6],self.env.rig.bases[side])
            arms[i]['bounded_command_fk_tcp']=command_p.tolist();arms[i]['bounded_command_fk_rotation']=command_r.tolist()
        report={'cartesian_bias_enabled':self.cartesian_bias_enabled,'bounded_action_14d':out.tolist(),'phase':phase,'phase_age_s':float(age),'next_phase':self.phase,'arms':arms,'position_error':arms[0]['position_error'],
                    'left_tcp':measured[0][2].tolist(),'left_target':lp.tolist(),'right_tcp':measured[1][2].tolist(),'right_target':rp.tolist(),
                    'entry_endpoint_bias_m':self.entry_endpoint_correction.value.tolist(),'entry_joint_reference':None if self.entry_joint_servo.reference is None else self.entry_joint_servo.reference.tolist(),'entry_joint_bias_rad':self.entry_joint_servo.bias.tolist(),'entry_joint_frozen':self.entry_joint_servo.frozen,'entry_latched_tcp':None if self.entry_latched_pose is None else self.entry_latched_pose[0].tolist(),'entry_goal_drift_m':self.entry_goal_drift,'left_joint_velocity_rad_s':self.left_joint_velocity,'entry_low_velocity_frames':self.entry_low_velocity_frames,'left_tcp_speed_m_s':self.left_tcp_speed,'boot_target_plug_local':BOOT_LOCAL.tolist(),'actual_boot_grasp_target':(plug[:3]+rotation@BOOT_LOCAL).tolist(),'clip_travel_m':travel,'grasp_drift_m':self.grasp_drift,'free_standoff_clearance':self.free_clearance,'entry_clearance':self.entry_clearance,'boot_jaw_prediction':self.boot_prediction,'boot_contact_force_gate':self.boot_force_gate,'hold_command_opening_m':GRIP_OPENING,'quasistatic_jaw_force_estimate_N':predicted_jaw_normal_forces(np.abs(measured[0][1]),lg*abs(FINGER_OPEN),measured[0][3][2,0]),'stationary_hold_clearance':self.stationary_hold_clearance,'hand_clearance_m':self.hand_clearance,'stable_frames':self.stable_frames,
                    'failure':self.failure,'complete':bool(self.failure is None and ((phase=='verified_boot_hold' and age>=1.) or (phase=='retention_rest' and age>=2.5 and self.retention_gate_s>=2.1))),
                    'events':self.events[-3:]}
        if self.release:
            point=self.point.copy();point[0]+=measured[1][1][0]+.028
            witness=measured[1][2]+measured[1][3]@point
            report.update({'measured_press_point':witness.tolist(),'measured_press_point_plug_local':(rotation.T@(witness-plug[:3])).tolist(),
                           'target_press_point':(rp+rr@point).tolist(),'actual_press_point_clip_local':self.actual_tab_witness,'press_depth_m':self.press_depth,'press_approach_certificate':self.press_approach_certificate,'flexible_press_contact_possible':bool(self.press_approach_certificate is not None and self.press_approach_certificate['wire_gap_m']<.0002),'trajectory_governor':self.trajectory_governor,'functional_handoff':self.functional_handoff_report,'press_servo_stage':self.press_servo_stage,'press_joint_reference':None if self.press_joint_servo.reference is None else self.press_joint_servo.reference.tolist(),'press_joint_bias_rad':self.press_joint_servo.bias.tolist(),'press_joint_frozen':self.press_joint_servo.frozen,'right_joint_velocity_rad_s':None if self.right_joint_velocity is None else self.right_joint_velocity.tolist(),'press_endpoint_low_velocity_frames':self.press_endpoint_low_velocity_frames,'press_endpoint_goal_drift_m':self.press_endpoint_goal_drift,'clip_joint_constraints':self.clip_constraints,'withdraw_target_distance_m':self.pull_distance,'measured_hook_front_x_m':getattr(self,'measured_hook_front',None),'guide_geometry':self.guide_geometry,'insertion_command_x_m':self.insertion_x,'insertion_retries':self.insertion_retries,'full_release_clearance':self.final_clearance,'released_hold_s':self.retention_gate_s})
            report.update({'right_native_joint_velocity_rad_s':None if self.right_native_joint_velocity is None else self.right_native_joint_velocity.tolist(),
                           'press_joint_d_rad':self.press_joint_servo.damping.tolist(),
                           'press_joint_total_correction_rad':np.clip(self.press_joint_servo.bias+self.press_joint_servo.damping,-.02,.02).tolist(),
                           'press_joint_conditional_integral_mask':self.press_joint_servo.integral_mask.tolist()})
        return out,report


class ReleaseProbe(GraspProbe):
    def __init__(self,env,plug):
        super().__init__(env,plug);self.release=True
        spec=json.loads(Path(__file__).with_name('press_point.json').read_text())
        self.point=np.array(spec['point_tcp']);self.pressR=np.array(spec['rotation'])
        self.local_contact=np.array(spec['contact'])
        self.press_depth=0.;self.pull_distance=0.;self.source_clear=False;self.target_origin=np.array(port(TARGET))
        self.alignment_bias=np.zeros(3);self.alignment_rotation_bias=np.zeros(3);self.insertion_x=None;self.insertion_retries=0;self.bad_fit_frames=0
        self.guide_geometry=None;self.final_clearance=None;self.retention_gate_s=0.;self.retention_last_position=None;self.escape_count=0
        self.press_approach_certificate=None
        self.actual_tab_witness=None
        self.press_joint_servo=JointEndpointCorrection();self.press_latched_pose=None;self.press_reference_error=(0.,0.)
        self.last_right_joints=None;self.right_joint_velocity=None;self.right_native_joint_velocity=None;self.press_endpoint_low_velocity_frames=0;self.press_endpoint_goal_drift=0.
        self.clip_constraints={};self.clip_constraints_safe=True
        self.press_servo_stage=None
        self.current_press_obstacles=None;self.current_press_left=None;self.prepress_path=None;self.trajectory_governor=None
        self.contact_path=None
        self.functional_errors=[];self.functional_handoff_report=None;self.hover_departure=None;self.prepress_last_q3_movement=None

    def stationary_press_correction(self,position,rotation,measured,stage,travel,exclude=()):
        slow=self.right_joint_velocity is not None and np.isfinite(self.right_joint_velocity).all() and np.max(np.abs(self.right_joint_velocity))<.01
        self.press_endpoint_low_velocity_frames=self.press_endpoint_low_velocity_frames+1 if slow else 0
        if self.press_servo_stage!=stage and self.press_endpoint_low_velocity_frames>=3 and self.failure is None and self.clip_constraints_safe:
            if self.certify_press_path(measured,position,rotation,'stationary_'+stage.lower(),exclude):
                reference,pe,re=ik.solve(position,ik.matrix_to_quat_xyzw(rotation),measured[1][0],base_pos=self.env.rig.bases['right'],iters=220,tol=1e-6)
                if pe>.0001 or re>.001:self.failure='stationary press endpoint IK exceeds 0.1 mm / 1 mrad'
                else:
                    # Reacquire the NEW unbiased endpoint once. The moving
                    # stroke carries the frozen hover bias, never its q*.
                    self.press_joint_servo.latch(reference,preserve_bias=stage in ('PREPRESS','PRESS_ENDPOINT'))
                    self.press_reference_error=(float(pe),float(re));self.press_latched_pose=(position.copy(),rotation.copy())
                    self.press_servo_stage=stage;self.functional_errors=[]
        if self.press_joint_servo.reference is not None and self.press_servo_stage==stage:
            self.press_endpoint_goal_drift=float(np.linalg.norm(position-self.press_latched_pose[0]))
            if self.press_endpoint_goal_drift>.0002:self.failure='fixed press endpoint moved more than 0.2 mm; explicit replan required'
            position,rotation=self.press_latched_pose
            self.press_joint_servo.update(measured[1][0],getattr(self,'command_dt',.05),
                                          allowed=self.failure is None and self.clip_constraints_safe and travel<RELEASE_TRAVEL)
            if self.press_joint_servo.invalid:self.failure=self.press_joint_servo.invalid
        return position,rotation

    def press_endpoint_ready(self,measured,position,rotation,stage):
        reference=self.press_joint_servo.reference
        stationary=bool(reference is not None and self.press_servo_stage==stage
                        and self.right_joint_velocity is not None and np.isfinite(self.right_joint_velocity).all()
                        and np.max(np.abs(self.right_joint_velocity))<=.002)
        if not stationary:self.functional_errors=[];return False
        pieces=self.geometry.pieces(measured[1][1])
        error=max(float(np.linalg.norm(a@measured[1][3].T+measured[1][2]-(a@rotation.T+position),axis=2).max()) for _,a in pieces)
        if not np.isfinite(error):self.functional_errors=[];return False
        self.functional_errors=(self.functional_errors+[error])[-5:]
        return True

    def rigid_motion_certificate(self,measured,start,end,pieces,exclude=()):
        """Fresh non-target rigid/other-hand certificate; wires are not reserved."""
        obstacles=self.current_press_obstacles
        if obstacles is None:
            plugs={i:self.object_pose(f'plug_{i}') for i in USED_PORTS}
            clips={i:self.object_pose(f'clip_{i}') for i in USED_PORTS}
            obstacles=object_obstacles(plugs,clips,include_wires=False)
        left=self.current_press_left
        if left is None:left=[a@measured[0][3].T+measured[0][2] for _,a in self.geometry.pieces(measured[0][1])]
        sweep=[np.concatenate([a@ik.slerp_matrix(start[1],end[1],s).T+start[0]+s*(end[0]-start[0])
                               for s in np.linspace(0.,1.,5)]) for _,a in pieces]
        angle=np.radians(rotation_angle_deg(start[1],end[1]));radius=max(float(np.linalg.norm(a,axis=2).max()) for _,a in pieces)
        return {'rigid_gap_m':minimum_separation(sweep,[a for owner,name,a in obstacles if (owner,name) not in exclude]),
                'other_hand_gap_m':minimum_separation(sweep,left),
                'rotation_chord_bound_m':radius*(1.-np.cos(angle/8.))}

    def governed_prepress(self,measured,endpoint,rotation,*,contact=False):
        """Adaptive path clock plus all-six-joint PI, with full-mesh reserve."""
        if self.failure is not None:return measured[1][2],measured[1][3],False
        base=self.env.rig.bases['right'];dt=getattr(self,'command_dt',.05)
        path=self.contact_path if contact else self.prepress_path
        if path is None:
            reference=self.press_joint_servo.reference
            if reference is None:self.failure='press trajectory requires an unbiased reference';return measured[1][2],measured[1][3],False
            start=ik.tcp_pose(reference,base)
            endq,pe,re=ik.solve(endpoint,ik.matrix_to_quat_xyzw(rotation),reference,base_pos=base,iters=220,tol=1e-6)
            if pe>.0001 or re>.001:
                self.failure='press trajectory endpoint IK is infeasible';return start[0],start[1],False
            end=ik.tcp_pose(endq,base)
            duration=PREPRESS_DURATION
            if contact:
                point=self.point.copy();point[0]+=measured[1][1][0]+.028
                # Smoothstep peaks at 1.5/duration. Include rotational travel
                # of the physical support point in its speed upper bound.
                distance=np.linalg.norm(end[0]-start[0])+np.linalg.norm(point)*np.radians(rotation_angle_deg(start[1],end[1]))
                if not np.isfinite(distance) or distance<=0:
                    self.failure='contact trajectory has invalid remaining distance';return start[0],start[1],False
                duration=1.5*float(distance)/.0007
            path={'clock':0.,'reference':reference.copy(),'start':start,'end':end,'target':endpoint.copy(),'duration':duration}
            if contact:self.contact_path=path
            else:self.prepress_path=path
            self.functional_errors=[]
        duration=path.get('duration',PREPRESS_DURATION)
        exclude=((f'clip_{SERVICE}','tab'),(f'clip_{SERVICE}','hook')) if contact else ()
        def certificate(start,end):
            if contact:return self.rigid_motion_certificate(measured,start,end,pieces,exclude=exclude)
            return self.rigid_motion_certificate(measured,start,end,pieces)
        if np.linalg.norm(endpoint-path['target'])>.0002:self.failure='prepress trajectory endpoint moved more than 0.2 mm'
        pieces=self.geometry.pieces(measured[1][1]);current=ik.tcp_pose(path['reference'],base)
        remaining=certificate(current,path['end'])
        budget=min(remaining['rigid_gap_m'],remaining['other_hand_gap_m'])-remaining['rotation_chord_bound_m']-.0002
        error=mesh_pose_distance(pieces,measured[1][2:],current)
        if not np.isfinite([budget,error]).all():
            self.failure='nonfinite prepress trajectory tracking budget';return current[0],current[1],False
        allowance=mesh_reference_step_allowance(budget,error)
        old_clock=path['clock'];chosen_clock=old_clock;chosen_q=path['reference'];chosen_pose=current;step=0.
        low=old_clock;high=min(duration,old_clock+dt) if allowance>0. else old_clock
        for attempt in range(11):
            candidate_clock=high if attempt==0 else (low+high)/2.
            if candidate_clock-old_clock<1e-9:break
            s=ik.smoothstep(candidate_clock/duration)
            p=path['start'][0]+s*(path['end'][0]-path['start'][0]);r=ik.slerp_matrix(path['start'][1],path['end'][1],s)
            q,pe,re=ik.solve(p,ik.matrix_to_quat_xyzw(r),path['reference'],base_pos=base,iters=220,tol=1e-6)
            pose=ik.tcp_pose(q,base);distance=mesh_pose_distance(pieces,current,pose)
            if pe<=.0001 and re<=.001 and distance<=allowance:
                chosen_clock,chosen_q,chosen_pose,step=candidate_clock,q,pose,distance;low=candidate_clock
                if attempt==0:break
            else:high=candidate_clock
        correction=certificate(measured[1][2:],chosen_pose)
        safe=bool(min(correction['rigid_gap_m'],correction['other_hand_gap_m'])-correction['rotation_chord_bound_m']>=.0002)
        self.trajectory_governor={'path_clock_s':chosen_clock,'duration_s':duration,'mesh_error_m':error,'contact':contact,
                                 'tracking_budget_m':budget,'allowed_mesh_step_m':allowance,'accepted_mesh_step_m':step,
                                 'nominal_remaining':remaining,'actual_to_reference':correction,'correction_safe':safe,
                                 'unbiased_reference':chosen_q.tolist(),'at_end':chosen_clock>=duration-1e-9,
                                 'scope':'Rigid geometry and other hand; no wire error reserve'}
        if not np.isfinite([budget,error,allowance,step]).all() or not safe or self.failure is not None:
            self.failure=self.failure or 'prepress trajectory correction lacks finite 0.2 mm rigid/hand clearance'
            return current[0],current[1],False
        path['clock'],path['reference']=chosen_clock,chosen_q.copy()
        lower=np.maximum(self.env._low[7:13],self.previous[7:13]-.04);upper=np.minimum(self.env._high[7:13],self.previous[7:13]+.04)
        self.press_joint_servo.track(chosen_q,measured[1][0],dt,lower,upper,velocity=self.right_native_joint_velocity)
        if self.press_joint_servo.invalid:self.failure=self.press_joint_servo.invalid
        at_end=chosen_clock>=duration-1e-9
        stage=('CONTACT' if at_end else 'CONTACT_MOTION') if contact else ('PREPRESS' if at_end else 'PREPRESS_MOTION')
        if self.press_servo_stage!=stage:self.functional_errors=[]
        self.press_servo_stage=stage
        return chosen_pose[0],chosen_pose[1],at_end

    def functional_press_certificate(self,measured,next_target,next_rotation,next_phase,stage,position,rotation,plug,bodyR,point,exclude):
        """Use measured geometry and progress, rather than all-joint precision."""
        if len(self.functional_errors)<5:return False
        witness=measured[1][2]+measured[1][3]@point
        local=bodyR.T@(witness-plug[:3]);next_witness=next_target+next_rotation@point
        down=float((witness-next_witness)@bodyR[:,2])
        qnext,pe,re=ik.solve(next_target,ik.matrix_to_quat_xyzw(next_rotation),self.press_joint_servo.reference,
                             base_pos=self.env.rig.bases['right'],iters=220,tol=1e-6)
        dq3=float(qnext[2]-measured[1][0][2]);error=max(self.functional_errors)
        report={'stage':stage,'max_mesh_error_5_frames_m':error,'next_witness_down_m':down,'next_q3_delta_rad':dq3,
                'wire_scope':'Measured gaps only; error reserve applies to non-target rigid geometry and other hand', 'accepted':False}
        self.functional_handoff_report=report
        if not np.isfinite([down,dq3,pe,re]).all() or pe>.0001 or re>.001 or down<=0. or dq3>=0.:return False
        if stage=='PREPRESS':
            if self.hover_departure is None:return False
            departed=self.hover_departure
            progress=float(departed['witness_local_z_m']-local[2]);qprogress=float(departed['q3_rad']-measured[1][0][2])
            report.update({'measured_down_progress_m':progress,'required_down_progress_m':.5*departed['planned_down_m'],
                           'measured_negative_q3_progress_rad':qprogress,'required_negative_q3_progress_rad':-.5*departed['planned_q3_delta_rad'],
                           'last_substantial_q3_movement_rad':self.prepress_last_q3_movement})
            if (progress<.5*departed['planned_down_m'] or qprogress<-.5*departed['planned_q3_delta_rad']
                    or self.prepress_last_q3_movement is None or self.prepress_last_q3_movement>=0.):return False
        # A failed nominal rigid reserve already rejects the conjunction.
        # Avoid actual and nominal cable projections on these waiting ticks.
        report['actual_path']=None;self.press_approach_certificate=None
        pieces=self.geometry.pieces(measured[1][1])
        planned=self.rigid_motion_certificate(measured,(position,rotation),(next_target,next_rotation),pieces,exclude=exclude)
        report['nominal_path']={**planned,'wire_scope':'wire_not_evaluated; nominal reserve uses rigid geometry and other hand only'}
        reserve=min(planned['rigid_gap_m'],planned['other_hand_gap_m'])-planned['rotation_chord_bound_m']-error
        report['non_target_rigid_and_hand_reserve_m']=reserve
        if not np.isfinite([planned['rigid_gap_m'],planned['other_hand_gap_m'],planned['rotation_chord_bound_m'],error,reserve]).all() or reserve<.0002:return False
        actual_safe=self.certify_press_path(measured,next_target,next_rotation,next_phase,exclude)
        actual=dict(self.press_approach_certificate);report['actual_path']=actual
        if not actual_safe:return False
        accepted=True;report['accepted']=accepted
        if accepted and stage=='HOVER':
            self.hover_departure={'witness_local_z_m':float(local[2]),'q3_rad':float(measured[1][0][2]),
                                  'planned_down_m':down,'planned_q3_delta_rad':dq3}
            self.prepress_last_q3_movement=None
        return accepted

    def certify_press_path(self,measured,target,rotation,next_phase,exclude=()):
        """Certify the complete next leg from the actual loaded hand poses.

        Convex hulls of sampled rigid poses cover translation. Deduct the
        rotational chord error between samples to bound the continuous sweep.
        Flexible cable gaps are reported, while rigid/mutual gaps gate motion.
        """
        right=measured[1];start,startR=right[2:];pieces=self.geometry.pieces(right[1])
        if not all(np.isfinite(v).all() for v in (start,startR,target,rotation,right[1],*measured[0])):
            self.failure='nonfinite press approach geometry';return False
        fractions=np.linspace(0.,1.,5)
        sweep=[np.concatenate([a@ik.slerp_matrix(startR,rotation,s).T+start+s*(target-start)
                               for s in fractions]) for _,a in pieces]
        angle=np.radians(rotation_angle_deg(startR,rotation))
        radius=max(float(np.linalg.norm(a,axis=2).max()) for _,a in pieces)
        chord_bound=radius*(1.-np.cos(angle/8.))
        cert=clearance(sweep,snapshot(self.object_pose),wire_margin=.0002,exclude=exclude)
        left=[a@measured[0][3].T+measured[0][2] for _,a in self.geometry.pieces(measured[0][1])]
        cert.update({'other_hand_gap_m':minimum_separation(left,sweep),
                     'rotation_chord_bound_m':chord_bound,'next_phase':next_phase,
                     'intended_contact_exclusions':[list(pair) for pair in exclude]})
        self.press_approach_certificate=cert
        safe=(min(cert['rigid_gap_m'],cert['other_hand_gap_m'])-chord_bound>=.0002)
        if not safe:self.failure='press approach lacks measured 0.2 mm rigid/interhand sweep clearance'
        return safe

    def press_handoff(self,condition,age,duration,next_phase,t,measured,next_target,rotation,exclude=(),timeout=5.,functional=None):
        if condition and age>=duration and self.stable_frames>=4:
            if functional is None:condition=self.certify_press_path(measured,next_target,rotation,next_phase,exclude)
            else:condition=self.functional_press_certificate(measured,next_target,rotation,next_phase,*functional,exclude)
            if next_phase=='depress_clip' and self.press_approach_certificate is not None:
                self.hand_clearance=self.press_approach_certificate['other_hand_gap_m']
            if condition and next_phase in ('prepress_endpoint','depress_clip'):
                if next_phase=='depress_clip':
                    # Carry the applied I+D preload once into the next leg.
                    self.press_joint_servo.bias=np.clip(self.press_joint_servo.bias+self.press_joint_servo.damping,-.02,.02)
                self.press_joint_servo.freeze();self.press_endpoint_low_velocity_frames=0
        self.gate(condition,age,duration,timeout,next_phase,t)

    def release_targets(self,t,age,phase,plug,bodyR,travel,measured,targets):
        lp,lr,lg,rp,rr,rg=targets
        point=self.point.copy()
        # Selected support is tip_left, also when used by the RIGHT arm.
        point[0]+=measured[1][1][0]+.028
        rr=bodyR@self.pressR;rg=PRESS_GRIP
        local_contact=self.local_contact
        above=plug[:3]+bodyR@(local_contact+np.array((0,0,.0041)))-rr@point
        prepress=above-bodyR@np.array((0,0,PREPRESS_DISTANCE))
        front=above+bodyR@np.array((-.040,0,0))
        far=front+bodyR@np.array((0,0,.025))
        actual_left,actual_right=measured[0][2:],measured[1][2:]
        clip_pose=getattr(self,'measured_clip',None)
        if clip_pose is None:clip_pose=self.object_pose(f'clip_{SERVICE}')
        witness=measured[1][2]+measured[1][3]@point
        self.actual_tab_witness=(quat_xyzw_to_matrix(clip_pose[3:]).T@(witness-clip_pose[:3])).tolist()
        tx,ty,_=self.actual_tab_witness
        tab_xy_safe=bool(np.isfinite(self.actual_tab_witness).all() and -.0127<=tx<=.0067 and abs(ty)<=.0022)
        left_ok=self.settled(actual_left,(lp,lr),.0015)
        jaw_ok=np.max(np.abs(measured[1][1]-PRESS_JAW))<.001
        actual_right_safe=(self.stationary_hold_clearance is None or
                           self.stationary_hold_clearance.get('right_non_target_rigid_gap_m',float('inf'))>=.0002)
        if phase in ('approach_above_clip','prepress_endpoint','depress_clip','withdraw_with_clip_pressed') and (not actual_right_safe or not self.stationary_hold_safe):
            self.failure='measured stationary hold lost rigid/interhand clearance during press approach'
            return lp,lr,lg,measured[1][2],measured[1][3],rg
        if phase in ('depress_clip','withdraw_with_clip_pressed') and not tab_xy_safe:
            self.failure='actual pressing witness left physical clip tab footprint'
            return lp,lr,lg,measured[1][2],measured[1][3],rg

        if phase=='prepare_press':
            p0,r0=self.press_start;s=ik.smoothstep(min(1.,age/4.));rp=p0+s*(far-p0);rr=ik.slerp_matrix(r0,rr,s);rg=1.+(PRESS_GRIP-1.)*s
            # This is a clear free-space standoff. Its wider settle gate still
            # requires a fresh complete next-leg rigid/interhand certificate.
            self.press_handoff(self.settled(actual_right,(far,bodyR@self.pressR),.002,2.) and jaw_ok and left_ok,
                               age,4.,'approach_above_clip',t,measured,above,bodyR@self.pressR)
        elif phase=='approach_above_clip':
            # A direct diagonal bypasses the loaded left jaw at the rear
            # standoff, without an unreachable high-forward waypoint.
            rp=far+(above-far)*ik.smoothstep(min(1.,age/4.))
            if age>=4.:rp,rr=self.stationary_press_correction(above,rr,measured,'HOVER',travel)
            settled=self.press_endpoint_ready(measured,rp,rr,'HOVER') and jaw_ok and left_ok and tab_xy_safe
            self.press_handoff(settled,age,4.,'prepress_endpoint',t,measured,prepress,rr,timeout=12.,
                               functional=('HOVER',rp,rr,plug,bodyR,point))
        elif phase=='prepress_endpoint':
            # Reverse the shoulder's direction while still above the
            # tab, then settle its bounded fixed-reference compensation.
            rp,rr,at_end=self.governed_prepress(measured,prepress,rr)
            self.press_depth=-.0041+PREPRESS_DISTANCE*ik.smoothstep(self.prepress_path['clock']/PREPRESS_DURATION) if self.prepress_path is not None else -.0041
            settled=at_end and self.press_endpoint_ready(measured,rp,rr,'PREPRESS') and jaw_ok and left_ok and tab_xy_safe
            # The clip is intended contact; all housing, guides, plugs and
            # other clips must clear the COMPLETE stroke before it begins.
            self.press_handoff(settled,age,PREPRESS_DURATION,'depress_clip',t,measured,
                               above-bodyR@np.array((0,0,.0064)),rr,
                               ((f'clip_{SERVICE}','tab'),(f'clip_{SERVICE}','hook')),timeout=12.,
                               functional=('PREPRESS',rp,rr,plug,bodyR,point))
        elif phase=='depress_clip':
            endpoint=plug[:3]+bodyR@(local_contact-np.array((0,0,.0023)))-rr@point
            rp,rr,at_endpoint=self.governed_prepress(measured,endpoint,rr,contact=True)
            self.press_depth=float(local_contact[2]-(bodyR.T@(rp+rr@point-plug[:3]))[2])
            if self.failure is not None:return lp,lr,lg,rp,rr,rg
            self.stable_frames=self.stable_frames+1 if travel>=RELEASE_TRAVEL else 0
            if self.stable_frames>=5:
                self.press_joint_servo.bias=np.clip(self.press_joint_servo.bias+self.press_joint_servo.damping,-.02,.02)
                self.press_joint_servo.freeze()
                if self.entry_joint_servo.reference is not None:
                    self.entry_joint_servo.freeze();self.left_unbiased_q=self.entry_joint_servo.reference.copy()
                    self.withdraw_start,self.withdraw_rotation=ik.tcp_pose(self.left_unbiased_q,self.env.rig.bases['left'])
                else:self.withdraw_start,self.withdraw_rotation=lp.copy(),lr.copy()
                self.withdraw_plug=plug[:3].copy();self.pull_distance=0.;self.transition('withdraw_with_clip_pressed',t)
            elif age>18.5:self.failure='clip did not reach release travel within 2.3 mm stroke and bounded endpoint correction'
        elif phase=='withdraw_with_clip_pressed':
            lr=getattr(self,'withdraw_rotation',lr)
            # Maintain real clip travel while advancing the grasp target slowly;
            # the pressing hand follows the measured plug, not a stale point.
            self.press_depth=float(np.clip(self.press_depth+.12*(.00195-travel),.0015,.0023))
            rp=plug[:3]+bodyR@(local_contact-np.array((0,0,self.press_depth)))-rr@point
            if travel>=RELEASE_TRAVEL-.0001:self.pull_distance=min(.038,self.pull_distance+.0015*.05)
            lp=self.withdraw_start+np.array((-self.pull_distance,0,0))
            # Only the real hook must remain depressed while crossing the
            # retaining ledge. Following the plug all the way out makes the
            # spare jaw collide with a neighbouring roof. Read the actual hook
            # front before raising the pressing finger, then finish withdrawal.
            clip_pose=getattr(self,'measured_clip',None)
            if clip_pose is None:
                hook_front=max((plug[:3]+bodyR@np.array((x,y,CLIP_Z-travel+z)))[0]
                               for x,y,z in ((.006,-.0025,.0005),(.006,.0025,.0005),(.006,-.0025,.0025),(.006,.0025,.0025),(.010,-.0025,.0005),(.010,.0025,.0005)))
            else:
                clipR=quat_xyzw_to_matrix(clip_pose[3:])
                hook_front=max((clip_pose[:3]+clipR@np.array(v))[0]
                               for v in ((.006,-.0025,.0005),(.006,.0025,.0005),(.006,-.0025,.0025),(.006,.0025,.0025),(.010,-.0025,.0005),(.010,.0025,.0005)))
            self.measured_hook_front=float(hook_front)
            if hook_front<.407-.0005:
                self.withdraw_end=lp.copy();self.press_retreat=(rp.copy(),rr.copy());self.transition('lift_press_finger',t)
            elif age>15.:self.failure='actual hook did not clear retaining ledge while clip held depressed'
            elif travel<RELEASE_TRAVEL-.0002 and age>1.:
                # Pause withdrawal when the measured clip springs back; never
                # continue a timed pull through an engaged hook.
                if age>6. and self.pull_distance<.001:self.failure='clip depression could not be maintained for withdrawal'
        elif phase=='lift_press_finger':
            lp=self.withdraw_end;lr=getattr(self,'withdraw_rotation',lr)
            p0,r0=self.press_retreat;rp=p0+np.array((0,0,.020))*ik.smoothstep(min(1.,age/2.));rr=r0
            self.gate(self.settled(actual_right,(rp,rr)),age,2.,5.,'finish_withdrawal',t)
        elif phase=='finish_withdrawal':
            lr=getattr(self,'withdraw_rotation',lr)
            self.pull_distance=min(.038,self.pull_distance+.002*.05)
            lp=self.withdraw_start+np.array((-self.pull_distance,0,0))
            p0,r0=self.press_retreat;rp=p0+np.array((0,0,.020));rr=r0
            tip=plug[:3]+bodyR@np.array((.012,0,0))
            if tip[0]<(.412-.027/2)-.003:
                self.source_clear=True;self.withdraw_end=lp.copy();self.transition('clear_press_finger',t)
            elif age>15.:self.failure='plug tip did not physically clear source housing after hook release'
        elif phase=='clear_press_finger':
            lp=self.withdraw_end;lr=getattr(self,'withdraw_rotation',lr)
            p0,r0=self.press_retreat;rp=p0+np.array((0,0,.020))+np.array((-.080,0,.030))*ik.smoothstep(min(1.,age/3.));rr=r0
            self.gate(self.settled(actual_right,(rp,rr)),age,3.,5.,'align_empty_target',t)
            if self.phase!=phase:
                self.right_clear=(rp.copy(),rr.copy());self.align_start=lp.copy();self.align_plug_start=plug[:3].copy();self.align_body_start=bodyR.copy()
        elif phase=='align_empty_target':
            rp,rr=self.right_clear
            pretarget=self.target_origin+np.array((-.038,0,0));s=ik.smoothstep(min(1.,age/3.))
            # Learn only outside the mouth. This compensates actual plug pose,
            # rather than declaring a 1.5 mm TCP residual to be guide alignment.
            if age>=3.:
                tip=plug[:3]+bodyR@np.array((.012,0,0))
                if tip[0]>(.412-.027/2)-.003:self.failure='target alignment attempted before plug cleared mouth'
                error=pretarget-plug[:3]
                self.alignment_bias=np.clip(self.alignment_bias+.04*error,[-.001,-.0025,-.0025],[.001,.0025,.0025])
                self.alignment_rotation_bias=np.clip(self.alignment_rotation_bias+.02*ik._rot_error(np.eye(3),bodyR),-np.radians(1.5),np.radians(1.5))
            angle=np.linalg.norm(self.alignment_rotation_bias)
            correction=np.eye(3) if angle<1e-10 else axis_rotation(self.alignment_rotation_bias/angle,angle)
            command=self.align_plug_start+s*(pretarget-self.align_plug_start)+self.alignment_bias
            body_goal=correction@ik.slerp_matrix(self.align_body_start,np.eye(3),s)
            lp,lr=plug_hand_target(command,body_goal,plug,measured[0][2],measured[0][3])
            self.guide_geometry=plug_fit(plug,TARGET)
            ready=(self.guide_geometry['minimum_clearance_m']>=.0001 and abs(plug[0]-pretarget[0])<.002)
            self.gate(ready,age,3.,8.,'insert_empty_target',t)
            if self.phase!=phase:self.insertion_x=float(plug[0]);self.bad_fit_frames=0;self.frozen_alignment_rotation=body_goal.copy()
        elif phase=='insert_empty_target':
            rp,rr=self.right_clear;self.guide_geometry=plug_fit(plug,TARGET)
            fit=self.guide_geometry
            # Guide contact is ordinary insertion contact. Stop advancing for
            # substantial transverse interference, and retract to realign;
            # no correction integral is accumulated against a wall.
            guide_safe=(fit['minimum_clearance_m']>=-.0001 and fit['axis_angle_deg']<5. and fit['lateral_error_m']<.001)
            self.bad_fit_frames=self.bad_fit_frames+1 if not guide_safe else 0
            # Progress and lag use the same effective commanded X. A frozen
            # -1 mm alignment correction must not create a permanent 1 mm
            # apparent lag when the physical plug tracks that command exactly.
            effective_x=self.insertion_x+self.alignment_bias[0]
            if guide_safe and effective_x-plug[0]<.0007:self.insertion_x=min(self.target_origin[0],self.insertion_x+.003*.05)
            command=self.target_origin+np.array((self.insertion_x-self.target_origin[0],0,0))+self.alignment_bias
            lp,lr=plug_hand_target(command,self.frozen_alignment_rotation,plug,measured[0][2],measured[0][3])
            tip=plug[:3]+bodyR@np.array((.012,0,0))
            seated=(self.insertion_x>=self.target_origin[0]-1e-8 and tip[0]>(.412-.027/2)+.008
                    and fit['lateral_error_m']<.001 and fit['axis_angle_deg']<5.)
            self.gate(seated,age,0.,22.,'seat_and_release_clip',t)
            if self.bad_fit_frames>=3:
                if self.insertion_retries>=2:self.failure='actual plug/guide fit failed after two bounded realignments'
                else:
                    self.insertion_retries+=1;self.retract_plug_start=plug[:3].copy();self.retract_body_start=bodyR.copy();self.transition('retract_for_alignment',t)
        elif phase=='retract_for_alignment':
            rp,rr=self.right_clear;goal=self.target_origin+np.array((-.038,0,0));s=ik.smoothstep(min(1.,age/3.))
            command=self.retract_plug_start+s*(goal-self.retract_plug_start)
            lp,lr=plug_hand_target(command,self.retract_body_start,plug,measured[0][2],measured[0][3])
            clear=abs(plug[0]-goal[0])<.002
            self.gate(clear,age,3.,5.,'align_empty_target',t)
            if self.phase!=phase:self.align_plug_start=plug[:3].copy();self.align_body_start=bodyR.copy();self.alignment_bias[:]=0.;self.alignment_rotation_bias[:]=0.
        elif phase=='seat_and_release_clip':
            command=self.target_origin+self.alignment_bias;lp,lr=plug_hand_target(command,self.frozen_alignment_rotation,plug,measured[0][2],measured[0][3]);rp,rr=self.right_clear
            face=retaining_face(getattr(self,'measured_clip',self.object_pose(f'clip_{SERVICE}')),TARGET)
            fit=plug_fit(plug,TARGET)
            self.gate(travel<.0003 and face['retained'] and fit['axis_angle_deg']<5. and fit['lateral_error_m']<.001,age,1.,5.,'release_boot',t)
            if self.phase!=phase:self.seated_hand_target=(lp.copy(),lr.copy())
        elif phase=='release_boot':
            lp,lr=self.seated_hand_target;rp,rr=self.right_clear;lg=GRIP+(ENTRY_GRIP-GRIP)*ik.smoothstep(min(1.,age/2.))
            self.gate(np.max(np.abs(measured[0][1]+.0065))<.001,age,2.,5.,'retreat_hands',t)
            if self.phase!=phase:self.retreat_start=(lp.copy(),lr.copy());self.hold_offset=None
        elif phase=='retreat_hands':
            p0,r0=self.retreat_start;s=ik.smoothstep(min(1.,age/3.));lp=p0+np.array((-.11,0,0))*s;lr=r0;lg=ENTRY_GRIP;rp,rr=self.right_clear
            self.gate(self.settled(actual_left,(lp,lr),.002),age,3.,5.,'certify_open_clearance',t)
        elif phase=='certify_open_clearance':
            lp,lr=measured[0][2].copy(),measured[0][3].copy();lg=ENTRY_GRIP;rp,rr=self.right_clear
            state=snapshot(self.object_pose)
            left=gripper_sweep(self.geometry,measured[0][1],lp,lr,end_fingers=[FINGER_OPEN,FINGER_OPEN])
            right=[a@measured[1][3].T+measured[1][2] for _,a in self.geometry.pieces(measured[1][1])]
            self.final_clearance=clearance(left+right,state)
            mutual=minimum_separation(left,right)
            if self.final_clearance['minimum_gap_m']>=.005 and mutual>=.005:
                self.final_hand_target=(lp.copy(),lr.copy());self.transition('open_clear_hands',t)
            elif self.escape_count>=3:self.failure='full jaw opening lacks measured object/cable clearance after three retreats'
            else:
                # Remain partially open while choosing a complete, certified
                # escape translation. Fully open jaws are never swept blindly
                # through the hanging cables at the old straight-back height.
                for delta in (np.array((0,0,.080)),np.array((-.050,0,.060)),np.array((0,.080,.060))):
                    moving=gripper_sweep(self.geometry,measured[0][1],lp,lr,translation=delta)
                    cert=clearance(moving+right,state,wire_margin=.001)
                    if cert['minimum_gap_m']>=.001 and minimum_separation(moving,right)>=.001:
                        self.escape_start=(lp.copy(),lr.copy());self.escape_delta=delta;self.escape_count+=1;self.transition('retreat_partially_open',t);break
                else:self.failure='partially open hand has no certified cable-clear retreat translation'
        elif phase=='retreat_partially_open':
            p0,r0=self.escape_start;lp=p0+self.escape_delta*ik.smoothstep(min(1.,age/3.));lr=r0;lg=ENTRY_GRIP;rp,rr=self.right_clear
            self.gate(self.settled(actual_left,(lp,lr),.002),age,3.,5.,'certify_open_clearance',t)
        elif phase=='open_clear_hands':
            lp,lr=self.final_hand_target;rp,rr=self.right_clear;lg=ENTRY_GRIP+(1-ENTRY_GRIP)*ik.smoothstep(min(1.,age/1.5))
            self.gate(np.max(np.abs(measured[0][1]-FINGER_OPEN))<.001,age,1.5,5.,'retention_rest',t)
        elif phase=='retention_rest':
            lp,lr=self.final_hand_target;lg=1.;rp,rr=self.right_clear
            state=snapshot(self.object_pose);parts=[]
            for arm in measured:parts.extend(a@arm[3].T+arm[2] for _,a in self.geometry.pieces(arm[1]))
            self.final_clearance=clearance(parts,state)
            face=retaining_face(state[1][SERVICE],TARGET);fit=plug_fit(plug,TARGET)
            speed=0. if self.retention_last_position is None else np.linalg.norm(plug[:3]-self.retention_last_position)/.05
            self.retention_last_position=plug[:3].copy()
            ready=(self.final_clearance['minimum_gap_m']>=.005 and face['retained'] and travel<.0003 and fit['axis_angle_deg']<5.
                   and fit['lateral_error_m']<.001 and speed<.01 and all(np.linalg.norm(arm[2]-plug[:3])>.070 for arm in measured))
            self.retention_gate_s=self.retention_gate_s+.05 if ready else 0.
            if age>12.:self.failure='actual unheld latch/whole-gripper clearance did not maintain a released hold'
        else:raise RuntimeError('unknown release phase '+phase)
        return lp,lr,lg,rp,rr,rg


class PressTrackingProbe(ReleaseProbe):
    """Unscored right-only tracking prefix; the harness stops before contact."""
    def __init__(self,env,plug):
        super().__init__(env,plug)
        self.tracking_only=True
        self.phase='prepare_press';self.phase_start=0.
        self.boot_goal,self.boot_rotation=(value.copy() for value in self.starts[0])
        self.press_start=tuple(value.copy() for value in self.starts[1])
        self.left_ready_joints=self.previous[:6].copy()

    def start_contact(self,reference,applied_command):
        """Seed an unscored, naturally settled contact-only diagnostic."""
        reference=np.asarray(reference,dtype=float);applied=np.asarray(applied_command,dtype=float)
        if reference.shape!=(6,) or applied.shape!=(6,) or not np.isfinite([reference,applied]).all():
            raise ValueError('contact warm start requires finite six-joint reference and command')
        bias=applied-reference
        if np.any(np.abs(bias)>.02):raise ValueError('contact warm start correction exceeds 20 mrad')
        self.press_joint_servo.latch(reference);self.press_joint_servo.bias=bias.copy()
        if not np.array_equal(self.press_joint_servo.command(),applied):raise ValueError('contact warm start command continuity failed')
        self.previous[7:13]=applied;self.q[1]=applied.copy()
        self.phase='depress_clip';self.phase_start=0.;self.stable_frames=0
        self.press_servo_stage='PREPRESS';self.contact_path=None
