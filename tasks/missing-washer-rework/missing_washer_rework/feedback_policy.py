# SPDX-License-Identifier: MIT
"""Physical rework demo, using explicitly privileged measured poses.

Adapted from the MIT square-cut-line measured-tool controller. Every movement
is a joint/gripper command; no body is attached or teleported in an episode.
"""
from __future__ import annotations
from dataclasses import dataclass
from pathlib import Path
import json
import math
import numpy as np
from inspect_robots.policy import PolicyBase,PolicyInfo
from inspect_robots.types import Action,ActionChunk
from inspect_robots_yam.config import action_box
from inspect_robots_yam.packing import STATE_KEY,pack,split
from yam_arena import ik,station
from yam_arena.constants import READY,FINGER_OPEN,PAD_TIP_PAST_TCP
from yam_arena.demo import grip_rotation
from yam_arena.urdf import quat_xyzw_to_matrix
from . import contract as c

MAX_POSITION_LEAD=.030
MAX_ROTATION_LEAD=math.radians(18.)
def _bounded(vector,maximum):
 length=float(np.linalg.norm(vector));return vector*min(1.,maximum/max(length,1e-12))
def _rotation_angle(rotation):return math.acos(float(np.clip((np.trace(rotation)-1)/2,-1,1)))
@dataclass
class _ServoState:
 command_q:np.ndarray
 command_pos:np.ndarray
 command_rot:np.ndarray
 grasp_pos:np.ndarray
 grasp_rot:np.ndarray
 saturated_s:float=0.
 drift_s:float=0.

class MissingWasherReworkDemo(PolicyBase):
    def __init__(self,mode='demo',debug_path=None):
        if mode!='demo':raise ValueError('Only mode=demo is implemented')
        self._debug_path=Path(debug_path) if debug_path else None
        self._dt=.05;self._bases=station.BASES
        low,high=np.r_[ik.LOWER,0.],np.r_[ik.UPPER,1.]
        self.info=PolicyInfo(name='missing_washer_rework_demo',action_space=action_box(pack(low,low),pack(high,high)))
    def bind(self,info):
        if info.control_hz:self._dt=1./info.control_hz
    def reset(self,scene):
        self._last=(np.r_[READY,1.],np.r_[READY,1.]);self._servo={}
        self._phase=0;self._phase_start=None;self._start=None;self._hold=0.;self._motion=None
        self._captured={};self._debug_observation={};self._previous_grippers={}
        self._previous_tool_poses={};self._previous_observation_time=None
        self._release_prepare_states={};self._release_states={}
    def _next(self,now):
        self._phase+=1;self._phase_start=now;self._hold=0.;self._motion=None
    def _stable(self,condition,duration,dt):
        self._hold=self._hold+dt if condition else 0.
        return self._hold>=duration-1e-9
    def _tcp(self,arm,q,goal,opening,elapsed,duration):
        if self._motion is None:
            pos,rot=ik.tcp_pose(q[:6],self._bases[arm]);self._motion=(pos.copy(),rot.copy())
        start,rot=self._motion;s=ik.smoothstep(min(1.,elapsed/duration))
        desired=start+(np.asarray(goal)-start)*s
        rotation=ik.slerp_matrix(rot,grip_rotation(-math.pi/2 if arm=='left' else 0.),s)
        target,err,ang=ik.solve(desired,ik.matrix_to_quat_xyzw(rotation),self._last[0 if arm=='left' else 1][:6],base_pos=self._bases[arm],iters=48,tol=5e-5)
        return np.r_[target,opening],err,ang
    def _track(self, arm, q, tool_pose, tool_rot, goal_pos, goal_rot, dt, meta, *,
               press=False, feedforward=None):
        base = self._bases[arm]
        tcp_pos, tcp_rot = ik.tcp_pose(q, base)
        grasp_pos = tcp_rot.T @ (tool_pose[:3] - tcp_pos)
        grasp_rot = tcp_rot.T @ tool_rot
        state = self._servo.get(arm)
        if state is None:
            # Retain the last pickup setpoint and its load-bearing lead.
            command_q = self._last[0 if arm == "left" else 1][:6].copy()
            command_pos, command_rot = ik.tcp_pose(command_q, base)
            state = self._servo[arm] = _ServoState(
                command_q, command_pos, command_rot, grasp_pos, grasp_rot
            )

        error = goal_pos - tool_pose[:3]
        rot_error = goal_rot @ tool_rot.T
        error_angle = _rotation_angle(rot_error)
        correction = error if feedforward is None else error + feedforward
        velocity = _bounded(correction * 3., .020 if press else .14)
        if press:
            velocity[2] = np.clip(velocity[2], -.010, .010)
        fraction = min(.5 * dt, math.radians(10.) * dt / max(error_angle, 1e-12))
        command_rot = ik.slerp_matrix(np.eye(3), rot_error, fraction) @ state.command_rot
        # Rotate about the commanded tool root, so yaw correction does not
        # sweep the far end of the square sideways about the gripper.
        tool_command = state.command_pos + state.command_rot @ grasp_pos + dt * velocity
        lead = tool_command - command_rot @ grasp_pos - tcp_pos
        rotation_lead = _rotation_angle(command_rot @ tcp_rot.T)
        position_lead = float(np.linalg.norm(lead))
        saturated = position_lead > MAX_POSITION_LEAD or rotation_lead > MAX_ROTATION_LEAD
        state.saturated_s = state.saturated_s + dt if saturated else 0.
        grasp_drift = float(np.linalg.norm(grasp_pos - state.grasp_pos))
        grasp_angle = _rotation_angle(grasp_rot @ state.grasp_rot.T)
        slipped = grasp_drift > .010 or grasp_angle > math.radians(20.)
        state.drift_s = state.drift_s + dt if slipped else 0.
        if state.saturated_s >= 2. - 1e-9:
            meta["feedback_fault"] = f"{arm}_servo_lead_saturated"
        elif state.drift_s >= .4 - 1e-9:
            meta["feedback_fault"] = f"{arm}_grasp_drift"

        # Clamp lead in tool space and blend rotation by the same fraction
        # to preserve the pivot while backing away from the lead limit.
        blend = min(1., MAX_POSITION_LEAD / max(position_lead, 1e-12),
                    MAX_ROTATION_LEAD / max(rotation_lead, 1e-12))
        command_rot = ik.slerp_matrix(tcp_rot, command_rot, blend)
        tool_command = tool_pose[:3] + blend * (tool_command - tool_pose[:3])
        lead = _bounded(tool_command - command_rot @ grasp_pos - tcp_pos, MAX_POSITION_LEAD)
        if press:
            # Allow a small normal lead into the surface without windup.
            lead[2] = np.clip(lead[2], -.004, .006)
        command_pos = tcp_pos + lead
        target, pos_error, angular_residual = ik.solve(
            command_pos, ik.matrix_to_quat_xyzw(command_rot), state.command_q,
            base_pos=base, iters=32, tol=5e-5,
        )
        state.command_q, state.command_pos, state.command_rot = target, command_pos, command_rot
        meta.update({
            arm + "_ik_error_mm": pos_error * 1000.,
            arm + "_ik_error_deg": math.degrees(angular_residual),
            arm + "_tool_error_mm": (error * 1000.).tolist(),
            arm + "_tool_error_deg": math.degrees(error_angle),
            arm + "_lead_mm": position_lead * 1000.,
            arm + "_lead_deg": math.degrees(rotation_lead),
            arm + "_grasp_drift_mm": grasp_drift * 1000.,
            arm + "_grasp_drift_deg": math.degrees(grasp_angle),
        })
        return np.r_[target, 0.]


    def _chunk(self,meta):
        data=pack(*self._last)
        if self._debug_path is not None:
            with self._debug_path.open('a',encoding='utf-8') as stream:
                stream.write(json.dumps({**self._debug_observation,**meta,'commanded_joint_state':data.tolist()},allow_nan=False)+'\n')
        return ActionChunk(actions=[Action(data=data,meta=meta)],control_hz=1./self._dt)

    def act(self,observation):
        try:
            tools=observation.extra['demo_tools']
            poses={n:np.asarray(tools[n+'_pose'],dtype=float) for n in c.DYNAMIC_NAMES}
            if any(p.shape!=(7,) or not np.isfinite(p).all() for p in poses.values()):raise ValueError('invalid pose')
            left,right=split(np.asarray(observation.state[STATE_KEY],dtype=float))
            now=float(observation.state_time)
            if not math.isfinite(now):raise ValueError('invalid time')
        except (KeyError,TypeError,ValueError) as exc:
            return self._chunk(dict(request_stop=True,stop_reason='demo_feedback_unavailable',detail=str(exc)))
        if self._start is None:self._start=self._phase_start=now
        elapsed=now-self._phase_start
        fp=poses['fixture'][:3];fr=quat_xyzw_to_matrix(poses['fixture'][3:])
        cap=poses['cap'][:3];washer=poses['washer'][:3]
        self._debug_observation=dict(t=now,poses={k:v.tolist() for k,v in poses.items()},joints=pack(left,right).tolist())
        # Controls are short controlled approaches and smooth, faster free carries.
        fgrip=fp+fr@np.asarray(c.FIXTURE_GRIP_LOCAL)
        handle_top=fp+fr@(np.asarray(c.FIXTURE_GRIP_LOCAL)+np.array([0,0,c.FIXTURE_GRIP_SIZE[2]/2]))
        press_tcp=handle_top+np.array([0,0,PAD_TIP_PAST_TCP-.001])
        capgrip=cap+quat_xyzw_to_matrix(poses['cap'][3:])@np.asarray(c.CAP_GRIP_LOCAL)
        wgrip=washer+quat_xyzw_to_matrix(poses['washer'][3:])@np.asarray(c.WASHER_GRIP_LOCAL)
        park=np.array([c.PARK_POS[0],c.PARK_POS[1],c.PARK_POS[2]+.003])
        phases=[
          ('left_hover','tcp','left',fgrip+np.array([0,0,.065]),1.,1.0),
          ('left_empty_close','empty_grip','left',None,0.,.6),
          ('left_press','tcp','left',press_tcp,0.,2.0),
          ('cap_hover','tcp','right',capgrip+np.array([0,0,.055]),1.,1.2),
          ('cap_descend','tcp','right',capgrip,1.,1.2),
          ('cap_close','grip','right',None,0.,.6),
          ('cap_lift','tool','cap',np.array([cap[0],cap[1],.8185]),0.,.8),
          ('cap_to_park','tool','cap',np.array([park[0],park[1],.8185]),0.,1.0),
          ('cap_park','tool_slow','cap',park,0.,1.0),
          ('cap_release_prepare','release_prepare','right',None,0.,.35),
          ('cap_release','grip','right',None,1.,.5),
          ('cap_park_clear','tcp','right',np.array([park[0],park[1],.88]),1.,.8),
          ('washer_hover','tcp','right',wgrip+np.array([0,0,.055]),1.,1.0),
          ('washer_descend','tcp','right',wgrip,1.,1.3),
          ('washer_close','grip','right',None,0.,.6),
          ('washer_lift','tool','washer',np.array([washer[0],washer[1],.8195]),0.,.8),
          ('washer_align','tool','washer',fp+np.array([0,0,.064]),0.,1.0),
          ('washer_seat','tool_slow','washer',fp+fr@np.asarray(c.WASHER_SEATED_LOCAL),0.,1.0),
          ('washer_release_prepare','release_prepare','right',None,0.,.35),
          ('washer_release','grip','right',None,1.,.5),
          ('washer_clear','tcp','right',np.array([wgrip[0],wgrip[1],.88]),1.,.8),
          ('cap_rehover','tcp','right',capgrip+np.array([0,0,.055]),1.,1.0),
          ('cap_redescend','tcp','right',capgrip,1.,1.2),
          ('cap_reclose','grip','right',None,0.,.6),
          ('cap_relift','tool','cap',np.array([cap[0],cap[1],.8185]),0.,.8),
          ('cap_align','tool','cap',fp+np.array([0,0,.063]),0.,1.0),
          ('cap_replace','tool_slow','cap',fp+fr@np.array([0,0,.022]),0.,1.0),
          ('cap_final_release_prepare','release_prepare','right',None,0.,.35),
          ('cap_final_release','grip','right',None,1.,.5),
          ('cap_vertical_clear','tcp','right',np.r_[ik.tcp_pose(right[:6],self._bases['right'])[0][:2],.89],1.,.8),
          ('right_clear','tcp','right',np.array([.23,-.20,.86]),1.,1.0),
          ('left_vertical_clear','tcp','left',np.r_[ik.tcp_pose(left[:6],self._bases['left'])[0][:2],.875],0.,1.0),
          ('left_release','grip','left',None,1.,.5),
          ('left_clear','tcp','left',np.array([.23,.24,.86]),1.,1.0),
          ('settle','wait','right',None,1.,2.0)]
        if self._phase>=len(phases):
            return self._chunk(dict(phase='complete',request_stop=True,stop_reason='demo_motion_complete'))
        name,kind,arm,goal,opening,duration=phases[self._phase]
        left_tcp,_=ik.tcp_pose(left[:6],self._bases['left'])
        meta=dict(phase=name,phase_s=elapsed,left_stabilizer_tcp=left_tcp.tolist(),
                  fixture_displacement_mm=float(np.linalg.norm(fp-np.asarray(c.FIXTURE_POS)))*1000.)
        if now-self._start>115 or elapsed>15:
            return self._chunk({**meta,'request_stop':True,'stop_reason':'demo_timeout_'+name})
        # Freeze each phase goal once: feedback must not chase its own moving datum.
        if self._phase not in self._captured:self._captured[self._phase]=None if goal is None else np.asarray(goal).copy()
        goal=self._captured[self._phase]
        commands=list(self._last)
        if kind=='tcp':
            idx=0 if arm=='left' else 1;q=left if idx==0 else right
            commands[idx],err,ang=self._tcp(arm,q,goal,opening,elapsed,duration)
            actual,_=ik.tcp_pose(q[:6],self._bases[arm])
            meta.update(ik_error_mm=err*1000,tcp_error_mm=float(np.linalg.norm(actual-goal))*1000)
            reached=np.linalg.norm(actual-goal)<.007
            hold=.10
            if name=='left_press':
                # Press onto the existing handle top rather than pinching its sides.
                # A close-enough free-space TCP alone does not establish contact.
                gap=actual[2]-PAD_TIP_PAST_TCP-handle_top[2]
                xy_error=float(np.linalg.norm(actual[:2]-handle_top[:2]))
                reached=gap<=.001 and xy_error<.004 and np.linalg.norm(actual-goal)<.004
                hold=.25
                meta.update(left_press_tip_gap_mm=float(gap*1000),left_press_xy_error_mm=xy_error*1000)
            if elapsed>=duration and self._stable(reached,hold,self._dt):self._next(now)
        elif kind.startswith('tool'):
            q=right;pose=poses[arm];rot=quat_xyzw_to_matrix(pose[3:])
            commands[1]=self._track('right',q[:6],pose,rot,goal,np.eye(3),self._dt,meta,press=kind=='tool_slow')
            error=np.linalg.norm(pose[:3]-goal)
            speed=0.;angular_speed=0.
            if arm in self._previous_tool_poses and self._previous_observation_time is not None:
                measurement_dt=now-self._previous_observation_time
                if measurement_dt>0:
                    previous=self._previous_tool_poses[arm]
                    speed=float(np.linalg.norm(pose[:3]-previous[:3])/measurement_dt)
                    angular_speed=_rotation_angle(rot@quat_xyzw_to_matrix(previous[3:]).T)/measurement_dt
            precise=kind=='tool_slow'
            pre_insertion=name in ('washer_align','cap_align')
            # A 22 mm opening around an 18 mm post has 2 mm side clearance.
            # Complete coarse transport above the post before precise descent.
            reached=error<(.001 if precise or pre_insertion else .006) and _rotation_angle(rot)<math.radians(2 if pre_insertion else 4)
            if precise or pre_insertion:reached=reached and speed<.008 and angular_speed<.10
            meta.update(seating_error_mm=error*1000.,measured_tool_speed_m_s=speed,
                        measured_tool_angular_speed_rad_s=angular_speed)
            if self._stable(reached,.30 if precise or pre_insertion else .20,self._dt):
                self._next(now)
        elif kind=='release_prepare':
            # Preserve the last commanded arm pose, including load-supporting lead.
            # Resetting it to measured q caused a real 6 mm shift in a prior trial.
            # Preserve the jaw target too: jumping it to measured width instantly
            # removes closing preload and displaced a seated part in a prior trial.
            if self._phase not in self._release_prepare_states:
                prepared=self._last[1].copy()
                self._release_prepare_states[self._phase]=prepared
            commands[1]=self._release_prepare_states[self._phase].copy()
            meta.update(release_prepare_commanded_joints=commands[1].tolist(),
                        release_prepare_measured_joints=right.tolist(),
                        release_prepare_preserves_arm_targets=True,
                        release_prepare_preserves_jaw_target=True,
                        measured_gripper_open=float(right[6]),
                        commanded_gripper_open=float(commands[1][6]),
                        jaw_target_error_per_finger_m=float((right[6]-commands[1][6])*abs(FINGER_OPEN)))
            if elapsed>=duration:self._next(now)
        elif kind=='empty_grip':
            commands[0]=commands[0].copy();commands[0][6]=0.
            measured=float(left[6])
            speed=abs(measured-self._previous_grippers.get('left',measured))/self._dt
            closed=measured<=.02 and speed<.10
            meta.update(empty_gripper_close=True,measured_gripper_open=measured,
                        gripper_open_speed=speed,empty_close_gate=closed)
            if elapsed>=duration and self._stable(closed,.15,self._dt):self._next(now)
        elif kind=='grip':
            idx=0 if arm=='left' else 1
            commands[idx]=commands[idx].copy();commands[idx][6]=opening
            if opening==1. and arm=='right':
                if self._phase not in self._release_states:
                    self._release_states[self._phase]=dict(initial=float(self._last[1][6]),actual_initial=float(right[6]),started=now)
                release=self._release_states[self._phase]
                commanded_open=min(1.,release['initial']+.20*(now-release['started']))
                commands[idx][6]=commanded_open
                meta.update(release_initial_commanded_gripper_open=release['initial'],
                            release_initial_actual_gripper_open=release['actual_initial'],
                            commanded_gripper_open=commanded_open,
                            measured_gripper_open=float(right[6]),
                            jaw_target_error_per_finger_m=float((right[6]-commanded_open)*abs(FINGER_OPEN)),
                            release_opening_rate_per_s=.20)
            ready=elapsed>=duration
            if opening==0.:
                # The driver closes at about one normalized opening per second.
                # A timer alone can finish while the jaws remain wider than the tab.
                # Wait for measured load-bearing width and settled opening before
                # initializing the tool-to-TCP grasp datum used by drift monitoring.
                measured=float((left if arm=='left' else right)[6])
                width=c.FIXTURE_GRIP_SIZE[0] if arm=='left' else c.CAP_GRIP_SIZE[1]
                expected=width/(2*abs(FINGER_OPEN))
                speed=abs(measured-self._previous_grippers.get(arm,measured))/self._dt
                contact_band=.65*expected<=measured<=expected+.04
                ready=self._stable(ready and contact_band and speed<.10,.20,self._dt)
                meta.update(measured_gripper_open=measured,expected_grasp_open=expected,
                            gripper_open_speed=speed,grasp_contact_band=contact_band)
            else:
                measured=float((left if arm=='left' else right)[6])
                speed=abs(measured-self._previous_grippers.get(arm,measured))/self._dt
                ready=self._stable(ready and measured>=.98 and speed<.10,.15,self._dt)
                meta.update(measured_gripper_open=measured,gripper_open_speed=speed,
                            full_open_gate=measured>=.98 and speed<.10)
            if ready:
                self._servo.pop('right',None)
                self._next(now)
        elif elapsed>=duration:self._next(now)
        self._previous_tool_poses={name:pose.copy() for name,pose in poses.items()}
        self._previous_observation_time=now
        self._previous_grippers=dict(left=float(left[6]),right=float(right[6]))
        self._last=tuple(commands)
        if 'feedback_fault' in meta:meta.update(request_stop=True,stop_reason=meta['feedback_fault'])
        return self._chunk(meta)

def make_demo_policy(**kwargs):return MissingWasherReworkDemo(**kwargs)
