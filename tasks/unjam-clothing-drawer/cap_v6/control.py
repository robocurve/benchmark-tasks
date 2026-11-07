"""Measured-state coordination, incremental pull, guarded Cartesian movements."""
import json
import time
from collections import deque
import numpy as np
from yam_arena import ik, station
from yam_arena.demo import grip_rotation
from geometry import Guard

from drawer_layout_v6 import *

def press_x(opening):return min(.300,PRESS_OFFSET-opening)

class Stop(RuntimeError):pass
class Done(Exception):pass

class Controller:
    def __init__(self,backend,rec,limit=1800):
        self.backend=backend;self.rec=rec;self.limit=limit;self.steps=0;self.phase='initial'
        self.guard=Guard(fixtures=fixtures());self.stage='start';self.command=self.q()
        self.held_a=None;self.grasp_offset=None;self.jam_opening=None;self.press_offset=None
        self.guard_passes=0;self.contact_verified=False;self.grip_targets=[1.,1.];self.started=time.perf_counter()
    def q(self):return np.asarray(self.backend.joints(),float).copy()
    def fingers(self):return self.backend.finger_positions()
    def tcp(self,arm,q=None):
        q=self.q() if q is None else q;i=0 if arm=='left' else 7
        return ik.tcp_pose(q[i:i+6],station.BASES[arm])[0]
    def state(self):
        s=self.backend.state()
        if not np.isfinite([s['opening'],s['compression']]).all():raise Stop('Invalid object observation')
        return s
    def checked(self,q):
        hit=self.guard.check(q,fingers=self.fingers())
        if hit:
            self.rec.write('guard_stop.jsonl',dict(step=self.steps,phase=self.phase,**hit))
            raise Stop('Geometry guard: '+json.dumps(hit))
        self.guard_passes+=1
    def freeze(self,label):
        self.phase=label;self.command=self.q();self.tick(self.command)
    def tick(self,target):
        if self.steps>=self.limit:raise Stop('Step budget reached')
        current=self.q();self.guard.fixtures=fixtures()+drawer_panels(self.state()['opening']);self.checked(current)
        self.backend.check_fk()  # Fail if the URDF and measured link poses disagree.
        target=np.asarray(target,float)
        if target.shape!=(14,) or not np.isfinite(target).all():raise Stop('Invalid command')
        # Keep A's jam posture and grip while B approaches and presses.
        # Resume A only after compression is measured; hold it again during B withdrawal.
        if self.stage in ('jammed','pressing','pressed','cleared'):
            if self.held_a is None:raise Stop('Missing held handle posture')
            target=target.copy();target[:6]=self.held_a
            if self.grip_targets[0]!=0.:raise Stop('A must keep the handle grasp')
            if abs(self.tcp('left',current)[0]+self.state()['opening']-self.grasp_offset)>.012:
                raise Stop('Handle grasp slipped while A was holding')
        # Maintain B pressure until the second pull begins.
        if self.stage=='pressed':
            s=self.state();pb=self.tcp('right',current)
            dz=float(np.clip(s['compression']-.016,-.0004,.0004))
            self.press_z=float(np.clip(self.press_z+dz,max(.822,pb[2]-.004),.86))
            target=self.solve('right',(self.press_hold_x,PRESS_Y,self.press_z),target)
        # Rate-limit changes in the previous COMMAND, not measured-position error.
        # This restores the drive's ability to overcome load while bounding command speed.
        proposed=self.command+np.clip(target-self.command,-.03,.03)
        proposed[[6,13]]=self.grip_targets
        lag=float(np.max(abs(proposed[np.r_[0:6,7:13]]-current[np.r_[0:6,7:13]])))
        if lag>.20:raise Stop(f'Joint tracking lag {lag:.3f} rad exceeds diagnostic limit; no further command')
        hit=self.guard.segment(current,proposed,fingers=self.fingers())
        if hit:
            self.rec.write('guard_stop.jsonl',dict(step=self.steps,phase=self.phase,**hit))
            raise Stop('Predicted geometry conflict: '+json.dumps(hit))
        self.command=proposed
        result=self.backend.step(proposed);self.steps+=1
        self.rec.capture(self.backend.obs,self.steps,self.phase)
        state=self.state()
        self.guard.fixtures=fixtures()+drawer_panels(state['opening'])
        self.checked(self.q())
        force=self.backend.contact_force()
        self.rec.write('steps.jsonl',dict(step=self.steps,phase=self.phase,joints=self.q().tolist(),finger_positions_m=self.fingers().tolist(),action=proposed.tolist(),**state,contact_force_norm=force))
        if self.steps%20==0:print(f'PROGRESS: {self.steps} steps, {self.steps/20:.1f} simulation seconds, {time.perf_counter()-self.started:.1f} wall seconds; opening={state["opening"]*1000:.1f} mm, compression={state["compression"]*1000:.1f} mm',flush=True)
        if result.get('success'):
            if self.stage not in ('finish','settle'):raise Stop('Premature evaluator success before policy sequence completed')
            raise Done()
    def hold(self,seconds):
        target=self.q()
        for _ in range(round(seconds*20)):self.tick(target)
    def grip(self,arm,value,seconds):
        i=6 if arm=='left' else 13;self.grip_targets[0 if arm=='left' else 1]=value;target=self.q();target[i]=value
        self.phase=arm+' gripper'
        for _ in range(round(seconds*20)):
            self.tick(target)
        if arm=='left' and value==0:
            s=self.state();self.grasp_offset=self.tcp('left')[0]+s['opening']
            if abs(self.grasp_offset-.28)>.012:raise Stop('A is not aligned with the handle')
            if self.backend.contact_force()<.05:raise Stop('No measured contact force after grasp; contact sensing is unverified')
            self.contact_verified=True;self.stage='pressed' if self.stage=='pressed' else 'grasped'
    def solve(self,arm,pos,q):
        i=0 if arm=='left' else 7
        p=np.asarray(pos,float)
        if not (.075<=p[0]<=.5 and -.35<=p[1]<=.35 and .79<=p[2]<=.93):raise Stop('Target outside task workspace')
        v,pe,re=ik.solve(p,ik.matrix_to_quat_xyzw(grip_rotation(np.pi/2 if arm=='right' else 0)),q[i:i+6],station.BASES[arm],iters=100)
        if pe>.002 or re>np.deg2rad(3):raise Stop(f'IK failed: {arm}, position {pe:.4f} m')
        out=q.copy();out[i:i+6]=v;return out
    def move(self,arm,target,seconds,label):
        self.phase=label;startq=self.q();i=0 if arm=='left' else 7
        start,r0=ik.tcp_pose(startq[i:i+6],station.BASES[arm]);plan=[];last=startq.copy()
        # Check the entire planned primitive before beginning it, with the other arm fixed.
        for k in range(1,round(seconds*20)+1):
            f=ik.smoothstep(k/(seconds*20));p=start+f*(np.asarray(target)-start)
            rot=ik.slerp_matrix(r0,grip_rotation(np.pi/2 if arm=='right' else 0),f)
            q,pe,re=ik.solve(p,ik.matrix_to_quat_xyzw(rot),last[i:i+6],station.BASES[arm],iters=100)
            if pe>.002 or re>np.deg2rad(3):raise Stop('No reachable path for '+label)
            command=startq.copy();command[i:i+6]=q
            hit=self.guard.segment(last,command,fingers=self.fingers())
            if hit:raise Stop('Primitive rejected before motion: '+label+' '+str(hit))
            plan.append(command);last=command
        for command in plan:self.tick(command)
        for _ in range(100):
            if np.max(abs(self.q()[i:i+6]-last[i:i+6]))<.012:break
            self.tick(last)
        error=float(np.linalg.norm(self.tcp(arm)-target))
        details=dict(step=self.steps,phase=label,target=list(target),measured_tcp=self.tcp(arm).tolist(),error_m=error,commanded_joints=self.command[i:i+6].tolist(),measured_joints=self.q()[i:i+6].tolist(),planned_joints=last[i:i+6].tolist())
        self.rec.write('primitives.jsonl',details)
        print('TRACKING:',json.dumps(details),flush=True)
        if error>.008:raise Stop(f'Motion did not reach target: {label}; TCP error {error*1000:.1f} mm')
    def pull_until_jam(self):
        if self.stage!='grasped':raise Stop('A must grasp before pulling')
        self.phase='A incremental first pull';window=deque(maxlen=12)
        held=self.q();initial=self.state()['opening'];peak=initial;desired_x=self.tcp('left')[0]
        for _ in range(260):
            s=self.state();opening=s['opening'];peak=max(peak,opening)
            if opening<peak-.004:raise Stop('Drawer moved inward during first pull')
            if opening>JAM_HIGH+.003:raise Stop('Expected obstruction did not block the first pull')
            window.append(opening)
            # Force is not a task score. It confirms load, while travel confirms stall.
            if JAM_LOW<opening<JAM_HIGH and len(window)==12 and max(window)-min(window)<.0006:
                self.jam_opening=opening;self.command=self.q();self.held_a=self.command[:6].copy();self.stage='jammed'
                self.freeze('A measured hold at jam');self.hold(.4)
                return
            actual=self.tcp('left')
            expected=self.grasp_offset-opening
            if abs(actual[0]-expected)>.012:raise Stop('Handle grasp slipped during pull')
            # Accumulate a bounded 5 mm load lead, without an inward target.
            desired_x=min(actual[0],max(expected-.005,desired_x-.0010))
            x=desired_x
            target=held.copy();target[:7]=self.q()[:7]
            target=self.solve('left',(x,HANDLE[1],HANDLE[2]),target)
            self.tick(target)
        raise Stop('No confirmed jam within first-pull budget')
    def press_hover(self):
        if self.stage!='jammed':raise Stop('B may approach only after A confirms the jam')
        return (press_x(self.state()['opening']),PRESS_Y,.9075)
    def press_until_clear(self):
        if self.stage!='jammed' or not self.contact_verified:raise Stop('Press preconditions not met')
        self.phase='B presses cloth; A holds handle';held=self.q();self.stage='pressing'
        hover=self.tcp('right');opening=self.state()['opening']
        hover[0]=press_x(opening);hover[1]=PRESS_Y
        self.press_offset=hover[0]+opening;self.press_hold_x=float(hover[0]);self.press_z=float(hover[2])
        for _ in range(230):
            s=self.state()
            if abs(s['opening']-self.jam_opening)>.006:raise Stop('Drawer did not stay at the jam while B approached/pressed')
            if s['compression']>=.016:
                self.stage='pressed';self.phase='Compression reached; A may resume';self.tick(self.command.copy());return
            p=self.tcp('right')
            if p[2]<.822:raise Stop('Compression not achieved above lower press limit')
            target=held.copy();target[7:]=self.q()[7:]
            speed=.0015 if p[2]>.875 else .0005
            self.press_z=max(self.press_z-speed,p[2]-.004)
            target=self.solve('right',(hover[0],PRESS_Y,self.press_z),target)
            self.tick(target)
        raise Stop('B did not compress the fold within budget')
    def pull_while_pressing(self):
        if self.stage!='pressed':raise Stop('Cannot resume pull before fabric compression')
        self.press_offset=self.tcp('right')[0]+self.state()['opening']
        self.phase='A resumes pull; B maintains fabric pressure';self.stage='release';peak=self.state()['opening'];desired_x=self.tcp('left')[0]
        for _ in range(250):
            s=self.state();opening=s['opening'];peak=max(peak,opening)
            if opening<peak-.004:raise Stop('Drawer reversed during release')
            if opening>=CLEAR_PULL:
                self.held_a=self.command[:6].copy();self.stage='cleared';return
            # Pause A if compression relaxes, while B corrects pressure.
            q=self.q();pa=self.tcp('left',q);pb=self.tcp('right',q)
            if abs(pa[0]+opening-self.grasp_offset)>.012:raise Stop('Handle grasp slipped during release')
            dz=float(np.clip(s['compression']-.016,-.0004,.0004))
            self.press_z=float(np.clip(self.press_z+dz,max(.822,pb[2]-.004),.86))
            z=self.press_z
            target=self.solve('right',(self.press_offset-opening,PRESS_Y,z),q)
            if s['compression']>=.014:
                desired_x=min(pa[0],max(self.grasp_offset-opening-.005,desired_x-.0009))
                x=desired_x
                target=self.solve('left',(x,HANDLE[1],HANDLE[2]),target)
            self.tick(target)
        raise Stop('Drawer did not clear while pressure was held')
    def raised_right(self):
        if self.stage!='cleared':raise Stop('B cannot withdraw before clearance')
        p=self.tcp('right');return (float(p[0]),float(p[1]),.9075)
    def finish_pull(self):
        if self.stage!='cleared':raise Stop('Finish requires cleared fold')
        self.stage='finish';self.phase='A finish; B held clear';held=self.q();desired_x=self.tcp('left')[0]
        for _ in range(100):
            s=self.state()
            if s['opening']>=FINISH_OPEN:self.stage='settle';self.freeze('Hold fully open');return
            p=self.tcp('left')
            if abs(p[0]+s['opening']-self.grasp_offset)>.012:raise Stop('Grasp slipped at finish')
            target=held.copy();target[:7]=self.q()[:7]
            desired_x=min(p[0],max(self.grasp_offset-s['opening']-.005,desired_x-.0009))
            x=desired_x
            self.tick(self.solve('left',(x,HANDLE[1],HANDLE[2]),target))
        raise Stop('Final opening not reached')

def fixtures():
    return [(name,np.array(pos),np.eye(3),np.array(size)/2)
            for name,(pos,size) in PANELS.items() if name!='cabinet_floor']

def drawer_panels(opening):
    origin=np.array(DRAWER_ORIGIN)-[opening,0,0]
    return [('drawer_'+name,origin+np.array(pos),np.eye(3),np.array(size)/2)
            for name,(pos,size) in DRAWER_PARTS.items() if not name.startswith('Handle')]
