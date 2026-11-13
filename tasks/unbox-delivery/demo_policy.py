"""Feedback demonstration using measured poses and ordinary robot commands.

The script has privileged object/hand telemetry. It never sets object poses,
applies forces or edits scoring/tape state. Dream receives the standard inputs.
"""
from dataclasses import replace
import math
import numpy as np
from yam_arena import ik
from yam_arena.demo import ArmPlan, grip_rotation, Waypoint
from yam_arena.urdf import axis_angle_matrix, quat_xyzw_to_matrix
from yam_arena.constants import FINGER_OPEN, READY
from yam_arena.scripted import YamScriptedPolicy
from inspect_robots.types import Action, ActionChunk
from inspect_robots_yam.packing import pack
from config import *
import telemetry


def rx(angle):
    return axis_angle_matrix(np.array([1.,0.,0.]),math.radians(angle))


def flap_target(side,angle,offset=None,relative_rotation=None):
    """Plain inner edge, approached obliquely by the opposite arm."""
    sign=1 if side=='left' else -1
    rotation=rx(-sign*angle)
    hinge=np.array([CENTER[0],sign*WIDTH/2,LID_Z])
    root_offset=np.array([0.,-sign*FLAP_WIDTH/2,0.])
    if offset is None:offset=np.array([-.05,-sign*(FLAP_CENTER_Y-(.012 if side=='left' else .002)),.006])
    if side=='left':hand_rotation=rx(max(35.,50.-angle))@grip_rotation()
    else:hand_rotation=rx(-max(35.,50.-angle))@grip_rotation()
    if relative_rotation is not None:hand_rotation=hand_rotation@relative_rotation
    return hinge+rotation@(root_offset+offset),hand_rotation


class ParcelFeedbackPolicy(YamScriptedPolicy):
    def __init__(self,opening_only=False):
        super().__init__()
        self.info=replace(self.info,name='parcel_feedback_demo')
        self.opening_only=opening_only

    def event(self,kind,**kwargs):
        telemetry.events.append(dict(time=self.now,kind=kind,phase=self.phase,**kwargs))

    def reset(self,scene):
        # Inspect resets the policy before the embodiment. Initialize from the
        # first real observation after the simulator reset has completed.
        self.initialized=False

    def initialize(self):
        if telemetry.latest is None: raise RuntimeError('demo requires measured initial telemetry')
        self.initialized=True
        self.now=0.;self.phase='cut';self.failure=None;self.motion=None
        self.target={};self.q={};self.q_override={}
        for i,side in enumerate(('left','right')):
            h=telemetry.latest['hands'][i]
            self.target[side]=(np.array(h['tcp']),np.array(h['tcp_rotation']),h['opening'])
            self.q[side]=np.array(h['joints'])
            self.q_override[side]=self.q[side].copy()
        self.cut_plan=ArmPlan(self._task.module.demo()['left'],self._bases['left'])
        self.park_plan=ArmPlan([Waypoint(3.,(.13,-.24,TOP+.17),FINGER_OPEN)],self._bases['right'])
        self.retries={'left':0,'right':0};self.pick_retries=0;self.carrying=False
        if self.opening_only:self.begin_opening()
        self.event('start',diagnostic_precut=self.opening_only)

    def hand(self,side):
        return telemetry.latest['hands'][0 if side=='left' else 1]

    def angle(self,side):
        return telemetry.latest['flap_angles'][0 if side=='left' else 1]

    def move(self,side,pos,rotation,opening,seconds,then):
        h=self.hand(side)
        self.q_override.pop(side,None)
        self.q[side]=np.array(h['joints'])
        self.motion=dict(side=side,start=self.now,p0=np.array(h['tcp']),r0=np.array(h['tcp_rotation']),
                         g0=self.target[side][2],p1=np.array(pos),r1=np.array(rotation),g1=opening,
                         seconds=seconds,then=then)
        self.phase='move:'+then

    def park(self,side,then):
        self.motion=dict(mode='joints',side=side,start=self.now,q0=np.array(self.hand(side)['joints']),
                         q1=np.array(READY),seconds=3.,then=then)
        self.phase='park:'+side

    def hold(self,seconds,then):
        self.phase='hold:'+then;self.until=self.now+seconds

    def fail(self,reason):
        self.failure=reason;self.event('failure',reason=reason);self.phase='failed'
        self.motion=None;self.carrying=False

    def edge_opening(self):
        return .018/abs(FINGER_OPEN)/2

    def edge_is_held(self,opening):
        return .028<opening<(.14 if self.side=='left' else .35)

    def begin_opening(self):
        self.side='left';self.arm='right';self.support_active=True
        self.move('left',(.32,-.035,LID_Z+.07),grip_rotation(),0.,2.,'support_approached')

    def begin_flap(self,side):
        self.side=side
        self.arm='right' if side=='left' else 'left'  # Cross-grasp each ordinary inner edge.
        p,r=flap_target(side,self.angle(side))
        self.event('approach_flap',side=side,angle=self.angle(side),attempt=self.retries[side]+1)
        self.move(self.arm,(.13,0.,TOP+.20),r,self.edge_opening(),2.,'flap_front_entry')

    def retry_flap(self,reason):
        side=self.side
        self.event('flap_retry',side=side,reason=reason,angle=self.angle(side))
        self.retries[side]+=1
        if self.retries[side]>2: return self.fail(side+' flap: '+reason)
        h=self.hand(self.arm);p=np.array(h['tcp']);r=np.array(h['tcp_rotation'])
        self.move(self.arm,p-r[:,2]*.06,r,self.edge_opening(),1.5,'flap_retry_clear')

    def start_rotation(self):
        side=self.side;h=self.hand(self.arm);flap=telemetry.latest['flaps'][side]
        if side=='left' and not self.edge_is_held(h['opening']):return self.retry_flap('edge lost while clearing support arm')
        r=quat_xyzw_to_matrix(np.array(flap['quat']))
        self.grasp_offset=r.T@(np.array(h['tcp'])-np.array(flap['pos']))
        _,nominal=flap_target(side,self.angle(side))
        self.grasp_rotation=nominal.T@np.array(h['tcp_rotation'])
        self.command_angle=self.angle(side);self.last_angle=self.angle(side)
        self.engagement_start=self.angle(side);self.engagement_verified=False
        self.last_progress=self.now;self.rotate_start=self.now;self.phase='rotate_flap'
        self.event('plain_edge_lift_probe',side=side,arm=self.arm,offset=self.grasp_offset.tolist(),opening=h['opening'])

    def begin_pick(self):
        self.carrying=False
        p=np.array(telemetry.latest['item']['pos'])+[0.,0.,.008]
        self.pick_point=p
        self.event('approach_item',item_position=telemetry.latest['item']['pos'])
        self.move('right',(*p[:2],TOP+.17),grip_rotation(),1.,2.,'item_approached')

    def transition(self,name):
        side=getattr(self,'side','left')
        arm=getattr(self,'arm','right')
        edge_gap=.002/abs(FINGER_OPEN)/2
        if name=='support_approached':
            self.move('left',(.32,-.035,LID_Z+.002),grip_rotation(),0.,2.,'support_pressed')
        elif name=='support_pressed':self.hold(.4,'support_check')
        elif name=='support_check':
            self.event('opposite_flap_depressed',angle=self.angle('right'))
            if self.angle('right')>-7.:return self.fail('opposite flap did not expose the seam edge')
            self.begin_flap('left')
        elif name=='support_withdrawn':
            self.park('left','support_parked')
        elif name=='support_parked':self.begin_flap('right')
        elif name=='flap_front_entry':
            p,r=flap_target(side,self.angle(side))
            self.move(arm,p-r[:,2]*.075,r,self.edge_opening(),1.5,'flap_approached')
        elif name=='flap_approached':
            p,r=flap_target(side,self.angle(side))
            self.move(arm,p,r,self.edge_opening(),2.,'flap_descended')
        elif name=='flap_descended':
            p,r=flap_target(side,self.angle(side)) if side=='right' else self.target[arm][:2]
            self.move(arm,p,r,edge_gap,.9,'flap_closed')
        elif name=='flap_closed': self.hold(.4,'flap_grasp_check')
        elif name=='flap_grasp_check':
            h=self.hand(arm)
            if side=='left' and not self.edge_is_held(h['opening']):return self.retry_flap('plain edge not pinched')
            if getattr(self,'support_active',False):
                self.move('left',(.32,-.035,LID_Z+.12),grip_rotation(),1.,1.5,'support_clear_lifted')
            else:self.start_rotation()
        elif name=='support_clear_lifted':self.park('left','support_clear_parked')
        elif name=='support_clear_parked':
            self.support_active=False;self.start_rotation()
        elif name=='flap_retry_clear':
            if side=='left' and self.angle(side)<10. and self.angle('right')>-7.:self.begin_opening()
            else:self.begin_flap(side)
        elif name=='flap_rotated':
            p,r=self.target[arm][:2]
            self.move(arm,p,r,self.edge_opening(),.9,'flap_released')
        elif name=='flap_released':
            h=self.hand(arm);p=np.array(h['tcp']);r=np.array(h['tcp_rotation'])
            self.move(arm,p-r[:,2]*.075,r,self.edge_opening(),1.8,'flap_withdrawn')
        elif name=='flap_withdrawn':
            self.park(arm,'flap_parked')
        elif name=='flap_parked':
            if self.angle(side)<80: return self.retry_flap('flap did not remain open after release')
            self.event('flap_open_verified',side=side,angle=self.angle(side))
            if side=='left':self.begin_flap('right')
            elif min(telemetry.latest['flap_angles'])<80: self.fail('first flap fell shut')
            elif self.opening_only:
                self.event('diagnostic_open_completed');self.phase='done'
            else: self.begin_pick()
        elif name=='item_approached': self.move('right',self.pick_point,grip_rotation(),1.,2.,'item_descended')
        elif name=='item_descended':
            self.move('right',self.target['right'][0],grip_rotation(),.038/abs(FINGER_OPEN)/2,1.,'item_closed')
        elif name=='item_closed':
            h=self.hand('right');p=np.array(h['tcp']);p[2]=TOP+.17
            self.move('right',p,grip_rotation(),.038/abs(FINGER_OPEN)/2,2.,'item_lifted')
        elif name=='item_lifted':
            h=self.hand('right');item=np.array(telemetry.latest['item']['pos']);tcp=np.array(h['tcp'])
            if item[2]<LID_Z+.035 or np.linalg.norm(item-tcp)>.045:
                self.pick_retries+=1
                if self.pick_retries>1: return self.fail('item pickup not established')
                return self.begin_pick()
            self.item_offset=grip_rotation().T@(item-tcp)
            self.carrying=True
            self.event('item_pickup_verified',height=float(item[2]),grasp_offset=self.item_offset.tolist())
            self.move('right',(.10,0.,TOP+.17),grip_rotation(),.038/abs(FINGER_OPEN)/2,2.,'item_front')
        elif name=='item_front':
            self.move('right',(.10,DESTINATION[1],TOP+.17),grip_rotation(),.038/abs(FINGER_OPEN)/2,2.,'item_outside')
        elif name=='item_outside':
            self.move('right',(*DESTINATION,TOP+.17),grip_rotation(),.038/abs(FINGER_OPEN)/2,1.5,'item_above_pad')
        elif name=='item_above_pad':
            goal=np.array([*DESTINATION,TOP+.023])-grip_rotation()@self.item_offset
            self.move('right',goal,grip_rotation(),.038/abs(FINGER_OPEN)/2,2.,'item_lowered')
        elif name=='item_lowered':
            self.carrying=False
            self.move('right',self.target['right'][0],grip_rotation(),1.,1.,'item_released')
        elif name=='item_released':
            self.move('right',(*DESTINATION,TOP+.17),grip_rotation(),1.,2.,'finish')
        elif name=='finish': self.hold(3.,'finish_check')
        elif name=='finish_check':
            self.event('finished',metrics=telemetry.latest['metrics'])
            if telemetry.latest['metrics']['score']<1.: self.fail('final placement did not reach full score')
            else: self.phase='done'
        else: raise RuntimeError('unknown transition '+name)

    def advance(self):
        if self.motion is not None:
            m=self.motion;u=(self.now-m['start'])/m['seconds'];blend=ik.smoothstep(u)
            if m.get('mode')=='joints':
                q=m['q0']+blend*(m['q1']-m['q0'])
                self.q_override[m['side']]=q
                self.q[m['side']]=q.copy()
                p,r=ik.tcp_pose(q,self._bases[m['side']])
                self.target[m['side']]=(p,r,1.)
            else:
                self.target[m['side']]=(m['p0']+blend*(m['p1']-m['p0']),
                                       ik.slerp_matrix(m['r0'],m['r1'],blend),m['g0']+blend*(m['g1']-m['g0']))
            if u>=1.:
                self.motion=None;self.transition(m['then'])
        elif self.phase.startswith('hold:'):
            if self.now>=self.until: self.transition(self.phase[5:])
        elif self.phase=='rotate_flap':
            a=self.angle(self.side);h=self.hand(self.arm)
            if not self.engagement_verified and a>=self.engagement_start+5.:
                self.engagement_verified=True;self.event('edge_engagement_verified',side=self.side,angle=a)
            if a>=(102. if self.side=='left' else 95.):
                self.event('flap_open_reached',side=self.side,angle=a)
                self.hold(.4,'flap_rotated');return
            if a>self.last_angle+1.:
                self.last_angle=a;self.last_progress=self.now
            if self.now-self.last_progress>2.5 or self.now-self.rotate_start>16.:
                return self.retry_flap('opening stalled')
            if self.side=='left' and h['opening']<.027:return self.retry_flap('plain edge slipped out')
            self.command_angle=min(107.,self.command_angle+18.*self._dt,a+10.)
            p,r=flap_target(self.side,self.command_angle,self.grasp_offset,self.grasp_rotation)
            self.target[self.arm]=(p,r,.002/abs(FINGER_OPEN)/2)


    def act(self,observation):
        if not self.initialized: self.initialize()
        self.now=float(observation.state_time)
        sample=telemetry.latest
        if sample is None or abs(sample['time']-self.now)>.075: raise RuntimeError('stale privileged demo telemetry')
        if self.phase=='cut':
            t=self.now+self._dt
            if t<=self.cut_plan.duration:
                a=self.cut_plan.action7(t)[0];b=self.park_plan.action7(t)[0]
                errors={}
                for side,action,plan in (('left',a,self.cut_plan),('right',b,self.park_plan)):
                    p,r=ik.tcp_pose(action[:6],self._bases[side]);goal,rot,_,_=plan.target(t)
                    errors[side]=dict(position_m=float(np.linalg.norm(p-goal)),rotation_rad=float(np.linalg.norm(ik._rot_error(rot,r))))
                telemetry.command=dict(phase='cut',ik_errors=errors,failure=None)
                return ActionChunk(actions=[Action(data=pack(a,b))],control_hz=1/self._dt)
            if sample['metrics']['cut_segments']!=CUT_SEGMENTS or sample['held']:
                self.fail('cutting and cutter return were not established')
            else:
                for side in ('left','right'):
                    h=self.hand(side)
                    self.target[side]=(np.array(h['tcp']),np.array(h['tcp_rotation']),h['opening'])
                    self.q[side]=np.array(h['joints'])
                    self.q_override[side]=self.q[side].copy()
                self.begin_opening()
        if self.carrying:
            h=self.hand('right')
            predicted=np.array(h['tcp'])+np.array(h['tcp_rotation'])@self.item_offset
            slip=float(np.linalg.norm(np.array(sample['item']['pos'])-predicted))
            if slip>.018:
                self.event('item_slip_detected',slip_m=slip)
                self.carrying=False;self.pick_retries+=1
                if self.pick_retries>1: self.fail('item slipped during transport')
                else: self.begin_pick()
        self.advance()
        actions=[];errors={}
        for side in ('left','right'):
            pos,rot,opening=self.target[side]
            if side in self.q_override:
                q=self.q_override[side];pe=re=0.
            else:
                q,pe,re=ik.solve(pos,ik.matrix_to_quat_xyzw(rot),self.q[side],base_pos=self._bases[side],iters=70)
            self.q[side]=q
            errors[side]=dict(position_m=pe,rotation_rad=re)
            actions.append(np.r_[q,np.clip(opening,0,1)])
        telemetry.command=dict(phase=self.phase,ik_errors=errors,failure=self.failure)
        meta={'request_stop':True} if self.phase in ('failed','done') else {}
        return ActionChunk(actions=[Action(data=pack(*actions),meta=meta)],control_hz=1/self._dt)


def register_policy(opening_only=False):
    from inspect_robots.registry import register
    register('policy','parcel_feedback_demo')(lambda:ParcelFeedbackPolicy(opening_only))
    return 'parcel_feedback_demo'
