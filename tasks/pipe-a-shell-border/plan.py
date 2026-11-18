"""Compile a fixed joint-action plan with pure kinematics, before simulation.

The plan has no object-state input or callback. Development trials can diagnose
slip from their recorded telemetry, but never repair it by moving an object or
altering later actions while an episode is running. This first plan is an
unvalidated pickup/positioning experiment, not a demonstrated piping policy.
"""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np

# Gripper X is its closing axis. Barrel lies along Y, so pads close along X.
DOWN=np.array([[1.,0.,0.],[0.,-1.,0.],[0.,0.,-1.]])
# Fixed tool-in-hand calibration from native026 frames400:439, after the
# firmer grasp and before piston contact. The largest translation range was
# 0.040 mm. This replaces the older pickup009 calibration, whose seating
# offset no longer matched this grasp. No live object feedback is used.
GRASP_TRANSLATION=np.array((.0000432964693,.000518241080,.009315861139))
GRASP_ROTATION=np.array((
    (-.999999889705,.000237975474,-.000404917306),
    (.000251620739,.999420459041,-.034039429154),
    (.000396582091,-.034039527285,-.999420408689)))


def calibrated_hand_pose(barrel_centre,barrel_rotation):
    hand_rotation=np.asarray(barrel_rotation)@GRASP_ROTATION.T
    hand_position=np.asarray(barrel_centre)-hand_rotation@GRASP_TRANSLATION
    return hand_position,hand_rotation


def rx(angle):
    c,s=np.cos(angle),np.sin(angle)
    return np.array([[1.,0.,0.],[0.,c,-s],[0.,s,c]])


def compile_plan(mode='pickup',press_stroke=.006,shell_count=1,release_tool=False, *,
                 first_press_stroke=None, head_hold_seconds=1.2, first_head_hold_seconds=None,
                 tail_seconds=.75, tail_stroke_fraction=0., shell_pitch=.012, tail_distance=.012,
                 start_y=None, tail_mid_stroke_fraction=1., linear_tail=False,
                 subsequent_head_extra=0.):
    if first_press_stroke is not None and not .001<=first_press_stroke<=.016:
        raise ValueError('First press stroke must be 1–16 mm')
    if not .1<=head_hold_seconds<=2. or not .2<=tail_seconds<=2. or not 0<=tail_stroke_fraction<=1:
        raise ValueError('Invalid fixed pressure technique')
    if first_head_hold_seconds is not None and not .1<=first_head_hold_seconds<=2.:
        raise ValueError('Invalid initial head hold')
    if not .010<=shell_pitch<=.020 or not .010<=tail_distance<=.020:
        raise ValueError('Shell pitch and tail distance must be 10–20 mm')
    if not tail_stroke_fraction<=tail_mid_stroke_fraction<=1.:
        raise ValueError('Pressure must ease monotonically through the tail')
    if not 0<=subsequent_head_extra<=.004 or press_stroke+subsequent_head_extra>.016:
        raise ValueError('Additional head press must fit the bounded 16 mm technique')
    if start_y is not None and not -.045<=start_y<=0.:
        raise ValueError('Initial head must start on the marked section')
    if shell_count not in range(1,9) or (mode!='shell' and shell_count!=1):
        raise ValueError('One to eight shell deposits; other probes use one press')
    from yam_arena import ik,station
    from yam_arena.constants import READY,FINGER_OPEN
    state={side:{'q':np.asarray(READY,float),'finger':FINGER_OPEN} for side in ('left','right')}
    frames=[];maximum_position_error=maximum_rotation_error=0.

    def transition(seconds,label,goals, *, linear=False):
        nonlocal maximum_position_error,maximum_rotation_error
        starts={side:(*ik.tcp_pose(s['q'],station.BASES[side]),s['finger']) for side,s in state.items()}
        steps=round(seconds*20)
        for i in range(1,steps+1):
            weight=i/steps if linear else ik.smoothstep(i/steps);action=[]
            for side,s in state.items():
                p0,r0,f0=starts[side]
                p1,r1,f1=goals.get(side,(p0,r0,f0))
                p=p0+(np.asarray(p1)-p0)*weight
                r=ik.slerp_matrix(r0,np.asarray(r1),weight)
                q,pe,re=ik.solve(p,ik.matrix_to_quat_xyzw(r),s['q'],station.BASES[side],iters=200,tol=1e-6)
                maximum_position_error=max(maximum_position_error,pe)
                maximum_rotation_error=max(maximum_rotation_error,re)
                if pe>.001 or re>np.deg2rad(.5):
                    raise RuntimeError(f'Unreachable fixed plan: {label}, {side}, {pe*1000:.3f} mm, {np.rad2deg(re):.3f} degrees')
                s['q']=q;s['finger']=f0+(f1-f0)*weight
                action.extend([*q.tolist(),float(np.clip(s['finger']/FINGER_OPEN,0,1))])
            frames.append({'phase':label,'action14':action})

    p=np.array((.18,-.16,.7775))
    transition(2.,'approach dispenser',{'right':(p+[0,0,.10],DOWN,FINGER_OPEN)})
    transition(2.,'lower around barrel',{'right':(p+[0,0,.009],DOWN,FINGER_OPEN)})
    transition(1.,'close on barrel',{'right':(p+[0,0,.009],DOWN,-.0145)})
    transition(2.,'lift dispenser',{'right':(p+[0,0,.10],DOWN,-.0145)})
    # Carry while horizontal, then rotate above the table. The intended nozzle
    # points -Y/down so the right wrist and left pressing hand both stay within
    # their independently checked kinematic workspace.
    tool_rotation=rx(np.pi/4)
    centre=np.array((.235,.102,.865))
    turning_centre=np.array((.235,.040,.865))
    tcp_offset=np.array((0.,0.,.009))
    transition(3.,'carry above cake',{'right':(turning_centre+tcp_offset,DOWN,-.0145)})
    transition(3.,'tilt nozzle downward',{'right':(turning_centre+tool_rotation@tcp_offset,tool_rotation@DOWN,-.0145)})
    transition(2.,'position nozzle above border',{'right':(centre+tool_rotation@tcp_offset,tool_rotation@DOWN,-.0145)})
    transition(2.,'hold for pickup inspection',{})
    if mode in ('press','shell'):
        axis=np.array((0.,-np.sqrt(.5),-np.sqrt(.5)))
        start_y=((-.039 if shell_count>1 else -.035) if mode=='shell' else -.040) if start_y is None else start_y
        tip=np.array((.235,start_y,.8045 if mode=='shell' else .8035));centre=tip-axis*.080
        push_rotation=rx(-np.pi/4)@DOWN@np.diag((-1.,-1.,1.))
        # The closed finger tips extend 13 mm beyond TCP. Position them behind
        # the actual 5 mm plunger face; no grasp/latch is added to the pusher.
        contact=centre-axis*(.060+.0025+.013)
        hand_p,hand_r=calibrated_hand_pose(centre,tool_rotation@np.diag((-1.,-1.,1.)))
        transition(3.,'position for first pressure contact',{
            'right':(hand_p,hand_r,-.0145),
            'left':(contact-axis*.04,push_rotation,FINGER_OPEN)})
        if mode=='shell':
            transition(1.,'firm barrel grasp for pressure',{'right':(hand_p,hand_r,-.0105)})
        transition(1.,'close pressing fingers',{'left':(contact-axis*.04,push_rotation,0.)})
        transition(1.,'approach plunger face',{'left':(contact,push_rotation,0.)})
        initial_contact,initial_hand=contact.copy(),hand_p.copy()
        for shell in range(shell_count):
            stroke=first_press_stroke if shell==0 and first_press_stroke is not None else press_stroke
            hold=first_head_hold_seconds if shell==0 and first_head_hold_seconds is not None else head_hold_seconds
            if shell:
                # Fixed technique and nominal consumed-volume allowance, never
                # live material state. 0.45 mm is about 0.22 ml per deposit.
                shift=np.array((0.,shell*shell_pitch,0.))
                contact=initial_contact+shift+axis*(shell*.00045)
                hand_p=initial_hand+shift
                transition(1.,f'position for shell {shell+1}',{
                    'right':(hand_p,hand_r,-.0105),
                    'left':(contact-axis*.008,push_rotation,0.)})
                transition(.5,f'contact plunger for shell {shell+1}',{
                    'left':(contact,push_rotation,0.)})
            head_stroke=stroke+(subsequent_head_extra if shell else 0.)
            transition(.5,'press passive plunger',{'left':(contact+axis*head_stroke,push_rotation,0.)})
            transition(hold if mode=='shell' else .4,'hold pressure pulse',{})
            if mode=='shell':
                # Both hands move a physical dispenser; material is produced
                # solely by actual piston pressure and aperture geometry.
                tail_delta=np.array((0.,tail_distance,-.001))
                transition(tail_seconds*(.35/.75),'draw tail under pressure',{
                    'right':(hand_p+tail_delta*.5,hand_r,-.0105),
                    'left':(contact+tail_delta*.5+axis*stroke*tail_mid_stroke_fraction,push_rotation,0.)},linear=linear_tail)
                transition(tail_seconds*(.4/.75),'ease pressure into pointed tail',{
                    'right':(hand_p+tail_delta,hand_r,-.0105),
                    'left':(contact+tail_delta+axis*stroke*tail_stroke_fraction,push_rotation,0.)},linear=linear_tail)
                contact=contact+tail_delta
                hand_p=hand_p+tail_delta
            transition(.5,'release pressure',{'left':(contact-axis*.008,push_rotation,0.)})
        transition(1.,'withdraw pressing hand',{'left':(contact-axis*.04,push_rotation,0.)})
        if mode=='shell':
            transition(1.,'lift nozzle clear',{'right':(hand_p+np.array((0.,0.,.025)),hand_r,-.0105)})
            parked_p,parked_r=calibrated_hand_pose(np.array((.18,-.16,.95)),tool_rotation@np.diag((-1.,-1.,1.)))
            transition(2.,'move clear of judging views',{'right':(parked_p,parked_r,-.0105)})
        if release_tool and mode=='shell':
            horizontal=np.diag((-1.,-1.,1.))
            raised_p,raised_r=calibrated_hand_pose(p+np.array((0.,0.,.12)),horizontal)
            rest_p,rest_r=calibrated_hand_pose(p+np.array((0.,0.,.004)),horizontal)
            transition(2.,'turn dispenser above rest',{'right':(raised_p,raised_r,-.0105)})
            transition(2.,'lower dispenser to rest',{'right':(rest_p,rest_r,-.0105)})
            transition(1.,'release dispenser',{'right':(rest_p,rest_r,FINGER_OPEN)})
            transition(2.,'withdraw empty hand',{'right':(raised_p,raised_r,FINGER_OPEN)})
        transition(2. if release_tool else 1.,'observe deposited material',{})
    return {'schema':'fixed-joint-plan-v1','control_hz':20,'status':'development '+mode+' only',
            'no_object_feedback':True,'fixed_pressure_technique':{'press_stroke_m':press_stroke,
                'first_press_stroke_m':first_press_stroke,'head_hold_s':head_hold_seconds,
                'first_head_hold_s':first_head_hold_seconds,'tail_seconds':tail_seconds,
                'tail_stroke_fraction':tail_stroke_fraction,'shell_pitch_m':shell_pitch,
                'tail_distance_m':tail_distance,'start_y_m':start_y,
                'tail_mid_stroke_fraction':tail_mid_stroke_fraction,'linear_tail':bool(linear_tail),
                'subsequent_head_extra_m':subsequent_head_extra},'frames':frames,'shell_count':shell_count,'release_tool':release_tool,
            'offline_grasp_calibration':'Native026 frames400:439 mean; fixed constants, no trial-state input' if mode in ('press','shell') else None,
            'maximum_ik_position_error_m':maximum_position_error,
            'maximum_ik_rotation_error_rad':maximum_rotation_error,
            'plan_source_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--out',type=Path,required=True)
    p.add_argument('--mode',choices=('pickup','press','shell'),default='pickup')
    p.add_argument('--press-stroke',type=float,default=.006)
    p.add_argument('--first-press-stroke',type=float)
    p.add_argument('--head-hold-seconds',type=float,default=1.2)
    p.add_argument('--first-head-hold-seconds',type=float)
    p.add_argument('--tail-seconds',type=float,default=.75)
    p.add_argument('--tail-stroke-fraction',type=float,default=0.)
    p.add_argument('--tail-mid-stroke-fraction',type=float,default=1.)
    p.add_argument('--linear-tail',action='store_true',help='Draw the tail continuously, without stopping at its midpoint')
    p.add_argument('--subsequent-head-extra',type=float,default=0.,help='Fixed extra head press after the first deposit; tail targets stay unchanged')
    p.add_argument('--shell-pitch',type=float,default=.012)
    p.add_argument('--tail-distance',type=float,default=.012)
    p.add_argument('--start-y',type=float)
    p.add_argument('--shell-count',type=int,choices=range(1,9),default=1)
    p.add_argument('--release-tool',action='store_true')
    args=p.parse_args()
    if not .001<=args.press_stroke<=.016:p.error('Pressure diagnostic stroke limited to 1–16 mm')
    plan=compile_plan(args.mode,args.press_stroke,args.shell_count,args.release_tool,
        first_press_stroke=args.first_press_stroke,head_hold_seconds=args.head_hold_seconds,
        first_head_hold_seconds=args.first_head_hold_seconds,tail_seconds=args.tail_seconds,
        tail_stroke_fraction=args.tail_stroke_fraction,shell_pitch=args.shell_pitch,
        tail_distance=args.tail_distance,start_y=args.start_y,
        tail_mid_stroke_fraction=args.tail_mid_stroke_fraction,linear_tail=args.linear_tail,
        subsequent_head_extra=args.subsequent_head_extra)
    args.out.parent.mkdir(parents=True,exist_ok=True)
    args.out.write_text(json.dumps(plan,indent=2)+'\n')
    print(json.dumps({k:v for k,v in plan.items() if k!='frames'}))


if __name__=='__main__':main()
