"""Privileged local feasibility controller. Free objects move only through robot contact.

Four-pad execution is gated by a separate one-pad result. Failed transitions raise
with evidence rather than continuing through a fictitious successful state.
"""
import numpy as np
from config import S,F,P,T,HZ,FLOOR_Z,PAD_H,FOOT_CENTERS
from control_utils import Arm,solve,rot_from,wait
from scoring import points,rotation
from yam_arena.constants import READY
import adapter


class Context:
    def __init__(self,emb):
        self.emb=emb
        self.arms={s:Arm(s,adapter.arm_state(emb._base,s)) for s in ('left','right')}
        self.stage='init'
        self.routes={}
        self.events=[]

    @property
    def measurement(self): return adapter.last_record['measurement']

    def foot(self,i): return points(self.measurement['furniture'],[FOOT_CENTERS[i]])[0]

    def pad(self,name): return np.asarray(self.measurement['pads'][name][:3])

    def action(self):
        if adapter.monitor.detail.get('tilt_deg',0)>12:
            raise RuntimeError('Furniture tilt exceeded the controlled insertion envelope')
        action=np.r_[self.arms['left'].command(),self.arms['right'].command()]
        if action.shape!=(14,) or not np.isfinite(action).all(): raise RuntimeError('Invalid 14-D action')
        return action

    def released(self): return bool(adapter.monitor.detail.get('stable_released'))


def park(ctx,side,duration=2.):
    from motion_plan import released_parking_goal
    arm=ctx.arms[side]
    arm.load_N=0.
    goal=released_parking_goal(ctx.measurement['furniture'],side)
    if side in ctx.routes:
        from motion_plan import World,path
        ctx.routes.pop(side)
        other='left' if side=='right' else 'right'
        world=World(ctx.measurement['furniture'],adapter.arm_state(ctx.emb._base,other),side,narrow=True)
        route=path(arm.q,goal,world)
        ctx.events.append({'event':'retreat_path','step':ctx.measurement['step'],
                           'side':side,'q':[v.tolist() for v in route]})
        for q in route[1:]:
            yield from arm.move_q(q,max(.5,float(np.max(abs(q-arm.q)))/.85),grip=1.)
    else:
        yield from arm.move_q(goal,duration,grip=1.)


def approach_support(ctx,side,goal):
    from motion_plan import World,path
    other='left' if side=='right' else 'right'
    world=World(ctx.measurement['furniture'],adapter.arm_state(ctx.emb._base,other),side,ctx.arms[side].grip,
                narrow=(ctx.stage.startswith(('move_approach','turn_approach','shift_approach','lift_audit'))
                        or (ctx.stage=='support_approach' and side=='left')))
    try:
        route=path(ctx.arms[side].q,goal,world)
    except RuntimeError as error:
        if world.narrow:raise
        # A merged link box can include empty space between its native hulls.
        # Retry with the existing finer conservative hull bounds; an actual
        # collision still fails this check before any arm action is issued.
        ctx.events.append({'event':'per_hull_path_retry','step':ctx.measurement['step'],
                           'side':side,'reason':str(error)})
        world=World(ctx.measurement['furniture'],adapter.arm_state(ctx.emb._base,other),
                    side,ctx.arms[side].grip,narrow=True)
        route=path(ctx.arms[side].q,goal,world)
    ctx.routes[side]=route
    ctx.events.append({'event':'support_path','step':ctx.measurement['step'],'side':side,'q':[v.tolist() for v in route],
                       'screened_states':len(world.cache)})
    for q in route[1:]:
        yield from ctx.arms[side].move_q(q,max(.5,float(np.max(abs(q-ctx.arms[side].q)))/.85))


def push_pad(ctx,name,target,side='left'):
    """Rim push in the side-to-centre direction, using the closed fingers as a pusher."""
    arm=ctx.arms[side]
    sign=1 if side=='left' else -1
    rot=rot_from((sign,0,0),(0,-sign*.342,-.940))
    p=ctx.pad(name)
    # Tip extends 13 mm beyond TCP; choose the height to catch the pad's upper rim.
    z=FLOOR_Z+PAD_H+.009
    start=np.array((p[0],p[1]+sign*(P['diameter_m']/2+.026),z))
    above=start+np.array((0,0,.10))
    ctx.stage='pad_approach'
    q=solve(above,rot,arm.q,arm.base)
    yield from arm.move_q(q,2.,grip=0.)
    yield from arm.move(start,rot,1.)
    ctx.stage='pad_preposition'
    # Closed fingertip feedback follows the disk and corrects lateral drift.
    # The two open fingertips did not both engage the short pad in the native geometry.
    for _ in range(220):
        pad=ctx.pad(name)
        if sign*(pad[1]-target[1])<.002 and abs(pad[0]-target[0])<.005: break
        x=pad[0]+np.clip((pad[0]-target[0])*1.2,-.020,.020)+.0015
        desired=np.array((x,pad[1]+sign*.040,z))
        delta=desired-arm.pos
        delta=np.clip(delta,-.0015,.0015)
        arm.aim(arm.pos+delta,rot)
        yield
    else: raise RuntimeError('Measured pad preposition timed out')
    yield from wait(.3)
    if np.linalg.norm(ctx.pad(name)[:2]-target[:2])>.018:
        raise RuntimeError(f'Pad rim push missed target: {ctx.pad(name).tolist()} versus {target.tolist()}')
    yield from arm.move(arm.pos+np.array((0,sign*.025,.065)),rot,1.)


def insert(ctx,foot_index,pad_name,refine_touchdown=False):
    foot=ctx.foot(foot_index)
    sign=1 if foot[1]>=0 else -1
    support_side='right' if sign==1 else 'left'
    insert_side='left' if sign==1 else 'right'
    support=ctx.arms[support_side]
    inserter=ctx.arms[insert_side]
    # Preposition while the support hand is parked; leave a visible clearance to the bare foot.
    target=foot.copy(); target[1]+=sign*.100
    if abs(ctx.pad(pad_name)[0]-target[0])>.025:
        yield from shift_pad_x(ctx,pad_name,target[0],insert_side)
        yield from park(ctx,insert_side,1.5)
    yield from push_pad(ctx,pad_name,target,insert_side)
    yield from park(ctx,insert_side,1.5)
    ctx.stage='support_approach'
    foot=ctx.foot(foot_index)
    # Mirroring this Y-axis pinch keeps closing +Y on both arms. Reversing it
    # unnecessarily rolled the left wrist by 180 degrees toward its limit.
    rot=rot_from((0,1,0),(.866,0,-.5))
    grasp=foot+np.array((0,0,.1325))
    approach_distance=.05 if refine_touchdown else .07
    pre=grasp-np.array((approach_distance,0,0))
    # Known branch; intermediate pregrasp approaches along the open leg corridor.
    seed=np.radians([87.09,132.13,115.92,-68.77,59.87,-95.8])
    if sign==-1: seed=np.array([-seed[0],seed[1],seed[2],seed[3],-seed[4],-seed[5]])
    q=solve(pre,rot,seed,support.base)
    yield from approach_support(ctx,support_side,q)
    yield from support.move(grasp,rot,1.5)
    ctx.stage='grasp'
    yield from support.move(grasp,rot,1.,grip=.43)
    yield from wait(.5)
    forces=adapter.last_record['diagnostics']['arms'][support_side]['finger_contacts_N']
    if min(forces['tip_left'][0],forces['tip_right'][0])<.5:
        raise RuntimeError('No opposing finger contact on support leg')
    initial=ctx.foot(foot_index).copy()
    ctx.stage='lift'
    # Ramp nominal load feedforward, then raise with observed clearance feedback.
    for k in range(1,21):
        support.load_N=F['mass_kg']*9.81*.5*k/20
        yield
    for _ in range(100):
        foot=ctx.foot(foot_index)
        if foot[2]-FLOOR_Z>=PAD_H+S['control']['lift_clearance_m']: break
        if support.pos[2]-grasp[2]>.065:
            raise RuntimeError('Lift failed within bounded TCP correction')
        support.aim(support.pos+np.array((0,0,.0006)),rot)
        yield
    else: raise RuntimeError('Lift clearance timeout')
    if ctx.foot(foot_index)[2]-initial[2]>S['control']['max_corner_lift_m']:
        raise RuntimeError('Corner lift exceeded 30 mm')
    ctx.stage='insert'
    pad=ctx.pad(pad_name)
    foot=ctx.foot(foot_index)
    pro=rot_from((sign,0,0),(0,-sign*.342,-.940))
    z=FLOOR_Z+PAD_H+.009
    start=np.array((pad[0],pad[1]+sign*(P['diameter_m']/2+.022),z))
    q=solve(start+np.array((0,0,.055)),pro,inserter.q,inserter.base)
    yield from inserter.move_q(q,2.,grip=0.)
    yield from inserter.move(start,pro,1.)
    for _ in range(160):
        foot=ctx.foot(foot_index)
        pad=ctx.pad(pad_name)
        if np.linalg.norm(pad[:2]-foot[:2])<.004: break
        if foot[2]-FLOOR_Z<PAD_H+.002:
            raise RuntimeError('Supporting grasp lost insertion clearance')
        x=pad[0]+np.clip((pad[0]-foot[0])*1.2,-.020,.020)+.0015
        desired=np.array((x,pad[1]+sign*.040,z))
        inserter.aim(inserter.pos+np.clip(desired-inserter.pos,-.0015,.0015),pro)
        yield
    else: raise RuntimeError('Measured insertion push timed out')
    yield from wait(.25)
    if np.linalg.norm(ctx.pad(pad_name)[:2]-ctx.foot(foot_index)[:2])>.006:
        raise RuntimeError('Insertion alignment exceeds the foot/pad margin')
    if refine_touchdown:
        # The rear-foot landing moved sideways by 4.4 mm as the tilted table
        # became level. Align again near touchdown, while the pad is unloaded,
        # so the raised-foot projection is not mistaken for its landing point.
        yield from inserter.move(inserter.pos+np.array((0,sign*.012,0)),pro,.5)
        ctx.stage='near_touchdown'
        for _ in range(100):
            if ctx.foot(foot_index)[2]-FLOOR_Z<=PAD_H+.0035:break
            support.aim(support.pos-np.array((0,0,.00025)),rot)
            yield
        else:raise RuntimeError('Near-touchdown lowering timed out')
        ctx.stage='refine_pad'
        for _ in range(90):
            foot=ctx.foot(foot_index);pad=ctx.pad(pad_name)
            error=pad[:2]-foot[:2]
            if abs(error[0])<.003 and abs(error[1])<.001 and np.linalg.norm(error)<.0032:break
            if sign*error[1]<-.002:
                raise RuntimeError('Pad overshot the near-touchdown target; opposite-side recovery required')
            if ctx.measurement['pad_loads_N'][pad_name]>1.:
                raise RuntimeError('Near-touchdown pad is already loaded; lift recovery required')
            x=pad[0]+np.clip((pad[0]-foot[0])*1.2,-.020,.020)+.0015
            desired=np.array((x,pad[1]+sign*.040,z))
            inserter.aim(inserter.pos+np.clip(desired-inserter.pos,-.0007,.0007),pro)
            yield
        else:raise RuntimeError('Near-touchdown pad refinement timed out')
        yield from wait(.25)
        ctx.events.append({'event':'near_touchdown_alignment','step':ctx.measurement['step'],
                           'foot':foot_index,'pad':pad_name,
                           'foot_minus_pad_m':(ctx.foot(foot_index)-ctx.pad(pad_name)).tolist()})
    yield from inserter.move(inserter.pos+np.array((0,sign*.04,.04)),pro,1.)
    yield from park(ctx,insert_side,2.)
    ctx.stage='lower'
    for _ in range(120):
        # A tilted rigid foot first bears on an edge, so centre height is not
        # the contact gap. Require contained geometry AND observed load transfer.
        matched=(foot_index,pad_name) in [tuple(v) for v in adapter.monitor.detail['matches']]
        if matched and ctx.measurement['pad_loads_N'][pad_name]>=1.: break
        support.aim(support.pos-np.array((0,0,.00025)),rot)
        yield
    else: raise RuntimeError('Lowering did not transfer support')
    # Transfer to the measured loaded configuration, not the old unloaded IK
    # target. Removing feedforward at that old target drove the hand downward
    # and stored strain before release in trial 006.
    from yam_arena import ik
    measured=adapter.arm_state(ctx.emb._base,support_side)
    support.q=support.command()[:6].copy()
    support.load_N=0.
    support.pos,support.rot=ik.tcp_pose(support.q,support.base)
    yield from support.move_q(measured,1.)
    # A second small measured-target transfer removes residual support force.
    yield from support.move_q(adapter.arm_state(ctx.emb._base,support_side),.5)
    yield from wait(.5)
    rot=support.rot.copy()
    ctx.stage='release'
    yield from support.move(support.pos,rot,.7,grip=1.)
    yield from support.move(support.pos-np.array((approach_distance,0,0)),rot,1.)
    yield from park(ctx,support_side,2.)
    ctx.stage='release_check'
    yield from wait(2.1)
    if [foot_index,pad_name] not in [list(v) for v in adapter.monitor.detail['matches']] or not ctx.released():
        raise RuntimeError('Released slider support check failed')


def move_furniture(ctx,distance,feet=(0,1),target_x=None):
    # Matching pushes on both front legs resist yaw caused by unequal drag on
    # the one padded and the remaining bare supports.
    rot=rot_from((0,-1,0),(.866,0,-.5))
    start=np.asarray(ctx.measurement['furniture'][:3])
    starts={}
    for side,i,sgn in [('left',feet[0],1),('right',feet[1],-1)]:
        arm=ctx.arms[side]; foot=ctx.foot(i)
        pre=np.array((foot[0]-.05,foot[1],FLOOR_Z+.1225))
        ctx.stage='move_approach_'+side
        yield from arm.move(arm.pos,arm.rot,.6,grip=0.)
        seed=np.radians([-88.8*sgn,65.,36.5,-59.5,-60.*sgn,-87.7*sgn])
        q=solve(pre,rot,seed,arm.base)
        yield from approach_support(ctx,side,q)
        starts[side]=pre
    ctx.stage='move'
    from yam_arena import ik
    length=distance+.04
    count=round(length/(.02 if target_x is None else .012)*HZ)
    dx=0.
    for k in range(1,count+1):
        if target_x is None:
            dx=length*ik.smoothstep(k/count)
        else:
            remaining=target_x-float(ctx.measurement['furniture'][0])
            if remaining<=.001:break
            dx+=min(.001*k/20,.001,max(.00025,remaining*.08))
            if dx>length+.025:raise RuntimeError('Goal push exceeds bounded contact take-up')
        for side,pre in starts.items():ctx.arms[side].aim(pre+np.array((dx,0,0)),rot)
        yield
    yield from wait(.5)
    if np.linalg.norm(np.asarray(ctx.measurement['furniture'][:2])-start[:2])<distance-.005:
        raise RuntimeError('Furniture push fell short of transport distance')
    ctx.stage='move_release'
    for side in ('left','right'):
        arm=ctx.arms[side]
        yield from arm.move(arm.pos-np.array((.06,0,0)),rot,1.)
    for side in ('left','right'):yield from park(ctx,side,2.)
    yield from wait(2.1)


def shift_pad_x(ctx,name,x,side):
    """Move a loose outer pad along X before the validated inward rim push."""
    arm=ctx.arms[side]
    direction=1. if x>ctx.pad(name)[0] else -1.
    rot=rot_from((0,-1,0),(0,0,-1))
    p=ctx.pad(name); target_y=p[1]; z=FLOOR_Z+PAD_H+.009
    start=np.array((p[0]-direction*(P['diameter_m']/2+.026),p[1],z))
    ctx.stage='shift_approach'
    yield from arm.move(arm.pos,arm.rot,.6,grip=0.)
    q=solve(start+np.array((0,0,.03)),rot,arm.q,arm.base)
    yield from approach_support(ctx,side,q)
    yield from arm.move(start,rot,1.)
    ctx.stage='shift_pad'
    for _ in range(260):
        p=ctx.pad(name)
        if direction*(x-p[0])<.003 and abs(p[1]-target_y)<.008:break
        desired=np.array((p[0]-direction*.040,p[1]+np.clip((p[1]-target_y)*1.2,-.020,.020),z))
        arm.aim(arm.pos+np.clip(desired-arm.pos,-.0015,.0015),rot)
        yield
    else:raise RuntimeError('Longitudinal loose-pad positioning timed out')
    yield from arm.move(arm.pos+np.array((-direction*.035,0,.03)),rot,1.)


def yaw_angle(ctx):
    r=rotation(ctx.measurement['furniture'][3:])
    return float(np.arctan2(r[1,0],r[0,0]))


def turn_half(ctx):
    """Two physical quarter turns using diagonal leg grasps to share the wrench.

    Both grippers guide the turn; supports remain free contacts. The table has
    no programmed pivot, attachment or orientation constraint.
    """
    from scipy.spatial.transform import Rotation
    from yam_arena import ik
    # Keep the two unused disks outside the swept circle. These are physical
    # rim pushes, not changes to the reset layout or pad poses.
    for side,name in [('left','slider_2'),('right','slider_3')]:
        target_x=float(ctx.measurement['furniture'][0]-F['foot_spacing_m']/2)
        yield from shift_pad_x(ctx,name,target_x,side)
        yield from park(ctx,side,1.5)
    for quarter,feet in enumerate(((0,3),(1,2))):
        backs={}
        for side,fi,local_rot,seed in (
            ('left',feet[0],rot_from((0,-1,0),(.866,0,-.5)),[-81.5,60.6,36.1,-51.1,-58.9,-73.4]),
            ('right',feet[1],rot_from((1,0,0),(0,.866,-.5)),[1.,138.,128.,-78.,-60.,-88.])):
            arm=ctx.arms[side];yaw=yaw_angle(ctx)
            error=float(np.arctan2(np.sin(yaw+quarter*np.pi/2),np.cos(yaw+quarter*np.pi/2)))
            rz=Rotation.from_euler('z',error).as_matrix();rot=rz@local_rot
            grasp=ctx.foot(fi)+np.array((0,0,.1325))
            back=rot[:,2].copy();back[2]=0.;back*=.05/np.linalg.norm(back)
            ctx.stage=f'turn_approach_{quarter}_{side}'
            q=solve(grasp-back,rot,np.radians(seed),arm.base)
            yield from approach_support(ctx,side,q)
            yield from arm.move(grasp,rot,1.2)
            yield from arm.move(grasp,rot,.8,grip=.43)
            yield from wait(.4)
            forces=adapter.last_record['diagnostics']['arms'][side]['finger_contacts_N']
            if min(forces['tip_left'][0],forces['tip_right'][0])<.5:
                raise RuntimeError('Quarter-turn grasp lacks opposing contacts')
            backs[side]=back
        centre=np.asarray(ctx.measurement['furniture'][:3]).copy()
        holds={}
        for side,arm in ctx.arms.items():
            p,r=ik.tcp_pose(adapter.arm_state(ctx.emb._base,side),arm.base)
            holds[side]=(p-centre,r)
        start_yaw=yaw_angle(ctx)
        target=-(quarter+1)*np.pi/2
        delta=float(np.arctan2(np.sin(target-start_yaw),np.cos(target-start_yaw)))
        ctx.stage=f'turn_{quarter}'
        count=round(abs(delta)/.12*HZ)
        for k in range(1,count+1):
            a=delta*ik.smoothstep(k/count)
            rr=Rotation.from_euler('z',a).as_matrix()
            for side,(offset,rot) in holds.items():
                ctx.arms[side].aim(centre+rr@offset,rr@rot)
            yield
            pairs=[tuple(v) for v in adapter.monitor.detail['matches']]
            if not all((i,f'slider_{i}') in pairs for i in (0,1)):
                raise RuntimeError('Installed pad lost support during physical turn')
        yield from wait(.5)
        actual_delta=float(np.arctan2(np.sin(yaw_angle(ctx)-start_yaw),np.cos(yaw_angle(ctx)-start_yaw)))
        ctx.events.append({'event':'quarter_turn','step':ctx.measurement['step'],'index':quarter,'arms':['left','right'],
                           'command_rad':delta,'actual_rad':actual_delta,'start_centre':centre.tolist(),
                           'end_centre':ctx.measurement['furniture'][:3]})
        if abs(actual_delta-delta)>np.radians(8):
            raise RuntimeError('Physical turn lag exceeds eight degrees; regrasp required')
        # Relax drive strain before opening, as in the validated insertion handoff.
        starts={s:a.q.copy() for s,a in ctx.arms.items()}
        ends={s:adapter.arm_state(ctx.emb._base,s) for s in ctx.arms}
        for k in range(1,15):
            for side,arm in ctx.arms.items():
                arm.q=starts[side]+ik.smoothstep(k/14)*(ends[side]-starts[side])
                arm.pos,arm.rot=ik.tcp_pose(arm.q,arm.base)
            yield
        ctx.stage=f'turn_release_{quarter}'
        grips={s:a.grip for s,a in ctx.arms.items()}
        for k in range(1,15):
            for side,arm in ctx.arms.items():arm.grip=grips[side]+ik.smoothstep(k/14)*(1-grips[side])
            yield
        for side,arm in ctx.arms.items():
            outward=Rotation.from_euler('z',actual_delta).as_matrix()@(backs[side]*.6)
            yield from arm.move(arm.pos-outward,arm.rot,1.)
        for side in ('left','right'):yield from park(ctx,side,2.)
        yield from wait(2.1)
        if not ctx.released() or len(adapter.monitor.detail['matches'])<2:
            raise RuntimeError('Two-pad released check failed after turn')


def whole_lift_audit(ctx):
    """Legitimate diagonal two-arm lift, to test the claimed support-change need.

    This is an alternative-solution audit, never a full-score shortcut. It does
    not insert pads, and the public scorer still requires supported sliding.
    """
    from yam_arena import ik
    grasps={}
    for side,fi,rot,seed in (
        ('left',0,rot_from((0,-1,0),(.866,0,-.5)),[-81.5,60.6,36.1,-51.1,-58.9,-73.4]),
        ('right',3,rot_from((1,0,0),(0,.866,-.5)),[1.,138.,128.,-78.,-60.,-88.])):
        arm=ctx.arms[side];p=ctx.foot(fi)+np.array((0,0,.1325))
        back=rot[:,2].copy();back[2]=0.;back*=.05/np.linalg.norm(back)
        ctx.stage='lift_audit_approach_'+side
        q=solve(p-back,rot,np.radians(seed),arm.base)
        yield from approach_support(ctx,side,q)
        yield from arm.move(p,rot,1.2)
        yield from arm.move(p,rot,.8,grip=.43)
        yield from wait(.4)
        forces=adapter.last_record['diagnostics']['arms'][side]['finger_contacts_N']
        if min(forces['tip_left'][0],forces['tip_right'][0])<.5:
            raise RuntimeError('Whole-lift audit lacks opposing leg contact')
        grasps[side]=(p.copy(),rot.copy(),back.copy())
    ctx.stage='lift_audit_raise'
    for k in range(1,31):
        for arm in ctx.arms.values():arm.load_N=F['mass_kg']*9.81*.5*k/30
        yield
    target=PAD_H+S['control']['lift_clearance_m']
    for _ in range(100):
        clearance=min(np.asarray(adapter.monitor.detail['foot_corners'])[:,:,2].ravel())-FLOOR_Z
        if clearance>=target:break
        for side,(start,rot,back) in grasps.items():
            arm=ctx.arms[side]
            if arm.pos[2]-start[2]>.065:raise RuntimeError('Whole-lift bounded correction exhausted')
            arm.aim(arm.pos+np.array((0,0,.0006)),rot)
        yield
    else:raise RuntimeError('Whole-lift clearance not achieved')
    ctx.stage='lift_audit_hold'
    observed=[]
    for _ in range(round(2.*HZ)):
        observed.append(float(np.asarray(adapter.monitor.detail['foot_corners'])[:,:,2].min()-FLOOR_Z))
        yield
    passed=min(observed)>=target-.001
    ctx.events.append({'event':'whole_lift_audit','step':ctx.measurement['step'],'passed':passed,'minimum_held_clearance_m':min(observed),
                       'hold_s':2.,'interpretation':'A passed lift weakens any claim that partial-load insertion is necessary.'})
    ctx.stage='lift_audit_lower'
    for _ in range(180):
        if np.asarray(adapter.monitor.detail['foot_corners'])[:,:,2].min()-FLOOR_Z<.0005:break
        for side,(_,rot,_) in grasps.items():
            arm=ctx.arms[side];arm.aim(arm.pos-np.array((0,0,.00025)),rot)
        yield
    else:raise RuntimeError('Whole-lift audit could not lower onto floor')
    starts={};ends={}
    for side,arm in ctx.arms.items():
        starts[side]=arm.command()[:6].copy();ends[side]=adapter.arm_state(ctx.emb._base,side)
        arm.load_N=0.
    for k in range(1,31):
        for side,arm in ctx.arms.items():
            arm.q=starts[side]+ik.smoothstep(k/30)*(ends[side]-starts[side])
            arm.pos,arm.rot=ik.tcp_pose(arm.q,arm.base)
        yield
    ctx.stage='lift_audit_release'
    for side,arm in ctx.arms.items():
        yield from arm.move(arm.pos,arm.rot,.7,grip=1.)
        yield from arm.move(arm.pos-grasps[side][2],arm.rot,1.)
    for side in ('left','right'):yield from park(ctx,side,2.)
    yield from wait(2.1)
    if not passed:raise RuntimeError('Whole-object lift slipped during the hold')


def script(ctx,mode):
    ctx.stage='stabilize'
    yield from wait(.5)
    if mode=='lift':
        yield from whole_lift_audit(ctx)
        return
    if mode=='rear-debug':
        ctx.stage='diagnostic_park'
        for side in ('left','right'):yield from park(ctx,side,1.)
        yield from wait(2.1)
        if not ctx.released():raise RuntimeError('Initialized rear diagnostic did not settle released')
    if mode not in ('continuation-debug','turn-debug','rear-debug'):
        yield from insert(ctx,0,'slider_0')
    if mode=='one-pad':
        yield from move_furniture(ctx,.105)
        if not adapter.monitor.gate_pass: raise RuntimeError('One-pad released movement gate failed')
        return
    if mode not in ('turn-debug','rear-debug'):yield from insert(ctx,1,'slider_1')
    if mode=='two-pad':return
    if mode!='rear-debug':yield from turn_half(ctx)
    if mode in ('turn','turn-debug'):return
    yield from insert(ctx,3,'slider_2',refine_touchdown=True)
    yield from insert(ctx,2,'slider_3',refine_touchdown=True)
    distance=float(S['layout']['goal_xy_m'][0]-ctx.measurement['furniture'][0])
    if distance<T['transport_distance_m']:
        raise RuntimeError('Insufficient goalward room for the required supported movement')
    yield from move_furniture(ctx,distance,feet=(3,2),target_x=S['layout']['goal_xy_m'][0])
    if not adapter.monitor.success:raise RuntimeError('Final four-pad score did not reach full success')
