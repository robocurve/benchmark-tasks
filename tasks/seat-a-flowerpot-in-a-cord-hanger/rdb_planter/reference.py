"""A state-assisted demonstration that moves only the robot's fourteen commanded joints."""
from dataclasses import dataclass, replace
import numpy as np
from scipy.spatial.transform import Rotation
from inspect_robots import Action, ActionChunk
from inspect_robots.policy import PolicyBase
from yam_arena import ik
from yam_arena.scripted import YamScriptedPolicy
from . import fixed_scene as s
from .web_center import center as supporting_center

# Gripper frame: x closes the jaws and z extends along the fingers.
DOWN = np.diag([1.,-1.,-1.])
SIDE = np.column_stack(([0.,0.,1.],[0.,-1.,0.],[1.,0.,0.])) @ Rotation.from_euler("y", -10, degrees=True).as_matrix()
ACTIVE = None

@dataclass
class Target:
    body: str
    delta: tuple
    jaw: float
    rotation: np.ndarray
    local: bool = False

def at(body, delta, jaw, rotation=DOWN, local=False):
    return Target(body, delta, jaw, rotation, local)

def entry_grip():
    # Reach around the outside lip from the open side of the suspended web.
    yaw,tilt=np.deg2rad([-145.,52.])
    radial=np.array([np.cos(yaw),np.sin(yaw),0.])
    forward=-np.sin(tilt)*radial-np.cos(tilt)*np.array([0.,0.,1.])
    tangent=np.cross(forward,radial)
    tangent/=np.linalg.norm(tangent)
    return radial,np.column_stack((np.cross(tangent,forward),tangent,forward))

def plan():
    # A rim pinch uses one finger inside the empty upper 3 cm and one outside.
    radial, _ = entry_grip()
    rim_offset=np.array([0.,-.058,.086])
    rim=at('pot',rim_offset,.005)
    incline = np.deg2rad(15.)
    forward = np.array([0., -np.cos(incline), -np.sin(incline)])
    closing = np.array([-1., 0., 0.])
    crosswise = np.column_stack((closing, np.cross(forward, closing), forward))
    def approach(distance, jaw):
        return at('far_pair', distance*forward, jaw, crosswise)
    return [
        ('clear the outside of the far cords',60,(approach(-.075,.35),at('pot',(0,-.058,.13),.48))),
        ('approach cord pair and pot rim',40,(approach(-.030,.35),at('pot',rim_offset,.42))),
        ('straddle the two far cords',44,(approach(0.,.28),rim)),
        ('close on the far pair with a fixed wrist',36,(at('same_wrist',(0,0,0),.006,crosswise),at('world',(.30,-.203,s.TOP+.118),.005))),
        ('spread the opening while raising the pot',128,(at('pad',(-.090,.135,.012),.006,crosswise),at('world',(.30,-.200,s.TOP+.217),.005))),
        ('turn the held rim toward the entry side',70,(None,at('turn_pot',(0,0,0),.005))),
        ('move around the side of the near cords',58,(None,at('mouth_goal',(-.115,0,.085),.005))),
        ('enter between the two strand pairs',42,(None,at('mouth_goal',(-.056,0,.080),.005))),
        ('centre between both pairs',66,(None,at('mouth_goal',(0,0,.065),.005))),
        ('follow the swinging cradle',40,(None,at('base_goal',(0,0,.024),.005))),
        ('offer weight to the supporting knot',174,(None,at('base_goal',(0,0,.006),.005))),
        ('centre the held base over the knot',40,(None,at('center_before_release',(-.013,.011,-.015),.005))),
        ('move both grips beneath the hook',150,(at('return_strand',(.008,-.105,-.027),.006,crosswise),at('carry_under_hook',(0.,0.,.1555),.005))),
        ('give the cord its freedom while holding the lip',35,(at('same_wrist',(0,0,0),.48,crosswise),None)),
        ('withdraw the strand fingers',65,(at('world',(.23,.215,s.TOP+.20),.72),None)),
        ('lower the base onto the relaxed web',45,(None,at('lower_under_hook',(0.,0.,.016),.005))),
        ('release the lip into the relaxed web',14,(None,at('same_wrist',(0,0,0),.53))),
        ('withdraw the right fingers from the lip',50,(None,at('pad',(0.,-.009,.080),.55))),
        ('retreat outside the released net',100,(None,at('out_of_net',(-.140,-.085,.055),.55))),
        ('watch suspension',150,(None,None))]

def rim_first(side):
    """Use the pot as the spreader while the hook and table supply support."""
    sign=1 if side=='left' else -1
    hand=0 if side=='left' else 1
    waypoints=[]
    def add(label,ticks,body,point,jaw):
        targets=[None,None]
        targets[hand]=at(body,point,jaw)
        waypoints.append((label,ticks,tuple(targets)))
    add('approach the near wall of the pot',44,'pot',(0.,sign*.058,.129),.55)
    add('straddle the open rim',28,'pot',(0.,sign*.058,.086),.45)
    add('pinch the pot wall',28,'pot',(0.,sign*.058,.086),.005)
    add('lift without external support',38,'pot',(0.,sign*.058,.18),.005)
    # Positions relative to the simulated, freely swinging lower knot.
    add('offer the far edge between the near cords',55,'bottom_knot',(-.012,sign*.132,.168),.005)
    add('push the pot through the opening rim first',52,'bottom_knot',(-.006,sign*.077,.153),.005)
    add('centre the base above the bottom knot',48,'bottom_knot',(0.,sign*.058,.122),.005)
    add('lower and give the cradle the load',48,'bottom_knot',(0.,sign*.058,.093),.005)
    add('release the lip',26,'pot',(0.,sign*.058,.086),.70)
    add('withdraw for the swing test',64,'world',(.24,sign*.25,s.TOP+.26),.74)
    return waypoints

class Reference(PolicyBase):
    def __init__(self,rig,variant=None):
        self.rig=rig;self.variant=variant
        self.info=replace(YamScriptedPolicy().info,name='planter_reference')
    def bind(self,embodiment_info): pass
    def reset(self,scene):
        if self.variant == 'idle':
            self.stages = []
        elif self.variant in ('left_only', 'right_only'):
            self.stages = rim_first(self.variant.split('_')[0])
        else:
            self.stages = plan()
        self.idx=0;self.elapsed=0;self.goal=None;self.log=[]
    def follow_base(self,joints,target):
        # Use measured geometry throughout placement. At 20 Hz the empty knot may
        # cross the pot during a single 2 s waypoint; a stale target pushes it away.
        base=np.array((0.,-.305,.75))
        tcp,rotation=ik.tcp_pose(joints,base)
        pad=tcp+rotation@np.array([0.,-.008,-.010])
        pot,pot_rot=self.rig.poses['pot']
        knot=self.rig.poses['bottom_knot'][0]
        centre=knot.copy()
        if target.body=='mouth_goal':
            section=self.rig.section(knot[2]+.115)
            if section is not None:
                centre[:2]=section[:,:2].mean(axis=0)
                # Approach an open side of the actual parallelogram between
                # near and far cord pairs, even if the net has twisted on its hook.
                spread=section[:2,:2].mean(axis=0)-section[2:,:2].mean(axis=0)
                if np.linalg.norm(spread)>.025:
                    side=np.array([-spread[1],spread[0]])/np.linalg.norm(spread)
                    if side[0]>0:side=-side
                    centre[:2]+=-float(target.delta[0])*side
                    centre[0]-=target.delta[0]
        # Track the suspended opening without integrating the arm's gravitational
        # sag into the next target. Freeze the measured pot-to-pad transform
        # just after the lift, when the rim is held well above the table.
        # The verified two-strand grip displaces the knot by 71 mm. Follow
        # the measured opening, subject to the task workspace and speed limit.
        delta=centre[:2]-s.HOOK[:2]
        centre[:2]=s.HOOK[:2]+delta*min(1.,.175/max(np.linalg.norm(delta),1e-8))
        centre+=np.array(target.delta)
        if target.body=='base_goal':
            grade=self.rig.final_grade or {}
            sections=grade.get('pot_local_sections',[])
            local_knot=np.asarray(grade.get('knot_local_m',[0.,0.,-1.]))
            if len(sections)>1:
                inside=supporting_center(sections[1]['cord_xy_m'],local_knot[:2])
                if inside is not None:
                    xy=pot[:2]+(pot_rot@np.r_[inside,0.])[:2]
                    # Use the lateral web correction only where the knot can
                    # remain under the physical 44 mm base at the same time.
                    span=xy-knot[:2]
                    centre[:2]=knot[:2]+span*min(1.,.040/max(1.e-9,np.linalg.norm(span)))

        if not hasattr(self,'transport_origin'):
            self.transport_origin=pot.copy()
            self.commanded_base=pot.copy()
            self.base_bias=np.zeros(3)
            self.held_rot=pot_rot.copy()
            self.local_pad=pot_rot.T@(pad-pot)
            self.local_wrist=pot_rot.T@rotation
            self.level_fraction=0.
            self.level_trim=np.zeros(3)
            self.approach_yaw=0.
            self.gravity_bias=np.clip(self.goal[7:13]-joints,-.055,.055)
        if target.body=='turn_pot':
            centre=self.transport_origin.copy()
            self.approach_yaw=np.deg2rad(-55.)*ik.smoothstep(min(self.elapsed/65.,1.))
        shift=centre-self.commanded_base
        self.commanded_base+=shift*min(1.,.0035/max(np.linalg.norm(shift),1e-8))
        # Keep the verified grip attitude during transit; level gradually just
        # above the lower web, around the base of the pot.
        if target.body=='base_goal':
            self.level_fraction=min(1.,self.level_fraction+.018)
        axis=np.cross(self.held_rot[:,2],[0.,0.,1.])
        angle=np.arctan2(np.linalg.norm(axis),self.held_rot[2,2])
        correction=Rotation.from_rotvec(axis/max(np.linalg.norm(axis),1e-8)*angle*self.level_fraction).as_matrix()
        pose=Rotation.from_euler('z',self.approach_yaw).as_matrix()@correction@self.held_rot
        if target.body=='base_goal':
            # Contact with a tensioned strand can roll the pot in the pads.
            # Trim against observed tilt while bearing the load.
            tilt_error=np.cross(pot_rot[:,2],[0.,0.,1.])
            self.level_trim=np.clip(self.level_trim+.025*tilt_error,-.21,.21)
            pose=Rotation.from_rotvec(self.level_trim).as_matrix()@pose
        wanted_rot=pose@self.local_wrist
        # An external cord load moved the bottom of the held pot by up to
        # 19 mm in the previous trial. Correct measured placement drift
        # gradually while both fingers still grip the wall.
        actual_error=self.commanded_base-pot
        self.base_bias=np.clip(self.base_bias+.065*np.clip(actual_error,-.0025,.0025),-.020,.020)
        wanted_pad=self.commanded_base+self.base_bias+pose@self.local_pad
        wanted_tcp=wanted_pad-wanted_rot@np.array([0.,-.008,-.010])
        q,err,ang=ik.solve(wanted_tcp,ik.matrix_to_quat_xyzw(wanted_rot),joints,base,iters=55,damping=.025,rot_weight=.20)
        q=np.clip(q+self.gravity_bias,ik.LOWER,ik.UPPER)
        return q,dict(position_error_m=float(err),orientation_error_rad=float(ang),
                      actual_base_error_m=float(np.linalg.norm(self.commanded_base-pot)),
                      desired_base=self.commanded_base.tolist(),base_trim_mm=(1000*self.base_bias).round(2).tolist())

    def act(self,observation):
        measured=np.asarray(observation.state['joint_pos'],dtype=float)
        if self.goal is None:self.goal=measured.copy()
        if self.idx<len(self.stages):
            name,n,targets=self.stages[self.idx]
            if self.elapsed==0:
                self.start=self.goal.copy();self.end=self.goal.copy()
                diagnostics=[]
                for i,target in enumerate(targets):
                    arm=('left','right')[i];j=i*7
                    if target is None or self.variant in ('idle',('right_only','left_only')[i]):continue
                    if target.body == 'same_wrist':
                        self.end[j+6] = target.jaw
                        continue
                    pad_offset = np.array([0.,-.004,.008]) if i == 0 and self.variant != 'left_only' else np.array([0.,-.008,-.010])
                    if target.body=='center_before_release':
                        base=(0.,-.305,.75)
                        joints=measured[7:13]
                        wrist,rotation=ik.tcp_pose(joints,base)
                        # In two free suspensions the knot landed toward the
                        # same quadrant of the base. Move the held pot into
                        # that part of the open web before lowering its weight.
                        _,pot_rotation=self.rig.poses['pot']
                        local=np.asarray(target.delta)
                        shift=pot_rotation@np.r_[local[:2],0.]
                        shift[2]=local[2]
                        goal=wrist+shift
                        q,err,rerr=ik.solve(goal,ik.matrix_to_quat_xyzw(rotation),joints,
                                            base,iters=100,damping=.022,rot_weight=.14)
                        self.start[7:13]=joints
                        self.end[7:13]=q+self.gravity_bias
                        self.end[13]=target.jaw
                        diagnostics.append(dict(arm=arm,goal=goal.tolist(),position_error_m=err,orientation_error_rad=rerr))
                        continue
                    if target.body in ('carry_under_hook','lower_under_hook'):
                        base=(0.,-.305,.75)
                        joints=measured[7:13]
                        wrist,rotation=ik.tcp_pose(joints,base)
                        pot,pot_rotation=self.rig.poses['pot']
                        knot=self.rig.poses['bottom_knot'][0]
                        if target.body=='carry_under_hook':
                            # Support the rim while the left fingers move the
                            # single loaded strand back beneath the hook. Leave
                            # vertical clearance for the lacing to redistribute.
                            center=np.r_[s.HOOK[:2],s.TOP+target.delta[2]]
                            axis=np.cross(pot_rotation[:,2],[0.,0.,1.])
                            angle=np.arctan2(np.linalg.norm(axis),pot_rotation[2,2])
                            turn=.4*angle*axis/max(np.linalg.norm(axis),1.e-9)
                            correction=Rotation.from_rotvec(turn).as_matrix()
                        else:
                            # The left fingers have already withdrawn; the knot
                            # can move freely as the pot comes back into contact.
                            center=knot+pot_rotation[:,2]*target.delta[2]
                            correction=np.eye(3)
                        goal=center+correction@(wrist-pot)
                        attitude=ik.matrix_to_quat_xyzw(correction@rotation)
                        candidates=[ik.solve(goal,attitude,seed,base,iters=100,damping=.022,rot_weight=.14)
                                    for seed in (joints,joints+[0.,-.25,-.55,.25,0.,0.])]
                        q,err,rerr=min(candidates,key=lambda fit:fit[1]+.14*fit[2])
                        self.end[7:13]=np.clip(q+self.gravity_bias,ik.LOWER,ik.UPPER)
                        self.end[13]=target.jaw
                        diagnostics.append(dict(arm=arm,goal=goal.tolist(),base_goal=center.tolist(),
                             knot_local=self.rig.final_grade['knot_local_m'],position_error_m=err,orientation_error_rad=rerr))
                        continue
                    if target.body=='return_strand':
                        # The tensioned section was held outside the hook. Feed
                        # it back to its measured free position before opening.
                        base=(0.,.305,.75)
                        wrist,rotation=ik.tcp_pose(measured[:6],base)
                        goal=wrist+np.asarray(target.delta)
                        q,err,rerr=ik.solve(goal,ik.matrix_to_quat_xyzw(rotation),measured[:6],
                                            base,iters=100,damping=.022,rot_weight=.14)
                        self.end[:6]=q+(self.start[:6]-measured[:6])
                        self.end[6]=target.jaw
                        diagnostics.append(dict(arm=arm,goal=goal.tolist(),position_error_m=err,orientation_error_rad=rerr))
                        continue
                    if target.body=='out_of_net':
                        origin,rotation=ik.tcp_pose(measured[j:j+6],(0.,-.305,.75))
                        self.exit_position=origin.copy();self.exit_rotation=rotation.copy()
                        continue
                    if target.body=='world':pos=np.asarray(target.delta)
                    elif target.body=='far_pair':
                        far=(self.rig.poses['cord_0_10'][0]+self.rig.poses['cord_1_10'][0])/2
                        pos=far+np.asarray(target.delta)
                    elif target.body in ('base_goal','mouth_goal','turn_pot','pad'):
                        # Estimate the actual pinch-to-base displacement at every waypoint.
                        tcp,rot=ik.tcp_pose(measured[j:j+6],(0,.305*(1-2*i),.75))
                        pad=tcp+rot@pad_offset
                        pos=pad+np.asarray(target.delta)
                        if target.body not in ('pad','turn_pot'):
                            knot=self.rig.poses['bottom_knot'][0].copy()
                            if target.body=='mouth_goal':
                                section=self.rig.section(knot[2]+.115)
                                if section is not None: knot[:2]=section[:,:2].mean(axis=0)
                            pos=pos+knot-self.rig.poses['pot'][0]
                    else:
                        p,r=self.rig.poses[target.body]
                        pos=p+(r@target.delta if target.local else np.asarray(target.delta))
                    base=np.array((0,.305*(1-2*i),.75))
                    orientation=target.rotation
                    if target.rotation is DOWN:
                        radial=pos-base;radial[2]=0.
                        radial/=max(np.linalg.norm(radial),1e-9)
                        axis=np.cross([0.,0.,-1.],radial)
                        orientation=Rotation.from_rotvec(axis*np.deg2rad(22.)).as_matrix()@DOWN
                    # At the -Y lip, the fingers must straddle the wall along Y.
                    # Closing along the tangent in v1 only pushed the pot on the table.
                    if i==1 or self.variant=='left_only':
                        sign=1. if i==0 else -1.
                        radial=np.array([0.,sign,0.])
                        z=np.array([0.,-sign*.52,-.854]);z/=np.linalg.norm(z)
                        y=np.cross(z,radial);y/=np.linalg.norm(y)
                        orientation=np.column_stack((np.cross(y,z),y,z))
                    if i==1 and target.body=='pad' and getattr(self,'approach_yaw',0.):
                        orientation=rot
                    pos=pos-orientation@pad_offset
                    q,err,rerr=ik.solve(pos,ik.matrix_to_quat_xyzw(orientation),measured[j:j+6],
                                        base,iters=85,damping=.025,rot_weight=.14)
                    self.end[j:j+6]=q;self.end[j+6]=target.jaw
                    diagnostics.append(dict(arm=arm,goal=pos.tolist(),position_error_m=err,orientation_error_rad=rerr))
                self.log.append(dict(tick=self.rig.tick,stage=name,ik=diagnostics))
            self.elapsed+=1
            fraction=ik.smoothstep(self.elapsed/n)
            desired=self.start+fraction*(self.end-self.start)
            if targets[1] is not None and targets[1].body in ('base_goal','mouth_goal','turn_pot'):
                q,feedback=self.follow_base(measured[7:13],targets[1])
                desired[7:13]=q
                desired[13]=targets[1].jaw
                if self.elapsed in (1,n):
                    self.log[-1]['feedback']=feedback
            if name=='spread the opening while raising the pot' and self.variant not in ('idle','right_only','left_only'):
                observed=getattr(self.rig,'final_grade',{}) or {}
                section=observed.get('cradle_section_m')
                loads=observed.get('left_finger_cord_normals_n')
                if section is not None and len(section)==4 and loads is not None:
                    xy=np.asarray(section)[:,:2]
                    opening=np.linalg.norm(xy[:2].mean(axis=0)-xy[2:].mean(axis=0))
                    opposing=np.asarray(loads).reshape(2,4,-1).sum(axis=2).min(axis=0)
                    if not hasattr(self,'hold_open_q') and opening>=.102 and min(opposing[:2])>1.:
                        # The tapered base fits through this aperture. Further
                        # pulling allowed one strand to escape over the pad tip.
                        self.hold_open_q=self.goal[:7].copy()
                        self.log[-1]['restrained_opening']=dict(tick=self.rig.tick,metres=float(opening),forces=opposing[:2].tolist())
                if hasattr(self,'hold_open_q'):
                    desired[:7]=self.hold_open_q
            self.goal+=np.clip(desired-self.goal,-.12,.12)
            if targets[1] is not None and targets[1].body=='out_of_net' and self.variant not in ('idle','left_only'):
                actual=measured[7:13];rotation=self.exit_rotation
                tip,_=ik.tcp_pose(actual,(0.,-.305,.75))
                requested=self.exit_position+ik.smoothstep(self.elapsed/n)*np.asarray(targets[1].delta)
                error=np.asarray(requested)-tip
                waypoint=tip+error*min(1.,.003/max(np.linalg.norm(error),1.e-9))
                solved,e,a=ik.solve(waypoint,ik.matrix_to_quat_xyzw(rotation),actual,(0.,-.305,.75),iters=70,rot_weight=.09)
                if e<.008 and a<.18:
                    self.goal[7:13]=actual+np.clip(solved-actual,-.025,.025)
                self.goal[13]=targets[1].jaw
            grade=getattr(self.rig,'final_grade',{}) or {}
            gap=grade.get('strand_max_gap_deg')
            knot_offset=grade.get('knot_local_m',(1.,1.,1.))
            surrounded=(name=='offer weight to the supporting knot' and gap is not None and
                        gap<177. and np.linalg.norm(knot_offset[:2])<.036 and -.021<knot_offset[2]<.008 and grade['tilt_deg']<10.)
            self.supported_ticks=getattr(self,'supported_ticks',0)+1 if surrounded else 0
            ready=self.supported_ticks>=5
            entry=(name in ('follow the swinging cradle','offer weight to the supporting knot') and gap is not None
                   and gap<165. and np.linalg.norm(knot_offset[:2])<.030
                   and -.034<knot_offset[2]<.008 and grade['tilt_deg']<12.)
            self.over_web_ticks=getattr(self,'over_web_ticks',0)+1 if entry else 0
            if self.over_web_ticks>=2:
                self.log[-1]['weight_handoff']=dict(step=self.rig.tick,knot=knot_offset,gap=gap)
                self.idx=next(i for i,stage in enumerate(self.stages) if stage[0]=='centre the held base over the knot')
                self.elapsed=0;self.over_web_ticks=0
            elif name=='offer weight to the supporting knot' and self.elapsed==n and not ready:
                # Remain in load transfer if there is still no supporting web.
                # Lower the held pot slowly to close a measured air gap.
                if np.linalg.norm(knot_offset[:2]) < .040 and knot_offset[2] < -.025:
                    self.base_bias[2] = max(-.030, self.base_bias[2]-.0007)
                self.elapsed-=1
            elif name=='withdraw the right fingers from the lip' and self.elapsed>=39 and grade.get('seated') and max(grade.get('right_pad_pot_normal_n',[100.]))<.15:
                # The grasp has unloaded. Turning away here leaves joint margin for the outside corridor.
                self.log[-1]['depart']=dict(tick=self.rig.tick,steps=self.elapsed)
                self.idx+=1
                self.elapsed=0
            elif self.elapsed==n or ready:
                if ready:self.log[-1]['released_over_web']=dict(step=self.rig.tick,knot=knot_offset,gap=gap)
                self.idx+=1;self.elapsed=0
        return ActionChunk(actions=[Action(data=self.goal.copy(),meta={})],control_hz=s.CONTROL_HZ)

def make_scripted(variant=None):
    return Reference(ACTIVE,variant)
