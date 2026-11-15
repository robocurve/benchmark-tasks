"""Fixed ordinary joint plan for remover pickup and a proposed lever motion.

No live object poses or material/scoring state enter these commands. Native
paper contact, grip and extraction are unproven; this is not a successful demo.
"""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from full_scene import TABLE_TOP,scene_layout,RESET_PROFILES
from plan_reach import TOOL_TCP,GRIP,insertion_pitch,heel_supported_pose,tool_frame


class Compiler:
    def __init__(self):
        from yam_arena import ik,station
        from yam_arena.constants import READY
        self.ik,self.station=ik,station
        self.q={side:np.asarray(READY,float).copy() for side in ('left','right')}
        self.opening={side:1. for side in self.q};self.frames=[];self.audit=[]

    def segment(self,label,seconds,targets=None,openings=None,joint=False,tool_path=None):
        targets=targets or {};openings=openings or {};ik=self.ik
        start={side:ik.tcp_pose(q,self.station.BASES[side]) for side,q in self.q.items()}
        q0={side:q.copy() for side,q in self.q.items()};opening0=self.opening.copy();goals={}
        maximum_position=maximum_rotation=0.
        if joint:
            for side,(p,R) in targets.items():
                q,pe,re=ik.solve(p,ik.matrix_to_quat_xyzw(R),self.q[side],self.station.BASES[side],iters=400,damping=.008,tol=1e-7)
                if pe>.0001 or re>.001:raise ValueError(f'Unreachable {label}/{side}: {pe}m,{re}rad')
                goals[side]=q
        count=max(1,round(seconds*20))
        for step in range(1,count+1):
            u=ik.smoothstep(step/count)
            for side in self.q:
                if side in goals:self.q[side]=q0[side]+u*(goals[side]-q0[side])
                elif side in targets or (side=='right' and tool_path is not None):
                    if side=='right' and tool_path is not None:
                        center,R=tool_path(u);p=center+R@TOOL_TCP;R=R@GRIP
                    else:
                        p0,R0=start[side];p1,R1=targets[side]
                        p=p0+u*(np.asarray(p1)-p0);R=ik.slerp_matrix(R0,R1,u)
                    q,pe,re=ik.solve(p,ik.matrix_to_quat_xyzw(R),self.q[side],self.station.BASES[side],iters=120,damping=.008,tol=1e-7)
                    maximum_position=max(maximum_position,pe);maximum_rotation=max(maximum_rotation,re);self.q[side]=q
                    if pe>.0001 or re>.001:raise ValueError(f'Unreachable path {label}/{side}/{step}: {pe}m,{re}rad')
                self.opening[side]=opening0[side]+u*(openings.get(side,opening0[side])-opening0[side])
            self.frames.append(dict(stage=label,action14=np.r_[self.q['left'],self.opening['left'],self.q['right'],self.opening['right']].tolist()))
        self.audit.append(dict(stage=label,seconds=count/20,interpolation='joint' if joint else 'Cartesian/tool',maximum_position_error_m=maximum_position,maximum_rotation_error_rad=maximum_rotation))


def compile_plan(stage='pickup',reset_profile='pocket-support-candidate'):
    from yam_arena import YAM_URDF
    from yam_arena.constants import READY,SHOULDER,WRIST,GRIPPER,FINGER_OPEN
    layout=scene_layout(reset_profile);paper_top=layout['paper_centres_z'][1]+.00005
    # Brace the far supported corner to keep the wrists on opposite sides.
    left_R=np.diag((-1.,1.,-1.));left=np.array((.405,.050,paper_top+.0129))
    initial=np.array((.28,-.20,TABLE_TOP+.003));tcp=lambda root,R:(root+R@TOOL_TCP,R@GRIP)
    c=Compiler();c.segment('Approach remover and papers',1.5,{'left':(left+[0.,0.,.06],left_R),'right':tcp(initial+[0.,0.,.08],np.eye(3))},openings={'left':0.},joint=True)
    c.segment('Lower to handle and hold papers',.8,{'left':(left,left_R),'right':tcp(initial,np.eye(3))})
    c.segment('Grip the remover',.6,openings={'right':.12})
    c.segment('Lift the free remover',.8,{'right':tcp(initial+[0.,0.,.06],np.eye(3))})
    c.segment('Hold pickup for inspection',.5)
    if stage=='lever':
        pitch=insertion_pitch(reset_profile);R=tool_frame(pitch);root=heel_supported_pose(pitch,reset_profile)
        c.segment('Orient above the crown',1.,{'right':tcp(root+[0.,0.,.06],R)},joint=True)
        c.segment('Approach supported side of crown',.7,{'right':tcp(root+[.012,0.,.006],R)})
        c.segment('Lower the heel to its support',.6,{'right':tcp(root+[.012,0.,0.],R)})
        c.segment('Slide the fork below the crown',.8,{'right':tcp(root,R)})
        c.segment('Lever against the support',1.,tool_path=lambda u:(heel_supported_pose(pitch*(1-u),reset_profile),tool_frame(pitch*(1-u))))
        root=heel_supported_pose(0.,reset_profile);R=tool_frame(0.)
        c.segment('Lift the crown and straighten legs',1.,{'right':tcp(root+[0.,0.,.012],R)})
        c.segment('Carry the remover clear',.8,{'right':tcp(root+[0.,-.06,.07],R)})
        c.segment('Release pressure on the papers',.7,{'left':(left+[0.,0.,.06],left_R)})
        c.segment('Inspect extraction',1.)
    initial_action=np.r_[READY,1.,READY,1.];actions=np.asarray([initial_action]+[r['action14'] for r in c.frames])
    limits=np.tile(np.array([SHOULDER['velocity']/20]*3+[WRIST['velocity']/20]*3+[GRIPPER['velocity']/abs(FINGER_OPEN)/20]),2)
    fraction=float(np.max(np.abs(np.diff(actions,axis=0))/limits))
    if fraction>1.001:raise ValueError(f'Ordinary actuator speed limit exceeded: {fraction}')
    return dict(schema='staple-fixed-motion-v1',kind='diagnostic_joint_commands_not_task_demo',control_hz=20,requested_stage=stage,
        reset_profile=reset_profile,mesh_profile='clearance-candidate' if reset_profile=='pocket-support-candidate' else 'coarse',robot_success=False,task_score=None,
        source_sha256={name:hashlib.sha256(Path(__file__).with_name(name).read_bytes()).hexdigest() for name in ('motion_plan.py','plan_reach.py','full_scene.py')},
        urdf_sha256=hashlib.sha256(Path(YAM_URDF).read_bytes()).hexdigest(),action_count=len(c.frames),seconds=len(c.frames)/20,
        max_speed_fraction=fraction,insertion_pitch_deg=float(np.degrees(insertion_pitch(reset_profile))),frames=c.frames,kinematic_audit=c.audit,
        limitations=['No native grip, paper contact or extraction proof','Candidate paper model still under contact diagnosis','Complete staple release and final10mm clearance not yet demonstrated'])


def load(path):
    plan=json.loads(Path(path).read_text())
    if plan.get('schema')!='staple-fixed-motion-v1' or plan.get('control_hz')!=20:raise ValueError('Unexpected staple plan')
    actions=np.asarray([r['action14'] for r in plan['frames']],float)
    if actions.ndim!=2 or actions.shape[1]!=14 or not 1<=len(actions)<=2400 or not np.isfinite(actions).all():raise ValueError('Invalid bounded14D plan')
    if np.any(actions[:,[6,13]]<0) or np.any(actions[:,[6,13]]>1):raise ValueError('Invalid opening')
    return plan,actions


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--stage',choices=('pickup','lever'),default='pickup');p.add_argument('--out',type=Path,required=True)
    p.add_argument('--reset-profile',choices=RESET_PROFILES,default='pocket-support-candidate')
    a=p.parse_args();plan=compile_plan(a.stage,a.reset_profile);a.out.parent.mkdir(parents=True,exist_ok=True);a.out.write_text(json.dumps(plan,indent=2)+'\n')
    print(json.dumps({key:plan[key] for key in ('requested_stage','action_count','seconds','max_speed_fraction','insertion_pitch_deg')}))
