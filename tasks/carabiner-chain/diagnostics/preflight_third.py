"""Third-link paths against a recorded released parent pose.

This is offline geometric planning, never an assembly demonstration.
"""
from pathlib import Path
import argparse,json,numpy as np
from yam_arena import ik,station
from yam_arena.constants import READY
from chain_task.geometry import *
from chain_task.motion import *
from collisions import *
ROOT=Path(__file__).resolve().parents[1]
ap=argparse.ArgumentParser();ap.add_argument('--state-log',type=Path,required=True);args=ap.parse_args()
d=json.loads([x for x in args.state_log.read_text().splitlines() if x.startswith('CHAIN_STATE ')][-1].split(' ',2)[2]);pp,pr=np.array(d['positions']['link0']),np.array(d['rotations']['link0'])
parent_tool=pr@np.diag([-1.,-1.,1.])@rx(np.deg2rad(-15));parent_tcp=pp+pr@[-.016,0,0]-parent_tool[:,2]*.004
normal_targets,pg,rg=parent_normalization(pp,pr,parent_tcp,parent_tool,80.)
rod=pg+rg@[-.002,-.030,0.];n=rg[:,0];yy=np.array([0,0,1.])-n*n[2];yy/=np.linalg.norm(yy);rd=np.column_stack([np.cross(yy,n),yy,n])
part=layout(17)['link2'];p,r=part['position'],part['rotation'];tcp,tool=grasp(p,r,1,'spine');off=r.T@(tcp-p);rel=tool.T@r
lt=pickup(p,r,1,'spine');lp=tcp+[0,0,.12];lt.append(Target(15,lp,tool,0.,'wait for parent stabilization'));center=np.array([.34,-.16,.94]);lt.extend([Target(4,center+rd@off,rd@rel.T,0.,'orient child'),Target(2,center+rd@off,rd@rel.T,0.,'settle child')])
for seconds,local,label in [(4,[.022,.020,.070],'normal approach'),(4,[.022,.020,0.],'align gate'),(2,[.022,.020,0.],'settle'),(4,[.007,.020,0.],'press'),(3,[.007,.024,0.],'round tip'),(3,[.002,.024,0.],'enter parent'),(3,[.002,.024,0.],'close')]:lt.append(Target(seconds,rod-rd@np.array(local)+rd@off,rd@rel.T,0.,label))
left=Plan(lt,'right');rt=[Target(3,parent_tcp-parent_tool[:,2]*.020,parent_tool,.18,'parent approach'),Target(3,parent_tcp,parent_tool,.18,'parent entry'),Target(2,parent_tcp,parent_tool,0.,'parent pinch'),Target(1,parent_tcp,parent_tool,0.,'settle parent grasp')]+normal_targets+[Target(600,normal_targets[-1].pos,normal_targets[-1].rot,0.,'counterhold')];right=Plan(rt,'left');roff=pr.T@(parent_tcp-pp);rrel=parent_tool.T@pr;errors=[];hits=[]
for t in np.arange(.1,left.duration+.001,.1):
 try:la,label=left.action7(t);ra=np.r_[READY,1.] if t<11 else right.action7(t-11)[0]
 except RuntimeError as exc:errors.append((float(t),str(exc)));break
 ls=robot(la[:6],max(la[6],.0045/.0939),'right');rs=robot(ra[:6],max(ra[6],.0045/.0939),'left');hp,hr=ik.tcp_pose(la[:6],station.BASES['right']);br=hr@rel;bp=hp-br@off
 if t<6.:bp,br=p,r
 if t<20.:parent_p,parent_r=pp,pr
 else:
  hp,hr=ik.tcp_pose(ra[:6],station.BASES['left']);parent_r=hr@rrel;parent_p=hp-parent_r@roff
 parent_obs=asset('parent',ROOT/'assets/frame0.usda',parent_p,parent_r)+asset('parent_gate',ROOT/'assets/gate.usda',parent_p+parent_r@GATE_OFFSET,parent_r)
 child_obs=asset('child',ROOT/'assets/frame2.usda',bp,br)+asset('child_gate',ROOT/'assets/gate.usda',bp+br@GATE_OFFSET,br);obs=static_scene(ROOT)+asset('upstream',ROOT/'assets/frame1.usda',np.array(d['positions']['link1']),np.array(d['rotations']['link1']))+asset('upstream_gate',ROOT/'assets/gate.usda',np.array(d['positions']['gate1']),np.array(d['rotations']['gate1']))+parent_obs+child_obs
 allowed_l={(tip,'child') for tip in ['tip_left','tip_right']} if t>=4.5 else set();allowed_r={(tip,'parent') for tip in ['tip_left','tip_right']} if t>=14. else set();lh,rh=against(ls,obs,allowed_l),against(rs,obs,allowed_r);inter=cross(ls,rs)
 if lh or rh or inter:hits.append((float(t),label,lh,rh,inter))
print(json.dumps(dict(planning_fixture_source=args.state_log.name,errors=errors,left_errors=[left.worst_position_error,left.worst_rotation_error],right_errors=[right.worst_position_error,right.worst_rotation_error],hits=hits,stabilized_parent_pose=pg.tolist()),indent=2))
