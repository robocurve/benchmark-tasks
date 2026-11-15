"""Offline collision screening of a fixed joint plan, not native contact proof.

The desired joint targets are placed using the URDF. A held remover is assumed
to follow its proposed grasp; native slip, deflection and contact response are
not predicted. Positive convex intersection identifies a planning problem;
absence at sampled poses does not establish continuous-time clearance.
"""
import argparse
import hashlib
import itertools
import json
from pathlib import Path
import numpy as np
from scipy.optimize import linprog
from scipy.spatial import ConvexHull
from motion_plan import load, TOOL_TCP, GRIP
from full_scene import scene_layout,support_parts,TABLE_TOP
from plan_reach import TOOL_TCP,GRIP


SIGNS=np.asarray(list(itertools.product((-1.,1.),repeat=3)))


def convex(name,points):
    points=np.asarray(points)
    h=ConvexHull(points)
    planes=np.unique(np.round(h.equations,12),axis=0)
    return dict(name=name,vertices=points,planes=planes,low=points.min(0),high=points.max(0))


def box(name,center,size,rotation=None):
    rotation=np.eye(3) if rotation is None else rotation
    return convex(name,(SIGNS*np.asarray(size)/2)@rotation.T+center)


def overlap(a,b,margin=.0002):
    if np.any(np.minimum(a['high'],b['high'])-np.maximum(a['low'],b['low'])<2*margin):return None
    # Largest ball inside the intersection: both plane sets have unit normals.
    planes=np.vstack((a['planes'],b['planes']))
    result=linprog([0.,0.,0.,-1.],A_ub=np.c_[planes[:,:3],np.ones(len(planes))],
                   b_ub=-planes[:,3],bounds=[(None,None)]*3+[(0.,None)],method='highs')
    if result.success and result.x[3]>margin:return float(result.x[3])
    return None


def candidate_pairs(a,b,margin):
    """Batch the same positive-overlap AABB predicate before convex solving."""
    low_a=np.asarray([h['low'] for h in a]);high_a=np.asarray([h['high'] for h in a])
    low_b=np.asarray([h['low'] for h in b]);high_b=np.asarray([h['high'] for h in b])
    return np.argwhere(np.all(np.minimum(high_a[:,None,:],high_b[None,:,:])-np.maximum(low_a[:,None,:],low_b[None,:,:])>=2*margin,axis=2))


def scene_props(reset_profile):
    layout=scene_layout(reset_profile)
    return [box('table',(.3,0.,TABLE_TOP-.02),(1.,1.,.04)),
            *[box('support/'+name,center,size) for name,center,size in support_parts(layout)]]


def tool_props(center,R):
    props=[box('remover/grip',center+R@np.array((0.,-.043,.005)),(.018,.080,.016),R),
           box('remover/heel',center+R@np.array((0.,-.004,0.)),(.015,.006,.006),R)]
    profile=np.array(((-.004,-.003),(.025,0.),(.025,.00005),(-.004,.003)))
    for side in (-1,1):
        vertices=np.asarray([(side*.0046+offset,y,z) for offset in (-.0012,.0012) for y,z in profile])
        props.append(convex('remover/fork'+str(side),vertices@R.T+center))
    return props


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('plan',type=Path);p.add_argument('--out',type=Path,required=True)
    p.add_argument('--stride',type=int,default=1);p.add_argument('--margin',type=float,default=.0002)
    args=p.parse_args();plan,actions=load(args.plan)
    from yam_arena import ik,station,YAM_URDF
    from yam_arena.analysis import link_hulls,fk_link_poses_world
    from yam_arena.constants import ARM_JOINTS,FINGER_JOINTS,FINGER_OPEN
    hulls=link_hulls();props=scene_props(plan['reset_profile'])
    samples=[];grasp_assumed=False
    for index in range(0,len(actions),args.stride):
        action=actions[index];stage=plan['frames'][index]['stage'];robots={}
        if stage=='Lift the free remover':grasp_assumed=True
        for side,offset in (('left',0),('right',7)):
            q=dict(zip(ARM_JOINTS,action[offset:offset+6]));q.update({name:action[offset+6]*FINGER_OPEN for name in FINGER_JOINTS})
            links=fk_link_poses_world(q,station.BASES[side]);robot=[]
            for name,T in links.items():
                for hi,h in enumerate(hulls.get(name,[])):
                    verts=h.vertices@T[:3,:3].T+T[:3,3];normals=h.planes[:,:3]@T[:3,:3].T
                    robot.append(dict(name=f'{side}/{name}/{hi}',vertices=verts,planes=np.c_[normals,-h.planes[:,3]-normals@T[:3,3]],low=verts.min(0),high=verts.max(0)))
            robots[side]=robot
        if grasp_assumed:
            pos,R=ik.tcp_pose(action[7:13],station.BASES['right']);R=R@GRIP.T;center=pos-R@TOOL_TCP
        else:center=np.array((.28,-.20,TABLE_TOP+.003));R=np.eye(3)
        released=plan.get('released_tool',{})
        if index+1>=released.get('from_step',float('inf')):
            center=np.asarray(released['position_m']);R=np.asarray(released['rotation'])
        remover=tool_props(center,R);hits=[];expected=[]
        for side,robot in robots.items():
            all_props=props+remover
            for hi,pi in candidate_pairs(robot,all_props,args.margin):
                h,prop=robot[hi],all_props[pi]
                depth=overlap(h,prop,args.margin)
                if depth is None:continue
                record=dict(body=h['name'],prop=prop['name'],intersection_ball_radius_m=depth)
                intentional=('/tip_' in h['name']) and (side=='right' and prop['name']=='remover/grip')
                (expected if intentional else hits).append(record)
        for ai,bi in candidate_pairs(robots['left'],robots['right'],args.margin):
            a,b=robots['left'][ai],robots['right'][bi];depth=overlap(a,b,args.margin)
            if depth is not None:hits.append(dict(body=a['name'],prop=b['name'],intersection_ball_radius_m=depth))
        # Tool/support clearance is essential even when hand reach is good.
        for hi,pi in candidate_pairs(remover,props,args.margin):
            h,prop=remover[hi],props[pi]
            if prop['name']=='table' and (not grasp_assumed or index+1>=released.get('from_step',float('inf'))):continue
            depth=overlap(h,prop,args.margin)
            if depth is None:continue
            record=dict(body=h['name'],prop=prop['name'],intersection_ball_radius_m=depth)
            intended=h['name']=='remover/heel' and prop['name'].startswith('support/')
            (expected if intended else hits).append(record)
        samples.append(dict(step=index+1,stage=stage,unintended_intersections=hits,intended_contact_targets=expected))
    report=dict(kind='offline_commanded_pose_collision_screen',plan_sha256=hashlib.sha256(args.plan.read_bytes()).hexdigest(),
        urdf_sha256=hashlib.sha256(Path(YAM_URDF).read_bytes()).hexdigest(),margin_m=args.margin,stride=args.stride,
        source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),samples=samples,score=None,
        limitations=['Desired joints and an assumed fixed remover grasp, not measured native poses','Flexible paper and wire contacts are not represented by this gross clearance screen',
                     'Inter-arm hulls checked; no within-arm self-collision or continuous-time audit',
                     'Recorded URDF and collider sources must match the pinned native runtime'])
    args.out.parent.mkdir(parents=True,exist_ok=True);args.out.write_text(json.dumps(report,indent=2)+'\n')
    hits=[h for s in samples for h in s['unintended_intersections']]
    print(json.dumps(dict(samples=len(samples),unintended_intersections=len(hits),max_intersection_radius_m=max((h['intersection_ball_radius_m'] for h in hits),default=0.)),indent=2))


if __name__=='__main__':main()
