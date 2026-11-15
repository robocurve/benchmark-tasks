"""Cheap offline rejection check for fixed robot trajectories.

Tests actual YAM collision-hull vertices against static fixture solids at every
commanded pose. Any negative distance is a concrete penetration. A clean result
is not a complete continuous collision test, nor evidence of cloth contact or
successful robot control. Native physics and video remain necessary.
"""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np



def segment_hull_interval(a,b,planes,shrink=.0002):
    """Clip a finite fixture axis against every convex face, not only vertices.

    A thin line can cross a hand while all hull vertices remain outside it.
    This rejects an actual axis penetration; surface proximity alone does not
    count and this does not claim complete cylinder/hull separation.
    """
    n=planes[:,:3];signed=n@a-planes[:,3]+shrink;rate=n@(b-a)
    parallel=abs(rate)<1e-12
    if np.any(signed[parallel]>0.):return None
    lower,upper=0.,1.
    positive=rate>1e-12;negative=rate< -1e-12
    if np.any(positive):upper=min(upper,float(np.min(-signed[positive]/rate[positive])))
    if np.any(negative):lower=max(lower,float(np.max(-signed[negative]/rate[negative])))
    return (lower,upper) if lower<=upper else None


def audit(plan):
    from yam_arena import station
    from yam_arena.analysis import link_hulls,fk_link_poses_world,box_signed_distance
    from yam_arena.constants import ARM_JOINTS,FINGER_OPEN
    from scene import support_geometry,rack_geometry,TABLE
    local_hulls=link_hulls()
    vertices={k:np.concatenate([h.vertices for h in hs])
              for k,hs in local_hulls.items() if k!='base'}
    boxes={**support_geometry(),**rack_geometry()}
    if not plan.get('line_pegs_required'):
        from peg_rests import geometry
        boxes.update(geometry())
    boxes['Table']=dict(kind='box',center=(.36,0.,TABLE-.03),size=(1.1,1.3,.06))
    violations=[]
    closest={}
    inter_arm_min=float('inf')
    for step,f in enumerate(plan['frames']):
        arm_hulls=[]
        for arm,side in enumerate(('left','right')):
            action=f['action14'][7*arm:7*(arm+1)]
            joints=dict(zip(ARM_JOINTS,action[:6]))
            joints.update(joint7=action[6]*FINGER_OPEN,joint8=action[6]*FINGER_OPEN)
            poses=fk_link_poses_world(joints,station.BASES[side])
            hull_group=[]
            for link,hulls in local_hulls.items():
                r=poses[link][:3,:3];p=poses[link][:3,3]
                for h in hulls:
                    w=h.vertices@r.T+p
                    hull_group.append((link,h,r,p,w,w.min(0),w.max(0)))
                    for fixture_name,fixture in boxes.items():
                        if fixture['kind']=='box':continue
                        center=np.asarray(fixture['center']);delta=np.zeros(3)
                        delta['XYZ'.index(fixture.get('axis','Z'))]=fixture['height']/2
                        a,b=center-delta,center+delta
                        if np.any(np.maximum(a,b)<w.min(0)) or np.any(np.minimum(a,b)>w.max(0)):continue
                        interval=segment_hull_interval((a-p)@r,(b-p)@r,h.planes)
                        if interval is not None:
                            violations.append(dict(step=step,stage=f['stage'],side=side,
                                link=link,box=fixture_name,penetration_m=fixture['radius'],
                                test='finite cylinder axis crosses convex hull interior',axis_interval=list(interval)))
            arm_hulls.append(hull_group)
            for link,v in vertices.items():
                w=v@poses[link][:3,:3].T+poses[link][:3,3]
                for name,box in boxes.items():
                    if box['kind']=='box':
                        distances=box_signed_distance(w,np.array(box['center']),np.eye(3),np.array(box['size']))
                    else:
                        v=w-np.array(box['center']);axis='XYZ'.index(box.get('axis','Z'))
                        radial=np.linalg.norm(np.delete(v,axis,axis=1),axis=1)-box['radius']
                        axial=abs(v[:,axis])-box['height']/2
                        q=np.column_stack((radial,axial))
                        distances=np.linalg.norm(np.maximum(q,0.),axis=1)+np.minimum(np.maximum(q[:,0],q[:,1]),0.)
                    distance=float(distances.min())
                    key=f'{side}:{link}:{name}'
                    closest[key]=min(closest.get(key,float('inf')),distance)
                    if distance<-.0005:
                        violations.append(dict(step=step,stage=f['stage'],side=side,
                            link=link,box=name,penetration_m=-distance))
        # Vectorized broad phase preserves the exact pair predicate while
        # avoiding a Python loop over every disjoint convex-hull pair.
        low_a=np.asarray([a[5] for a in arm_hulls[0]]);high_a=np.asarray([a[6] for a in arm_hulls[0]])
        low_b=np.asarray([b[5] for b in arm_hulls[1]]);high_b=np.asarray([b[6] for b in arm_hulls[1]])
        pairs=np.argwhere(np.all(np.maximum(low_b[None,:,:]-high_a[:,None,:],low_a[:,None,:]-high_b[None,:,:])<=0.,axis=2))
        for ai,bi in pairs:
                a,b=arm_hulls[0][ai],arm_hulls[1][bi]
                def inside(source,target):
                    local=(source[4]-target[3])@target[2]
                    planes=target[1].planes
                    return float((local@planes[:,:3].T-planes[:,3]).max(1).min())
                distance=min(inside(a,b),inside(b,a))
                inter_arm_min=min(inter_arm_min,distance)
                if distance<-.0005:
                    violations.append(dict(step=step,stage=f['stage'],side='bimanual',
                        link_pair=[a[0],b[0]],box=None,penetration_m=-distance))
    return dict(kind='offline_kinematic_collision_rejection_check',checked_frames=len(plan['frames']),
                min_inter_arm_signed_vertex_distance_m=None if np.isinf(inter_arm_min) else inter_arm_min,
                check='collision-hull vertices against fixtures/opposite-arm hulls (0.5 mm); finite cylinder axes clipped against whole hulls (0.2 mm)',
                passed_rejection_check=not violations,violations=violations,
                minimum_signed_vertex_distances_m=closest,
                limitation='Does not certify continuous collisions, robot tracking, cloth physics or grasp success')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('plan',type=Path)
    p.add_argument('--out',type=Path,required=True)
    args=p.parse_args()
    from fixed_motion import load
    plan,_=load(args.plan)
    result=audit(plan)
    result['motion_sha256']=hashlib.sha256(args.plan.read_bytes()).hexdigest()
    result['peg_rest_source_sha256']=hashlib.sha256(Path(__file__).with_name('peg_rests.py').read_bytes()).hexdigest()
    result['fixture_source_sha256']=hashlib.sha256(Path(__file__).with_name('scene.py').read_bytes()).hexdigest()
    args.out.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(dict(passed=result['passed_rejection_check'],penetrating_samples=len(result['violations']),
                         max_penetration_m=max((v['penetration_m'] for v in result['violations']),default=0.)),indent=2))
    if result['violations']:
        raise SystemExit(1)


if __name__=='__main__':
    main()
