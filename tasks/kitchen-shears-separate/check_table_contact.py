"""Audit diagnostic target hands, metal, table, object path and scorer guard.

Only commanded hand hulls and nominal object poses: not actual arm tracking.
"""
import argparse,json,math
from pathlib import Path
import numpy as np
import task,geometry as g,collision as c
from table_plan import object_plan,grasp_target
from hand_geometry import load_hand_hulls,convex_planes,world_planes,point_near_hull
from yam_arena.urdf import forward_kinematics
from yam_arena.demo import ArmPlan
from yam_arena.constants import LEFT_BASE_POS,RIGHT_BASE_POS
from yam_arena.ik import smoothstep
from scoring import Observation,Scorer

def overlap(a,b):
    return not (np.any(np.min(b[0]@a[1][:,:3].T+a[1][:,3],axis=0)>1e-7) or
                np.any(np.min(a[0]@b[1][:,:3].T+b[1][:,3],axis=0)>1e-7))
def pairs(aa,bb):
    alo=np.array([a[2] for a in aa]);ahi=np.array([a[3] for a in aa])
    blo=np.array([a[2] for a in bb]);bhi=np.array([a[3] for a in bb])
    candidates=np.all(np.minimum(ahi[:,None],bhi[None,:])-np.maximum(alo[:,None],blo[None,:])>1e-7,axis=-1)
    return [(int(i),int(j)) for i,j in np.argwhere(candidates) if overlap(aa[i],bb[j])]
def pack(v,planes):return v,planes,v.min(0),v.max(0)

def objects(t):
    phases=object_plan(task.TOP);prev=phases[0];t=max(0.,t-5.)
    for ph in phases:
        if t<ph['duration']:
            n=max(1,int(np.ceil(np.max(np.abs(ph['angles']-prev['angles']))/np.radians(1))))
            progress=t/ph['duration']*n;k=min(n-1,int(progress))
            f=(k+smoothstep(progress-k))/n
            return prev['positions']*(1-f)+ph['positions']*f,prev['angles']*(1-f)+ph['angles']*f
        t-=ph['duration'];prev=ph
    return prev['positions'],prev['angles']

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--urdf',type=Path,required=True);args=parser.parse_args()
    hulls=load_hand_hulls(args.urdf);relative={}
    plans={s:ArmPlan(task.demo()[s],b) for s,b in [('left',LEFT_BASE_POS),('right',RIGHT_BASE_POS)]}
    local=[];local_all=[]
    for i in range(2):
        local.append([convex_planes(s.vertices[s.triangles]) for s in g.half(i) if s.kind!='handle'])
        local_all.append([convex_planes(s.vertices[s.triangles]) for s in g.half(i)])
    def hands(tcp,r,finger):
        key=round(float(finger),8)
        if key not in relative:
            fk=forward_kinematics(args.urdf,{'joint7':key,'joint8':key})
            inv=np.linalg.inv(fk['gripper']);relative[key]={h.link:inv@fk[h.link] for h in hulls}
        root=tcp-r[:,2]*.134;out=[]
        for h in hulls:
            rel=relative[key][h.link];rr=r@rel[:3,:3];pp=r@rel[:3,3]+root
            v=h.vertices@rr.T+pp;out.append(pack(v,world_planes(h.planes,rr,pp)))
        return out
    capture=[]
    for side,i in [('right',0),('left',1)]:
        tcp,r,_,_=plans[side].target(5.)
        closed=hands(tcp,r,0.);p,angles=objects(5.)
        witnesses=[]
        for s in g.handle_ring(i):
            # Exact polygon side faces, including both inner and outer walls.
            for a,b in [(s.vertices[0],s.vertices[1]),(s.vertices[2],s.vertices[3])]:
                for f in np.linspace(0,1,9):
                    for z in [.001,.003,.006,.009,.011]:
                        v=a*(1-f)+b*f;v[2]=z;witnesses.append(v)
        pts=np.array(witnesses)@g.rz(angles[i]).T+p[i];counts={}
        for name in ['tip_left','tip_right']:
            hit=np.zeros(len(pts),bool)
            for h,placed in zip(hulls,closed):
                if h.link==name:hit|=point_near_hull(pts,placed[1])
            counts[name]=int(hit.sum())
        capture.append({'side':side,'closed_pad_surface_witnesses':counts,'pass':all(counts.values())})
    errors=[];min_table=1.;scorer=Scorer(task.TOP);total=0;max_pivot_error=0.
    local_grasps=[grasp_target(i,np.zeros(3),0.) for i in range(2)]
    for t in np.arange(0,plans['left'].duration+.001,.05):
        total+=1;p,angles=objects(t);rots=np.array([g.rz(a) for a in angles])
        targets={s:plans[s].target(float(t)) for s in plans}
        if t>=6.5:
            # Reconstruct from the actual interpolated target, assuming fixed
            # purchase; catches small arc/chord pivot errors without simulating.
            for side,i in [('right',0),('left',1)]:
                tcp,hand,_,_=targets[side]
                local_tcp,local_hand=local_grasps[i]
                rots[i]=hand@local_hand.T;p[i]=tcp-rots[i]@local_tcp
            if t<=11.:max_pivot_error=max(max_pivot_error,float(np.linalg.norm(p[0,:2]-p[1,:2])))
        hh={};tips=[];centers=[];radii=[]
        for side in plans:
            tcp,r,f,label=targets[side]
            # 8mm loop wall prevents the commanded zero gap in a real pinch.
            f=-max(abs(float(f)),.004) if t>=5. else f
            hh[side]=hands(tcp,r,f)
            mid=tcp+r[:,2]*.013;root=tcp-r[:,2]*.134;half=abs(float(f))
            tips.extend([mid+half*r[:,0],mid-half*r[:,0]])
            n=math.ceil(.147/.010)+1
            for a in np.linspace(0,1,n):
                centers.append(root*(1-a)+mid*a)
                radii.append((1-a)*.040+a*(half+.008)+.147/(2*(n-1)))
        bottom=min(h[2][2] for side in hh for h in hh[side]);min_table=min(min_table,bottom-task.TOP)
        if bottom<task.TOP-.0001:errors.append({'t':float(t),'kind':'below-table'})
        inter=pairs(hh['left'],hh['right'])
        if inter:errors.append({'t':float(t),'kind':'hand-hand','pairs':inter[:3]})
        metal=[]
        for i in range(2):
            metal.extend(pack(v@rots[i].T+p[i],world_planes(planes,rots[i],p[i])) for v,planes in local[i])
        for side in hh:
            inter=pairs(hh[side],metal)
            if inter:errors.append({'t':float(t),'kind':'hand-metal','arm':side,'pairs':inter[:3]})
        if 2.5<=t<=5.:
            all_scene=[]
            for i in range(2):
                all_scene.extend(pack(v@rots[i].T+p[i],world_planes(planes,rots[i],p[i])) for v,planes in local_all[i])
            for side in hh:
                inter=pairs(hh[side],all_scene)
                if inter:errors.append({'t':float(t),'kind':'open-hand-object','arm':side,'pairs':inter[:3]})
        inter=c.half_intersections(p,rots)
        if inter:errors.append({'t':float(t),'kind':'half-half','pairs':inter[:3]})
        scorer.update(Observation(float(t),np.vstack((p,[*g.TRAY_XY,task.TOP])),np.array([*rots,np.eye(3)]),
                       np.array(tips),np.array(centers),np.array(radii)))
    out={'kind':'nominal targets and object poses only, full arm collision/actual contact unverified',
         'samples':total,'minimum_table_clearance_mm':min_table*1000,'collision_samples':len(errors),
         'maximum_assembled_pivot_mismatch_mm':max_pivot_error*1000,
         'examples':errors[:10],'scorer_violations':sorted(scorer.violations),'capture':capture,
         'pass':not errors and not scorer.violations and all(x['pass'] for x in capture) and max_pivot_error<.0001}
    print(json.dumps(out,indent=2))
    (Path(__file__).parents[1]/'table-contact-check.json').write_text(json.dumps(out,indent=2)+'\n')
    raise SystemExit(0 if out['pass'] else 1)
if __name__=='__main__':main()
