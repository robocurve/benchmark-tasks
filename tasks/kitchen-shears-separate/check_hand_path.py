"""Target-path hand interference screen against public reference convex parts."""
import json,os,pathlib,sys
import numpy as np
import task,geometry as g
from hand_geometry import load_hand_hulls,world_planes
from yam_arena.urdf import forward_kinematics
from yam_arena.demo import ArmPlan
from yam_arena.constants import LEFT_BASE_POS,RIGHT_BASE_POS
import argparse
parser=argparse.ArgumentParser()
parser.add_argument('--urdf',type=pathlib.Path)
args=parser.parse_args()
if args.urdf is None:
    from yam_arena import YAM_URDF
    args.urdf=pathlib.Path(YAM_URDF)

def main():
    urdf=args.urdf;hulls=load_hand_hulls(urdf)
    plans={s:ArmPlan(task.demo()[s],b) for s,b in [('left',LEFT_BASE_POS),('right',RIGHT_BASE_POS)]}
    relative={}
    def placed(tcp,r,finger):
        key=round(float(finger),8)
        if key not in relative:
            fk=forward_kinematics(urdf,{'joint7':key,'joint8':key})
            inv=np.linalg.inv(fk['gripper']);relative[key]={n:inv@fk[n] for n in ('gripper','tip_left','tip_right')}
        hand=np.eye(4);hand[:3,:3]=r;hand[:3,3]=tcp-r[:,2]*.134;out=[]
        for h in hulls:
            tf=hand@relative[key][h.link];v=h.vertices@tf[:3,:3].T+tf[:3,3]
            out.append((h.source,v,world_planes(h.planes,tf[:3,:3],tf[:3,3]),v.min(0),v.max(0)))
        return out
    candidates=[];edge_hits=[];below=[]
    for t in np.arange(0,plans['left'].duration+.001,.05):
        targets={s:plans[s].target(float(t)) for s in plans}
        hand={s:placed(*targets[s][:3]) for s in plans}
        lo=[np.array([h[3] for h in hand[s]]) for s in ('left','right')]
        hi=[np.array([h[4] for h in hand[s]]) for s in ('left','right')]
        overlap=np.all(np.minimum(hi[0][:,None],hi[1][None,:])-np.maximum(lo[0][:,None],lo[1][None,:])>0,axis=-1)
        for i,j in np.argwhere(overlap):
            a,b=hand['left'][i],hand['right'][j]
            # A separating face plane proves clearance. Lack of one is a
            # conservative candidate, not an asserted actual intersection.
            sep=(np.any(np.min(b[1]@a[2][:,:3].T+a[2][:,3],axis=0)>1e-6) or
                 np.any(np.min(a[1]@b[2][:,:3].T+b[2][:,3],axis=0)>1e-6))
            if not sep:candidates.append({'t':float(t),'left':a[0],'right':b[0]})
        if any(v[:,2].min()<task.TOP-.0001 for s in hand for name,v,planes,low,high in hand[s]):below.append(float(t))
    result={'kind':'hand targets only, not actual grasp or simulation',
            'samples':int(round(plans['left'].duration/.05))+1,'hand_hand_candidates':len(candidates),'edge_hits':len(edge_hits),
            'below_table_samples':len(below),'examples':candidates[:5]+edge_hits[:5]}
    print(json.dumps(result,indent=2));raise SystemExit(bool(candidates or edge_hits or below))

if __name__=='__main__':main()
