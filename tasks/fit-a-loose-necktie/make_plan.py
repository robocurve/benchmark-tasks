"""Fixed robot actions; the material model never reads these stages."""
import argparse,json
from pathlib import Path
import numpy as np
from material import Tie

def yaw(degrees):
    a=np.radians(degrees);return np.array([[np.cos(a),0,np.sin(a)],[-np.sin(a),0,np.cos(a)],[0,-1,0]])
def build():
    from yam_arena import ik,station
    from yam_arena.constants import READY,SHOULDER,WRIST,GRIPPER,FINGER_OPEN
    t=Tie();left=t.loose.points[30].copy();right=t.center+[-(.018+.06*t.slack),0,0]
    lr,rr=yaw(180),yaw(0);lp=left+.008*lr[:,2];rp=right+.008*rr[:,2]
    thread_finish=np.array([.319,.025,.832])+.008*lr[:,2]
    lf=thread_finish+[0,0,-.050]
    compact=Tie(drawn=.05,threaded=True,wide_base=.25,wrap_base=.32);narrow=compact.narrow_root+compact.narrow_direction*(compact.narrow_length-.025)
    np0=narrow+.008*lr[:,2]
    fitted=Tie(z=t.z+.08,y=0.,drawn=.05,threaded=True,wide_base=.25,wrap_base=.32)
    nfinal=fitted.narrow_root+np.array([0,0,-(fitted.narrow_length-.025)])+.008*lr[:,2]
    rf=rp+[0,-.025,.080]
    a=np.r_[READY,1.,READY,1.];frames=[];audit=[]
    steps=[(3.,'approach the wide blade',{'left':(lp-.07*lr[:,2],lr,.8)},'joint'),
        (2.,'enclose the wide blade',{'left':(lp,lr,.8)},'cartesian'),
        (1.,'grasp the wide blade',{'left':(lp,lr,.1)},'cartesian'),
        (3.,'approach the loose knot',{'right':(rp-.08*rr[:,2],rr,.8)},'joint'),
        (2.,'enclose the outer wrap',{'right':(rp,rr,.8)},'cartesian'),
        (1.,'hold the knot',{'right':(rp,rr,.1)},'cartesian'),
        (5.,'lift the free wide blade outside the loop',{'left':(np.array([.249,.025,1.20])+.008*lr[:,2],lr,.1)},'cartesian'),
        (3.,'bring the free tip over the front loop',{'left':(np.array([.319,.025,1.20])+.008*lr[:,2],lr,.1)},'cartesian'),
        (4.,'feed the wide tip down through the front loop',{'left':(np.array([.319,.025,1.03])+.008*lr[:,2],lr,.1)},'cartesian'),
        (5.,'draw the wide blade through the loop',{'left':(thread_finish,lr,.1)},'cartesian'),
        (4.,'snug the outer wrap with wide blade',{'left':(lf,lr,.1)},'cartesian'),
        (1.,'release wide blade',{'left':(lf,lr,.8)},'cartesian'),
        (2.,'clear the wide blade',{'left':(lf-.08*lr[:,2],lr,.8)},'cartesian'),
        (2.,'approach the exposed narrow blade',{'left':(np0-.06*lr[:,2],lr,.8)},'cartesian'),
        (1.5,'enclose the narrow blade',{'left':(np0,lr,.8)},'cartesian'),
        (1.,'grasp narrow blade',{'left':(np0,lr,.1)},'cartesian'),
        (6.,'hold narrow blade and slide knot to collar',{'left':(nfinal,lr,.1),'right':(rf,rr,.1)},'cartesian'),
        (1.,'release fitted knot',{'right':(rf+[-.035,0,0],rr,.8)},'cartesian'),
        (2.,'withdraw from knot',{'right':(rf+[-.035,-.10,0],rr,.8)},'cartesian'),
        (1.,'release narrow blade behind wide blade',{'left':(nfinal,lr,.8)},'cartesian'),
        (2.,'withdraw from blade',{'left':(nfinal-.10*lr[:,2],lr,.8)},'cartesian'),
        (5.,'hands clear for judging',{},'cartesian')]
    for sec,label,targets,mode in steps:
        b=a.copy();poses={s:ik.tcp_pose(a[o:o+6],station.BASES[s]) for s,o in [('left',0),('right',7)]};goals={};errors=[0,0]
        if mode=='joint':
            from scipy.optimize import least_squares
            for side,(p,R,g) in targets.items():
                o=0 if side=='left' else 7
                def residual(q):
                    pp,rrr=ik.tcp_pose(q,station.BASES[side]);return np.r_[pp-p,.3*ik._rot_error(R,rrr)]
                sols=[least_squares(residual,seed,bounds=(ik.LOWER,ik.UPPER),max_nfev=200) for seed in [a[o:o+6],(ik.LOWER+ik.UPPER)/2]]
                sol=min(sols,key=lambda v:np.linalg.norm(v.fun));assert np.linalg.norm(sol.fun)<.01,(label,sol.fun);goals[side]=sol.x
        for k in range(1,int(sec*20)+1):
            u=ik.smoothstep(k/(sec*20))
            for side,(p,R,g) in targets.items():
                o=0 if side=='left' else 7;p0,R0=poses[side]
                if side in goals:a[o:o+6]=b[o:o+6]+u*(goals[side]-b[o:o+6])
                else:
                    q,pe,re=ik.solve(p0+u*(p-p0),ik.matrix_to_quat_xyzw(ik.slerp_matrix(R0,R,u)),a[o:o+6],base_pos=station.BASES[side],iters=100);a[o:o+6]=q;errors=[max(errors[0],float(pe)),max(errors[1],float(re))]
                a[o+6]=b[o+6]+u*(g-b[o+6])
            frames.append(dict(action14=a.tolist(),stage=label))
        audit.append(dict(stage=label,position_error_m=errors[0],orientation_error_rad=errors[1]))
    limit=np.tile([SHOULDER['velocity']/20]*3+[WRIST['velocity']/20]*3+[GRIPPER['velocity']/abs(FINGER_OPEN)/20],2)
    assert np.max(abs(np.diff(np.vstack([np.r_[READY,1.,READY,1.]]+[f['action14'] for f in frames]),axis=0))/limit)<1.001
    assert max(x['position_error_m'] for x in audit)<.01,audit
    return dict(control_hz=20,frames=frames,kinematic_audit=audit)
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);a=p.parse_args();d=build();a.out.write_text(json.dumps(d,indent=2)+'\n');print(json.dumps({'steps':len(d['frames']),'audit':d['kinematic_audit']},indent=2))
