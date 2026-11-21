"""Fixed-layout RGB-only rehearsal: offline kinematics, no live object state."""
import math
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
import spec
DOWN=np.diag([-1.,1.,-1.])
FRONT=np.array([[0.,0.,1.],[0.,1.,0.],[-1.,0.,0.]])
RIGHT=Rotation.from_euler('z',180,degrees=True).as_matrix()@DOWN
CROSS=Rotation.from_euler('z',90,degrees=True).as_matrix()@DOWN
SIDE=Rotation.from_euler('x',75,degrees=True).as_matrix()@CROSS
OPPOSITE=Rotation.from_euler('z',180,degrees=True).as_matrix()@Rotation.from_euler('x',-90,degrees=True).as_matrix()@CROSS
HANDLE=Rotation.from_euler('y',15,degrees=True).as_matrix()@FRONT@Rotation.from_euler('z',180,degrees=True).as_matrix()

def plan(ik,constants,station,initial_only=False):
    left=(np.array([.16,.35,.95]),DOWN,.075)
    right=(np.array([.16,-.35,.95]),RIGHT,.075)
    rows=[]
    def row(t,l=None,r=None):
        nonlocal left,right
        if l is not None:left=(np.array(l[0]),l[1],l[2])
        if r is not None:right=(np.array(r[0]),r[1],r[2])
        rows.append((t,*left,*right))
    row(0)
    # Fixed public reset coordinates; this rehearsal uses no live object poses.
    row(2,l=([.320,.150,.730],CROSS,.055),r=([.275,-.200,.680],CROSS,.055))
    row(4,l=([.320,.150,.591],CROSS,.055),r=([.275,-.200,.543],CROSS,.055))
    row(5,l=([.320,.150,.591],CROSS,.008),r=([.275,-.200,.543],CROSS,.008))
    row(7,l=([.320,.150,.830],CROSS,.008),r=([.275,-.200,.780],CROSS,.008))
    turn=lambda angle:Rotation.from_euler('z',angle,degrees=True).as_matrix()@CROSS
    row(9,l=([.400,.100,.800],turn(-90),.008),r=([.3387,-.200,.715],SIDE,.008))
    row(11,r=([.3387,-.040,.724],SIDE,.008))
    row(12,r=([.3387,.097,.724],SIDE,.008))
    t=12
    for sweep,z in enumerate([.724,.726,.728]):
        row(t+1,r=([.3387,.097,z],SIDE,.008));t+=1
        for angle in ([0,85] if sweep%2==0 else [0,-90]):
            row(t+3,l=([.400,.100,.800],turn(angle),.008));t+=3
    row(t+1,r=([.3387,-.040,.728],SIDE,.008))
    row(t+3,r=([.454,-.050,.740],SIDE,.008))
    row(t+5,r=([.454,-.050,.740],OPPOSITE,.008))
    row(t+7,r=([.458,.097,.7125],OPPOSITE,.008));t+=7
    for sweep,z in enumerate([.7125,.7145,.7165,.7185]):
        row(t+1,r=([.458,.097,z],OPPOSITE,.008));t+=1
        for angle in ([0,-90] if sweep%2==0 else [0,85]):
            row(t+3,l=([.400,.100,.800],turn(angle),.008));t+=3
    row(t+1,r=([.454,-.050,.740],OPPOSITE,.008));t+=1
    row(t+2)
    if initial_only:rows=rows[:1]
    dense=[rows[0]]
    for a,b in zip(rows[:-1],rows[1:]):
        count=math.ceil((b[0]-a[0])/.25)
        for i in range(1,count+1):
            u=i/count;v=u*u*(3-2*u)
            dense.append((a[0]+(b[0]-a[0])*u,a[1]*(1-v)+b[1]*v,ik.slerp_matrix(a[2],b[2],v),a[3]*(1-v)+b[3]*v,
                          a[4]*(1-v)+b[4]*v,ik.slerp_matrix(a[5],b[5],v),a[6]*(1-v)+b[6]*v))
    rows=dense
    q={s:np.array(constants.READY) for s in ['left','right']};result=[];errors=[];unbiased=[]
    for sec,lp,lr,lg,rp,rr,rg in rows:
        poses={}
        for side,p,R,g in [('left',lp,lr,lg),('right',rp,rr,rg)]:
            seed=q[side].copy(); candidates=[]
            # Offline branch search avoids a joint-limit local minimum. No
            # simulator state is used for this deterministic kinematic solve.
            for start in [seed,np.array(constants.READY),np.array([0.,.5,1.5,-1.,0.,0.]),
                          np.array([0.,1.5,1.,-1.,1.,0.])]:
                def residual(joints):
                    position,rotation=ik.tcp_pose(joints,station.BASES[side])
                    return np.r_[position-p,.3*ik._rot_error(R,rotation)]
                fit=least_squares(residual,np.clip(start,ik.LOWER+1e-8,ik.UPPER-1e-8),
                                  bounds=(ik.LOWER,ik.UPPER),max_nfev=200)
                solved=fit.x;ep=np.linalg.norm(fit.fun[:3]);er=np.linalg.norm(fit.fun[3:])/.3
                candidates.append((ep+.3*er,solved,ep,er))
                if ep<.003 and er<.02:break
            _,q[side],ep,er=min(candidates,key=lambda item:item[0])
            errors.append({'time':sec,'side':side,'position_m':ep,'rotation_rad':er})
            command=q[side].copy()
            f=spec.finger_joint_position(g);poses[side]=[*command.tolist(),f,f]
        result.append((int(sec*spec.CONTROL_HZ),poses))
        unbiased.append({side:q[side].copy() for side in q})
    bad=[e for e in errors if e['position_m']>.003 or e['rotation_rad']>.02]
    if bad:raise RuntimeError('Offline IK reachability failure: '+str([(e['time'],e['side'],round(float(e['position_m']),4),round(float(e['rotation_rad']),4)) for e in bad])+f'; bad count={len(bad)}')
    # Fixed velocity feed-forward offsets the native drive's zero-velocity
    # damping while following the planned path. No measured state is involved.
    for i in range(1,len(result)-1):
        dt=(result[i+1][0]-result[i-1][0])/spec.CONTROL_HZ
        for side in ['left','right']:
            velocity=(unbiased[i+1][side]-unbiased[i-1][side])/dt
            command=np.array(result[i][1][side][:6])+velocity*np.array([5/80]*3+[1.5/10]*3)
            limited=np.clip(command,ik.LOWER,ik.UPPER)
            errors[2*i+(side=='right')]['feedforward_clipped_rad']=float(np.max(abs(command-limited)))
            if np.max(abs(command-limited))>.03:raise RuntimeError(f'Excessive offline feed-forward clipping at step {result[i][0]}, {side}: {np.max(abs(command-limited))}')
            result[i][1][side][:6]=limited.tolist()
    return result,errors

class Controller:
    def __init__(self,ik,constants,station):
        self.plan,self.errors=plan(ik,constants,station);self.observations=[]
    def action(self,step,rgb):
        if set(rgb)!={'overhead_cam','left_wrist_cam','right_wrist_cam'}:raise ValueError('Three RGB streams required')
        if step%20==0:self.observations.append({'step':step,'frame_shapes':{k:list(v.shape) for k,v in rgb.items()}})
        for (a,p),(b,q) in zip(self.plan[:-1],self.plan[1:]):
            if step<=b:
                u=np.clip((step-a)/max(1,b-a),0,1)
                return {s:(1-u)*np.array(p[s])+u*np.array(q[s]) for s in p}
        return {s:np.asarray(v) for s,v in self.plan[-1][1].items()}
