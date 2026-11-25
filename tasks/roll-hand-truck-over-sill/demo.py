"""Fixed-layout rehearsal with RGB inputs only and offline joint kinematics."""
import math
import numpy as np
import spec

def ry(a):
    c,s=math.cos(a),math.sin(a);return np.array([[c,0,s],[0,1,0],[-s,0,c]])

def plan(ik,constants,station):
    # Axle coordinates and frame pitch are nominal rehearsal waypoints, never
    # simulator readings. Wheel contact determines the actual cart motion.
    rows=[(0,.435,.0635,0,.070,.050),(3,.455,.0635,0,.070,0.),
          (5,.455,.0635,0,0.,0.),(10,.455,.0635,-10,0.,0.),
          (15,.432,.070,-10,0.,0.),(20,.390,.0785,-10,0.,0.),
          (24,.370,.0785,-10,0.,0.),(29,.325,.0635,-10,0.,0.),
          # Keep the load tilted while the tires traverse the corner. These
          # are fixed rehearsal poses, not live simulator pose feedback.
          (32,.415,.0730,-15,0.,0.),(37,.380,.0785,-15,0.,0.),
          (42,.350,.0750,-15,0.,0.),(47,.320,.0435,-15,0.,0.),
          (52,.220,.0435,-15,0.,0.),(59,-.030,.0435,-15,0.,0.),
          (64,.020,.0635,3,0.,0.),(67,.020,.0635,3,.070,0.),
          (70,.020,.0635,3,.075,.100)]
    q={s:np.array(constants.READY) for s in station.BASES};result=[];errors=[];rng=np.random.default_rng(17)
    for sec,x,z,pitch,gap,above in rows:
        turn=ry(math.radians(pitch));poses={}
        for side,sign in [('left',1),('right',-1)]:
            p=np.array([x,0,z])+turn@np.array([0,sign*spec.GRIP_Y,spec.HANDLE_LOCAL_Z])+np.array([0,0,above])
            R=turn@(np.diag([-1.,1.,-1.]) if side=='left' else np.diag([1.,-1.,-1.]))
            candidates=[]
            for seed in [q[side],constants.READY,*[rng.uniform(ik.LOWER,ik.UPPER) for _ in range(10)]]:
                trial=ik.solve(p,ik.matrix_to_quat_xyzw(R),seed,station.BASES[side],iters=400,damping=.012)
                candidates.append(trial)
                if trial[1]<.0005 and trial[2]<.005:break
            q[side],ep,er=min(candidates,key=lambda v:v[1]+.2*v[2])
            errors.append({'time':sec,'side':side,'position_m':ep,'rotation_rad':er})
            f=spec.finger_joint_position(gap);poses[side]=[*q[side],f,f]
        result.append((int(sec*20),poses))
    if any(e['position_m']>.003 or e['rotation_rad']>.02 for e in errors):raise RuntimeError('Unreachable nominal waypoint: '+str(errors))
    return result,errors

class Controller:
    def __init__(self,ik,constants,station):self.plan,self.errors=plan(ik,constants,station);self.observations=[]
    def action(self,step,rgb):
        if set(rgb)!={'overhead_cam','left_wrist_cam','right_wrist_cam'}:raise ValueError('Three RGB views required')
        if step%20==0:
            self.observations.append({'step':step,'bright_case_pixels':{k:int(((v[:,:,0]>140)&(v[:,:,1]>90)&(v[:,:,2]<100)).sum()) for k,v in rgb.items()}})
        for (a,p),(b,q) in zip(self.plan[:-1],self.plan[1:]):
            if step<=b:
                u=np.clip((step-a)/max(1,b-a),0,1);u=u*u*(3-2*u)
                return {s:(1-u)*np.array(p[s])+u*np.array(q[s]) for s in p}
        return {s:np.asarray(v) for s,v in self.plan[-1][1].items()}
