"""RGB-only fixed-layout rehearsal; wrench is physically held at reset."""
import math
import numpy as np
import spec

def rz(a):
    c,s=math.cos(a),math.sin(a);return np.array([[c,-s,0],[s,c,0],[0,0,1.]])
def ry(a):
    c,s=math.cos(a),math.sin(a);return np.array([[c,0,s],[0,1,0],[-s,0,c]])

def plan(ik,constants,station):
    t=spec.TABLE_Z;down=np.array([[0.,-1.,0.],[-1.,0.,0.],[0.,0.,-1.]])
    left_grip=np.diag([-1.,1.,-1.]);right_side=down
    axis=np.array([[0.,0.,-1.],[0.,1.,0.],[1.,0.,0.]])
    grip=np.array(spec.DRIVER_GRIP_LOCAL);tip=np.array(spec.CLAMP)+axis@np.array([0,0,-.0005])
    start=np.asarray(spec.PIVOT)+rz(spec.INITIAL_YAW)@np.array([.031,0,.043])
    aligned=np.asarray(spec.ALIGNED_PIVOT)+np.array([.031,0,.043]);rest=np.array([.16,.40,t+.19])
    def tool(p,a=0.,yaw=0.):
        rotation=rz(yaw)@axis@rz(math.radians(a))
        return np.asarray(p)+rotation@grip,rotation@right_side
    def pose(p,g,R):return (np.asarray(p),g,R)
    initial=tool(spec.DRIVER_POS,yaw=spec.INITIAL_YAW)
    hold=pose(initial[0],0.,initial[1]);lhold=pose(aligned,.030,left_grip)
    rows=[(0,pose(rest,.075,down),pose(initial[0],.020,initial[1]),0.),
          (2,pose(start+[0,0,.008],.055,rz(spec.INITIAL_YAW)@left_grip),hold,0.),
          (4,pose(start,.055,rz(spec.INITIAL_YAW)@left_grip),hold,0.),
          (6,pose(start,.030,rz(spec.INITIAL_YAW)@left_grip),hold,0.),
          (9,lhold,pose(*[tool(tip)[0],0.,tool(tip)[1]]),0.)]
    for sec,angle,torque in [(11,-12,.77),(13,-25,1.54),(15,-40,2.42),(17,-50,3.01),(19,-55,3.30),(21,-55,3.30),(23,-35,0.)]:
        # Fixed nominal axial relief compensates arm compliance under the
        # prescribed wrench load; no scene measurements enter this command.
        relief=axis@np.array([0.,0.,.0025*min(torque/3.30,1.)])
        p,R=tool(tip+relief,angle);rows.append((sec,lhold,pose(p,0.,R),torque))
    p,R=tool(tip+[-.045,0,0],-35);rows.append((25,lhold,pose(p,0.,R),0.))
    # Hold the withdrawn wrench clear while the standard right wrist camera
    # watches the physical detector LED. This is offline fixed-layout geometry.
    R=axis@right_side;camera=station.wrist_camera('right')
    direction=-R@ik.quat_xyzw_to_matrix(np.asarray(camera.rot_xyzw))[:,2]
    led=np.asarray(spec.ALIGNED_PIVOT)+np.array([.031,0.,.050])
    camera_position=led-.28*direction
    p=camera_position-R@(np.asarray(camera.pos)-np.asarray(constants.TCP_OFFSET))
    park=pose(p,0.,R)
    rows.extend([(28,pose(aligned,.055,left_grip),park,0.),(30,pose(rest,.075,left_grip),park,0.)])
    cx,cy,_=spec.CARD_START;pickup=ry(math.radians(-25))@down
    test=np.array([[-1.,0.,0.],[0.,0.,-1.],[0.,-1.,0.]])
    for sec,p,g,R in [(32,(cx,cy,t+.16),.065,pickup),(34,(cx,cy,t+.052),.065,pickup),
                      (36,(cx,cy,t+.052),.025,pickup),(39,(cx,cy,t+.17),.025,pickup),
                      (42,(.59,0,t+.110),.025,test),(44,(.59,0,t+.110),.025,test),
                      (47,(cx,cy,t+.17),.025,pickup),(50,(cx,cy,t+.052),.025,pickup),
                      (52,(cx,cy,t+.052),.065,pickup),(55,rest,.075,left_grip)]:
        rows.append((sec,pose(p,g,R),park,0.))
    q={s:np.array(constants.READY) for s in ['left','right']};rng=np.random.default_rng(42);result=[];errors=[]
    gains=np.array([constants.SHOULDER['stiffness']]*3+[constants.WRIST['stiffness']]*3)
    def compensate(j,R,torque):
        offset=np.array([.0507,0.,.0388])-grip
        def center(v):
            p,r=ik.tcp_pose(v,station.BASES['right']);return p+r@right_side.T@offset
        p=center(j);tcp=ik.tcp_pose(j,station.BASES['right'])[0];jac=np.empty((3,6));jt=np.empty((3,6));eps=1.e-5
        for k in range(6):
            v=j.copy();v[k]+=eps;jac[:,k]=(center(v)-p)/eps;jt[:,k]=(ik.tcp_pose(v,station.BASES['right'])[0]-tcp)/eps
        radial=(R@right_side.T)[:,0];force=torque/grip[0]*np.cross([1.,0.,0.],radial)
        return np.clip(j-jac.T@np.array([0.,0.,-.220*9.81])/gains+jt.T@force/gains,ik.LOWER,ik.UPPER)
    for sec,left,right,torque in rows:
        poses={}
        for side,(p,g,R) in [('left',left),('right',right)]:
            candidates=[]
            for seed in [q[side],constants.READY,*[rng.uniform(ik.LOWER,ik.UPPER) for _ in range(12)]]:
                trial=ik.solve(p,ik.matrix_to_quat_xyzw(R),seed,station.BASES[side],iters=500,damping=.012);candidates.append(trial)
                if trial[1]<.0005 and trial[2]<.005:break
            q[side],ep,er=min(candidates,key=lambda v:v[1]+.2*v[2]);errors.append({'time':sec,'side':side,'position_m':ep,'rotation_rad':er})
            command=compensate(q[side],R,torque) if side=='right' and sec>0 else q[side]
            f=spec.finger_joint_position(g);poses[side]=[*command.tolist(),f,f]
        result.append((int(sec*20),poses))
    if any(e['position_m']>.003 or e['rotation_rad']>.02 for e in errors):raise RuntimeError('Offline reachability failed: '+str([e for e in errors if e['position_m']>.003 or e['rotation_rad']>.02]))
    return result,errors

class Controller:
    def __init__(self,ik,constants,station):self.plan,self.errors=plan(ik,constants,station);self.observations=[]
    def action(self,step,rgb):
        if set(rgb)!={'overhead_cam','left_wrist_cam','right_wrist_cam'}:raise ValueError('Three RGB streams required')
        if step%20==0:
            self.observations.append({'step':step,'yellow_pixels':{k:int(((v[:,:,0]>160)&(v[:,:,1]>120)&(v[:,:,2]<90)).sum()) for k,v in rgb.items()}})
        for (a,p),(b,q) in zip(self.plan[:-1],self.plan[1:]):
            if step<=b:
                u=np.clip((step-a)/max(1,b-a),0,1);u=u*u*(3-2*u)
                return {s:(1-u)*np.array(p[s])+u*np.array(q[s]) for s in p}
        return {s:np.asarray(v) for s,v in self.plan[-1][1].items()}
