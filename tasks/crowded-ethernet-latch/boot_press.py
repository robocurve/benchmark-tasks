"""Hold the cable boot with the left gripper and depress the latch with the right fingertip."""
import json
from pathlib import Path

import numpy as np

import pinch_ik
from collision_geometry import FingerGeometry
from config import CLIP_Z, RELEASE_TRAVEL, SERVICE, TARGET, port
from feedback import retaining_face
from yam_arena import ik
from yam_arena.constants import FINGER_OPEN
from yam_arena.urdf import axis_angle_matrix, quat_xyzw_to_matrix


class BootPress:
    def __init__(self,env,spec):
        self.env=env
        self.spec=spec
        self.geometry=FingerGeometry()
        path=Path(__file__).parent
        self.prefix=np.load(path/'boot_prefix.npz')['left']
        tip=json.loads((path/'press_point.json').read_text())
        self.point=np.array(tip['point_tcp'])
        self.rr=np.array(tip['rotation'])
        self.aim=np.array(spec.get('aim_mm',[-11.,-.6,4.5]))*.001
        self.ready=env._ready.detach().cpu().numpy().reshape(14).copy()
        self.previous=self.ready.copy()
        self.q=[self.ready[:6].copy(),self.ready[7:13].copy()]
        self.bias=[np.zeros(6),np.zeros(6)]
        self.phase='grasp_boot'
        self.begin=0.
        self.streak=0
        self.failure=None
        self.depth=-.0035
        self.force=0.
        self.pull=0.
        self.offset=np.zeros(3)
        self.rotation_trim=np.eye(3)
        self.target=np.array(port(TARGET))+[.001,0,0]
        self.prev_clip=0.

    def transition(self,phase,t):
        self.phase,self.begin,self.streak=phase,t,0

    def steady(self,cond,count=4):
        self.streak=self.streak+1 if cond else 0
        return self.streak>=count

    def pose(self,name):
        p=self.env._base.arena_world.get_pose_w(name)
        return (p.torch if hasattr(p,'torch') else p).detach().cpu().numpy()[0]

    def command(self,t,observations=None):
        if observations is None:
            joints=[]
            for side in ('left','right'):
                v=self.env._base.scene[side+'_robot'].data.joint_pos
                joints.append((v.torch if hasattr(v,'torch') else v).detach().cpu().numpy()[0])
            plug=self.pose(f'plug_{SERVICE}')
            clip=self.pose(f'clip_{SERVICE}')
        else:
            joints,plug,clip=observations
        coords=[];fingers=[];positions=[];rotations=[]
        for i,side in enumerate(('left','right')):
            ids,fids=self.env._ids[side+'_robot']
            coords.append(joints[i][ids])
            fingers.append(joints[i][fids])
            p,r,_=pinch_ik.geometry(coords[i],self.env.rig.bases[side])
            positions.append(p);rotations.append(r)
        plug_r=quat_xyzw_to_matrix(plug[3:])
        travel=CLIP_Z-(plug_r.T@(clip[:3]-plug[:3]))[2]
        phase,age=self.phase,t-self.begin
        out=self.ready.copy()
        index=min(len(self.prefix)-1,round(t*20))
        out[:7]=self.prefix[index]
        right_r=plug_r@self.rr
        point=self.point.copy();point[0]+=fingers[1][0]+.028
        aim=plug[:3]+plug_r@self.aim
        surface=aim-right_r@point
        hover=surface+plug_r[:,2]*.0035
        far=hover+plug_r@np.array([-.040,0,.028])
        right_p=positions[1];r_target=rotations[1]
        left_p=positions[0];l_target=rotations[0]
        right_use=False;left_use=False
        strength=self.spec.get('boot_mm',2.70)*.001
        if phase=='grasp_boot':
            if index==len(self.prefix)-1:
                self.lp,self.lr=pinch_ik.geometry(out[:6],self.env.rig.bases['left'])[:2]
                self.left_seed=out[:6].copy()
                self.left_rel=plug_r.T@(self.lp-plug[:3]);self.left_rel_rot=plug_r.T@self.lr
                self.init_right=right_p.copy();self.init_rr=r_target.copy()
                self.transition('standoff',t)
        else:
            out[6]=strength/abs(FINGER_OPEN)
            right_use=True
            out[13]=.028/abs(FINGER_OPEN)
            if phase=='standoff':
                s=ik.smoothstep(age/3.5);right_p=self.init_right+s*(far-self.init_right);r_target=ik.slerp_matrix(self.init_rr,right_r,s)
                if age>=3.6 and self.steady(np.linalg.norm(positions[1]-far)<.002):
                    self.transition('hover',t)
            elif phase=='hover':
                s=ik.smoothstep(age/3.5);right_p=far+s*(hover-far);r_target=right_r
                if age>3.6 and self.steady(np.linalg.norm(positions[1]-hover)<.0012):
                    self.transition('press',t)
            elif phase in ('press','unhook'):
                if phase=='press':
                    self.depth=min(self.depth+.000034,.00215)
                    if self.depth>=.0003:
                        self.force=min(self.spec.get('max_n',1.2),self.force+.006)
                    if self.steady(travel>=RELEASE_TRAVEL,5):
                        self.withdraw0=plug[:3].copy();self.withdraw_r=plug_r.copy()
                        self.left_command=out[:6].copy()
                        self.transition('unhook',t)
                else:
                    self.depth=float(np.clip(self.depth+.10*(.00204-travel),.0013,.0024))
                    if travel<.00195:self.force=min(self.spec.get('max_n',1.2),self.force+.008)
                    if travel>=.00165:self.pull=min(.018,self.pull+.00010)
                    left_use=True;left_p=self.lp-[self.pull,0.,0.];l_target=self.lr
                    hook=max((clip[:3]+quat_xyzw_to_matrix(clip[3:])@np.array([.010,y,.0005]))[0] for y in (-.0025,.0025))
                    if hook<.4065:
                        self.clear_right=positions[1].copy();self.clear_r=rotations[1].copy()
                        self.transition('withdraw',t)
                right_p=surface-plug_r[:,2]*self.depth;r_target=right_r
            elif phase in ('withdraw','transfer','insert','open','retreat','rest'):
                s=ik.smoothstep(age/2.) if phase=='withdraw' else 1.
                right_p=self.clear_right+s*np.array([-.075,-.060,.060]);r_target=self.clear_r
                left_use=True
                if phase=='withdraw':
                    self.pull=min(.035,self.pull+.00013)
                    left_p=self.lp-[self.pull,0.,0.];l_target=self.lr
                    if self.pull>=.032 and plug[0]<.380:
                        self.realign=plug[:3].copy();self.realign_r=plug_r.copy()
                        self.carried_rel=plug_r.T@(left_p-plug[:3]);self.carried_rr=plug_r.T@l_target
                        self.transition('transfer',t)
                elif phase in ('transfer','insert'):
                    pre=self.target-[.034,0.,0.]
                    if phase=='transfer':
                        z=ik.smoothstep(age/4.);goal=self.realign+z*(pre-self.realign);goal_r=ik.slerp_matrix(self.realign_r,np.eye(3),z)
                        if age>=4. and self.steady(np.linalg.norm(plug[:3]-pre)<.00075 and plug_r[0,0]>.997):
                            self.insertion=0.;self.transition('insert',t)
                    else:
                        if np.linalg.norm(plug[1:3]-self.target[1:3])<.00085 and pre[0]+self.insertion-plug[0]<.001:
                            self.insertion=min(.034,self.insertion+.00013)
                        goal=pre+[self.insertion,0,0];goal_r=np.eye(3)
                        if self.steady(retaining_face(clip,TARGET)['retained'] and travel<.0003 and np.linalg.norm(plug[1:3]-self.target[1:3])<.001,4):
                            self.seat=positions[0].copy();self.seat_r=rotations[0].copy();self.transition('open',t)
                    self.offset=np.clip(self.offset+.045*(goal-plug[:3]),-.008,.008)
                    angular=ik._rot_error(goal_r,plug_r);an=np.linalg.norm(angular)
                    if an>1e-7:
                        a=axis_angle_matrix(angular/an,min(.006,.035*an))@self.rotation_trim
                        if np.linalg.norm(ik._rot_error(a,np.eye(3)))<.65:self.rotation_trim=a
                    l_target=self.rotation_trim@goal_r@self.carried_rr
                    left_p=goal+self.rotation_trim@goal_r@self.carried_rel+self.offset
                else:
                    l_target=self.seat_r;left_p=self.seat
                    if phase=='open':
                        out[6]=(strength+(.0065-strength)*ik.smoothstep(age/1.3))/abs(FINGER_OPEN)
                        if age>1.5:self.transition('retreat',t)
                    else:
                        left_p=self.seat+np.array([-.12,0.,.020])*(ik.smoothstep(age/3.) if phase=='retreat' else 1.)
                        out[6]=.0065/abs(FINGER_OPEN)
                        if phase=='retreat' and age>3.2:self.transition('rest',t)
                        if phase=='rest':out[6]+=(1.-out[6])*ik.smoothstep(age/1.2)
        for i,(active,side,p,r) in enumerate(zip((left_use,right_use),('left','right'),(left_p,right_p),(l_target,r_target))):
            if not active:continue
            if i==0 and self.q[0] is not None and np.linalg.norm(self.q[0]-coords[0])>.25:self.q[0]=coords[0].copy()
            self.q[i]=pinch_ik.solve(p,r,self.q[i],self.env.rig.bases[side])
            self.bias[i]=np.clip(self.bias[i]+np.clip(.045*(self.q[i]-coords[i]),-.00018,.00018),-.025,.025)
            compensation=self.bias[i].copy()
            if i==1 and phase in ('press','unhook'):
                # Jacobian-transpose fingertip load; the only output remains normal joint targets.
                _,rw,j=pinch_ik.geometry(self.q[i],self.env.rig.bases[side])
                jac=j[:3]+np.cross(j[3:].T,rw@point).T
                ff=jac.T@(-plug_r[:,2]*self.force)/np.array([80.,80.,80.,10.,10.,10.])
                compensation+=np.clip(ff,-.018,.018)
            out[7*i:7*i+6]=self.q[i]+compensation
        bounds=np.array([.04]*6+[.05]+[.04]*6+[.05])
        self.previous=np.clip(np.clip(out,self.previous-bounds,self.previous+bounds),self.env._low,self.env._high)
        limits={'standoff':11,'hover':12,'press':20,'unhook':18,'withdraw':23,'transfer':21,'insert':31}
        if age>limits.get(phase,200):self.failure='timed out in '+phase
        return self.previous,dict(phase=phase,failure=self.failure,complete=phase=='rest' and age>=3.4,
            clip_travel_m=float(travel),opening_command_mm=float(self.previous[13]*abs(FINGER_OPEN)*1000),
            jaws_mm=(fingers[1]*1000).tolist(),tcp=positions[1].tolist(),target=right_p.tolist(),
            arms=[dict(side='left',position_error=float(np.linalg.norm(positions[0]-left_p))),dict(side='right',position_error=float(np.linalg.norm(positions[1]-right_p)))])
