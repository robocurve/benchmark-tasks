"""Privileged reference, issuing only stock robot joint/jaw targets."""
import json
import numpy as np
from inspect_robots import Action,ActionChunk
from yam_arena import ik
from yam_arena.constants import READY,FINGER_OPEN
from . import layout as L
from .fixed_scene import ARMS,BASES,CONTROL_HZ
OPEN=float(FINGER_OPEN)


def frame(approach,closing):
    z=np.asarray(approach,dtype=float);z/=np.linalg.norm(z)
    x=np.asarray(closing,dtype=float);x-=z*(x@z);x/=np.linalg.norm(x)
    return np.column_stack((x,np.cross(z,x),z))


class ToasterReference:
    def __init__(self):
        self.joints={a:np.asarray(READY,dtype=float).copy() for a in ARMS}
        self.aperture=dict.fromkeys(ARMS,OPEN);self.points={};self.rotations={};self.state=None;self.measured=None
        self.tick=0;self.phases=[]

    def observe(self,joint_pos,demo_state):self.measured=np.asarray(joint_pos).copy();self.state=demo_state

    def phase(self,name):
        self.phases.append([self.tick,name]);print('TOASTER_PHASE '+json.dumps(dict(tick=self.tick,phase=name,
            state=None if self.state is None else self.state['poses'].tolist(),measured_joint_pos=None if self.measured is None else self.measured.tolist())),flush=True)

    def emit(self):
        self.tick+=1
        target=np.concatenate([np.r_[self.joints[a],np.clip(self.aperture[a]/OPEN,0.,1.)] for a in ARMS])
        return ActionChunk(actions=[Action(data=target,meta={})],control_hz=CONTROL_HZ)

    def hold(self,seconds):
        for _ in range(max(1,round(seconds*CONTROL_HZ))):yield self.emit()

    def solve(self,arm,point,rotation):
        side=1 if arm=='left' else -1
        seeds=[self.joints[arm],np.array((0.,1.6,1.4,-.8,0.,0.)),np.array((side*.4,2.2,1.,-.5,side,0.)),np.array((0.,2.1,1.,-.4,side*1.5,-.1))]
        best=None
        for seed in seeds:
            q,pe,re=ik.solve(point,ik.matrix_to_quat_xyzw(rotation),seed,BASES[arm],iters=100)
            if best is None or pe+.01*re<best[1]+.01*best[2]:best=q,pe,re
            if pe<.0002 and re<.01:break
        q,pe,re=best
        if pe>.003 or re>.06:raise RuntimeError(f'{arm} unreachable {point}: {pe}m/{re}rad')
        return q

    def go(self,arm,point,rotation,seconds):
        before=self.joints[arm].copy();target=self.solve(arm,point,rotation)
        count=max(1,round(seconds*CONTROL_HZ))
        for i in range(1,count+1):
            f=i/count;f=f*f*(3-2*f);self.joints[arm]=before+f*(target-before);yield self.emit()
        self.points[arm]=np.asarray(point,dtype=float).copy();self.rotations[arm]=np.asarray(rotation).copy()

    def line(self,arm,point,seconds,gap=None):
        start=self.points[arm].copy();old=self.aperture[arm];count=max(1,round(seconds*CONTROL_HZ))
        for i in range(1,count+1):
            f=i/count;f=f*f*(3-2*f);self.points[arm]=start+f*(np.asarray(point)-start)
            actual,_=ik.tcp_pose(self.measured[:6] if arm=='left' else self.measured[7:13],BASES[arm])
            correction=np.clip(.7*(self.points[arm]-actual),-.012,.012)
            self.joints[arm]=self.solve(arm,self.points[arm]+correction,self.rotations[arm])
            if gap is not None:self.aperture[arm]=old+f*(gap-old)
            yield self.emit()

    def together(self,targets,seconds,gaps=None,smooth=True):
        starts={a:self.points[a].copy() for a in targets};old=self.aperture.copy();count=max(1,round(seconds*CONTROL_HZ))
        for i in range(1,count+1):
            f=i/count
            if smooth:f=f*f*(3-2*f)
            for a,dest in targets.items():
                self.points[a]=starts[a]+f*(np.asarray(dest)-starts[a])
                self.joints[a]=self.solve(a,self.points[a],self.rotations[a])
            for a,gap in (gaps or {}).items():self.aperture[a]=old[a]+f*(gap-old[a])
            yield self.emit()

    def orient(self,arm,rotation,seconds):
        point=self.points[arm].copy()
        yield from self.go(arm,point,rotation,seconds)

    def follow_body(self,arm,body,local,rotation,seconds):
        # Read-only task-body pose; ordinary joint targets remain the only action.
        pose=self.state['poses'][body];point=pose[:3]+L.quat_matrix(pose[3:])@np.asarray(local)
        yield from self.line(arm,point,seconds)

    def rotate(self,arm,rotation,seconds):
        start=self.rotations[arm].copy();point=self.points[arm].copy()
        from scipy.spatial.transform import Rotation,Slerp
        interp=Slerp([0.,1.],Rotation.from_matrix([start,rotation]))
        for i in range(1,round(seconds*CONTROL_HZ)+1):
            f=i/(seconds*CONTROL_HZ);f=f*f*(3-2*f)
            r=interp(f).as_matrix();self.rotations[arm]=r;self.joints[arm]=self.solve(arm,point,r)
            yield self.emit()

    def track_tray_to(self,center,seconds):
        # Correct only ordinary robot targets using read-only actual tray pose.
        center=np.asarray(center);target=self.points['left']+center-self.state['poses'][L.TRAY,:3]
        yield from self.line('left',target,seconds)

    def servo_tray(self,center,seconds,leading_height=None):
        # Measured object feedback changes only stock joint targets. Correct
        # load deflection and grip rotation before entering the real guides.
        from scipy.spatial.transform import Rotation
        start=self.state['poses'][L.TRAY,:3].copy();center=np.asarray(center,dtype=float)
        local_goal=L.RZ.T@(center-L.TOASTER_CENTER)
        count=max(1,round(seconds*CONTROL_HZ));initial_height=None
        for i in range(1,count+1):
            f=min(1.,i/(.60*count));f=f*f*(3-2*f)
            pose=self.state['poses'][L.TRAY];housing=self.state['poses'][L.TOASTER]
            rr=L.quat_matrix(pose[3:]);rb=L.quat_matrix(housing[3:])
            goal=housing[:3]+rb@local_goal;wanted=start+f*(goal-start)
            if leading_height is not None:
                edge=np.array(((-L.TRAY_SIZE[0]/2,-L.TRAY_SIZE[1]/2,0.),(L.TRAY_SIZE[0]/2,-L.TRAY_SIZE[1]/2,0.)))@rr.T+pose[:3]
                if initial_height is None:initial_height=float(edge[:,2].min())
                desired_height=housing[2]+leading_height
                wanted[2]=pose[2]+initial_height+f*(desired_height-initial_height)-float(edge[:,2].min())
            actual,_=ik.tcp_pose(self.measured[:6],BASES['left'])
            correction=np.clip(.22*(wanted-pose[:3]),-.004,.004)
            self.points['left']=np.clip(self.points['left']+correction,actual-.025,actual+.025)
            self.points['left'][2]=max(.030,self.points['left'][2])
            error=Rotation.from_matrix(rb@rr.T).as_rotvec()
            turn=np.clip(.10*error,-np.deg2rad(.35),np.deg2rad(.35))
            candidate=Rotation.from_rotvec(turn).as_matrix()@self.rotations['left']
            # The stock wrist has less pitch freedom close to its own base.
            # Respect that physical workspace rather than commanding a
            # below-table TCP when a rail is supporting the held tray.
            pitch=np.arctan2(-candidate[2,2],np.linalg.norm(candidate[:2,2]))
            if self.points['left'][0]<.35 and pitch<np.deg2rad(12.):
                candidate=Rotation.from_rotvec([0,np.deg2rad(12.)-pitch,0]).as_matrix()@candidate
            try:
                q=self.solve('left',self.points['left'],candidate)
                self.rotations['left']=candidate
            except RuntimeError:
                q=self.solve('left',self.points['left'],self.rotations['left'])
            self.joints['left']=q
            yield self.emit()

    def sequence(self):
        arm='left';tilt=np.deg2rad(20.)
        rot=frame((np.cos(tilt),0,-np.sin(tilt)),(0,-1,0))
        horizontal=frame((1,0,0),(0,-1,0))
        grip=self.state['poses'][L.TRAY,:3]+L.quat_matrix(self.state['poses'][L.TRAY,3:])@L.HANDLE_LOCAL
        self.phase('approach the ordinary front crumb-tray pull handle')
        yield from self.go(arm,grip+[-.060,0,.220],rot,2.)
        yield from self.line(arm,grip+[-.060,0,.040],2.)
        yield from self.line(arm,grip+[.001,0,.001],1.8)
        self.phase('grasp the plastic pull handle with the stock parallel pads')
        yield from self.line(arm,self.points[arm],1.0,gap=-.010)
        yield from self.hold(.4)
        self.phase('draw the loaded tray horizontally out of its actual guides')
        yield from self.line(arm,self.points[arm]+[-.278,0,0],3.)
        yield from self.hold(.4)
        self.phase('keep the tray level while carrying it above the waste bin')
        yield from self.line(arm,self.points[arm]+[0,0,.170],2.)
        yield from self.track_tray_to([*L.BIN_CENTER[:2],.193],3.)
        yield from self.hold(.5)
        self.phase('tip the actual metal tray and let dry bread crumbs fall into the bin')
        yield from self.rotate(arm,horizontal,1.5)
        a=np.deg2rad(-118.);c,s=np.cos(a),np.sin(a);rx=np.array(((1,0,0),(0,c,-s),(0,s,c)))
        yield from self.rotate(arm,rx@horizontal,3.)
        yield from self.hold(1.5)
        # A gentle physical jostle releases pieces held by flange friction
        # just short of the stock wrist's120degree rotation stop.
        for _ in range(2):
            before=self.points[arm].copy()
            yield from self.line(arm,before+[0,-.025,0],.25)
            yield from self.hold(.5)
            yield from self.line(arm,before,.75)
            yield from self.hold(.25)
        yield from self.rotate(arm,horizontal,3.)
        yield from self.rotate(arm,rot,1.5)
        yield from self.hold(.5)
        self.phase('carry the empty tray back to the guide entrance')
        target=L.TRAY_CENTER+[-.278,0,.170]
        yield from self.track_tray_to(target,3.)
        # A measured leading-edge loop keeps both floor corners above the
        # physical rails despite stock arm compliance. No task body is written.
        yield from self.servo_tray([L.TRAY_CENTER[0]-.278,L.TRAY_CENTER[1],.045],4.5,leading_height=.0215)
        self.phase('set the leading tray edge onto the actual guide rails')
        yield from self.servo_tray([L.TRAY_CENTER[0]-.238,L.TRAY_CENTER[1],.045],1.5,leading_height=.0215)
        yield from self.servo_tray(L.TRAY_CENTER+[-.238,0,.001],1.5)
        yield from self.hold(.3)
        self.phase('slide the tray all the way home against its real rear stop')
        yield from self.servo_tray(L.TRAY_CENTER+[-.0005,0,.0002],5.)
        yield from self.hold(.7)
        self.phase('let go and leave the clean dry tray installed')
        yield from self.line(arm,self.points[arm],1.,gap=OPEN)
        yield from self.line(arm,self.points[arm]+[-.060,0,.060],1.5)
        while True:yield self.emit()
