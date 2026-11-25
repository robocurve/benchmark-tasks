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


class KeyboardReference:
    def __init__(self):
        self.joints={a:np.asarray(READY,dtype=float).copy() for a in ARMS}
        self.aperture=dict.fromkeys(ARMS,OPEN);self.points={};self.rotations={};self.state=None;self.measured=None
        self.tick=0;self.phases=[]

    def observe(self,joint_pos,demo_state):self.measured=np.asarray(joint_pos).copy();self.state=demo_state

    def phase(self,name):
        self.phases.append([self.tick,name]);print('KEYBOARD_PHASE '+json.dumps(dict(tick=self.tick,phase=name,
            state=None if self.state is None else self.state['poses'].tolist())),flush=True)

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

    def body_point(self,local):
        b=self.state['poses'][0]
        return b[:3]+L.quat_matrix(b[3:])@np.asarray(local)

    def body_translation(self,target,seconds,arms=ARMS,rotation=None):
        from scipy.spatial.transform import Rotation
        target=np.asarray(target,dtype=float)
        for i in range(round(seconds*CONTROL_HZ)):
            error=target-self.state['poses'][0,:3]
            shift=.30*np.clip(error,-.010,.010)
            correction=np.eye(3)
            if rotation is not None:
                rr=L.quat_matrix(self.state['poses'][0,3:])
                correction=Rotation.from_rotvec(np.clip(.08*Rotation.from_matrix(rotation@rr.T).as_rotvec(),-.006,.006)).as_matrix()
            for arm in arms:
                actual,_=ik.tcp_pose(self.measured[:6] if arm=='left' else self.measured[7:13],BASES[arm])
                # A loaded stock wrist is compliant. Permit the ordinary
                # upward joint-target bias needed to carry the real load;
                # clipping at 35 mm instead made the target follow sag down.
                room=np.array((.035,.035,.080))
                self.points[arm]=np.clip(self.points[arm]+shift,actual-room,actual+room)
                candidate=correction@self.rotations[arm]
                try:q=self.solve(arm,self.points[arm],candidate);self.rotations[arm]=candidate
                except RuntimeError:q=self.solve(arm,self.points[arm],self.rotations[arm])
                self.joints[arm]=q
            yield self.emit()
            if i>=10 and np.linalg.norm(target-self.state['poses'][0,:3])<.005:
                yield from self.hold(.35)
                if np.linalg.norm(target-self.state['poses'][0,:3])<.007:return
        raise RuntimeError(f'Actual keyboard did not reach lift/landing target {target}: {self.state["poses"][0,:3]}')

    def grasp_pair(self,local,diagonal):
        # Both wrists approach before either jaw closes. Capture the actual
        # body frame once, then hold the two world targets during closure;
        # following a pushed body would turn contact drift into positive feedback.
        rotations={a:diagonal.copy() for a in ARMS}
        for arm in ARMS:
            R=L.quat_matrix(self.state['poses'][0,3:])
            turn=L.rz(np.deg2rad(-30.)) if arm=='left' else np.eye(3)
            ori=R@L.START_ROT.T@turn@rotations[arm]
            point=self.body_point(local[arm]);self.aperture[arm]=-.014
            yield from self.go(arm,point+R@[-.055,0,0]+[0,0,.10],ori,1.5)
        reference=self.state['poses'][0].copy();reference_R=L.quat_matrix(reference[3:])
        for ahead,seconds,closing in [(-.055,1.5,False),(0.,1.5,False),(0.,2.,True)]:
            starts={a:self.points[a].copy() for a in ARMS};gaps=self.aperture.copy();count=round(seconds*CONTROL_HZ)
            for i in range(1,count+1):
                f=i/count;f=f*f*(3-2*f);R=reference_R
                for arm in ARMS:
                    point=reference[:3]+R@(local[arm]+[ahead,0,0])
                    self.points[arm]=point if closing else starts[arm]+f*(point-starts[arm])
                    # Align the central wrist while the open fingers enter;
                    # the offset pre-approach avoids its real wrist stop.
                    yaw=(-30. if ahead<0 else -30.*(1-f) if not closing else 0.) if arm=='left' else 0.
                    ori=R@L.START_ROT.T@L.rz(np.deg2rad(yaw))@rotations[arm]
                    actual,_=ik.tcp_pose(self.measured[:6] if arm=='left' else self.measured[7:13],BASES[arm])
                    adjust=np.clip(.65*(self.points[arm]-actual),-.010,.010)
                    self.joints[arm]=self.solve(arm,self.points[arm]+adjust,ori);self.rotations[arm]=ori
                    if closing:self.aperture[arm]=gaps[arm]+f*(-.005-gaps[arm])
                yield self.emit()
        yield from self.hold(.5)

    def regrasp(self,arm,local,rotation,gap=-.005):
        # The other native grasp bears the weight during this physical release,
        # retreat, alignment and reacquisition; no attachment is introduced.
        yield from self.line(arm,self.points[arm],.6,gap=-.022)
        R=L.quat_matrix(self.state['poses'][0,3:])
        yield from self.line(arm,self.points[arm]+R@[-.040,0,0],.6)
        point=self.body_point(local);ori=R@L.START_ROT.T@rotation
        yield from self.go(arm,point+R@[-.040,0,0],ori,.8)
        yield from self.line(arm,self.body_point(local),.8)
        yield from self.line(arm,self.body_point(local),.8,gap=gap)
        yield from self.hold(.3)

    def sequence(self):
        down=frame((0,0,-1),(0,-1,0))
        self.phase('approach the first folded rear support')
        for idx,arm in ((2,'left'),(1,'right')):
            body=self.state['poses'][0];R=L.quat_matrix(body[3:]);foot=self.state['poses'][idx]
            local=np.array((-.021,0,0))
            tip=foot[:3]+L.quat_matrix(foot[3:])@local
            point=tip+np.array((0,0,.007))
            self.aperture[arm]=-.011
            yield from self.go(arm,point+[0,0,.10],down,1.5)
            yield from self.line(arm,point,1.5)
            self.phase('pinch and unfold rear support '+str(idx))
            yield from self.line(arm,point,1.,gap=-.0065)
            # Follow the actual leaf, at most 1.5 degrees ahead. A fixed-time
            # 120-degree arc outran the loaded stock wrist and lost contact.
            # The longer leaf is pinched beyond the pivot ears, with native
            # jaw force only; no joint drive or attachment is introduced.
            for _ in range(300):
                actual_angle=L.angles(self.state['poses'])[idx-1]
                if actual_angle<np.deg2rad(-119.):break
                body=self.state['poses'][0];R=L.quat_matrix(body[3:]);foot=self.state['poses'][idx]
                actual=foot[:3]+L.quat_matrix(foot[3:])@local
                next_angle=max(L.OPEN_ANGLE,actual_angle-np.deg2rad(1.5))
                wanted=body[:3]+R@(L.PIVOTS[idx-1]+L.ry(next_angle)@local)
                self.points[arm]+=.5*np.clip(wanted-actual,-.001,.001)
                self.joints[arm]=self.solve(arm,self.points[arm],self.rotations[arm])
                yield self.emit()
            else:raise RuntimeError(f'Rear support {idx} did not reach its actual open stop: {np.rad2deg(actual_angle)}')
            yield from self.hold(.25)
            # Release completely in place before retreating, so an upward
            # motion cannot drag the over-centre leaf back toward vertical.
            yield from self.line(arm,self.points[arm],.6,gap=-.018)
            yield from self.hold(.2)
            yield from self.line(arm,self.points[arm]+[-.060,0,.040],1.)
            if L.angles(self.state['poses'])[idx-1]>np.deg2rad(-115.):
                raise RuntimeError(f'Rear support {idx} folded after physical release')
        if np.any(L.angles(self.state['poses'])>np.deg2rad(-115.)):
            raise RuntimeError('Both rear supports must remain physically open before lifting')
        self.phase('grasp the front edge and lift from the protective notebook')
        t=np.deg2rad(15.)
        diagonal=frame((np.cos(t),0,-np.sin(t)),(np.sin(t),0,np.cos(t)))
        horizontal=frame((1,0,0),(0,0,1))
        local={'left':np.array((-.055,0.,.0078)),
               'right':np.array((-.055,.14,.0078))}
        yield from self.grasp_pair(local,diagonal)
        self.phase('both real edge grasps closed')
        yield from self.body_translation((.3745,0,.20),5.,rotation=L.START_ROT)
        if self.state['poses'][0,2]<.18:raise RuntimeError('Keyboard was not lifted by the real grasps')
        self.phase('leave the mass centre supported by the original central grasp')
        # The left hand starts centrally. No single edge grasp must resist
        # a widthwise gravity moment during an in-air handover.
        yield from self.together({a:self.points[a] for a in ARMS},.7,
                                 gaps={a:0. for a in ARMS})
        yield from self.line('right',self.points['right'],.6,gap=OPEN)
        yield from self.go('right',(.20,-.35,.35),diagonal,1.2)
        yield from self.body_translation((.3745,0,.30),4.,arms=('left',),rotation=L.START_ROT)
        self.phase('roll the keyboard upright with the real central grasp')
        if self.state['poses'][0,2]<.285:raise RuntimeError('Insufficient actual clearance for the full-size keyboard roll')
        center=self.state['poses'][0,:3].copy()
        actual,_=ik.tcp_pose(self.measured[:6],BASES['left'])
        # Rotate the measured physical grasp offset, keeping the controller's
        # gravity compensation in world coordinates rather than rotating it
        # as if it were part of the keyboard.
        offset=actual-center;bias=self.points['left']-actual
        before=self.rotations['left'].copy()
        for f in np.linspace(0,1,131)[1:]:
            angle=np.pi*f*f*(3-2*f)
            pitch=np.clip((angle-np.deg2rad(120.))/np.deg2rad(60.),0.,1.)
            Q=L.ry(np.deg2rad(30.)*pitch)@L.rx(angle)
            self.rotations['left']=Q@before
            actual,_=ik.tcp_pose(self.measured[:6],BASES['left'])
            wanted_center=center+[0,0,-.05*pitch]
            bias+=.25*np.clip(wanted_center-self.state['poses'][0,:3],-.003,.003)
            bias=np.clip(bias,-.080,.080)
            self.points['left']=wanted_center+Q@offset+bias
            self.joints['left']=self.solve('left',self.points['left'],self.rotations['left'])
            yield self.emit()
        yield from self.hold(.3)
        self.phase('lower the actual rear supports onto the desk')
        # Complete the roll with the front edge raised. The stock fingers
        # remain clear while the rear supports reach the actual desk. Slow
        # release then lets gravity lower the front pads into their rest.
        wanted_rotation=L.ry(np.deg2rad(30.))
        # Lower before translating toward the bases. A diagonal high return
        # passes through a stock wrist stop even though both endpoints fit.
        yield from self.body_translation((.3745,0,.10),5.,arms=('left',),rotation=wanted_rotation)
        yield from self.body_translation((.22,0,.10),5.,arms=('left',),rotation=wanted_rotation)
        # Land with the actual toes behind their hinges. At a 30-degree
        # front-up body tilt the 120-degree leaf is nearly vertical, so even
        # normal wrist tracking error can make the desk force close it.
        center=self.state['poses'][0,:3].copy()
        actual,_=ik.tcp_pose(self.measured[:6],BASES['left'])
        offset=actual-center;bias=self.points['left']-actual
        before=self.rotations['left'].copy()
        for f in np.linspace(0,1,41)[1:]:
            Q=L.ry(np.deg2rad(-14.)*f*f*(3-2*f))
            self.points['left']=center+Q@offset+bias
            self.rotations['left']=Q@before
            self.joints['left']=self.solve('left',self.points['left'],self.rotations['left'])
            yield self.emit()
        yield from self.hold(.35)
        R=L.quat_matrix(self.state['poses'][0,3:])
        pitch=np.rad2deg(np.arctan2(R[0,2],R[2,2]))
        if not 5.<pitch<23.:
            raise RuntimeError(f'Actual landing pitch does not keep toes behind hinges: {pitch}')
        for _ in range(200):
            if np.any(self.state['desk_counts'][:3]):break
            self.points['left']+=[0,0,-.001]
            actual,_=ik.tcp_pose(self.measured[:6],BASES['left'])
            correction=np.clip(.65*(self.points['left']-actual),-.010,.010)
            self.joints['left']=self.solve('left',self.points['left']+correction,self.rotations['left'])
            yield self.emit()
        else:raise RuntimeError('No real keyboard-to-desk contact during controlled landing')
        yield from self.hold(.3)
        self.phase('release and leave the keyboard supported on the desk')
        yield from self.line('left',self.points['left'],1.5,gap=OPEN)
        yield from self.line('left',self.points['left']+[-.06,0,.06],1.5)
        while True:yield self.emit()
