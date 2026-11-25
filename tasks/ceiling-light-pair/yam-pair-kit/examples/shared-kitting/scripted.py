"""Independent geometric controllers, one per robot, used by task.py's demo() as yam_pair closed-loop controllers
(CONTRACT.md section 5.2). They read object poses; they are a scripted reference, not a policy.

Each instance only owns its own bimanual joint array and its assigned parts.
The reservation receives requests/releases, never produces joint commands.
"""
import numpy as np
from yam_arena import station
from yam_arena.constants import READY, FINGER_OPEN
from yam_arena.demo import ArmPlan, Waypoint

TOP=.7475

class ScriptedController:
    def __init__(self, agent):
        self.agent=agent
        self.time=0.; self.part=0; self.plan=None
        self.last=np.array([*READY,1.,*READY,1.])
        self.finished=False; self.phase='waiting';self.release=False
        self.parking=False;self.park_time=0.;self.park_settled=0;self.previous_park_q=None
        self.worst_mm=0.;self.worst_deg=0.
        self._prepare()

    def _prepare(self):
        if self.part>=2:
            self.finished=True;self.phase='done';return
        # In each robot's own station frame, objects and orders are mirrored by world placement.
        self.side='left' if self.part==0 else 'right'
        sy=1 if self.side=='left' else -1
        # B starts with world +y (its right), so use local +y first consistently.
        x=.425; y=sy*.12; z=TOP+.006+(.0115 if self.agent=='a' else .017)+.013
        dest=(.20,sy*.16,z+.008)
        hover=TOP+.105
        clear=(.27,sy*.21,hover)
        wp=[Waypoint(1.8,(x,y,hover),FINGER_OPEN,'approach shared tray'),
            Waypoint(1.2,(x,y,z),FINGER_OPEN,'descend'),
            Waypoint(.8,(x,y,z),0.,'grasp'),
            Waypoint(1.1,(x,y,hover),0.,'lift'),
            Waypoint(1.5,clear,0.,'clear shared region'),
            Waypoint(1.2,(*dest[:2],hover),0.,'carry outside shared region'),
            Waypoint(1.2,dest,0.,'lower to order tray'),
            Waypoint(.8,dest,FINGER_OPEN,'release'),
            Waypoint(1.1,(*dest[:2],hover),FINGER_OPEN,'retreat'),
            Waypoint(1.3,(.18,sy*.32,TOP+.16),FINGER_OPEN,'withdraw to own side')]
        sl=slice(0,7) if self.side=='left' else slice(7,14)
        self.plan=ArmPlan(wp,station.BASES[self.side],q_start=self.last[sl][:6],finger_start=FINGER_OPEN)
        self.time=0.;self.release=False

    @property
    def requesting(self):
        return not self.finished and not self.parking and self.time<6.4

    def _park(self, obs):
        """Withdraw the used arm before letting its own other arm enter the tray.

        The previous version left the first arm hovering over its order tray;
        its elbow/wrist obstructed the partner's second approach. Each local
        controller now returns its own used arm to READY and verifies measured
        settling. The partner's joints are never accessed here.
        """
        self.park_time+=.05
        t=min(self.park_time/1.5,1.);s=t*t*(3-2*t)
        i=0 if self.side=='left' else 7
        self.last[i:i+6]=self.park_start+(np.array(READY)-self.park_start)*s
        self.last[i+6]=1.
        actual=obs.state[i:i+6]
        error=np.max(np.abs(actual-np.array(READY)))
        speed=np.inf if self.previous_park_q is None else np.max(np.abs(actual-self.previous_park_q))/.05
        self.previous_park_q=actual.copy()
        self.park_settled=self.park_settled+1 if t>=1 and error<.02 and speed<.1 else 0
        self.phase='return used arm to READY' if t<1 else 'wait for own arm to settle'
        self.release=False
        if self.park_settled>=6:
            self.last[i:i+6]=READY
            self.parking=False;self.part+=1;self._prepare()
        return self.last.copy()

    def act(self, obs, granted=True):
        if obs.agent!=self.agent or obs.state.shape!=(14,):raise ValueError('local observation required')
        if self.finished:return self.last.copy()
        if self.parking:return self._park(obs)
        if not granted and self.time==0:
            self.phase='waiting for reservation';return self.last.copy()
        self.time+=.05
        a,self.phase=self.plan.action7(self.time)
        sl=slice(0,7) if self.side=='left' else slice(7,14)
        self.last[sl]=a
        self.worst_mm=max(self.worst_mm,self.plan.worst_ik_mm)
        self.worst_deg=max(self.worst_deg,self.plan.worst_ik_deg)
        # Release token only after the physical controller has reached the exit waypoint.
        i=0 if self.side=='left' else 7
        actual=obs.state[i:i+6]
        from yam_arena import ik
        pos,_=ik.tcp_pose(actual,station.BASES[self.side])
        self.release=self.time>=6.4 and pos[0]<.30
        if self.time>=self.plan.duration+.5:
            self.parking=True;self.park_start=self.last[sl][:6].copy()
            self.park_time=0.;self.park_settled=0;self.previous_park_q=None
        return self.last.copy()
