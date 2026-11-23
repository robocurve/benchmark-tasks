"""Pure Python bimanual waypoint specification; Isaac adapter is imported lazily."""
from __future__ import annotations
from dataclasses import dataclass
from math import ceil

TABLE=.7475
TIP=.013
OPEN=-.04695
PINCH=-.012

@dataclass(frozen=True)
class Step:
    duration:float
    pos:tuple[float,float,float]
    finger:float
    label:str
    yaw:float=0.
    roll:float=0.
    pitch:float=0.

class Track(list):
    def go(self,point,finger,label,seconds=1.0,yaw=0.,roll=0.,pitch=0.):
        self.append(Step(float(seconds),tuple(float(x) for x in point),float(finger),str(label),
                         float(yaw),float(roll),float(pitch)))
    @property
    def duration(self):
        return sum(p.duration for p in self)
    def wait_until(self,t):
        if self.duration<t-1e-7:
            p=self[-1]
            self.go(p.pos,p.finger,'hold',t-self.duration,p.yaw,p.roll,p.pitch)

def waypoints(left,right):
    from edamame_policy import EdamameWaypoint
    return {k:[EdamameWaypoint(s.duration,s.pos,s.finger,s.label,s.yaw,s.roll,s.pitch)
               for s in tr] for k,tr in (('left',left),('right',right))}

def check_tracks(tracks,steps=2200):
    for arm,track in tracks.items():
        assert track and all(s.duration>.05 and OPEN<=s.finger<=0 for s in track),arm
        assert all(.16<s.pos[0]<.71 and -.37<s.pos[1]<.37
                   and TABLE+.007<s.pos[2]<=TABLE+.168 for s in track),arm
        assert ceil(track.duration*20)+60<steps,arm
