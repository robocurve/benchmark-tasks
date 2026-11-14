"""Cartesian motions for lifting a clipped flat stack and passing it between the arms.
Only arm joint actions are produced; object poses and dynamics are never changed.
"""
import math
import os
import numpy as np
import yam_arena.demo as D
from yam_arena.demo import Waypoint

class Grip(float):
    def __new__(cls, yaw, tilt=0.0):
        obj = float.__new__(cls, yaw)
        obj.tilt = tilt
        return obj

_down = getattr(D.grip_rotation, 'straight_down', D.grip_rotation)
def rotation(yaw=0.0):
    a = getattr(yaw, 'tilt', 0.0)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]]) @ _down(float(yaw))
rotation.straight_down = _down
D.grip_rotation = rotation

class Plan:
    def __init__(self, top):
        self.top, self.w = top, []
    def add(self,t,p,f,label,yaw=0.0,tilt=0.0):
        self.w.append(Waypoint(t,(p[0],p[1],self.top+p[2]),f,label,Grip(yaw,tilt)))
        return self
    @property
    def time(self):return sum(w.duration for w in self.w)
    def wait(self,t):
        w=self.w[-1];self.w.append(Waypoint(t,w.pos,w.finger,'hold',w.yaw));return self
    def until(self,t):
        if t>self.time:self.wait(t-self.time)

def make(top, clip_xy, clip_z, slot_top, dish):
    mode=os.environ.get('TASK_DEMO','success')
    L,R=Plan(top),Plan(top)
    O,C=-0.0469,0.0
    x,y=clip_xy
    lift=.16
    L.add(2,(x+.003,y,.16),O,'above clip body')
    L.add(1.5,(x+.003,y,clip_z+.015),O,'down beside clip body')
    L.add(.8,(x+.003,y,clip_z+.015),C,'grip clip across its width')
    L.add(2,(x+.003,y,lift),C,'lift clipped paper off desk')
    a=math.radians(-85.5)
    off=np.array([[math.cos(a),0,math.sin(a)],[0,1,0],[-math.sin(a),0,math.cos(a)]])@np.array([.003,0,.015])
    edge=np.array([.22,y,.15]); hold=edge+off
    L.add(2.5,hold,C,'turn paper upright',tilt=a)
    L.wait(1)
    if mode=='pickup':return {'left':L.wait(2).w}
    R.add(2,(.22,-.33,.19),O,'right waits clear',math.pi/2)
    R.until(L.time)
    R.add(2,(.22,-.08,.19),O,'right above upright paper',math.pi/2)
    R.add(1.5,(.22,-.08,.108),O,'right beside upright paper',math.pi/2)
    R.add(.8,(.22,-.08,.108),C,'right holds paper',math.pi/2)
    R.wait(.5); L.until(R.time)
    L.add(.6,hold,O,'release clip body',tilt=a)
    L.add(1,(hold[0]-.025,hold[1],hold[2]+.02),O,'withdraw from clip',tilt=a)
    L.add(1.8,(.22,y,.215),O,'above clip handles',-math.pi/2)
    if mode=='failure':
        L.add(1,(.18,.25,.18),O,'left clear',-math.pi/2)
    else:
        L.add(1.2,(.22,y,.1735),O,'down around handles',-math.pi/2)
        L.add(.8,(.22,y,.1735),C,'pinch clip handles',-math.pi/2)
        L.add(.5,(.22,y,.1735),C,'open sprung jaw',-math.pi/2)
        L.add(1.4,(.22,y,.215),C,'lift clip off paper',-math.pi/2)
        dest=(.32,-.23) if mode=='clipslot' else dish
        h=slot_top+.035 if mode=='clipslot' else .065
        L.add(2,(*dest,.19),C,'carry removed clip',-math.pi/2)
        L.add(1,(*dest,h),C,'lower removed clip',-math.pi/2)
        L.add(.6,(*dest,h),O,'put clip down',-math.pi/2)
        L.add(1,(*dest,.19),O,'left clear',-math.pi/2)
    R.until(L.time)
    if mode in ('cliponly','clipslot'):return {'left':L.wait(3).w,'right':R.wait(3).w}
    R.add(2,(.22,-.08,.192),C,'raise paper above shredder',math.pi/2)
    R.add(2,(.32,-.36,.192),C,'carry paper to slot',math.pi/2)
    R.add(1.5,(.32,-.36,.164),C,'feed paper edge first',math.pi/2)
    R.add(.6,(.32,-.36,.164),O,'let rollers take paper',math.pi/2)
    R.add(1,(.32,-.36,.20),O,'right up',math.pi/2)
    R.add(1.5,(.22,-.33,.19),O,'right clear',math.pi/2)
    R.wait(5); L.until(R.time)
    return {'left':L.w,'right':R.w}
