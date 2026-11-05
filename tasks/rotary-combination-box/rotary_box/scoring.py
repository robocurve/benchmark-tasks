"""Measured angles, retained settings and physical-fixture integrity."""
import math
import numpy as np

PITCH=2*math.pi/10
TOLERANCE=math.radians(5)

def error(angle,digit):
    return (angle-digit*PITCH+math.pi)%(2*math.pi)-math.pi

class History:
    def __init__(self,code,centers):
        self.code=tuple(code)
        self.centers=np.asarray(centers)
        self.held=[0]*len(code)
        self.verified=[False]*len(code)
        self.invalid=[False]*len(code)
        self.angles=[0.]*len(code)

    def update(self,angles,positions,tilts,speeds,clear):
        self.angles=list(angles)
        for i,(angle,digit) in enumerate(zip(angles,self.code)):
            if np.linalg.norm(np.asarray(positions[i])-self.centers[i])>.005 or abs(tilts[i])>math.radians(5):
                self.invalid[i]=True
            correct=abs(error(angle,digit))<=TOLERANCE and not self.invalid[i]
            if not correct:self.verified[i]=False
            if correct and abs(speeds[i])<.1 and clear[i]:self.held[i]+=1
            else:self.held[i]=0
            if self.held[i]>=20:self.verified[i]=True

    def wheel_score(self):
        return .2*sum(v and not bad and abs(error(a,d))<=TOLERANCE for v,bad,a,d in zip(self.verified,self.invalid,self.angles,self.code))

    def update_lid(self,opening,position_error,tilt_error,unlocked):
        if not hasattr(self,'bypassed'):
            self.bypassed=False;self.opened=False
        if position_error>.005 or tilt_error>math.radians(5) or (opening>math.radians(5) and not unlocked):self.bypassed=True
        self.opened=bool(unlocked and opening>=math.radians(60) and not self.bypassed)

    def score(self):return self.wheel_score()+(.4 if getattr(self,'opened',False) else 0.)
