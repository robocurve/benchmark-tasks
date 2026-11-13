"""Selective extraction scorer. The other chips must stay in the pouch."""
from dataclasses import dataclass
import math
import numpy as np
import design as d


@dataclass
class Sample:
    t: float
    packet_pos: tuple
    packet_quat: tuple
    chip_pos: tuple
    chip_quat: tuple
    chip_speed: tuple
    chip_spin: tuple
    hand_clearance: float


class Score:
    def __init__(self,table=.7475):
        self.table=table
        self.start=np.array((*d.PACKET_XY,table+.002))
        self.last_t=None
        self.initialized=False
        self.invalid=False
        self.extracted=set()
        self.departed=set()
        self.stable_since=None
        self.value=0.
        self.reason='not extracted'

    def update(self,s):
        try:
            pp=np.asarray(s.packet_pos,dtype=float)
            cp=np.asarray(s.chip_pos,dtype=float)
            speeds=np.asarray(s.chip_speed,dtype=float)
            spins=np.asarray(s.chip_spin,dtype=float)
            if pp.shape!=(3,) or cp.shape!=(3,3) or speeds.shape!=(3,) or spins.shape!=(3,):
                raise ValueError('invalid shape')
            values=np.r_[s.t,pp,cp.ravel(),speeds,spins,s.hand_clearance]
            if not np.isfinite(values).all() or s.t<0 or np.any(speeds<0) or np.any(spins<0) or s.hand_clearance<0:
                raise ValueError('invalid magnitude')
            if self.last_t is not None and abs(s.t-self.last_t-.05)>1e-4:
                raise ValueError('invalid time')
            pr=d.rotation(s.packet_quat)
            if len(s.chip_quat)!=3: raise ValueError('invalid quaternion count')
            all_corners=[d.corners(p,q) for p,q in zip(cp,s.chip_quat)]
        except (ValueError,TypeError):
            self.invalid=True;self.value=0.;self.reason='invalid telemetry';return 0.
        self.last_t=s.t
        if self.invalid:return 0.
        if np.linalg.norm(pp-self.start)>.030 or pr[2,2]<math.cos(math.radians(15)):
            self.invalid=True;self.value=0.;self.reason='packet tipped or displaced';return 0.
        local=[(c-pp)@pr for c in all_corners]
        inside=[bool((np.abs(c[:,:2])<=[.044,.034]).all() and c[:,2].min()>=-.002 and c[:,2].max()<=.065) for c in local]
        if not self.initialized:
            if not all(inside):
                self.invalid=True;self.value=0.;self.reason='chips did not start inside';return 0.
            self.initialized=True
        if min(c[:,2].min() for c in all_corners)<self.table-.002:
            self.invalid=True;self.value=0.;self.reason='chip dropped below table';return 0.
        for i,c in enumerate(local):
            # Clear the entire top rim before moving sideways. A chip pushed
            # through a side wall or poured out does not establish extraction.
            if c[:,2].min()>=.065:self.extracted.add(i)
        self.departed.update(i for i,contained in enumerate(inside) if not contained)
        if len(self.departed)>1:
            self.invalid=True;self.value=0.;self.reason="more than one chip left the packet";return 0.
        candidates=[]
        for i in range(3):
            others=all(inside[j] for j in range(3) if j!=i)
            if i in self.extracted and others:candidates.append(i)
        self.value=.5 if candidates else 0.
        self.reason='extracted' if candidates else 'not extracted or other chips disturbed'
        good=False
        for i in candidates:
            c=all_corners[i]
            contained=bool((np.abs(c[:,:2]-d.PLATE_XY)<=[.047,.047]).all())
            resting=.003<=c[:,2].min()-self.table<=.006
            if contained and resting and speeds[i]<=.005 and spins[i]<=.1 and s.hand_clearance>=.060:
                good=True
        if good:
            if self.stable_since is None:self.stable_since=s.t
            if s.t-self.stable_since>=1.-1e-8:self.value=1.;self.reason='released on plate'
        else:self.stable_since=None
        return self.value
