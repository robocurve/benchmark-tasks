"""Scorer independent of the simulator and of any supposed pivot-unlocked flag."""
from collections import deque
from dataclasses import dataclass
import math
import numpy as np
import geometry as g

@dataclass
class Observation:
    time_s: float
    positions: np.ndarray
    rotations: np.ndarray
    fingertips: np.ndarray
    hand_centers: np.ndarray
    hand_radii: np.ndarray
    damage: bool=False

class Scorer:
    def __init__(self,top):
        self.top=float(top); self.local=[g.vertices(i) for i in range(3)]
        self.reset()
    def reset(self):
        self.last=None; self.history=deque(maxlen=21); self.violations=set()
        self.result=0.; self.reason='not observed'
    def update(self,o):
        self.result=0.
        try: self._update(o)
        except (ValueError,TypeError,IndexError,FloatingPointError) as e:
            self.violations.add('invalid observation'); self.reason=str(e)
        return self.result
    def _update(self,o):
        for a,shape in ((o.positions,(3,3)),(o.rotations,(3,3,3)),(o.fingertips,(4,3))):
            if np.shape(a)!=shape or not np.isfinite(a).all(): raise ValueError('invalid pose or tips')
        for r in o.rotations:
            if not np.allclose(r.T@r,np.eye(3),atol=1e-5) or abs(np.linalg.det(r)-1)>1e-5:
                raise ValueError('nonrigid rotation')
        if (not math.isfinite(o.time_s) or o.time_s<0 or self.last is not None and abs(o.time_s-self.last.time_s-.05)>.001):
            raise ValueError('requires consecutive 20Hz measurements')
        if (np.ndim(o.hand_centers)!=2 or o.hand_centers.shape[1]!=3 or len(o.hand_centers)<2 or
            o.hand_radii.shape!=(len(o.hand_centers),) or not np.isfinite(o.hand_centers).all() or
            not np.isfinite(o.hand_radii).all() or (o.hand_radii<0).any()): raise ValueError('invalid hands')
        verts=[g.world(self.local[i],o.positions[i],o.rotations[i]) for i in range(3)]
        if o.damage: self.violations.add('fracture or permanent deformation over 1mm')
        for i in range(2):
            distances=g.metal_distance(np.vstack((o.fingertips,o.hand_centers)),o.positions[i],o.rotations[i],i)
            tipdist=distances[:4]
            handdist=distances[4:]-o.hand_radii
            if min(float(tipdist.min()),float(handdist.min()))<=0:
                self.violations.add('hand contacts metal outside molded handles')
        self.history.append(np.concatenate(verts)); self.last=o
        if self.violations: self.reason='; '.join(sorted(self.violations)); return
        if len(self.history)<21: self.reason='less than 1s'; return
        h=np.asarray(self.history)
        for j in range(20):
            if np.max(np.linalg.norm(h[j+1:]-h[j],axis=-1))>=.001:
                self.reason='at least 1mm motion in final second'; return
        for i in range(3):
            if np.min(g.bbox_distance(o.fingertips,o.positions[i],o.rotations[i],self.local[i]))<.080:
                self.reason='fingertips under 80mm clear'; return
            if np.min(g.bbox_distance(o.hand_centers,o.positions[i],o.rotations[i],self.local[i])-o.hand_radii)<.002:
                self.reason='hand may support object'; return
        # A 5mm gap between complete overhead footprints guarantees independent
        # halves, regardless of angle or execution sequence; no historical credit.
        lo=[v[:,:2].min(0) for v in verts[:2]]; hi=[v[:,:2].max(0) for v in verts[:2]]
        gap=max(float(np.max(lo[1]-hi[0])),float(np.max(lo[0]-hi[1])))
        if gap<.005: self.reason='halves not independently separated'; return
        if o.rotations[2][2,2]<math.cos(math.radians(5)) or abs(verts[2][:,2].min()-self.top)>.002:
            self.reason='tray not upright on table'; return
        inside=[]; supported=[]; flat=[]
        for i in range(2):
            v=(verts[i]-o.positions[2])@o.rotations[2]
            within=bool(np.all(np.abs(v[:,:2])<=g.TRAY_SIZE[:2]/2-g.WALL-.003))
            onfloor=abs(float(v[:,2].min())-g.FLOOR)<=.002
            offtray=any(verts[i][:,a].max()<verts[2][:,a].min()-.002 or
                        verts[i][:,a].min()>verts[2][:,a].max()+.002 for a in (0,1))
            ontable=abs(float(verts[i][:,2].min())-self.top)<=.002
            inside.append(within and onfloor)
            supported.append((within and onfloor) or (offtray and ontable))
            flat.append(abs(o.rotations[i][2,2])>=math.cos(math.radians(5)))
        if not all(supported): self.reason='half floating or propped on tray edge'; return
        self.result=1. if all(inside) and all(flat) else .5
        self.reason='both detached halves in tray' if self.result==1 else 'detached halves independently set down'
