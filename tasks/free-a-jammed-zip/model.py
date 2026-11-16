"""Reduced zip-slider contact and flexible caught lining.

The slider follows its sewn track. The trapped cloth is pinched by its tapered
entrance: forward travel narrows the gap and increases friction; reversing the
slider opens it. Cloth motion comes from gravity, stretch and measured grips.
No demo phase, requested action, or score can release the snag.
"""
import numpy as np
from common import Soft, Grips, sheet, TABLE

X=.34
Z=TABLE+.038
START=.035
LOW=-.090
HIGH=.110

class Model:
 def __init__(self):
  p,e,k,self.faces=sheet((X-.070,.043,Z+.002),(.140,.060),13,7)
  self.soft=Soft(p,e,k,.0004,floor=Z)
  self.initial=p.copy();self.tip=np.arange(12*7,13*7)
  self.slider=START;self.velocity=0.;self.pull=Grips();self.events=[]
  self.time=0.;self.release=0.;self.failed=False;self.damage=0.
  self.clear=False;self.checked_low=False;self.checked_high=False
  self.jam_force=0.;self.max_pull=0.
 def pull_point(self):return np.array([X,self.slider-.012,Z+.011])
 def contacts(self,p,slider=None):
  y=self.slider if slider is None else slider
  # The lining sits at the left entrance; both longitudinal clearance and
  # lateral extraction are necessary. Effective double-layer thickness .8 mm.
  along=p[:,1]-y
  inside=(p[:,0]>X-.006)&(p[:,0]<X+.006)&(along>-.003)&(along<.018)
  gap=.00035+np.maximum(along,0.)*.055
  compression=np.maximum(0.,.0008-gap)
  return inside,compression
 def update(self,hands,dt=.05):
  self.time+=dt
  self.pull.update(np.array([self.pull_point()]),hands)
  others={s:h for s,h in hands.items() if s not in self.pull.held}
  self.soft.grips.update(self.soft.p,others)
  # Cloth at the two remote corners is sewn to the lining of the washbag.
  pins={0:self.initial[0],6:self.initial[6]}
  grip_goals=self.soft.grips.targets(others)
  inside,compression=self.contacts(self.soft.p)
  self.jam_force=float(np.sum(compression[inside])*2400.)
  targets=self.pull.targets(hands)
  if targets:
   desired=list(targets.values())[0][1]+.012
   force=float(np.clip((desired-self.slider)*160.,-8.,8.))
   self.max_pull=max(self.max_pull,abs(force))
   # Coulomb resistance from compressed lining. Backing away unloads the wedge.
   friction=.18+self.jam_force*(.85 if force>0 else .20)
   net=np.sign(force)*max(0.,abs(force)-friction)
   self.velocity=float(np.clip(self.velocity+dt*(net-5.*self.velocity)/.035,-.12,.12))
   self.slider=float(np.clip(self.slider+self.velocity*dt,LOW,HIGH))
   self.damage+=max(0.,abs(force)-6.5)*dt if inside.any() else 0.
  else:self.velocity*=.1
  trapped,compression=self.contacts(self.soft.p)
  old=self.soft.p.copy()
  def contact(p):
   hit,c=self.contacts(p)
   # Friction resists lateral escape while compressed; it is finite, not a weld.
   strength=np.clip(c*3500.,0.,.96)
   p[hit,0]+=(old[hit,0]-p[hit,0])*strength[hit]
   p[hit,2]=np.maximum(p[hit,2],Z+.0004)
  self.soft.step(others,dt,collide=contact,pins=pins,substeps=4,iterations=8)
  # A safe re-zip check must occur after the lining has physically cleared.
  extracted=bool(np.max(self.soft.p[self.tip,0])<X-.008)
  hit,compression=self.contacts(self.soft.p)
  free=not bool(np.any(hit & (compression>0)))
  self.clear=free
  if extracted and self.slider<START-.025:self.checked_low=True
  if free and self.checked_low and self.slider>START+.045:self.checked_high=True
  self.failed=self.failed or self.soft.failed or self.damage>.35
  self.release=self.release+dt if not self.pull.held and not self.soft.grips.held else 0.
 def measure(self):
  return dict(lining_free=self.clear,slider_y_m=self.slider,checked_below_snag=self.checked_low,
              checked_above_snag=self.checked_high,released_seconds=self.release,
              pinching_force_n=self.jam_force,failed=self.failed)
 def score(self):
  if self.failed or not self.clear:return 0.
  if self.checked_high:return 1. if self.release>=2. else .75
  return .5
 def state(self):return dict(points=self.soft.p,slider=self.slider,held={s:g['index'] for s,g in self.soft.grips.held.items()},slider_held=list(self.pull.held))
 def draw(self,d):
  d.mesh('yellow_lining',self.soft.p,self.faces,(.95,.68,.12))
  for side in [-1,1]:
   label='left' if side<0 else 'right'
   d.box('tape_'+label,(X+side*.010,.01,Z),(.012,.24,.001),(.15,.17,.20))
   for i,y in enumerate(np.arange(LOW,HIGH+.002,.004)):
    off=.003 if y<=self.slider else .007
    d.box('tooth_%s_%d'%(label,i),(X+side*off,y,Z+.001),(.004,.002,.002),(.65,.66,.68))
  d.box('slider_body',(X,self.slider,Z+.004),(.010,.013,.006),(.73,.75,.78))
  d.box('pull_tab',self.pull_point(),(.008,.025,.002),(.78,.79,.82))
