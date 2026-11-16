"""Flexible webbing through a finite sliding-tongue slot.

Webbing is a mass-point ribbon with stretch/shear/bending constraints. The four
slot bars exclude cloth geometrically. Initial static slot friction retains the
twist until the tongue is moved; subsequent cloth motion is constraint/contact
resolution, not a prescribed untwisting animation.
"""
import numpy as np
from common import Soft,Grips,TABLE

CENTER=np.array([.32,0.,TABLE+.040])
ROWS=41
COLS=7
WIDTH=.047
LENGTH=.50
SPAN=.44

class Model:
 def __init__(self):
  ys=np.linspace(-SPAN/2,SPAN/2,ROWS);xs=np.linspace(-WIDTH/2,WIDTH/2,COLS)
  angle=np.pi*np.exp(-(ys/.085)**4)
  p=np.array([CENTER+[x*np.cos(a),y,x*np.sin(a)] for y,a in zip(ys,angle) for x in xs])
  flat=np.array([[x,y,0.] for y in np.linspace(-LENGTH/2,LENGTH/2,ROWS) for x in xs]);edges=[];k=[];self.faces=[]
  for i in range(ROWS):
   for j in range(COLS):
    a=i*COLS+j
    for di,dj,st in [(1,0,1.),(0,1,1.),(1,1,.9),(1,-1,.9),(2,0,.12),(0,2,.4)]:
     if i+di<ROWS and 0<=j+dj<COLS:edges.append((a,(i+di)*COLS+j+dj));k.append(st)
    if i+1<ROWS and j+1<COLS:self.faces.append((a,a+1,a+COLS+1,a+COLS))
  self.soft=Soft(p,edges,k,.0006);self.soft.rest=np.linalg.norm(flat[self.soft.edges[:,1]]-flat[self.soft.edges[:,0]],axis=1)
  self.initial=p.copy();self.plate=CENTER.copy();self.plate_grips=Grips();self.events=[]
  self.static_contact=True;self.time=0.;self.release=0.;self.failed=False
  self.plate_pull=np.array([.037,0.,.006]);self.old_plate=self.plate.copy()
  pins={i:self.initial[i] for i in list(range(COLS))+list(range((ROWS-1)*COLS,ROWS*COLS))+list(range((ROWS//2)*COLS,(ROWS//2+1)*COLS))}
  # Deterministic passive reset settling. The trial starts after this, with no
  # robot contact and the twist still trapped at the tongue.
  for _ in range(60):self.soft.step({},.05,collide=self.contact,pins=pins,substeps=4,iterations=12)
 def pull_point(self):return self.plate+self.plate_pull
 def contact(self,p):
  # Tongue plane is XZ; 49 x 3.5 mm opening, 1.5 mm steel thickness.
  q=p-self.plate
  slab=abs(q[:,1])<.00135
  solid=(abs(q[:,0])<.034)&(abs(q[:,2])<.026)&((abs(q[:,0])>.0245)|(abs(q[:,2])>.00175))
  hit=slab&solid
  sign=np.where(q[:,1]>=0,1.,-1.)
  p[hit,1]=self.plate[1]+sign[hit]*.00135
  # Continuous edge/slot intersections: testing only particle centres lets a
  # thin plate pass between rows. Both anchored ends remain threaded, so every
  # longitudinal crossing must stay within the finite opening.
  i=np.arange((ROWS-1)*COLS);j=i+COLS
  a=p[i];b=p[j];den=b[:,1]-a[:,1]
  t=(self.plate[1]-a[:,1])/np.where(abs(den)>1e-10,den,1.)
  hit=(abs(den)>1e-10)&(t>=0)&(t<=1)
  i=i[hit];j=j[hit];t=t[hit];cross=p[i]*(1-t[:,None])+p[j]*t[:,None]
  goal=cross.copy();goal[:,0]=np.clip(goal[:,0],self.plate[0]-.0239,self.plate[0]+.0239)
  goal[:,2]=np.clip(goal[:,2],self.plate[2]-.00115,self.plate[2]+.00115)
  corr=(goal-cross)/np.maximum((1-t)**2+t*t,1e-10)[:,None]
  np.add.at(p,i,corr*(1-t[:,None]));np.add.at(p,j,corr*t[:,None])
  # Coarse cloth self-contact. Adjacent material points are excluded.
  from scipy.spatial import cKDTree
  pairs=cKDTree(p).query_pairs(.0012,output_type='ndarray')
  if len(pairs):
   i,j=pairs.T;near=(abs(i//COLS-j//COLS)<=1)&(abs(i%COLS-j%COLS)<=1)
   i=i[~near];j=j[~near];delta=p[j]-p[i];length=np.linalg.norm(delta,axis=1)
   v=delta*((.0012-length)/np.maximum(length,1e-10)*.5)[:,None]
   np.add.at(p,i,-v);np.add.at(p,j,v)
 def update(self,hands,dt=.05):
  self.time+=dt;self.plate_grips.update(np.array([self.pull_point()]),hands)
  goals=self.plate_grips.targets(hands);others={s:h for s,h in hands.items() if s not in self.plate_grips.held}
  pins={i:self.initial[i] for i in list(range(COLS))+list(range((ROWS-1)*COLS,ROWS*COLS))}
  self.old_plate=self.plate.copy()
  if goals:
   target=goals[0]-self.plate_pull
   # Webbing keeps the tongue near its plane, but it can slide along its length.
   delta=target-self.plate
   self.plate[1]+=float(np.clip(delta[1],-.001,.001))
   self.plate[0]+=float(np.clip(delta[0],-.0005,.0005))
   self.plate[2]+=float(np.clip(delta[2],-.0005,.0005))
   if np.linalg.norm(self.plate-CENTER)>.002:self.static_contact=False
  if self.static_contact:
   for i in range((ROWS//2)*COLS,(ROWS//2+1)*COLS):pins[i]=self.initial[i]
  self.soft.step(others,dt,collide=self.contact,pins=pins,substeps=8,iterations=16)
  self.failed=self.failed or self.soft.failed
  self.release=self.release+dt if not self.plate_grips.held and not self.soft.grips.held else 0.
 def measure(self):
  p=self.soft.p.reshape(ROWS,COLS,3);director=p[:,-1]-p[:,0]
  angles=np.unwrap(np.arctan2(director[:,2],director[:,0]))
  widths=np.linalg.norm(director,axis=1)
  twist=float(np.max(abs(angles))*180/np.pi)
  # A folded ribbon is not a successful flat belt even if its director is small.
  flat=bool(np.min(widths)>.040 and twist<15.)
  centres=p.mean(axis=1);j=int(np.argmin(abs(centres[:,1]-self.plate[1])))
  retained=bool(centres[:,1].min()<self.plate[1]<centres[:,1].max() and abs(centres[j,0]-self.plate[0])<.0245 and abs(centres[j,2]-self.plate[2])<.004)
  return dict(maximum_twist_degrees=twist,minimum_width_mm=float(widths.min()*1000),flat=flat,tongue_retained=retained,released_seconds=self.release,failed=self.failed)
 def score(self):
  m=self.measure()
  if self.failed or not m['tongue_retained']:return 0.
  if m['flat']:return 1. if self.release>=3. else .75
  return .5 if m['maximum_twist_degrees']<90 else 0.
 def state(self):return dict(points=self.soft.p,plate=self.plate,static_contact=self.static_contact,held={s:g['index'] for s,g in self.soft.grips.held.items()},plate_held=list(self.plate_grips.held))
 def draw(self,d):
  d.mesh('webbing',self.soft.p,self.faces,(.20,.23,.26))
  p=self.soft.p.reshape(ROWS,COLS,3)
  d.tube('blue_edge',p[:,0],.0007,(.15,.55,.95),6)
  d.tube('yellow_edge',p[:,-1],.0007,(.95,.75,.15),6)
  for sign in [-1,1]:
   label='a' if sign<0 else 'b'
   d.box('slot_side_'+label,self.plate+[sign*.02925,0,0],(.0095,.0015,.052),(.70,.72,.75))
   d.box('slot_top_'+label,self.plate+[0,0,sign*.013875],(.049,.0015,.02425),(.70,.72,.75))
  d.box('tongue_grip',self.pull_point(),(.016,.003,.020),(.68,.70,.73))
