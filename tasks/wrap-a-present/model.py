"""Pre-creased gift paper: eight passive fold hinges and contact-set tape.

The weighted box holds the centre of a cross-cut paper template. Each flap has
one side fold and one top fold. Panels preserve their shape between creases;
this deliberately omits crumpling and arbitrary new creases. Grasp/contact
inputs are measured hands. There are no scripted object animations.
"""
import numpy as np
from common import Grips,TABLE
BOX=np.array([.35,0,TABLE+.020]);HEIGHT=.040
class Model:
 def __init__(self):
  self.base=np.array([[.35,.080,TABLE+.0004],[.35,-.080,TABLE+.0004],[.410,0,TABLE+.0004],[.290,0,TABLE+.0004]])
  self.out=np.array([[0,1,0],[0,-1,0],[1,0,0],[-1,0,0]],float);self.width=[.120,.120,.160,.160];self.length=[.100,.100,.080,.080]
  self.angles=np.tile([0.,.30],(4,1));self.rest_angles=np.tile([0.,.45],(4,1));self.vel=np.zeros((4,2));self.grips=Grips();self.events=self.grips.events;self.failed=False;self.time=0.;self.release=0.
  self.tape=np.array([[.22,.14,TABLE+.001],[.22,.24,TABLE+.001]]);self.tape_R=np.tile(np.eye(3),(2,1,1));self.tape_offsets=[np.array([[0,0,0],[.005,0,0],[-.005,0,0],[0,.018,0],[0,-.018,0]]),np.array([[0,0,0],[.018,0,0],[-.018,0,0],[0,.005,0],[0,-.005,0]])];self.relR={};self.bonded=np.zeros(2,bool);self.press_time=np.zeros(2)
 def joints(self,k):
  a,b=self.angles[k];v=self.out[k];p=self.base[k];q=p+v*HEIGHT*np.cos(a)+[0,0,HEIGHT*np.sin(a)];r=q+v*self.length[k]*np.cos(b)+[0,0,self.length[k]*np.sin(b)];return np.array([p,q,r])
 def points(self):
  return np.array([self.joints(k)[-1]+np.cross(self.out[k],[0,0,1])*self.width[k]*t for k in range(4) for t in [-.5,-.25,0,.25,.5]]+[self.tape[j]+self.tape_R[j]@offset for j in range(2) for offset in self.tape_offsets[j]])
 def update(self,hands,dt=.05):
  self.time+=dt;old=set(self.grips.held);self.grips.update(self.points(),hands)
  for s in set(self.grips.held)-old:
   i=self.grips.held[s]['index']
   if i>=20:self.relR[s]=hands[s]['rotation'].T@self.tape_R[(i-20)//5]
  raw_goals=self.grips.targets(hands);goals={}
  for i,point in raw_goals.items():
   if i<20:
    k=i//5;t=[-.5,-.25,0,.25,.5][i%5];goals[k]=point-np.cross(self.out[k],[0,0,1])*self.width[k]*t
   else:goals[i]=point
  for k in range(4):
   a,b=self.angles[k];target=None
   if k in goals:
    d=goals[k]-self.base[k];r=float(d@self.out[k]);z=float(d[2]);l=HEIGHT;m=self.length[k]
    if abs(d@np.cross(self.out[k],[0,0,1]))>.045 or np.hypot(r,z)>l+m+.025:self.failed=True
    c=np.clip((r*r+z*z-l*l-m*m)/(2*l*m),-1,1);delta=np.arccos(c)
    candidates=[]
    for bend in [-delta,delta]:
     aa=np.arctan2(z,r)-np.arctan2(m*np.sin(bend),l+m*np.cos(bend));bb=aa+bend
     aa=aa%(2*np.pi);bb=bb%(2*np.pi)
     if aa>np.pi:aa-=2*np.pi
     if bb>np.pi*1.5:bb-=2*np.pi
     candidates.append(np.array([aa,bb]))
    target=min(candidates,key=lambda v:np.linalg.norm(v-self.angles[k])+10*max(0,-v[0])+10*max(0,v[0]-np.pi/2)+10*max(0,-v[1]))
   pair=0 if k<2 else 1
   for _ in range(10):
    a,b=self.angles[k];torque=-np.array([.018*np.cos(a),.012*np.cos(b)])-.06*self.vel[k]
    torque+=.08*(self.rest_angles[k]-self.angles[k])
    if target is not None:
     torque+=.8*(target-self.angles[k])
     # A loaded paper crease takes a set; gravity alone does not reset it.
     error=self.angles[k]-self.rest_angles[k];self.rest_angles[k]+=np.sign(error)*np.maximum(abs(error)-.10,0.)
    if self.bonded[pair]:torque+=1.2*(np.array([np.pi/2,np.pi])-self.angles[k])
    self.vel[k]+=torque/.003*(dt/10);self.angles[k]+=self.vel[k]*(dt/10)
    # Table and box are unilateral contacts. Crease 1 meets the box side;
    # inward crease 2 rests on the box top, rather than falling through it.
    low=np.array([0.,0.]);high=np.array([np.pi/2,np.pi])
    hit=(self.angles[k]<low)|(self.angles[k]>high);self.angles[k]=np.clip(self.angles[k],low,high);self.vel[k,hit]=0.
    if self.angles[k,1]>np.pi/2:
     self.angles[k,0]=np.pi/2;self.vel[k,0]=0.
  # Unilateral finger-pad contact presses an overlapping flap onto the box.
  # The pad position is measured from the native finger geometry, not a demo phase.
  for k in range(4):
   if self.angles[k,0]<np.radians(80) or self.angles[k,1]<=np.pi/2:continue
   q=self.joints(k)[1];across=np.cross(self.out[k],[0,0,1])
   for h in hands.values():
    for sign in [-1,1]:
     pad=h['tcp']+h['rotation']@np.array([sign*h['opening']*.094/2,0,.01284])
     inward=float((q-pad)@self.out[k]);side=abs(float((pad-q)@across));b=self.angles[k,1]
     if .005<inward<-self.length[k]*np.cos(b)+.003 and side<=self.width[k]/2+.003 and q[2]-.003<=pad[2]<=q[2]+.025:
      height=q[2]-inward*np.tan(b)
      if pad[2]<height:
       self.angles[k,1]=max(b,np.pi-np.arctan2(max(0.,pad[2]-q[2]),inward));self.vel[k,1]=0.
       self.rest_angles[k,1]=max(self.rest_angles[k,1],self.angles[k,1]-.10)
  for s,g in self.grips.held.items():
   i=g['index']
   if i>=20 and not self.bonded[(i-20)//5]:
    j=(i-20)//5;self.tape_R[j]=hands[s]['rotation']@self.relR[s]
    self.tape[j]=goals[i]-self.tape_R[j]@self.tape_offsets[j][(i-20)%5]
  for j in range(2):
   if self.bonded[j]:continue
   held=any(g['index']>=20 and (g['index']-20)//5==j for g in self.grips.held.values())
   if not held:self.tape[j,2]=max(TABLE+.001,self.tape[j,2]-dt*.10)
   pair=range(0,2) if j==0 else range(2,4)
   closed=all(self.angles[k,0]>np.radians(80) and self.angles[k,1]>np.radians(165) for k in pair)
   # A 40x12 mm strip must bridge both flap edges at the top centre.
   flat=abs(self.tape_R[j,2,2])>np.cos(np.radians(15))
   near=abs(self.tape[j,2]-(TABLE+HEIGHT+.002))<.004
   # Tape must cross the visible edge of the upper flap, not merely sit on
   # top of a region where two sheets happen to overlap underneath it.
   def surface(k,point):
    q=self.joints(k)[1];b=self.angles[k,1];along=float((point-q)@self.out[k])/min(np.cos(b),-1e-6)
    side=abs(float((point-q)@np.cross(self.out[k],[0,0,1])))
    return 0<=along<=self.length[k] and side<=self.width[k]/2,q[2]+along*np.sin(b)
   axis=self.tape_R[j,:,1 if j==0 else 0];contact=False;lower,upper=list(pair)
   for sign in [-1,1]:
    lo=self.tape[j]+sign*.012*axis;hi=self.tape[j]-sign*.012*axis
    in_lower,zlo=surface(lower,lo);in_hidden,_=surface(upper,lo);in_upper,zhi=surface(upper,hi)
    contact=contact or (in_lower and not in_hidden and in_upper and abs(lo[2]-zlo)<.004 and abs(hi[2]-zhi)<.004)
   self.press_time[j]=self.press_time[j]+dt if closed and flat and near and contact and held else 0.
   if self.press_time[j]>=.4:
    self.bonded[j]=True;self.tape[j,2]=TABLE+HEIGHT+.002+j*.0006;self.events.append(dict(event='tape_bond',strip=j,time=self.time))
  self.release=self.release+dt if not self.grips.held else 0.
 def measure(self):return dict(fold_degrees=np.degrees(self.angles).tolist(),secured_pairs=int(self.bonded.sum()),released_seconds=self.release,failed=self.failed)
 def score(self):
  if self.failed:return 0.
  if self.bonded.all() and self.release>=3:return 1.
  return .25*int(self.bonded.sum())
 def state(self):return dict(angles=self.angles,tape=self.tape,bonded=self.bonded,held={s:g['index'] for s,g in self.grips.held.items()})
 def draw(self,d):
  d.box('paper_under_box',[.35,0,TABLE+.0002],[.120,.160,.0004],(.18,.48,.68))
  for k in range(4):
   p=self.joints(k);across=np.cross(self.out[k],[0,0,1])*self.width[k]/2;verts=np.array([q+s*across for q in p for s in [-1,1]])
   # Render paper just outside a coplanar box face; physics uses the original fold.
   if self.angles[k,0]>np.radians(89.9):verts[:4]+=self.out[k]*.0006
   d.mesh('paper_flap_'+str(k),verts,[(0,1,3,2),(2,3,5,4)],(.18,.48,.68))
  for j in range(2):
   size=[.012,.040,.0007] if j==0 else [.040,.012,.0007]
   d.box('tape_'+str(j),self.tape[j],size,(.91,.87,.66),self.tape_R[j])
