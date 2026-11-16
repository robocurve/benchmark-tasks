"""Measured-hand spoon mechanics, finite liquid and a care-training mouth.

The rigid spoon has a passive finite pinch attachment and gravity when free.
All geometry is life size. The liquid is modeled separately, never attached
to the spoon or teleported when the utensil reaches a target.
"""
import numpy as np
from common import TABLE,Grips
from liquid import Liquid,A,B,DEPTH

REST=np.array([.23,-.15,TABLE+.065])
MOUTH=np.array([.39,0.,TABLE+.15])
GRIP_LOCAL=np.array([-.090,0.,-.004])

def ellipsoid(center,axes,n=20,m=12):
 p=[]
 for i in range(m+1):
  t=np.pi*i/m
  for j in range(n):
   a=2*np.pi*j/n;p.append(np.asarray(center)+np.asarray(axes)*[np.sin(t)*np.cos(a),np.sin(t)*np.sin(a),np.cos(t)])
 f=[(i*n+j,i*n+(j+1)%n,(i+1)*n+(j+1)%n,(i+1)*n+j) for i in range(m) for j in range(n)]
 return np.array(p),f

class Model:
 def __init__(self):
  self.center=REST.copy();self.R=np.eye(3);self.velocity=np.zeros(3);self.grip=Grips();self.rel={};self.previous_opening={}
  self.liquid=Liquid();self.events=self.grip.events;self.time=0.;self.failed=False;self.reasons=[];self.release=0.;self.max_face_penetration=0.;self.max_near_speed=0.;self.previous_tip=None
  self.far_arm_contact=False;self.insertion=0.
  theta=np.arange(40)*2*np.pi/40
  self.bowl_local=np.vstack([np.c_[r*A*np.cos(theta),r*B*np.sin(theta),np.full(40,-DEPTH*(1-r*r))] for r in np.linspace(.0,1.,9)])
  self.bowl_faces=[(i*40+j,i*40+(j+1)%40,(i+1)*40+(j+1)%40,(i+1)*40+j) for i in range(8) for j in range(40)]
 def fail(self,reason):
  self.failed=True
  if reason not in self.reasons:self.reasons.append(reason)
 def receiver(self,before,p):
  # The throat collector lies behind an elliptical lip opening. Liquid must
  # physically cross the mouth plane; proximity of the spoon earns nothing.
  if before[0]<MOUTH[0]<=p[0]:
   a=(MOUTH[0]-before[0])/(p[0]-before[0]);q=before+a*(p-before)-MOUTH
   if (q[1]/.022)**2+(q[2]/.010)**2>=1:return 'spill'
  q=p-MOUTH;b=before-MOUTH
  if 0<=q[0]<=.045:
   # Ceiling, side and rear walls intercept liquid; pouring through the head
   # cannot bypass the lips and earn delivery credit.
   if q[2]>.010 or abs(q[1])>=.022 or b[0]>.045:return 'spill'
   if abs(q[1])<.021 and b[2]>-.010>=q[2]:return 'delivered' 
  if p[2]<TABLE+.001:return 'spill'
  if p[0]>MOUTH[0]+.06 or abs(p[1])>.6:return 'spill'
  return None
 def update(self,hands,dt=.05):
  self.time+=dt;old=self.center.copy();points=np.array([self.center+self.R@GRIP_LOCAL])
  before=set(self.grip.held)
  # Closing fingers must actually surround the 28 mm padded handle.
  available={s:h for s,h in hands.items() if s in self.grip.held or
    (self.previous_opening.get(s,1)-h['opening']>.0001 and h['opening']<.32)}
  self.previous_opening={s:h['opening'] for s,h in hands.items()}
  self.grip.update(points,available)
  for s in set(self.grip.held)-before:self.rel[s]=hands[s]['rotation'].T@self.R
  if self.grip.held:
   s=next(iter(self.grip.held));h=hands[s];g=self.grip.held[s]
   self.R=h['rotation']@self.rel[s];self.center=h['tcp']+h['rotation']@g['local']-self.R@GRIP_LOCAL
   if np.linalg.norm(self.center-old)>dt*1.2:self.fail('spoon moved faster than supported grip model')
   self.release=0.
  else:
   self.velocity[2]-=9.81*dt;self.center+=self.velocity*dt
   over=np.linalg.norm((self.center-REST)[:2])<.025
   floor=REST[2] if over else TABLE+DEPTH
   lowest=self.center[2]-DEPTH
   if lowest<floor-DEPTH:
    self.center[2]=floor;self.velocity[:]=0.
   self.release+=dt
  self.velocity=(self.center-old)/dt
  pts=self.bowl_local@self.R.T+self.center
  # A shallow insertion is allowed between the lips; nose/cheek/lip impacts
  # are latched failures. Full-scale mouth, not an oversized receiving pit.
  q=pts-MOUTH;front=q[:,0]>.001
  opening=(q[:,1]/.022)**2+(q[:,2]/.010)**2<1
  penetration=float(np.max(np.maximum(q[front & ~opening,0],0),initial=0.))
  self.max_face_penetration=max(self.max_face_penetration,penetration)
  self.insertion=max(self.insertion,float(np.max(q[:,0],initial=0.)))
  if penetration>.002 or self.insertion>.018:self.fail('spoon contacted face or entered mouth too deeply')
  import itertools
  for center,size in [(GRIP_LOCAL,[.100,.028,.012]),(np.array([-.039,0,-.005]),[.040,.010,.004])]:
   tool=np.array(list(itertools.product([-1,1],repeat=3)))*np.asarray(size)/2+center
   tool=tool@self.R.T+self.center-MOUTH
   if np.any((tool[:,0]>.002)&(tool[:,0]<.120)&(abs(tool[:,1])<.080)&(tool[:,2]>-.080)&(tool[:,2]<.135)):
    self.fail('spoon handle or neck contacted head')
  tip=self.center+self.R@np.array([A,0,0])
  speed=0. if self.previous_tip is None else np.linalg.norm(tip-self.previous_tip)/dt
  self.previous_tip=tip.copy()
  if np.linalg.norm(tip-MOUTH)<.05:
   self.max_near_speed=max(self.max_near_speed,speed)
   if speed>.12:self.fail('rapid approach at the mouth')
  for side,h in hands.items():
   if h['tcp'][0]>MOUTH[0]-.015 and abs(h['tcp'][1])<.075 and abs(h['tcp'][2]-(MOUTH[2]+.035))<.10:self.fail('gripper contacted face')
  bounds=hands.get('left',{}).get('native_body_bounds',{})
  face_low=MOUTH+[-.027,-.080,-.080];face_high=MOUTH+[.12,.080,.135]
  for path,pair in bounds.items():
   lo,hi=np.asarray(pair)
   if np.all(hi>face_low) and np.all(lo<face_high):self.fail('robot collision shape entered head clearance envelope')
  for j in range(10):self.liquid.advance(old+(j+1)/10*(self.center-old),self.R,dt/10,self.receiver)
  if self.liquid.max_balance_error>1e-10:self.fail('liquid balance error')
 def measure(self):
  return dict(**self.liquid.measure(),failed=self.failed,failure_reasons=self.reasons,maximum_face_penetration_mm=self.max_face_penetration*1000,maximum_insertion_mm=self.insertion*1000,maximum_near_mouth_speed_m_s=self.max_near_speed,released_seconds=self.release,returned_to_rest=bool(np.linalg.norm(self.center-REST)<.015))
 def score(self):
  q=self.measure()
  if self.failed or q['spilled_ml']>.25:return 0.
  if q['delivered_ml']>=4.5 and q['airborne_ml']<.05 and self.release>=1 and q['returned_to_rest']:return 1.
  if q['delivered_ml']>=4.5:return .75
  if q['delivered_ml']>=3.:return .5
  if q['delivered_ml']>=1.5:return .25
  return 0.
 def state(self):return dict(spoon_center=self.center,spoon_rotation=self.R,liquid=self.liquid.measure())
 def draw(self,d):
  d.box('stand',REST+[0,0,-.027],(.085,.060,.035),(.20,.32,.38))
  d.box('handle',self.center+self.R@GRIP_LOCAL,(.100,.028,.012),(.2,.45,.8),self.R)
  d.box('neck',self.center+self.R@[-.039,0,-.005],(.040,.010,.004),(.78,.79,.82),self.R)
  d.mesh('bowl',self.bowl_local@self.R.T+self.center,self.bowl_faces,(.78,.79,.82))
  # Render connected liquid surface, not a collection of solid balls.
  from scipy.spatial import Delaunay
  if not hasattr(self,'liquid_faces'):self.liquid_faces=Delaunay(self.liquid.xy).simplices.tolist()
  wet=self.liquid.local_surface[:,2]-self.liquid.bottom>1e-5
  visible=[f for f in self.liquid_faces if wet[f].all()]
  if self.liquid.volume>1e-9:d.mesh('liquid',self.liquid.local_surface@self.R.T+self.center,visible,(.65,.15,.32))
  else:d.mesh('liquid',np.array([[0,0,0]]),[],(.65,.15,.32))
  # The front is an elliptical face annulus with an actual elliptical mouth.
  n=64;angles=np.arange(n)*2*np.pi/n;points=[]
  for r in np.linspace(0,1,12):
   yz=np.c_[((1-r)*.024+r*.075)*np.cos(angles),((1-r)*.012+r*.10)*np.sin(angles)+r*.03]
   x=MOUTH[0]+.042*r*r
   points.extend(np.c_[np.full(n,x),yz[:,0]+MOUTH[1],yz[:,1]+MOUTH[2]])
  faces=[(i*n+j,i*n+(j+1)%n,(i+1)*n+(j+1)%n,(i+1)*n+j) for i in range(11) for j in range(n)]
  d.mesh('face',points,faces,(.70,.52,.40))
  back=[]
  for t in np.linspace(0,np.pi/2,13):
   back.extend(MOUTH+np.c_[np.full(n,.042+.075*np.sin(t)),.075*np.cos(t)*np.cos(angles),.030+.10*np.cos(t)*np.sin(angles)])
  back_faces=[(i*n+j,i*n+(j+1)%n,(i+1)*n+(j+1)%n,(i+1)*n+j) for i in range(12) for j in range(n)]
  d.mesh('head_back',back,back_faces,(.70,.52,.40))
  p=MOUTH+np.c_[np.zeros(n),.024*np.cos(angles),.012*np.sin(angles)]
  d.tube('lips',np.vstack([p,p[:1]]),.002,(.58,.25,.25))
  d.box('neck_base',MOUTH+[.045,0,-.105],(.070,.075,.070),(.65,.48,.38))
  for y in [-.032,.032]:
   p,f=ellipsoid(MOUTH+[.014,y,.068],[.006,.012,.006]);d.mesh('left_eye' if y>0 else 'right_eye',p,f,(.07,.08,.10))
  p,f=ellipsoid(MOUTH+[-.008,0,.036],[.018,.012,.025]);d.mesh('nose',p,f,(.70,.52,.40))
  # Oral cavity has a curved floor and sides, not a painted target rectangle.
  p=[]
  for x in np.linspace(0,.045,12):
   p.extend(MOUTH+np.c_[np.full(n,x),.022*np.cos(angles),.010*np.sin(angles)])
  d.mesh('oral_cavity',p,faces,(.19,.045,.055))
  # Training-manikin drain and graduated collector, not a swallowing model.
  d.tube('drain',np.array([MOUTH+[.035,0,-.010],MOUTH+[.035,0,-.065],MOUTH+[.025,-.070,-.065]]),.003,(.40,.42,.45))
  collector=MOUTH+[.025,-.070,-.108]
  d.box('collector_back',collector+[.016,0,.022],(.002,.022,.044),(.82,.85,.88))
  d.box('collector_base',collector,(.034,.026,.002),(.82,.85,.88))
  for i in range(6):
   d.box('graduation'+str(i),collector+[.015,-.014,i*1e-6/(.030*.020)],(.002,.006,.0008),(.08,.08,.08))
  level=self.liquid.delivered/(.030*.020)
  if level>1e-6:d.box('collected_liquid',collector+[0,0,level/2+.001],(.030,.020,level),(.65,.15,.32))
  for i in range(max(len(self.liquid.parcels),getattr(self,'last_drop_count',0))):
   if i<len(self.liquid.parcels):
    p,v,vol=self.liquid.parcels[i];rr=(3*vol/(4*np.pi))**(1/3);pts,f=ellipsoid(p,[rr]*3,8,4)
   else:pts,f=np.array([[0,0,0]]),[]
   d.mesh('drop'+str(i),pts,f,(.65,.15,.32))
  self.last_drop_count=len(self.liquid.parcels)
