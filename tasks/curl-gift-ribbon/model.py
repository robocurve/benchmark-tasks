"""Flexible ribbon with contact-created plastic bend memory.

A narrow-strip reduced rod retains length and collides with the edge and board.
Only ribbon that slides over the edge under two-ended tension gains intrinsic
curvature. Its free shape follows spring constraints, gravity and self-contact;
there is no prescribed coil trajectory or success-triggered pose change.
"""
import numpy as np
from scipy.sparse import coo_matrix,diags
from scipy.sparse.linalg import factorized
from scipy.optimize import minimize
from common import TABLE,Grips

EDGE=np.array([.32,0.,TABLE+.065])
N=81
LENGTH=.400
DS=LENGTH/(N-1)
WIDTH=.010

class Model:
 def __init__(self):
  y=np.linspace(-.16,.24,N)
  z=np.full(N,TABLE+.0003)
  self.p=np.c_[np.full(N,.235),y,z]
  self.rest=np.linalg.norm(np.diff(self.p,axis=0),axis=1)
  self.v=np.zeros_like(self.p);self.memory=np.zeros(N-2);self.dose=np.zeros(N-2)
  self.grips=Grips();self.previous={};self.events=[];self.time=0.;self.release=0.
  self.failed=False;self.max_strain=1.;self.overstrain=0.;self.contact_distance=0.
  self.last_contact=None;self.load=0.;self.plastic_length=0.
  self.i,self.j=np.triu_indices(N,k=4)
  ids=np.arange(N-1);D=coo_matrix((np.r_[-np.ones(N-1),np.ones(N-1)],(np.r_[ids,ids],np.r_[ids,ids+1])),shape=(N-1,N)).tocsc()
  ids=np.arange(N-2);B=coo_matrix((np.r_[np.ones(N-2),-2*np.ones(N-2),np.ones(N-2)],(np.r_[ids,ids,ids],np.r_[ids,ids+1,ids+2])),shape=(N-2,N)).tocsc()
  self.D,self.B=D,B;self.ks=100000.;self.kb=5.
  self.elastic=self.ks*(D.T@D)+self.kb*(B.T@B)


 def self_contact(self,w):
  a,b=self.i,self.j;d=self.p[b]-self.p[a];dist=np.linalg.norm(d,axis=1)
  hit=(dist<.0014)&(dist>1e-8)
  if not hit.any():return
  a,b,d,dist=a[hit],b[hit],d[hit],dist[hit];den=np.maximum(w[a]+w[b],1e-9)
  push=d*((.0014-dist)/np.maximum(dist*den,1e-9))[:,None]
  np.add.at(self.p,a,-push*w[a,None]);np.add.at(self.p,b,push*w[b,None])

 def plastic_contact(self,dt):
  self.last_contact=None
  held=sorted(g['index'] for g in self.grips.held.values())
  if len(held)!=2:return
  lo,hi=held;inside=np.arange(lo,hi)
  crossings=inside[self.p[inside,1]*self.p[inside+1,1]<0]
  if len(crossings)!=1:return
  i=int(crossings[0]);a,b=self.p[i:i+2];u=-a[1]/(b[1]-a[1]);q=a*(1-u)+b*u
  if abs(q[0]-EDGE[0])>.043 or abs(q[2]-EDGE[2])>.003:return
  material_length=self.rest[lo:hi].sum()
  geometric=np.linalg.norm(self.p[lo]-q)+np.linalg.norm(self.p[hi]-q)
  strain=max(0.,geometric/max(material_length,1e-9)-1.)
  # Effective axial stiffness and bending load are disclosed engineering
  # estimates. Too little tension does not curl; excessive tension damages.
  contact_load=100.*strain
  edge_bend=np.arctan2(np.linalg.norm(np.cross(q-self.p[lo],self.p[hi]-q)),np.dot(q-self.p[lo],self.p[hi]-q))
  coord=float(np.sum(self.rest[:i])+u*self.rest[i])
  previous=self.previous.get('coordinate',coord)
  distance=abs(coord-previous);self.previous['coordinate']=coord
  self.last_contact=coord
  if .15<=contact_load<=5. and edge_bend>.35 and .0001<distance<.02:
   speed=distance/dt
   if speed>.12:return
   j=np.arange(N-2);centres=np.cumsum(self.rest)[:-1]
   band=abs(centres-coord)<.009
   self.dose[band]+=distance/.012*np.clip(contact_load/1.,0,1)*min(1.,.025/max(speed,.001))
   self.memory=.65*np.clip(self.dose,0,1)
   self.contact_distance+=distance

 def free_step(self,dt):
  # Inextensible implicit beam with lateral freedom and segment contact.
  # Contact covers the strip width, not just its sparse centreline vertices.
  old=self.p.copy();prediction=self.p+self.v*dt*np.exp(-5*dt)
  delta=np.diff(self.p,axis=0);angles=np.unwrap(np.arctan2(delta[:,2],delta[:,1]))
  tilt=np.arctan2(delta[:,0],np.linalg.norm(delta[:,1:],axis=1))
  seed=np.r_[self.p[0],angles,tilt];mass=3.5e-6;inertia=mass/dt**2;EI=2e-6
  ii,jj=np.triu_indices(N-1,k=3);scale=np.array([WIDTH,.001,.001])
  def objective(q):
   theta=q[3:N+2];phi=q[N+2:];ct=np.cos(theta);st=np.sin(theta);cp=np.cos(phi);sp=np.sin(phi)
   vectors=self.rest[:,None]*np.c_[sp,cp*ct,cp*st]
   xyz=np.vstack([q[:3],q[:3]+np.cumsum(vectors,axis=0)])
   diff=xyz-prediction;g=inertia*diff
   value=.5*inertia*np.sum(diff*diff)+mass*9.81*np.sum(xyz[:,2]);g[:,2]+=mass*9.81
   support=(xyz[:,1]>=-.0025)&(xyz[:,1]<=.0225)&(abs(xyz[:,0]-EDGE[0])<.05)
   sd=np.minimum(0.,xyz[:,2]-(TABLE+.055))*support
   value+=10*np.sum(sd*sd);g[:,2]+=20*sd
   for i in np.flatnonzero(xyz[:-1,1]*xyz[1:,1]<0):
    a,b=xyz[i:i+2];u=-a[1]/(b[1]-a[1]);point=(1-u)*a+u*b
    if point[2]>=EDGE[2]+.0001 or abs(point[0]-EDGE[0])>.045:continue
    gap=point[2]-(EDGE[2]+.0001);normal=np.array([0.,-(b[2]-a[2])/(b[1]-a[1]),1.])
    value+=10*gap*gap;g[i]+=20*gap*(1-u)*normal;g[i+1]+=20*gap*u*normal
   depth=np.minimum(0.,xyz[:,2]-(TABLE+.0003));value+=10*np.sum(depth*depth);g[:,2]+=20*depth
   # Closest points on nonadjacent complete segments, in an anisotropic
   # 10 mm width / 1 mm thickness contact envelope. Lateral displacement can
   # separate crossing coils; centreline points cannot silently pass through.
   scaled=xyz/scale;a,b,c,d=scaled[ii],scaled[ii+1],scaled[jj],scaled[jj+1]
   u=b-a;v=d-c;r=a-c
   aa=np.sum(u*u,axis=1);bb=np.sum(u*v,axis=1);cc=np.sum(v*v,axis=1);dd=np.sum(u*r,axis=1);ee=np.sum(v*r,axis=1)
   den=aa*cc-bb*bb
   s=np.clip(np.where(abs(den)>1e-12,(bb*ee-cc*dd)/np.maximum(den,1e-12),0.),0,1)
   t=np.clip((bb*s+ee)/np.maximum(cc,1e-12),0,1);s=np.clip((bb*t-dd)/np.maximum(aa,1e-12),0,1)
   separation=a+s[:,None]*u-c-t[:,None]*v;distance=np.linalg.norm(separation,axis=1)
   overlap=np.minimum(0.,distance-1.);contact_k=.002
   value+=.5*contact_k*np.sum(overlap*overlap)
   force=contact_k*overlap[:,None]*separation/np.maximum(distance[:,None],1e-9)/scale
   for ids,weight in [(ii,1-s),(ii+1,s),(jj,-(1-t)),(jj+1,-t)]:np.add.at(g,ids,weight[:,None]*force)
   turn=np.diff(theta);midphi=.5*(phi[:-1]+phi[1:])
   bend=turn*np.cos(midphi)-self.memory;sidebend=np.diff(phi);twist=turn*np.sin(midphi)
   value+=.5*EI/DS*(np.sum(bend*bend)+np.sum(sidebend*sidebend)+.5*np.sum(twist*twist))
   gt=np.zeros(N-1);gp=np.zeros(N-1)
   turn_gradient=EI/DS*(bend*np.cos(midphi)+.5*twist*np.sin(midphi))
   tilt_gradient=EI/DS*(-bend*turn*np.sin(midphi)+.5*twist*turn*np.cos(midphi))
   gt[:-1]-=turn_gradient;gt[1:]+=turn_gradient
   gp[:-1]-=EI/DS*sidebend;gp[1:]+=EI/DS*sidebend
   gp[:-1]+=.5*tilt_gradient;gp[1:]+=.5*tilt_gradient
   downstream=np.cumsum(g[::-1],axis=0)[::-1][1:]
   gt+=np.sum(downstream*(self.rest[:,None]*np.c_[np.zeros(N-1),-cp*st,cp*ct]),axis=1)
   gp+=np.sum(downstream*(self.rest[:,None]*np.c_[cp,-sp*ct,-sp*st]),axis=1)
   return float(value),np.r_[np.sum(g,axis=0),gt,gp]
  result=minimize(objective,seed,jac=True,method='L-BFGS-B',bounds=[(None,None)]*(N+2)+[(-.6,.6)]*(N-1),options=dict(maxiter=140,ftol=1e-12,gtol=1e-8,maxls=30))
  q=result.x;theta=q[3:N+2];phi=q[N+2:]
  delta=self.rest[:,None]*np.c_[np.sin(phi),np.cos(phi)*np.cos(theta),np.cos(phi)*np.sin(theta)]
  self.p[:]=np.vstack([q[:3],q[:3]+np.cumsum(delta,axis=0)])
  self.p[:,2]=np.maximum(self.p[:,2],TABLE+.0003);self.v=(self.p-old)/dt
  contact=self.p[:,2]<TABLE+.0004;self.v[contact,:2]*=.75;self.v[contact,2]=np.maximum(0,self.v[contact,2])

 def update(self,hands,dt=.05):
  self.time+=dt;self.grips.update(self.p,hands);targets=self.grips.targets(hands)
  starts={i:self.p[i].copy() for i in targets};w=np.ones(N);w[list(targets)]=0.
  self.load=0.
  if len(targets)==2:
   lo,hi=sorted(targets);a,b=targets[lo],targets[hi]
   if a[1]*b[1]<0 and max(a[2],b[2])<EDGE[2]:
    imposed=np.linalg.norm(a-EDGE)+np.linalg.norm(b-EDGE)
    self.load=100*max(0.,imposed/self.rest[lo:hi].sum()-1.)
  if not targets and np.max(self.memory)>.01:
   self.free_step(dt)
  else:
   h=dt/8;mass=3.5e-6;inertia=mass/h**2
   for sub in range(8):
    old=self.p.copy();self.v[:,2]-=9.81*h;self.v*=np.exp(-8*h);prediction=self.p+self.v*h
    A=self.elastic+diags(np.full(N,inertia));rhs_base=inertia*prediction
    for i,goal in targets.items():
     A[i,i]+=1e8;rhs_base[i]+=1e8*(starts[i]+(sub+1)/8*(goal-starts[i]))
    Az=A.copy();rhs_z=rhs_base[:,2].copy()
    floor=np.flatnonzero(prediction[:,2]<TABLE+.0003)
    for i in floor:Az[i,i]+=1e5;rhs_z[i]+=1e5*(TABLE+.0003)
    for i in np.flatnonzero(self.p[:-1,1]*self.p[1:,1]<=0):
     a,b=self.p[i],self.p[i+1];den=b[1]-a[1]
     if abs(den)<1e-9:continue
     u=-a[1]/den;q=(1-u)*a+u*b
     if abs(q[0]-EDGE[0])>.045 or q[2]>EDGE[2]+.002:continue
     for j,c in [(i,1-u),(i+1,u)]:
      rhs_z[j]+=1e6*c*(EDGE[2]+.0001)
      for k,d in [(i,1-u),(i+1,u)]:Az[j,k]+=1e6*c*d
    solve=factorized(A.tocsc());solve_z=factorized(Az.tocsc())
    for iteration in range(12):
     delta=np.diff(self.p,axis=0);length=np.linalg.norm(delta,axis=1)
     projected=delta*(self.rest/np.maximum(length,1e-9))[:,None]
     tangent=delta[:-1]+delta[1:];normal=np.cross(np.tile([1.,0,0],(N-2,1)),tangent)
     normal/=np.maximum(np.linalg.norm(normal,axis=1)[:,None],1e-9)
     curvature=normal*(2*DS*np.sin(self.memory/2))[:,None]
     forces=self.ks*(self.D.T@projected)+self.kb*(self.B.T@curvature)
     rhs=rhs_base+forces
     self.p[:]=np.c_[solve(rhs[:,0]),solve(rhs[:,1]),solve_z(rhs_z+forces[:,2])]
     self.self_contact(w)
    self.p[:,2]=np.maximum(self.p[:,2],TABLE+.0003)
    self.v=(self.p-old)/h
    contact=self.p[:,2]<TABLE+.0004;self.v[contact,:2]*=.7;self.v[contact,2]=np.maximum(0,self.v[contact,2])
  strain=float(np.max(np.linalg.norm(np.diff(self.p,axis=0),axis=1)/self.rest));self.max_strain=max(self.max_strain,strain)
  self.overstrain=self.overstrain+dt if strain>1.6 else 0.
  self.failed |= not np.isfinite(self.p).all() or self.overstrain>.2
  self.plastic_contact(dt)
  self.release=self.release+dt if not self.grips.held and all(np.min(np.linalg.norm(self.p-h['tcp'],axis=1))>.04 for h in hands.values()) else 0.
  self.events=self.grips.events

 def overlap_mm(self):
  # Diagnostic for invalid crossings of the rendered strip faces. This is
  # not a physical judging tool; a real ribbon cannot pass through itself.
  i,j=np.triu_indices(N-1,k=3);a=self.p[i,1:];b=self.p[j,1:]
  u=self.p[i+1,1:]-a;v=self.p[j+1,1:]-b;r=b-a
  cross=lambda x,y:x[:,0]*y[:,1]-x[:,1]*y[:,0]
  den=cross(u,v);safe=np.where(abs(den)>1e-12,den,1.)
  s=cross(r,v)/safe;t=cross(r,u)/safe
  valid=(abs(den)>1e-12)&(s>0)&(s<1)&(t>0)&(t<1)
  if not valid.any():return 0.
  xi=self.p[i,0]+s*(self.p[i+1,0]-self.p[i,0]);xj=self.p[j,0]+t*(self.p[j+1,0]-self.p[j,0])
  return float(max(0.,np.max(WIDTH-abs(xi[valid]-xj[valid])))*1000)

 def measure(self):
  d=np.diff(self.p,axis=0);a,b=d[:-1],d[1:]
  cross=np.cross(a,b);cross_length=np.linalg.norm(cross,axis=1)
  angles=np.arctan2(cross_length,np.sum(a*b,axis=1))
  normals=cross/np.maximum(cross_length[:,None],1e-12)
  # Count actual retained turns in a contiguous same-direction curl. Rest
  # curvature/dose alone earns nothing: the final centreline must curl.
  segment=.5*(np.linalg.norm(a,axis=1)+np.linalg.norm(b,axis=1))
  radius=segment/np.maximum(abs(angles),1e-8)
  good=(angles>.04)&(radius<.030)&(radius>.004)
  best=0.;current=0.;axis=np.zeros(3)
  for theta,normal,ok in zip(angles,normals,good):
   same=current>0 and np.dot(normal,axis)>.7
   current=(current+theta if same else theta) if ok else 0.
   if ok:
    axis=axis*.8+normal*.2 if same else normal.copy()
    axis/=max(np.linalg.norm(axis),1e-9)
   else:axis[:]=0.
   best=max(best,current)
  edge_support=False
  for i in np.flatnonzero(self.p[:-1,1]*self.p[1:,1]<=0):
   a,b=self.p[i:i+2];den=b[1]-a[1]
   if abs(den)<1e-9:continue
   q=a+(-a[1]/den)*(b-a)
   edge_support |= abs(q[0]-EDGE[0])<.047 and abs(q[2]-EDGE[2])<.003
  return dict(retained_turns=float(best/(2*np.pi)),processed_length_mm=float(np.sum(self.memory>.30)*DS*1000),
              released_seconds=self.release,loaded_slide_mm=self.contact_distance*1000,current_tension_proxy_N=self.load,
              intact=not self.failed,max_strain=self.max_strain,strip_interpenetration_mm=self.overlap_mm(),
              on_table=bool(TABLE-.001<=np.min(self.p[:,2])<=TABLE+.002 and np.max(self.p[:,2])<TABLE+.12 and not edge_support))
 def score(self):
  m=self.measure()
  if self.failed or not m['on_table'] or m['strip_interpenetration_mm']>1.:return 0.
  turns=m['retained_turns']
  if turns>=2:return 1. if self.release>=3 else .75
  if self.release<3:return 0.
  return .5 if turns>=1 else .25 if turns>=.5 else 0.
 def state(self):return dict(points=self.p,plastic_angles=self.memory,plastic_work=self.dose,held={s:g['index'] for s,g in self.grips.held.items()})
 def draw(self,d):
  p=np.stack([self.p+[-WIDTH/2,0,0],self.p+[WIDTH/2,0,0]],axis=1).reshape(-1,3)
  faces=[(2*i,2*i+1,2*i+3,2*i+2) for i in range(N-1)]
  d.mesh('curling_ribbon',p,faces,(.83,.16,.27))
