"""Signed planar spanning-surface intersections and conservative crossing audit."""
import numpy as np

def _polygon(points):
 p=np.asarray(points,dtype=float)
 if p.ndim!=2 or p.shape[1]!=3 or len(p)<4 or not np.isfinite(p).all():return None
 if np.linalg.norm(p[0]-p[-1])>1e-7:p=np.vstack([p,p[0]])
 n=np.cross(p[:-1],p[1:]).sum(0);length=np.linalg.norm(n)
 if length<1e-10:return None
 n/=length;u=p[1]-p[0];u/=np.linalg.norm(u);v=np.cross(n,u)
 if np.max(np.abs((p-p[0])@n))>.0002:return None
 return p,p[0],n,u,v,np.column_stack([(p-p[0])@u,(p-p[0])@v])

def _inside(point,polygon):
 p=np.asarray(point);a=polygon[:-1];b=polygon[1:];d=b-a;den=np.sum(d*d,axis=1);t=np.clip(np.sum((p-a)*d,axis=1)/np.maximum(den,1e-20),0,1);distance=float(np.linalg.norm(p-a-t[:,None]*d,axis=1).min())
 x,y=p;ay=a[:,1];by=b[:,1];cross=(ay>y)!=(by>y);at=a[:,0]+(y-ay)*(b[:,0]-a[:,0])/np.where(np.abs(by-ay)>1e-20,by-ay,1.)
 return bool(np.count_nonzero(cross&(x<at))%2),distance

def directed(surface,curve,guard=.00255,epsilon=.0002):
 model=_polygon(surface)
 if model is None:return 0,True
 _,origin,normal,u,v,poly=model;b=np.asarray(curve,dtype=float)
 if np.linalg.norm(b[0]-b[-1])<1e-8:b=b[:-1]
 if len(b)<3 or not np.isfinite(b).all():return 0,True
 distance=(b-origin)@normal;sign=np.where(distance>epsilon,1,np.where(distance<-epsilon,-1,0));total=0;ambiguous=False
 def count(point,direction,tangent=False):
  nonlocal total,ambiguous
  xy=np.array([(point-origin)@u,(point-origin)@v]);inside,edge=_inside(xy,poly)
  if edge<=guard:ambiguous=True
  elif inside:
   if tangent:ambiguous=True
   else:total+=int(direction)
 for i in range(len(b)):
  j=(i+1)%len(b)
  if sign[i]*sign[j]==-1:
   t=distance[i]/(distance[i]-distance[j]);count(b[i]+t*(b[j]-b[i]),sign[j])
  elif sign[i]==0:
   previous=(i-1)%len(b)
   if sign[previous]==0 or sign[j]==0:
    xy=np.array([(b[i]-origin)@u,(b[i]-origin)@v]);inside,edge=_inside(xy,poly)
    if inside or edge<=guard:ambiguous=True
   elif sign[previous]*sign[j]==-1:count(b[i],sign[j])
   else:count(b[i],0,True)
 return total,ambiguous

def linked(a,b,guard=.00255):
 ab,bad_a=directed(a,b,guard);ba,bad_b=directed(b,a,guard)
 return bool(not bad_a and not bad_b and abs(ab)==1 and ab==ba)

def segment_distance(a,b,c,d):
 a,b,c,d=map(np.asarray,(a,b,c,d));u=b-a;v=d-c;w=a-c;aa=u@u;bb=u@v;cc=v@v;dd=u@w;ee=v@w;den=aa*cc-bb*bb
 def point_segment(p,x,y):
  z=y-x;t=np.clip((p-x)@z/max(z@z,1e-20),0,1);return np.linalg.norm(p-x-t*z)
 values=[point_segment(a,c,d),point_segment(b,c,d),point_segment(c,a,b),point_segment(d,a,b)]
 if den>1e-20:
  s=(bb*ee-cc*dd)/den;t=(aa*ee-bb*dd)/den
  if 0<=s<=1 and 0<=t<=1:values.append(np.linalg.norm(w+s*u-t*v))
 return float(min(values))

class CrossingAudit:
 def __init__(self):self.previous=set();self.open_seen={};self.invalid=False;self.steps=0;self.reason=None
 def update(self,curves,closed):
  self.steps+=1;names=sorted(curves);edges=set()
  for i,a in enumerate(names):
   for b in names[i+1:]:
    pair=(a,b);opened=not(closed[a] and closed[b])
    if opened:self.open_seen[pair]=True
    # Separated loops cannot interlink. This broad phase also avoids treating
    # unrelated distant coplanar vertices as contact ambiguities.
    pa,pb=curves[a],curves[b]
    if np.linalg.norm(pa.mean(0)-pb.mean(0))>np.ptp(pa,axis=0).max()+np.ptp(pb,axis=0).max():
     if not opened:self.open_seen[pair]=False
     continue
    if not opened and linked(pa,pb):
     edges.add(pair)
     if pair not in self.previous and not self.open_seen.get(pair,False):self.invalid=True;self.reason='Closed loops became linked without a measured open gate'
    if not opened:self.open_seen[pair]=False
  self.previous=edges
  return edges

def segment_matrix_distance(first,second):
 """All centerline distances, including parallel and endpoint cases."""
 a,b=np.asarray(first)[:,0,None,:],np.asarray(first)[:,1,None,:]
 c,d=np.asarray(second)[None,:,0,:],np.asarray(second)[None,:,1,:]
 u,v,w=b-a,d-c,a-c;aa=np.sum(u*u,-1);bb=np.sum(u*v,-1);cc=np.sum(v*v,-1);dd=np.sum(u*w,-1);ee=np.sum(v*w,-1);den=aa*cc-bb*bb
 values=[]
 for p,x,y in [(a,c,d),(b,c,d),(c,a,b),(d,a,b)]:
  z=y-x;t=np.clip(np.sum((p-x)*z,-1)/np.maximum(np.sum(z*z,-1),1e-20),0,1);values.append(np.linalg.norm(p-x-t[...,None]*z,axis=-1))
 s=(bb*ee-cc*dd)/np.maximum(den,1e-20);t=(aa*ee-bb*dd)/np.maximum(den,1e-20);valid=(den>1e-20)&(s>=0)&(s<=1)&(t>=0)&(t<=1)
 values.append(np.where(valid,np.linalg.norm(w+s[...,None]*u-t[...,None]*v,axis=-1),np.inf));return np.min(values,axis=0)

class RodAudit:
 """Sweep exact capsule/cylinder primitives at<=1mm endpoint intervals.

Capsule bounds screen pairs; true flat-ended cylinder contacts confirm gate
candidates. The0.9mm solver-overlap limit is unchanged.
 """
 def __init__(self):self.previous={};self.invalid=False;self.reason=None;self.narrow_checks=0
 @staticmethod
 def primitive(points,radius,cylinder):
  import hppfcl as fcl
  a,b=points;axis=b-a;length=float(np.linalg.norm(axis));z=axis/max(length,1e-20);helper=np.array([0.,0.,1.]) if abs(z[2])<.9 else np.array([1.,0.,0.]);x=np.cross(helper,z);x/=max(np.linalg.norm(x),1e-20);rotation=np.column_stack([x,np.cross(z,x),z]);shape=(fcl.Cylinder if cylinder else fcl.Capsule)(float(radius),length)
  return fcl.CollisionObject(shape,fcl.Transform3f(rotation,(a+b)/2))
 def exact_penetration(self,a,ra,ca,b,rb,cb):
  import hppfcl as fcl
  self.narrow_checks+=1;request=fcl.CollisionRequest();request.enable_contact=True;request.num_max_contacts=4;request.gjk_tolerance=1e-8;result=fcl.CollisionResult();fcl.collide(self.primitive(a,ra,ca),self.primitive(b,rb,cb),request,result);contacts=result.getContacts();return max([0.]+[float(c.penetration_depth) for c in contacts])
 def update(self,groups):
  if self.invalid:return False
  groups={n:(np.asarray(v[0]),np.asarray(v[1]),np.asarray(v[2],dtype=bool) if len(v)>2 else np.zeros(len(v[0]),dtype=bool)) for n,v in groups.items()};names=list(groups);motion={}
  for n,(points,radii,types) in groups.items():
   if n in self.previous:
    motion[n]=float(np.max(np.linalg.norm(points-self.previous[n][0],axis=-1)))
    if motion[n]>.05:self.invalid=True;self.reason=dict(reason='unexplained_substep_motion',body=n,endpoint_motion_m=motion[n]);return False
   else:motion[n]=0.
  for i,a in enumerate(names):
   for b in names[i+1:]:
    pa,ra,ca=groups[a];pb,rb,cb=groups[b];old_a=self.previous.get(a,groups[a])[0];old_b=self.previous.get(b,groups[b])[0]
    if min(np.linalg.norm(pa.mean((0,1))-pb.mean((0,1))),np.linalg.norm(old_a.mean((0,1))-old_b.mean((0,1))))>.13:continue
    samples=max(1,int(np.ceil(max(motion[a],motion[b])/.001)))
    for fraction in np.linspace(0,1,min(samples,64)+1)[1:]:
     sa,sb=old_a+fraction*(pa-old_a),old_b+fraction*(pb-old_b);distance=segment_matrix_distance(sa,sb);depth=ra[:,None]+rb[None,:]-distance
     for x,y in np.argwhere(depth>.0009):
      penetration=self.exact_penetration(sa[x],ra[x],ca[x],sb[y],rb[y],cb[y]) if ca[x] or cb[y] else float(depth[x,y])
      if penetration>.0009:
       self.invalid=True;self.reason=dict(reason='swept_interbody_rod_penetration',pair=[a,b],segments=[int(x),int(y)],flat_cylinders=[bool(ca[x]),bool(cb[y])],penetration_m=penetration,bounding_capsule_penetration_m=float(depth[x,y]));return False
  self.previous={n:(p.copy(),r.copy(),k.copy()) for n,(p,r,k) in groups.items()};return True
