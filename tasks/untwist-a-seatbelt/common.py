"""Small, explicit geometry and reduced-material helpers. Metres, seconds, kg.

All grasp targets come from measured robot poses. These are idealized captures,
not resolved finger friction. No stage label, policy target or score is accepted.
"""
import numpy as np

TABLE=.7475

def rotation(q):
 x,y,z,w=np.asarray(q,float);return np.array([[1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y)],[2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x)],[2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)]])

def down(deg=0.):
 a=np.radians(deg);return np.array([[np.cos(a),np.sin(a),0],[np.sin(a),-np.cos(a),0],[0,0,-1.]])

def grip_candidates(points,h):
 local=(np.asarray(points)-h['tcp'])@h['rotation'];gap=h['opening']*.094
 hit=(abs(local[:,0])<=gap/2+.002)&(abs(local[:,1])<=.012)&(local[:,2]>=-.028)&(local[:,2]<=.0152)
 return np.flatnonzero(hit),local

class Grips:
 def __init__(self):self.held={};self.events=[]
 def update(self,points,hands):
  for side,h in hands.items():
   if h['opening']>.43:
    if side in self.held:self.events.append(dict(event='release',side=side,index=int(self.held[side]['index'])))
    self.held.pop(side,None);continue
   if side in self.held or h['opening']>.30:continue
   ids,local=grip_candidates(points,h)
   used={g['index'] for g in self.held.values()};ids=[i for i in ids if i not in used]
   if not ids:continue
   i=min(ids,key=lambda i:np.linalg.norm(local[i]-[0,0,-.008]));self.held[side]=dict(index=int(i),local=local[i].copy())
   self.events.append(dict(event='capture',side=side,index=int(i)))
 def targets(self,hands):
  return {g['index']:hands[s]['tcp']+hands[s]['rotation']@g['local'] for s,g in self.held.items()}

class Soft:
 """Position-based mass points with stretch/shear constraints and gravity.

 Collision objects and adhesive constraints are supplied by the task. A sheet
 uses a coarse bending stencil; a cable uses capsule-pair self-contact.
 """
 def __init__(self,points,edges,stiffness=None,radius=.001,floor=TABLE):
  self.p=np.array(points,float);self.v=np.zeros_like(self.p);self.edges=np.array(edges,int)
  self.rest=np.linalg.norm(self.p[self.edges[:,1]]-self.p[self.edges[:,0]],axis=1)
  self.k=np.ones(len(edges)) if stiffness is None else np.asarray(stiffness,float)
  self.radius=radius;self.floor=floor;self.failed=False;self.grips=Grips();self.elapsed=0.;self.released=0.
  self.self_pairs=None;self.last_targets={}
  # Edge-coloured Gauss-Seidel projections preserve webbing length under gravity.
  groups=[];used=[]
  for edge,(a,b) in enumerate(self.edges):
   for q,seen in enumerate(used):
    if a not in seen and b not in seen:break
   else:q=len(groups);groups.append([]);used.append(set())
   groups[q].append(edge);used[q].update((a,b))
  self.edge_groups=[np.array(g,dtype=int) for g in groups]
 def cable_collisions(self):
  i,j=np.triu_indices(len(self.p)-1,k=3);self.self_pairs=(i,j)
 def self_contact(self,w):
  i,j=self.self_pairs;a,b,c,d=self.p[i],self.p[i+1],self.p[j],self.p[j+1]
  u=b-a;v=d-c;r=a-c;aa=np.einsum('ij,ij->i',u,u);bb=np.einsum('ij,ij->i',u,v);cc=np.einsum('ij,ij->i',v,v);dd=np.einsum('ij,ij->i',u,r);ee=np.einsum('ij,ij->i',v,r)
  den=aa*cc-bb*bb;s=np.clip(np.where(abs(den)>1e-14,(bb*ee-cc*dd)/np.maximum(den,1e-14),0.),0,1)
  t=np.clip((bb*s+ee)/np.maximum(cc,1e-12),0,1);s=np.clip((bb*t-dd)/np.maximum(aa,1e-12),0,1)
  diff=a+s[:,None]*u-c-t[:,None]*v;dist=np.linalg.norm(diff,axis=1);hit=(dist<2*self.radius)&(dist>1e-9)
  if not hit.any():return
  i,j,s,t,diff,dist=[x[hit] for x in [i,j,s,t,diff,dist]]
  coeff=[1-s,s,-(1-t),-t];ids=[i,i+1,j,j+1]
  inv=sum(w[idx]*co*co for idx,co in zip(ids,coeff));force=diff*((2*self.radius-dist)/np.maximum(dist*inv,1e-12))[:,None]
  for idx,co in zip(ids,coeff):np.add.at(self.p,idx,force*(w[idx]*co)[:,None])
 def step(self,hands,dt=.05,collide=None,pins=None,substeps=8,iterations=8,gravity=9.81):
  self.grips.update(self.p,hands);targets=self.grips.targets(hands);pins={} if pins is None else dict(pins)
  for i,p in targets.items():pins[i]=p
  starts={i:self.p[i].copy() for i in pins};w=np.ones(len(self.p));w[list(pins)]=0.
  h=dt/substeps;e=self.edges;i,j=e[:,0],e[:,1];den=np.maximum(w[i]+w[j],1e-12)
  for sub in range(substeps):
   old=self.p.copy();g=np.array([0.,0.,-gravity]) if np.ndim(gravity)==0 else np.asarray(gravity);self.v+=g*h;self.v*=np.exp(-2.*h);self.p+=self.v*h
   for repeat in range(iterations):
    for group in self.edge_groups:
     a=i[group];b=j[group];delta=self.p[b]-self.p[a];length=np.linalg.norm(delta,axis=1)
     corr=delta*((length-self.rest[group])*self.k[group]/np.maximum(length*den[group],1e-12))[:,None]
     self.p[a]+=corr*w[a,None]*.9;self.p[b]-=corr*w[b,None]*.9
    if self.self_pairs is not None:self.self_contact(w)
    if collide is not None:collide(self.p)
    if self.floor is not None:self.p[:,2]=np.maximum(self.p[:,2],self.floor+self.radius)
    for idx,goal in pins.items():self.p[idx]=starts[idx]+(sub+1)/substeps*(goal-starts[idx])
   self.v=(self.p-old)/h
   if self.floor is not None:
    contact=self.p[:,2]<=self.floor+self.radius+1e-5;self.v[contact,:2]*=.7;self.v[contact,2]=np.maximum(self.v[contact,2],0.)
  lengths=np.linalg.norm(self.p[j]-self.p[i],axis=1)
  if not np.isfinite(self.p).all() or np.max(lengths/np.maximum(self.rest,1e-9))>2.:self.failed=True
  self.elapsed+=dt;self.released=self.released+dt if not self.grips.held else 0.
  return targets

def sheet(center,size,nx,ny):
 x=np.linspace(-size[0]/2,size[0]/2,nx);y=np.linspace(-size[1]/2,size[1]/2,ny)
 p=np.array([[a,b,0] for a in x for b in y])+np.array(center);edges=[];stiff=[];faces=[]
 for i in range(nx):
  for j in range(ny):
   a=i*ny+j
   for di,dj,k in [(1,0,1.),(0,1,1.),(1,1,.6),(1,-1,.6),(2,0,.035),(0,2,.035)]:
    if 0<=i+di<nx and 0<=j+dj<ny:edges.append((a,(i+di)*ny+j+dj));stiff.append(k)
   if i+1<nx and j+1<ny:faces.extend([(a,a+ny,a+1),(a+1,a+ny,a+ny+1)])
 return p,edges,stiff,faces

class Drawing:
 """USD geometry of the CURRENT physical state; no animation timeline."""
 def __init__(self,stage):self.stage=stage;self.meshes={}
 def mesh(self,name,points,faces,color):
  from pxr import UsdGeom,UsdShade,Sdf,Gf,Vt
  if name not in self.meshes:
   path='/World/TaskMaterial/'+name;m=UsdGeom.Mesh.Define(self.stage,path);m.CreateSubdivisionSchemeAttr('none');m.CreateDoubleSidedAttr(True)
   mat=UsdShade.Material.Define(self.stage,path+'/Material');shader=UsdShade.Shader.Define(self.stage,path+'/Material/Surface');shader.CreateIdAttr('UsdPreviewSurface');shader.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color));shader.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(.65);mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),'surface');UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(mat);self.meshes[name]=m
  m=self.meshes[name];m.GetPointsAttr().Set(Vt.Vec3fArray(np.asarray(points,dtype=np.float32).tolist()));m.GetFaceVertexCountsAttr().Set([len(f) for f in faces]);m.GetFaceVertexIndicesAttr().Set([int(v) for f in faces for v in f])
 def tube(self,name,points,radius,color,sides=8):
  p=np.asarray(points);t=np.gradient(p,axis=0);t/=np.maximum(np.linalg.norm(t,axis=1)[:,None],1e-12)
  axis=np.tile([0.,0.,1.],(len(p),1));axis[abs(t[:,2])>.9]=[0,1,0];u=np.cross(t,axis);u/=np.maximum(np.linalg.norm(u,axis=1)[:,None],1e-12);v=np.cross(t,u)
  angles=np.arange(sides)*2*np.pi/sides;pts=(p[:,None,:]+radius*(u[:,None,:]*np.cos(angles)[None,:,None]+v[:,None,:]*np.sin(angles)[None,:,None])).reshape(-1,3)
  faces=[(i*sides+j,i*sides+(j+1)%sides,(i+1)*sides+(j+1)%sides,(i+1)*sides+j) for i in range(len(p)-1) for j in range(sides)];faces.extend([tuple(range(sides-1,-1,-1)),tuple(range((len(p)-1)*sides,len(p)*sides))]);self.mesh(name,pts,faces,color)
 def box(self,name,center,size,color,R=None):
  c=np.asarray(center);p=np.array([[x,y,z] for x in [-1,1] for y in [-1,1] for z in [-1,1]])*np.asarray(size)/2
  if R is not None:p=p@np.asarray(R).T
  faces=[(0,1,3,2),(4,6,7,5),(0,4,5,1),(2,3,7,6),(0,2,6,4),(1,5,7,3)];self.mesh(name,p+c,faces,color)
