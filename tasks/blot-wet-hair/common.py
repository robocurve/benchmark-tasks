"""Geometry and drawing helpers; task objects use native contact dynamics."""
import numpy as np
TABLE=.7475

def rotation(q):
 x,y,z,w=np.asarray(q,float);return np.array([[1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y)],[2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x)],[2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)]])

def down(deg=0.):
 a=np.radians(deg);return np.array([[np.cos(a),np.sin(a),0],[np.sin(a),-np.cos(a),0],[0,0,-1.]])

class Drawing:
 """USD geometry of the CURRENT physical state; no animation timeline."""
 def __init__(self,stage):self.stage=stage;self.meshes={}
 def mesh(self,name,points,faces,color):
  from pxr import UsdGeom,UsdShade,Sdf,Gf,Vt
  if name not in self.meshes:
   path='/World/TaskMaterial/'+name;m=UsdGeom.Mesh.Define(self.stage,path);m.CreateSubdivisionSchemeAttr('none');m.CreateDoubleSidedAttr(True)
   mat=UsdShade.Material.Define(self.stage,path+'/Material');shader=UsdShade.Shader.Define(self.stage,path+'/Material/Surface');shader.CreateIdAttr('UsdPreviewSurface');shader.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color));shader.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(.65);mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),'surface');UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(mat);self.meshes[name]=m
   m.GetFaceVertexCountsAttr().Set([len(f) for f in faces]);m.GetFaceVertexIndicesAttr().Set([int(v) for f in faces for v in f])
  m=self.meshes[name];m.GetPointsAttr().Set(Vt.Vec3fArray(np.asarray(points,dtype=np.float32).tolist()))
 def tube(self,name,points,radius,color,sides=8):
  p=np.asarray(points);t=np.gradient(p,axis=0);t/=np.maximum(np.linalg.norm(t,axis=1)[:,None],1e-12)
  axis=np.tile([0.,0.,1.],(len(p),1));axis[abs(t[:,2])>.9]=[0,1,0];u=np.cross(t,axis);u/=np.maximum(np.linalg.norm(u,axis=1)[:,None],1e-12);v=np.cross(t,u)
  angles=np.arange(sides)*2*np.pi/sides;pts=(p[:,None,:]+radius*(u[:,None,:]*np.cos(angles)[None,:,None]+v[:,None,:]*np.sin(angles)[None,:,None])).reshape(-1,3)
  faces=[(i*sides+j,i*sides+(j+1)%sides,(i+1)*sides+(j+1)%sides,(i+1)*sides+j) for i in range(len(p)-1) for j in range(sides)];faces.extend([tuple(range(sides-1,-1,-1)),tuple(range((len(p)-1)*sides,len(p)*sides))]);self.mesh(name,pts,faces,color)
 def box(self,name,center,size,color,R=None):
  c=np.asarray(center);p=np.array([[x,y,z] for x in [-1,1] for y in [-1,1] for z in [-1,1]])*np.asarray(size)/2
  if R is not None:p=p@np.asarray(R).T
  faces=[(0,1,3,2),(4,6,7,5),(0,4,5,1),(2,3,7,6),(0,2,6,4),(1,5,7,3)];self.mesh(name,p+c,faces,color)
