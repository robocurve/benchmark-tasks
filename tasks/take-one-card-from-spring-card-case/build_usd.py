"""USD authoring/readback only; no simulator startup or stepping."""
from pathlib import Path
import numpy as np
from pxr import Gf,Sdf,Usd,UsdGeom,UsdPhysics,UsdShade
import geometry as g

def body(stage,path,pos,shapes,friction=.3):
 x=UsdGeom.Xform.Define(stage,path);x.AddTranslateOp(UsdGeom.XformOp.PrecisionFloat).Set(Gf.Vec3f(*map(float,pos)))
 UsdPhysics.RigidBodyAPI.Apply(x.GetPrim());x.GetPrim().AddAppliedSchema('PhysxRigidBodyAPI')
 masses=np.array([s.mass for s in shapes]);com=sum(m*np.array(s.center)for m,s in zip(masses,shapes))/masses.sum()
 I=np.zeros((3,3))
 for m,s in zip(masses,shapes):
  a,b,c=s.size;r=np.array(s.center)-com
  I+=np.diag(m/12*np.array([b*b+c*c,a*a+c*c,a*a+b*b]))+m*(r@r*np.eye(3)-np.outer(r,r))
 ma=UsdPhysics.MassAPI.Apply(x.GetPrim());ma.CreateMassAttr(float(masses.sum()));ma.CreateCenterOfMassAttr(Gf.Vec3f(*map(float,com)))
 eig,V=np.linalg.eigh(I)
 if np.linalg.det(V)<0:V[:,0]*=-1
 from scipy.spatial.transform import Rotation
 q=Rotation.from_matrix(V).as_quat()
 ma.CreateDiagonalInertiaAttr(Gf.Vec3f(*map(float,eig)))
 ma.CreatePrincipalAxesAttr(Gf.Quatf(float(q[3]),Gf.Vec3f(*map(float,q[:3]))))
 for k,v in [('solverPositionIterationCount',32),('solverVelocityIterationCount',4)]:
  x.GetPrim().CreateAttribute('physxRigidBody:'+k,Sdf.ValueTypeNames.Int).Set(v)
 mat=UsdShade.Material.Define(stage,path+'/material');pm=UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
 pm.CreateStaticFrictionAttr(float(friction));pm.CreateDynamicFrictionAttr(float(friction));pm.CreateRestitutionAttr(0.)
 for s in shapes:
  c=UsdGeom.Cube.Define(stage,path+'/'+s.name);c.CreateSizeAttr(1.)
  c.AddTranslateOp(UsdGeom.XformOp.PrecisionFloat).Set(Gf.Vec3f(*map(float,s.center)));c.AddScaleOp(UsdGeom.XformOp.PrecisionFloat).Set(Gf.Vec3f(*map(float,s.size)));c.CreateDisplayColorAttr([Gf.Vec3f(*map(float,s.color))])
  UsdPhysics.CollisionAPI.Apply(c.GetPrim());c.GetPrim().AddAppliedSchema('PhysxCollisionAPI');UsdShade.MaterialBindingAPI.Apply(c.GetPrim()).Bind(mat,materialPurpose='physics')
  c.GetPrim().CreateAttribute('physxCollision:contactOffset',Sdf.ValueTypeNames.Float).Set(.00005)
  c.GetPrim().CreateAttribute('physxCollision:restOffset',Sdf.ValueTypeNames.Float).Set(0.)
 return x.GetPrim()

def author(stage,path,kind,pos,orientation=(0,0,0,1)):
 root=UsdGeom.Xform.Define(stage,path);root.AddTranslateOp(UsdGeom.XformOp.PrecisionFloat).Set(Gf.Vec3f(*map(float,pos)))
 root.AddOrientOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(Gf.Quatf(float(orientation[3]),*map(float,orientation[:3])))
 if kind=='case':
  body(stage,path+'/body',(0,0,0),g.case_shapes(),friction=.2)
  body(stage,path+'/follower',(0,0,0),g.follower_shapes(),friction=.2)
  j=UsdPhysics.PrismaticJoint.Define(stage,path+'/follower_slide');j.CreateAxisAttr('Z')
  j.CreateBody0Rel().SetTargets([Sdf.Path(path+'/body')]);j.CreateBody1Rel().SetTargets([Sdf.Path(path+'/follower')])
  j.CreateLocalPos0Attr(Gf.Vec3f(0,0,0));j.CreateLocalPos1Attr(Gf.Vec3f(0,0,0))
  j.CreateLowerLimitAttr(-.005);j.CreateUpperLimitAttr(.0038);j.CreateCollisionEnabledAttr(False)
  d=UsdPhysics.DriveAPI.Apply(j.GetPrim(),'linear');d.CreateTypeAttr('force');d.CreateStiffnessAttr(g.SPRING_K)
  d.CreateDampingAttr(.2);d.CreateTargetPositionAttr(g.SPRING_REST)
  UsdPhysics.ArticulationRootAPI.Apply(stage.GetPrimAtPath(path+'/body'));stage.GetPrimAtPath(path+'/body').AddAppliedSchema('PhysxArticulationAPI')
 else:
  body(stage,path+'/body',(0,0,0),[g.Shape('PVC',(0,0,0),tuple(g.CARD_SIZE),g.CARD_DENSITY,(.92,.92,.84))],friction=.25)
 return root.GetPrim()

def cold_export(path):
 stage=Usd.Stage.CreateNew(str(path));UsdGeom.SetStageUpAxis(stage,'Z');UsdGeom.SetStageMetersPerUnit(stage,1.)
 UsdGeom.Xform.Define(stage,'/World');stage.SetDefaultPrim(stage.GetPrimAtPath('/World'))
 author(stage,'/World/card_case','case',g.ORIGIN)
 for name,p in g.initial_cards().items():author(stage,'/World/'+name,'card',p)
 stage.GetRootLayer().Save();return stage
if __name__=='__main__':
 import sys
 cold_export(Path(sys.argv[1]))
