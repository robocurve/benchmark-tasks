"""Native physical shapes and passive joints for disclosed custom maintenance parts."""
import numpy as np
from scipy.spatial.transform import Rotation
ROOT='/World/envs/env_0'
POSITION_ITERATIONS=128
VELOCITY_ITERATIONS=1

def collider(prim,friction=.6):
 from pxr import UsdPhysics,PhysxSchema,UsdShade
 UsdPhysics.CollisionAPI.Apply(prim); c=PhysxSchema.PhysxCollisionAPI.Apply(prim);c.CreateContactOffsetAttr(.00005);c.CreateRestOffsetAttr(0.)
 mat=UsdShade.Material.Define(prim.GetStage(),str(prim.GetPath())+'/physical_material'); m=UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
 m.CreateStaticFrictionAttr(friction);m.CreateDynamicFrictionAttr(friction*.8);m.CreateRestitutionAttr(0.)
 UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat,materialPurpose='physics')

def box(stage,path,center,size,color=(.45,.5,.55),rotation=None,collision=True):
 from pxr import UsdGeom,Gf,UsdShade,Sdf
 from yam_arena.station import linear_rgb
 g=UsdGeom.Cube.Define(stage,path);g.CreateSizeAttr(1.);g.AddTranslateOp().Set(Gf.Vec3d(*np.asarray(center,float).tolist()))
 if rotation is not None:
  q=Rotation.from_matrix(rotation).as_quat();g.AddOrientOp().Set(Gf.Quatf(float(q[3]),Gf.Vec3f(*q[:3].tolist())))
 g.AddScaleOp().Set(Gf.Vec3f(*np.asarray(size,float).tolist()))
 mat=UsdShade.Material.Define(stage,path+'/look');sh=UsdShade.Shader.Define(stage,path+'/look/shader');sh.CreateIdAttr('UsdPreviewSurface')
 sh.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*linear_rgb(color)));sh.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(.5)
 mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(),'surface');UsdShade.MaterialBindingAPI.Apply(g.GetPrim()).Bind(mat)
 if collision:collider(g.GetPrim())
 return g

def body(stage,path,position,mass,rotation=np.eye(3)):
 from pxr import UsdGeom,UsdPhysics,PhysxSchema,Gf
 p=UsdGeom.Xform.Define(stage,path).GetPrim();q=Rotation.from_matrix(rotation).as_quat()
 UsdGeom.Xformable(p).AddTranslateOp().Set(Gf.Vec3d(*np.asarray(position,float).tolist()));UsdGeom.Xformable(p).AddOrientOp().Set(Gf.Quatf(float(q[3]),Gf.Vec3f(*q[:3].tolist())))
 UsdPhysics.RigidBodyAPI.Apply(p);UsdPhysics.MassAPI.Apply(p).CreateMassAttr(float(mass))
 rb=PhysxSchema.PhysxRigidBodyAPI.Apply(p);rb.CreateSolverPositionIterationCountAttr(POSITION_ITERATIONS);rb.CreateSolverVelocityIterationCountAttr(VELOCITY_ITERATIONS);rb.CreateMaxDepenetrationVelocityAttr(.5)
 rb.CreateLinearDampingAttr(.05);rb.CreateAngularDampingAttr(.05)
 PhysxSchema.PhysxContactReportAPI.Apply(p).CreateThresholdAttr(0.)
 return p

