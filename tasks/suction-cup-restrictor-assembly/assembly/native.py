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

def wedge(stage,path,profile,a0,a1,color=(.15,.65,.35),offset=(0,0,0)):
 """Convex radial wedge around local Y. Profile is convex [(radius,y),...]."""
 from pxr import UsdGeom,UsdPhysics,Gf
 from yam_arena.station import linear_rgb
 vertices=np.array([[r*np.cos(a),y,r*np.sin(a)] for a in [a0,a1] for r,y in profile])+offset
 n=len(profile);faces=[list(range(n-1,-1,-1)),list(range(n,2*n))]
 faces += [[i,(i+1)%n,(i+1)%n+n,i+n] for i in range(n)]
 mesh=UsdGeom.Mesh.Define(stage,path);mesh.CreatePointsAttr(vertices.tolist());mesh.CreateFaceVertexCountsAttr([len(f) for f in faces]);mesh.CreateFaceVertexIndicesAttr([i for f in faces for i in f]);mesh.CreateSubdivisionSchemeAttr('none');mesh.CreateDisplayColorAttr([Gf.Vec3f(*linear_rgb(color))])
 collider(mesh.GetPrim());UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr('convexHull')
 return mesh

def ring(stage,path,inner,outer,y0,y1,color=(.3,.65,.38),segments=32,offset=(0,0,0)):
 for i in range(segments):wedge(stage,path+f'/s{i:02d}',[(inner,y0),(outer,y0),(outer,y1),(inner,y1)],i*2*np.pi/segments,(i+1)*2*np.pi/segments,color,offset)

def body(stage,path,position,mass,rotation=np.eye(3)):
 from pxr import UsdGeom,UsdPhysics,PhysxSchema,Gf
 p=UsdGeom.Xform.Define(stage,path).GetPrim();q=Rotation.from_matrix(rotation).as_quat()
 UsdGeom.Xformable(p).AddTranslateOp().Set(Gf.Vec3d(*np.asarray(position,float).tolist()));UsdGeom.Xformable(p).AddOrientOp().Set(Gf.Quatf(float(q[3]),Gf.Vec3f(*q[:3].tolist())))
 UsdPhysics.RigidBodyAPI.Apply(p);UsdPhysics.MassAPI.Apply(p).CreateMassAttr(float(mass))
 rb=PhysxSchema.PhysxRigidBodyAPI.Apply(p);rb.CreateSolverPositionIterationCountAttr(POSITION_ITERATIONS);rb.CreateSolverVelocityIterationCountAttr(VELOCITY_ITERATIONS);rb.CreateMaxDepenetrationVelocityAttr(.5)
 rb.CreateLinearDampingAttr(.05);rb.CreateAngularDampingAttr(.05)
 PhysxSchema.PhysxContactReportAPI.Apply(p).CreateThresholdAttr(0.)
 return p

def radial_joint(stage,path,parent,child,direction,stiffness=180.,lo=-.003,hi=.0015):
 from pxr import UsdPhysics,Gf,PhysxSchema
 # Both bodies use identical frames. Local joint X is the prescribed radial axis.
 x=np.array(direction);y=np.array([0.,1.,0.]);z=np.cross(x,y);r=np.column_stack([x,y,z]);q=Rotation.from_matrix(r).as_quat();quat=Gf.Quatf(float(q[3]),Gf.Vec3f(*q[:3].tolist()))
 j=UsdPhysics.PrismaticJoint.Define(stage,path);j.CreateBody0Rel().SetTargets([parent]);j.CreateBody1Rel().SetTargets([child]);j.CreateAxisAttr('X')
 j.CreateLocalRot0Attr(quat);j.CreateLocalRot1Attr(quat);j.CreateLowerLimitAttr(lo);j.CreateUpperLimitAttr(hi);j.CreateExcludeFromArticulationAttr(False);j.CreateCollisionEnabledAttr(False)
 d=UsdPhysics.DriveAPI.Apply(j.GetPrim(),'linear');d.CreateTypeAttr('force');d.CreateStiffnessAttr(stiffness);d.CreateDampingAttr(.8);d.CreateTargetPositionAttr(0.)
 return j

def filter_pairs(stage,paths):
 from pxr import UsdPhysics
 for a in paths:
  UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(a)).CreateFilteredPairsRel().SetTargets([b for b in paths if b!=a])
