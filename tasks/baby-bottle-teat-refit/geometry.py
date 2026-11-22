"""Free rigid bottle and collar, deformable silicone teat and 20 g upright fill."""
import numpy as np
from specimen import specimen
import threads

def wedges(stage, path, profile, segments, color):
    return threads.cleared_wedges(plain_wedges, stage, path, profile, segments, color)

ROOT='/World/envs/env_0/refit'
TABLE=.7475
RING=np.array([.318,.112,TABLE+.0145])  # origin at middle of retaining lip
BOTTLE=np.array([.304,.018,TABLE+.00035]); HEIGHT=.071
SPARE=np.array([.301,-.131,TABLE+.00015])
TEAT=SPARE+[0,0,.0122]

def plain_wedges(stage,path,profile,segments,color):
    from pxr import UsdGeom,UsdPhysics,PhysxSchema,Gf
    n=len(profile);faces=[list(range(n-1,-1,-1)),list(range(n,2*n))]+[[i,(i+1)%n,(i+1)%n+n,i+n] for i in range(n)]
    for i in range(segments):
        p=np.array([(r*np.cos(t),r*np.sin(t),z) for t in np.arange(i,i+2)*2*np.pi/segments for r,z in profile])*.001
        mesh=UsdGeom.Mesh.Define(stage,path+f'{i:02d}');mesh.CreatePointsAttr(p.tolist());mesh.CreateFaceVertexCountsAttr([len(f) for f in faces]);mesh.CreateFaceVertexIndicesAttr(sum(faces,[]));mesh.CreateDoubleSidedAttr(True);mesh.CreateDisplayColorAttr([Gf.Vec3f(*color)]);mesh.CreateSubdivisionSchemeAttr('none')
        prim=mesh.GetPrim();UsdPhysics.CollisionAPI.Apply(prim);UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr('convexHull');col=PhysxSchema.PhysxCollisionAPI.Apply(prim);col.CreateContactOffsetAttr(.00025);col.CreateRestOffsetAttr(0.)

def body(stage,name,center,mass):
    from pxr import UsdGeom,UsdPhysics,PhysxSchema,Gf
    geom=UsdGeom.Xform.Define(stage,ROOT+'/'+name);geom.AddTranslateOp().Set(Gf.Vec3d(*center));prim=geom.GetPrim();UsdPhysics.RigidBodyAPI.Apply(prim);UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(mass)
    rb=PhysxSchema.PhysxRigidBodyAPI.Apply(prim);rb.CreateSolverPositionIterationCountAttr(64);rb.CreateMaxDepenetrationVelocityAttr(.35);PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.)
    return str(geom.GetPath())

def build(stage):
    from pxr import UsdGeom,UsdPhysics,PhysxSchema,Gf,Vt,Sdf
    from omni.physx.scripts import deformableUtils,physicsUtils
    collar=body(stage,'collar',RING,.016);white=(.86,.90,.88)
    wedges(stage,collar+'/skirt',[(28,-14),(31,-14),(31,0),(28,0)],32,white)
    wedges(stage,collar+'/lip',[(22.8,-1.2),(31,-1.2),(31,1.2),(22.8,1.2),(22.24,.96),(22,.4),(22,-.4),(22.24,-.96)],48,white)
    bottle=body(stage,'bottle',BOTTLE,.046);aqua=(.55,.68,.7)
    # Solid lower collision body and open upper neck. Thread flanks are added below.
    shape=UsdGeom.Cylinder.Define(stage,bottle+'/lower');shape.CreateRadiusAttr(.030);shape.CreateHeightAttr(.046);shape.CreateAxisAttr('Z');shape.AddTranslateOp().Set((0,0,.023));shape.CreateDisplayColorAttr([Gf.Vec3f(*aqua)]);UsdPhysics.CollisionAPI.Apply(shape.GetPrim())
    wedges(stage,bottle+'/shoulder',[(21,45),(30,45),(26.4,58),(22.8,58)],32,aqua)
    wedges(stage,bottle+'/mouth',[(22.8,58),(26.4,58),(26.4,70.6),(26.0,71),(23.2,71),(22.8,70.6)],48,aqua)
    path=ROOT+'/teat';root=UsdGeom.Xform.Define(stage,path).GetPrim();local,tet,_=specimen();p=local+TEAT
    pts=Vt.Vec3fArray.FromNumpy(p.astype('float32'));indices=Vt.Vec4iArray.FromNumpy(tet);mesh=UsdGeom.TetMesh.Define(stage,path+'/volume');mesh.CreatePointsAttr(pts);mesh.CreateTetVertexIndicesAttr(indices)
    prim=mesh.GetPrim();assert prim.ApplyAPI('OmniPhysicsVolumeDeformableSimAPI');prim.GetAttribute('omniphysics:restShapePoints').Set(pts);prim.GetAttribute('omniphysics:restTetVtxIndices').Set(indices);UsdPhysics.CollisionAPI.Apply(prim)
    col=PhysxSchema.PhysxCollisionAPI.Apply(prim);col.CreateContactOffsetAttr(.0003);col.CreateRestOffsetAttr(.00003);mesh.CreatePurposeAttr('guide')
    facets=tet[:,[[1,2,3],[0,3,2],[0,1,3],[0,2,1]]].reshape(-1,3);_,first,count=np.unique(np.sort(facets,axis=1),axis=0,return_index=True,return_counts=True);tri=facets[first[count==1]]
    mesh.CreateSurfaceFaceVertexIndicesAttr(Vt.Vec3iArray.FromNumpy(tri.astype('int32')))
    skin=UsdGeom.Mesh.Define(stage,path+'/surface');skin.CreatePointsAttr(pts);skin.CreateFaceVertexCountsAttr([3]*len(tri));skin.CreateFaceVertexIndicesAttr(tri.ravel().tolist());skin.CreateSubdivisionSchemeAttr('none');skin.CreateDisplayColorAttr([Gf.Vec3f(.69,.76,.73)])
    for node in (prim,skin.GetPrim()):
        assert node.ApplyAPI('OmniPhysicsDeformablePoseAPI','default');node.CreateAttribute('deformablePose:default:omniphysics:purposes',Sdf.ValueTypeNames.TokenArray).Set(['bindPose']);node.CreateAttribute('deformablePose:default:omniphysics:points',Sdf.ValueTypeNames.Point3fArray).Set(pts)
    assert root.ApplyAPI('OmniPhysicsDeformableBodyAPI') and root.ApplyAPI('PhysxBaseDeformableBodyAPI')
    for key,val in [('selfCollision',False),('linearDamping',2.),('solverPositionIterationCount',100)]:root.GetAttribute('physxDeformableBody:'+key).Set(val)
    mat=path+'/material';assert deformableUtils.add_deformable_material(stage,mat,density=1100.,static_friction=.55,dynamic_friction=.4,youngs_modulus=400000.,poissons_ratio=.44);physicsUtils.add_physics_material_to_prim(stage,root,mat)
    threads.add(stage, ROOT+'/bottle/thread', False)
    threads.add(stage, ROOT+'/collar/lug', True)
    add_water_load(stage, 20)


def add_water_load(stage, volume_ml):
    """An internal fixed mass approximates a small upright fill during assembly."""
    from math import pi
    from pxr import Gf, UsdGeom, UsdPhysics

    radius = .027
    mass = volume_ml * .001
    height = volume_ml * 1e-6 / (pi * radius**2)
    offset = .002 + height / 2
    path = ROOT + '/water_load'
    model = UsdGeom.Xform.Define(stage, path)
    model.AddTranslateOp().Set(Gf.Vec3d(*BOTTLE) + Gf.Vec3d(0, 0, offset))
    UsdPhysics.RigidBodyAPI.Apply(model.GetPrim())
    properties = UsdPhysics.MassAPI.Apply(model.GetPrim())
    properties.CreateMassAttr(mass)
    side = mass * (3 * radius**2 + height**2) / 12
    properties.CreateDiagonalInertiaAttr(Gf.Vec3f(side, side, mass * radius**2 / 2))
    joint = UsdPhysics.FixedJoint.Define(stage, path + '/to_bottle')
    joint.CreateBody0Rel().SetTargets([ROOT+'/bottle'])
    joint.CreateBody1Rel().SetTargets([path])
    joint.CreateLocalPos0Attr(Gf.Vec3f(0, 0, offset))
