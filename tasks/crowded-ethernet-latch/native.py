"""Actual passive native retention and flexible rigid-link cable approximation.

No latch release flags, body pose writes, distance-triggered welds or software
retention forces. Joined latch/plug contact is disabled because the compliant
clip is represented by a spring slider rather than a bending plastic beam.
"""
import math
from pathlib import Path
from pxr import Usd,UsdGeom,UsdPhysics,PhysxSchema,Gf
from config import *
from geometry import socket_boxes,plug_boxes,hook_mesh

def collider(shape):
    UsdPhysics.CollisionAPI.Apply(shape.GetPrim())
    c=PhysxSchema.PhysxCollisionAPI.Apply(shape.GetPrim());c.CreateContactOffsetAttr(.0001);c.CreateRestOffsetAttr(0.)

def cube(stage,path,size,pos,color):
    c=UsdGeom.Cube.Define(stage,path);c.CreateSizeAttr(1.)
    c.AddTranslateOp().Set(Gf.Vec3d(*pos));c.AddScaleOp().Set(Gf.Vec3f(*size));c.CreateDisplayColorAttr([Gf.Vec3f(*color)]);collider(c)
    return c

def wedge(stage,path,color):
    # Leading +x face slopes down; trailing -x face retains against the ledge.
    m=UsdGeom.Mesh.Define(stage,path)
    v,faces=hook_mesh()
    m.CreatePointsAttr([Gf.Vec3f(*p) for p in v])
    m.CreateFaceVertexCountsAttr([len(f) for f in faces]);m.CreateFaceVertexIndicesAttr([i for f in faces for i in f]);m.CreateSubdivisionSchemeAttr('none');m.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    collider(m);UsdPhysics.MeshCollisionAPI.Apply(m.GetPrim()).CreateApproximationAttr('convexHull')

def export(path,name):
    stage=Usd.Stage.CreateNew(str(path));UsdGeom.SetStageMetersPerUnit(stage,1.);UsdGeom.SetStageUpAxis(stage,'Z')
    root=UsdGeom.Xform.Define(stage,'/Object');stage.SetDefaultPrim(root.GetPrim())
    color=(.82,.86,.88)
    if name=='switch':
        # Internal guides prevent the hanging cable from pitching a seated
        # shoulder into the latch roof. Dimensions remain a candidate model.
        colors={'case':(.55,.59,.63),'stand':(.25,.28,.31),'socket':color,'contact':(.76,.59,.18)}
        for label,center,size,kind in socket_boxes():
            cube(stage,'/Object/'+label,size,center,colors[kind])
    else:
        UsdPhysics.RigidBodyAPI.Apply(root.GetPrim())
        mass=.0003 if name.startswith('clip') else .008 if name.startswith('plug') else .001
        UsdPhysics.MassAPI.Apply(root.GetPrim()).CreateMassAttr(mass)
        if name.startswith(('plug','clip')):
            # Resolve the slider's locked angular/transverse axes accurately
            # under an off-centre tab load; no added inertia or spring force.
            body=PhysxSchema.PhysxRigidBodyAPI.Apply(root.GetPrim())
            body.CreateSolverPositionIterationCountAttr(64)
            body.CreateSolverVelocityIterationCountAttr(16)
        PhysxSchema.PhysxContactReportAPI.Apply(root.GetPrim()).CreateThresholdAttr(0.)
        if name.startswith('plug'):
            i=int(name.split('_')[1]);col=(1.,.38,.025) if i==SERVICE else (.05,.36,.75)
            for label,center,size in plug_boxes():cube(stage,'/Object/'+label,size,center,color)
            boot=UsdGeom.Cylinder.Define(stage,'/Object/boot');boot.CreateRadiusAttr(.0035);boot.CreateHeightAttr(.014);boot.CreateAxisAttr('X');boot.AddTranslateOp().Set(Gf.Vec3d(-.0205,0,-.0005));boot.CreateDisplayColorAttr([Gf.Vec3f(*col)]);collider(boot)
        elif name.startswith('clip'):
            cube(stage,'/Object/tab',(.020,.005,.001),(-.003,0,0),color);wedge(stage,'/Object/hook',color)
        else:
            _,i,k=name.split('_');i=int(i);k=int(k);points=cable_points(i);a,b=points[k:k+2]
            delta=Gf.Vec3d(*[v-u for u,v in zip(a,b)])
            c=UsdGeom.Capsule.Define(stage,'/Object/cable');c.CreateRadiusAttr(CABLE_RADIUS);c.CreateHeightAttr(delta.GetLength());c.CreateAxisAttr('Z')
            q=Gf.Rotation(Gf.Vec3d(0,0,1),delta).GetQuat();c.AddOrientOp().Set(Gf.Quatf(q.GetReal(),Gf.Vec3f(*q.GetImaginary())))
            c.CreateDisplayColorAttr([Gf.Vec3f(*((1.,.38,.025) if i==SERVICE else ((.02,.4,.8) if i%2 else (.03,.58,.28))))]);collider(c)
    stage.GetRootLayer().Save();return str(Path(path).resolve())

def install_joints(stage,parent):
    for i in USED:
        plug=f'{parent}/plug_{i}';clip=f'{parent}/clip_{i}'
        j=UsdPhysics.PrismaticJoint.Define(stage,plug+'/clip_spring');j.CreateBody0Rel().SetTargets([plug]);j.CreateBody1Rel().SetTargets([clip]);j.CreateAxisAttr('Z');j.CreateLocalPos0Attr(Gf.Vec3f(0,0,CLIP_Z));j.CreateLocalPos1Attr(Gf.Vec3f(0,0,0));j.CreateLowerLimitAttr(-CLIP_TRAVEL);j.CreateUpperLimitAttr(0.);j.CreateCollisionEnabledAttr(False);j.CreateExcludeFromArticulationAttr(True)
        d=UsdPhysics.DriveAPI.Apply(j.GetPrim(),'linear');d.CreateTypeAttr('force');d.CreateTargetPositionAttr(0.);d.CreateStiffnessAttr(CLIP_STIFFNESS);d.CreateDampingAttr(.04)
        points=cable_points(i)
        for k in range(LINKS):
            b=f'{parent}/wire_{i}_{k}';a=plug if k==0 else f'{parent}/wire_{i}_{k-1}'
            j=UsdPhysics.SphericalJoint.Define(stage,b+'/bend');j.CreateBody0Rel().SetTargets([a]);j.CreateBody1Rel().SetTargets([b]);j.CreateExcludeFromArticulationAttr(True);j.CreateCollisionEnabledAttr(False)
            anchor=points[k]
            center_a=port(i) if k==0 else tuple((u+v)/2 for u,v in zip(points[k-1],points[k]))
            center_b=tuple((u+v)/2 for u,v in zip(points[k],points[k+1]))
            j.CreateLocalPos0Attr(Gf.Vec3f(*[u-v for u,v in zip(anchor,center_a)]));j.CreateLocalPos1Attr(Gf.Vec3f(*[u-v for u,v in zip(anchor,center_b)]));j.CreateConeAngle0LimitAttr(35.);j.CreateConeAngle1LimitAttr(35.)
            PhysxSchema.PhysxJointAPI.Apply(j.GetPrim()).CreateJointFrictionAttr(.0002)
