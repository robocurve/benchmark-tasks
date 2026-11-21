"""Native contacts for a full-size cart, free wheels, case and sill."""
import math
import numpy as np
from pxr import Gf, Sdf, UsdGeom, UsdPhysics, UsdShade, PhysxSchema
import spec

def material(stage,path,color,emission=False):
    m=UsdShade.Material.Define(stage,path);s=UsdShade.Shader.Define(stage,path+'/surface')
    s.CreateIdAttr('UsdPreviewSurface');s.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    s.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(.65)
    if emission:s.CreateInput('emissiveColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    m.CreateSurfaceOutput().ConnectToSource(s.ConnectableAPI(),'surface');return m

def root(stage,path,pos,mass=None,yaw=0.,quat=None):
    r=UsdGeom.Xform.Define(stage,path);r.AddTranslateOp().Set(Gf.Vec3d(*pos))
    r.AddOrientOp().Set(quat or Gf.Quatf(math.cos(yaw/2),0,0,math.sin(yaw/2)))
    if mass is not None:
        UsdPhysics.RigidBodyAPI.Apply(r.GetPrim());UsdPhysics.MassAPI.Apply(r.GetPrim()).CreateMassAttr(mass)
        api=PhysxSchema.PhysxRigidBodyAPI.Apply(r.GetPrim())
        api.CreateSolverPositionIterationCountAttr(64);api.CreateSolverVelocityIterationCountAttr(0)
        # Limit recovery speed for transient tool/finger penetrations.
        api.CreateMaxDepenetrationVelocityAttr(.2)
        api.CreateEnableCCDAttr(True)
        PhysxSchema.PhysxContactReportAPI.Apply(r.GetPrim()).CreateThresholdAttr(0.)
    return r

def box(stage,path,size,pos,color,mass=None,yaw=0.,collision=True,mat=None):
    r=root(stage,path,pos,mass,yaw);shape=UsdGeom.Cube.Define(stage,path+'/shape')
    shape.CreateSizeAttr(1.);shape.AddScaleOp().Set(Gf.Vec3f(*size))
    if collision:
        UsdPhysics.CollisionAPI.Apply(shape.GetPrim());p=PhysxSchema.PhysxCollisionAPI.Apply(shape.GetPrim())
        p.CreateContactOffsetAttr(.00005);p.CreateRestOffsetAttr(0.)
        UsdShade.MaterialBindingAPI.Apply(shape.GetPrim()).Bind(UsdShade.Material(stage.GetPrimAtPath('/World/contact')),materialPurpose='physics')
    UsdShade.MaterialBindingAPI.Apply(shape.GetPrim()).Bind(mat or material(stage,path+'/look',color))
    return r

def cylinder(stage,path,radius,height,pos,color,mass=None,sides=32,quat=None,collision=True):
    r=root(stage,path,pos,mass,quat=quat)
    points=[(radius*math.cos(a),radius*math.sin(a),z) for z in [-height/2,height/2] for a in np.arange(sides)*2*math.pi/sides]
    faces=[list(reversed(range(sides))),list(range(sides,2*sides))]
    faces += [[i,(i+1)%sides,(i+1)%sides+sides,i+sides] for i in range(sides)]
    m=UsdGeom.Mesh.Define(stage,path+'/shape');m.CreatePointsAttr(points)
    m.CreateFaceVertexCountsAttr([len(f) for f in faces]);m.CreateFaceVertexIndicesAttr(sum(faces,[]));m.CreateSubdivisionSchemeAttr('none')
    if collision:
        UsdPhysics.CollisionAPI.Apply(m.GetPrim());UsdPhysics.MeshCollisionAPI.Apply(m.GetPrim()).CreateApproximationAttr('convexHull')
        ph=PhysxSchema.PhysxCollisionAPI.Apply(m.GetPrim());ph.CreateContactOffsetAttr(.00003);ph.CreateRestOffsetAttr(0.)
        UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(UsdShade.Material(stage.GetPrimAtPath('/World/contact')),materialPurpose='physics')
    UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(material(stage,path+'/look',color));return r

def joint(stage,name,kind,b0,b1,p0,p1,axis='Z',angle0=0.):
    j=kind.Define(stage,'/World/joints/'+name)
    j.CreateBody0Rel().SetTargets([b0] if b0 else []);j.CreateBody1Rel().SetTargets([b1])
    j.CreateLocalPos0Attr(Gf.Vec3f(*p0));j.CreateLocalPos1Attr(Gf.Vec3f(*p1))
    j.CreateLocalRot0Attr(Gf.Quatf(math.cos(angle0/2),0,0,math.sin(angle0/2)))
    j.CreateLocalRot1Attr(Gf.Quatf(1.));j.CreateCollisionEnabledAttr(True)
    j.CreateExcludeFromArticulationAttr(True)
    if hasattr(j,'CreateAxisAttr'):j.CreateAxisAttr(axis)
    return j

def build(stage):
    UsdGeom.SetStageMetersPerUnit(stage,1.);UsdGeom.SetStageUpAxis(stage,'Z')
    ph=UsdPhysics.Scene.Define(stage,'/World/physics');ph.CreateGravityDirectionAttr(Gf.Vec3f(0,0,-1));ph.CreateGravityMagnitudeAttr(9.81)
    api=PhysxSchema.PhysxSceneAPI.Apply(ph.GetPrim());api.CreateTimeStepsPerSecondAttr(spec.PHYSICS_HZ)
    api.CreateEnableGPUDynamicsAttr(False);api.CreateBroadphaseTypeAttr('MBP');api.CreateEnableCCDAttr(True)
    for name,static,dynamic in [('contact',.65,.55),('pad_contact',1.2,1.)]:
        m=UsdPhysics.MaterialAPI.Apply(UsdShade.Material.Define(stage,'/World/'+name).GetPrim())
        m.CreateStaticFrictionAttr(static);m.CreateDynamicFrictionAttr(dynamic);m.CreateRestitutionAttr(0.)
    box(stage,'/World/floor',(1.8,1.4,.06),(.35,0,-.03),(.10,.12,.14))
    box(stage,'/World/sill',(.020,1.3,spec.SILL_HEIGHT),(.380,0,spec.SILL_HEIGHT/2),(.60,.62,.65))
    for side,sign in [('left',1),('right',-1)]:
        box(stage,'/World/pedestal_'+side,(.15,.15,.950),(0,sign*.305,.475),(.35,.39,.42))
    root(stage,'/World/cart',spec.AXLE,spec.CART_MASS-.4)
    box(stage,'/World/cart/plate',spec.PLATE_SIZE,(spec.PLATE_SIZE[0]/2,0,-spec.AXLE[2]+.002),(.40,.44,.48))
    for sign in [-1,1]:
        box(stage,'/World/cart/rail_'+str(sign).replace('-','n'),(.020,.020,.900),(0,sign*.145,.450),(.62,.65,.68))
    for h in [.15,.36,.65]:
        box(stage,'/World/cart/cross_'+str(h).replace('.','_'),(.020,.300,.020),(0,0,h),(.62,.65,.68))
    box(stage,'/World/cart/handle',(.022,.340,.025),(0,0,spec.HANDLE_LOCAL_Z),(.10,.12,.14))
    cylinder(stage,'/World/cart/axle',.008,.420,(0,0,0),(.38,.40,.43),quat=Gf.Quatf(math.sqrt(.5),math.sqrt(.5),0,0))
    for side,sign in [('left',1),('right',-1)]:
        pos=np.asarray(spec.AXLE)+[0,sign*spec.WHEEL_Y,0]
        cylinder(stage,'/World/wheel_'+side,spec.WHEEL_RADIUS,spec.WHEEL_WIDTH,pos,(.055,.06,.065),.2,sides=64,quat=Gf.Quatf(math.sqrt(.5),math.sqrt(.5),0,0))
        j=joint(stage,'wheel_'+side,UsdPhysics.RevoluteJoint,'/World/cart','/World/wheel_'+side,(0,sign*spec.WHEEL_Y,0),(0,0,0),'Y')
        j.CreateLocalRot1Attr(Gf.Quatf(math.sqrt(.5),-math.sqrt(.5),0,0));j.GetCollisionEnabledAttr().Set(False)
        d=UsdPhysics.DriveAPI.Apply(j.GetPrim(),'angular');d.CreateTypeAttr('force');d.CreateStiffnessAttr(0.)
        d.CreateDampingAttr(.002*math.pi/180);d.CreateTargetVelocityAttr(0.);d.CreateMaxForceAttr(.01)
        # Visible side-wall mark for ordinary video inspection of wheel rotation.
        box(stage,'/World/wheel_'+side+'/mark',(.045,.004,.0004),(.025,0,.0143),(.8,.8,.8),collision=False)
    box(stage,'/World/case',spec.CASE_SIZE,spec.CASE_START,(.82,.45,.06),spec.CASE_MASS)
    box(stage,'/World/case/lid_seam',(.221,.301,.002),(0,0,.05),(.16,.16,.15),collision=False)
    return {name:'/World/'+name for name in ['cart','wheel_left','wheel_right','case']}
