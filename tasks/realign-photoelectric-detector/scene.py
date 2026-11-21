"""Native contacts for the mount, loose driver and optical test block."""
import math
import numpy as np
from pxr import Gf, Sdf, UsdGeom, UsdPhysics, UsdShade, PhysxSchema
import spec
import clamp

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
        api.CreateSolverPositionIterationCountAttr(255);api.CreateSolverVelocityIterationCountAttr(0)
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

def socket_wall(stage,path,angle):
    """Estimated 0.3 mm radial, 1 mm deep entry bevel on an 8 mm socket."""
    root(stage,path,(0,0,0),yaw=angle)
    section=[(.00545,0.),(.00545,.012),(.00405,.012),(.00405,.001),(.00435,0.)]
    points=[(r,t,z) for t in [-.00275,.00275] for r,z in section]
    faces=[list(range(5)),list(reversed(range(5,10)))]
    faces += [[i,i+5,(i+1)%5+5,(i+1)%5] for i in range(5)]
    shape=UsdGeom.Mesh.Define(stage,path+'/shape');shape.CreatePointsAttr(points)
    shape.CreateFaceVertexCountsAttr([len(f) for f in faces]);shape.CreateFaceVertexIndicesAttr(sum(faces,[]))
    shape.CreateSubdivisionSchemeAttr('none')
    UsdPhysics.CollisionAPI.Apply(shape.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(shape.GetPrim()).CreateApproximationAttr('convexHull')
    ph=PhysxSchema.PhysxCollisionAPI.Apply(shape.GetPrim());ph.CreateContactOffsetAttr(.00003);ph.CreateRestOffsetAttr(0.)
    UsdShade.MaterialBindingAPI.Apply(shape.GetPrim()).Bind(UsdShade.Material(stage.GetPrimAtPath('/World/contact')),materialPurpose='physics')
    UsdShade.MaterialBindingAPI(shape.GetPrim()).Bind(material(stage,path+'/look',(.55,.58,.60)))

def build(stage):
    UsdGeom.SetStageMetersPerUnit(stage,1.);UsdGeom.SetStageUpAxis(stage,'Z')
    ph=UsdPhysics.Scene.Define(stage,'/World/physics');ph.CreateGravityDirectionAttr(Gf.Vec3f(0,0,-1));ph.CreateGravityMagnitudeAttr(9.81)
    api=PhysxSchema.PhysxSceneAPI.Apply(ph.GetPrim());api.CreateTimeStepsPerSecondAttr(spec.PHYSICS_HZ)
    api.CreateEnableGPUDynamicsAttr(False);api.CreateBroadphaseTypeAttr('MBP');api.CreateEnableCCDAttr(True)
    contact=UsdShade.Material.Define(stage,'/World/contact');m=UsdPhysics.MaterialAPI.Apply(contact.GetPrim())
    m.CreateStaticFrictionAttr(spec.FRICTION);m.CreateDynamicFrictionAttr(spec.FRICTION);m.CreateRestitutionAttr(0.)
    pad=UsdShade.Material.Define(stage,'/World/pad_contact')
    m=UsdPhysics.MaterialAPI.Apply(pad.GetPrim())
    m.CreateStaticFrictionAttr(spec.PAD_STATIC_FRICTION);m.CreateDynamicFrictionAttr(spec.PAD_DYNAMIC_FRICTION);m.CreateRestitutionAttr(0.)
    axial=UsdShade.Material.Define(stage,'/World/axial_contact')
    m=UsdPhysics.MaterialAPI.Apply(axial.GetPrim())
    m.CreateStaticFrictionAttr(0.);m.CreateDynamicFrictionAttr(0.);m.CreateRestitutionAttr(0.)
    PhysxSchema.PhysxMaterialAPI.Apply(axial.GetPrim()).CreateFrictionCombineModeAttr('min')
    box(stage,'/World/table',(1.1,1.3,.06),(.36,0,spec.TABLE_Z-.03),(.42,.29,.18))
    # Fixed conveyor section and bracket pedestal; fixed reflector at its opposite side.
    # Conveyor travel is along Y; the beam crosses its width along X.
    reset_force,reset_torque,reset_plate_cap=clamp.load(0.)
    x0=spec.CLAMP_POST[0]
    box(stage,'/World/frame',(.36,.36,.045),(x0+.220,.09,spec.TABLE_Z+.0225),(.35,.39,.42))
    box(stage,'/World/belt',(.33,.34,.003),(x0+.225,.09,spec.TABLE_Z+.0465),(.055,.06,.065))
    box(stage,'/World/pedestal',(.065,.045,.029),(x0,.0273,spec.TABLE_Z+.0145),(.28,.31,.34))
    box(stage,'/World/pedestal_bridge',(.070,.045,.006),(x0+.035,.0273,spec.TABLE_Z+.026),(.28,.31,.34))
    # E39-L261 on a vertical 12 mm post: post yaw and height are adjustable;
    # the circular sensor plate pitches about a horizontal axis. The M5 screw
    # axis is horizontal and perpendicular to both axes, as in the drawing.
    post=np.asarray(spec.CLAMP_POST)
    cylinder(stage,'/World/post',.006,.060,(post[0],post[1],spec.TABLE_Z+.0415),(.48,.51,.54))
    block=root(stage,'/World/clamp_body',post,.070,spec.INITIAL_YAW)
    for name,size,position in [
        ('left',(.005,.0423,.023),(-.009,.00385,0)),
        ('right',(.005,.0423,.023),(.009,.00385,0)),
        ('back',(.013,.012,.023),(0,.019,0)),
        ('front',(.013,.0108,.023),(0,-.0119,0))]:
        box(stage,'/World/clamp_body/'+name,size,position,(.55,.57,.60))
    j=joint(stage,'post_clamp',UsdPhysics.Joint,None,'/World/clamp_body',post,(0,0,0))
    for axis in ('transX','transY','rotX','rotY'):
        lim=UsdPhysics.LimitAPI.Apply(j.GetPrim(),axis);lim.CreateLowAttr(1.);lim.CreateHighAttr(-1.)
    lim=UsdPhysics.LimitAPI.Apply(j.GetPrim(),'transZ');lim.CreateLowAttr(-.025);lim.CreateHighAttr(.025)
    for axis in ('transZ','rotZ'):
        d=UsdPhysics.DriveAPI.Apply(j.GetPrim(),axis);d.CreateTypeAttr('force')
        d.CreateStiffnessAttr(spec.CONTACT_LINEAR_STIFFNESS if axis=='transZ' else spec.CONTACT_ANGULAR_STIFFNESS*math.pi/180)
        d.CreateDampingAttr(spec.CONTACT_LINEAR_DAMPING if axis=='transZ' else spec.CONTACT_ANGULAR_DAMPING*math.pi/180)
        d.CreateTargetPositionAttr(0. if axis=='transZ' else math.degrees(spec.INITIAL_YAW))
        d.CreateTargetVelocityAttr(0.);d.CreateMaxForceAttr(spec.FRICTION*reset_force*(.006 if axis=='rotZ' else 1.))
    mount=root(stage,'/World/mount',spec.PIVOT,.155,spec.INITIAL_YAW)
    load=PhysxSchema.PhysxForceAPI.Apply(mount.GetPrim())
    load.CreateModeAttr('force');load.CreateWorldFrameEnabledAttr(True)
    load.CreateForceEnabledAttr(True);load.CreateForceAttr(Gf.Vec3f(0.));load.CreateTorqueAttr(Gf.Vec3f(0.))
    # Supplied circular plate plus an explicitly custom adapter for the retained
    # SICK sensor bracket. No claim of direct manufacturer-listed compatibility.
    cylinder(stage,'/World/mount/clamp_plate',.012,.005,(0,.010,0),(.5,.52,.55),
             quat=Gf.Quatf(math.sqrt(.5),math.sqrt(.5),0,0))
    box(stage,'/World/mount/adapter',(.030,.004,.024),(.012,.006,.006),(.5,.52,.55))
    box(stage,'/World/mount/foot',(.030,.0235,.002),(-.006,0,0),(.5,.52,.55))
    box(stage,'/World/mount/upright',(.002,.019,.0465),(.009,0,.02425),(.5,.52,.55))
    box(stage,'/World/mount/housing',spec.SENSOR_SIZE,spec.SENSOR_CENTER_LOCAL,(.08,.22,.55))
    box(stage,'/World/mount/lens',(.001,.012,.020),spec.LENS_LOCAL,(.10,.012,.018),collision=False)
    led=material(stage,'/World/led_material',(.025,.02,.0),emission=True)
    box(stage,'/World/mount/indicator',(.005,.005,.001),(.031,0,.050),(0,0,0),collision=False,mat=led)
    j=joint(stage,'mount',UsdPhysics.RevoluteJoint,'/World/clamp_body','/World/mount',spec.PLATE_OFFSET,(0,.010,0),axis='Y')
    j.GetCollisionEnabledAttr().Set(False)  # Reduced groove capture, no double-counted friction.
    d=UsdPhysics.DriveAPI.Apply(j.GetPrim(),'angular');d.CreateTypeAttr('force')
    d.CreateStiffnessAttr(spec.CONTACT_ANGULAR_STIFFNESS*math.pi/180);d.CreateTargetPositionAttr(0.)
    d.CreateDampingAttr(spec.CONTACT_ANGULAR_DAMPING*math.pi/180)
    d.CreateTargetVelocityAttr(0.);d.CreateMaxForceAttr(reset_plate_cap)
    # The supplied M5 nut turns on a captive counter-held screw. Axial thread
    # micromotion is reduced to compression, with native rotational resistance.
    def rotate_z(v):
        c,s=math.cos(spec.INITIAL_YAW),math.sin(spec.INITIAL_YAW)
        return np.array([c*v[0]-s*v[1],s*v[0]+c*v[1],v[2]])
    bolt_offset=np.array(spec.CLAMP)-post
    bolt_pos=post+rotate_z(bolt_offset)
    axis_q=Gf.Quatf(math.sqrt(.5),0,-math.sqrt(.5),0)
    yaw_q=Gf.Quatf(math.cos(spec.INITIAL_YAW/2),0,0,math.sin(spec.INITIAL_YAW/2))
    cylinder(stage,'/World/bolt',.008/math.sqrt(3),.0035,bolt_pos,(.45,.48,.50),.004,sides=6,quat=yaw_q*axis_q)
    box(stage,'/World/bolt/witness',(.003,.0008,.0002),(.002,0,.00185),(.015,.015,.015),collision=False)
    j=joint(stage,'bolt',UsdPhysics.RevoluteJoint,'/World/clamp_body','/World/bolt',bolt_offset,(0,0,0))
    j.GetLocalRot0Attr().Set(axis_q);j.GetCollisionEnabledAttr().Set(False)
    j.CreateLowerLimitAttr(-170.);j.CreateUpperLimitAttr(20.)
    d=UsdPhysics.DriveAPI.Apply(j.GetPrim(),'angular');d.CreateTypeAttr('force')
    d.CreateStiffnessAttr(spec.CONTACT_ANGULAR_STIFFNESS*math.pi/180);d.CreateTargetPositionAttr(0.)
    d.CreateDampingAttr(spec.CONTACT_ANGULAR_DAMPING*math.pi/180)
    d.CreateTargetVelocityAttr(0.);d.CreateMaxForceAttr(spec.RESET_TORQUE)
    box(stage,'/World/reflector_post',(.012,.018,.059),(spec.REFLECTOR[0]+.004,0,spec.TABLE_Z+.0295),(.35,.39,.42))
    box(stage,'/World/reflector',spec.REFLECTOR_SIZE,spec.REFLECTOR,(.50,.045,.02))
    # Calibrated spring-indicating torque wrench: two independent rigid bodies.
    # The seated hex is reduced to the sliding-key constraint defined below.
    # A 120 mm lever keeps forces attainable; grasping remains contact driven.
    socket=root(stage,'/World/socket',spec.DRIVER_POS,.100,quat=yaw_q*axis_q)
    head=spec.DRIVER_HEAD_Z
    box(stage,'/World/socket/shaft',(.012,.012,head-.023),(0,0,(head+.001)/2),(.55,.58,.60))
    cylinder(stage,'/World/socket/clutch_disk',.018,.010,(0,0,head-.006),(.72,.72,.70))
    box(stage,'/World/socket/index',(.030,.002,.0005),(0,0,head-.0005),(.03,.03,.03),collision=False)
    for i in range(6):socket_wall(stage,f'/World/socket/socket_{i}',math.pi/6+i*math.pi/3)
    driver=root(stage,'/World/driver',spec.DRIVER_POS,.120,quat=yaw_q*axis_q)
    box(stage,'/World/driver/beam',(.120,.012,.008),(.060,0,head+.005),(.55,.58,.60))
    box(stage,'/World/driver/handle',(.035,.020,.020),spec.DRIVER_GRIP_LOCAL,(.10,.10,.11))
    box(stage,'/World/driver/index',(.034,.002,.0005),(0,0,head+.010),(.95,.65,.03),collision=False)
    # A geared dial makes the measured spring deflection readable in RGB.
    # The internal pointer gearing is reduced to its calibrated 90 deg/Nm
    # transfer, not an additional policy observation or score indication.
    cylinder(stage,'/World/socket/dial',.030,.001,(0,0,head+.011),(.90,.90,.85))
    for value in [0.,1.,2.,3.,3.5,4.]:
        a=math.radians(-value*spec.DIAL_DEG_PER_NM)
        box(stage,f'/World/socket/tick_{str(value).replace(".","_")}',(.004,.0008,.0003),
            (.026*math.cos(a),.026*math.sin(a),head+.0117),(.03,.03,.03),yaw=a,collision=False)
    for i,value in enumerate(np.linspace(spec.INDICATED_TORQUE_MIN,spec.INDICATED_TORQUE_MAX,10)):
        a=math.radians(-value*spec.DIAL_DEG_PER_NM)
        box(stage,f'/World/socket/target_{i}',(.005,.0008,.0003),
            (.0255*math.cos(a),.0255*math.sin(a),head+.0118),(.02,.55,.08),yaw=a,collision=False)
    root(stage,'/World/socket/needle',(0,0,head+.0122))
    box(stage,'/World/socket/needle/line',(.027,.0015,.0004),(.0135,0,0),(.04,.04,.04),collision=False)
    j=joint(stage,'torque_spring',UsdPhysics.RevoluteJoint,'/World/socket','/World/driver',(0,0,head),(0,0,head))
    j.GetCollisionEnabledAttr().Set(False)
    d=UsdPhysics.DriveAPI.Apply(j.GetPrim(),'angular');d.CreateTypeAttr('force')
    d.CreateStiffnessAttr(spec.DRIVER_SPRING_NM_RAD*math.pi/180);d.CreateTargetPositionAttr(0.)
    d.CreateDampingAttr(spec.DRIVER_SPRING_DAMPING*math.pi/180)
    d.CreateTargetVelocityAttr(0.);d.CreateMaxForceAttr(100.)
    # Reduced positive hex engagement: a sliding keyed connection transmits
    # torque and constrains transverse clearance while allowing withdrawal.
    # It has no connection to a hand and never sets a body pose or velocity.
    j=joint(stage,'socket_key',UsdPhysics.Joint,'/World/bolt','/World/socket',(0,0,0),(0,0,0))
    j.GetCollisionEnabledAttr().Set(False)
    for axis in ('transX','transY'):
        lim=UsdPhysics.LimitAPI.Apply(j.GetPrim(),axis)
        lim.CreateLowAttr(-.00005);lim.CreateHighAttr(.00005)
    for axis in ('rotX','rotY'):
        lim=UsdPhysics.LimitAPI.Apply(j.GetPrim(),axis)
        lim.CreateLowAttr(1.);lim.CreateHighAttr(-1.)
    lim=UsdPhysics.LimitAPI.Apply(j.GetPrim(),'rotZ')
    lim.CreateLowAttr(-.5);lim.CreateHighAttr(.5)
    box(stage,'/World/card_pad',(.075,.075,.001),(*spec.CARD_START[:2],spec.TABLE_Z+.0005),(.12,.30,.10))
    box(stage,'/World/card',spec.CARD_SIZE,spec.CARD_START,(.80,.22,.025),.045)
    return {name:'/World/'+name for name in ['clamp_body','mount','bolt','driver','socket','card']}
