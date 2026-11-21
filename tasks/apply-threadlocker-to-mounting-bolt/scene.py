"""M16 bolt and semi-solid threadlocker applicator on a maintenance bench."""
import math
from pxr import Gf,Sdf,UsdGeom,UsdPhysics,UsdShade,PhysxSchema
import spec

def root(stage,path,pos,mass=None):
    x=UsdGeom.Xform.Define(stage,path);x.AddTranslateOp().Set(Gf.Vec3d(*pos));x.AddOrientOp().Set(Gf.Quatf(1.))
    if mass is not None:
        UsdPhysics.RigidBodyAPI.Apply(x.GetPrim());UsdPhysics.MassAPI.Apply(x.GetPrim()).CreateMassAttr(mass)
        p=PhysxSchema.PhysxRigidBodyAPI.Apply(x.GetPrim());p.CreateSolverPositionIterationCountAttr(96);p.CreateSolverVelocityIterationCountAttr(0);p.CreateSleepThresholdAttr(0.);p.CreateStabilizationThresholdAttr(0.);PhysxSchema.PhysxContactReportAPI.Apply(x.GetPrim()).CreateThresholdAttr(0.)
    return x

def look(stage,path,color):
    m=UsdShade.Material.Define(stage,path);s=UsdShade.Shader.Define(stage,path+'/surface');s.CreateIdAttr('UsdPreviewSurface')
    s.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color));s.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(.45)
    m.CreateSurfaceOutput().ConnectToSource(s.ConnectableAPI(),'surface');return m

def collision(stage,prim):
    UsdPhysics.CollisionAPI.Apply(prim)
    if prim.IsA(UsdGeom.Mesh):UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr('convexHull')
    p=PhysxSchema.PhysxCollisionAPI.Apply(prim);p.CreateContactOffsetAttr(.0001);p.CreateRestOffsetAttr(0.)
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(UsdShade.Material(stage.GetPrimAtPath('/World/contact')),materialPurpose='physics')

def box(stage,path,size,pos,color):
    x=root(stage,path,pos);s=UsdGeom.Cube.Define(stage,path+'/shape');s.CreateSizeAttr(1.);s.AddScaleOp().Set(Gf.Vec3f(*size));collision(stage,s.GetPrim())
    UsdShade.MaterialBindingAPI.Apply(s.GetPrim()).Bind(look(stage,path+'/look',color));return x







def cylinder(stage,path,radius,length,pos,color,axis='X'):
    x=root(stage,path,pos);shape=UsdGeom.Cylinder.Define(stage,path+'/shape')
    shape.CreateAxisAttr(axis);shape.CreateRadiusAttr(radius);shape.CreateHeightAttr(length)
    collision(stage,shape.GetPrim());UsdShade.MaterialBindingAPI.Apply(shape.GetPrim()).Bind(look(stage,path+'/look',color));return x

def polygon(stage,path,radius,height,z,n,color,collide=True):
    pts=[(radius*math.cos(2*math.pi*i/n),radius*math.sin(2*math.pi*i/n),z+q) for q in [-height/2,height/2] for i in range(n)]
    faces=[list(range(n-1,-1,-1)),list(range(n,2*n))]+[[i,(i+1)%n,(i+1)%n+n,i+n] for i in range(n)]
    return mesh(stage,path,pts,faces,color,collide)
def mesh(stage,path,points,faces,color,collide=True):
    m=UsdGeom.Mesh.Define(stage,path);m.CreatePointsAttr(points);m.CreateFaceVertexCountsAttr([len(f) for f in faces]);m.CreateFaceVertexIndicesAttr(sum(faces,[]));m.CreateSubdivisionSchemeAttr('none')
    if collide:collision(stage,m.GetPrim())
    m.CreateDisplayColorAttr([Gf.Vec3f(*color)]);return m
def build(stage):
    import thread_geometry
    for name,mu in [('contact',.35),('pad_contact',1.2)]:
        m=UsdShade.Material.Define(stage,'/World/'+name);p=UsdPhysics.MaterialAPI.Apply(m.GetPrim());p.CreateStaticFrictionAttr(mu);p.CreateDynamicFrictionAttr(1. if name=='pad_contact' else .30);p.CreateRestitutionAttr(0.)
    steel=(.15,.17,.20);grey=(.40,.42,.44)
    box(stage,'/World/floor',(1.6,1.5,.06),(.5,0,-.03),(.4,.4,.4))
    box(stage,'/World/table',(1.10,1.30,.06),(.5,0,.47),(.35,.29,.23))
    for sign in [-1,1]:box(stage,f'/World/pedestal_{str(sign).replace("-","n")}',(.15,.15,.75),(0,sign*.305,.375),grey)
    root(stage,'/World/bolt',spec.BOLT_START,spec.BOLT_MASS)
    polygon(stage,'/World/bolt/core',spec.MINOR_RADIUS,spec.BOLT_LENGTH,spec.BOLT_LENGTH/2,48,steel).MakeInvisible()
    polygon(stage,'/World/bolt/shank_proxy',spec.MAJOR_RADIUS,spec.BOLT_LENGTH-.016,(spec.BOLT_LENGTH+.016)/2,48,steel).MakeInvisible()
    polygon(stage,'/World/bolt/head',spec.HEAD_AF/math.sqrt(3),spec.HEAD_HEIGHT,spec.BOLT_LENGTH+spec.HEAD_HEIGHT/2,6,steel)
    for i,(pts,faces) in enumerate(thread_geometry.ridge_segments()):mesh(stage,f'/World/bolt/thread_contact_{i}',pts,faces,steel).MakeInvisible()
    pts,faces,uv=thread_geometry.surface();visual=mesh(stage,'/World/bolt/thread_surface',pts,faces,steel,False)
    visual.GetDisplayColorPrimvar().SetInterpolation('uniform');visual.GetDisplayColorAttr().Set([Gf.Vec3f(*steel)]*len(faces))
    # An open bore supports only the tip during reset; the head is accessible.
    cx,cy,cz=spec.BOLT_START
    for i in range(24):
        a=2*math.pi*i/24;b=2*math.pi*(i+1)/24
        pts=[(cx+r*math.cos(t),cy+r*math.sin(t),z) for z in [.502,.516] for r,t in [(.018,a),(.018,b),(.0087,b),(.0087,a)]]
        mesh(stage,f'/World/start_nest_{i}',pts,[[0,3,2,1],[4,5,6,7],[0,1,5,4],[1,2,6,5],[2,3,7,6],[3,0,4,7]],(.20,.30,.35))
    box(stage,'/World/start_floor',(.040,.040,.004),(cx,cy,.502),(.20,.30,.35))
    root(stage,'/World/tool',spec.TOOL_START,spec.TOOL_MASS)
    box(stage,'/World/tool/handle',(.080,.024,.024),(0,0,0),(.74,.76,.80))
    # Fresh rectangular wax plug in a dimensioned carrier; no guessed branded tube.
    wx,wy,wz=spec.TOOL_START;root(stage,'/World/wax',(wx+spec.WAX_OFFSET,wy,wz),spec.MASSES['wax'])
    box(stage,'/World/wax/active',spec.WAX_SIZE,(0,0,0),(.015,.12,.85))
    j=UsdPhysics.PrismaticJoint.Define(stage,'/World/wax_compliance');j.CreateBody0Rel().SetTargets(['/World/tool']);j.CreateBody1Rel().SetTargets(['/World/wax']);j.CreateAxisAttr('X');j.CreateLocalPos0Attr(Gf.Vec3f(spec.WAX_OFFSET,0,0));j.CreateLocalPos1Attr(Gf.Vec3f(0));j.CreateLowerLimitAttr(-.002);j.CreateUpperLimitAttr(0.);j.CreateExcludeFromArticulationAttr(True)
    d=UsdPhysics.DriveAPI.Apply(j.GetPrim(),'linear');d.CreateTypeAttr('force');d.CreateStiffnessAttr(1000.);d.CreateDampingAttr(1.);d.CreateMaxForceAttr(3.);d.CreateTargetPositionAttr(0.)
    for x in [.250,.300]:
        box(stage,f'/World/tool_rest_{str(x).replace(".","_")}',(.012,.060,.028),(x,-.200,.514),(.20,.30,.35))
    return {n:'/World/'+n for n in ['bolt','tool','wax']}
