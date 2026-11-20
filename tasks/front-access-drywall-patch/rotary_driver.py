"""Native rotary screwdriver and coarse-thread test coupon. Dimensions in metres."""
import math
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from surface_process_probe import WORLD, box, body

PITCH = .0254 / 8
LENGTH = .03175
HEAD_RADIUS = .325 * .0254 / 2
TIP_X = .078
START_PROUD = 8 * PITCH
RPM = 75.
STEEL = (.29, .31, .33)
RUBBER = (.16, .19, .20)
SHELL = (.18, .41, .45)


def orient(shape, rotation):
    from pxr import Gf, UsdGeom
    x,y,z,w=rotation.as_quat()
    xf=UsdGeom.Xformable(shape.GetPrim())
    ops=xf.GetOrderedXformOps()
    q=xf.AddOrientOp()
    q.Set(Gf.Quatf(float(w),Gf.Vec3f(float(x),float(y),float(z))))
    # Rotate before applying a cube's nonuniform scale.
    scales=[op for op in ops if op.GetOpType()==UsdGeom.XformOp.TypeScale]
    xf.SetXformOpOrder([op for op in ops if op not in scales]+[q]+scales)


def cyl(stage,path,x,length,radius,color):
    from pxr import Gf, UsdGeom, UsdPhysics, PhysxSchema
    s=UsdGeom.Cylinder.Define(stage,path)
    s.CreateAxisAttr("X"); s.CreateHeightAttr(length); s.CreateRadiusAttr(radius)
    s.AddTranslateOp().Set(Gf.Vec3d(x,0,0))
    s.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    UsdPhysics.CollisionAPI.Apply(s.GetPrim())
    c=PhysxSchema.PhysxCollisionAPI.Apply(s.GetPrim())
    c.CreateContactOffsetAttr(.000015); c.CreateRestOffsetAttr(0.)
    return s


def small_box(stage,path,xyz,size,color):
    """Shared shader via displayColor; no material prim per thread facet."""
    from pxr import Gf, UsdGeom, UsdPhysics, PhysxSchema
    s=UsdGeom.Cube.Define(stage,path)
    s.CreateSizeAttr(1.)
    s.AddTranslateOp().Set(Gf.Vec3d(*map(float,xyz)))
    s.AddScaleOp().Set(Gf.Vec3f(*map(float,size)))
    s.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    UsdPhysics.CollisionAPI.Apply(s.GetPrim())
    api=PhysxSchema.PhysxCollisionAPI.Apply(s.GetPrim())
    api.CreateContactOffsetAttr(.000015); api.CreateRestOffsetAttr(0.)
    return s


def joint(stage, schema, name, parent, child, anchor0, anchor1):
    from pxr import Gf
    j=schema.Define(stage,WORLD+"/"+name)
    if parent:
        j.CreateBody0Rel().SetTargets([WORLD+"/"+parent])
    j.CreateBody1Rel().SetTargets([WORLD+"/"+child])
    j.CreateLocalPos0Attr(Gf.Vec3f(*map(float,anchor0)))
    j.CreateLocalPos1Attr(Gf.Vec3f(*map(float,anchor1)))
    j.CreateAxisAttr("X")
    j.CreateExcludeFromArticulationAttr(True)
    j.CreateCollisionEnabledAttr(False)
    return j


def build_driver(stage,grip,depth_sleeve=False,cross_phase=0.):
    """Free straight electric driver with a separate rotating spindle and cross bit."""
    from pxr import UsdPhysics
    shell=WORLD+"/driver"
    body(stage,shell,grip,.294)
    cyl(stage,shell+"/rear_bumper",-.108,.012,.017,RUBBER)
    cyl(stage,shell+"/battery",-.074,.061,.0175,SHELL)
    cyl(stage,shell+"/grip",-.013,.065,.018,RUBBER)
    cyl(stage,shell+"/motor",.029,.021,.017,SHELL)
    cyl(stage,shell+"/clutch",.044,.014,.014,RUBBER)
    cyl(stage,shell+"/nose",.053,.007,.010,STEEL)
    # Shallow flats to stop rolling on the bench and give the fingers a purchase.
    for sign in [-1,1]:
        small_box(stage,shell+f"/grip_flat_{sign+1}",(-.012,sign*.0165,0),(.058,.0035,.017),RUBBER)
    small_box(stage,shell+"/switch",(-.054,0,.017),(.015,.008,.003),RUBBER)
    if depth_sleeve:
        # Independent of the spindle. The annulus touches the intact face,
        # outside the head, when the bit protrudes 1.72 mm beyond the rim.
        front=TIP_X-.00172
        for i in range(16):
            theta=i*2*math.pi/16
            outer=small_box(stage,shell+f"/sleeve_{i}",
                ((.054+front)/2,.00625*math.cos(theta),.00625*math.sin(theta)),
                (front-.054,.0024,.00258),RUBBER)
            orient(outer,Rotation.from_euler("x",theta))
    rotor=WORLD+"/spindle"
    body(stage,rotor,grip,.016)
    cyl(stage,rotor+"/chuck",.057,.012,.0076,STEEL)
    cyl(stage,rotor+"/bit_shank",.065,.012,.0031,STEEL)
    cyl(stage,rotor+"/bit_neck",.073,.0065,.0017,STEEL)
    for i,(dy,dz) in enumerate([(.0048,.00066),(.00066,.0048)]):
        blade=small_box(stage,rotor+f"/blade_{i}",(.077,0,0),(.002,dy,dz),STEEL)
        if cross_phase:
            orient(blade,Rotation.from_euler("x",cross_phase,degrees=True))
    j=joint(stage,UsdPhysics.RevoluteJoint,"spindle_bearing","driver","spindle",(.055,0,0),(.055,0,0))
    m=UsdPhysics.DriveAPI.Apply(j.GetPrim(),"angular")
    m.CreateTypeAttr("force")
    m.CreateStiffnessAttr(0.)
    m.CreateDampingAttr(.00004)
    m.CreateMaxForceAttr(.002)
    m.CreateTargetVelocityAttr(0.)
    return m


def build_screw(stage,position):
    p=WORLD+"/screw"
    body(stage,p,position,.0016)
    cyl(stage,p+"/root",.016,.028,.00106,STEEL)
    cyl(stage,p+"/point",.03075,.002,.00073,STEEL)
    # Recess floor and stepped bugle profile. Four sectors leave a cross-shaped void.
    cyl(stage,p+"/recess_floor",.00168,.00055,HEAD_RADIUS,STEEL)
    for i,r in enumerate([.00355,.00278,.00201,.00143]):
        cyl(stage,p+f"/bugle_{i}",.0021+i*.00042,.00047,r,STEEL)
    from pxr import UsdGeom, UsdPhysics, PhysxSchema
    for quadrant in range(4):
        rad=.00049
        polygon=[(rad,rad)]+[(HEAD_RADIUS*math.cos(t),HEAD_RADIUS*math.sin(t)) for t in
            np.linspace(math.asin(rad/HEAD_RADIUS),math.acos(rad/HEAD_RADIUS),8)]
        c,s=math.cos(quadrant*math.pi/2),math.sin(quadrant*math.pi/2)
        verts=[(x,c*y-s*z,s*y+c*z) for x in (0.,.00149) for y,z in polygon]
        n=len(polygon)
        faces=[list(range(n))[::-1],list(range(n,2*n))]+[[i,(i+1)%n,(i+1)%n+n,i+n] for i in range(n)]
        m=UsdGeom.Mesh.Define(stage,p+f"/cheek_{quadrant}")
        m.CreatePointsAttr(verts);m.CreateFaceVertexCountsAttr([len(f) for f in faces])
        m.CreateFaceVertexIndicesAttr([k for face in faces for k in face]);m.CreateSubdivisionSchemeAttr("none")
        m.CreateDisplayColorAttr([STEEL])
        UsdPhysics.CollisionAPI.Apply(m.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(m.GetPrim()).CreateApproximationAttr("convexHull")
        api=PhysxSchema.PhysxCollisionAPI.Apply(m.GetPrim())
        api.CreateContactOffsetAttr(.000015);api.CreateRestOffsetAttr(0.)
    # Piecewise helical crest. Boxes overlap tangentially; the faces transfer real impulses.
    slices=12
    for i in range(9*slices):
        theta=(i+.5)*2*math.pi/slices
        radial=np.array([0.,math.cos(theta),math.sin(theta)])
        tangent=np.array([PITCH/(2*math.pi),-.0014*math.sin(theta),.0014*math.cos(theta)])
        tangent/=np.linalg.norm(tangent)
        rotation=Rotation.from_matrix(np.column_stack((np.cross(radial,tangent),radial,tangent)))
        xyz=(.002+i*PITCH/slices+PITCH/(2*slices),.0014*radial[1],.0014*radial[2])
        crest=small_box(stage,p+f"/crest_{i}",xyz,(.00052,.00067,.00087),STEEL)
        orient(crest,rotation)


def thread_lands(stage,path,substrate_start,thickness,head_x,xyz_origin,lead_in=0.):
    """Interrupted material flanks inside a prepared bore. No body or joint couples the screw to them."""
    origin=np.array(xyz_origin)
    for phase,theta in enumerate([0.,math.pi/2,math.pi,3*math.pi/2]):
        for turn in range(20):
            x=head_x+.002+PITCH*(turn+.5+theta/(2*math.pi))
            if not substrate_start+lead_in+.0004 < x < substrate_start+thickness-.0004:
                continue
            radial=np.array([0.,math.cos(theta),math.sin(theta)])
            center=origin+np.array([x-substrate_start,0.,0.])+radial*.00170
            tooth=small_box(stage,path+f"/land_{phase}_{turn}",center,(.00062,.00084,.00068),(.66,.53,.38))
            orient(tooth,Rotation.from_euler("x",theta))


def build_cell(stage,g):
    """Axis-constrained tool bench for mechanical controls; the screw is a free rigid body."""
    from pxr import UsdPhysics
    head_x=g.FRONT-START_PROUD
    grip=(head_x-TIP_X+.0013,g.FAST_Y,g.FAST_Z)
    if g.MODE=="rotary-backdrive":
        grip=(grip[0]-.0017,grip[1],grip[2])
    motor=build_driver(stage,grip,depth_sleeve=g.MODE=="rotary-stop",cross_phase=45. if g.MODE=="rotary-backdrive" else 0.)
    rail=joint(stage,UsdPhysics.PrismaticJoint,"test_slide",None,"driver",grip,(0,0,0))
    rail.CreateLowerLimitAttr(-.005);rail.CreateUpperLimitAttr(.035)
    feed=UsdPhysics.DriveAPI.Apply(rail.GetPrim(),"linear")
    high_load=g.MODE=="rotary-backdrive"
    feed.CreateTypeAttr("force");feed.CreateStiffnessAttr(1400. if high_load else 80.)
    feed.CreateDampingAttr(3. if high_load else 2.5);feed.CreateMaxForceAttr(6. if high_load else .11)
    feed.CreateTargetPositionAttr(0.)
    build_screw(stage,(head_x,g.FAST_Y,g.FAST_Z))
    guide=screw_guide(stage,g,head_x)
    clamp=build_backer_fixture(stage,g,head_x)
    # Fixed material collider group is a single rigid static geometry container.
    from pxr import UsdGeom
    thread=WORLD+"/gypsum_bore"
    UsdGeom.Xform.Define(stage,thread)
    thread_lands(stage,thread,g.FRONT,g.BOARD,head_x,(g.FRONT,g.FAST_Y,g.FAST_Z),lead_in=.0039)
    return motor,feed,clamp,guide


def screw_guide(stage,g,head_x):
    from pxr import UsdPhysics
    # The instrumented bench suppresses screw tilt and lateral drift only.
    # The screw remains unconstrained in axial translation and roll, the two
    # degrees of freedom whose coupling is measured from actual contact.
    from pxr import Gf
    guide=UsdPhysics.Joint.Define(stage,WORLD+"/bench_screw_cylindrical_guide")
    guide.CreateBody1Rel().SetTargets([WORLD+"/screw"])
    guide.CreateLocalPos0Attr(Gf.Vec3f(head_x,g.FAST_Y,g.FAST_Z))
    for axis in ("transY","transZ","rotY","rotZ"):
        limit=UsdPhysics.LimitAPI.Apply(guide.GetPrim(),axis)
        limit.CreateLowAttr(1.);limit.CreateHighAttr(-1.)
    return guide


def build_backer_fixture(stage,g,head_x):
    from pxr import UsdPhysics
    # The component rig holds the strip against the wall during driving.
    # It is a load fixture, separate from the documented loose-stock trials.
    z=g.FAST_Z-(.036 if g.MODE=="rotary-miss" else 0.)
    inner=g.FRONT+g.BOARD
    backer=WORLD+"/drive_backer"
    body(stage,backer,(inner+g.THICK/2,0,z),.049)
    clamp=UsdPhysics.FixedJoint.Define(stage,WORLD+"/bench_backer_clamp")
    clamp.CreateBody1Rel().SetTargets([backer])
    from pxr import Gf
    clamp.CreateLocalPos0Attr(Gf.Vec3f(inner+g.THICK/2,0,z))
    def stock(n,p,s):
        small_box(stage,backer+"/"+n,p,s,g.WOOD)
    g.pierced(stock,"wood",-.008,.008,-g.LENGTH/2,g.LENGTH/2,-g.STRIP_WIDTH/2,g.STRIP_WIDTH/2,g.FAST_Y,0,.00190)
    thread_lands(stage,backer,inner,g.THICK,head_x,(-g.THICK/2,g.FAST_Y,0))
    return clamp
