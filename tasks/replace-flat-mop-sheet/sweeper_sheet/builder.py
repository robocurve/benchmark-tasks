"""Native articulated end clamps and a loose, two-strip dry mop pad."""

import math

from . import config as c
from .usd_scene import Writer

PLASTIC = (.53,.66,.70)
JAW = (.10,.26,.30)
CLOTH = (.89,.87,.69)
STRIP = (.74,.73,.54)
RUBBER = (.19,.22,.24)


def hinge(w, stage, name, parent, child, pos, limits, spring=0., damping=.001, target=0., axis="X"):
    joint = w.Phy.RevoluteJoint.Define(stage, "/Asset/" + name)
    joint.CreateBody0Rel().SetTargets([parent])
    joint.CreateBody1Rel().SetTargets([child])
    joint.CreateLocalPos0Attr(w.Gf.Vec3f(*pos))
    joint.CreateLocalPos1Attr(w.Gf.Vec3f(0))
    joint.CreateAxisAttr(axis)
    joint.CreateLowerLimitAttr(limits[0])
    joint.CreateUpperLimitAttr(limits[1])
    joint.CreateCollisionEnabledAttr(False)
    drive = w.Phy.DriveAPI.Apply(joint.GetPrim(), "angular")
    drive.CreateTypeAttr("force")
    drive.CreateTargetPositionAttr(target)
    drive.CreateStiffnessAttr(spring * math.pi / 180)
    drive.CreateDampingAttr(damping * math.pi / 180)
    return joint


def room(w):
    stage = w.stage("workcell")
    w.box(stage, "/Asset/Bench", (1.28, 1.25, .06), (.35, 0., c.TABLE - .03), (.58,.48,.34))
    w.box(stage, "/Asset/Floor", (6.,6.,.025), (.0,0.,-.0375), (.29,.32,.36))
    w.box(stage, "/Asset/Tile", (.158, .414, c.PLATFORM-c.TABLE),
          (c.X, 0., (c.PLATFORM+c.TABLE)/2), (.44,.46,.49))
    w.box(stage, "/Asset/FloorSample", (.256,.478,c.PLATFORM-c.TABLE),
          (.142,0.,(c.TABLE+c.PLATFORM)/2), (.66,.63,.57))
    for index, y in enumerate((-.119, 0., .119)):
        w.box(stage, f"/Asset/Grout{index}", (.256,.0026,.0004),
              (.142,y,c.PLATFORM+.0003), (.43,.41,.37), collide=False)
    w.box(stage, "/Asset/PoleStand", (.078,.165,.233),
          (.88,0.,c.TABLE+.1165), (.29,.33,.34))
    w.cylinder(stage, "/Asset/PoleRoller", .017,.162,
               (.88,0.,c.TABLE+.233), (.27,.30,.32), axis="Y")
    for side in (-1,1):
        w.box(stage,f"/Asset/SideLocator{side}",(.014,.156,.015),
              (c.X+side*.081,0.,c.PLATFORM+.0075),(.33,.35,.39))
        for x in (-.044,.044):
            w.box(stage,f"/Asset/EndLocator{side}_{x}",(.022,.015,.014),
                  (c.X+x,side*.223,c.PLATFORM+.007),(.33,.35,.39))
        w.box(stage,f"/Asset/FixtureFoot{side}",(.122,.039,c.PLATFORM-c.TABLE),
              (c.X,side*.223,(c.PLATFORM+c.TABLE)/2),(.44,.46,.49))
        # Loose fixation strips lean against this small guide until folded inward.
        w.box(stage,f"/Asset/StripGuide{side}",(.043,.013,.041),
              (c.X+.0215,side*.222,c.PLATFORM+.0205),(.44,.46,.49))
    return w.save(stage)


def head(w):
    stage = w.stage("head")
    base = w.body(stage, "/Asset/Head", .357, articulation=True, ccd=True)
    w.Px.PhysxArticulationAPI.Get(stage, base).CreateEnabledSelfCollisionsAttr(False)
    w.box(stage,base+"/LowerDeck",(c.HEAD_DEPTH,2*c.HALF_WIDTH,.004), (0,0,.002), RUBBER)
    w.box(stage,base+"/UpperDeck",(c.HEAD_DEPTH,2*c.HALF_WIDTH,.010), (0,0,.009), PLASTIC)
    # The folding frame remains open and mechanically latched throughout pad changes.
    for side in (-1,1):
        w.box(stage,base+f"/Bridge{side}",(.068,.035,.012), (0.,side*.069,.020), PLASTIC)
        w.box(stage,base+f"/BiteAnvil{side}",(.057,.029,.003), (0.,side*.179,.0155), JAW)
    shaft = w.body(stage, "/Asset/Socket", .013, position=(0.,0.,.038))
    w.cylinder(stage,shaft+"/SocketHub",.019,.022,(0,0,0),JAW,axis="X")
    w.cylinder(stage,shaft+"/HandleStub",.0115,.058,(0,0,.026),PLASTIC)
    w.cylinder(stage,shaft+"/HandleGrip",.014,.023,(0,0,.036),RUBBER)
    hinge(w,stage,"HandleSwivel",base,shaft,(0.,0.,.038),(-62.,62.),.095,.025)
    w.box(stage,shaft+"/OffsetSwivelFork",(.052,.026,.015),(.026,0.,.024),JAW)
    pole = w.body(stage, "/Asset/Pole", .162, position=(.050,0.,.064), ccd=True)
    w.cylinder(stage,pole+"/HingePin",.011,.021,(0,0,0),JAW,axis="Y")
    w.cylinder(stage,pole+"/AluminiumPole",.010,c.POLE_LENGTH,(0,0,c.POLE_LENGTH/2),(.74,.77,.77))
    w.cylinder(stage,pole+"/TopGrip",.014,.165,(0,0,c.POLE_LENGTH-.078),JAW)
    w.cylinder(stage,pole+"/Collar",.013,.045,(0,0,.210),JAW)
    hinge(w,stage,"PolePitch",shaft,pole,(.050,0.,.026),(0.,88.),0.,.22,axis="Y")
    for tag,side in c.END_POINTS.items():
        p = (0., side*c.CLAMP_Y, c.CLAMP_Z)
        clip = w.body(stage,"/Asset/"+tag+"Clip",.013,position=p)
        w.box(stage,clip+"/Lever",(.057,.045,.006),(0.,side*.022,.0018),JAW)
        w.box(stage,clip+"/Bite",(.054,.016,.0036), (0.,side*.036,-.0016),RUBBER)
        w.cylinder(stage,clip+"/Pivot",.0042,.057,(0.,0.,0.),JAW,axis="X",collide=False)
        hinge(w,stage,tag+"ClipPivot",base,clip,p,
              (0.,64.) if side>0 else (-64.,0.),0.,.025)
    return w.save(stage)


def cloth(w):
    stage = w.stage("cloth")
    base = w.body(stage,"/Asset/Pad",.048,articulation=True,ccd=True)
    w.Px.PhysxArticulationAPI.Get(stage,base).CreateEnabledSelfCollisionsAttr(False)
    surface=w.Phy.MaterialAPI.Get(stage,"/Asset/Physics")
    surface.CreateStaticFrictionAttr(.86)
    surface.CreateDynamicFrictionAttr(.65)
    w.box(stage,base+"/Microfibre",(.127,2*c.WRAP_Y,.0045),(0.,0.,0.),CLOTH)
    for tag,side in c.END_POINTS.items():
        wrap_pos=(0.,side*c.WRAP_Y,0.)
        wrap=w.body(stage,"/Asset/"+tag+"Wrap",.0028,position=wrap_pos,ccd=True)
        wing_pos=(0.,side*(c.WRAP_Y+c.WRAP_LENGTH),0.)
        strip=w.body(stage,"/Asset/"+tag+"Strip",.0036,position=wing_pos,ccd=True)
        w.box(stage,wrap+"/ReinforcedHem",(.042,c.WRAP_LENGTH-.0006,.0028),
              (0.,side*c.WRAP_LENGTH/2,0.),STRIP)
        w.box(stage,strip+"/FixationStrip",(.043,c.STRIP_LENGTH-.0006,.0029),
              (0.,side*c.STRIP_LENGTH/2,0.),STRIP)
        hinge(w,stage,tag+"WrapBend",base,wrap,wrap_pos,(-104.,104.),0.,.0023)
        hinge(w,stage,tag+"StripBend",wrap,strip,(0.,side*c.WRAP_LENGTH,0.),
              (-125.,125.),0.,.00165)
    return w.save(stage)


def create_assets(directory):
    w=Writer(directory)
    assets=[
        dict(name="workcell",type="base",file=room(w)),
        dict(name="cloth",type="articulation",file=cloth(w),position=(c.X,0.,c.FABRIC_Z)),
        dict(name="head",type="articulation",file=head(w),position=(c.X,0.,c.HEAD_Z)),
    ]
    w.inventory()
    return {"assets": assets, "settle_actions": 16}
