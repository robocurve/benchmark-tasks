"""Sharpen a loose pencil with a desk-mounted X-ACTO KS 1031 hand crank.

The pencil and crank are native contact bodies. The effective cutter face
retreats as material is removed, so the pencil must keep advancing under load.
The visible taper grows with the cut. No workpiece pose is assigned.
"""
import math
import numpy as np
from yam_arena import ik, station
from yam_arena.constants import READY, FINGER_OPEN as OPEN, FINGER_CLOSED as CLOSED, PAD_TIP_PAST_TCP as PAD
from yam_arena.demo import Waypoint, grip_rotation
from yam_arena.task_file import Box, TABLE_TOP
from yam_arena.urdf import axis_angle_matrix
from dynamic_task_moving_crank_sharpener.geometry import Solid
from dynamic_task_moving_crank_sharpener.scoring import _rotate

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SHARPENER_MODEL = "X-ACTO KS 1031 (1031LMR)"
SHARPENER_YAW = math.radians(-30.)
AXIAL = np.array((math.cos(SHARPENER_YAW),math.sin(SHARPENER_YAW),0.))
LATERAL = np.array((-AXIAL[1],AXIAL[0],0.))
UP = np.array((0.,0.,1.))
AXIS_Z = TOP + .0735
CRANK_RADIUS = .040
KNOB_OFFSET_X = .033
STANDARD_HOLE = 5 / 16 * .0254
PENCIL_DIAMETER = .007
PENCIL_LENGTH = .140
PENCIL_RADIUS = PENCIL_DIAMETER / 2
PENCIL_GRIP_BACK = .025
PENCIL_THICKNESS = math.sqrt(3) * PENCIL_RADIUS
CRANK_START = 11.
CRANK_DURATION = 44.
CRANK_END = CRANK_START + CRANK_DURATION
KNOB_RELEASE_S = .85
TOOL = Box("tool", pos=(.215,-.040,AXIS_Z), size=(.078,.069,.108),
           yaw=SHARPENER_YAW, mass=.50, color=(.66,.69,.72))


def tool_position(distance, height=0.):
    position = np.asarray(TOOL.pos) + distance*AXIAL + height*UP
    return tuple(map(float, position))


WHEEL = Box("wheel", pos=tool_position(.076), size=(.026,.018,.052),
            yaw=SHARPENER_YAW, mass=.080, color=(.27,.30,.32))
KNOB = Box("knob", pos=tool_position(.076+KNOB_OFFSET_X, -CRANK_RADIUS),
           size=(.026,.017,.017), yaw=SHARPENER_YAW, mass=.009, color=(.12,.12,.14))
CUTTER_OFFSET = .019
CUTTER = Box("cutter", pos=tool_position(CUTTER_OFFSET),
             size=(.003,.009,.009), yaw=SHARPENER_YAW, mass=.005, color=(.32,.34,.36))
PENCIL = Box("pencil", pos=(.140,.120,TOP+PENCIL_THICKNESS/2),
             size=(PENCIL_LENGTH,PENCIL_DIAMETER,PENCIL_THICKNESS), yaw=SHARPENER_YAW,
             mass=.006, color=(.95,.68,.10))
OBJECTS = [TOOL,WHEEL,KNOB,CUTTER,PENCIL]
CUTTER_HALF_THICKNESS = .0015
MIN_COMPRESSION = .0014
MAX_COMPRESSION = .0050
CUTTER_STIFFNESS = 780.
CUT_LENGTH = .015
CUT_TIERS = (.005,.010)
CUT_PER_TURN = .0024
CUT_RATE_LIMIT_PER_STEP = True
CRANK_DAMPING_EMPTY = .006
CRANK_DAMPING_CUT = .026


def ring(outer, inner, length, x, *, segments=24, color=None):
    """A thin-walled circular guide around the native cutting axis."""
    radius = (outer+inner)/2
    return tuple(Solid("box", (length, outer-inner, 2*outer*math.tan(math.pi/segments)),
                       (x,radius*math.cos(angle),radius*math.sin(angle)), roll=angle, color=color)
                 for angle in (i*math.tau/segments for i in range(segments)))


def selector():
    """The eight-hole face at the fixed standard-pencil detent."""
    dial_radius, pitch, center_z = .029, .017, -.017
    hole_sizes = (5/16,21/64,23/64,13/32,7/16,31/64,15/64,17/64)
    holes = [(pitch*math.sin(i*math.pi/4),center_z+pitch*math.cos(i*math.pi/4),diameter*.0254/2)
             for i,diameter in enumerate(hole_sizes)]
    strips = []
    width = .002
    for i in range(-14,15):
        y = i*width
        extent = math.sqrt(max(0.,dial_radius*dial_radius-(abs(y)+width/2)**2))
        intervals = [(center_z-extent,center_z+extent)]
        for hole_y,hole_z,radius in holes:
            distance = max(0.,abs(y-hole_y)-width/2)
            if distance >= radius:
                continue
            height = math.sqrt(radius*radius-distance*distance)
            lo,hi = hole_z-height,hole_z+height
            intervals = [(a,b) for start,end in intervals for a,b in
                         ((start,min(end,lo)),(max(start,hi),end)) if b-a > .00015]
        strips.extend(Solid("box",(.003,width,b-a),(.0015,y,(a+b)/2),color=(.80,.81,.82))
                      for a,b in intervals if b>a)
    return tuple(strips)


BODY_PARTS = {
    "tool": (
        *ring(.033,.014,.065,.0375,color=(.68,.71,.74)),
        *ring(.033,.012,.005,.0725,color=(.18,.20,.22)),
        *ring(.0072,STANDARD_HOLE/2,.008,.007,segments=16,color=(.53,.56,.59)),
        *selector(),
        Solid("box",(.0508,.0397,.008),(.050,0,-.0695),color=(.13,.15,.17)),
        Solid("box",(.030,.032,.038),(.057,0,-.049),color=(.15,.17,.19)),
        Solid("cylinder",(.003,.006),(.0,0,-.017),axis="X",color=(.46,.48,.51)),
    ),
    "wheel": (Solid("cylinder",(.008,.016),axis="X"),
              Solid("box",(.006,.011,CRANK_RADIUS+.007),(0,0,-CRANK_RADIUS/2)),
              Solid("cylinder",(.003,.026),(.016,0,-CRANK_RADIUS),axis="X")),
    "knob": (Solid("cylinder",(.0085,.026),axis="X"),),
    # The union of these three inscribed rectangles is a regular hexagon.
    # The flat underside keeps the loose pencil in place on the bare table.
    "pencil": tuple(Solid("box",(PENCIL_LENGTH,PENCIL_RADIUS,PENCIL_THICKNESS),
                          roll=i*math.pi/3) for i in range(3)),
}
SUPPORT_JOINTS = {"tool":{"kind":"fixed"}}
CONTACT_OFFSETS_M = {"pencil":.00035,"tool":.00045,"cutter":.00045}
OBJECT_JOINTS = {
    "wheel":{"kind":"revolute","parent":"tool","anchor":WHEEL.pos,"axis":"X",
             "drive":{"damping":CRANK_DAMPING_EMPTY,"max_force":.08}},
    "knob":{"kind":"revolute","parent":"wheel","anchor":KNOB.pos,"axis":"X",
            "drive":{"damping":.0001,"max_force":.005}},
    "cutter":{"kind":"prismatic","parent":"tool","anchor":CUTTER.pos,"axis":"X",
              "lower":0.,"upper":CUT_LENGTH+.006,
              "drive":{"stiffness":CUTTER_STIFFNESS,"damping":3.,"max_force":8.}},
}
TRACK_OBJECTS={"left":"pencil","right":"knob"}
TRACK_PHASES={"left":{"to pencil","descend pencil","grip pencil"},
              "right":{"to knob","descend knob","grip knob","hold knob","wait feed"}}
GRIP_OFFSET=-.008*grip_rotation(SHARPENER_YAW)[:,2]
KNOB_OPEN=-.024
SUCCESS_HOLD_S=1.0
MAX_PARTIAL_SCORE=.75
LIVE_CONTEXT={"tcp":{},"body_origins":{},"gripper":{}}
_previous=-math.pi/2
_loaded_by_end={1:0.,-1:0.}
_cut_depth_by_end={1:0.,-1:0.}
_active_end=1
_cutting=False
_cutter_drive=None
_crank_drive=None
_tip_display=None
_display_depths=None
_initial_crank_angle=None
_feed_bias=0.

def on_reset():
    global _previous,_loaded_by_end,_cut_depth_by_end,_active_end,_cutting
    global _cutter_drive,_crank_drive,_tip_display,_display_depths,_initial_crank_angle,_feed_bias
    _previous=-math.pi/2
    _loaded_by_end={1:0.,-1:0.}
    _cut_depth_by_end={1:0.,-1:0.}
    _active_end=1
    _cutting=False
    _cutter_drive=_crank_drive=_tip_display=_display_depths=None
    _initial_crank_angle=None
    _feed_bias=0.

def feed_state(objects,context):
    """Measure real travel and compression relative to the current cut depth."""
    root=np.asarray(context['body_origins']['pencil']);tool=np.asarray(context['body_origins']['tool'])
    axis=np.asarray(_rotate((1,0,0),tuple(objects['pencil'].quat_xyzw)))
    # Both blunt ends look the same. Accept whichever end points into the tool,
    # rather than requiring the policy to infer an invisible body-frame sign.
    parallel=float(axis @ AXIAL)
    end_sign=1 if parallel>=0 else -1
    tip=root-tool+end_sign*PENCIL_LENGTH/2*axis
    tip_along=float(tip @ AXIAL)
    rest_face=CUTTER_OFFSET-CUTTER_HALF_THICKNESS
    cutter_face=float((context['body_origins']['cutter']-tool) @ AXIAL)-CUTTER_HALF_THICKNESS
    feed_depth=tip_along-rest_face
    compression=float(cutter_face-rest_face-_cut_depth_by_end[end_sign])
    aligned=abs(parallel)>math.cos(math.radians(5)) and np.linalg.norm(tip-tip_along*AXIAL)<.0012
    touching=abs(tip_along-cutter_face)<.0012
    loaded=aligned and touching and MIN_COMPRESSION<=compression<=MAX_COMPRESSION
    return bool(loaded),compression,feed_depth,end_sign

def observe(objects,context,dt):
    global _previous,_active_end,_cutting
    delta=context['body_origins']['knob']-context['body_origins']['wheel']
    angle=math.atan2(delta @ UP,delta @ LATERAL)
    increment=(angle-_previous+math.pi)%(2*math.pi)-math.pi
    _previous=angle
    loaded,_,feed_depth,end_sign=feed_state(objects,context)
    # Change the effective face for the other end only while the workpiece is
    # clear of every cutting depth. The two ends retain independent tapers.
    if feed_depth<-.003:
        _active_end=end_sign
    if loaded and end_sign==_active_end:
        _loaded_by_end[end_sign]=max(0.,_loaded_by_end[end_sign]-increment)
    _cutting=bool(loaded and end_sign==_active_end and _cut_depth_by_end[end_sign]<CUT_LENGTH)
    if _cutting and increment<0.:
        angular_limit=_loaded_by_end[end_sign]*CUT_PER_TURN/math.tau
        travel_limit=feed_depth-MIN_COMPRESSION
        step_limit=_cut_depth_by_end[end_sign]-increment*CUT_PER_TURN/math.tau
        depth=min(CUT_LENGTH,angular_limit,travel_limit,step_limit)
        _cut_depth_by_end[end_sign]=max(_cut_depth_by_end[end_sign],depth)

def success(objects):
    p=objects['pencil'];root=LIVE_CONTEXT['body_origins']['pencil']
    axis=np.asarray(_rotate((1,0,0),tuple(p.quat_xyzw)))
    clear=all(np.linalg.norm(root-v)>.10 for v in LIVE_CONTEXT['tcp'].values())
    center=np.asarray(station.TABLE_CENTER[:2])
    half=np.asarray(station.TABLE_SIZE[:2])/2
    returned=(TOP-.002<=p.low[2]<=TOP+.003 and abs(axis[2])<math.sin(math.radians(10))
              and np.all(p.low[:2]>center-half) and np.all(p.high[:2]<center+half))
    return bool(max(_cut_depth_by_end.values())>=CUT_LENGTH and returned
                and p.resting() and clear)

def score(objects):
    if objects['pencil'].low[2]<TOP-.03:return 0.
    if success(objects):return 1.
    depth=max(_cut_depth_by_end.values())
    if depth>=CUT_LENGTH:return .75
    if depth>=CUT_TIERS[1]:return .50
    return .25 if depth>=CUT_TIERS[0] else 0.

def update_materials():
    """Retreat the loaded native face and render each new visible taper."""
    global _cutter_drive,_crank_drive,_tip_display,_display_depths
    import omni.usd
    from pxr import UsdGeom,UsdPhysics,UsdShade,Gf,Sdf
    stage=omni.usd.get_context().get_stage()
    if _cutter_drive is None:
        joints={p.GetName():p for p in stage.Traverse() if p.IsA(UsdPhysics.Joint)}
        _cutter_drive=UsdPhysics.DriveAPI(joints['cutter_joint'],'linear')
        _crank_drive=UsdPhysics.DriveAPI(joints['wheel_joint'],'angular')
    position=_cut_depth_by_end[_active_end]
    damping=CRANK_DAMPING_CUT if _cutting else CRANK_DAMPING_EMPTY
    if _cutter_drive.GetTargetPositionAttr().Get()!=position:
        _cutter_drive.GetTargetPositionAttr().Set(position)
    if _crank_drive.GetDampingAttr().Get()!=damping:
        _crank_drive.GetDampingAttr().Set(damping)

    depths=tuple(round(_cut_depth_by_end[end],5) for end in (-1,1))
    if depths==_display_depths or max(depths)<.0002:return
    if _tip_display is None:
        body,=(p for p in stage.Traverse() if p.GetName()=='pencil' and p.HasAPI(UsdPhysics.RigidBodyAPI))
        root=str(body.GetPath())+'/geometry'
        for i in range(len(BODY_PARTS['pencil'])):
            UsdGeom.Imageable(stage.GetPrimAtPath(root+f'/solid_{i}')).MakeInvisible()
        _tip_display={}
    else:
        root=str(_tip_display['shaft'][0].GetPath().GetParentPath())

    def shade(name, shape, color):
        material=UsdShade.Material.Define(stage,root+'/'+name+'_material')
        shader=UsdShade.Shader.Define(stage,root+'/'+name+'_material/shader');shader.CreateIdAttr('UsdPreviewSurface')
        shader.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
        material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),'surface')
        UsdShade.MaterialBindingAPI.Apply(shape.GetPrim()).Bind(material)

    def shape_at(name,kind,height,radius,center,color,reverse=False):
        if name in _tip_display:
            shape,translation=_tip_display[name]
            shape.GetHeightAttr().Set(height)
            shape.GetRadiusAttr().Set(radius)
            translation.Set(Gf.Vec3d(center,0,0))
            return
        shape=kind.Define(stage,root+'/'+name)
        shape.CreateAxisAttr('X');shape.CreateHeightAttr(height);shape.CreateRadiusAttr(radius)
        translation=shape.AddTranslateOp();translation.Set(Gf.Vec3d(center,0,0))
        if reverse:shape.AddRotateZOp().Set(180.)
        shade(name,shape,color)
        _tip_display[name]=(shape,translation)

    length=PENCIL_LENGTH-sum(depths)
    center=(depths[0]-depths[1])/2
    if 'shaft' not in _tip_display:
        shaft=UsdGeom.Mesh.Define(stage,root+'/shaft')
        points=[Gf.Vec3f(x,PENCIL_RADIUS*math.cos(i*math.pi/3),PENCIL_RADIUS*math.sin(i*math.pi/3))
                for x in (-.5,.5) for i in range(6)]
        sides=[(i,(i+1)%6,(i+1)%6+6,i+6) for i in range(6)]
        shaft.CreatePointsAttr(points)
        shaft.CreateFaceVertexCountsAttr([6,6]+[4]*6)
        shaft.CreateFaceVertexIndicesAttr([5,4,3,2,1,0,6,7,8,9,10,11]+
                                          [index for face in sides for index in face])
        shaft.CreateSubdivisionSchemeAttr('none')
        shaft.AddTranslateOp()
        shaft.AddScaleOp()
        shade('shaft',shaft,(.95,.68,.10))
        _tip_display['shaft']=(shaft,shaft.GetOrderedXformOps())
    _,(translation,scale)=_tip_display['shaft']
    translation.Set(Gf.Vec3d(center,0,0))
    scale.Set(Gf.Vec3f(length,1,1))

    radius=PENCIL_RADIUS
    half=PENCIL_LENGTH/2
    for end,depth in zip((-1,1),depths,strict=True):
        if depth<.0002:continue
        suffix='left' if end<0 else 'right'
        graphite=max(0.,depth-.76*CUT_LENGTH)*.65
        shape_at('wood_'+suffix,UsdGeom.Cone,depth,radius,end*(half-depth/2),(.72,.49,.26),end<0)
        if graphite>0.:
            shape_at('graphite_'+suffix,UsdGeom.Cone,graphite,1.02*radius*graphite/depth,
                     end*(half-graphite/2),(.07,.07,.08),end<0)
    _display_depths=depths

def demo_target(arm,label,t,pos,rotation,finger):
    global _initial_crank_angle,_feed_bias
    if arm=='right' and label=='crank':
        if _initial_crank_angle is None:
            offset=LIVE_CONTEXT['body_origins']['knob']-LIVE_CONTEXT['body_origins']['wheel']
            _initial_crank_angle=math.atan2(offset @ UP,offset @ LATERAL)
        angle=_initial_crank_angle-(t-CRANK_START)*math.tau/4.
        radial=math.cos(angle)*LATERAL+math.sin(angle)*UP
        pos=LIVE_CONTEXT['body_origins']['wheel']+KNOB_OFFSET_X*AXIAL+CRANK_RADIUS*radial+GRIP_OFFSET
    elif arm=='right' and label=='release knob':pos=LIVE_CONTEXT['body_origins']['knob']+GRIP_OFFSET
    elif arm=='left' and label in {'align pencil','feed pencil','hold feed','withdraw pencil'}:
        pos=pos+LIVE_CONTEXT['body_origins']['tool']-np.asarray(TOOL.pos)
        offset=LIVE_CONTEXT['body_origins']['pencil']-LIVE_CONTEXT['tcp']['left']
        pos=pos+np.clip(PENCIL_GRIP_BACK*AXIAL-PAD*UP-offset,-.015,.015)
        # Align the actual pencil tip, including its small tilt inside the pads.
        # Only the robot command changes; no object pose or joint is edited.
        axis=np.asarray(_rotate((1,0,0),tuple(LIVE_OBJECTS['pencil'].quat_xyzw)))
        axis*=1 if axis @ AXIAL>=0 else -1
        # The nominal pose bounds roll and keeps the pencil aligned on withdrawal.
        wrist=LIVE_CONTEXT['tcp_rotation']['left']
        nominal_axis=rotation @ (wrist.T @ axis)
        correction=np.cross(nominal_axis,AXIAL)
        sine=np.linalg.norm(correction)
        if sine>1e-6:
            angle=min(math.atan2(sine,nominal_axis @ AXIAL),math.radians(5))
            rotation=axis_angle_matrix(correction/sine,angle) @ rotation
        across=axis-(axis @ AXIAL)*AXIAL
        pos-=np.clip(PENCIL_LENGTH/2*across,-.005,.005)
        if label=='hold feed':
            _,compression,_,end_sign=feed_state(LIVE_OBJECTS,LIVE_CONTEXT)
            # Compensate arm compliance through a bounded Cartesian command.
            # The native passive spring still supplies the entire axial load.
            error=.0028-compression
            _feed_bias=float(np.clip(_feed_bias+.12*error,-.003,.011))
            depth=_cut_depth_by_end[end_sign]+_feed_bias+np.clip(.7*error,-.002,.004)
            pos+=depth*AXIAL
        if label=='withdraw pencil':
            depth=(max(_cut_depth_by_end.values())+_feed_bias)*(1.-ik.smoothstep((t-CRANK_END)/2.))
            pos+=depth*AXIAL
    return pos,rotation,finger

def demo():
    # Put the finger tips below the pencil center and clear of the tabletop.
    pick=np.asarray(PENCIL.pos)-PENCIL_GRIP_BACK*AXIAL
    pick[2]=TOP+PAD+.0003
    hover=(pick[0],pick[1],TOP+.17)
    lever=PENCIL_LENGTH/2+PENCIL_GRIP_BACK
    align=np.asarray(TOOL.pos)-(.013+lever)*AXIAL+PAD*UP
    feed=np.asarray(CUTTER.pos)+(.0033-CUTTER_HALF_THICKNESS-lever)*AXIAL+PAD*UP
    knob=np.asarray(KNOB.pos)+GRIP_OFFSET
    hover_knob=(knob[0],knob[1],TOP+.156)
    def pose(duration, position, finger, label):
        return Waypoint(duration, tuple(position), finger, label, yaw=SHARPENER_YAW)

    return {
      'left':[pose(1.5,hover,OPEN,'to pencil'),pose(1.5,pick,OPEN,'descend pencil'),
              pose(1.45,pick,CLOSED,'grip pencil'),pose(1.25,hover,CLOSED,'lift pencil'),
              pose(2.1,align,CLOSED,'align pencil'),pose(3.2,feed,CLOSED,'feed pencil'),
              pose(CRANK_DURATION,feed,CLOSED,'hold feed'),pose(2.,align,CLOSED,'withdraw pencil'),
              pose(2.,hover,CLOSED,'return over table'),pose(1.5,pick,CLOSED,'lower pencil'),
              pose(.8,pick,OPEN,'release pencil'),pose(1.5,hover,OPEN,'clear pencil')],
      'right':[pose(1.5,hover_knob,KNOB_OPEN,'to knob'),
               pose(2.,knob,KNOB_OPEN,'descend knob'),pose(1.4,knob,CLOSED,'grip knob'),
               pose(.8,knob,CLOSED,'hold knob'),pose(5.3,knob,CLOSED,'wait feed'),
               pose(CRANK_DURATION,knob,CLOSED,'crank'),
               pose(KNOB_RELEASE_S,knob,KNOB_OPEN,'release knob'),
               pose(1.5,(.30,-.25,TOP+.16),OPEN,'withdraw'),pose(12.,(.30,-.25,TOP+.16),OPEN,'done')],
    }
