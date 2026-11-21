"""Feed a pencil into a moving mounted sharpener while turning its crank.

The pencil and crank are native contact bodies. The effective cutter face
retreats as material is removed, so the pencil must keep advancing under load.
The visible taper grows with the cut. No workpiece pose is assigned.
"""
import math
import numpy as np
from yam_arena import ik, station
from yam_arena.constants import READY, FINGER_OPEN as OPEN, FINGER_CLOSED as CLOSED, PAD_TIP_PAST_TCP as PAD
from yam_arena.demo import Waypoint
from yam_arena.task_file import Box, TABLE_TOP
from dynamic_task_moving_crank_sharpener.geometry import Solid
from dynamic_task_moving_crank_sharpener.motion import OscillatingPlatform
from dynamic_task_moving_crank_sharpener.scoring import _rotate

SCENE = "station"
TOP = TABLE_TOP[SCENE]
TOOL = Box("tool", pos=(.245,-.04,TOP+.12), size=(.06,.07,.07), mass=.50, color=(.15,.30,.65))
STAGE = Box("stage", pos=(.245,-.04,TOP+.015), size=(.18,.14,.085), mass=5., color=(.28,.28,.30))
WHEEL = Box("wheel", pos=(.285,-.04,TOP+.12), size=(.008,.09,.09), mass=.080, color=(.75,.20,.15))
KNOB = Box("knob", pos=(.331,-.04,TOP+.085), size=(.030,.016,.016), mass=.008, color=(.12,.12,.14))
CUTTER = Box("cutter", pos=(.232,-.04,TOP+.12), size=(.003,.008,.008), mass=.005, color=(.45,.45,.48))
PENCIL = Box("pencil", pos=(.120,.10,TOP+.0335), size=(.140,.007,.007), mass=.006, color=(.95,.68,.10))
CRADLES = [Box("cradle_left",pos=(.055,.10,TOP+.015),size=(.016,.04,.03),color=(.28,.28,.30)),
           Box("cradle_right",pos=(.160,.10,TOP+.015),size=(.016,.04,.03),color=(.28,.28,.30))]
OBJECTS = [STAGE,TOOL,WHEEL,KNOB,CUTTER,PENCIL,*CRADLES]
CUTTER_OFFSET = CUTTER.pos[0] - TOOL.pos[0]
CUTTER_HALF_THICKNESS = .0015
MIN_COMPRESSION = .0014
MAX_COMPRESSION = .0050
CUTTER_STIFFNESS = 780.
CUT_LENGTH = .018
CUT_PER_TURN = .0024
CRANK_DAMPING_EMPTY = .006
CRANK_DAMPING_CUT = .026
HOLE = .009
BAR = (.070-HOLE)/2
SHIFT = (.070+HOLE)/4
BODY_PARTS = {
    "stage": (Solid("box",(.18,.14,.030)),Solid("box",(.025,.050,.055),(0,0,.0425))),
    "tool": (Solid("box",(.060,BAR,.070),(0,-SHIFT,0)),
             Solid("box",(.060,BAR,.070),(0,SHIFT,0)),
             Solid("box",(.060,HOLE,BAR),(0,0,-SHIFT)),
             Solid("box",(.060,HOLE,BAR),(0,0,SHIFT))),
    "wheel": (Solid("cylinder",(.008,.018),axis="X"),
              Solid("box",(.008,.012,.043),(0,0,-.0175)),
              Solid("cylinder",(.003,.040),(.020,0,-.035),axis="X")),
    "knob": (Solid("cylinder",(.008,.030),axis="X"),),
    "pencil": (Solid("cylinder",(.0035,.140),axis="X"),),
}
MOTIONS = {"stage":OscillatingPlatform(STAGE.pos,(0,.005,0),8.0)}
SUPPORT_JOINTS = {"stage":{"axis":"Y","lower":-.020,"upper":.020}}
OBJECT_JOINTS = {
    "tool":{"kind":"fixed","parent":"stage","anchor":TOOL.pos},
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
SIDE=np.array([[0.,1.,0.],[0.,0.,1.],[1.,0.,0.]])
GRIP_OFFSET=np.array((0,-.008,0))
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

def on_reset():
    global _previous,_loaded_by_end,_cut_depth_by_end,_active_end,_cutting
    global _cutter_drive,_crank_drive,_tip_display,_display_depths
    _previous=-math.pi/2
    _loaded_by_end={1:0.,-1:0.}
    _cut_depth_by_end={1:0.,-1:0.}
    _active_end=1
    _cutting=False
    _cutter_drive=_crank_drive=_tip_display=_display_depths=None

def feed_state(objects,context):
    """Measure real travel and compression relative to the current cut depth."""
    root=np.asarray(context['body_origins']['pencil']);tool=np.asarray(context['body_origins']['tool'])
    axis=np.asarray(_rotate((1,0,0),tuple(objects['pencil'].quat_xyzw)))
    # Both blunt ends look the same. Accept whichever end points into the tool,
    # rather than requiring the policy to infer an invisible body-frame sign.
    end_sign=1 if axis[0]>=0 else -1
    tip=root+end_sign*.070*axis
    rest_face=tool[0]+CUTTER_OFFSET-CUTTER_HALF_THICKNESS
    cutter_face=context['body_origins']['cutter'][0]-CUTTER_HALF_THICKNESS
    feed_depth=float(tip[0]-rest_face)
    compression=float(cutter_face-rest_face-_cut_depth_by_end[end_sign])
    aligned=abs(axis[0])>math.cos(math.radians(5)) and np.linalg.norm(tip[1:]-tool[1:])<.0012
    touching=abs(tip[0]-cutter_face)<.0012
    loaded=aligned and touching and MIN_COMPRESSION<=compression<=MAX_COMPRESSION
    return bool(loaded),compression,feed_depth,end_sign

def observe(objects,context,dt):
    global _previous,_active_end,_cutting
    delta=context['body_origins']['knob']-context['body_origins']['wheel']
    angle=math.atan2(delta[2],delta[1]);increment=(angle-_previous+math.pi)%(2*math.pi)-math.pi
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
        depth=min(CUT_LENGTH,angular_limit,travel_limit)
        _cut_depth_by_end[end_sign]=max(_cut_depth_by_end[end_sign],depth)

def success(objects):
    p=objects['pencil'];root=LIVE_CONTEXT['body_origins']['pencil']
    axis=np.asarray(_rotate((1,0,0),tuple(p.quat_xyzw)))
    clear=all(np.linalg.norm(root-v)>.10 for v in LIVE_CONTEXT['tcp'].values())
    returned=abs(root[0]-.120)<.015 and abs(root[1]-.10)<.012 and TOP+.029<=p.low[2]<=TOP+.034
    return bool(max(_cut_depth_by_end.values())>=CUT_LENGTH and returned
                and abs(axis[0])>math.cos(math.radians(10)) and p.resting() and clear)

def score(objects):
    if objects['pencil'].low[2]<TOP-.03:return 0.
    if success(objects):return 1.
    depth=max(_cut_depth_by_end.values())
    if depth>=CUT_LENGTH:return .75
    if depth>=.012:return .50
    return .25 if depth>=.006 else 0.

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
        UsdGeom.Imageable(stage.GetPrimAtPath(root+'/solid_0')).MakeInvisible()
        _tip_display={}
    else:
        root=str(_tip_display['shaft'][0].GetPath().GetParentPath())

    def shape_at(name,kind,height,radius,center,color,reverse=False):
        if name in _tip_display:
            shape,translation=_tip_display[name]
            shape.GetHeightAttr().Set(height)
            translation.Set(Gf.Vec3d(center,0,0))
            return
        shape=kind.Define(stage,root+'/'+name)
        shape.CreateAxisAttr('X');shape.CreateHeightAttr(height);shape.CreateRadiusAttr(radius)
        translation=shape.AddTranslateOp();translation.Set(Gf.Vec3d(center,0,0))
        if reverse:shape.AddRotateZOp().Set(180.)
        material=UsdShade.Material.Define(stage,root+'/'+name+'_material')
        shader=UsdShade.Shader.Define(stage,root+'/'+name+'_material/shader');shader.CreateIdAttr('UsdPreviewSurface')
        shader.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
        material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),'surface')
        UsdShade.MaterialBindingAPI.Apply(shape.GetPrim()).Bind(material)
        _tip_display[name]=(shape,translation)

    shape_at('shaft',UsdGeom.Cylinder,.140-sum(depths),.0035,(depths[0]-depths[1])/2,(.95,.68,.10))
    for end,depth in zip((-1,1),depths,strict=True):
        if depth<.0002:continue
        suffix='left' if end<0 else 'right'
        graphite=max(0.,(depth-.012)/2)
        wood=depth-graphite
        shape_at('wood_'+suffix,UsdGeom.Cone,wood,.0035,end*(.070-depth+wood/2),(.72,.49,.26),end<0)
        if graphite>0.:
            shape_at('graphite_'+suffix,UsdGeom.Cone,graphite,.000525,end*(.070-graphite/2),(.07,.07,.08),end<0)
    _display_depths=depths

def demo_rotation(arm,label,t,default):
    if arm!='right':return default
    start=ik.tcp_pose(np.array(READY),station.BASES['right'])[1]
    return ik.slerp_matrix(start,SIDE,ik.smoothstep(min(t/2,1.)))

def demo_target(arm,label,t,pos,rotation,finger):
    if arm=='right' and label=='crank':
        angle=-math.pi/2-(t-10.)*2*math.pi/4.
        pos=LIVE_CONTEXT['body_origins']['wheel']+np.array((.046,.035*math.cos(angle),.035*math.sin(angle)))+GRIP_OFFSET
    elif arm=='right' and label=='release knob':pos=LIVE_CONTEXT['body_origins']['knob']+GRIP_OFFSET
    elif arm=='left' and label in {'align pencil','feed pencil','hold feed','withdraw pencil'}:
        pos=pos+LIVE_CONTEXT['body_origins']['tool']-np.asarray(TOOL.pos)
        offset=LIVE_CONTEXT['body_origins']['pencil']-LIVE_CONTEXT['tcp']['left']
        pos=pos+np.clip(np.array((0,0,-PAD))-offset,-.015,.015)
        # Align the actual pencil tip, including its small tilt inside the pads.
        # Only the robot command changes; no object pose or joint is edited.
        axis=np.asarray(_rotate((1,0,0),tuple(LIVE_OBJECTS['pencil'].quat_xyzw)))
        axis*=1 if axis[0]>=0 else -1
        pos[1:]-=np.clip(.070*axis[1:],-.005,.005)
        pos[1:]+=np.clip(.15*LIVE_OBJECTS['tool'].lin_vel[1:],-.002,.002)
        if label=='hold feed':
            _,compression,_,end_sign=feed_state(LIVE_OBJECTS,LIVE_CONTEXT)
            # Compensate arm compliance through a bounded Cartesian command.
            # The native passive spring still supplies the entire axial load.
            pos[0]+=_cut_depth_by_end[end_sign]+np.clip(2*(.0033-compression),-.002,.006)
        if label=='withdraw pencil':
            pos[0]+=max(_cut_depth_by_end.values())*(1.-ik.smoothstep(np.clip((t-52.)/2.,0.,1.)))
    return pos,rotation,finger

def demo():
    # Thin pencils need pad overlap; the cube convention puts only pad tips
    # level with the pencil axis and misses its 7 mm cylindrical cross-section.
    pick=np.asarray(PENCIL.pos)+np.array((0,0,PAD-.008));hover=(.120,.10,TOP+.20)
    align=(.130,-.04,TOP+.12+PAD);feed=(.164,-.04,TOP+.12+PAD)
    knob=np.asarray(KNOB.pos)+GRIP_OFFSET;end=(KNOB.pos[0],WHEEL.pos[1]-.035+GRIP_OFFSET[1],WHEEL.pos[2])
    return {
      'left':[Waypoint(1.5,hover,OPEN,'to pencil'),Waypoint(1.5,tuple(pick),OPEN,'descend pencil'),
              Waypoint(.8,tuple(pick),CLOSED,'grip pencil'),Waypoint(1.2,hover,CLOSED,'lift pencil'),
              Waypoint(2.,align,CLOSED,'align pencil'),Waypoint(3.,feed,CLOSED,'feed pencil'),
              Waypoint(41.,feed,CLOSED,'hold feed'),Waypoint(2.,align,CLOSED,'withdraw pencil'),
              Waypoint(2.,hover,CLOSED,'return over cradle'),Waypoint(1.5,tuple(pick),CLOSED,'lower pencil'),
              Waypoint(.8,tuple(pick),OPEN,'release pencil'),Waypoint(1.5,hover,OPEN,'clear pencil')],
      'right':[Waypoint(2.,(knob[0],knob[1]-.045,knob[2]+.020),OPEN,'to knob'),
               Waypoint(1.5,tuple(knob),OPEN,'descend knob'),Waypoint(.8,tuple(knob),CLOSED,'grip knob'),
               Waypoint(.7,tuple(knob),CLOSED,'hold knob'),Waypoint(5.,tuple(knob),CLOSED,'wait feed'),
               Waypoint(41.0,end,CLOSED,'crank'),Waypoint(.8,end,OPEN,'release knob'),
               Waypoint(1.5,(.30,-.25,TOP+.16),OPEN,'withdraw'),Waypoint(12.,(.30,-.25,TOP+.16),OPEN,'done')],
    }
