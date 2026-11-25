"""Film one continuous crossing by panning a damped compact video head."""

from dataclasses import dataclass
import math

import numpy as np
from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint, grip_rotation
from yam_arena.ik import matrix_to_quat_xyzw
from yam_arena.rig import APERTURE_MM, Camera, Lens
from yam_arena.task_file import Box, TABLE_TOP
from yam_arena.urdf import quat_xyzw_to_matrix

from dynamic_task_fluid_head_subject_tracking.geometry import Solid
from dynamic_task_fluid_head_subject_tracking.motion import PoseVelocity

SCENE = 'station'
RENDER_ANTIALIASING = 'DLAA'
TOP = TABLE_TOP[SCENE]
PIVOT = np.array((.435, -.190, TOP+.132))
CAMERA_LOCAL = np.array((.063, 0., .074))
HANDLE_LOCAL = np.array((-.130, -.048, -.002))
SUBJECT_X = .923
SUBJECT_Z = float(PIVOT[2]+CAMERA_LOCAL[2])
START_Y, END_Y = -.395, .135
START_S, DURATION_S = 3.25, 8.
END_S = START_S+DURATION_S
TARGET_H, TARGET_W = .104, .058
HALF_WIDTH = math.tan(math.radians(17.))
HALF_HEIGHT = HALF_WIDTH * 9/16
BODY = (.0435,.1055,.060)
BLACK = (.071,.077,.082)
METAL = (.28,.31,.36)
RAIL = (.095,.116,.136)
WHITE = (.87,.89,.86)
ORANGE = (.78,.205,.044)

TASK_TABLE = Box('camera_bench', size=(1.18,1.23,.038), pos=(.437,0.,TOP-.019))


@dataclass(frozen=True)
class SubjectCrossing:
    start: tuple[float, float, float]
    displacement_y: float
    start_s: float
    duration_s: float

    def at(self, seconds):
        part = min(1., max(0., (seconds-self.start_s)/self.duration_s))
        y = self.start[1]+self.displacement_y*part*part*(3.-2.*part)
        speed_y = 6.*self.displacement_y*part*(1.-part)/self.duration_s
        return PoseVelocity((self.start[0], y, self.start[2]),0.,(0.,speed_y,0.),(0.,0.,0.))


def yaw_matrix(angle):
    c,s=math.cos(angle),math.sin(angle)
    return np.array(((c,-s,0.),(s,c,0.),(0.,0.,1.)))


def aim_at(subject_y):
    return math.atan2(subject_y-PIVOT[1], SUBJECT_X-PIVOT[0])


BEGIN_YAW = aim_at(START_Y)
MOUNT = Box('tripod_base',pos=tuple(map(float,PIVOT-np.array((0.,0.,.046)))),size=(.164,.186,.172),
            mass=.85,color=BLACK)
HEAD = Box('pan_head', pos=tuple(map(float,PIVOT)),size=(.218,.107,.142),yaw=BEGIN_YAW,mass=.54,color=BLACK)
RAIL_BASE = Box('linear_rail',pos=(SUBJECT_X,(START_Y+END_Y)/2.,TOP+.019),size=(.098,.692,.038),
                color=RAIL)
SUBJECT = Box('moving_subject',pos=(SUBJECT_X,START_Y,SUBJECT_Z),size=(.076,.085,.252),
              mass=.78,color=ORANGE)
OBJECTS = [MOUNT,HEAD,RAIL_BASE,SUBJECT]
BODY_PARTS = {
    MOUNT.name: (
        Solid('cylinder',(.038,.013),(0.,0.,.042),color=METAL),
        Solid('cylinder',(.026,.086),(0.,0.,.002),color=BLACK),
        *(Solid('box',(.087,.020,.016),(.047*math.cos(a),.047*math.sin(a),-.078),yaw=a)
          for a in (0.,math.tau/3,2*math.tau/3)),
        *(Solid('box',(.032,.026,.011),(.079*math.cos(a),.079*math.sin(a),-.0805),yaw=a)
          for a in (0.,math.tau/3,2*math.tau/3)),
    ),
    HEAD.name: (
        Solid('cylinder',(.0375,.019),(0.,0.,.004)),
        Solid('box',(.058,.067,.032),(0.,0.,.021),color=METAL),
        Solid('box',(.049,.083,.009),(0.,0.,.037),color=RAIL),
        Solid('box',BODY,(0.,0.,.071),color=BLACK),
        Solid('cylinder',(.026,.026),(.034,0.,.074),axis='X',color=METAL),
        Solid('cylinder',(.020,.012),(.051,0.,.074),axis='X',color=(.040,.091,.115)),
        Solid('box',(.004,.055,.034),(-.023,0.,.073),color=(.062,.108,.125)),
        Solid('cylinder',(.011,.140),(-.085,-.048,-.002),axis='X',color=METAL),
        Solid('cylinder',(.014,.053),tuple(HANDLE_LOCAL),axis='X',color=BLACK),
        Solid('cylinder',(.016,.023),(0.,-.043,.011),axis='Y',color=BLACK),
    ),
    RAIL_BASE.name: (
        Solid('box',RAIL_BASE.size),
        *(Solid('box',(.016,.674,.006),(dx,0.,.022),color=METAL) for dx in (-.034,.034)),
        Solid('box',(.013,.655,.002),(0.,0.,.0197),color=BLACK),
    ),
    SUBJECT.name: (
        Solid('box',(.019,TARGET_W,TARGET_H),(0.,0.,0.)),
        Solid('box',(.018,.018,.093),(0.,0.,-.097),color=RAIL),
        Solid('box',(.081,.083,.018),(0.,0.,TOP+.0545-SUBJECT_Z),color=METAL),
    ),
}
VISUAL_PARTS = {
    SUBJECT.name: (
        Solid('box',(.001,.033,.006),(-.010,0.,0.),color=WHITE),
        Solid('box',(.001,.006,.031),(-.010,0.,0.),color=WHITE),
    ),
    RAIL_BASE.name: tuple(Solid('box',(.084,.004,.001),(0.,y-RAIL_BASE.pos[1],.026),color=WHITE)
                         for y in (START_Y,END_Y)),
}
OBJECT_MATERIALS={MOUNT.name:'tabletop tripod with aluminium legs and rubber feet',
                  HEAD.name:'damped aluminium fluid head, rubber grip and compact camera',
                  RAIL_BASE.name:'aluminium motorized slider rail',
                  SUBJECT.name:'orange rigid sign and its slider carriage'}
SUPPORT_JOINTS={MOUNT.name:{'kind':'fixed'},
                SUBJECT.name:{'kind':'prismatic','axis':'Y','lower':-.007,'upper':END_Y-START_Y+.007}}
OBJECT_JOINTS={HEAD.name:{'parent':MOUNT.name,'kind':'revolute','axis':'Z','anchor':tuple(PIVOT),
                          'drive':{'stiffness':0.,'damping':.0035,'max_force':.15}}}
SURFACE_FRICTION={HEAD.name:(.76,.62)}
CONTACT_OFFSETS_M={HEAD.name:.0006,SUBJECT.name:.0008}
MOTIONS={SUBJECT.name:SubjectCrossing(SUBJECT.pos,END_Y-START_Y,START_S,DURATION_S)}

capture_lens=Lens.from_focal(APERTURE_MM/(2.*HALF_WIDTH),640,360)
TASK_RECORD_CAMERA=(HEAD.name,Camera(tuple(map(float,CAMERA_LOCAL)),(0.,0.,0.,1.),'world',capture_lens))
top_position=np.array((-.16,-.04,1.98))
top_forward=np.array((.465,-.04,.86))-top_position
top_forward /= np.linalg.norm(top_forward)
top_right=np.array((0.,-1.,0.))
top_rotation=np.column_stack((top_right,np.cross(top_right,top_forward),-top_forward))
TASK_TOP_CAMERA=Camera(tuple(map(float,top_position)),tuple(map(float,matrix_to_quat_xyzw(top_rotation))),
                       'opengl',Lens.from_focal(12.4,640,360))

TARGET_CORNERS=np.array([(-.0095,y,z) for y in (-TARGET_W/2,TARGET_W/2)
                         for z in (-TARGET_H/2,TARGET_H/2)])
SUCCESS_HOLD_S=.10
MAX_PARTIAL_SCORE=.50
LIVE_CONTEXT={}
_inside=_centered=_frames=0
_invalid=False


def on_reset():
    global _inside,_centered,_frames,_invalid
    _inside=_centered=_frames=0
    _invalid=False


def framed_corners(objects, origins):
    camera_rotation=quat_xyzw_to_matrix(objects[HEAD.name].quat_xyzw)
    camera_center=origins[HEAD.name]+camera_rotation@CAMERA_LOCAL
    subject_rotation=quat_xyzw_to_matrix(objects[SUBJECT.name].quat_xyzw)
    corners=origins[SUBJECT.name]+TARGET_CORNERS@subject_rotation.T
    local=(corners-camera_center)@camera_rotation
    if np.min(local[:,0]) <= .02:
        return False,False
    horizontal=float(np.max(np.abs(local[:,1]/local[:,0])))
    vertical=float(np.max(np.abs(local[:,2]/local[:,0])))
    roll=abs(math.atan2(camera_rotation[2,1],camera_rotation[2,2]))
    inside=horizontal<=HALF_WIDTH and vertical<=HALF_HEIGHT
    return inside,inside and horizontal<=HALF_WIDTH/2. and roll<=math.radians(2.)


def observe(objects, context, dt):
    global _inside,_centered,_frames,_invalid
    t=context['time_s']
    frame=int(round(t/dt))
    first=int(round(START_S/dt))
    last=int(round(END_S/dt))
    if not first < frame <= last:
        return
    origins=context['body_origins']
    expected=MOTIONS[SUBJECT.name].at(t).position
    if math.dist(origins[SUBJECT.name],expected)>.018 or math.dist(origins[MOUNT.name],MOUNT.pos)>.004:
        _invalid=True
    inside,centered=framed_corners(objects,origins)
    _inside+=int(inside)
    _centered+=int(centered)
    _frames+=1


def success(objects):
    return bool(not _invalid and _frames>=160 and _centered>=144)


def score(objects):
    if _invalid:
        return 0.
    if success(objects):
        return 1.
    return .50 if _inside>=80 else 0.


PINCH=-.010
GRASP=PIVOT+yaw_matrix(BEGIN_YAW)@HANDLE_LOCAL


def demo():
    return {'right': [
        Waypoint(1.35,tuple(GRASP+(0.,0.,.069)),FINGER_OPEN,'approach'),
        Waypoint(1.05,tuple(GRASP),FINGER_OPEN,'descend'),
        Waypoint(.85,tuple(GRASP),PINCH,'grasp the rubber pan handle'),
        Waypoint(DURATION_S,tuple(GRASP),PINCH,'follow the marked crossing'),
        Waypoint(.55,tuple(GRASP),FINGER_OPEN,'release after the crossing'),
        Waypoint(1.0,tuple(GRASP+(0.,0.,.093)),FINGER_OPEN,'lift clear'),
    ]}


def demo_target(arm, phase, time_s, position, rotation, finger):
    if time_s <= START_S:
        return position,rotation,finger
    y=MOTIONS[SUBJECT.name].at(min(END_S,time_s+.09)).position[1]
    yaw=aim_at(y)
    trim=math.radians(8.2)*(1.-math.exp(-12.*(time_s-START_S)))
    grip_yaw=yaw-trim
    target=PIVOT+yaw_matrix(grip_yaw)@HANDLE_LOCAL
    target[2]+=position[2]-GRASP[2]
    return target,yaw_matrix(grip_yaw-BEGIN_YAW)@grip_rotation(),finger
