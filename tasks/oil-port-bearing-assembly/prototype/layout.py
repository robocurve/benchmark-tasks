"""Real through-port geometry; clearance-fit optical/mechanical prototype only."""
import math,os
from yam_arena.task_file import Box,TABLE_TOP
TOP=TABLE_TOP['station']
SCENE_REVISION='oil-port-true-diameter-v6'
INSTRUCTION=('Insert the sleeve fully into the housing, with its radial oil hole aligned '
             'to the housing oil passage. Release it and leave it seated and still for '
             'at least one second. Gravity seating is allowed. This operation ends '
             'before a separate locking operation.')
MODE=os.environ.get('ROBOCURVE_PROBE_MODE','setup')
HOUSING=(.32,.025,TOP+.034)
START=(.20,-.16,TOP+.024)
SEATED=(HOUSING[0],HOUSING[1],HOUSING[2])
SLEEVE_POS=SEATED if MODE.startswith('seated_') else START
SLEEVE_YAW=(math.pi/2 if MODE=='seated_aligned' else 3*math.pi/2 if MODE=='seated_blocked' else math.pi/4)
MOVING=['sleeve']
OBJECTS=[Box('housing',pos=HOUSING,size=(.002,.002,.002),color=(.35,.41,.44)),
         Box('staging',pos=(START[0],START[1],TOP+.004),size=(.002,.002,.002),color=(.28,.31,.34)),
         Box('sleeve',pos=SLEEVE_POS,size=(.002,.002,.002),mass=.270,yaw=SLEEVE_YAW,color=(.76,.55,.26))]
R_IN=.019;R_OUT=.026;HEIGHT=.032;SLEEVE_PORT=.006
HOUSING_IN=.0268;HOUSING_OUT=.055;HOUSING_HEIGHT=.038;HOUSING_PORT=.005
PORT_YAW=math.pi/2

def parts(name):
    if name=='housing':
        # Four pieces leave the central 40 mm opening real, including in the base.
        return [('base_front',(0.,.04125,-.027),(.135,.0425,.014),(.35,.41,.44)),
                ('base_back',(0.,-.04125,-.027),(.135,.0425,.014),(.35,.41,.44)),
                ('base_left',(-.04375,0.,-.027),(.0475,.040,.014),(.35,.41,.44)),
                ('base_right',(.04375,0.,-.027),(.0475,.040,.014),(.35,.41,.44))]
    if name=='staging':return [('pad',(0.,0.,0.),(.08,.08,.008),(.28,.31,.34))]
    if name=='sleeve':return []
    raise KeyError(name)

def ring(name):
    if name=='housing':return dict(inner=HOUSING_IN,outer=HOUSING_OUT,height=HOUSING_HEIGHT,
                                  port_radius=HOUSING_PORT,port_angle=PORT_YAW,color=(.35,.41,.44))
    if name=='sleeve':return dict(inner=R_IN,outer=R_OUT,height=HEIGHT,
                                 port_radius=SLEEVE_PORT,port_angle=0.,color=(.76,.55,.26))
    return None

def joints():return []

def extra_rings(name):
    if name!='housing':return []
    # A small number of broad convex support sectors avoids an overconstrained
    # contact manifold between hundreds of coplanar wedges. The inner octagon
    # remains outside the 19 mm central-bore radius, including between vertices.
    return [('seat',dict(inner=.0207,outer=HOUSING_IN,height=.004,port_radius=0.,
                        port_angle=0.,segments=8,color=(.35,.41,.44)),-.018)]

def waypoints(mode):
    from yam_arena.demo import Waypoint
    from yam_arena.constants import FINGER_OPEN as O,FINGER_CLOSED as C
    C=-.0250  # 50 mm commanded jaw spacing; preserve the official gripper gains.
    initial_yaw=math.pi/4 if mode!='covered_grasp' else -math.pi/4
    target_yaw=initial_yaw+math.pi/4
    if mode=='gravity_wrong_angle':target_yaw-=math.pi/2
    x,y,_=START;z=START[2]+.010;hover=TOP+.12
    # Fixed known-layout demo calibration for the small grasp-centering offset.
    # Only ordinary commanded TCP targets change; object/score/fixture do not.
    hx,hy=HOUSING[0],HOUSING[1]+(.0008 if mode.startswith('gravity_') else 0.)
    # Stop above the housing rim. Full press fitting is deliberately not claimed.
    mouth_z=HOUSING[2]+HOUSING_HEIGHT/2+HEIGHT/2+.010+.005
    sequence=[Waypoint(3.,(x,y,hover),O,'approach sleeve',initial_yaw),
            Waypoint(2.,(x,y,z),O,'descend',initial_yaw),
            Waypoint(1.,(x,y,z),C,'grasp',initial_yaw),
            Waypoint(2.,(x,y,hover),C,'lift',initial_yaw),
            Waypoint(3.,(hx,hy,hover),C,'orient toward oil port',target_yaw),
            Waypoint(2.,(hx,hy,mouth_z),C,'approach mouth',target_yaw),
            Waypoint(4.,(hx,hy,mouth_z),C,'inspect held sleeve',target_yaw),
            Waypoint(2.,(hx,hy,hover),C,'retreat',target_yaw)]
    if mode in ('gravity_assembly','gravity_wrong_angle','gravity_held'):
        # The body is genuinely entered 8 mm before the jaws open. Its remaining
        # descent is unactuated rigid-body gravity/contact, not a release callback.
        lead=.008
        insertion_z=HOUSING[2]+HOUSING_HEIGHT/2+HEIGHT/2+.010-lead
        sequence=sequence[:-1]+[
            Waypoint(3.,(hx,hy,insertion_z),C,'guide body into bore',target_yaw),
            Waypoint(1.,(hx,hy,insertion_z),C,'hold partial insertion',target_yaw),
            Waypoint(1.5,(hx,hy,insertion_z),O,'release to gravity',target_yaw),
            Waypoint(2.,(hx,hy,hover),O,'withdraw open gripper',target_yaw),
            Waypoint(4.,(hx,hy,hover),O,'released settling',target_yaw)]
        if mode=='gravity_held':
            sequence=sequence[:-3]+[Waypoint(7.5,(hx,hy,insertion_z),C,
                                             'remain grasped above seat',target_yaw)]
    return sequence


def view_targets():
    return [('supply_port',(.32,.23,TOP+.13),HOUSING),
            ('staged_hole',(.35,-.01,TOP+.15),START)]
