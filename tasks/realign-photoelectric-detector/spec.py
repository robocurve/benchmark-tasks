"""Prototype SI parameters. Manufacturer envelopes; estimated friction/compliance."""
import math
CONTROL_HZ = 20
PHYSICS_HZ = 2400
CLAMP_UPDATE_HZ = 480
DWELL_STEPS = 10
TABLE_Z = .7475
FINGER_FORCE_LIMIT = 40.
WORKSPACE = ((-.19,-.65,TABLE_Z-.01),(.91,.65,1.25))
BASE = (.36, 0., TABLE_Z+.025)
CLAMP_POST = (.330, .0273, TABLE_Z+.060)
INITIAL_YAW = math.radians(-14)
MOUNT_OFFSET = (0.,-.0273,0.)
PLATE_OFFSET = (0.,-.0173,0.)
def mount_origin(yaw):
    c,s=math.cos(yaw),math.sin(yaw)
    x,y,z=MOUNT_OFFSET
    return (CLAMP_POST[0]+c*x-s*y,CLAMP_POST[1]+s*x+c*y,CLAMP_POST[2]+z)
PIVOT = mount_origin(INITIAL_YAW)
ALIGNED_PIVOT = mount_origin(0.)
# Nut centre at the rear of the clamp; the socket axis points toward -x.
CLAMP = (CLAMP_POST[0]-.0137,CLAMP_POST[1]+.015,CLAMP_POST[2])
SENSOR_CENTER_LOCAL = (.031,0,.02525)
SENSOR_SIZE = (.042,.0156,.0485)
LENS_LOCAL = (.0525,0,.02525)
REFLECTOR = (CLAMP_POST[0]+.400,0,TABLE_Z+.08525)
REFLECTOR_SIZE = (.004,.018,.060)
CARD_SIZE = (.040,.040,.060)
CARD_START = (.25,.18,TABLE_Z+.031)
def tool_reset_origin():
    c,s=math.cos(INITIAL_YAW),math.sin(INITIAL_YAW)
    x,y,z=(CLAMP[0]-CLAMP_POST[0]+.0015,CLAMP[1]-CLAMP_POST[1],0.)
    return (CLAMP_POST[0]+c*x-s*y,CLAMP_POST[1]+s*x+c*y,CLAMP_POST[2]+z)
DRIVER_POS = tool_reset_origin()
DRIVER_HEAD_Z = .045
DRIVER_GRIP_LOCAL = (.120,0.,.045)
DRIVER_SPRING_NM_RAD = 10.
DRIVER_SPRING_DAMPING = .10
DIAL_DEG_PER_NM = 90.
CONTACT_ANGULAR_STIFFNESS = 1000.  # N m/rad, estimated elastic microslip.
CONTACT_ANGULAR_DAMPING = .2  # N m s/rad, within the native force cap.
CONTACT_LINEAR_STIFFNESS = 1.e6  # N/m, estimated axial microslip.
CONTACT_LINEAR_DAMPING = 10.  # N s/m.
TORQUE_MIN = 3.0
TORQUE_MAX = 3.5
RESET_TORQUE = .10
INDICATED_TORQUE_MIN = 3.10
INDICATED_TORQUE_MAX = 3.40
THREAD_PITCH = .0008
CLAMP_STIFFNESS = 4.e7
CLAMP_FRICTION_RADIUS = .010
MAX_CLAMP_FORCE = 16000.
NUT_FACTOR = .20
BOLT_DIAMETER = .005
FRICTION = .25
PAD_STATIC_FRICTION = 1.2  # Estimated rubber-like pad, matching upstream YAM.
PAD_DYNAMIC_FRICTION = 1.0
PAD_TORSIONAL_RADIUS = .002  # Estimated finite rubber contact patch, native TGS friction.
CABLE_STIFFNESS_NM_RAD = .05
CABLE_DAMPING_NMS_RAD = .001
BEAM_HALF_ANGLE_RAD = math.atan(.0125/1.5)
MAX_REFLECTOR_INCIDENCE_DEG = 30.

def finger_joint_position(aperture):
    if not 0<=aperture<=.075: raise ValueError('Aperture exceeds 75 mm')
    return -min(aperture,.0749)/2
