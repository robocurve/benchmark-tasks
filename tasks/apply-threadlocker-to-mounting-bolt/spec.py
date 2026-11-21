"""Real-size M16 x 80 mm fastener; SI units. Local bolt Z runs tip to head."""
import math
CONTROL_HZ=20
PHYSICS_HZ=480
DWELL_STEPS=10
FINGER_FORCE_LIMIT=40.
PAD_TORSIONAL_RADIUS=.002
TABLE_Z=.500
MAJOR_RADIUS=.008
PITCH=.002
MINOR_RADIUS=MAJOR_RADIUS-math.sqrt(3)*(5/12-1/16)*PITCH
BOLT_LENGTH=.080
HEAD_AF=.024
HEAD_HEIGHT=.010
BOLT_START=(.320,.150,.504)
TOOL_START=(.280,-.200,.540)
WAX_OFFSET=.044
WAX_SIZE=(.008,.010,.010)
WAX_DENSITY=1030.
TOOL_MASS=.050
# Midpoint quadrature of the actual thread profile, plus solid hex head.
def thread_radius(theta,z):
    phase=abs((z-PITCH*theta/(2*math.pi)+PITCH/2)%PITCH-PITCH/2)
    return MAJOR_RADIUS if phase<=PITCH/16 else MINOR_RADIUS if phase>=5*PITCH/12 else MAJOR_RADIUS-(phase-PITCH/16)*math.sqrt(3)
MEAN_R2=sum(thread_radius(0,(i+.5)*PITCH/4096)**2 for i in range(4096))/4096
BOLT_MASS=7850*(math.pi*MEAN_R2*BOLT_LENGTH+math.sqrt(3)/2*HEAD_AF**2*HEAD_HEIGHT)
MASSES={'bolt':BOLT_MASS,'tool':TOOL_MASS,'wax':math.prod(WAX_SIZE)*WAX_DENSITY}
TARGET_BAND=(.004,.010)
ANGLE_CELLS=72
AXIAL_CELLS=160
FILM_VISIBLE_M=.000025
FILM_CAP_M=.000150
TRANSFER_COEFFICIENT=5e-6  # m3 per N m; provisional reduced model, not calibrated product data.
WORKSPACE=((-0.15,-.65,.490),(1.00,.65,1.40))
def finger_joint_position(aperture):
    if not 0<=aperture<=.075:raise ValueError('Aperture exceeds 75 mm')
    return -min(aperture,.0749)/2
