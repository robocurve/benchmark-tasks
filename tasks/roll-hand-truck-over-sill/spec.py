"""Full-size compact hand truck; SI units throughout."""
CONTROL_HZ = 20
PHYSICS_HZ = 240
DWELL_STEPS = 10
WHEEL_SUPPORT_GAP = .003
FINGER_FORCE_LIMIT = 40.
PAD_TORSIONAL_RADIUS = .002
WORKSPACE = ((-.35,-.65,-.005),(1.1,.65,1.4))
AXLE = (.455,0.,.0635)
WHEEL_RADIUS = .0635
WHEEL_WIDTH = .028
WHEEL_Y = .1955
CART_MASS = 3.084
CASE_MASS = 1.
CASE_SIZE = (.220,.300,.160)
CASE_START = (.575,0.,.084)
HANDLE_HEIGHT = .9781
HANDLE_LOCAL_Z = HANDLE_HEIGHT-AXLE[2]
GRIP_Y = .125
PLATE_SIZE = (.2794,.381,.004)
SILL_X = (.370,.390)
SILL_HEIGHT = .015
FINISH_X = .365

def finger_joint_position(aperture):
    if not 0<=aperture<=.075:raise ValueError('Aperture exceeds 75 mm')
    return -min(aperture,.0749)/2
