# SPDX-License-Identifier: MIT
"""Single source of SI-unit scene, articulation and measurement geometry."""
PHYSICS_SOLVER = 'PGS'
POSITION_ITERATIONS = 64
VELOCITY_ITERATIONS = 16
TABLE_TOP = .7475
CARRIER_INITIAL_POS = (.280, 0., TABLE_TOP + .008)
CARRIER_INITIAL_YAW = 0.
CARRIER_INITIAL_Q = .050
CARRIER_FINAL_POS = (.475, 0., TABLE_TOP + .008)
CARRIER_SIZE = (.180, .140, .016)
CARRIER_MASS = .250
CROSSBAR_MASS = .040
CROSSBAR_SIZE = (.020, .140, .016)
CROSSBAR_ORIGIN_LOCAL = (0., 0., .022)
CROSSBAR_GRIP_SIZE = (.020, .024, .028)
CROSSBAR_GRIP_LOCAL = (0., 0., .024)
CARRIER_GRIP_SIZE = (.024, .024, .032)
CARRIER_GRIP_LOCAL = (-.120, 0., .056)
SLIDE_LIMITS = (-.055, .055)
CARRIER_STATIC_FRICTION = .15
CARRIER_DYNAMIC_FRICTION = .10
CARRIER_VISCOUS_FRICTION = .05
CARRIER_DAMPING = .50
CARRIER_MAX_FORCE = .40
DOCK_MOUTH = .380
DOCK_BACK = .585
DOCK_HALF_WIDTH = .075
DOCK_HEIGHT = .075
DOCK_POS = (.4825, 0., TABLE_TOP + .0425)
DOCK_SIZE = (.225, .170, .085)
BLUE = (.025, .38, .80)
ORANGE = (1., .30, .015)
WHITE = (.98, .98, .90)
GREY = (.34, .38, .43)
# Tuples: collider name, dimensions, centre in rigid-link frame, colour.
CARRIER_PARTS = (
 ('Body', CARRIER_SIZE, (0.,0.,0.), BLUE),
 ('RearStem', (.040,.024,.016), (-.110,0.,0.), BLUE),
 ('HandlePost', (.024,.024,.048), (-.120,0.,.024), BLUE),
 ('Handle', CARRIER_GRIP_SIZE, CARRIER_GRIP_LOCAL, BLUE),
 # Two simple guide cheeks visibly explain the captive y-slide.
 ('GuideFront', (.008,.140,.024), (.018,0.,.020), BLUE),
 ('GuideBack', (.008,.140,.024), (-.018,0.,.020), BLUE),
)
CROSSBAR_PARTS = (
 ('Bar', CROSSBAR_SIZE, (0.,0.,0.), ORANGE),
 ('Grip', CROSSBAR_GRIP_SIZE, CROSSBAR_GRIP_LOCAL, ORANGE),
)
# The scoring payload is the body and the complete sliding link; rear handle
# and stem intentionally remain outside the dock. Guide cheeks also enter.
PAYLOAD_CARRIER_PART_NAMES = ('Body','GuideFront','GuideBack')
DOCK_PARTS = (
 ('LeftWall', (.205,.010,.085), (0.,.080,0.), GREY),
 ('RightWall', (.205,.010,.085), (0.,-.080,0.), GREY),
 ('BackWall', (.010,.170,.085), (.1075,0.,0.), GREY),
 ('RoofFront', (.012,.170,.010), (-.0965,0.,.0375), GREY),
 ('RoofSecond', (.012,.170,.010), (-.0625,0.,.0375), GREY),
 ('RoofThird', (.012,.170,.010), (.0125,0.,.0375), GREY),
 ('RoofRear', (.012,.170,.010), (.0965,0.,.0375), GREY),
)
DYNAMIC_NAMES = ('carrier','crossbar')
REFERENCE_NAMES = ('dock',)
INSTRUCTION = ('Centre the orange sliding crossbar within the blue carrier. '
 'Push the intact carrier into the grey dock until its body and crossbar are '
 'fully inside; the rear handle stays outside. Release it and move both hands clear.')
TRACE_SCHEMA = 'captive-crossbar-docking-measurements-v1'
# Each 20 Hz record: t, step, poses[name]=XYZ+XYZW, velocities[name]=
# linearXYZ+angularXYZ, joint_position, joint_velocity, tcp[left/right]=
# XYZ+XYZW, grippers[left/right]=normalized opening, joints=14 robot angles.
