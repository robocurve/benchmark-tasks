# SPDX-License-Identifier: MIT
"""Shared SI-unit geometry and measurement contract; imports no simulator."""
PHYSICS_SOLVER = "PGS"
POSITION_ITERATIONS = 64
VELOCITY_ITERATIONS = 16
TABLE_TOP = 0.7475
FIXTURE_POS = (0.29, 0.0, TABLE_TOP + 0.008)
FIXTURE_SIZE = (0.080, 0.280, 0.016)
POST_COLOR = (.93, .92, .82)
POST_SIZE = (0.018, 0.018, 0.040)
POST_CENTER_LOCAL = (0., 0., .028)
BASE_TOP_LOCAL = .008
WASHER_SIZE = (.068, .068, .014)
WASHER_HOLE = .022
WASHER_SEATED_LOCAL = (0., 0., .015)
CAP_SIZE = (.042, .042, .050)
CAP_HOLE = .022
CAP_CAVITY_HEIGHT = .044
CAP_GRIP_SIZE = (.022, .020, .032)
WASHER_GRIP_SIZE = (.009, .020, .032)
FIXTURE_GRIP_SIZE = (.026, .020, .024)
CAP_GRIP_LOCAL = (0., 0., .066)
WASHER_GRIP_LOCAL = (-.0295, 0., .023)
FIXTURE_EXTENSION_SIZE = (.030, .100, .016)
FIXTURE_EXTENSION_LOCAL = (0., .190, 0.)
FIXTURE_GRIP_LOCAL = (0., .220, .038)
CAP_INITIAL = (.29, 0., TABLE_TOP + .016)
WASHER_INITIAL = (.23, -.22, TABLE_TOP + .007)
REFERENCE_POS = (.43, .19, TABLE_TOP + .008)
PARK_SIZE = (.09, .09, .006)
PARK_POS = (.17, -.12, TABLE_TOP + .003)
DYNAMIC_NAMES = ('fixture', 'cap', 'washer')
REFERENCE_NAMES = ('reference_fixture', 'reference_washer', 'reference_cap')
INSTRUCTION = ('Compare the two fixtures. Restore the missing orange spacer washer beneath '
               'the grey protective cap on the blue fixture so it matches the complete reference. '
               'Stabilize the blue fixture, remove and park its cap, fit the loose washer over '
               'the white post, then replace the cap. Release the parts and move both hands clear. '
               'Do not drop any part off the table.')
# Raw measurement schema, sampled at each 20 Hz control step after reset.
# {t: float seconds, step: int, poses: {name:[x,y,z,qx,qy,qz,qw]},
#  velocities:{name:[vx,vy,vz,wx,wy,wz]},
#  tcp:{left:[x,y,z,qx,qy,qz,qw],right:[...]},
#  grippers:{left:normalized_open,right:normalized_open}, joints:[14 floats]}
TRACE_SCHEMA = 'missing-washer-rework-measurements-v1'
