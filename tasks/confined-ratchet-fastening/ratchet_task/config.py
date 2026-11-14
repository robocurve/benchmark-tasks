"""SI units; a small instrument bracket, not a service-torque connection."""
import numpy as np
TABLE = .7475
AXIS = np.array([.34, 0., TABLE + .093])
PITCH = .00125
BOLT_LENGTH = .016
BRACKET_THICKNESS = .003
THREAD_ENTRY_Z = AXIS[2]-BRACKET_THICKNESS
HEAD_AF = .013
HEAD_HEIGHT = .0053
BOLT_MASS = .014
HANDLE_MASS = .3954052755732919 + .004  # Source shell plus two light rubber grip pads.
PAWL_MASS = .02835521567254335
SOCKET_MASS = .040
CAPTURE_OFFSET = .0007
CAPTURE_TILT = np.deg2rad(5)
THREAD_START = np.deg2rad(15)
SOLVER_POSITION = 64
SOLVER_VELOCITY = 16
PHYSICS_HZ = 480
CONTROL_HZ = 20
MAX_STEPS = 4000
# Seating uses the actual HeadContact/Bracket collision surfaces. Offsets only
# create candidate contacts; positive speculative contacts do not prove seating.
BRACKET_TOP = .093
BRACKET_BORE_RADIUS = .0046
BRACKET_SECTORS = 48
SEAT_CONTACT_OFFSET = .00001
SEAT_ROUNDOFF = .000002
SEAT_MAX_PENETRATION = .00005
SEAT_MAX_EDGE_GAP = .00002
SEAT_MIN_FORCE = .001
SEAT_CONTACT_FRACTION = .9
# Backup angular limit sits behind the physical seat, so it cannot support a
# separated head. This is an integrity backstop, not the scored seating surface.
SEAT_STOP_GAP = -.0001
INITIAL_GAP = .0025  # Reproducible under-head clearance, measured from bracket top.
BOLT_START = AXIS + np.array([0.,0.,INITIAL_GAP])
TOOL_PARK = np.array([.30,-.21,TABLE+.036])
TOOL_WORK_START = BOLT_START + np.array([0.,0.,.0285])
INSTRUCTION = ('Pick up the ratchet from its cradle, fit the socket onto the started bracket '
 'bolt, then tighten it with short back-and-forth strokes between the walls until the bolt is seated.')
