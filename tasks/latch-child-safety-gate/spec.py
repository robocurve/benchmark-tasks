"""Regalo-style pressure-mounted walk-through child safety gate; SI units."""
import math

CONTROL_HZ = 20
PHYSICS_HZ = 240
DWELL_STEPS = 10
FINGER_FORCE_LIMIT = 40.
PAD_TORSIONAL_RADIUS = .002

# Installed gate geometry (generic reconstruction of Regalo Easy Step class).
# Opening ~762 mm; walk-through door ~406 mm; overall height 762 mm (30 in).
OPENING_WIDTH = .762
DOOR_WIDTH = .406
GATE_HEIGHT = .762
BAR_GAP = .051
FRAME_TUBE = .019
# Clears pedestals when open at −35°, still inside left-arm reach for a mid-stile push.
HINGE_X = .500
HINGE_Y = OPENING_WIDTH / 2 - FRAME_TUBE
# Door yaw about +Z; 0 = fully closed across the opening toward -Y.
# Negative yaw swings the free stile toward the arms (−X).
DOOR_START_YAW = math.radians(-35)
DOOR_CLOSED_YAW = 0.
DOOR_CLOSED_TOL = math.radians(7)
# Top lock lever: 0 = locked (down), positive = raised/open. Units: radians about local Y.
LEVER_START = math.radians(55)
LEVER_LOCKED = 0.
LEVER_LOCKED_TOL = math.radians(8)
# Bottom base lock: 0 = locked (down toward floor stop), pi/2 = horizontal (swing free).
BASE_START = math.pi / 2
BASE_LOCKED = 0.
BASE_LOCKED_TOL = math.radians(12)
# Spring latch must overlap the receiving catch when the door and lever are locked.
LATCH_OVERLAP_MIN = .004
# Prismatic extension along door +X (authored seat is outside the stile; spring target ~20 mm).
LATCH_SEAT_X = .014
LATCH_EXTEND_MIN = .010  # prismatic slide beyond seat
# After a closed, hands-clear release, opening past this angle from fully closed zeros the trial.
# Must stay above DOOR_CLOSED_TOL so a door released anywhere inside the closed band cannot trip it.
REOPEN_LIMIT = DOOR_CLOSED_TOL + math.radians(5)

DOOR_MASS = 2.4
LEVER_MASS = .045
BASE_LOCK_MASS = .035
SPRING_LATCH_MASS = .025

# Rectangular top handle on the free stile (door local: -Y toward latch side).
HANDLE_SIZE = (.022, .090, .025)
HANDLE_LOCAL = (.020, -(DOOR_WIDTH - .050), GATE_HEIGHT - .040)

WORKSPACE = ((-.55, -.90, -.01), (1.25, .90, 1.50))

INSTRUCTION_FALLBACK = (
    "Close the child safety gate across the hallway opening and lock it: "
    "swing the walk-through door shut, push the top lock lever down so the spring latch engages, "
    "and turn the bottom base lock down. Leave both hands clear."
)


def finger_joint_position(aperture):
    if not 0 <= aperture <= .075:
        raise ValueError('Aperture exceeds 75 mm')
    return -min(aperture, .0749) / 2
