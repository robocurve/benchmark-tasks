"""Geometry and trial constants, metres, radians. Z is up, x forward, y left.

The station is a research-scale polycarbonate hinged safety-guard mock-up.
Its steel latch bolt passes through an actual keeper slot when the door closes.
A spring-return plunger is physically displaced by a door-mounted actuator.
These are illustrative engineering tolerances, not certified machinery limits.
"""
from __future__ import annotations
import math

# The guard USD origin is the hinge at the left-hand frame upright.
# Source geometry is local to it (door closes along -Y and opens toward -X).
HINGE_X = 0.315
HINGE_Y = 0.104
# Height is computed from yam_arena's station table in task.py.
DOOR_HEIGHT = 0.145
DOOR_WIDTH = 0.170
DOOR_THICKNESS = 0.008
DOOR_HANDLE_LOCAL = (-.060, -.111, .132)
DOOR_HANDLE_WIDTH = 0.025
GUIDE_INNER_APERTURE_M = 0.018  # Stationary guide vertical aperture
KEEPER_WIDTH_M = 0.0095  # From -5 mm to +4.5 mm about the shaft center
LOCK_SHAFT_DIAMETER_M = 0.008

# Distances are measured from the guard hinge, not a manually guessed world origin.
CLOSED_RAD = 0.0
OPEN_RAD = math.radians(-53.0)
HINGE_MIN_RAD = math.radians(-65.0)
HINGE_MAX_RAD = math.radians(3.0)
CLOSED_TOL_RAD = math.radians(5.0)
UNSAFE_SWITCH_OPEN_RAD = math.radians(12.0)

# Thumb pad is outboard of the stationary slotted guide. Its steel neck
# overlaps the shaft and travels through the guide's open side.
BOLT_PAD_LOCAL = (-.052,-.125,.055)
BOLT_START = 0.0               # prismatic translation relative to slide's home
BOLT_TRAVEL = 0.052            # travel toward +Y through door's mechanical eyelet
BOLT_SEATED = 0.039           # shaft projects beyond the keeper by >=~12mm
BOLT_TOL = 0.003

SWITCH_TRAVEL = 0.007          # door tongue must depress spring switch
SWITCH_ACTIVE = 0.0045
SWITCH_RELEASE = 0.0025

GRIPPER_CLEARANCE = 0.060     # whole-arm approximation to hinge, bolt and door
INTERLOCK_HOLD_S = 1.0        # sustained measured state once arms withdrawn
CONTROL_HZ = 20
CLEAR_TICKS = int(INTERLOCK_HOLD_S * CONTROL_HZ) + 1

# Area behind the guard front; the robot must not reach into the machine-access
# opening while guard is open. This is an approximate link-capsule screening box.
DANGER_X = (HINGE_X + .031, 0.48)
DANGER_Y = (-0.105, 0.095)
DANGER_Z_REL = (0.016, 0.145)
LINK_RADIUS = 0.020

# Mechanical constitutive parameters / material spec for real-world scale model.
BASE_MASS_KG = 1.6
DOOR_MASS_KG = 0.18
BOLT_MASS_KG = 0.036
SWITCH_MASS_KG = 0.008
HINGE_OPEN_STIFFNESS_NM_RAD = 1.2  # passive spring bias toward OPEN_RAD, never closed
HINGE_DAMPING_NMS_RAD = 0.12  # dynamic damping for open-biased physical hinge
SWITCH_SPRING_N_M = 85.0
SWITCH_DAMPING_NS_M = 2.0
BOLT_RETRACT_STIFFNESS_N_M = 18.0  # bolt spring returns toward BOLT_START under zero contact
BOLT_SLIDE_DAMPING_NS_M = 7.0
STATIC_FRICTION = 0.65
DYNAMIC_FRICTION = 0.50
RESTITUTION = 0.04
CONTACT_OFFSET_M = 0.0005
REST_OFFSET_M = 0.0

# Model measured switch/bolt in their respective joint's displacement coordinate.
def switch_ready(door_angle: float, switch_travel: float) -> bool:
    return abs(door_angle) <= CLOSED_TOL_RAD and switch_travel >= SWITCH_ACTIVE


def bolt_engaged(door_angle: float, bolt_travel: float) -> bool:
    return abs(door_angle) <= CLOSED_TOL_RAD and bolt_travel >= BOLT_SEATED


def within_joint_limits(hinge: float, bolt: float, switch: float, tol=0.001):
    return (HINGE_MIN_RAD-tol <= hinge <= HINGE_MAX_RAD+tol
            and -tol <= bolt <= BOLT_TRAVEL+tol
            and -tol <= switch <= SWITCH_TRAVEL+tol)
