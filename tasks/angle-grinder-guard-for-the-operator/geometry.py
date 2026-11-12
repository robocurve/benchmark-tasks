"""Geometry of the angle-grinder scene on the yam_arena station table, shared by build_assets.py, task.py, demo.py and
the tests. Pure Python (no Isaac Sim), so the task file can import it before the simulator starts.

World frame (yam_arena's): x forward from the arms, y to the robot's left, z up, metres. Heights are above the table
top (TOP) unless they say "world". Arm bases: left (0, +0.305), right (0, -0.305).

THE SCENE. A 115 mm angle grinder lies on its side on the table, disc facing up: its motor housing along the asset's
-y axis, its gear head at the origin, the spindle vertical through the origin. A stamped-steel guard (a hood over
about 190 degrees of the disc) sits on the gear head's collar, clamped by a quick-release lever. With the lever shut
the guard cannot turn; with the lever raised it turns freely about the spindle. The guard's closed side must face
the operator, who stands at a printed OPERATOR mark.

ANGLES. The guard's "closed-side bisector" is measured as a bearing seen from above, anticlockwise from world +x, in
degrees in this file's comments and radians in code. The asset is built with the bisector along its own +x, at joint
angle 0.
"""

from __future__ import annotations

import math

TOP = 0.7475                                  # yam_arena.task_file.TABLE_TOP["station"]

# -- the grinder body (asset frame: origin on the spindle axis at table height; the motor lies along -y) -----------------
BODY_MASS = 1.55                              # kg (a real 115 mm, 750 W grinder is 1.5 to 1.8 kg)
MOTOR_R = 0.029                               # 58 mm across
MOTOR_Y0, MOTOR_Y1 = -0.215, -0.035           # the motor housing's ends along y
CAP_R, CAP_Y0 = 0.027, -0.240                 # the rear cap: -0.240 .. -0.215
HEAD_HX, HEAD_Y0, HEAD_Y1, HEAD_H = 0.031, -0.035, 0.037, 0.058   # gear head box: 62 mm wide, 58 mm tall
NECK_R, NECK_Z0, NECK_Z1 = 0.0215, 0.058, 0.068   # the 43 mm collar the guard clamps on
SPINDLE_R, SPINDLE_Z1 = 0.0055, 0.100
HANDLE_R, HANDLE_LEN, HANDLE_Z = 0.0115, 0.115, 0.034   # the side handle, screwed into the head's -x side, along x
GRIND_FRICTION = (0.6, 0.5)                   # the grinder on the table (static, dynamic)

# -- the disc ----------------------------------------------------------------------------------------------------------------
DISC_R, DISC_T = 0.0575, 0.006                # 115 x 6 mm grinding disc
DISC_Z0 = 0.074                               # its underside: 3 mm over the guard's plate
DISC_Z1 = DISC_Z0 + DISC_T
FLANGE_R, FLANGE_Z0, FLANGE_Z1 = 0.020, 0.071, DISC_Z0    # the inner flange under the disc
NUT_R, NUT_Z0, NUT_Z1 = 0.020, DISC_Z1, DISC_Z1 + 0.010   # the clamp nut on top

# -- the guard (link frame: origin on the spindle axis at PLATE_Z0; the closed-side bisector along +x) --------------------
GUARD_MASS = 0.18
PLATE_Z0, PLATE_T = 0.068, 0.003              # the hood's flat plate, over the collar
SKIRT_R_IN, SKIRT_T, SKIRT_Z1 = 0.062, 0.0025, 0.100   # the skirt: 4.5 mm clear of the disc rim, up to 100 mm
SKIRT_SPAN = math.radians(190)                # the hood covers 190 degrees (the closed side)
LUG_PHI = math.radians(60)                    # the adjusting lug, 60 degrees anticlockwise of the bisector (it leads the lever)
LUG_L, LUG_W, LUG_H = 0.025, 0.008, 0.024     # radial length, thickness along the circle, height: a tab standing 20 mm above the skirt's rim
LUG_Z0 = SKIRT_Z1 - 0.004                     # its foot is 4 mm below the rim (it is welded to the skirt)
LUG_MID = LUG_Z0 + LUG_H / 2                  # its middle (heights above the table)
LEVER_PHI = math.radians(30)                  # the clamp lever's hinge, 30 degrees anticlockwise of the bisector
LEVER_HINGE_R, LEVER_HINGE_Z = 0.066, 0.090   # on the skirt's outer face
LEVER_L, LEVER_W, LEVER_T = 0.055, 0.014, 0.008   # lever arm: 55 mm long, 14 mm wide (the jaws' bite), 8 mm thick
LEVER_MASS = 0.02
LEVER_OPEN = math.radians(-90)                # joint angle of the fully raised lever (negative raises it)
LEVER_DAMPING = 0.03                          # N m s/rad
LEVER_K = 0.03                                # N m/rad: the lever's over-centre spring (task code flips its target)
LEVER_PRESS = 0.10                            # rad: the spring's target lies this far past each stop, so it keeps pressing on it
LEVER_TOGGLE_UP, LEVER_TOGGLE_DOWN = math.radians(45), math.radians(40)   # the lever snaps open past 45, shut below 40
GUARD_DAMPING = 0.02                          # N m s/rad: a free guard turns easily but does not drift

# -- the monitor's thresholds (the scoring rules; task.json and the README quote these) ------------------------------------
LEVER_OPENED = math.radians(60)               # raised at least this far counts as opened
LEVER_SHUT = math.radians(8)                  # within this of flat counts as shut (and the clamp holds the guard)
BEARING_OK = math.radians(30)                 # 1.00: the guard's closed side within 30 degrees of the operator
BEARING_NEAR = math.radians(45)               # 0.75
BEARING_HALF = math.radians(90)               # 0.50
MOVED_OK, YAW_OK, TILT_OK = 0.020, math.radians(10), math.radians(10)       # success: the grinder where it started
MOVED_BAD, YAW_BAD, TILT_BAD = 0.040, math.radians(20), math.radians(30)    # shoved or knocked over: latched
FORCED = math.radians(10)                     # the guard turned this far while clamped: forced (latched)
DISC_TOUCH = 0.006                            # a fingertip this close to the disc: touched (latched)
CLEAR_R = 0.13                                # arms clear: no fingertip within this of the spindle axis, above the table

# -- the operator mark (a flat printed mat) --------------------------------------------------------------------------------
MAT_SIZE = (0.14, 0.10)
MAT_DIST = 0.140                              # from the spindle axis to the mat's centre

# -- the two layouts, TASK_VARIANT=A or B --------------------------------------------------------------------------------
# A: the grinder at (0.255, +0.120), the motor towards -y, the operator in front (towards -x): bearing 180 degrees, the
#    guard starts with its closed side towards +x (bearing 0): 180 degrees to turn.
# B: the mirror: the grinder at (0.255, -0.100), the motor towards +y (asset yaw 180), the operator at the right
#    (towards -y): bearing 270 degrees, the guard starts at bearing 150: 120 degrees to turn.
LAYOUTS = {
    "A": dict(spindle=(0.255, +0.120), yaw=0.0, operator_bearing=math.radians(180), start_bearing=math.radians(0)),
    "B": dict(spindle=(0.255, -0.100), yaw=math.pi, operator_bearing=math.radians(270), start_bearing=math.radians(150)),
}


def layout(variant: str = "A") -> dict:
    """The placement of the grinder and the operator mark for a layout, plus derived values."""
    L = dict(LAYOUTS[variant.upper()])
    sx, sy = L["spindle"]
    b = L["operator_bearing"]
    L["mat"] = (sx + MAT_DIST * math.cos(b), sy + MAT_DIST * math.sin(b))
    L["guard_q0"] = wrap(L["start_bearing"] - L["yaw"])     # the guard joint's start angle
    L["name"] = variant.upper()
    return L


def wrap(a: float) -> float:
    """An angle in (-pi, pi]."""
    return math.remainder(a, 2 * math.pi)


def bearing_error(guard_bearing: float, operator_bearing: float) -> float:
    """How far (0..pi) the guard's closed-side bisector points from the direction of the operator."""
    return abs(wrap(guard_bearing - operator_bearing))


def guard_bearing(yaw: float, q: float) -> float:
    """The closed-side bisector's bearing: the asset's yaw plus the guard joint angle."""
    return wrap(yaw + q)


def polar(spindle, r: float, bearing: float, z: float = 0.0):
    """A world point at radius r from the spindle axis, at a bearing, at height z above the table."""
    return (spindle[0] + r * math.cos(bearing), spindle[1] + r * math.sin(bearing), TOP + z)


def lug_point(L: dict, q: float):
    """The lug's middle (world) for the guard at joint angle q."""
    return polar(L["spindle"], SKIRT_R_IN + SKIRT_T + LUG_L / 2, L["yaw"] + q + LUG_PHI, LUG_MID)


def lever_hinge(L: dict, q: float):
    """The lever's hinge (world) for the guard at joint angle q."""
    return polar(L["spindle"], LEVER_HINGE_R, L["yaw"] + q + LEVER_PHI, LEVER_HINGE_Z)
