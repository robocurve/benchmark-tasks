"""Geometry, masses and kinematics of the lathe set-up. NumPy only (no Isaac).

Units: metres, kilograms, radians unless a name ends in _DEG; quaternions xyzw. Station frame: x forward from the
line between the arm bases, y left, z up; TOP is the table top. Heights below are above the table top.

The lathe: a fixed headstock and chuck on the left holding an 8 mm round bar whose axis runs along y at height HC.
The tool approaches the bar from the robot's side (-x). Its carriage rides a prismatic joint along x (the cross
slide), driven by a soft spring toward a target set from the cross-feed crank (so the tool pinches the rule
lightly), and the tool rides a prismatic joint along z on the carriage, driven by a stiff spring toward a target set
from the height crank (the tool holder's height-adjusting screw). Both cranks are revolute joints (vertical axes)
turned through a knob that spins freely on the crank arm, as on the dividing head.

The steel rule is a free rigid plate. Trapped between the tool's cutting edge and the bar, it leans by about
(edge height - centre height) / bar radius: forward (toward the robot) when the tool is low, back when it is high.
"""

from __future__ import annotations

import math

import numpy as np

TOP = 0.7475  # station table top (yam_arena.task_file.TABLE_TOP["station"])

# -- lathe and bar -------------------------------------------------------------------------------------
HC = 0.100  # spindle centre height above the table
BAR_R = 0.004  # 8 mm bar
BAR_X = 0.300  # bar axis x
BAR_Y = (-0.040, 0.010)  # bar from its free end to the chuck face (50 mm out)
CHUCK_R, CHUCK_Y = 0.040, (0.010, 0.045)
HEAD_X, HEAD_Y, HEAD_Z = (0.250, 0.350), (0.045, 0.130), (0.0, 0.155)
BED_X, BED_Y, BED_Z = (0.220, 0.370), (-0.150, 0.130), (0.0, 0.030)

# -- tool ---------------------------------------------------------------------------------------------
CONTACT_Y = -0.025  # the tool and the rule meet the bar here (15 mm from its free end)
E0 = 0.00075  # the cutting edge starts 0.75 mm ABOVE centre height (fixed layout)
GAP0 = 0.008  # start gap between the cutting edge and the bar's surface (room to lower the rule in)
TIP_X0 = BAR_X - BAR_R - GAP0  # the cutting edge's front at the start
EDGE_T = 0.0003  # the edge: a 0.3 mm tall, 1 mm deep strip at the bit's top front
BIT = (0.020, 0.008, 0.008)  # tool bit (x long, y wide, z tall) behind the edge
RELIEF = 0.0015  # the bit body below the edge sits this far behind the edge's front (front clearance)
HOLDER = (0.040, 0.030, 0.050)  # tool holder block (visual) behind the bit
HOLDER_TOP_DZ = 0.020  # holder top above the edge

# -- cranks ------------------------------------------------------------------------------------------
FEED_PER_TURN = 0.00254  # cross feed: 0.100 in per turn (a 10 TPI screw), advancing for a clockwise turn
PITCH = 0.0254 / 24  # height screw: 24 threads per inch (1.058 mm per turn), lowering for a clockwise turn
CROSS_AXIS = np.array([0.200, 0.230])  # cross-feed crank axis (on a pedestal, front left: clear of the headstock and the rule hand)
CROSS_HUB_Z = 0.060
HEIGHT_AXIS = np.array([TIP_X0 - 0.050, CONTACT_Y])  # height crank axis, on top of the tool holder
HEIGHT_HUB_Z = HC + HOLDER_TOP_DZ  # holder top (the crank turns on it)
CRANK_ARM_R = 0.024  # knob radius from the crank axis
ARM_T = 0.006  # crank arm thickness
KNOB_R, KNOB_H = 0.008, 0.030
KNOB_DZ = ARM_T + KNOB_H / 2  # knob centre above the hub top
CRANK_DAMPING = 0.03 * math.pi / 180.0  # N m per (deg/s): 0.03 N m s/rad (0.2 stalled the compliant arm)

# -- cross slide and height drives -----------------------------------------------------------------
X_K, X_DAMP = 2000.0, 60.0  # N/m, N s/m: soft, so a tool pushed onto the rule pinches it with ~0.6 N
Z_K, Z_DAMP = 2.0e5, 3000.0  # stiff: the edge height holds to ~0.01 mm under the rule's pinch
PINCH = 0.0003  # cross-slide target this far past the rule's face once advanced (0.6 N on the rule)
X_TRAVEL = (-0.004, GAP0 + 0.0015)  # cross-slide joint limits from the start (m): past the bar's surface
Z_TRAVEL = (-0.004, 0.004)

# -- steel rule ----------------------------------------------------------------------------------------
RULE = (0.0010, 0.013, 0.150)  # thickness (x), width (y), length (z): a 6 in rigid steel rule (1 mm thick)
RULE_MASS = 0.0153  # 150 x 13 x 1 mm of steel
RULE_STAND_XY = np.array([0.170, -0.120])  # the rule stands in a slotted block here (in front of the right arm)
STAND = (0.030, 0.040, 0.035)  # slotted block (x, y, z); the slot runs along y
SLOT_W = 0.0013  # 1.3 mm slot for the 1 mm rule: it stands within 0.5 degrees of upright
RULE_PINCH_FROM_TOP = 0.100  # where the demonstration traps the rule (100 mm from its top end)

# -- masses --------------------------------------------------------------------------------------------
MASS = {"carriage": 0.30, "tool": 0.20, "xcrank": 0.08, "zcrank": 0.08, "xknob": 0.01, "zknob": 0.01}
INERTIA = {"carriage": (3e-4, 3e-4, 3e-4), "tool": (1e-4, 1e-4, 1e-4), "xcrank": (2e-5, 2e-5, 3e-5),
           "zcrank": (2e-5, 2e-5, 3e-5), "xknob": (1e-6, 1e-6, 5e-7), "zknob": (1e-6, 1e-6, 5e-7)}
FRICTION = {"lathe": (0.4, 0.35), "tool": (0.25, 0.2), "rule": (0.6, 0.5), "knob": (1.0, 0.9)}

# -- scoring ------------------------------------------------------------------------------------------
FULL_M = 0.00025  # full credit at or under 0.25 mm height error (the rule method's real accuracy, ~0.01 in)


def wrap_deg(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def yaw_quat(a: float) -> np.ndarray:
    return np.array([0.0, 0.0, math.sin(a / 2), math.cos(a / 2)])


def yaw_of(q) -> float:
    x, y, z, w = (float(v) for v in q)
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def pitch_of(q) -> float:
    """Rotation about world y (rad) of a body whose other rotations are small: the rule's lean in the x-z plane.
    Positive: the body's +z axis tips toward +x (the rule's top leans away from the robot, toward the bar)."""
    x, y, z, w = (float(v) for v in q)
    # body z axis in world: (2(xz + wy), 2(yz - wx), 1 - 2(x^2 + y^2))
    zx, zz = 2 * (x * z + w * y), 1 - 2 * (x * x + y * y)
    return math.atan2(zx, zz)


def edge_z0() -> float:
    return TOP + HC + E0


def crank_poses(prefix: str, axis_xy, hub_z: float, deg: float) -> dict[str, np.ndarray]:
    """Crank arm and knob poses for a crank angle (deg; 0 = knob on +x of the axis)."""
    a = math.radians(deg)
    q = yaw_quat(a)
    hub = np.array([axis_xy[0], axis_xy[1], TOP + hub_z])
    knob = hub + np.array([CRANK_ARM_R * math.cos(a), CRANK_ARM_R * math.sin(a), KNOB_DZ])
    return {f"{prefix}crank": np.array([*hub, *q]), f"{prefix}knob": np.array([*knob, *q])}


def knob_center(prefix: str, deg: float) -> np.ndarray:
    axis, hub = (CROSS_AXIS, CROSS_HUB_Z) if prefix == "x" else (HEIGHT_AXIS, HEIGHT_HUB_Z)
    return crank_poses(prefix, axis, hub, deg)[f"{prefix}knob"][:3]


START_DEG = {"x": 180.0, "z": 90.0}  # the cross-feed knob on the robot's side, the height knob on the left


def start_poses() -> dict[str, np.ndarray]:
    ident = [0.0, 0.0, 0.0, 1.0]
    poses = {"lathe": np.array([0.0, 0.0, TOP, *ident]),
             "carriage": np.array([TIP_X0, CONTACT_Y, edge_z0(), *ident]),
             "tool": np.array([TIP_X0, CONTACT_Y, edge_z0(), *ident]),
             "rule": np.array([RULE_STAND_XY[0], RULE_STAND_XY[1], TOP + 0.010 + RULE[2] / 2, *ident])}  # on the slot floor
    poses.update(crank_poses("x", CROSS_AXIS, CROSS_HUB_Z, START_DEG["x"]))
    poses.update(crank_poses("z", HEIGHT_AXIS, HEIGHT_HUB_Z, START_DEG["z"]))
    return poses


def x_target(cross_turns_cw: float) -> float:
    """Cross-slide joint target (m from the start) for the cross-feed crank's clockwise turns."""
    limit = (BAR_X - BAR_R - RULE[0] + PINCH) - TIP_X0  # past the rule's face by PINCH; no further
    return float(min(limit, max(X_TRAVEL[0], FEED_PER_TURN * cross_turns_cw)))


def z_target(height_turns_cw: float) -> float:
    """Tool-height joint target (m from the start) for the height crank's clockwise turns (lowering)."""
    return float(np.clip(-PITCH * height_turns_cw, Z_TRAVEL[0] + 1e-4, Z_TRAVEL[1] - 1e-4))


def height_error(tool_pose) -> float:
    """Signed cutting-edge height above the spindle centre (m), from the tool's measured pose."""
    return float(tool_pose[2] - (TOP + HC))


def turns_to_centre() -> float:
    """Clockwise height-crank turns that bring the edge from its start to centre height."""
    return E0 / PITCH


def check_layout(p: dict[str, np.ndarray]) -> dict[str, float]:
    """Joint-anchor mismatches (m): all ~0 when the joints hold."""
    out = {}
    for pre, axis, hub in (("x", CROSS_AXIS, CROSS_HUB_Z), ("z", HEIGHT_AXIS, HEIGHT_HUB_Z)):
        c = p[f"{pre}crank"][:3]
        out[f"{pre}crank_axis_m"] = float(np.linalg.norm(c - np.array([axis[0], axis[1], TOP + hub])))
        yaw = yaw_of(p[f"{pre}crank"][3:])
        want = c + np.array([CRANK_ARM_R * math.cos(yaw), CRANK_ARM_R * math.sin(yaw), KNOB_DZ])
        out[f"{pre}knob_off_arm_m"] = float(np.linalg.norm(p[f"{pre}knob"][:3] - want))
    car, tool = p["carriage"][:3], p["tool"][:3]
    out["carriage_off_line_m"] = float(np.hypot(car[1] - CONTACT_Y, car[2] - edge_z0()))
    out["tool_off_carriage_m"] = float(np.hypot(tool[0] - car[0], tool[1] - car[1]))
    return out
