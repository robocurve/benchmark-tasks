"""Geometry, masses and kinematics of the piston-stop engine trainer. NumPy only (no Isaac).

Units: metres, kilograms, radians unless a name ends in _DEG; quaternions xyzw. Station frame: x forward from
the line between the arm bases, y left, z up; TOP is the table top.

The trainer is a single-cylinder engine with a vertical crankshaft and a horizontal cylinder, as in small
vertical-shaft engines. The crank axis is vertical at CRANK_XY. The cylinder axis runs from the crank axis
toward -y (direction U). Inside an opaque housing: the crank throw (radius R), the connecting rod (length L) and
the piston. Above the housing lid: the degree wheel and the crank handle, both fixed to the crankshaft.

Angles:
    rotor_yaw    rotor yaw about +z (world), the crank throw's direction (rotor local +x).
    theta  crank angle from top dead centre (TDC): theta = rotor_yaw - PHI_TDC. TDC (theta = 0) is the throw pointing
           along U, the piston at its farthest point toward the cylinder head.
The removable stop sits in the bore near the head, so the piston meets it at theta = +-CONTACT_DEG and the crank
cannot pass through TDC while it is in.
"""

from __future__ import annotations

import math

import numpy as np

TOP = 0.7475  # station table top (yam_arena.task_file.TABLE_TOP["station"])

# -- engine layout ---------------------------------------------------------------------
CRANK_XY = np.array([0.25, 0.14])  # crank axis, in front of the left arm
U = np.array([0.0, -1.0])  # cylinder axis direction (from the crank toward the head), horizontal
PHI_TDC = math.atan2(U[1], U[0])  # -90 degrees
Z_MECH = 0.035  # height of the crank, rod and piston plane above the table

R = 0.030  # crank throw (60 mm stroke)
L = 0.095  # connecting rod, centre to centre
CROWN = 0.020  # wrist pin to piston crown face
SKIRT = 0.010  # wrist pin to piston skirt
PISTON = (0.040, CROWN + SKIRT, 0.030)  # x width, length along U, height (hidden box)

# -- stop and cylinder head -------------------------------------------------------------
STOP_T = 0.010  # stop shank thickness along U
STOP_W = 0.036  # shank width (across the bore, x)
STOP_GAP = 0.0005  # shank to head wall, seated
NOMINAL_CONTACT_DEG = 30.0  # where the piston meets the stop (geometry); see CONTACT_DEG for the measured value

# -- housing (opaque) --------------------------------------------------------------------
WALL = 0.006
PLATE_T = 0.006
LID_Z = (0.066, 0.072)  # lid bottom and top above the table
HOUSE_HALF_X = 0.056
HOUSE_BACK = 0.056  # housing end behind the crank axis (opposite U)

# -- above the lid: degree wheel, handle, pointer ---------------------------------------------
WHEEL_R = 0.085
WHEEL_Z = (0.074, 0.078)
TICK_EVERY_DEG = 10
WHEEL_ZERO_LOCAL_DEG = 113.0  # zero tick in the rotor frame: it sits 23 degrees from the pointer at TDC
POINTER_WORLD_DEG = 0.0  # the fixed pointer, outside the wheel rim, in this world direction from the axis
ARM_Z = (0.080, 0.086)  # handle arm (crank web above the wheel)
HANDLE_RADIUS = 0.070  # crank axis to handle sleeve axis
HANDLE_OFFSET_DEG = 37.0  # handle direction minus crank-throw direction (rotor frame)
SLEEVE_R, SLEEVE_H = 0.010, 0.040
SLEEVE_Z0 = 0.088  # sleeve bottom above the table

# -- stop body (T-shaped: shank in the slot, plate on the lid, grip block on top) ----------------
STOP_PLATE = (0.050, 0.030, 0.008)
STOP_GRIP = (0.020, 0.030, 0.035)  # x (fingers close along x), along U, height
STOP_SHANK_Z = (0.012, LID_Z[1])  # bottom and top of the shank above the table
STOP_MASS = 0.08

# -- tray (task file fixture) ---------------------------------------------------------------
TRAY_XY = np.array([0.21, -0.17])
TRAY_INNER = (0.16, 0.14)  # x, y
TRAY_RIM = (0.008, 0.012)  # wall thickness, height above the floor
TRAY_FLOOR = 0.004

# -- masses and friction ----------------------------------------------------------------------
MASS = {"rotor": 0.50, "rod": 0.06, "piston": 0.10, "handle": 0.02}
INERTIA = {  # principal moments about each body's centre of mass (kg m^2), declared
    "rotor": (6.0e-4, 6.0e-4, 1.2e-3),
    "rod": (2.0e-6, 5.0e-5, 5.0e-5),
    "piston": (1.5e-5, 1.5e-5, 1.5e-5),
    "handle": (3.0e-6, 3.0e-6, 1.0e-6),
}
FRICTION = {"frame": (0.6, 0.5), "rotor": (0.6, 0.5), "handle": (1.0, 0.9), "piston": (0.3, 0.25),
            "stop": (0.6, 0.5)}
# Bearing and ring drag on the crankshaft: a declared joint damping, N m per (deg/s). 0.02 N m s/rad.
CRANK_DAMPING = 0.02 * math.pi / 180.0

# -- start --------------------------------------------------------------------------------------------
THETA_START_DEG = 75.0  # crank 75 degrees past TDC: piston mid-stroke, clear of the stop


def piston_s(theta: float) -> float:
    """Wrist-pin distance from the crank axis along U for crank angle theta (rad)."""
    return R * math.cos(theta) + math.sqrt(L * L - (R * math.sin(theta)) ** 2)


def ds_dtheta(theta: float) -> float:
    return -R * math.sin(theta) * (1.0 + R * math.cos(theta) / math.sqrt(L * L - (R * math.sin(theta)) ** 2))


STOP_FACE_S = piston_s(math.radians(NOMINAL_CONTACT_DEG)) + CROWN  # piston-side face of the seated stop
STOP_S = STOP_FACE_S + STOP_T / 2  # seated shank centre
HEAD_S = STOP_FACE_S + STOP_T + STOP_GAP  # head wall face
HEAD_T = 0.010
HOUSE_FRONT = HEAD_S + HEAD_T  # housing end at the head
SLOT_CLEAR = 0.001  # slot opening beyond the shank, piston side and across

# Measured contact angle: |theta| where the crank rests against the seated stop. Probe run 2026-11-05 (crank driven by
# a 0.3 N m velocity drive, arms idle; telemetry runs/probe): rests at +-28.467 deg with a piston-stop gap of
# 0.00 mm, symmetric to 0.001 deg. Smaller than NOMINAL_CONTACT_DEG because the stop first slides STOP_GAP back
# to the head wall (0.5 mm / 0.334 mm per deg = 1.5 deg).
CONTACT_DEG = 28.47
SIDE_TOL_DEG = 1.0  # a side counts when the crank is within this of the contact angle, stop seated
FORCED_DEG = 3.0  # the crank this far past the contact angle with the stop seated: forced (bent rod)
TDC_TOL_DEG = 2.0


def along(s: float, z: float = Z_MECH) -> np.ndarray:
    """World point at distance s from the crank axis along U, at height z above the table."""
    xy = CRANK_XY + s * U
    return np.array([xy[0], xy[1], TOP + z])


def axis_point(z: float = Z_MECH) -> np.ndarray:
    return np.array([CRANK_XY[0], CRANK_XY[1], TOP + z])


def yaw_quat(a: float) -> np.ndarray:
    return np.array([0.0, 0.0, math.sin(a / 2), math.cos(a / 2)])


def yaw_of(q) -> float:
    x, y, z, w = (float(v) for v in q)
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def tilt_of(q) -> float:
    """Angle (rad) between the body's +z and world +z."""
    x, y, z, w = (float(v) for v in q)
    zz = 1 - 2 * (x * x + y * y)
    return math.acos(max(-1.0, min(1.0, zz)))


def wrap_deg(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def theta_of_phi(rotor_yaw: float) -> float:
    return math.radians(wrap_deg(math.degrees(rotor_yaw - PHI_TDC)))


def handle_center(theta: float) -> np.ndarray:
    """World centre of the handle sleeve at crank angle theta (rad)."""
    a = PHI_TDC + theta + math.radians(HANDLE_OFFSET_DEG)
    xy = CRANK_XY + HANDLE_RADIUS * np.array([math.cos(a), math.sin(a)])
    return np.array([xy[0], xy[1], TOP + SLEEVE_Z0 + SLEEVE_H / 2])


def sleeve_local() -> np.ndarray:
    """Sleeve centre in the rotor frame (origin on the axis at Z_MECH)."""
    a = math.radians(HANDLE_OFFSET_DEG)
    return np.array([HANDLE_RADIUS * math.cos(a), HANDLE_RADIUS * math.sin(a), SLEEVE_Z0 + SLEEVE_H / 2 - Z_MECH])


def stop_seated_pos() -> np.ndarray:
    """Body origin of the seated stop: the shank centre."""
    return along(STOP_S, (STOP_SHANK_Z[0] + STOP_SHANK_Z[1]) / 2)


def chain(theta: float) -> dict[str, np.ndarray]:
    """Poses (xyz + quat xyzw) of the moving parts at crank angle theta (rad), all consistent with the joints."""
    rotor_yaw = PHI_TDC + theta
    pin = axis_point() + R * np.array([math.cos(rotor_yaw), math.sin(rotor_yaw), 0.0])
    wrist = along(piston_s(theta))
    d = wrist - pin
    psi = math.atan2(d[1], d[0])
    q_rotor = yaw_quat(rotor_yaw)
    sleeve_rot = handle_center(theta)
    return {
        "rotor": np.array([*axis_point(), *q_rotor]),
        "rod": np.array([*pin, *yaw_quat(psi)]),
        "piston": np.array([*wrist, 0.0, 0.0, 0.0, 1.0]),
        "handle": np.array([*sleeve_rot, *q_rotor]),
    }


def start_poses() -> dict[str, np.ndarray]:
    poses = chain(math.radians(THETA_START_DEG))
    poses["frame"] = np.array([CRANK_XY[0], CRANK_XY[1], TOP, 0.0, 0.0, 0.0, 1.0])
    poses["stop"] = np.array([*stop_seated_pos(), 0.0, 0.0, 0.0, 1.0])
    return poses


def tray_inner_bounds() -> tuple[np.ndarray, np.ndarray]:
    half = np.array(TRAY_INNER) / 2
    return TRAY_XY - half, TRAY_XY + half


def check_layout(poses: dict[str, np.ndarray] | None = None) -> dict[str, float]:
    """Joint-anchor mismatches (m) of a layout: all should be ~0 for a consistent start."""
    p = start_poses() if poses is None else poses
    rot = p["rotor"]
    rotor_yaw = yaw_of(rot[3:])
    pin = rot[:3] + R * np.array([math.cos(rotor_yaw), math.sin(rotor_yaw), 0.0])
    rod = p["rod"]
    psi = yaw_of(rod[3:])
    small = rod[:3] + L * np.array([math.cos(psi), math.sin(psi), 0.0])
    sleeve = rot[:3] + np.array([[math.cos(rotor_yaw), -math.sin(rotor_yaw), 0], [math.sin(rotor_yaw), math.cos(rotor_yaw), 0], [0, 0, 1]]) \
        @ sleeve_local()
    piston = p["piston"][:3]
    off_axis = np.cross(np.r_[U, 0.0], piston - axis_point())[2]
    return {"crank_pin_m": float(np.linalg.norm(pin - rod[:3])), "wrist_pin_m": float(np.linalg.norm(small - piston)),
            "sleeve_m": float(np.linalg.norm(sleeve - p["handle"][:3])), "piston_off_axis_m": abs(float(off_axis)),
            "rotor_off_axis_m": float(np.linalg.norm(rot[:3] - axis_point()))}
