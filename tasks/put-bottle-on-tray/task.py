"""On the station table, on the robot's left side: two cylindrical chemical bottles standing upright—a red
bleach bottle (5.6 cm diameter, 11 cm tall) and a green ammonia cleaner bottle (5.6 cm diameter, 11 cm tall).
Further back on the table sit two separate cabinet spill trays—one red and one green (each 12 cm deep,
12 cm wide, and 1.5 cm thick), spaced 4 cm apart so the two chemicals are stored separately. The bottles are
loose; nothing else is on the table.

Task: Using the left arm, pick up the red bleach bottle and the green ammonia cleaner bottle and place each
one upright in the cabinet spill tray of the same color.
"""

import numpy as np

from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN, Waypoint
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]  # z of table upper surface

# World frame: x away from the arm bases, y to the robot's left, z up (m).
RED = (0.82, 0.16, 0.16)
GREEN = (0.18, 0.65, 0.28)

BOTTLE_R = 0.028  # 56 mm diameter (fits comfortably inside the parallel-jaw gripper)
BOTTLE_H = 0.110  # 110 mm tall
BOTTLE_MASS = 0.15

BLEACH_START_XY = (0.18, 0.22)
AMMONIA_START_XY = (0.18, 0.08)

BOTTLES = [
    Cylinder(
        "bleach",
        pos=(*BLEACH_START_XY, TOP + BOTTLE_H / 2),
        radius=BOTTLE_R,
        height=BOTTLE_H,
        mass=BOTTLE_MASS,
        color=RED,
    ),
    Cylinder(
        "ammonia",
        pos=(*AMMONIA_START_XY, TOP + BOTTLE_H / 2),
        radius=BOTTLE_R,
        height=BOTTLE_H,
        mass=BOTTLE_MASS,
        color=GREEN,
    ),
]

# Two color-coded cabinet spill trays positioned beyond the bottles in the left arm's workspace
TRAY_D = 0.120  # x depth (12 cm)
TRAY_W = 0.120  # y width (12 cm)
TRAY_H = 0.015  # z thickness (1.5 cm)
TRAY_TOP = TOP + TRAY_H

RED_TRAY_XY = (0.35, 0.23)    # outer-left red spill tray (for bleach)
GREEN_TRAY_XY = (0.35, 0.07)  # inner-left green spill tray (for ammonia; 4 cm gap between trays)

TRAYS = [
    Box("red_spill_tray", pos=(*RED_TRAY_XY, TOP + TRAY_H / 2), size=(TRAY_D, TRAY_W, TRAY_H), color=RED),
    Box("green_spill_tray", pos=(*GREEN_TRAY_XY, TOP + TRAY_H / 2), size=(TRAY_D, TRAY_W, TRAY_H), color=GREEN),
]

TRAY_CENTERS = {"red_spill_tray": RED_TRAY_XY, "green_spill_tray": GREEN_TRAY_XY}
TARGET_TRAY = {"bleach": "red_spill_tray", "ammonia": "green_spill_tray"}
OBJECTS = [*BOTTLES, *TRAYS]

ON_TOL = 0.005  # resting on tray top within 5 mm
UPRIGHT_DEG = 10.0  # bottle upright: vertical axis within 10 deg of world z
EPS = 1e-6


def _rot(st) -> np.ndarray:
    x, y, z, w = np.asarray(st.quat_xyzw, float) / np.linalg.norm(st.quat_xyzw)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _tilt_deg(st) -> float:
    """Angle between the bottle's vertical axis and world up (0 deg = upright, 180 deg = upside down)."""
    return float(np.degrees(np.arccos(np.clip(_rot(st)[2, 2], -1.0, 1.0))))


def _bottle_bottom_z(st) -> float:
    return float(st.pos[2]) - (BOTTLE_H / 2) * float(_rot(st)[2, 2])


def _which_tray(st) -> str | None:
    """Return the name of the spill tray the bottle is resting upright inside, or None."""
    if not st.resting() or _tilt_deg(st) > UPRIGHT_DEG + EPS:
        return None
    if abs(_bottle_bottom_z(st) - TRAY_TOP) > ON_TOL + EPS:
        return None
    bx, by = float(st.pos[0]), float(st.pos[1])
    hx, hy = TRAY_D / 2 - BOTTLE_R, TRAY_W / 2 - BOTTLE_R
    for name, (tx, ty) in TRAY_CENTERS.items():
        if abs(bx - tx) <= hx + EPS and abs(by - ty) <= hy + EPS:
            return name
    return None


def success(objects) -> bool:
    """Both bleach and ammonia are upright, at rest, and placed in their matching colored spill trays."""
    return (
        _which_tray(objects["bleach"]) == TARGET_TRAY["bleach"]
        and _which_tray(objects["ammonia"]) == TARGET_TRAY["ammonia"]
    )


def distance(objects) -> float:
    """Sum of horizontal distances from each bottle to its matching colored spill tray."""
    b_xy = np.asarray(objects["bleach"].pos[:2], float)
    a_xy = np.asarray(objects["ammonia"].pos[:2], float)
    d_bleach = np.linalg.norm(b_xy - np.asarray(RED_TRAY_XY, float))
    d_ammonia = np.linalg.norm(a_xy - np.asarray(GREEN_TRAY_XY, float))
    return float(d_bleach + d_ammonia)


def score(objects) -> float:
    """1.0: both bottles upright in their matching colored spill trays.
    0.5: exactly one bottle upright in its matching colored spill tray (and not sharing a tray with the other).
    0.0: neither bottle in its matching tray, or both placed in the same tray (unsafe mixing).
    """
    t_bleach = _which_tray(objects["bleach"])
    t_ammonia = _which_tray(objects["ammonia"])
    if t_bleach is not None and t_bleach == t_ammonia:
        return 0.0
    matched = int(t_bleach == TARGET_TRAY["bleach"]) + int(t_ammonia == TARGET_TRAY["ammonia"])
    return 0.5 * matched


# ---- scripted demo ------------------------------------------------------------------------------------------
# Single-arm (left) pick-and-place: grasp the red bleach cylinder near its midpoint, lift it, place it upright
# in red_spill_tray, release, then repeat for the green ammonia cylinder into green_spill_tray.
TIP = 0.013  # finger tips below the TCP
GRASP_Z = TOP + BOTTLE_H * 0.55 + TIP  # grasp slightly above bottle midpoint
PLACE_Z = TRAY_TOP + BOTTLE_H * 0.55 + TIP + 0.004  # 4 mm clearance above tray before opening
TRAVEL_Z = TOP + BOTTLE_H + 0.12  # safe clearance above standing bottles
LEFT_YAW = 0.0


def _pick_and_place(src_xy, dst_xy, name, tray_name):
    sx, sy = src_xy
    dx, dy = dst_xy

    def wp(t, x, y, z, finger, label):
        return Waypoint(t, (float(x), float(y), float(z)), finger, label, LEFT_YAW)

    return [
        wp(1.5, sx, sy, TRAVEL_Z, FINGER_OPEN, f"move above {name}"),
        wp(1.2, sx, sy, GRASP_Z, FINGER_OPEN, f"lower onto {name}"),
        wp(0.6, sx, sy, GRASP_Z, FINGER_CLOSED, f"grasp {name}"),
        wp(1.2, sx, sy, TRAVEL_Z, FINGER_CLOSED, f"lift {name}"),
        wp(1.8, dx, dy, TRAVEL_Z, FINGER_CLOSED, f"carry {name} over {tray_name}"),
        wp(1.2, dx, dy, PLACE_Z, FINGER_CLOSED, f"lower {name} onto {tray_name}"),
        wp(0.5, dx, dy, PLACE_Z, FINGER_OPEN, f"release {name}"),
        wp(1.0, dx, dy, TRAVEL_Z, FINGER_OPEN, f"retract above {name}"),
    ]


def demo():
    return {
        "left": [
            *_pick_and_place(BLEACH_START_XY, RED_TRAY_XY, "bleach", "red_spill_tray"),
            *_pick_and_place(AMMONIA_START_XY, GREEN_TRAY_XY, "ammonia", "green_spill_tray"),
        ]
    }