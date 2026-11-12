from __future__ import annotations

"""Demo plan for keyboard_tray_latch (T22)."""

import os
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TABLE_HEIGHT
from yam_arena.demo import Waypoint

TABLE_TOP = TABLE_HEIGHT
SCALE = float(os.getenv("DEMO_TIME_SCALE", "1.0"))


def _s(t: float) -> float:
    return max(0.1, t * SCALE)


def make_settle_plan():
    return {
        "left": [Waypoint(_s(8.0), (0.20, 0.15, TABLE_TOP + 0.15), FINGER_OPEN, "l_settle")],
        "right": [Waypoint(_s(8.0), (0.20, -0.15, TABLE_TOP + 0.15), FINGER_OPEN, "r_settle")],
    }


def make_demo_plan(ablation: bool = False):
    # Left arm braces desk front edge at y = +0.15 m
    # Right arm pushes keyboard tray at y = -0.10 m
    # TCP distance = 250 mm (clearance 165 mm >= 30 mm)
    if ablation:
        # Left arm idle -> desk shifts when Right pushes -> drift gate failure
        left = [Waypoint(_s(15.0), (0.20, 0.25, TABLE_TOP + 0.15), FINGER_OPEN, "l_idle")]
        right = [
            Waypoint(_s(2.0), (0.16, -0.10, TABLE_TOP + 0.05), FINGER_CLOSED, "r_hover"),
            Waypoint(_s(1.5), (0.16, -0.10, TABLE_TOP + 0.02), FINGER_CLOSED, "r_push_unbraced"),
            Waypoint(_s(11.5), (0.30, -0.10, TABLE_TOP + 0.02), FINGER_CLOSED, "r_hold"),
        ]
        return {"left": left, "right": right}

    left = [
        # Approach & brace desk edge at (0.20, 0.15, TABLE_TOP + 0.05)
        Waypoint(_s(2.0), (0.20, 0.15, TABLE_TOP + 0.14), FINGER_OPEN, "l_hover_brace"),
        Waypoint(_s(1.5), (0.20, 0.15, TABLE_TOP + 0.05), FINGER_OPEN, "l_descend_brace"),
        Waypoint(_s(0.8), (0.20, 0.15, TABLE_TOP + 0.05), FINGER_CLOSED, "l_clamp_brace"),
        # Hold firm for 14 s while Right arm drives tray in
        Waypoint(_s(14.0), (0.20, 0.15, TABLE_TOP + 0.05), FINGER_CLOSED, "l_hold_firm"),
        Waypoint(_s(0.8), (0.20, 0.15, TABLE_TOP + 0.05), FINGER_OPEN, "l_release"),
        Waypoint(_s(1.5), (0.20, 0.15, TABLE_TOP + 0.14), FINGER_OPEN, "l_lift_clear"),
        Waypoint(_s(2.0), (0.20, 0.15, TABLE_TOP + 0.15), FINGER_OPEN, "l_standby"),
        Waypoint(_s(4.0), (0.20, 0.15, TABLE_TOP + 0.15), FINGER_OPEN, "l_settle"),
    ]

    right = [
        # Hover & approach tray handle at (0.04, -0.22, TABLE_TOP + 0.02)
        Waypoint(_s(2.0), (0.04, -0.22, TABLE_TOP + 0.12), FINGER_CLOSED, "r_hover_tray", yaw=1.5708),
        Waypoint(_s(1.5), (0.04, -0.22, TABLE_TOP + 0.02), FINGER_CLOSED, "r_descend_tray", yaw=1.5708),
        # Push tray along +X by 28 cm into latch at x = 0.44
        Waypoint(_s(4.0), (0.44, -0.22, TABLE_TOP + 0.02), FINGER_CLOSED, "r_push_latch", yaw=1.5708),
        # Dwell 2.0 s to let latch engage
        Waypoint(_s(2.0), (0.44, -0.22, TABLE_TOP + 0.02), FINGER_CLOSED, "r_dwell_latch", yaw=1.5708),
        Waypoint(_s(1.5), (0.35, -0.22, TABLE_TOP + 0.10), FINGER_OPEN, "r_lift_clear", yaw=0.0),
        Waypoint(_s(2.0), (0.20, -0.22, TABLE_TOP + 0.15), FINGER_OPEN, "r_standby", yaw=0.0),
        Waypoint(_s(13.6), (0.20, -0.22, TABLE_TOP + 0.15), FINGER_OPEN, "r_settle", yaw=0.0),
    ]
    return {"left": left, "right": right}


def plans():
    if os.getenv("SETTLE_ONLY"):
        return make_settle_plan()
    if os.getenv("ABLATION"):
        return make_demo_plan(ablation=True)
    return make_demo_plan(ablation=False)
