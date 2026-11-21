"""Task: Reassemble commercial manual citrus juicer.

Occupational Relevance:
    Food Preparation Workers (SOC 35-2021, >1.02M US workforce) and Fast Food / Counter
    Workers (SOC 35-3023, >3.54M US workforce) operate and sanitize commercial manual
    citrus juice presses (e.g. Hamilton Beach Commercial 932) in restaurant kitchens,
    juice bars, and cocktail lounges. Under FDA Food Code §4-602.11 sanitation standards,
    the juice extractor must be disassembled, washed, and reassembled between shifts.

Reassembly procedure:
    1. Right arm picks up the food-grade stainless juice collector bowl from its staging
       pad, aligns it into the base receiver collar, lowers it into place, and rotates
       it 30 degrees clockwise to engage the bayonet tabs into internal collar detents.
    2. Left arm picks up the perforated stainless strainer cone insert by its central
       reamer knob, aligns it coaxially above the seated collector bowl, and lowers it
       flush onto the internal support shelf.
    3. Left arm moves to the press lever arm and depresses it through a 15-degree mechanical
       test stroke to verify that the upper compressor plunger clears the seated strainer cone
       without collision, then releases the lever to return to its rest stop.
    4. Both arms retract clear (>= 25 mm margin) and hold stationary for 1.0 second.
"""

from __future__ import annotations

import math
import os
from pathlib import Path
import sys
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parent))

import numpy as np
from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN, Waypoint
from yam_arena.task_file import ObjectState, TABLE_TOP

import mechkit as mk

TAG = "reassemble_commercial_citrus_juicer"
SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0

kit = mk.Kit()

TOP = TABLE_TOP[SCENE]

INSTRUCTION = (
    "Reassemble the commercial citrus juicer: seat the juice collector bowl"
    " into the base collar and rotate it to lock, drop the strainer cone"
    " flush inside, swing the press lever through a test stroke, and clear"
    " both arms."
)

# Colors
CAST_BLACK = (0.16, 0.17, 0.18)
CAST_IRON = (0.24, 0.25, 0.27)
STAINLESS_BRIGHT = (0.86, 0.88, 0.90)
STAINLESS_MATTE = (0.72, 0.74, 0.76)
RUBBER_BLACK = (0.10, 0.10, 0.10)
CHROME_ACCENT = (0.92, 0.93, 0.95)

# Coordinate Layout
BASE_X = 0.220
BASE_Y = 0.000
BASE_Z = TOP + 0.015

# Collar Center & Aperture (inside juicer base)
COLLAR_X = 0.220
COLLAR_Y = 0.000
COLLAR_Z = TOP + 0.055

# Collector Bowl initial & target poses
BOWL_INIT_POS = np.array([0.220, -0.140, TOP + 0.030])
BOWL_SEAT_Z = TOP + 0.050

# Strainer Cone initial & target poses
CONE_INIT_POS = np.array([0.320, -0.140, TOP + 0.024])
CONE_SEAT_Z = TOP + 0.053

# Lever Hinge Anchor & Parameters
LEVER_ANCHOR = np.array([BASE_X - 0.080, 0.000, TOP + 0.165])
LEVER_STROKE_THRESHOLD_DEG = 35.0


# -- 1. Cast Iron Base Assembly (bench-mounted) --------------------------------
kit.part(
    name="juicer_base",
    pos=(BASE_X, BASE_Y, BASE_Z),
    boxes=[
        # Heavy horseshoe footplate
        mk.box((0.200, 0.160, 0.025), at=(-0.020, 0.000, 0.000), color=CAST_BLACK),
        # Rubber non-skid suction pads
        mk.box((0.040, 0.040, 0.008), at=(-0.090, 0.060, -0.015), color=RUBBER_BLACK),
        mk.box((0.040, 0.040, 0.008), at=(-0.090, -0.060, -0.015), color=RUBBER_BLACK),
        mk.box((0.040, 0.040, 0.008), at=(0.060, 0.060, -0.015), color=RUBBER_BLACK),
        mk.box((0.040, 0.040, 0.008), at=(0.060, -0.060, -0.015), color=RUBBER_BLACK),
        # Upright column spine at rear
        mk.box((0.050, 0.050, 0.145), at=(-0.080, 0.000, 0.0725), color=CAST_IRON),
        # Top hinge mount bracket
        mk.box((0.070, 0.060, 0.025), at=(-0.080, 0.000, 0.1425), color=CAST_IRON),
        # Receiver collar support bracket connecting rear column to collar rim
        mk.box((0.030, 0.035, 0.020), at=(-0.060, 0.000, 0.040), color=CAST_IRON),
        # Receiver collar perimeter walls (centered at 0, 0 with 115 mm clear opening)
        mk.box((0.015, 0.140, 0.025), at=(0.065, 0.000, 0.040), color=CAST_IRON),
        mk.box((0.015, 0.140, 0.025), at=(-0.065, 0.000, 0.040), color=CAST_IRON),
        mk.box((0.140, 0.015, 0.025), at=(0.000, 0.065, 0.040), color=CAST_IRON),
        # Front collar rim segmented to form an 110 mm clearance bayonet rotation notch at -Y
        mk.box((0.010, 0.015, 0.025), at=(0.060, -0.065, 0.040), color=CAST_IRON),
        mk.box((0.010, 0.015, 0.025), at=(-0.060, -0.065, 0.040), color=CAST_IRON),
        # Internal bayonet detent stops inside collar (positioned at side walls)
        mk.box((0.010, 0.015, 0.010), at=(0.050, -0.020, 0.030), color=CAST_BLACK),
        mk.box((0.010, 0.015, 0.010), at=(-0.050, 0.020, 0.030), color=CAST_BLACK),
    ],
    mass=None,  # Solid bench-mounted unit
)

# -- 2. Sprung Press Lever & Plunger Assembly ----------------------------------
kit.part(
    name="press_lever",
    pos=(LEVER_ANCHOR[0], LEVER_ANCHOR[1], LEVER_ANCHOR[2]),
    boxes=[
        # Main chrome lever arm bar
        mk.box((0.140, 0.025, 0.020), at=(0.070, 0.000, 0.000), color=CHROME_ACCENT),
        # Ergonomic grip knob
        mk.box((0.025, 0.025, 0.025), at=(0.105, 0.000, -0.010), color=CAST_BLACK),
        # Vertical plunger rack link
        mk.box((0.020, 0.020, 0.035), at=(0.080, 0.000, -0.015), color=STAINLESS_MATTE),
        # Upper stainless compressor cone cup (centered directly over collar at X = +0.080 relative to anchor)
        mk.box((0.060, 0.060, 0.025), at=(0.080, 0.000, -0.030), color=STAINLESS_BRIGHT),
    ],
    mass=0.450,
)

# Revolute hinge joint for press lever with spring return to +85 deg rest position
kit.hinge(
    name="lever_pivot",
    parent="juicer_base",
    child="press_lever",
    anchor=tuple(LEVER_ANCHOR),
    axis=(0.0, -1.0, 0.0),
    lower_deg=-5.0,
    upper_deg=95.0,
    start_deg=85.0,
    rest_deg=85.0,
    stiffness=1.2,
    damping=0.3,
)

# -- 3. Stainless Juice Collector Funnel Bowl ----------------------------------
kit.part(
    name="collector_bowl",
    pos=(BOWL_INIT_POS[0], BOWL_INIT_POS[1], BOWL_INIT_POS[2]),
    boxes=[
        # Funnel cup walls (80 mm outer width, 68 mm inner opening, 45 mm high)
        mk.box((0.080, 0.006, 0.045), at=(0.000, 0.037, 0.000), color=STAINLESS_BRIGHT),
        mk.box((0.080, 0.006, 0.045), at=(0.000, -0.037, 0.000), color=STAINLESS_BRIGHT),
        mk.box((0.006, 0.080, 0.045), at=(0.037, 0.000, 0.000), color=STAINLESS_BRIGHT),
        mk.box((0.006, 0.080, 0.045), at=(-0.037, 0.000, 0.000), color=STAINLESS_BRIGHT),
        # Bottom drainage spout nozzle walls (44 mm outer width, 38 mm inner opening, 16 mm high)
        mk.box((0.044, 0.003, 0.016), at=(0.000, 0.0205, -0.020), color=STAINLESS_BRIGHT),
        mk.box((0.044, 0.003, 0.016), at=(0.000, -0.0205, -0.020), color=STAINLESS_BRIGHT),
        mk.box((0.003, 0.044, 0.016), at=(0.0205, 0.000, -0.020), color=STAINLESS_BRIGHT),
        mk.box((0.003, 0.044, 0.016), at=(-0.0205, 0.000, -0.020), color=STAINLESS_BRIGHT),
        # Low-profile connector strut through collar notch
        mk.box((0.012, 0.045, 0.008), at=(0.000, -0.065, 0.012), color=STAINLESS_BRIGHT),
        # Elongated rectangular grip post (28 mm wide x 18 mm thick) for high-torque twist lock
        mk.box((0.028, 0.018, 0.045), at=(0.000, -0.090, 0.035), color=CAST_BLACK),
        # Bayonet locking tabs on rim (positioned to pass detents on drop, latch on 30 deg CW twist)
        mk.box((0.012, 0.010, 0.008), at=(0.046, 0.012, 0.018), color=STAINLESS_BRIGHT),
        mk.box((0.012, 0.010, 0.008), at=(-0.046, -0.012, 0.018), color=STAINLESS_BRIGHT),
        # Annular support shelf ring for strainer cone (54 mm outer lip, 36 mm hollow center opening)
        mk.box((0.054, 0.009, 0.008), at=(0.000, 0.0225, -0.008), color=STAINLESS_BRIGHT),
        mk.box((0.054, 0.009, 0.008), at=(0.000, -0.0225, -0.008), color=STAINLESS_BRIGHT),
        mk.box((0.009, 0.036, 0.008), at=(0.0225, 0.000, -0.008), color=STAINLESS_BRIGHT),
        mk.box((0.009, 0.036, 0.008), at=(-0.0225, 0.000, -0.008), color=STAINLESS_BRIGHT),
    ],
    mass=0.045,
    contact_offset=0.0005,
    solver_position_iterations=128,
)

# -- 4. Stainless Perforated Strainer Cone Insert ------------------------------
kit.part(
    name="strainer_cone",
    pos=(CONE_INIT_POS[0], CONE_INIT_POS[1], CONE_INIT_POS[2]),
    boxes=[
        # Perforated strainer basket (32 mm wide x 10 mm high, sits cleanly inside upper funnel cup)
        mk.box((0.032, 0.032, 0.010), at=(0.000, 0.000, -0.008), color=STAINLESS_MATTE),
        # Outer seating flange lip (48 mm wide, rests flush on 54 mm internal shelf)
        mk.box((0.048, 0.048, 0.008), at=(0.000, 0.000, 0.000), color=STAINLESS_MATTE),
        # Central grasp handle knob post (28 mm wide x 18 mm thick x 52 mm high) in 100% open clear air
        mk.box((0.028, 0.018, 0.052), at=(0.000, 0.000, 0.030), color=STAINLESS_BRIGHT),
    ],
    mass=0.035,
    contact_offset=0.0005,
    solver_position_iterations=128,
)

OBJECTS = kit.objects()


# -- Scorer Implementation -----------------------------------------------------

def _on_step(objects: dict[str, Any]) -> None:
    M = mk.MEMORY
    step_idx = mk._EP.get("step", 0)

    bp, bq = kit.pose(objects, "collector_bowl")
    cp, cq = kit.pose(objects, "strainer_cone")
    lever_deg = kit.q_deg(objects, "lever_pivot")

    # 1. Collector bowl placed in collar
    dr_bowl = math.sqrt((bp[0] - COLLAR_X) ** 2 + (bp[1] - COLLAR_Y) ** 2)
    dz_bowl = abs(bp[2] - BOWL_SEAT_Z)
    bowl_placed = dr_bowl <= 0.025 and dz_bowl <= 0.020
    if bowl_placed:
        M["bowl_placed"] = True

    # Check bowl bayonet twist angle (strictly clockwise locking rotation between -15° and -35°)
    b_mat = mk.quat_to_mat(bq)
    bowl_yaw_deg = math.degrees(math.atan2(b_mat[1, 0], b_mat[0, 0]))
    bowl_locked = bool(
        M.get("bowl_placed")
        and dr_bowl <= 0.020
        and dz_bowl <= 0.015
        and -35.0 <= bowl_yaw_deg <= -15.0
    )
    if bowl_locked:
        M["bowl_locked"] = True

    # 2. Strainer cone nested coaxially in collector bowl
    dr_cone = math.sqrt((cp[0] - bp[0]) ** 2 + (cp[1] - bp[1]) ** 2)
    dz_cone = abs(cp[2] - (bp[2] + 0.003))
    cone_seated = bool(
        M.get("bowl_locked")
        and dr_cone <= 0.020
        and dz_cone <= 0.022
    )
    if cone_seated:
        M["cone_seated"] = True

    # 3. Press lever mechanical test stroke
    min_lever = min(M.get("min_lever_deg", 85.0), lever_deg)
    M["min_lever_deg"] = min_lever
    lever_stroked = bool(
        M.get("cone_seated")
        and min_lever <= LEVER_STROKE_THRESHOLD_DEG
        and lever_deg >= 55.0  # Returned to top rest position (rest ~ 85 deg)
    )
    if lever_stroked:
        M["lever_stroked"] = True

    # 4. Robot hands cleared
    hands_clear = bool(
        _released(objects["collector_bowl"], 0.025)
        and _released(objects["strainer_cone"], 0.025)
        and _released(objects["press_lever"], 0.025)
    )

    # 5. Success hold timer
    if lever_stroked and hands_clear and not failed(objects):
        hold = M.get("hold_timer", 0) + 1
        M["hold_timer"] = hold
    else:
        M["hold_timer"] = 0

    if step_idx % 40 == 0:
        print(
            f"[{TAG}] step={step_idx} bp=({bp[0]:.3f},{bp[1]:.3f},{bp[2]:.3f}) dr_b={dr_bowl:.3f} dz_b={dz_bowl:.3f} yaw_b={bowl_yaw_deg:.1f} "
            f"dr_c={dr_cone:.3f} dz_c={dz_cone:.3f} lever={lever_deg:.1f} min_l={min_lever:.1f} "
            f"placed={M.get('bowl_placed')} locked={M.get('bowl_locked')} seated={M.get('cone_seated')} "
            f"stroke={M.get('lever_stroked')} clear={hands_clear} hold={M.get('hold_timer',0)} "
            f"score={score(objects)}",
            flush=True,
        )


kit.on_step(_on_step)


# -- Scorer Helpers ------------------------------------------------------------

def _released(obj_state: ObjectState, margin: float = 0.025) -> bool:
    """True if no robot gripper or finger is within margin of the object."""
    tcps = mk.tcps()
    if not tcps:
        return True
    pos = np.asarray(obj_state.pos, dtype=float)
    return all(np.linalg.norm(np.asarray(tcp, dtype=float) - pos) >= margin for tcp in tcps)


def bowl_placed(objects: dict) -> bool:
    """True if collector bowl is currently seated in receiver collar."""
    bp, _ = kit.pose(objects, "collector_bowl")
    dr = math.sqrt((bp[0] - COLLAR_X) ** 2 + (bp[1] - COLLAR_Y) ** 2)
    dz = abs(bp[2] - BOWL_SEAT_Z)
    return bool(dr <= 0.025 and dz <= 0.020)


def bowl_locked(objects: dict) -> bool:
    """True if collector bowl is currently seated and bayonet locked into detents."""
    if not bowl_placed(objects):
        return False
    bp, bq = kit.pose(objects, "collector_bowl")
    dr = math.sqrt((bp[0] - COLLAR_X) ** 2 + (bp[1] - COLLAR_Y) ** 2)
    dz = abs(bp[2] - BOWL_SEAT_Z)
    b_mat = mk.quat_to_mat(bq)
    yaw_deg = math.degrees(math.atan2(b_mat[1, 0], b_mat[0, 0]))
    return bool(
        dr <= 0.020
        and dz <= 0.015
        and -35.0 <= yaw_deg <= -15.0
    )


def cone_seated(objects: dict) -> bool:
    """True if strainer cone is currently coaxially nested flush inside collector bowl."""
    if not bowl_locked(objects):
        return False
    bp, _ = kit.pose(objects, "collector_bowl")
    cp, _ = kit.pose(objects, "strainer_cone")
    dr = math.sqrt((cp[0] - bp[0]) ** 2 + (cp[1] - bp[1]) ** 2)
    dz = abs(cp[2] - (bp[2] + 0.003))
    return bool(dr <= 0.020 and dz <= 0.022)


def lever_stroked(objects: dict) -> bool:
    """True if press lever completed stroke and is currently returned to top rest."""
    if not cone_seated(objects):
        return False
    min_l = mk.MEMORY.get("min_lever_deg", 85.0)
    cur_l = kit.q_deg(objects, "lever_pivot")
    return bool(min_l <= LEVER_STROKE_THRESHOLD_DEG and cur_l >= 55.0)


def failed(objects: dict) -> bool:
    """True if base tipped, parts dropped off table, or components displaced."""
    base_p, base_q = kit.pose(objects, "juicer_base")
    if abs(base_p[0] - BASE_X) > 0.025 or abs(base_p[1] - BASE_Y) > 0.025:
        return True
    b_mat = mk.quat_to_mat(base_q)
    if b_mat[2, 2] < 0.95:  # Base tipped > 18 deg
        return True

    for name in ("collector_bowl", "strainer_cone"):
        p, _ = kit.pose(objects, name)
        if p[2] < TOP - 0.030:  # Dropped below table edge
            return True
        if math.sqrt(p[0] ** 2 + p[1] ** 2) > 0.65:
            return True
    return False


def score(objects: dict) -> float:
    """Staged scoring with partial credit: 0.25 -> 0.50 -> 0.75 -> 1.00."""
    if failed(objects):
        return 0.0

    if not bowl_placed(objects):
        return 0.0

    if not bowl_locked(objects):
        return 0.25

    if not cone_seated(objects):
        return 0.50

    if not lever_stroked(objects):
        return 0.75

    hands_clear = (
        _released(objects["collector_bowl"], 0.025)
        and _released(objects["strainer_cone"], 0.025)
        and _released(objects["press_lever"], 0.025)
    )
    hold = mk.MEMORY.get("hold_timer", 0)

    if hands_clear and hold >= 20:
        return 1.00

    return 0.75


def success(objects: dict) -> bool:
    return score(objects) >= 1.0


# Exported constants
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0


# -- Scripted Demo Waypoints ---------------------------------------------------

def demo() -> dict[str, list[Waypoint]]:
    cheat = os.environ.get("JUICER_CHEAT", "")

    if cheat == "nothing":
        left = [Waypoint(10.0, (0.180, 0.200, TOP + 0.140), FINGER_OPEN, "park_left")]
        right = [Waypoint(10.0, (0.180, -0.200, TOP + 0.140), FINGER_OPEN, "park_right")]
        return {"left": left, "right": right}

    # Bowl handle coordinates (at -Y rim of the bowl, aligned with gripper fingers)
    h_x = BOWL_INIT_POS[0]
    h_y = BOWL_INIT_POS[1] - 0.090
    h_z = BOWL_INIT_POS[2] + 0.035
    BOWL_CARRY_Z = TOP + 0.150

    # Cone knob coordinates (elevated apex knob stem for 100% collision-free finger clamp)
    c_x = CONE_INIT_POS[0]
    c_y = CONE_INIT_POS[1]
    c_z = CONE_INIT_POS[2] + 0.045
    CONE_CARRY_Z = TOP + 0.150

    # Right arm: installs collector bowl, locks bayonet detents, then seats strainer cone
    right = [
        # 1. Grasp and lift collector bowl
        Waypoint(1.2, (h_x, h_y, BOWL_CARRY_Z), FINGER_OPEN, "bowl_approach"),
        Waypoint(0.8, (h_x, h_y, h_z), FINGER_CLOSED, "bowl_grasp"),
        Waypoint(1.0, (h_x, h_y, BOWL_CARRY_Z), FINGER_CLOSED, "bowl_lift"),
        # Carry bowl into receiver collar along proven linear trajectory
        Waypoint(0.8, (h_x, h_y + 0.035, BOWL_CARRY_Z), FINGER_CLOSED, "bowl_carry_1"),
        Waypoint(0.8, (h_x, h_y + 0.070, BOWL_CARRY_Z), FINGER_CLOSED, "bowl_carry_2"),
        Waypoint(0.8, (h_x, h_y + 0.105, BOWL_CARRY_Z), FINGER_CLOSED, "bowl_carry_3"),
        Waypoint(0.8, (COLLAR_X, COLLAR_Y - 0.090, BOWL_CARRY_Z), FINGER_CLOSED, "bowl_align"),
        Waypoint(1.2, (COLLAR_X, COLLAR_Y - 0.090, TOP + 0.080), FINGER_CLOSED, "bowl_insert"),
    ]

    if cheat == "bowl_only":
        right += [
            Waypoint(0.8, (COLLAR_X, COLLAR_Y - 0.090, TOP + 0.080), FINGER_OPEN, "bowl_release"),
            Waypoint(1.2, (0.180, -0.200, BOWL_CARRY_Z), FINGER_OPEN, "right_retract"),
            Waypoint(8.0, (0.180, -0.200, BOWL_CARRY_Z), FINGER_OPEN, "right_park"),
        ]
        left = [
            Waypoint(12.0, (0.180, 0.200, BOWL_CARRY_Z), FINGER_OPEN, "left_park"),
        ]
        return {"left": left, "right": right}

    # Lock bowl with 30-deg clockwise bayonet rotation
    lock_hx = COLLAR_X - 0.090 * math.sin(math.radians(30.0))
    lock_hy = COLLAR_Y - 0.090 * math.cos(math.radians(30.0))
    right += [
        Waypoint(1.2, (lock_hx, lock_hy, TOP + 0.080), FINGER_CLOSED, "bowl_twist", yaw=math.radians(-30.0)),
        Waypoint(0.8, (lock_hx, lock_hy, TOP + 0.080), FINGER_OPEN, "bowl_release", yaw=math.radians(-30.0)),
        Waypoint(1.0, (lock_hx, lock_hy, BOWL_CARRY_Z + 0.020), FINGER_OPEN, "bowl_clear", yaw=math.radians(-30.0)),
        Waypoint(1.0, (c_x, c_y, CONE_CARRY_Z), FINGER_OPEN, "cone_approach"),
    ]

    if cheat == "bowl_locked":
        right += [
            Waypoint(1.2, (0.180, -0.200, BOWL_CARRY_Z), FINGER_OPEN, "right_retract"),
            Waypoint(8.0, (0.180, -0.200, BOWL_CARRY_Z), FINGER_OPEN, "right_park"),
        ]
        left = [
            Waypoint(12.0, (0.180, 0.200, BOWL_CARRY_Z), FINGER_OPEN, "left_park"),
        ]
        return {"left": left, "right": right}

    # 2. Grasp and seat strainer cone
    # When seated flush at cp = BOWL_SEAT_Z + 0.002 = 0.805, the knob top section is at 0.805 + 0.045 = 0.850 m.
    # Seating and releasing at 0.850 m keeps fingertips 13 mm above the bowl rim (0.8255 m),
    # completely avoiding any contact between fingers and the inner cup walls.
    # Aligning yaw to -25.0 deg matches the locked bowl orientation (-21.5 deg) for maximum concentric clearance.
    cone_seat_z = TOP + 0.096  # 0.844 m
    right += [
        Waypoint(0.8, (c_x, c_y, c_z), FINGER_CLOSED, "cone_grasp"),
        Waypoint(1.0, (c_x, c_y, CONE_CARRY_Z), FINGER_CLOSED, "cone_lift"),
        # Carry cone into collar along -Y approach corridor
        Waypoint(0.8, (0.270, -0.090, CONE_CARRY_Z), FINGER_CLOSED, "cone_carry_1", yaw=math.radians(-25.0)),
        Waypoint(0.8, (0.220, -0.040, CONE_CARRY_Z), FINGER_CLOSED, "cone_carry_2", yaw=math.radians(-25.0)),
        Waypoint(0.8, (COLLAR_X, COLLAR_Y, CONE_CARRY_Z), FINGER_CLOSED, "cone_align", yaw=math.radians(-25.0)),
        Waypoint(1.2, (COLLAR_X, COLLAR_Y, cone_seat_z), FINGER_CLOSED, "cone_seat", yaw=math.radians(-25.0)),
    ]

    if cheat == "no_release":
        right += [
            Waypoint(10.0, (COLLAR_X, COLLAR_Y, cone_seat_z), FINGER_CLOSED, "cone_hold", yaw=math.radians(-25.0)),
        ]
        left = [
            Waypoint(12.0, (0.180, 0.200, BOWL_CARRY_Z), FINGER_OPEN, "left_park"),
        ]
        return {"left": left, "right": right}

    right += [
        Waypoint(0.8, (COLLAR_X, COLLAR_Y, cone_seat_z), FINGER_OPEN, "cone_release", yaw=math.radians(-25.0)),
        Waypoint(1.0, (COLLAR_X, COLLAR_Y, CONE_CARRY_Z), FINGER_OPEN, "cone_clear", yaw=math.radians(-25.0)),
        Waypoint(1.0, (0.180, -0.200, BOWL_CARRY_Z), FINGER_OPEN, "right_retract"),
        Waypoint(12.0, (0.180, -0.200, BOWL_CARRY_Z), FINGER_OPEN, "right_park"),
    ]

    if cheat == "cone_seated":
        left = [
            Waypoint(12.0, (0.180, 0.200, BOWL_CARRY_Z), FINGER_OPEN, "left_park"),
        ]
        return {"left": left, "right": right}

    # Left arm: waits for cone to be seated (t = 19.8s), then executes mechanical test stroke
    left = [
        # Park left arm while Right arm installs bowl and cone
        Waypoint(19.8, (0.180, 0.200, BOWL_CARRY_Z), FINGER_OPEN, "left_wait"),
        # Approach and grasp lever knob with yaw=90 deg for infinite mechanical capture along stroke arc
        Waypoint(1.0, (0.171, 0.000, 1.025), FINGER_OPEN, "lever_approach", yaw=math.radians(90.0)),
        Waypoint(0.8, (0.171, 0.000, 1.016), FINGER_CLOSED, "lever_grasp", yaw=math.radians(90.0)),
        # Execute mechanical test stroke past 25 deg
        Waypoint(0.8, (0.210, 0.000, 0.985), FINGER_CLOSED, "lever_stroke_1", yaw=math.radians(90.0)),
        Waypoint(0.8, (0.240, 0.000, 0.935), FINGER_CLOSED, "lever_stroke_2", yaw=math.radians(90.0)),
        Waypoint(0.8, (0.248, 0.000, 0.880), FINGER_CLOSED, "lever_stroke_3", yaw=math.radians(90.0)),
        # Release lever to spring-return to rest stop
        Waypoint(0.8, (0.171, 0.000, 1.016), FINGER_OPEN, "lever_release", yaw=math.radians(90.0)),
        Waypoint(1.0, (0.180, 0.200, BOWL_CARRY_Z), FINGER_OPEN, "left_retract"),
        Waypoint(6.0, (0.180, 0.200, BOWL_CARRY_Z), FINGER_OPEN, "left_park"),
    ]

    return {"left": left, "right": right}
