"""Clamp safety grounding lead to enclosure busbar before pulling ceramic fuse carrier.

High-voltage and industrial electrical safety procedures (OSHA 1910.269(n) and NFPA 70E §120.5)
mandate personal protective grounding: before de-energizing or servicing fused disconnect
equipment, workers must bond a heavy spring-loaded grounding clamp to the enclosure busbar.
Only after the system is positively grounded may the ceramic pull-out fuse carrier be safely
extracted and placed in the safety parts tray.
"""

from __future__ import annotations

import math
import os
import pathlib
import sys
from typing import Any

# Ensure task directory is on sys.path for local helper modules like mechkit
_TASK_DIR = str(pathlib.Path(__file__).resolve().parent)
if _TASK_DIR not in sys.path:
    sys.path.insert(0, _TASK_DIR)

import numpy as np

import mechkit as mk
from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN, Waypoint

TAG = "clamp_grounding_clamp_fuse_carrier"

# Physical Materials and Colors
ANSI_GRAY = (0.50, 0.52, 0.55)        # NEMA 3R painted steel enclosure
COPPER_SHINE = (0.75, 0.45, 0.25)     # 110 copper busbar tab
BRONZE_CAST = (0.55, 0.42, 0.22)      # Cast bronze grounding clamp body
SAFETY_YELLOW = (0.92, 0.75, 0.12)    # Dielectric insulated grip handle
CERAMIC_WHITE = (0.90, 0.90, 0.88)    # High-dielectric glazed ceramic carrier body
CONTACT_BRASS = (0.80, 0.65, 0.25)    # Terminal clips and fuse ferrule contacts
SAFETY_ORANGE = (0.88, 0.35, 0.12)    # Anti-static parts tray

SCENE = "station"
TABLE_TOP = {"station": 0.748}
TOP = TABLE_TOP[SCENE]

# Station Positions (Left arm at +Y, Right arm at -Y)
BOX_X = 0.220
BOX_Y = 0.000
BOX_BASE_Z = TOP + 0.055

# Ground busbar lug on the accessible +Y flange of the enclosure
BUSBAR_POS = np.array([BOX_X, 0.120, TOP + 0.045])
CLAMP_INIT_POS = np.array([0.180, 0.200, TOP + 0.010])
FUSE_SEATED_POS = np.array([BOX_X, 0.000, TOP + 0.055])
TRAY_POS = np.array([0.220, -0.180, TOP + 0.015])


# -- Kit Setup -----------------------------------------------------------------

kit = mk.Kit()


# -- 1. Commercial Fused Disconnect Enclosure (NEMA 3R 60A Box) -----------------
kit.part(
    name="disconnect_box",
    pos=(BOX_X, BOX_Y, BOX_BASE_Z),
    boxes=[
        # Heavy-gauge steel backplate mounting chassis
        mk.box((0.140, 0.180, 0.010), at=(0.000, 0.000, -0.045), color=ANSI_GRAY),
        # Enclosure side walls (left +Y, right -Y, top +X, bottom -X)
        mk.box((0.140, 0.008, 0.080), at=(0.000, 0.086, 0.000), color=ANSI_GRAY),
        mk.box((0.140, 0.008, 0.080), at=(0.000, -0.086, 0.000), color=ANSI_GRAY),
        mk.box((0.008, 0.180, 0.080), at=(0.066, 0.000, 0.000), color=ANSI_GRAY),
        mk.box((0.008, 0.180, 0.080), at=(-0.066, 0.000, 0.000), color=ANSI_GRAY),
        # Open enclosure door swung open at top (+X rim) clear of lateral tray transfer
        mk.box((0.006, 0.180, 0.080), at=(0.106, 0.000, 0.040), color=ANSI_GRAY),
        # Broad Copper Ground Busbar Mounting Shelf extending along +Y (80x80 mm support)
        mk.box((0.080, 0.080, 0.012), at=(0.000, 0.126, -0.040), color=COPPER_SHINE),
        # Copper Ground Busbar Terminal Tab extending vertically from the shelf (10 mm thick in Y x 40 mm wide in X)
        mk.box((0.040, 0.010, 0.035), at=(0.000, 0.120, -0.015), color=COPPER_SHINE),
        # Ground symbol identification marker plate
        mk.box((0.018, 0.002, 0.012), at=(0.000, 0.126, -0.015), color=SAFETY_YELLOW),
        # Internal fuse receptacle socket base block
        mk.box((0.090, 0.050, 0.020), at=(0.000, 0.000, -0.030), color=CERAMIC_WHITE),
        # Upper terminal spring receiver clips (at x = 0.025, flanking y=0.000 with 14 mm gap for 5 mm blade)
        mk.box((0.018, 0.003, 0.016), at=(0.025, 0.007, -0.025), color=CONTACT_BRASS),
        mk.box((0.018, 0.003, 0.016), at=(0.025, -0.007, -0.025), color=CONTACT_BRASS),
        mk.box((0.018, 0.017, 0.004), at=(0.025, 0.000, -0.035), color=CONTACT_BRASS),
        # Lower terminal clips (at x = -0.025, flanking y=0.000)
        mk.box((0.018, 0.003, 0.016), at=(-0.025, 0.007, -0.025), color=CONTACT_BRASS),
        mk.box((0.018, 0.003, 0.016), at=(-0.025, -0.007, -0.025), color=CONTACT_BRASS),
        mk.box((0.018, 0.017, 0.004), at=(-0.025, 0.000, -0.035), color=CONTACT_BRASS),
    ],
    mass=1.200,
)


# -- 2. Heavy Bronze Spring-Loaded Grounding Clamp -----------------------------
# Broad stable base plate and jaw housing with ergonomic upright capture T-handle
kit.part(
    name="grounding_clamp",
    pos=(CLAMP_INIT_POS[0], CLAMP_INIT_POS[1], CLAMP_INIT_POS[2]),
    boxes=[
        # Stable bronze base plate and housing (50 mm in X x 40 mm in Y x 18 mm high in Z)
        mk.box((0.050, 0.040, 0.018), at=(0.000, 0.000, 0.000), color=BRONZE_CAST),
        # Bronze duckbill jaws extending along -Y (flanking x = 0 with 10 mm gap for 6 mm busbar)
        mk.box((0.005, 0.025, 0.018), at=(0.008, -0.030, 0.000), color=BRONZE_CAST),
        mk.box((0.005, 0.025, 0.018), at=(-0.008, -0.030, 0.000), color=BRONZE_CAST),
        # Central dielectric insulated grip handle stem (14x14 mm stem x 40 mm high)
        mk.box((0.014, 0.014, 0.040), at=(0.000, 0.000, 0.028), color=SAFETY_YELLOW),
        # Mechanical capture T-bar cross handle (38 mm wide along Y x 22 mm deep along X x 10 mm thick)
        mk.box((0.022, 0.038, 0.010), at=(0.000, 0.000, 0.052), color=SAFETY_YELLOW),
    ],
    mass=0.045,
)


# -- 3. Ceramic 60A Pull-Out Fuse Carrier Assembly -----------------------------
kit.part(
    name="fuse_carrier",
    pos=(FUSE_SEATED_POS[0], FUSE_SEATED_POS[1], FUSE_SEATED_POS[2]),
    boxes=[
        # Glazed white ceramic main insulator block (75 mm long x 40 mm wide x 22 mm deep)
        mk.box((0.075, 0.040, 0.022), at=(0.000, 0.000, 0.000), color=CERAMIC_WHITE),
        # Dual terminal male bayonet contact blades (insert between clip leaves)
        mk.box((0.014, 0.005, 0.018), at=(0.025, 0.000, -0.018), color=CONTACT_BRASS),
        mk.box((0.014, 0.005, 0.018), at=(-0.025, 0.000, -0.018), color=CONTACT_BRASS),
        # Front central ergonomic pull-handle stem (14 mm wide x 14 mm thick x 35 mm high)
        mk.box((0.014, 0.014, 0.035), at=(0.000, 0.000, 0.025), color=SAFETY_YELLOW),
        # Pull-handle capture T-bar cross handle (36 mm wide along Y x 22 mm deep along X x 10 mm thick)
        mk.box((0.022, 0.036, 0.010), at=(0.000, 0.000, 0.044), color=SAFETY_YELLOW),
    ],
    mass=0.040,
)


# -- 4. Molded Anti-Static Parts Safety Tray -----------------------------------
kit.part(
    name="safety_tray",
    pos=(TRAY_POS[0], TRAY_POS[1], TRAY_POS[2]),
    boxes=[
        # Generous tray base plate (140 mm x 110 mm)
        mk.box((0.140, 0.110, 0.008), at=(0.000, 0.000, -0.010), color=SAFETY_ORANGE),
        # Perimeter containment rim walls
        mk.box((0.140, 0.006, 0.025), at=(0.000, 0.052, 0.005), color=SAFETY_ORANGE),
        mk.box((0.140, 0.006, 0.025), at=(0.000, -0.052, 0.005), color=SAFETY_ORANGE),
        mk.box((0.006, 0.110, 0.025), at=(0.067, 0.000, 0.005), color=SAFETY_ORANGE),
        mk.box((0.006, 0.110, 0.025), at=(-0.067, 0.000, 0.005), color=SAFETY_ORANGE),
    ],
    mass=0.250,
)

OBJECTS = kit.objects()


# -- State Tracking and Scoring ------------------------------------------------

def _on_step(objects: dict[str, Any]) -> None:
    M = mk.MEMORY
    step_idx = mk._EP.get("step", 0)

    gp, gq = kit.pose(objects, "grounding_clamp")
    fp, fq = kit.pose(objects, "fuse_carrier")

    # Record initial reference positions at start
    if "init_fuse_pos" not in M:
        M["init_fuse_pos"] = fp.copy()
    if "init_clamp_pos" not in M:
        M["init_clamp_pos"] = gp.copy()

    # 1. Grounding clamp status
    # Jaws at (gp[0], gp[1] - 0.030, gp[2]) clamped over copper busbar tab at BUSBAR_POS
    jaw_x = gp[0]
    jaw_y = gp[1] - 0.030
    jaw_z = gp[2]
    dr_clamp = math.sqrt((jaw_x - BUSBAR_POS[0]) ** 2 + (jaw_y - BUSBAR_POS[1]) ** 2)
    dz_clamp = abs(jaw_z - BUSBAR_POS[2])
    # Tighter clamp grounding threshold: strictly requires jaws straddling copper busbar tab (<= 25 mm)
    clamped = dr_clamp <= 0.025 and dz_clamp <= 0.025
    if clamped:
        M["clamp_grounded"] = True

    # Clamp lifted from table
    clamp_lifted = gp[2] > M["init_clamp_pos"][2] + 0.020
    if clamp_lifted:
        M["clamp_lifted"] = True

    # 2. Hazard Fault Trigger Check (Rule 1: Hazard cleared before action)
    # If fuse carrier is displaced before clamp is firmly grounded, trip fault interlock
    dist_fuse_init = math.sqrt(
        (fp[0] - M["init_fuse_pos"][0]) ** 2 +
        (fp[1] - M["init_fuse_pos"][1]) ** 2 +
        (fp[2] - M["init_fuse_pos"][2]) ** 2
    )
    if dist_fuse_init > 0.015 and not M.get("clamp_grounded", False):
        # UNGROUNDED EXTRACTION HAZARD TRIPPED!
        M["ungrounded_hazard_tripped"] = True

    # 3. Fuse Carrier Status
    # Extracted from enclosure spring clips (moved > 35 mm from initial seated position)
    fuse_extracted = dist_fuse_init > 0.035 and not M.get("ungrounded_hazard_tripped", False)
    if fuse_extracted:
        M["fuse_extracted"] = True

    # Placed strictly inside safety parts tray (tray interior is 140x110 mm)
    dr_tray = math.sqrt((fp[0] - TRAY_POS[0]) ** 2 + (fp[1] - TRAY_POS[1]) ** 2)
    dz_tray = abs(fp[2] - (TRAY_POS[2] + 0.018))
    # Tighter tray containment threshold: strictly within 45 mm of tray center
    fuse_in_tray = dr_tray <= 0.045 and dz_tray <= 0.025 and not M.get("ungrounded_hazard_tripped", False)
    if fuse_in_tray:
        M["fuse_in_tray"] = True

    # 4. Hands clear
    hands_clear = (
        _released(objects["grounding_clamp"], 0.035)
        and _released(objects["fuse_carrier"], 0.035)
    )

    # 5. Success hold counter
    if (
        M.get("clamp_grounded", False)
        and fuse_in_tray
        and hands_clear
        and not failed(objects)
        and not M.get("ungrounded_hazard_tripped", False)
    ):
        hold = M.get("hold_timer", 0) + 1
        M["hold_timer"] = hold
    else:
        M["hold_timer"] = 0

    if step_idx % 20 == 0:
        print(
            f"[{TAG}] step={step_idx} dr_c={dr_clamp:.3f} dz_c={dz_clamp:.3f} "
            f"dist_f={dist_fuse_init:.3f} dr_tray={dr_tray:.3f} dz_tray={dz_tray:.3f} "
            f"grounded={clamped} hazard={M.get('ungrounded_hazard_tripped', False)} "
            f"in_tray={fuse_in_tray} clear={hands_clear} hold={M.get('hold_timer', 0)} "
            f"score={score(objects)}",
            flush=True,
        )


kit.on_step(_on_step)


def _released(obj_state: Any, margin: float = 0.035) -> bool:
    """True if no robot gripper or finger is within margin of the object."""
    tcps = mk.tcps()
    if not tcps:
        return True
    pos = np.asarray(obj_state.pos, dtype=float)
    return all(np.linalg.norm(np.asarray(tcp, dtype=float) - pos) >= margin for tcp in tcps)


def failed(objects: dict[str, Any]) -> bool:
    """True if enclosure tipped, components dropped off bench, or hazard tripped."""
    if mk.MEMORY.get("ungrounded_hazard_tripped", False):
        return True

    # Verify enclosure stayed upright on table
    box_pos, _ = kit.pose(objects, "disconnect_box")
    if box_pos[2] < TOP + 0.030 or box_pos[2] > TOP + 0.150:
        return True

    # Verify parts did not tumble off the table
    for name in ("grounding_clamp", "fuse_carrier", "safety_tray"):
        pos, _ = kit.pose(objects, name)
        if pos[2] < TOP - 0.050:
            return True

    return False


def success(objects: dict[str, Any]) -> bool:
    """True if full task requirements are satisfied."""
    return score(objects) >= 1.0


def score(objects: dict[str, Any]) -> float:
    """Calculate staged task score with safety lockout (0.0 to 1.0)."""
    M = mk.MEMORY

    # Hard safety prerequisite: if fuse was pulled before grounding, 0.0
    if M.get("ungrounded_hazard_tripped", False):
        return 0.0

    if failed(objects):
        return 0.0

    pts = 0.0

    # Stage 1: Grounding clamp lifted from table (0.20)
    if M.get("clamp_lifted", False):
        pts += 0.20

    # Stage 2: Grounding clamp attached to copper busbar (0.30 -> total 0.50)
    if M.get("clamp_grounded", False):
        pts += 0.30

    # Stage 3: Ceramic fuse carrier extracted from panel (0.25 -> total 0.75)
    if M.get("fuse_extracted", False):
        pts += 0.25

    # Stage 4: Fuse carrier placed in safety tray and hands clear (0.25 -> total 1.0)
    if M.get("fuse_in_tray", False) and _released(objects["fuse_carrier"], 0.035):
        # Require 15 consecutive steps of stable hold
        if M.get("hold_timer", 0) >= 15:
            pts += 0.25

    return pts


# -- Scripted Demo Waypoints ---------------------------------------------------

def demo() -> dict[str, list[Waypoint]]:
    cheat = os.environ.get("DISCONNECT_CHEAT", "")

    CARRY_Z = TOP + 0.160

    if cheat == "nothing":
        left = [Waypoint(10.0, (0.180, 0.200, CARRY_Z), FINGER_OPEN, "park_left")]
        right = [Waypoint(10.0, (0.180, -0.200, CARRY_Z), FINGER_OPEN, "park_right")]
        return {"left": left, "right": right}

    # Clamp handle grip coordinates (Left side +Y)
    cg_x = CLAMP_INIT_POS[0]
    cg_y = CLAMP_INIT_POS[1]
    cg_z = CLAMP_INIT_POS[2] + 0.025

    # Busbar tab attachment coordinates for clamp handle (+Y side of enclosure)
    # Jaws at (x, y - 0.030, z) slide over BUSBAR_POS (y=0.120). Shifted by -0.045m to straddle copper tab directly.
    attach_x = BUSBAR_POS[0]
    attach_y = BUSBAR_POS[1] + 0.030 - 0.045
    attach_z = BUSBAR_POS[2] + 0.025

    # Fuse pull handle coordinates (Center Y=0)
    fh_x = FUSE_SEATED_POS[0]
    fh_y = FUSE_SEATED_POS[1]
    fh_z = FUSE_SEATED_POS[2] + 0.025

    # Tray placement coordinates (Right side -Y)
    tray_place_z = TRAY_POS[2] + 0.028

    if cheat == "no_ground":
        # Right arm pulls fuse WITHOUT grounding (trips hazard fault detector -> score 0.0)
        left = [Waypoint(10.0, (0.180, 0.200, CARRY_Z), FINGER_OPEN, "left_park")]
        right = [
            Waypoint(1.2, (fh_x, fh_y, CARRY_Z), FINGER_OPEN, "fuse_approach"),
            Waypoint(0.8, (fh_x, fh_y, fh_z), FINGER_CLOSED, "fuse_grasp"),
            Waypoint(1.2, (fh_x, fh_y, CARRY_Z), FINGER_CLOSED, "fuse_extract_unprotected"),
            Waypoint(8.0, (fh_x, fh_y, CARRY_Z), FINGER_CLOSED, "fuse_hold"),
        ]
        return {"left": left, "right": right}

    left = [
        # 1. Approach and grasp grounding clamp handle (+Y side)
        Waypoint(1.2, (cg_x, cg_y, CARRY_Z), FINGER_OPEN, "clamp_approach"),
        Waypoint(0.8, (cg_x, cg_y, cg_z), FINGER_CLOSED, "clamp_grasp"),
        Waypoint(1.0, (cg_x, cg_y, CARRY_Z), FINGER_CLOSED, "clamp_lift"),
        # Carry clamp toward enclosure busbar tab
        Waypoint(0.8, (attach_x, attach_y + 0.040, CARRY_Z), FINGER_CLOSED, "clamp_carry_1"),
        Waypoint(0.8, (attach_x, attach_y + 0.040, attach_z), FINGER_CLOSED, "clamp_align"),
        # Push duckbill jaws over copper busbar tab
        Waypoint(1.2, (attach_x, attach_y, attach_z), FINGER_CLOSED, "clamp_attach"),
        # Release handle so spring jaws bite into copper busbar
        Waypoint(0.8, (attach_x, attach_y, attach_z), FINGER_OPEN, "clamp_release"),
        # Lift straight UP along +Z to CARRY_Z to avoid dragging across clamp
        Waypoint(1.0, (attach_x, attach_y, CARRY_Z), FINGER_OPEN, "clamp_clear_up"),
        Waypoint(1.0, (0.180, 0.220, CARRY_Z), FINGER_OPEN, "left_retract"),
        Waypoint(14.0, (0.180, 0.220, CARRY_Z), FINGER_OPEN, "left_park"),
    ]

    if cheat == "ground_only":
        right = [Waypoint(18.0, (0.180, -0.200, CARRY_Z), FINGER_OPEN, "right_park")]
        return {"left": left, "right": right}

    # Right arm: waits for grounding clamp attachment (t = 8.0s), then extracts fuse
    right = [
        # Park until grounding clamp is attached to busbar
        Waypoint(8.0, (0.180, -0.200, CARRY_Z), FINGER_OPEN, "right_wait"),
        # Approach and grasp ceramic fuse carrier pull-handle
        Waypoint(1.2, (fh_x, fh_y, CARRY_Z), FINGER_OPEN, "fuse_approach"),
        Waypoint(0.8, (fh_x, fh_y, fh_z), FINGER_CLOSED, "fuse_grasp"),
        # Withdraw fuse carrier straight up out of spring-clip terminals (20 N draw)
        Waypoint(1.2, (fh_x, fh_y, CARRY_Z), FINGER_CLOSED, "fuse_extract"),
    ]

    if cheat == "no_tray":
        right += [
            Waypoint(10.0, (fh_x, fh_y, CARRY_Z), FINGER_CLOSED, "fuse_hold"),
        ]
        return {"left": left, "right": right}

    right += [
        # Carry fuse carrier over to safety parts tray (-Y side)
        Waypoint(1.2, (TRAY_POS[0], TRAY_POS[1], CARRY_Z), FINGER_CLOSED, "tray_approach"),
        Waypoint(1.0, (TRAY_POS[0], TRAY_POS[1], tray_place_z), FINGER_CLOSED, "tray_place"),
        # Release fuse carrier into tray (generous time to open fully clear of T-bar)
        Waypoint(1.2, (TRAY_POS[0], TRAY_POS[1], tray_place_z), FINGER_OPEN, "fuse_release"),
        # Lift straight up to CARRY_Z before any lateral motion to prevent dragging
        Waypoint(1.2, (TRAY_POS[0], TRAY_POS[1], CARRY_Z), FINGER_OPEN, "tray_clear_up"),
        # Retract clear and park
        Waypoint(1.0, (0.180, -0.220, CARRY_Z), FINGER_OPEN, "right_retract"),
        Waypoint(8.0, (0.180, -0.220, CARRY_Z), FINGER_OPEN, "right_park"),
    ]

    return {"left": left, "right": right}
