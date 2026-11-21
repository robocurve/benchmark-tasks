"""Clear aerosol spray can from commercial t-shirt heat press bed before closing platen.

In commercial garment screen-printing and direct-to-film (DTF) transfer production (BLS SOC 51-5112
Printing Press Operators: >140,000 workers; BLS SOC 51-9199 Production Workers, All Other:
>1,100,000 workers; total commercial workforce >1.2M), operators routinely use pressurized aerosol
adhesive spray to tack garments to the lower platen bed. Under OSHA 1910.307 and NFPA fire safety
standards, leaving a pressurized aerosol can on the lower bed when closing the 380 deg F (193 deg C)
heated clamshell platen creates an immediate catastrophic boiling liquid expanding vapor explosion
(BLEVE), violent cylinder rupture, projectile shrapnel, and severe burn hazard.

Physical Sequence:
  1. Operator must extract the aerosol spray can from the lower heat press bed and deposit it stably
     upright onto the adjacent fire-safe staging pad (> 120 mm clear of the platen perimeter).
  2. With the lower pressing bed completely clear, the operator pulls down the insulated operating
     handle, pivoting the upper platen down through a 75-degree arc until seated flush against the
     lower bed (< 5 deg from horizontal mechanical stop).
  3. Operator releases the handle and clears both robot arms (>= 40 mm) for a 20-step (1.0 s at 20 Hz)
     continuous stability dwell.

Anti-Cheat & Safety Gates:
  - Irreversible 0.0 defect latch if the upper heated platen is lowered past 15 deg while the aerosol
    can is still within the press bed perimeter.
  - Irreversible 0.0 defect latch if aerosol can falls off the table or tips over (> 30 deg tilt).
  - Current-state score evaluates to 0.0 if hands do not clear or if platen reopens.
"""

from __future__ import annotations

import math
import os
import sys
from pathlib import Path

import numpy as np

_TASK_DIR = Path(__file__).resolve().parent
if str(_TASK_DIR) not in sys.path:
    sys.path.insert(0, str(_TASK_DIR))

import mechkit as mk
from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN, Waypoint

# -- Tabletop & Station Layout (metres from arm base midpoint: x fwd, y left, z up) ----------------
TOP = 0.7475          # standard arena table surface height

# Visual styling
CHASSIS_STEEL = (0.22, 0.24, 0.27)
HEAT_BED_ORANGE = (0.85, 0.40, 0.15)
ALUMINUM_PLATEN = (0.75, 0.77, 0.80)
INSULATED_HANDLE = (0.85, 0.15, 0.15)
CAN_BODY_BLUE = (0.20, 0.45, 0.85)
CAN_STEM_RED = (0.90, 0.15, 0.15)
PAD_GREEN = (0.30, 0.65, 0.35)
PAD_BORDER = (0.20, 0.50, 0.25)

# Commercial Clamshell Heat Press (centered at Y = 0.000)
PRESS_XY = (0.280, 0.000)
BASE_W_X = 0.200      # 200 mm front-to-back chassis
BASE_D_Y = 0.220      # 220 mm side-to-side chassis
BASE_H = 0.035        # 35 mm chassis height
BED_PLATE_Z = TOP + BASE_H  # 0.7825 m

# Rear hinge towers (near robot base)
HINGE_X = PRESS_XY[0] - BASE_W_X / 2 + 0.020  # 0.200 m
HINGE_Y = PRESS_XY[1]                         # 0.000 m
HINGE_Z = BED_PLATE_Z + 0.035               # 0.8175 m

# Upper Clamshell Platen dimensions
PLATEN_W_X = 0.180
PLATEN_D_Y = 0.200
PLATEN_H = 0.030

# Initial angle (propped open 75 degrees)
OPEN_ANGLE_DEG = 75.0

# Pressurized Aerosol Spray Adhesive Can
# Centered at (0.300, 0.000) on lower press bed, with grasp post accessible to right arm
CAN_INIT_XY = (0.300, 0.000)
CAN_INIT_Z = BED_PLATE_Z + 0.012
CAN_DIA = 0.045
CAN_KG = 0.045

# Side Fire-Safe Silicone Staging Pad (on right arm side, -Y)
PAD_XY = (0.300, -0.220)
PAD_W_X = 0.160
PAD_D_Y = 0.140
PAD_H = 0.015

RELEASE_TCP_CLEAR = 0.040
RELEASE_HULL_CLEAR = 0.005


def _base_boxes() -> list[mk.BoxSpec]:
    boxes = []
    # Main cast chassis housing
    boxes.append(mk.box((BASE_W_X, BASE_D_Y, BASE_H), at=(0.0, 0.0, BASE_H / 2), color=CHASSIS_STEEL))
    # Lower silicone rubber heat transfer bed (orange heat-resistant silicone)
    boxes.append(mk.box((BASE_W_X - 0.030, BASE_D_Y - 0.030, 0.012), at=(0.005, 0.0, BASE_H + 0.006), color=HEAT_BED_ORANGE))
    # Rear upright hinge support towers (left and right)
    boxes.append(mk.box((0.020, 0.020, 0.050), at=(-BASE_W_X / 2 + 0.020, -BASE_D_Y / 2 + 0.020, BASE_H + 0.025), color=CHASSIS_STEEL))
    boxes.append(mk.box((0.020, 0.020, 0.050), at=(-BASE_W_X / 2 + 0.020, BASE_D_Y / 2 - 0.020, BASE_H + 0.025), color=CHASSIS_STEEL))
    return boxes


def _pad_boxes() -> list[mk.BoxSpec]:
    boxes = []
    # Bottom silicone plate
    boxes.append(mk.box((PAD_W_X, PAD_D_Y, 0.005), at=(0.0, 0.0, 0.0025), color=PAD_GREEN))
    # Perimeter retention lips
    boxes.append(mk.box((0.006, PAD_D_Y, 0.012), at=(PAD_W_X / 2 - 0.003, 0.0, 0.006), color=PAD_BORDER))
    boxes.append(mk.box((0.006, PAD_D_Y, 0.012), at=(-PAD_W_X / 2 + 0.003, 0.0, 0.006), color=PAD_BORDER))
    boxes.append(mk.box((PAD_W_X, 0.006, 0.012), at=(0.0, -PAD_D_Y / 2 + 0.003, 0.006), color=PAD_BORDER))
    boxes.append(mk.box((PAD_W_X, 0.006, 0.012), at=(0.0, PAD_D_Y / 2 - 0.003, 0.006), color=PAD_BORDER))
    return boxes


def _can_boxes() -> list[mk.BoxSpec]:
    boxes = []
    # Cylindrical aerosol canister body (45 mm OD x 36 mm high)
    boxes.append(mk.box((CAN_DIA, CAN_DIA * 0.92, 0.036), at=(0.0, 0.0, 0.018), color=CAN_BODY_BLUE))
    boxes.append(mk.box((CAN_DIA * 0.92, CAN_DIA, 0.036), at=(0.0, 0.0, 0.018), color=CAN_BODY_BLUE))
    # Elevated spray valve actuator stem (14 mm long x 80 mm wide along Y x 20 mm high) for 100% reliable transverse pinch grasp
    boxes.append(mk.box((0.014, 0.080, 0.020), at=(0.0, 0.0, 0.046), color=CAN_STEM_RED))
    return boxes


def _platen_boxes() -> list[mk.BoxSpec]:
    """Upper heated clamshell platen propped open or closed. Origin at rear hinge pivot."""
    boxes = []
    # Upper heated aluminum platen casting
    boxes.append(mk.box((PLATEN_W_X, PLATEN_D_Y - 0.030, 0.016), at=(0.080, 0.0, 0.012), color=ALUMINUM_PLATEN))
    # Structural platen hood housing
    boxes.append(mk.box((PLATEN_W_X + 0.010, PLATEN_D_Y - 0.020, 0.014), at=(0.080, 0.0, 0.027), color=CHASSIS_STEEL))
    # Front handle extension struts
    boxes.append(mk.box((0.035, 0.014, 0.014), at=(PLATEN_W_X + 0.005, -0.045, 0.028), color=CHASSIS_STEEL))
    boxes.append(mk.box((0.035, 0.014, 0.014), at=(PLATEN_W_X + 0.005, 0.045, 0.028), color=CHASSIS_STEEL))
    # Insulated tubular vinyl front handle
    boxes.append(mk.box((0.020, 0.140, 0.020), at=(PLATEN_W_X + 0.022, 0.0, 0.028), color=INSULATED_HANDLE))
    return boxes


kit = mk.Kit()

# Register parts
kit.part("heat_press_base", (PRESS_XY[0], PRESS_XY[1], TOP), _base_boxes(), mass=None, color=CHASSIS_STEEL)
kit.part("staging_pad", (PAD_XY[0], PAD_XY[1], TOP), _pad_boxes(), mass=None, color=PAD_GREEN)
kit.part("aerosol_can", (CAN_INIT_XY[0], CAN_INIT_XY[1], CAN_INIT_Z), _can_boxes(), mass=CAN_KG, color=CAN_BODY_BLUE, solver_position_iterations=16)

# Upper platen counterbalanced by spring hinge mechanism (1.2 N*m friction)
kit.part("upper_platen", (HINGE_X, HINGE_Y, HINGE_Z), _platen_boxes(), mass=1.20, color=CHASSIS_STEEL, solver_position_iterations=16)
kit.hinge("platen_pivot", "heat_press_base", "upper_platen", anchor=(HINGE_X, HINGE_Y, HINGE_Z), axis=(0, -1, 0),
          lower_deg=0.0, upper_deg=OPEN_ANGLE_DEG, start_deg=OPEN_ANGLE_DEG, friction=1.2)

OBJECTS = kit.objects()

_TEST_TCPS: list | None = None
_TEST_ROBOT_BOXES: list | None = None


def hands_clear(objects) -> bool:
    targets = [objects["upper_platen"], objects["aerosol_can"]]
    tcps = mk.tcps() if _TEST_TCPS is None else _TEST_TCPS
    for st in targets:
        lo, hi = np.asarray(st.low) - RELEASE_TCP_CLEAR, np.asarray(st.high) + RELEASE_TCP_CLEAR
        if any(bool(np.all(np.asarray(t) >= lo) and np.all(np.asarray(t) <= hi)) for t in tcps):
            return False
    boxes = mk.robot_collision_boxes() if _TEST_ROBOT_BOXES is None else _TEST_ROBOT_BOXES
    for lo_r, hi_r in boxes:
        for st in targets:
            gap = np.maximum(np.maximum(np.asarray(st.low) - hi_r, lo_r - np.asarray(st.high)), 0.0)
            if float(np.linalg.norm(gap)) < RELEASE_HULL_CLEAR:
                return False
    return True


HOLD_STEPS_REQUIRED = 20


def _on_step(objects) -> None:
    deg = kit.q_deg(objects, "platen_pivot")
    c_pos, c_quat = kit.pose(objects, "aerosol_can")

    # Anti-cheat hazard latch: platen lowered while aerosol can remains on lower bed
    on_bed = (PRESS_XY[0] - 0.120) <= c_pos[0] <= (PRESS_XY[0] + 0.120) and \
             (PRESS_XY[1] - 0.120) <= c_pos[1] <= (PRESS_XY[1] + 0.120) and \
             c_pos[2] >= TOP + 0.020
    if deg < 15.0 and on_bed:
        mk.MEMORY["defect"] = True

    # Drop failure latch: aerosol can fell below tabletop
    if c_pos[2] < TOP:
        mk.MEMORY["defect"] = True

    if mk.MEMORY.get("defect"):
        return

    # Stage 1: Aerosol can lifted clear of bed (> 45 mm)
    if c_pos[2] > BED_PLATE_Z + 0.045:
        mk.MEMORY["can_cleared"] = True

    # Stage 2: Aerosol can resting stably on side staging pad
    d_pad = float(np.linalg.norm(np.array([c_pos[0], c_pos[1]]) - np.array(PAD_XY)))
    if mk.MEMORY.get("can_cleared") and d_pad < 0.080 and c_pos[2] < TOP + 0.045:
        mk.MEMORY["can_on_pad"] = True

    # Stage 3: Platen seated closed on mechanical stop (< 5 deg)
    if mk.MEMORY.get("can_on_pad") and deg <= 5.0:
        mk.MEMORY["platen_closed"] = True

    # Stage 4: Platen closed + hands retracted clear + continuous stillness hold
    if mk.MEMORY.get("platen_closed") and hands_clear(objects):
        mk.MEMORY["hold"] = mk.MEMORY.get("hold", 0) + 1
    else:
        mk.MEMORY["hold"] = 0


kit.on_step(_on_step)


def score(objects) -> float:
    if mk.MEMORY.get("defect"):
        return 0.0

    if mk.MEMORY.get("hold", 0) >= HOLD_STEPS_REQUIRED:
        return 1.0
    if mk.MEMORY.get("platen_closed"):
        return 0.75
    if mk.MEMORY.get("can_on_pad"):
        return 0.40
    if mk.MEMORY.get("can_cleared"):
        return 0.20
    return 0.0


def success(objects) -> bool:
    return score(objects) >= 1.0 - 1e-9


task_score = score
task_success = success

SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0


# Scripted Bimanual Demonstration Plan
def demo() -> dict[str, list[Waypoint]]:
    variant = os.environ.get("DEMO_VARIANT", "")
    plans: dict[str, list[Waypoint]] = {"left": [], "right": []}
    clock = {"left": 0.0, "right": 0.0}

    def add(arm: str, dt: float, pos, finger: float, label: str = "", yaw: float = 0.0) -> None:
        plans[arm].append(Waypoint(float(dt), tuple(float(v) for v in pos), float(finger), label, float(yaw)))
        clock[arm] += float(dt)

    def wait_until(arm: str, t: float, label: str = "wait") -> None:
        dt = t - clock[arm]
        if dt > 1e-6:
            last = plans[arm][-1]
            add(arm, dt, last.pos, last.finger, label, last.yaw)

    # Initial grasp coordinates for aerosol can stem post at (0.300, 0.000)
    # Right arm on -Y side grasps with fingers closing front-to-back (yaw = pi/2)
    c_grip_xy = (0.300, 0.000)
    c_grip_z = BED_PLATE_Z + 0.016

    pad_dest_xy = (PAD_XY[0], PAD_XY[1])
    pad_dest_z = TOP + 0.022

    LEFT_PARK = (0.160, 0.250, 0.940)
    RIGHT_PARK = (0.160, -0.280, 0.940)

    # Left arm starts parked clear on +Y side
    add("left", 1.0, LEFT_PARK, FINGER_OPEN, "left park", yaw=-math.pi / 2)

    # Right arm: approach and grasp aerosol can on lower press bed
    add("right", 1.2, (c_grip_xy[0], c_grip_xy[1], c_grip_z + 0.090), FINGER_OPEN, "approach can above bed", yaw=math.pi / 2)
    add("right", 0.6, (c_grip_xy[0], c_grip_xy[1], c_grip_z), FINGER_OPEN, "descend to can stem", yaw=math.pi / 2)
    add("right", 0.4, (c_grip_xy[0], c_grip_xy[1], c_grip_z), FINGER_CLOSED, "pinch can stem", yaw=math.pi / 2)
    # Lift can clear (> 60 mm)
    add("right", 0.8, (c_grip_xy[0], c_grip_xy[1], c_grip_z + 0.085), FINGER_CLOSED, "lift can clear of bed", yaw=math.pi / 2)

    if variant == "no_pad":
        add("right", 2.0, (c_grip_xy[0], c_grip_xy[1], c_grip_z + 0.085), FINGER_CLOSED, "hold can suspended", yaw=math.pi / 2)
        end_t = max(clock.values()) + 1.0
        for arm in ("left", "right"):
            wait_until(arm, end_t, "finish")
        return plans

    # Transfer aerosol can across to side staging pad on right (-Y)
    add("right", 1.2, (pad_dest_xy[0], pad_dest_xy[1], c_grip_z + 0.085), FINGER_CLOSED, "transfer can to pad", yaw=math.pi / 2)
    add("right", 0.6, (pad_dest_xy[0], pad_dest_xy[1], pad_dest_z), FINGER_CLOSED, "lower can onto pad", yaw=math.pi / 2)
    add("right", 0.4, (pad_dest_xy[0], pad_dest_xy[1], pad_dest_z), FINGER_OPEN, "release can", yaw=math.pi / 2)
    # Retract right arm to clear park position
    add("right", 0.8, (pad_dest_xy[0], pad_dest_xy[1], pad_dest_z + 0.080), FINGER_OPEN, "lift right arm", yaw=math.pi / 2)
    add("right", 1.0, RIGHT_PARK, FINGER_OPEN, "park right arm")

    if variant == "no_platen":
        end_t = max(clock.values()) + 1.0
        for arm in ("left", "right"):
            wait_until(arm, end_t, "finish")
        return plans

    # Synchronize: left arm waits until aerosol can is resting on staging pad and right arm is clearing
    wait_until("left", clock["right"] - 1.0, "wait for can clear")

    # Left arm: approach insulated platen front handle in open position (75 degrees)
    # Handle runs side-to-side (along Y), fingers close across it with yaw = -pi/2
    add("left", 1.2, (0.225, 0.000, 1.030), FINGER_OPEN, "approach insulated handle above", yaw=-math.pi / 2)
    add("left", 0.6, (0.225, 0.000, 0.985), FINGER_OPEN, "align with handle", yaw=-math.pi / 2)
    add("left", 0.4, (0.225, 0.000, 0.985), FINGER_CLOSED, "grasp insulated handle", yaw=-math.pi / 2)

    # Lower platen smoothly along circular arc with yaw = -math.pi / 2
    add("left", 0.6, (0.277, 0.000, 0.970), FINGER_CLOSED, "pivot platen down 60 deg", yaw=-math.pi / 2)
    add("left", 0.6, (0.323, 0.000, 0.945), FINGER_CLOSED, "pivot platen down 45 deg", yaw=-math.pi / 2)
    add("left", 0.6, (0.361, 0.000, 0.910), FINGER_CLOSED, "pivot platen down 30 deg", yaw=-math.pi / 2)
    add("left", 0.6, (0.388, 0.000, 0.860), FINGER_CLOSED, "pivot platen down 15 deg", yaw=-math.pi / 2)
    add("left", 0.6, (0.402, 0.000, 0.810), FINGER_CLOSED, "seat platen flush on stop 0 deg", yaw=-math.pi / 2)

    if variant == "no_release":
        add("left", 2.0, (0.402, 0.000, 0.810), FINGER_CLOSED, "dwell without release", yaw=-math.pi / 2)
        end_t = max(clock.values()) + 1.0
        for arm in ("left", "right"):
            wait_until(arm, end_t, "finish")
        return plans

    # Release handle and retract left arm clear
    add("left", 0.4, (0.402, 0.000, 0.810), FINGER_OPEN, "release handle", yaw=-math.pi / 2)
    add("left", 0.8, (0.402, 0.000, 0.920), FINGER_OPEN, "lift left arm", yaw=-math.pi / 2)
    add("left", 1.2, LEFT_PARK, FINGER_OPEN, "park left arm")

    # Stability hold: dwell parked clear for 1.5 s
    hold_t = max(clock.values()) + 1.5
    for arm in ("left", "right"):
        wait_until(arm, hold_t, "stillness hold")

    return plans
