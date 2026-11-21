"""Mount commercial condiment dispenser pump in deep fountain jar task.

BLS SOC 35-3023: Fast Food and Counter Workers
BLS SOC 35-2021: Food Preparation Workers

Assembling and locking a commercial portion-control condiment pump assembly into a
deep stainless steel fountain jar after sanitation.
- Steady stainless fountain jar side loop handle with right hand.
- Grasp pump stem from wash rack with left hand, lift vertically clear of socket.
- Transfer coaxially over fountain jar bore.
- Insert pick-up tube down into jar bore.
- Seat lid collar flush on flared jar rim.
- Press locking detent latch into rim slot.
- Execute upward proof pull against mechanical detent to verify retention.
- Release stem and retract both hands clear for 1.0 s hold.
"""

from __future__ import annotations

import math
import os
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mechkit as mk

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# Palette
STAINLESS = (0.75, 0.77, 0.80)
DARK_STAINLESS = (0.45, 0.47, 0.50)
STAND_STEEL = (0.55, 0.58, 0.62)
PLUNGER_WHITE = (0.92, 0.92, 0.94)

# 1. Stainless Fountain Jar (standard commercial countertop insert)
JAR_XY = (0.260, 0.080)
JAR_W_X = 0.110  # 110 mm width
JAR_D_Y = 0.130  # 130 mm depth
JAR_H = 0.075    # 75 mm exterior height
JAR_THICK = 0.006
JAR_RIM_Z = TOP + JAR_H  # 0.7475 + 0.075 = 0.8225 m
JAR_KG = 1.20

# Jar loop handle (on -Y side, towards right robot arm)
JAR_HANDLE_XY = (JAR_XY[0], JAR_XY[1] - 0.095)               # (0.260, -0.015) m
JAR_HANDLE_Z = TOP + 0.0015 + JAR_H / 2                      # 0.7865 m

# 2. Sanitary Wire Wash Rack
RACK_XY = (0.240, 0.250)
RACK_KG = 0.50
RACK_H = 0.065
SOCKET_DEPTH = 0.050

# 3. Commercial Portion-Control Dispenser Pump
PUMP_KG = 0.380
TUBE_L = 0.065
PUMP_COLLAR_INIT_Z = TOP + RACK_H + 0.002  # 0.8145 m
PUMP_START_POS = (RACK_XY[0], RACK_XY[1], PUMP_COLLAR_INIT_Z)
PUMP_GRIP_INIT_Z = PUMP_COLLAR_INIT_Z + 0.040  # center of 50 mm stem: 0.8545 m

# Grasps
STEM_PINCH = FINGER_CLOSED
JAR_PINCH = FINGER_CLOSED

# Park poses (within reach of YAM arms)
LEFT_PARK = (0.160, 0.280, 0.920)
RIGHT_PARK = (0.180, -0.260, 0.920)

RELEASE_TCP_CLEAR = 0.035
RELEASE_HULL_CLEAR = 0.005


def _jar_boxes() -> list[mk.BoxSpec]:
    """Hollow rectangular chamber: floor plate + 4 walls + hollow rim flange + side loop handle."""
    boxes = []
    # Floor plate
    boxes.append(mk.box((JAR_W_X, JAR_D_Y, JAR_THICK), at=(0.0, 0.0, JAR_THICK / 2), color=STAINLESS))
    # Side walls (X-extent = JAR_THICK)
    boxes.append(mk.box((JAR_THICK, JAR_D_Y, JAR_H), at=(-JAR_W_X / 2 + JAR_THICK / 2, 0.0, JAR_H / 2), color=STAINLESS))
    boxes.append(mk.box((JAR_THICK, JAR_D_Y, JAR_H), at=(JAR_W_X / 2 - JAR_THICK / 2, 0.0, JAR_H / 2), color=STAINLESS))
    # Front / Back walls
    boxes.append(mk.box((JAR_W_X - 2 * JAR_THICK, JAR_THICK, JAR_H), at=(0.0, -JAR_D_Y / 2 + JAR_THICK / 2, JAR_H / 2), color=STAINLESS))
    boxes.append(mk.box((JAR_W_X - 2 * JAR_THICK, JAR_THICK, JAR_H), at=(0.0, JAR_D_Y / 2 - JAR_THICK / 2, JAR_H / 2), color=STAINLESS))
    # Hollow flared rim flange (4 outer perimeter strips around open mouth)
    flange_t = 0.008
    flange_w = JAR_W_X + 2 * flange_t
    flange_d = JAR_D_Y + 2 * flange_t
    boxes.append(mk.box((flange_t, flange_d, 0.006), at=(-JAR_W_X / 2 - flange_t / 2, 0.0, JAR_H - 0.003), color=DARK_STAINLESS))
    boxes.append(mk.box((flange_t, flange_d, 0.006), at=(JAR_W_X / 2 + flange_t / 2, 0.0, JAR_H - 0.003), color=DARK_STAINLESS))
    boxes.append(mk.box((JAR_W_X, flange_t, 0.006), at=(0.0, -JAR_D_Y / 2 - flange_t / 2, JAR_H - 0.003), color=DARK_STAINLESS))
    boxes.append(mk.box((JAR_W_X, flange_t, 0.006), at=(0.0, JAR_D_Y / 2 + flange_t / 2, JAR_H - 0.003), color=DARK_STAINLESS))
    # Welded side loop handle on -Y face (grip bar + 2 standoffs)
    h_bar_len = 0.060
    h_ext = 0.035
    h_thick = 0.010
    # Standoffs
    boxes.append(mk.box((h_thick, h_ext, h_thick), at=(-h_bar_len / 2 + h_thick / 2, -JAR_D_Y / 2 - h_ext / 2, JAR_H / 2), color=STAINLESS))
    boxes.append(mk.box((h_thick, h_ext, h_thick), at=(h_bar_len / 2 - h_thick / 2, -JAR_D_Y / 2 - h_ext / 2, JAR_H / 2), color=STAINLESS))
    # Outer grip bar
    boxes.append(mk.box((h_bar_len, h_thick, h_thick), at=(0.0, -JAR_D_Y / 2 - h_ext + h_thick / 2, JAR_H / 2), color=DARK_STAINLESS))
    return boxes


def _rack_boxes() -> list[mk.BoxSpec]:
    """Sanitary wire wash rack with a hollow vertical socket holding the pick-up tube."""
    wall_t = 0.008
    sock_w = 0.046
    sock_d = 0.046
    boxes = []
    # Weighted base
    boxes.append(mk.box((0.140, 0.140, RACK_H - SOCKET_DEPTH), at=(0.0, 0.0, (RACK_H - SOCKET_DEPTH) / 2), color=STAND_STEEL))
    # 4 hollow socket side walls (forming an open vertical tube)
    boxes.append(mk.box((wall_t, sock_d, SOCKET_DEPTH), at=(-sock_w / 2 + wall_t / 2, 0.0, RACK_H - SOCKET_DEPTH / 2), color=STAND_STEEL))
    boxes.append(mk.box((wall_t, sock_d, SOCKET_DEPTH), at=(sock_w / 2 - wall_t / 2, 0.0, RACK_H - SOCKET_DEPTH / 2), color=STAND_STEEL))
    boxes.append(mk.box((sock_w - 2 * wall_t, wall_t, SOCKET_DEPTH), at=(0.0, -sock_d / 2 + wall_t / 2, RACK_H - SOCKET_DEPTH / 2), color=STAND_STEEL))
    boxes.append(mk.box((sock_w - 2 * wall_t, wall_t, SOCKET_DEPTH), at=(0.0, sock_d / 2 - wall_t / 2, RACK_H - SOCKET_DEPTH / 2), color=STAND_STEEL))
    return boxes


def _pump_boxes() -> list[mk.BoxSpec]:
    """Commercial portion-control condiment dispenser pump assembly.
    Collar is at local Z = 0. Pick-up tube projects downward (Z in [-TUBE_L, 0.0]).
    Plunger stem and portion knob project upward (Z in [0.0, 0.080]).
    """
    boxes = []
    # Pick-up cylinder (downward into socket or jar cavity)
    boxes.append(mk.box((0.024, 0.024, TUBE_L), at=(0.0, 0.0, -TUBE_L / 2), color=STAINLESS))
    # Rectangular lid collar (Z in [0.0, 0.010])
    boxes.append(mk.box((JAR_W_X + 0.008, JAR_D_Y + 0.008, 0.010), at=(0.0, 0.0, 0.005), color=STAINLESS))
    # Vertical grip stem (Z in [0.010, 0.065])
    boxes.append(mk.box((0.022, 0.022, 0.055), at=(0.0, 0.0, 0.0375), color=STAINLESS))
    # Portion knob / plunger (Z in [0.065, 0.080])
    boxes.append(mk.box((0.044, 0.044, 0.015), at=(0.0, 0.0, 0.0725), color=PLUNGER_WHITE))
    # Collar locking detent tab (projects forward)
    boxes.append(mk.box((0.014, 0.020, 0.010), at=(-JAR_W_X / 2 - 0.004, 0.0, 0.005), color=DARK_STAINLESS))
    return boxes


kit = mk.Kit()

# Register parts
kit.part("fountain_jar", (JAR_XY[0], JAR_XY[1], TOP + 0.0015), _jar_boxes(), mass=JAR_KG, color=STAINLESS, solver_position_iterations=16)
kit.part("wire_rack", (RACK_XY[0], RACK_XY[1], TOP), _rack_boxes(), mass=None, color=STAND_STEEL)
kit.part("condiment_pump", PUMP_START_POS, _pump_boxes(), mass=PUMP_KG, color=STAINLESS, solver_position_iterations=16)

FIXTURES = []
OBJECTS = kit.objects()

_TEST_TCPS: list | None = None
_TEST_ROBOT_BOXES: list | None = None


# -- Scorer logic via kit.on_step --------------------------------------------------------------------------------

def _all_finite(objects) -> bool:
    for name in ("fountain_jar", "condiment_pump"):
        st = objects.get(name)
        if st is None:
            return False
        if not (
            np.all(np.isfinite(st.pos))
            and np.all(np.isfinite(st.quat_xyzw))
            and np.all(np.isfinite(st.lin_vel))
            and np.all(np.isfinite(st.ang_vel))
        ):
            return False
    return True


def hands_clear(objects) -> bool:
    targets = [objects["fountain_jar"], objects["condiment_pump"]]
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


def _step_rules(objects) -> None:
    step_id = mk._EP["step"]
    if mk.MEMORY.get("_last_step") == step_id and step_id != 0:
        return
    mk.MEMORY["_last_step"] = step_id

    if not _all_finite(objects):
        mk.MEMORY["invalid_state"] = True
        return

    # Exact origin tracking via kit.point
    collar_pt = kit.point(objects, "condiment_pump", (0.0, 0.0, 0.0))
    tube_tip_pt = kit.point(objects, "condiment_pump", (0.0, 0.0, -TUBE_L))
    jar_center_pt = kit.point(objects, "fountain_jar", (0.0, 0.0, 0.0))
    jar_rim_pt = kit.point(objects, "fountain_jar", (0.0, 0.0, JAR_H))

    c_x, c_y, c_z = float(collar_pt[0]), float(collar_pt[1]), float(collar_pt[2])
    j_x, j_y, j_z = float(jar_center_pt[0]), float(jar_center_pt[1]), float(jar_center_pt[2])
    rim_z = float(jar_rim_pt[2])
    tube_z = float(tube_tip_pt[2])

    # Check jar displacement from starting anchor
    j_disp = math.hypot(j_x - JAR_XY[0], j_y - JAR_XY[1])
    if j_disp > 0.035:
        mk.MEMORY["jar_fault"] = True
        return

    # Check dropped pump
    if c_z < TOP + 0.020 and math.hypot(c_x - JAR_XY[0], c_y - JAR_XY[1]) > 0.120 and math.hypot(c_x - RACK_XY[0], c_y - RACK_XY[1]) > 0.100:
        mk.MEMORY["drop_fault"] = True
        return

    # Stage 1: Lift pump clear of rack socket
    if c_z > PUMP_COLLAR_INIT_Z + 0.030:
        mk.MEMORY["lifted"] = True

    # Relative alignment over jar rim
    r_xy = math.hypot(c_x - j_x, c_y - j_y)

    # Stage 2: Pick-up tube entered jar bore
    if mk.MEMORY.get("lifted") and r_xy < 0.035 and tube_z < rim_z - 0.010:
        mk.MEMORY["entered"] = True

    # Stage 3: Collar seated flush on jar rim
    gap_z = abs(c_z - rim_z)
    if mk.MEMORY.get("entered") and r_xy < 0.025 and gap_z < 0.015:
        mk.MEMORY["seated"] = True

    # Stage 4: Collar locking detent engaged
    if mk.MEMORY.get("seated") and (c_x - j_x) < -0.002:
        mk.MEMORY["locked"] = True

    # Stage 5: Proof pull sustained
    if mk.MEMORY.get("locked"):
        mk.MEMORY["proof_steps"] = mk.MEMORY.get("proof_steps", 0) + 1

    if step_id % 20 == 0 or step_id in (1, 10, 50, 100, 150, 200, 250, 300, 320):
        faults = [k for k in ("invalid_state", "jar_fault", "drop_fault") if mk.MEMORY.get(k)]
        print(f"[STEP {step_id:04d}] score={score(objects):.2f} cz={c_z:.4f} collar=({c_x:.3f},{c_y:.3f}) jar=({j_x:.3f},{j_y:.3f}) r_xy={r_xy:.4f} gap_z={gap_z:.4f} tube_z={tube_z:.4f} rim_z={rim_z:.4f} lifted={mk.MEMORY.get('lifted')} entered={mk.MEMORY.get('entered')} seated={mk.MEMORY.get('seated')} locked={mk.MEMORY.get('locked')} faults={faults}", flush=True)


kit.on_step(_step_rules)


def failed(objects) -> bool:
    return bool(mk.MEMORY.get("invalid_state") or mk.MEMORY.get("jar_fault") or mk.MEMORY.get("drop_fault"))


def score(objects) -> float:
    if not _all_finite(objects) or failed(objects):
        return 0.0

    if not mk.MEMORY.get("lifted"):
        return 0.0

    if not mk.MEMORY.get("entered"):
        return 0.20

    if not mk.MEMORY.get("seated"):
        return 0.40

    if not mk.MEMORY.get("locked"):
        return 0.65

    # If locked, check if hands are clear and proof held
    still = objects["condiment_pump"].resting() and objects["fountain_jar"].resting()
    if mk.MEMORY.get("proof_steps", 0) >= 10 and still and hands_clear(objects):
        return 1.00

    return 0.85


def success(objects) -> bool:
    return score(objects) >= 1.0 - 1e-9


task_score = score
task_success = success


# -- Policy demonstration ----------------------------------------------------------------------------------------

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

    jx, jy = JAR_XY
    rx, ry = RACK_XY
    hx, hy = JAR_HANDLE_XY

    carry_z = 0.940  # generous clearance above both rack socket and jar rim

    # Right arm: steady the stainless fountain jar side loop handle at (0.260, -0.015, 0.7865)
    add("right", 1.2, (hx, hy, JAR_HANDLE_Z + 0.040), FINGER_OPEN, "approach jar handle")
    add("right", 0.6, (hx, hy, JAR_HANDLE_Z), FINGER_OPEN, "descend to jar handle")
    add("right", 0.4, (hx, hy, JAR_HANDLE_Z), JAR_PINCH, "pinch jar handle")

    # Left arm: approach and pick up pump from wash rack
    add("left", 1.0, (rx, ry, PUMP_GRIP_INIT_Z + 0.040), FINGER_OPEN, "approach rack above pump")
    add("left", 0.6, (rx, ry, PUMP_GRIP_INIT_Z), FINGER_OPEN, "descend onto pump stem")
    add("left", 0.5, (rx, ry, PUMP_GRIP_INIT_Z), STEM_PINCH, "grasp pump stem")
    # Lift vertically clear of the 50 mm socket
    add("left", 1.2, (rx, ry, carry_z), STEM_PINCH, "lift pump clear of rack socket")

    if variant == "no_insert":
        add("left", 2.0, (rx, ry, carry_z), STEM_PINCH, "hold pump above rack")
        end_t = max(clock.values()) + 1.0
        for arm in ("left", "right"):
            wait_until(arm, end_t, "finish")
        return plans

    # Transfer pump above fountain jar
    add("left", 1.5, (jx, jy, carry_z), STEM_PINCH, "coaxial align over jar")

    # Insert pick-up tube down into jar bore
    add("left", 1.2, (jx, jy, JAR_RIM_Z + 0.045), STEM_PINCH, "insert tube halfway")

    if variant == "no_seat":
        add("left", 2.0, (jx, jy, JAR_RIM_Z + 0.045), STEM_PINCH, "hold suspended in bore")
        end_t = max(clock.values()) + 1.0
        for arm in ("left", "right"):
            wait_until(arm, end_t, "finish")
        return plans

    # Seat lid collar flush on jar rim (collar sits on rim at JAR_RIM_Z, stem grip at JAR_RIM_Z + 0.040)
    add("left", 0.8, (jx, jy, JAR_RIM_Z + 0.040), STEM_PINCH, "seat collar flush on rim")

    if variant == "no_lock":
        add("left", 2.0, (jx, jy, JAR_RIM_Z + 0.040), STEM_PINCH, "dwell seated without latch")
        end_t = max(clock.values()) + 1.0
        for arm in ("left", "right"):
            wait_until(arm, end_t, "finish")
        return plans

    # Press collar locking tab into detent slot
    add("left", 0.6, (jx - 0.008, jy, JAR_RIM_Z + 0.038), STEM_PINCH, "press locking tab forward into detent")
    add("left", 0.4, (jx, jy, JAR_RIM_Z + 0.040), STEM_PINCH, "latch detent engaged")

    # Upward verification proof pull (lift with >=5.0 N upward force against mechanical detent)
    add("left", 0.8, (jx, jy, JAR_RIM_Z + 0.042), STEM_PINCH, "apply 5 N upward proof pull")
    add("left", 0.5, (jx, jy, JAR_RIM_Z + 0.040), STEM_PINCH, "proof sustained")

    # Release pump stem and retract left arm
    add("left", 0.6, (jx, jy, JAR_RIM_Z + 0.040), FINGER_OPEN, "release pump stem")
    add("left", 0.8, (jx, jy, JAR_RIM_Z + 0.090), FINGER_OPEN, "retract left arm")
    add("left", 1.2, LEFT_PARK, FINGER_OPEN, "park left arm")

    # Right arm release jar and retract
    wait_until("right", clock["left"] - 1.5, "hold jar through proof")
    add("right", 0.6, (hx, hy, JAR_HANDLE_Z + 0.040), FINGER_OPEN, "release jar handle")
    add("right", 1.2, RIGHT_PARK, FINGER_OPEN, "park right arm")

    # Stillness hold: both arms parked clear
    end_t = max(clock.values()) + 1.8
    for arm in ("left", "right"):
        wait_until(arm, end_t, "stillness hold")

    return plans
