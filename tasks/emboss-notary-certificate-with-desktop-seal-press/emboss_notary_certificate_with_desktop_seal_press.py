"""Emboss notary certificate with desktop seal press.

BLS SOC 23-2011: Paralegals and Legal Assistants
BLS SOC 43-6012: Legal Secretaries and Administrative Assistants
BLS SOC 43-6014: Secretaries and Administrative Assistants

Universal 1:1 legal office certification task: official authentication of legal deeds,
affidavits, and certificates using a heavy-duty cast-iron desktop seal embosser press
clamped to the workstation bench.
- Left gripper grasps stiff 300 gsm notary certificate card by its trailing tab, slides
  it along the integrated precision document bed into the press throat against the
  mechanical depth stop, and holds it aligned over the lower die platen.
- Right gripper grasps the lever handle, depresses it from 35 deg down to seated crimp
  position against calibrated friction resistance, holds the crimp for dwell, and lifts
  the lever back up to 35 deg to release the embossed certificate.
- Left gripper releases certificate and both arms retract clear for a 1.0 s stability dwell.
"""

from __future__ import annotations

import math
import os
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mechkit as mk

from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN, Waypoint
from yam_arena.task_file import TABLE_TOP

SCENE = "station"
TOP = TABLE_TOP[SCENE]
BASE_Z = TOP + 0.003

# Color palette: heavy cast iron black enamel, brass/chrome fittings, ivory certificate
PRESS_BLACK = (0.12, 0.13, 0.15)
LEVER_CHROME = (0.75, 0.77, 0.82)
KNOB_BLACK = (0.08, 0.08, 0.09)
DIE_BRASS = (0.78, 0.64, 0.32)
CARD_IVORY = (0.94, 0.92, 0.84)
GOLD_SEAL = (0.85, 0.72, 0.28)
RUBBER_BLACK = (0.10, 0.10, 0.10)

RELEASE_TCP_CLEAR = 0.045
RELEASE_HULL_CLEAR = 0.010

LEFT_PARK = (0.180, 0.220, TOP + 0.220)
RIGHT_PARK = (0.180, -0.220, TOP + 0.220)

PRESS_CENTER_XY = (0.280, -0.010)


def _press_base_boxes() -> list[mk.BoxSpec]:
    boxes = []
    # Heavy cast iron platen base bed with integrated document feed table
    # (width 100 mm along X, length 280 mm along Y from -0.060 to +0.220, height 12 mm)
    boxes.append(mk.box((0.100, 0.280, 0.012), at=(0.0, 0.080, 0.006), color=PRESS_BLACK))
    # 4 non-slip rubber bench pads
    for fx in (-0.035, 0.035):
        for fy in (-0.040, 0.200):
            boxes.append(mk.box((0.018, 0.018, 0.004), at=(fx, fy, -0.002), color=RUBBER_BLACK))
    # Rear structural vertical column spine behind throat (height 120 mm, local Y = -0.055)
    boxes.append(mk.box((0.070, 0.050, 0.120), at=(0.0, -0.055, 0.072), color=PRESS_BLACK))
    # Upper overhanging jaw platen housing die drive (extends over throat from -0.080 to +0.050)
    boxes.append(mk.box((0.070, 0.130, 0.024), at=(0.0, -0.015, 0.120), color=PRESS_BLACK))
    # Lower die platen disc insert (brass finish, flush-inset into bed surface at Y = 0.0)
    boxes.append(mk.box((0.046, 0.046, 0.002), at=(0.0, 0.0, 0.011), color=DIE_BRASS))
    # Throat depth stop rib at back of opening (front face at local Y = -0.020)
    boxes.append(mk.box((0.070, 0.010, 0.020), at=(0.0, -0.025, 0.022), color=PRESS_BLACK))
    return boxes


def _lever_boxes() -> list[mk.BoxSpec]:
    boxes = []
    # Main lever beam extending towards -Y (length 160 mm, from 0 to -0.160)
    boxes.append(mk.box((0.024, 0.160, 0.016), at=(0.0, -0.080, 0.0), color=LEVER_CHROME))
    # Upper die crimp stamper head under beam (height 108 mm, reaches card at q=0)
    # Pivot is at press local Y = +0.020; die is at press local Y = 0.0 (lever local Y = -0.020)
    boxes.append(mk.box((0.044, 0.044, 0.108), at=(0.0, -0.020, -0.062), color=DIE_BRASS))
    # Graspable palm handle at lever tip (lever local Y = -0.160)
    boxes.append(mk.box((0.028, 0.020, 0.040), at=(0.0, -0.160, 0.020), color=KNOB_BLACK))
    boxes.append(mk.box((0.040, 0.030, 0.012), at=(0.0, -0.160, 0.040), color=KNOB_BLACK))
    return boxes


def _card_boxes() -> list[mk.BoxSpec]:
    boxes = []
    # Stiff 300 gsm legal certificate card (120 mm in Y, 80 mm in X, 1.5 mm thickness)
    boxes.append(mk.box((0.080, 0.120, 0.0015), at=(0.0, 0.0, 0.0008), color=CARD_IVORY))
    # Gold foil notary seal target patch centered at local Y = -0.040
    boxes.append(mk.box((0.040, 0.040, 0.0015), at=(0.0, -0.040, 0.0008), color=GOLD_SEAL))
    # Trailing grip pinch tab for left gripper at local Y = +0.045
    boxes.append(mk.box((0.028, 0.018, 0.045), at=(0.0, 0.045, 0.0225), color=CARD_IVORY))
    return boxes


kit = mk.Kit()

# Heavy stationary press base clamped to benchtop
kit.part("press_base", (PRESS_CENTER_XY[0], PRESS_CENTER_XY[1], BASE_Z), _press_base_boxes(),
         mass=None, color=PRESS_BLACK, solver_position_iterations=16)

# Lever handle: in reference pose (q=0), lever beam is horizontal at Z = BASE_Z + 0.130.
# Anchor is at rear pivot: (PRESS_CENTER_XY[0], PRESS_CENTER_XY[1] + 0.020, BASE_Z + 0.130).
lever_anchor = (PRESS_CENTER_XY[0], PRESS_CENTER_XY[1] + 0.020, BASE_Z + 0.130)
kit.part("lever_arm", lever_anchor, _lever_boxes(),
         mass=0.18, color=LEVER_CHROME, solver_position_iterations=16)

# Revolute hinge with internal friction holding lever propped against gravity:
# Axis = (-1, 0, 0) so positive q elevates handle in air (+Z).
# Rest pose q=35.0 deg (elevated in air), lower stop q=0.0 deg (pressed flat onto crimp die).
# Joint friction = 0.40 N*m holds lever securely propped at 35 deg without sagging.
kit.hinge("lever_hinge", "press_base", "lever_arm", anchor=lever_anchor, axis=(-1, 0, 0),
          lower_deg=0.0, upper_deg=36.0, start_deg=35.0, friction=0.40)

# Staged certificate card resting on the integrated document feed bed at Y = +0.140
CARD_START_XY = (PRESS_CENTER_XY[0], PRESS_CENTER_XY[1] + 0.150)
kit.part("certificate_card", (CARD_START_XY[0], CARD_START_XY[1], BASE_Z + 0.0125), _card_boxes(),
         mass=0.08, color=CARD_IVORY, solver_position_iterations=16)

FIXTURES = []
OBJECTS = kit.objects()

_TEST_TCPS: list | None = None
_TEST_ROBOT_BOXES: list | None = None


def _all_finite(objects) -> bool:
    for name in ("press_base", "lever_arm", "certificate_card"):
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
    all_tcps = mk.tcps()
    if all_tcps:
        for tcp in all_tcps:
            for name in ("press_base", "lever_arm", "certificate_card"):
                st = objects[name]
                lo = np.asarray(st.low) - RELEASE_TCP_CLEAR
                hi = np.asarray(st.high) + RELEASE_TCP_CLEAR
                if np.all(tcp >= lo) and np.all(tcp <= hi):
                    return False
        return True

    if _TEST_TCPS is not None:
        for tcp in _TEST_TCPS:
            pt = kit.point(objects, "lever_arm", (0.0, -0.160, 0.020))
            if float(np.linalg.norm(np.asarray(tcp) - pt)) < RELEASE_TCP_CLEAR:
                return False

    if _TEST_ROBOT_BOXES is not None:
        for r_pos, r_size in _TEST_ROBOT_BOXES:
            pt = kit.point(objects, "lever_arm", (0.0, -0.160, 0.020))
            gap = np.maximum(0.0, np.abs(np.asarray(r_pos) - pt) - np.asarray(r_size) / 2)
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

    card_st = objects["certificate_card"]
    cx, cy = float(card_st.pos[0]), float(card_st.pos[1])

    # Card insertion progress: staged at Y = +0.140, inserted into throat along -Y
    card_disp_y = CARD_START_XY[1] - cy
    if card_disp_y > 0.040 and abs(cx - PRESS_CENTER_XY[0]) < 0.030:
        mk.MEMORY["card_advanced"] = True

    # Card seated squarely against throat depth stop: target cy = PRESS_CENTER_XY[1] + 0.040 = +0.030
    if mk.MEMORY.get("card_advanced") and abs(cy - (PRESS_CENTER_XY[1] + 0.040)) < 0.020:
        mk.MEMORY["card_seated"] = True

    # Lever angle q in degrees (starts at 35 deg, depressed down to <= 6 deg)
    q_deg = math.degrees(kit.q(objects, "lever_hinge"))

    # Stage 3: Lever stroke initiated while card is seated
    if mk.MEMORY.get("card_seated") and q_deg < 22.0:
        mk.MEMORY["lever_stroke_mid"] = True

    # Stage 4: Full crimp compression stroke reached (q <= 6.0 deg)
    if mk.MEMORY.get("lever_stroke_mid") and q_deg <= 6.0:
        mk.MEMORY["seal_crimped"] = True

    # Stage 5: Stillness hold after release with hands retracted clear
    if mk.MEMORY.get("seal_crimped") and hands_clear(objects):
        mk.MEMORY["hold_steps"] = mk.MEMORY.get("hold_steps", 0) + 1

    if step_id % 20 == 0 or step_id in (1, 10, 50, 100, 150, 200, 250):
        faults = [k for k in ("invalid_state", "base_slip_fault") if mk.MEMORY.get(k)]
        print(f"[STEP {step_id:04d}] score={score(objects):.2f} q_deg={q_deg:.1f} c_y={cy:.4f} "
              f"adv={mk.MEMORY.get('card_advanced')} seated={mk.MEMORY.get('card_seated')} "
              f"mid={mk.MEMORY.get('lever_stroke_mid')} crimp={mk.MEMORY.get('seal_crimped')} "
              f"hold={mk.MEMORY.get('hold_steps', 0)} faults={faults}", flush=True)


kit.on_step(_step_rules)


def failed(objects) -> bool:
    return bool(mk.MEMORY.get("invalid_state"))


def score(objects) -> float:
    if not _all_finite(objects) or failed(objects):
        return 0.0

    if not mk.MEMORY.get("card_advanced"):
        return 0.0

    if not mk.MEMORY.get("card_seated"):
        return 0.20

    if not mk.MEMORY.get("lever_stroke_mid"):
        return 0.40

    if not mk.MEMORY.get("seal_crimped"):
        return 0.65

    if mk.MEMORY.get("hold_steps", 0) < 20:
        return 0.85

    return 1.00


def success(objects) -> bool:
    return score(objects) >= 1.0 - 1e-9


task_score = score
task_success = success


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

    # 1. Left arm: approach certificate grip tab at (CARD_START_XY[0], CARD_START_XY[1] + 0.045, BASE_Z + 0.035)
    c_tab_x = CARD_START_XY[0]
    c_tab_y = CARD_START_XY[1] + 0.045
    c_tab_z = BASE_Z + 0.035

    add("left", 1.2, (c_tab_x, c_tab_y, c_tab_z + 0.060), FINGER_OPEN, "hover above card tab")
    add("left", 0.8, (c_tab_x, c_tab_y, c_tab_z), FINGER_OPEN, "descend onto card tab")
    add("left", 0.5, (c_tab_x, c_tab_y, c_tab_z), FINGER_CLOSED, "pinch certificate card tab")

    if variant == "no_advance":
        add("left", 2.0, (c_tab_x, c_tab_y, c_tab_z), FINGER_CLOSED, "hold staged")
        end_t = max(clock.values()) + 1.0
        for arm in ("left", "right"):
            wait_until(arm, end_t, "finish")
        return plans

    # Slide card into press throat along the document feed bed into mechanical stop rib
    # Target card center is at PRESS_CENTER_XY[1] + 0.040 = +0.030
    # Tab seated world Y: +0.030 + 0.045 = +0.075
    seat_y = PRESS_CENTER_XY[1] + 0.040 + 0.045
    # Lift 2 mm during slide to eliminate bed friction drag
    add("left", 1.5, (c_tab_x, seat_y + 0.010, c_tab_z + 0.002), FINGER_CLOSED, "slide card towards stop")
    add("left", 0.6, (c_tab_x, seat_y, c_tab_z), FINGER_CLOSED, "seat squarely into stop rib")
    add("left", 0.8, (c_tab_x, seat_y, c_tab_z), FINGER_CLOSED, "hold card firmly registered")

    # 2. Right arm: approach lever handle at elevated rest pose (q=35 deg)
    # Lever anchor is at (PRESS_CENTER_XY[0], PRESS_CENTER_XY[1] + 0.020, BASE_Z + 0.130)
    # Handle at q=35 deg: rel Y = -0.1196, rel Z = +0.1082
    knob_x = PRESS_CENTER_XY[0]
    knob_y_start = (PRESS_CENTER_XY[1] + 0.020) - 0.1196
    knob_z_start = BASE_Z + 0.130 + 0.1082

    add("right", 1.5, (knob_x, knob_y_start, knob_z_start + 0.060), FINGER_OPEN, "hover over lever handle")
    add("right", 0.8, (knob_x, knob_y_start, knob_z_start), FINGER_OPEN, "descend to handle")
    add("right", 0.5, (knob_x, knob_y_start, knob_z_start), FINGER_CLOSED, "grasp lever handle")

    # Synchronize: ensure card is firmly seated before beginning lever crimp stroke
    wait_until("right", clock["left"], "sync before stroke")

    if variant == "no_stroke":
        add("right", 2.0, (knob_x, knob_y_start, knob_z_start), FINGER_CLOSED, "hold at top")
        end_t = max(clock.values()) + 1.0
        for arm in ("left", "right"):
            wait_until(arm, end_t, "finish")
        return plans

    # Depress lever handle from 35 deg down to crimp stop (q = 0 deg):
    # Mid stroke (q ~ 20 deg): knob at (knob_x, knob_y_start - 0.020, BASE_Z + 0.190)
    add("right", 1.0, (knob_x, knob_y_start - 0.020, BASE_Z + 0.190), FINGER_CLOSED, "press lever mid-stroke")

    # Seated full crimp (q = 0 deg): knob at (knob_x, (PRESS_CENTER_XY[1] + 0.020) - 0.160, BASE_Z + 0.125)
    knob_y_flat = (PRESS_CENTER_XY[1] + 0.020) - 0.160
    knob_z_flat = BASE_Z + 0.125
    add("right", 1.2, (knob_x, knob_y_flat, knob_z_flat), FINGER_CLOSED, "depress to full crimp stop")
    add("right", 1.0, (knob_x, knob_y_flat, knob_z_flat), FINGER_CLOSED, "dwell under crimp compression")

    if variant == "no_release":
        add("right", 2.0, (knob_x, knob_y_flat, knob_z_flat), FINGER_CLOSED, "hold lever down without release")
        end_t = max(clock.values()) + 1.0
        for arm in ("left", "right"):
            wait_until(arm, end_t, "finish")
        return plans

    # Return stroke: right arm lifts lever handle back up to 35 deg to open press throat
    add("right", 1.0, (knob_x, knob_y_start - 0.020, BASE_Z + 0.190), FINGER_CLOSED, "lift lever midway")
    add("right", 1.0, (knob_x, knob_y_start, knob_z_start), FINGER_CLOSED, "return lever to elevated stop")
    add("right", 0.6, (knob_x, knob_y_start, knob_z_start + 0.080), FINGER_OPEN, "release lever handle")
    add("right", 1.2, RIGHT_PARK, FINGER_OPEN, "park right arm")

    # Left arm releases certificate card tab and retracts
    wait_until("left", clock["right"] - 1.2, "hold card steady during crimp cycle")
    add("left", 0.6, (c_tab_x, seat_y, c_tab_z + 0.080), FINGER_OPEN, "release certificate tab")
    add("left", 1.2, LEFT_PARK, FINGER_OPEN, "park left arm")

    # Stillness hold: 2.0 s dwell with both arms parked
    end_t = max(clock.values()) + 2.0
    for arm in ("left", "right"):
        wait_until(arm, end_t, "stillness hold")

    return plans
