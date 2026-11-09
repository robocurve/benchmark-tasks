# Interlocked Drawer Transfer (`interlocked-drawer-transfer`)

A dual-drawer cabinet equipped with a mechanical anti-tip interlock mechanism. In commercial filing cabinets and heavy tool chests, opening multiple drawers simultaneously shifts the center of gravity forward, creating a tipping hazard. The interlock prevents both drawers from being open simultaneously. The robot must transfer a white block from the left drawer to the right drawer, closing the left drawer before the right drawer is opened. Opening both drawers simultaneously (> 15 mm each) for >= 0.2 s triggers a catastrophic **TIP FAULT**, illuminating the red TIP indicator lamp and permanently capping the score.

## Instruction
"Move the white block from the left drawer to the right drawer, never letting both drawers be open at the same time."

## Objects and Physical Layout
- **Cabinet Enclosure**: 150 (x) x 400 (y) x 80 (z) mm fixture plate centered at `(0.365, 0.000, 0.7875)` with front face at `x = 0.290 m`.
- **Sliding Drawers**:
  - Two side-by-side open-top drawer bays:
    - Left drawer at `y = +0.120 m`.
    - Right drawer at `y = -0.120 m`.
  - Interior cavity: 130 (x) x 110 (y) x 60 (z) mm each.
  - Friction-held sliding along -X (toward robot), 110 mm travel (0.110 m), spring rate 0.02 N/m, damping 10.0 N s/m.
  - Front pull handle: 20 (x) x 45 (y) x 30 (z) mm centered on drawer front face (`x = 0.280 m` at rest).
- **White Block**: 40 x 40 x 40 mm cube (mass 0.050 kg, friction 1.5). Starts at rear floor of left drawer (`x = 0.400, y = +0.120, z = 0.7815`). Covered by cabinet top rear cover until drawer is pulled open >= 70 mm.
- **Staging Pad**: 60 x 60 x 4 mm marked pad at cabinet top front-centre (`x = 0.310, y = 0.000, z = 0.8295`).
- **Indicator Lamps**: Mounted on a crossbar at `x = 0.420 m`:
  - Green "OK" lamp (25 mm diameter) at `y = +0.035 m`.
  - Red "TIP" lamp (25 mm diameter) at `y = -0.035 m`.

## Solution Sequence
1. Left arm pulls left drawer open >= 70 mm (achieved 85 mm).
2. Left arm grasps white block and lifts it clear out of drawer.
3. Left arm places block on cabinet top staging pad (`0.310, 0.000, 0.8295`) and releases.
4. Left arm pushes left drawer closed (<= 5 mm) and returns home.
5. Right arm pulls right drawer open >= 70 mm (achieved 85 mm).
6. Right arm grasps block from staging pad.
7. Right arm places block inside right drawer cavity.
8. Right arm pushes right drawer closed (<= 5 mm) and returns home.
9. Both arms settle still (held >= 1.0 s).

## Timing Margin Table
| Requirement | Specification | Scripted Arm Achieved | Safety Margin |
|---|---|---|---|
| Left Drawer Open Stroke | >= 70 mm | 95 mm stroke (+0.6 s hold) | +25 mm (+35.7%) |
| Block Lift Clear Height | Above rim (>= 0.8215 m) | 0.8565 m | +35 mm |
| Left Drawer Closed Threshold | <= 5 mm | 0.0 mm (+10 mm overshoot, 0.4 s hold) | +5 mm |
| Right Drawer Open Stroke | >= 70 mm | 95 mm stroke (+0.6 s hold) | +25 mm (+35.7%) |
| Staging Grasp Height | Clear of pad/lip | +12 mm above center (0.8635 m) | Palm clear |
| Right Drawer Closed Threshold | <= 5 mm | 0.0 mm (+10 mm overshoot, 0.4 s hold) | +5 mm |
| Anti-tip Interlock Time | >= 0.20 s crossing | 0.0 s overlap | Inf margin |
| Success Hold Duration | >= 1.0 s (20 steps) | 2.0 s (40 steps) | +1.0 s (+100%) |

## Contact and Kinematic Clearances
- **Kinematic Reach Margins**: Reach verification script declared in `check_reach.py` (target joint margin >= 0.02 rad, position error <= 3.0 mm, rotation error <= 3.0 deg).
  - Measured worst position error: **0.10 mm** (spec <= 3.0 mm)
  - Measured worst rotation error: **0.01 deg** (spec <= 3.0 deg)
  - Measured minimum joint margin: **0.228 rad** (spec >= 0.02 rad)
  - Status: **PASSED across all 7 modes** (`demo`, `park`, `both_open`, `leave_open`, `wrong_drawer`, `block_dropped`, `settle`).
- **Arm-Arm Clearance**: Script declared in `check_collisions.py` (target clearance >= 30.0 mm).
  - Measured minimum arm-arm clearance: **257.45 mm** at t = 30.20 s (`L:tip_left vs R:tip_right`).
  - Status: **PASSED** (spec >= 30.0 mm).
- **Scene Clearances**: All unintended scene clearances target >= 1.0 mm with intended contacts declared by label.
  - Measured minimum scene clearance: **1.40 mm**.
  - Unintended collisions: **0**.
  - Status: **ALL CLEARANCES SATISFIED**.

## Scoring Ladder
- **0.25**: Left drawer opened >= 70 mm and block lifted out clear above drawer rim.
- **0.50**: Left drawer closed again (<= 5 mm) while block is held outside or parked on staging pad.
- **0.75**: Right drawer opened >= 70 mm.
- **1.00**: Block resting inside right drawer, both drawers closed (<= 5 mm), grippers open, arms still and clear of cabinet, no TIP fault, held for 1.0 s (20 steps at 20 Hz).

## Zero-Gates
- **TIP FAULT**: Both drawers open (> 15 mm) simultaneously for >= 0.2 s triggers TIP FAULT, illuminating the red TIP indicator lamp and capping score at 0.25 permanently.
- **Dropped Block**: the white block falling below the table plane (`z < TOP - 0.05 m`) triggers the drop zero-gate, locking the score to 0.0. A block dropped onto the table surface does not trigger it (measured, see below).

## Modes and Predicted Scores
- `demo`: 1.00 (correct bimanual carry transfer)
- `park`: 1.00 (correct staging pad transfer)
- `both_open`: 0.25 (right drawer opened while left open -> TIP fault)
- `leave_open`: 0.75 (right drawer left open at end)
- `wrong_drawer`: 0.25 (block returned to the left drawer; the left drawer cannot be closed again with the block inside, so the score stays at the first milestone)
- `block_dropped`: 0.25 (block dropped onto the table beside the cabinet: it lands on the table surface, so the below-table-plane gate does not fire and the first milestone stays)
- `settle`: 0.00 (idle hold at home)
- `dream`: 0.00 (baseline dream policy)

## Measured in the simulator (GPU, 20 Hz)

| Mode | Measured | Note |
|---|---|---|
| `demo` | 1.00 | carry transfer, both drawers closed, held 1.0 s |
| `park` | 1.00 | staging-pad transfer |
| `both_open` | 0.25 | TIP fault (`tip_fault`), score capped |
| `leave_open` | 0.75 | right drawer left open at the end |
| `wrong_drawer` | 0.25 | predicted 0.50: the left drawer stalls at ~64 mm and cannot be closed with the block inside |
| `block_dropped` | 0.25 | predicted 0.00: the block lands on the table, not below it |
| `settle` | 0.00 | idle hold |
| `dream` | 0.00 | Dream-1 baseline |

Changes after the first GPU runs: slider spring rates are 0.02 N/m (the simulator rejects 0), drawer pull stroke 95 mm
(the drawer opens ~77 mm because the arm is soft sideways), closing push overshoots 30 mm and holds 1.0 s, the handle
is grasped at the measured open position (`DEMO_DRAWER_ACTUAL_OPEN`, 77 mm), and the fingers lift straight up off the
handle before backing away.
