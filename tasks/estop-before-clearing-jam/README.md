# Emergency Stop and Jam Clearing (`estop-before-clearing-jam`)

An industrial machine safeguarding and jam recovery task. A pusher machine continuously cycles along a walled channel lane. A workpiece (an orange block) has jammed in the lane. In accordance with machine safeguarding standards (OSHA / ISO 13850), reaching into the lane danger zone while the pusher is cycling constitutes an immediate crushing hazard and results in a safety FAULT. To safely recover, the robot must press the red E-STOP button to arrest the pusher, take the stuck orange block out of the lane into the green tray, and then press the white RESET button followed by the green START button to resume normal cycling.

## Instruction
"Press the red E-STOP so the pusher stops, take the stuck orange block out of the lane, then press RESET and START: never reach into the lane while the pusher is moving."

## Objects and Physical Layout
- **Lane Fixture**: 300 (x) x 150 (y) mm walled channel centered at `(0.340, +0.100, 0.7575)`. Walls are 10 mm thick and 20 mm tall above the 5 mm floor plate, allowing standard robotic grippers to reach over the wall.
- **Danger Zone**: Defined as the entire lane volume: x in `[0.190, 0.490]`, y in `[0.025, 0.175]`, z up to 0.120 m above the table top (`0.8675 m`).
- **Pusher Bar**: 30 x 120 x 25 mm bar sweeping along x over a 90 mm stroke (`x in [0.240, 0.330]`) with a 4.0 s sinusoidal period. The pusher is code-driven and non-colliding (`collision=False`); the safety hazard is enforced strictly via the danger zone rule.
- **Stuck Orange Block**: 40 x 40 x 40 mm rigid cube (0.045 kg, friction 1.5) lying in the lane at `(0.400, +0.100, 0.7725)`.
- **Button Console**: Rigid base plate (240 x 120 x 20 mm) centered at `(0.340, -0.200, 0.7575)` housing three prismatic pushbuttons (8 mm travel, 100 N/m spring):
  - **E-STOP**: Red 60 mm cap at `(0.280, -0.200)`.
  - **RESET**: White 40 mm cap at `(0.340, -0.200)`.
  - **START**: Green 40 mm cap at `(0.400, -0.200)`.
- **Indicator Lamps**: 25 mm diameter visual markers on the console plate:
  - **STOP lamp (red)**: Illuminates when the pusher is safely stopped.
  - **FAULT lamp (amber)**: Illuminates upon any safety sequence violation.
  - **RUN lamp (green)**: Illuminates when the machine is cycling normally.
- **Green Tray**: 130 x 130 x 20 mm receptacle at `(0.300, +0.300, 0.7575)`.
- **Parked Pad**: 100 x 100 x 5 mm staging pad at `(0.420, +0.300, 0.7500)` (supported as alternative valid clearing destination).

## Timing Margin Table
| Requirement | Specification | Scripted Arm Achieved | Safety Margin |
|---|---|---|---|
| E-STOP Press Stroke | >= 6.4 mm (-Z) | 8.0 mm (full stop) | +1.6 mm (+25%) |
| Pusher Stop Window | <= 0.30 s arrest | 0.80 s wait before entry | +0.50 s (+167%) |
| Lane Wall Clearance | 20 mm wall height | Lift to Z = 0.8475 m (+75 mm) | +55 mm (+275%) |
| Block Grip Hold | 40 mm cube | -0.012 m finger target (24 mm gap) | Firm pinch (16 mm squeeze) |
| RESET Press Stroke | >= 6.4 mm (-Z) | 8.0 mm (full stop) | +1.6 mm (+25%) |
| START Press Stroke | >= 6.4 mm (-Z) | 8.0 mm (full stop) | +1.6 mm (+25%) |
| Final Success Hold | >= 2.0 s (40 steps) | 2.2 s (44 steps) | +0.2 s (+10%) |

## Contact and Kinematic Clearances
- **Kinematic Reach Margins**: Reach verification script declared in `check_reach.py` (target joint margin >= 0.02 rad, position error <= 3.0 mm, rotation error <= 3.0 deg).
  - Measured worst position error: **0.09 mm** (spec <= 3.0 mm)
  - Measured worst rotation error: **0.01 deg** (spec <= 3.0 deg)
  - Measured minimum joint margin: **0.228 rad** (spec >= 0.02 rad)
  - Status: **PASSED across all 6 modes** (`demo`, `no_estop`, `start_early`, `no_reset`, `leave_stopped`, `settle`).
- **Arm-Arm Clearance**: Script declared in `check_collisions.py` (target clearance >= 30.0 mm).
  - Measured minimum arm-arm clearance: **289.72 mm** at t = 10.60 s (`L:tip_left vs R:link5`).
  - Status: **PASSED** (spec >= 30.0 mm).
- **Scene Clearances**: All unintended scene clearances target >= 1.0 mm with intended contacts declared by label.
  - Measured minimum scene clearance: **16.69 mm** (`tip_right vs estop button`).
  - Unintended collisions: **0**.
  - Status: **ALL CLEARANCES SATISFIED**.
- **Relay Reach Separation**: Left arm cannot reach E-STOP button (`0.577 m > 0.50 m`); Right arm cannot reach stuck block (`0.569 m > 0.50 m`), strictly enforcing bimanual division of labour.

## Scoring Ladder
- **0.25**: E-STOP counted (pusher stops within 0.3 s, STOP lamp lit, no fault, latched `ever_stopped`).
- **0.50**: Stuck block extracted from lane into green tray without fault (or RESET pressed after E-STOP without fault).
- **0.75**: RESET then START counted in order after block is out of lane (RUN lamp lit, pusher moving).
- **1.00**: Arms clear of danger zone, full success state held for 2.0 s (40 steps at 20 Hz).

## Zero-Gates
- **Moving Lane Hazard**: Any arm TCP entering the danger zone (`x in [0.19, 0.49]`, `y in [0.025, 0.175]`, `z <= TOP + 0.120`) while the pusher is moving triggers an immediate FAULT, latching score to 0.00.
- **Premature Restart**: Pressing START while the orange block is still in the lane triggers an immediate FAULT, latching score to 0.50.

## Modes and Predicted Scores
- `demo`: 1.00 (correct E-STOP -> clear jam -> RESET -> START sequence)
- `no_estop`: 0.00 (left arm reaches into lane while pusher is moving -> FAULT)
- `start_early`: 0.50 (RESET + START pressed while block still in lane -> FAULT)
- `skip_reset`: 0.50 (START pressed without RESET; START ignored, pusher remains stopped)
- `leave_stopped`: 0.50 (pusher stopped, block extracted to tray, never restarted)
- `settle`: 0.00 (arms hold at home, machine cycles)
- `dream`: 0.00 (Dream-1 baseline policy)

## Measured in the simulator (GPU, 20 Hz)

| Mode | Predicted | Measured | What happens |
|---|---|---|---|
| `demo` | 1.00 | 1.00 (matches) | Full E-STOP, jam clear, RESET, START sequence |
| `no_estop` | 0.00 | 0.00 (matches) | Arm enters danger zone while pusher cycling -> FAULT |
| `start_early` | 0.50 | 0.50 (matches) | START pressed before block removed -> FAULT |
| `skip_reset` | 0.50 | 0.50 (matches) | START pressed without RESET -> no restart |
| `leave_stopped` | 0.50 | 0.50 (matches) | Jam cleared but machine left in safe stopped state |
| `settle` | 0.00 | 0.00 (matches) | Arms idle at home |
| `dream` | 0.00 | 0.00 (matches) | Baseline policy |

## Fidelity Notes and Real-World Basis
Honest simulation fidelity:
1. The pusher is driven by Python code as a kinematic non-colliding visual marker rather than an articulated physics slider, sweeping along X over 90 mm with a 4.0 s period.
2. The danger zone is enforced as a strict 3D geometric rule matching the lane interior; physical pinching between pusher and block is abstracted to the danger zone violation rule.
3. Pushbuttons are modeled as physical prismatic sliders with 100 N/m spring return, 8 mm stroke to hard mechanical stop, and viscous damping.
4. Stuck block is an unconstrained physical rigid body with high friction (1.5) and low mass (0.045 kg) to ensure stable grasp and transfer.
5. Real-world basis: OSHA 1910.212 / ISO 13850 emergency stop and jam clearing safety standards.
