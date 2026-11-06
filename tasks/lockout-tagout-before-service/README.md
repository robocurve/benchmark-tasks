# Lockout-Tagout Before Service (`lockout-tagout-before-service`)

A standard electrical lockout-tagout (LOTO) safety procedure. Prior to opening an electrical service enclosure, technicians must switch the main circuit breaker off, physically lock it out by inserting a red lockout pin into the breaker hole, verify zero electrical energy by pressing the TEST button (which illuminates the green DEAD indicator lamp), and only then pull and hold open the spring-return service cover to press the internal blue service button for 1.0 s. Opening the cover while the breaker is live or unpinned triggers an immediate safety FAULT.

## Instruction
"Switch the breaker off, lock it with the red pin, press TEST, then hold the cover open and press the blue service button: never open the cover while the breaker is on."

## Objects and Physical Layout
- **Console Base**: 240 x 540 x 20 mm dark grey fixture plate (10.0 kg, friction 1.0) centered at `(0.300, +0.060, 0.7575)`.
- **Main Circuit Breaker** (left side at `y = +0.180`):
  - Lever slider: 40 mm throw along +X. Starts ON at `x = 0.280 m` (toward robot), pushed OFF to `x = 0.320 m`.
  - 28 mm square lockout pin hole through the plate at `(0.280, +0.180)`: covered by the lever when ON, exposed only when the lever is OFF.
- **Red Lockout Pin**: 16 mm diameter x 50 mm tall cylinder (0.035 kg) starting on pin stand at `(0.220, +0.280)`. Seated when inserted into hole within 4 mm of bottom.
- **TEST Station** (center at `y = +0.035`):
  - White pushbutton (40 x 40 x 12 mm, 8 mm travel, 100 N/m spring) at `(0.300, +0.035)`.
  - 30 mm diameter lamps at `x = 0.360 m`:
    - LIVE lamp (amber): lit while breaker is ON.
    - DEAD lamp (green): illuminates for 3.0 s upon TEST press when breaker is OFF.
- **Service Enclosure** (right side at `y = -0.040`):
  - Spring-return sliding cover: 120 x 80 x 15 mm steel plate with 45 mm handle, 90 mm travel along -X (toward robot) against 18 N/m return spring.
  - Blue SERVICE button: 40 x 40 x 12 mm button located under cover at `(0.300, -0.040)`, accessible only when cover is pulled out >= 70 mm.

## Timing Margin Table
| Requirement | Specification | Scripted Arm Achieved | Safety Margin |
|---|---|---|---|
| Breaker Push Off | >= 32 mm throw | 40 mm + 12 mm overshoot | +20 mm (+62%) |
| Pin Seated Depth | <= 4 mm from bottom | 1.0 mm from bottom | +3.0 mm (+75%) |
| DEAD Lamp Timer | 3.0 s window | 1.5 s to initiate cover pull | +1.5 s (+50%) |
| Cover Pull Hold | >= 70 mm open | 90 mm open | +20 mm (+28%) |
| SERVICE Press Hold | >= 1.0 s (20 steps) | 1.2 s (24 steps) | +0.2 s (+20%) |
| Final Success Hold | >= 1.0 s (20 steps) | 1.5 s (30 steps) | +0.5 s (+50%) |

## Contact and Kinematic Clearances
- **Bimanual Arm-Arm Clearance**: Minimum **45.05 mm** at t = 14.70 s between Left link 5 and Right gripper tip (threshold >= 30.0 mm).
- **Scene Clearances**: All unintended link-to-obstacle clearances >= **16.58 mm** (threshold >= 1.0 mm).
- **Reach Margins**: Minimum joint margin **0.161 rad** (threshold >= 0.02 rad), worst position error **0.09 mm** (limit <= 3.0 mm), worst rotation error **0.01 deg** (limit <= 3.0 deg).
- **Relay Reach Separation**: Right arm to Pin Stand = **0.625 m** (> 0.50 m unreachable), enforcing bimanual coordination.

## Scoring Ladder
- **0.25**: Breaker switched OFF (LIVE amber lamp turns off).
- **0.50**: Red lockout pin seated in breaker hole.
- **0.75**: TEST pressed and verified DEAD (green DEAD lamp illuminates).
- **1.00**: Blue SERVICE button pressed and held >= 1.0 s while cover is held open >= 70 mm, pin still seated, with no FAULT, held for 1.0 s (20 steps at 20 Hz).

## Zero-Gates
- **Safety FAULT**: Opening the service cover >= 20 mm before the breaker is OFF, before the pin is seated, or before TEST verifies DEAD triggers an immediate FAULT. A FAULT permanently prevents achieving success (1.00).

## Modes and Predicted Scores
- `demo`: 1.00 (full correct bimanual sequence)
- `cover_first`: 0.00 (cover opened before breaker switched OFF -> FAULT)
- `no_pin`: 0.25 (breaker OFF -> 0.25, pin skipped, cover opened -> FAULT)
- `no_test`: 0.50 (breaker OFF, pin seated -> 0.50, test skipped, cover opened -> FAULT)
- `no_hold`: 0.75 (breaker OFF, pin seated, test dead -> 0.75, cover released early -> cannot press service)
- `order_swapped`: 0.00 (pin attempted before breaker OFF -> hole blocked by lever -> pin not seated)
- `settle`: 0.00 (idle hold at home)
- `dream`: 0.00 (baseline)


## Measured in the simulator (GPU, 20 Hz)

| Mode | Predicted | Measured | What happens |
|---|---|---|---|
| `demo` | 1.00 | 1.00 | full sequence, success held 1.5 s |
| `cover_first` | 0.00 | 0.00 | cover pulled with the breaker still ON -> FAULT |
| `no_pin` | 0.25 | 0.25 | breaker OFF and TEST pressed, pin skipped, cover pull -> FAULT |
| `no_test` | 0.50 | 0.50 | breaker OFF and pin seated, TEST skipped, cover pull -> FAULT |
| `no_hold` | 0.75 | 0.75 | cover released early, service button never counts |
| `order_swapped` | 0.00 | 0.00 | pin attempted with the lever still ON: hole blocked |
| `settle` | 0.00 | 0.00 | arms hold at home for 8 s, nothing moves |
| `dream` | 0.00 | 0.00 | Dream-1 baseline |

## Changes made after the first simulator runs
The first simulator run scored 0.25 because the scene itself was wrong; the logic was fine. Fixes:
1. **Pin hole is real.** The base plate was a solid slab, so the pin could never reach its seated height. The plate is now four boxes around a 28 mm square through-hole (6 mm clearance per side: arm placement error is a few mm, a 2 mm clearance made the pin land on the hole edge).
2. **Button caps** (TEST and SERVICE) sit one stroke (8 mm) above the plate; the fingertip is commanded 14 mm below the rest cap top (`PRESS_Z`). Before, the caps started flush with the plate and the press target was 12 mm too deep.
3. **Cover floats 24 mm above the plate**, clear of the service button cap (they overlapped before). Its handle moved to the front edge (sticks out toward the robot) so it can be gripped from above with open fingers (grip value -0.014) and pulled in -X; the old plan tried to hook a handle that sat on top of the cover.
4. **Cover displacement is read as an absolute value.** The cover slides in -X, so its joint value is negative; the logic expected a positive opening, so "cover open" and the cover FAULT never triggered.
5. **Breaker lever has almost no return spring** (0.02 N/m instead of 20 N/m); with the real spring it slid back toward ON after being pushed, un-latching the breaker.
6. **Pin grip** is 12 mm above the pin centre and the pin is gripped with -0.005 (a 16 mm pin needs a firm squeeze), so the pads stay clear of the plate while the pin is inserted.
7. **Cover wait** for the right arm is 2.3 s to match the longer pull (grasp step added).
8. Task and run scripts: `run.sh` uses the working `inspect-robots` call, `reset` takes the environment argument, `demo()` follows the mode from the environment, and every plan is stretched by `DEMO_TIME_SCALE` (1.8).
