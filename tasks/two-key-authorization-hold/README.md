# Two-Key Authorization Hold (`two-key-authorization-hold`)

A dual-control / two-person rule safety authorization procedure. Two operators (or both arms of an autonomous robot) must physically slide open protective red lids over two key buttons spaced 0.56 m apart, then press and hold both keys simultaneously for 3.0 s until the ARMED lamp turns green. While the station remains armed, the white CONFIRM button must be pressed before the 10.0 s countdown timer expires.

## Instruction
"Slide both red guards aside, then press and hold both key buttons together until the ARMED lamp turns green, and press the white CONFIRM button before the lamp turns off."

## Objects and Physical Layout
- **Controller Fixture**: 240 x 680 x 20 mm dark grey plate (8.0 kg, friction 1.0) centered at `(0.300, 0.000, 0.7575)`.
- **Key Buttons**: Two 40 x 40 x 12 mm yellow spring buttons (8 mm travel, 100 N/m) located at `(0.300, +0.280)` and `(0.300, -0.280)`. Spaced 560 mm apart, exceeding single-arm reach (~450 mm span).
- **Protective Guards**: Two 90 x 90 x 25 mm red sliding lids on prismatic sliders. Left guard slides +Y outward (80 mm travel); right guard slides -Y outward (80 mm travel). Both must be pushed outward >= 60 mm to expose key caps.
- **CONFIRM Button**: 40 x 40 x 12 mm white spring button at `(0.300, 0.000)`.
- **Status Indicators**:
  - ARM PROGRESS strip: 4 cyan marker lamps (25 mm diameter) at x = 0.400 m, lighting up at 0.75 s intervals during simultaneous hold.
  - ARMED lamp: 30 mm green disc at `(0.360, 0.000)`.
  - FAULT lamp: 30 mm red disc at `(0.360, +0.060)`.

## Timing Margin Table
| Requirement | Specification | Scripted Arm Achieved | Safety Margin |
|---|---|---|---|
| Simultaneous Key Hold | >= 3.0 s | 4.5 s | +1.5 s (+50%) |
| ARMED Confirm Window | <= 10.0 s | 1.5 s after ARMED | +8.5 s (+85%) |
| Final Success Hold | >= 1.0 s (20 steps) | 2.0 s (40 steps) | +1.0 s (+100%) |

## Contact and Kinematic Clearances
- **Commanded Overshoot**:
  - Guards: 80 mm travel + 5 mm overshoot (commanded displacement 85 mm).
  - Buttons: 8 mm travel + 6 mm commanded overshoot (commanded depth 14 mm below rest top).
- **Arm-Arm Clearance**: Minimum **113.97 mm** at t = 19.00 s (threshold >= 30.0 mm).
- **Scene Clearances**: All unintended link-to-obstacle clearances >= **3.63 mm** (threshold >= 1.0 mm).
- **Reach Margins**: Minimum joint margin **0.219 rad** (threshold >= 0.02 rad), worst IK error **0.08 mm** / **0.01 deg**.

## Scoring Ladder
- **0.25**: Both sliding guards opened >= 60 mm.
- **0.50**: Both keys held down simultaneously >= 1.5 s.
- **0.75**: ARMED reached (simultaneous hold >= 3.0 s).
- **1.00**: CONFIRM pressed while ARMED within 10.0 s, keys released, grippers open, arms still for 1.0 s (20 steps at 20 Hz).

## Zero-Gates
- **Strike Lockout**: Releasing either key before 3.0 s adds 1 strike and resets progress. 2 strikes permanently locks the console and illuminates the red FAULT lamp, capping score at 0.25.
- **ARMED Timeout**: Failing to press CONFIRM within 10.0 s after arming locks the console, capping score at 0.25.
- **Fixture Disturbance**: Moving base plate > 10 mm drops score to 0.0.

## Modes and Predicted Scores
- `demo`: 1.00 (full bimanual solution)
- `sequential`: 0.25 (guards opened, keys pressed one by one without overlap)
- `one_key`: 0.25 (both guards opened, only right key pressed)
- `release_early`: 0.25 (both keys held 1.8 s twice -> 2 strikes -> lockout)
- `no_confirm`: 0.25 (guards opened, keys held 4.5 s -> ARMED, confirm omitted -> the 10 s ARMED timer runs out -> lockout caps the score at 0.25)
- `settle`: 0.00 (idle hold at home)
- `dream`: 0.00 (baseline)


## Measured in the simulator
| Mode | Predicted | Measured (GPU) |
|---|---|---|
| demo | 1.00 | 1.00 |
| sequential | 0.25 | 0.25 |
| one_key | 0.25 | 0.25 |
| release_early | 0.25 | 0.25 |
| no_confirm | 0.25 | 0.25 (10 s ARMED timer ran out -> lockout) |
| settle | 0.00 | 0.00 |
| Dream-1 | 0.00 | 0.00 |

## Changes made after the first simulator runs
1. The controller plate was built with its box centred at the world position inside its own frame, which put it 0.3 m forward and 0.76 m up and threw it off the table at start-up. It is now centred in its own frame.
2. The guards are free sliders and creep back a few millimetres on their own, so the "guards open" reading can fall below 60 mm again before a lockout. The scorer now remembers that the guards were opened, so a lockout caps the score at 0.25 as the rules say (before, `no_confirm` ended at 0.00).
3. The `no_confirm` prediction is 0.25, not 0.75: after ARMED the 10 s timer runs out while the plan is still running, which locks the console.
4. The plans are stretched by `DEMO_TIME_SCALE` (1.8), like the other tasks, so the arms keep up.
