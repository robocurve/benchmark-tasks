# Task F: Servo Bus ID Commissioning

Give three identical servo modules their IDs, one at a time, on a shared bus.

## Overview

In robot manufacturing, hardware bring-up, and automated maintenance, multi-drop daisy-chained actuator buses (e.g. Dynamixel, CANopen, RS-485) are standard. When new replacement servos arrive from the factory, they all share a default unconfigured bus address (ID None / 0). Plugging multiple un-commissioned servos into a shared bus creates immediate address collisions, blinding the controller.

The robot must follow an exact hardware commissioning procedure:
1. Isolate the bus by unplugging clashing servo modules.
2. With only one un-numbered servo on the bus, trigger `ASSIGN` to assign the next sequential ID.
3. Plug the next servo module and repeat until all three are numbered (1 for red, 2 for green, 3 for blue).
4. Settle all modules in their keyed sockets with the bus lamp confirming successful, clash-free operation.

## Instruction

The task instruction is strictly two sentences, identical across `run.sh`, `task.py`, `task.json`, and this `README.md`:

> "Three new servo modules (red, green, blue) are plugged into the bus and the bus status lamp is red. Press the white ASSIGN button on the front of the controller box (numbers the single un-numbered servo on the bus; yellow RESET button behind it clears numbers) until red is 1, green is 2, blue is 3 (the 3 x 3 readout board lights each servo's number in its colour row, columns 1 to 3), all three are plugged in, and the lamp turns green."

## Files

| File | Description |
|---|---|
| `task.py` | The yam_arena task file defining objects, scoring hooks, and demo interface. |
| `task.json` | Program specification, instruction, setup details, and criteria metadata. |
| `geometry.py` | All dimensional constants, fixture layout, socket cavities, and plug definitions. |
| `parts.py` | Compound rigid parts, spring-loaded sliders, collision configuration, and runtime pose teleportation. |
| `logic.py` | Hidden-state machine tracking servo IDs, bus clash arbitration, and indicator lamps. |
| `scoring.py` | Stateful step scorer enforcing numeric tolerances, failure gates, and the score ladder. |
| `demo_plan.py` | Scripted bimanual demonstration with tunable contact and timing knobs. |
| `compliance.py` | Arm tip compliance from the joint PD gains and the static cap travel it allows (used by `tests/test_button_press.py`). |
| `check_reach.py` | Offline inverse kinematics reachability and joint-limit margin verification. |
| `check_collisions.py` | Offline collision hull analysis checking arm-arm and fixture clearances. |
| `run.sh` | Execution runner for simulator trials (demo, settle, ablations, recover, dream). |
| `tests/` | Comprehensive test suite covering geometry, logic, scoring boundaries, modes, and task loading. |
| `VM_NOTES.md` | Verification runbook for Isaac Sim evaluation on the GPU VM using Brev CLI. |

## Credits

`parts.py` adapts the runtime hooking technique first demonstrated in `robocurve/benchmark-tasks` PR #40 (`tasks/bearing-press/partkit.py`), providing an independent implementation for compound rigid bodies and spring sliders. Changes versus the reference task's `parts.py`: (1) `Part.collision: bool = True`; a `collision=False` part (the lit-lamp markers) gets no colliders and is made kinematic, so gravity cannot drop it; (2) `write_pose(environment, name, pos, ...)` teleports a part's root body each step (API `write_root_pose_to_sim_index`, legacy `write_root_pose_to_sim` as fallback) and logs a WARNING once per object if anything fails (missing scene entry, missing API, exception) and one INFO line on the first success, so a dead lamp is visible in the run log; (3) slider axes "+X/+Y/+Z" with swapped limits, for the plunger that moves toward +y.

## How to Run

From this folder inside the Isaac Lab-Arena environment (`source /data/IsaacLab-Arena/yam.env`):

```bash
./run.sh demo          # Scripted bimanual demonstration (target score: 1.0)
./run.sh settle        # Baseline resting stability check (target score: 0.0)
./run.sh all_plugged   # Ablation: pressing ASSIGN with 3 clashing modules (target score: 0.25)
./run.sh wrong_order   # Ablation: commissioning in reversed order without RESET (target score: 0.25)
./run.sh recover       # Ablation: wrong order, RESET press, then correct order (did not reach 1.0 in the first simulator run: 0.25)
./run.sh dream         # Baseline evaluation with Dream-1 policy
```

To view the live Isaac Sim viewport on port 6080:
```bash
WATCH=1 ./run.sh demo
```

## Review Notes

### 1. Importance and Novelty
Automating physical robot assembly and hardware bring-up directly accelerates AI timelines by closing the loop between hardware construction and software verification. In industrial robotics, commissioning a multi-drop bus cannot be solved by passive insertion alone; the robot must understand the relationship between physical connection state and digital bus arbitration.

**Comparison with neighbouring benchmark tasks:**
- **TK-2K8E9N (Plug a keyed connector into a loose socket block):** Single connector insertion into a loose, unconstrained block. Task F uses a fixed 3-socket bus strip and tests sequential bimanual commissioning, address collision handling, two-button state control, and visual feedback from a 3x3 readout grid.
- **DORL "Reseat the half-inserted plug" & "Patch cords per the port-map card":** Purely passive wiring tasks with static mapping. No hidden state, no address collision logic, and no interactive pushbuttons.
- **Triton "Slide a carriage onto a linear rail until it clicks":** 1D linear alignment and mechanical click detent, but lacking multi-device communication bring-up.

### 2. Construct Validity
The task is constructed so that unintended shortcuts cannot score:
- **Guessing or pressing ASSIGN without unplugging:** If two or more unassigned modules are plugged in, bus address arbitration blocks assignment, keeping the status lamp red. Score remains 0.25 (partial credit for button press).
- **Commissioning in the wrong order:** If Green is numbered first (ID 1), Red cannot obtain ID 1 unless the yellow RESET button is pressed to clear the bus. Without RESET, the score latches at 0.25.
- **Key rib orientation:** the key rib (6 × 3 mm) sits 1 mm to the right of centre (toward -y) and the key slot is offset by the same 1 mm, so the fit is 1.000 mm per side (tested in `test_world_key_slot_clearance`). Upside-down insertion would put the rib against the cavity floor by geometry; this is NOT exercised in the simulator (the scripted demo never tries it).
- **Retention detent:** the spring plunger (4 mm round tip, 12 N/mm) is on the socket's +y wall, the same side as the groove of the seated nose, with its tip 1.0 mm inside the groove (tested in `test_plunger_in_groove_alignment`). Whether the click needs a deliberate pull is untested; the first GPU run shows it (does the pull-out in the demo work, and how far does the plunger travel in the trace).
- **Zero-gates:** Fixture movement > 5.0 mm, fixture tilt > 5.0 deg, or dropping any module (> 20 cm below table plane) triggers an immediate, irreversible zero-score (0.0).

### 3. Scoring
Scoring uses strictly numeric criteria measurable with calipers and camera frames:
- **0.0:** Initial state or zero-gate failure.
- **0.25:** At least one module unplugged (fails seated criteria: depth < 16 mm, lateral offset > 1.5 mm, or angle > 10.0 deg) or either button depressed >= 2.0 mm (`assign_q <= -0.002` or `reset_q <= -0.002`).
- **0.50:** Red module assigned ID 1 (requires valid ASSIGN press with travel >= 6.4 mm while Red is the sole unassigned seated module).
- **0.75:** Red has ID 1 and Green has ID 2.
- **1.0:** Red=1, Green=2, Blue=3; all three modules seated (depth >= 16 mm, lateral offset <= 1.5 mm, angle <= 10.0 deg); bus lamp green; both grippers open; all objects still (linear speed <= 5 mm/s, angular <= 0.2 rad/s); held continuously for 1.0 s (20 control steps).
- **Score Tier Latching:** Partial credit tiers (0.25, 0.50, 0.75) latch the highest tier achieved during the episode. Tier 1.0 requires simultaneous fulfillment of all success conditions held for 1.0 s.

### 4. Clarity and Reproducibility
The setup is fully specified from the midpoint of the arm bases (0, 0, TOP = 0.7475 m):
- **Bus Strip:** Rigid fixture (8.0 kg, friction 1.0), 50 × 220 × 50 mm, coupled via 40 × 110 × 10 mm tie bar to controller box.
- **Sockets (three):** Centred at mouth x = 24.0 cm, z = 2.5 cm above table; y = -5.0 cm (socket 0, blue), y = -12.0 cm (socket 1, red), y = -19.0 cm (socket 2, green). Cavity 30 × 18 mm (1.0 mm clearance per side to 28 × 16 mm nose), 8 × 3 mm key slot offset 1 mm to the right of centre (toward -y, with the key rib), 12 N/mm spring plunger on +y wall standing 1.5 mm proud.
- **Controller Box:** 140 × 80 × 40 mm, centre at x = 26.0 cm, y = 10.0 cm. White ASSIGN button at (21.5, 8.5) cm and yellow RESET button at (28.5, 8.5) cm: 36 × 36 × 12 mm caps, 4 g, rest top 6.0 cm above the table (8 mm above the box), 8 mm travel to a hard stop (cap bottom then flush with the box top), soft 100 N/m spring (0.8 N at the stop; see the compliance note below). Status lamp 30 mm dome at (25.0, 12.0) cm (bright red off, bright green on).
- **Readout Board:** 120 × 80 × 6 mm beside controller box at x = 26.0 cm, y = 21.0 cm. 3 × 3 grid of 20 mm lamps at 30 mm pitch (rows red, green, blue at x = 23.0, 26.0, 29.0 cm; columns 1, 2, 3 at y = 24.0, 21.0, 18.0 cm). Columns are marked on the far edge of the board by 1, 2, 3 white dots; the lit disc takes the colour of the servo's row. Projection test: all nine lamps and the status lamp are inside the top-camera frame and unoccluded by the fixture (camera at (-0.19, 0, 1.18)); the real legibility at 320 × 176 is a GPU check.
- **Servo Modules (three):** 80 g each, friction 0.8; 45 × 18 × 28 mm housing with red, green, or blue central band; steel nose 28 × 16 × 18 mm with 6 × 3 mm key rib and 5 × 1.5 mm groove.

### 5. Feasibility and Verification Numbers
- **Kinematic Reachability:** Verified offline with `check_reach.py` across full trajectory:
  - Left arm: worst position error 0.08 mm, rotation error 0.01 deg, min joint margin 0.386 rad.
  - Right arm: worst position error 0.09 mm, rotation error 0.01 deg, min joint margin 0.385 rad.
- **Collision Clearances:** Verified offline with `check_collisions.py` (checking every waypoint without label exemptions):
  - Arm-arm: every control step checked, all link pairs more than 50 mm apart; smallest bounding-box gap 104.2 mm.
  - Button press (fingertip modelled with the cap at its 8 mm end stop): fingertip 6.2 mm above the controller box at the hold, 5.8 mm below the cap top at the stop (commanded 6 mm past the stop; limit 6.5 mm), neighbouring RESET cap and status lamp far clear (see GATE_F.md for the per-waypoint table).
  - Socket insertion/pull: nose 0.93-1.00 mm from the socket walls (design clearance 1.0 mm), housing front face stops 2.0-2.5 mm into the shell mouth at the commanded 2.5 mm overshoot (the intended hard stop); gripper fingertips 3.5 mm from the socket walls; transit 8.0 mm.
- **Timing and Run Limit:**
  - Standard demo: the plan is stretched 1.8x for the soft arm (`DEMO_TIME_SCALE`); the measured run ended at step 1,038 (success held 1 s); `run.sh` STEPS = 1,500.
  - Recover mode about 2,150 steps (`run.sh` STEPS = 2,700); wrong-order mode about 1,140 steps, all_plugged about 250 steps.
  - Run limit: 3,000 steps (2.5 minutes at 20 Hz), providing > 75% spare time.
- **Ablation scores (measured in the simulator, one run each; predicted in brackets):**
  | Mode | Measured | Description |
  |---|---|---|
  | `settle` | 0.00 [0.00] | Arms hold the ready pose, parts rest stably |
  | `all_plugged` | 0.25 [0.25] | ASSIGN pressed with 3 unnumbered servos (address clash): nothing is numbered |
  | `wrong_order` | 0.25 [0.25] | Green numbered first without a reset (red cannot get ID 1) |
  | `recover` | 0.25 [1.00] | Wrong order, RESET, correct order: did not work in the first run (the plan ended with green = 1, the others unnumbered); not used as evidence |
  | `demo` | 1.00 [1.00] | Standard sequence (red -> 1, green -> 2, blue -> 3), also reached once in the simulator |
  | Dream-1 | 0.00 | One run, 3,000 steps, no servo numbered |

### 6. Known limits (honest)
- The state is readable from the readout board; the reasoning left is the order (isolate, ASSIGN, plug, ASSIGN) and RESET after a mistake. A harder variant (ASSIGN numbering free sockets) was not adopted; see `work/tasks/QUESTIONS.md`.
- Button physics: the arms' joints are soft (vertical tip compliance 1.8-2.2 mm/N at the press poses), so the cap spring is soft (100 N/m) and the press is commanded 6 mm past the cap's end stop; offline equilibrium travel is 8.0 mm vs the 6.4 mm trigger (`tests/test_button_press.py`). Whether the simulator agrees is a GPU check.
- Everything else in the simulator (plunger click, pull-out force, marker display via `write_pose`, camera legibility) is untested until the GPU run.

## Changes made after the first simulator runs

- The soft arm lags the open-loop plan: every waypoint duration is stretched 1.8x (`DEMO_TIME_SCALE`).
- The unplugged servos rest 3.9 degrees tilted in their sockets and 7.1 degrees after a release (1 mm of play), so the
  "seated" test is nose depth >= 16 mm and tilt <= 10 degrees (it was 17 mm and 3 degrees).
- The fingers open fully (94 mm) when coming down on a servo lying at its staging spot; with 54 mm a pad knocked it over.
- The insertion height is raised by 0.7 mm (`DEMO_INSERT_DZ`) so the nose meets the socket on its axis.
