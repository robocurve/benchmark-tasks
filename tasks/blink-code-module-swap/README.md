# Task 4: Blink Code Module Swap

Read the status lamp's blink code, then swap the module it names.

## Overview

In automated data centres, server racks, and autonomous robotic fleets, field-replaceable units (FRUs) report internal hardware errors and subsystem faults via blink patterns on diagnostic status LEDs. To perform autonomous maintenance, a service robot must observe the LED over time, decode the temporal blink pattern to identify which specific socket has failed, extract and replace only the designated module using a staged spare, and confirm recovery via diagnostic verification without disturbing healthy neighbouring modules.

## Instruction

The task instruction is strictly one sentence (<= 2 sentences), identical across `run.sh`, `task.py`, `task.json`, and this `README.md`:

> "While you hold the white DIAG button the status lamp blinks the number of the failed socket (count the dots beside each socket); replace that module with the teal spare, then hold DIAG again until the lamp stays green."

## Files

| File | Description |
|---|---|
| `task.py` | The yam_arena task file defining objects, visual lamp teleportation, and scoring hooks. |
| `task.json` | Program specification, instruction, setup details, and criteria metadata. |
| `geometry.py` | All dimensional constants, fixture layout, 4 socket cavities, dots, spare staging, and DIAG panel. |
| `parts.py` | Compound rigid parts, spring-loaded sliders, collision configuration, and runtime pose teleportation. |
| `logic.py` | Hidden-state machine tracking button hold, 3-pulse blink cycles, error states, and green verification. |
| `scoring.py` | Stateful step scorer enforcing numeric tolerances, healthy module zero-gates, and score ladder. |
| `demo_plan.py` | Scripted bimanual demonstration with open-hand descent, 1.8x stretch, and ablation modes. |
| `check_reach.py` | Offline inverse kinematics reachability and joint-limit margin verification. |
| `check_collisions.py` | Offline collision hull analysis checking arm-arm and scene clearances. |
| `run.sh` | Execution runner for simulator trials (demo, wrong_socket, no_diag, no_verify, no_swap, settle, dream). |
| `tests/` | Comprehensive test suite covering geometry, logic, scoring boundaries, modes, and task loading. |
| `VM_NOTES.md` | Verification runbook for Isaac Sim evaluation on the GPU VM using Brev CLI. |

## How to Run

From this folder inside the Isaac Lab-Arena environment (`source /data/IsaacLab-Arena/yam.env`):

```bash
./run.sh demo          # Scripted bimanual demonstration (target score: 1.0)
./run.sh settle        # Baseline resting stability check (target score: 0.0)
./run.sh wrong_socket  # Ablation: swaps healthy module 2 instead of 3 (target score: 0.0 zero-gate)
./run.sh no_diag       # Ablation: swaps module 3 without pressing DIAG (target score: 0.75)
./run.sh no_verify     # Ablation: initial DIAG press and swap, skips verify press (target score: 0.75)
./run.sh no_swap       # Ablation: presses DIAG only, no module swapped (target score: 0.25)
./run.sh dream         # Baseline evaluation with Dream-1 policy
```

To view the live Isaac Sim viewport on port 6080:
```bash
WATCH=1 ./run.sh demo
```

## Review Notes

### 1. Importance and Novelty
Field replacement of faulty modules based on dynamic diagnostic feedback is a quintessential capability for lights-out data centre operations and autonomous robotics infrastructure. Unlike static pick-and-place tasks or printed port maps, the information is encoded purely in the time domain: the robot must hold the diagnostic test button, observe multiple optical pulses, decode the socket address, execute the mechanical swap, and verify resolution.

**Comparison with neighbouring benchmark tasks:**
- **TK-2K8E9N (Plug a keyed connector into a loose socket block):** Single connector insertion into a loose, unconstrained block. Task 4 tests reading temporal blink codes, selective module extraction, bimanual verification, and healthy module preservation across a 4-socket multi-drop bus.
- **HawAII "Swap the blown fuse for the right rating":** Static printed text on a fuse; no temporal signal, no active diagnostic button, no verification hold.
- **DORL "Reseat the half-inserted plug" & "Patch cords per the port-map card":** Purely passive wiring with static visual port mapping. No active state machine or hidden diagnostic logic.

### 2. Construct Validity
The task is constructed so that unintended shortcuts cannot score:
- **Swapping the wrong module:** In the `wrong_socket` ablation, replacing module 2 instead of module 3 triggers the zero-gate because healthy module 2 remains unseated, yielding a score of 0.0.
- **Skipping the diagnostic readout:** Swapping module 3 without ever holding the DIAG button cannot verify resolution, capping score at 0.75 (`no_diag`).
- **Skipping post-swap verification:** Replacing the module without holding DIAG to verify steady green caps score at 0.75 (`no_verify`).
- **Touching or removing healthy modules:** Any healthy module unseated for > 1.0 s (20 control steps) and not re-seated by the end triggers an immediate zero-gate (0.0).
- **Physical zero-gates:** Fixture movement > 5.0 mm, fixture tilt > 5.0 deg, or dropping any module (> 20 cm below table plane) permanently zeros score.

### 3. Scoring
Scoring uses strictly numeric criteria measurable with calipers and camera frames:
- **0.0:** Initial resting state or zero-gate failure.
- **0.25:** DIAG button held >= 3.0 s once (60 control steps, code was shown) or any module touched / displaced > 5.0 mm or DIAG button depressed >= 2.0 mm.
- **0.50:** Failed module (module 3) out of socket 3 (depth < 16 mm, lateral offset > 1.5 mm, or angle > 10.0 deg) with every healthy module (1, 2, 4) still seated.
- **0.75:** Teal spare module seated in socket 3 (depth >= 16 mm, lateral error <= 1.5 mm, angle error <= 10.0 deg), failed module not in any socket, and all healthy modules seated.
- **1.00:** Additionally DIAG held >= 3.0 s showing steady green lamp, all four sockets seated, both grippers open, and all objects still (linear speed <= 5 mm/s, angular <= 0.2 rad/s) held continuously for 1.0 s (20 control steps).
- **Latching:** Partial tiers (0.25, 0.50, 0.75) latch the highest tier achieved. Tier 1.00 requires simultaneous satisfaction of all conditions held for 1.0 s.

### 4. Clarity and Reproducibility
The setup is fully specified from the midpoint of the arm bases (0, 0, TOP = 0.7475 m):
- **Bus Strip:** Rigid fixture (8.0 kg, friction 1.0), 50 × 290 × 50 mm, coupled via 40 × 150 × 10 mm tie bar to controller box.
- **Sockets (four):** Centred at mouth x = 24.0 cm, z = 2.5 cm above table; y = -5.0 cm (socket 1, 1 dot), y = -12.0 cm (socket 2, 2 dots), y = -19.0 cm (socket 3, 3 dots), y = -26.0 cm (socket 4, 4 dots). Cavity 30 × 18 mm (1.0 mm clearance per side to 28 × 16 mm nose), 8 × 3 mm key slot, 12 N/mm spring plunger on +y wall standing 1.5 mm proud. Each socket marked by 1–4 white dots (4 × 4 × 1 mm) on the shell top.
- **DIAG Controller Box:** 140 × 120 × 40 mm, centre at x = 25.0 cm, y = 20.0 cm. White DIAG button cap 36 × 36 × 12 mm, 4 g, centre at (22.0, 20.0) cm, rest top 6.0 cm above table, 8 mm travel to hard stop, soft 100 N/m spring (0.8 N at stop). Status lamp 60 mm diameter, 15 mm tall dome at (28.5, 20.0) cm (off = dark grey, blink = amber/yellow, verified = green, healthy missing = red).
- **Servo Modules:** 80 g each, friction 0.8; 45 × 18 × 28 mm housing with grey central bands (modules 1–4) and steel nose 28 × 16 × 18 mm with key rib and groove; spare module with teal central band staged on table at (38.0, -15.0) cm.

### 5. Feasibility and Verification Numbers
- **Kinematic Reachability:** Verified offline with `check_reach.py`:
  - Left arm: worst position error 0.08 mm, rotation error 0.01 deg, min joint margin 0.396 rad.
  - Right arm: worst position error 0.10 mm, rotation error 0.01 deg, min joint margin 0.291 rad.
  - Both arms clear joint limits with margin >= 0.291 rad (well above 0.02 rad requirement).
- **Workspace Separation:**
  - DIAG button at y = +0.200 m is reachable only by the left arm (right arm fails IK with 36.2 mm error).
  - Failed socket 3 at y = -0.190 m and socket 4 at y = -0.260 m are reachable only by the right arm (left arm fails IK with 20.8 mm and 58.7 mm error).
  - Hands maintain >= 250 mm separation (> 100 mm requirement) throughout the trial.
