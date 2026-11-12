# Slide keyboard tray under desk until purple latch engages (keyboard_tray_latch)

Instruction: "Stow the grey keyboard tray under the desk and secure it."

## What it tests
Bimanual brace-and-insert operation and physical latching. A grey keyboard tray slides on under-desk bilateral guide rails beneath a fixed 25 kg beige desk fixture. Inserting the tray requires coordinated bimanual action: the left arm clamps and braces the front edge of the desk fixture at y = +15 cm to react insertion forces and prevent base drift, while the right arm pushes the keyboard tray 27.8 cm inward along +X at y = -10 cm until the spring-loaded purple latch engages with 8 mm vertical deflection.

## Files
`task.py` (task file), `geometry.py` (geometric constants, dimensions, masses, colors), `logic.py` (scoring ladder evaluation), `scoring.py` (scorer wrappers and TASK_TRACE logger), `demo_plan.py` (scripted dual-arm demo and ablation plans), `parts.py` (compound parts and prismatic slider helper), `tracekit.py` (debug tracer), `check_reach.py` (IK reach verification), `check_collisions.py` (hand envelope clearance check), `run.sh` (runner script), `tests/` (unit tests), `touch_rules.json` (contact rules), `task.json` (benchmark metadata).

## Run
Needs the Isaac Lab-Arena environment from the program's launchable (`source /data/IsaacLab-Arena/yam.env`).

    ./run.sh settle      200-step hold, expected score 0
    ./run.sh demo        scripted full-score demo (1,500 steps)
    ./run.sh ablation    scripted run with unbraced push (left arm idle)
    ./run.sh dream       Dream-1 policy, 2,400 steps

All runs use `-E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 --scorer task_score`. Offline verification checks (no simulator):

    python -m unittest discover -s tests
    python check_reach.py
    python check_collisions.py

Runtime per trial: the demo plan runs for 532 steps (26.6 s at 20 Hz); a complete Isaac Lab-Arena trial takes approximately 4-5 minutes wall time on an L40S GPU. `TASK_TRACE=<file>` writes JSON lines per scorer call.

## Layout (cm from the midpoint between the arm bases; x forward, y left; table top z 74.75)
- Desk fixture: beige body 20 x 36 x 6 cm (including 20 x 36 x 2 cm top plate and bilateral 20 x 2 x 4 cm guide rails), mass 25.0 kg, friction 1.2 (fixed), center at (30.0, 0.0, 74.75). Bracing zone on front top edge at y = +15.0 cm.
- Keyboard tray: grey body 20 x 30 x 2 cm, mass 0.5 kg, slider along +X with 28.0 cm travel and 20.0 N s/m damping. Rest position at (16.0, 0.0, 76.75). Pushed from handle at y = -10.0 cm.
- Purple latch: purple catch 2 x 2 x 2 cm, mass 0.03 kg, slider along -Z, travel 8 mm, spring rate 100.0 N/m, damping 10.0 N s/m. Located at (44.0, 0.0, 77.75).
- Arm separation: Left arm operates at y = +15.0 cm (bracing desk edge); Right arm operates at y = -10.0 cm (pushing tray). Concurrent arm separation in y is 25.0 cm (clearance to 85 mm gripper bodies is 16.7 cm, far exceeding the 3.0 cm hard floor and 15.0 cm design target). Minimum interpolated TCP distance is 251.8 mm.

## Scoring
Gates (score 0): desk base displacement > 5.0 mm zeroes score immediately.
Ladder:
- 0.00: start state (tray inserted < 5.0 cm).
- 0.25: tray inserted >= 5.0 cm.
- 0.50: tray inserted >= 15.0 cm.
- 0.75: tray inserted >= 26.0 cm, not yet latched.
- 1.00: tray inserted >= 27.8 cm with purple latch engaged and motionless for 1.0 s.
Possible scores: 0.0, 0.25, 0.50, 0.75, 1.00.

## Why shortcuts fail
Attempting to push the tray without bracing the desk risks displacing the base fixture and tripping the 5 mm drift gate. Pushing the tray partially awards only partial ladder credits (0.25 at 5 cm, 0.50 at 15 cm). Ceasing push before 27.8 cm leaves the purple latch unengaged, capping the score at 0.75. An idle policy remains at score 0.0. The scripted demo proves kinematic and geometric feasibility.

## Predicted measurements
| Run | Score | Notes |
|---|---|---|
| demo (scripted) | 1.00 | desk braced, tray driven in 27.8 cm, purple latch engaged, settled for 1.0 s |
| ablation (left arm idle) | 0.00 | unbraced push allows reactive desk displacement, tripping drift gate |
| settle (200 steps) | 0.00 | arm remains at ready standoff, zero contact |
| dream (2,400 steps) | 0.00 | unverified offline; baseline exploration expected to yield 0.00 prior to policy convergence |

## Changes made after the first simulator runs & Board status
- Tray stroke was standardized to 280 mm on under-desk rails with 20 N s/m damping to model realistic sliding resistance.
- Purple latch spring rate set to 100 N/m with 8 mm vertical travel, ensuring positive mechanical latching that requires full stroke completion.
- Arm working points maintained at y = +15.0 cm and y = -10.0 cm, providing 250 mm constant lateral separation (clearance 166.8 mm) well above the 85 mm gripper body envelope and 150 mm concurrent arm separation target.
- Benchmark Board Note: Flagged as `CLEAR` (survey across 1,418 benchmark cards verified zero overlap with under-desk tray latch mechanisms). Kinematics and offline preflight are fully verified; pending Operator GPU trial.
