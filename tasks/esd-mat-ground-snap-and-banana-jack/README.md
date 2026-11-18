# Press a ground cord snap onto a mat corner tab and seat its banana plug into a ground block

Instruction: "Latch the mat corner tab flat, snap the ground cap onto its stud, route the cord clear of the work zone, seat the plug, and clear both grippers."

Files in this folder: `task.json`, `esd_mat_ground_snap_and_banana_jack.py`, `labkit.py`, `mechkit.py`, `pitch_demo.py`, `arena_assets_fix.py`, `test_esd_mat_ground_snap_and_banana_jack.py`, `README.md`, `DEVIATIONS.md`.

## Environment
Isaac Lab-Arena 0.3.1 on Isaac Sim 6.1 (Brev launchable), `inspect-robots` with the `yam_arena` embodiment, `station` scene.
Task file SHA-256 (first 12): see freeze log.

## Offline check (no GPU)
Run `PYTHONDONTWRITEBYTECODE=1 pytest -q -p no:cacheprovider test_esd_mat_ground_snap_and_banana_jack.py` with `yam_arena` on `PYTHONPATH`: verifies zero start score, every scoring tier (`0.0`, `0.25`, `0.50`, `0.60`, `0.85`, `1.0`), threshold boundaries (`4.0 deg`, `5.0 mm`, `1.2 mm`, `3.0 mm`, `8.0 mm`, `45 mm`), work-zone cord infringement, partial/unlatched jack states, dynamic body mass floor (`>= 25 g`), demo constant purity, and IK kinematic reach (`pitch_demo.check_plan`).

## Run and score one Dream-1 trial
```
inspect-robots run --embodiment yam_arena -E task=esd_mat_ground_snap_and_banana_jack.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --policy dream \
  --instruction "Latch the mat corner tab flat, snap the ground cap onto its stud, route the cord clear of the work zone, seat the plug, and clear both grippers." \
  --scorer task_score --max-steps 1000 --seed 0
```

## Scripted demo
```
inspect-robots run --embodiment yam_arena -E task=esd_mat_ground_snap_and_banana_jack.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --policy yam_scripted \
  --instruction "Latch the mat corner tab flat, snap the ground cap onto its stud, route the cord clear of the work zone, seat the plug, and clear both grippers." \
  --scorer task_score --max-steps 1000 --seed 0
```
`demo()` uses scene geometry constants only (never live simulation state). The left gripper presses the corner boss (`-65, +88` mm) straight down until the `50 g` corner flap locks flat beneath the bench `tab_catch` (`0.0..1.2 deg`), holding while the right arm lifts the `34 g` `snap_cap`, aligns over the `10 mm` (`5.6 mm` tall pin) stud inside `55 deg` funnel cheeks, and presses downward past the `60 deg` `snap_jaw` retainer (`22 N/m`). Left parks `>= 45 mm` clear, right picks the `34 g` `banana_plug` with `-Y/+Y` pad-caging flanges, sweeps the coiled cord through the `+X` bypass lane (`x > 255 mm`) clear of the PCB work zone, drives `-Y` into the `120 g` `ground_block` past the `6 N/m` `jack_latch`, runs a `~5 N` reverse pull check, and parks `>= 45 mm` clear for `3.0 s`. Open-loop plan duration is `33.4 s` (`668` nominal steps; early-terminates at `593` steps / `29.65 s` once `1.0` holds for `3.0 s`, inside `run_limit = 1000` steps).

## Scoring
Judged at the end of the trial using physical end-state geometry (verified with calipers and digital protractor); succeeds once `1.0` holds for `3.0 s` (`60` steps):
- `1.0`: corner tab flat (`<= 4.0 deg`); snap cap centered within `5.0 mm` radially and `-1.5..3.0 mm` vertically above flat stud base with `jaw_open <= 1.2 mm`; both cord links outside `x = 155..255 mm, y = -120..-15 mm`; ground block within `8.0 mm` of its bench nest; banana plug shoulder within `-3.0..3.0 mm` of the front face (`<= 5.0 mm` lateral, `<= 8.0 deg` tilt, `jl_open <= 1.2 mm`); both grippers `>= 45 mm` clear.
- `0.85`: fully grounded and cord clear, gripper within `45 mm`.
- `0.60`: stud snapped and cord clear, plug only partially entered (`-3.0..18.0 mm` gap, `<= 8.0 mm` lateral) or unlatched.
- `0.50`: stud snapped and plug latched home, cord crosses marked work zone.
- `0.25`: stud snapped and ground block in nest only.
- `0.0`: otherwise (noop, snap unlatched, flap curled past `4.0 deg`, block pushed past `8.0 mm`, or 1-hand snap without corner flatten).
Possible scores: `0.0`, `0.25`, `0.50`, `0.60`, `0.85`, `1.0`.

## Runtime per trial
Runtime per trial: 1000 control steps at 20 Hz (50 s of simulated time, about 5 min wall clock on one GPU); scripted demo completes once the 3.0 s stability hold finishes (593 steps / 29.65 s).

## Seeds
Seeds: 0 (`object_jitter=0`, fixed starting layout).

## Reset between trials
Pull the banana plug from the ground block, release the snap cap and corner retainer catch so the mat flap springs back to `+14 deg`, and stage the snap cap at `(0.115, -0.060)` m and banana plug at `(0.255, -0.225)` m with the coiled cord elbow around `(0.145, -0.195)` m.

## Mechanism and sources
| part | real item | model | precedent |
|---|---|---|---|
| ESD workmat & bench zone | static-dissipative electronics bench mat (`$19` kit with ground cord & block: https://www.amazon.com/dp/B00009XT3H) | fixed mat `(160 x 118 x 4 mm)` + yellow PCB keep-out zone + `50 g` hinged corner flap (`0.35 N*m/rad`) + `28 g` `tab_catch` (`14 N/m`) | bimanual counter-hold fixture |
| 10 mm ground snap & cord | `10 mm` female snap cord with banana plug & strain boots (`https://www.desco.com/catalog/grounding/bench-grounding/common-point-ground-cords/09864`) | 6-link articulated chain (`34 g` `snap_cap` + `26 g` `strain_boot` + `30 g` `cord_prox` + `30 g` `cord_dist` + `26 g` `plug_boot` + `34 g` `banana_plug`) | articulated cable assembly |
| common-point ground block | `120 g` bench-edge banana jack ground block (`https://www.desco.com/catalog/grounding/bench-grounding/common-point-grounds/09820`) | `120 g` nested block (`68 x 54 x 30 mm`) with `34 mm` flared mouth + rear hood + `28 g` `jack_latch` (`60 deg`, `6 N/m`) | spring-latched jack insert |

## Real-world build
Parts: 10 mm ESD bench mat snap kit with coiled ground cord and banana plug (`~$11`), common-point bench ground block (`~$5`), and spring corner retainer clip (`~$3`, `~$19` total BOM: https://www.amazon.com/dp/B00009XT3H). Stage the mat centered at `x = 228 mm, y = -68 mm` with the corner flap curled upward at `+14 deg` and the ground block nestled on the bench edge at `x = 375 mm, y = -205 mm`.

## Fidelity
Rigid-body simulation (Isaac Sim 6.1, PhysX, 20 Hz control). Simplified: elastomer mat flap bending (`0.35 N*m/rad` revolute hinge), flexible strain-relief boots and coiled wire (two yaw-hinged rigid links plus pitch-hinged boots), and cantilevered snap/jack springs (`60 deg` beveled `22 N/m` `snap_jaw`, `14 N/m` `tab_catch`, `6 N/m` `jack_latch`) replace continuous viscoelastic deformation; small bodies (`snap_cap`, `strain_boot`, `plug_boot`, latches) are scaled to `26-34 g` for rigid-body contact stability. Friction and contact softness are verified across `0.7x-1.5x` friction and soft-contact compliance.

## Licence
All shapes are procedural primitives authored in `esd_mat_ground_snap_and_banana_jack.py`; no external third-party mesh, texture, or font files are included (original own work).
