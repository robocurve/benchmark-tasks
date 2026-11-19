# Apple wedger: center it over an apple and press it level through to the board

Instruction: "Drive the apple wedger level through the apple flush onto the cutting board so all eight wedges separate, keep the apple where it stands, and clear both grippers."

Files: `task.json`, `apple_wedger_press_through_to_board.py`, `labkit.py`, `mechkit.py`, `pitch_demo.py`, `arena_assets_fix.py`, `test_apple_wedger_press_through_to_board.py`, `README.md`, `DEVIATIONS.md`.

## Environment
Isaac Lab-Arena 0.3.1, `inspect-robots` with the `yam_arena` embodiment, `station` scene. Offline pre-check (MuJoCo 4.0.1, CPU) before any GPU run.

## Offline check (no GPU)
```
PYTHONDONTWRITEBYTECODE=1 pytest -q -p no:cacheprovider test_apple_wedger_press_through_to_board.py
```
19 tests: as spawned 0.0; the 1.0 positive control; every tier (0.0 / 0.50 / 0.75 / 1.0); the core-press,
level-not-descended, cocked-wedger, partial-cut (3 of 8 = 0, 6 of 8 = 0.50), ring-3-mm-high, press-that-stops-short,
off-centre-8-mm, tilted-8-deg and unheld-wedge negatives; the rule releasing all eight on a centred level press; hold
parity between the scorer and the scoring text; gripper-near capping at 0.50; shortcut states; demo purity and IK
reach; a top-camera line-of-sight check that the scored regions (apple, wedges, ring on the board, both grips) are
unoccluded at the demo's end pose, with a blocker negative that proves the check can fail; and two geometry invariants
(every radial blade and corer flat lies on a seam plane; wedges sit antiphasely between blades).

## Run and score one Dream-1 trial
```
inspect-robots run --embodiment yam_arena -E task=apple_wedger_press_through_to_board.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --policy dream \
  --instruction "Drive the apple wedger level through the apple flush onto the cutting board so all eight wedges separate, keep the apple where it stands, and clear both grippers." \
  --scorer task_score --max-steps 440 --seed 0
```

## Scripted demo
```
inspect-robots run --embodiment yam_arena -E task=apple_wedger_press_through_to_board.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --policy yam_scripted \
  --instruction "Drive the apple wedger level through the apple flush onto the cutting board so all eight wedges separate, keep the apple where it stands, and clear both grippers." \
  --scorer task_score --max-steps 440 --seed 0
```
`demo()` reads scene constants only (never simulator state; pinned by `test_plan_reach_and_purity`). Both grippers splay
outward 14 deg, straddle the +-49 mm cushioned grips, pinch them gently (2.5 mm past contact, ~12 N), lift the 80 g
wedger, center it over the 66 mm apple, register level 4 mm above the crown, drive it flush to the board at 0.9 mm per
step, open 12 mm in place, retract clear and park for 3 s. Plan duration
14.4 s; `run_limit` is 440 steps (22 s at 20 Hz).

## Scoring
Judged at the end of the trial from observable end-state geometry. Succeeds once 1.0 holds for 3 s (60 steps).
- `1.0`: all 8 wedges separated, each wedge at most 12 mm from where it stood, ring resting on the board within 1.5 mm,
  wedger level within 5 deg and within 10 mm of the apple centre, both grippers > 120 mm clear of wedger and apple for 3 s.
- `0.75`: as 1.0 with the clear hold shorter than 3 s.
- `0.50`: ring on the board and level, at least 6 of 8 separated; or all 8 separated with a gripper still within 120 mm.
- `0.0`: otherwise (nothing, sweeping, an off-centre drop, a wedge knocked off its seat, a one-hand cocked jam, or a
  press that stops short of the board: an uncut seam is not an observable separation).
Possible scores: `0.0`, `0.50`, `0.75`, `1.0`.

## Runtime per trial
440 control steps at 20 Hz (22 s simulated). The offline MuJoCo pre-check takes ~7 s per full rollout on CPU; the
demo reaches 1.0 in 326 steps.

## Seeds
Seed 0, `object_jitter=0`, one fixed starting layout.

## Reset between trials
Lift the wedger off the board, discard the cut apple, stand a fresh conditioned apple on the board centre, wipe the
blades and board dry, and place the wedger level on the dark staging mat at x = 0.185 m, y = 0.000 m with its grips along Y.

## Mechanism and sources
| part | real item | model |
|---|---|---|
| cutting board | solid maple board 203 x 305 x 19 mm (8 x 12 x 0.75 in) | fixed 3-box compound, plain deck, no pocket |
| apple specimen | Granny Smith, 64-67 mm diameter band (66 mm modelled), 54 mm tall, 150 g | 8 free wedges (5 radial bands each, red skin on the outer band) plus a core plug with stem and leaf, each glued to the world on its own seam line |
| apple wedger | OXO Good Grips 32681 apple slicer, corer and divider: dual-grip stainless 8-blade wedger, 114 mm blade span, 80 g, 23 mm OD hollow coring tube, two cushioned side grips (ASIN B00004OCKT, https://www.amazon.com/dp/B00004OCKT) | 44-box rigid body: 12-gon blade ring (OD 114 mm, ID 104 mm, 60 mm tall) with a dark collar, 8-sided hollow coring tube, 8 radial 1.2 mm blades on the seam planes, pillow-block grips at +-49 mm |

## Real-world build
Parts: OXO Good Grips 32681 (ASIN B00004OCKT), 80 g, 2.1 x 4.5 in, blades fit apples up to ~4.5 in; a solid maple
board 203 x 305 x 19 mm; a dark silicone staging mat 130 x 210 x 3 mm; apples 64-67 mm across. Specimen spec: Granny
Smith, 64-67 mm diameter, 54 mm tall, 150 g, at 20 +- 2 C, one apple per trial, wiped dry. Stand-in for repeatability:
a machinable wax cylinder, 0.93 g/cm^3, 66 mm diameter x 54 mm, with 8 pre-scored 6 mm kerfs on the 45 deg seam planes.
One fixed layout: board centre 360 mm forward of the robot origin, mat centre at x = 185 mm, wedger at rest on the mat.

## Fidelity
Rigid-body simulation (PhysX, 20 Hz control). The apple does not deform: it is 8 rigid wedges and a core plug, each
held by its own breakable glue line. The eight blades and the coring tube pass through the fruit without collision and
cutting force is not modelled (the accepted #699 convention: the knife passes through the bread); the cut is a rule on
the wedger's measured pose relative to the apple. A seam's glue is released only when a blade is within 2.5 mm and
6 deg of that seam plane, with the blade bottoms driven through to the board, the wedger level within 5 deg and every
blade within 5 mm of its seam. The cut therefore grows exactly where a blade is, and nowhere else; nothing the scorer
reads is written from a hook. In the real fruit, pressing one grip only rolls the ring past 5 deg, so no seam is
released and the blades bind; the sim does not model that resistance (see the known limits below).

Known limits (measured, not hidden): a MuJoCo robustness probe of our own reports 0.0 at one of five start-jitter
seeds (+-5 mm, +-3 deg) because the scripted demo is open-loop and its grip approach is calibrated to one fixed layout;
that is our own probe, not an official score, which runs a fixed layout with `object_jitter=0`. In the MuJoCo stand-in
a one-hand press also reaches 1.0: the blades meet no physical resistance there (see DEVIATIONS, known limits), so the
one-grip roll described in `task.json` is a property of the real fruit that this sim does not reproduce.

## Licence
All shapes are procedural primitives authored in `apple_wedger_press_through_to_board.py`; no external third-party
mesh, texture or font file is included.
