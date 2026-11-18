# Latch a sprung inline-skate cuff buckle shut using its notched strap and hinged ratchet lever

Instruction: "Latch the inline-skate cuff ratchet buckle snugly closed on the bench pad with the strap engaged and lever folded flat, keeping the cuff seated and grippers clear."

Files in this folder: `task.json`, `inline_skate_ratchet_cuff_buckle.py`, `labkit.py`, `mechkit.py`, `pitch_demo.py`, `arena_assets_fix.py`, `test_inline_skate_ratchet_cuff_buckle.py`, `README.md`, `DEVIATIONS.md`.

## Environment
Isaac Lab-Arena 0.3.1 on Isaac Sim 6.1 (Brev launchable), `inspect-robots` with the `yam_arena` embodiment, `station` scene.
Task file SHA-256 (first 12): see freeze log.

## Offline check (no GPU)
Run `PYTHONDONTWRITEBYTECODE=1 pytest -q -p no:cacheprovider test_inline_skate_ratchet_cuff_buckle.py` with `yam_arena` on `PYTHONPATH`: verifies initial state scores 0.0, all scoring tiers (0.0 / 0.5 / 1.0), 1-hand unstabilized / unheld-wing failure, displaced cuff failure, sub-gate ablations (notches < 2, lever un-folded, wing > 1.5 deg open, gripper < 5 cm, hold < 3 s), shortcuts scoring 0.0, top-camera occlusion proxy clearance, scoring numeric parity, demo purity, and IK reachability.

## Run and score one Dream-1 trial
```
inspect-robots run --embodiment yam_arena -E task=inline_skate_ratchet_cuff_buckle.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --policy dream \
  --instruction "Latch the inline-skate cuff ratchet buckle snugly closed on the bench pad with the strap engaged and lever folded flat, keeping the cuff seated and grippers clear." \
  --scorer task_score --max-steps 720 --seed 0
```

## Scripted demo
```
inspect-robots run --embodiment yam_arena -E task=inline_skate_ratchet_cuff_buckle.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --policy yam_scripted \
  --instruction "Latch the inline-skate cuff ratchet buckle snugly closed on the bench pad with the strap engaged and lever folded flat, keeping the cuff seated and grippers clear." \
  --scorer task_score --max-steps 720 --seed 0
```
`demo()` reads scene geometry constants only. The left gripper straddles and pinches the opposing front-left cuff ears shut against the 6 N opening spring while stabilizing the unbolted 454 g cuff assembly on the rubber pad. The right gripper picks the 42 g notched ladder strap on the -Y side, feeds it into the buckle receiver channel within 3 deg to catch the entry pawl, releases the strap, grasps the 24 x 18 x 20 mm lever knob, pumps three full 42 deg lever arcs via `hinge_turn` to advance 3 ratchet teeth (pulling the wing snug <= 1.5 deg), folds the lever flat, and both arms park >= 5 cm clear for 3 s. Plan duration is 27.0 s; `run_limit` is 720 steps (36 s at 20 Hz).

## Scoring
Judged at the end of the trial from observable end-state geometry (measured in real trials with a caliper/ruler and angle gauge); succeeds once 1.0 holds for 1.0 s:
- `1.0`: unbolted cuff within 25 mm of pad center and <= 8 deg tilt; strap seated >= -7 deg in buckle mouth (entry within 3 deg while wing <= 3.5 deg); cuff latched <= 1.5 deg open (requires >= 2 ratchet notches at 6 mm pitch from 14 mm slack via >= 28 deg lever pumps) with lever folded <= 8 deg throughout a 3 s (60 steps) hands-off hold; both grippers >= 5 cm clear of the cuff envelope.
- `0.5`: cuff seated, strap >= -7 deg, wing <= 8 deg open, and hands >= 5 cm clear for 3 s, but wing > 1.5 deg (< 2 notches) or lever > 8 deg.
- `0.0`: otherwise (noop, displaced cuff, unengaged strap / withdrawn < -7 deg, wing > 8 deg open, gripper within 5 cm, or hold < 3 s).
Possible scores: `0.0`, `0.5`, `1.0`.

## Runtime per trial
Runtime per trial: 720 control steps at 20 Hz (36 s of simulated time, about 4 min wall clock on one A10 GPU); scripted demo completes at 494 steps (24.7 s).

## Seeds
Seeds: 0 (`object_jitter=0`, fixed starting layout).

## Reset between trials
Unlatch ratchet catch, swing strap out 70 deg, let overlap wing spring open 32 deg, fold lever flat, and center cuff upright at (0.270, 0.000) m on the bench pad.

## Mechanism and sources
| part | real item | model | precedent |
|---|---|---|---|
| skate cuff shell & form | adult inline-skate replacement cuff shell (~260 g molded polymer, 108 mm shell midline diameter, 126 mm top height; ~$10 retail reference: https://www.amazon.com/dp/B07D5Q3M7X) | unbolted 260 g shell + boot sole + 94 mm OD foam ankle form + 120 g `wing` hinge (`0.55 N*m/rad` opening spring to 32 deg, ~7.3 N ear shut load) on fixed rubber `bench_pad` | `#618` / `#1862` sprung hinges |
| notched ladder strap | 18 mm x 68 mm polymer ratchet strap with 6 mm-pitch teeth (~$4 retail reference: https://www.amazon.com/dp/B08P2T8N9R) | 42 g hinged `strap` link (`-70 deg` start) with four 6 mm-pitch tooth bars and 16 mm-thick pinch tab | `#1862` calibrated retention |
| ratchet buckle & lever | spider-style micro-adjustable cuff ratchet buckle with hinged lever (~$4 replacement clasp; ~$18 total BOM) | stainless U-channel receiver + orange catch shroud + 32 g hinged `lever` (`24 x 18 x 20 mm` knob) driving unilateral ratchet pawl retention (`K_LOCK = 25 N*m/rad`, 14 mm slack, 6 mm pitch) | coupled ratchet state machine |

## Real-world build
Parts: replacement inline-skate cuff shell (~$10, https://www.amazon.com/dp/B07D5Q3M7X) + spider ratchet buckle receiver and notched ladder strap kit (~$8, https://www.amazon.com/dp/B08P2T8N9R) mounted on a 94 mm OD foam ankle insert and weighted flat sole plate (~$18 total BOM), placed unbolted on a dark rubber bench mat 270 mm forward of the robot origin.

## Fidelity
Rigid-body simulation (Isaac Sim 6.1, PhysX, 20 Hz control). Simplified: because `labkit` sets `SelfCollisions=False` within an articulation, the entry catch and 6 mm-pitch ratchet teeth cannot rely on intra-part geom contacts; instead, `_rules` + `_drives` enforce coupled joint-state engagement (`|wing| <= 3.5 deg` alignment gate, `>= 28 deg` lever pump cycle, and `K_LOCK = 25 N*m/rad` unilateral pawl retention stiffness holding `hold_angle(notches)` against the 6 N opening spring).

## Licence
All shapes are procedural primitives authored in `inline_skate_ratchet_cuff_buckle.py`; no external third-party mesh, texture, or font files are included (original own work).
