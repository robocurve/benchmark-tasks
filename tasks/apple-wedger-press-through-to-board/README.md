# Center a dual-handle apple wedger over a seated apple and press level through to the board

Instruction: "Drive the apple wedger level through the seated apple flush onto the cutting board while keeping the apple centered in its cradle, and clear both grippers."

Files in this folder: `task.json`, `apple_wedger_press_through_to_board.py`, `labkit.py`, `mechkit.py`, `pitch_demo.py`, `arena_assets_fix.py`, `test_apple_wedger_press_through_to_board.py`, `README.md`, `DEVIATIONS.md`.

## Environment
Isaac Lab-Arena 0.3.1 on Isaac Sim 6.1 (Brev launchable), `inspect-robots` with the `yam_arena` embodiment, `station` scene.
Task file SHA-256 (first 12): see freeze log.

## Offline check (no GPU)
Run `PYTHONDONTWRITEBYTECODE=1 pytest -q -p no:cacheprovider test_apple_wedger_press_through_to_board.py` with `yam_arena` on `PYTHONPATH`: verifies initial state scores 0.0, all scoring tiers (0.0 / 0.25 / 0.50 / 0.75 / 1.0), 1-hand cocking/unheld ablation, displaced cradle failure, shortcuts scoring 0.0, top-camera occlusion proxy clearance, scoring numeric parity, demo purity, and IK reachability.

## Run and score one Dream-1 trial
```
inspect-robots run --embodiment yam_arena -E task=apple_wedger_press_through_to_board.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --policy dream \
  --instruction "Drive the apple wedger level through the seated apple flush onto the cutting board while keeping the apple centered in its cradle, and clear both grippers." \
  --scorer task_score --max-steps 440 --seed 0
```

## Scripted demo
```
inspect-robots run --embodiment yam_arena -E task=apple_wedger_press_through_to_board.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --policy yam_scripted \
  --instruction "Drive the apple wedger level through the seated apple flush onto the cutting board while keeping the apple centered in its cradle, and clear both grippers." \
  --scorer task_score --max-steps 440 --seed 0
```
`demo()` reads scene geometry constants only (never simulator state). Both grippers splay outward 18 deg, straddle the +-69 mm raised grip spines above the wing flanges, pinch for 1.0 s, lift the 210 g wedger, center it over the 180 g apple on the cutting board, register level on the crown, drive the 48 mm cut-through stroke cleanly to the cutting board deck, open in place, retract clear, and park for 3 s. Open-loop plan duration is 12.4 s; live runs complete in ~12.3 s (~245 steps) once 1.0 holds for 0.5 s (`run_limit` is 440 steps / 22 s at 20 Hz).

## Scoring
Judged at the end of the trial using physical end-state geometry (verified in real trials with a caliper/ruler and digital protractor); succeeds once 1.0 holds for 0.5 s:
- `1.0`: apple within 12 mm of cradle center and tilted <= 8 deg; wedger within 10 mm radially of apple; cut depth >= 42 mm of 48 mm stroke (against 18 N / 4.5 mm yield); wedger rests flush within 4 mm of deck and <= 5 deg tilt; both grippers > 120 mm clear of wedger and apple for 3 s (60 steps).
- `0.75`: cut complete and flush on deck and clear, hold < 3 s.
- `0.50`: cut complete and flush on deck, gripper within 120 mm.
- `0.25`: initial cut >= 10 mm, incomplete stroke or not flush/tilted > 5 deg on deck.
- `0.0`: otherwise (noop, sweep, off-center drop, or 1-hand cocked jam).
Possible scores: `0.0`, `0.25`, `0.50`, `0.75`, `1.0`.

## Runtime per trial
Runtime per trial: 440 control steps at 20 Hz (22 s of simulated time, about 3 min wall clock on one A10 GPU); scripted demo completes once the 0.5 s hold finishes (~245 steps / 12.3 s).

## Seeds
Seeds: 0 (`object_jitter=0`, fixed starting layout).

## Reset between trials
Lift wedger back to the dark staging mat at x = 0.200 m, reset the apple core anvil inside the 77 mm recessed cutting board cradle at x = 0.345 m.

## Mechanism and sources
| part | real item | model | precedent |
|---|---|---|---|
| cutting board | bamboo/wood 170 x 200 x 18 mm board with 77 mm pocket (~$5 retail reference: https://www.amazon.com/dp/B07JQ3L8T4) | fixed 14-box compound board with walnut cradle trim and 4 corner rubber feet | recessed station fixtures |
| apple specimen | medium 72 mm / 180 g apple (~$2 grocery produce; USDA standard ~182 g) | 28-geom 8-wedge red-skin/cream-flesh assembly + 90 g prismatic core slide (`18 N`, `4.5 mm` yield, `48 mm` stroke) | `#1862` calibrated yield slide |
| apple wedger | dual-handle stainless 8-blade corer/wedger 108 mm OD, 23 mm OD / 18.4 mm ID corer, 160 mm grip span (~$13 retail reference: https://www.amazon.com/dp/B00004OCKT) | 46-geom compound rigid body (210 g): 12-gon blade ring (`36 mm`) + collar (`42 mm`), 8 radial blades, 8-gon hollow corer, dual wing flanges & 20 mm grip bosses | dual-handle bimanual tool |

## Real-world build
Parts: dual-handle 8-blade stainless apple wedger/corer (~$13, https://www.amazon.com/dp/B00004OCKT) + medium 72 mm apple / foam cut-test specimen (~$2) on a routed 170 x 200 x 18 mm wooden board with a 6 mm deep, 77 mm square cradle pocket and rubber feet (~$5, ~$20 total BOM). Position the cutting board center 345 mm forward of the robot origin and the dark silicone staging mat at 200 mm.

## Fidelity
Rigid-body simulation (Isaac Sim 6.1, PhysX, 20 Hz control). Simplified: continuous fruit flesh cutting is modelled as an 8-wedge pre-scored assembly around a central prismatic joint governed by an irreversible 18 N (4000 N/m, 4.5 mm yield) severing law (`_advance_cut_yield`), rather than finite-element fracture or blade edge wear. Pressing one handle only creates an overturning moment that cocks the blade ring > 5 deg and physically jams against the wedges before cut-through.

## Licence
All shapes are procedural primitives authored in `apple_wedger_press_through_to_board.py`; no external third-party mesh, texture, or font files are included (original own work).
