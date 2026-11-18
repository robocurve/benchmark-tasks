# Dispense a mash portion from a thumb-lever disher into a ramekin and return it to its tray

Instruction: "Dispense the mash portion cleanly into the centered ramekin by drawing the thumb lever, return the disher to its rest tray, and clear both grippers."

Files in this folder: `task.json`, `portion_disher_level_and_sweep_release.py` (scene, release rule, scorer, scripted demo),
`labkit.py` (multi-shape parts), `mechkit.py` (compound bodies, joints, bonds), `pitch_demo.py` (full-orientation
waypoints), `arena_assets_fix.py` (library asset paths), `test_portion_disher_level_and_sweep_release.py` (offline tests),
`README.md`, `DEVIATIONS.md`.

## Environment
Isaac Lab-Arena 0.3.1 on Isaac Sim 6.1 (Brev launchable), `inspect-robots` with the `yam_arena` embodiment, `station` scene.
Task file SHA-256 (first 12): see the freeze log.

## Offline check (no GPU)
Run `PYTHONDONTWRITEBYTECODE=1 pytest -q -p no:cacheprovider test_portion_disher_level_and_sweep_release.py` with `yam_arena` on `PYTHONPATH`: verifies initial state scores 0.0, all scoring tiers (`0.0`, `0.4`, `0.7`, `1.0`), each scorer sub-gate flipped independently (incomplete sweep, missed mash, unparked/tilted disher, displaced/tipped ramekin and tub, gripper within 45 mm), listed shortcuts (shaking/flipping without clearing the rack detent), top-camera line-of-sight clearance, scoring number parity, demo reachability, and demo purity.

## Run and score one Dream-1 trial
```
inspect-robots run --embodiment yam_arena -E task=portion_disher_level_and_sweep_release.py -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 --policy dream \
  --instruction "Dispense the mash portion cleanly into the centered ramekin by drawing the thumb lever, return the disher to its rest tray, and clear both grippers." \
  --scorer task_score --max-steps 780 --seed 0
```

## Scripted demo
```
inspect-robots run --embodiment yam_arena -E task=portion_disher_level_and_sweep_release.py -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 --policy yam_scripted \
  --instruction "Dispense the mash portion cleanly into the centered ramekin by drawing the thumb lever, return the disher to its rest tray, and clear both grippers." \
  --scorer task_score --max-steps 780 --seed 0
```
`demo()` reads scene geometry constants only (never simulator state). The right gripper clamps the T-pommel sleeve on the disher handle, lifts it off the rest tray, and holds the loaded bowl centered above the ramekin. The left gripper pinches the flanged amber thumb tab and draws the rack 30 mm along +Y, sliding its detent block clear so the internal wiper sweeps past 150 deg and releases the 65 g mash plug into the ramekin. The left gripper retracts clear; the right arm returns the empty disher onto its rest tray, releases, and both arms park >= 45 mm clear for > 3.0 s (`run_limit` is 780 steps / 39 s at 20 Hz).

## How it is modelled
| Mechanism | Physical joint / spring / contact | Source for the numbers |
|---|---|---|
| Bowl & handle | 54 mm mouth across flats, rolled rim at 99 mm over table at rest, elastomer T-pommel grip (`DISHER_M = 0.099 kg`) | size-16 (2 oz) stainless portion disher, Real-world build |
| Thumb rack | prismatic joint along +y, 30 mm stroke (`RACK_STROKE = 0.030`), 60 N/m return spring + 1.0 N Coulomb friction | geared thumb lever ratio softened to arm push budget (DEVIATIONS) |
| Wiper sweep & detent | hinge about bowl axis (`0..180 deg`, `0.005 N*m/rad` toward 180 deg) physically blocked at ~20 deg by rack detent block until rack >= 27 mm | internal scraper gear & travel stop |
| Mash cohesion | breakable `mechkit` bond (`200 N`) on 65 g stepped plug, released once `thumb_rack >= 27 mm` and `wiper_sweep >= 150 deg` | cohesive food plug adhesion (cannot drop by shaking) |
| Unbolted vessels | 95 g ramekin and 280 g supply tub on 9 mm rubber coasters | bimanual stabilization requirement |

Every scoring threshold in `task.json` matches a named constant in `portion_disher_level_and_sweep_release.py` (`CLEAR`, `RAMEKIN_TRAVEL`, `TUB_TRAVEL`, `VESSEL_TILT_DEG`, `RACK_FULL`, `SWEEP_MIN_DEG`, `PARK_XY`, `PARK_Z`, `PARK_TILT_DEG`, `MASH_XY`, `MASH_Z`, `SUCCESS_HOLD_S`).

## Scoring
Judged purely from physical end-state geometry (verified on bench with a scale, caliper/ruler, and angle gauge); succeeds once 1.0 holds for 3.0 s:
- `1.0`: `sweep_deg >= 150 deg`, mash plug inside ramekin (`<= 20 mm` off axis, `12..70 mm` height), ramekin within `20 mm` and `8 deg`, tub within `25 mm` and `8 deg`, disher back on tray (`<= 45 mm` XY, `<= 40 mm` Z, `<= 15 deg` tilt), and both grippers `>= 45 mm` clear.
- `0.7`: `sweep_deg >= 150 deg`, mash in ramekin, vessels intact, but disher unparked or a gripper within `45 mm`.
- `0.4`: `sweep_deg >= 150 deg`, vessels intact, but mash missed the ramekin.
- `0.0`: otherwise (displaced/tipped vessel, or `sweep_deg < 150 deg` from shaking/flipping without clearing the rack detent).
Possible scores: `0.0`, `0.4`, `0.7`, `1.0`.

## Real-world build
- Size-16 (2 oz, 54 mm bowl) stainless thumb-lever portion disher (~$16–$28, e.g. Winco LDI-16 https://www.wincofoodservice.com/products/ldi-16 or Vollrath 47160).
- Ceramic ramekin (95 g, ~70 mm mouth, 350 mL, ~$1.50) and PP deli mash tub (280 g filled, ~$0.30).
- Black silicone bench mat (460 x 460 mm, ~$9) and two 9 mm rubber coaster rings (~$0.50 each).
- Total BOM for one trial set: ~$28 USD at 1:1 scale.

Real-trial verification protocol:
- Weigh the ejected portion in the ramekin on a 0.1 g scale (`65 g +/- 3 g`) and inspect the bowl to confirm complete wiper sweep (`>= 150 deg`).
- Measure ramekin (`<= 20 mm`), tub (`<= 25 mm`), and rest-tray (`<= 45 mm`) mark offsets with a ruler/caliper and level tilt (`<= 8 deg` / `<= 15 deg`) with an angle gauge.

## Runtime per trial
780 control steps max at 20 Hz (39 s simulated time); the scripted demo finishes once the 3.0 s hold completes (~468 steps / 23.4 s).

## Seeds
Seed 0 (`object_jitter=0`, fixed starting layout).

## Reset between trials
Reset wiper to 0 deg behind the rack detent, re-seat a fresh 65 g mash plug in the bowl, return disher to its rest tray, and center ramekin and tub on their coasters.

## Licence
All shapes are procedural primitives authored in `portion_disher_level_and_sweep_release.py`; no external third-party mesh, texture, or font files are included (original own work).
