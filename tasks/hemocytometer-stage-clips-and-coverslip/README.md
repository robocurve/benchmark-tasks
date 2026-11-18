# Seat a counting chamber in the microscope stage pocket, latch both stage clips, and lay the coverslip flat

Instruction: "Seat the counting chamber in the stage pocket, latch both stage clips over its shoulders, lay the coverslip flat across the central rails, and clear both grippers."

Files in this folder: `task.json`, `hemocytometer_stage_clips_and_coverslip.py`, `labkit.py`, `mechkit.py`, `pitch_demo.py`, `arena_assets_fix.py`, `test_hemocytometer_stage_clips_and_coverslip.py`, `README.md`, `DEVIATIONS.md`.

## Environment
Isaac Lab-Arena 0.3.1 on Isaac Sim 6.1 (Brev launchable), `inspect-robots` with the `yam_arena` embodiment, `station` scene.
Task file SHA-256 (first 12): see freeze log.

## Offline check (no GPU)
Run `PYTHONDONTWRITEBYTECODE=1 pytest -q -p no:cacheprovider test_hemocytometer_stage_clips_and_coverslip.py` with `yam_arena` on `PYTHONPATH`: verifies initial state scores 0.0, all pure end-state scoring tiers (0.0 / 0.25 / 0.50 / 0.75 / 1.0), 1-hand unbraced datum-ejection and unseated-chamber ablations, unlatched stage-clip springback failure, handling-grip and coverslip clearance gates, shortcuts scoring 0.0, top-camera occlusion proxy clearance, scoring numeric parity, demo purity, and IK reachability.

## Run and score one Dream-1 trial
```
inspect-robots run --embodiment yam_arena -E task=hemocytometer_stage_clips_and_coverslip.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --policy dream \
  --instruction "Seat the counting chamber in the stage pocket, latch both stage clips over its shoulders, lay the coverslip flat across the central rails, and clear both grippers." \
  --scorer task_score --max-steps 800 --seed 0
```

## Scripted demo
```
inspect-robots run --embodiment yam_arena -E task=hemocytometer_stage_clips_and_coverslip.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --policy yam_scripted \
  --instruction "Seat the counting chamber in the stage pocket, latch both stage clips over its shoulders, lay the coverslip flat across the central rails, and clear both grippers." \
  --scorer task_score --max-steps 800 --seed 0
```
`demo()` reads scene geometry constants only (never simulator state). The left gripper clamps the counting chamber's offset 30 x 42 x 24 mm handling grip along world X (`yaw = -90 deg`) and slides the 68 g chamber 34 mm into the funnelled microscope stage pocket at `(0.235, -0.045) m` against the 28 g, 95 N/m front datum plunger, bracing it steady while the right gripper sweeps both 32 g spring-loaded chrome stage clips from 52 deg open into their 28 g, 45 N/m brass detents behind the outer shoulders. Once both clips cage the chamber, the left gripper releases and parks while the right gripper picks the 36 g ground fused-quartz coverslip from the front cradle at `(0.235, -0.148) m`, lays it flat across the 10 mm central H-moat support rails, retracts clear, and parks. Open-loop plan duration is 18.3 s (`366` steps); live runs complete once `1.0` holds continuously for 3.0 s (`run_limit` is 800 steps / 40 s at 20 Hz).

## Scoring
Judged at the end of the trial from pure observable end-state geometry (verified in real trials with a caliper, ruler, and digital protractor); succeeds once 1.0 holds continuously for 3.0 s:
- `1.0`: counting chamber within 14 mm horizontally and 5 mm vertically of the stage pocket and tilted <= 5 deg; both spring stage clips latched <= 19 deg over the outer shoulders; quartz coverslip within 18 mm horizontally and 5 mm vertically of the 10 mm central support rails and tilted <= 6 deg; both grippers > 120 mm clear of the chamber, handling grip, stage clips, and coverslip.
- `0.75`: chamber seated, both clips latched, and coverslip flat on the central rails, gripper within 120 mm.
- `0.50`: chamber seated <= 14 mm and both stage clips latched <= 19 deg, coverslip not yet flat on the central rails.
- `0.25`: chamber seated in the pocket without both stage clips latched.
- `0.0`: otherwise (noop, left-arm-parked unseated chamber at 34 mm, or chamber displaced > 14 mm).
Possible scores 0.0, 0.25, 0.50, 0.75, 1.0.

## Runtime per trial
Runtime per trial: 800 control steps at 20 Hz (40 s of simulated time, about 3 min wall clock on one A10 GPU); scripted demo completes once the 3.0 s continuous hold finishes (~330 steps / 16.5 s).

## Seeds
Seeds: 0 (`object_jitter=0`, fixed starting layout).

## Reset between trials
Lift the 36 g quartz coverslip back to the front staging cradle at `(0.235, -0.148) m`, retract both brass clip detent carriages toward `-Y` so both chrome stage clips spring open to 52 deg, and slide the 68 g counting chamber back to the loading shelf at `(0.235, -0.011) m`.

## Mechanism and sources
| part | real item | model | precedent |
|---|---|---|---|
| microscope stage | anodized aluminum mechanical microscope stage 300 x 430 x 16 mm with condenser port & datum pocket (~$12 fixture, https://www.amazon.com/dp/B07GZJ7R5K) | fixed 18-box compound stage (`friction=0.04`) + `28 g` sprung brass front datum plunger (`95 N/m`) | microscopy stage fixtures |
| stage clips + detents | pair of spring-loaded chrome microscope stage clips + detent catches, 120 g (~$8 retail reference: https://www.amazon.com/dp/B08H881Z3R) | two `32 g` 6-box horizontal-hinge clips (`0.045 N m/rad` opening springs) + two `28 g` 3-box sprung brass detents (`45 N/m`, 18-deg cam ramps) | horizontal cam-detent clips |
| counting chamber + coverslip | double-grid optical glass hemocytometer counting chamber (68 g) + ground fused-quartz cover block (36 g) (~$16 retail reference: https://www.amazon.com/dp/B07D58WZZT) | 13-box `68 g` glass chamber with twin rhodium grids, 10 mm central rails, 9 mm shoulders, and offset handling grip + 5-box `36 g` stepped quartz cover block | wet-lab optical slides |

## Real-world build
Parts: double-grid optical glass cell-counting hemocytometer set with ground quartz cover block (~$16, https://www.amazon.com/dp/B07D58WZZT) + pair of spring-loaded chrome microscope stage clips (~$8, https://www.amazon.com/dp/B08H881Z3R) mounted on a 300 x 430 mm anodized microscope stage deck with datum pocket and front staging cradle (~$12, https://www.amazon.com/dp/B07GZJ7R5K, ~$36 total BOM). Stage the counting chamber on the loading shelf 34 mm outside the pocket at `(0.235, -0.011) m` and place the quartz cover block in the front cradle at `(0.235, -0.148) m`.

## Fidelity
Rigid-body simulation (Isaac Sim 6.1, 20 Hz control). Stage clip retention uses two 32 g horizontal-hinge chrome clips (`0.045 N m/rad` opening springs) engaging separate 28 g sprung brass detent plungers (`45 N/m`, 18 deg cam ramps) behind the 68 g chamber's outer shoulders against a 28 g, `95 N/m` front datum plunger; the fused-quartz coverslip is modelled as a 36 g stepped optical block with a 26 mm frosted handling bridge. Releasing the chamber before latching the clips pushes it back into the clip path so the clips cannot latch (`score <= 0.25`).

## Licence
All shapes are procedural primitives authored in `hemocytometer_stage_clips_and_coverslip.py`; no external third-party mesh, texture, or font files are included (original own work).
