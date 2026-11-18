# Release the card-file follower clutch, slot the tabbed divider upright, and slide the follower snug until it locks

Instruction: "Release the follower clutch, slot the tabbed divider upright between the two card packs, slide the follower snug until it locks, and clear both grippers."

Files in this folder: `task.json`, `card_file_follower_block_compress_and_lock.py`, `labkit.py`, `mechkit.py`, `pitch_demo.py`, `arena_assets_fix.py`, `test_card_file_follower_block_compress_and_lock.py`, `README.md`, `DEVIATIONS.md`.

## Environment
Isaac Lab-Arena 0.3.1 on Isaac Sim 6.1 (Brev launchable), `inspect-robots` with the `yam_arena` embodiment, `station` scene.
Task file SHA-256 (first 12): see freeze log.

## Offline check (no GPU)
Run `PYTHONDONTWRITEBYTECODE=1 pytest -q -p no:cacheprovider test_card_file_follower_block_compress_and_lock.py` with `yam_arena` on `PYTHONPATH`: verifies initial state scores 0.0, all pure end-state scoring tiers (0.0 / 0.25 / 0.50 / 0.75 / 1.0), 1-hand unbraced tray-shift ablation, unreleased-clutch closed-pack divider block, unlocked follower back-slip failure, proximity clearance gates, shortcuts scoring 0.0, top-camera sight-line clearance, scoring numeric parity, demo purity, and IK reachability.

## Run and score one Dream-1 trial
```
inspect-robots run --embodiment yam_arena -E task=card_file_follower_block_compress_and_lock.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --policy dream \
  --instruction "Release the follower clutch, slot the tabbed divider upright between the two card packs, slide the follower snug until it locks, and clear both grippers." \
  --scorer task_score --max-steps 800 --seed 0
```

## Scripted demo
```
inspect-robots run --embodiment yam_arena -E task=card_file_follower_block_compress_and_lock.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --policy yam_scripted \
  --instruction "Release the follower clutch, slot the tabbed divider upright between the two card packs, slide the follower snug until it locks, and clear both grippers." \
  --scorer task_score --max-steps 800 --seed 0
```
`demo()` reads scene geometry constants only (never simulator state). The left gripper clamps the steel card-file tray's offset 30 x 42 x 24 mm rear stabilizer handle along world X (`yaw = -90 deg`) at `(0.295, 0.145) m` while the right gripper presses the front `-X` red cam-clutch release paddle inward so the rear locking tooth swings clear and the 110 N/m rear card-pack spring plus 65 N/m follower spring slide the 52 g follower block 34 mm back, opening a 42 mm slot between the two upright card packs. The right gripper then lifts the 42 g amber-tabbed index divider from the front oak cradle at `(0.235, -0.172) m`, slots it upright between the packs, pushes the follower block forward past the 20 deg cam ramp until the seesaw clutch snaps flat behind the keeper heel, and both arms retract clear and park. Open-loop plan duration is 16.9 s (`338` steps); live runs complete once `1.0` holds continuously for 3.0 s (`run_limit` is 800 steps / 40 s at 20 Hz).

## Scoring
Judged at the end of the trial from pure observable end-state geometry (verified in real trials with a caliper, ruler, and digital protractor); succeeds once 1.0 holds continuously for 3.0 s:
- `1.0`: card-file tray within 16 mm horizontally and 6 mm vertically of its desk blotter datum zone and tilted <= 5 deg; tabbed index divider upright <= 10 deg on the tray floor within 22 mm laterally and 15 mm longitudinally of the slot between the two card packs; follower block inside its 12 to 26 mm divider-installed snug window from open rest with the cam-clutch lever locked <= 8 deg; both grippers > 120 mm clear of the tray, rear handle, divider, follower block, and clutch paddle.
- `0.75`: tray registered, divider upright between the card packs, and follower snug (12 to 26 mm) and locked, gripper within 120 mm.
- `0.50`: tray registered, divider upright between the card packs, and follower inside the 12 to 26 mm snug window, cam-clutch lever not locked <= 8 deg.
- `0.25`: tray registered and divider upright between the card packs, follower block outside the 12 to 26 mm snug window.
- `0.0`: otherwise (noop, divider resting atop closed packs without releasing clutch, or unbraced tray shifted > 16 mm).
Possible scores 0.0, 0.25, 0.50, 0.75, 1.0.

## Runtime per trial
Runtime per trial: 800 control steps at 20 Hz (40 s of simulated time, about 3 min wall clock on one A10 GPU); scripted demo completes once the 3.0 s continuous hold finishes (~305 steps / 15.3 s).

## Seeds
Seeds: 0 (`object_jitter=0`, fixed starting layout).

## Reset between trials
Press the red `-X` cam-clutch paddle inward, lift the 42 g tabbed divider back to the front oak cradle at `(0.235, -0.172) m`, push the 52 g follower block 34 mm forward until the first ratchet lug latches with the two card packs 8 mm apart, and seat the 260 g steel tray on the desk blotter at `(0.235, -0.060) m`.

## Mechanism and sources
| part | real item | model | precedent |
|---|---|---|---|
| steel card-file box + desk mat | open-top steel 4x6 desktop card-file tray with centre rod (~$14 retail reference: https://www.amazon.com/dp/B001GIP20S) on 300 x 430 mm desk blotter | `260 g` 15-box steel tray + fixed 13-box desk blotter (`friction=0.03`) | desktop organizers |
| cam-clutch follower + rear pack | spring-clutch steel follower block + ruled index-card pack (~$4 component) | `52 g` 7-box follower (`65 N/m`) + `34 g` rear pack (`110 N/m`) + `30 g` 5-box seesaw clutch lever (`0.11 N m/rad`, 20-deg cam ramp) | horizontal cam-detent latches |
| tabbed index divider | rigid pressboard 4x6 A-Z index guide divider (~$4 set reference: https://www.amazon.com/dp/B00006IBUV) | `42 g` 7-box tabbed pressboard divider on 20 mm weighted brass base shoe | upright slotted cards |

## Real-world build
Parts: open-top steel 4x6 desktop index-card file tray with spring-clutch follower block (~$14, https://www.amazon.com/dp/B001GIP20S) + rigid pressboard 4x6 A-Z tabbed index dividers and ruled index cards (~$8, https://www.amazon.com/dp/B00006IBUV, ~$22 total BOM) on a smooth desk blotter. Lock the follower block forward against the two card packs (8 mm gap) and stage one upright tabbed divider in the front cradle at `(0.235, -0.172) m`.

## Fidelity
Rigid-body simulation (Isaac Sim 6.1, 20 Hz control). Card-stack expansion and follower rod stiction use a 34 g, `110 N/m` movable rear card pack and a 52 g, `65 N/m` steel follower block engaging a separate 30 g, `0.11 N m/rad` horizontal seesaw cam-clutch lever (20 deg non-locking cam ramp, flat keeper heel, and two ratchet positions: 34 mm empty-pack lock and 12 to 26 mm divider-installed lock); the index divider is a 42 g rigid tabbed block with a 20 mm weighted base shoe. Pushing the follower without bracing the tray shifts the tray past 16 mm (`score = 0.0`).

## Licence
All shapes are procedural primitives authored in `card_file_follower_block_compress_and_lock.py`; no external third-party mesh, texture, or font files are included (original own work).
