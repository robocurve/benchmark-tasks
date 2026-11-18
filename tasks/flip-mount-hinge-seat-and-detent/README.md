# Seat a hinged optical flip mount on its post, snug its collar clamp, and latch the filter flat in its detent

Instruction: "Seat the flip mount collar on the post shoulder, snug its clamp lever, latch the filter holder flat over the beam port, and clear both grippers."

Files in this folder: `task.json`, `flip_mount_hinge_seat_and_detent.py`, `labkit.py`, `mechkit.py`, `pitch_demo.py`, `arena_assets_fix.py`, `test_flip_mount_hinge_seat_and_detent.py`, `README.md`, `DEVIATIONS.md`.

## Environment
Isaac Lab-Arena 0.3.1 on Isaac Sim 6.1 (Brev launchable), `inspect-robots` with the `yam_arena` embodiment, `station` scene.
Task file SHA-256 (first 12): see freeze log.

## Offline check (no GPU)
Run `PYTHONDONTWRITEBYTECODE=1 pytest -q -p no:cacheprovider test_flip_mount_hinge_seat_and_detent.py` with `yam_arena` on `PYTHONPATH`: verifies initial state scores 0.0, all pure end-state scoring tiers (0.0 / 0.25 / 0.50 / 0.75 / 1.0), 1-hand unbraced post-shift/tip ablation, unlatched springback failure, stabilizer-boss and side-tab gripper clearance gates, shortcuts scoring 0.0, top-camera occlusion proxy clearance, scoring numeric parity, demo purity, and IK reachability.

## Run and score one Dream-1 trial
```
inspect-robots run --embodiment yam_arena -E task=flip_mount_hinge_seat_and_detent.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --policy dream \
  --instruction "Seat the flip mount collar on the post shoulder, snug its clamp lever, latch the filter holder flat over the beam port, and clear both grippers." \
  --scorer task_score --max-steps 800 --seed 0
```

## Scripted demo
```
inspect-robots run --embodiment yam_arena -E task=flip_mount_hinge_seat_and_detent.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --policy yam_scripted \
  --instruction "Seat the flip mount collar on the post shoulder, snug its clamp lever, latch the filter holder flat over the beam port, and clear both grippers." \
  --scorer task_score --max-steps 800 --seed 0
```
`demo()` reads scene geometry constants only (never simulator state). The left gripper clamps the rear 28 x 32 x 20 mm knurled stabilizer boss along world X at `(0.300, 0.135) m` (`yaw = -90 deg`), while the right gripper presses the 64 g split-collar crown straight down 11.2 mm past the 28 g sprung brass post staging detent onto the machined post flat, sweeps the 32 g horizontal brass cam-clamp lever from 48 deg open to ~1 deg snugged, sweeps the 44 g 88-deg upright filter holder forward and presses its knurled side block down past the 32 g sprung lower detent plunger so the horizontal brass catch lip locks the 25.4 mm filter flat over the calibration beam port, retracts both grippers clear, and parks. Open-loop plan duration is 16.3 s (`326` steps); live runs complete once `1.0` holds continuously for 3.0 s (`run_limit` is 800 steps / 40 s at 20 Hz).

## Scoring
Judged at the end of the trial from pure observable end-state geometry (verified in real trials with a caliper, ruler, and digital protractor); succeeds once 1.0 holds continuously for 3.0 s:
- `1.0`: post pedestal within 16 mm horizontally and 6 mm vertically of its breadboard datum pocket and tilted <= 5 deg; split collar within 3 mm of the post shoulder flat; cam-clamp lever snugged <= 15 deg; hinged 25.4 mm filter holder held flat <= 6 deg by the lower detent with the filter centered within 16 mm of the calibration beam port; both grippers > 120 mm clear of the post, rear stabilizer boss, collar, clamp lever, filter ring, and side actuator block.
- `0.75`: post registered, collar seated, clamp snugged, and filter detented flat over the beam port, gripper within 120 mm.
- `0.50`: post registered, collar seated <= 3 mm, and clamp snugged <= 15 deg, filter holder not detented flat over the beam port.
- `0.25`: post registered and filter holder swung below 45 deg without collar seated and snugged.
- `0.0`: otherwise (noop, unbraced post shifted > 16 mm or tilted > 5 deg, or incomplete staging).
Possible scores 0.0, 0.25, 0.50, 0.75, 1.0.

## Runtime per trial
Runtime per trial: 800 control steps at 20 Hz (40 s of simulated time, about 3 min wall clock on one A10 GPU); scripted demo completes once the 3.0 s continuous hold finishes (~293 steps / 14.7 s).

## Seeds
Seeds: 0 (`object_jitter=0`, fixed starting layout).

## Reset between trials
Slide the brass lower detent carriage toward `-Y` to release the filter holder back to its 88 deg upright stop, swing the horizontal cam-clamp lever back to 48 deg open, raise the flip mount collar 11.2 mm onto the post staging detent, and seat the 310 g post pedestal in the breadboard datum channel at `(0.265, -0.025) m`.

## Mechanism and sources
| part | real item | model | precedent |
|---|---|---|---|
| breadboard | anodized aluminum optical breadboard 260 x 300 x 15 mm with collimated sensor port (~$18 fixture, https://www.amazon.com/dp/B07W4F4W4Y) | fixed 22-box compound breadboard (`friction=0.10`) with emerald reticle port and pedestal datum channel | optical bench fixtures |
| optical post + pedestal | 12.7 mm stainless optical post + slotted pedestal base + staging & lower detent brackets, 310 g (~$22 retail reference: https://www.thorlabs.com/thorproduct.cfm?partnumber=TR3) | 12-box free body (`310 g`) + `28 g` sprung brass staging detent (`520 N/m`) + `32 g` sprung brass lower detent (`110 N/m`, 16-deg ramp + horizontal catch lip) | optical post assemblies |
| hinged flip mount | 90-degree hinged 25.4 mm (1-inch) optical filter flip mount + bandpass glass, 140 g (~$49 retail reference: https://www.thorlabs.com/thorproduct.cfm?partnumber=FM90) | 22-box articulated mount: `64 g` split collar (`2.6 N` slider) + `32 g` horizontal cam lever (`0.32 N m` vertical-axis hinge) + `44 g` hinged filter cell (`0.28 N m/rad` return spring) | hinged detent mounts |

## Real-world build
Parts: 90-degree hinged 25.4 mm optical filter flip mount with 1-inch bandpass glass (~$49, https://www.thorlabs.com/thorproduct.cfm?partnumber=FM90) + 12.7 mm stainless optical post and slotted pedestal base (~$22, https://www.thorlabs.com/thorproduct.cfm?partnumber=TR3) on a 260 x 300 mm anodized optical breadboard with datum channel (~$18, https://www.amazon.com/dp/B07W4F4W4Y, ~$89 total BOM). Place the post pedestal in the datum channel at `(0.265, -0.025) m` and pre-stage the collar 11.2 mm above the post shoulder flat.

## Fidelity
Rigid-body simulation (Isaac Sim 6.1, 20 Hz control). Split-collar clamping on the 12.7 mm post uses a 28 g sprung brass staging detent (`520 N/m`) and a `2.6 N` prismatic post slider paired with a `0.32 N m` horizontal cam-clamp hinge lever rather than elastic metal flexure; the lower beam-path latch uses a separate 32 g sprung brass plunger (`110 N/m`) with a 16 deg cam ramp and horizontal catch lip holding against the `0.28 N m/rad` hinge return spring. Releasing before the detent snaps the holder upright (`score = 0.50`); actuating with one hand shifts the unbraced 310 g pedestal past 16 mm (`score = 0.0`).

## Licence
All shapes are procedural primitives authored in `flip_mount_hinge_seat_and_detent.py`; no external third-party mesh, texture, or font files are included (original own work).
