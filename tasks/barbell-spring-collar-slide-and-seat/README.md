# Squeeze a spring collar open and slide it past the sleeve shoulder flush against the outer weight plate

Instruction: "Seat the spring collar past the sleeve shoulder flush against the outer plate clamped onto the barbell sleeve without shifting the bar, and clear both grippers."

Files in this folder: `task.json`, `barbell_spring_collar_slide_and_seat.py`, `labkit.py`, `mechkit.py`, `pitch_demo.py`, `arena_assets_fix.py`, `test_barbell_spring_collar_slide_and_seat.py`, `README.md`, `DEVIATIONS.md`.

## Environment
Isaac Lab-Arena 0.3.1 on Isaac Sim 6.1 (Brev launchable), `inspect-robots` with the `yam_arena` embodiment, `station` scene.
Task file SHA-256 (first 12): see freeze log.

## Offline check (no GPU)
Run `PYTHONDONTWRITEBYTECODE=1 pytest -q -p no:cacheprovider test_barbell_spring_collar_slide_and_seat.py` with `yam_arena` on `PYTHONPATH`: verifies initial state scores 0.0, all pure end-state scoring tiers (0.0 / 0.25 / 0.50 / 0.75 / 1.0), 1-hand unbraced bar-shift ablation, unsqueezed shoulder-blocked failure, left-gripper-on-stabilizer-handle clearance gate, shortcuts scoring 0.0, top-camera occlusion proxy clearance, scoring numeric parity, demo purity, and IK reachability.

## Run and score one Dream-1 trial
```
inspect-robots run --embodiment yam_arena -E task=barbell_spring_collar_slide_and_seat.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --policy dream \
  --instruction "Seat the spring collar past the sleeve shoulder flush against the outer plate clamped onto the barbell sleeve without shifting the bar, and clear both grippers." \
  --scorer task_score --max-steps 440 --seed 0
```

## Scripted demo
```
inspect-robots run --embodiment yam_arena -E task=barbell_spring_collar_slide_and_seat.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --policy yam_scripted \
  --instruction "Seat the spring collar past the sleeve shoulder flush against the outer plate clamped onto the barbell sleeve without shifting the bar, and clear both grippers." \
  --scorer task_score --max-steps 440 --seed 0
```
`demo()` reads scene geometry constants only (never simulator state). The left gripper clamps the radial 28 mm knurled stabilizer handle along world X at `(0.330, 0.135) m` (`yaw = -90 deg`), while the right gripper straddles the 320 g spring collar handles at `(0.187, -0.025) m`, squeezes both crossed wire jaws open past the 58 mm sleeve shoulder against 35 N preload, slides the collar flush against the outer weight plate hub while the left gripper braces the 780 g barbell assembly in its cradle, opens in place so the spring jaws clamp the 50 mm sleeve, retracts both grippers clear, and parks. Open-loop plan duration is 13.9 s (`278` steps); live runs complete once `1.0` holds continuously for 3 s (`run_limit` is 440 steps / 22 s at 20 Hz).

## Scoring
Judged at the end of the trial from pure observable end-state geometry (verified in real trials with a caliper, ruler, and digital protractor); succeeds once 1.0 holds continuously for 3 s:
- `1.0`: barbell within 12 mm horizontally and 8 mm vertically of its stand cradle and tilted <= 6 deg; collar coaxial within 10 mm and <= 12 deg of sleeve; collar rests past the shoulder flush within 8 mm of the outer plate hub; both spring jaws clamped <= 4 mm on the sleeve; both grippers > 120 mm clear of the collar, barbell sleeve, and radial stabilizer handle.
- `0.75`: collar flush within 8 mm of the outer plate past the shoulder and clamped <= 4 mm with bar in cradle, gripper within 120 mm.
- `0.50`: collar flush within 8 mm of the outer plate past the shoulder with bar in cradle, spring jaws not clamped <= 4 mm.
- `0.25`: barbell seated and collar coaxial and slid >= 30 mm inward along sleeve, not yet flush within 8 mm of the outer plate past the shoulder.
- `0.0`: otherwise (noop, unsqueezed push blocked behind the 58 mm shoulder, off-axis drop, or unbraced bar shifted > 12 mm).
Possible scores 0.0, 0.25, 0.50, 0.75, 1.0.

## Runtime per trial
Runtime per trial: 440 control steps at 20 Hz (22 s of simulated time, about 3 min wall clock on one A10 GPU); scripted demo completes once the 3 s continuous hold finishes (~255 steps / 12.8 s).

## Seeds
Seeds: 0 (`object_jitter=0`, fixed starting layout).

## Reset between trials
Seat the 780 g barbell sleeve assembly on the two stand saddles at `(0.295, -0.025) m` with the outer plate hub at `x = 0.285 m`, squeeze the 320 g spring collar open, slide it back outside the 58 mm shoulder to `x = 0.187 m`, and release so it clamps the outer pilot.

## Mechanism and sources
| part | real item | model | precedent |
|---|---|---|---|
| cradle stand | steel benchtop barbell V-cradle 310 x 240 mm with polymer channel liners and 3 mm rubber feet (~$5 shop fixture) | fixed 17-box compound stand (`friction=0.10`) with 282 mm saddle wheelbase | benchtop cradle fixtures |
| barbell sleeve assembly | 50 mm (2-inch) standard loadable sleeve + 118 mm and 134 mm fractional rubber plates + 28 mm radial grip handle, 780 g (~$12 retail reference: https://www.amazon.com/dp/B07B42BV29) | 23-geom compound rigid body (`780 g`): 128 mm chrome sleeve, 26 mm anti-rotation flats, 58 mm shoulder lugs, two color-coded plates, 72 mm flange, 160 mm radial knurled handle | loadable sleeve hardware |
| spring clip collar | 50 mm (2-inch) standard three-turn chrome spring barbell collar with red vinyl handles, 320 g (~$8 retail reference: https://www.amazon.com/dp/B017763GQE) | 43-geom articulated assembly (`250 g` 3-turn 8-gon coil + 50 mm concentric guide bushings + two `35 g` crossed spring-wire `slide` jaws at `2200 N/m`, `35 N` at `8 mm` stroke, `63 mm` open span) | crossed spring-clip mechanism |

## Real-world build
Parts: 50 mm (2-inch) standard loadable short sleeve adapter with two fractional rubber plates (~$12, https://www.amazon.com/dp/B07B42BV29) + 50 mm dual-handle chrome spring clip collar (~$8, https://www.amazon.com/dp/B017763GQE) on a 310 x 240 mm low-friction polymer-lined bench cradle (~$5, ~$25 total BOM). Place the stand at `(0.295, -0.025) m` with the sleeve centerline 76 mm above the tabletop and pre-mount the collar on the outer pilot at `x = 0.187 m`.

## Fidelity
Rigid-body simulation (Isaac Sim 6.1, 20 Hz control). Simplified: helical three-coil torsion is represented as two crossed prismatic spring-wire stirrup jaws (`2200 N/m` per leg, `0.0` joint friction, non-overlapping handles) inside an 8-sided three-turn coil housing with 50 mm concentric guide skids and 26 mm sleeve anti-rotation flats. Pushing without squeezing blocks behind the 58 mm shoulder; pushing with one hand only slides the unbraced 780 g barbell past 12 mm out of its cradle (`score = 0.0`).

## Licence
All shapes are procedural primitives authored in `barbell_spring_collar_slide_and_seat.py`; no external third-party mesh, texture, or font files are included (original own work).
