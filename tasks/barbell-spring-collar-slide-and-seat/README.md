# Thread a spring clip collar onto a loadable dumbbell sleeve and seat it flush against the weight plate

Instruction: "Squeeze the spring clip collar open, thread it onto the dumbbell sleeve, slide it flush against the outer weight plate, release it so the wire jaws grip the sleeve, and clear both grippers."

Files in this folder: `task.json`, `barbell_spring_collar_slide_and_seat.py`, `labkit.py`, `mechkit.py`, `pitch_demo.py`, `arena_assets_fix.py`, `test_barbell_spring_collar_slide_and_seat.py`, `README.md`, `DEVIATIONS.md`.

## Environment
Isaac Lab-Arena 0.3.1 on Isaac Sim 6.1, `inspect-robots` with the `yam_arena` embodiment, `station` scene.

## What the task is
A 1.20 kg loadable dumbbell sits in a benchtop cradle stand. The near 49 mm chrome sleeve is loaded with two rubber-coated fractional plates; the outer plate face is the axial stop. A stock 0.320 kg three-turn spring clip collar stands upright on its coil in a felt-lined timber holster 150 mm off the sleeve axis, **off the bar** — the collar starts exactly where a person would set it down. Because a spring clip collar is a closed ring whose crossed wire jaws are preloaded 4 mm inside the sleeve at rest, it cannot be pushed onto the sleeve unsqueezed and cannot be brought on from the side: the robot must squeeze both handles open, carry the collar to the bar's axis, thread it over the free end, slide it home against the plate face, and release so the wire jaws grip the sleeve. The retention test in a real trial is a spring-scale pull; in simulation it is the released, out-of-contact end state (below).

## Scoring
Judged from physical end-state geometry with a caliper, ruler and digital protractor. Possible scores **0.0, 0.25, 0.50, 1.0** (no 0.75 rung: this revision merges the old "clamped but a gripper still in the way" rung into 0.50).

| Score | What holds | Real-trial reading |
|---|---|---|
| **1.0** | dumbbell seated (<= 12 mm horizontal, <= 8 mm vertical, <= 6 deg tilt); collar coaxial (<= 10 mm, <= 12 deg); foremost surface **flush <= 3 mm** of the outer plate face; **both grippers > 120 mm clear** | feeler gauge or ruler between the collar's foremost part and the plate: gap <= 3 mm; protractor: bar and collar square; both hands off and away |
| **0.50** | as 1.0 but gap up to 8 mm, **or** a gripper still within 120 mm | seated and squared but not quite home, or a hand still on it |
| **0.25** | dumbbell seated and collar coaxial and slid >= 30 mm in along the sleeve, not yet within 8 mm of the plate | collar on the sleeve, not yet at the plate |
| **0.0** | otherwise: nothing done, collar left in the holster, unsqueezed collar stopped at the sleeve end, collar dropped off-axis or cocked, dumbbell knocked out of its cradle | any failure to get it on and home |

The run succeeds once 1.0 holds continuously for 3 s (60 steps at 20 Hz). `run_limit` is 900 steps / 45 s; the scripted demo's plan is 28.9 s (578 steps), so the plan plus the required 3 s hold finishes well inside the limit.

## Why the check can fail (power, not just a threshold)
The scored quantities are pure end-state geometry read from the two object poses, with no joint displacement, no step-accumulated state and no code-held target. The retention evidence was verified load-bearing by a reviewer-side one-variable ablation run against this task file with the offline pre-check: play the shipped demo, then partway through the plan's own retention pull either leave the collar's wire spring live or zero its actuator force and let the tug finish. The kill instant is derived from the plan's retention-pull waypoint (21.4 s on this plan), never hardcoded: a fixed second stops testing the retention phase as soon as the plan's waypoints move, and it then silently prints 1.0 on both arms.
- spring live: gap 1.3 mm, radial 6.4 mm, 8.62 N in the wire, score **1.0** (the released jaws grip the sleeve and
  the collar stays at the plate)
- spring killed: the same plan drags the collar to a 14.0 mm gap with the radial error growing to 7.4 mm and no
  force in the jaws, score **0.25**

Both numbers are from this tree; the kill instant used was 21.40 s, the pull starting at 20.90 s.

The two answers differ, so the retention evidence has power. A separate closed-loop negative run on the same task file shows an unsqueezed collar never reaches the sleeve: with the squeeze replaced by a light touch the collar stays in the holster and the run ends at 0.0.

The flush gate is proved the same way, on the bench reading a person makes, by three unit cases in `test_barbell_spring_collar_slide_and_seat.py`: a collar stopped 5 mm short of contact scores 0.50; a collar that reaches the plate but whose jaws do not grip, so the retention tug walks it 20 mm back out, scores 0.25; seated and gripping scores 1.0. Three different answers for the same collar, bar and grippers, so the gate is load-bearing. A fourth test measures the geometry the gate rests on: the collar's collision hull is 33.0 mm along the sleeve, its foremost plate-side surface is the two plastic squeeze handles at 16.5 mm, the coil face is 13.5 mm, and the gap is read from the handles.

## Offline check (no GPU)
```
PYTHONDONTWRITEBYTECODE=1 pytest -q -p no:cacheprovider test_barbell_spring_collar_slide_and_seat.py
```
with `yam_arena` on `PYTHONPATH` (17 tests, all passing). Covers: the tray start scores 0.0; every ladder rung (0.0 / 0.25 / 0.50 / 1.0) is reachable from `components()` and matches `task.json`; the single-hand bar-shift ablation; the unsqueezed-collar failure; off-axis / cocked collar failures; the jaw spring geometry is a real preloaded interference fit that clears at full stroke; no code-held scored state; plan purity and IK reach < 2 mm; `run_limit` headroom; threshold + rung parity between the scorer, `task.json` and this table; top-camera sight lines clear at the demo end.

## Run and score one Dream-1 trial
```
inspect-robots run --embodiment yam_arena -E task=barbell_spring_collar_slide_and_seat.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --policy dream \
  --instruction "Squeeze the spring clip collar open, thread it onto the dumbbell sleeve, slide it flush against the outer weight plate, release it so the wire jaws grip the sleeve, and clear both grippers." \
  --scorer task_score --max-steps 900 --seed 0
```

## Scripted demo
```
inspect-robots run --embodiment yam_arena -E task=barbell_spring_collar_slide_and_seat.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --policy yam_scripted \
  --instruction "Squeeze the spring clip collar open, thread it onto the dumbbell sleeve, slide it flush against the outer weight plate, release it so the wire jaws grip the sleeve, and clear both grippers." \
  --scorer task_score --max-steps 900 --seed 0
```
`demo()` reads scene geometry constants only (never simulator state). The left gripper clamps the far 134 mm plate across its faces 60 mm off the axis and holds the dumbbell for the whole operation, its hold derived from the other arm's own schedule so it always covers the retention pull; the right gripper straddles the collar's two vinyl squeeze handles in the holster, squeezes them open against the jaw preload, lifts the open collar out, carries it round the bar's free end, threads it over the end and along the sleeve to the plate, opens its fingers over a full second while still holding the collar pressed against the plate so the released jaws clamp onto the sleeve with the collar home, retracts clear, applies the retention pull on the coil (a light friction-limited pinch, the spring-scale tug of the real trial) so the collar must hold its place, then retracts and parks. Measured end state: gap 1.3 mm, radial 6.4 mm, jaws +3.9 / -3.4 mm on the sleeve with 8.62 N in the wire, both grippers clear.

## Real-trial build (stock parts)
Loadable dumbbell with 49 mm plain round chrome sleeves and a 28 mm knurled centre handle, two rubber-coated fractional plates per sleeve, a stock 50 mm (2-inch) three-turn spring clip collar with crossed wire stirrup jaws and vinyl-dipped shoes, a timber bench cradle with UHMW saddle tape, and a felt-lined timber collar holster (scrap timber; no custom machining). The same bench test uses a caliper for the plate gap and a spring scale for the retention pull; no fixture in this task is custom to the simulation.

## Runtime per trial
Runtime per trial: 900 control steps at 20 Hz (45 s simulated, about 4 min wall clock on one A10 GPU); the scripted demo finishes once the 3 s continuous hold completes (593 steps / 29.7 s in the offline pre-check at 20 Hz).

## Seeds
Seeds: 0 (`object_jitter=0`, fixed starting layout).

## Reset between trials
Set the dumbbell level in the cradle and stand the spring collar upright in the timber holster 150 mm off the sleeve axis, jaws at rest (`COLLAR_HALF_X` = 16.5 mm each side of the seat, seat centre 26.5 mm inboard of the plate face).

## Mechanism and sources
| part | real item | model |
|---|---|---|
| cradle stand | steel benchtop dumbbell V-cradle, polymer channel liners, rubber feet (~$5 shop fixture) | fixed compound stand (`friction=0.10`), two saddles 120 mm apart on the 28 mm handle |
| loadable dumbbell | 49 mm (2-inch) plain round chrome loading sleeves + 118 mm and 134 mm fractional rubber plates + 28 mm knurled centre handle, 1.20 kg (no verified listing: a 2-inch loadable dumbbell handle of this class is sold by several vendors; the part is stock, the exact SKU is a gap) | rigid primitive cylinders, two mirrored sleeves about the centre handle |
| spring clip collar | 50 mm (2-inch) three-turn chrome spring barbell collar with red vinyl handles, 320 g (verified listing: https://www.amazon.com/dp/B07B6FMH2V, ProsourceFit Olympic Barbell Spring Clip Collar 2-inch Pair; also CAP B000BJKWXK) | 3-turn 8-gon coil + two crossed spring-wire `slide` jaws at 2200 N/m, 4 mm preload, 12.5 mm stroke |
| timber holster | felt-lined timber slot on the bench | fixed 5-box timber part, 38 mm walls |

## Licence
All shapes are procedural primitives authored in `barbell_spring_collar_slide_and_seat.py`; no external third-party mesh, texture, or font files are included (original own work).
