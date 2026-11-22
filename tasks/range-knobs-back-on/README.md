# Put a gas cooktop's knobs back on: find each D stem by feel with light pressure, push home at OFF, never turn the gas on

A section of a gas cooktop stands on the table: its black enamelled top panel with two burners at the back and, along
the front, two valve stems sticking 18 mm up through the panel, 100 mm apart, both at OFF. The two knobs that were
pulled off them for cleaning stand upright on the panel beside them, turned away from OFF. The instruction states only
the goal: *Put the two burner knobs back on their valve stems, each pushed fully home with its pointer at OFF, and
leave every burner off.*

Most US homes cook on a range (89%), and about 38% cook with natural gas (EIA, Residential Energy Consumption Survey
2020). Taking the knobs off to wipe round them and putting them back is a routine part of cleaning one. What makes it a
physical skill rather than a placing task:

- **The flat is hidden.** A valve stem is a 1/4 in "D" shaft and the knob's bore has a matching flat; the knob goes on
  in one orientation only, and the knob itself hides both flats. People rest the knob on the stem and turn it until it
  drops on (it does within about 5 degrees of lined up).
- **The valve is push-to-turn.** Gas cooktop and range valves are locked at OFF until the stem is pushed in (here a lug
  on the stem sits in a notch of the valve body, and clears it 3.5 mm in, at about 10 N). Turned with light pressure,
  the knob slips on the stem's tip, and once it drops onto the flat the lock stops it, at OFF. Turned while pressed
  hard, the stem goes in and the knob carries it round: the gas is on, and the trial scores 0.
- **Then push it home straight.** The knob's spring clip needs about 5 N to slide it home, so the second phase is a
  firm, straight push with no twist.

So the policy has to search by feel under a force ceiling, and then change to a firm push. This differs from the
accepted "Refit a gas burner after cleaning: turn the head until its slot drops round the igniter" and "Reinstall a
microwave turntable" tasks: those are also "turn until it drops in", but pressing harder is harmless there; here the
valve's own mechanism punishes it. LIBERO's "turn on the stove" and RoboCasa's stove-knob tasks turn a knob that is
already mounted; nothing in those sets, or on the board, mounts a knob on its shaft.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: the cooktop, valve stems, knobs and clips, their joints, `success()` and `score()` |
| `knobkit.py` | Rigid bodies made of several shapes (boxes, cylinders and convex lofts) and always-on D6 joints (adapted from the bodykit module used by other tasks in this repository) |
| `demo.py` | The scripted demo (with a calibration mode) |
| `test_scoring.py` | Offline checks (no GPU): every scoring outcome and attempts to score without doing the task, the runtime's ending, the valve lock and the D bore's geometry from the shapes the simulator gets, the layout, the demo's reach, and that `task.json` states the numbers in the code |

`knobkit.py` hooks `yam_arena.scene` and the embodiment's reset and step without modifying yam_arena, so the task runs
with the standard `yam_arena` embodiment and the `task_score` scorer.

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough, from this folder:

```bash
INSTRUCTION=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 1600 --store-frames --log-dir ~/yam-eval/knobs-dream
```

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode. Set
`KNOB_DEBUG=1` to print, every 10 steps, each stem's turn and push depth and each knob's offset, tilt, gap and pointer
against each stem.

## Scripted demo

```bash
python demo.py task.py demo_out --max-steps 1600 --cam 640x352   # demo_*_cam.mp4, setup.png, result.json
DEMO_CALIBRATE=1 python demo.py task.py cal_out --no-record      # prints the CAL numbers (see below)
python -m pytest -p no:cacheprovider test_scoring.py             # offline checks
```

1. Both arms take their knob by the grip fin from above, fingers closing across the fin, and lift it.
2. The left arm carries its knob over the left stem, turned 30 degrees clockwise of OFF, and sets it on the stem's tip
   (the bore's round mouth, 3.5 mm deep, centres it there, its moulded flat resting on the tip), commanded 8 mm
   below where it stops, so it presses lightly (the stem does not move: under the 6 N that starts to move it).
3. It turns the knob slowly counter-clockwise to 2 degrees past OFF. The knob slips on the tip until its flat lines
   up, drops 4.5 mm onto the stem's flat, and is then stopped by the stem's lock at OFF.
4. It lets go, closes its fingers, and pushes the knob straight down with the fingertips on top of the fin until the
   stem's tip bottoms in the bore (skirt 4 mm off the panel); then draws back to its ready pose.
5. The right arm does the same with its knob on the right stem, from 30 degrees counter-clockwise of OFF, turning
   clockwise (toward OFF's stop).

The demo runs open loop: it reads no simulator state. Where each knob sits in the hand after it is picked, and how far
below the stopping point the light press and the push home are commanded (the arms are compliant, so a press pushes only
as hard as it is commanded past where the part stops), are measured once with `DEMO_CALIBRATE=1` and fixed in `CAL` in
`demo.py`; that run also prints each knob's height and its stem's push depth after the turn and after the push, which
must show the stem never pushed in during the turn. (The knob's offset in the hand that calibration reads, about 4 mm, is mostly the arm's own lag behind its
commanded pose, which the move over the stem repeats, so the plan keeps the nominal grasp; see the comment in `CAL`.) `result.json` reports `score()` on the last step as the trial's
score, and the largest turn any stem made.

## Scoring

Evaluated every control step; the score at the end of the episode counts (`SCORE_OVER = "end"`).

- **0 for the whole trial** once either stem has turned more than 10 degrees from OFF at any time (the gas is on),
  whatever happens after.
- **1.0** once both knobs are seated at OFF (below), no part of either gripper is within 2 cm of either knob (in
  simulation: every vertex of both grippers' finger and body collision hulls), and nothing (knobs, stems) has moved more
  than 1 mm over the preceding 1 s, all held together for 1 s (20 control steps in a row). `score()` gives 1.0 from the
  step this has held for 1 s, and `success()` returns true only on steps where `score()` is 1.0; the runtime ends the
  episode once `success()` has stayed true for its own 1 s, so an episode can only end with success at 1.0.
- Otherwise, at the end, **per stem**:
  - **0.45** for a knob seated on it: its bore's axis within 1.5 mm of the stem's at the mouth, tilted no more than 3
    degrees, its skirt within 5 mm of the panel (4 mm when home), its pointer within 5 degrees of OFF;
  - **0.2** for a knob on its flat but not home: within 1.5 mm, tilted no more than 5 degrees, skirt within 12 mm
    of the panel (10 mm when it has just dropped onto the flat and rests on the clip), pointer within 10 degrees;
  - else 0. A knob resting unaligned on the tip stands 14.5 mm up and earns nothing.
- One knob can count for one stem only (the stems are 100 mm apart).

In a real trial: the witness lines across each stem's collar and valve body are watched on video through the valve
box's clear front (ticks at 10 degrees either side; a microswitch on the valve's igniter cam with an LED works too);
the skirt's gap with a feeler gauge or a ruler; the pointer on a top photo against the printed OFF line; the 2 cm
clearance by eye.

## How it is modelled

Everything is a rigid body at real size; the effect that matters comes from contact.

- **The stem** (`stem_i`, 30 g with the plug it turns) is a D-section brass shaft (a convex hull: a 6.35 mm circle with
  a flat 1.625 mm off the axis) with a lug (2.0 x 5.0 x 2.5 mm) pointing to the front under the panel. A PhysX D6 joint
  to the cooktop (`valve_i`) holds it on its axis: it slides 0 to 4.5 mm in, pushed out against its stop by a
  spring (6 N preload, 1200 N/m), and turns between the OFF stop (2 degrees clockwise of OFF) and LO (300 degrees
  counter-clockwise), with a drag of 0.05 N m at one turn a second for the plug's grease. **Nothing in the joint locks
  it at OFF.** The lock is a real contact: the valve body's lip (a brass ring 4-9 mm from the axis, 2-6 mm under the
  panel) has a 2.4 mm notch at OFF that the lug sits in; the joint lets the stem and the cooktop collide, so the stem
  can turn only once it is pushed 3.5 mm in (about 10 N), with the lug below the lip. Released off OFF, the spring holds
  the lug against the lip's underside, as in the real valve.
- **The knob** (`knob_i`, 22 g) is a skirt, top and grip fin round a boss whose bore is twelve convex segments
  (a round bore 0.275 mm clear of the stem, with a 1 mm lead-in chamfer at the mouth), with a moulded flat from 3.5
  to 7.5 mm up the bore, 0.25 mm clear of the stem's flat. Unaligned, the flat rests on the stem's tip (the 3.5 mm
  round mouth keeps it there); within about 5 degrees of aligned it passes the stem's flat and the knob drops 4.5 mm.
- **The clip** (`clip_i`, 1.5 g) is a steel leaf along the bore's flat from 8 to 13.5 mm, on a slide in the knob
  (`clip_i` joint, 0 to 0.6 mm outward) pressed toward the axis with 7 N; it stands 0.45 mm proud of the stem's flat, so
  the stem's tip pushes it out up its 1 mm ramp, and it then grips the flat: sliding the knob home takes about 5 N
  (friction 0.3 against the brass), and it holds the knob there.

**Simplifications.** The plug's grease friction is viscous (proportional to speed), not dry. The valve's inner parts
other than the stem, lug, lip and stops are not modelled (no gas path). The knob's clip is one rigid leaf on a spring,
not a bent spring. The spring, clip and drag forces, and the stem's protrusion, are estimates (below).

## Numbers and where they come from

| Quantity | Value | Source |
|---|---|---|
| Households cooking with natural gas | 38% (89% have a range) | EIA, Residential Energy Consumption Survey 2020 |
| Push-to-turn valve with a lock at OFF | lug in a notch, unlocked by pushing the stem in | Range service manuals ("before the stem can be turned, it must be pushed in"); US patent 11,473,779 describes the lock tang and detent |
| Stem | 1/4 in (6.35 mm) D shaft | Replacement range knobs and valves are sold for 1/4 in D stems |
| Across the flat | 4.8 mm | **Estimate**: measure with calipers |
| Stem above the panel | 18 mm | **Estimate**: set by how the valves are mounted in the rig |
| Push to unlock / travel | 3.5 mm / 4.5 mm | **Estimates** |
| Stem spring | 6 N at rest, 10.2 N at 3.5 mm | **Estimates** |
| Plug drag | 0.05 N m at one turn a second | **Estimate** |
| Knob | 44 mm across, 33 mm tall, 22 g, 14 mm deep bore | Typical gas-cooktop knob; **measure the knobs used** |
| Clip | 7 N, about 5 N to push the knob home | **Estimate** |

### Measuring the valve and the knob

- **Stem:** calipers across the round (6.35 mm) and across the flat; a ruler from the panel to the tip.
- **Unlock push and travel:** stand the valve stem-up on a digital kitchen scale and zero it. Press the stem down
  slowly with a flat-ended rod; the reading as it starts to move is the preload; keep pressing while gently turning the
  rod and note the reading (and the stem's travel, on a dial gauge or calipers against the valve body) when it first
  turns: that is the unlock force and depth (force in N = reading in kg x 9.81). Do it ten times and use the median for
  `SPRING_PRELOAD`, `SPRING_RATE` and `UNLOCK` in `task.py`.
- **Drag:** with the stem pushed in, turn it with a luggage scale on a 50 mm lever (or a torque screwdriver bit on the
  flat) and note the steady reading.
- **Knob:** weigh it; push it onto a stem on the scale (stem held in by a spacer so it cannot move) and note the force
  that slides it home (`CLIP_PRELOAD` is set so that force comes out right).

## Real-world set-up

All parts are bought: two push-to-turn range or cooktop surface-burner valves with 1/4 in D stems (for example
Whirlpool WPW10206288), **not connected to any gas supply**; two matching burner knobs (for example WP98006102; check
the knob fits the valve's stem before buying more); a 40 mm plywood box with a clear acrylic front and a 1 mm steel
sheet top (painted black) with two 12 mm holes 100 mm apart, the valves screwed under it so the stems stand 18 mm
proud; two sealed burner heads for the back (optional, they are not touched); white and red paint pen for the OFF and
HI lines and the witness marks. With no gas line, the trial is safe; the "gas on" outcome is read from the witness
marks.

## Runtime and seeds

The layout is fixed (`-E object_jitter=0`); there are no seeds to choose. A trial runs at most 1,600 steps (80 s).
The demo's script is 796 control steps (40 s); it reaches full score at step 756, while the right arm is drawing
back, and the runtime ends the episode by success at step 775.

## Results

| Run | Score | Notes |
|---|---|---|
| Scripted demo (`demo.mp4`, top camera) | 1.0 | Full score from step 756, success at 775; largest stem turn 1.7 degrees (the notch's play); stems never pushed in while the knobs were turned |
| Same script, turning while pressing hard (`DEMO_PRESS_MM=40`) | 0.0 | The left stem was pushed 4.6 mm in and carried 44 degrees off OFF from step 225: the gas was on |
| Dream-1 (`dream.mp4`) | 0.0 | 1,600 steps; the arms barely moved and the knobs were not touched |

A probe (external forces on teleported knobs) gave the same picture: a knob set unaligned on a tip rests 14.5 mm
up; pressed with 15 N and twisted with 0.03 N m, it pushed the stem fully in and turned it 121 degrees.
