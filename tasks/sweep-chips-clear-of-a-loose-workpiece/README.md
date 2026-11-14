# Sweep metal chips into a scrap trough with a bar, around a loose workpiece that must not move

Instruction given to the policy (the `instruction` field of `task.json`):
`Leave the loose blue workpiece exactly where it is, and keep every chip on the surface: using the yellow bar as a scraper, slide all eight metal chips across the plate and over its far edge into the scrap trough. Never lift a chip or pick one up.`

Files: `task.json`, `sweep_chips.py` (scene, layouts, scoring, the sweep planner and the scripted demo),
`check_demo.py` (offline checks), `README.md`.

## Environment

The program's Brev launchable (Isaac Lab-Arena, Isaac Sim, yam-arena), with the launchable's `yam.env` sourced
in every shell (`source <ARENA_DIR>/yam.env`). Versions used:

- inspect-robots: 1.0.0
- yam-arena commit: 0059d02a1a91
- task file SHA-256, first 12 hex: see `eval.environment_revision` in the logs (it changes with every edit to
  `sweep_chips.py`)

## Scene

Coordinates in metres: x forward from the line through the arm bases, y left from the midline, z up from the
table top. Everything is in front of the left arm (base at y = +0.305).

- **Plate** (fixed, steel): 280 x 240 x 15 mm, x 0.12 to 0.40, y 0.10 to 0.34.
- **Scrap trough** (fixed): 40 mm wide beyond the plate's far edge (x 0.40 to 0.44), its floor the table top,
  15 mm below the plate top. A red lip (10 mm thick, 30 mm tall above the table) closes the far side and two red
  end walls (6 mm thick, 30 mm tall) close the ends at y 0.10 and 0.34.
- **Bar** (loose, yellow aluminium): 20 x 75 x 20 mm, 81 g, lying along y at x 0.15, y 0.22.
- **Workpiece** (loose, blue acetal): 80 x 60 x 40 mm, 270 g, standing at x 0.30 and y 0.15, 0.22 or 0.29.
- **Chips** (loose, aluminium): eight 15 mm cubes, 9 g each, on a 25 mm grid of cells at x 0.205 to 0.380 and
  y 0.125 to 0.325, each at least 10 mm from the workpiece.

## Layouts

The layout number comes from the environment variable `SWEEP_LAYOUT_SEED` (a whole number, default 0), read when
the task file loads. It is not an `-E` option: the `yam_arena` embodiment accepts only its own `-E` options, and it
does not pass the run's `--seed` to task files. The layout number is printed at the end of the task description
the embodiment reports (`Layout number N.`).

Rule for any layout number N (`layout()` in `sweep_chips.py`):

1. Workpiece y = (0.15, 0.22, 0.29)[int(SHA-256 hex of `sweep-wp:N`, base 16) mod 3].
2. The free cells are the grid cells (x = 0.205 + 0.025 i, y = 0.125 + 0.025 j, i = 0..7, j = 0..8) whose chip
   footprint is at least 10 mm from the workpiece's footprint.
3. For attempt = 0, 1, 2, ...: order the free cells by the SHA-256 hex of `sweep-chip:N:<attempt>:<cell>` and
   take the first eight. Keep the first draw with at least two chips within 25 mm of a workpiece face and at least
   one in its shadow (in front of it, between it and the trough, with |y - workpiece y| < 30 mm). Chips 1 to 8 take
   the eight cells in that order.

`<cell>` is the cell's coordinates as Python prints the tuple of the two floats computed as above, for example
`(0.205, 0.125)` or `(0.22999999999999998, 0.175)`. Some values carry float rounding digits, and the hash uses
those exact strings.

Positions in mm (x, y):

| Layout | Workpiece y | chip 1 | chip 2 | chip 3 | chip 4 | chip 5 | chip 6 | chip 7 | chip 8 |
|---|---|---|---|---|---|---|---|---|---|
| 0 | 220 | 255, 275 | 380, 325 | 280, 275 | 305, 325 | 305, 125 | 305, 150 | 205, 125 | 380, 225 |
| 1 | 220 | 280, 275 | 205, 125 | 230, 250 | 380, 225 | 380, 250 | 305, 125 | 255, 325 | 305, 325 |
| 2 | 150 | 380, 250 | 205, 225 | 380, 325 | 380, 175 | 280, 200 | 205, 275 | 305, 200 | 305, 275 |
| 3 | 290 | 380, 275 | 280, 200 | 255, 175 | 230, 300 | 205, 150 | 230, 225 | 230, 275 | 380, 300 |
| 4 | 220 | 255, 275 | 305, 150 | 380, 225 | 355, 325 | 305, 325 | 205, 250 | 230, 175 | 355, 300 |
| 5 | 150 | 280, 300 | 205, 150 | 380, 175 | 255, 325 | 355, 200 | 330, 325 | 380, 300 | 255, 200 |
| 6 | 290 | 255, 125 | 230, 325 | 380, 250 | 230, 225 | 230, 275 | 380, 175 | 380, 275 | 330, 125 |
| 7 | 290 | 230, 275 | 230, 325 | 280, 200 | 355, 200 | 380, 275 | 230, 150 | 255, 125 | 205, 275 |
| 8 | 220 | 230, 250 | 330, 325 | 330, 300 | 380, 175 | 280, 150 | 380, 225 | 230, 275 | 255, 325 |
| 9 | 290 | 380, 275 | 230, 325 | 230, 125 | 280, 225 | 205, 175 | 380, 300 | 230, 275 | 355, 200 |

The screenshot and the submitted runs use layout 2. **Only layout 2 has been run in the simulator.** Every other
layout has only been checked offline (`check_demo.py`, with a kinematic model and no physics). For real-world
trials, draw a layout number per trial (a ten-sided die for 0 to 9) and set out the workpiece and chips on the grid
drawn on the plate.

## Scoring

A chip is credited if it ends in the trough and was never lifted and never moved except by the bar.

- **In the trough:** its whole footprint inside the trough (x 0.40 to 0.44, y 0.10 to 0.34), still (under 1 cm/s
  and 0.5 rad/s), and either its bottom at least 3 mm below the plate top (it has fallen in: on the floor, or
  propped or leaning at an angle) or resting on another chip that is itself in the trough (within 3 mm). The
  footprint is the chip's world-aligned bounding box with no tolerance, so a tilted chip must clear x 0.40
  entirely. A chip resting on a chip that rests on the floor counts. A chip held above another trough chip,
  hanging on or straddling the plate edge, still moving, or anywhere else counts for nothing.
- **Never lifted:** at no moment is its bottom more than 4 mm above the plate top, unless it rests (within 3 mm) on
  a chip that is itself down, as when a chip rides on another during a sweep, or it is over the trough, its lip
  or end walls. The lip and end walls stand 15 mm above the plate top, so the only way into the trough without
  rising that high is over the plate edge.
- **Moved only by the bar:** at no moment does it move (1 cm/s or 0.5 rad/s or more) over the plate while more
  than 10 mm from the bar, unless it touches (within 2 mm) a moving chip that is itself bar-driven. The chain
  keeps a legitimate sweep's credit: in the probe run, chips pushed along by other chips moved up to 15 mm from
  the bar, and every one passed the rule with the chain. Pushing chips with the fingertips, the workpiece or
  anything else fails it.
- **Workpiece factor:** 1.0 if at the end its centre is within 5 mm of its start and it is turned no more than
  3 degrees (folded modulo 180 degrees); 0.5 if within 15 mm; 0 if further. It is also 0 if at any moment it
  moved more than 15 mm, was lifted more than 4 mm off the plate, tipped more than 5 degrees or left the plate.
  The "at any moment" rules stop a policy from moving the workpiece aside, sweeping through its spot and putting
  it back.
- **Score** = credited chips / 8 x workpiece factor. **Success:** all eight credited and factor 1.0. `task_score`
  is 1.0 on success (the episode ends once it has held 1 s), otherwise the score at the end of the episode.

**State between trials.** A task file gets no reset call, so the record of lifts, unbarred moves and workpiece
moves clears when `score()` sees a trial's start: every chip and the workpiece lying still within 3 mm of their
start places. `success()` and `score()` share one update per control step.

**Real-world trials.** Before the trial, draw the workpiece's outline on the plate, plus a line 15 mm outside it.
A camera records the plate from above for the whole trial. The judge reviews the video:

- For each chip: was it ever picked up, pinched or held off the plate? Did it ever move without the bar, or a
  chip the bar was pushing, against it?
- For the workpiece: was it ever lifted or tipped, or did any part of it cross the 15 mm line?

After the arm stops, measure the workpiece's offset from its outline with a ruler or caliper. Shift is the move of
its centre (the mean of the offsets of opposite sides). Turn is the difference between the offsets at the two ends
of an 80 mm side: 3 degrees is 4.2 mm. Count the chips in the trough from a top photo.

## Offline check, no GPU

From the root of this repository:

```bash
python tasks/sweep-chips-clear-of-a-loose-workpiece/check_demo.py     # about 45 s
```

- **Layouts 0 to 9:** eight distinct grid cells on the plate, each chip at least 10 mm from the workpiece and clear
  of the bar's start, at least two near the workpiece and one in its shadow, and a sweep plan exists. Layouts 0 to
  199 all have a plan (smallest margin 3.75 mm).
- **Bar length:** the plan's smallest margin for 70, 75, 80 and 100 mm bars on layouts 0 to 9. The 100 mm bar
  cannot fit beside the middle spot (y 0.22: 90 mm between the workpiece and the plate edge); four layouts have no
  plan. 80 mm fits with no slack and leaves three layouts without a plan; 70 mm drops to 2.5 mm. 75 mm (three
  25 mm chip pitches, so lane ends can fall mid-gap between chip rows) gives 5 mm on eight layouts and 3.75 mm on
  layouts 4 and 8.
- **Scoring, on hand-built sequences of states:**
  - Nothing done: 0.
  - Every chip picked up by hand and set in the trough: 0. A chip raised 3 mm keeps its credit; 5 mm is a lift.
  - Every chip pushed into the trough with the closed fingers, the bar left at its start: 0.
  - A chain (bar, chip, chip) keeps its credit; a chip moving 15 mm from the bar, touching nothing driven, does
    not.
  - A chip riding on another chip on the plate is not lifted; one held 5 mm above another is.
  - The record clears at the next trial's start.
  - The demo's sweep with the workpiece untouched: 1.0. With the workpiece then moved 4 mm: 1.0; 10 mm: 0.5;
    20 mm: 0; turned 5 degrees in place: 0.5; pushed 20 mm aside and put back: 0; lifted 10 mm and put back: 0;
    tipped onto its side: 0; tipped 8 degrees and back: 0; in the trough: 0.
  - A chip stacked in the trough on another chip: credited (1.0). Not credited: held 5 mm above a trough chip,
    straddling the plate edge, hanging on the plate edge at 22 degrees (centre x 0.405), still moving in the
    trough, on the table past an end wall. A chip propped 5 mm above the trough floor: credited.
- **The demo, replayed offline for layouts 0 to 9:** through `yam_arena.demo.ArmPlan` at 20 Hz. The chip model is
  kinematic: a chip touched by the bar's moving face is pushed ahead of it, chips push chips, and a chip past the
  plate edge drops into the trough. `score()` and `success()` run every step, as in the simulator. In every layout:
  final score 1.0, with success held 1 s by step 770 to 1,027; the bar never lowered onto a chip and no chip
  pushed off the plate. Worst IK error 0.097 mm / 0.013 deg, every joint at least 13 degrees inside its limits.
  Smallest gaps: bar to workpiece 3.75 mm (the plan margin), fingers to workpiece 7.2 mm, bar to the trough end
  walls 2.5 mm, fingers to the walls 6.4 mm, fingertips over the chip tops 2.2 mm. The finger envelope comes from
  the URDF's collision meshes. This checks the plan's geometry and the scoring, not that the sweep works in
  physics.

## Run and score one Dream-1 trial

From the root of this repository. The run goes in the background with `nohup`; run one simulator at a time.
The instruction is read from `task.json`.

```bash
mkdir -p logs
INSTR=$(python -c 'import json,sys; print(json.load(open(sys.argv[1]))["instruction"])' \
  tasks/sweep-chips-clear-of-a-loose-workpiece/task.json)
SWEEP_LAYOUT_SEED=2 nohup \
  inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/sweep-chips-clear-of-a-loose-workpiece/sweep_chips.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score --max-steps 3000 --seed 0 --no-prompt --store-frames \
  --log-dir logs/sweep-dream > logs/sweep-dream.out 2>&1 &
```

`-E object_jitter=0` is required: the chips are 10 mm apart and the lane margins are 3.75 to 5 mm, so the default
3 cm jitter breaks the layout. `-E control_hz=20` matters: the run limit is 3,000 steps at 20 Hz. Scoring is
automatic (see Scoring). The CLI prints `task_score: <value>` (in `logs/sweep-dream.out`); the log keeps it at
`results.metrics.task_score`.

## Run and score the full-score demo

```bash
mkdir -p logs
INSTR=$(python -c 'import json,sys; print(json.load(open(sys.argv[1]))["instruction"])' \
  tasks/sweep-chips-clear-of-a-loose-workpiece/task.json)
SWEEP_LAYOUT_SEED=2 nohup \
  inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/sweep-chips-clear-of-a-loose-workpiece/sweep_chips.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score --max-steps 3000 --seed 0 --no-prompt --store-frames \
  --log-dir logs/sweep-demo > logs/sweep-demo.out 2>&1 &
```

The demo is `demo()`, a scripted (not learned) policy run through `yam_scripted`. It uses the left arm only. It
is **open loop**: the plan is computed from the layout's start positions when the episode starts, and the demo
never reads the simulator's poses, so it does not correct for a chip that goes astray. The bar is held by its
middle from above, the fingers closing across its 20 mm width, with its length along y the whole time. The pad
tips sit 14 mm up the bar, 2 mm above the chip tops. Three kinds of pass:

1. **Side lanes:** the bar is lowered behind every chip (x 0.15), 3 mm above the plate, and swept along +x at a
   mean 5 cm/s until its front face is 5 mm past the plate edge, then lifted to 60 mm (20 mm over the workpiece)
   to travel. Every lane misses the workpiece in y by at least the plan's margin, and every chip is either fully
   inside a lane, with each lane end at least that margin beyond it, or at least that margin clear of it. The
   planner picks the largest margin, from 5 mm down to 2.5 mm, that covers every chip it can.
2. **End pushes:** a chip no lane can reach (behind the workpiece or in its shadow, inside its y band) is pushed
   along y with the bar's end face, one x row at a time, out of the band to twice the plan margin beyond the
   workpiece's side (10 mm at a 5 mm margin). The
   bar is centred on the row and lands 4 mm short of the row's nearest chip, in ground the side lanes have
   already cleared. The bar can't do this as a lane in front of the workpiece: the gap between the workpiece and
   a shadow chip is 32.5 mm, and the bar plus the two fingers is 32.8 mm.
3. **Re-sweep:** one more side lane over the pushed chips.

Layout 2: lane 1 at y 0.225 (chips 1, 2, 5, 7), lane 2 at y 0.300 (chips 3, 6, 8), an end push of chip 4 (the
shadow chip at x 0.38, y 0.175) from y 0.126 to 0.1525 (bar centre), and a re-sweep at y 0.2225. The plan is
896 steps (44.8 s). The episode should end about 770 steps in, once all eight chips have been credited for 1 s.

## Run limit, runtime and seeds

- Run limit: 3,000 control steps at 20 Hz (2.5 minutes). The demo plans take 896 to 1,153 steps on layouts 0
  to 9.
- Runtime per trial: the probe's 484-step scripted run took 74 s of wall time, including simulator start-up, on
  the launchable's GPU. Expect about 2 minutes for the demo and a few minutes for a full 3,000-step Dream-1
  trial, which also runs the policy server. To be replaced with measured times from the submitted runs.
- Seeds: `SWEEP_LAYOUT_SEED=2` (the layout), `--seed 0`, `-E object_jitter=0`. The demo is deterministic;
  Dream-1 is not seeded.

## Submitted runs

Both runs are done, on layout 2:

- Scripted demo: `task_score` 1.0 (layout 2).
- Dream-1: `task_score` 0.0 (layout 2, 3,000 steps).
- `task.json` `scores` records both, and `media` holds the uploaded videos and setup screenshot.

## Limitations

- **Simulator coverage:** an earlier one-lane probe ran on layout 2 with a 100 mm bar. In it, the bar held 17 mm
  below the TCP throughout, so it did not slip in the grip; it sagged up to 2.6 mm below plan while sweeping (arm
  sag, not slip); and the workpiece did not move. The probe swept 6 chips and put 5 in the trough. The sixth came
  to rest on top of another chip, which the old rule didn't count; the stacking rule above now counts it. The
  full demo (75 mm bar, two lanes, the end push and the re-sweep) scored 1.0 in the simulator on layout 2 (all 8
  chips credited, workpiece factor 1.0, 829 steps). The end push and the 3.75 mm margins have only been checked
  offline. The demo was only run in the simulator on layout 2; layouts 0 to 9 were checked offline only.
- **Held chips:** in the simulator, a chip held still in the air over the trough more than 3 mm below the plate top
  would count as in the trough; the written rule says the judge catches pinched or held chips on the video.
- **Tight spots:**
  - The planned fingertip clearance over the chip tops is 2 mm. The probe's 2.6 mm sag could bring the front
    finger onto a chip's top edge as the bar pushes it; in the probe, no chip rose more than 0.3 mm.
  - On some lanes the bar's end is 2.5 mm from a trough end wall during the last 5 mm of a sweep.
  - In an end push the bar's 20 mm face pushes a 15 mm chip, with 2.5 mm to spare on each side.
- **One arm:** the right arm stays at its ready pose. The task doesn't need two arms.
- **Prior work:** sweeping tasks exist, for example in Meta-World (sweep, sweep-into) and RLBench (sweep to
  dustpan). What differs here:
  - The loose workpiece among the chips, which must not be moved, so the tool has to work around it, with chips
    beside, behind and in front of it.
  - The swept-not-lifted check: every chip must reach the trough without leaving the surface and only by the
    tool.
- **Fidelity:** see `task.json`. Chips are cubes, not swarf curls, and the friction is one rubber-like value for
  everything. In a real trial, the 4 mm lift threshold and the 10 mm bar rule are judged from video, not
  measured.

## Reset between trials

Nothing is consumed. Take the chips out of the trough and set out the next layout from the table (workpiece spot
and eight chip cells on the drawn grid). Put the workpiece on its outline and the bar back at x 0.15, y 0.22,
along y. Start a new overhead video.

## Development log (not needed to run or score)

With `SWEEP_POSE_LOG=<dir>` set, every control step appends one JSON line to `<dir>/sweep_poses.jsonl`: the
plan's TCP target and label, the bar's pose and offset from the TCP, the workpiece's shift, turn and tilt, and per
chip its pose, speed, distance to the bar and scoring flags. Unset (the default), nothing is written.
