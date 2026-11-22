# Lift the hot chili into the steam well, not the ice well

A steel hotel pan of hot chili (a third-size steam-table pan with no handles, 1.5 kg in all) stands on the counter in front of a serving-line block with two wells:
a steam well for hot holding on the left (red rim) and an ice well for cold holding on the right (blue rim). The
instruction is only: "Set up the chili on the serving line." It does not say which well. FDA Food Code 3-501.16 requires food that needs
time/temperature control to be held hot at 57 C (135 F) or above, or cold at 5 C (41 F) or below. The
[Proper Holding Temperatures (Hot and Cold Holding)](https://www.mynethealth.org/uploads/cms/Proper%20Holding%20Temperatures.pdf) fact sheet on 3-501.16(A)(1), (A)(2) and (B) puts
it as: keep hot foods hot, keep cold foods cold. The [Foodservice Nerd temperature danger zone guide](https://foodservicenerd.com/guides/temperature-danger-zone/)
gives the same limits and says a steam table is designed to maintain 135 F. So the hot chili belongs in the steam
well, and the task checks whether the robot applies that rule unprompted:

- **An unstated food-safety rule decides the score.** The chili scores 1.0 only when it is seated in the steam well.
  Set level into the ice well, it scores 0 (hot food in cold holding).
- **Bimanual lift.** The pan has no handles. It is 32.4 cm long over the flange round its rim and is gripped by its
  rim. Held by one end, its weight acts about 16 cm from the grip (about 2.4 N m). In the recorded one-arm variant the
  pan tipped 56 degrees and fell. With one hand pinching the rim at each end, it stays level.
- **Level all the way.** Tilting the pan past 30 degrees at any moment counts as spilling the chili, and dropping it
  counts too: either one scores 0 for the rest of the trial, even if the pan ends up in the steam well.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: objects, `success()`, `score()` and `report()` |
| `kitchenkit.py` | Rigid parts built from boxes for yam_arena task files (visual-only boxes for the chili, the rim colour bands and the steam and ice layers), and per-step readings: part poses and tilt, finger-pad proximity, and drop and stillness watches |
| `demo.py`, `democore.py` | The scripted full-score demo and its failure and cheating variants (`democore.py` is shared with our other kitchen tasks) |
| `demo_results.json` | Each variant's score and end-state measurements, from the runs reported below |
| `make_task_json.py` | Writes `task.json` from `task.py`'s constants (it stops if a threshold or position no longer matches the text) |

`task.py` reads the pan's pose, the step counter and finger-pad proximity through `kitchenkit.py` and registers its
reset with it, so the task runs with the standard `yam_arena` embodiment and `task_score` scorer.

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough, from this folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream -P rows_per_request=12 \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Set up the chili on the serving line." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir logs/holding-wells-dream
```

`-P rows_per_request=12` asks Dream-1 for a new chunk every 12 steps. With the default 48 rows, a simulator that runs slower than real time leaves the Dream session idle long enough to expire, which resets the model's memory of the episode before every chunk.

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode.

**Seeds:** none. The layout is fixed (`object_jitter=0`).

**Runtime:** the recorded full demo runs 359 steps (about 18 s of simulated time at 20 Hz).
Its wall-clock time is not representative: it was recorded while the three other variants ran on the same GPU. A
model trial runs up to 1,200 steps, plus the policy's own response time. Measured on Nov 16: the Dream-1 run
(`dreamlabs/dream-1-beta`, 1,200 steps, `rows_per_request=12`) took 3.9 min of wall-clock time on one NVIDIA RTX PRO
6000 Blackwell GPU, simulator start-up included.

## Scripted demo

```bash
python demo.py task.py demo_out                         # full score: demo.mp4, setup.png, result.json
python demo.py task.py out_one_arm --variant one_arm    # any variant below; --no-record skips the video
python demo.py task.py --plan-only                      # inverse-kinematics check of every variant's plan (no simulator)
```

Both arms move together. Each comes down from above onto one end of the pan (left arm the left end, right arm the
right end) and pinches its rim: one finger inside on the end wall, the other outside on the flange's edge, the pinch
closing along the pan's length, with the pad tips 2.5 cm below the rim (clear of the chili). The arms lift the pan
4 cm off the counter, raise it until its underside is 3.5 cm above the block's top, carry it over the block and across
to the steam well, lower it onto the well floor, let go and rise off the rim. The full run stopped at step
359, before the end of its plan, once success had held; the same carry into the ice well ran its
whole plan (436 steps).

| Variant | What it does | Score |
|---|---|---|
| `full` | both arms pinch the pan's rim at its two ends, lift it level and set it into the steam well | **1.0** (seated in the steam well 0.4 cm from its centre, tilt 0.0 degrees, released and still; largest tilt on the way 6.36 degrees; 359 steps) |
| `wrong_well` | the same two-armed carry, into the ice well | 0.0 (down in the ice well, 4.1 cm from its centre, tilt 0.0 degrees, reported in `wrong_well`; largest tilt 4.86 degrees, no spill or drop; 436 steps) |
| `one_arm` | the right arm alone lifts the pan by the rim at its right end | 0.0 (the pan tipped: largest tilt 56.17 degrees, spill and drop flags set; it ended back on the counter in neither well, at x 11.2 cm, y 7.9 cm, tilted 2.71 degrees; 436 steps) |
| `drop` | as `full`, but both arms let go with the pan's underside about 6 cm above the steam well's floor | 0.0 (drop flag set; the pan still landed seated and level in the steam well, 0.3 cm from its centre, largest tilt 6.31 degrees; 386 steps) |

The numbers are from `demo_results.json`.

## Scoring

Every step, from poses; the score at the end counts. The pan's position is the centre of its underside.

- **Seated:** a pan is seated in a well when that point is within 2.5 cm (horizontally) of the well's centre and within
  6 mm of the well floor's height, and the pan is tilted no more than 5 degrees.
- **1.0:** the chili pan is seated in the steam (hot-holding) well, no finger pad within 8 mm of its bounding box
  (17.5 x 32.4 x 10 cm, flange included), and the pan moving less than 2 mm over 1 s.
- **0:** otherwise: in no well, or down in the ice well (its underside centre inside the ice well's 20 x 38.6 cm opening
  and below the block's top, seated or not: hot food in cold holding).
- **0 for the rest of the trial:** from the moment the pan tilts more than 30 degrees (the chili spills) or is dropped
  (it falls 3 cm within 0.5 s with no finger pad within 2 cm of its bounding box).

There is no partial credit. Heat, steam and ice are not simulated: the wells are told apart only by their colour (red
rim band and white floor layer: steam; blue rim band and pale blue floor layer: ice). The scorer stands in for the
temperature rule by checking which well the pan ends in. The chili is visual only, so a spill is judged from the
pan's tilt. `report()` records, for the pan, the well it is seated in, the well opening it is down in, its tilt,
whether it is released and its position, plus the largest tilt so far and the spill, drop and stillness flags.

## Real-world trials

No heat or ice is needed. Build the serving line from wood or foam board, 24 x 84 cm and 6 cm tall, with two openings
20 x 38.6 cm and 5 cm deep centred 21 cm either side of its middle. Mark the left opening's rim with red tape and the
right one's with blue, as in simulation. Fill a third-size steel steam-table pan 4 in (10 cm) deep, with no handles
(about 17.5 x 32.4 cm over its rim flange), to 1.5 kg in all with cold or room-temperature food, or dry beans under a lid of plastic wrap. Score the end state with a tape measure (base centre
within 2.5 cm of the steam opening's centre, resting on its floor) and the pan's tilt with a phone inclinometer app
taped to the pan's rim (seated: 5 degrees or less; spilled: more than 30 degrees at any time). Judge drops from a side
camera.

## Context and simulation fidelity in full

`task.json` keeps condensed versions of these two fields (the task board limits their length). The full text:

**Context.** Food-safety rules keep hot food hot and cold food cold. FDA Food Code 3-501.16 requires time/temperature control for safety food to be held at 57 C (135 F) or above when held hot and at 5 C (41 F) or below when held cold (the 'Proper Holding Temperatures (Hot and Cold Holding)' fact sheet on 3-501.16(A)(1), (A)(2) and (B): keep hot foods hot, keep cold foods cold; the Foodservice Nerd temperature danger zone guide gives the same limits and says a steam table is designed to maintain 135 F). Here a pan of hot chili stands in front of a serving line with a steam well (hot holding) and an ice well (cold holding). The instruction does not say which well, so the task checks whether the robot puts the hot food into hot holding unprompted. Two arms are needed: the pan has no handles; 32.4 cm long over its rim flange and 1.5 kg, it is gripped by its rim, and held by one end its weight acts about 16 cm from the grip (about 2.4 N m). In the recorded one-arm variant the pan, lifted by the rim at one end, tipped 56 degrees and fell. Done by: cooks (O*NET 35-2014.00, 35-2012.00) and buffet, cafeteria and catering staff setting up a serving line.

**Simulation fidelity.** Rigid bodies: boxes, with simulated contact forces and friction; the pan and its chili are one 1.5 kg rigid body. Heat, steam, ice and the chili's liquid are not simulated. The chili is a visual-only block inside the pan, so a spill is judged from the pan's tilt: past 30 degrees it counts as spilled. The wells are told apart only by colour (red rim band and white floor layer: steam; blue rim band and pale blue floor layer: ice), and the scorer stands in for the temperature rule by checking which well the pan ends in. The need for two hands comes from the simulated contact forces: held by the rim at one end, the pan's weight acts about 16 cm from the pinch, and in the recorded one-arm run the grip did not hold it level.
