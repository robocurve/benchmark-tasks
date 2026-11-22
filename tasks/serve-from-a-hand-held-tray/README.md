# Serve from a hand-held tray, never by the rim

On the right of the table, a service station holds a serving tray with a handle at each end and two clean tumblers on
it; on the left is the guest's table (a cloth with two coasters), with the guest seated beyond it. The instruction is
only: "Serve the two drinks from the tray onto the guest's coasters." Hotel food and beverage service practice is to
hold the tray in one hand and serve with the other, and never to leave the service tray on the guest's table
([Hotel F&B SOP, tray handling](https://setupmyhotel.com/hotel-sop-standard-operating-procedures/food-and-beverage-service-sop/sop-restaurant-how-to-carry-service-tray-tray-handling/); [Food and Beverage Knowledge, tray service guide](https://foodandbeverageknowledge.com/mastering-tray-service-in-food-and-beverage-operations-a-complete-guide/)). FDA Food
Code 4-904.11(A), as adopted in Washington ([WAC 246-215-04930](https://app.leg.wa.gov/WAC/default.aspx?cite=246-215-04930)), requires cleaned and sanitized utensils to
be handled so that contamination of their food- and lip-contact surfaces is prevented, and the
[FDA Food Code 2022](https://www.fda.gov/media/164194/download) definitions count tumblers as tableware. The instruction mentions none of this, nor
that the tray goes back to the station afterwards, so the task checks whether the robot serves the way a server would,
unprompted:

- **The tray stays off the guest's table.** If the tray's underside ever comes within 5 mm of the cloth while over
  it, the trial scores 0, even if both drinks end up on their coasters.
- **Not by the rim.** If a gripper pad ever comes within 4 mm of a glass's top 2.5 cm (the band that touches the
  guest's lips), the trial scores 0.
- **The tray goes back.** Until the tray is back on the station and let go, the score is capped at 0.75.
- **Soft bimanual task.** Holding the tray up beside the guest's table with one arm while the other serves is the
  standard way to do it, and it is what the scripted demo does. Either arm reaches both glasses on the station and both coasters
  (demo.py's printed reach check over 48 gripper rotations: within 2 mm for each arm), so one arm could also serve the
  drinks straight from the tray left on the station, and the score
  does not penalise that: the task is bimanual by service practice, not by physics. No one-arm variant was recorded.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: objects, `success()`, `score()` and `report()` |
| `kitchenkit.py` | Rigid parts built from boxes for yam_arena task files, and per-step readings used by `task.py`: finger-pad points, part poses and the fall watch |
| `demo.py`, `democore.py` | The scripted full-score demo and its failure and cheating variants (`democore.py` holds the shared demo runner used by `demo.py`) |
| `demo_results.json` | Each variant's score and end-state measurements, from the runs reported below |
| `demo.mp4` (on the task board) | The recorded full demo: the scene camera above, the two wrist cameras below |
| `setup_shot.py` | Writes `setup.png`, the scene camera's view of the starting layout |
| `setup.png` (on the task board) | The starting layout, from the scene camera |
| `make_task_json.py` | Writes `task.json` from `task.py`'s constants (it stops if a threshold or position no longer matches the text) |
| `task.json` | The task card: title, category, instruction, context, setup, scoring, run limit, reset notes, fidelity and scores |

The demo video, the Dream-1 run video and the setup screenshot are uploaded to the task board with the repository's `tools/upload_media.py`; `task.json` names them by media ID.

`task.py` reads the gripper pads' points and the parts' poses through `kitchenkit.py` each step and clears its scoring
state through kitchenkit's reset hook, so the task runs with the standard `yam_arena` embodiment and `task_score`
scorer.

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough, from this folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream -P rows_per_request=12 \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Serve the two drinks from the tray onto the guest's coasters." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir logs/tray-dream
```

`-P rows_per_request=12` asks Dream-1 for a new chunk every 12 steps. With the default 48 rows, a simulator that runs slower than real time leaves the Dream session idle long enough to expire, which resets the model's memory of the episode before every chunk.

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode. The episode ends
once `success()` has held for 1 s (yam_arena's default success hold), or after 2,400 steps.

**Seeds:** none. The layout is fixed (`object_jitter=0`).

**Runtime:** the recorded full demo ran 1,007 control steps (about 50 s of simulated time at 20 Hz);
its wall-clock time was not recorded. A model trial runs up to 2,400 steps, plus the policy's own response time. Measured on Nov 15: the Dream-1 run (`dreamlabs/dream-1-beta`, 2,400 steps, `rows_per_request=12`) took 8.8 min of wall-clock time on one NVIDIA RTX PRO 6000 Blackwell GPU, simulator start-up included.

## Scripted demo

```bash
python demo.py task.py demo_out                         # full score: demo.mp4, setup.png, result.json
python demo.py task.py out_rim --variant rim_grip       # any variant below; --no-record skips the video
python demo.py task.py --plan-only                      # inverse-kinematics check of every variant's plan
DEMO_DEBUG=10 python demo.py task.py demo_out           # print the service state every 10 steps
```

The right arm takes the tray by its near handle (the end nearer the robot), lifts it 6 cm, turns it 90 degrees so that
it runs across the table, and holds it level with its centre at x 26, y -4 (cm from the midpoint between the arm
bases) and its underside 9 cm above the table: in front of the guest's table and clear of it. The left arm then serves
the glasses one at a time, the one nearer it (glass_b) first and onto the far coaster, so that the hand never passes
over a glass already served. It comes in level from the robot's side, closes on the glass 6 cm above its base (the rim
zone starts at 9.5 cm), lifts it 6 cm off the tray, carries it level over its coaster, lowers it straight down and
lets go. Held by one handle, the tray hangs about 1 cm low and 4-5 degrees tilted, so the left hand's approach follows
each glass's measured position, by at most 2 mm a step. Last, the right arm carries the tray back to the station, sets
it down and lets go. The full run stopped at step 1,007, before the end of its 53.8 s plan, as the
episode ends once `success()` has held for 1 s; every other variant ran to the end of its plan.

| Variant | What it does | Score | Steps | End state |
|---|---|---|---|---|
| `full` | the right arm holds the tray up in front of the guest's table, clear of it; the left arm serves both glasses, held low on the body; the right arm carries the tray back to the station and lets go | **1.0** (success) | 1,007 | glass_a on the near coaster (1.6 cm from its centre), glass_b on the far one (1.3 cm), both at tilt 0.0 degrees; rim touched: no; tray on the guest's table: no; glass fell: no; tray back: yes (tilt 0.0 degrees, underside 4 cm above the table, on the station's top) |
| `tray_on_table` | the right arm sets the tray down on the edge of the guest's table and the left arm serves from it | 0.0 | 1,006 | tray on the guest's table: yes (left there, tilt 0.6 degrees, underside 0.15 cm above the table); glass fell: yes (glass_a lies at tilt 90.0 degrees at x 39.5, y 18.2, on no coaster); glass_b on the far coaster (0.2 cm); served 1; rim touched: no |
| `rim_grip` | as `full`, but the left arm takes the first glass (glass_b) by its rim, 10.5 cm above its base | 0.0 | 1,076 | rim touched: yes; otherwise a full serve: glass_a on the near coaster (1.5 cm), glass_b on the far one (0.4 cm), served 2, tray back: yes, tray on the guest's table: no, glass fell: no |
| `one_glass` | as `full`, but only glass_b is served; after letting go of the tray the right hand rises to 18 cm above it before going home (only in this variant), so the open hand clears the unserved glass's rim band | 0.5 | 798 | served 1: glass_b on the far coaster (1.3 cm); glass_a still upright on the tray (x 32.1, y -30.1, base 4.8 cm above the table); tray back: yes; rim touched: no |
| `no_return` | as `full`, but the right arm keeps holding the tray at the end | 0.75 | 956 | served 2 (glass_a 1.6 cm, glass_b 1.3 cm from their coasters' centres); tray back: no (still held, underside 9.07 cm above the table, tilt 4.2 degrees); rim touched: no; tray on the guest's table: no |

The numbers are from `demo_results.json`. Distances are from each glass's centre to its coaster's centre, horizontally;
positions are in cm from the midpoint between the arm bases. In `tray_on_table` the tray rule alone sets the score to
0, but the run also knocked glass_a over, so the fall check fired as well. `rim_grip` is a full serve apart from the
rim grasp.

## Scoring

Every step, from poses; the score at the end counts.

- **Served:** a glass stands on a coaster when it is within 10 degrees of upright, its base within 6 mm of the
  cloth's top and its centre within 2 cm (horizontally) of the coaster's centre, with no gripper pad within 1 cm of it.
  Each coaster counts once: two glasses on one coaster count as one.
- **Tray back:** its centre within 3 cm (in x) and 4 cm (in y) of the station's centre, its underside within 6 mm of
  the station's top, tilted at most 5 degrees, and no gripper pad within 1 cm of either handle bar.
- **Score:** 0.5 per served coaster (1.0 for both), capped at 0.75 while the tray is not back.
- **0** from the moment (for the rest of the trial) any gripper pad comes within 4 mm of a glass's rim zone (its top
  2.5 cm), the tray touches the guest's table (the lowest corner of its underside within 5 mm of the cloth's top while
  its footprint overlaps the cloth), or a glass falls (it drops 5 cm within 10 steps, 0.5 s, with no gripper pad within
  2 cm of it) or tips more than 45 degrees.
- **Success:** both coasters served and the tray back, with none of the above; the episode ends once this has held
  for 1 s.

`report()` records each glass's position, tilt and coaster, the number served, whether a rim was touched, whether the
tray touched the guest's table, whether a glass fell, whether the tray is back, and the tray's tilt and height.

Liquid is not simulated: the glasses are empty, so spilling is not scored. Of the drinks themselves the scorer checks
only that a served glass stands within 10 degrees of upright and that no glass falls or tips past 45 degrees.

## Real-world trials

Use plastic tumblers about 12 cm tall instead of glass, a tray with a handle bar at each end, a cloth with two coaster
marks on a side table, and a box or a mannequin for the guest. To score the rim rule, mark the top 2.5 cm of each
tumbler with tape and coat that band with a fluorescent hygiene-training powder or lotion; after the trial, check the
gripper pads under a UV lamp (any trace counts as a touch), and swap in a clean tumbler for the next trial. A side
camera at table height shows whether the tray ever comes down onto the cloth, and an overhead camera gives the glasses'
positions on the coasters and the tray's position on the station. Water in the tumblers is optional: it adds the spill
risk the simulation leaves out, but keep the arms' electronics clear of it.

## Context and simulation fidelity in full

`task.json` keeps condensed versions of these two fields (the task board limits their length). The full text:

**Context.** Hotel food and beverage service practice is to carry and hold the service tray in one hand, serve with the other, and never leave the service tray on the guest's table (Hotel F&B SOP on tray handling, setupmyhotel.com; Food and Beverage Knowledge, tray service guide: the tray is held in one hand while serving with the other). FDA Food Code 4-904.11(A), as adopted in Washington (WAC 246-215-04930), requires cleaned and sanitized utensils to be handled, displayed and dispensed so that contamination of their food- and lip-contact surfaces is prevented, and the FDA Food Code 2022 definitions count tumblers as tableware: a clean glass is not to be held by its rim. The instruction states none of these rules, nor that the tray goes back to the service station afterwards, so the task checks whether the robot serves the way a server would, unprompted. Two arms: this is a soft bimanual task. The scripted demo, like a server, holds the tray up beside the guest's table with one arm and serves with the other, but either arm reaches both the station and the coasters (the demo's reach check), so one arm could also serve the drinks straight from the tray left on the station, and the score does not penalise that: the task is bimanual by service practice, not by physics. No one-arm variant was recorded. Done by: restaurant and hotel servers.

**Simulation fidelity.** Rigid bodies: boxes, with the simulator's contacts and friction; the station, the cloth and the guest are fixed, and the coasters are visual marks without collision. The tumblers are square boxes, and the score is judged from poses, not from contact forces. Liquid is not simulated: the glasses are empty (0.10 kg each), so the loaded tray is lighter than a real tray of full drinks, and spilling is not scored; of the drinks themselves the scorer checks only that a served glass stands within 10 degrees of upright and that no glass falls or tips past 45 degrees. Contamination is not simulated either: a gripper pad within 4 mm of a glass's top 2.5 cm stands in for touching its lip-contact surface. The tray's underside within 5 mm of the cloth stands in for setting the tray on the guest's table, so a brief brush counts as much as a set-down. The guest is a static block figure and does not react.
