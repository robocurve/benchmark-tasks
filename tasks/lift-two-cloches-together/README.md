# Lift two cloches together, straight up

Two plated main courses stand on the station table under silver cloches, one in front of each arm, and an empty
service tray lies on the table nearer the robot. The instruction is only: "Unveil the two guests' plates and put the cloches on the service tray."

In plated service each guest's cloche comes off at the same moment, and straight up. A
[Wasserstrom](https://www.wasserstrom.com/blog/2026/01/08/4-ways-to-use-a-cloche-to-enhance-food-presentation/) blog post on using cloches for food presentation describes synchronised service, one server
per guest lifting in a single smooth motion, and says a cloche that scrapes the plate breaks the presentation; the
[International Wedding Institute](https://internationalweddinginstitute.com/weddipedia/cloched-plate-service/) guide to cloched plate service says servers may remove the cloches
simultaneously, a coordinated reveal. The instruction states neither the timing nor the straight-up lift, so the task
checks whether the robot follows the service convention unprompted:

- **Synchronised lift-off.** Full score needs both cloches to leave their plates (knob 1 cm up) within 5 steps
  (0.25 s) of each other; within 1 s scores 0.5, within 3 s 0.25, and more than 3 s apart 0.
- **Straight up, no scrape.** Before a cloche's knob moves 3 cm sideways it must have risen 5 cm; a dragged cloche
  caps the score at 0.25.
- **Bimanual by service rule, not by load.** Each cloche weighs 0.25 kg and one gripper can carry it, but one gripper
  holds one cloche by its knob and the cloches stand 32 cm apart, so two lift-offs within 0.25 s take both arms. In the
  recorded runs a 0.6 s stagger (lift-offs 12 steps apart) scores 0.5,
  and unveiling one plate after the other (lift-offs 324 steps apart) scores
  0.0, although both cloches end on the tray in both runs.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: objects, `success()`, `score()` and `report()` |
| `kitchenkit.py` | Rigid parts built from boxes for yam_arena task files (visual-only parts for the placemats and food), and per-step readings: part poses, finger-pad proximity, fall and stillness watches |
| `demo.py`, `democore.py` | The scripted full-score demo and its failure and cheating variants (`democore.py` is shared with our other kitchen tasks) |
| `demo_results.json` | Each variant's score and end-state measurements, from the runs reported below |
| `make_task_json.py` | Writes `task.json` from `task.py`'s constants (it stops if a threshold or position no longer matches the text) |

`kitchenkit.py` hooks `yam_arena.scene` and the embodiment's step and reset without modifying yam_arena, so the task
runs with the standard `yam_arena` embodiment and `task_score` scorer.

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough, from this folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream -P rows_per_request=12 \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Unveil the two guests' plates and put the cloches on the service tray." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir logs/cloches-dream
```

`-P rows_per_request=12` asks Dream-1 for a new chunk every 12 steps. With the default 48 rows, a simulator that runs slower than real time leaves the Dream session idle long enough to expire, which resets the model's memory of the episode before every chunk.

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode.

**Seeds:** none. The layout is fixed (`object_jitter=0`).

**Runtime:** the full scripted demo ends at step 290 (14.5 s of simulated time at 20 Hz),
once success has held for 1 s; the longest variant run (`sequential`) is 688 steps. A model
trial runs up to 1,200 steps, plus simulator start-up and the policy's own response time. Measured on Nov 15: the Dream-1 run (`dreamlabs/dream-1-beta`, 1,200 steps, `rows_per_request=12`) took 3.3 min of wall-clock time on one NVIDIA RTX PRO 6000 Blackwell GPU, simulator start-up included.

## Scripted demo

```bash
python demo.py task.py demo_out                            # full score: demo.mp4, setup.png, result.json
python demo.py task.py out_staggered --variant staggered   # any variant below; --no-record skips the video
python demo.py task.py --plan-only                         # inverse-kinematics check of every variant's plan
DEMO_DEBUG=10 python demo.py task.py demo_out              # also print the cloches' state every 10 steps
```

Each arm opens above the knob of the cloche in front of it, closes on the knob, lifts the cloche straight up 6 cm,
carries it back over the tray and sets it down on the tray's centre line (x 12) at y 10 (left) or y -10 (right), lets
go and goes home. Both arms run the same timing, so the two lift-offs fall on the same step.

| Variant | What it does | Lift-off step (left / right) | Gap, steps | Dragged (left / right) | End state | Steps run | Score |
|---|---|---|---|---|---|---|---|
| `full` | Each arm pinches the knob of the cloche in front of it; both lift together, straight up 6 cm, carry the cloches back over the tray, set them down and let go | 113 / 113 | 0 (0 s) | no / no | both on the tray, released | 290 | **1.0** |
| `staggered` | As `full`, but the left arm starts 0.6 s after the right | 125 / 113 | 12 (0.6 s) | no / no | both on the tray, released | 376 | 0.5 |
| `sequential` | The left arm starts only after the right arm has put its cloche on the tray and gone home | 437 / 113 | 324 (16.2 s) | no / no | both on the tray, released | 688 | 0.0 |
| `drag` | Both lift only 1.5 cm, slide the cloches 12 cm back toward the tray at that height, then rise and carry on as `full` | 117 / 117 | 0 (0 s) | yes / yes | both on the tray, released | 414 | 0.25 |
| `on_table` | Both lift together, but set the cloches down on the table beside the plates (x 28, y 38 and x 28, y -38) instead of on the tray | 113 / 113 | 0 (0 s) | no / no | neither on the tray: both rims 1.25 cm below their start height (on the table); released | 364 | 0.0 |

The numbers are from `demo_results.json`. A trial that succeeds ends once success has held for 1 s; the others run
their whole plan.

## Scoring

Every step, from poses; the score at the end counts. Each cloche is followed by its knob, the point the gripper holds.

- **Lift-off:** the first step at which a knob has risen more than 1 cm above its starting height. The lift-off gap is
  the number of 20 Hz steps between the two cloches' lift-offs.
- **Straight up:** the first time a knob has moved more than 3 cm horizontally from its plate's centre, it must
  already have risen at least 5 cm; otherwise that cloche counts as dragged.
- **On the tray:** the cloche's rim within 6 mm of the tray's top surface, tilted at most 10 degrees, and all four rim
  corners inside the tray's edges or overhanging them by at most 1 cm. **Released:** no finger pad within 8 mm of the
  cloche (its 18 x 18 x 10 cm bounding box, knob included). **Still:** both cloches moving less than 2 mm over 1 s.
- **Any credit** needs both cloches lifted off, both on the tray, released and still at the end, and neither dropped
  at any time (dropped: it falls 5 cm within 0.5 s with no finger pad within 2 cm of it).
- **1.0** (success): neither cloche dragged and a lift-off gap of at most 5 steps (0.25 s).
- **0.5:** a gap of 6 to 20 steps (up to 1 s), neither dragged.
- **0.25:** a gap of 21 to 60 steps (up to 3 s), or a dragged cloche with a gap of at most 60 steps.
- **0:** a gap of more than 60 steps (3 s), and otherwise.

`report()` records both lift-off steps, the gap, the dragged and dropped flags, and each cloche's rise, sideways
distance from its plate's centre, on-tray and released state.

## Real-world trials

No hot food is needed: plate a cold or prop main course (the food is only seen, never touched), which removes any burn
or steam risk. Use light cloches with a knob the YAM gripper can pinch (the simulated ones weigh 0.25 kg; a round dome
works as well as a square one). Mark each plate's centre and the tray's outline with tape. Film from the side at 20
frames per second or more, with a ruler or grid behind the plates: lift-off is the first frame in which a rim is 1 cm
up, the gap is the difference in frames (5 frames at 20 frames per second is 0.25 s), and the straight-up check reads
whether each knob is 5 cm up before it has moved 3 cm sideways. Check the end state with a tape measure: both rims
flat on the tray, at most 1 cm overhang, grippers open and clear.

## Context and simulation fidelity in full

`task.json` keeps condensed versions of these two fields (the task board limits their length). The full text:

**Context.** In plated service the cloches over a table's main courses come off together. A Wasserstrom blog post on using cloches for food presentation describes synchronised service, one server per guest lifting in a single smooth motion, and says a cloche that scrapes the plate breaks the presentation; the International Wedding Institute's guide to cloched plate service says servers may remove the cloches simultaneously, a coordinated reveal. Here two plated main courses stand under cloches, one in front of each arm, and the instruction only asks to unveil the plates and put the cloches on the service tray. It states neither the timing nor the straight-up lift, so the task checks whether the robot follows the service convention unprompted. Two arms are needed because of the service rule, not the load: each cloche weighs 0.25 kg and one gripper can carry it, but one gripper holds one cloche by its knob and the cloches stand 32 cm apart, so two lift-offs within 0.25 s take both arms. A staggered or one-after-the-other unveiling earns partial or no credit. Done by: waiters and banquet servers in restaurants and at weddings and other plated banquets.

**Simulation fidelity.** Rigid bodies: boxes, with PhysX contact and friction. The cloches are square, open-bottomed boxes standing in for round silver domes; each is held only by friction in the gripper's pinch on its knob, so a slipping grip or a dropped cloche is physical. The plates and tray are fixed to the table. Not simulated: the food's heat and steam, and the food itself has no collision (it is visual only), so a rim dragged through it meets nothing. The scorer stands in for the scrape the service rule forbids with the straight-up check: a cloche's knob must have risen 5 cm, more than the height of any food on the plate, before it moves 3 cm sideways. The reveal is judged from lift-off times (knob 1 cm up), not from what a guest would see, and heat lost while a plate stands uncovered is not scored.
