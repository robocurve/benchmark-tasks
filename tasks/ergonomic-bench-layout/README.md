# Lay out parts bins for a seated assembler

A static stand-in for a seated worker sits just beyond the far edge of the table, facing the arms, with both
forearms resting on the bench. A thin pad between the worker's hands and the arms marks the worker's reach zone.
Three closed parts bins of the same material, clearly different in size (180, 140 and 100 mm long; 300, 157 and
69 g), stand on the bench outside the pad. The robot must put all three on the pad, upright and not stacked,
ordered by the standard ergonomic rule: the heaviest nearest the worker's body, the lightest farthest, with at
least 20 mm between bins. Weight is read from size (same parts, same fill), and the order is by distance to the
person, not along a fixed table axis.

Files: `task.py` (scene, scoring, scripted demo), `task.json`, `test_task.py` (offline checks).

## Why this task

- **Category:** Impact on jobs (assembly-line setup).
- **Setting:** industrial engineers and line leads lay out assembly workstations for the assemblers who work at
  them.
- **The ordering rule:** keep heavy items close to the body and lighter ones farther out. The load on the shoulder
  and lower back grows with a load's weight times its horizontal distance from the body, so the heaviest bin
  belongs at the shortest reach. The task applies this ordering rule; it does not model a complete ergonomic
  workstation (the marked pad is farther from the chest than a normal seated reach zone, to suit the robot's
  reach; see `fidelity`).
- **What it tests:** finding a person-relative frame (where the worker's body is, which way their reach runs) from
  a static scene; inferring mass from size under a stated same-material rule; an ordering rule measured by
  distance to the body; placement inside a marked zone with spacing; not taking the shortcut of stacking.
- **Estimated human time:** about 8-12 s for an adult (three one-handed pick-and-places over 30-50 cm, with a
  glance to rank them; an estimate, not timed).
- **Signature:** `pick-place+sequence-order / rigid / ordering-rule+human-in-loop / one`.

### Novelty check

- **Task board** (90 pending tasks as of 2026-11-02): none sets up a
  workstation for a person or orders objects by distance to a person. The human-involving tasks are setting a
  table with a human, packing or sorting alongside a human, restoring an arrangement a human disturbed, and
  handing a knife or scissors to a mannequin hand. None uses a reach zone or a weight rule.
- **Popular sets** (LIBERO, RoboCasa, SimplerEnv, BEHAVIOR, DataCenterBench, RoboDojo, plus RoboTwin 2.0 and
  RoboCasa365): the nearest are size rankings along a table
  axis or in a stack: RoboTwin 2.0 `blocks_ranking_size` (three blocks, largest to smallest, left to right), RoboDojo
  `sort_nesting_dolls_by_size`, RoboCasa365 `SizeSorting` (stack largest to smallest) and `OrganizeMeasuringCups`.
  This task differs in structure: the order's reference is a person's body (radial distance to the chest), placement
  is constrained to the person's reach zone with spacing, the rule is a stated ergonomic weight rule (size is only
  the cue), and stacking, the move a size-stacking task rewards, scores 0.

## Run a trial

Built on `yam-isaac-lab-arena` at `bd62771` (the `yam_arena` embodiment for Inspect Robots). From the folder that
holds this task:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=ergonomic-bench-layout/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "A worker sits at the far side of the bench, facing you, with both forearms resting on the bench. The pad between the worker's hands and you is the worker's reach zone. Set up the worker's station: put all three parts bins on the pad, standing upright and not stacked, with the heaviest bin nearest the worker's body and the lightest farthest from it, and at least 2 cm of clear space between bins. The bins hold the same parts, so a bigger bin is a heavier bin." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/ergonomic-dream
```

Scripted demo (right arm only, 34.1 s = 682 planned steps). The last release ends at 29.6 s (592 planned steps), so
success can fire from about 1 s later; with the measured 60-70 step lag, expect it between about step 650 and 750
(the recorded demo ended on success at step 610). `--max-steps` is the plan plus 100:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=ergonomic-bench-layout/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "A worker sits at the far side of the bench, facing you, with both forearms resting on the bench. The pad between the worker's hands and you is the worker's reach zone. Set up the worker's station: put all three parts bins on the pad, standing upright and not stacked, with the heaviest bin nearest the worker's body and the lightest farthest from it, and at least 2 cm of clear space between bins. The bins hold the same parts, so a bigger bin is a heavier bin." \
  --scorer task_score --max-steps 782 --store-frames --log-dir ~/yam-eval/ergonomic-demo
```

`--scorer task_score` prints the trial's score at the end: 1.0 on success, otherwise `score()` at the last step
(`SCORE_OVER = "end"`).

- **Run limit:** 2,400 steps (2 minutes at 20 Hz), about 3.5 times the scripted demo.
- **Seeds / jitter:** pass `-E object_jitter=0`. The layout is fixed; the medium bin starts 23 mm off the pad's near
  edge, and the default 3 cm jitter could put it partly on the pad. With no jitter the seed does not change the
  layout.
- **Runtime per trial:** about 70 s wall clock for the demo and 225 s for the Dream-1 trial on an RTX 5090, Isaac start-up included. Simulated time: up to 120 s for the Dream-1 trial, about 34 s
  for the demo, plus 50-85 s of Isaac start-up per run once shaders are cached.
- **Offline tests:** `YAM_ARENA=~/yam-isaac-lab-arena python -m pytest -q test_task.py` (58 tests, about 15 s,
  numpy and pytest only).
- **Assets:** none. Every object is a box, cylinder or sphere primitive defined in `task.py`; nothing to attribute.

## How it is scored

**Distance to the worker** is measured horizontally from the middle of the worker's chest (x 940, y -210 mm) to a
bin's centre. On the goal rows the three bins sit 515, 630 and 740 mm from the chest.

**On the pad:** centre over the pad (x 170-480 mm, y -340 to -80 mm), lowest point at most 15 mm above the pad's top
(so not held in the air and not on another bin), and upright (vertical axis within 20 deg; a tipped bin spills its
parts).

**Zero outright:** any bin on the floor (centre below the bench top), or any bin resting on another (its centre over
the other's footprint and its bottom at or above the other's top, less 10 mm). A stack is not a layout and puts
the heavy bin's load on top of the worker's reach.

| part | credit |
|---|---|
| each bin on the pad | 0.15 (three bins: 0.45) |
| each pair in the right order: both on the pad, the heavier at least 20 mm nearer the chest (large-medium, medium-small, large-small) | 0.15 (three pairs: 0.45) |
| all three on the pad with at least 20 mm of clear space between every two, seen from above | 0.10 |

Score = the sum, at most 1.0. Success: score 1.0 and every bin still (`resting()`: under 1 cm/s and 0.5 rad/s) for
1 s.

Outcomes (all tested): the start 0; one bin on the pad 0.15; two on the pad in order 0.45; all three in reverse
order 0.55; one neighbouring pair swapped 0.85; the large and small swapped 0.55; all three in order but touching
0.90; spacing 19 mm 0.90 and 21 mm 1.0; a stack of all three 0; one bin on another, on or off the pad, 0; any bin
on the floor 0; a tipped bin on the pad earns nothing for itself; a bin held 30 mm over the pad is not on it; two
bins side by side at the same distance from the chest are not an order; a row laid along the chest line instead of
toward the worker scores 1.0 only when the radial distances are in order. Random layouts stay in 0..1.

`distance()`: horizontal distance from the pad, summed over the bins, plus 0.1 m per pair out of order (0 at the
goal).

### Scoring a real trial with a tape measure or a photo

Mark the middle of the worker stand-in's chest at bench height. At the end of the trial:
1. For each bin, find the centre of its top (where the diagonals cross). It is on the pad if that point is over the
   pad, the bin stands on the pad (not on another bin, not held) and is not tipped over.
2. Measure from the chest mark to each bin's centre. Each pair counts if the bigger bin is at least 20 mm nearer.
3. Measure the narrowest gap between each two bins: 20 mm or more for the spacing credit.
A top-down photo with the pad, the bins and the forearm tips in frame settles all three; on the goal rows the bins
are 110-115 mm apart in distance and 56-61 mm apart in clear space, so mistakes are large.

## Geometry choices (and why)

- **Where the worker sits:** beyond the far edge, facing the arms, in front of the right arm (y -210 mm). The
  first top-camera frame shows the right arm's ready pose covering the table from about y -0.38 to -0.58 m across
  the whole reachable depth (and the left arm mirrors it), so a worker at the right edge with a zone in front of
  them would have the zone hidden at step 0. Across the bench, the zone sits in the clear band between the two.
- **Gripper:** pads are 93.9 mm apart fully open (`FINGER_OPEN = -0.04695` m per finger in
  `yam_arena/constants.py`); usable grip taken as 84 mm, as in the earlier tasks. Gripped widths are 64, 54 and
  44 mm (the bins' widths), at least 20 mm under the usable grip. The demo opens each pad only 12 mm wider than the
  bin on the way down and 10 mm to let go.
- **Bin masses:** 300 g at most, under the 370 g that the first GPU round held by a 2.5 cm band with no slip. The
  pads cover 28-48 mm of each bin's height (TCP 30-40 mm above the bin's base; fingertips 17-27 mm above the
  table; bin tops 15-35 mm above the TCP, at least 19 mm under the finger carriage).
- **Sizes for the camera:** lengths differ by 40 mm (180 / 140 / 100), at least 4 px apart at 320 x 176 on the goal
  rows (tested); widths and heights also grow with size, so no cue contradicts another.
- **Reach:** the large bin's goal (x 425, y -210 mm) is near the right arm's limit: x 440 mm at y -100 mm and x 455
  mm at y -190 mm failed the IK check. Every pick, every goal row for every bin (so a wrong order is buildable)
  and the whole demo pass at 20 Hz with IK error under 0.5 mm / 0.1 deg and joints in range. Joint 6 limits the
  gripper's yaw: the demo grips at +90 deg near the arm and parks at yaw 0.
- **Row spacing:** 56 and 61 mm of clear space on the goal rows. The grip offset falls along the finger closing
  axis, which is x at the goal (the ordering axis); the kinematic sweep shifts every carried bin by 0, ±15 and
  ±20 mm and still finds no contact between the fingers, the carried bin, a placed bin, the pad or the worker, and
  the end state still scores 1.0 with at least 35 mm between bins.
- **Order of moves:** large to the row nearest the worker, then medium, then small, so no carried bin passes over a
  standing one. The carry height (TCP 130 mm above the bench) keeps carried bottoms 15 mm or more above the tallest
  bin anyway.
- **Arm and worker:** the right arm's links stay at least 14 cm from the stand-in all run (the forearm tips are the
  nearest part); the torso and head are about 47 cm away.
- **Camera:** in the first frame (both arms at the ready pose) and the plan's parked pose (right arm at x 150,
  y -450 mm, 200 mm up, off to the right of the pad) nothing of the arms lies within 5 cm of the line of sight to
  any pad point or bin top (tested with an FK skeleton). The episode ends on success before that park, so in the
  recorded demo's last frame the right wrist still covers part of the medium and small bins and the gap between
  them. The top camera frames the pad, the bins at the start and the
  goal, the forearms and the lower chest; the shoulders and head are above the frame.

## Verification

Run on an RTX 5090 with `yam-isaac-lab-arena`, using the commands above (`object_jitter=0`):

- Scripted demo (`demo()`): `task_score` 1.0, ended by success; 70 s wall clock including Isaac start-up.
- Dream-1: `task_score` 0.0 after the full 2,400-step run limit; 224 s wall clock.
- Offline tests: 58 passed.

## Real-world notes

Real counterparts, cheap and buyable: three lidded polypropylene small-parts boxes from one product line in three
sizes (about 180 x 64 x 75, 140 x 54 x 60 and 100 x 44 x 45 mm outside; exact product not checked), each filled
with the same parts (for example M4 nuts) to 300, 157 and 69 g and taped shut; a sheet of 3 mm craft foam cut to
310 x 260 mm and taped flat; a worker stand-in (a seated mannequin, or a stool with a cardboard box as the torso, a
foam ball as the head and two foam blocks 70 x 60 mm as the forearms laid on the bench). The task is safe on a real
YAM: nothing is fragile or sharp, the heaviest load is 300 g, and the stand-in does not move.
