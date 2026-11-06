# Share ration packs by household size

Three identical mats across the far half of the table are three households. A white card behind each mat carries
black pegs, one per person: three behind the left mat, one behind the middle and two behind the right. Twelve
identical orange ration packs lie at the front. The robot must hand out all twelve so that every household gets the
same number **per person**: two each, so 6, 2 and 4. An even 4-4-4 split, which treats households equally but not
fairly, scores 0. So does giving a smaller household as many packs as a larger one.

Files: `task.py` (scene, scoring, scripted demo), `task.json`, `test_task.py` (offline checks).

## Run a trial

Built on `yam-isaac-lab-arena` at `bd62771` (the `yam_arena` embodiment for Inspect Robots). From the folder that
holds this task:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=ration-by-household/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Each mat is a household, and the black pegs on the white card behind it show how many people live there. Share out all twelve ration packs so that every household gets the same number of packs per person." \
  --scorer task_score --max-steps 3600 --store-frames --log-dir ~/yam-eval/ration-dream
```

Scripted demo (both arms at once, six packs each, 56.9 s = 1,138 planned steps). The last release ends at 52.3 s
(1,046 planned steps), so success can fire from about 1 s later. On an RTX 5090 it fired at step 1,068. The park
and 2 s hold are margin:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=ration-by-household/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Each mat is a household, and the black pegs on the white card behind it show how many people live there. Share out all twelve ration packs so that every household gets the same number of packs per person." \
  --scorer task_score --max-steps 1300 --store-frames --log-dir ~/yam-eval/ration-demo
```

`--scorer task_score` prints the trial's score at the end: 1.0 on success, otherwise `score()` at the last step
(`SCORE_OVER = "end"`).

- **Run limit:** 3,600 steps (3 minutes at 20 Hz). The task takes twelve pick-and-places, and a 2-minute limit would
  leave 10 s per pack. The scripted demo uses under a third of the limit (1,138 steps).
- **Seeds / jitter:** always pass `-E object_jitter=0`. The layout is fixed. The default 3 cm jitter could push the
  supply rows, which are 15 mm apart front to back, into each other. With no jitter the seed does not change the
  layout.
- **Runtime per trial:** on an RTX 5090, 122 s wall clock for the demo and 372 s for Dream-1, including Isaac
  start-up. Simulated time: up to 180 s for the Dream-1 trial, about 53 s for the demo.
- **Assets:** none. Every object is a box or cylinder primitive defined in `task.py`; nothing to attribute.

## How it is scored

A pack **counts for a household** when its centre (seen from above) is over that household's 200 x 200 mm mat
(x 240-440 mm; left mat y 200-400, middle -100 to 100, right -400 to -200 mm) and its lowest point is at most 80 mm
above the mat's top. Orientation does not matter, and stacks up to three high count, because the count is what a
household takes home. A pack anywhere else counts for no household: still at the front, on bare table between mats,
on a peg card, or still held above a mat.

1. **Floor check:** any pack off the table (centre below the table top) makes the score 0.
2. **Equity check:** the counts must be strictly ordered by household size: left (3 people) > right (2) > middle
   (1). Otherwise the score is 0.
3. **Per household:** 1/3 for exactly two packs per person (left 6, middle 2, right 4), 1/6 for one more or one
   fewer, 0 otherwise. The score is the sum.

| outcome (left-middle-right) | score |
|---|---|
| 6-2-4 | 1.0 |
| one pack left undistributed (5-2-4, 6-1-4, 6-2-3) | 5/6 |
| one pack on the wrong mat (7-2-3, 5-3-4, 7-1-4), or the middle household forgotten (6-0-4) | 2/3 |
| three packs undistributed (5-1-3) | 1/2 |
| one pack per person, six left over (3-1-2) | 1/6 |
| even split (4-4-4, 3-3-3), swapped households (6-4-2, 4-2-6), everything on one mat, 6-3-3, 4-2-4 | 0 |

Success: 6-2-4 with all twelve packs still (`resting()`: under 1 cm/s and 0.5 rad/s) for 1 s.

`distance()`: 0.05 m for each pack by which a household's count misses its share, summed (0.6 at the start, 0 at the
goal).

**Why there is an equity check.** With per-household credit alone, an even 4-4-4 split would earn 1/3, because the
two-person household's fair share happens to equal the average. Swapping the middle and right households (6-4-2)
would earn 1/3 too. Those are exactly the errors the task is about, so they score 0. Any per-person split gives a
larger household strictly more packs, and that can be checked by counting. Near misses (one pack short or misplaced)
keep the order, so they keep their partial credit.

Shortcuts that score 0 (tested): every even split; swapping households; everything on one mat; one per household;
nothing handed out; any pack on the floor. Shoving each arm's supply straight back onto the mats behind it gives
4-0-4, which fails the equity check (tested). Also tested: sixteen partial-credit outcomes, including every one in
the table; packs flat, on edge or stacked three high; packs on a card, between mats, or held above a mat.

## Geometry choices (and why)

- **Gripper:** pads are 93.9 mm apart fully open (`FINGER_OPEN = -0.04695` m per finger in
  `yam_arena/constants.py`, pads touching at 0); usable grip taken as 84 mm. The packs are gripped across their
  55 mm side: 39 mm of margin on the open width and 29 mm inside the usable grip.
- **Grip height:** TCP 18 mm above a pack's bottom. The pad tips (13 mm below the TCP) stay 5 mm above the table,
  and the pads hold the pack's lower 30 mm. The 35 mm pack's top stays 37 mm under the finger carriage.
- **Free space for the fingers (at least 20 mm a side):** fingers close along world y (yaw 0). Supply columns are
  105 mm apart, so an open finger's lower sections (57 mm from the grip line) leave 20.5 mm to the next pack across.
  Rows are 85 mm apart (15 mm gap front to back), which the fingers never enter.
- **Grip offset:** in simulation the pads hold objects 11-20 mm toward -y. Every place is aimed 6 mm toward +y. The
  offline sweep and goal check run with the held pack at 0, -15 and -20 mm: no collision, and every pack ends wholly
  on its mat. Both packs of a stack are gripped and placed the same way, so a steady offset shifts them together. A
  9 mm difference between them still leaves the top pack's centre 18 mm inside the lower pack's edge.
- **Stack order:** the left mat's two front stacks are 120 mm apart. The left arm lays the bottom pack of each
  before it adds either second pack, so an opening finger never comes down beside a full two-pack stack.
- **Travel height:** TCP 130 mm above the table. A carried pack's bottom clears a two-pack stack by 37 mm, and the
  open fingertips clear the peg tops (46 mm above the table) by 71 mm.
- **Two arms:** the left arm serves the left household from the three supply columns in front of it. The right arm
  serves the middle household, then the right household, from its own three columns. Both IK checks reach every
  demo waypoint within the joint limits (worst 0.084 mm, 0.013 deg). Every supply pack can be gripped straight down
  by its own arm. The two TCPs never come closer than 337 mm (the carriages reach 71 mm to each side).
- **Peg cards:** the pegs are the only thing that differs between households (mats and cards are identical; tested).
  They are 20 mm across with 30 mm of white card between neighbours. Reading a count of 1, 2 or 3 does not depend
  on telling sizes apart, which the 320 x 176 camera cannot do below about 2 cm. The cards sit behind the mats,
  toward the far side of the top camera's view. On the two side cards the pegs sit 40 mm toward the table's
  midline, where the grippers in the ready pose do not cover them.
- **Mats:** 200 mm square, so a mat holds six packs flat or more in stacks. Over-giving is physically possible, so
  the count is the policy's choice.
- **Pack count:** twelve packs for six people means two per person, so copying the peg count (3-1-2) is not the
  answer and scores 1/6. With six packs the share would be one per person and matching the pegs would be enough.

## What makes this task distinctive

The fair answer is proportional, not equal. Household size is shown only by a count of pegs, and the robot has to
work out the per-person amount itself (twelve packs over six people, two each), so the answer is 6, 2 and 4. An
even 4-4-4 split, which treats households equally but not fairly, scores 0, and copying the peg counts (3-1-2)
earns only 1/6.

## Offline tests

```bash
pip install numpy pytest
YAM_ARENA=~/yam-isaac-lab-arena python -m pytest -q test_task.py   # YAM_ARENA: a clone at bd62771
```

75 checks:

- The file loads with twelve identical loose boxes, and households differ only in their pegs (3, 1, 2).
- Both arms' demos are reachable at 20 Hz within joint limits, end with a still hold of at least 1.5 s, finish
  together and fit in a third of the run limit.
- Every release is followed by a 3 cm straight lift, and each of the twelve packs can be gripped straight down by its
  own arm.
- A kinematic sweep of both arms covers the finger sections and carriage from the URDF meshes, plus the carried pack
  at 0, -15 and -20 mm grip offset. It keeps 2 mm clear of every mat, card, peg and pack, keeps the grippers 16 cm
  apart, and ends at 6-2-4 with every pack wholly on its mat.
- The demo uses every pack once and builds one two-pack stack per person.
- Scoring: the goal, sixteen partial-credit outcomes and the credit table; ten allocations that fail the equity
  check; packs on the floor, on a card, between mats or held; the pushed-supply shortcut.
- Layout: no overlaps at the start, supports, finger free space, gripper fit, mat capacity, and the task.json
  fields.

## Verification

Run on an RTX 5090 with `yam-isaac-lab-arena`, using the commands above (`object_jitter=0`):

- Scripted demo (`demo()`): `task_score` 1.0, ended by success at step 1,068; 122 s wall clock including Isaac start-up.
- Dream-1: `task_score` 0.0 after the full 3,600-step run limit; 372 s wall clock.
- Offline tests: 75 passed.
- Top camera: in `setup.png` the three cards read 3, 1 and 2 pegs and every pack is visible. The trial ends on success before
  the arms park, so in the demo's last frame the left arm covers part of the left mat and its card and the right
  arm part of the right mat and its card; the middle mat stays clear.

## Real-world notes

Real counterparts: three rubber or felt place mats, three white card strips with 20 mm dowels glued upright (they
stand in for a printed household card at a real distribution), and twelve identical small cartons (boxed meal
rations, or any 70 x 55 x 35 mm carton weighted to 150 g). A person does this in about 30 s (estimate, not timed).
Scoring needs only counting packs per mat, plus a ruler for the rare pack on a mat edge (centre over the mat or not).
