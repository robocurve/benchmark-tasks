# Set six blocks evenly in a ring around a post

A dark dowel stands on the midline of the table, and six identical blue 3 cm cubes wait in a straight row nearer the
arms. The robot must set the six blocks out in a ring around the post:

> Place the six blue blocks in a ring around the post, each about 10 cm from the post's centre and evenly spaced
> all the way round.

Nothing on the table marks the ring: no template, no circle, no dots. "About 10 cm" has to be estimated, and
"evenly spaced" is judged only by the blocks' positions relative to one another, so the robot has to plan a layout,
not hit marked targets.

| File | What |
|---|---|
| `task.py` | the task file: objects, `success`, `score`, `distance`, scripted `demo` |
| `task.json` | the submission fields (`media` is added by the upload script) |
| `test_task.py` | offline checks (NumPy and pytest, no Isaac Sim) |

## Run a trial

From a checkout of the arena (yam-isaac-lab-arena, tested at `bd62771`) on the GPU VM, with this folder at
`tasks/ring-around-post/`. The layout is fixed (`-E object_jitter=0`), so no seed changes the scene. The scorer
itself needs no fixed start (it reads only where the blocks end up relative to the post), but the scripted demo is
open loop and does.

Dream-1:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/ring-around-post/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Place the six blue blocks in a ring around the post, each about 10 cm from the post's centre and evenly spaced all the way round." \
  --scorer task_score --max-steps 3600 --store-frames --log-dir ~/yam-eval/ring-around-post-dream
```

Scripted demo (38.2 s = 764 steps of waypoints: the right arm runs 5 s behind the left, and both end parked clear
of the ring in a still hold; the plan plus about 100 steps):

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/ring-around-post/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Place the six blue blocks in a ring around the post, each about 10 cm from the post's centre and evenly spaced all the way round." \
  --scorer task_score --max-steps 900 --store-frames --log-dir ~/yam-eval/ring-around-post-demo
```

Videos, one MP4 per camera, in each log's `frames/` folder:

```bash
for log in ~/yam-eval/ring-around-post-*/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score`: 1.0 when the trial ended in success, otherwise `score()` at the last step. Put it in
`task.json` `scores`, e.g. `"Dream-1 run: 0.17; demo: 1.0"`.

Runtime per trial: the run limit is 3,600 steps (3 minutes of simulated time at 20 Hz) for Dream-1 and 900 steps
(45 s) for the demo. Wall clock on an RTX 5090, Isaac start-up included: 87 s for the demo, 316 s for the Dream-1 run.
Never run two Isaac instances on one GPU.

Offline tests (about 3 minutes, most of it the arm-hull sweep and the collinear search):

```bash
YAM_ARENA_DIR=/path/to/yam-isaac-lab-arena python -m pytest tasks/ring-around-post/test_task.py -q
```

## Scoring in one paragraph

A block is **in the ring** when it sits flat on the table (a face within 10 deg of flat, bottom within 5 mm of the
table top), is still, and its centre is **80-120 mm** from the post's axis. Blocks not in the ring are ignored.
Sort the in-ring blocks by their angle round the post; a block is **placed** when the angles to both its
neighbours (the next in-ring block each way round) are **50-70 deg**. `score = placed / 6`. Success = all six
placed and still, held 1 s. Constants: `RING_R`, `RING_TOL`, `GAP_DEG`, `GAP_TOL_DEG`, `FLAT_DEG`, `ON_TABLE` at
the top of `task.py`.

| Outcome | Score |
|---|---|
| six evenly round, 80-120 mm out | 1.0 |
| five evenly placed, one missing (the two beside the gap face 120 deg) | 0.5 |
| one of six 15 deg out of place (it and both neighbours lose credit) | 0.5 |
| one of six outside 80-120 mm (it drops out; its neighbours face 120 deg) | 0.5 |
| four in a run of 60 deg steps | 0.33 |
| three in a run | 0.17 |
| two, or one, or none | 0 |

Every gap is computed from angles **sorted round the post**, so the result does not depend on which block went
where (they are identical) or on where the ring starts: any rotation of the ring scores the same.

## Shortcuts that score 0 (all tested)

| Strategy | Why it fails |
|---|---|
| Do nothing | every block starts 190-320 mm from the post, outside the band |
| Pile the blocks in a heap 10 cm from the post | neighbours in the heap are a few degrees apart |
| Stack them (a tower at one slot) | only the bottom block is on the table; one block alone has 360 deg on each side |
| A straight row of any length, anywhere | proved by exhaustive search: no three collinear blocks can give any block two 50-70 deg gaps (below) |
| Blocks pressed against the post | their centres are 30 mm from its axis |
| An even ring at 5, 15 or 20 cm | outside 80-120 mm |
| Five blocks evenly round (72 deg gaps) | 72 is outside 50-70 |
| Six blocks 45 deg apart on one side | 45 is outside 50-70; the end blocks face 135 deg |
| Blocks held up, propped on an edge, or knocked off the table | not flat on the table, so not in the ring |

**Why a straight row can never score.** Credit is per block and needs both its gaps to be 50-70 deg. If a block
earns it in some layout, then that block and its two neighbours alone still give it the same two gaps, so it is
enough to rule out three collinear blocks. On a line that crosses the 80-120 mm band once, every in-band point lies
within 2 x acos(80/120) = 96 deg, less than the 100 deg the middle block needs. On a line that crosses it twice
(passing nearer than 80 mm to the axis), each crossing spans at most about 40 deg, so two blocks on the same
crossing are under 50 deg apart and a lone block on one crossing faces at least 320 deg on its far side.
`test_no_three_blocks_in_a_straight_line_can_earn_anything` checks this numerically over every line 0-125 mm from
the axis and every triple of blocks 30 mm or more apart on a 3 mm grid. This is why there is no per-block radius
term (such as 1/12 for radius alone): on its own it pays for a heap or a row at the right distance.

## Design numbers

- **Gripper.** At `FINGER_OPEN` (-0.04695 m per finger) the pads are 94 mm apart (arena `constants.py`); the
  usable grip is taken as 84 mm (1 cm margin). Each cube is gripped across its 30 mm faces, 54 mm under that. The
  demo opens only to 54 mm on the way down (12 mm per side, `APPROACH_CLEARANCE`; 8 mm tipped a block in testing) and backs each pad off to 6 mm
  per side to let go.
- **Grip height.** The TCP grips 20 mm above the table: the pad tips are 7 mm above it and the pads hold the upper
  23 mm of the cube. The block is let go 3 mm above the table.
- **Reach.** The post sits on the midline at x = 0.30, where both halves of a 10 cm ring are in reach. The demo
  orients the ring with blocks at 30, 90, 150, 210, 270 and 330 deg (0 = straight out from the arms), so no slot
  sits on the midline: the far slots are at (0.387, +-0.05), not (0.40, 0), which is the edge of reach. The left
  arm places the three slots with y > 0 and the right arm the three with y < 0; neither crosses the midline (an
  arm cannot reach in front of the other arm's base). IK over the whole plan at 20 Hz: worst 0.087 mm / 0.013 deg
  (left) and 0.095 mm / 0.013 deg (right), every joint inside its limits; the tightest is joint4, 0.16 rad from its
  lower limit.
- **Joint6.** At the pick the grip yaw must be a multiple of 90 deg (the cube sits square to the table). Of the
  equivalent yaws, `DEMO_PLAN` picks those within 70 deg of the arm's heading; the naive choice put joint6 past
  its +-120 deg limit on the near blocks (25 deg of IK error before the fix).
- **The two arms.** The left arm fills its slots far to near (30, 90, 150 deg) and the right arm near to far
  (210, 270, 330 deg), so the grippers are never both at the far slots, which are only 10 cm apart, and **the right
  arm starts 5 s after the left** (`RIGHT_DELAY_S`, about half a block's 9.9 s cycle). Run in step, the offline
  hull sweep found the two forearms (link4 of one, link5 of the other) 11 mm apart over the midline in the last
  carry; reversing either arm's order made them overlap. With the stagger the arms stay more than 6 cm apart
  (the test asks for 5 cm), every link more than 2 cm from the post, and more than 15 mm from every block the arm
  is not working on (the block it has just let go is excluded while the gripper goes straight up from it: the pad
  tips start 1 cm over its top). The row was also moved 4 cm out from the midline for the same reason.
- **Carry height.** The TCP travels at 10 cm, so the held block's bottom is 8 cm up, 2 cm over the 6 cm post and
  5 cm over the blocks on the table. After each release the gripper rises 3 cm straight up before it moves.
- **Last frames.** The plan ends with each arm parked over its end of the row (x 0.15, y +-0.20), clear of the
  ring. The episode ends on success before that park, so in the recorded demo's last frame the right gripper
  still covers part of one block; the ring is readable.
- **Changes from the first design.** Post 3 cm x 6 cm at (0.30, 0) and 3 cm cubes as first designed; the row is
  split three and three (y = +-12, +-20, +-28 cm) with a 24 cm gap on the midline, so each arm has its own blocks
  and the arms keep apart. Radius tolerance widened
  from +-12 mm to +-20 mm (below). The first scoring (1/12 per block on radius, 1/12 per gap) was replaced by
  "1/6 per block with both gaps right", because the radius term alone pays for heaps and rows. Gap tolerance kept
  at +-10 deg.

### Why the tolerances are what they are (the grip offset)

Earlier simulation runs measured closing pads holding objects 6-20 mm off centre: one finger leads and the other
stays near its open stop, so the offset is roughly the approach clearance per pad (6 mm at 12 mm clearance on a
bar; 11-12 mm on cans and 15-20 mm on a jar and board gripped from fully open). It stays there at
release. On a ring of 10 cm, 15 mm sideways is 8.6 deg, most of the gap tolerance. The demo handles it in three
ways, and **the radius tolerance is widened rather than the place targets offset**:

1. **Small approach clearance** (12 mm per pad; 8 mm tipped a block in testing), which should keep the offset
   near 12 mm.
2. **The offset is aimed along the radius.** The grip turns during the carry (at most 30 deg) so that over each
   slot the fingers close along the ring's radius. An off-centre hold then moves the block toward or away from the
   post and leaves its angle round the post unchanged
   (`test_the_grip_offset_moves_a_block_along_the_radius_not_round_the_ring`).
3. **Radius band 100 +/- 20 mm** (first design: +/- 12). Budget: 15 mm modelled offset (above the 12 mm
   expected) + 3 mm wrist error at rest (unloaded; earlier runs measured about 2 mm unloaded, 5 mm under a 0.37 kg load)
   + 2 mm margin. 80-120 mm is still "about 10 cm", and the straight-row proof above holds up to +/- 21 mm.
   The demo's place targets are not offset, because the sign of the offset at each slot depends on which finger
   leads, which has not been measured at these yaws.

The gap tolerance stays at +/- 10 deg (50-70). Angle errors come only from TCP error across the radius:
`test_the_demo_still_scores_full_with_an_off_centre_grip_and_wrist_error` sweeps a +/-15 mm offset and up to 3 mm
of TCP error in x and y at the grip and at the release, chosen separately and adversarially for every block. Worst
case: 19.2 mm off the radius and 9.7 deg off a 60 deg gap, both inside the bands. With 5 mm of independent wrist
error per block the worst gap would be 15 deg; errors that are common to an arm (a steady sag) shift its half of
the ring together and change the gaps much less, but that is not tested here.

## How it differs from the nearest tasks (novelty)

Checked against the six named task sets and the board snapshot of 2026-11-02:

- **RoboDojo `align_blocks`:** a straight row against a set square. Here the layout is polar (a radius and equal
  angles), no tool or reference edge is given, and a straight row scores 0.
- **RoboCasa `PlaceVegetablesEvenly`:** two vegetables "evenly distributed" in a pan, judged as not overlapping.
  Here six items must reach a stated radius and equal 60 deg spacing, scored numerically.
- **Board B79 "Build a DUPLO structure from an image":** matching a shown target. Here there is no target image or
  mark; the goal exists only as numbers relative to the post and to the other blocks.
- **Board "Restore an arrangement a human disturbed" and other zone or line tasks:** goals are marked zones,
  lines or a remembered layout, not a metric pattern built from scratch.
- **embodied-first-exam `roboarc/slotted_post_ring_*`:** a metal-ring puzzle taken off a post; it shares only the
  words "post" and "ring".

What is new: a metric, mark-free radial layout where the score depends on the blocks' spacing relative to one
another, so the robot must plan the whole layout (six positions at equal angles) rather than move each block to a
target.

## Assets and licences

None. The post is a primitive cylinder and the blocks primitive boxes defined in `task.py`; the room and robot
come from the arena.

## Verification

Run on an RTX 5090 with `yam-isaac-lab-arena`, using the commands above (`object_jitter=0`):

- Scripted demo (`demo()`): `task_score` 1.0, ended by success; 87 s wall clock including Isaac start-up.
- Dream-1: `task_score` 0.0 after the full 3,600-step run limit; 316 s wall clock.
- Offline tests: 88 passed.
