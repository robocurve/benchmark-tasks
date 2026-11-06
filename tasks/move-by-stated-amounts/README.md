# Move one bar 5 cm, turn another a quarter turn, leave the third alone

Three painted wooden bars (8 x 2.5 x 2.5 cm) lie in a row across the table, each along y: red on the left,
green in the middle, blue on the right. The instruction gives one metric move, one metric turn and one "leave it"
constraint in a single sentence:

> Move the red bar 5 cm to the left (toward the left arm), turn the blue bar a quarter turn where it lies, and
> leave the green bar where it is.

Nothing on the table marks a goal. Every amount is relative to the bar's own unmarked start, so the robot has to
turn "5 cm" and "a quarter turn" into motion, and leave the third bar in its starting position.

| File | What |
|---|---|
| `task.py` | the task file: objects, `success`, `score`, `distance`, scripted `demo` |
| `task.json` | the submission fields (`media` is added by the upload script) |
| `test_task.py` | offline checks (NumPy and pytest, no Isaac Sim) |

## Run a trial

From a checkout of the arena (yam-isaac-lab-arena, tested at `bd62771`) on the GPU VM, with this folder at
`tasks/move-by-stated-amounts/`. **The layout must be fixed (`-E object_jitter=0`)**: the scorer measures each bar
against its starting position in `task.py` (hooks see only the current state), so a jittered start would be scored
against the wrong origin. No seed changes the scene; the scripted demo is open loop and needs the fixed layout too.

Dream-1:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/move-by-stated-amounts/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Move the red bar 5 cm to the left (toward the left arm), turn the blue bar a quarter turn where it lies, and leave the green bar where it is." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/move-by-stated-amounts-dream
```

Scripted demo (both arms at once, 12.2 s = 244 steps of waypoints, ending in a 2 s still hold; the plan plus
about 100 steps):

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/move-by-stated-amounts/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Move the red bar 5 cm to the left (toward the left arm), turn the blue bar a quarter turn where it lies, and leave the green bar where it is." \
  --scorer task_score --max-steps 350 --store-frames --log-dir ~/yam-eval/move-by-stated-amounts-demo
```

Videos, one MP4 per camera, in each log's `frames/` folder:

```bash
for log in ~/yam-eval/move-by-stated-amounts-*/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score`: 1.0 when the trial ended in success, otherwise `score()` at the last step. Put it in
`task.json` `scores`, e.g. `"Dream-1 run: 0.25; demo: 1.0"`.

Runtime per trial: the run limit is 2,400 steps (2 minutes of simulated time at 20 Hz) for Dream-1 and 350
steps (17.5 s) for the demo. Wall clock on an RTX 5090, Isaac start-up included: 27 s for the demo, 224 s for
the Dream-1 run. Never run two Isaac instances on one GPU.

Offline tests (about 20 s, 49 tests):

```bash
YAM_ARENA_DIR=/path/to/yam-isaac-lab-arena python -m pytest tasks/move-by-stated-amounts/test_task.py -q
```

## Scoring in one paragraph

`score = (red + blue) x green`. Red and blue are each worth 0.5 (full) or 0.25 (partial); green is a multiplier
(x1 left alone, x0.5 bumped, x0 moved or knocked over). Exact bands are in `task.json` `scoring` and in the
constants at the top of `task.py` (`RED_FULL`, `RED_PART`, `BLUE_FULL`, `BLUE_PART`, `GREEN_STILL`,
`GREEN_BUMPED`). Success = score 1.0 with all three bars still, held 1 s.

- **A quarter turn either way counts.** The blue bar's turn is the angle between its long axis now and at the
  start, folded to 0-90 deg because a bar looks the same end for end. +90 (anticlockwise from above), -90
  (clockwise) and 270 all score as a quarter turn; 180 is no turn. The instruction does not name a direction, so
  the scorer does not either.
- **Slide or lift, same score.** Only the final state counts. The demo lifts the red bar 15 mm and sets it down;
  a policy that pushes it along the table scores the same if it ends in the same place with the same heading
  (`test_pushing_instead_of_lifting_scores_the_same`).
- **The green bar is a gate, not a reward.** The original idea gave 0.2 for leaving green alone; that would hand a
  do-nothing policy 0.2. As a multiplier, doing nothing scores 0, and disturbing green costs half or all of what
  the other two bars earned.

## Shortcuts that score 0 (all tested)

| Strategy | Why it fails |
|---|---|
| Do nothing | red and blue earn 0; green's x1 multiplies 0 |
| Move every bar 5 cm left, or turn every bar | green moved/turned: x0 |
| Move the right amount but the wrong bar (green slid 5 cm; blue slid and red turned) | green x0, or red/blue each fail their own test |
| Red moved 5 cm right, or 5 cm toward/away from the arms | red needs +35 to +65 mm along y |
| Red moved 8 cm or 2 cm | outside even the 25-75 mm partial band |
| Blue turned a half turn (180 deg) or 45 deg | the folded axis angle is 0 or 45, under 65 |
| Blue turned but carried 4.5 cm away | centre must stay within 40 mm for any credit |
| A bar held in the air, standing on end, propped, stacked, or off the table | not lying flat on the table: 0 for that bar, 0 overall if it is green |

## Design numbers

- **Gripper.** At `FINGER_OPEN` (-0.04695 m per finger) the pads are 94 mm apart (arena `constants.py`); the
  usable grip is taken as 84 mm (1 cm margin). Each bar is gripped across its 25 mm width, 59 mm under that.
  The demo opens only to 45 mm on the way down (10 mm per side, `APPROACH_CLEARANCE`) to keep the finger travel,
  and with it the off-centre grip, small, and backs each pad off to 6 mm to let go.
- **Grip height.** The TCP grips 19 mm above the table, so the pad tips are 6 mm above it and the pads hold the
  top 19 mm of the 25 mm bar. The lowest arm link in the whole plan is 6.2 mm above the table.
- **Reach.** All bars at x = 0.30 m, y = +0.16 / 0 / -0.16 m. The left arm handles red (ends at y = 0.21), the
  right arm handles blue. Both grip with the fingers closing along x (left yaw -90 deg, right +90 deg); the right
  arm turns to yaw 0 over the bar, which turns blue clockwise seen from above. IK over the whole plan at 20 Hz:
  worst 0.090 mm / 0.013 deg (left) and 0.076 mm / 0.015 deg (right), every joint inside its limits with at
  least 0.47 rad to spare.
- **Spacing.** 8 cm of bare table between neighbours (at least 2 cm of finger clearance plus the 5 cm move = 7 cm
  was the minimum). Red moves away from green, so its gap grows to 13 cm; blue turned spans 8 cm along x and 2.5
  cm along y, so its gap to green grows too. The offline sweep of the arm's collision hulls finds the closest
  approach to the green bar is 98.5 mm (left arm) and 92.0 mm (right arm), and over 23 cm to the other arm's bar.
- **Changes from the original idea.** Bars lie along y, not along x, so the red bar moves along its own
  length: the grip offset is then along x and does not change the 5 cm. Spacing 16 cm centre to centre instead of
  12 cm (the original 12 cm left 4 cm between bars lying along y). Green became a multiplier (see above).
  Instruction says "move" rather than "slide" so a lift-and-place demo does not contradict it, and says which way
  "left" is. Tolerances widened as below.

### Why the tolerances are what they are

The original idea proposed 50 +/- 8 mm for full credit. The first GPU round measured 5 mm of wrist error at rest and
1 cm while moving, and a 12-20 mm off-centre grip along the fingers' closing direction. Against those:

- **Red slide (35-65 mm full, 25-75 mm partial).** The slide is measured along y and the grip closes along x, so
  the grip offset does not enter it. The error is the TCP's y error at the grip and at the release (both at
  rest): up to 5 + 5 = 10 mm. 50 +/- 8 would fail an exact open-loop plan on a bad run, so full credit is
  +/- 15 mm (10 mm of robot error plus 5 mm of margin). It still separates 5 cm from "a bit" (2 cm) and from
  "a lot" (8-10 cm).
- **Red drift along x (25 mm full, 40 mm partial).** This is where the grip offset lands: 15 mm modelled offset
  plus 5 mm wrist error = 20 mm, plus margin.
- **Red heading (15 deg full, 30 deg partial).** The original idea's 10 deg; widened to 15 because a parallel grip on a
  square bar can twist it a few degrees and the test models 5 deg of slip. A turned red bar is the "turned the
  wrong bar" shortcut, which still fails.
- **Blue turn (at least 80 deg full, 65 partial).** Close to the original idea's 90 +/- 8 / +/- 20. The wrist turns the
  bar; the test allows 5 deg of slip.
- **Blue centre (25 mm full, 40 mm partial).** The original idea's 10 mm cannot hold: closing the fingers pulls the bar
  up to 15 mm off its start along the closing direction, and turning the gripper 90 deg about the TCP swings that
  offset round instead of undoing it. Add 5 mm of wrist error along the bar at the grip and 5 mm in x and y at the
  release, and the worst case in the offline sweep is 22.5 mm (bound: sqrt(15^2 + 5^2) + sqrt(5^2 + 5^2) = 23 mm). 25 mm is under a third of
  the bar's 80 mm length, so the bar is still turned "where it lies".
- **Green (8 mm and 8 deg for x1, 20 mm and 20 deg for x0.5).** The original idea's 5 mm. Nothing touches green, so its
  only error is settling and solver drift; 8 mm leaves room for that while a bump by a finger (which moves a
  25 g bar centimetres) still falls out.

`test_the_demo_still_scores_full_with_an_off_centre_grip_and_wrist_error` checks every combination of +/-15 mm
grip offset, +/-5 mm wrist error in x and y at grip and release, and +/-5 deg of yaw slip: both bars keep full
credit.

## How it differs from the nearest tasks (novelty)

Nearest in popular task sets:

- **CALVIN `push_red_block_left` / `rotate_red_block_left` (and blue, pink).** Direction only, scored as a
  threshold (moved or turned past a fixed amount), one object per instruction. Here the instruction states the
  amount (5 cm, a quarter turn), overshoot is penalised as well as undershoot, and two different adjustments plus a
  hands-off constraint come in one sentence.
- **RoboDojo `general_pickup` ("Pick up the target by 10 cm").** One metric amount, vertical, one object, and
  the lift is not kept. Here the amounts are horizontal and rotational, the result must stay, and three bars are
  judged together.
- **RoboDojo `push_t` / ManiSkill `PushT-v1`.** Pose matching against a visible target pad. Here there is no
  target to aim at; the goal exists only relative to the bar's own start.
- **abc-sim `put_relative` ("to the right of the boxed food").** Relative to another object, with no metric
  amount.
- **CALVIN and RoboCasa reorientation tasks (`RotatePan`, `OrganizeMeasuringCups`).** A target orientation or
  a vague "rotate", not a stated angle from the current one.
- **Language Table**: blocks pushed to language goals such
  as board regions or other blocks, not by stated distances or angles.

Nearest on the task board (as of 2026-11-02, all "Planned, previous cohort"):

- **"Place the sponge that starts to the left of the cup in zone A; leave the other sponge and cup in place."**
  Has a leave-in-place constraint, but the goal is a marked zone. Here nothing is marked.
- **"Slide the crate onto the tape line"** and **"Push a partly filled bottle to a line without tipping it"**:
  sliding to a visible line, not by an amount.
- **"Use the sponge to clear the left marked patch, then the right patch, without moving the cup"**: a
  don't-disturb constraint around wiping, not metric placement.

What is new: a single instruction combining a metric translation, a metric rotation and a "leave it" constraint,
with no marks: a stated translation, a stated rotation and a requirement to leave another bar in place, each
scored with a two-sided tolerance.

## Assets and licences

None. Every object is a primitive box defined in `task.py`; the room and robot come from the arena.

## Verification

Run on an RTX 5090 with `yam-isaac-lab-arena`, using the commands above (`object_jitter=0`):

- Scripted demo (`demo()`): `task_score` 1.0, ended by success; 27 s wall clock including Isaac start-up.
- Dream-1: `task_score` 0.0 after the full 2,400-step run limit; 224 s wall clock.
- Offline tests: 49 passed.
