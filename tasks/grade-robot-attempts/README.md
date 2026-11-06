# Grade three robot attempts and flag the failures

Three grading bays stand side by side across the table. Each bay is a grey card with a white cross. An orange
cube on each card shows where an earlier robot attempt to put the cube on the cross ended. The robot is the
judge. It puts a green token on the yellow flag pad in front of each bay whose attempt succeeded and a red token
in front of each one that failed, and it must leave the cubes where they are. Grading is hands off, and it uses
only the end state, as a success detector or a human episode reviewer would.

Instruction: *Each of the three bays shows how a robot's attempt to put the orange cube on the white cross ended.
An attempt succeeded if its cube covers the centre of the cross. Put a green token on the yellow pad in front of
every bay whose attempt succeeded and a red token on the pad of every bay whose attempt failed. Don't move the
cubes. A wrong flag is worse than none.*

The true verdicts are: bay 1 (left) **failed**, bay 2 (middle) **succeeded**, bay 3 (right) **failed**.

Files: `task.py` (the task file: objects, `success`, `distance`, `score`, `demo`), `task.json`,
`test_task.py` (offline checks).

## What is on the table

Positions are in metres. x runs forward from the arm bases, y runs left from the midline, and the table top is
z = 0.7475.

- **Bay cards (fixed):** each is 16 x 16 cm and 3 mm thick, with cross centres at x 0.40 and y +0.20 / 0 / -0.20.
  Each card is tiled from seven flush slabs: a white strip 2 cm wide along x, two white half-strips along y,
  and four grey 7 x 7 cm corners.
- **Cubes (loose, orange, 50 mm, 75 g):** bay 1's cube stands on the near-left grey corner, 4.5 cm off the cross
  centre along both axes (6.4 cm in all), with 1 cm of grey between it and each arm of the cross. Bay 2's cube
  is on the cross centre, turned 30 deg. Bay 3's cube is on the far-right grey corner, 4.5 cm off along both
  axes.
- **Flag pads (fixed, yellow):** each is 10 x 10 cm and 3 mm thick, centred at x 0.25 in front of each bay. Each
  pad ends 2 cm short of its card.
- **Tokens (loose):** six wooden cylinders, 40 mm across and 40 mm tall (30 g), in a row at x 0.13. From left to
  right they are green, red, green, red, green, red, at y +0.27, +0.16, +0.05, -0.05, -0.16 and -0.27.

## Run and score a trial

Built against the YAM Isaac Lab-Arena embodiment at commit `bd62771`. Run these from the root of the task
repository (the folder that holds `tasks/grade-robot-attempts/`), on the GPU machine with `inspect-robots`
installed.

Dream-1 (the run limit is 2,400 steps, 2 minutes at 20 Hz):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/grade-robot-attempts/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Each of the three bays shows how a robot's attempt to put the orange cube on the white cross ended. An attempt succeeded if its cube covers the centre of the cross. Put a green token on the yellow pad in front of every bay whose attempt succeeded and a red token on the pad of every bay whose attempt failed. Don't move the cubes. A wrong flag is worse than none." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/grade-attempts-dream
```

Scripted demo (20.5 s of waypoints including a 2 s final hold, 410 steps at 20 Hz; success ends it during the
hold):

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/grade-robot-attempts/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Each of the three bays shows how a robot's attempt to put the orange cube on the white cross ended. An attempt succeeded if its cube covers the centre of the cross. Put a green token on the yellow pad in front of every bay whose attempt succeeded and a red token on the pad of every bay whose attempt failed. Don't move the cubes. A wrong flag is worse than none." \
  --scorer task_score --max-steps 520 --store-frames --log-dir ~/yam-eval/grade-attempts-demo
```

At the end the run prints `task_score`. It is 1.0 when the trial ends in success; otherwise it is `score()` at
the last step (`SCORE_OVER = "end"`).

- **Runtime per trial:** on an RTX 5090, 52 s wall clock for the demo and 265 s for Dream-1, including Isaac
  start-up. Simulated time is at most 120 s for Dream-1 and about 17 s for the demo.
- **Seeds:** run with `-E object_jitter=0`, so the start layout is fixed and the seed does not change it. Do not
  use the default jitter (0.03 m), for two reasons. A 3 cm shift could move the success cube off the cross
  centre, which changes the true verdict. It could also push a failed cube onto the cross. The verdicts are
  fixed by the layout. A variant with a different mix needs a one-line change to `CUBE_PLACEMENT` in `task.py`,
  and `VERDICT` follows from it automatically.
- **Assets:** none. Every object is a `Box` or `Cylinder` primitive, so there is nothing to attribute.

## Scoring

- A pad's flag is the set of token colours **resting** on it. A token counts when its centre is over the pad and
  its bottom is within 1 cm of the pad top, standing or lying.
- Per bay: **+1/3** for the right colour only (one or more tokens), **-2/3** for the wrong colour (alone or
  together with the right one), and **0** for an empty pad. The total is floored at 0, so the possible scores
  are 0, 1/3, 2/3 and 1.
- **0 whenever any cube has been moved:** its centre shifted more than 1 cm (horizontally or vertically), or its
  top face tipped more than 10 deg. A cube turned in place still counts as untouched.
- Success (1.0) means all three pads carry the right colour only and no cube has moved, held for 1 s.

Why a wrong flag costs twice what a right one earns: a mislabelled attempt corrupts a training set, but an
unlabelled one is only lost. With one success and two failures, every constant strategy therefore scores 0.
Flagging every bay red earns 2/3 and loses 2/3. Abstaining is cheap: two right flags and an empty pad score 2/3.
The instruction says "a wrong flag is worse than none", so this rule is not hidden from the robot.

Shortcuts that score 0, all checked in the tests:
- every bay flagged green;
- every bay flagged red;
- both colours on every pad;
- the opposite verdicts;
- the token nearest each pad;
- "repairing" a failed attempt by moving its cube onto the cross and then flagging green.

Random colours average 0.125 (one colouring in eight is the truth), and only the true verdicts reach 1.0.

## Offline tests

```bash
YAM_ARENA_DIR=~/yam-isaac-lab-arena python -m pytest -q tasks/grade-robot-attempts/test_task.py
```

`YAM_ARENA_DIR` (or `YAM_ARENA_PATH`) points at a clone of the embodiment repo. The tests need only numpy and
pytest. All 52 pass. They check:
- the file loads, and the verdicts are 1 red, 2 green, 3 red;
- both arms' demo plans solve IK at 20 Hz, within 0.5 mm and 0.1 deg and inside the joint limits;
- the demo fits the run limit and ends with a 2 s still hold;
- the demo puts the verdict colour on every pad;
- the TCPs stay more than 18 cm apart and never go past the pads;
- no arm link comes within 4 cm of a cube at any point of the played plan (URDF collision hulls);
- the open fingers clear neighbouring tokens, rise 3 cm straight up after each release, and keep their tips off
  the table;
- neither the ready pose (first frame) nor the parked pose (last frame) hides a bay card, cube or pad from the
  top camera, checked by projecting the arm hulls along camera rays. The tokens are also visible in the first
  frame;
- each failed cube appears at least 10 px from its cross centre in the 320 x 176 top view, the success cube
  within 1 px, and the cross arms about 4 px wide;
- every partial-credit branch, including a goal state with an 18 mm grip offset plus a 10 mm release shift;
- the construct-validity shortcuts above, the cube-moved gate (moved 11 mm, lifted, tipped 15 deg, knocked off
  the table) and its tolerance (8.5 mm, or turned in place);
- no start overlaps, flush and complete bay cards, loose objects on a support, tokens that fit the gripper
  with 1 cm to spare, and every `task.json` field present.

## Design numbers

- **Gripper:** the YAM pads open to **94 mm** (`FINGER_OPEN = -0.04695` m per finger), so with a 1 cm margin
  anything up to 84 mm can be grasped. Tokens are 40 mm across. The open fingers' outer faces are 57 mm from
  the grip centre, and the tokens are 10-11 cm apart along y, so the open fingers clear each neighbour by at
  least 2.3 cm.
- **Grip:** the TCP grips 16 mm below a token's top, so the pads hold its upper 29 mm and the tips stay 11 mm
  above the table. A token is let go with its bottom 6 mm above the pad, aimed 6 mm toward +y. The pad is
  10 cm wide, so with the measured 18-19 mm grip offset and a 10 mm release shift, the token centre is still
  2 cm inside the pad edge.
- **Camera:** the failure offsets (6.4 cm) are about 12 px in the 320 x 176 top view. That is well above the
  2 cm visibility floor measured on the GPU. Each failed cube stands wholly on grey, so its whole white cross
  is visible, while the success cube hides the cross centre. Colours do the rest: orange cubes, white cross,
  grey card, yellow pads, green and red tokens.
- **Demo:** the left arm moves the red token at y +0.16 to bay 1's pad and then the green token at y +0.05 to
  bay 2's pad. At the same time, the right arm moves the red token at y -0.27 to bay 3's pad. Neither TCP goes
  past x 0.25, and the nearest cube edge is at x 0.33. Both arms then park outboard (TCP at x 0.20, y +-0.42,
  12 cm up, fingers closed) and hold still for 2 s.

## What makes this task distinctive

The robot does no manipulation job of its own here: it grades three finished attempts at one goal from their end
state alone, hands off, and records each verdict with a token, as a success detector or an episode reviewer labels
robot data. The successful attempt's cube is turned 30 deg on the cross, so a neat-looking placement is not the
test; only whether the cube covers the cross centre is. A wrong flag costs twice what a right one earns, so every
constant strategy scores 0, and moving any cube, including "repairing" a failed attempt, zeroes the score.

## Verification

Run on an RTX 5090 with `yam-isaac-lab-arena`, using the commands above (`object_jitter=0`):

- Scripted demo (`demo()`): `task_score` 1.0, ended by success at step 339; 52 s wall clock including Isaac start-up.
- Dream-1: `task_score` 0.0 after the full 2,400-step run limit; 265 s wall clock.
- Offline tests: 52 passed.
- Top camera: in `setup.png` all three bays, cubes, pads and tokens are visible, and the turned success cube covers its cross
  centre. The trial ends on success about 1 s after the last release, before the arms park, so in the demo's last
  frame the left arm covers part of bays 1 and 2; all three pads and their tokens stay visible.
