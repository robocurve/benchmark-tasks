# Tilt a long board with both arms to roll a ball into a cup

A long oak board lies flat in a fixed guide slot in front of both arms. A 10 cm foam ball rests near its right end,
between two low rails, and a fixed open cup stands just past that end. The robot has to grip the board with both
grippers (one in each open gap of the slot), lift it clear of the table, and then make the right end lower than the
rest while keeping it above the cup's rim, so the ball rolls off into the cup. The ball is wider than an open
gripper and its centre is below the rim while the board lies on the table, so lifting and tilting the board is the
only way in. Doing that takes both arms.

| File | What |
|---|---|
| `task.py` | the task file: objects, `success`, `distance`, `score` (`SCORE_OVER = "end"`), `demo` (both arms), and `one_arm_demo` (a check, not a hook) |
| `task.json` | the submission metadata (`media` is added by the upload script) |
| `test_task.py` | offline checks, NumPy and pytest only |

## Run a trial

On the VM with `yam-isaac-lab-arena` set up (its Quickstart), copy this folder to `~/two-arm-tilt-roll`. Always run
with `-E object_jitter=0` (see Seeds).

```bash
export INSTR="Hold the board with both grippers, one in each open gap of the guide slot, lift it clear of the table and tilt it so the ball rolls off the right end of the board into the cup."
```

Scripted demo (full score expected):

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$HOME/two-arm-tilt-roll/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 600 --store-frames --log-dir ~/yam-eval/tilt-roll-demo
```

Dream-1:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$HOME/two-arm-tilt-roll/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 1200 --store-frames --log-dir ~/yam-eval/tilt-roll-dream
```

One-arm check: play `one_arm_demo` from a scratch copy kept outside this folder.

```bash
mkdir -p ~/scratch && sed 's/^def demo():/def two_arm_demo():/; s/^def one_arm_demo():/def demo():/' $HOME/two-arm-tilt-roll/task.py > ~/scratch/tilt_roll_one_arm.py
```

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$HOME/scratch/tilt_roll_one_arm.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 600 --store-frames --log-dir ~/yam-eval/tilt-roll-one-arm
```

Videos (one MP4 per camera, in each log's `frames/` folder):

```bash
for log in ~/yam-eval/tilt-roll-*/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score: ...` at the end: 1.0 when the episode ends in success, otherwise `score()` of the
final state. Put that number in `task.json` `scores`, e.g. `"Dream-1 run: 0.3; demo: 1.0"`.

Offline checks (here or on the VM):

```bash
YAM_ARENA=~/yam-isaac-lab-arena python -m pytest -q test_task.py
```

`YAM_ARENA` is only needed when the `yam_arena` package is not installed. All 43 checks pass against
yam-isaac-lab-arena at commit bd62771 (about 6 s).

## Runtime per trial

- Demo: 22.0 s of plan (440 steps at 20 Hz). The ball should drop during the tilt (plan time 16-19 s), so the
  episode should end with success around step 400-440. If it does not, `yam_scripted` holds 3 s past the plan and
  stops (about 500 steps plus the usual 60-70); `--max-steps 600` covers that.
- Dream-1: up to 1,200 steps (1 minute of simulated time) unless it succeeds earlier.
- Wall clock on an RTX 5090, Isaac start-up included: 48 s for the demo, 126 s for the Dream-1 run.

## Seeds

`-E object_jitter=0` is required. A seeded reset shifts each loose object by up to 3 cm in x and y on its own. That
would push the 8 cm board into the slot's walls (5 mm on each side) and put the ball on a rail or off the board. No
seed is needed with jitter 0; the layout is fixed.

## Assets and licences

None. Every object is a primitive defined in `task.py` (boxes and one sphere); no meshes, textures or library
assets are used.

## Why one arm cannot do it

The API allows one primitive per loose object, so a tray with rims and a cup that move together cannot be built.
Here the board is a single box, and the fixed slot, rails and cup do the work of rims and handles. Every one-arm
route considered, and what stops it:

| One-arm route | What stops it | How it is checked |
|---|---|---|
| Pick up the ball and drop it in the cup | The ball is 100 mm across and the open gripper is 94 mm | geometry, test |
| Push or roll the ball into the cup along the flat board or the table | The ball's centre (6.6 cm above the table on the board, 5 cm on the table) is below the 7.5 cm rim, so a push meets the wall above the ball's equator and cannot lift it | geometry, test |
| Grip the board anywhere between the two gaps or between the rails | The slot leaves 5 mm beside the board and the rails 12 mm. The finger is 10 mm thick just above its pad and 24 mm thick 4-6 cm above the TCP (URDF collision meshes), so it cannot reach the board's side there. A gripper cannot close around board and wall together (11.4 or 12 cm outside, 9.4 cm open) | geometry from the meshes, test |
| Left arm alone lifts the left end | The board pivots on its right end, which stays on the table. The ball rolls off at table height with its centre at most 6.7 cm up, below the rim, and it cannot rise | geometry, test (lifts of 6-30 cm) |
| Right arm alone lifts its gap | The board pivots on its left end, so the board's right part rises past the grip and the ball rolls back against the gripper, away from the cup | geometry, test (lifts of 2-20 cm) |
| One arm holds the whole board clear of the table by one gap and tilts it | The board's weight puts 1.74 N m (right gap) or 3.45 N m (left gap) about the fingers' closing axis. Two pads can resist at most about 1.22 N m by friction (2 pads x static friction 1.2 x the 50 N gripper force limit x 10.2 mm, the corner radius of a 15.6 x 13 mm pad patch), so the board turns in the grip until an end is back on the table | statics, test; **verify on Brev** (item 8) |

Both arms remove all of these: each holds one end of the board's weight, so neither grip has to resist a moment, and
the height difference between the two hands sets the tilt. The last row is the only one that rests on simulated
physics rather than geometry, and it was not run on its own; the recorded one-arm check (see Verification)
covers the right-arm-alone row.

## Novelty

Nearest tasks on the board and in the existing sets, and how this one differs:

- **Board: "Carry an open tray level"** (carry a tray of loose parts with both arms, tilt within 15 deg of level) and
  **"Two-arm tray carry"** (one handle per gripper, carry a tray with a cup, set it down). Both reward keeping the
  carried object level. Here the tilt is the goal: the height difference between the hands is the control input,
  its sign has to change mid-task (left end low while lifting so the ball parks against the right hand, then left end
  high so it rolls out), and its size is bounded (the board's right end must stay above the rim).
- **embodied-first-exam abc-sim/ball_tray** (both grippers on a tray's handles, keep a pushed ball on the tray):
  a balancing task that keeps the ball on. Here the ball has to leave, at a chosen moment, through one outlet.
- **Board: "Intercept a rolling item"** (catch an item rolling down a chute) and **ManiSkill RollBall** (push a ball
  to a goal): the ball rolls, but the robot does not shape the slope that moves it.
- **RoboCasa TiltPan / ShakePan**, **RoboTwin dump_bin_bigbin**, **RoboDojo pour_balls_into_vase**: one gripper
  tilts or pours from a container it holds. Here no single grip can hold the board up (see above), so the tilt has
  to come from two hands at different heights, the way two people tip a long board or trough.

## Design notes

- **Gripper.** Fully open, the pads are 94 mm apart (`FINGER_OPEN = -0.04695` m per finger in `constants.py`). With at
  least 1 cm of margin, nothing gripped may exceed 84 mm. The board is gripped across its 80 mm width with the grip
  turned 90 deg (`GRIP_YAW`), so the fingers close along x, across the board. The TCP rides level with the board's
  top face: the pad tips are 3 mm above the table at the grip, with 13 mm of pad on the board's side.
- **Layout.** The board runs along y at x 0.28 m from y +0.400 to -0.448 m. The left gripper holds it at y +0.365,
  beyond the slot; the right gripper holds it at y -0.25, in the 12.8 cm gap between the slot and the rails, in front
  of the right arm's base. Each gripper is 6.8 cm long along the board (URDF meshes) and has 3 cm of clearance to the
  slot or rail ends. The ball starts 10.8 cm right of the right grip, clear of the fingers, with 4 cm of board left
  past it. The cup's near wall is 1 cm past the board's end.
- **Demo (both arms, same clock).** Both arms turn straight down 20 cm above the table (a direct move from the ready
  pose swept a finger within 1 mm of the slot's end), descend over their gaps, grip, and lift together for 6 s:
  the left end to 11.6 cm, the right grip to 12.1 cm (top-face heights). The left end is 5 mm low, so the right part
  slopes up toward the cup and the ball rolls about 2.5 cm back against the right gripper's fingers. After a 1.5 s
  pause the left arm raises its end by 2 cm in 3 s while the right arm holds. The right part then slopes down 2.4 %
  (1.4 deg) toward the cup, with the board's end 10.0 cm above the table at its lowest, 2.5 cm above the rim. The ball
  rolls about 11 cm and off the end, and both arms hold still for 5 s.
- **Heights and walls.** The cup's rim (7.5 cm) is above the ball's centre on the flat board (6.6 cm). While the board
  is held, its top face stays at least 1.5 cm below the tops of the slot (16 cm) and the rails (14 cm) at their ends,
  so the board stays in the slot and the ball between the rails. Offline, every arm link's collision mesh stays at
  least 2.8 cm from every fixed box along the whole demo, and the joint limits leave at least 0.10 rad
  (joint4 at the 20 cm approach).
- **Scoring.** `SCORE_OVER = "end"`: the final state decides, so a ball that rode the lifted board and was then
  dropped scores 0, as stated. The 0.3-0.6 band needs the whole board at least 1 cm clear of the table with the ball
  still on its top face. Neither one-arm pivot reaches it, and "untouched" scores 0. Progress is the ball's
  horizontal approach to the cup's centre from its start (17.8 cm away), so a ball parked against the right hand
  (farther away) gets the 0.3 floor, and one at the board's end about 0.44.
- **Camera.** Projected through the calibrated top camera into the 320 x 176 frames Dream-1 receives, the ball's top
  at the start is at about (232, 93) and the cup's centre at (258, 103), both unobstructed by the slot, rails or cup
  walls (ray test). The two grips are at about (89, 103) and (205, 103). In the setup frame the right wrist covers
  the right side of the ball. The episode ends on success while the arms still hold the board, so in the recorded
  demo's last frames the right arm covers the cup and the ball in it; the scorer's 1.0 is the record of the ball
  resting in the cup.

## Verification

Run on an RTX 5090 with `yam-isaac-lab-arena`, using the commands above (`object_jitter=0`):

- Scripted demo (`demo()`): `task_score` 1.0, ended by success; 48 s wall clock including Isaac start-up.
- Dream-1: `task_score` 0.0 after the full 1,200-step run limit; 126 s wall clock.
- One-arm check (`one_arm_demo`, right arm alone): `task_score` 0. The board pivoted on its left end, which stayed
  on the table, and the ball rolled back against the right gripper and stayed there; it never reached the cup.
- Offline tests: 43 passed.
