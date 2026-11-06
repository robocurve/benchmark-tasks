# Set up an out-of-distribution test scene away from the training outlines

A dark grey mat lies in front of the arms. Three pale painted discs on it are the "outlines" of where three objects
stood while a policy was trained, and each object (a red tin, a yellow jar, a blue tub) starts standing on its own
outline. The robot has to build a test scene that is out of distribution with respect to that layout:

> The pale outlines on the grey mat show where each object stood while a robot policy was trained. Set up a test
> scene unlike training: move every object so its centre is at least 10 cm from the centre of its own outline,
> standing upright and entirely on the mat, and leave every outline uncovered.

There is no target to aim at. The goal is a set of constraints: a minimum shift from each object's own outline,
three keep-out discs (all outlines, including the other objects'), a boundary (the mat's edges) and a pose
(upright). The robot must find free spots that satisfy all of them and place three objects there.

| File | What |
|---|---|
| `task.py` | the task file: objects, `success`, `score`, `distance`, scripted `demo` |
| `task.json` | the submission fields (`media` is added by the upload script) |
| `test_task.py` | offline checks (NumPy and pytest, no Isaac Sim) |

## Run a trial

From a checkout of the arena (yam-isaac-lab-arena, tested at `bd62771`) on the GPU VM, with this folder at
`tasks/ood-test-scene/`. Run with a fixed layout (`-E object_jitter=0`) so the start matches the setup
screenshot and the open-loop demo finds the objects. The scorer itself measures every shift from the fixed
outline centres, not from where an object spawned, so a jittered start would still be scored correctly. No seed
changes the scene.

Dream-1:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/ood-test-scene/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The pale outlines on the grey mat show where each object stood while a robot policy was trained. Set up a test scene unlike training: move every object so its centre is at least 10 cm from the centre of its own outline, standing upright and entirely on the mat, and leave every outline uncovered." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/ood-test-scene-dream
```

Scripted demo (23.5 s = 470 steps of waypoints per arm, ending with both arms parked beside the mat and a 2 s
still hold; the plan plus about 100 steps):

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/ood-test-scene/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The pale outlines on the grey mat show where each object stood while a robot policy was trained. Set up a test scene unlike training: move every object so its centre is at least 10 cm from the centre of its own outline, standing upright and entirely on the mat, and leave every outline uncovered." \
  --scorer task_score --max-steps 650 --store-frames --log-dir ~/yam-eval/ood-test-scene-demo
```

Videos, one MP4 per camera, in each log's `frames/` folder:

```bash
for log in ~/yam-eval/ood-test-scene-*/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score`: 1.0 when the trial ended in success, otherwise `score()` at the last step.

Runtime per trial: the run limit is 2,400 steps (2 minutes of simulated time at 20 Hz) for Dream-1 and 650 steps
(32.5 s) for the demo. Wall clock on an RTX 5090, including Isaac start-up: 47 s for the demo and 218 s for
Dream-1.

Offline tests (NumPy and pytest only; 36 tests, about 7 s):

```bash
YAM_ARENA_DIR=/path/to/yam-isaac-lab-arena python -m pytest tasks/ood-test-scene/test_task.py -q
```

## Scoring in one paragraph

Each object is worth 1/3. It earns credit only if it stands upright (axis within 10 deg of vertical, either flat
end down; all three are closed), is still, stands on the mat itself (lowest point within 4 mm of the mat top), has
its whole base on the mat, and covers no outline. Then it earns 1/3 at 10 cm or more from the centre of its own
outline, 1/6 at 5-10 cm, 0 under 5 cm. The score is the sum, and 0 if any object fell off the table. Success =
score 1.0 held for 1 s. Constants: `FULL_SHIFT`, `PART_SHIFT`, `UPRIGHT_DEG`, `ON_MAT`, `OUTLINE_BORDER` at the top
of `task.py`.

- **"Covering" an outline is footprint overlap seen from above**: object centre to outline centre under the
  object's radius plus the outline's radius. Each outline is 1 cm larger in radius than its object, so it shows as
  a 1 cm ring at the start and can be checked with a ruler.
- **The partial band still requires the object to be clear of its own outline.** The red tin overlaps its own
  outline until its centre is 6.4 cm away, so it earns 1/6 only from 6.4 cm to 10 cm (tested).
- **Many answers score 1.** In front of the outlines, behind them, beside them, or mixed (tested with four
  layouts). Only the end state counts; lifting and sliding score the same.

## Shortcuts that score 0 (all tested)

| Strategy | Why it fails |
|---|---|
| Do nothing, or nudge each object under 5 cm | shift under 5 cm (and still on its own outline) |
| Rotate the objects round onto each other's outlines | every shift is over 10 cm, but each base covers an outline |
| Push everything off the mat onto the table | not on the mat |
| Leave an object overhanging the mat edge | base not entirely on the mat |
| Knock an object over, or leave it tilted 25 deg | not upright |
| Stack one object on another, or keep holding it | not standing on the mat |
| Sweep an object off the table | the whole score is 0 |

## Design numbers

- **Gripper.** At `FINGER_OPEN` (-0.04695 m per finger) the pads are 94 mm apart (arena `constants.py`); the
  usable grip is taken as 84 mm (1 cm margin). The widest object is the 54 mm tin; the demo opens each pad only
  10 mm beyond the object on the way down (74 mm for the tin), backs off 6 mm to let go, and grips at yaw 0 (fingers
  closing along y). Heights over diameter: tin 1.02, tub 1.56, jar 1.48.
- **Grip height.** The TCP grips 40 / 48 / 55 mm above the bottom of the tin / jar / tub, so the pad tips are
  27-42 mm up the side, and the lowest arm link stays 30 mm or more above the mat in the whole plan.
- **Layout.** Outlines at (0.30, +0.11), (0.36, 0) and (0.30, -0.11) m, with at least 5 cm of mat between any
  two. The mat (x 0.12-0.46, y -0.22-0.22) leaves free room in front of and behind the outlines, but not
  at the sides: a 10 cm outward slide takes the tin or the tub off the mat (centre at |y| 0.21, radius 2.7 cm,
  edge at 0.22), and an inward slide runs them toward the yellow outline and each other's.
- **Demo.** The right arm takes the blue tub to (0.17, -0.14) while the left arm takes the red tin to
  (0.17, +0.14); then the left arm takes the yellow jar straight toward the arms to (0.22, 0). Each move: 1.5 s to
  the object, 1.5 s down, 0.8 s close, 1 s lift (bottom 3 cm over the mat), 2 s carry, 1 s lower to 3 mm, 0.6 s
  release, 0.8 s rise 3 cm straight up, 0.8 s up to that object's hover height. Every sideways move to or from an object
  is made with the open pad tips 2 cm above its top (TCP at TOP + 93 / 103 / 113 mm for tin / jar / tub). Places are aimed 6 mm toward +y against the
  measured -y grip lead. Both arms then park at (0.14, +-0.33), TCP 12 cm above the table, beside the mat, and hold
  still (left 2 s, right 12 s). The trial ends on success before the left arm reaches its park pose (see Verification).
- **Shifts and margins.** Planned shifts: red 13.3 cm, yellow 14.0 cm, blue 13.3 cm. The offline sweep of +-20 mm
  grip offset along y, +-5 mm wrist error in x and y at grip and release, and a 0-10 mm push toward +y on release
  keeps every object at full credit; the smallest shift is 12.0 cm (2 cm to spare). Each goal footprint is at least 2 cm
  inside the mat edges and at least 3 cm from every outline; the goals are at least 5 cm apart.
- **Reach and clearances.** IK over the whole plan at 20 Hz: worst 0.084 mm / 0.013 deg (left) and 0.080 mm /
  0.011 deg (right), every joint inside its limits (closest 0.08 rad, left arm over the jar's outline at x 0.36,
  the far end of the reach at that height). No arm link comes within 22 mm of any object other than the one it is
  moving, wherever that object is at the time, and the two arms' link bounding boxes stay more than 5 cm apart
  (offline hull sweep).
- **Measurement choices.** Shifts are measured centre to centre, so a tape measure settles them. Outlines are
  filled thin discs (a primitive cannot be a ring), a pale tint of each object's colour, 1 cm larger in radius so
  the ring shows. Objects are closed cylinders, so "upright" accepts either flat end down and the simulator's
  render matches the scorer.

## What makes this task distinctive

The goal has no target pose. The robot must build a scene that differs from a reference layout by a measured
amount: every object at least 10 cm from its own outline, clear of all three outlines (not only its own), entirely
on the mat and upright. Many layouts score 1, so the robot has to find free space that meets all four constraints
at once. Rotating the objects onto each other's outlines moves each one far enough and still scores 0.

## Assets and licences

None. Every object is a primitive defined in `task.py`; the room and robot come from the arena.

## Verification

Run on an RTX 5090 with `yam-isaac-lab-arena`, using the commands above (`object_jitter=0`):

- Scripted demo (`demo()`): `task_score` 1.0, ended by success at step 383; 47 s wall clock including Isaac start-up.
- Dream-1: `task_score` 0.0 after the full 2,400-step run limit; 218 s wall clock.
- Offline tests: 36 passed.
- Top camera: in `setup.png` all three objects and their outline rings are visible. The trial ends on success before the left
  arm parks, so in the demo's last frame the left arm covers part of the yellow jar and the red outline; the other
  objects and outlines stay visible.
