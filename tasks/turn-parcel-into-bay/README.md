# Turn a long parcel in place between two neighbours, then push it lengthwise into a taped bay

A long cardboard parcel lies across the table between two other boxes, with only about 2 cm to spare at each end. It
has to end up lying flat and lengthwise in a loading bay taped out in red ahead of it, with its front end against the
bay's stop bar, and be left there, without bumping either neighbour. Two bollards at the mouth of the bay stand closer
together (37 cm) than the parcel is long (42 cm), so it cannot go forward lying across: it has to be turned a quarter
turn first, right where it lies. Pushed at one end, a long box swings about a point near its other end and slides
sideways into a neighbour; turned about its centre, its corners sweep a circle that just clears the neighbours and the
bollards. That takes both arms pushing the two ends in opposite directions at the same time; once turned, the parcel
slides between the bollards into the bay. The instruction only says where the parcel has to end up. The task is the
task file [`turn_parcel_into_bay.py`](turn_parcel_into_bay.py) for the `yam_arena` embodiment (scene, scoring and the
scripted demo), with [`bodykit.py`](bodykit.py) next to it.

## Requirements

[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) (tested at commit `0059d02`) on
Isaac Lab-Arena 0.3.1 / Isaac Sim 6.1, on Linux with an NVIDIA RTX GPU. Its `setup.sh`, or the program's Brev
launchable, installs everything, including the Dream-1 policy. No third-party assets are used: every object is a box
defined in the task file, so the task needs no models from Arena's object library.

`bodykit.py` is our own helper. Importing it hooks the `yam_arena` embodiment when the simulator starts, without
changing yam_arena, so that the task file can look at the arms after every control step (the scorer checks that the
parcel has been let go) and clear what it tracks at every reset. The parcel, the white boxes, the bollards, the stop
bar and the tape are plain `yam_arena` boxes.

What is modelled and what is not: the three cartons are rigid, uniformly dense blocks at their real size and weight
(no flex, no shifting contents); the bollards and the stop bar are fixed; the tape is a 0.5 mm raised strip. Loose
objects get yam_arena's rubber-like friction (1.2 static, 1.0 dynamic), more than cardboard on a real table.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` has held for 2 s in a row, otherwise `score()` at the end of the
trial (0.5 or 0). The boxes' poses are read from the simulation after every control step, and each arm's collision
meshes (links 2 to 5, the gripper and both fingers, as in the robot's URDF, covered with points no more than 1.5 cm
apart) are placed by forward kinematics from its measured joint positions.

- Two rules hold for the whole trial and are latched: if the long parcel's centre is ever more than 4 cm to either
  side of the centre line, or either white box ever moves more than 1 cm from its start, the trial scores 0. The task
  file keeps this state itself and clears it at every reset.
- 1.0: the parcel is let go (no part of either arm within 1 cm of it), lies flat on the table (each of the four
  corners of its underside within 3 mm of the table top), lies inside the bay (those four corners inside the red
  tape's inner edges, or at most 5 mm onto the tape), its long sides within 5 degrees of front to back, both corners of
  its front end within 1 cm of the stop bar, and it and both white boxes are at rest (moving slower than 1 cm/s and
  turning slower than 0.1 rad/s), all of this without a break for 2 s.
- 0.5: at the end of the trial the parcel is let go, lies flat on the table (as for 1.0) and is at rest, turned to
  within 10 degrees of front to back, but is not placed as for 1.0 (for instance it stopped more than 1 cm short of
  the stop bar).
- 0: anything else, including a parcel held or propped above the table.

`-E object_jitter=0` keeps every object exactly where the task file puts it, so every trial starts from the same
layout.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=turn_parcel_into_bay.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Get the long brown parcel into the red-taped bay, lying lengthwise with its long sides running front to back and its front end against the stop bar, and leave it there, without bumping the white boxes on either side of it." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/parcel-dream
```

The scripted demo, which reaches full score (both closed grippers push the parcel near its two ends in opposite
directions, in small steps along circles about its centre, with one regrasp halfway, so it turns a quarter turn in
place; then one gripper comes round behind it, pushes it straight forward between the bollards until it meets the
stop bar, backs off and rises clear, and the parcel stays against the bar):

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=turn_parcel_into_bay.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Get the long brown parcel into the red-taped bay, lying lengthwise with its long sides running front to back and its front end against the stop bar, and leave it there, without bumping the white boxes on either side of it." \
  --scorer task_score --max-steps 900 --store-frames --log-dir ~/yam-eval/parcel-demo
```

It succeeds after 705 steps. The parcel ends flush against the stop bar (0 mm at both front corners) with its
underside on the table; the 2 s count starts at step 666, once the gripper has backed off (while it still pressed the
parcel against the bar, nothing counted).

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/parcel-dream/*.json; do inspect-robots video "$log"; done
```

`TP_DEBUG=1` prints, every 10 steps (`TP_DEBUG_EVERY=5` for every 5), the parcel's position and angle, how far the
corners of its underside are above the table, whether it is inside the bay, the gaps between its front corners and
the stop bar, each arm's closest distance to it, whether it is let go and at rest, how far it has strayed sideways and
how far the white boxes have moved, whether the 1.0 conditions hold (and for how many steps), and what the first
version of this task's rule (which had no table, stop-bar or let-go test) would have given.

Runs that check the scorer (each with `TP_DEBUG=1` as well):

- `TP_ONE=1` runs the demo the wrong way, with one arm: the left arm stays back and the right gripper pushes alone
  near the parcel's right end. The parcel swings instead of turning in place: it turns only about 49 degrees while its
  centre slides 11 cm back and 5 cm to the left, into the left white box. Its centre passes 4 cm to the side at about
  step 365 and the white box has moved more than 1 cm by step 381 (11 mm): the run scores 0.
- `TP_HOVER=1` holds the parcel motionless (made a `bodykit` body with gravity off) in its finished place, turned,
  2 mm from the stop bar, but with its underside 1 cm above the table; the arms stay still. It scores 0 (the first
  version's rule gave it 1.0 after 2 s).
- `TP_SHORT=1` starts the parcel turned and at rest on the table in the bay, its front end 3.5 cm short of the stop
  bar; the arms stay still. It scores 0.5 (the first version's rule gave it 1.0).
- `TP_NO_LET_GO=1` runs the demo, but the gripper keeps pushing the parcel against the stop bar to the end instead of
  backing off. It scores 0 (the first version's rule gave it 1.0 at step 633).

`TP_PUSH_AT`, `TP_PUSH_IN`, `TP_STEPS`, `TP_STEP_S`, `TP_OVERSHOOT`, `TP_FWD_LEAD` and `TP_FWD_S` tune the scripted
demo; their defaults reach full score.

## Runtime per trial

On one RTX PRO 6000 (Blackwell), shared with other jobs at the time: about 10 minutes for a Dream-1 trial of 2,400
steps (605 s on the clock, with Dream answering in 0.56 s on average), and about 5 minutes for the scripted demo while
rendering 1280 x 720 video (705 steps, 35 s of simulated time; 320 s). Isaac Sim's first start on a new machine adds a
few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed and the scripted demo is deterministic.

## Building it for real

A 42 x 14 x 10 cm shipping carton weighted to 800 g, two 20 x 14 x 10 cm cartons weighted to 1.2 kg, two 3 x 3 cm
posts 8 cm tall screwed to the table, a 22 cm long bar (2 x 4 cm section) screwed down as the stop, and red floor tape
for the bay. Mark the three cartons' start positions with tape so they can be reset quickly. Scoring by hand: a 3 mm
card must not slide under any corner of the parcel; a ruler at both front corners gives the gaps to the stop bar; a
protractor against the tape gives the parcel's angle; the overhead video shows whether either arm still touches the
parcel at the end, whether its centre strayed more than 4 cm to the side, and whether a white box moved from its
marks by more than 1 cm.
