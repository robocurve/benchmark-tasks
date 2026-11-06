# Launch a pizza from the peel onto an oven stone that is out of reach

A small pizza sits on a wooden peel on a steel counter. Beyond the counter is an open-topped oven whose stone deck
starts 740 mm out, where neither arm can reach at deck height. The robot has to slide the peel in and then get it out
from under the pizza, so that the pizza stays on the stone. If the peel comes out at an ordinary pace, static friction
brings the pizza back out with it. A sharp pull, or a tilt of the blade, leaves the pizza behind. The task is
pass/fail.

The task is `task.py` for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM embodiment for Isaac
Lab-Arena), on the `station` scene. It uses `parts.py` (in this folder; NumPy and `yam_arena` only) to build the
multi-colour counter, oven, peel and pizza. `near_misses.py` lists near-successes and wrong outcomes with the score
each must get. They are plain-Python checks of `score()` and `success()` and need no simulator.

## Run a trial

From this folder on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Use the peel to put the pizza on the oven stone, then bring the peel back out of the oven."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/pizza-dream

# Scripted demo (reaches 1.0 in about 10 s)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 300 --store-frames --log-dir ~/yam-eval/pizza-demo
```

The run prints `task_score`. It is 1.0 if the success condition held for 1 s; otherwise it is `score()` at the
last step. Scoring is `score()` and `success()` in `task.py`, described in words in `task.json`.

- **Runtime per trial:** at most 30 s of simulated time (600 steps at 20 Hz), about 1 to 2 minutes of wall time on
  an L40S once Isaac Sim has started.
- **Seeds:** none needed. Submitted runs use `-E object_jitter=0`, so every object starts exactly where `task.py`
  puts it. With jitter on, seeded resets would shift the peel and the pizza separately, and could start the pizza
  off-centre on the blade.

## Scoring details for the simulator

- Pizza on the stone: its up axis within 5 deg of vertical; its body origin (the crust's bottom centre) at least
  45 mm (the crust radius) inside the stone's front, back and side edges; its lowest bounding-box point within 2 mm
  of the stone top (on the blade it is 4 mm up); and still (under 1 cm/s and 0.5 rad/s). The limits are compared
  with a 1e-6 m margin.
- Peel out: the peel's world bounding box ends at or before x 740 mm, the far edge of the mouth line, and its lowest
  point is no more than 5 mm below the counter top (on the counter or held; a peel knocked onto the table or stood
  on end fails). A tilted peel's box is larger than the peel, so this is never looser than the real peel.
- Whole-trial record: the pizza has touched the counter if, at any control step, part of the crust is over the
  counter (centre less than 45 mm beyond x 740) while its lowest point is less than 1.5 mm above the counter top. It
  has fallen if its lowest point is more than 5 mm below the counter top. Either one zeroes the score for the rest of
  the trial. A scene reset (an `on_reset` callback) clears the record. So does, offline, a call in which every loose
  object is back within 0.5 mm of its start and at rest after a jump of more than 20 mm since the previous call.
  The robot cannot cause either.
- Friction is set per object (static = dynamic): pizza 0.1, peel 0.4, counter 0.4, stone 1.0. PhysX averages the
  two values of a contact, giving the pair values in `task.json`. In a GPU trace, the pizza slipping on the peel
  during the sharp pull decelerated at about 2.0 to 2.3 m/s². That fits the 0.25 pair value (2.45 m/s²).

## Scripted demo

`demo()` uses the left arm only. It grips the handle from above, 50 mm from its end, with the pad tips 9 mm above
the counter. It slides the peel 170 mm into the oven over 3 s, waits 1 s, then pulls back 200 mm in 0.1 s
(commanded). The arm lags the command: the peel reached about 0.95 m/s and lifted briefly at its handle end, up to
about 16 deg. In the GPU trace (trace-20261106T014944; the submitted demo run scored the same), the pizza went in
with the peel (centre at x 827 mm). During the pull it slid 29 mm back with the blade and dropped onto the stone. It
came to rest flat at x 798.5 mm, 13.5 mm inside the stone's front edge, with the peel's tip back at x 700. Score
1.0.

Ordinary-pace check (a GPU trace, the same plan with the pull taking 1.5 s): the pizza stayed on the blade
and came back out of the oven with it (centre at x 634 mm). Score 0.

## Dream-1 run

The submitted Dream-1 run (600 steps) scored 0. From its contact sheet: the right arm reached across and gripped the
handle, then moved the peel part way toward the oven and back. The pizza stayed on the blade for the whole run and
never reached the stone. (The run log keeps actions, not object states, so this reading is from the frames.)

## Checks without a simulator

```bash
python near_misses.py   # needs yam_arena importable
```

The cases include: the pizza flat on the stone with the peel back on the counter, held in the air or tilted; the
pizza exactly at the front, back and side edges; tilted exactly 5 deg; its lowest point exactly 2 mm up; and the
peel's tip exactly at the mouth line, and the peel's lowest point exactly 5 mm below the counter top (all 1.0). They
also cover 1 mm past the back or side edge, 6 deg of tilt, upside down, on edge, sliding, 2.5 mm up, the peel 1 mm
past the line, left in the oven, 6 mm below the counter top, knocked onto the table or stood on end, the pizza still
on the peel in or out of the oven, the pizza half over the counter, dumped on the counter or dropped on the table,
and each of those then moved onto the stone (all 0).
