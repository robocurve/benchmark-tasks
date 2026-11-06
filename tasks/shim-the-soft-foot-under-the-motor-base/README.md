# Find the short foot of a rocking motor base and shim only that foot

A motor base stands on a steel baseplate. One of its four feet is short, so the base rests level on the other three
with a gap under that one, and it rocks if that corner is pressed. The robot has to find the short foot (by looking
low or by pressing corners) and slide the shim under that foot only. A shim under any other foot either cannot go
in or leaves the base tilted.

The task is `task.py` for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM embodiment for Isaac
Lab-Arena, run through Inspect Robots), on the `station` scene. It uses `parts.py` (in this folder) to build the
motor base and the shim, each as one rigid body made of several coloured shapes. `near_misses.py` lists
near-successes and wrong outcomes with the score each must get; it runs without a simulator.

## Run a trial

From this folder, on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Shim the motor base so it sits level and solid on all four feet, then let go."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 1200 --store-frames

# Scripted demo (reaches 1.0 after about 13 s)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 1200 --store-frames
```

The run prints `task_score`: 1.0 on success, otherwise 0.

- **Runtime per trial:** at most 60 s of simulated time (1,200 steps at 20 Hz), about 1-2 minutes of wall time on an
  L40S once Isaac Sim has started.
- **Seeds:** none needed. Submissions run with `-E object_jitter=0`, so every object starts exactly where `task.py`
  puts it. The layout is fixed, so the same foot (near-left) is short in every trial.

## Physics, as measured

The parts have even density, which puts the motor base's centre of mass 12 mm to the far-right side of the
diagonal through the near-right and far-left feet. With no robot motion, the base rests level on its three long
feet, and the gap under the near-left pad stays at 5.0 mm. Pressing the frame above the near-left foot
(`SHIM_DEMO=press`, commanded 25 mm into the frame) rocks it 3.2 degrees about that diagonal, and the far-right pad
lifts about 6 mm. On release it rocks straight back, level, on the same three feet.

## Scoring details

`success()` reads the two parts' poses. Each pad's coverage is the exact area of the pad that lies over the shim's
plate, seen from above (a polygon clip that allows for either part being turned). A pad covered by at least 75% stands
on the shim's top. A pad covered by less than 2% stands on the baseplate. A pad covered by anything in between
fails the trial (shim part-way under). A foot's gap is the height of its pad's underside centre above that surface.
It must be between -0.5 mm and 1.0 mm (a 1e-6 m margin applies to every limit). After the demo the gap is 0.8 mm
(10 mm foot - 5 mm short foot - 4.2 mm shim). Level means the frame's vertical within 0.5 degrees of straight up.
"In place" means the frame centre within 3 mm of its start and its turn within 2 degrees. "Still" means under
1 cm/s and 0.5 rad/s for both parts, and success must hold for 1 s. `score()` is 1.0 when `success()` holds,
otherwise 0.

The scorer cannot see the gripper. A shim lying where the demo leaves it is the same physical state whether or not
the fingers still touch its tab, because it rests on the baseplate and carries the foot either way. The demo lets go
before success is reached.

No record is kept over the whole trial. Every credited state is an end state that holds with the robot clear, so a
scene-reset rule is not needed.

## Scripted demo

`demo()` uses fixed waypoints only, worked out from the starting layout. The left arm grips the shim's tab from
above, fingers closing across it. It lifts the shim 12 mm and carries it sideways in front of the near-left foot, then
sets it down. It slides the shim straight in along the baseplate, commanded 1 mm low and 4 mm further than wanted,
because the soft arm lags against friction. Then it lets go and leaves. The line is aimed 10 mm to the left of the
foot's centre, because on the GPU the shim ended about 13 mm right of the commanded line. Measured: the plate centre
ended 1.3 mm right of the pad's centre and its far end 6 mm past the pad (full cover), with the base unmoved.

Failure replays (same command with the scripted policy and `SHIM_DEMO=...`). Each was run once as a trace on the GPU:
- `press`: press the near-left corner down and let go. The base rocks 3.2 degrees and back; score 0.
- `wrong`: the same slide aimed at the near-right foot, which already bears. The shim stops against the pad 1.5 mm
  short and cannot go under; the base does not move; score 0.
- `part`: the demo's slide stopped 20 mm short. The pad is 58% covered; score 0.
- `shift`: the right arm first shoves the frame's right side at mid-height, commanded 10 cm past it, then the demo
  runs. The base tips 12 degrees onto its left feet instead of sliding, drops back 1.3 mm from its start, and the shim
  then goes under the short foot as normal; score 1.0. A sideways shove cannot shift the base off its marks. A base
  moved more than 3 mm or turned more than 2 degrees is checked offline in `near_misses.py`.

The shim under the far-right foot (the short foot's diagonal partner) was not run on the GPU. It needs one arm
holding the near-left corner down while the other carries the shim round to the far side. `near_misses.py` checks
the state it leads to: the base turned about the near-right/far-left diagonal until the far-right pad sits on the
shim. No foot gap is then over 0.8 mm, but the base is tilted 2.6 degrees, so the score is 0.

## Checks without a simulator

```bash
python near_misses.py   # needs yam_arena importable; prints each case's score and the four foot gaps
```
