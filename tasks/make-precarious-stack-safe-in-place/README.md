# Square up a leaning stack of cartons in place

Three cartons are stacked so that each one is shifted toward the robot from the carton below it; the top carton
overhangs the bottom one by 48 mm and the stack is close to toppling. The robot has to square the stack, so that no
carton overhangs the one below by more than 5 mm, without lifting any carton off the stack and without knocking it
over.

The task is one file, `task.py`, for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM
embodiment for Isaac Lab-Arena), on the `station` scene. `near_misses.py` lists near-successes and wrong outcomes
with the score each must get; they are plain-Python checks of `score()` and `success()` and need no simulator.

## Run a trial

From this folder on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="This stack could fall. Square it up by pushing, without taking it apart: in the end no carton may overhang the carton below it by more than 5 mm. Do not lift any carton off the one below it, and do not move the bottom carton more than 2 cm."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/stack-dream

# Scripted demo (reaches 1.0 in about 11 s)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/stack-demo
```

The run prints `task_score`. Scoring is `score()` and `success()` in `task.py`, described in words in `task.json`.
Lifting a carton, restacking a fallen one, and moving the bottom carton more than 20 mm are judged over the whole
trial: `task.py` keeps that record in a module-level variable and clears it only on a scene reset (every carton back
within 0.5 mm of its start after a jump of more than 20 mm in one step). A lift is measured as each carton's gap
above the carton below it, from true corners, so the stack rocking as a whole is not a lift. Overhang is measured from each carton's
true corners and turn, not from its bounding box.

- **Runtime per trial:** at most 60 s of simulated time (1,200 steps at 20 Hz); about 1 to 2 minutes of wall time
  on an L40S once Isaac Sim has started.
- **Seeds:** none needed. Run with `-E object_jitter=0`, as submissions are: the layout is then the same every
  trial. The task needs `object_jitter=0`: jitter shifts each carton independently by up to 3 cm, which changes the
  overhangs and can exceed the stack's 7 mm tipping margin.

## Scripted demo

`demo()` drives the left arm with its closed fingers pointing down. The gripper's front edge is not flat: it reaches
14 to 20 mm ahead of the tool point at the finger tips but 34 mm ahead 50 to 85 mm higher up, and the wrist sticks
out about 90 mm further up still. So the demo pushes once, slowly, away from the robot, with the tool point 40 mm
above the table: the 34 mm band then bears on the top of the middle carton and the bottom of the top carton, and
carries both to the same line, flush with the bottom carton, while the tips stay short of the bottom carton. The
stack rocks about 5 degrees before the middle carton slides and the arm stops about 30 mm short of its target
(measured in traces), so the demo aims 30 mm further; the cartons settle about 3 mm short of flush.

## Checks without a simulator

```bash
python near_misses.py   # needs yam_arena importable; prints each case's score
```

The cases include both joints squared (1.0), 4.5 mm off either way (1.0), 6 mm off (0.5), only one joint squared
(0.5), a top carton turned 5 degrees (corners 3.3 mm over, 1.0) or 10 degrees (6.3 mm over, 0.5), tilted 5 degrees,
held 1.4 mm above the stack, still sliding, knocked off or the whole stack toppled (0), the stack shoved 25 mm (0),
every limit at exactly its value (passes), the stack rocked 8 degrees as one and settling back (1.0), and taking it
apart: a carton lifted 10 mm or 30 mm and set back, a fallen carton restacked, the bottom carton dragged 25 mm and
pushed back, or the start rebuilt by hand, all still score 0 for that trial.
