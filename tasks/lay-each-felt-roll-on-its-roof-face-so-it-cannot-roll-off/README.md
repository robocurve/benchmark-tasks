# Lay a roll of roofing felt on each of two roof faces so that neither can roll off

A model roof has two faces pitched 22 degrees, falling in directions 90 degrees apart: the left face falls toward the
robot, the right face toward the robot's right. Two felt rolls lie on the table with their axes left to right. A roll
laid along a face's fall line stays where it is put; laid across the slope it rolls off the low edge. The robot has to
read each face's slope and turn each roll to match: the right roll can go on as it lies, the left roll needs a quarter
turn.

The task is `task.py` for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM embodiment for Isaac
Lab-Arena, run through Inspect Robots), on the `station` scene. The roof and rolls are built with `parts.py`, a small
parts kit included in this folder (multi-shape parts with hinges and sliders, built as Isaac Lab articulations; it needs
only NumPy and `yam_arena`). `near_misses.py` lists near-successes and wrong outcomes with the score each must get; they
are plain-Python checks of `score()` and `success()` and need no simulator.

## Run a trial

From this folder on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Put one roll of roofing felt on each roof face, lying on its side, so that it stays where you leave it."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/felt-dream

# Scripted demo (reaches 1.0 in about 16 s)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/felt-demo
```

The run prints `task_score`. Scoring is `score()` and `success()` in `task.py`, described in words in `task.json`.

- **Runtime per trial:** at most 60 s of simulated time (1,200 steps at 20 Hz); about 1-2 minutes of wall time on an
  L40S once Isaac Sim has started.
- **Seeds:** none needed. Run with `-E object_jitter=0`: jitter would push the rolls into their battens. With it,
  every object starts exactly where `task.py` puts it.

## How it is modelled

- The parts kit cannot tilt a fixed shape, so each roof board is a hinge link on the fixed roof part, hinged along its
  high edge and written level. During the embodiment's settle steps before the first observation it swings down under
  gravity onto its upper joint stop at 22 degrees and rests there (0.384 rad in every trace). A face counts only while
  it is within 1 degree of that stop.
- Each roll is one loose rigid part: a cylinder plus a square core whose four edges stand 1 mm proud of the round
  surface (the roll's lap edges). In the traces a roll laid along a fall line stayed within about 1 cm of where it was
  put, and a roll laid across the 22 degree slope rolled off.

## How the simulator scores it

For each roll and face, in the face's own frame: the roll's four outline corners (each end of its axis, plus and minus
its radius across) must lie within the board's 20 x 16 cm outline; its axis must lie within 5 degrees of the board's
plane and within 20 degrees of the fall line; and the gap between roll and board (centre height minus radius) must be
at most 3 mm. The roll must be at rest (under 1 cm/s and 0.5 rad/s). A module-level record notes every roll that has
been on a face (outline within the board and touching it, in any direction, so a roll laid across that rolls off is
recorded too), and spoils a roll for the rest of the trial once its lowest point then comes within 2 mm of the table
top (or below it, off the table). The record is cleared only by a scene reset: both rolls within 0.5 mm of their start
and at rest after a jump of more than 20 mm in one control step (or at the first step). Pass/fail: 1.0 only when both faces hold an
unspoiled roll at rest, otherwise 0; success must hold for 3 s (`SUCCESS_HOLD_S = 3.0`). All limits are compared with a 1e-6 margin.

The simulator cannot see the gripper. A roll held still on a face along its fall line would count, but that is also
the state the roll keeps once released. A roll held across the slope does not count (the 20 degree rule).

## Scripted demo

`demo()`: each arm grips its roll across the middle from above, lifts it, carries it over its face's middle, lowers it
level until its lowest point is 5 mm above the board, holds still for 1.5 s (the arm lags its target, and a roll
released while moving keeps rolling), opens and lifts away. The left arm turns its roll a quarter turn on the way so
that it points down the left face. In the GPU trace the left roll settled on its face and the right roll drifted 8 mm
sideways and stopped against a ridge; both stayed, and the trial ended at 1.0. A probe of the habit (the left roll laid
on as it lies, across the slope; `HABIT = True` in `task.py`) rolled off the left face onto the table; with pass/fail
scoring that run scores 0 (it recorded 0.5 under the earlier per-face scoring).

## Checks without a simulator

```bash
python near_misses.py   # needs yam_arena importable; prints each case's score
```

Each single case starts from a scene reset. The task is pass/fail. The cases cover success; rolls swapped between faces (1.0); one face only (0); both rolls on
one face (0); a roll held across the slope (0); 19.9 and 20.1 degrees off the fall line; a roll's low end exactly
at and 1 mm past the low edge; its side exactly at and 1 mm past a side edge; 3 mm and 4 mm above the board; a moving
roll; a roll stood on its end; a face pushed up to 15 degrees; and a whole-trial chain in which a roll rolls off onto
the table and is put back (0), a scene reset, and a roll laid across that rolls off and is then laid correctly (still 0).
