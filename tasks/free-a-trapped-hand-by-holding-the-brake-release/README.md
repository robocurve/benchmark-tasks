# Free a hand trapped by a stopped, braked robot arm

A small stopped robot arm has trapped a training hand against a fixture: the palm stands in a recess in the fixture
face, the arm's bar lies across it, and the forearm passes under the bar. The arm's joint is held by a pin brake (a
pin under a ring of teeth). Holding the black button down lowers the pin; let go, and a spring pushes it back into
the next gap, locking the arm again. One robot arm has to keep the button down while the other swings the stopped
arm away from the hand; then the hand can be lifted out and laid on the green mat.

The task is `task.py`, for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM embodiment for Isaac
Lab-Arena), on the `station` scene. It needs `parts.py` (in this folder), which adds the jointed robot arm, the
spring button and the multi-block hand to the official task-file API; `task.py` imports it from its own folder.
`near_misses.py` lists near-successes and wrong outcomes with the score each must get; they are plain-Python checks
of `score()` and `success()` and need no simulator.

## Run a trial

From this folder on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="The stopped robot arm has trapped the training hand against the fixture. Free the hand and lay it on the green mat. The black button on the arm's base releases its brake."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/trapped-hand-dream

# Scripted demo (reaches 1.0 in about 24 s)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/trapped-hand-demo
```

The run prints `task_score`. Scoring is `score()` and `success()` in `task.py`, described in words in `task.json`.

- **Runtime per trial:** at most 60 s of simulated time (1,200 steps at 20 Hz); about 1 to 2 minutes of wall time
  on an L40S once Isaac Sim has started.
- **Seeds:** none needed. Submissions run with `-E object_jitter=0`, so every object starts where `task.py` puts it.
  Jitter would move the hand by up to its value, and the hand has only 1 to 3 mm of room in the trap; run this task
  with `object_jitter=0`.

## How the simulator scores it

- The hand is on the mat when its lowest point is within 1 mm of the mat's top, its own vertical axis leans under
  10 degrees, the centre of its world-aligned bounding box is inside the mat's outline (inclusive, with a 1e-6 m
  margin), and it moves under 1 cm/s and turns under 0.5 rad/s.
- The hand is out of the recess when its world-aligned bounding box ends nearer the robot than x 0.330 m (the front
  of the recess side blocks), its lowest point is within 1 mm of the table top or the mat top, and it is still.
  This scores 0.5 whatever the arm's angle. Turning the arm without freeing the hand scores 0.
- Both are read from the state at the last step, not from the trial's history.
- The arm's angle is not read once the hand is on the mat, so swinging the arm back afterwards still scores 1.0.

## Scripted demo

The left arm comes down on the button with closed fingers and pushes 26 mm past first contact (the button bottoms
out flush with the base top after 20 mm), and holds it. The right arm grips the knob, its grip turned 52 degrees
about the vertical so the arc stays within its joint limits, and follows the knob's arc for 75 degrees toward the
robot, turning with it. The left arm then lets go; the pin springs back into a gap between the teeth. The right arm
lets go of the knob, grips the forearm 3.2 cm in front of the palm, lifts the hand 7 cm straight up out of the recess,
carries it to the mat and opens 7 mm above it, so the hand drops onto the mat after the fingers open.

Measured on the GPU (trace runs): the arm reached -74.8 degrees and stayed there; the pin went back to its rest
height after the button was let go. In a separate test plan (not part of the task), gripping the forearm and pulling,
lifting and dragging the hand without pressing the button, then pulling the knob along the arc, moved the hand under
3 mm and the arm 1.2 degrees (the pin's play), and scored 0.

## Checks without a simulator

```bash
python near_misses.py   # needs yam_arena importable
```

It prints the score of every case: the button held with the arm unmoved (0), arm turned 19 degrees or exactly 20
degrees with the hand still in the recess (0); with the arm at 50 degrees, the hand left in the recess (0), freed
onto the table (0.5), held above the mat (0) or on the floor (0); on the mat centred or turned 90 degrees (1.0), on
the mat with the arm swung back (1.0), 1 mm (1.0) and 1.5 mm (0) above the mat, still moving (0), leaning 15 degrees
in the air (0), on its side on the mat (0.5), its centre exactly on the mat edge (1.0) or 5 mm past it, with its far
end past x 33.0 (0). With the arm at 15 or 0 degrees: the hand freed and resting on the table (0.5), pulled 10 mm
with the palm still in the recess (0), the palm's back face exactly at the recess front (0) or 1 mm nearer the robot
(0.5), held 2 cm up (0), still sliding (0), tilted in the air over the mat (0), on the floor (0).
