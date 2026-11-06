# Let a carton down a parcel slide onto the dock without letting it run

A brown carton stands on a model trailer bed at the top of a 30 degree steel parcel slide. The slide runs down to a
green dock plate, and two white cartons stand just past the dock's far end. A carton pushed over the top and let go
slides down by itself, runs across the dock and knocks into the white cartons. To score, the robot has to bring the
carton down under control (held back from its downhill side, or met at the bottom) and leave it standing on the dock.
The instruction gives only the goal.

The task is `task.py` for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM embodiment for Isaac
Lab-Arena, run through Inspect Robots), on the `station` scene. The bed, slide and dock are built with `parts.py`, a
small parts kit included in this folder (multi-shape parts with hinges and sliders, built as Isaac Lab articulations; it
needs only NumPy and `yam_arena`). `near_misses.py` lists near-successes and wrong outcomes with the score each must
get; they are plain-Python checks of `score()` and `success()` and need no simulator.

## Run a trial

From this folder on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Bring the brown carton down the slide onto the green dock."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/slide-dream

# Scripted demo (reaches 1.0 in about 26 s)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/slide-demo
```

The run prints `task_score`. Scoring is `score()` and `success()` in `task.py`, described in words in `task.json`.

- **Runtime per trial:** at most 60 s of simulated time (1,200 steps at 20 Hz); about 1-3 minutes of wall time on an
  L40S once Isaac Sim has started.
- **Seeds:** none needed. Run with `-E object_jitter=0`, as for all submissions; every object then starts exactly
  where `task.py` puts it.

## How it is modelled

- The parts kit cannot tilt a fixed shape, so the slide is a hinge link on the fixed bed-and-dock part, hinged along
  its high edge at the bed and written level. During the embodiment's settle steps it swings down under gravity onto
  its upper joint stop at 30 degrees (0.5236 rad in every trace). The dock plate runs 1 cm under the slide's foot (a
  link does not collide with its own part), so the slide's top meets the dock top with no step.
- The slide, bed top, dock plate and brown carton have friction 0.2. On the flat bed the carton stays put; on the
  slide it slides by itself.
- Probes of letting go (`RELEASE = "top"`, `"mid"` or `"foot"` in `task.py`; GPU traces at friction 0.2; "largest
  step" is the carton centre's largest move between consecutive control steps, 1/20 s apart, as the scorer measures it):
  - Let go at the top (pushed over the edge, nothing below it): largest step 49.2 mm; it ran across the dock into the
    white cartons and moved one 14.5 mm. Scores 0 (step limit and white cartons).
  - Let go mid-slide (carried part way down, then the arm lifts away): largest step 30.2 mm; it came to rest upright on
    the dock, moving a white carton only 1.7 mm. Scores 0 on the step limit only. Without that limit this run would
    score 1.0: position alone does not separate a run that happens to stop on the dock from a controlled descent.
  - Let go just above the foot: largest step 15.7 mm; it stopped straddling the foot, front on the dock and back on the
    slide. Scores 0 (not wholly on the dock).
- Scripted demo (GPU trace): largest step 10.9 mm, white cartons moved 0.0 mm, score 1.0. The largest step is when the
  carton tips over the bed edge into the waiting fingers.
- Not traced: friction 0.4 (bare steel). The scripted push did not get the carton off the bed at 0.4.

## How the simulator scores it

`_carton_on_dock()`: the carton's up axis within 10 degrees of vertical; its lowest point within 1 mm of the dock top;
its four base corners (from its pose and true size) between the slide's foot and the dock's far end and within 10 cm of
the dock's centre line front to back; and at rest (under 1 cm/s and 0.5 rad/s). A module-level record keeps, over the
whole trial, each white carton's largest displacement from its start, the largest move of the carton's centre
between consecutive control steps (moves over 10 cm in one step are scene edits and are skipped, as is the reset jump) and whether the brown carton's lowest point ever
came within 2 mm of the table top (it fell off the slide or the dock). The score is 0 once a white carton has moved more
than 3 mm, the carton's centre has moved more than 20 mm in one 1/20 s control step (0.4 m/s, `MAX_STEP`) or the brown carton has touched the table, even if things are put back. The record is cleared only by a scene
reset: every loose object within 0.5 mm of its start and at rest after a jump of more than 20 mm in one control step (or
at the first step). Pass/fail; success must hold for 2 s (`SUCCESS_HOLD_S = 2.0`). All limits are compared with a 1e-6
margin.

The simulator cannot see the gripper. The carton is credited only when it stands on the dock plate, which is also the
state it keeps once let go.

## Scripted demo

`demo()` uses fixed waypoints only. The left arm's closed fingers push the carton along the bed until it tips over the
edge onto the slide, then back off. The right arm's closed fingers wait low on the slide a few centimetres below where
the carton lands; the carton slides into them, and the right arm walks it down at about 5 cm/s. At the foot the carton
stops with its front on the dock and its back on the slide (the same happens to a carton lowered slowly in every
trace), so the right arm goes round its side, comes in behind it and pushes it over the foot onto the dock, then backs
off and lifts away. In the GPU trace the carton ended upright with its uphill face 1.6 cm past the slide's foot and the
white cartons unmoved, and the trial ended at 1.0.

## Checks without a simulator

```bash
python near_misses.py   # needs yam_arena importable; prints each case's score
```

Each single case starts from a scene reset. The cases cover success; doing nothing; the carton resting on the slide;
its uphill face 1 mm onto the dock (1.0); straddling the foot with its uphill edge on the slide, tilted 5 degrees and at
30 degrees (0); its downhill face exactly at and 1 mm past the dock's end; its
side exactly at and 1 mm past the dock's side; 1 mm and 2 mm above the dock; still moving; tilted 9.9 and 10.1
degrees; tipped onto its side; a white carton moved 3.1 mm (0) and exactly 3 mm (1.0); white cartons pushed out of the
way first (0); and whole-trial chains: the carton runs into a white carton that is then put back (0), a scene reset,
the carton knocked off the slide onto the table and then lifted onto the dock (0), and the carton run off the dock's
end onto the table and pushed back (0), the carton coming to rest on the dock after a
20.1 mm step (0) and after a 19.9 mm step (1.0).
