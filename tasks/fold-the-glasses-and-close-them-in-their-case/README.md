# Fold the glasses' temples and close the glasses in their hard case

A pair of glasses stands open on the table, front toward the robot, and a fixed hard glasses case stands to the
robot's left with its lid open. The robot is asked to put the glasses away in the case and close it. Open, the glasses
are 146 mm deep and the case is 50 mm inside, so they only go in with both temples folded. The instruction does not
mention folding. The temples fold one behind the other (the right temple's hinge is nearer the front).

The task is `task.py` for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM embodiment for Isaac
Lab-Arena), on the `station` scene. It needs `parts.py` (in this folder) for the hinged temples and lid; it imports
only NumPy and `yam_arena`. `near_misses.py` lists near-successes, wrong outcomes and fixed habits with the score each
must get; they are plain-Python checks of `score()` and `success()` and need no simulator.

## Run a trial

From this folder on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Put the glasses away in their case and close it."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/glasses-dream

# Scripted demo (reaches 1.0 in about 37 s)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 800 --store-frames --log-dir ~/yam-eval/glasses-demo
```

The run prints `task_score`. Scoring is `score()` and `success()` in `task.py`, described in words in `task.json`.

- **Runtime per trial:** at most 60 s of simulated time (1,200 steps at 20 Hz), about 1 to 2 minutes of wall time on
  an L40S once Isaac Sim has started.
- **Seeds:** none needed. Run with `-E object_jitter=0`: every object starts exactly where `task.py` puts it. Jitter
  has not been tested with this task.

## Scoring details the simulator applies

- "Still" is `ObjectState.resting()` on the glasses' front: under 1 cm/s and 0.5 rad/s.
- **Released:** a `parts.on_step` callback reads both arms' joint angles each control step and computes each
  gripper's tip point (between the finger tips, from the arm's own kinematics) and half opening. The glasses count as
  released when every tip point is more than 5 mm above the glasses' top, or farther from their bounding box sideways
  than its half opening plus 10 mm (a finger's thickness) plus 5 mm. Offline (near misses) there are no grippers.
- **Folded:** each temple hinge at least 75 deg (fully folded is 90 deg; open is 0, and it cannot open further).
- **In the case:** the glasses' world-aligned bounding box (every part, links included) lies within the case's inner
  walls in x and y, and its lowest point is within 1 mm of the case floor. Any orientation counts.
- **Lid shut:** the lid hinge within 3 deg of closed.
- **0.25:** folded, released and still, not in the case, the front's up axis within 20 deg of vertical, and the
  lowest point within 1 mm of the table top, or of the case floor with the bounding box's centre over the case.
- **Knocked over (whole trial):** a module-level record is set, and the trial scores 0 from then on, if the front's
  up axis is ever more than 45 deg from vertical while the glasses' lowest point is within 5 mm of the table top and
  their centre is not over the case. Tipped onto their lenses, the free temples can fall shut by gravity, so this stops
  credit for folding by knocking the glasses over. The record is cleared only by a scene reset: the glasses back within
  0.5 mm of their start and at rest, after a jump of over 20 mm in one control step.
- The trial ends with `success` once the 1.0 condition has held for 1 s; otherwise `task_score` is `score()` at the
  last step.

## Hinges

`task.py` registers a `parts.on_step` callback. Each temple's drive target follows the temple, so it turns freely
(damping 0.01 N m s/rad), until it is folded to within 8 deg; a 0.02 N m/rad detent then holds it folded until it is
pushed back open past 20 deg. The lid's drive (0.06 N m/rad) holds it upright until it is closed past 45 deg, then
pulls it shut, like a spring hinge. A scene reset clears both (`parts.on_reset`). Moving links do not collide with
their own part's body in this simulator, so the joint limits are the stops: the temples stop at open and at folded,
and the lid at upright and at shut.

## The scripted demo

The demo uses only fixed waypoints. The right arm's closed fingers push the right temple through its arc, keeping
the fingers' side along it, until it folds behind the front, and then park. The left arm then folds the left temple
the same way, grips the front over the left lens, lifts the glasses over the case, lowers them upright onto its floor
and lets go 4 mm above it. It then reaches behind the open lid and pulls it past halfway, and the spring hinge shuts
it.

## Licences

No third-party assets: every object is built from boxes in `task.py`.
