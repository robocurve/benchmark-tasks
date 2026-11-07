# Carry the microscope by its arm, not by its eyepiece

A bench teaching microscope stands upright and side-on, its arm toward the robot's left. Its eyepiece is a loose
tube sitting in a collar on top of the head, held only by gravity. As on a real upright microscope, the head and
eyepiece are over the middle of the base and near the centre of mass, and the arm rises at the back; so a grasp at
the object's centre from above, at its highest point, or at the centre of its outline from above all land on the
eyepiece. The robot is asked to put the microscope on a white mark on a shelf 35 mm high. Lifted by the eyepiece,
the tube slides up out of the collar. Lifted by the arm (or the base, or the head below the collar), the microscope
comes up in one piece. The instruction does not say how to hold it.

The task is `task.py` for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM embodiment for Isaac
Lab-Arena), on the `station` scene. It needs `parts.py` (in this folder) for the sliding eyepiece. `parts.py`
imports only NumPy and `yam_arena`. `near_misses.py` lists near-successes and wrong outcomes with the score each must
get. They are plain-Python checks of `score()` and `success()` and need no simulator.

## Run a trial

From this folder on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Put the microscope on the white mark on the shelf."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/microscope-dream

# Scripted demo (reaches 1.0 in about 11 s)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/microscope-demo
```

The run prints `task_score`. Scoring is `score()` and `success()` in `task.py`, described in words in `task.json`.

- **Runtime per trial:** at most 60 s of simulated time (1,200 steps at 20 Hz), about 1 to 2 minutes of wall time
  on an L40S once Isaac Sim has started.
- **Seeds:** none needed. Run with `-E object_jitter=0`: the microscope starts exactly where `task.py` puts it.

## Scoring details the simulator applies

- "Still" means moving under 1 cm/s and turning under 0.5 rad/s (`ObjectState.resting()`).
- **Eyepiece:** the slider joint's position (m) above fully seated. Over 3 mm at any step sets the fault record; at
  the end it must be at most 1 mm.
- **Tilt:** the angle between the microscope's up axis and vertical. Over 15 degrees at any step sets the fault
  record; at the end it must be under 3 degrees.
- **Fall:** the microscope body's downward speed over 0.6 m/s at any step sets the fault record.
- **On the mark:** the lowest of the base's four bottom corners lies between 2 mm below and 1 mm above the mark's
  top, the tilt is at most 3 degrees, and the base's centre (from the body pose) is within 20 mm of the mark's
  centre, measured flat.
- **Let go:** a `parts.on_step` callback reads both arms' finger and gripper link poses every step and measures the
  shortest distance from their collision hulls (yam_arena.analysis) to every shape of the microscope. Success needs
  more than 5 mm. A microscope set on the mark and still held scores 0. Offline (`near_misses.py`) there is no
  robot, so this check counts as clear.
- **Fault record (whole trial):** once set, the score is 0 for the rest of the trial, even if the eyepiece is pushed
  back in. It is cleared only by a scene reset: the microscope back within 0.5 mm of its start and at rest, after a
  jump of over 20 mm in one control step. A robot cannot move it back that way.
- **Release height:** the demo opens the fingers with the base about 5 mm above the mark (commanded 15 mm; the
  loaded arm sags), so it lands after the release.

## Scripted demo

`demo()` drives the left arm only. It comes straight down on the arm with the fingers open 40 mm, closing along the
microscope's front-to-back line, one finger on the arm's front face and one on its back, the tips 92 mm up (6 mm above
the head bracket). Gripped that way, the off-centre weight turns the microscope about an axis across the fingers'
faces, which the two pads resist; gripped across the arm's width instead, it swung 64 degrees in the grip in an
earlier trace. It lifts the base 45 mm, carries it over the mark, lowers it, opens, goes straight up clear of the
arm and then back. It plays fixed waypoints and reads nothing from the simulator.

`DEMO_BY_EYEPIECE = True` is a counter-test only: the arm grips the eyepiece 4 mm above the collar (where a centre or
highest-point grasp lands) and lifts. It is never used for the submitted demo.

## Measured in simulation

- **Scripted demo** (trace-20261107T113530): the eyepiece stayed seated (0.0 mm) and the microscope tilted at most
  1.6 degrees in the grip. Held, its base stopped about 5 mm above the mark; released, it dropped flat onto the
  mark, the fingers cleared it, and the run scored 1.0.
- **Eyepiece counter-test** (trace-20261107T113804): as the arm rose, the eyepiece slid up out of its collar to the
  32 mm stop, and only then did the microscope come off the table, hanging from the tube about 8 mm up. The trial
  scored 0.

## Other ways to succeed, and what scores 0

- **Also full score:** gripping the arm, the base, or the head below the collar, as long as the eyepiece stays in;
  two-arm carries; any heading on the mark.
- **Score 0:**
  - lifting by the eyepiece (it leaves the collar at once), even if it is pushed back in later;
  - tipping the microscope over 15 degrees, or dropping it from over about 18 mm;
  - pushing it along the table (the shelf is 35 mm high);
  - grasping it at its centre or highest point from above, which takes the eyepiece;
  - setting it on the shelf off the mark, holding it just above the mark, or still holding it on the mark.

The cases in `near_misses.py` cover the 1 mm, 20 mm, 3 degree, 15 degree, 3 mm and 1 mm eyepiece limits on both
sides, a turned microscope, a tilt on the mark's edge, a fall and a fast landing, a pushed-back eyepiece, a
centre-of-object grasp and a highest-point grasp (both take the eyepiece), and scene resets.

## Recorded runs

- Scripted demo: 1.0 (reached at about 11 s).
- Dream-1, 1,200 steps: 0.0. The arms moved toward the microscope and the shelf but did not lift it (judged from the
  five-frame contact sheet; the run logs no object states).
