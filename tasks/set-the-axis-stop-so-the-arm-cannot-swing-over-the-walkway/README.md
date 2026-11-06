# Fit the stop pin that lets a swinging arm reach its work pad but not the walkway

An orange arm turns on a vertical hinge over a stop plate with four sockets. The robot has to work out which socket,
holding the steel stop pin, lets the arm swing round onto the blue work pad but stops it before any part of it is
over the yellow walkway; fit the pin there; then swing the arm round onto the pad. The pin blocks the arm about
18 deg before its socket's angle, so the socket nearest the pad (70 deg, just past the pad's centre at 68 deg) stops
the arm at about 52 deg, short of the pad, and the right one is the 115 deg socket: although it lies inside the
walkway's angular band, it stops the arm at about 97 deg, before the walkway begins at 108 deg. The 160 deg socket
stops the arm at about 142 deg, after it has crossed the walkway; the 30 deg socket stops it at about 12 deg.

The task is `task.py` for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM embodiment for Isaac
Lab-Arena), on the `station` scene. It uses `parts.py` (in this folder; NumPy and `yam_arena` only) for the
multi-colour stop station and the arm's hinge, which Isaac Lab simulates as an articulation. `near_misses.py`
lists near-successes and wrong outcomes with the score each must get; they are plain-Python checks of `score()`
and `success()` and need no simulator.

## Run a trial

From this folder on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Put the steel stop pin in the socket that lets the orange arm swing round onto the blue work pad but stops it before it reaches the yellow walkway. Then swing the arm round onto the pad and leave it there."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/axis-stop-dream

# Scripted demo (reaches 1.0 in about 31 s)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 800 --store-frames --log-dir ~/yam-eval/axis-stop-demo
```

The run prints `task_score`: 1.0 if the success condition held for 3 s (`SUCCESS_HOLD_S`), otherwise `score()` at
the last step. Scoring is `score()` and `success()` in `task.py`, described in words in `task.json`.

- **Runtime per trial:** at most 60 s of simulated time (1,200 steps at 20 Hz); about 1 to 2 minutes of wall time
  on an L40S once Isaac Sim has started.
- **Seeds:** none needed. Submitted runs use `-E object_jitter=0`, so every object starts exactly where `task.py`
  puts it. With jitter on, seeded resets shift only the pin.

## Scoring details for the simulator

- The pin's foot is the centre of its bottom face, from its pose; it must lie inside a socket's 28 mm hole. Its
  lowest point must be between 3 mm below and 1 mm above the socket floor (the lower bound only allows for contact
  penetration). "Still" is under 1 cm/s and 0.5 rad/s.
- The arm's angle is the hinge position. The knob is on the pad when the knob centre, 170 mm from the pivot along
  the arm, is inside the pad's 60 mm square seen from above (arm angle 56.6 to 78.5 deg). "Not passed" a socket
  means the arm's angle is at most that socket's angle (1e-6 rad margin).
- The 3 s success hold (longer than the 1 s default) makes the demo trial end only after the pushing gripper has
  let go of the arm. The arm has no spring, so it stays where it is left; the credited state holds without the
  robot.

## Scripted demo

`demo()` uses the left arm only. It grips the pin from above near its top, carries it to the 115 deg socket and
lets it drop from 6 mm above the socket floor. The drop point is offset 5.8 mm in x and -4.4 mm in y to cancel the
arm's tracking error measured at this reach (without it the pin landed on a socket wall). It then comes down behind
the orange arm's trailing side with its fingers closed, 110 mm from the pivot, and pushes it round its arc,
commanded to 106 deg: in the GPU trace the pin stopped the arm at 100 deg while pressed (the pin leaned about
12 deg in its socket), short of the walkway at 108 deg. The gripper lifts over, comes down in front of the arm's
leading side 130 mm from the pivot, pushes it back to 66 deg and lets go; the arm rested at 62.4 deg, knob over the
pad, and the pin stood upright again.

## Checks without a simulator

```bash
python near_misses.py   # needs yam_arena importable
```

The cases include: the 115 deg socket with the knob on the pad, at both pad edges, leaning 15 deg, pushed against
a wall and with its foot exactly 1 mm up (1.0), and 1.5 mm up (0); the 115 deg socket with the arm unswung, short
of the pad, or left against the pin at 97 deg (0.5); the arm exactly at 115 deg (0.5) and at 115.01 deg (0); the
70 deg socket unswung, against the pin, exactly at 70 deg and at 70.01 deg (0); the 30 deg socket (0); the 160 deg
socket (0); the arm swung past the socket before the pin went in (0); the pin moving, leaning
25 deg, standing on the plate between sockets, lying across the arm's path, left on the table or knocked over (0).
