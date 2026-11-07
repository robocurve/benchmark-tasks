# Open a locker whose door a tipped case is leaning on, and take the case out upright

A small locker with a roof, its door hinged on the right and held by a magnetic catch. Inside, a thin case stands on
end, tipped forward past its base so that its top rests on the inside of the door. Once the door is pulled past
the catch, the case pushes it open and leans further with it; opened in one go, the case falls out. The ability
tested: hold the door partly open against the load and extract the case through the gap. The case holds a glass
instrument and is marked this way up, so tipping it past 45 degrees scores 0 even if it is laid down slowly. The
instruction only asks for the case to be taken out and stood on the mat.

The task is `task.py` for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM embodiment for Isaac
Lab-Arena), station scene. `parts.py` (a small parts kit) builds the locker and its hinged door; `tilt.py` lets the
scripted demo tilt the gripper. Both files must sit next to `task.py`. `near_misses.py` checks `score()` and
`success()` on near-successes and wrong outcomes without a simulator.

## Run a trial

From this folder on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Take the blue case out of the locker and stand it upright on the green mat."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/locker-dream

# Scripted demo (reaches 1.0 in about 21 s)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/locker-demo
```

- **Runtime per trial:** at most 60 s simulated (1,200 steps at 20 Hz), about a minute of wall time on an L40S once
  Isaac Sim has started.
- **Seeds:** none needed. Run with `-E object_jitter=0` (the case starts against the door; jitter would push it
  into the door or the walls). The layout is then the same every trial.

## How the scene is built

- **The tipped start.** The task-file API places loose objects upright only. A reset hook (`parts.on_reset`) sets
  the case's pose after every reset: tipped 12.5 degrees forward about the front edge of its base, its top 0.5 mm
  behind the shut door, so it settles onto the door in the first steps. The case is 25 mm thick and 160 mm tall,
  so it tips past its base at 8.9 degrees and is held up only by the door.
- **The door.** A hinge with damping 0.3 N m s/rad. An on-step hook makes the catch: within the first 0.05 rad the
  door is pulled shut with 0.045 N m; past that it turns freely. Traced on the GPU: with the door held at 0.92 rad
  the case leaned 34 degrees; opened past about 1.1 rad unheld, the case passes 45 degrees and falls out.
- **The tipping record** (case ever past 45 degrees) is cleared by the same reset hook, so nothing the robot does can erase it.

## Scripted demo

The right arm grips the handle (4 cm from the hinge), pulls the door past the catch and opens it slowly to
0.95 rad, holds it while the left arm takes the case, then swings it on to 1.5 rad, clear of the opening, and lets
go. The left arm uses `tilt.py`: with its fingers closing front to back across the case and its fingertips leaning
30 degrees away from the robot, it reaches down into the gap between the open door and the locker front and
pinches the case's top. When the door is clear it lifts the case, draws it out, turns the grip back so the case
hangs upright, and sets it on the mat from 5 mm up. The demo uses fixed waypoints only; the mat target is offset by
the 16 mm and 21 mm the case hangs from the pinch point, measured in a trace.

`tilt.py` replaces `yam_arena.demo.grip_rotation` at import, in the task's own process, so a waypoint can carry a
tilted gripper rotation. It changes only the scripted demo; Dream-1 and scoring do not use it.

## Checks without a simulator

```bash
python near_misses.py   # needs yam_arena importable
```

Cases include: standing on the mat (1.0), front face on the mat's edge (1.0), leaning 4 degrees on the mat (1.0),
overhanging the mat by 5 mm (0.5), standing on the table beside the mat (0.5), moving (0), held 1 cm up (0),
leaning 8 degrees (0), the starting lean (0), pushed upright inside the locker (0), a 44-degree lean (not tipping),
the door opened to 90 degrees and held (0), lying on its face (0), and stood on the mat after tipping (0).

## Habits that score 0

There is one case and one door, so there is no "nearest" or "leftmost" choice to guess. The habits that fail are
ways of opening: pulling the door open in one go, opening it slowly but all the way before reaching in, or opening
it with one arm and letting go (the case then pushes it open). Each lets the case lean past 45 degrees, which
scores 0 for the rest of the trial; `near_misses.py` has a case for each, and for tipping the case out onto the
mat. The layout is fixed (no seeds), so a memorised replay of the demo's waypoints cannot be ruled out.
