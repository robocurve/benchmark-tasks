# Seat an organizer tray in a drawer and close it, past a pen hidden at the back

A plastic drawer unit stands on the table with its one drawer pulled about halfway out. An organizer tray lies beside
it. The instruction asks only for the tray to go in fully seated and the drawer to be closed. A black marker pen lies
on the drawer's floor near its back, under the unit's top, where none of the cameras can see it while the drawer is
half shut. Put straight in, the tray lands on the pen, rides up at the back, and the drawer won't close past it. The
robot has to pull the drawer out far enough to see the back, or notice the tray isn't sitting flat and look. Then it
takes the pen out (or tucks it anywhere it isn't under the tray), seats the tray flat and closes the drawer. The
pen's place along the back of the drawer changes from trial to trial.

The task is `task.py` for the `yam_arena` embodiment of yam-isaac-lab-arena (station scene; Isaac Lab Arena 0.3.1,
Isaac Sim 6.1, Isaac Lab 3.0, inspect-robots 0.60.0). The folder also has:
- `labkit.py`, a kit for multi-shape parts with sliding and hinged joints (the drawer is a slide on the unit),
  finger-pad contact patches, and `on_step` / `on_reset` callbacks. Its reset hook passes on the trial's seed and,
  when a hook moves objects, renders again before the first observation.
- `arena_assets_fix.py`, imported by `labkit.py`. It points Arena's object library at NVIDIA's production bucket.
  This task uses no library assets.
- `near_misses.py`, plain-Python checks of `score()`, `success()`, the pen's placement and the clearances that need
  no simulator (`python near_misses.py`).

The helper files patch the stock `yam_arena` package in the task's own process only. Nothing is changed on disk.

## Run a trial

From this folder, on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Put the organizer in the drawer, fully seated, and close the drawer."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --grader none --max-steps 2400 --seed 1 \
  --store-frames --log-dir ~/yam-eval/drawer-organizer-dream

# Scripted demo
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --grader none --max-steps 2400 --seed 1 \
  --store-frames --log-dir ~/yam-eval/drawer-organizer-demo
```

- **Run limit:** 2,400 control steps (2 minutes of simulated time at 20 Hz; `-E control_hz=20` is needed, since the
  embodiment's default is 10 Hz). The scripted demo ends early on success at 35.5 s.
- **Runtime per trial:** at most 2 minutes of simulated time. The scripted demo's run (35.5 s of simulated time, ended early on
  success) took about 5 minutes of wall time on an L40S at 640 x 360, including Isaac Sim's start-up.
- **Seeds:** the pen's place along the back of the drawer (y -0.060, 0 or +0.060) comes from the trial's seed.
  `task.py` prints it at every reset (`[drawer] seed=... pen at y=...`), and `pen_spot(seed)` gives it offline. The
  same seed always gives the same place. With inspect-robots' `--seed 1`, `2` and `5` the pen lies at +0.060, 0 and
  -0.060. `-E object_jitter=0` keeps everything else exactly where `task.py` puts it. For a real trial, roll a die
  for the pen's place.

## Objects

Coordinates are from the robot: x away from the arm bases, y to the left, heights above the table.

| object | what it is | where |
|---|---|---|
| drawer unit | light grey polypropylene, 300 wide x 228 deep x 87 tall; opening 54 mm above the drawer floor | front face at x 0.377, y -0.150 to +0.150 |
| drawer | inside 200 deep x 260 wide, 50 mm walls, 3 mm thick; front 12 x 266 x 78 | slides out 205 mm to a stop, starts out 90 mm |
| handle | 10 mm round bar pull, 100 mm long, 30 mm proud of the front | its axis 66 mm up |
| pen | black whiteboard marker, 14 mm x 130 mm, 15 g, 35 mm pocket clip on its cap | on the drawer floor along y, rolled onto its clip, axis 56 mm in front of the back wall, y -0.060, 0 or +0.060 |
| tray | white polypropylene organizer, 180 x 240 x 45, 100 g, 3 mm walls and floor | on the table, centred at (0.250, +0.262), 240 mm side along y |

The tray has a full-height divider down its middle along its 180 mm length and two cross dividers 31 mm tall. It
fits the drawer with 10 mm to spare all round. Seated, it stands 45 mm tall, under the opening's 54 mm. On the pen
it stands 59 mm, so the drawer can't close over it. The drawer slides with 4 N of friction, so it stays where it is
left. Pulled out to its stop, all of its inside is clear of the unit's top and the pen shows.

For a real trial: any plastic desktop drawer unit and organizer tray with these inside sizes, and a whiteboard marker.

## How the simulator scores it

- **In:** the tray's world-aligned bounding box lies inside the drawer's walls (3 mm allowance), at the drawer's
  current position, and its underside is below the walls' tops.
- **Flat:** the tray's underside is within 2 mm of the drawer's floor and it is tilted under 1.5 deg.
- **Closed:** the drawer is within 5 mm of fully shut.
- **Score:** 1.0 in, flat and closed; 0.5 in and flat, the drawer not closed; 0.25 in but not flat (on the pen, or
  tipped); 0 otherwise. The pen may end up anywhere, in the tray included. `task_score` takes the score at the last
  step. A full score held for 2 s, with the tray at rest, ends a trial early.
- In a real trial: a photo of the tray in the drawer (flat: a 2 mm feeler gauge won't go under any corner) and of the
  drawer shut (a 5 mm gauge won't go between the front and the unit).

## Scripted demo

`demo()` reads what a camera would see: the drawer's position and, once the drawer is out, the pen's position.
1. The left hand takes the handle and pulls the drawer out to its stop. That brings the back of the drawer, and the
   pen, out from under the unit's top. It then moves clear.
2. The right hand picks the pen up where it is seen, lays it on the table to the right and moves clear.
3. The left hand lifts the tray by its middle divider, carries it level, high enough to clear the open drawer's
   front, and lowers it into the drawer. It aims by the tray's seen place in its fingers, so a tray that has slipped
   in the grip still goes in square. It lets go 16 mm above the floor, so the pads stay above the drawer's walls.
4. The left hand takes the handle again and pushes the drawer shut.

With `--seed 1`, `2` and `5` (the pen at each of its three places) the demo scores 1.0 and ends early on success at
35.5 s.

**Dream-1** (`--seed 1`, the Dream-1 command above) scores 0. It picks the tray up from the table and holds it beside
the drawer for the full 2,400 steps, but never puts it in. It doesn't open the drawer further or reach for the pen.

**Shortcut check.** `DRAWER_CHECK=leave_pen` runs the demo but leaves the pen in the drawer. The tray lands on the pen and
stays tipped (about 6 deg, its underside 9 mm up), and the drawer stops 180 mm out when pushed: it scores 0.25.

## Limits

- Rigid primitives at true size: the drawer is a damped slide with friction and a stop, the pen a plain cylinder
  with a box for its clip, and the tray rigid. Nothing flexes, and a real polypropylene tray would give a little.
- The unit, drawer and tray are built to the sizes above rather than modelled on one product.
- Third-party assets: none; every shape is a primitive.
