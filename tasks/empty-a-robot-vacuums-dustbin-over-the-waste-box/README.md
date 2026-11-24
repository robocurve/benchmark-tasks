# Empty a robot vacuum's dustbin: hold it over the waste box while the other hand holds its door release

A robot vacuum cleaner stands on the table. Its dustbin, a smoky grey box with a grip fin on its lid, sits in a
shallow seat on the robot's deck, full of dust balls. The bin empties through two trap doors in its bottom, which swing
open while the orange release button on its lid is held down and spring shut when it is let go. An open waste box
stands on the table in front of the robot.

The job: lift the bin off the robot and hold it over the waste box; hold its release button down so the dust falls
into the box, without spilling any outside it; then put the bin back in its seat with its doors shut, and take both
arms back to where they started. The task is one task file,
[`empty_a_robot_vacuums_dustbin_over_the_waste_box.py`](empty_a_robot_vacuums_dustbin_over_the_waste_box.py), for the
`yam_arena` embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from
the same folder.

**What it tests:**
- holding a container in place over a target while the other hand works its release;
- pouring granular material into a container without spilling;
- putting a part back in its seat;
- finishing with both hands free.

**Why two hands:** the doors are open only while the button is held down. The hand that holds the bin by its fin
cannot also press the button on its lid, and a bin set down on anything has its doors blocked shut. The one-hand
check scored 0: the right hand did its part as in the demo while the left never moved. The bin went over the box
and back to its seat with its doors shut and all ten dust balls still inside.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The robot vacuum (body, bumper, lidar turret and the
bin's seat) and the waste box are kinematic bodies made of shapes from `bodykit.py`. The bin, its two trap doors, its
button and the ten dust balls are rigid bodies.

- **The doors:** each door hangs on a hinge (a `Hinge` from `bodykit.py`) along one of the bin's long bottom edges,
  held shut by a spring (0.01 N m per degree). The button rides on a slider (a `Slider`) with a spring (300 N/m)
  holding it up. After every control step the task file checks how far the button is pushed down: past 1.5 mm, it
  moves both doors' spring targets to 90 degrees open, so they swing down; let go, it moves them back to shut.
- **The dust:** ten balls 13 mm across (2 g each) lying on the doors inside the bin.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena. It gives
`score()` each body's own pose and lets the task file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`. It gives 1.0 when `success()` has held for 1 s; `success()` means full score (which
includes both arms back where they started) with the bin still. Otherwise it gives `score()` at the end of the
episode: 0, 0.25, 0.5 or 1.0 (see `task.json`).

- `-E object_jitter=0` keeps every object exactly where the task file puts it, so every trial starts from the same
  layout.
- The cameras render at 1280 x 704 for sharp videos. Dream-1's client scales every frame to 320 x 176 before sending
  it, so `-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=empty_a_robot_vacuums_dustbin_over_the_waste_box.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Empty the robot vacuum's dustbin into the waste box: lift the bin off the robot by its grip fin and hold it over the waste box, then hold the orange release button on its lid down so its bottom doors open and the dust falls into the box, without spilling dust outside the box. Then put the bin back in its seat on the robot with its doors shut, and take both arms back to where they started." \
  --scorer task_score --max-steps 700 --store-frames --log-dir ~/yam-eval/vacuum-dream
```

The scripted demo reaches full score:
1. The right hand comes straight down on the bin's grip fin, fingers closing across it, lifts the bin out of its seat
   and carries it back over the waste box, holding it there.
2. The left hand comes down on the orange release button with its fingers closed, leaning outward, and holds it down:
   the trap doors swing open and the dust falls into the box.
3. The left lets go (the doors spring shut), rises and goes back to where it started.
4. The right carries the bin back to the robot, lowers it into its seat, lets go and goes back to where it started.

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=empty_a_robot_vacuums_dustbin_over_the_waste_box.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Empty the robot vacuum's dustbin into the waste box: lift the bin off the robot by its grip fin and hold it over the waste box, then hold the orange release button on its lid down so its bottom doors open and the dust falls into the box, without spilling dust outside the box. Then put the bin back in its seat on the robot with its doors shut, and take both arms back to where they started." \
  --scorer task_score --max-steps 700 --store-frames --log-dir ~/yam-eval/vacuum-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/vacuum-dream/*.json; do inspect-robots video "$log"; done
```

Set `BIN_DEBUG=1` to print what the score is built from twice a second: how far the button is pushed and whether the
doors are open, where the bin is and its tilt, how far the front door has swung, how many dust balls are in the waste
box, in the bin and spilled, and where both TCPs are.

## Runtime per trial

On one RTX 5090: about 3 minutes for a Dream-1 trial of 700 steps (181 s on the clock, with Dream answering in 0.79 s on average), and about 2 minutes for the scripted demo (422 steps, 21.1 s of simulated time, 121 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.
