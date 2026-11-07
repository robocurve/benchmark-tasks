# Hold a large gear turned so its teeth line up with one hand while the other pushes a small gear into mesh

On an aluminium gearbox plate, a large brass spur gear is fitted on one steel shaft and a small steel gear waits
2.5 cm up the next shaft, its teeth resting above the large gear's, half a tooth out of line. One hand turns the large gear by its knob until the teeth
line up and holds it there; the other pushes the small gear down its shaft into mesh. Then the large gear is turned
on, the small one turning with it, and both hands let go and go back to where they started. The task is one task file,
[`mesh_a_gear_turning_its_partner_to_line_up_the_teeth.py`](mesh_a_gear_turning_its_partner_to_line_up_the_teeth.py),
for the `yam_arena` embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports
from the same folder.

Why two hands: a detent springs the large gear back out of line as soon as it is let go, and the small gear only goes
down while the teeth are lined up, against a spring that also holds it up against its own weight (it does not drop in
by itself). Turned and let go, the large gear is back out of line before a hand reaches the small gear; one hand holds
the large gear lined up while the other pushes the small one in.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The plate and the two gears are rigid bodies made of
shapes from `bodykit.py`; the plate is kinematic, fixed where it is put.

- The large gear turns on its shaft on a hinge (a `Hinge` from `bodykit.py`) with a detent spring (0.03 N m per
  degree) toward its rest, where its teeth are half a tooth (5 degrees) out of line with the small gear's.
- The small gear slides on its keyed shaft on a slide (a `Slider` from `bodykit.py`). After every control step the task
  file reads the large gear's turn and the small gear's height: while the teeth are out of line (more than 3 degrees),
  the small gear is held where it is by a stiff spring; lined up, it is free to go down against a spring that pushes it
  up (about 2 N at first, 6 N at the bottom). Once it is 3 mm down, in among the large gear's teeth, the task file locks
  the large gear where it is; all the way down, it latches the small gear there and frees the large gear again.
- In mesh, the pair turns together: the small gear's teeth (with a mark on its face) are shown only, and after every
  control step the task file turns them with the large gear, 36 to 24 (half as fast again, the other way).
- The teeth are drawn but do not collide; the gears do not collide with the plate or each other.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, which includes both arms back where they started, with
both gears still) has held for 1 s, otherwise
`score()` at the end of the episode (0, 0.25, 0.5 or 1.0; see `task.json`). `-E object_jitter=0` keeps every object
exactly where the task file puts it, so every trial starts from the same layout. The cameras render at 1280 x 704 for
sharp videos; Dream-1's client scales every frame to 320 x 176 before sending it, so
`-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=mesh_a_gear_turning_its_partner_to_line_up_the_teeth.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Turn the large gear by its knob until its teeth line up with the small gear's and hold it there with one hand, while the other pushes the small gear down its shaft into mesh; then turn the large gear on at least 30 degrees, the small gear turning with it, let go with both hands, and take both arms back to where they started." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/gear-dream
```

The scripted demo, which reaches full score: the left hand comes straight down on the large gear's knob, grips it and
turns it about 5 degrees counterclockwise, lining its teeth up with the small gear's, and holds it. The right hand comes
straight down on the small gear's knob, grips it and pushes it down its shaft into mesh; it clicks in, and the right
lets go and goes back to where it started. The left turns the large gear on 60 degrees, the small gear turning with it
the other way, then lets go and goes back to where it started:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=mesh_a_gear_turning_its_partner_to_line_up_the_teeth.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Turn the large gear by its knob until its teeth line up with the small gear's and hold it there with one hand, while the other pushes the small gear down its shaft into mesh; then turn the large gear on at least 30 degrees, the small gear turning with it, let go with both hands, and take both arms back to where they started." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/gear-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/gear-dream/*.json; do inspect-robots video "$log"; done
```

Set `GEAR_DEBUG=1` to print what the score is built from (the large gear's turn and how far it is out of line, how far
the small gear is down, the state of the fit (waiting, free, meshed), whether the gear was held lined up for 1 s,
whether both grippers are clear, and where each gripper's TCP is) twice a second.

## Runtime per trial

On one RTX 5090: about 2 minutes for a Dream-1 trial of 1,200 steps (132 s on the clock, with Dream answering in 0.66 s on average), and about 1 minute for the scripted demo (369 steps, 18.4 s of simulated time, 73 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.
