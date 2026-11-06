# Hold a sprung belt tensioner at its mark with one hand while the other turns its lock knob, then let go

A belt drive module lies on the table: two pulleys with a timing belt round them, and a tensioner lever pivoting in
front of the belt, sprung off it. One hand swings the lever back by the peg at its tip until its pointer is in the
green band (the idler wheel against the belt) and holds it there; the other turns the lock knob on the lever's pivot a
quarter turn clockwise, which locks the lever. Then both hands let go. The task is one task file,
[`tension_a_timing_belt_and_lock_the_idler.py`](tension_a_timing_belt_and_lock_the_idler.py), for the `yam_arena`
embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from the same
folder.

Why two hands: the spring swings the lever off the belt as soon as it is let go, and the knob locks the lever wherever
it is when turned. Swung to the mark and let go, the lever is back on its stop before a hand gets to the knob; turned
first, the knob locks it slack. One hand holds the lever at the mark while the other turns the knob.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The module, the lever and the knob are rigid bodies
made of shapes from `bodykit.py`; the module is kinematic, fixed where it is put.

- The lever is joined to the module by a hinge about its pivot bolt (a `Hinge` from `bodykit.py`) with a spring
  (0.03 N m per degree, preloaded against its forward stop) and a stop 12 degrees back.
- The knob is joined to the module by a damped hinge on the same axis, stopped at a quarter turn clockwise. After every
  control step the task file reads both angles: once the knob is turned 75 degrees, it sets the lever's hinge to hold
  the lever where it is with a stiff spring (locked); turned back, the soft spring again.
- The belt is shown but not simulated. The idler stands 1.1 cm off the belt at rest and meets it at 6.3 degrees, the
  middle of the green band. The lever and the knob do not collide with the module.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, with the lever and the knob still) has held for 1 s,
otherwise `score()` at the end of the episode (0, 0.25, 0.5 or 1.0; see `task.json`). `-E object_jitter=0` keeps every
object exactly where the task file puts it, so every trial starts from the same layout. The cameras render at 1280 x 704
for sharp videos; Dream-1's client scales every frame to 320 x 176 before sending it, so
`-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tension_a_timing_belt_and_lock_the_idler.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Swing the tensioner lever back by the red peg at its tip until its pointer is in the green band, hold it there with one hand while the other turns the lock knob on its pivot a quarter turn clockwise, then let go with both hands." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/belt-dream
```

The scripted demo, which reaches full score: the left hand comes straight down round the peg and swings the lever back
along its arc until the idler meets the belt, and holds it. The right hand, waiting over the knob, comes straight down
on it, grips it and turns it a quarter turn clockwise. Then the right hand lets go and rises, and the left hand lets go
and rises:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tension_a_timing_belt_and_lock_the_idler.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Swing the tensioner lever back by the red peg at its tip until its pointer is in the green band, hold it there with one hand while the other turns the lock knob on its pivot a quarter turn clockwise, then let go with both hands." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/belt-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/belt-dream/*.json; do inspect-robots video "$log"; done
```

Set `BELT_DEBUG=1` to print what the score is built from (the lever's angle, how far the knob is turned, the angle it
was locked at, whether it was held in the band for 1 s, whether both grippers are clear, and where each gripper's TCP
is) twice a second.

## Runtime per trial

On one RTX 5090: about 3 minutes for a Dream-1 trial of 1,200 steps (165 s on the clock, with Dream answering in 0.78 s on average), and under 1 minute for the scripted demo (282 steps, 14.1 s of simulated time, 50 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.
