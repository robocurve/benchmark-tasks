# Keep a cabinet key turned with one hand while the other pulls the drawer open

A small cabinet stands on the table with a locked drawer. A key sticks out of the lock on the drawer front's left part;
a pull handle stands on its right part. The lock has a spring: the key turns back as soon as it is let go and the bolt
catches again. One hand turns the key a quarter turn clockwise and keeps it turned while the other pulls the drawer open;
once the drawer is on its way out, the key is let go. The task is one task file,
[`keep_a_key_turned_while_pulling_a_drawer_open.py`](keep_a_key_turned_while_pulling_a_drawer_open.py), for the
`yam_arena` embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from
the same folder.

What it tests: two hands acting at the same time on one object, one holding a sprung part turned while the other pulls;
turning a key with a level grip by rolling the wrist.

Why two hands: the key has to be held turned while the drawer starts out; turned and let go first, it turns back and
the bolt catches before the drawer moves. With the key turned and let go first and the drawer pulled afterwards, as a
single hand would have to do it, the demo scored 0.25: the bolt had caught again and the drawer did not move.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The cabinet is kinematic, fixed where it is put; the
drawer and the key are loose rigid bodies made of shapes from `bodykit.py`.

- The drawer slides on a slider joint (a `Slider` from `bodykit.py`) with drag; the key turns in the drawer front on a
  hinge (a `Hinge` from `bodykit.py`) with a spring that turns it back upright.
- The bolt: a fixed joint between the cabinet and the drawer (a `Glue` from `bodykit.py`), on at the start. After every
  control step the task file switches it off once the key is turned 60 degrees, and on again (pulling the drawer shut)
  if the key turns back below 60 degrees before the drawer is 8 mm out, or once the drawer is pushed back to within
  3 mm of shut with the key let back.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, with the drawer and the key still) has held for 1 s, otherwise `score()` at the end of the
episode (0, 0.25, 0.5 or 1.0; see `task.json`). `-E object_jitter=0` keeps every object exactly where the task file
puts it, so every trial starts from the same layout. The cameras render at 1280 x 704 for sharp videos; Dream-1's
client scales every frame to 320 x 176 before sending it, so `-E cam_width=320 -E cam_height=176` gives the policy the
same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=keep_a_key_turned_while_pulling_a_drawer_open.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Turn the key in the drawer's lock a quarter turn clockwise and keep it turned with one hand while the other pulls the drawer open by its handle; once the drawer is on its way out, let go of the key." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/key-dream
```

The scripted demo, which reaches full score: the left hand comes level from the front onto the key's bow, fingers
closing across it, rolls its wrist a quarter turn clockwise, turning the key, and holds it there. The right hand comes
straight down on the pull's bar, fingers closing across it, and pulls the drawer straight out. The left keeps the key
turned, following it out, until the drawer is 2.5 cm out (the bolt past the strike), then lets it go (it turns back)
and backs off and up, out of the drawer's way, while the right waits; the right pulls the drawer on out to about 9 cm,
lets go of the pull and rises:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=keep_a_key_turned_while_pulling_a_drawer_open.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Turn the key in the drawer's lock a quarter turn clockwise and keep it turned with one hand while the other pulls the drawer open by its handle; once the drawer is on its way out, let go of the key." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/key-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/key-dream/*.json; do inspect-robots video "$log"; done
```

Set `KEY_DEBUG=1` to print what the score is built from twice a second: how far the drawer is out (and the most so
far), how far the key is turned, whether the bolt holds the drawer and whether it was ever drawn, and where each TCP is
from the key's bow and the pull's bar.

## Runtime per trial

On one RTX 5090: about 2 minutes for a Dream-1 trial of 1,200 steps (146 s on the clock, with Dream answering in 0.90 s on average), and about 2 minutes for the scripted demo (236 steps, 11.8 s of simulated time, 91 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.
