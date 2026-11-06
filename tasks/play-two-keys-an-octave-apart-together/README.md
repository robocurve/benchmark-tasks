# Play two keys an octave apart as one chord: press both at the same moment, one hand on each, and hold them

A toy keyboard lies on the table with two of its keys, the C keys an octave apart, marked with red dots. Both must go
down at the same moment (within 0.1 s), stay down together for a second, and no other key may be pressed. The task is
one task file, [`play_two_keys_an_octave_apart_together.py`](play_two_keys_an_octave_apart_together.py), for the
`yam_arena` embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from
the same folder.

Why two hands: the two marked keys are 21 cm apart, far wider than one gripper spans. One hand can hold down one of
them (0.25 at most); the chord needs a hand on each, pressing together.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The keyboard and its fifteen white keys are rigid
bodies made of shapes from `bodykit.py`; the keyboard (with its black keys) is kinematic, fixed where it is put.

- Each white key is joined to the keyboard by a hinge at its back end (a `Hinge` from `bodykit.py`) with a spring that
  holds it up (preloaded against its upper stop) and a stop 8 mm down at its front. The keys do not collide with the
  keyboard; neighbouring keys have a 1 mm gap.
- After every control step the task file reads each key's angle, notes when each key went down (front pressed at
  least 5 mm) and whether any unmarked key has been pressed, and keeps the best score reached.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, and lets the task
file look at the simulation after every control step.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (the chord played) has held for 1 s, otherwise `score()` at the end
of the episode (0, 0.25, 0.5 or 1.0; see `task.json`). `-E object_jitter=0` keeps every object exactly where the task
file puts it, so every trial starts from the same layout. The cameras render at 1280 x 704 for sharp videos; Dream-1's
client scales every frame to 320 x 176 before sending it, so `-E cam_width=320 -E cam_height=176` gives the policy the
same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=play_two_keys_an_octave_apart_together.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Press the two keys marked with red dots at the same moment, one hand on each, hold them down together for a second, and press no other key." \
  --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/keys-dream
```

The scripted demo, which reaches full score: each hand comes over its marked key, fingers closed and closing across the
keys (the closed fingertips are narrower than a key), just above the red dot. Both press down together, hold the chord
and let the keys up:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=play_two_keys_an_octave_apart_together.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Press the two keys marked with red dots at the same moment, one hand on each, hold them down together for a second, and press no other key." \
  --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/keys-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/keys-dream/*.json; do inspect-robots video "$log"; done
```

Set `KEYS_DEBUG=1` to print what the score is built from (how far each key's front is pressed, when each marked key
went down, how long both have been down together, whether another key was pressed, and the score so far) four times a
second.

## Runtime per trial

On one RTX 5090: about 1 minute for a Dream-1 trial of 600 steps (78 s on the clock, with Dream answering in 0.82 s on average), and under 1 minute for the scripted demo (145 steps, 7.2 s of simulated time, 27 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.
