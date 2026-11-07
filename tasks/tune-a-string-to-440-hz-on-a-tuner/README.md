# Turn a tuning key with one hand while the other plucks the string until a tuner reads 440 Hz

A one-string zither (a spruce box) lies on the table: a steel string from a bridge to a tuning pin, a key on the pin, and a clip-on
tuner at its end. The string starts flat, at 412 Hz. One hand turns the key clockwise a little at a time while the other
plucks the string again and again, watching the tuner's needle; once it points straight up and the green light comes
on, the string is plucked once more with the key still held, and both hands let go. The task is one task file,
[`tune_a_string_to_440_hz_on_a_tuner.py`](tune_a_string_to_440_hz_on_a_tuner.py), for the `yam_arena` embodiment (scene,
scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from the same folder.

What it tests: closing a feedback loop with two hands, one acting (turning) and the other probing (plucking), reading
a gauge that only updates when probed and stopping at the right moment.

Why two hands: the tuner shows the pitch only when the string is plucked, and the tuned reading counts only if the
string was plucked while the other hand still held the key (as tuners do it: the key is let go only after a reading
shows the string in tune). With one hand (the right hand turning the key as in the demo, the left never plucking) the
demo scored 0.5: the string reached 440.4 Hz, but the tuner never read it.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The zither and the tuner are kinematic, fixed where
they are put; the key is a loose rigid body on a stiff hinge (a `Hinge` from `bodykit.py`) about the pin's upright
axis, so it stays where it is turned.

- The pitch: after every control step the task file reads how far the key has turned from where it started (clockwise
  from above) and sets the string's pitch to 412 Hz plus 0.2 Hz per degree.
- A pluck: a gripper's TCP coming within 1.6 cm of the string over its plucking part, after being at least 3 cm from
  it. At each pluck the task file notes the pitch and whether the other gripper's TCP is on the key, and moves the
  tuner's needle (and its green light, within 3 Hz of 440) to show that pitch.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, with the key still) has held for 1 s, otherwise `score()` at the end of the
episode (0, 0.25, 0.5 or 1.0; see `task.json`). `-E object_jitter=0` keeps every object exactly where the task file
puts it, so every trial starts from the same layout. The cameras render at 1280 x 704 for sharp videos; Dream-1's
client scales every frame to 320 x 176 before sending it, so `-E cam_width=320 -E cam_height=176` gives the policy the
same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tune_a_string_to_440_hz_on_a_tuner.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Tune the string to 440 Hz: with one hand turn the tuning key clockwise, a little at a time, and with the other pluck the string again and again, watching the tuner; stop when its needle points straight up and its green light comes on, pluck once more while still holding the key, then let go of both." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/tune-dream
```

The scripted demo, which reaches full score: the right hand comes straight down on the key, fingers closing across its
flats, and holds it while the left, fingers closed, sweeps across the string, plucking it: the tuner reads 411 Hz. The
right turns the key clockwise in two strokes (grip, roll the wrist, let go, roll back, grip again), about 140 degrees
in all, and holds it; the left plucks again, the tuner reads 440 Hz and its green light comes on. Then both hands rise:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tune_a_string_to_440_hz_on_a_tuner.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Tune the string to 440 Hz: with one hand turn the tuning key clockwise, a little at a time, and with the other pluck the string again and again, watching the tuner; stop when its needle points straight up and its green light comes on, pluck once more while still holding the key, then let go of both." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/tune-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/tune-dream/*.json; do inspect-robots video "$log"; done
```

Set `TUNE_DEBUG=1` to print what the score is built from twice a second: how far the key has turned and the string's
pitch, how many plucks so far, the tuner's reading and whether it was taken with the key held, which hands are on the
key, whether both hands are clear, and where each TCP is from the string and the key.

## Runtime per trial

On one RTX 5090: about 2 minutes for a Dream-1 trial of 1,200 steps (136 s on the clock, with Dream answering in 0.81 s on average), and about 2 minutes for the scripted demo (373 steps, 18.6 s of simulated time, 122 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.
