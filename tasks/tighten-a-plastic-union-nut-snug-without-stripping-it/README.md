# Hold a plastic pipe still with one hand while the other tightens its union nut snug, stopping before the thread strips

A short plastic pipe lies in two supports with a union nut started on its threaded end. One hand holds the pipe; the
other tightens the nut until it is snug against the pipe's collar, and stops there: plastic threads strip if they are
forced. Then both hands let go. The task is one task file,
[`tighten_a_plastic_union_nut_snug_without_stripping_it.py`](tighten_a_plastic_union_nut_snug_without_stripping_it.py),
for the `yam_arena` embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it
imports from the same folder.

What it tests: knowing when to stop. Snug, the nut gets much harder to turn (its drag goes from 0.3 to 0.8 N m s/rad);
turned on half a turn past snug, the thread strips and the trial scores 0, however well the rest went. Running the
scripted demo on for twelve strokes instead of six strips it (0.0).

Why two hands: the pipe lies loose in smooth supports and turns with the nut. With the pipe unheld, the demo's strokes
turn the pipe along and the nut hardly goes on (0.25; checked by running the demo with the left hand kept clear).

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The pipe (with its collar), the nut and the supports
are rigid bodies made of several shapes from `bodykit.py`; the supports are kinematic, fixed where they are put, and
smooth (they hardly grip the pipe).

- PhysX has no screw thread. The nut sits on the pipe on a joint (a `Thread` from `bodykit.py`, on from the start) that
  keeps it on the pipe's axis and drags as they turn against each other. After every control step the task file adds
  up the nut's turn against the pipe and moves the joint's spring target 4 mm along the pipe per turn tightened; a turn
  and a half brings it against the collar (snug), where the target stops and the drag goes up. Half a turn past snug,
  the thread strips: the drag drops almost to nothing and the nut spins loose where it is.
- The nut does not collide with the pipe.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file look at the simulation after every control step.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, with the pipe and the nut still) has held for 1 s,
otherwise `score()` at the end of the episode (0, 0.25, 0.5 or 1.0; see `task.json`). `-E object_jitter=0` keeps every
object exactly where the task file puts it, so every trial starts from the same layout. The cameras render at 1280 x 704
for sharp videos; Dream-1's client scales every frame to 320 x 176 before sending it, so
`-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tighten_a_plastic_union_nut_snug_without_stripping_it.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Hold the pipe still with one hand and with the other tighten the union nut on its end until it is snug against the collar, without forcing it further, then let go with both hands." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/union-dream
```

The scripted demo, which reaches full score: the left hand comes straight down on the pipe near its left end and holds
it. The right hand, level and pointing along the pipe, comes down onto the nut (its fingers either side), closes on it
and tightens it in strokes of a third of a turn (the wrist rolls only so far): roll, let go, roll back open, take the
nut again, following it along the thread. Six strokes bring it snug (the fingers slip a little each stroke); then both
hands let go and rise:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tighten_a_plastic_union_nut_snug_without_stripping_it.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Hold the pipe still with one hand and with the other tighten the union nut on its end until it is snug against the collar, without forcing it further, then let go with both hands." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/union-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/union-dream/*.json; do inspect-robots video "$log"; done
```

Set `UNION_DEBUG=1` to print what the score is built from (where the pipe is and how far it has turned, the nut's turns
and how far along the thread it is, whether it is snug or stripped and both grippers clear, and where each gripper's TCP
is) twice a second.

## Runtime per trial

On one RTX 5090: about 3 minutes for a Dream-1 trial of 1,200 steps (152 s on the clock, with Dream answering in 0.74 s on average), and about 1 minute for the scripted demo (510 steps, 25.5 s of simulated time, 73 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.
