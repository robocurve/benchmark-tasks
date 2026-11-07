# Hold a box grater steady by its handle with one hand while the other grates a block of cheese on it

A steel box grater stands on a beech-wood cutting board with a block of cheese beside it. One hand holds the grater by the flat handle
reaching left from its top; the other takes the cheese, presses its end against the grating face and rubs it up and
down, so that shreds of cheese fall inside the grater onto the board. The task is one task file,
[`grate_cheese_holding_the_grater_steady.py`](grate_cheese_holding_the_grater_steady.py), for the `yam_arena` embodiment
(scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from the same folder.

Why two hands: the grater is light (180 g) and tall, and its teeth bite on every stroke. Unheld, it tips over on the
first upstrokes: with the left hand kept clear, the scripted demo's strokes tip it to 55 degrees within a second and
grate 3 shreds before it falls (0.25). One hand holds it while the other grates.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The grater, the cheese, the board and the shreds are
rigid bodies made of shapes from `bodykit.py`; the board is kinematic, fixed where it is put.

- After every control step the task file checks whether the cheese's end is on the grating face (within 4 mm of it,
  inside its grating area) and how far it has moved up or down along it since the last step. Each 2 cm rubbed, with
  the grater standing, brings in one shred: twelve small sticks of cheese wait in a closed box high above the scene,
  out of the cameras' view, and the task file moves the next one inside the grater, just behind the face at the
  cheese's height, from where it falls onto the board.
- The teeth's bite: while the cheese is rubbed on the face, the task file pulls the grater along the stroke at the
  face, 60 N per m/s of rubbing, at most 4 N. Held by its handle the grater barely moves; unheld, an upstroke lifts its
  face side and tips it over.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, which includes the cheese put down and both arms back
where they started, with the shreds, the grater and the cheese still) has held for 1 s, otherwise
`score()` at the end of the episode (0, 0.25, 0.5 or 1.0; see `task.json`). `-E object_jitter=0` keeps every object
exactly where the task file puts it, so every trial starts from the same layout. The cameras render at 1280 x 704 for
sharp videos; Dream-1's client scales every frame to 320 x 176 before sending it, so
`-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=grate_cheese_holding_the_grater_steady.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Hold the grater steady by its handle with one hand, and with the other rub the cheese up and down its grating face, pressed against it, to grate the cheese onto the board; then put the cheese down, let go of the grater and take both arms back to where they started." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/grate-dream
```

The scripted demo, which reaches full score: the left hand comes straight down on the grater's handle near its end and
holds it. The right hand comes straight down on the cheese near its right end, lifts it, presses its left end on the
grating face and rubs it down and up the face six times, then puts it back down. Then the left lets go of the grater
and both arms go back to where they started, leaving the grated cheese on the board:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=grate_cheese_holding_the_grater_steady.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Hold the grater steady by its handle with one hand, and with the other rub the cheese up and down its grating face, pressed against it, to grate the cheese onto the board; then put the cheese down, let go of the grater and take both arms back to where they started." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/grate-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/grate-dream/*.json; do inspect-robots video "$log"; done
```

Set `GRATE_DEBUG=1` to print what the score is built from (where the grater is and how tilted, whether the cheese is
rubbing on the face and at what height, how far it has been rubbed, how many shreds have dropped and lie on the board,
and where each gripper's TCP is) twice a second.

## Runtime per trial

On one RTX 5090: about 3 minutes for a Dream-1 trial of 1,200 steps (176 s on the clock, with Dream answering in 0.90 s on average), and about 1 minute for the scripted demo (620 steps, 31.0 s of simulated time, 85 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.
