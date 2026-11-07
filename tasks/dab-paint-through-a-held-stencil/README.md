# Press a stencil flat with one hand while the other dabs paint through its three holes

A sheet of paper lies on the table with a thin plastic stencil on it: three square holes in a row. A sponge dauber with a wooden handle,
inked red, stands in a holder beside it. One hand presses the stencil flat on the paper; the other takes the dauber and
dabs it down through each hole, once each, printing three red squares, then lifts it clear. The task is one task file,
[`dab_paint_through_a_held_stencil.py`](dab_paint_through_a_held_stencil.py), for the `yam_arena` embodiment (scene,
scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from the same folder.

What it tests: placing a tool accurately through openings, again and again, while the other hand keeps the stencil
still.

Why two hands: this is how stencilling is done, and the score asks for it. A stencil that is not held lifts at its
edges and the paint bleeds under it; the simulation does not model that, so a print dabbed while no other hand is
pressing on the stencil counts as smudged (and shows wider). Dabbed with one hand, three prints score 0.5 at most. One
hand presses the stencil flat while the other dabs.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The paper and the dauber's holder are kinematic,
fixed where they are put; the stencil (a sheet built round its three holes) and the dauber are loose rigid bodies made
of shapes from `bodykit.py`.

- The paint: 16 red squares (8 clean, 8 wider smudged ones) wait hidden under the paper. After every control step, when
  the dauber's pad comes down within 1 mm of the paper (after being at least 1 cm up), the task file lays a print where
  the pad is: a clean one if the other gripper's TCP is pressing on the stencil (within 3 cm above it), else a smudged
  one.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, with the dauber and the stencil still) has held for 1 s,
otherwise `score()` at the end of the episode (0, 0.25, 0.5 or 1.0; see `task.json`). `-E object_jitter=0` keeps every
object exactly where the task file puts it, so every trial starts from the same layout. The cameras render at
1280 x 704 for sharp videos; Dream-1's client scales every frame to 320 x 176 before sending it, so
`-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=dab_paint_through_a_held_stencil.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Press the stencil flat on the paper with one hand; with the other take the sponge dauber from its holder and dab it down through each of the stencil's three holes, once each, then lift it clear." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/stencil-dream
```

The scripted demo, which reaches full score: the left hand comes straight down on the stencil's left end, fingers
closed, and presses it flat on the paper. The right hand comes straight down on the dauber's handle, fingers closing
across it, lifts it out of its holder and dabs it straight down through each hole in turn, right to left, then lifts
it clear:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=dab_paint_through_a_held_stencil.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Press the stencil flat on the paper with one hand; with the other take the sponge dauber from its holder and dab it down through each of the stencil's three holes, once each, then lift it clear." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/stencil-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/stencil-dream/*.json; do inspect-robots video "$log"; done
```

Set `DAB_DEBUG=1` to print what the score is built from (where the pad and the stencil are, every print so far and
whether it was clean, how high the left TCP is above the stencil, how many squares are done and how many prints are
smudged) twice a second.

## Runtime per trial

On one RTX 5090: about 2 minutes for a Dream-1 trial of 1,200 steps (147 s on the clock, with Dream answering in 0.75 s on average), and about 3 minutes for the scripted demo (458 steps, 22.9 s of simulated time, 181 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.
