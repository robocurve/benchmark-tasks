# Hold a loaf with one hand, clear of the blade, while the other saws a slice 1 to 2 cm thick off its end

A square sandwich loaf lies on a cutting board with a bread knife in a rest beside it. One hand holds the loaf down on
the board, fingertips well back from where the cut will be; the other takes the knife and saws a slice 1 to 2 cm thick
off the loaf's right end. Neither hand may come within 3 cm of the blade at any time. The task is one task file,
[`slice_a_loaf_holding_it_with_a_claw_grip.py`](slice_a_loaf_holding_it_with_a_claw_grip.py), for the `yam_arena`
embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from the same
folder.

Why two hands: sawing drags the loaf along with the blade. With the loaf unheld, it slides away with the knife and the
scripted demo's strokes cut nothing (0.0; checked by running the demo with the left hand kept clear). One hand has to
hold the loaf while the other saws, and the safety rule keeps that hand back from the cut.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The loaf, the knife, the board and the knife rest are
rigid bodies made of shapes from `bodykit.py`; the board and the rest are kinematic, fixed where they are put.

- The loaf's right 3 cm is six 5 mm slabs, held to each other and to the rest of the loaf by fixed joints (a `Glue` from
  `bodykit.py`, switched back on at every reset). Each slab shows crumb on its faces inside the crust, so a cut face
  looks cut.
- The knife does not collide with the bread. After every control step the task file finds which joint line the
  blade's plane is on (within 2.5 mm; once a cut has started, the blade follows its kerf within 6 mm) and how far the
  blade has been sawn along itself against the loaf. The cut there grows by 20% of the distance sawn, no deeper than
  the blade's edge. While the edge is deeper than the cut, the uncut bread pushes the knife back up (1500 N/m, at most
  25 N, applied at the handle where it is held). The blade drags the loaf along (150 N per m/s of sawing, at most 5 N;
  an unheld loaf slides at about 2 N). A cut through the loaf's whole height switches that joint off: the slice is
  free, and a nudge at its top for half a second tips it away from the loaf.
- Both grippers' fingertip centres are checked against the blade (not the handle) after every step.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, and lets the task
file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, with the loaf still) has held for 1 s, otherwise
`score()` at the end of the episode (0, 0.25, 0.5 or 1.0; see `task.json`). `-E object_jitter=0` keeps every object
exactly where the task file puts it, so every trial starts from the same layout. The cameras render at 1280 x 704 for
sharp videos; Dream-1's client scales every frame to 320 x 176 before sending it, so
`-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=slice_a_loaf_holding_it_with_a_claw_grip.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Hold the loaf on the board with one hand, keeping that hand at least 3 cm from the blade, and with the other cut a slice 1 to 2 cm thick off its right end with the bread knife." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/slice-dream
```

The scripted demo, which reaches full score: the left hand comes straight down on the loaf's top, 11 cm from the cut,
fingers closed, and presses it onto the board. The right hand comes straight down on the knife's handle, grips it,
lifts the knife out of the rest, sets the blade across the loaf's top 1.5 cm from its right end and saws, back and
forth along the blade, lowering it as the cut deepens, until the edge is through to the board; then it lifts the knife
out:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=slice_a_loaf_holding_it_with_a_claw_grip.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Hold the loaf on the board with one hand, keeping that hand at least 3 cm from the blade, and with the other cut a slice 1 to 2 cm thick off its right end with the bread knife." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/slice-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/slice-dream/*.json; do inspect-robots video "$log"; done
```

Set `SLICE_DEBUG=1` to print what the score is built from (where the loaf is, where the knife is against it, which
joint line the blade is on, how deep each cut is and which are through, the drag on the loaf and the push on the knife,
how far each hand is from the blade, and the score so far) twice a second.

## Runtime per trial

On one RTX 5090: about 4 minutes for a Dream-1 trial of 1,200 steps (213 s on the clock, with Dream answering in 0.81 s on average), and about 1 minute for the scripted demo (293 steps, 14.7 s of simulated time, 65 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.
