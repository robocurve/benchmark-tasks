# Lift a heavy two-handled pot with both hands and pour the soup into a bowl

Both arms lift a stockpot off a stove by its two side lugs, carry it to a serving bowl, tip it together to pour the soup
(90 beads) into the bowl without spilling, and set it back on the stove. The pot is too heavy to hold by one lug: held
that way it swings down about the lug, so the arms have to move as one. The task is one task file,
[`pour_a_pot_with_both_hands.py`](pour_a_pot_with_both_hands.py), for the `yam_arena` embodiment (scene, scoring and
the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from the same folder.

## Requirements

[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) (tested at commit `bd62771`) on
Isaac Lab-Arena 0.3.1 / Isaac Sim 6.1, on Linux with an NVIDIA RTX GPU. Its `setup.sh`, or the program's Brev
launchable, installs everything, including the Dream-1 policy. The task needs no models from Arena's object library.

`yam_arena` task files place single boxes, cylinders and spheres. The pot (floor, round wall, two lugs) and each bead
are loose rigid bodies built from several shapes, so `bodykit.py` adds such bodies: importing it hooks
`yam_arena.scene` when the simulator starts, without changing yam_arena, and gives `score()` each body's own pose. It
also lets the scripted demo give a grip as a full rotation, since pouring needs level grips that roll, not just
straight-down ones.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, with the pot and every bead still) has held for 1 s,
otherwise `score()` at the end of the episode (0, 0.25, 0.5 or 1.0 by the beads in the bowl, the beads spilled and
where the pot ends; see `task.json`). `-E object_jitter=0` keeps every object exactly where the task file puts it, so
every trial starts from the same layout. The cameras render at 1280 x 704 for sharp videos; Dream-1's client scales
every frame to 320 x 176 before sending it, so `-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=pour_a_pot_with_both_hands.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Lift the pot off the stove by both handles, pour all of the soup into the white bowl without spilling, and set the pot back down on the stove." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/pot-dream
```

The scripted demo, which reaches full score: both arms turn their grips level and come down round the lugs (fingers
closing front and back across each lug), lift the pot together, carry it over the bowl, roll both wrists to tip it
110 degrees about the line through the lugs, tip it back, and set it down on the stove:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=pour_a_pot_with_both_hands.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Lift the pot off the stove by both handles, pour all of the soup into the white bowl without spilling, and set the pot back down on the stove." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/pot-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/pot-dream/*.json; do inspect-robots video "$log"; done
```

Set `POT_DEBUG=1` to print what the score is built from (beads in the bowl, in the pot and spilled; the pot's tilt)
once a second.

## Runtime per trial

On one RTX 5090: about 4 minutes for a Dream-1 trial of 1,200 steps (218 s on the clock, with Dream answering in 0.77 s on average), and about 2 minutes for the scripted demo (485 steps, 24.2 s of simulated time, 102 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed and the scripted demo is deterministic. With a nonzero
`object_jitter`, the trial's seed sets how far the free objects (the pot and the beads) shift.
