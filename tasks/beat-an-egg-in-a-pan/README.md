# Beat an egg in a pan with a whisk while holding the pan steady by its handle

One arm holds a pan by its handle while the other takes a balloon whisk from its holder and beats an "egg" (a yellow
yolk of beads inside a ring of white beads) until the yolk is spread through the white, without spilling it or pushing
the pan off the burner. The task is one task file, [`beat_an_egg_in_a_pan.py`](beat_an_egg_in_a_pan.py), for the
`yam_arena` embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from
the same folder.

## Requirements

[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) (tested at commit `bd62771`) on
Isaac Lab-Arena 0.3.1 / Isaac Sim 6.1, on Linux with an NVIDIA RTX GPU. Its `setup.sh`, or the program's Brev
launchable, installs everything, including the Dream-1 policy. The task needs no models from Arena's object library.

`yam_arena` task files place single boxes, cylinders and spheres. The pan (floor, round wall, handle, painted lines),
the whisk (eight wires, collar, handle) and the beads are each one loose rigid body built from several shapes, so
`bodykit.py` adds such bodies: importing it hooks `yam_arena.scene` when the simulator starts, without changing
yam_arena, and gives `score()` each body's own pose.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, with the pan, the whisk and every bead still) has held
for 1 s, otherwise `score()` at the end of the episode: 1.0, 0.5 or 0 by the largest patch of yellow beads still linked
together (centres under 1.7 cm apart), and 0 if the pan was tipped or lifted, a finger went into the egg, beads were
spilled or the pan was pushed off the burner (see `task.json`). `-E object_jitter=0` keeps
every object exactly where the task file puts it, so every trial starts from the same layout.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=beat_an_egg_in_a_pan.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Hold the pan steady by its handle and beat the egg in it with the whisk until the yolk and the white are mixed together. Touch the egg only with the whisk, and keep all of the egg in the pan and the pan flat on the burner." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/egg-dream
```

The scripted demo, which reaches full score: the left arm grips the pan's handle; the right arm takes the whisk from
its holder, beats in loops whose centre travels round the pan, finishes with one slow circle round the outer ring, and
stands the whisk back in its holder:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=beat_an_egg_in_a_pan.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Hold the pan steady by its handle and beat the egg in it with the whisk until the yolk and the white are mixed together. Touch the egg only with the whisk, and keep all of the egg in the pan and the pan flat on the burner." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/egg-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/egg-dream/*.json; do inspect-robots video "$log"; done
```

Set `EGG_DEBUG=1` to print what the score is built from (largest yolk patch, pan tilt, fingers) once a second.

## Runtime per trial

On one RTX 5090: about 7 minutes for a Dream-1 trial of 2,400 steps (396 s on the clock, with Dream answering in 0.70 s on average), and about 6 minutes for the scripted demo (1,706 steps, 85.3 s of simulated time, 339 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed and the scripted demo is deterministic. With a nonzero
`object_jitter`, the trial's seed sets how far the free objects (the pan, the beads and the whisk) shift.
