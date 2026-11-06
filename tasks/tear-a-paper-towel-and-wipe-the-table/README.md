# Tear a sheet off a paper-towel roll with both hands and wipe crumbs off a mat with it

A loose paper-towel roll stands on the table with two sheets unrolled from it, standing on edge; the outer sheet is
joined to the inner one along a perforation. One arm holds the roll while the other tears the outer sheet off, then
uses the torn sheet to wipe six yellow cubes (crumbs) off a blue mat. Pulled with one hand alone, the strip only drags
the light roll along, so the tear needs both arms. The task is one task file,
[`tear_a_paper_towel_and_wipe_the_table.py`](tear_a_paper_towel_and_wipe_the_table.py), for the `yam_arena`
embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from the same
folder.

## Requirements

[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) (tested at commit `bd62771`) on
Isaac Lab-Arena 0.3.1 / Isaac Sim 6.1, on Linux with an NVIDIA RTX GPU. Its `setup.sh`, or the program's Brev
launchable, installs everything, including the Dream-1 policy. The task needs no models from Arena's object library.

`yam_arena` task files place single boxes, cylinders and spheres, with no joints. The roll, the sheets and the cubes
are loose rigid bodies from `bodykit.py`, which also joins two bodies with a PhysX revolute joint (`Hinge`), optionally
one that breaks under a set force: the perforation. Importing it hooks `yam_arena.scene` when the simulator starts,
without changing yam_arena, gives `score()` each body's own pose, and switches a broken joint off and on again at every
reset, so PhysX rebuilds it and each trial starts with the strip whole.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, with every cube still) has held for 1 s, otherwise
`score()` at the end of the episode (0, 0.25, 0.5 or 1.0; see `task.json`). Whether a cube was wiped off with the
torn sheet is decided after every control step, when the cube's centre leaves the mat: the sheet must be within 5 cm
of it and both grippers' TCPs at least 6 cm away, so cubes pushed off by the fingers, or swept off before the sheet is
near, do not count. `-E object_jitter=0` keeps every object exactly where the task file puts it, so every trial starts
from the same layout. The cameras render at 1280 x 704 for sharp videos; Dream-1's client scales every frame to
320 x 176 before sending it, so `-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tear_a_paper_towel_and_wipe_the_table.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Hold the paper-towel roll with one hand and tear the outer sheet off at the perforation with the other, then use the torn sheet to wipe all the yellow cubes off the blue mat." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/towel-dream
```

The scripted demo, which reaches full score: the left arm comes straight down on the roll and grips its top; the right
arm pinches the outer sheet's upper edge 5 cm past the perforation and pulls straight away along the strip until the
perforation parts. It lifts the torn sheet clear of the cubes, sets its lower edge down just in front of the mat and
pushes the sheet forward across the mat, sweeping all six cubes off its far edge:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tear_a_paper_towel_and_wipe_the_table.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Hold the paper-towel roll with one hand and tear the outer sheet off at the perforation with the other, then use the torn sheet to wipe all the yellow cubes off the blue mat." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/towel-demo
```

The same demo with the left arm left at rest (one hand only) does not tear the sheet: the strip drags the roll some
18 cm across the table and the trial scores 0.

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/towel-dream/*.json; do inspect-robots video "$log"; done
```

Set `TEAR_DEBUG=1` to print what the score is built from (torn or not, cubes on the mat, cubes wiped off, where the
roll is) once a second.

## Runtime per trial

On one RTX 5090: about 2 minutes for a Dream-1 trial of 1,200 steps (148 s on the clock, with Dream answering in 0.71 s on average), and under 1 minute for the scripted demo (302 steps, 15.1 s of simulated time, 48 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed and the scripted demo is deterministic. With a nonzero
`object_jitter`, the trial's seed sets how far the free objects (the roll, the sheets and the cubes) shift.
