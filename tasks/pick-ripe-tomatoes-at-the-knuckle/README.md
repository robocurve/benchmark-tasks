# Pick the ripe tomatoes off the truss at the knuckle

A tomato plant's stem rises at the back of the table with one truss hanging from it at working height: three tomatoes
on short pedicels, the two nearest the stem red and the one at the far end green. A shallow harvest tray stands under
the truss. The task is to pick the ripe tomatoes and put them in the tray, leaving the green one on the truss, the way
greenhouse pickers do it: take the fruit in the hand and bend it up and over toward the truss until its pedicel snaps
at the knuckle (the abscission joint), which leaves the calyx on the fruit. Pulling the fruit off along its stem
does not part the knuckle: it drags the stem and tears the truss. The task is the task file
[`pick_ripe_tomatoes_at_the_knuckle.py`](pick_ripe_tomatoes_at_the_knuckle.py) for the `yam_arena` embodiment (scene,
scoring and the scripted demo), with [`bodykit.py`](bodykit.py) next to it.

## Requirements

[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) (tested at commit `0059d02`) on
Isaac Lab-Arena 0.3.1 / Isaac Sim 6.1, on Linux with an NVIDIA RTX GPU. Its `setup.sh`, or the program's Brev
launchable, installs everything, including the Dream-1 policy. The task needs no models from Arena's object library.

`yam_arena` task files place single boxes, cylinders and spheres, upright, and cannot join them. The plant, each
tomato and the tray are rigid bodies built from several shapes, and each tomato hangs from its pedicel on a joint, so
`bodykit.py` adds such bodies and joints: importing it hooks `yam_arena.scene` when the simulator starts, without
changing yam_arena, and gives `score()` each body's own pose. The knuckle is a ball joint that lets the tomato swing up
to 35 degrees in any direction and breaks once it carries more than 0.12 N m at that limit; it never breaks by
pulling: pulled along its stem, the pedicel stretches on a spring (1,000 N/m), and stretched 6 mm the truss counts as
torn. These values are modelling choices, not measurements.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` has held for 1 s, otherwise the score at the end of the run. A tomato
is off the truss when its knuckle has moved 4 mm or more sideways from the end of its pedicel. The green tomato coming
off, a tomato pulled 6 mm along its stem (the truss torn), or a tomato falling onto the table outside the tray sets
the score to 0 for the rest of the run. Otherwise the score is the
best stage reached: 0.25 for each red tomato off the truss and 0.25 more for each one in the tray, at most 0.75; full
score is both red tomatoes in the tray, the green one still on the truss, both grippers 5 cm or more from every tomato,
still for 1 s. `-E object_jitter=0` keeps every object exactly where the task file puts it, so every trial starts from
the same layout.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=pick_ripe_tomatoes_at_the_knuckle.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Pick the ripe tomatoes off the truss and put them in the tray." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/tomato-dream
```

The scripted demo, which reaches full score (the right gripper comes at each red tomato from the front, level, fingers
either side of it, closes on it, swings it up and toward itself around the knuckle until the knuckle snaps, draws it
back and down over the tray and lets it go; the left arm does not move):

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=pick_ripe_tomatoes_at_the_knuckle.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Pick the ripe tomatoes off the truss and put them in the tray." \
  --scorer task_score --max-steps 700 --store-frames --log-dir ~/yam-eval/tomato-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/tomato-dream/*.json; do inspect-robots video "$log"; done
```

`TOMATO_DEBUG=1` prints, every 10 steps, where each tomato is, whether it is on the truss, off it or in the tray, how
far the grippers are from the tomatoes, and the stage reached so far.

`TOMATO_PULL=1` runs the demo with a pull instead of the bend: the gripper closes on the first red tomato and pulls it
straight down along its stem. The knuckle holds, the stem stretches past 6 mm, the truss counts as torn and the run
scores 0, with the tomato still on the truss.

## Runtime per trial

On one RTX PRO 6000 (Blackwell) shared with another job: about 3 minutes for the scripted demo while rendering
1280 x 720 video (575 steps, 29 s of simulated time; 193 s), and about 16 minutes for a Dream-1 trial of 2,400 steps
(960 s).
Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed and the scripted demo is deterministic.

## Building it for real

A truss of a cocktail-plum or medium tomato variety (fruit 4.5 to 5 cm across) with two red fruit and one green, cut
with a length of stem and clamped to a stand so that it hangs 15 to 17 cm above the table, and a 13 x 26 cm harvest
tray about 2.5 cm deep under it. Each trial uses a fresh truss.
