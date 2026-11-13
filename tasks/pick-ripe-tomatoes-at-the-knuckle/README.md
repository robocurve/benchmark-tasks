# Pick the ripe tomatoes off the truss at the knuckle

A tomato plant's stem rises at the back of the table with one truss hanging from it at working height: three tomatoes
on short pedicels, the two nearest the stem red and the one at the far end green. A shallow harvest tray stands under
the truss. The instruction asks to pick the ripe tomatoes off the truss and put them in the tray. The green one stays on
the truss to ripen, and no tomato may be crushed. A ripe tomato comes off cleanly when it is taken in the hand and bent
up and over toward the truss until its pedicel snaps at the knuckle (the abscission zone, a joint on the fruit's
stalk), which leaves the calyx on the fruit. Pulling the fruit along its stalk does not part the knuckle: it drags the
stalk and the truss. The task is the task file
[`pick_ripe_tomatoes_at_the_knuckle.py`](pick_ripe_tomatoes_at_the_knuckle.py) for the `yam_arena` embodiment (scene,
scoring and the scripted demo), with [`bodykit.py`](bodykit.py) next to it.

## Why it matters

Picking tomatoes off the plant is one of the most familiar home-garden jobs. In the National Gardening Association's
survey report *The Impact of Home and Community Gardening in America* (B. Butterfield, 2009;
[archived PDF](http://web.archive.org/web/20261002055649/https://gardenresearch.com/files/2009-Impact-of-Gardening-in-America-White-Paper.pdf)),
"31 percent of all U.S. households, or an estimated 36 million households, participated in food gardening in 2008",
and "the most popular vegetables grown include tomatoes (86 percent)". Its later report *Garden to Table: A 5-Year Look
at Food Gardening in America* (2014,
[PDF](https://ampleharvest.org/wp-content/uploads/2021/07/2014_0402_NGA-Garden-to-Table.pdf)) counts 42 million
food-gardening households, 35 percent of all US households, in 2013. Choosing the ripe fruit, parting it at the knuckle
without dragging the truss, and holding a soft fruit without bruising it are what every one of those gardeners does at
harvest time, and what greenhouse pickers do all day.

## The numbers behind the rules

- Squeeze. Guo et al., "Research on the Bionic Flexible End-Effector Based on Tomato Harvesting", *Journal of Sensors*
  2022, 2564952 ([doi:10.1155/2022/2564952](https://doi.org/10.1155/2022/2564952)), pressed ripe tomatoes between
  plates and conclude that "the clamping force of the flexible fingers should be less than 30 N to ensure that the
  tomato will not be obviously damaged in appearance". Kuta et al., "Tomato Fruit Quality as Affected by Ergonomic
  Conditions While Manually Harvested", *Agriculture* 2023, 13, 1831
  ([doi:10.3390/agriculture13091831](https://doi.org/10.3390/agriculture13091831)), pressed bright-red tomatoes until
  the skin failed, at 42 to 46 N on average ("critical compression values (about 42–45 N and 138–146 kPa) causing
  tissue damage"), while pickers' fingers squeezed them with 29 to 33 N. Both studies used larger fruit (6.6 to 9.3 cm
  and 7.4 cm across) than the 4.8 cm tomatoes here; the scorer's limit, 20 N, is below both thresholds with room for
  that. A YAM gripper closed fully on a tomato presses with its 50 N limit; the scripted demo aims its fingers 3 mm
  inside the fruit's faces, which presses with about 15 N (14.5 N at most in the demo run).
- Knuckle. Weng et al., "Tomato Pedicel Physical Characterization for Fruit-Pedicel Separation Tomato Harvesting
  Robot", *Agronomy* 2024, 14, 2274 ([doi:10.3390/agronomy14102274](https://doi.org/10.3390/agronomy14102274)), broke
  pedicels at the abscission zone with 22 to 42 N in three-point bending over a 12 mm span (a bending moment of about
  0.07 to 0.12 N m: force x span / 4) and needed 40 to 72 N to pull them apart along their length. The simulated
  knuckle snaps at 0.12 N m and never parts by pulling.

## Requirements

[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) (tested at commit `0059d02`) on
Isaac Lab-Arena 0.3.1 / Isaac Sim 6.1, on Linux with an NVIDIA RTX GPU. Its `setup.sh`, or the program's Brev
launchable, installs everything, including the Dream-1 policy. No third-party assets are used: the plant, the tomatoes
and the tray are built in the task file from boxes, cylinders and spheres, and the task needs no models from Arena's
object library.

## What is modelled and what is not

`yam_arena` task files place single boxes, cylinders and spheres, upright, and cannot join them. The plant, each
tomato and the tray are rigid bodies built from several shapes, and each tomato hangs from its pedicel on a joint, so
`bodykit.py` adds such bodies and joints: importing it hooks `yam_arena.scene` when the simulator starts, without
changing yam_arena, and gives `score()` each body's own pose. The plant and the tray are fixed in place.

- The knuckle is a ball joint that lets the tomato swing up to 35 degrees in any direction and breaks once it carries
  more than 0.12 N m at that limit.
- Pulled along its stem, the knuckle never parts (real knuckles need 40 to 72 N). The pedicel stretches on a spring
  instead (1,000 N/m), and 6 mm of stretch, a 6 N pull, counts as a torn truss. The simulated truss is rigid and cannot
  tear, so this rule stands in for what a real trial checks by inspecting the truss (below); it is a modelling choice,
  not a measured tearing force, and it is stricter than the real check (6 N is about twice the push at the fruit that
  snaps the knuckle by bending).
- The tomatoes are rigid spheres: skin, flesh, bruising and juice are not simulated. The squeeze is measured instead:
  every control step (20 times a second) the scorer reads from PhysX the contact force with which each link of either
  arm (fingers, gripper body, wrist and the rest) presses on each tomato, and a tomato pressed with more than 20 N at
  any time counts as crushed for the rest of the run. The rule sees pushes by any part of either arm, not only the
  grip, but only at the moments it reads (a squeeze shorter than 50 ms can be missed).
- Rolling resistance is not simulated (a tomato's spin is damped instead).

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` has held for 1 s, otherwise the score at the end of the run.

- A tomato is off the truss when its knuckle has moved 4 mm or more sideways from the end of its pedicel. It is in the
  tray when it is off, its centre lies inside the tray's walls and its lowest point is within 3 mm of the tray floor
  (resting on it, not held above it).
- Crushed: pressed with more than 20 N by any link of either arm at any time (above). A crushed red tomato earns
  nothing for the rest of the run, wherever it ends up.
- The score is 0 for the rest of the run if the green tomato comes off the truss or is crushed, a tomato is pulled
  6 mm along its stem (the truss counts as torn), or a tomato falls onto the table outside the tray.
- Otherwise each red tomato earns the best it has reached: 0.25 once off the truss, 0.5 once off and in the tray; the
  score is their sum, at most 0.75.
- Full score: both red tomatoes in the tray, neither crushed, the green one still on the truss, both grippers 5 cm or
  more from every tomato (from the point between each gripper's finger pads), the red tomatoes still (moving less than
  1 mm) for 1 s.

`-E object_jitter=0` keeps every object exactly where the task file puts it, so every trial starts from the same
layout.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=pick_ripe_tomatoes_at_the_knuckle.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Pick the ripe tomatoes off the truss and put them in the tray." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/tomato-dream
```

The scripted demo, which reaches full score (the right gripper comes at each red tomato from the front, level, fingers
either side of it, closes on it just enough to hold it, swings it up and toward itself around the knuckle until the
knuckle snaps, draws it back and down over the tray and lets it go; the left arm does not move):

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

Environment variables (set in front of the command):

- `TOMATO_DEBUG=1` prints, every 10 steps, where each tomato is, whether it is on the truss, off it, in the tray or
  crushed, the squeeze on each tomato now and its peak so far, the force of each gripper's two finger drives, how far
  the grippers are from the tomatoes, and the score so far; and a line when a tomato is crushed.
- `TOMATO_PULL=1` (the wrong way) runs the demo with a pull instead of the bend: the gripper closes on the first red
  tomato and pulls it straight down along its stem. The knuckle holds, the stem stretches past 6 mm, the truss counts
  as torn and the run scores 0, with the tomato still on the truss.
- `TOMATO_CRUSH=1` runs the demo with the gripper closed fully on the first red tomato (it presses with its 50 N
  limit) and otherwise the same motion: both tomatoes end resting in the tray with the gripper clear, but the first
  was crushed (squeezed with up to 50.2 N) and earns nothing, so the run scores 0.5, the second tomato's credit (the
  first version of this task scored it 1.0). `TOMATO_CRUSH=2` crushes both (50.2 N each) and scores 0, with both
  resting in the tray.

## Scoring a real trial

Score the same stages from the overhead and wrist videos, then check the fruit and the plant right after the run:

- Each tomato: photograph it all round. A red tomato with split skin, a dent or flat spot, or juice showing earns
  nothing; the same damage on the green one scores 0 for the trial.
- The truss: inspect and photograph it. A torn, split or kinked peduncle or pedicel, a calyx left on the truss (pulled
  off its fruit), or the green tomato off or hanging from a torn pedicel scores 0 for the trial. A red tomato counts as
  picked only if it came off at the knuckle, with its calyx and a short stub of stalk on it.
- In the tray: off the truss and resting on the tray floor inside its walls, not held. A tomato dropped on the table
  scores 0 for the trial.
- Let go: both grippers 5 cm or more from every tomato (tape measure), the tomatoes still for 1 s (video).

## Runtime per trial

On one RTX PRO 6000 (Blackwell) shared with other jobs: 1.5 to 6 minutes for the scripted demo at 320 x 176 (575 steps,
29 s of simulated time; 94 s and 337 s in two runs under different loads), and about 20 minutes for a Dream-1 trial of
2,400 steps (1,191 s). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed and the scripted demo is deterministic.

## Building it for real

A truss of a cocktail-plum or medium tomato variety (fruit 4.5 to 5 cm across) with two red fruit and one green, cut
with a length of stem and clamped to a stand so that it hangs 15 to 17 cm above the table, and a 13 x 26 cm black
plastic harvest tray about 2.5 cm deep under it. Each trial uses a fresh truss.
