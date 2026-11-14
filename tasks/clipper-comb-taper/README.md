# Clipper-over-comb taper

A barber's training head faces away from the robot. Twelve tufts of hair on the back of the head (three columns, four rows from nape to crown) all start 35 mm long. The robot tapers them to 8, 12, 16 and 22 mm, row by row, with a guardless clipper run over a comb, then puts the comb back in its rack and the clipper back on its holder.

## What the task tests

- **Two hands, two roles.** One hand holds the comb flat in the hair at the distance from the scalp that sets the length; the other runs the clipper along the comb's back. A clipper on its own either misses the hair, which lies flat, or shaves it to the scalp.
- **A curved head.** A comb held flat across the three columns leaves the side columns 5 to 7 mm too long, so the comb has to follow the head column by column.
- **Irreversible cuts.** A tuft under 5 mm is a bald patch and the trial scores 0. A tuft counts as right from 2 mm short to 3 mm long.
- **Safety.** A tool touching an ear pad scores 0.

## Running a trial

Put `clipper_comb.py` and `hingekit.py` in one folder, then:

```bash
inspect-robots run --embodiment yam_arena --policy <policy> \
  -E task=$PWD/clipper_comb.py -E object_jitter=0 -E control_hz=20 \
  --instruction "Taper the back of the training head with the clipper over the comb: the bottom row of hair 8 mm, then 12 mm, 16 mm and 22 mm up to the top row. Then put the comb and the clipper back where they were." \
  --scorer task_score --max-steps 3600
```

- **Policy:** `--policy yam_scripted` runs the scripted demo (`demo()` in the task file).
- **Offline check:** `python clipper_comb.py` checks the demo plan and its IK without a GPU.

## Scoring

The scoring is `score()` in `clipper_comb.py`, evaluated at the end of the trial. Cuts are tracked every control step by `monitor()`. A tuft's length is how far it reaches from its root when held straight out from the scalp, and each tuft earns a credit:

| Tuft | Credit |
|---|---|
| Right: from 2 mm shorter to 3 mm longer than its row's length | 1 |
| Cut (at least 1 mm shorter than 35 mm) but still too long | 0.25 |
| Uncut, or too short | 0 |

| Score | Condition |
|---|---|
| 0.0 | Any tuft under 5 mm (a bald patch), or a tool came within 1 cm of an ear pad |
| 1.0 | All 12 tufts right, and both tools put back (the comb resting in or beside its rack, the clipper lying flat on or beside its holder) |
| 0.9 × average credit | Everything else (0 to 0.9, rounded to 0.001) |

Example scores from scripted strategies (offline):

| Strategy | Score |
|---|---|
| Nothing done | 0 |
| All tufts right, tools not put back | 0.9 |
| Comb held flat across the head, not following its curve | 0.413 |
| Only the middle column, done right | 0.337 |
| The same length on every row (no taper) | 0.281 |
| Everything cut 5 mm too long | 0.225 |
| Clipper without the comb | 0 |

In a real trial, measure each tuft with a ruler. "Touching an ear" is judged directly.

## Runtime and seeds

- **Runtime:** the scripted demo reaches full score at about 135 s (about 2,700 steps), within the 3,600-step limit.
- **Seeds:** the layout is fixed, and `object_jitter=0` was used for all runs. The scoring is deterministic.

## Simulation notes

- **Hair.** Hair is not simulated. Each tuft is a drawn lock of five thin strands that nothing collides with. It hangs down unless the comb's plate is in it, and is then drawn standing straight out through the comb. Its length (measured straight out from the root) is shortened by rule from where the clipper's blade face and the comb's plate cross that line. The comb's teeth shield the hair inside them. The locks are redrawn every step (`kit.on_kinematic` in `hingekit.py`); the drooping is display only and does not change the scoring.
- **Head.** The head is stacked cylinders, so its curve up the back is stepped.
- **Preview-only settings.** The `PB_*` names are used only by an offline pybullet preview and do nothing in Isaac.
- **Assets.** There are no external assets: every object is built from boxes and cylinders in the task file.
