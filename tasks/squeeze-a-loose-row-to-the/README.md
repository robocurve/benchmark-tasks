# Squeeze a loose row to the centre mark

Five light crates stand in a loose row across the table. The two arms push the two ends inward at the same time
until the row is closed up and centred on a taped mark. The scene, scorer and scripted demo are in
`squeeze_a_loose_row_to_the.py`.

## Run a trial

Run from the repository root. Dream-1 (needs the Dream credentials set up as in the simulator's README):

    inspect-robots run --embodiment yam_arena --policy dream \
      -E task=tasks/squeeze-a-loose-row-to-the/squeeze_a_loose_row_to_the.py -E object_jitter=0 \
      -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "Push the two ends of the loose row of five grey crates inward until the row is closed up, with every gap between neighbouring crates at most 1.5 cm, the middle of the row (halfway between its two outer faces) within 1.5 cm of the yellow mark, and every crate upright and within 2 cm of the row's original line." \
      --scorer task_score --store-frames --max-steps 2400 --log-dir runs/dream

Scripted demo (same flags, `--policy yam_scripted --max-steps 400`). The score is printed as `task_score`.
Render videos with `inspect-robots video <log>.json`.

## Scoring

`score()` reads only object poses at the end of the run. Any credit needs all five crates upright, on the table
and within 2 cm of the row line (x 35 cm). Gap = distance between facing faces of neighbouring crates; centre =
halfway between the two outer faces. 1.0: every gap at most 1.5 cm and centre within 1.5 cm of the mark.
0.5: every gap at most 3 cm and centre within 4 cm. Otherwise 0. A tape measure is enough.

## Layout design

- The row starts at pitch 12.5, 12.5, 13.0, 12.0 cm, centred on the midline. The mark is 5 cm to the left of the
  midline, so the obvious guess (close the row symmetrically about the midline) is 5 cm off and scores 0, and
  the two arms must push different distances (17 cm from the right end, 8 cm from the left end).
- Reach (offline inverse kinematics, 0.1 to 0.2 mm error for every demo waypoint): the left arm cannot reach the
  right end crate (at least 100 mm short at its start), and the right arm cannot reach the left end crate (at
  least 99 mm short). One arm alone cannot do the job.

## Shortcut checks (scripted, 300 steps, one sim each)

| Variant | Score |
|---|---|
| Do nothing | 0 |
| Left arm only (pushes its end until the row is closed) | 0 |
| Right arm only | 0 |
| Both arms, half the distance (row not closed) | 0 |
| Both arms, closed up symmetrically about the midline | 0 |

## Runtime and seeds

- Dream-1: 2,400 steps (2 minutes of simulated time), about 5 minutes of wall time on one GPU.
- Scripted demo: 322 steps (16.1 s of simulated time), about 1.5 minutes of wall time including start-up.
- No randomness: run with `-E object_jitter=0`, so every object starts exactly where the file puts it. No seeds.

## Results

Dream-1: 0.0. Demo: 1.0.
