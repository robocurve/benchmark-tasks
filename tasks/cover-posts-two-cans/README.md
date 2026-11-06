# Site two cans to cover five posts

Two arms each stand one soup can inside their own taped zone so that all five fixed grey posts are within
18 cm of one of the cans. The scene, scorer and scripted demo are in `cover_posts_two_cans.py`.

## Run a trial

Run from the repository root. Dream-1 (needs the Dream credentials set up as in the simulator's README):

    inspect-robots run --embodiment yam_arena --policy dream \
      -E task=tasks/cover-posts-two-cans/cover_posts_two_cans.py -E object_jitter=0 \
      -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "Stand the two cans on the table so that every grey post is within 18 cm of one of the cans, and each can is at least 12 cm from every post, with all distances measured between centres, and each can inside its own arm's taped zone." \
      --scorer task_score --store-frames --max-steps 2400 --log-dir runs/dream

Scripted demo (same flags, `--policy yam_scripted --max-steps 300`). The score is printed as `task_score`.
Render videos with `inspect-robots video <log>.json`.

## Scoring

`score()` reads only object poses at the end of the run. Conditions for any credit: one can has its centre inside
each zone, both cans upright and on the table, each can at least 12 cm from every post and from the other can.
Then 1.0 if every post is within 18 cm of its nearest can, 0.5 if exactly one is farther, else 0. Distances are
centre to centre, so a tape measure is enough.

## Layout design

The post layout was found by brute force over can positions on a 1 cm grid inside each zone (400 cells per zone):

- Full-score region: 17 cm2 for the left can and 19 cm2 for the right can, 4 to 5 percent of each zone.
  Placements within 1 cm of the best spot still score 1.0.
- The best spot is at least 1.5 cm from every constraint (12 cm keep-off, 18 cm cover, zone edge).
- The split of posts is 3 for the left can and 2 for the right, and posts 2 and 4 sit within 6 cm of the midline.
- Reach: each arm reaches every demo waypoint (inverse kinematics error 0.1 to 0.2 mm). The other arm
  cannot reach the zone (at least 13 mm short at its nearest corner, and 44 mm or more short at the two can targets).

## Shortcut checks (scripted, 300 steps)

| Variant | Score |
|---|---|
| Do nothing | 0 |
| One can only (left can moved to its target, right can left at its start) | 0.5 |
| Both cans at the zone centres | 0 |
| Both cans at the zone corners nearest the posts | 0 |

The one-can result is the intended partial credit: the left can covers its three posts, and the right can at its
start covers post 1 but not post 4, so exactly one post is uncovered.

## Runtime and seeds

- Dream-1: 2,400 steps (2 minutes of simulated time), about 8 to 12 minutes of wall time on one GPU.
- Scripted demo: 232 steps (11.6 s of simulated time), about 1 minute of wall time.
- No randomness: run with `-E object_jitter=0`, so every object starts exactly where the file puts it. No seeds.

## Results

Dream-1: 0.0. Demo: 1.0.
