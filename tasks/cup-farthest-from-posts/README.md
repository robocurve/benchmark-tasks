# Place a cup farthest from every post

Instruction: "Put the cup inside the green taped box so that it is as far as possible from every post."

The scene, the scorer (`score()`) and a scripted demo (`demo()`) are all in `cup_farthest_from_posts.py`.
Loading the file runs a 1 cm grid search over the allowed area (cup centre at least 3.5 cm inside the tape) and
asserts that the best spot is unique (the next best is at least 0.5 cm worse), that it reaches the 10 cm full-score
distance, that the box centre scores 0 and that no corner of the allowed area scores 1.0. To print the best grid
spot, the corners, the box centre and the optimum on a 1 mm grid:

    python cup_farthest_from_posts.py

The exact optimum is the point equally far from posts A, B and C: 35 cm forward of the arm-base line and 7.54 cm
right of the midline, 11.96 cm from each of the three.

## Run and score a trial

From the repository root, with the `yam_arena` embodiment installed:

    inspect-robots run --embodiment yam_arena --policy dream \
      --instruction "Put the cup inside the green taped box so that it is as far as possible from every post." \
      -E task=tasks/cup-farthest-from-posts/cup_farthest_from_posts.py \
      -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --scorer task_score --store-frames --no-prompt --max-steps 2400

The scripted demo is the same command with `--policy yam_scripted`. The score is printed as `task_score`.
Render the camera videos with `inspect-robots video <log>.json`.

## Runtime and seeds

- A full 2,400-step Dream-1 trial takes about 8 to 15 minutes of wall time on one L40S GPU shared with another
  simulation; the scripted demo takes about 1.5 minutes.
- No seeds are needed: `object_jitter=0` gives the same starting layout every trial, and the scripted demo is
  deterministic.

## Results

| Run | Nearest post | Score |
| --- | --- | --- |
| Demo (right arm carries the cup to the best spot) | 11.9 cm | 1.0 |
| Dream-1, 2,400 steps (the cup never moves) | | 0.0 |
| Do nothing (cup stays outside the box) | | 0.0 |
| Cup placed at the box centre (hits post C and tips) | 4.7 cm | 0.0 |
| Cup between the best spot and the box centre | 9.4 cm | 0.5 |
| Near corner, right end (hits post C, ends outside the allowed area) | 4.7 cm | 0.0 |
| Far corner, right end (hits post C and tips) | 8.6 cm | 0.0 |
| Near corner, midline end | 9.1 cm | 0.5 |
| Far corner, midline end | 9.0 cm | 0.5 |
