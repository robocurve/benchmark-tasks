# Pick the one domino push that topples the red tile but spares the vase

A task file for the `yam_arena` embodiment ([yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena), Isaac Lab-Arena 0.3.1) on the station scene: [domino_one_push.py](domino_one_push.py).

The dominoes form a Y: a trunk ends at a wide splitter tile, which starts two branches, one ending at the red tile and one at the vase. Pushing the trunk topples both branches and the vase. The robot has to predict the chain reaction and push a tile on the red branch, past the split. The red tile and the vase stand out of reach, so the only way to topple the red tile is the chain.

## Run a trial

Dream-1:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/domino-one-push/domino_one_push.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Make the red domino fall by pushing just one tile, and keep the vase standing." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/domino-dream
```

Scripted demo (scores 1.0): the same command with `--policy yam_scripted -P hold_s=6 --max-steps 600`. The right arm presses the top edge of the second tile on the red branch forward; `hold_s=6` lets the chain finish before the trial ends.

The run prints `task_score` at the end. `score()` in the task file returns 0 if the vase was ever knocked, 1.0 if the red tile fell, 0.5 if only part of the red branch fell. The task defines no `success()`, so the trial runs on and a vase knocked late still counts.

## Runtime and seeds

- Demo: 270 steps (14 s simulated), about a minute on an L40S.
- Dream-1: 2,400 steps (2 minutes simulated), about 6 minutes on an L40S.
- `object_jitter=0` keeps the layout fixed, so no seed is needed; jitter would break the chain's 6 cm spacing. The scripted demo is open loop and deterministic.
