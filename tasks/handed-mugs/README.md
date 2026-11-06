# Set each person's mug handle-first for their dominant hand

A task file for the `yam_arena` embodiment ([yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena), Isaac Lab-Arena 0.3.1) on the station scene: [handed_mugs.py](handed_mugs.py). The mugs are `mug_ycb_robolab` from Arena's object library.

## Run a trial

Dream-1:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/handed-mugs/handed_mugs.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The person at the green placemat is left-handed and the person at the yellow placemat is right-handed. Set a mug on each placemat so each person can pick it up by the handle." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/mugs-dream
```

Scripted demo (scores 1.0): the same command with `--policy yam_scripted --max-steps 600`. Each arm turns its mug 90 degrees on the way, in opposite directions.

The run prints `task_score` at the end. `score()` in the task file reads each handle's direction from the mug's orientation: 0.5 per placemat with a mug turned to that person's dominant hand, 0.1 per placemat with a mug facing elsewhere, 0 if a mug tipped; `success()` is a score of 1.0.

## Runtime and seeds

- Demo: 202 steps (10 s simulated), about a minute on an L40S.
- Dream-1: 2,400 steps (2 minutes simulated), about 8 minutes on an L40S.
- `object_jitter=0` keeps the layout fixed, so no seed is needed. The scripted demo is open loop and deterministic.
