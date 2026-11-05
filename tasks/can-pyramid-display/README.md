# Build a three-tier can pyramid on a shop display mat

A task file for the `yam_arena` embodiment ([yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena), Isaac Lab-Arena 0.3.1) on the station scene: [can_pyramid_display.py](can_pyramid_display.py).

Six cans go from the delivery tray onto the display mat as a 3-2-1 pyramid. Each upper can rests across two below it, so the bottom row has to sit close enough to support both, and each can has to be set down upright and centred without knocking the tiers already built. `score()` counts upright cans tier by tier from the mat and gives 0 if any can ends tipped over.

## Run a trial

Dream-1:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/can-pyramid-display/can_pyramid_display.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Build a pyramid display of the six cans on the green mat: three in the bottom row, two on those, and one on top." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/pyramid-dream
```

Scripted demo (scores 1.0): the same command with `--policy yam_scripted --max-steps 1300`. The left arm builds the pyramid can by can, the bottom row 7 cm apart, gripping each can with the fingers closing front to back so they never touch a neighbour, then moves clear of the camera's view.

The run prints `task_score` at the end: 1.0 for the full pyramid, 0.5 for the bottom two tiers, 0.25 for the bottom row alone, and 0 if any can is tipped over. The task defines no `success()`, so the trial is scored on where the cans end up.

## Runtime and seeds

- Demo: 1,253 steps (63 s simulated), about two minutes on an L40S.
- Dream-1: 2,400 steps (2 minutes simulated), about 6 minutes on an L40S.
- `object_jitter=0` keeps the layout fixed, so no seed is needed. The scripted demo is open loop and deterministic.
