# Hide the present behind the box from Mia's point of view, not the robot's

A task file for the `yam_arena` embodiment ([yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena), Isaac Lab-Arena 0.3.1) on the station scene: [hide_from_view.py](hide_from_view.py).

Mia sits at the green placemat at the far end of the table, facing the robot. "Behind the box" from the robot's side, or on top of it, is in her plain view; the present has to go on the robot's side of the box, where the box blocks her line of sight. `score()` checks this with straight lines from the present's top corners to her eye point.

## Run a trial

Dream-1:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/hide-from-view/hide_from_view.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Hide the present behind the cardboard box so Mia, sitting at the green placemat, can't see it." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/hide-dream
```

Scripted demo (scores 1.0): the same command with `--policy yam_scripted --max-steps 600`. The left arm sets the present down between the box and the robot, then moves clear of the camera's view.

The run prints `task_score` at the end: 1.0 with all four top corners hidden from Mia's eyes, 0.5 with only the top centre hidden, 0 if she can see it or the box was moved. The task defines no `success()`, so the trial is scored on where the present ends up.

## Runtime and seeds

- Demo: 292 steps (15 s simulated), about a minute on an L40S.
- Dream-1: 2,400 steps (2 minutes simulated), about 6 minutes on an L40S.
- `object_jitter=0` keeps the layout fixed, so no seed is needed. The scripted demo is open loop and deterministic.
