# Set a blind diner's water and salt at the clock positions she was told

A task file for the `yam_arena` embodiment ([yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena), Isaac Lab-Arena 0.3.1) on the station scene: [blind_diner_clock.py](blind_diner_clock.py).

Mia is blind and sits at the far end of the table, facing the robot; her folded cane marks her seat. Clock positions are read from her seat, so her 12 o'clock is the side of the plate nearest the robot and her 3 o'clock is on the robot's left. `score()` measures each item's angle around the plate centre in her frame.

## Run a trial

Dream-1:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/blind-diner-clock/blind_diner_clock.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Mia is blind. Put her water at 2 o'clock and her salt at 10 o'clock around her plate, as she was told." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/blind-diner-dream
```

Scripted demo (scores 1.0): the same command with `--policy yam_scripted --max-steps 600`. The left arm sets the water at Mia's 2 o'clock, then the right arm sets the salt at her 10 o'clock, and each moves clear of the camera's view.

The run prints `task_score` at the end: 0.5 for each item upright within 15 degrees of its clock position and 8 to 25 cm from the plate centre. The task defines no `success()`, so the trial is scored on where the items end up.

## Runtime and seeds

- Demo: 525 steps (26 s simulated), about a minute on an L40S.
- Dream-1: 2,400 steps (2 minutes simulated), about 6 minutes on an L40S.
- `object_jitter=0` keeps the layout fixed, so no seed is needed. The scripted demo is open loop and deterministic.
