# Take one bottle from a row of five without knocking any over

A task file for the `yam_arena` embodiment ([yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena), Isaac Lab-Arena 0.3.1) on the station scene: [crowded_shelf_bottle.py](crowded_shelf_bottle.py).

## Run a trial

Dream-1:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/crowded-shelf-bottle/crowded_shelf_bottle.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put the red bottle in the tray without knocking over any of the other bottles." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/crowded-shelf-dream
```

Scripted demo (scores 1.0): the same command with `--policy yam_scripted --max-steps 600`.

The run prints `task_score` at the end. `score()` in the task file returns 0 if any bottle ever tilts past 45 degrees, 1.0 for the red bottle upright in the tray with the amber bottles undisturbed, and partial credit otherwise; `success()` is a score of 1.0.

A fall or disturbance counts for the whole trial, even if the bottle is stood back up. The task module stays loaded across trials, so `score()` clears its record only on a reset, which it recognises by the bottles jumping back to their marks within one control step (more than 3 cm or 30 degrees, then all resting on their marks). An arm restoring a bottle moves it gradually and never triggers this. Checked with a pure-Python replay (a bottle tipped to 90 degrees and stood back up, then the red bottle placed, scores 0; the next trial after a reset scores 1.0) and with the scripted demo over three epochs (each reset detected, every epoch 1.0).

## Runtime and seeds

- Demo: 229 steps (11.5 s simulated), under a minute on an L40S.
- Dream-1: 2,400 steps (2 minutes simulated), about 5 minutes on an L40S.
- `object_jitter=0` keeps the layout fixed, so no seed is needed. The scripted demo is open loop and deterministic.
