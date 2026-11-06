# Mark the map with a pin a red-green colour-blind colleague can tell from the red one

A task file for the `yam_arena` embodiment ([yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena), Isaac Lab-Arena 0.3.1) on the station scene: [colourblind_map_pin.py](colourblind_map_pin.py).

Mia's red pin marks the first stop on the map. To someone with red-green colour blindness, the green, orange and brown pins in the tray look like the red one, so only the blue pin works for the second stop. `score()` gives 0 if any of the confusable pins ends on the cross.

## Run a trial

Dream-1:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/colourblind-map-pin/colourblind_map_pin.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Mia is red-green colour-blind. Mark the second stop, the white cross, with a pin she can tell apart from her red one." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/colourblind-dream
```

Scripted demo (scores 1.0): the same command with `--policy yam_scripted --max-steps 500`. The left arm stands the blue pin on the white cross, then moves clear of the camera's view.

The run prints `task_score` at the end: 1.0 with the blue pin within 3 cm of the cross, 0.5 within 10 cm, and 0 if a green, orange or brown pin is on the cross or Mia's red pin was moved. The task defines no `success()`, so the trial is scored on where the pins end up.

## Runtime and seeds

- Demo: 293 steps (15 s simulated), under a minute on an L40S.
- Dream-1: 2,400 steps (2 minutes simulated), about 6 minutes on an L40S.
- `object_jitter=0` keeps the layout fixed, so no seed is needed. The scripted demo is open loop and deterministic.
