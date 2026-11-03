# Move drinks away from a keyboard and phone

A task file for the `yam_arena` embodiment ([yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena), Isaac Lab-Arena 0.3.1) on the station scene: [drinks_away_from_electronics.py](drinks_away_from_electronics.py). The keyboard, phone, mug and cup are models from Arena's object library.

## Run a trial

Dream-1:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/drinks-away-from-electronics/drinks_away_from_electronics.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Move both drinks at least 15 cm away from the keyboard and the phone, without moving the keyboard or the phone." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/drinks-dream
```

Scripted demo (scores 1.0): the same command with `--policy yam_scripted --max-steps 600`. Both arms move a drink out to the side at the same time.

The run prints `task_score` at the end. `score()` in the task file returns 0 if a drink ever tilts past 30 degrees or a device is moved, otherwise 0.5 per drink at least 15 cm from both devices; `success()` is a score of 1.0.

## Runtime and seeds

- Demo: 197 steps (10 s simulated), about a minute on an L40S.
- Dream-1: 2,400 steps (2 minutes simulated), about 5 minutes on an L40S.
- `object_jitter=0` keeps the layout fixed, so no seed is needed. The scripted demo is open loop and deterministic.
