# Put a cup on a tray

A `yam_arena` task file for Isaac Lab Arena: `cup_on_tray.py` defines the objects, the scorer
(`success()` and `score()`) and the scripted full-score demo.

## Run a trial

On the YAM launchable (Isaac Lab Arena venv), with the task file at `~/cup_on_tray.py`:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=~/cup_on_tray.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "put the cup on the tray" --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/cup-dream

# Scripted full-score demo
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=~/cup_on_tray.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "put the cup on the tray" --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/cup-demo
```

`--scorer task_score` reports the score: 1.0 on success, otherwise the partial credit from `score()` at
the end of the episode.

## Runtime and seeds

- Run limit: 2,400 control steps (2 minutes at 20 Hz). The scripted demo runs at half speed and finishes in about
  20 s of simulated time.
- Fixed layout: `-E object_jitter=0`, so no seed is needed; every trial starts from the same poses.
