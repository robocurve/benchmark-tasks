# Set up a stroke patient's bedside on his working side

A `yam_arena` task file for Isaac Lab Arena: `stroke_bedside.py` defines the objects, the scorer
(`success()` and `score()`) and the scripted full-score demo.

## Run a trial

On the YAM launchable (Isaac Lab Arena venv), from the folder holding `stroke_bedside.py`:

```bash
INSTRUCTION="Sam had a stroke and can't use his left arm. Put his call button, tissues and TV remote where he can reach them."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/stroke_bedside.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score \
  --max-steps 2400 --store-frames --log-dir runs/dream

# Scripted full-score demo
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/stroke_bedside.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score \
  --max-steps 2400 --store-frames --log-dir runs/demo
```

`--scorer task_score` reports the score: 1.0 on success, otherwise the partial credit from `score()` at
the end of the episode.

## Runtime and seeds

- Run limit: 2,400 control steps (2 minutes at 20 Hz). The scripted demo finishes in about 48 s of
  simulated time.
- Fixed layout: `-E object_jitter=0`, so no seed is needed; every trial starts from the same poses.
