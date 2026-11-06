# Set a one-handed diner's place with the right dish on her non-slip mat

A `yam_arena` task file for Isaac Lab Arena: `one_handed_meal.py` defines the objects, the scorer
(`success()` and `score()`) and the scripted full-score demo.

## Run a trial

On the YAM launchable (Isaac Lab Arena venv), from the folder holding `one_handed_meal.py`:

```bash
INSTRUCTION="Ana can only use her right hand. Set out her cup and whichever of the two dishes is easier for her to eat from."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/one_handed_meal.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score \
  --max-steps 2400 --store-frames --log-dir runs/dream

# Scripted full-score demo
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/one_handed_meal.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score \
  --max-steps 2400 --store-frames --log-dir runs/demo
```

`--scorer task_score` reports the score: 1.0 on success, otherwise the partial credit from `score()` at
the end of the episode.

## Runtime and seeds

- Run limit: 2,400 control steps (2 minutes at 20 Hz). The scripted demo finishes in about 35 s of
  simulated time.
- Fixed layout: `-E object_jitter=0`, so no seed is needed; every trial starts from the same poses.
