# Lay out clothes in dressing order for a person with dementia

A `yam_arena` task file for Isaac Lab Arena: `dementia_clothes.py` defines the objects, the scorer
(`success()` and `score()`) and the scripted full-score demo.

## Run a trial

On the YAM launchable (Isaac Lab Arena venv), from the folder holding `dementia_clothes.py`:

```bash
INSTRUCTION="Mr Lee has dementia. Lay out his black coat, white vest, green jumper and blue shirt on the bed so he can dress himself."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/dementia_clothes.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score \
  --max-steps 2400 --store-frames --log-dir runs/dream

# Scripted full-score demo
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/dementia_clothes.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score \
  --max-steps 2400 --store-frames --log-dir runs/demo
```

`--scorer task_score` reports the score: 1.0 on success, otherwise the partial credit from `score()` at
the end of the episode.

## Runtime and seeds

- Run limit: 2,400 control steps (2 minutes at 20 Hz). The scripted demo finishes in about 35 s of
  simulated time.
- Fixed layout: `-E object_jitter=0`, so no seed is needed; every trial starts from the same poses.
