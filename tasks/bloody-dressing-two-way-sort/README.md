# Sort dressing-change waste: the blood-soaked dressing to biohazard, the packaging to trash

One task file, `task.py`, for the `yam_arena` embodiment (Isaac Lab-Arena, bimanual YAM, `station` scene).
`task.py` builds the scene, scores it and holds the scripted demo; its docstrings explain every rule.

## Run a trial

Scripted demo (reaches full score):

    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=tasks/bloody-dressing-two-way-sort/task.py \
      -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "Throw away the waste from the dressing change." --scorer task_score --max-steps 900

Model run (Dream-1): the same command with `--policy dream --max-steps 2400`.

Render the videos of a run with `inspect-robots video <log.json>`.

## Scoring

`--scorer task_score` gives 1.0 once `success()` has held for 1 s; otherwise the trial scores `score()` at the
end of the run. The rules and possible scores are in `task.json` (`scoring`) and in the docstrings of
`success()` and `score()`.

## Runtime and seeds

- Demo: 504 control steps (25.2 s simulated).
- Model run limit: 2,400 steps (2 minutes at 20 Hz).
- Seed 0, one fixed layout (`object_jitter=0`).
