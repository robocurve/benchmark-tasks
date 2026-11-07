# Flame the inoculating loop before and after streaking a plate

One task file, `task.py`, for the `yam_arena` embodiment (Isaac Lab-Arena, bimanual YAM, `station` scene).
`task.py` builds the scene, scores it and holds the scripted demo; its docstrings explain every rule.

## Run a trial

Scripted demo (reaches full score):

    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=tasks/flame-the-loop/task.py \
      -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "Streak the sample onto the fresh plate." --scorer task_score --max-steps 800

Model run (Dream-1): the same command with `--policy dream --max-steps 2400`.

Render the videos of a run with `inspect-robots video <log.json>`.

## Scoring

`--scorer task_score` gives 1.0 once `success()` has held for 1 s; otherwise the trial scores `score()` at the
end of the run. The rules and possible scores are in `task.json` (`scoring`) and in the docstrings of
`success()` and `score()`.

The scorer remembers what happened during the run: `success()` and `score()` are called after every control
step, and it follows the loop's tip: flamed, carrying bacteria, or neither. A breach of aseptic technique sets a
flag that nothing later clears. A reset (the loop jumping back to its start at rest) starts a new history.

## Runtime and seeds

- Demo: about 630 control steps (32 s simulated).
- Model run limit: 2,400 steps (2 minutes at 20 Hz).
- Seed 0, one fixed layout (`object_jitter=0`).
