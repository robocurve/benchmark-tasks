# Add a fresh pad on top of a soaked-through dressing and hold it in place

Two files for the `yam_arena` embodiment (Isaac Lab-Arena, bimanual YAM, `station` scene). `task.py` builds the scene,
scores it and holds the scripted demo; its docstrings explain every rule. `yamkit.py` spawns the modeled props (visible
meshes with simple box colliders underneath) and gives the scorer the grippers' positions. It patches `yam_arena.scene`
in memory at import and changes no files.

## Run a trial

Scripted demo (reaches full score):

    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=tasks/soaked-dressing-add-on-top/task.py \
      -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "The dressing on the training arm has soaked through. Put on a fresh pad and hold it in place." --scorer task_score --max-steps 600

Model run (Dream-1): the same command with `--policy dream --max-steps 2400`.

Render the videos of a run with `inspect-robots video <log.json>`.

## Scoring

`--scorer task_score` gives 1.0 once `success()` has held for 1 s; otherwise the trial scores `score()` at the
end of the run. The rules and possible scores are in `task.json` (`scoring`) and in the docstrings of
`success()` and `score()`.

The scorer remembers what happened during the run: `success()` and `score()` are called after every control
step, and lifting or moving the soaked pad sets a flag that nothing later clears. The hold is timed from the
gripper positions that `yamkit.py` adds. A reset (every free object jumping back to its start at rest) starts a new
history.

## Runtime and seeds

- Demo: 284 control steps (14 s simulated).
- Model run limit: 2,400 steps (2 minutes at 20 Hz).
- Seed 0, one fixed layout (`object_jitter=0`).
