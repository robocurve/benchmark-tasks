# Clamp a printed part in a vise at a marked height

A small drill-press vise, open, and a 3D-printed block with a line marking its clamp height. One arm holds the part
upright between the jaws with the line level with the jaw tops; the other turns the vise's knob until the jaws grip it.
Let go early and the part drops onto the vise's bed, 12 mm too deep.

## Files

- `task.py`: scene, scoring and scripted demo (yam_arena task file). Every control step, an `on_step` hook sets the
  moving jaw's drive target from the knob's angle (the screw), capped 3 mm past the jaw so the clamping force stays
  under 30 N.
- `labkit.py`: parts built from several shapes, with hinges and slides (the jaw is a slide, the knob a damped hinge).
  It hooks `yam_arena.scene` and the embodiment from the task's import, so no change to yam_arena is needed.
- `arena_assets_fix.py`: imported by `labkit.py`; points Arena library assets at NVIDIA's production bucket. This task
  uses no library assets.

## Run a trial

Inside the Isaac Lab-Arena environment with `yam-isaac-lab-arena` installed (the Robocurve Brev launchable):

    inspect-robots run --embodiment yam_arena --policy dream \
        -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 \
        --instruction "$(python -c 'import json;print(json.load(open("task.json"))["instruction"])')" \
        --scorer task_score --max-steps 2400 --store-frames --log-dir logs

Scripted demo: the same command with `--policy yam_scripted` (about 37 s of motion).

The score is `task_score` in the log, computed by `score()` in `task.py` from the final state.

## Runtime and seeds

- 2,400 control steps (2 minutes of simulated time) per trial; about 6 minutes of wall time on one L40S with frames
  stored at 640 x 352 (estimated from similar tasks; not yet measured).
- One fixed starting layout: run with `-E object_jitter=0`. No other randomness.
