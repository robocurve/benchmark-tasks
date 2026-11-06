# Switch on a furnace variac at 0 V, then ramp it to 60 V

A variac feeds a tube furnace. One arm switches it on at 0 V; the other turns its knob to 60 V, reading the dial, never faster than
10 V per second and never above 65 V. The knob travels 270 degrees, so it takes a regrip.

## Files

- `task.py`: scene, scoring (it records the dial reading at every control step) and scripted demo.
- `labkit.py`: parts built from several shapes with hinges and slides (the knob and the power rocker are hinges with joint friction). It
  hooks `yam_arena.scene` and the embodiment from the task's import, so no change to yam_arena is needed.
- `arena_assets_fix.py`: imported by `labkit.py`; points Arena library assets at NVIDIA's production bucket. This task
  uses no library assets.

## Run a trial

Inside the Isaac Lab-Arena environment with `yam-isaac-lab-arena` installed (the Robocurve Brev launchable):

    inspect-robots run --embodiment yam_arena --policy dream \
        -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 \
        --instruction "$(python -c 'import json;print(json.load(open("task.json"))["instruction"])')" \
        --scorer task_score --max-steps 1200 --store-frames --log-dir logs

Scripted demo: the same command with `--policy yam_scripted` (about 34 s). Run one trial per process: the ramp
record is kept in the task module and cleared at each reset.

## Runtime and seeds

- 1,200 control steps (60 s of simulated time) per trial; about 3 minutes of wall time on one L40S with frames stored.
- One fixed starting layout: run with `-E object_jitter=0`. No other randomness.
