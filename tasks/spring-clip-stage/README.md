# Clamp a chip carrier pin-1-aligned under a stage's spring clip

A chip carrier lies on the table next to an inspection stage. The stage has a spring clip: pressing its thumb tab
lifts the jaw. The carrier has to end up clamped under the jaw, centred on the stage's target square, chip up, with its pin-1 dot at the stage's red mark (it starts a quarter turn off). One arm
holds the clip open while the other slides the carrier in.

## Files

- `task.py`: scene, scoring and scripted demo (yam_arena task file).
- `labkit.py`: parts built from several shapes, with hinges and slides (the clip is a hinge with a torsion spring).
  It hooks `yam_arena.scene` and the embodiment from the task's import, so no change to yam_arena is needed.
- `arena_assets_fix.py`: imported by `labkit.py`; points Arena library assets at NVIDIA's production bucket. This task
  uses no library assets, so it only matters if the task is extended.

## Run a trial

Inside the Isaac Lab-Arena environment with `yam-isaac-lab-arena` installed (the Robocurve Brev launchable):

    inspect-robots run --embodiment yam_arena --policy dream \
        -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 \
        --instruction "$(python -c 'import json;print(json.load(open("task.json"))["instruction"])')" \
        --scorer task_score --max-steps 1200 --store-frames --log-dir logs

Scripted demo: the same command with `--policy yam_scripted` (about 30 s of motion).

The score is `task_score` in the log, computed by `score()` in `task.py` from the final state.

## Runtime and seeds

- 1,200 control steps (60 s of simulated time) per trial; about 3 minutes of wall time on one L40S with frames stored at 640 x 352.
- One fixed starting layout: run with `-E object_jitter=0`. No other randomness.
