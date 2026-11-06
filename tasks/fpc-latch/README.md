# Connect a sensor's flex cable to a ZIF connector on a board held in helping hands

A bare sensor board is held level in the alligator clips of a helping-hands stand, lit by the stand's small LED work
light, with a flip-lock ZIF connector at its edge. A tactile sensor pad lies on a foam block in front of it, its flat flex
cable (FPC) lying flat along the block towards the board, bridging a slot cut across the block near the cable's end. One
hand unclips the connector: it pushes the actuator's tab back until the actuator flips open. The other hand picks the
cable up where it crosses the slot, carries it over, lays its end on the board in front of the connector and slides it
straight in to the slot's back, and holds it there while the first hand pushes the actuator's tab forward until it snaps
shut on the cable. Then both let go. A cable pushed into the slot while the actuator is shut scores 0.

## Files

- `task.py`: scene, scoring and scripted demo (yam_arena task file). The board and the stand are fixed; the board has one
  labkit joint, the actuator's hinge. The cable is the sensor pad's part: 21 short links on spring hinges, the last one
  the rigid stiffened contact end. An `on_step` hook gives the actuator a detent (its drive target flips to open past 30
  degrees and to shut short of it), holds the cable's bend springs at straight, and records whether the actuator has
  been opened, whether the cable entered the slot while the actuator was shut, and whether both hands' grip points are
  at least 3 cm from the connector. At each reset it adds the work light (a small sphere light in front of its panel) to
  the stage once, and leaves the cable's own weight out (gravity acts on the sensor pad only).
- `labkit.py`: parts built from several shapes, with hinges and slides. It hooks `yam_arena.scene` and the embodiment
  from the task's import, so no change to yam_arena is needed.
- `arena_assets_fix.py`: imported by `labkit.py`; points Arena library assets at NVIDIA's production bucket. This task
  uses no library assets; every object is built in `task.py`.

## Run a trial

Inside the Isaac Lab-Arena environment with `yam-isaac-lab-arena` installed (the Robocurve Brev launchable):

    inspect-robots run --embodiment yam_arena --policy dream \
        -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 \
        --instruction "$(python -c 'import json;print(json.load(open("task.json"))["instruction"])')" \
        --scorer task_score --max-steps 1200 --store-frames --log-dir logs

Scripted demo: the same command with `--policy yam_scripted` (about 30 s of motion; the trial ends 10 s after full
score). The demo reads only what the cameras see: the board's pose and, while it lines the cable up and pushes it in, the
cable's offset across the slot and whether its end has reached the slot's back. It never reads the actuator's angle.
Run one trial per process: the trial's record is kept in the task module and cleared at each reset.

The score is `task_score` in the log, computed by `score()` in `task.py`.

## Runtime and seeds

- 1,200 control steps (60 seconds of simulated time) per trial; about 6 minutes of wall time on one L40S with frames
  stored at 640 x 352.
- One fixed starting layout: run with `-E object_jitter=0`. No other randomness.
