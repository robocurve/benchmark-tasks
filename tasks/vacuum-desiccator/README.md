# Load a sample into a vacuum desiccator and pump it back down

A 24 cm vacuum desiccator under vacuum, a red stopcock on its side, a vacuum line running past a dial gauge beside the stopcock to a pump
controller box with a rocker switch, and a sample vial on the bench. One arm vents the desiccator by swinging the stopcock
open slowly with the pump off, lifts the dome off by its knob and holds it up beside the base; the other arm stands the
vial on a ring marked on the plate inside. The dome goes back on, the pump is switched on until the gauge reads
-90 kPa or lower, the stopcock is shut and the pump switched off. Last, a pull on the knob checks the dome is held
down: under vacuum it does not lift.

## Files

- `task.py`: scene, air model, scoring and scripted demo (yam_arena task file). An `on_step` hook updates the box's
  pressure from the stopcock's opening and the pump switch, drives the gauge needle, and pushes the dome down onto the
  base with the vacuum load while it is sealed. The base and dome are lathe meshes built from their profiles.
- `labkit.py`: parts built from several shapes and meshes, with hinges and slides. It hooks `yam_arena.scene` and the
  embodiment from the task's import, so no change to yam_arena is needed.
- `arena_assets_fix.py`: imported by `labkit.py`; points Arena library assets at NVIDIA's production bucket. This task
  uses no library assets.

## Run a trial

Inside the Isaac Lab-Arena environment with `yam-isaac-lab-arena` installed (the Robocurve Brev launchable):

    inspect-robots run --embodiment yam_arena --policy dream \
        -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 \
        --instruction "$(python -c 'import json;print(json.load(open("task.json"))["instruction"])')" \
        --scorer task_score --max-steps 2400 --store-frames --log-dir logs

Scripted demo: the same command with `--policy yam_scripted` (about 95 s of motion; the trial ends 20 s after full
score, which leaves time for the seal check). The demo reads only what a policy can see: the gauge's dial,
which sits beside the stopcock in view of the right wrist camera while that hand works the lever. It never reads the
box's pressure, a joint angle or an object pose. The hand on the stopcock's post opens the lever further only while the
gauge's reading rises slower than 7 kPa a second, then swings it fully open; the dome is set back over the base's axis;
the pump runs for a fixed time before the stopcock is shut. Run one
trial per process: the box's pressure is kept in the task module and cleared at each reset.

The score is `task_score` in the log, computed by `score()` in `task.py` from the final state: the vial's pose on the
plate, the dome's pose on the flange, the box's pressure, the stopcock's angle, the pump switch, and whether the box
was ever vented faster than 12 kPa a second.

## Runtime and seeds

- 2,400 control steps (120 seconds of simulated time) per trial; about 9 minutes of wall time on one L40S with frames
  stored at 640 x 352.
- One fixed starting layout: run with `-E object_jitter=0`. No other randomness.
