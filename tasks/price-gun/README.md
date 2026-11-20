# Price three spice bottles with a price gun

Three small spice bottles stand in a row, and a hand labeller (a price gun) lies on its side beside them. Each full
squeeze of the gun's trigger feeds one label, which is wiped onto a bottle's cap with the roller at the gun's nose.
One arm works the gun while the other holds the bottle, which is light enough to tip at a touch.

## Files

- `task.py`: scene, label model, scoring and scripted demo (yam_arena task file). A step hook winds the trigger's
  spring past its rest stop (a preload), runs the label model and prints lines starting `PG|` (state, every 5 control
  steps) and `PG!` (label events).
- `labkit.py`: parts built from several shapes, with hinges and slides. The gun's trigger is a sprung hinge and its
  roller a hinge. It hooks `yam_arena.scene` and the embodiment from the task's import, so no change to yam_arena is
  needed. This copy adds display-only shapes (`DISPLAY`), used to draw the labels.
- `arena_assets_fix.py`: imported by `labkit.py`; points Arena library assets at NVIDIA's production bucket. This task
  uses no library assets.
- `task.json`: the task's specification.

## Run a trial

Inside the Isaac Lab-Arena environment with `yam-isaac-lab-arena` installed (the Robocurve Brev launchable):

    inspect-robots run --embodiment yam_arena --policy dream \
        -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 \
        --instruction "$(python -c 'import json;print(json.load(open("task.json"))["instruction"])')" \
        --scorer task_score --max-steps 2400 --store-frames --log-dir logs

Scripted demo: the same command with `--policy yam_scripted` (it takes 62 s, 1,246 control steps).

The score is `task_score` in the log, computed by `score()` in `task.py`.

## How the demo holds the gun

One gripper straddles the handle (behind) and the trigger (in front). Closing fully is a full stroke and a firm hold;
opening part-way lets the trigger out and still holds the gun, because the trigger's spring is wound. The arm's drives
are soft and carry the gun's 4 N, so the demo aims each pose of the loaded arm high by the sag a static model predicts.

## Runtime and seeds

- 2,400 control steps (2 minutes of simulated time) per trial.
- One fixed starting layout: run with `-E object_jitter=0`. No other randomness.

## Third-party assets

None. Every shape is built in `task.py` from boxes and cylinders.

## Simulation fidelity (full note)

Rigid bodies in PhysX at 240 Hz. The bottles are rigid cylinders of the stated size and 90 g; a real plastic bottle gives slightly under the fingers. The gun is built from boxes: a body, a nose, a handle, a sprung trigger on a hinge, with a finger rest on its front, and a 16 mm roller under the nose. Estimated, not measured: the gun's outline (182 mm long, 182 mm tall, 50 mm wide), its trigger's travel and force, and the bottle's width and cap size (from the listed height and product photos). Measure them on the real items before real trials. Labels are state, not film. A stroke past 90% of the trigger's travel followed by its return to within 25% of rest feeds one label under the roller; a shorter stroke feeds nothing; a second full stroke while a label waits there doubles it. Where the roller then first touches a cap (within 1 mm), the label is laid down from that line back along the wipe. It is four cells of 10 x 5.5 mm; a cell sticks when the roller passes over its centre while touching, so a wipe that is too short, or a roller held askew, leaves cells lifted. A waiting label that touches the table first sticks there as a stray. In simulation only the roller can lay a label. The label's pull as it is wiped on is a 0.4 N drag at the roller (inferred, not measured). Printing, ink, the liner take-up and label curl are not modelled.
