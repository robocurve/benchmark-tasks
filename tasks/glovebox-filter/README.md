# Replace the cabin air filter behind a car's glove box

A passenger-side glove box mock-up stands on the table. It is unlatched and resting open on its two stops, and the
cabin air filter sits in its housing behind it. Each stop is a lug on a side wall that catches on the dash. As on many
cars, the box only swings down past the stops while both side walls are squeezed in at the same time. Then the old filter
slides out of its slot by its pull tab, out through the box's opening. The new filter waits on a stand beside the dash
and goes into the same slot through the same opening.
Squeezing one side at a time does nothing. Forcing the box down without squeezing would break a stop, and is scored 0.

## Files

- `task.py`: scene, scoring and scripted demo (yam_arena task file).
- `labkit.py`: parts built from several shapes, with hinged and sliding joints (the box's hinge and its two flexing
  side walls). It hooks `yam_arena.scene` and the embodiment from the task's import, so no change to yam_arena is
  needed.
- `arena_assets_fix.py`: imported by `labkit.py`; points Arena library assets at NVIDIA's production bucket. This task
  uses no library assets.

## Run a trial

Inside the Isaac Lab-Arena environment with `yam-isaac-lab-arena` installed:

    inspect-robots run --embodiment yam_arena --policy dream \
        -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 \
        --instruction "$(python -c 'import json;print(json.load(open("task.json"))["instruction"])')" \
        --scorer task_score --max-steps 1800 --store-frames --log-dir logs

Scripted demo: the same command with `--policy yam_scripted`.

The score is `task_score` in the log, computed by `score()` in `task.py` from the final state. A forced stop (the box
opening through its stops while either side wall is pushed in less than 4.5 mm) is latched every control step in
`_watch()` and scores 0 for the rest of the trial. `_watch()` also latches the box having passed its stops (open more
than 54 degrees); drawing the old filter from its housing, or putting the new one into it, before that scores 0
(picking the new one up first is fine), and full credit needs the box to have passed its stops. The dash's cheeks
run up to its top panel, so the filter is reached only through the glove box's opening.

## Runtime and seeds

- 1,800 control steps (90 s of simulated time) per trial.
- One fixed starting layout: run with `-E object_jitter=0`. No other randomness.
