# Measure a pin's diameter with a hand-held digital caliper, then put the caliper back on its rest

A digital caliper lies on a wooden rest, its jaws pointing at the robots and its head overhanging the rest, and a
turned aluminium pin 23.40 mm across stands on end on the bench. One arm picks the caliper up by its head and holds it
level, off the bench, with the open jaws either side of the pin and the fixed jaw close to its side, as an inspector
measures a part standing on the bench. The other arm pushes the slider's thumb rest until the sliding jaw has carried
the pin onto the fixed jaw and both just touch it, then lets go. The live display then shows the diameter. A tilted
caliper or a hard push gives a wrong reading or squeezes the pin. Once the reading is taken, the holding arm lifts
the caliper off the pin, lays it back on its rest and lets go, and both arms move away.

## Files

- `task.py`: scene, scoring and scripted demo (yam_arena task file). Every control step, an `on_step` hook reads the
  slider's position and drives the display's 28 segments to show the jaw gap it gives. The jaws are rigid: the sliding
  jaw is fixed to the slider by a locked joint, and the score reads the jaw force from that joint's reaction (as a load
  cell in the jaw would), averaged over half a second. The caliper counts as held only with a fingertip within 20 mm of
  its beam and nothing else under it (bench or rest). `task.py` lets a demo waypoint carry a full gripper rotation (the
  `Orient` class), so yam_arena itself is unchanged.
- `labkit.py`: parts built from several shapes, with hinges and slides, including joints chained on other joints
  (`parent=`): the slider is a damped slide with friction, and the sliding jaw and the display segments ride on it.
  `TORSIONAL_PATCH` gives a part's colliders a contact patch, so a finger pad holds it by friction over its area
  (PhysX otherwise treats a pad contact as points, and a pinched part turns about the line between the pads). It hooks
  `yam_arena.scene` and the embodiment from the task's import, so no change to yam_arena is needed.
- `arena_assets_fix.py`: imported by `labkit.py`; points Arena library assets at NVIDIA's production bucket. This task
  uses no library assets.

## Run a trial

Inside the Isaac Lab-Arena environment with `yam-isaac-lab-arena` installed (the Robocurve Brev launchable):

    inspect-robots run --embodiment yam_arena --policy dream \
        -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 \
        --instruction "$(python -c 'import json;print(json.load(open("task.json"))["instruction"])')" \
        --scorer task_score --max-steps 1600 --store-frames --log-dir logs

Scripted demo: the same command with `--policy yam_scripted` (about 54 s of motion). There is no early end on
success: the trial runs its full 80 s, so the caliper going back on its rest, both hands moving away and the
stillness at the end are seen. The demo reads only what a camera
sees: the pin's and caliper's poses and the display. The holding hand's target is placed from the pin's seen position
and trimmed from the caliper's seen pose: under the caliper's weight the arm sags and tips it (about 16 degrees at
the wrist), so the hand's tilt, height and sideways place are corrected until the caliper sits level with its fixed
jaw beside the pin; once the jaws are on the pin the hand stays put. The pushing finger starts just short of the thumb
rest, where the display says it is, advances at 3 mm/s (1 mm/s for the last 3 mm), and stops once the jaw is seen on
the pin, the pin against the fixed jaw and the reading has not fallen for 0.5 s; it is then held where it actually is
(the arm lags its target by several mm) and backs slowly off. Set `CAL_DEBUG=1` to print
the state every half second. Run one trial per process: the demo's state is kept in the task module and cleared at
each reset.

The score is `task_score` in the log, computed by `score()` in `task.py`. The measurement is latched while the
caliper is held: every step that the caliper is held off the bench and the rest, the pin is between the jaws, the
caliper and the pin are still (points on them, the sliding jaw and thumb rest included, moved under 1 mm over the
last 21 samples, 1 s) and both fingertips of the other hand are 80 mm or more from them, the display reading, the
pin's squareness and the jaw force are scored (`_tier`), and the latest such moment counts. The final score is that
measurement's score only if the trial ends with the caliper back on its rest (flat within 5 degrees, its underside
within 3 mm of the rest's top), all four fingertips of both hands 80 mm or more from the caliper and the pin, and
everything still for the last second; otherwise 0. The fingertips are the pad tips, each read from its own finger's
link pose (the two jaws stop independently).

## Runtime and seeds

- 1,600 control steps (80 seconds of simulated time) per trial; about 4 minutes of wall time on the lab GPU.
- One fixed starting layout: run with `-E object_jitter=0`. No other randomness.

## Shortcut check

`CAL_CHECK=letgo` runs the demo, then has the holding hand let go of the caliper on the pin once it has measured,
instead of putting it back. The caliper drops onto the pin and the bench, is not on its rest at the end, and the
trial must score 0 (lab GPU, Nov 14: 0; the measurement, 23.43 mm, was latched while held, then the caliper fell onto
the pin instead of going back on its rest).
