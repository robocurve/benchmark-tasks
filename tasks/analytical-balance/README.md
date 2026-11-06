# Weigh out 1.00 g of granules on an analytical balance with a draft shield

An analytical balance with a glass draft shield whose top door slides back. One arm runs the door and the TARE key;
the other puts the empty weighing boat on the pan and, once the balance is tared with the door shut, pours granules
from a vial into the boat until the live display reads 1.00 g. Then the door is shut again.

## Files

- `task.py`: scene, scoring and scripted demo (yam_arena task file). The balance's reading is worked out every control
  step from what rests on the pan; the task records the last TARE press (what was on the pan, whether the door was
  shut) and drives the display's segments to show the reading.
- `labkit.py`: parts built from several shapes, with hinges and slides (the top door, the sprung TARE key and the 28
  display segments are slides). It hooks `yam_arena.scene` and the embodiment from the task's import, so no change to
  yam_arena is needed. This copy adds two things: a box may carry a yaw or a (yaw, pitch) turn (the vial's spout wall
  leans out 30 degrees), and colours listed in `labkit.GLASS_COLORS` are drawn with Omniverse's glass material
  (falling back to a translucent, then a plain surface).
- The 70 pellets are labkit parts (one short cylinder each). In simulation a pellet weighs 0.2 g and the boat 12 g, so
  falling pellets neither upset the contact solve nor shove the boat; the reading still counts 0.03 g and 1.2 g.
- `arena_assets_fix.py`: imported by `labkit.py`; points Arena library assets at NVIDIA's production bucket. This task
  uses no library assets.

## Run a trial

Inside the Isaac Lab-Arena environment with `yam-isaac-lab-arena` installed (the Robocurve Brev launchable):

    inspect-robots run --embodiment yam_arena --policy dream \
        -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 \
        --instruction "$(python -c 'import json;print(json.load(open("task.json"))["instruction"])')" \
        --scorer task_score --max-steps 2400 --store-frames --log-dir logs

Scripted demo: the same command with `--policy yam_scripted` (about 100 s of motion; 2,100 steps are enough). The
demo's pour is closed loop on the display, in short bursts as a person taps out a powder: the vial tips slowly until
the reading goes up by a pellet, tips back 6 degrees, waits a second for the reading to settle, and goes on; at
0.97 g it levels the vial. Run one trial
per process: the tare record and the pour state are kept in the task module and cleared at each reset.

The score is `task_score` in the log, computed by `score()` in `task.py` from the final state.

## Runtime and seeds

- 2,400 control steps (120 s of simulated time) per trial; about 12 minutes of wall time on one L40S with frames stored at 640 x 352 (73 bodies, 70 of them pellets).
- One fixed starting layout: run with `-E object_jitter=0`. No other randomness.
