# Pack a tray with cubes without any sticking out

`packing_without_overfilling.py` is a `yam_arena` task file: the scene, the score hook and a scripted demo.

## Run a trial

The instruction is the one in `task.json`:

    INSTR="Pack as many cubes as you can into the tray. No cube may stick out above the top of the tray's walls."

Dream-1:

    inspect-robots run --embodiment yam_arena --policy dream \
      -E task=packing_without_overfilling.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "$INSTR" --scorer task_score --grader none --max-steps 3600 --store-frames --log-dir ~/yam-eval/packing-dream

Scripted demo (reaches 1.0):

    inspect-robots run --embodiment yam_arena --policy yam_scripted \
      -E task=packing_without_overfilling.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "$INSTR" --scorer task_score --grader none --max-steps 3600 --store-frames --log-dir ~/yam-eval/packing-demo

## Score

`task_score` reports the score of the final state. There is no early success: a trial runs to its limit
(3,600 steps, 3 minutes at 20 Hz), so a policy can still overfill after it has packed five cubes.

- 0 if any cube over the tray has a point more than 2 mm above the rim.
- Otherwise the number of cubes in the tray divided by 5.

## Runtime and seeds

- A model trial is 3,600 steps (3 minutes of simulated time). The demo plan is 62 s, then it holds for 3 s and
  stops.
- Run every trial with `-E object_jitter=0`: the cubes start at their positions in `task.json`. The exact
  positions are not critical; each cube needs only at least 4 cm of clear table around it for the gripper.
