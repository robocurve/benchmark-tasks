# Aim two mug handles at a pin

Two arms each turn one mug in place so its handle points at a fixed red pin in the middle of the table. The scene,
scorer and scripted demo are in `mug_handles_aim_pin.py`.

## Run a trial

Run from the repository root. Dream-1:

    inspect-robots run --embodiment yam_arena --policy dream \
      -E task=tasks/mug-handles-aim-pin/mug_handles_aim_pin.py -E object_jitter=0 \
      -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "Turn each of the two mugs in place, keeping it upright and its centre within 2 cm of the centre of its blue taped square, so that its handle points at the red pin to within 5 degrees." \
      --scorer task_score --store-frames --max-steps 2400 --log-dir runs/dream

Scripted demo: same flags with `--policy yam_scripted --max-steps 300`. The score is printed as `task_score`.
Render videos with `inspect-robots video <log>.json`.

## Scoring

Per mug, all of: upright on the table; body centre within 2 cm of its square's centre; handle bearing within 5 degrees
of the bearing from the mug's centre to the pin's centre (-66.8 degrees for the left mug, +66.8 for the right, counter
clockwise from straight ahead). 1.0 if both mugs pass, 0.5 if one, else 0. The scorer subtracts the 0.93 cm offset of
the model's bounding-box centre from the body axis, so turning a mug about its own axis keeps its measured centre fixed.

## Reach and layout

- Each mug is 28 cm from the midline. Offline inverse kinematics: each arm reaches its own mug's waypoints (error
  0.1 to 0.2 mm); the other arm misses them by 156 mm (left mug) and 215 mm (right mug). Both arms are needed.
- Starting handle bearings are 40 degrees (left) and 200 degrees (right), 107 and 133 degrees from the goal.
- The goal bearing (about 67 degrees from straight ahead) is not an axis direction, not the direction to the
  midline and not the mirror of the start.

## Shortcut checks (scripted, 330 steps)

| Variant | Score |
|---|---|
| Do nothing | 0 |
| Left mug only turned | 0.5 |
| Right mug only turned | 0.5 |
| Both handles straight ahead | 0 |
| Both handles toward the midline | 0 |
| Both handles pointing away from the pin | 0 |
| Correct turn, mugs moved 3 cm forward | 0 |

One mug only gives 0.5 by design (partial credit).

## Runtime and seeds

- Dream-1: 2,400 steps (2 minutes simulated), about 10 minutes of wall time on one GPU.
- Scripted demo: about 295 steps (15 s simulated), about 2 minutes of wall time.
- No randomness: run with `-E object_jitter=0`. No seeds.

## Results

Dream-1: 0.0. Demo: 1.0.
