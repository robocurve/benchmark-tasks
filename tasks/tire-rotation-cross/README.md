# Rotate a car's wheels in the rearward-cross pattern

A car's four wheels lie flat on corner pads. They must be rotated in the rearward-cross pattern: rear wheels straight
forward on the same side, front wheels crossed to the opposite rear corner. Every wheel moves and the four form one
cycle, so one wheel has to be set aside on the table first.

## Files

| File | What |
|---|---|
| `task.json` | Task details for review |
| `tire_rotation_cross.py` | `yam_arena` task file: scene, `success()`, `score()`, `distance()` and the scripted `demo()` |

## Run a trial

Station scene, fixed layout, no randomization (`object_jitter=0`), no seeds. Run from the repository root.

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/tire-rotation-cross/tire_rotation_cross.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Rotate the car's wheels in the rearward-cross pattern: move each rear wheel straight forward to the front pad on the same side, and move each front wheel to the rear pad on the opposite side. Leave every wheel lying flat on its new pad." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/tire-dream

# Scripted demo (reaches 1.0)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/tire-rotation-cross/tire_rotation_cross.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Rotate the car's wheels in the rearward-cross pattern: move each rear wheel straight forward to the front pad on the same side, and move each front wheel to the rear pad on the opposite side. Leave every wheel lying flat on its new pad." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/tire-demo
```

Runtime per trial: up to 2 minutes of robot time (2,400 steps at 20 Hz); the scripted demo finishes its motions
after about 83 s.

## Scoring

`score()` gives 0.25 per wheel lying flat on its rearward-cross pad (rubric in `task.json`). Common wrong patterns
score low: doing nothing or the forward cross scores 0, and swapping front and rear on the same side, or crossing all
four, scores 0.5.

## Scripted demo

Seven moves, one arm at a time: red (front left) is set aside on the table; green moves forward on the left; blue
(front right) goes to a handoff spot in front of the car; yellow moves forward on the right; the left arm takes blue
to rear left and red to the handoff spot; the right arm takes red to rear right.
