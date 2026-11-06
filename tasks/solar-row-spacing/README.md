# Space two rows of solar modules so neither shades the other

A short and a tall solar module must both stand on a roof's grey area, clear of the red fire-access border, with a
gap behind each module of at least twice its height. The deck is only deep enough with the short module in front.

## Files

| File | What |
|---|---|
| `task.json` | Task details for review |
| `solar_row_spacing.py` | `yam_arena` task file: scene, `success()`, `score()`, `distance()` and the scripted `demo()` |

## Run a trial

Station scene, fixed layout, no randomization (`object_jitter=0`), no seeds. Run from the repository root.

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/solar-row-spacing/solar_row_spacing.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Install both solar modules on the roof's grey area, long side facing you, clear of the red border. Leave a gap behind each module at least twice its height before the next module, so no module shades another." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/solar-dream

# Scripted demo (reaches 1.0)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/solar-row-spacing/solar_row_spacing.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Install both solar modules on the roof's grey area, long side facing you, clear of the red border. Leave a gap behind each module at least twice its height before the next module, so no module shades another." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/solar-demo
```

Runtime per trial: up to 2 minutes of robot time (2,400 steps at 20 Hz); the scripted demo finishes its motions
after about 26 s.

## Why only one order works

The grey area is 140 mm deep and each module 30 mm deep. Short module in front: 30 + 50 (2 x 25 mm) + 30 = 110 mm,
which fits. Tall module in front: 30 + 100 (2 x 50 mm) + 30 = 160 mm, which doesn't. Side by side facing the sun
needs 120 mm of the grey area's 100 mm width, and turning the modules 90 degrees breaks the facing rule.

## Scoring

`score()` implements the rubric in `task.json`, from each module's pose: footprint from its corners, tilt and yaw from
its orientation. `success()` needs 1.0 with both modules still; the episode ends after it holds for 1 s.

## Scripted demo

The left arm stands the short module in the front row, 7 mm behind the red border, and moves clear. The right arm
then stands the tall module in the back row, leaving a 62 mm gap behind the short module (50 mm needed). Each module
is let go 3 mm above the deck.
