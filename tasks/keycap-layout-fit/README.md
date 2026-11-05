# Fit keycaps to a keyboard row by a layout card

Fit four colored keycaps onto four switches in the order shown on a layout card, pressing each one
down, while the other arm keeps the loose keyboard plate from sliding.

## Files

| File | What |
|---|---|
| `task.json` | Task details for review |
| `keycap_layout_fit.py` | `yam_arena` task file: scene, `success()`, `score()`, `distance()` and the scripted `demo()` |

## Run a trial

Station scene, fixed layout, no randomization (`object_jitter=0`), no seeds. Run from the repository root.

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/keycap-layout-fit/keycap_layout_fit.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Fit the four keycaps onto the four switches in the color order shown on the white card, pressing each one fully down, and keep the keyboard plate where it is." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/keycap-dream

# Scripted demo (reaches 1.0)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/keycap-layout-fit/keycap_layout_fit.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Fit the four keycaps onto the four switches in the color order shown on the white card, pressing each one fully down, and keep the keyboard plate where it is." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/keycap-demo
```

Runtime per trial: TODO (the scripted demo reaches success after about 48 s of robot time).

## Scoring

`score()` implements the rubric in `task.json`: 0.25 per keycap sitting flat on the switch whose card
square has its color and centered within 3 mm, 0.1 if it rests on that switch within 6 mm but is not
seated, capped at 0.5 once the plate has moved more than 10 mm. `success()` needs all four seated with
the plate in place and the keycaps still; the episode ends after it holds for 1 s.

## Scripted demo

The right arm presses its closed fingers on the plate's right end for the whole task. The left arm takes
the keycaps starting from the right-hand switch: it grips each one across the row's direction with the
fingers opened only 30 mm, stops 12 mm above its switch until the arm settles, lowers it, lets go 2 mm
above the switch, then taps it down with closed fingers. Starting from the right keeps the hand, which
trails its target slightly toward the left while moving, away from keycaps already in place. The two hands stay about 18 cm apart, because the YAM wrist housings
stick out about 3 cm toward each other.
