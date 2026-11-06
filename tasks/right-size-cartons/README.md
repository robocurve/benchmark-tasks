# Pack each product in the smallest carton it fits

Two products, three open cartons of different sizes. Each product goes into the smallest carton it fits in:
the blue product is just too long for the small carton, and the orange one just too long for the medium one.

## Files

| File | What |
|---|---|
| `task.json` | Task details for review |
| `right_size_cartons.py` | `yam_arena` task file: scene, `success()`, `score()`, `distance()` and the scripted `demo()` |

## Run a trial

Station scene, fixed layout, no randomization (`object_jitter=0`), no seeds. Run from the repository root.

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/right-size-cartons/right_size_cartons.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Pack each product into the smallest carton it fits in, one product per carton." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/right-size-dream

# Scripted demo (reaches 1.0)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/right-size-cartons/right_size_cartons.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Pack each product into the smallest carton it fits in, one product per carton." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/right-size-demo
```

Runtime per trial: TODO (the scripted demo reaches success after about 24 s of robot time).

## Scoring

`score()` implements the rubric in `task.json`: 0.5 per product lying flat inside the smallest carton it fits
in, 0.2 per product lying flat in a larger carton, else 0. `success()` needs both products in their smallest
cartons and still; the episode ends after it holds for 1 s.

## Scripted demo

The left arm grips the blue product across its short side, carries it over the medium carton, stops just above
the rim until the arm settles, and lets go 4 mm above the rim, so the product drops flat onto the carton floor;
then it moves clear.
The right arm does the same with the orange product and the large carton.
