# Move spoil piles back from a trench edge to the safe band

Two spoil piles sit 0.3 m (at 1:10) from a trench edge, inside OSHA's 0.6 m minimum. Both must go to the far side,
at least 0.6 m from the edge and wholly inside a fence, a 66 mm deep band for a 40 mm pile. The setback line isn't
drawn, so the distance has to be judged from the stated scale.

## Files

| File | What |
|---|---|
| `task.json` | Task details for review |
| `trench_spoil_setback.py` | `yam_arena` task file: scene, `success()`, `score()`, `distance()` and the scripted `demo()` |

## Run a trial

Station scene, fixed layout, no randomization (`object_jitter=0`), no seeds. Run from the repository root.

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/trench-spoil-setback/trench_spoil_setback.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "The spoil piles are too close to the trench. Move both of them to the far side of the trench, at least 0.6 m (6 cm in this 1:10 model) from its edge and wholly inside the red fence. Leave the near side clear for the workers." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/trench-dream

# Scripted demo (reaches 1.0)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/trench-spoil-setback/trench_spoil_setback.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "The spoil piles are too close to the trench. Move both of them to the far side of the trench, at least 0.6 m (6 cm in this 1:10 model) from its edge and wholly inside the red fence. Leave the near side clear for the workers." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/trench-demo
```

Runtime per trial: up to 2 minutes of robot time (2,400 steps at 20 Hz); the scripted demo finishes its motions
after about 26 s.

## Scoring

`score()` implements the rubric in `task.json` from each pile's footprint (from its corners) and pose. Pushing the
piles back on the near side scores 0; setting them just across the trench scores 0.15 each.

## Scripted demo

The left arm carries the left pile over the trench and sets it 13 mm beyond the setback (13 mm short of the fence),
then moves clear; the right arm does the same with the right pile.
