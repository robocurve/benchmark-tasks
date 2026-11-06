# Load a trailer with 60% of the weight ahead of the axle

Three crates (one 160 g steel, two 40 g wooden) must go onto a trailer bed with at least 60% of their weight ahead of
the axle, on the hitch side. The steel crate alone is 67% of the weight, so it must be ahead of the axle; with it
behind, at most 33% is ahead.

## Files

| File | What |
|---|---|
| `task.json` | Task details for review |
| `trailer_sixty_forty.py` | `yam_arena` task file: scene, `success()`, `score()`, `distance()` and the scripted `demo()` |

## Run a trial

Station scene, fixed layout, no randomization (`object_jitter=0`), no seeds. Run from the repository root.

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/trailer-sixty-forty/trailer_sixty_forty.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Load all three crates onto the trailer bed so that at least 60% of their total weight is ahead of the axle (the red line), on the hitch side. The grey steel crate weighs 160 g and each wooden crate 40 g." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/trailer-dream

# Scripted demo (reaches 1.0)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/trailer-sixty-forty/trailer_sixty_forty.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Load all three crates onto the trailer bed so that at least 60% of their total weight is ahead of the axle (the red line), on the hitch side. The grey steel crate weighs 160 g and each wooden crate 40 g." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/trailer-demo
```

Runtime per trial: up to 2 minutes of robot time (2,400 steps at 20 Hz); the scripted demo finishes its motions
after about 37 s.

## Scoring

`score()` implements the rubric in `task.json`. Which side of the axle a crate is on is decided by its centre, so a
crate straddling the stripe counts on the side its centre is on (measurable with a tape from the stripe). Any layout
with the steel crate ahead of the axle and all three crates on the bed scores 1.0; with the steel crate behind it,
0.6 at most.

## Scripted demo

The left arm loads the steel crate near the hitch end, then the middle wooden crate just ahead of the axle, and moves
clear; the right arm then loads the other wooden crate behind the axle. 200 g of 240 g (83%) ends up ahead.
