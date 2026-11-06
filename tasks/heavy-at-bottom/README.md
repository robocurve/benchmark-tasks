# Stack the boxes with the heavy steel case at the bottom

A task file for the `yam_arena` embodiment ([yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena), Isaac Lab-Arena 0.3.1) on the station scene: [heavy_at_bottom.py](heavy_at_bottom.py).

A steel parts case and two cardboard cartons of the same size are to be stacked on the pallet mark. The instruction says nothing about order, but the heavy steel case belongs at the bottom. `score()` gives 0.5 for the case on the mark and 0.25 for each carton stacked on it, and 0 if the case ends on top of another box or any box tips over.

## Run a trial

Dream-1:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/heavy-at-bottom/heavy_at_bottom.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Stack the three boxes on the yellow pallet mark." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/heavy-at-bottom-dream
```

Scripted demo (scores 1.0): the same command with `--policy yam_scripted --max-steps 900`. The left arm carries the steel case low onto the pallet mark, going forward first so it clears the cartons, then stacks the two cartons on it and moves clear of the camera's view.

The run prints `task_score` at the end. The task defines no `success()`, so the trial is scored on where the boxes end up.

## Checks

Scripted variants of the same scene, each replacing only `demo()`:

| Variant | Score |
|---|---|
| Demo: steel case at the bottom, both cartons on it | 1.0 |
| Steel case on the mark, nothing on it | 0.5 |
| Cartons at the bottom, steel case on top | 0.0 |
| Nothing moved | 0.0 |

## Runtime and seeds

- Demo: about 830 steps (42 s simulated), about two minutes on an L40S.
- Dream-1: 2,400 steps (2 minutes simulated), about 6 minutes on an L40S.
- `object_jitter=0` keeps the layout fixed, so no seed is needed. The simulator is not bit-exact, so where each box lands varies by a few millimetres between runs, well inside the 3 cm tolerance.
