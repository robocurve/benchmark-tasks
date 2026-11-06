# Load a delivery van in reverse route order

Three parcels go into an open cargo bay so they can be unloaded in route order: the route card lists stops 1 to 3,
and the parcel for stop 3 must go in deepest.

## Files

| File | What |
|---|---|
| `task.json` | Task details for review |
| `van_load_reverse_route.py` | `yam_arena` task file: scene, `success()`, `score()`, `distance()` and the scripted `demo()` |

## Run a trial

Station scene, fixed layout, no randomization (`object_jitter=0`), no seeds. Run from the repository root.

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/van-load-reverse-route/van_load_reverse_route.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Load the parcels into the van so they can be unloaded in route order: the parcel for the last stop on the route card goes in deepest and the one for the first stop goes nearest the door." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/van-dream

# Scripted demo (reaches 1.0)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/van-load-reverse-route/van_load_reverse_route.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Load the parcels into the van so they can be unloaded in route order: the parcel for the last stop on the route card goes in deepest and the one for the first stop goes nearest the door." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/van-demo
```

Runtime per trial: TODO (the scripted demo reaches success after about 34 s of robot time).

## Scoring

`score()` implements the rubric in `task.json`: 0.2 per parcel upright inside the bay, plus 0.4 once all three are
in and ordered blue, green, red from the back wall to the door (each at least 20 mm deeper than the next).

## Scripted demo

The left arm loads the deepest slot first: blue against the back wall, then green, then red by the door. It grips
each parcel across its side-to-side width, carries it over its slot, stops just above the wall tops until the arm
settles, and lets go 4 mm above them, so the parcel drops upright onto the bay floor.
