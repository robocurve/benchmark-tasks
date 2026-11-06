# Mask a bar so only its coating band is exposed

A steel bar has an orange coating band between two grey zones that must stay bare. Two masking plates must cover
both grey zones without covering the band: too far in blocks the band, too far out leaves bare steel exposed.

## Files

| File | What |
|---|---|
| `task.json` | Task details for review |
| `mask_coating_band.py` | `yam_arena` task file: scene, `success()`, `score()`, `distance()` and the scripted `demo()` |

## Run a trial

Station scene, fixed layout, no randomization (`object_jitter=0`), no seeds. Run from the repository root.

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/mask-coating-band/mask_coating_band.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Mask the bar for coating: lay one masking plate flat on each grey zone so both grey zones are covered and the orange band stays fully uncovered." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/mask-dream

# Scripted demo (reaches 1.0)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/mask-coating-band/mask_coating_band.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Mask the bar for coating: lay one masking plate flat on each grey zone so both grey zones are covered and the orange band stays fully uncovered." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/mask-demo
```

Runtime per trial: up to 2 minutes of robot time (2,400 steps at 20 Hz); the scripted demo finishes its motions
after about 26 s.

## Scoring

`score()` implements the rubric in `task.json`. Coverage is sampled on a 1 mm grid over the bar's top face. A grey
zone point counts as covered only under the actual outline (with its yaw) of a plate lying flat on the bar. A band
point counts as covered under any plate's top-down shadow, at any height or tilt, so a plate held over the band
also costs points. `success()` needs both sides at 0.5 and both plates still; the episode ends after it holds for 1 s.

## Scripted demo

The left arm grips its plate across the 50 mm side, carries it over the left grey zone with the plate's inner edge
1 mm outside the band, settles, and lets go 3 mm above the bar so the plate drops flat; then it moves clear. The
right arm then does the same on the right.
