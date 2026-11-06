# Hang a weight on a spring hook without shock-loading it

The robot hangs a 0.4 kg weight by its ring on the hook of a hanging spring scale (150 N/m). The weight alone
stretches the hook 26.2 mm. If the robot lets go as soon as the ring touches the pin, the hook overshoots to well over
37 mm, and the scale's maximum-reading (drag) pointer ends in the red overload range. If the robot lowers past contact until the hook carries the weight and only then opens, the
peak stays near the static stretch.

Built for yam-isaac-lab-arena (the program's YAM embodiment for Isaac Lab-Arena), station scene. `parts.py` (the
parts kit: multi-colour parts with sliders) must sit next to `task.py`; `task.py` imports it.

## Run and score a trial

```
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Hang the grey weight on the hook by its ring and let go." \
  --scorer task_score --max-steps 900 --store-frames --log-dir logs
```

Use `--policy yam_scripted` (and `-E cam_width=640 -E cam_height=360`) for the scripted demo.

- Runtime: about 1–2 minutes per trial on an L40S, including start-up. Run limit 900 control steps (45 s).
- Seeds: fixed layout. Submitted runs use `-E object_jitter=0`; the stand is fixed and only the weight would be
  jittered.

## Scoring as implemented

`score()` and `success()` in `task.py`:

- **Hung:** where the pin's centre line crosses the plane of the ring, it lies inside the ring's opening (30 mm wide,
  28 mm high), and the ring is between the pin's tip and its carriage. The weight's lowest point is at least 20 mm
  above the table. The weight moves slower than 3 cm/s and turns slower than 1 rad/s (the hung weight keeps a
  sub-millimetre contact jitter in the simulator, so a tighter limit flickers). The hook's extension below its
  unloaded rest is within 2 mm of 26.2 mm, the weight's own static stretch: the hook carries the whole weight to
  within 0.3 N. The simulator cannot see the fingers; this load test is how it checks the weight has been let go.
- **No visible motion over the last 1.5 s:** an `on_step` callback counts the control steps in a row for which the
  test above has held; `score()` and `success()` need 30 (1.5 s at 20 Hz). The offline checks in `near_misses.py`
  run without the simulator, so they do not apply this window.
- **Peak:** the largest hook extension seen at any control step (20 Hz) since the scene was reset. The drag pointer
  is driven to it after every step. The record is cleared only by a scene reset (an `on_reset` callback), which the
  robot cannot cause, so pushing the pointer back does not help.
- 1.0 when hung and the peak is under 37 mm; otherwise 0, including hung with the peak 37 mm or more (pass/fail).
  `success()` is "hung and peak under 37 mm", held for 4 s (`SUCCESS_HOLD_S`), so the trial (and its video) runs on
  while the arm backs away.

What the scorer cannot see: a gripper that keeps its closed fingers on a still weight while carrying less than
0.3 N of it would count as having let go. A person scoring a photo also checks that the fingers are open. The demo
holds the weight with the hook stretched about 20 mm, outside the 24.2–28.2 mm band, until the fingers open.

## Measured in traces (scripted, object_jitter=0)

| Release | Peak hook extension | Score |
|---|---|---|
| Lowered about 30 mm past contact, held 1.5 s, then opened (earlier demo) | 28.8 mm | 1.0 |
| Lowered about 21 mm past contact (hook at about 20 mm), held 1.5 s, then opened (final demo) | under 37 mm (scored 1.0; not traced) | 1.0 |
| Lowered to contact, opened at once (a debugging variant of the demo, since removed) | about 41 mm | 0 |
| Pressed 46 mm past contact before opening (early trace) | 39.7 mm | 0 |

The carried weight sits about 20 mm to the robot's right of the commanded tool point and 4 mm lower, so the demo
offsets its commands from the pin onward (`COMP_Y`, `COMP_Z`).

## Files

- `task.py`: scene, scoring and scripted demo.
- `parts.py`: the parts kit (unchanged copy).
- `near_misses.py`: score boundaries and unintended strategies, checked offline.
