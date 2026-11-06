# Press two new floor tiles down flush with the laid ones

A strip of floor has three laid tiles and two new ones on fresh mortar, standing about 5.5 mm and 8.5 mm proud. The
robot must press each new tile down until it is level with its neighbours, within 1.0 mm either way. The mortar
holds the depth it is pressed to, so a tile pushed too far cannot be brought back up.

Built for yam-isaac-lab-arena (the program's YAM embodiment for Isaac Lab-Arena), station scene. `parts.py` (the
parts kit: multi-colour parts with sliders and step callbacks) must sit next to `task.py`; `task.py` imports it.

## Run and score a trial

```
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Bed the two new tiles: press them down until they are flush with the tiles around them." \
  --scorer task_score --max-steps 600 --store-frames --log-dir logs
```

Use `--policy yam_scripted` (and `-E cam_width=640 -E cam_height=360`) for the scripted demo.

- Runtime: about 1–2 minutes per trial on an L40S, including start-up. Run limit 600 control steps (30 s).
- Seeds: fixed layout. Nothing is loose, so `object_jitter` has no effect; submitted runs use `-E object_jitter=0`.

## The mortar

Each new tile is a kit part: its slab piece is the fixed body, and a slider link (straight down, 15 mm travel, 50 g)
carries the mortar block and the tile, on a 1,000 N/m spring. An `on_step` callback moves the spring's rest point
(`set_target`) to 1.0 mm above the deepest point the tile has reached. So the tile:

- rests 0.49 mm below its rest point under the weight of tile and bed (50 g together);
- yields only when pressed 1.0 mm past its set depth: 1.0 N of spring force, of which the weight supplies 0.49 N, so a
  press of about 0.51 N; it then stays where it was pressed;
- springs back 0.51 mm when the press stops; the spring pulls it back down if it is lifted, and the slider's limit
  stops it at its starting height, so it can never rise above where it started.

The tiles cannot be gripped: the 3 mm joints leave no room for fingers.

The same ratchet runs inside `score()` from the joint positions, so the offline checks and the simulator agree.

## Scoring as implemented

`score()` and `success()` in `task.py`:

- Lippage of a tile = its starting height above flush (6 or 9 mm of mortar) minus (its set depth + 0.49 mm): the
  height its top will rest at above the laid tiles' tops once nothing presses on it.
- Let go: the tile sits no more than 0.255 mm deeper than that resting height (pressed with under about 0.26 N).
  A tile held down by a gripper with the yield force (0.51 N) on it sits 0.51 mm deeper and does not count.
- Flush: lippage within 1.0 mm either way (1e-6 m margin) and let go.
- 1.0: both flush. 0.5: exactly one flush and let go, the other still proud (it cannot be below flush, see next).
  0: neither.
- Sunk: once either tile has been pressed so deep that it rests more than 1.0 mm below the laid tiles, the score is
  0 for the rest of the trial. The record is cleared
  by the kit's `on_reset` callback, and, for the scorer on its own, when both tiles jump back up by more than 2 mm in
  one step to within 0.5 mm of their start, which pressing cannot do.

`success()` is a score of 1.0, held for 3 s (`SUCCESS_HOLD_S`), so the trial (and its video) runs on while the arms
move away.

## Measured in traces (scripted, object_jitter=0)

The demo presses one tile at a time with closed fingers at its centre (left arm the left tile, then right arm the
right one) to a fixed fingertip height below flush, holds 0.5 s, lifts, and parks the arm 120 mm in front of the
row, 100 mm above it and 200 mm to its own side, while the other arm waits parked. Traces 1-3 and the earlier demo
pressed both tiles at once; there the two wrists (link 4) overlapped by about 22 mm (an offline hull check, which
`check.py` does not make), and the arms pushed on each other, which is why those runs needed very different depths
per arm. With one arm at a time the two arms need nearly the same commanded depth (`PRESS_BELOW`).

| Run | Mortar | Commanded fingertip below flush (left, right) | Tiles at rest (left, right lippage) | Score |
|---|---|---|---|---|
| trace 1 | 3,000 N/m, 0.3 mm yield | 5.0, 5.0 mm | -3.4, -1.5 mm (sunk) | 0 |
| trace 2 | 3,000 N/m, 0.3 mm yield | 1.6, 3.5 mm | -0.4, +0.8 mm | 1.0 (while still pressed) |
| trace 3 | as now | 1.6, 4.7 mm | -0.5, +0.7 mm | 1.0 (after the arms lifted) |
| earlier demo (both at once, 1 s success hold) | as now | 1.1, 5.4 mm | not traced | 1.0 |
| trace 4 (one at a time) | as now | 1.1, 5.4 mm | +1.7, -2.7 mm (sunk) | 0 |
| trace 5 (one at a time) | as now | 2.8, 2.7 mm | -0.3, 0.0 mm | 1.0 (from 14.5 s, after the right arm lifted) |
| final demo | as now | 2.8, 2.7 mm | not traced | 1.0 |

Trace 1 confirmed that the kit's `set_target` holds: after the arms lifted away, both tiles stayed at the depth
they had been pressed to.

## Files

- `task.py`: scene, mortar ratchet, scoring and scripted demo.
- `parts.py`: the parts kit (unchanged copy).
- `near_misses.py`: score boundaries and unintended strategies, checked offline.
