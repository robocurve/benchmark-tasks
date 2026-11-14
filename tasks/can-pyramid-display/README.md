# Build a three-tier can pyramid on a shop display mat

A task file for the `yam_arena` embodiment ([yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena), Isaac Lab-Arena 0.3.1) on the station scene: [can_pyramid_display.py](can_pyramid_display.py).

Six cans go from the delivery tray onto the display mat as a 3-2-1 pyramid. Each upper can rests across two below it, so the bottom row has to sit close enough to support both, and each can has to be set down upright and centred without knocking the tiers already built.

`score()` checks the support relationships, not only heights. The bottom row is three upright cans on the mat in a straight line (the middle can's centre within 1 cm of the line through the outer two), each next to the next. A can in the second tier counts only when it bridges a neighbouring pair of that row: its centre within 1.5 cm of the midpoint between the two, its base within 1 cm of both their tops, and the two 6.6 to 8.3 cm apart. The top can must bridge the two second-tier cans in the same way. A can resting on any other object adds nothing. The trial scores 0 if any can is tipped more than 45 degrees or stands centred on a single can, as in a tower.

## Run a trial

Dream-1:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/can-pyramid-display/can_pyramid_display.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Build a pyramid display of the six cans on the green mat: three in the bottom row, two on those, and one on top." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/pyramid-dream
```

Scripted demo (scores 1.0): the same command with `--policy yam_scripted --max-steps 1300`. The left arm builds the pyramid can by can, the bottom row 7 cm apart, gripping each can with the fingers closing front to back so they never touch a neighbour, then moves clear of the camera's view.

The run prints `task_score` at the end. The task defines no `success()`, so the trial is scored on where the cans end up.

## Checks

`score()` on exact can poses (upright and at rest unless stated):

| Layout | Score |
|---|---|
| Full pyramid (the demo's layout) | 1.0 |
| Bottom row and both bridging cans | 0.5 |
| Bottom row and one bridging can | 0.25 |
| Bottom row only | 0.25 |
| Two towers of three and two plus a third can on the mat | 0 |
| Three towers of two | 0 |
| Upper cans set straight on top of bottom cans | 0 |
| Second-tier can 2.5 cm off the midpoint (centred over one can) | 0 |
| Full pyramid, top can tilted 20 degrees, held 3 cm up, or still moving | 0.5 |
| Bottom row spread 10 cm apart, pyramid built on it | 0 |
| Bottom row in an L instead of a line | 0 |
| Bottom row bent to 106 degrees, cans 8.3 cm apart | 0 |
| Bottom row bent to 150 degrees, pyramid built on it | 0 |
| Middle bottom can 0.8 cm off the line, pyramid built on it | 1.0 |
| Middle bottom can 1.3 cm off the line | 0 |
| Three cans in a row on the delivery tray | 0 |
| Any can tipped over 50 degrees | 0 |
| Nothing done | 0 |

## Runtime and seeds

- Demo: 1,253 steps (63 s simulated), about two minutes on an L40S.
- Dream-1: 2,400 steps (2 minutes simulated), about 6 minutes on an L40S.
- `object_jitter=0` keeps the layout fixed, so no seed is needed. The scripted demo is open loop and deterministic.

## Licence and third-party material

This task is released under the MIT licence. It uses no third-party models, textures or code: every object is a box or cylinder declared in the task file.
