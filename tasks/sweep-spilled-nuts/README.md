# Sweep spilled nuts into a dustpan

A task file for the `yam_arena` embodiment ([yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena), Isaac Lab-Arena 0.3.1) on the station scene: [sweep_spilled_nuts.py](sweep_spilled_nuts.py).

Ten M8 nuts lie in two piles, each out of the other arm's reach, with a dustpan between them and a hand brush on the right. One hand holds the dustpan while the other sweeps its pile in. Then the hands swap the brush and the dustpan, the second pile goes in, and the dustpan is lifted off the table.

## Run a trial

From the repository root. Dream-1:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/sweep-spilled-nuts/sweep_spilled_nuts.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Sweep all the spilled nuts into the dustpan with the hand brush, then lift the dustpan off the table." \
  --scorer task_score --max-steps 3600 --store-frames --log-dir ~/yam-eval/sweep-dream
```

Scripted demo (scores 1.0): the same command with `--policy yam_scripted --max-steps 1400`. The demo is open loop, with no snapping or attachments: the brush and the dustpan are held only by the grippers' friction.

The run prints `task_score` at the end. `score()` in the task file returns 0 if the dustpan is tilted more than 30 degrees. Otherwise it is the share of nuts whose centre is over the dustpan's floor, inside its walls and less than 3 cm above its base, halved unless the dustpan's lowest point is at least 5 cm above the table. A nut whose lowest point has risen more than 2 cm above the table outside the dustpan was carried, not swept, and counts 0 for the rest of the trial. `score()` keeps track of these nuts between steps and forgets them when every nut is back at its start, which marks a new trial. `success()` is a score of 1.0, held for 1 s.

Control run: with the left hand parked aside and nobody holding the dustpan, the right arm's sweep pushes the dustpan 6.6 cm back toward the robot and 1 of the 5 right-pile nuts lands in it (5 of 5 when it is held).

## Layout

Positions in metres from the midpoint between the arm bases (x forward, y to the left, table top at z 0.7475):

| Object | Position | Notes |
|---|---|---|
| Dustpan | (0.20, 0.00) | turned 90 degrees: mouth toward +x, handle bar at x 0.08 |
| Brush | (0.16, -0.20) | lengthwise along x |
| Right pile | (0.30, -0.17), (0.315, -0.215), (0.33, -0.165), (0.345, -0.235), (0.36, -0.19) | nuts 0-4 |
| Left pile | (0.305, 0.215), (0.315, 0.175), (0.33, 0.235), (0.345, 0.19), (0.36, 0.16) | nuts 5-9 |

## Runtime and seeds

- Demo: 1,256 steps (63 s simulated), about 2 minutes on an RTX 4090 including start-up.
- Dream-1: 3,600 steps (3 minutes simulated), about 5 minutes on an RTX 4090.
- `object_jitter=0` keeps the layout fixed, so no seed is needed. The scripted demo is deterministic.

## Files

- [sweep_spilled_nuts.py](sweep_spilled_nuts.py): the scene, scoring and scripted demo.
- [bodykit.py](bodykit.py): compound rigid bodies built from boxes, copied unchanged from [connect-a-sensor-cable-holding-the-sensor-while-turning-its-coupling-ring](../connect-a-sensor-cable-holding-the-sensor-while-turning-its-coupling-ring/bodykit.py).
