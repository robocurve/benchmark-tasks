# Move a row of fragile tiles into a box in as few trips as possible

Five thin ceramic tiles stand on edge in a row. The robot must move all of them into a box in as few trips
as possible, without dropping a tile or letting one tip over. A grasp of the whole row at once scores 1.0, but
every extra tile in a grasp adds risk. One tile at a time is safer but scores only 0.2.

Files:

- `thin_plate_pnp.py`: the scene, the scoring hooks and a scripted demo (one trip, score 1.0).
- `task.json`: the task's details.

## Run a trial

Set up the environment once per shell, then run from the repository root. The task needs `-E object_jitter=0`:
the 6 mm gaps between the tiles are smaller than the default 3 cm jitter.

```bash
source ~/yam/IsaacLab-Arena/yam.env

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/thin-plate-pnp/thin_plate_pnp.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Move all five tiles into the box in as few trips as possible, without dropping or tipping over any." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/tiles-dream

# Scripted demo (one trip, about 16 s)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/thin-plate-pnp/thin_plate_pnp.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Move all five tiles into the box in as few trips as possible, without dropping or tipping over any." \
  --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/tiles-demo
```

The run prints `task_score`. Render the camera videos with `inspect-robots video <log>.json`. Add
`-E headless=false` to watch a run in the Isaac Sim window.

- **Runtime per trial:** up to 2,400 control steps (2 minutes at 20 Hz). The demo ends after about 330 steps,
  when it succeeds.
- **Seeds:** none. The layout is fixed (`object_jitter=0`), and the scripted demo is deterministic.

## Scoring

The score is read from the tile poses after every control step:

- **Broken (score 0):** a tile's face tilts more than 60 degrees from vertical (it lies rather than leans), or its centre drops faster than
  0.5 m/s (a fall of about 1.3 cm or more), at any step. This also applies inside the box.
- **Not done (score 0):** at the end, not every tile's centre is inside the box frame, with the tile at rest on
  the table.
- **Otherwise:** (N - K + 1) / N with N = 5 tiles and K trips.

A trip starts when a tile's lowest point rises more than 1 cm above the table. Lifts within 10 control steps
(0.5 s at 20 Hz) of the first lift of a trip join that trip, so two arms that lift at the same time make one trip.
A tile counts as set down when it is back below 1 cm and at rest. A later lift of that tile starts a new trip.

The hooks get no reset signal, so the counter starts again whenever every tile stands at its start pose. This is
true at the start of every trial.

## Score a real trial

1. Count the trips in the video: each time one or more tiles leave the table, with lifts within 0.5 s of each
   other counted as one trip.
2. Score 0 if a tile fell or was dropped (also into the box), or if a tile tipped more than 60 degrees.
3. Score 0 if not all five tiles stand inside the frame at the end.
4. Otherwise the score is (6 - K) / 5.
