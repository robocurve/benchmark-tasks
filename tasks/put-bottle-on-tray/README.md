# Sort a red bleach bottle and green ammonia bottle into their matching colored cabinet spill trays

Two cylindrical chemical bottles stand upright on the robot's left side: a red bleach bottle and a green ammonia
cleaner bottle (each 56 mm in diameter, 110 mm tall, and 150 g). Further back on the table sit two fixed,
color-coded cabinet spill trays—one red and one green (each 120 x 120 mm and 15 mm thick)—spaced 40 mm apart so
incompatible chemicals are stored separately. Using the left arm, the robot must pick up each bottle and place it
upright inside the spill tray of the same color. Placing both bottles in the same tray scores 0 to reflect unsafe
mixing.

The task is one file, `task.py`, for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM
embodiment for Isaac Lab-Arena, run through Inspect Robots), on the `station` scene.

## Run a trial

From this folder on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Pick up the red bleach bottle and the green ammonia cleaner bottle and place each one upright in the cabinet spill tray of the same color."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 2400 --store-frames --log-dir $PWD/yam-eval/dream

# Scripted demo
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 600 --store-frames --log-dir $PWD/yam-eval/demo

```

The run prints `task_score`. Scoring is `score()` and `success()` in `task.py`, described in words in `task.json`.

- **Runtime per trial:** at most 120 s of simulated time (2,400 steps at 20 Hz) for Dream-1, and 30 s (600 steps)
  for the scripted demo.
- **Seeds:** none needed. Submitted runs use `-E object_jitter=0`, so both bottles start at their exact coordinates
  defined in `task.py`.


## Scoring, exact definitions

This is the full rule that `score()` implements; the scoring field in `task.json` is a shorter summary of it.

Evaluated from the simulator state. Every limit below is inclusive (a value at the limit passes within floating-point
epsilon `1e-6`). A bottle is **upright** when the angle between its vertical axis and world up is at most 10 degrees.
A bottle is **at rest** when `st.resting()` is true. A bottle is **on a tray top** when its lowest point along its
vertical axis (`z - (H / 2) * R[2, 2]`) is within 5 mm of the tray top surface (15 mm above the table). A bottle is
**inside a tray** when it is at rest, upright, on the tray top, and its entire circular base (radius 28 mm) lies
within the 120 x 120 mm tray footprint—meaning its centre is within 32 mm of the tray centre along both the x and y
axes.

- **Score 1.0:** the red bleach bottle is inside `red_spill_tray` and the green ammonia bottle is inside
  `green_spill_tray`.
- **Score 0.5:** exactly one bottle is inside its matching colored spill tray, and the other bottle is not inside
  that same tray.
- **Score 0.0:** neither bottle is inside its matching colored spill tray, or both bottles are inside the same
  spill tray (unsafe mixing).