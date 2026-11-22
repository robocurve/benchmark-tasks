# Roll a dark blue bowling ball down the table to knock down miniature bowling pins with the right arm

A dark blue bowling ball (58 mm in diameter, 140 g) rests near the front of the station table on the robot's right
side. Further back across the table, beyond the robot arm's reach, stand three white miniature bowling pins
(20 × 20 mm square footprint, 140 mm tall, 10 g each) arranged in a two-row triangular rack (1-2 formation). Using
the right arm, the robot must pick up the bowling ball and roll it forward down the table to knock down at least a
majority of the pins.

The task is one file, `task.py`, for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM
embodiment for Isaac Lab-Arena, run through Inspect Robots), on the `station` scene.

## Run a trial

From this folder on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Using the right arm, roll the blue bowling ball forward down the table to knock down as many white pins as possible."

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
- **Seeds:** none needed. Submitted runs use `-E object_jitter=0`, so the bowling ball and pins start at their exact
  coordinates defined in `task.py`.

## Scoring, exact definitions

This is the full rule that `score()` implements; the scoring field in `task.json` is a shorter summary of it.

Evaluated from the simulator state. Every limit below is inclusive (a value at the limit passes within floating-point
epsilon `1e-6`). Knocked pins count toward the score when:

- **Minimum forward roll:** the bowling ball centre has advanced forward to at least `x = 0.39 m` (`MIN_ROLL_X =
  BALL_START_XY[0] + 0.15`, i.e., 39.0 cm in front of the arm bases and 15.0 cm forward of its starting position).
- **Knocked pin:** a pin's centre height (`z`) is at or below `TOP + 49 mm` (`TOPPLED_Z_MAX = TOP + PIN_H * 0.35`,
  where `PIN_H = 140 mm`)—i.e., toppled onto its side at ~10 mm above the table top or knocked off the table edge.

Neither the bowling ball nor the pins need to be completely at rest (`st.resting()` is not required) at episode
termination.

- **Score 1.0:** at least 2 of the 3 pins (a majority, `MIN_KNOCKED = 2`) are knocked down after rolling the ball.
- **Score 0.5:** 1 of the 3 pins is knocked down after rolling the ball.
- **Score 0.0:** no pins are knocked down, or the bowling ball was not rolled at least 15.0 cm forward.