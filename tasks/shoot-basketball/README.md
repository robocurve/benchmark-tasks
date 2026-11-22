# Throw a mini orange basketball into a miniature basket goal with the right arm

A mini orange basketball (54 mm in diameter, 60 g) rests near the front of the station table on the robot's right
side. Further back on the right side sits a fixed miniature basketball goal consisting of a white backboard
(180 mm wide, 160 mm tall, 12 mm thick) and a red square basket (130 x 130 mm inner opening, 80 mm tall walls,
rim at 140 mm above the table) mounted on a 50 mm pedestal. Using the right arm, the robot must pick up the
basketball and throw it into the basket goal so that it lands inside the basket.

The task is one file, `task.py`, for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM
embodiment for Isaac Lab-Arena, run through Inspect Robots), on the `station` scene.

## Run a trial

From this folder on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Use the right arm to pick up the orange basketball and throw it into the red basket."

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
- **Seeds:** none needed. Submitted runs use `-E object_jitter=0`, so the basketball starts at its exact coordinates
  defined in `task.py`.

## Scoring, exact definitions

This is the full rule that `score()` implements; the scoring field in `task.json` is a shorter summary of it.

Evaluated from the simulator state. Every limit below is inclusive (a value at the limit passes within floating-point
epsilon `1e-6`). The basketball is **inside the basket** when:

- Its horizontal centre is within 51.5 mm (`BASKET_INNER / 2 - BALL_R * 0.5`) of the basket centre along both the
  x and y axes.
- Its lowest point (`z - BALL_R`, where `BALL_R = 27 mm`) is between 52 mm and 113 mm above the table top—i.e., from
  8 mm below the 60 mm basket floor up to the ball centre sitting at the 140 mm rim height.

The basketball does not need to be completely at rest (`st.resting()` is not required), so a ball still rolling
inside the basket at episode termination still counts.

- **Score 1.0:** the basketball is inside the basket goal.
- **Score 0.0:** the basketball is not inside the basket goal.