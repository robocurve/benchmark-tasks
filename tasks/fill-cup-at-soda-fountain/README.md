# Fill a cup at a lever soda fountain to just below the rim

A task file for the `yam_arena` embodiment ([yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena), Isaac Lab-Arena 0.3.1) on the station scene: [fill_cup_at_soda_fountain.py](fill_cup_at_soda_fountain.py), with the helper [trialkit.py](trialkit.py) next to it.

A fast-food counter with a single-valve soda fountain. An empty paper cup stands on the counter in front of it. The drink runs only while the valve's lever paddle is pushed in, and the paddle is sprung: let go of the cup and the paddle pushes it away and the flow stops. So the robot has to hold the cup with its mouth under the nozzle, keep its rim pressed against the paddle while the cup fills, pull back at the right level and set the cup down.

## How it is modelled

- **Lever:** a steel paddle on a sprung hinge under the valve head (0.03 N m per degree, about 4 N at the cup rim when pushed 8 degrees in; it stops at 15 degrees). The drink runs while it is pushed in more than 6 degrees.
- **Drink:** liquid is not simulated. While the drink runs, the task file raises the drink level by 1.6 cm a second if the cup's mouth is within 2.5 cm of the nozzle and the cup is upright, and counts it as spilled onto the drip tray otherwise. The level is drawn as eleven dark cola layers that appear inside the cup one by one, so the cameras see the cup filling.
- **Cup:** a tapered paper cup (6 cm across at the base, 9 cm at the rim, 11.6 cm tall). It collides as a straight 7.6 cm cylinder inside the visible wall, which is what the fingers and the paddle touch.

## Scoring

`score()` runs every control step and the trial is scored at the end:

- **0** if the drink overflows (level above the rim), more than 0.5 cm of drink runs outside the cup, or the cup tips more than 30 degrees.
- **1.0** if the drink ends 0.5 to 2 cm below the rim and the cup is set down: upright within 10 degrees, base on the counter, still.
- **0.5** if the cup is at least half full and set down, but the level is outside that band.
- **0** otherwise.

## Run a trial

From this folder, in the Isaac Lab-Arena environment (`source /data/IsaacLab-Arena/yam.env`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=fill_cup_at_soda_fountain.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Fill the paper cup with soda at the fountain, to about 1 cm below the rim, and set it down on the counter." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/soda-dream
```

Scripted demo (scores 1.0): the same command with `--policy yam_scripted`. The left arm holds the cup from behind with the gripper tilted 75 degrees forward and the fingers round the cup's sides, so the hand stays clear of the nozzle. It lifts the cup 3 cm (clear of the drip tray's edge), pushes the rim against the paddle, holds it there for 6.3 s, pulls back, sets the cup down and moves out of the camera's view. `SODA_DEBUG=1` prints the paddle angle, drink level and cup position every half second.

The task defines no `success()`, so the trial runs to the step limit and is scored on the final state.

## Checks

Scripted with `SODA_VARIANT`:

| Run | Drink level | Score |
|---|---|---|
| Demo (`fill`): pressed 6.3 s | 10.5 cm, 1.1 cm below the rim | 1.0 |
| `underfill`: pressed 4 s | 6.8 cm | 0.5 |
| `overfill`: pressed 8.5 s | over the rim | 0 |
| `offcentre`: pressed with the cup 3.5 cm off the nozzle | ran onto the drip tray | 0 |
| `release`: lets go of the cup while pressing | the paddle pushed the cup off, flow stopped at 1.5 cm | 0 |

The `release` run shows that the cup only fills while it is held against the lever.

## Real trials

Use a real single-valve dispenser (or a water dispenser with the same paddle valve) and a 16 oz paper cold cup. Measure the final level from the rim with a ruler, and look for drink on the drip tray or counter. A safe task: cold drink, light cup.

## Runtime and seeds

- Demo: about 505 steps (25 s simulated), about two minutes on an L40S.
- Dream-1: 1,200 steps (60 s simulated), about four minutes.
- The layout is fixed; `object_jitter=0` keeps every position as listed.

## Licence and third-party material

This task is released under the MIT licence, including [trialkit.py](trialkit.py), a helper written for these tasks: compound parts, sprung hinges, parts shown or hidden while the simulation runs, and tilted gripper waypoints.

No third-party models, textures or code are used: every object is built from boxes and cylinders declared in the task file.
