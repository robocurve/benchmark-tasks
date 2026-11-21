# Pick up a tennis ball with a rolling wire-cage ball picker and lift it off the court

The robot's arms stand on a short bench at the edge of a hard-court mat. On the mat stand a rolling wire-cage ball
picker, its long pole leaning back towards the robot, and a tennis ball just in front of the cage. The robot must
roll the cage over the ball so the ball pops inside, then lift the picker off the court with the ball in it.

Players, coaches and gardeners pick balls and nuts off the ground with rolling wire-cage pickers, standing up, without
bending down. The difficulty is physical: the cage is rolled over the ball under a steady downward press, so two of
its springy wires spread round the ball and close behind it. And the tool is a 1.5 m pole worked from standing height
at the floor: both hands have to work it together to lift the picker with the ball still in it.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: scene, `score()`, `success()`, the touch watch, `self_test()` and the scripted `demo()` |
| `scenekit.py` | USD bodies for yam_arena task files (hooks yam_arena without changing it) |
| `assets/picker.usda` (642 KB) | The ball picker: cage of 30 wires on two hubs, U-bail, pole and grip |
| `assets/ball.usda` (1 KB) | The tennis ball |
| `assets/court.usda` (1 KB) | The hard-court mat |

The assets are simple solids built to the product's published sizes. No third-party assets.

## Setup

Positions in cm from the midpoint between the two arm bases (x forward, y to the robot's left); heights above the
court.

| Object | Where | What |
|---|---|---|
| Bench | front edge at x 12 | The arms' bench, 130 cm wide, its top 76 cm above the floor |
| Court mat | x 15 to 175, centred on y 0 | Green hard-court mat, 160 x 160 cm, 4 mm thick, on the floor |
| Ball picker | cage's axle at (101.6, 0), across y; pole's end at x 20, 127 cm up | Garden Weasel Large Nut Gatherer cage (model 95404): 30 spring-steel wires 1 mm thick between two hubs, 20 cm across, takes objects 3.8-7.6 cm; turning in a red steel U-bail; on a 152 cm (60 in) aluminium pole with a 20 cm black rubber grip at its end; 0.81 kg in all. The pole leans back towards the robot at 55 degrees |
| Tennis ball | (119.6, 0), at rest on the mat | ITF type 2, 6.7 cm, 58 g, yellow felt |

## Scoring

Tools for a real trial: a tape measure and a phone camera from the side.

- **Inside:** the ball sits within the wire cage, between the hubs and behind the wires.
- **Lifted:** the cage's lowest point is at least 10 cm above the court.
- **Touched:** the robot touches the ball at any time (checked on the video).

| Outcome (at the end of the trial) | Score |
|---|---|
| The ball not inside the cage, or the robot touched the ball | 0 |
| The ball inside the cage, the cage not lifted | 0.5 |
| The ball inside the cage and the cage lifted, held for 1 s | 1.0 |

## Run a trial

From this folder, in a shell with the yam_arena environment active:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Pick up the tennis ball with the ball picker: roll its cage over the ball so the ball pops inside, then lift the picker off the court with the ball in it." \
  --scorer task_score --max-steps 2400 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/balls-dream
```

`task_score` prints the trial's score: 1.0 on success (held for 1 s), otherwise `score()` at the end of the episode.
A trial takes about 5 minutes on one RTX 4080 (scene start-up included). Seed 0; the scene has no random placement
(`-E object_jitter=0`).

Offline checks (no GPU): `python task.py` runs the scoring rules and the sizes on 21 hand-built cases and replays the
demo's plan through the arms' kinematics.

## Scripted demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 \
  --instruction "Pick up the tennis ball with the ball picker: roll its cage over the ball so the ball pops inside, then lift the picker off the court with the ball in it." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/balls-demo
```

The hands press the cage onto the court and walk the picker forward 28 cm, so the
cage rolls over the ball and it pops in. They pull the cage back towards the robot, standing the pole up, and take the
pole again lower down, one hand after the other. Then they lift the picker with the ball in it; the trial reaches 1.0
at about 20 s.

A wrong strategy, lifting the picker without rolling its cage over the ball, scores 0.

## Reset

Open the cage with its spreader to take the ball out. Stand the picker back on its mark, the pole leaning back at 55
degrees as in the setup, and the ball on its mark in front of the cage.
