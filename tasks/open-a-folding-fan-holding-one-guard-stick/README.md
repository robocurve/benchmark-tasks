# Open a folding fan fully: hold one guard stick still by its tip while the other hand sweeps the other one round

A lacquered folding fan lies half open on a round felt-topped display stand: two black guard sticks with gilt tips and
six bamboo ribs on one brass rivet, with a painted paper leaf. One hand pinches one guard stick's tip and holds it
still; the other pinches the other guard stick's tip and carries it round the rivet, turning with it, until the leaf
is spread wide. Both hands let go, the fan lying open on its stand, and both arms go back to where they started. The
task is one task file, [`open_a_folding_fan_holding_one_guard_stick.py`](open_a_folding_fan_holding_one_guard_stick.py),
for the `yam_arena` embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it
imports from the same folder.

What it tests: holding a light part still by a small feature while the other hand moves a linked part along an arc,
turning the grip with it; letting go so the result stays.

Why two hands: the fan's rivet is stiff and the fan is light, so a guard stick pushed round on its own carries the
whole fan round with it on the felt; only with the other guard stick held do the folds give and the leaf spread. With
one hand (the right doing its part as in the demo, the left never moving) the demo scored 0: the whole fan turned
about its rivet with the right hand, and its spread stayed at 76 degrees.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres; the bodies here are made of several shapes each, from
`bodykit.py`.

- The fan: eight rigid sticks (two guard sticks, six ribs), each a body whose origin is on the rivet, joined in a
  chain by hinges about the rivet's vertical axis (`Hinge` from `bodykit.py`), each fold between 0 and 21.4 degrees
  (the leaf's pleat), so the fan spreads 150 degrees at most. The sticks never collide with each other. The leaf is
  drawn on the sticks (seen only), half of each pleat on each stick, so it tiles when the fan is open.
- The stiff rivet: each fold has a spring (0.04 N m per degree). After every control step the task file moves a
  fold's rest angle to where the fold is if it has been pushed more than 1 degree past it, so a fold resists turning
  with up to 0.04 N m and then stays where it is left.
- The stand is fixed; the fan lies loose on its felt.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score with every stick of the fan still) has held for 1 s,
otherwise `score()` at the end of the episode (0, 0.25, 0.5 or 1.0; see `task.json`). `-E object_jitter=0` keeps
every object exactly where the task file puts it, so every trial starts from the same layout. The cameras render at
1280 x 704 for sharp videos; Dream-1's client scales every frame to 320 x 176 before sending it, so `-E cam_width=320
-E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=open_a_folding_fan_holding_one_guard_stick.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Open the folding fan fully: hold one of its guard sticks still by its tip with one hand while the other hand carries the other guard stick round the rivet until the fan is spread wide. Then let go, leaving the fan lying open on its stand, and take both arms back to where they started." \
  --scorer task_score --max-steps 800 --store-frames --log-dir ~/yam-eval/fan-dream
```

The scripted demo, which reaches full score: the left hand comes straight down on the left guard stick's tip, beyond
the stand's edge, fingers across the stick, and pinches it. The right does the same on the other guard stick's tip
and carries it round the rivet in 5-degree steps, turning the grip with the stick, 78 degrees clockwise: the folds
give and the leaf spreads to 150 degrees. The right lets go and rises, then the left, and both arms go back to where
they started:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=open_a_folding_fan_holding_one_guard_stick.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Open the folding fan fully: hold one of its guard sticks still by its tip with one hand while the other hand carries the other guard stick round the rivet until the fan is spread wide. Then let go, leaving the fan lying open on its stand, and take both arms back to where they started." \
  --scorer task_score --max-steps 800 --store-frames --log-dir ~/yam-eval/fan-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/fan-dream/*.json; do inspect-robots video "$log"; done
```

Set `FAN_DEBUG=1` to print what the score is built from twice a second: the fan's spread, each fold's angle, where
the rivet is relative to the stand's middle and its height above the felt, whether the fan lies on its stand, whether
both arms are back, and where each TCP is.

## Runtime per trial

On one RTX 5090: about 2 minutes for a Dream-1 trial of 800 steps (110 s on the clock, with Dream answering in 0.82 s on average), and about 1 minute for the scripted demo (348 steps, 17.4 s of simulated time, 63 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.
