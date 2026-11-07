# Hold a puzzle cube's left two layers with one hand while the other turns its right layer a quarter turn

A solved 3x3 puzzle cube (ABS plastic, vinyl stickers) sits loose on a slim plastic display stand. One hand holds the cube's left two layers; the other grips
its right layer and turns it a quarter turn, until it clicks into line. Then both hands let go. The task is one task
file, [`turn_a_puzzle_cube_layer_a_quarter_turn.py`](turn_a_puzzle_cube_layer_a_quarter_turn.py), for the `yam_arena`
embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from the same
folder.

Why two hands: the cube is light (90 g) and only sits on the stand's small pad, and its layer needs a firm twist
(0.25 N m) to get past the click. Turned on its own, the layer takes the whole cube with it: with the left hand kept
clear, the scripted demo's turn rolls the cube off its stand, and the layer gets at most 28 degrees against the rest
before it clicks back (0.25). One hand holds the rest of the cube still while the other turns the layer.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The cube and the stand are rigid bodies made of
shapes from `bodykit.py`; the stand is kinematic, fixed where it is put.

- The cube is two bodies: its left two layers as one, and its right layer. They are joined by a hinge (a `Hinge` from
  `bodykit.py`) along the line through the cube's left and right faces, so only the right layer turns. The stickers
  are flat coloured shapes that do not collide.
- The click: the hinge has a spring (0.25 N m at 45 degrees from a quarter turn), and after every control step the task
  file points it at the quarter turn nearest the layer's turn. Past 45 degrees the layer snaps on into line at the next
  quarter turn and stays there; short of that it springs back.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, with the cube still) has held for 1 s, otherwise
`score()` at the end of the episode (0, 0.25, 0.5 or 1.0; see `task.json`). `-E object_jitter=0` keeps every object
exactly where the task file puts it, so every trial starts from the same layout. The cameras render at 1280 x 704 for
sharp videos; Dream-1's client scales every frame to 320 x 176 before sending it, so
`-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=turn_a_puzzle_cube_layer_a_quarter_turn.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Hold the puzzle cube's left two layers with one hand, and with the other turn its right layer a quarter turn until it clicks into line; then let go with both hands." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/cube-dream
```

The scripted demo, which reaches full score: both hands come in level from their own side. The left one, fingers
closing on the cube's top and bottom, grips its left two layers. The right one, fingers closing on the right layer's
front and back, grips that layer and rolls the wrist a quarter turn; the layer clicks into line. Then the right hand
lets go and backs away, and so does the left:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=turn_a_puzzle_cube_layer_a_quarter_turn.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Hold the puzzle cube's left two layers with one hand, and with the other turn its right layer a quarter turn until it clicks into line; then let go with both hands." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/cube-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/cube-dream/*.json; do inspect-robots video "$log"; done
```

Set `CUBE_DEBUG=1` to print what the score is built from (where the cube is and how far it is tipped, the right
layer's turn and the most it has turned, where each gripper's TCP is from the cube's centre, and whether both have let
go) twice a second.

## Runtime per trial

On one RTX 5090: about 2 minutes for a Dream-1 trial of 1,200 steps (138 s on the clock, with Dream answering in 0.64 s on average), and under 1 minute for the scripted demo (271 steps, 13.6 s of simulated time, 59 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.
