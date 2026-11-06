# Hold a wine bottle in its cradle by the neck with one hand while the other uncorks it with a corkscrew

A corked bottle of wine lies in a wooden pouring cradle, tipped up 15 degrees, its neck toward the right arm. One hand
holds the bottle by its neck; the other takes a corkscrew from its stand, sets its point on the middle of the cork,
screws it in and pulls the cork out along the neck. The task is one task file,
[`uncork_a_wine_bottle_in_its_cradle.py`](uncork_a_wine_bottle_in_its_cradle.py), for the `yam_arena` embodiment
(scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from the same folder.

Why two hands: the cork gives way only to a pull of 12 N, and the cradle does not hold the bottle against it: pulled
up the cradle along its axis, the 1.2 kg bottle is held back by about 10 N (friction and its weight down the 15 degree
slope), so with nothing holding it the bottle comes up out of the cradle with the cork still in. One hand has to hold
the bottle while the other pulls.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The cradle, the bottle (body, shoulder, a hollow neck
and its lip), the cork, the corkscrew and its stand are rigid bodies made of several shapes from `bodykit.py`; the
cradle and the stand are kinematic, fixed where they are put.

- The cork sits in the neck on a slider joint (a `Thread` from `bodykit.py`, switched on from the start) along the
  neck's axis, held by a spring. Pulled more than 3 mm out against it (12 N), it gives way and stays where it is
  pulled to; pulled 4.5 cm out, it is free of the bottle. It hardly turns in the neck.
- PhysX has no screw thread, and the worm is a plain rod. The task file follows the corkscrew's turn against the cork
  after every control step: with the point 1 to 12 mm into the cork's face, within 6 mm of its middle and lined up
  within 12 degrees, a quarter turn clockwise (seen from the knob) catches the thread and switches on a joint between
  them that keeps the cork on the worm's axis, drags as they turn against each other (0.08 N m s/rad) and draws the
  cork 12 mm along the worm per turn. Turned back far enough, the worm comes out of the cork.
- The corkscrew and the cork, and the cork and the bottle, do not collide.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file look at the simulation after every control step.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, with the bottle still) has held for 1 s, otherwise
`score()` at the end of the episode (0, 0.25, 0.5 or 1.0; see `task.json`). `-E object_jitter=0` keeps every object
exactly where the task file puts it, so every trial starts from the same layout. The cameras render at 1280 x 704 for
sharp videos; Dream-1's client scales every frame to 320 x 176 before sending it, so
`-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=uncork_a_wine_bottle_in_its_cradle.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Hold the wine bottle by its neck with one hand, and with the other screw the corkscrew into the cork and pull the cork out of the bottle." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/cork-dream
```

The scripted demo, which reaches full score: the left hand comes straight down on the bottle's neck and holds it. The
right hand comes straight down on the corkscrew's knob, lifts it out of its stand, turns it to point down the neck,
sets the point on the middle of the cork and screws it in, in strokes of a third of a turn (the wrist turns only so
far): turn, let go, turn back open, take the knob again, following the worm in. Then it pulls straight back along the
neck, drawing the cork out:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=uncork_a_wine_bottle_in_its_cradle.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Hold the wine bottle by its neck with one hand, and with the other screw the corkscrew into the cork and pull the cork out of the bottle." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/cork-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/cork-dream/*.json; do inspect-robots video "$log"; done
```

Set `CORK_DEBUG=1` to print what the score is built from (how deep the worm's point is in the cork, how far off its
middle and how tilted, the turn before the catch and the turns since, how far the cork is out, how far the bottle has
moved and turned, and where each gripper's TCP is) twice a second.

## Runtime per trial

On one RTX 5090: about 3 minutes for a Dream-1 trial of 1,200 steps (157 s on the clock, with Dream answering in 0.81 s on average), and about 2 minutes for the scripted demo (679 steps, 34.0 s of simulated time, 102 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.
