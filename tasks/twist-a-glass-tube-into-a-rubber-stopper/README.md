# Hold a glass tube close below a rubber stopper with one hand while the other twists the stopper onto it

A short glass tube stands clamped upright in a stand, a cloth wrapped round it just below its top end; a rubber stopper
lies on the table beside it. One hand holds the tube by the cloth, close below its end; the other picks up the stopper,
sets its hole on the tube's end and twists it down the tube in short turns, then lets go. Twisting while the tube is
held only far below the stopper (or not at all) levers the glass, and it snaps: the classic lab injury. The task is one
task file, [`twist_a_glass_tube_into_a_rubber_stopper.py`](twist_a_glass_tube_into_a_rubber_stopper.py), for the
`yam_arena` embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from
the same folder.

What it tests: following a lab safety rule about where to hold glass while working it; turning a part in strokes
(grip, turn, let go, turn back, grip again) while following it down.

Why two hands: while the stopper turns against the tube, the tube has to be held close below it, or it snaps. One hand
holds the tube while the other twists the stopper. With one hand (the right hand twisting the stopper on as in the
demo, the left never moving) the demo scored 0.0: the glass snapped at the first turn, the top piece dropped on the
table and the stopper fell free.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The stand and the tube (two pieces, its top piece
carrying the cloth) are kinematic, fixed where they are put; the stopper is a loose rigid body made of shapes from
`bodykit.py`.

- The fit: set on the tube's end and turned 45 degrees clockwise against it, the stopper catches: the task file switches
  on a thread joint between the stopper and the tube (a `Thread` from `bodykit.py`), and after every control step moves
  its slide target down the tube 16 mm per turn the stopper has made against the tube. The stopper does not collide
  with the tube's top piece (a thread switched on while running would not stop the two colliding), so before it
  catches the tube does not hold it.
- The glass: after every control step in which a hand on the stopper turns it at least 1 degree against the tube, the
  task file checks the hands on the tube. If none holds it within 3.5 cm below the stopper's underside, the glass snaps:
  the task file switches the thread off (the stopper comes free) and drops the tube's top piece on the table.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, with the stopper still) has held for 1 s, otherwise `score()` at the end of the
episode (0, 0.25, 0.5 or 1.0; see `task.json`). `-E object_jitter=0` keeps every object exactly where the task file
puts it, so every trial starts from the same layout. The cameras render at 1280 x 704 for sharp videos; Dream-1's
client scales every frame to 320 x 176 before sending it, so `-E cam_width=320 -E cam_height=176` gives the policy the
same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=twist_a_glass_tube_into_a_rubber_stopper.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Hold the glass tube still with one hand by the cloth just below its top end; with the other pick up the rubber stopper, set its hole on the tube's end and twist it clockwise down the tube in short turns, at least 12 mm; then let go of the stopper." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/tube-dream
```

The scripted demo, which reaches full score: the left hand comes level from the left onto the tube's cloth, 2.6 cm
below its top end, fingers closing across it, and holds it. The right hand comes straight down on the stopper, fingers
closing across its flats just below its rim, picks it up, sets its hole on the tube's end and twists it down clockwise
in four quarter-turn strokes (grip, roll the wrist, let go, roll back, grip again), following it down. Then both hands
let go and draw back:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=twist_a_glass_tube_into_a_rubber_stopper.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Hold the glass tube still with one hand by the cloth just below its top end; with the other pick up the rubber stopper, set its hole on the tube's end and twist it clockwise down the tube in short turns, at least 12 mm; then let go of the stopper." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/tube-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/tube-dream/*.json; do inspect-robots video "$log"; done
```

Set `TUBE_DEBUG=1` to print what the score is built from twice a second: how far the tube's end is up into the stopper,
how far off its axis and how tilted, whether it is seated, how far it has been turned toward catching, whether it has
caught and how many turns since, how far below the stopper the tube is held, whether the glass has snapped, and where
each TCP is from the tube's end and the stopper.

## Runtime per trial

On one RTX 5090: about 2 minutes for a Dream-1 trial of 1,200 steps (147 s on the clock, with Dream answering in 0.79 s on average), and about 3 minutes for the scripted demo (628 steps, 31.4 s of simulated time, 189 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.
