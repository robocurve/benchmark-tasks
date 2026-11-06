# Hold a hanging lamp's shade still with one hand while the other screws a bulb into its socket

A lamp hangs low over the table on three chains: a shallow drum shade with an empty socket in its middle, facing up.
One hand holds the shade still by its rim; the other takes a slim bulb from its holder, sets its cap in the socket's
mouth and screws it in, in strokes, until it is seated. Then both hands let go. The task is one task file,
[`screw_a_bulb_into_a_hanging_bowl_lamp.py`](screw_a_bulb_into_a_hanging_bowl_lamp.py), for the `yam_arena` embodiment
(scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from the same folder.

Why two hands: the lamp hangs free on its chains and spins at the slightest turn, and turning the bulb in its socket
drags it round. With the shade unheld, the scripted demo's strokes catch the thread and then turn the whole lamp along:
the bulb never gets a full turn in (0.25; checked by running the demo with the left hand kept clear). One hand has to
hold the shade while the other turns the bulb.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The lamp (shade, socket and chains), the ceiling rose,
the bulb and its holder are rigid bodies made of several shapes from `bodykit.py`; the rose and the holder are
kinematic, fixed where they are put.

- The lamp hangs from the rose by a ball joint at the top of its chains (a `Ball` from `bodykit.py`: a PhysX D6 joint
  with its slides locked): it swings, a little damped, and spins freely about the chains. The chains are rigid and do
  not collide.
- PhysX has no screw thread. The task file follows the bulb's turn against the lamp after every control step: with the
  cap's tip in the socket's mouth (up to 12 mm deep), within 6 mm of its axis and lined up within 12 degrees, a quarter
  turn clockwise (seen from above) catches the thread and switches on a joint between them (a `Thread`) that keeps the
  bulb on the socket's axis, drags as they turn against each other (0.3 N m s/rad) and draws the bulb 8 mm in per turn.
  Turned back far enough, the bulb comes out again.
- The bulb does not collide with the lamp (its cap passes into the socket on the thread joint).

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file look at the simulation after every control step.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, with the lamp and the bulb still) has held for 1 s,
otherwise `score()` at the end of the episode (0, 0.25, 0.5 or 1.0; see `task.json`). `-E object_jitter=0` keeps every
object exactly where the task file puts it, so every trial starts from the same layout. The cameras render at 1280 x 704
for sharp videos; Dream-1's client scales every frame to 320 x 176 before sending it, so
`-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=screw_a_bulb_into_a_hanging_bowl_lamp.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Hold the hanging lamp's shade still by its rim with one hand, and with the other take the bulb from its holder and screw it into the lamp's socket until it is seated, then let go with both hands." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/bulb-dream
```

The scripted demo, which reaches full score: the left hand comes straight down on the shade's rim on its left side,
fingers closing across the rim, and holds it. The right hand comes straight down on the bulb's glass, lifts it out of
its holder, carries it over the socket, sets the cap in the socket's mouth and screws it in, in strokes of a third of a
turn clockwise (the wrist turns only so far): turn, let go, turn back open, take the glass again, following the bulb in.
Then both hands let go and rise:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=screw_a_bulb_into_a_hanging_bowl_lamp.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Hold the hanging lamp's shade still by its rim with one hand, and with the other take the bulb from its holder and screw it into the lamp's socket until it is seated, then let go with both hands." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/bulb-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/bulb-dream/*.json; do inspect-robots video "$log"; done
```

Set `BULB_DEBUG=1` to print what the score is built from (where the lamp is, how far it has spun and swung, how deep the
cap's tip is in the socket, how far off its axis and how tilted, the turn before the catch and the turns since, whether
the bulb is seated and both grippers clear, and where each gripper's TCP is) twice a second.

## Runtime per trial

On one RTX 5090: about 2 minutes for a Dream-1 trial of 1,200 steps (148 s on the clock, with Dream answering in 0.75 s on average), and about 2 minutes for the scripted demo (734 steps, 36.7 s of simulated time, 100 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.
