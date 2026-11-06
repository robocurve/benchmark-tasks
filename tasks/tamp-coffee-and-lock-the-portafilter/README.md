# Hold a portafilter level with one hand while the other tamps the coffee, then lock it into the espresso machine

A portafilter filled with loose ground coffee rests on the counter, tipped on its spout and the end of its handle, in
front of an espresso machine. One hand lifts the handle until the basket is level, its spout still on the counter, and
holds it there while the other hand presses the coffee down with a tamper. Then the portafilter goes up into the
machine's group head, its two ears past the head's lips, and a turn of the handle sets the ears on the lips; both hands
let go and the portafilter hangs locked in the head. The task is one task file,
[`tamp_coffee_and_lock_the_portafilter.py`](tamp_coffee_and_lock_the_portafilter.py), for the `yam_arena` embodiment
(scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from the same folder.

Why two hands: a tamp is only good if it is pressed square into a level basket. On the counter the basket is tipped
almost 8 degrees, and tamped as it lies it scores no more than 0.25; one hand has to hold the basket level while the
other presses. (Locking it into the head afterwards is one hand's work.)

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The machine, the portafilter, the coffee and the
tamper are rigid bodies made of several shapes from `bodykit.py`; the machine is kinematic, fixed where it is put.

- The ground coffee is a disc filling the basket, on a slider joint (a `Thread` from `bodykit.py`, switched on from
  the start) down the basket's axis. A spring holds it where it was last pressed: pressed more than 1 mm deeper, it
  stays there (springing back 1 mm when the tamper lifts). It never collides with the basket or the machine.
- The group head is a bayonet: two lips at its bottom with open sides between them, a ceiling above, and a stepped
  boss under the ceiling that goes into the basket's rim and centres it. The portafilter's ears pass up at the open
  sides; turned, they come over the lips, and let go, the portafilter hangs on them. The head is polished (low
  friction), so the handle turns easily.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file look at the simulation after every control step: it keeps the coffee
where it was pressed, notes how level the basket was while it was pressed, and follows where the grippers are.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, with the portafilter still) has held for 1 s, otherwise
`score()` at the end of the episode (0, 0.25, 0.5 or 1.0; see `task.json`). `-E object_jitter=0` keeps every object
exactly where the task file puts it, so every trial starts from the same layout. The cameras render at 1280 x 704 for
sharp videos; Dream-1's client scales every frame to 320 x 176 before sending it, so
`-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tamp_coffee_and_lock_the_portafilter.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Hold the portafilter level by its handle with one hand while the other tamps the ground coffee in its basket with the tamper. Then lock the portafilter into the espresso machine's group head: push it up into the head with its ears at the open sides and turn the handle toward you, then let go." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/coffee-dream
```

The scripted demo, which reaches full score: the left hand comes straight down on the handle near its end and lifts
it, turning its wrist, until the basket is level on its spout. The right hand takes the tamper by its neck, sets it in
the basket, presses the coffee down, lifts it out and stands it back. The left hand lifts the portafilter level under
the group head, its ears at the open sides, pushes it up into the head, turns the handle toward the front about the
head's axis, lets go and backs away:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tamp_coffee_and_lock_the_portafilter.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Hold the portafilter level by its handle with one hand while the other tamps the ground coffee in its basket with the tamper. Then lock the portafilter into the espresso machine's group head: push it up into the head with its ears at the open sides and turn the handle toward you, then let go." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/coffee-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/coffee-dream/*.json; do inspect-robots video "$log"; done
```

Set `COFFEE_DEBUG=1` to print what the score is built from (where the portafilter is and how tipped, how far the
coffee is pressed and how level the basket was while it was pressed, where each gripper's TCP is, the handle's turn,
whether the portafilter is in the head and locked, and whether both hands have let go) twice a second.

## Runtime per trial

On one RTX 5090: about 2 minutes for a Dream-1 trial of 1,200 steps (150 s on the clock, with Dream answering in 0.81 s on average), and about 1 minute for the scripted demo (574 steps, 28.7 s of simulated time, 75 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.
