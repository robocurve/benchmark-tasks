# Hold a sprung-up M.2 drive flat in its socket with one hand while the other turns the latch over its end

An M.2 drive sits in its socket on a motherboard, sprung up 20 degrees, as a drive does once its edge is pushed into the
socket. One hand presses it down flat and holds it there; the other turns the knob of the quarter-turn latch at the
drive's far end, so that the latch's arm comes over the drive's end. Then both hands let go and the latch holds the
drive down. The task is one task file, [`hold_an_m2_drive_flat_and_latch_it.py`](hold_an_m2_drive_flat_and_latch_it.py),
for the `yam_arena` embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it
imports from the same folder.

Why two hands: the latch's arm turns at a fixed height, just over the top of the flat drive. It cannot press the drive
down, only hold it once it is down: turned while the drive is sprung up, it passes under the drive's end, and then
stops the drive from going flat (the drive rests on it, more than 3 degrees up: not latched). Pressed flat and let go,
the drive springs straight back up. And the hand on the latch's knob, on its 4 cm stem, cannot reach the drive. So one
hand holds the drive flat while the other turns the latch. (Holding the drive flat alone scores 0.25.)

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The motherboard, the drive and the latch are rigid
bodies made of several shapes from `bodykit.py`; the motherboard is kinematic, fixed where it is put.

- The drive is joined to the motherboard by a hinge at its socket, about the socket's line, that lets it turn from flat
  to 20 degrees up. A spring in the hinge pulls it up: 0.03 N m holds it 20 degrees up, 0.09 N m pressed flat (about
  2.3 N pressed 4 cm from the socket). The drive does not collide with the motherboard; the hinge's stop holds it flat.
- The latch is joined to the motherboard by a hinge about its standoff's axis, damped (it stays where it is turned)
  and stopped at a quarter turn. Its arm is 1 mm over the flat drive's top, its hub goes through the slot at the drive's
  end, and it collides with the drive.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file look at the simulation after every control step (how long the drive has
been held flat, and where the grippers are).

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, with the drive and the latch still) has held for 1 s,
otherwise `score()` at the end of the episode (0, 0.25, 0.5 or 1.0; see `task.json`). `-E object_jitter=0` keeps every
object exactly where the task file puts it, so every trial starts from the same layout. The cameras render at 1280 x 704
for sharp videos; Dream-1's client scales every frame to 320 x 176 before sending it, so
`-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=hold_an_m2_drive_flat_and_latch_it.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Hold the M.2 drive down flat in its socket with one hand while the other turns the latch's knob a quarter turn counterclockwise so its arm holds the drive's end down, then let go with both hands." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/m2-dream
```

The scripted demo, which reaches full score: the left hand comes down on the drive near its socket, fingers closed and
tipped 40 degrees toward the right (so its wrist stays clear of the other), and presses it flat with the lowest corner
of its fingers. The right hand, waiting over the latch until the drive is down, comes straight down on the knob, grips
it and turns it a quarter turn counterclockwise. Then the right hand lets go and rises, and the left hand rises: the
drive springs up against the latch's arm:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=hold_an_m2_drive_flat_and_latch_it.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Hold the M.2 drive down flat in its socket with one hand while the other turns the latch's knob a quarter turn counterclockwise so its arm holds the drive's end down, then let go with both hands." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/m2-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/m2-dream/*.json; do inspect-robots video "$log"; done
```

Set `M2_DEBUG=1` to print what the score is built from (how far the drive is up from flat, how far the latch is turned,
whether the drive has been held flat for 1 s, whether it is latched and both grippers clear, and where each gripper's
TCP is against where the demo aims it) twice a second.

## Runtime per trial

On one RTX 5090: about 2 minutes for a Dream-1 trial of 1,200 steps (141 s on the clock, with Dream answering in 0.64 s on average), and under 1 minute for the scripted demo (232 steps, 11.6 s of simulated time, 38 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.
