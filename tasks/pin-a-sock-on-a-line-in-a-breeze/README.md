# Hold a sock still on a clothesline in a breeze with one hand while the other clips a clothespin over it

A short steel clothesline runs across the table between two posts, with a long green cotton sock folded over it along its length. A
breeze pushes the sock to and fro, swinging it about the line. A wooden clothespin stands in a holder beside it. One
hand holds the sock still on the line; the other takes the clothespin by its handle (squeezing it opens the jaws),
brings it down astride the line over the sock and lets go, so the pin's spring shuts its jaws on them. Then the first
hand lets go too. The task is one task file,
[`pin_a_sock_on_a_line_in_a_breeze.py`](pin_a_sock_on_a_line_in_a_breeze.py), for the `yam_arena` embodiment (scene,
scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from the same folder.

What it tests: holding a light, moving object still with one hand while the other places a sprung tool precisely over
it, timing the release of one hand on the other.

Why two hands: left alone, the sock swings up to about 55 degrees either way, and a clothespin let go over a sock that
nobody holds does not catch it. The simulation does not model the pin catching only a corner of the sock or the breeze
pulling it loose, so the score counts a pin as clipped only if the other gripper is holding the sock as it is let go.
With one hand (the right hand carrying the pin as in the demo, the left never moving) the demo scored 0.0: the pin
went on the line, nothing held the sock, and the swinging sock knocked the pin off again.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The line, with its posts, and the clothespin's holder
are kinematic, fixed where they are put. The sock and the clothespin's two jaws are loose rigid bodies made of shapes
from `bodykit.py`.

- The sock: one rigid body, a fold resting on the line and two stiff 5 cm sides hanging down either side of it. It
  swings freely about the line. The breeze is a force the task file puts on its lower part after every control step,
  across the line, sinusoidal with a 1.2 s period, up to 0.05 N.
- The clothespin: two jaws joined by a spring hinge (a `Hinge` from `bodykit.py`). While a gripper's TCP is within 2 cm
  of the handle block's middle with its fingers closed on it, the task file drives the moving jaw open to 35 degrees;
  once the fingers open, the spring shuts it.
- The clip: when the pin is let go on the line over the sock while the other gripper holds the sock, the task file
  fixes the pin and the sock where they are (a `Glue` from `bodykit.py`, to a fixed block hidden under the table), and
  the breeze stops pushing. A pin let go over a sock that nobody holds is not fixed.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, with the sock and the pin still) has held for 1 s, otherwise `score()` at the end of the
episode (0, 0.25, 0.5 or 1.0; see `task.json`). `-E object_jitter=0` keeps every object exactly where the task file
puts it, so every trial starts from the same layout. The cameras render at 1280 x 704 for sharp videos; Dream-1's
client scales every frame to 320 x 176 before sending it, so `-E cam_width=320 -E cam_height=176` gives the policy the
same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=pin_a_sock_on_a_line_in_a_breeze.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Hold the sock still on the line with one hand; with the other take the clothespin by its handle, squeeze it open, bring it down astride the line over the sock and let go so it clips on; then let go of the sock." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/sock-dream
```

The scripted demo, which reaches full score: the left hand comes straight down astride the sock near its left end and
pinches its two sides together round the line, holding it still. The right hand comes straight down on the
clothespin's handle, fingers closing across it, which opens the jaws; it lifts the pin out of its holder, carries it
over the line near the sock's right end, brings it down astride the line and the sock, and opens its fingers: the
spring shuts the jaws on the sock. Then both hands let go and rise:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=pin_a_sock_on_a_line_in_a_breeze.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Hold the sock still on the line with one hand; with the other take the clothespin by its handle, squeeze it open, bring it down astride the line over the sock and let go so it clips on; then let go of the sock." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/sock-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/sock-dream/*.json; do inspect-robots video "$log"; done
```

Set `PIN_DEBUG=1` to print what the score is built from twice a second: where the sock and the pin's fulcrum are, how far
the sock has swung, how far the jaws are open, whether the pin is held (and each gripper's finger opening), whether it
is on the line, over the sock and clipped, whether the sock is hung and both hands are clear, and where each TCP is from
the pin's handle and the sock.

## Runtime per trial

On one RTX 5090: about 2 minutes for a Dream-1 trial of 1,200 steps (147 s on the clock, with Dream answering in 0.85 s on average), and about 2 minutes for the scripted demo (352 steps, 17.6 s of simulated time, 122 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.
