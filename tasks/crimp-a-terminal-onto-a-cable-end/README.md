# Hold a cable's stripped end in a crimper's terminal with one hand while the other presses the lever to crimp it

A steel bench crimper stands on the table with a ring terminal seated in its die, the terminal's barrel pointing toward the
arms. A short PVC-insulated cable with a stripped end lies on a plastic rest beside it. One hand picks up the cable, pushes its bare end
straight into the barrel to the right depth and holds it there; the other presses the crimper's lever down until it
closes, crimping the terminal onto the cable, and lets it spring open again. Then the first hand pulls the cable out
with the terminal crimped on its end, holds it up to show it and lays it back on its rest, and both arms go back to
where they started. The task is one task file,
[`crimp_a_terminal_onto_a_cable_end.py`](crimp_a_terminal_onto_a_cable_end.py), for the `yam_arena` embodiment (scene,
scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from the same folder.

What it tests: inserting a cable to a set depth and keeping it straight in a loose fit while the other hand works a
lever.

Why two hands: until it is crimped, the cable is loose in the barrel (13 mm across, the copper 6 mm), and the 16 cm of
cable outside tips it. Let go of, it tips: with the left hand letting go right after pushing the cable in, the scripted
demo's crimp catches it tipped 15 degrees, a bad crimp (0.5). One hand holds the cable straight in the barrel while
the other presses the lever.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The crimper (base, die, a cone of panels in front of
the terminal's mouth as a lead-in, and the lever's bracket), the terminal (its barrel and tongue), the lever, the
cable's rests and the cable are rigid bodies made of shapes from `bodykit.py`; the crimper, the terminal and the rests
are kinematic, fixed where they are put.

- The lever turns on a hinge (a `Hinge` from `bodykit.py`) between stops 12 degrees open and closed, with a spring that
  holds it open. Its upper die does not collide; the lever only has to be pressed closed.
- The crimp: the first time the lever closes (within 1.5 degrees), the task file reads how deep the cable's bare end is
  in the barrel and how far the cable is tipped from the barrel's axis, and, if the bare end is in the barrel, swaps the
  terminal for a crimped one (its barrel drawn squeezed onto the copper, shown only), which from then on it moves with
  the cable after every control step, so the cable comes out of the crimper with the terminal on it.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, which includes the cable laid back on its rest and both
arms back where they started, with the lever and the cable still) has held for 1 s,
otherwise `score()` at the end of the episode (0, 0.25, 0.5 or 1.0; see `task.json`). `-E object_jitter=0` keeps every
object exactly where the task file puts it, so every trial starts from the same layout. The cameras render at
1280 x 704 for sharp videos; Dream-1's client scales every frame to 320 x 176 before sending it, so
`-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=crimp_a_terminal_onto_a_cable_end.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Pick up the cable and push its bare end straight into the terminal's barrel in the crimper with one hand, and hold it there while the other presses the crimper's lever down until it closes; let the lever spring open, then pull the cable straight out with the terminal crimped on it, hold it up to show it, lay it back on its rest, and take both arms back to where they started." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/crimp-dream
```

The scripted demo, which reaches full score: the left hand comes straight down on the cable 5.5 cm from its tip,
fingers closing across it, lifts it off its rests, lines it up with the barrel 2 cm in front of the mouth and pushes
the bare end straight in, 1 cm deep, and holds it. The right hand comes straight down on the lever's rubber grip,
fingers closed, presses the lever down until it closes, holds it a moment and rises; the lever springs open, and the
right goes back to where it started. The left pulls the cable straight out with the terminal crimped on its end, holds
it up to show it, lays it back on its rest and goes back to where it started:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=crimp_a_terminal_onto_a_cable_end.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Pick up the cable and push its bare end straight into the terminal's barrel in the crimper with one hand, and hold it there while the other presses the crimper's lever down until it closes; let the lever spring open, then pull the cable straight out with the terminal crimped on it, hold it up to show it, lay it back on its rest, and take both arms back to where they started." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/crimp-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/crimp-dream/*.json; do inspect-robots video "$log"; done
```

Set `CRIMP_DEBUG=1` to print what the score is built from (the lever's angle, how deep the bare end is in the barrel,
the cable's angle from the barrel's axis and its direction, how far its tip is off that axis, whether and how it was
crimped, and where each gripper's TCP is from the cable's tip and from the lever's grip) twice a second.

## Runtime per trial

On one RTX 5090: about 2 minutes for a Dream-1 trial of 1,200 steps (144 s on the clock, with Dream answering in 0.83 s on average), and about 1 minute for the scripted demo (503 steps, 25.1 s of simulated time, 76 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.
