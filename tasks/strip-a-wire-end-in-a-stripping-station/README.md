# Strip 10 mm off a cable in a benchtop stripper, one hand working the cable and the other holding the blades shut

A steel benchtop wire-stripping station stands on the table: a block with a hole in its top, whose blades close round
whatever is in the hole while its lever is held down. A short PVC-insulated cable with a plastic plug on one end stands upright in a holder
beside it. One hand takes the cable by its plug and puts its free end 10 mm down into the hole; the other presses the
lever down, closing the blades, and keeps it down while the first pulls the cable straight back out. The cut
insulation stays in the station and 10 mm of copper is bare. The task is one task file,
[`strip_a_wire_end_in_a_stripping_station.py`](strip_a_wire_end_in_a_stripping_station.py), for the `yam_arena`
embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from the same
folder.

What it tests: setting an insertion depth to within 2 mm, and two hands working at once on one tool, one holding it
shut while the other pulls.

Why two hands: the blades hold the cut insulation only while the lever is held down, and the cable has to be pulled
out at the same time; if the lever comes up first, the cut insulation comes back out on the cable. One hand keeps the
lever down while the other pulls the cable, so a single hand cannot strip it.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The station (a block with a funnel and a tube down
into it) and the cable's holder are kinematic, fixed where they are put; the lever turns on a hinge (a `Hinge` from
`bodykit.py`) with a spring that holds it up. The cable is a rigid body; the insulation over its last 24 mm is 12 rings
of 2 mm, each held to the cable by a fixed joint (a `Glue` from `bodykit.py`), shown but not colliding.

- The cut: the first time the lever closes with the cable's end in the hole, the task file switches the rings below
  the blades (the station's top) from the cable to the station, fixed where they are.
- Stripped: if the cable's end comes up out past the blades while the lever is still down, the cut rings stay in the
  station. If the lever comes up first, they go back onto the cable.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, with the lever and the cable still) has held for 1 s,
otherwise `score()` at the end of the episode (0, 0.25, 0.5 or 1.0; see `task.json`). `-E object_jitter=0` keeps every
object exactly where the task file puts it, so every trial starts from the same layout. The cameras render at
1280 x 704 for sharp videos; Dream-1's client scales every frame to 320 x 176 before sending it, so
`-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=strip_a_wire_end_in_a_stripping_station.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Take the cable by its plug and put its free end down into the stripping station's hole, 10 mm deep; with the other hand press the station's lever down and keep it down while the first hand pulls the cable straight back out; then let the lever up." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/strip-dream
```

The scripted demo, which reaches full score: the left hand comes straight down on the cable's plug, fingers closing
across it, lifts the cable straight out of its holder, carries it over the station and lowers its end 10 mm into the
hole. The right hand comes straight down on the lever's rubber grip, fingers closed, presses the lever down and holds
it while the left pulls the cable straight up out of the hole. Then the right rises and the lever springs back up:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=strip_a_wire_end_in_a_stripping_station.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Take the cable by its plug and put its free end down into the stripping station's hole, 10 mm deep; with the other hand press the station's lever down and keep it down while the first hand pulls the cable straight back out; then let the lever up." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/strip-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/strip-dream/*.json; do inspect-robots video "$log"; done
```

Set `STRIP_DEBUG=1` to print what the score is built from (the lever's angle, how deep the cable's end is below the
blades and how far off the hole's axis, how many rings were cut, whether it is stripped, and where each gripper's TCP
is from the cable's end and from the lever's grip) twice a second.

## Runtime per trial

On one RTX 5090: about 3 minutes for a Dream-1 trial of 1,200 steps (180 s on the clock, with Dream answering in 0.91 s on average), and about 2 minutes for the scripted demo (514 steps, 25.7 s of simulated time, 105 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.
