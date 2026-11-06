# Hold a hand coffee grinder steady with one hand and turn its crank three times with the other

A light manual coffee grinder stands loose on the table. Its crank turns against the burrs' resistance, so one arm has
to hold the grinder's body while the other turns the crank three full turns clockwise; the knob spins freely on its
pin. Turned with one hand, the whole grinder spins round, slides and wobbles instead. The task is one task file,
[`grind_coffee_in_a_hand_grinder.py`](grind_coffee_in_a_hand_grinder.py), for the `yam_arena` embodiment (scene,
scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from the same folder.

## Requirements

[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) (tested at commit `bd62771`) on
Isaac Lab-Arena 0.3.1 / Isaac Sim 6.1, on Linux with an NVIDIA RTX GPU. Its `setup.sh`, or the program's Brev
launchable, installs everything, including the Dream-1 policy. The task needs no models from Arena's object library.

`yam_arena` task files place single boxes, cylinders and spheres, with no joints. The grinder's body, the crank (hub and
arm) and the knob are loose rigid bodies from `bodykit.py`, which also joins two bodies with a PhysX revolute joint
(`Hinge`), here with a damping drive on the crank's axle for the grinding resistance. Importing it hooks
`yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()` each body's own pose, and
lets the task file look at the simulation after every control step, where it follows the crank's turn.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, with the grinder still) has held for 1 s, otherwise
`score()` at the end of the episode (0, 0.25, 0.5 or 1.0; see `task.json`). The crank's angle against the grinder's
body is read after every control step (it turns well under half a turn a step) and the changes are added up, so
spinning the whole grinder round does not count, and turning back undoes turns. `-E object_jitter=0` keeps every object
exactly where the task file puts it, so every trial starts from the same layout. The cameras render at 1280 x 704 for
sharp videos; Dream-1's client scales every frame to 320 x 176 before sending it, so
`-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=grind_coffee_in_a_hand_grinder.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Hold the coffee grinder steady with one hand and turn its crank three full turns clockwise with the other." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/grind-dream
```

The scripted demo, which reaches full score: the left arm slides in level from its side and grips the grinder's body
14 cm up; the right arm comes straight down on the knob and grips it near its top, then carries it round the axis,
clockwise, through three and a quarter turns at one turn every 3.2 s:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=grind_coffee_in_a_hand_grinder.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Hold the coffee grinder steady with one hand and turn its crank three full turns clockwise with the other." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/grind-demo
```

The same demo with the left arm left at rest (one hand only) turns the crank only about two thirds of a turn against
the body, which spins half round and wobbles as the knob is carried round, and scores 0.

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/grind-dream/*.json; do inspect-robots video "$log"; done
```

Set `GRIND_DEBUG=1` to print what the score is built from (turns so far, whether the grinder stands, where it is and
how far it has spun) once a second.

## Runtime per trial

On one RTX 5090: about 2 minutes for a Dream-1 trial of 1,200 steps (143 s on the clock, with Dream answering in 0.67 s on average), and about 1 minute for the scripted demo (429 steps, 21.4 s of simulated time, 66 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed and the scripted demo is deterministic. Keep it at 0: a
nonzero `object_jitter` shifts the grinder, its crank and its knob each by its own random amount, and the joints
between them would snap them back together at the first step.
