# Press an overfull case's lid shut with one hand while the other pulls its zip closed

An overfull carry case lies on the table, its lid pushed up by what is packed inside. One hand presses the lid shut;
the other takes the zip's pull ring at the front end of the case's right side and pulls it all the way to the back.
A zip only runs where its two sides meet: with the lid up, the slider jams. The task is one task file,
[`press_an_overfull_case_shut_and_zip_it.py`](press_an_overfull_case_shut_and_zip_it.py), for the `yam_arena` embodiment
(scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from the same folder.

Why two hands: the lid springs up as soon as it is let go, and the zip jams while the lid is up. With the lid unheld,
the scripted demo's pull leaves the zip jammed at the front end (0.0; checked by running the demo with the left hand
kept clear). The zip runs 4.8 cm from the lid's edge, out of reach of the fingers of a hand on the pull ring. One hand
holds the lid shut while the other pulls; once a third of the way is zipped, the zip holds the lid by itself and the
pressing hand can let go.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The case, its lid and the zip's slider are rigid
bodies made of shapes from `bodykit.py`; the case is kinematic, fixed where it is put.

- The lid is joined to the case by a hinge along its left top edge (a `Hinge` from `bodykit.py`) with a spring that
  pushes it up (the overfull contents): it rests about 7 degrees up, and about 6 N pressing 12 cm from the hinge holds
  it shut.
- The zip's slider is joined to the case by a slide along the ledge (a `Slider` from `bodykit.py`). After every control
  step the task file reads the lid's angle: while its free edge is more than 3 mm up, the slider is held where it is
  by a stiff spring (jammed); otherwise it runs freely against a little drag. Once the slider is 30% of the way along,
  the task file holds the lid shut with a stiff spring in its hinge (the zipped part holds it).
- The lid and the slider do not collide with the case.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, with the zip still) has held for 1 s, otherwise
`score()` at the end of the episode (0, 0.25, 0.5 or 1.0; see `task.json`). `-E object_jitter=0` keeps every object
exactly where the task file puts it, so every trial starts from the same layout. The cameras render at 1280 x 704 for
sharp videos; Dream-1's client scales every frame to 320 x 176 before sending it, so
`-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=press_an_overfull_case_shut_and_zip_it.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Press the overfull case's lid shut with one hand and with the other pull the zip by its red pull ring from the front end all the way to the back." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/zip-dream
```

The scripted demo, which reaches full score: the left hand comes straight down on the lid, 17 cm left of the zip,
fingers closed, and presses it shut. The right hand, waiting over the pull ring, comes straight down on it, grips it
and pulls it along to the back end. Once the zip is a good way along, the left hand rises and moves out of the way:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=press_an_overfull_case_shut_and_zip_it.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Press the overfull case's lid shut with one hand and with the other pull the zip by its red pull ring from the front end all the way to the back." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/zip-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/zip-dream/*.json; do inspect-robots video "$log"; done
```

Set `ZIP_DEBUG=1` to print what the score is built from (how far the lid is up and the gap at its edge, how far the
zip is along, whether it is jammed and whether it holds the lid, and where each gripper's TCP is) twice a second.

## Runtime per trial

On one RTX 5090: about 2 minutes for a Dream-1 trial of 1,200 steps (141 s on the clock, with Dream answering in 0.73 s on average), and under 1 minute for the scripted demo (276 steps, 13.8 s of simulated time, 41 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.
