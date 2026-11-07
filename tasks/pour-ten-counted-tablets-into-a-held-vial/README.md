# Hold a vial up to a counting tray's chute with one hand while the other tips the tray to pour ten counted tablets

A plastic pharmacy counting tray lies on the table. Ten counted tablets wait in the trough along its right side, which opens at
the back into a short chute; six more lie on the tray's floor, kept out of the trough by a low divider. One hand holds
an empty plastic vial up under the chute; the other lifts the tray by its grip fin and tips it back, so the ten counted tablets
slide down the trough and out of the chute into the vial, while the extras stay on the tray. Too many or too few in the
vial is a miscount. The task is one task file,
[`pour_ten_counted_tablets_into_a_held_vial.py`](pour_ten_counted_tablets_into_a_held_vial.py), for the `yam_arena`
embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from the same
folder.

What it tests: an exact count, kept by pouring only the trough and never the tray's floor (tipped sideways or too far,
the extras can slide over the divider or the rim), with the vial held right up to the chute.

Why two hands: this is how pharmacy staff are taught to do it, and the score asks for it. The vial is held up to the
chute, off the counter, when the tablets go in, so that none bounce out; poured into a vial standing on the table, the
count scores 0.5 at most. One hand holds the vial while the other lifts and tips the tray.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The tray (floor, rims, divider, chute and grip fin)
and the vial (a square open box) are rigid bodies made of shapes from `bodykit.py`; each tablet is a small rigid disc.
The tablets are slick, like film-coated tablets: they take the lower of the two frictions in every contact (0.15), so
they slide down the trough once the tray is tipped back past about 9 degrees.

The task file notes how high the vial's underside is when the first tablet enters it.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, with everything still) has held for 1 s, otherwise
`score()` at the end of the episode (0, 0.25, 0.5 or 1.0; see `task.json`). `-E object_jitter=0` keeps every object
exactly where the task file puts it, so every trial starts from the same layout. The cameras render at 1280 x 704 for
sharp videos; Dream-1's client scales every frame to 320 x 176 before sending it, so
`-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=pour_ten_counted_tablets_into_a_held_vial.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Pour the ten counted tablets in the counting tray's trough into the vial, and none of the others: hold the vial up off the table under the tray's chute with one hand, and with the other lift the tray by its grip fin and tip it back." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/tablets-dream
```

The scripted demo, which reaches full score: the right hand comes straight down on the vial, fingers closing on its
left and right, lifts it and holds it up where the chute will pour. The left hand comes straight down on the tray's grip
fin, grips it and lifts the tray, tipped back a little (6 degrees: the tablets stay put), and brings the chute in
between the right hand's fingers until its lip is over the vial. Then it tips the tray on back: the ten tablets in the
trough slide out into the vial, and the extras slide only to the back of the tray's floor. Then it tips the tray
forward again and draws it away:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=pour_ten_counted_tablets_into_a_held_vial.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Pour the ten counted tablets in the counting tray's trough into the vial, and none of the others: hold the vial up off the table under the tray's chute with one hand, and with the other lift the tray by its grip fin and tip it back." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/tablets-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/tablets-dream/*.json; do inspect-robots video "$log"; done
```

Set `TAB_DEBUG=1` to print what the score is built from (where the tray and the vial are, how far the tray is tipped
back, where the chute's lip is from the vial, how many tablets are in the vial, on the tray and spilt, and how high the
vial's underside was when the first tablet went in) twice a second.

## Runtime per trial

On one RTX 5090: about 3 minutes for a Dream-1 trial of 1,200 steps (167 s on the clock, with Dream answering in 0.81 s on average), and about 1 minute for the scripted demo (476 steps, 23.8 s of simulated time, 74 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.
