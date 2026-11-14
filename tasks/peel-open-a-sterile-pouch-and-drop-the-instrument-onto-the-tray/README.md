# Transfer sterile forceps from their peel pouch onto a sterile drape without contaminating the forceps or the drape

A pair of sterile dressing forceps waits in a peel pouch standing upright in an acrylic holder: a white paper sheet
on the left and a clear film sheet on the right, sealed together along three edges, the forceps lying inside on the
bottom seal. Above the seal the sheets' top flaps are folded out to either side, each ending in a stiff pull tab
(blue on the paper, orange on the film). Beyond the holder lies a sterile instrument tray lined with a green drape;
a waste bin stands on either side. Both hands take the pouch by its tabs, lift it out of the holder and hold it over
the tray, then pull the sheets apart: the seal gives and the forceps drop onto the drape, touched by nothing
non-sterile. Each hand drops its sheet in the bin on its side, and both arms go back to where they started. The
instruction states only the goal and the sterility rules; how to get the forceps out untouched is left to the policy. The task is one task file,
[`peel_open_a_sterile_pouch_and_drop_the_instrument_onto_the_tray.py`](peel_open_a_sterile_pouch_and_drop_the_instrument_onto_the_tray.py),
for the `yam_arena` embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it
imports from the same folder.

What it tests: a two-handed, no-touch transfer onto a sterile field, as circulating nurses and surgical technicians
do many times a day: carrying a light package together, pulling it apart in one line, and keeping the package away
from the field and the hands away from the instrument.

Why two hands: the seal gives only when the two sheets are pulled apart with about 7 N, and nothing holds the pouch
but the hands. With one hand (the right taking the film's tab, lifting, carrying and pulling as in the demo, the left
never moving) the demo scored 0.0: the whole pouch hung from the one tab and followed the hand, and the seal never
opened.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The holder, the tray and the bins are kinematic,
fixed where they are put; the two sheets and the forceps are loose rigid bodies made of shapes from `bodykit.py`.

- The sheets: stiff 2 mm plates, 1 cm apart. The paper carries the bottom seal and the front and back seals across
  the gap, and each sheet has its top flap folded out, rigid, with its pull tab at the end. The film is drawn clear:
  an invisible collision plate with blue seal lines printed round its edges, so the forceps show through.
- The seal: a prismatic joint (a `Slider` from `bodykit.py`) lets the film move away from the paper, sideways only,
  against a 600 N/m spring; every other way the two are held together, so the pouch carries as one piece. After
  every control step the task file measures how far apart the sheets have been pulled; at 12 mm it switches the
  joint off, and the sheets come apart.
- The forceps rest on the bottom seal. Once the sheets are a few millimetres further apart than the forceps are
  thick, nothing holds them and they drop.
- Contamination is tracked after every control step, for the whole trial, from geometry: points on the grippers'
  collision meshes (the gripper body and both fingers, one per 4 mm cell, read from the robot's USD asset and kept in
  [`yam_gripper_hull.py`](yam_gripper_hull.py)) are checked against the forceps and the tray (within 3 mm), points
  over every surface of both sheets (faces, flaps, tabs) against the tray, drape and rim, and points over the
  forceps against the table, the holder, the bins, the tray's rim and the sheets' outer faces, flaps and tabs
  (within 1.5 mm). The first contact latches the score at 0.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, which includes both arms back where they started, with
the forceps and both sheets still) has held for 1 s, otherwise `score()` at the end of the episode (0, 0.25, 0.5 or
1.0; see `task.json`). `-E object_jitter=0` keeps every object exactly where the task file puts it, so every trial
starts from the same layout. The cameras render at 1280 x 704 for sharp videos; Dream-1's client scales every frame
to 320 x 176 before sending it, so `-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=peel_open_a_sterile_pouch_and_drop_the_instrument_onto_the_tray.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Transfer the sterile forceps from their peel pouch onto the green drape without contaminating them: only the forceps, the inside of the pouch and the drape are sterile, so the forceps may touch nothing else, and nothing but the forceps may touch the drape or the tray. Then put both pouch sheets in the waste bins and take both arms back to where they started." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/pouch-dream
```

The scripted demo, which reaches full score: both hands come straight down on the pull tabs, fingers closing across
them, and together lift the pouch 5 cm out of the holder and carry it 21 cm forward, over the tray. Then they pull
the sheets apart, the left hand to the left and the right hand to the right: the seal gives, and the forceps drop onto
the drape. Each hand carries its sheet to the waste bin on its side and drops it in, and both arms go back to where
they started:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=peel_open_a_sterile_pouch_and_drop_the_instrument_onto_the_tray.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Transfer the sterile forceps from their peel pouch onto the green drape without contaminating them: only the forceps, the inside of the pouch and the drape are sterile, so the forceps may touch nothing else, and nothing but the forceps may touch the drape or the tray. Then put both pouch sheets in the waste bins and take both arms back to where they started." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/pouch-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/pouch-dream/*.json; do inspect-robots video "$log"; done
```

Set `POUCH_DEBUG=1` to print what the score is built from twice a second: how far apart the sheets have been pulled
and whether the seal has given, where the forceps are and whether they lie on the drape, how close any gripper has
come to them, whether (and how) anything has been contaminated, where each sheet is and whether both are in the bins,
and where each TCP is.

## Runtime per trial

On one RTX 5090: about 5 minutes for a Dream-1 trial of 1,200 steps (321 s on the clock, with Dream answering in 0.73 s on average), and about 1 minute for the scripted demo (384 steps, 19.2 s of simulated time, 88 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.
