# Slide a tray of soup out of a serving pass with both hands, set it on the table, then serve the bowl

A bowl of soup (15 beads) stands on a tray in a two-level serving pass, under the upper shelf, where it cannot be
reached. Both arms take the tray by its side rims, slide it out, lower it flat to the table without spilling the soup,
and then one arm serves the bowl onto a placemat. The task is one task file,
[`unload_a_tray_from_a_pass_shelf.py`](unload_a_tray_from_a_pass_shelf.py), for the `yam_arena` embodiment (scene,
scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from the same folder.

## Requirements

[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) (tested at commit `bd62771`) on
Isaac Lab-Arena 0.3.1 / Isaac Sim 6.1, on Linux with an NVIDIA RTX GPU. Its `setup.sh`, or the program's Brev
launchable, installs everything, including the Dream-1 policy. The task needs no models from Arena's object library.

`yam_arena` task files place single boxes, cylinders and spheres. The rimmed tray, the handled bowl (floor and wall
panels) and each bead are loose rigid bodies built from several shapes, so `bodykit.py` adds such bodies: importing
it hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, and gives `score()` each body's own
pose.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, with the tray, the bowl and every bead still) has held
for 1 s, otherwise `score()` at the end of the episode (0, 0.25, 0.5 or 1.0; see `task.json`). `-E object_jitter=0`
keeps every object exactly where the task file puts it, so every trial starts from the same layout. The cameras render
at 1280 x 704 for sharp videos; Dream-1's client scales every frame to 320 x 176 before sending it, so
`-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=unload_a_tray_from_a_pass_shelf.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "With both hands on the tray's side rims, slide the tray out of the pass and set it down flat on the table without spilling the soup, then put the bowl on the blue placemat." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/tray-dream
```

The scripted demo, which reaches full score: both arms come straight down on the tray's side rims, lift it 1 cm off
the shelf, draw it straight back until it is clear of the shelf, lower it to the table and let go; the right arm then
takes the bowl by its handle and sets it on the placemat:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=unload_a_tray_from_a_pass_shelf.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "With both hands on the tray's side rims, slide the tray out of the pass and set it down flat on the table without spilling the soup, then put the bowl on the blue placemat." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/tray-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/tray-dream/*.json; do inspect-robots video "$log"; done
```

Set `TRAY_DEBUG=1` to print what the score is built from (beads in the bowl; where the tray and the bowl are) once a
second.

## Runtime per trial

On one RTX 5090: about 3 minutes for a Dream-1 trial of 1,200 steps (174 s on the clock, with Dream answering in 0.76 s on average), and about 1 minute for the scripted demo (526 steps, 26.3 s of simulated time, 75 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed and the scripted demo is deterministic. With a nonzero
`object_jitter`, the trial's seed sets how far the free objects (the tray, the bowl and the beads) shift.
