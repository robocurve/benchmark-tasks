# Drain a pan of dumplings into the sink with both hands, holding the lid ajar

A saucepan of dumplings in their cooking water (beads) stands on a stove block with its lid on. One arm holds the lid
slid slightly back, so that a narrow gap opens at the front rim, while the other lifts the pan by its handle and tips
it over the sink: the water runs out through the gap, the dumplings stay in. Then the pan goes back on the stove.
Tipped with the lid loose, the lid slides off and the dumplings follow the water, so the task needs both arms. The task
is one task file, [`drain_a_pot_through_a_lid_held_ajar.py`](drain_a_pot_through_a_lid_held_ajar.py), for the
`yam_arena` embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from
the same folder.

## Requirements

[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) (tested at commit `bd62771`) on
Isaac Lab-Arena 0.3.1 / Isaac Sim 6.1, on Linux with an NVIDIA RTX GPU. Its `setup.sh`, or the program's Brev
launchable, installs everything, including the Dream-1 policy. The task needs no models from Arena's object library.

`yam_arena` task files place single boxes, cylinders and spheres. The pan (floor, wall panels and handle), the lid
(disc and knob) and each bead are loose rigid bodies built from several shapes, so `bodykit.py` adds such bodies:
importing it hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, and gives `score()` each
body's own pose.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, with the pan and every bead still) has held for 1 s,
otherwise `score()` at the end of the episode (0, 0.25, 0.5 or 1.0; see `task.json`). `-E object_jitter=0` keeps every
object exactly where the task file puts it, so every trial starts from the same layout. The cameras render at
1280 x 704 for sharp videos; Dream-1's client scales every frame to 320 x 176 before sending it, so
`-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=drain_a_pot_through_a_lid_held_ajar.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Drain the water into the sink but keep the dumplings in the pan: hold the lid slightly ajar with one hand while the other tips the pan over the sink by its handle, then set the pan back on the stove." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/drain-dream
```

The scripted demo, which reaches full score: the right arm grips the lid's knob high up and slides the lid 2.5 cm back,
opening a gap of about 1.2 cm at the front rim, and keeps it pressed on; the left arm slides in from the left and grips
the handle with its wrist rolled so that the fingers close top and bottom (the pan's weight then turns it about an axis
the finger pads hold). Together they lift the pan 3 cm, carry it 4 cm forward and tip it 100 degrees about its front
rim over the sink, rock it a little to free the last of the water, stand it up and set it back on the stove. While the
pan is off the stove each hand aims 1 cm into the other along the pan's axis, and the left one aims the pan 10 degrees
back against the sag of its soft wrist joints, so that the lid stays where the right hand put it:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=drain_a_pot_through_a_lid_held_ajar.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Drain the water into the sink but keep the dumplings in the pan: hold the lid slightly ajar with one hand while the other tips the pan over the sink by its handle, then set the pan back on the stove." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/drain-demo
```

The same demo with the right arm left at rest (the left hand tips the pan alone) scores 0: the lid slides off as the
pan tips and all six dumplings fall into the sink with the water.

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/drain-dream/*.json; do inspect-robots video "$log"; done
```

Set `DRAIN_DEBUG=1` to print what the score is built from (water beads in the sink and in the pan, dumplings in the
pan, how far the pan is tipped, where the lid sits on it) once a second.

## Runtime per trial

On one RTX 5090: about 3 minutes for a Dream-1 trial of 1,200 steps (177 s on the clock, with Dream answering in 0.81 s on average), and about 2 minutes for the scripted demo (616 steps, 30.8 s of simulated time, 105 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed and the scripted demo is deterministic. With a nonzero
`object_jitter`, the trial's seed sets how far the free objects (the pan, the lid and the beads) shift.
