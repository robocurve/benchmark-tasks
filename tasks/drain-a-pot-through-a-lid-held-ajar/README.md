# Drain the water off a pan of dumplings into the sink, keeping every dumpling in the pan

A saucepan of dumplings in their cooking water stands on a stove block with its lid on (a real trial uses 300 ml of
water; the simulation stands in 20 beads of 15 ml each). One arm holds the lid slid slightly back, so that a narrow
gap opens at the front rim, while the other lifts the pan by its handle and tips it over the sink: the water runs out
through the gap into the sink, the dumplings stay in. Then the pan goes back on the stove. Tipped with the lid loose,
the lid slides off and the dumplings follow the water, so the task needs both arms. Only the water collected in the
sink counts as drained; water poured anywhere else is a spill. The task is one task file, [`drain_a_pot_through_a_lid_held_ajar.py`](drain_a_pot_through_a_lid_held_ajar.py), for the
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
otherwise `score()` at the end of the episode (0, 0.25, 0.5 or 1.0 by the water collected in the sink, the dumplings
left in the pan and where the pan ends; see `task.json`). `-E object_jitter=0` keeps every object exactly where the
task file puts it, so every trial starts from the same layout. The cameras render at 1280 x 704 for sharp videos;
Dream-1's client scales every frame to 320 x 176 before sending it, so `-E cam_width=320 -E cam_height=176` gives the
policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=drain_a_pot_through_a_lid_held_ajar.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Drain the water from the pan into the sink but keep all the dumplings in the pan, then set the pan back on the stove." \
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
  --instruction "Drain the water from the pan into the sink but keep all the dumplings in the pan, then set the pan back on the stove." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/drain-demo
```

The same demo with the right arm left at rest (the left hand tips the pan alone) scores 0: the lid slides off as the
pan tips and all six dumplings fall into the sink with the water.

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/drain-dream/*.json; do inspect-robots video "$log"; done
```

Set `DRAIN_DEBUG=1` to print what the score is built from (water beads in the sink, in the pan and spilled, also in
ml; dumplings in the pan and in the sink; how far the pan is tipped; where the lid sits on it) once a second.

## A real trial

The rig: a 12 cm steel milk pan (inside 11 cm across, 7.5 cm deep) with a flat lid that overhangs its rim by 1 cm and a
straight handle, on a cold stove block 13 cm tall, and in front of it the sink: a small plugged collecting basin with
no drain, inside 16 cm square and 6.6 cm deep, smaller than a kitchen sink. A real trial uses a basin of that size (a
square plastic tub serves), laid out as in the setup. In the pan: 300 ml of room-temperature tap water and six
cooked potato gnocchi or mini dumplings about 2.5 cm across. The basin starts empty and dry.

What counts as drained is the water actually collected in the sink, never the water the pan lost:

1. Lift any dumplings out of the basin with a slotted spoon, shake them over it and count them. They are lost
   dumplings and their weight never counts as water.
2. Pour the water in the basin into a measuring jug (10 ml marks), or weigh the basin with its water on a kitchen
   scale (1 g resolution) and take off its dry weight (1 g = 1 ml).
3. Count the dumplings in the pan and check that the pan stands back on the stove.

Water on the table, the stove, the floor or the outside of the basin is spilled and earns nothing, and so does water
left in the pan.

| Score | Real trial | Simulation (each water bead = 15 ml) |
| --- | --- | --- |
| 1.0 | at least 240 ml collected in the sink, all 6 dumplings in the pan | at least 16 of the 20 water beads inside the basin, all 6 dumplings in the pan |
| 0.5 | at least 150 ml collected, at least 5 dumplings in the pan | at least 10 beads inside the basin, at least 5 dumplings in the pan |
| 0.25 | at least 75 ml collected, all 6 dumplings in the pan | at least 5 beads inside the basin, all 6 dumplings in the pan |
| 0.0 | anything less, or the pan not back on the stove (tilted 10 degrees or more, or its centre more than 4 cm off) | the same |

In simulation a water bead is collected only if its centre ends inside the basin: inside its four walls, above its
floor and below its rim. Dumplings that end in the sink are counted separately and are lost dumplings, not water.

### Spill check

Pouring the water anywhere but into the sink earns no drain credit. The same scripted demo, altered to carry the pan
14.5 cm to the left of the sink and tip it there (the water lands on the table beside the sink), scores 0.0: at the
end 0 of the 20 water beads are in the sink (0 ml drained), 20 are spilled (300 ml), and all 6 dumplings are still in
the pan. A scorer check with the pan standing on the stove and all 20 beads placed on the table beside the sink, on
the table in front of it, or on top of its wall likewise gives 0 ml drained and a score of 0.0, while the same beads
inside the basin give 300 ml and 1.0.

## Runtime per trial

On one RTX 5090: about 8 minutes for a Dream-1 trial of 1,200 steps (507 s on the clock, with Dream answering in 0.77 s on average), and about 5 minutes for the scripted demo (616 steps, 30.8 s of simulated time, 282 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed and the scripted demo is deterministic. With a nonzero
`object_jitter`, the trial's seed sets how far the free objects (the pan, the lid and the beads) shift.

## Changes in this revision

This is a revision of TK-9GWQ55. Its review said:

> Construct validity (not met): Weighing the pan before and after can award credit for water deliberately spilled outside the sink.
>
> Scoring (not met): The proposed mass-loss and remaining-volume measurements do not establish how much water actually reached the sink, and mass loss also includes lost dumplings.
>
> What to change: Measure the water actually collected in the sink, rather than inferring it solely from pan mass loss or remaining volume. Exclude spills and account for any dumplings collected with the water so that the stated partial-credit thresholds remain accurate.

The review before that (TK-ML6LPR) asked for actual water and representative dumplings in the real-world setup, a stated initial water volume, measurable drained-volume thresholds for each score, and a check of the full-score demo's final contents and pan placement; that was done in the previous revision (300 ml of water, six cooked gnocchi or mini dumplings, 240 / 150 / 75 ml tiers) and is kept, with the drained volume now measured as described below.

What changed in response:

- **Construct validity: no credit for water spilled outside the sink.** The drained amount is now only the water actually collected in the sink, never the water the pan lost. The sink basin has no drain (a plugged sink, inside 16 cm square and 6.6 cm deep) and starts empty and dry. Water that lands on the table, the stove, the floor or the outside of the basin is a spill and never counts, and neither does water left in the pan. Pan weighing and remaining-volume measurements are no longer used anywhere.
- **Scoring: measure the water that reached the sink, and keep dumplings out of it.** After the trial any dumplings in the sink are lifted out with a slotted spoon, shaken over the basin and counted; they are lost dumplings and are never counted as water. The collected water is then poured into a measuring jug (10 ml marks) or weighed in the basin on a kitchen scale (1 g resolution, the dry basin's weight taken off, 1 g = 1 ml). The partial-credit thresholds are unchanged and now apply to that collected volume: 1.0 needs at least 240 ml of the 300 ml collected in the sink and all 6 dumplings in the pan; 0.5 at least 150 ml and at least 5 dumplings in the pan; 0.25 at least 75 ml and all 6 dumplings in the pan; each needs the pan back on the stove (tilted less than 10 degrees, centre within 4 cm).
- **The simulation measures the same thing.** Each of the 20 water beads stands for 15 ml and counts only if its centre ends inside the basin (inside its four walls, above its floor and below its rim): 16, 10 and 5 beads are exactly 240, 150 and 75 ml. Beads anywhere else count as spilled. Dumplings that end in the sink are counted separately, as lost dumplings, never as water. The scoring text, README (a table pairs each tier's real-trial measurement with its simulation count), code and fidelity note agree.
- **Spill test, as asked: pouring outside the sink scores no drain credit.** The scripted demo altered to carry the pan 14.5 cm to the left of the sink and tip it there (the water lands on the table beside the sink) scores 0.0: 0 of the 20 water beads in the sink (0 ml drained), 20 spilled (300 ml), all 6 dumplings still in the pan. A scorer check with the pan standing back on the stove and all 20 beads on the table beside the sink, on the table in front of it, or on top of its wall also gives 0 ml drained and 0.0, while the same beads inside the basin give 300 ml and 1.0. The README's "A real trial" section reports this.
- **Demo under the revised scorer:** 1.0 (18 of the 20 water beads, 270 ml, collected in the sink; 1 bead left in the pan and 1 spilled; all 6 dumplings in the pan, none in the sink; the pan back on the stove). The setup now also says plainly that the sink is a small plugged collecting basin, 16 cm square inside and 6.6 cm deep, smaller than a kitchen sink, and that a real trial uses a basin of that size; the scene is unchanged. New demo and Dream-1 videos.
