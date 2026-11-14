# Crimp a tube lug onto a 16 mm² cable in a bench crimping press, then lay the cable back on its rest

A pneumatic bench crimping press for cable lugs stands on the table with a tinned-copper tube lug for 16 mm² cable
seated in its 16 mm² hexagonal die, the lug's barrel (6 mm inside, 16 mm long) pointing toward the arms. A short
16 mm² single-core PVC cable (8 mm across; its stranded copper, 5 mm across, stripped 16 mm, the barrel's length)
lies on a plastic rest beside it. The instruction gives only the goal. In the demo, one hand picks up the cable, pushes its
bare end straight into the barrel until the insulation, too wide to enter, stops on the lead-in just in front of the
barrel's mouth (14.2 mm of the 16 mm copper in the barrel, measured), and holds it there; the other presses the
crimper's lever down until it closes, crimping the terminal onto the cable, and lets it spring open again. Then the
first hand pulls the cable out with the terminal crimped on its end, holds it up to show it and lays it back on its
rest, and both arms go back to where they started. The task is one task file,
[`crimp_a_terminal_onto_a_cable_end.py`](crimp_a_terminal_onto_a_cable_end.py), for the `yam_arena` embodiment (scene,
scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from the same folder.

What it tests: inserting a cable fully and straight into a close-fitting lug barrel (0.5 mm of play round the
copper), working a press lever, and taking the crimped cable out and putting it back where it lay.

Why it matters: power conductors of this size are terminated with crimped compression lugs, which then bolt onto
breakers, busbars, transformers and motor terminals. Lugging a cable end is part of the O*NET task statements for
electricians "Connect wires to circuit breakers, transformers, or other components" and "Install ground leads and
connect power cables to equipment, such as motors" (O*NET 47-2111.00). Electricians held about 821,000 US jobs in
2025 (BLS Occupational Outlook Handbook), and electrical and electronic equipment assemblers, who build control
panels and switchgear, another 246,300. Getting the strip length and the insertion right (the copper fully in the
barrel, the insulation at its mouth, the cable straight while the die closes) is what separates a crimp that passes
the pull test from one that pulls out or runs hot.

Two hands: the press's lever is to the right of the die and the cable's rest to the left, so the demo holds the cable
in the barrel with the left hand while the right closes the lever. With the matched lug the fit itself keeps the cable
in line: with the left hand letting go right after pushing the cable in, the crimp still caught it 13.9 mm deep and 3
degrees off its axis (a good crimp; that check then scored 0.5 only because nothing took the cable out afterwards). So
the second hand is not forced by the physics here, only by where the parts are.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The crimper (base, die, a cone of panels in front of
the terminal's mouth as a lead-in, and the lever's bracket), the terminal (its barrel and tongue), the lever, the
cable's rests and the cable are rigid bodies made of shapes from `bodykit.py`; the crimper, the terminal and the rests
are kinematic, fixed where they are put.

- The lever turns on a hinge (a `Hinge` from `bodykit.py`) between stops 12 degrees open and closed, with a spring that
  holds it open. Its upper die does not collide; the lever only has to be pressed closed. On the real press, closing the
  lever fully runs its air cylinder through one crimp; the cylinder is not modelled.
- The crimp: the first time the lever closes (within 1.5 degrees), the task file reads how deep the cable's bare end is
  in the barrel and how far the cable is tipped from the barrel's axis, and, if the bare end is in the barrel, swaps the
  terminal for a crimped one (its barrel drawn squeezed onto the copper, shown only), which from then on it moves with
  the cable after every control step, so the cable comes out of the crimper with the lug on it. The simulation does not test the crimp's hold; a real
  trial adds a pull test (the lug must hold a steady 450 N pull on the cable, the UL 486A pull-out force for this size).

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
  --instruction "Crimp the cable lug sitting in the crimping press onto the cable's stripped end, then lay the cable, lug on, back on its rest and take both arms back to where they started." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/crimp-dream
```

The scripted demo, which reaches full score: the left hand comes straight down on the cable 5.5 cm from its tip,
fingers closing across it, lifts it off its rests, lines it up with the barrel 2 cm in front of the mouth and pushes
the bare end straight in, aiming 19 mm past the mouth (3 mm past full depth), so that the insulation stopping on the
lead-in sets the depth, and holds it. Measured in the demo: 14.2 mm of the copper in the barrel when the die closed
(inside the 13 to 16.5 mm window for full credit), the insulation's end 1.8 mm in front of the mouth, and the cable
2.5 degrees off the barrel's axis. The right hand comes straight down on the lever's rubber grip,
fingers closed, presses the lever down until it closes, holds it a moment and rises; the lever springs open, and the
right goes back to where it started. The left pulls the cable straight out with the terminal crimped on its end, holds
it up to show it, lays it back on its rest and goes back to where it started:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=crimp_a_terminal_onto_a_cable_end.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Crimp the cable lug sitting in the crimping press onto the cable's stripped end, then lay the cable, lug on, back on its rest and take both arms back to where they started." \
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

On one RTX 5090: about 7 minutes for a Dream-1 trial of 1,200 steps (409 s on the clock, with Dream answering in 0.59 s on average), and about 1 minute for the scripted demo (503 steps, 25.1 s of simulated time, 77 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.

## Changes in this revision

This is a revision of TK-K6ECAU. Its review said:

> Flags: Prior revision feedback was not provided
>
> Importance (not met): Crimping cable lugs is regular work for electricians and panel-building assemblers, but the submission selects Other rather than Impact on jobs.
>
> What to change: Change the category to Impact on jobs and explain how cable-lug crimping forms part of electricians' regular work. Please also reconcile the README's stated 10 mm demo insertion with the 13 mm minimum for full credit; provide the measured demo depth and, if it did not reach the threshold, a full-score demonstration.

The review before that (TK-Y3RQVT) asked for a matched cable, ring terminal and crimper with a realistic insertion depth and crimping setup, full-credit scoring that checks the completed crimp (for example a specified pull test and observable insertion marks), and an updated demo. That was done in the previous revision and is kept: a 16 mm² cable stripped 16 mm in a 16 mm² tube lug with a 6 mm barrel, a 16 mm² hexagonal die, 13 to 16.5 mm insertion for full credit and a 450 N pull test in a real trial. The caliper reading of the insulation's gap at the barrel's mouth, added now, is the observable insertion mark.

What changed in response:

- **Importance / category:** the category is now Impact on jobs. The previous review had suggested Other; this follows the latest review. The context and a new "Why it matters" paragraph explain the job. Power conductors of this size are terminated with crimped compression lugs, which then bolt onto breakers, busbars, transformers and motor terminals. Two O*NET task statements for electricians (47-2111.00) cover this: "Connect wires to circuit breakers, transformers, or other components" and "Install ground leads and connect power cables to equipment, such as motors". Electricians held about 821,000 US jobs in 2025 (BLS Occupational Outlook Handbook). Electrical and electronic equipment assemblers, who include control-panel builders, held another 246,300.
- **Demo depth vs. the 13 mm minimum:** the README's "1 cm deep" was left over from an earlier version and did not describe the demo. The demo hand aims 19 mm past the barrel's mouth, 3 mm past full depth, so the depth is set by the insulation, which is too wide for the 6 mm barrel and stops on the lead-in. Measured in a fresh run of the demo:
  - 14.2 mm of the 16 mm copper was in the barrel when the die closed;
  - the insulation's end sat 1.8 mm in front of the mouth;
  - the cable was 2.5 degrees off the barrel's axis.

  That depth is inside the 13 to 16.5 mm window for full credit, and the demo scores 1.0. An earlier run of the same demo also measured 14.2 mm. The README, its demo description and the demo's code comment now give this measured depth.
- **Real-world check of the depth:** the scoring now says how a real trial reads the depth. After the crimp, a caliper measures the bare gap between the insulation's end and the barrel's mouth. A 16 mm strip sits in a 16 mm barrel, so a gap of at most 3 mm means at least 13 mm is in. A good crimp still has to hold a steady 450 N pull.
- The scene, the scorer, the demo's waypoints and the instruction are unchanged, so the posted demo (1.0) and Dream-1 (0.0) videos still show the current version.
