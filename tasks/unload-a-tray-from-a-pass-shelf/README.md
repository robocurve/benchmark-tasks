# Unload a tray of soup from a serving pass and serve the bowl onto a placemat

A bowl of soup stands on a tray in a two-level serving pass, under the upper shelf, where it cannot be reached. The
job: unload the tray without spilling the soup, leaving the tray lying flat on the table and the bowl standing on a
blue placemat. The instruction states only this goal and its constraint. One way, as in the scripted demo: both arms
take the tray by its side rims, slide it out, lower it flat to the table, and then one arm serves the bowl onto the
placemat. The task is one task file,
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

## The soup, and scoring the spill

- **A real trial** uses 150 ml of room-temperature water tinted orange with food colouring (soup without the heat),
  filling the bowl (inside 9 cm across, 5 cm deep, 318 ml to the brim) about 2.4 cm deep, 2.6 cm below its rim; it
  reaches the rim if the bowl tilts about 30 degrees. Weigh the bowl with its soup on a kitchen scale (1 g resolution)
  just before the trial. After it, lift the bowl straight up from where it ended, wipe its outside and base dry and
  weigh it again: the loss in grams is the millilitres spilled. 1.0 allows at most 10 g lost; 0.5 and 0.25 at most
  30 g. A bowl ever tipped 45 degrees or more (judged from the video), or ending tilted 15 degrees or more, scores 0.
- **In simulation** the soup is 15 orange beads 8 mm across, each standing for 10 ml (one fifteenth of the 150 ml).
  A bead whose centre has left the bowl's inside counts as 10 ml spilled, so 1.0 allows 1 bead out and 0.5 and 0.25
  allow 3. The beads weigh 22.5 g; the soup's other 127.5 g is added to the bowl's body, so the bowl with its soup
  weighs 0.40 kg, as in a real trial. The beads roll and spill but do not slosh: real water is harder to keep in.
- **The demo's carry.** The simulated fingers hold a pinched handle against little torque, so in the scripted demo the
  bowl, carried by its handle, sags about 25 degrees about the grip on the way to the placemat (counter-rolling the
  wrist did not level it, and the sag was the same with and without the soup's mass). The real fill of 150 ml reaches
  the rim only at about 30 degrees of tilt, more than that sag, though sloshing in a real carry adds to it.
- **Serving with a small spill.** The 0.5 tier needs the tray lying flat on the table and at most 30 ml (3 beads)
  spilled, whether or not the bowl has been served: a bowl on the placemat with 20 ml (2 beads) spilled scores 0.5, the
  same as one left on the tray.

## Why two hands

The tray is 36 cm wide and the bowl stands in its middle. Held by one side rim, the loaded tray hangs from a single
pinch with its centre of mass 18 cm away, and the free side drops as soon as it leaves the shelf. One-hand check (the
right arm doing its part of the demo, the left never moving): 0. The right drew the tray out by its right rim; the left
side fell as it came off the shelf, the bowl slid across the tray and tipped 45 degrees, and the tipped-bowl rule
scored the trial 0. The instruction does not name the hands; both are needed for the physical reason above.

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
  --instruction "Unload the tray from the pass without spilling the soup, leaving the tray lying flat on the table and the bowl of soup standing on the blue placemat." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/tray-dream
```

The scripted demo, which reaches full score: both arms come straight down on the tray's side rims, lift it 1 cm off
the shelf, draw it straight back until it is clear of the shelf, lower it to the table and let go; the right arm then
takes the bowl by its handle and sets it on the placemat:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=unload_a_tray_from_a_pass_shelf.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Unload the tray from the pass without spilling the soup, leaving the tray lying flat on the table and the bowl of soup standing on the blue placemat." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/tray-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/tray-dream/*.json; do inspect-robots video "$log"; done
```

Set `TRAY_DEBUG=1` to print what the score is built from (beads in the bowl; where the tray and the bowl are) once a
second.

## Runtime per trial

On one RTX 5090: about 7 minutes for a Dream-1 trial of 1,200 steps (428 s on the clock, with Dream answering in 0.60 s on average), and about 2 minutes for the scripted demo (526 steps, 26.3 s of simulated time, 119 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed and the scripted demo is deterministic. With a nonzero
`object_jitter`, the trial's seed sets how far the free objects (the tray, the bowl and the beads) shift.

## Changes in this revision

This is a revision of TK-3E7BYW. Its review said:

> Flags: Scorer awards 0.5 when the bowl is on the placemat with two or three beads out; written scoring requires the bowl not to be on the placemat.
>
> Construct validity (not met): The real-world setup specifies only 15 beads at the bottom of a 5 cm deep bowl, so it cannot test the sloshing and spill control required for soup.
>
> What to change: For real-world trials, specify a realistic quantity of water in the bowl and measurable spill thresholds for each score tier; the beads can remain as the documented simulation approximation. Also align the written scoring and code for a bowl on the placemat with two or three beads out.

What changed in response:

- **Bowl on the placemat with 2-3 beads out:** the code was right and the text is now fixed to match. 0.5 needs the tray flat on the table and at most 3 beads (30 ml) spilled, whether or not the bowl has been served, so serving with a small spill never scores below not serving. The scoring text, task docstring and README now say exactly this.
- **Real soup:** a real trial uses 150 ml of room-temperature water, tinted orange. It fills the 9 cm-wide, 5 cm-deep bowl (318 ml to the brim) about 2.4 cm deep and reaches the rim at about 30 degrees of tilt.
- **Spill thresholds:** the spill is measured by weighing the bowl on a kitchen scale (1 g) before the trial and after it (outside wiped dry). 1 g lost is 1 ml spilled. 1.0 allows at most 10 g lost; 0.5 and 0.25 allow at most 30 g. A bowl ever tipped 45 degrees, or ending tilted 15 degrees or more, scores 0.
- **Beads in simulation:** the beads remain the documented simulation stand-in. Each of the 15 beads stands for 10 ml (1 bead out allowed for 1.0, 3 for 0.5 and 0.25). The soup's remaining 127.5 g is now added to the simulated bowl, so the bowl with its soup weighs 0.40 kg, as in a real trial. The README discloses that the simulated pinch on the handle lets the bowl sag about 25 degrees in the demo, below the 30 degrees at which 150 ml would reach the rim.
- **Instruction:** "With both hands on the tray's side rims" is gone. It now reads "Unload the tray from the pass without spilling the soup, leaving the tray lying flat on the table and the bowl of soup standing on the blue placemat." The title is method-free too. A new "Why two hands" section gives the physical reason, backed by a one-hand check that scores 0.
