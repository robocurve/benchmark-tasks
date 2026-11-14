# Saw a 45-degree mitre through a picture-frame moulding in a mitre box

On the table stands a beech mitre box on a long base board: a channel between a front wall and a back fence, with
saw slots cut through both walls at 90 degrees and at 45 degrees either way. In front of it lies a length of
black-lacquered pine picture-frame moulding with a gold lip, a white pencil line across it at 45 degrees near its right
end. To the right a backsaw (a steel blade with a brass back and a turned rosewood handle) stands in its wooden rest.
The job: saw the moulding right through at its pencil line in the 45-degree slot, the moulding against the back fence,
then leave the saw back in its rest, the cut moulding on the table in front of the box and both arms back where they
started. The instruction states only this goal and these constraints. One way, as in the scripted demo: one hand lays
the moulding in the box at its mark against the fence and holds it there, while the other saws back and forth in the
slot until the offcut comes free and stands the saw back in its rest; the first hand then lifts the cut moulding, its
fresh 45-degree end showing bare wood, out onto the table. The task is one task file,
[`saw_a_mitre_in_a_mitre_box_holding_the_moulding_to_the_fence.py`](saw_a_mitre_in_a_mitre_box_holding_the_moulding_to_the_fence.py),
for the `yam_arena` embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it
imports from the same folder.

What it tests: placing a long, light workpiece precisely in a fixture (against a fence, a mark on a slot), holding it
there with one hand for a long time while the other hand works a tool through a guide, a repeated stroke that only
counts when the workpiece stays put, and putting the tool and the work back where they belong.

Why two hands: the moulding weighs 94 g and the box's floor and fence are smooth, while the saw's teeth grip the
bottom of the cut: every stroke drags a moulding nobody holds along the box with the saw, off its mark, and a stroke
only counts while the moulding stays put. One-hand check (the moulding already laid at the mark, the right hand doing
its part of the demo, the left never moving): 0.25, only for the moulding having been at the mark. The first strokes
dragged it 3.1 cm along the box, off the mark, and no stroke counted. The left hand alone (laying and holding the
moulding) saws nothing.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The mitre box and the saw rest are kinematic, fixed
where they are put; the moulding (two pieces, see below), the saw and the plate at the bottom of the cut are loose
rigid bodies made of shapes from `bodykit.py`.

- The box: the walls' collision is built from thin slices with openings where the three slots cross them (1.4 cm
  wide, so a saw held a few millimetres off still goes in); what is drawn is a solid wall with a thin dark kerf at
  each slot. The box's floor and fence are smooth (their contacts take the lower of the two frictions, 0.18): a
  moulding nobody holds slides in the box.
- The cut: the moulding is two bodies joined by a fixed joint (a `Glue` from `bodykit.py`), the length left of the
  pencil line and the 5 cm offcut right of it, each built of slices across its width so that both ends at the line
  run at 45 degrees. The saw passes through the moulding (they do not collide); its teeth ride instead on a plate
  that is not drawn, held inside the moulding at the pencil line by a sprung slide (a `Slider` from `bodykit.py`).
  Its friction with the saw's teeth (1.2) drags the moulding along with every stroke.
- A stroke: after every control step the task file finds where the saw lies against the 45-degree slot (along it,
  across it, how it is turned and leaning) and how high its teeth are over the plate. The saw is in the slot when it
  is turned within 8 degrees of the slot, leans less than 12 degrees, lies within 9 mm of the slot's line, and its
  teeth over the plate are within 7 mm above and 8 mm below the plate's top. A stroke counts when the saw, in the slot,
  is pushed 3 cm along it while the moulding lies at the mark and moves no more than 4 mm. Each stroke lowers the
  plate 2.875 mm (a deeper cut); after the eighth the simulation counts the moulding as cut through, and the joint
  between the two pieces switches off. This stroke count stands in for sawing, which the simulation does not model; a
  real backsaw's depth per stroke varies with the wood, the teeth and the pressure, so real trials measure the kerf
  instead (see below).
  Sawdust is drawn on the box's floor after two and six strokes, and the saw cut across the moulding's top while
  sawing.
- At the mark: the moulding's pencil line within 1 cm of the 45-degree slot along the box, its back face within 6 mm
  of the fence, turned less than 4 degrees and lying on the box's floor.

## Scoring a real trial

The tiers are the same; the cut is measured on the wood instead of counted in strokes:
- **Where the cut is.** After the trial the cut counts only where it crosses the moulding within 1 cm of the pencil
  line, at 45 degrees (plus or minus 4) to its edge: the cut that the 45-degree slot gives with the moulding at its
  mark. Check it with a combination square or a protractor.
- **0.5: kerf depth.** A kerf at least 11 mm deep, half the moulding's 22 mm height, measured with a caliper's depth
  rod from the moulding's top face to the deepest point of the kerf. In simulation, four counted strokes (11.5 mm).
- **0.75: cut through.** The offcut has come free along the kerf, sawn over the moulding's full height and width. An
  offcut snapped or torn off with uncut fibres is not a through-cut; the kerf's sawn depth scores instead. In
  simulation, the eighth counted stroke (23 mm).
- **1.0** adds the end state, judged as in simulation: the saw standing back in its rest, the cut moulding on the table
  in front of the box and both arms back where they started.

The real rig: a beech (or hardwood) mitre box with the slots as specified, screwed or clamped down; a 34 cm length of
26 x 22 mm pine picture-frame moulding (about 94 g), black-lacquered, marked with a 45-degree pencil line 5 cm from its
right end; and a 24 cm backsaw with a brass back and fine teeth (about 15 per inch), standing in a wooden rest. Use a
fresh moulding for each trial.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, which includes both arms back where they started, with
the saw and both pieces of the moulding still) has held for 1 s, otherwise `score()` at the end of the episode (0,
0.25, 0.5, 0.75 or 1.0; see `task.json`). `-E object_jitter=0` keeps every object exactly where the task file puts it,
so every trial starts from the same layout. The cameras render at 1280 x 704 for sharp videos; Dream-1's client scales
every frame to 320 x 176 before sending it, so `-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=saw_a_mitre_in_a_mitre_box_holding_the_moulding_to_the_fence.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Saw the moulding right through at its pencil line in the mitre box's 45-degree slot, with the moulding against the box's back fence. Leave the saw standing back in its rest, the cut moulding lying on the table in front of the box and both arms back where they started." \
  --scorer task_score --max-steps 1400 --store-frames --log-dir ~/yam-eval/mitre-dream
```

The scripted demo, which reaches full score: the left hand comes straight down on the moulding 12 cm from its left end, fingers closing across it, lifts it over the box's front wall and lays it in the channel 7 mm clear of the fence with the pencil line at the 45-degree slot, then slides it back against the fence and holds it there, pressing it down. The right hand comes down on the saw's handle, fingers closing across it, lifts the saw out of its rest, turns it 45 degrees over the slot and sets it in, teeth on the pencil line. It saws twelve strokes, 6.5 cm forward and back along the slot, each aimed a little deeper (the first ones go while the saw settles onto the line); the offcut comes free after eight that count. It lifts the saw out, turns it back and stands it in its rest, and lets go. The left hand lifts the cut moulding out of the box over the front wall and lays it on the table in front of the box, its fresh 45-degree end showing bare wood, and lets go; both arms go back to where they started:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=saw_a_mitre_in_a_mitre_box_holding_the_moulding_to_the_fence.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Saw the moulding right through at its pencil line in the mitre box's 45-degree slot, with the moulding against the box's back fence. Leave the saw standing back in its rest, the cut moulding lying on the table in front of the box and both arms back where they started." \
  --scorer task_score --max-steps 1400 --store-frames --log-dir ~/yam-eval/mitre-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/mitre-dream/*.json; do inspect-robots video "$log"; done
```

Set `MITRE_DEBUG=1` to print what the score is built from four times a second: where the moulding is against its
mark and how it is turned, whether it is at the mark, where the saw lies against the slot (along, across, how high
its teeth are over the bottom of the cut) and how it is turned, leaning and tipped, whether it is in the slot, how far
the moulding has moved during the stroke, how many strokes have counted, whether the cut is through, how deep the
cut is, where the saw is against its rest, each hand's finger opening and where each TCP is.

## Runtime per trial

On one RTX 5090: about 7 minutes for a Dream-1 trial of 1,400 steps (406 s on the clock, with Dream answering in 0.72 s on average), and about 4 minutes for the scripted demo (771 steps, 38.5 s of simulated time, 236 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.

## Changes in this revision

This is a revision of TK-X5B2Y5. Its review said:

> Scoring (not met): The rule that eight 3 cm strokes cut through the wood can award cut credit without an actual through-cut in a real trial.
>
> What to change: Keep the simulated stroke-count approximation in the fidelity note, but specify real-world scoring using measured kerf depth for partial credit and an actual through-cut for cut credit. Do not assume that eight 3 cm strokes always sever the moulding.

What changed in response:

- **Real-world scoring:** the tiers now name what is measured on the wood after a real trial.
  - 0.5 needs a kerf at least 11 mm deep (half the moulding's 22 mm height), measured with a caliper's depth rod from the top face to the deepest point of the kerf.
  - 0.75 (cut credit) needs an actual through-cut: the offcut free along the kerf, sawn over the full height and width. An offcut snapped or torn off with uncut fibres does not count; its sawn depth scores instead.
  - A cut counts only within 1 cm of the pencil line, at 45 ± 4 degrees to the moulding's edge.
- **Stroke count:** the text no longer claims that eight 3 cm strokes always sever the moulding. The stroke count is kept as the simulation's stand-in for kerf depth (2.875 mm per counted stroke: four strokes are 11.5 mm, the eighth frees the offcut). The fidelity note says real depth per stroke varies, which is why real trials measure the kerf.
- **Real rig:** a new README section, "Scoring a real trial", specifies it: black-lacquered pine moulding (26 x 22 mm, 34 cm, about 94 g, a fresh one each trial) and a fine-toothed backsaw (about 15 teeth per inch).
- **Instruction:** it no longer assigns hand roles or steps. It now gives the goal (saw the moulding through at its pencil line in the 45-degree slot, against the back fence) plus the end state (saw back in its rest, cut moulding on the table, arms back where they started). The title is method-free too: "Saw a 45-degree mitre through a picture-frame moulding in a mitre box".
- **Unchanged:** the scene, scorer and demo. The demo and Dream-1 runs were redone with the new instruction.
