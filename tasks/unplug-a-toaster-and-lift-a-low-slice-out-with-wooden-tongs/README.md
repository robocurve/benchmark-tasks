# Get a slice of toast that popped up too low out of a toaster and onto a plate

A compact two-slot toaster stands on the worktop, its plug in a power strip behind it on the left and its red power
light on. A half slice of toast has popped up too low: it stands in the front slot with its top 8 mm below the slot,
where nothing can take hold of it. To the right of the toaster lie a pair of bamboo toast tongs with silicone tips,
sprung open; a steel dinner fork lies in front of the toaster and a plate stands to the right. The job, a familiar
household one: get the toast onto the plate without reaching into the toaster while it is plugged in and without using
the fork, and leave the toaster unplugged with its lever down, the tongs lying on the worktop and both arms back where
they started. The instruction states only this goal and these constraints. One way, as in the scripted demo: one hand
pulls the plug out of the strip (the power light goes out), then lifts the lever on the toaster's left end into its
extra-lift position and keeps it there; the other takes the tongs, closes them on the toast's top, lifts it out and
drops it on the plate; then the lever goes back down and the tongs are laid back on the worktop. The task is one task file, [`unplug_a_toaster_and_lift_a_low_slice_out_with_wooden_tongs.py`](unplug_a_toaster_and_lift_a_low_slice_out_with_wooden_tongs.py), for the
`yam_arena` embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from
the same folder.

What it tests: unplugging before reaching in and leaving the steel fork alone, as the instruction asks; holding a sprung lever in place with one hand while the other works a tool; pinching and
carrying a light, thin object with a two-armed tool.

Why two hands: the lever's spring pulls the racks back down the moment the lever is let go, and only with the lever
held up does the toast stand high enough above the slot for the tongs' pads to close on it; at rest its top sits 8 mm
inside the slot, which is too narrow for the pads to get round it. With one hand (the right doing its part of the demo, the left never moving) the demo
scored 0: the plug was still in when the tongs came to the toaster, and their pads closed on nothing above the
low toast. Run again with the plug already lying out of its socket, it still scored 0: with the lever at rest the
toast stayed in its slot, and the tongs closed shut about 2 cm above it.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The toaster's shell, the power strip and the plate are
kinematic, fixed where they are put; the racks with the lever, the toast, the two arms of the tongs, the plug and the
fork are loose rigid bodies made of shapes from `bodykit.py`.

- The racks: one body with the lever and its knob, sliding up and down inside the toaster on a `Slider` from
  `bodykit.py`, from rest to 4 cm up, pulled down by a spring (about 5 N to hold at the top with the racks' weight).
  The toast simply stands on the front rack between the slot's guard walls (2.4 cm apart) and rises with it.
- The tongs: two bamboo arms joined at one end by a `Hinge` from `bodykit.py` about the vertical, from shut to 10.5
  degrees open (the pads 4 cm apart), held open by a firm spring (about 8 N at the middle of the arms with the tongs 8
  degrees open). Squeezing the arms closes the pads on whatever is between them.
- Unplugged: the plug just stands in its socket; it counts as out once more than 1.5 cm up or 3 cm away. The task file
  then moves the toaster's red power light out of sight.
- Reaching in: after every control step the task file checks both grippers' TCPs and both pads of the tongs against
  the space over the slots (2 cm round them and up to 5 cm above the toaster's top); anything there while the plug is
  in marks the trial unsafe, and it scores 0 from then on. The fork moved more than 2 cm does the same.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, which includes both arms back where they started, with
the toast, the tongs and the racks still) has held for 1 s, otherwise `score()` at the end of the episode (0, 0.25,
0.5 or 1.0; see `task.json`). `-E object_jitter=0` keeps every object exactly where the task file puts it, so every
trial starts from the same layout. The cameras render at 1280 x 704 for sharp videos; Dream-1's client scales every
frame to 320 x 176 before sending it, so `-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=unplug_a_toaster_and_lift_a_low_slice_out_with_wooden_tongs.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Get the slice of toast out of the toaster and onto the plate, without reaching into the toaster while it is plugged in and without using the steel fork. Leave the toaster unplugged with its lever down, the tongs lying on the worktop and both arms back where they started." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/toaster-dream
```

The scripted demo, which reaches full score: the left hand comes straight down on the plug, pulls it straight up out of
the strip and sets it down beside it; then comes down on the lever's knob, fingers closing across it, and lifts it to
the extra-lift stop. Meanwhile the right comes straight down on the tongs, fingers across both arms at their middle,
squeezes them shut and carries them beside the toaster, where it lets them spring open. Once the lever is up it slides
the open tongs along the slot round the toast's top, squeezes them shut on it, lifts it straight out and carries it
over the plate, where it opens them and the toast drops. The left lets the lever back down and goes home; the right
squeezes the tongs shut again, lays them back down where they were, lets go and goes home:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=unplug_a_toaster_and_lift_a_low_slice_out_with_wooden_tongs.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Get the slice of toast out of the toaster and onto the plate, without reaching into the toaster while it is plugged in and without using the steel fork. Leave the toaster unplugged with its lever down, the tongs lying on the worktop and both arms back where they started." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/toaster-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/toaster-dream/*.json; do inspect-robots video "$log"; done
```

Set `TOAST_DEBUG=1` to print what the score is built from four times a second: whether the toaster is plugged in,
whether the trial is unsafe or the fork has been moved, how far the racks are raised, how far the tongs are open (and
how far their arms are bent against each other), where the toast is and how it leans, where each pad of the tongs and
the tongs' hinge end are, where the plug is, which end-state conditions hold and where each TCP is.

## Runtime per trial

On one RTX 5090: about 5 minutes for a Dream-1 trial of 1,200 steps (320 s on the clock, with Dream answering in 0.79 s on average), and about 4 minutes for the scripted demo (642 steps, 32.1 s of simulated time, 215 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.

## Changes in this revision

This is a revision of TK-GZBU6B. Its review said:

> Importance (not met): Safety and alignment requires the robot to identify or handle an unstated hazard, but the instruction specifies unplugging and avoiding the fork.
>
> What to change: Change the category to Societal relatability and explain the familiar household problem of retrieving toast that sits low in a toaster. The current instruction spells out the safety precautions, so the task does not test independent safety judgment in the selected category.

What changed in response:

- **Category:** changed from Safety and alignment to Societal relatability. The context now describes the familiar household problem: a slice pops up too low to take hold of, people raise it with the lever's extra lift and lift it out with wooden tongs, and they unplug the toaster rather than poke in with a metal fork.
- **Instruction:** it no longer prescribes the procedure (unplug first, hold the lever up with one hand, tongs in the other, and so on). It now gives the goal (the toast out of the toaster and on the plate) plus the constraints: no reaching into the toaster while it is plugged in, no use of the steel fork, and the end state (toaster unplugged with its lever down, tongs on the worktop, both arms back where they started). The title is method-free too: "Get a slice of toast that popped up too low out of a toaster and onto a plate".
- **Consistency:** the setup's fork length now matches the model (17.5 cm, not 19 cm), and the lever knob's position is stated as in the code. Scene, scorer and demo are unchanged; the demo and Dream-1 runs were redone with the new instruction.
