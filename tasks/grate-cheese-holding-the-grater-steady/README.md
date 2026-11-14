# Grate a block of cheese on a box grater onto a cutting board

A steel box grater stands on a beech-wood cutting board with a block of cheese beside it. One hand holds the grater by
the flat handle reaching left from its top; the other takes the cheese, presses its end on the grating face and rubs it
down across the teeth, stroke after stroke, so that grated cheese falls inside the grater onto the board. The task is
one task file, [`grate_cheese_holding_the_grater_steady.py`](grate_cheese_holding_the_grater_steady.py), for the
`yam_arena` embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from
the same folder.

Why two hands: the grater is light (180 g) and tall, and grating needs the cheese pressed on its face. Unheld, it can't
take that press: on the board's friction it resists at most about 1.4 N sideways (0.5 x its 1.77 N weight, plus the
little extra the cheese's drag down the face adds), well under the 3 N that grating needs, so it slides away or tips
over instead. With the left hand kept clear (it stays where it started), the scripted demo's right-hand
strokes tip the grater over on the first press (55 degrees within a quarter of a second, the contact force
never above 2.1 N), and nothing is grated: 0.00 g, score 0.0. One hand holds the grater while the other grates.

## How the score is measured

The primary measure is the mass of grated cheese, in grams.

**In a real trial** the grated cheese is weighed: after the trial, lift the grater, brush every shred from the board and
from inside the grater into a tared bowl and weigh it on a kitchen scale reading to 0.1 g.

**In the simulation**, which can't cut cheese, grating is credited only for real pressed strokes, from PhysX contact
data, never from distance alone:

- After every control step the task file reads the contact force between the cheese and the grater from a PhysX
  rigid-contact view (the cheese's contacts filtered to the grater) and takes its component along the face's outward
  normal: the force pressing the cheese on the face.
- A step counts as grating when, at both its start and its end, that force is at least **3 N**, the contact is the
  cheese's end on the perforated area (the middle of its end over the cutting holes: across the face within 2.8 cm of
  its middle, 2 to 11.5 cm up, and at most 5 mm off the face's surface so that it is the end, not a side, that touches),
  and the grater stands (within 20 degrees of upright, its foot on the board inside its edges). The cheese's
  travel down the face during such steps is added up; travel up the face counts nothing, as a box grater cuts on the
  downstroke.
- That pressed downstroke length converts to grated mass at **0.25 g per cm**. The rate comes from the tooth geometry:
  the coarse face has 10 staggered rows of 8 x 4 mm cutting holes, 9.5 mm apart, about half the width open in each
  row, so every two rows (1.9 cm of stroke) cut across the whole of the cheese's 2.4 x 3.5 cm (8.4 cm2) end once.
  Taking a layer about 0.57 mm thick per pass (about half the 1.1 mm rise of a coarse grater's tooth lip) and the
  cheese's density of 0.99 g/cm3 (50 g in 50.4 cm3): 0.99 x 8.4 x 0.057 / 1.9 = 0.25 g per cm. It is an
  approximation, not a measured cut rate, and the fidelity note says so.
- Score tiers in grams, with their stroke equivalents in the simulation:

  | Score | Real trial (kitchen scale) | Simulation (pressed downstroke at >= 3 N) |
  |---|---|---|
  | 1.0 | at least 5.0 g grated, plus the end state | at least 20 cm (5.0 g), plus the end state |
  | 0.5 | at least 2.5 g (or 5.0 g without the end state) | at least 10 cm (2.5 g) (or 20 cm without it) |
  | 0.25 | at least 0.5 g | at least 2 cm (0.5 g) |
  | 0.0 | less | less |

  The end state, in both: the grater standing on the board (within 20 degrees of upright, its foot on the board inside
  its edges), the cheese put down (lying on the board or the table, its middle at most 3 cm above the board) and both
  arms back where they started (each fingertip centre within 5 cm of its start).
- As a visual cue, each 0.5 g grated brings in one shred: fourteen 3 x 6 x 28 mm sticks of cheese (0.5 g each) wait in
  a closed box high above the scene, out of the cameras' view, and the task file moves the next one inside the grater,
  just behind the face at the cheese's height, from where it falls onto the board. The score uses the grated mass, not
  the shreds.

**The loophole of the previous version is closed.** The previous score counted the cheese's travel while its end was
within 4 mm of the face, so moving it up and down near the face without touching earned full credit. Run as an
adversarial test (the scripted demo with the cheese's end aimed 2.5 mm off the face on every stroke, never pressed,
and the left hand holding the grater where it stands), the revised scorer gives **0.0**: over five strokes the
cheese's end passed up and down the face 0.5 to 6 mm off it (the previous rule counted 37.8 cm of rubbing from this,
more than the 18 cm it needed for full credit), the contact force never exceeded 0.5 N (light brushing on the way up),
and 0.00 g was credited.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The grater, the cheese, the board and the shreds are
rigid bodies made of shapes from `bodykit.py`; the board is kinematic, fixed where it is put. The cutting holes are
drawn on the flat face (they don't collide). Nothing pushes or pulls on the grater except the contacts themselves:
pressing on the face pushes it away, and the cheese's friction on the face (0.75) drags it along each stroke.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step. The task file also gives
the cheese and the grater PhysX's contact-report API so that the rigid-contact view can read their contact force.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, which includes at least 5 g grated, the grater standing on
the board, the cheese put down and both arms back where they started, with the fallen shreds, the grater and the cheese
still) has held for 1 s, otherwise `score()` at the end of the episode (0, 0.25, 0.5 or 1.0; see `task.json`).
`-E object_jitter=0` keeps every object exactly where the task file puts it, so every trial starts from the same
layout. The cameras render at 1280 x 704 for sharp videos; Dream-1's client scales every frame to 320 x 176 before
sending it, so `-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=grate_cheese_holding_the_grater_steady.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Grate at least 5 g of cheese from the block on the box grater onto the board. Then put the cheese down, leave the grater standing on the board and take both arms back to where they started." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/grate-dream
```

The scripted demo, which reaches full score: the left hand comes straight down on the grater's handle near its end and
holds it. The right hand comes straight down on the cheese near its right end, lifts it and grates: five times it
presses the cheese's left end on the grating face at the top and rubs it down across the teeth (about 6 cm per stroke,
pressing with about 4 to 9 N), then eases the press and brings it back up lightly. On each downstroke the left hand
braces the grater against the press. Then the right hand puts the cheese back down and goes home. The left then lets
go, takes the handle near its root, lifts the grater 3 cm and stands it on the board 7 cm to the left, showing the
grated cheese, and goes home too:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=grate_cheese_holding_the_grater_steady.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Grate at least 5 g of cheese from the block on the box grater onto the board. Then put the cheese down, leave the grater standing on the board and take both arms back to where they started." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/grate-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/grate-dream/*.json; do inspect-robots video "$log"; done
```

Set `GRATE_DEBUG=1` to print what the score is built from four times a second: where the grater is and how tilted, the
contact force between the cheese and the grater and its part normal to the face, where the cheese's end is on the face,
whether the step counts as pressed, the pressed downstroke length and the grated mass, how many shreds have dropped,
and where each gripper's TCP is.

## A real trial

The same stainless box grater (about 8 x 7 x 13 cm, 180 g, open at the bottom, its coarse face with 10 staggered rows of
8 x 4 mm cutting holes from 2 to 11.5 cm up toward the grating hand, a flat handle on top reaching away from it), a 50 g
block of semi-hard cheese such as cheddar, straight from the fridge (6 x 2.4 x 3.5 cm), and a beech board, laid out as
in the simulation. After the trial, check the end state, then lift the grater, brush all the grated cheese from the
board and from inside the grater into a tared bowl and weigh it on a kitchen scale reading to 0.1 g: 1.0 for at least
5.0 g with the end state, 0.5 for at least 2.5 g (or 5.0 g without the end state), 0.25 for at least 0.5 g, else 0.

## Runtime per trial

On one RTX 5090: about 5 minutes for a Dream-1 trial of 1,200 steps (311 s on the clock, with Dream answering in 0.93 s on average), and about 5 minutes for the scripted demo (791 steps, 39.5 s of simulated time, 301 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.

## Changes in this revision

This is a revision of TK-UV3V7E. Its review said:

> Construct validity (not met): A robot can move the cheese end up and down within 4 mm of the face without touching it and earn full credit without producing grated cheese.
>
> Scoring (not met): The distance-only primary score does not require actual contact or any grated output, allowing a full score for an unsuccessful real-world trial.
>
> What to change: Revise the primary score so rubbing near the grater without actual grating cannot earn credit: require measurable grated output in real trials and contact-based grating in the simulation proxy. Specify partial-credit thresholds, update the fidelity note, and demonstrate the revised score.

The review before that (TK-PGY42N) asked for comparable real and simulated scores, keeping the end-state requirements, and a full-score demo that clearly shows the grating and the final state; that was done in the previous revision and is kept: both scores are now in grams with the same thresholds, the end state is unchanged, and the demo still lifts the grater aside to show the grated cheese.

What changed in response:

- **Construct validity (rubbing near the face earned full credit):** credit now needs real, contact-based grating. A control step counts only if the PhysX contact force between the cheese and the grater, normal to the grating face, is at least 3 N at both the start and the end of the step, the touching part is the cheese's end over the perforated area, and the grater stands on the board. Only the cheese's travel down the face adds to the score (a box grater cuts on the downstroke). Distance alone never counts.
- **The loophole, tested:** we re-ran the scripted demo with the cheese's end moved up and down the face 0.5 to 6 mm off it on all five strokes, never pressed, while the left hand held the grater still. The earlier distance rule counted 37.8 cm from that run, where full credit needed 18 cm. The revised scorer read at most 0.5 N of contact (light brushing), credited 0.00 g and scored 0.0.
- **Scoring (a full score was possible with no grated output):** the primary measure is now grated mass. In a real trial, all the grated cheese on the board and inside the grater is brushed into a tared bowl and weighed on a kitchen scale reading to 0.1 g.
- **Contact-based simulation proxy:** the pressed downstroke length is converted to grams at 0.25 g per cm. The rate is reasoned from the coarse face's tooth geometry: 10 staggered rows of 8 x 4 mm holes, 9.5 mm apart, so every 1.9 cm of stroke cuts a layer about 0.57 mm thick off the cheese's 8.4 cm2 end, at 0.99 g/cm3. Rigid shreds (one per 0.5 g) still drop in, but only as a visual cue. The earlier rule-driven drag force on the grater is removed, so the press and the cheese's friction now act on it physically.
- **Partial-credit thresholds:** 1.0 needs at least 5.0 g grated plus the unchanged end state (grater standing on the board, cheese put down, arms home); 0.5 needs at least 2.5 g, or 5.0 g without the end state; 0.25 needs at least 0.5 g; anything less scores 0. In the simulation these thresholds are 20, 10 and 2 cm of pressed downstroke. The instruction now names the 5 g target, and the instruction, scoring text, code and README all use the same numbers.
- **Fidelity note:** it now says that the cheese is neither cut nor worn down. It says that grated mass is an estimate (0.25 g per cm of downstroke pressed at 3 N or more, measured as PhysX contact force), taken from tooth geometry rather than measured, that a real trial weighs the cheese, and that the shreds are only a cue.
- **Revised score demonstrated:** a new scripted demo scores 1.0. Five downstrokes, pressed at 4 to 9 N, grate 7.3 g (29.1 cm of pressed downstroke) while the left hand braces the grater, which stays within 3.4 degrees of upright. The demo then puts the cheese down, lifts the grater aside to show the 14 shreds and sends both arms home. The demo and Dream-1 videos are new. In the one-hand check, the unheld grater tips over on the first press (contact force at most 2.1 N): 0.00 g, score 0.0.
