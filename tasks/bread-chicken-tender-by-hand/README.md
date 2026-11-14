# Bread a chicken tender by hand: press the breading onto every face, then grip it as little as possible

Files for the `yam_arena` embodiment (Isaac Lab-Arena, bimanual YAM, `station` scene). Three eighth-size sheet pans
stand side by side: a raw chicken tender, wet from its marinade, in the left one; seasoned flour 1 cm deep in the
middle one; a wire rack in the right one. The robot breads the tender (lays it in the flour and presses it, turns it
over and presses the other face) and leaves it on the rack, gripping it as little as it can once it is coated. The
setup, scoring and fidelity notes are in `task.json`.

## Run a trial

Scripted demo (reaches full score):

    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=tasks/bread-chicken-tender-by-hand/task.py \
      -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "Bread the chicken tender and put it on the rack for the fryer." \
      --scorer task_score --max-steps 1600 --store-frames

Model run (Dream-1): the same command with `--policy dream --max-steps 2400`.

Render the videos of a run with `inspect-robots video <log.json>`. `COAT_DEBUG=1` prints the pan's force, the presses,
the grips and the coverage twice a second. Offline checks (no simulator): `python3 test_task.py` and
`python3 test_coatkit.py`.

The demo: the left hand takes the tender out of the marinade across its width, lays it flat in the flour and presses three spots with its closed fingertips; while the left hand waits parked, the right hand takes it near one end, lifts it 15 cm and rolls it 90 degrees onto its edge; the left hand, rolled the other way, takes its far half and the right lets go; the left rolls level, which turns the tender over, lays it back in the flour and presses three spots on the other face; the right hand takes it at the same near end as before (so its grip wipes the spots its first grip already wiped), lays it on the rack and lets go. Held by one end, the tender droops a little at its hinges; rolled onto its edge, the hinges stand upright and it straightens. Offline, this sequence of presses and grips leaves 8.2 cm^2 bare; in the simulator it left about 8.8 cm^2 (92.8 % coated).

## Scoring

`--scorer task_score` gives 1.0 when the task succeeds (the full condition held for 1 s) and otherwise `score()` at
the end of the trial. The tender's surface is 122.5 cm^2, counted on its 11.5 x 3.5 x 1.4 cm body (two 11.5 x 3.5 cm faces, two 11.5 x 1.4 cm sides, two
3.5 x 1.4 cm ends). The score is set by how much of it is bare, read once the tender rests on the rack's grid (its
middle over the grid, the lowest corner of any of its three parts from 3 mm below to 1 cm above the grid's top) and has been still (no part moving over 1 cm/s or turning over 10 degrees/s) for 1 s:

| Bare area | Score | Why |
|---|---|---|
| at most 10 cm^2, and no hand on it | 1.0 | Turning the tender over and moving it to the rack take grips, and each grip wipes the coat under the fingertips. Gripped at one spot each time, they leave about 8 cm^2 of fingertip marks; 10 cm^2 allows that and nothing more. |
| at most 25 cm^2 | 0.75 | Each extra grip at a new spot after breading strips about 4 to 5 cm^2: one extra grip costs the top tier, and 25 cm^2 is about three or four extra grips. |
| at most 50 cm^2 | 0.5 | Both faces pressed, but handled all over or pressed only in places. |
| at most 85 cm^2 | 0.25 | One face pressed and the other left bare leaves about 58 cm^2 bare. Pressing each face at one spot only leaves about 65 cm^2. |
| more than 85 cm^2 | 0 | Barely breaded: dropped into the flour and never pressed leaves it all bare. |

- **Why bare area matters.** Pressed breading stays on wet meat; breading only dusted on falls off. A bare patch fries dark and
  greasy, and the coat around it flakes off in the oil. The fingertip marks are the reason cooks handle a breaded
  piece as little as possible.
- **A hand still on the tender** at the end caps the score at 0.5 (`touching()` in `guardkit.py`: the gripper's own shape
  within 2 mm of the tender, a contact check, not a distance zone). Until it is let go the cook is still handling it,
  and any grip can still wipe the coat. The arm's pose is not scored.
- **Latched 0: dropped.** The tender's lowest point within 3 mm of the table top while it is over none of the three
  pans, for two control steps in a row (one step can be a physics pop), or the tender below the table. Raw chicken
  that has touched the bench or the floor is thrown away.
- **Off the rack at the end:** 0 (left in the breading pan, back in the marinade, held over the rack). The job is a
  tender waiting for the fryer: a coated tender left in the breading is not ready to fry and gets turned in the flour
  again by the next cook, so the coat alone earns nothing.
- Doing nothing scores 0 (the tender stays in the marinade). Possible scores: 0, 0.25, 0.5, 0.75, 1.0. Each threshold
  is defined once in `task.py` (`TIERS`, `HELD_CAP`, `ON_RACK_GAP`, `TABLE_TOUCH`, `STILL_SPEED`, `STILL_SPIN`,
  `HOLD_STEPS`, and the coat law `PRESS_N`, `PRESS_STEPS`, `PRESS_HALF`, `WIPE_N`, `FLOUR_DEPTH`).

Scoring a real trial:

1. Shed loose breading: lift the rack 2 cm by one end and let it drop back once. Unpressed breading falls off, as it
   would in the fryer; pressed breading stays.
2. Lay a clear acetate sheet ruled in 5 mm squares on the tender and photograph its top, then both long sides. Slide a
   thin spatula under it, turn it onto a sheet of white paper and photograph the other face and both ends.
3. A square is bare when more than half of it shows pink meat (the paprika tints the breading pale tan, so bare meat
   stands out). Squares cut by an edge count by the part on that face. The bare area is the number of bare squares
   times 0.25 cm^2. 5 mm squares, not 1 cm: a fingertip mark on the 1.4 cm side straddles squares, and 1 cm rounding
   (up to half a square per cut edge) would be about the size of the demo's margin under the 10 cm^2 line.
4. Check on the video that the tender never touched the table and that no hand was on it at the end.

## The breading model

The simulator has no flour, so the coat is a scored model on the tender's surface (`coatkit.py`), driven by PhysX
contact forces read before every physics step (240 Hz). A rigid contact view on the tender's three parts, filtered to the breading pan and to each robot's palm and finger links,
gives the force of each on the whole tender.

- **Cells.** The tender's six faces are cut into 2.5 mm cells (2,008 in all), bare at the start.
- **In the flour.** The flour has no collider: the tender lies on the breading pan's floor with its lower 1 cm in the
  flour, as a tender pressed into a real pan of breading does. A cell is in the flour when it is over the pan's inside,
  3 mm in from its walls, and no more than 1 cm above its floor. Lying flat, that is the bottom face and the lower 1 cm of the sides and ends;
  turned over, the other face and the upper part of the sides. So both faces have to be pressed to coat it all.
- **Pressed, not dropped.** A press is the pan pushing up on the tender with at least 2.0 N (four times the 0.49 N
  weight of a 50 g tender) while a gripper pushes it down with at least 0.3 N, for 6 physics steps (25 ms) in a row. A tender lying in
  the flour under its own weight, or dropped in (an impact lasting a step or two), or pushed against the pan by
  nothing but the pan, sets nothing. 2.0 N is a firm fingertip press; the demo presses by commanding its closed
  fingertips 8 mm below the tender's top.
- **Where it is pressed.** A press sets the in-flour cells along a 6 cm stretch of the tender's length, 3 cm each way
  from the spot the gripper pushes on (its tool point projected onto the tender's long axis): pushing one spot of a
  1.4 cm thick tender into a soft flour bed beds about 6 cm of it.
  The 11.5 cm tender needs pressing at two or three spots per face.
- **Wiped where gripped.** A gripper grips the tender when both of its fingers push on it with at least 0.2 N each,
  from opposite sides. While it grips, every coated cell within 3 mm of its finger pads' inner faces goes bare: a pinch that
  carries the tender's weight by friction shears the coat off under the pads. The pads' shape is the fit in `guardkit.py` of
  the YAM's finger collision meshes (the 4 cm pad and the 2.6 cm tip). A fist pushing down (both fingers pushing the
  same way) is a press, not a grip. A gripped cell can't be coated in the same step.
  0.2 N is below what any carrying pinch needs: two fingers hold the tender's 0.49 N weight by friction with about
  0.25 N each at a friction coefficient of 1.0 (more on a slippery coat), so a pinch light enough to escape the wipe
  can't lift it. Not modeled: the tender clamped end to end between two closed fists (each hand's fingers push the
  same way, so neither is a grip); a real two-fist carry would mark the ends (4.9 cm^2 each).
- **Patching.** A wiped spot pressed into the flour again is coated again, as a cook patches a bare spot.

What it does not model: the breading as particles (no flow, no heaping over the top, no running out); the tender's
softness beyond two hinges (it bends through its thickness at two places, but does not stretch, twist or squash); a coat that is only partly pressed (unpressed breading counts as
bare, which the real trial's tap test matches); pressure varying across the tender (one force, placed by where the
hand pushes). The score checks what a real trial checks: how much of the surface is bare, where the tender ends up and
whether it touched the bench.

How this differs from symbolic "covered" goals in household task suites (a goal that some breading particles touch
the chicken): here the coat has to be pressed on face by face with a measured force, it comes off where the fingers
hold it, and the score is the bare area a photo shows.

## Runtime and seeds

- Demo: a 73.7 s plan (1,474 control steps), run with `--max-steps 1600`. In the simulator it scored 1.0, succeeding
  (the 1 s hold on the rack complete) at step 1,413.
- Model run limit: 2,400 steps (2 minutes at 20 Hz).
- Seeds: none. The layout is fixed (`object_jitter=0`).

## Real-world build

- Pans: three aluminum eighth-size sheet pans, 9.6 x 6.6 x 1 in. (24.4 x 16.8 x 2.5 cm), each on a 20 x 25 cm piece
  of non-slip shelf liner so a press can't slide it, long sides away from the robot, centered at x 28.0 and y 19.5,
  0 and -19.5 cm (2.7 cm apart).
- Breading: all-purpose flour with 2 % sweet paprika by weight (about 210 g of flour and 4 g of paprika), sifted
  into the middle pan and leveled with a straightedge at 1 cm deep (about 400 mL).
- Rack: an eighth-size stainless wire cooling rack, about 22.0 x 14.5 cm, wires 3 mm thick and 1.2 cm apart, its grid
  top 1.6 cm above the table, set in the right pan.
- Tender: one raw chicken breast tenderloin about 13.5 cm long, 3.5 to 3.7 cm wide and 1.4 to 1.8 cm thick (trim it
  with a knife if needed), weighed (50 g modeled;
  tenderloins run about 40 to 60 g), marinated 10 minutes in buttermilk and drained on a rack for 30 s, then laid
  flat in the left pan, long side along x, centered at (28.0, 19.5).
- Safety: raw poultry. Wear food-safe gloves to set up, wash the gripper pads with soap after each session, and
  discard the tenders. Nothing hot, sharp or heavy.

## How the file works

- **The pans and the rack** are fixed yamkit `Prop`s with box colliders (floor and walls); the rack's grid collides
  as one flat plate. The flour is a visible slab with no collider.
- **The tender** is a loose yamkit `Mechanism`, 50 g: three box colliders (3.8 cm parts of an 11.5 x 3.5 x 1.4 cm
  body), the middle one the base and the end ones on revolute hinges across its width at 1.9 cm either side of the
  middle. Each hinge is driven toward straight by PhysX's own joint drive (0.05 N m/rad, damping 0.003 N m s/rad, at
  most 60 degrees either way): held level by one end part, the rest droops about 14 degrees at the first hinge and
  18 at the far part. It is drawn as one tenderloin, 13.5 cm tip to tip, 3.7 cm wide and 1.8 cm thick, rounded and
  tapered, split at the hinges with a 3 mm overlap so a bent hinge shows no gap. The scorer judges stillness from how
  far each part moved since the last control step.
- **The coat.** `coatkit.py` hooks the scene's `write_data_to_sim` and, before every physics step, reads the poses of the
  tender's three parts, the contact forces on them (pan, and each robot's `gripper`, `tip_left` and `tip_right` links), and the grippers'
  poses and openings, and steps the coat. The scorer reads the coat after every control step. A new trial (counted by
  `yamkit.py`) starts a bare tender.
- **Visible coat.** Each coated cell is drawn as a thin crumb flake (three shades: flour, toasted crumb, paprika; varied
  in size and turn) laid on the drawn tenderloin's surface, with Isaac Lab's `VisualizationMarkers`, so camera-based
  policies see the coat grow face by face. The flakes are only a picture: if drawing fails, they turn off with one
  warning and scoring carries on. The flour bed is drawn with fine crumbs and paprika flecks; it has no collider.

## Files

- `task.py`: the scene, `score()`, `success()` and the scripted `demo()`.
- `coatkit.py`: the task's helper module for the breading model (cells coated by a press, wiped by a grip).
- `yamkit.py`: the task's helper module that spawns the modeled props.
- `guardkit.py`: the task's helper module for the grippers' own shapes (contact checks and the finger pads).
- `test_task.py`, `test_coatkit.py`: offline checks of the scorer, the breading model and the demo plan.
