# Streak a blood agar plate for single colonies

Streak the plate for single colonies, then discard the loop.

A clinical specimen has already been spread on the far quarter of a 90 mm sheep blood agar plate. The robot streaks it
out for isolation with a sterile single-use 10 µL loop, the way a medical laboratory scientist does at the plating
bench: lift the lid only partly and keep it over the plate, drag the loop lightly through the specimen into the second
quadrant in several back-and-forth lines, then into the third through the second's lines, then into the fourth
through the third's, without carrying material back into an earlier quadrant and without cutting the agar; then close
the plate and drop the used loop in the discard tray. The work belongs to
[O\*NET 29-2011.00 medical and clinical laboratory technologists](https://www.onetonline.org/link/summary/29-2011.00)
and [19-1022.00 microbiologists](https://www.onetonline.org/link/summary/19-1022.00). These occupations total under
500,000 US workers, so the task is filed under **Other** (specialised laboratory work): streaking for isolation is the
first physical step of almost every bacterial culture in clinical, food-safety, pharmaceutical and research labs, and
every identification and antibiotic-susceptibility result depends on getting single colonies from it.

What makes it physical: the loop's 1.2 mm neck flexes under a few grams of load, so the tip has to be kept riding on
the gel (light enough not to cut it, firm enough to stay on it) along four ordered zigzags inside a dish 15 mm deep,
with the tool held at an angle so its neck clears the rim; meanwhile the other hand holds a 9 g lid ajar over the
plate and finally seats it back on a base with 0.7 mm of radial clearance. Unlike tasks that flame or carry a loop, or
draw a fixed mark, the streak pattern itself is scored, and it is scored only where the loop actually rests on the
agar.

## Run and score

From this task folder, with the official YAM Arena environment installed:

```sh
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=streak_agar_plate_for_single_colonies.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Streak the plate for single colonies, then discard the loop." \
  --scorer task_score --max-steps 2600 --store-frames --log-dir ./demo-run
```

For a model trial replace `--policy yam_scripted` with `--policy dream`. Use zero object jitter. The scene has no
hidden or seeded state, so every `--seed` gives the same layout. Offline scorer tests:
`python -m pytest -q -p no:cacheprovider` in this folder with `yam_arena` installed.

Each trial allows 2,600 control steps (130 s at 20 Hz). The scripted demo lasts 64.8 s including a 3.5 s stillness
tail and plays open loop from the known start layout and elapsed time only; it reads no object pose, joint or score.
On the GPU it reaches success at about 62 s of simulated time (1,237 steps). Wall time is
about 0.2 s per control step: a demo trial takes about 5 minutes including simulator start-up, a full-limit trial
about 10. The scorer prints `STREAK`, `HANDS` and `TRACE` diagnostic lines (sector detail, gripper points, recorded
trace); they do not affect the score.

The plate base, lid and loop are free rigid bodies (the loop has one passive hinge); the rack and tray are fixed. All
geometry is primitives and procedurally revolved meshes from the included kit modules (`yam_parts.py`,
`yam_joints.py`, `yam_mesh.py`) and our own code (`streak_ink.py` draws the streak lines); no third-party models,
textures, materials or fonts are used.

## Setup (world metres; x forward from the arm bases, y left, z up)

Arm bases (0, ±0.305, 0.750); bench top z = 0.7475. Every part has zero yaw.

| Item | Real product type | Material | Dimensions | Mass | Pose |
|---|---|---|---|---|---|
| Plate base | 90 × 15 mm Petri dish base (EN ISO 24998 size class), e.g. Thermo Scientific Sterilin 101R20 | clear polystyrene | Ø 88.0 mm outside, wall 0.8 mm, floor 1.0 mm, height 14.0 mm; outer rim edge rounded 0.4 mm (a chamfer in the model); wall revolved with 128 sides | 9.4 g | free; underside centre (0.165, 0.000, 0.7475); friction 0.4 / 0.3 |
| Agar | 5 % sheep blood agar (Columbia base), 23.2 mL | gel, 1.03 g/mL | Ø 86.0 mm, 4.0 mm deep, surface 5.0 mm above the bench | 23.9 g | one body with the base (33.3 g together) |
| Lid | matching 90 mm Petri dish lid | clear polystyrene | Ø 91.0 mm outside, top 1.0 mm, skirt 8.0 mm high, wall 0.8 mm, rim inner edge flared 0.4 mm; 0.7 mm radial clearance to the base | 8.7 g | free; seated, skirt rim 6.0 mm and top 15.0 mm above the bench; friction 1.0 / 0.9 (polystyrene on the gripper's rubber pads) |
| Specimen | primary inoculum already spread | visual only | annular sector 16-38 mm from the plate centre, ±30° about +x | — | far (+x) quarter |
| Loop | sterile single-use 10 µL inoculating loop, 200 mm (e.g. Thermo Scientific Microspec 10 µL) | polystyrene | round handle Ø 5.0 mm for 44 mm above the neck, then a flat finger grip 7.0 × 2.6 mm for 56 mm; neck Ø 1.6 mm × 40 mm then Ø 1.2 mm × 55 mm; ring Ø 5.0 mm outside, 0.8 mm thick, in the plane through the axis that contains the grip's thin direction | 2.15 g (2.06 g handle + 0.09 g neck and ring) | free; upright, ring down, axis at (0.130, -0.200); shoulder (handle-to-neck step) on the rack's lower tier at z = 0.8557; tip 8 mm above the bench; grip thin along x |
| Loop rack | two-tier slotted stand (build below) | clear acrylic, 4 mm | post 10 × 30 × 148 mm, front face 15 mm behind (−x) the loop axis; tiers with tops at 108 and 148 mm, each two 30 × 8 mm tines forming a slot open toward +x: lower 3.0 mm wide (neck), upper 6.4 mm (round handle); each slot's closed end 0.3 mm behind the loop | fixed | base at (0.130, -0.200, 0.7475) |
| Discard tray | autoclavable benchtop biohazard discard tray | red polypropylene | 100 × 230 × 30 mm outside, 2.5 mm walls and floor (95 × 225 mm inside) | fixed | centre (0.275, -0.150, 0.7475) |

**Loop neck.** The bending of the thin neck is lumped into one hinge 60 mm from the tip, about the axis normal to the
ring plane: stiffness 0.015 N·m/rad (= 3EI/L for a 1.2 mm polystyrene neck, E = 3 GPa, L = 60 mm), preloaded 2°
against its straight stop, 20° bend limit, damping 0.001 N·m·s/rad, armature 1e-5 kg·m². Calibrate a physical loop by
clamping the handle level and hanging 1.5 g on the ring: the tip should drop about 3.5 mm (tip stiffness 4.2 N/m).
Friction: handle 1.0 / 0.8; ring 0.10 / 0.08 (on wet agar). Loop dimensions are those of a typical moulded 10 µL loop; measure the loop
used in a real trial (length, grip, neck) and keep the 1.5 g / 3.5 mm neck calibration.

**Rack build.** Cut from 4 mm acrylic: a 10 × 30 mm post 148 mm tall glued square to a base plate, and two tiers glued
to its front face with their tops at 108 mm and 148 mm. Each tier is a U: two 30 × 8 mm tines with a 3.0 mm (lower) or
6.4 mm (upper) slot between them, closed at the post end, open toward the plate. The loop drops in from above with its
neck in the lower slot; its shoulder rests on the lower tines.

**Streak lines on screen.** Where the tip has ridden on the agar, cream dots 2.2 mm across appear (a fixed grid of
visual-only discs under the plate body; the scorer never reads them). The specimen area is drawn the same way.

## Scoring

Trace: every 50 ms step, the tip (lowest point of the ring) is recorded in the plate's frame when it rests on the agar:
the neck is bent at least 1.0° (tip pushed back at least 1 mm, about 0.02 N), the tip is within 2 mm of the agar
surface and at least 2 mm inside the wall. Consecutive recorded steps form one unbroken line (a touch). A hovering
tip, or one pressed on anything else, records nothing.

Quadrants: the two diagonals through the plate centre at 45° either side of the specimen's direction split the plate
into quadrant 1 (specimen) and quadrants 2, 3, 4 going round in either direction, set by the side the first line
leaving quadrant 1 goes to. A point is clearly in a quadrant when it is at least 3 mm from both diagonals.

Quadrant k (2, 3, 4) counts as streaked when all of these hold:

- (a) the touch that first goes clearly into it has, earlier in the same touch, picked up material: for quadrant 2
  any point inside the specimen area; for 3 and 4 it crosses (passes within 1 mm of) a line lying in the previous
  quadrant that was drawn earlier (in an earlier touch, or at least 10 mm earlier along the same touch);
- (b) its lines clearly inside it total at least 50 mm and make at least 3 back-and-forth strokes, each running at
  least 8 mm across the quadrant;
- (c) from its start until the next quadrant starts, nothing is drawn clearly in a quadrant two or more back
  (quadrant 3 stays out of 1; quadrant 4 out of 1 and 2);
- (d) the quadrants before it count.

| Score | Condition |
|---|---|
| 0 | No quadrant streaked, or any sticky violation below |
| 0.2 / 0.4 / 0.6 | Quadrants 2 / 2-3 / 2-4 streaked in sequence |
| +0.2 | Plate closed (needs ≥ 1 quadrant): lid seated on the base (centre within 2.5 mm of the base centre, rim within 2 mm of its seated height, tilt within 3° of the base), both grippers' centre points ≥ 30 mm from the lid's bounding box for 1 s, and lid and base moved < 1 mm and < 1° over that second |
| +0.2 | Loop discarded (needs ≥ 1 quadrant): the loop's whole bounding box inside the tray's inner walls and below 10 mm above its rim, ≥ 5 mm clear of the plate's and lid's boxes, both grippers ≥ 30 mm from it for 1 s, and the loop moved < 1 mm and < 1° over that second (neck < 0.5 rad/s) |
| 1.0 | All three quadrants, plate closed, loop discarded; success needs 1.0 held one more second |

Sticky zero for the rest of the episode, even if undone later: a gouge (the tip on the agar with the neck bent 18° or
more, a tip load of about 0.15 N, or the tip more than 2 mm below the agar surface); the lid touching the bench (its
lowest point within 2 mm of the bench while not seated on the base); the lid taken off the plate (its centre more than
80 mm horizontally from the base centre); the plate tipped more than 15°. Doing nothing scores 0, and partial credit
is the current state of the plate and items, never a best-so-far.

**Real trial.** Use a mock specimen (blue food dye in 10 % glycerol spread on the far quarter) or a BSL-1 strain such
as *Micrococcus luteus* incubated overnight at 35-37 °C. Photograph the plate from above through a transparent overlay
of the two diagonals with 3 mm bands; measure line lengths and strokes with a ruler. A video of the trial gives the
order of the quadrants, the lid's path and contacts, and the 1 s stillness. Any tear, cut or furrow in the agar is a
gouge. No live hazard is involved: the loop is plastic, the plate a standard consumable, and the arm has an e-stop;
keep people outside its reach.

## Demo

The left hand pinches the lid near its +y edge across an 89-90 mm chord (pads 2 mm past the skirt, about 10 N), lifts
it straight up 12 mm until the skirt clears the base wall, then raises it and opens it 30° as a clamshell over the
plate's +y side, where it stays until the streaking is done. The right hand comes in beside the loop, slides its open
fingers onto the flat grip, closes, lifts the shoulder 8 mm off the ledge, slides the loop out of the rack along +x,
then tilts it 35° from vertical toward +y. It streaks at 20 mm/s with the tip commanded 5 mm below the agar surface
(the neck bends about 7-10°, which carries the tip about 9 mm forward, so the plan aims 9 mm short in +y): quadrant 2 starts with two passes from the specimen, then a five-line zigzag;
quadrants 3 and 4 each start with two passes from the middle of the previous quadrant's zigzag. Near the right hand
(quadrant 4) the tip stays 14 mm inside the wall so the rising neck clears the rim. The loop is then lifted out, the
lid flattened above the plate, lowered straight down and released, and the loop lowered into the tray and released;
both hands back off and hold still.

## Reset

Fresh poured plate (23 mL blood agar) with the mock specimen on the far quarter, lid on, at the marked spot; a new
sterile loop upright in the rack; tray emptied into autoclave waste. The reset clears the recorded trace, timers and
sticky flags. Object jitter must be zero.

## Fidelity

Agar is rigid in the simulator: contact is read from the loop neck's bend (the flexible neck lumped into one sprung
hinge) plus tip height, and the streak lines are a visual readout of that same contact. The ring is a solid 5 mm disc.
The specimen is a drawn patch and no colonies grow, so isolation is judged from the streak geometry that produces it.
A gouge is a tip-load threshold, not a modelled tear. Dish walls are 128-sided; the rim roundings are 0.4 mm chamfers.
Lid and loop have their real masses (8.7 g, 2.15 g). At 320 × 176 the loop is a few pixels wide and the streak lines
1-2 pixels; the lid and plate are about 30 pixels across.
