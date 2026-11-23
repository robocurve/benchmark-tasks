# Put a binder clip on a stack of paper

Instruction: "Clip the stack of paper standing in the letter sorter with the binder clip on the desk: squeeze its two handles to open it, slide it down over the stack's top edge near the right corner as far as it goes, let go, then fold both handles down flat against the paper."

The opposite act to the merged task `remove-binder-clip-before-shredding`, which takes a clip off: here a medium binder clip goes on. Its steel jaw is a strong spring that opens only while the two wire handles are squeezed. The robot squeezes them with one gripper, slides the open jaw over the paper, lets it spring shut on the stack, and folds each handle down. The other hand holds the stack still in the sorter.

## Scoring

Judged once both grippers are at least 30 mm from the clip and the paper, and neither has moved more than 1 mm for 1 s. Before that the score is 0.

**Score 0 if** any of these holds:
- the clip is not on the stack: the paper's top edge is less than 3 mm into the jaw, the jaw is not open around the paper (lips less than 2.5 mm apart), either end of the clip is less than 3 mm inside the paper's ends, or the clip lies on the table;
- the stack is out of place: its foot is more than 10 mm off the slot floor, or the stack stands more than 10 mm off the front wall or off the divider it leans on.

**Otherwise the score is the lowest of these four:**

| Term | 1.0 | 0.75 | 0.5 | 0.25 |
|---|---|---|---|---|
| Seat: paper's top edge to the back of the jaw | <= 3 mm | <= 6 mm | <= 12 mm | more |
| Square: the clip's lip line against the paper's top edge | <= 3 deg | <= 8 deg | more | - |
| Stack: foot to the front wall, back face to the divider | <= 2 mm | <= 5 mm | <= 10 mm | - |
| Handles folded down flat on the paper | both | one | none | - |

A handle counts as folded when its loop end lies 20 mm or more below the jaw's lips and within 10 mm of the paper's face on its own side. Every outcome has one score: doing nothing scores 0 (the clip stays on the desk), and the start state meets no tier above 0. A clip let go with the paper only a few millimetres into the jaw scores 0.25 at most (0 under 3 mm). A fully seated clip with both handles still up scores 0.5. One folded handle scores 0.75, and 1.0 needs both.

### Real trial

Use a photo and a 150 mm ruler after the robot lets go. Seat: on the paper's face, measure from the top edge down to the clip's lip line. The jaw is 19.5 mm deep (estimate; measure the clip), so a lip line 16.5 mm or more below the edge scores 1.0. Square: the angle between the clip's lip line and the paper's top edge (a clip rocked a little on the edge still counts as square). Stack: the gap between the stack's foot and the sorter's front wall, and between its back and the divider's top edge. Folded handles lie down along the paper below the jaw, their loop ends within 10 mm of it. On the clip: lift the bundle 2 cm by the clip; the paper must come with it.

## Scene (millimetres from the midpoint between the arm bases; x forward, y to the robot's left; heights above the table)

- **Letter sorter**: a small three-slot metal mesh desk letter sorter, 178 long x 83 deep x 133 tall (VANRA small letter sorter, listing https://www.amazon.com/dp/B08MJJJ85C: 7 x 3.25 x 5.25 in). It stands at x 185-268, y -80 to +98, its slots running left to right. Wall heights front to back are 60, 85, 110 and 133, with 24 mm slots (estimates; ruler check).
- **Paper**: 40 sheets of 80 g/m2 A5 copy paper, 210 x 148 (ISO 216), 4.0 thick, 100 g (0.10 mm and 2.5 g a sheet are estimates; check with a ruler and a kitchen scale). The stack stands on its long edge in the front slot, foot against the front wall, leaning back 13.5 degrees on the first divider. It spans y -130 to +80, so its right end stands 50 past the sorter. Its top edge is 147 above the table.
- **Binder clip**: a medium binder clip, 32 wide with 16 capacity (1-1/4 in, 5/8 in; e.g. ACCO 72050, https://www.amazon.com/dp/B001FLREDE; Universal 10210, https://www.ontimesupplies.com/unv10210-binder-clips-medium-black-silver-12-box.html). It has a black spring-steel body and two silver wire handles. It stands on its closed jaws at x 150, y -165, jaw line along y, both handles up.
  - Estimates, each checkable with a ruler: spine 17 wide, jaw 19.5 deep, steel 0.4, rolled lips 2.6 across, handle wire 1.4, handle loop 37 long, mass about 8 g.
  - Raised, the handles bear on the spine's corners and continue the line of the jaw walls, about 22 degrees outward.
  - Spring (estimates; check by pressing the handles together on a kitchen scale): the lips press together at about 3 N when empty and clamp the 4 mm stack at about 7 N. Holding the jaw 9.5 mm open takes about 17 N at the handle ends; fully open (16 mm) takes about 25 N.

Cost: about US$20 for the sorter, about US$8 for a box of 12 clips and about US$10 for a ream of A5 paper. Reset (1 min): take the clip off, tap the sheets square, stand them back in the front slot (foot against the front wall, right end 50 mm past the sorter, leaning on the first divider), and stand the clip on its jaws at its mark with both handles up.

## Simulation fidelity

GPU PhysX at 240 Hz, 20 Hz control. The 40 sheets are one rigid 4 mm block built from four convex tiles. Single sheets, their bending and their slip on one another are not modelled, so the scorer checks the stack as a whole: where it stands, and how far and how square the clip sits on its edge. The clip is an articulation: a spine, two jaw walls hinged at the spine corners on one torsion spring each, and two wire handles hinged in the rolled lips. The handles have a raised stop and joint friction, so they stay where they are put. Rolled lips and wires are convex prisms. Sorter panels are solid 3 mm plates, not mesh. Every unpublished size, mass, friction and spring figure is an estimate (listed above). Nothing is attached or teleported. The scripted demo reads object poses to aim each move.

Solver settings, not physical claims: the clip's articulation and the paper stack use 64 position iterations (the merged binder-clip task uses the same), the jaw hinges carry a small added inertia (armature 2e-5 kg m2) and damping (5e-3 N m s/rad) that settle contact chatter of the clamped jaw, and contact offsets are 2 mm. Stillness is judged from poses (under 1 mm in 1 s) because that chatter shows in velocity readings, not in the pose. A back handle folded against the stack's overhanging back face does not stay put for long: in the simulation it creeps back up at about 1 degree per second once the finger leaves it.

Why the squeeze stops short of the stop: squeezed until the raised handles stand parallel, a 2-degree tilt puts one handle's pad on its fulcrum and that jaw snaps shut. The demo therefore holds the jaw about 13.5 degrees open (lips about 9.5 mm apart), then opens its pads over 1.6 s so the jaw closes gently on the paper. A person does the same, squeezing only as far as needed.

## Run

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=put_a_binder_clip_on_a_stack_of_paper.py -E object_jitter=0 -E control_hz=20 \
  --scorer task_score --max-steps 1800 --store-frames --log-dir runs/demo
```

The layout is fixed (`object_jitter=0`): the stack stands leaning in a 24 mm slot, so a random shift would push it into the sorter's walls. The scripted demo reads object poses where it needs them.

## Results

- Demo (scripted, fixed layout): 1.0. Seat 2.2 mm, lip line 0.1 degrees off the edge, stack 0.1 mm off its backstops, both handles folded (loop ends 8.4 mm and 2.5 mm from the paper); full credit at 42.3 s.
- Wrong way (let go with the paper aimed only 4.5 mm into the jaw, then fold both handles): 0.0. The clip hangs on by its lips with the paper 2.8 mm into the jaw, under the 3 mm minimum.
- Dream-1: 0.0 (full 1,800 steps).

## Files

`put_a_binder_clip_on_a_stack_of_paper.py` holds the scene, clip, scorer and demo. `yam_parts.py`, `yam_joints.py` and `yam_mesh.py` are the shared part kit. Every shape is built from boxes and convex prisms in the module; no third-party assets.
