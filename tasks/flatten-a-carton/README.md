# Knock an empty cardboard box down flat for recycling

An empty corrugated shipping box lies on its side on the table with both ends open: its top flaps were opened to
unpack it and the tape on its bottom was slit, so all eight flaps are folded out in line with their panels. The robot
has to flatten it for the recycling bin and let go of it.

**Instruction:** "Flatten the empty cardboard box for recycling, so that it lies flat on the table, and let go of it."

The box is light and its creases hold it square, so it does not fold just because it is pushed. Pushed level at its
side, it slides across the table; pressed part way down, it springs back up when let go. It folds flat only under a
push aimed steeply enough, down as well as over, to hold it to the table while it shears, kept up as the top edge
travels along its arc, and carried through until the creases set.

## Where this comes up

Every delivered box ends up empty, and almost every empty box has to be broken down before it can go anywhere:
collection bins, balers and recycling pickups all take cardboard flat, and an open box is mostly air. The box in this
task holds 4.5 litres open and takes under 1 litre flattened, about a fifth of the room.

- **Homes.** Online orders and grocery deliveries leave a steady stream of boxes. Flattening them for the recycling
  bin is one of the most common household chores that a home robot would be asked to take over, and one that people
  put off because it is repetitive.
- **Shops and supermarkets.** Restocking shelves is unpacking cases, and every case unpacked leaves a carton to break
  down for the back-room baler. Shop staff do this many times a shift, usually by hand, standing at a bench or a cage.
- **Warehouses, receiving docks and returns.** Goods arrive in cartons that are emptied, checked and decanted into
  totes or onto shelves. Returns processing opens a box for every parcel. The empty cartons are broken down at the
  station before they are carted to the compactor.
- **Kitchens, cafes and bars.** Ingredient and supply deliveries come in boxes that have to be flattened and stacked
  for the recycling collection before the space is needed.
- **Offices, mail rooms, labs and hospitals.** Supplies and equipment arrive boxed; the boxes are broken down where
  they were opened.

In all of these, breaking down the box is the step that follows unpacking. A robot that can unpack a delivery but
leaves the empty cartons standing has not finished the job, and the empty boxes quickly take over the space it works
in.

## Potential impact

- **Closing the unpacking loop.** Unpacking, restocking and decanting are central to retail, warehouse and home
  robotics. Breaking down the empty carton is what lets that work run without a person stepping in to clear the
  boxes, so a robot that cannot do it hands the end of every unpacking job back to a person.
- **A skill that transfers.** The ability tested here is choosing a force direction on a light object that would
  rather slide than move the way it is meant to, keeping that direction while the contact point moves, and pushing
  through to a definite end state. The same ability is needed to collapse other folding things: folding crates and
  collapsible bins, pop-up laundry hampers, folding boxes and trays, tote dividers, folding stools and chairs. It is
  also what any press-to-set action needs: closing a lid that springs open until it clicks, seating a part that sits
  proud until it is pushed home.
- **A failure that a benchmark should catch.** A policy that pushes in roughly the right place but at the wrong angle
  gets no visible progress here: the box slides away, or folds part way and springs back. Success needs the force to
  be right, not just the contact position, and that is a gap that current models show (the Dream-1 run below never
  pressed on the box at all).
- **Safe to run for real.** The real-world version needs only an empty box and a table, nothing sharp, hot or heavy,
  so it can be run on hardware as often as needed, and a box can be reused for several trials.

## What it measures

- **Choosing the force direction.** The box weighs 116 g, and each of its four corner creases resists folding like
  a spring (0.19 N m at 90 degrees). A level push on the wall facing the arms slides the box across the table before
  it shears more than about 11 degrees (control run below). A push 70 degrees below level folds it without sliding:
  its sideways part is then 0.36 of its downward part, under the board's friction on the table (0.45).
- **Keeping that direction while the contact moves.** The top edge being pressed travels a 15 cm arc, from straight
  above the crease at the bottom of the near wall down to the table, and turns from facing sideways to facing up. The
  press has to follow it.
- **Finishing the fold.** The creases spring back toward square, so a box pressed part way and let go stands back up.
  Only once it is pressed fully flat (each corner within 6 degrees of flat) do its creases set and keep it flat, and
  it then has to be let go.

It is not a flap task: every flap starts open and stays open. Tasks that fold a box's flaps closed for taping test a
different skill, on a box that keeps its shape.

## Files

| File | What it is |
|---|---|
| `task.py` | The task file: scene, crease setting, scoring, the instruction |
| `cartonkit.py` | Builds the box: four rigid panels on four crease hinges in a closed loop (an articulation plus one loop-closing joint), with crease springs that the task file can switch off at run time; reports every panel's pose and the shear |
| `demo.py` | The scripted demo and the sideways control run |
| `test_scoring.py` | Offline tests of the scoring rule (no simulator) |
| `task.json` | The submission's details |

## Requirements

The yam_arena embodiment and Isaac Lab-Arena, as set up by the program's GPU instance image (an NVIDIA RTX GPU is
needed). `test_scoring.py` needs only Python with numpy, pytest and the `yam_arena` package.

## Run a trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Flatten the empty cardboard box for recycling, so that it lies flat on the table, and let go of it." \
  --scorer task_score --max-steps 1200
```

Run limit: 1,200 steps (1 minute at 20 Hz). Second layout: set `CARTON_LOT=B` in the environment (default `A`).

## Scripted demo

```bash
python demo.py task.py out_dir                  # layout A; CARTON_LOT=B for layout B
python demo.py task.py out_dir --mode sideways  # the control run
```

1. The right hand, tilted 40 degrees forward with its fingers closed, comes onto the box's near top edge (where the
   wall facing the arms meets the top panel) from outside and above.
2. It presses on that edge 70 degrees below level, toward the far side, and follows the arc the edge travels as the
   box shears: the near wall folds down onto the bottom panel, the top panel comes down and across, the far wall folds
   outward onto the table.
3. It keeps pressing, harder toward the end, until the box is flat and its creases set, holds it there for 0.8 s, and
   lifts clear.

The script is open loop: every pose comes from the layout constants in `task.py`, and object states are read only for
the result file. `--mode sideways` pushes the near wall square to its face, 9.5 cm up, as a level push would.

## Scoring

Tools: a ruler, a protractor or angle app, and the trial video. Read at the end of the run.

1.0 if the box lies flat:

- its cross-section is sheared at least 80 degrees from square, either way (each wall within 10 degrees of lying flat);
- no part of it is more than 2.5 cm above the table;
- its lowest point is within 1 cm of the table top (it is on the table);
- it is still: every panel moving slower than 5 cm/s;
- it is let go: no gripper's grasp point lower than 6 cm above the table over the box's outline grown by 3 cm.

Else 0. A box standing square, half folded, folded but held down, knocked off the table, or still springing back
scores 0. There is no partial credit: a box pressed part way springs back square once let go, so no intermediate end
state reflects progress.

In code: `score()` and `is_flat()` in `task.py`. The shear is read from the crease between the bottom panel and the
wall facing the arms, and the height from all four panels' corners.

## Verification

| Run | Score | Notes |
|---|---|---|
| Scripted demo, layout A | 1.0 | Flat at step 213 (11 s); box moved 0.4 cm; highest point 0.8 cm above the table |
| Scripted demo, layout B | 1.0 | Flat at step 213; box moved 0.5 cm |
| Control: level push on the near wall, 9.5 cm up | 0.0 | Sheared to 10.8 degrees, then slid 10 cm across the table and sprang back square |
| Dream-1, 1,200 steps | 0.0 | Both grippers went to the ends of the box several times; it was never pressed and stayed standing square |
| `pytest test_scoring.py` | 9 passed | Flat either way, standing, half folded, too high, held, moving, off the table, the let-go zone |

## Runtime and seeds

About 1.5 minutes per demo run and 4 minutes per Dream-1 run on one L40S, including about a minute of start-up.
`object_jitter=0`; the layout is fixed per lot (A or B), and there is no randomness in the task.

## Fidelity

- Each panel is one rigid 3 mm board with its two end flaps; a real board bows a little, and its flaps can bend at
  their own creases. Here the flaps stay in line with their panels.
- Each corner crease is a hinge with a spring of 0.12 N m/rad toward square, which is how a used carton's creases
  hold it up. Real creases are uneven and soften with use.
- Real creases give way once a box is pressed fully flat. That is modelled as the springs switching off when the
  shear reaches 84 degrees (each corner within 6 degrees of flat).
- The panels do not collide with each other; hinge limits at 88 degrees stop the layers where the boards would meet.
- Board on the table: 0.45 static, 0.38 sliding friction (assumed values for kraft board on a laminate top). Mass:
  550 g per square metre of B-flute board.
- Grip force is not judged.

## Real-world build

Any empty single-wall regular slotted carton about 20 x 15 x 15 cm inside, opened at both ends with the flaps folded
out, laid on a 20 cm panel at the layout's position on a smooth table. Opening and flattening it a few times beforehand
gives creases close to the modelled ones; replace it when it no longer stands square by itself. Measure the shear on
the end view with a protractor and the height with a ruler.

## Third-party assets

None: the box is built from primitive shapes in `cartonkit.py`.
