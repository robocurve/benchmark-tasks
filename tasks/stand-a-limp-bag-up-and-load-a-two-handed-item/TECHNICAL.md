## Why this task

O*NET lists "Bag, box, wrap, or gift-wrap merchandise, and prepare packages for shipment." among the tasks of
Cashiers (41-2011.00, https://www.onetonline.org/link/summary/41-2011.00). O*NET files that statement under
"Supplemental" tasks, not core tasks. The same page gives "Employment (2025)" as 3,106,300. Both were read from
the O*NET page on 20 November 2026.

Closest existing tasks, and how this one differs:

- Tasks in which one hand holds a bag's mouth open while the other drops a small part in (for example an
  antistatic bag, or a pillowcase): there the opening is the problem and a hand stays on the bag. Here the item
  needs both hands, so the bag must be made to stand by something other than a hand, and it must stay standing
  after the robot lets go.
- Tasks in which rigid boxes are moved with two arms: there the container or destination is rigid. Here the
  container is limp, and the end state is tested by lifting it.
- Putting up a folding crate's four walls until its corner catches click: those walls are stiff and lock once
  up. Here nothing locks; the walls stand only while the contents hold them, and fall again if the box is
  taken out.
- Stowing tasks with a rigid bin or shelf: no limp container.

## Physical reference and model parameters

The [cutting pattern](docs/tote-pattern.svg) and [assembly and acceptance instructions](README.md#build-the-tote) define the physical reference. Dimensions and masses below are build targets. The original parameter labels in `bag_model.py` describe how the simulation was authored. No physical sample was measured for this revision.

| Part | Physical target | Simulator representation |
| --- | --- | --- |
| Base and walls | 310 x 160 mm board inside a non-woven polypropylene shell, rim 180 mm above the table, one crease at 90 mm | 4 mm board and two 88 mm panels per wall give a 180 mm rim. Fabric can gather at sewn corners and sag between its ends. |
| Handles | Two centred stiffened loops, each 150 x 90 mm outside, padded grip 18 x 10 mm, exterior hinge and inward stop at each root | Rigid loops with 16 mm legs and a revolute joint along each large wall's rim. |
| Empty tote | 109 g assembled | 45 g board, four 8 g side panels, four 4 g end panels and two 8 g handles. The physical mass check is for the whole tote. |
| Groceries | 60 x 132 x 215 mm closed carton at 280 g and 200 x 100 x 200 mm film-wrapped four-roll bundle at 360 g | Rigid cuboids omit carton flex, roll gaps and film compression. |
| Contact and folds | Non-woven body, plastic board, paperboard carton, wrapped paper rolls and webbing | Estimated friction coefficients are 0.50 (fabric and board), 0.45 (carton), 0.90 (film) and 0.70 (strap). Wall and crease stiffness and friction have not been measured on a built sample. |

## How it is modelled

PhysX in Isaac Lab-Arena, 240 Hz physics, 20 Hz control, rigid bodies only. Nothing is moved, attached, held up
or latched by code.

- The bag is one floating articulation rooted on its base board. Each of the four walls is two rigid panels, 88 mm
  high: the lower panel is hinged to the edge of the base board, the upper panel to the lower one along a crease
  half-way up. Both hinges carry a weak spring (0.002 N m/rad, damping 0.001 N m s/rad) whose rest angle is a
  folded wall, standing for the crease a folded bag remembers. Gravity on a wall is about ten times the spring.
- The two large walls fold with the crease inside the bag, the two end walls with the crease outside, so that
  every wall's top edge stays above its bottom edge as it comes down. Four rigid walls folding inward would jam
  at the corners. The pattern uses the same crease directions and allows the fabric to gather at the corners.
- The sewn corners are four ties between the top corners of neighbouring walls: distance-limited joints with 2 mm
  of slack and 1000 N/m beyond it, outside the articulation. A wall that hangs lower than its neighbours is pulled
  up by them. A tie stretched more than 30 mm (28 N) counts as a torn seam in the scorer.
- Each handle is one rigid loop on a free hinge along its wall's top edge, with stops: hanging down it stands
  25 degrees off the wall (its grip 34 mm from the wall), and it cannot fold more than 10 degrees in over the
  mouth. The physical stops use square-ended stiffeners under a webbing hinge. The lab tests the finished grip against the mouth line under 2 N.
- Colliders: wall panels 2 mm thick, base board 4 mm. Panel masses are raised to 8 g (large walls) and 4 g (end
  walls) from nominal fabric-only masses of about 3 g and 1.5 g in the original estimate; lighter bodies made the solver unstable.
- Box and pack are single rigid boxes.

Recorded simulator checks without the robot (`tools/physx_probe.py goals`, last run on the files in this folder):

| Check | Requirement | Result |
| --- | --- | --- |
| `collapse` | empty bag, every wall pulled straight, let go: must fall below 144 mm within 2 s | pass: below 144 mm after 0.62 s; lowest wall top edge at the end 108 mm |
| `stand` | box standing in the bag, walls straight, let go: must stay above 144 mm | pass: lowest wall top edge at the end 177 mm |
| `push` | as `stand`, then a 1 N push on the left wall's top edge, outward 2 s and inward 2 s: must not tip | pass: lowest wall top edge at the end 177 mm |
| `block` | pack let down on the collapsed bag: must not reach the base | pass: below 144 mm after 0.12 s; lowest wall top edge at the end 10 mm |
| `tent` | empty bag let go, then the pack lowered onto it: must not reach the base | pass: below 144 mm after 0.62 s; lowest wall top edge at the end 106 mm |
| `insert` | box in the bag, pack lowered from just above the top edge: must reach the base, walls stay up | pass: lowest wall top edge at the end 177 mm |
| `full` | both items in the bag, let go: walls must stay above 144 mm | pass: lowest wall top edge at the end 177 mm |
| `lift` | both items in, pulled up by both grips toward the box's end: must hang level and come back upright | pass: base 54 mm up, 1.4 degrees off level; lowest wall top edge at the end 177 mm |
| `lift_mid` | as `lift`, pulled by the middle of the grips (information only) | not met: base 57 mm up, 11.6 degrees off level; lowest wall top edge at the end 177 mm |

`lift_mid` (lifting by the middle of the grips rather than toward the heavy end) tilts the bag about 12 degrees
and is reported for information; it is not a requirement.



## Physical grading

Scored on the state at the end of the trial (the run limit, or as soon as the 1.0 state has lasted 2 seconds). Nothing is scored for the order in which the items go in or for how they are held. Definitions. An item rests on the base: its lowest point is within 3 mm of the bag's base board, it stands on one of its faces (that face within 10 degrees of the board) and, seen from above, it lies wholly within the board's outline; an item lying on a folded-in wall is higher than that and does not count. The bag stands upright: its base board is on the table (lowest point within 5 mm of it), level within 10 degrees, its middle within 50 mm of its tape marks, and the top edge of each of its four walls is at least 144 mm above the table (80 % of 180). The lift is done once, for 2 seconds without a break, the base board's lowest point is at least 50 mm above the table, both handle grips are pulled at least 30 mm above the bag's top edge and both items remain in the bag. In the lift, each item’s centre stays over the base outline and its lower edge stays below the top of the bag. Check that neither exits on the video. Check the 3 mm base gap only after set-down. 1.0: both items rest on the base, the bag stands upright, the lift is done, and for the last 2 seconds no part of the robot touches the bag or an item and both grippers' centre points are at least 50 mm from the bag (its outline up to its top edge, and its handle grips). 0.75: both items rest on the base and the bag stands upright, but the lift is not done or the robot has not let go. 0.5: the pack of toilet rolls rests on the base and the bag stands upright (the box is anywhere). 0.25: only the box of crackers rests on the base and the bag stands upright. 0: anything else, including a bag that at the end is held up, lying over, slumped or more than 50 mm off its marks. Also 0, whatever happens later, once an item has fallen off the worktable, an item has come out of the bag while the bag was off the table, or a corner seam of the bag has been pulled more than 30 mm apart (torn). On a real trial: film from the side; at the end hold a ruler against each wall, photograph the bag from above and from the side, and try a 3 mm card under each item inside the bag. Possible scores: 0, 0.25, 0.5, 0.75, 1.0.

Qualify each build using the [physical acceptance check](README.md#physical-acceptance-check). Film the lift from the side with a 50 mm height reference. Use the end view or an overhead view to confirm the two groceries remain in the bag. Check their seating only after set-down. [Cashier tasks](https://www.onetonline.org/link/summary/41-2011.00).
