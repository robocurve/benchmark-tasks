# Close and bolt a double-door enclosure whose astragal forces the leaf order

Instruction: "Close the blue-grey enclosure's two doors and throw the door bolt."

A small steel enclosure has two doors that meet in the middle with no centre post. The right (active) door carries
an astragal, a steel strip that overlaps the seam from the front. The left (passive) door has a vertical bolt that
slides up into a keeper in the enclosure's top header. Closing the doors properly takes the order the hardware
imposes: the passive leaf must be shut before the active leaf, because a shut astragal blocks the passive leaf by
contact. The passive leaf must also be bolted. The bolt can be thrown before or after the active leaf closes, and
the scorer checks only that it is thrown at the end. Two independent cabinet doors can be closed in either order;
these doors cannot.

The enclosure stands in for a no-centre-post electrical cabinet. Electrical safety practice (NFPA 70E) treats
equipment as in normal operation only with its doors closed and secured; overlapping door pairs with an astragal
are also common on fire-rated doors (NFPA 80). This tabletop model is not a certified electrical or fire assembly.

## Setup

World axes: x forward from the arms, y to the robot's left, z up, metres. The table top is at z = 0.7475 m and the
arm bases stand at (0, +0.305, 0.75) and (0, -0.305, 0.75). The three mechanisms (the enclosure, the left door and
the right door) share one origin at (0.215, 0, 0.7475) m with yaw 0: on the table top, 215 mm in front of the arm
bases and midway between them, in the plane of the closed doors. Below, all positions are millimetres in that
frame: local x points away from the robot (into the enclosure), y to the robot's left, z up from the table top.
The doors open toward the robot.

Materials and colours: the shell and the door sheets are powder-coated (painted) steel, blue-grey RGB
(0.12, 0.32, 0.52); the keeper, bolt guides, bolt rod and astragal are bare steel (0.48, 0.52, 0.57); the pulls,
the bolt post and the catch ball are brass (0.85, 0.56, 0.12). Every contact has friction 0.8 static and 0.6
dynamic. The enclosure body and the two hinge pins are fixed to the table; there are no other objects.

| Moving body | Joint, anchor (mm) | Range | Start | Mass | Passive coefficients |
| --- | --- | --- | --- | --- | --- |
| Left (passive) leaf | hinge about z at (0, 120, 85) | -100..0 degrees | -50 degrees (50 degrees open) | 180 g with its pull and guides | damping 0.020 N m s/rad, no spring |
| Right (active) leaf | hinge about z at (0, -120, 85) | 0..100 degrees | +70 degrees (70 degrees open) | 200 g with its pull and astragal | damping 0.020 N m s/rad, no spring |
| Bolt, on the passive leaf | slide along z | 0..15 mm up | 0 (down) | 20 g | damping 0.020 N s/m |
| Catch ball, in the enclosure | slide along x | -4..0 mm | 0 | 3 g | spring 300 N/m toward +x (rest +3.8 mm), damping 0.05 N s/m |

0 degrees is closed for both leaves; the hinge stops at 0 degrees set the closed position. Every collider, at zero
joint values (doors closed, bolt down); cylinders are vertical:

| Body / shape | Primitive | Local centre (mm) | Size (mm) | Material |
| --- | --- | --- | --- | --- |
| enclosure / floor | box | (70, 0, 3) | 140 x 252 x 6 | painted steel |
| enclosure / roof | box | (76, 0, 167) | 128 x 252 x 6 | painted steel |
| enclosure / back | box | (137, 0, 85) | 6 x 252 x 170 | painted steel |
| enclosure / left side | box | (74, 123, 85) | 132 x 6 x 170 | painted steel |
| enclosure / right side | box | (74, -123, 85) | 132 x 6 x 170 | painted steel |
| enclosure / right flange | box | (13, -114, 85) | 6 x 12 x 158 | painted steel |
| enclosure / left flange | box | (13, 114, 85) | 6 x 12 x 158 | painted steel |
| enclosure / lower header bar | box | (13, 0, 10) | 6 x 216 x 6 | painted steel |
| enclosure / upper header bar | box | (13, 0, 160) | 6 x 216 x 6 | painted steel |
| enclosure / keeper front wall | box | (-20, 40, 174) | 4 x 24 x 12 | bare steel |
| enclosure / keeper right wall | box | (-10, 30, 174) | 16 x 4 x 12 | bare steel |
| enclosure / keeper rear wall | box | (0, 40, 174) | 4 x 24 x 12 | bare steel |
| enclosure / keeper left wall | box | (-10, 50, 174) | 16 x 4 x 12 | bare steel |
| enclosure / keeper rear bridge to the header | box | (7, 40, 174) | 10 x 24 x 12 | bare steel |
| enclosure / catch ball | sphere | (-15.8, 40, 167) | radius 3 | brass |
| left door / hinge pin (fixed) | cylinder | (0, 120, 85) | radius 3, height 140 | painted steel |
| left leaf / sheet | box | (0, 61, 85) | 3 x 118 x 152 | painted steel |
| left leaf / pull bar | cylinder | (-14, 24, 92) | radius 4, height 50 | brass |
| left leaf / pull standoffs | 2 boxes | (-7, 24, 70), (-7, 24, 114) | 14 x 8 x 6 each | brass |
| left leaf / lower bolt guide, at z = 125 | 4 boxes | (-17.5, 40), (-2.5, 40); (-10, 32.5), (-10, 47.5) | 3 x 18 x 5; 12 x 3 x 5 | bare steel |
| left leaf / upper bolt guide, at z = 148 | 4 boxes | as the lower guide | as the lower guide | bare steel |
| bolt / rod | cylinder | (-10, 40, 134.75) | radius 4, height 29.5 (z 120..149.5) | bare steel |
| bolt / neck | cylinder | (-10, 40, 152) | radius 3, height 5 (z 149.5..154.5) | bare steel |
| bolt / rod top | cylinder | (-10, 40, 156.25) | radius 4, height 3.5 (z 154.5..158) | bare steel |
| bolt / rounded tip | sphere | (-10, 40, 158) | radius 4 | bare steel |
| bolt / arm | box | (-34, 40, 122) | 48 x 6 x 6 | bare steel |
| bolt / post (the handle) | cylinder | (-58, 40, 137.5) | radius 8, height 25 | brass |
| right door / hinge pin (fixed) | cylinder | (0, -120, 85) | radius 3, height 140 | painted steel |
| right leaf / sheet | box | (0, -61, 85) | 3 x 118 x 152 | painted steel |
| right leaf / pull bar | cylinder | (-14, -24, 92) | radius 4, height 50 | brass |
| right leaf / pull standoffs | 2 boxes | (-7, -24, 70), (-7, -24, 114) | 14 x 8 x 6 each | brass |
| right leaf / astragal | box | (-9, 5, 85) | 3 x 18 x 152 | bare steel |
| right leaf / astragal spacer | box | (-4.5, -5, 85) | 6 x 4 x 152 | bare steel |

The bolt does not collide with its own leaf; the guides are visual and the slide joint keeps the rod on its axis.
(At bolt travel 0 to 2.5 mm the bolt arm passes through the lower guide's front wall without contact.)

Clearances that decide the task, from this geometry:

- With both doors shut, the astragal (local x -10.5..-7.5, y -4..14) covers the passive sheet's free edge (y = 2)
  from the front, 6 mm in front of the passive sheet. The passive leaf can therefore sit ajar up to 2.9 degrees
  behind a shut astragal (4.0 degrees if the active leaf is 1 degree open).
- With the active leaf shut, a closing passive leaf meets the front of the astragal at 6.46 degrees open (about
  7.7 degrees with the active leaf 1 degree open, inside the 8 degree band top) and cannot close further.
- The keeper's bore is 16 mm square, from z = 168 to 180, 4 mm of clearance around the rod. With the bolt down, the
  rod's tip is 6 mm below the keeper. A passive leaf closing with its bolt already raised hits the keeper at 12.25
  degrees open and cannot shut; a bolt raised on an open leaf, away from the keeper, falls back under its own weight.
- At full throw (15 mm), the spring catch ball sits in the rod's neck: it holds the bolt up (its shoulder supports
  about 6.8 times the bolt's weight) and presses the passive leaf toward closed.

The active leaf has no latch or lock and stays shut only through its hinge damping; the passive leaf has a single
top bolt and no bottom bolt.

Reset: set the passive leaf to 50 degrees open, the active leaf to 70 degrees open, the bolt down (0) and the catch
at 0, all at rest, and clear the scorer's history. No part is consumed.

## Scoring

The score is the end state at the last step, except for one sticky rule. Angles are hinge angles; for a tape
measure, the gap between a leaf's free edge and its closed line is 118 mm x sin(angle) (1 degree is 2 mm).

| Score | End state |
| --- | --- |
| 1.0 | Both leaves within 1.0 degree of closed (free edge within 2 mm of its closed line), and the bolt raised at least 10 mm of its 15 mm travel (its rounded tip at least 4 mm inside the 12 mm tall keeper bore), with both grippers at least 30 mm from each leaf's and the bolt's bounding box, hinge speeds below 0.05 rad/s and bolt speed below 0.010 m/s, all continuously for 1 s |
| 0.75 | The same closed and bolted geometry, while held, moving, or before the 1 s has passed |
| 0.5 | Both leaves within 1.0 degree of closed, the bolt raised less than 10 mm |
| 0.25 | The passive leaf within 1.0 degree of closed with the bolt raised at least 10 mm; the active leaf more than 1.0 degree open |
| 0 | Every other end state, including the untouched start, a passive leaf more than 1.0 degree open, and a raised bolt on an open passive leaf |
| 0 for the rest of the trial | The passive leaf 4.5 to 8 degrees open (free edge about 9 to 16 mm out) while the active leaf is within 1.0 degree of closed, for at least 0.25 s: the passive leaf forced against the front of a shut astragal |

The sticky zero stays even if the doors are later reopened and closed in the right order. Passing through the
band for less than 0.25 s, or with the active leaf more than 1 degree open, does not set it. A passive leaf ajar up
to 2.9 degrees behind the shut astragal is not a violation: it can still be pushed shut and bolted for 1.0.

Online, speeds are the change in measured joint values between consecutive 20 Hz steps; offline (static layouts
in the tests) they are the reported joint velocities, release is assumed and a held condition counts as already
held. The trial ends in success once the score has been 1.0 for a further 1 s (`SUCCESS_HOLD_S`).

## The scripted demo

The demo acts only on the known start layout and elapsed time. Each arm, gripper closed, pushes its leaf shut by
the outer face: the pad follows the leaf's arc in about 5-degree steps of 0.28 s, 85 mm from the centreline (35 mm
from the hinge) at 155 mm above the table, then presses 6 degrees past closed for 0.8 s against the hinge stop.
It backs off 60 mm straight toward the robot before any sideways move.

The left arm closes the passive leaf first. It then rises, pinches the brass bolt post at local (-58, 40) and
145 mm above the table, lifts it 20 mm (the 15 mm stroke plus 5 mm to absorb arm compliance), holds 0.5 s, lets
go, rises and parks at (0.105, +0.250, table + 0.180) m. The right arm waits at (0.105, -0.250, table + 0.180) m
for 17 s after its 1.5 s start move, then closes the active leaf over the passive one and parks.

## Run a trial

From this folder, in the official simulator environment:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=close_double_door_enclosure.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Close the blue-grey enclosure's two doors and throw the door bolt." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/double-door-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=close_double_door_enclosure.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Close the blue-grey enclosure's two doors and throw the door bolt." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/double-door-demo
```

The run prints `task_score` at the end. Offline checks of the scorer and the demo's reach (no GPU):
`pytest test_close_double_door_enclosure.py` in an environment with `yam_arena` installed.

## Runtime and seeds

- The right arm's plan takes 26.68 s of simulated time (the left arm finishes the bolt at 13.56 s), then both arms
  hold clear for 4 s, 30.68 s in total; with the 1 s stillness hold and the 1 s success hold it ends in success well within the 1,200-step (60 s) limit. On the official stack
  the demo, and a copy with every waypoint at 0.8x duration, each ended in success at score 1.0.
- One fixed layout: `-E object_jitter=0`, so the seed doesn't change the start.

No third-party assets; all geometry is built from primitives in this folder's kit files.
