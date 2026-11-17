# Empty-seat installation and reset

This is an unoccupied passenger-seat buck, not an unobstructed tabletop belt.
The seat cushion, back, outboard pillar and door-side panel are solid obstacles
in the native simulation. Both robot arms must approach the belt around them.

Build the buck from 18 mm plywood, timber battens and upholstery foam, or use
an unpowered salvage passenger seat on a stable stand that matches these clear
spaces. Use a retired **47 mm polyester three-point belt with a freely sliding
steel tongue**, no cuts or frayed edges. Use a mechanical reel without a live
pretensioner, locked at the specified extension. The buck is a bench prop, not
an installation for carrying a person. Approximate bare plywood surfaces replace
trim and seat softness, which are not the skill being tested.

## Dimensions

Coordinates are in millimetres in the simulation's world frame. X runs from the
robot towards the seat back; positive Y is outboard. The dimensions in
`installation.py` are authoritative and are shared by collision and rendering.

| Part | Centre (X, Y, Z) | Size (X × Y × Z) |
| --- | --- | --- |
| Cushion | 365, −170, 540 | 450 × 450 × 100 |
| Seat back | 625, −170, 845 | 70 × 450 × 510 |
| Outboard pillar | 640, 155, 855 | 100 × 100 × 650 |
| Door-side panel | 365, 240, 680 | 600 × 30 × 400 |
| Seat support | 365, −170, 450 | 410 × 410 × 80 |
| Floor platform | 365, −80, 290 | 700 × 700 × 60 |
| Locked reel housing | 680, 155, 530 | 70 × 65 × 90 |

Round exposed plywood edges to a 3 mm radius. Screw the seat and pillar to the
platform with battens and four screws at each joint; ballast or clamp the
platform so a 10 N hand pull on the pillar does not move it. Do not add a
restraint to the working belt or tongue. The robot mounts are on a separate
140 × 900 mm crossbar centred at (200, 100, 797.5) mm; arm bases retain their
ordinary spacing, raised 80 mm. This places the arms beside the empty seat.

## Belt route

Attach the lower webbing end at **(144, 80, 597) mm**. The shoulder guide is at
**(586, 80, 1113) mm**, fixed to the pillar's forward/inboard corner. Leave
**750 mm of webbing** between those points, whose straight separation is
680 mm. Thread the original sliding tongue on this working span. From the
shoulder guide, route a further approximately **650 mm** behind the pillar to
the reel at (680, 155, 530) mm, and lock the reel length. It must not reel in
or pay out during this trial. The tongue remains unbuckled and can slide;
neither webbing end may be detached.

The simulated remote guide-to-reel section is fixed because the reel is locked;
the entire exposed working span remains flexible. This omits reel dynamics and
force calibration. The simulation uses a 49 × 3.5 mm slot in a 1.5 mm steel
plate. Use a tongue with an equivalent clear slot; record the donor belt and
slot dimensions. Do not widen its slot for the trial.

## Repeatable reset

Mark the two working-span endpoints and its 375 mm material midpoint. Add thin
blue/yellow edge marks that do not stiffen the webbing. With the ends attached,
turn the midpoint over by 180 degrees while leaving both endpoint edge colours
unchanged. Slide the tongue onto that reversed midpoint: two opposite half-
twists remain, one on each side of the slot. The blue edge starts on the
outboard side at both anchors and on the inboard side at the tongue.

Let the belt settle without robot contact. Match the photographed starting
layout and marked midpoint, with the tongue approximately at (365, 80, 855) mm.
A hand may support the tongue while preparing the reset, but remove that support
before the trial. The slot's own static friction retains the prepared snag; no
clip, tie, glue or concealed pin may hold the working span. Reject a reset that
falls flat by itself or slides out of the pictured start before the robot acts.
The simulator uses six seconds of passive settling with a reduced initial slot
stiction model. Its initial holding force has not been calibrated to a donor
belt; disclose any hardware mismatch instead of adding hidden support.

Use the same belt, extension and dimensions for each run. Open both hands and
keep the seat empty. No battery, powered seat mechanism or live restraint
system is needed. The tools for preparing/resetting the buck are removed before
recording. No material is consumed unless the webbing is damaged.

## Judging

Use front and outboard photographs showing both marked belt edges, the slot and
both hands. Measure width and roll in the belt's installation plane, rather than
against a horizontal tabletop. The whole working span must meet the task's
flatness requirement; a remaining twist outside the camera crop still fails.
Keep a three-second hands-clear hold in the trial video. Inspect the fixed
anchors and guide afterwards; detaching or damaging hardware invalidates credit.
