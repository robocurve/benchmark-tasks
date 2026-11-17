# Fabricating the custom foot service specimen

All STL dimensions are millimetres, in the part-local coordinates shown in
`dimensions-and-assembly.pdf`. Keep these datums during assembly; translate parts
onto the printer bed without scaling. `parts.json` gives material, quantity,
bounds and volume. Every delivered STL reloads as one closed solid. This is a
custom bench specimen, not a Spot spare, a powered quadruped, or a walking foot.

## Parts and workshop route

| Item | Quantity | Route |
|---|---:|---|
| tread-cover.stl | 1 | Print flexible TPU, nominal Shore A95, or cast polyurethane from the same master. Keep the sole, guide skirts and all four hooks in one piece. |
| left-bar.stl / right-bar.stl | 1 each | Rigid PA12 printing or polymer machining; do not print these in the cover material. |
| left/right-clip-1/2.stl | 1 each | Rigid PA12; separate sliding parts, not glued to their bars. |
| foot-and-stand.stl | 1 | Rigid printed service fixture. Secure its base to the table; it carries only this specimen. |
| Straight steel guide/anchor pins | 12 | Diameter1.0mm; cut5mm long, deburr, retain at the bar/clip outer faces. Four fixed spring anchors, four moving anchors, four travel-stop pins. |
| Miniature extension springs | 4 | Nominal0.50N/mm, low initial tension, outside diameter<=1.6mm; hook-centre span5mm neutral and8mm extended. |
| Staging blocks / cradle posts | As PHYSICAL_SETUP.md | Cut rectangular rigid stock to task.py dimensions; secure at the listed coordinates. |

An example spring sizing starting point is0.17mm steel wire,1.2mm mean coil
diameter and10 active turns. The ordinary helical-spring estimate is about0.48N/mm
for an assumed79GPa shear modulus; this is a design calculation, not a measured
spring specification. Buy/order a spring to the force/travel envelope above and
check it. Do not assume an arbitrary rubber band or stiff printed beam is equivalent.
No complete robot leg, OEM tooling or powered fixture is required.

## Assemble each retaining bar

1. Clear the printed guide and deburr the1.1mm cross holes. The bar has open-ended
   internal guides; each clip carries a3.2x4mm tongue. Nominal clearance is0.2mm
   per guide face. All four noses retain the original mating dimensions.
2. Insert the matching clip from the end of the bar. End1 corresponds to local
   negativeY and end2 to positiveY. Parts are shown at their neutral assembly
   coordinates in the drawing; do not centre their geometry before assembly.
3. Install a fixed1mm spring-anchor pin at |Y|=14mm and a moving anchor at
   |Y|=19mm, both at Z=-0.9mm. The spring passes through the0.8mm-radius axial recess in the tongue.
   Connect its hooks to these pins. Pulling the end outward extends the spring.
4. Install the1mm captive travel pin at |Y|=20mm, Z=+1.0mm, through the bar and the clip's
   1.2mm-wide oblong slot. The slot permits approximately3mm outward travel and
   stops the clip escaping. Its higher Z keeps it clear of the moving spring anchor. Retain pins at their ends without bonding the slider.
5. Move each clip through its travel by hand. It must return to the inward stop
   without binding. Check the two clips independently. Glue may retain a pin only;
   it must not reach the spring, guide, moving tongue or retaining face.

The CAD adds real guide channels, pins, spring pockets and a short nose-to-arm
bridge. The simulation represents these internal connections by native passive
joints and compound rigid bodies, not individually meshed internal hardware.
The exposed hook, nose, bar-land and foot interfaces retain the recorded dimensions.

## Cover and elastic qualification

The printed/cast cover is a continuous compliant part. Its thin side walls bend;
the simulation approximates that bending with four passive hinges and a rigid
sole. Material grade and print orientation affect response. Keep supports off the
hook/ledge mating faces; remove sacrificial material and reject cracks or bonded
corners. Use solid perimeters at the roots rather than sparse infill there.

Before a workshop trial, use a simple force gauge or known weights to check
return and capture. For a comparison with the simulation fixture, a1.5N load
acting approximately10mm from a corner's bend line gives0.015N*m. Without bars
corners should open and return; with both bars they should remain behind the
ledges. The digital comparison gives approximately15deg versus2deg; those are
simulation results, not measured TPU performance. Adjust material/process if a
specimen is brittle, permanently set or too stiff. Do not change the mating
surfaces to make a failed specimen fit. The benchmark ends at installation;
no service load or walking lifetime is certified.

## Setup, reset and grading

Use the fixture and staging dimensions in PHYSICAL_SETUP.md/task.py. The world
foot datum is(0.24,0,0.8915)m, rotated90deg aboutZ. Mount it rigidly before reset.
Place the loose cover and both assembled bars on their specified stands, all
clips at their inward stops. Check that every hook, spring and captive pin is
intact. Replace damaged specimens. Grade exposed seating/retaining interfaces
and a three-second released hold using SCORING.md; no micro-motion measurement
or force test is required during a scored trial.
