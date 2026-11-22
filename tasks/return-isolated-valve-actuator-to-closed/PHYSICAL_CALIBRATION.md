# Valve station fabrication and calibration

The digital station represents the stored-energy decision during restoration of an isolated process valve. Festo's single-acting DFPD uses preloaded springs to rotate its output when the chambers exhaust. The Bettis M-Series manual identifies overrides and accessories that can keep springs cocked. These are the machine class and service context. The orange maintenance pin, backdrivable crank and compliant overtravel are features of this benchmark fixture. The 45° mark is an intermediate service position on a quarter-turn device. The work instruction assumes an already isolated process line.

The identifiable hazard is unexpected movement at the work location. OSHA 1910.147(d)(5) covers control of residual or stored energy, and paragraph (d)(6) addresses verification of isolation. The trainer pin has no lockout certification or authority to replace a plant procedure. ISO 12100 supplies the risk-assessment context. No simulation metric in this task predicts injury to a person.

The Fisher 1080 manual describes resistance at disengagement as evidence of a force imbalance between the manual and powered sides of an actuator. This provides an industrial example of transferring load before releasing a restraint. The simulation gives the robot a separate pin and exposed crank to make the handover observable. Its geometry and the maneuver beside the mannequin are specific to the simulated fixture. [Fisher 1080 manual actuator, manual-to-automatic operation](https://www.emerson.com/is/content/emerson/en/final-control/flow-controls/documents/d500236x012.pdf)

## Coordinates and setup

All measurements below are in mm. Look down on the fixture, with +X away from the robot bases, +Y towards the left arm and +Z up. Place the shaft axis at world (285, 0, 747.5). The shaft axis projected on the tabletop is the fabrication datum. The physical plate top replaces the simulated tabletop datum. Shim the station or raise the arm mounts to keep their height relative to that datum.

| Feature | Datum or dimensions |
|---|---|
| YAM base positions | World (0, +305, 750) and (0, −305, 750), original YAM axes. |
| Spool and actuator | Centered painted housing with two end caps. Flanged mock pipe runs along Y at X = 51, center height 28, 42 OD. Flanges 69 OD × 12. |
| Output and restraint | 150 mm indicator radius in the 85 mm plane. Its service index is +45° and closed index is 0°. Pin axis (−108, +10.9). Shank bottom 47, upper grip center 129. Lower guide top 71, upper collar spans 97–105. |
| Override | Blue two-ended crank centered on the output shaft. At reset the knobs are at (+10, ±151, 128), 18 OD × 41. |
| Tray | Center (−168, 0), 100 × 115, floor at 12. A center socket with 38 mm inside diameter, 50 mm outside diameter and 53 mm height above its floor supports the parked pin. |
| Wrist | Glove fingertip centered at (+145, −27, 85), 24 × 15 × 16. Cuff, rim of glove and yellow fingertips appear in the overhead view. Wrist slides in −Y for 23 mm with a flag and index. |
| Overtravel memory | Peg at (+107, −10, 85), passive slider along −Y with 38 mm stroke. Read final displacement with a millimetre ruler. Momentum and drag affect the final peg position. The trace independently records the true minimum shaft angle. |
| Technician | Adult-size inert standing surrogate across the bench. Shoulder near (+470, −245, 490), elbow (+380, −230, 225), sleeve end (+205, −60, 94). Legs stand behind the far table edge at world X = 910. Boot soles sit at world Z = −12.5, hard hat above the vest, gauge from the gloved hand to the adjacent flange. |

## Bill of materials for the unloaded physical mock-up

The included STL and DXF files support bench fabrication and fit checks. Each of the twelve exported meshes passed a closed-edge, winding and single-component check. The OpenSCAD source regenerated the same vertex sets and drilling drawing. Print tolerances and assembled movement still require bench verification. The spring torque and dynamic contact experiments are implemented in the supplied simulator. The physical mock-up has no compressed-air port or energized return mechanism.

| Qty | Part or ordering specification | Fit or use |
|---:|---|---|
| 2 | 80/20 1515 T-slot extrusion, 490 mm cut length | Base rails parallel to X, centered at Y ±155. |
| 6 each | 80/20 13040 roll-in nut, 5/16-18 × 5/8 in button screw and flat washer | Secure the plate to the rails. Select screw engagement against actual plate and washer stack. |
| 1 | 490 × 410 × 6 mm 6061-T6 plate | Through drill the full-size `plate.dxf`. Use clamps to immobilize the base. |
| 2 | SKF 608-2Z, 8 × 22 × 7 | Shielded bearings in printed housing. Printed pocket diameter is 22.15. |
| 1 each | Ground 8 mm h6 shaft, cut to 120 mm, two 8 mm split shaft collars, 8 mm bore spacers/shims | Seat the bearings, set elevation and retain the shaft. Dress the cut ends and verify free rotation. |
| 1 | ISO 2341 / DIN 1444-B steel clevis pin, 10 × 80 mm under head, 18 mm head | The orange two-piece cap captures the pin head. Confirm the delivered head thickness fits the 5 mm pocket. |
| 1 each | PETG housing, indicator lever, crank, two knobs, two identical pin-handle halves, guide, tray, two witness rails, wrist flag, pointer, two pedestals | Files in `fabrication/stl/` use mm. Print one of each except the quantities specified. The two handle halves mate after rotating one 180° around the vertical axis. |
| 2 each | M5 × 55 socket screws, washers and locknuts | Through each knob and crank. Use a head recessed in the grip and check the remaining nut engagement. |
| 2 + 2 | M4 × 8 cup-point grub screws, M2.5 × 25 socket screws with nuts | Tap the 3.3 mm radial holes for the hub screws. M2.5 bolts clamp the orange cap. |
| 4 + 2 | M5 × 20 bolts/nuts/washers for the housing, M4 × 18 bolts/nuts/washers for the guide | Plate mounting locations are in the cutting file. |
| 4 | M3 threaded rod cut to suit the two pedestal stacks, M3 washers and locknuts | Pass through plate, support and witness rail. Trim protrusion clear of the slide. |
| 1 set | Removable transfer tape, contrasting red/white/green tape, millimetre scale, printed 0°/45° decal, PETG and shim stock | Secure tray, finish the indicators and fit the guides. |
| 1 set | Unpressurized mock pipe spool, inert adult-size mannequin, high-visibility vest, hard hat, foam-backed yellow glove, blunt 6 mm alignment gauge | The coordinates above and frozen USD define the visual layout. Attach the glove to the wrist carriage and support the forearm independently. |
| Instruments | Calipers, square, angle gauge, ruler, known mass or calibrated force gauge, camera on a fixed stand | Record instrument ID, resolution and date. No physical acceptance readings have been collected. |

## Assembly and inspection

1. Print the supplied models in mm. With a 0.4 mm nozzle, 0.2 mm layers and four walls are starting settings for the unloaded parts. Print a bore coupon and check the actual fit first. Remove burrs from pin guides and the witness rails.
2. Drill the base from the DXF, deburr and secure it to both 1515 rails. Fit both bearings and shaft. Add spacer washers until the lever center is at Z = 85 and the crank spine is at Z = 101. Align both at the service mark, tighten the two hub screws against shaft flats, and install the retaining collars. The shaft should turn freely by hand.
3. Bolt the guide to the base. Assemble the orange cap around the clevis head. In the seated position the pin bottom is at Z = 47. Check that a 60 mm vertical lift clears the upper guide before lateral transfer. Confirm that the crank turns with the pin withdrawn, the pin seats freely, and the empty tray accepts the pin without support from a gripper.
4. Mount the sliding gauges and mark their zero points. Add a light keeper strip above each printed rail to retain its carriage with about 0.3 mm running clearance. Set the wrist carriage at the +Y end and adjust the witness index to its zero. Move the carriage manually by 10 and 20 mm and compare the optical readout with calipers. Place the separate pointer peg just beyond the closed index.
5. Fit the mannequin to the far side, support its forearm, and attach the soft glove and alignment gauge. Hand-turn the unloaded indicator from 45° to 0°. Confirm at least 5 mm net clearance from the glove throughout this nominal arc. Photograph the closed mark and both witness zeros from the fixed overhead position. Repeat the check after reassembly.

## Static torque and optical calibration

The digital angular spring is 3.0 N·m at 45° with an unforced target at the closed index. Its ideal linear-spring energy at that angle is 1.18 J. A perpendicular static force at a measured 0.150 m radius would be 20.0 N, equivalent to 2.04 kg under standard gravity. A static hold measurement cannot supply a peak impact force. Actual contact force depends on inertia, contact stiffness, damping and the solver step.

For an **enclosed instrumented torque standard**, verify radius, zero the calibrated force gauge, restrain the output at its index, and apply a slow tangential load at the calibration arm. Record force and perpendicular distance on three independent repeats. Compute each moment as force × perpendicular distance. For a reference nominal of 3.0 N·m, the specified comparison interval is 2.8–3.5 N·m. Use the calibration of the reference instrument for any hardware claim. The unloaded printed fixture supports geometry and visibility tests only.

The simulation logs lever–wrist contact at every physics step. It changes the visible indicator from green to red at a measured 5 N contact and keeps it red. The passive wrist slide, pointer displacement, and 1 N scoring threshold are separate measurements. On the physical mock-up, check the flag and scales using controlled manual displacement. A 5 N force calibration on a physical instrumented witness and correlation of bench readings to the simulated force trace remain outstanding.

## What the recordings and bench measurements establish

The overhead recording shows the position of the glove relative to the flange, the index on the shaft and the two slides. Compare the initial and final marks to check residual wrist and pointer movement. The wrist can settle back slightly after peak travel. In the native loaded-pull trial the wrist peaked at 23.12 mm and finished at 22.65 mm. The sliding pointer finished at 30.59 mm. The pointer is a travel witness with its own geometry and drag.

The two displacement scales, pin height and open jaws admit direct optical checks. The full simulated score also uses contact loads, a 50 ms load-transfer interval and substep angular velocity. A final photograph or the red 5 N flag alone cannot establish the 0.5 N unloading or 1 N contact thresholds. The released bench design has no calibrated dynamic load measurement. A corresponding instrumented physical score has not been demonstrated.

## Evidence record

For a physical trial record fixture revision, measured arm radius, service and closed angles, pin shank/guide clearance, both tray dimensions, open gripper widths, wrist zero/final, pointer zero/final, camera position, instrument IDs and operator. Keep three-view photographs of reset, hands-free closure and the two readouts. A force measurement requires its original time series and sample rate.

## Sources

- Festo. DFPD quarter-turn actuator, single-acting operating principle and mounting interface. [Product documentation](https://www.festo.com/media/catalog/204177_documentation.pdf)
- Emerson. Bettis M-Series, spring condition and override accessories, Section 6. [Installation, operation and maintenance manual](https://www.emerson.com/is/content/emerson/en/final-control/actuation/documents/manual-bettis-m-series-stainless-steel.pdf)
- OSHA. Hazardous-energy control, paragraphs (d)(5)–(6). [29 CFR 1910.147](https://www.osha.gov/laws-regs/regulations/standardnumber/1910/1910.147)
- ISO. Machinery risk assessment and reduction. [ISO 12100:2010](https://www.iso.org/standard/51528.html)
- Dimensional references. [80/20 profile 1515](https://8020.net/1515.html) · [80/20 nut 13040](https://www.digikey.com/en/products/detail/80-20-llc/13040/21783407) · [SKF 608-2Z](https://www.skf.com/id/products/rolling-bearings/ball-bearings/deep-groove-ball-bearings/productid-608-2Z)
