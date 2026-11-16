# Setup and judging

Use a training manikin and coloured water, never a person or real medicine.
An operator loads exactly 5.0 ml with an oral dosing syringe before the trial.
The fixed spoon starts level in its rest, with its handle toward the robot.

## Objects and layout

Coordinates are metres in the YAM station frame: x away from the arm bases,
y left, z up. The table top is z = 0.7475 m. Both arms start in YAM ready pose.

| Item | Dimensions, material and start |
| --- | --- |
| Adaptive medicine spoon | Stainless scoop, elliptical paraboloid 48 × 32 mm at its rim, 9.5 mm deep; geometric capacity about 5.73 ml. Rim centre (0.230, −0.150, 0.8125). Level, long axis along x. Plastic padded handle 100 × 28 × 12 mm, centred 90 mm behind and 4 mm below the rim centre; 40 × 10 × 4 mm metal neck. |
| Test liquid | 5.0 ml water with food colouring, measured before the trial; room temperature. No drug is represented. |
| Spoon rest | Fixed plastic block 85 × 60 × 35 mm, centre (0.230, −0.150, 0.7855). |
| Care manikin head | Fixed, approximately 150 mm wide × 200 mm high × 140 mm deep, silicone skin over a rigid training shell. Mouth faces the robot. Lip-opening centre (0.390, 0, 0.8975); clear opening 44 × 20 mm, 45 mm oral depth, curved face and projecting nose. |
| Support and collector | Fixed neck support 70 × 75 × 70 mm. Mouth drain leads to a collector of known volume; its cutaway simulation display has a 30 × 20 mm cross-section and 1 ml graduations. In a physical trial recover the liquid with a graduated oral syringe. |

Replace spilled liquid, empty the collector, dry the scoop and face, and reload
5.0 ml before each reset. Use the same layout as the setup image. The operator
prepares the dose; the robot only transports and delivers it.

## Score at the time limit

First apply the invalidation rules. Otherwise award the highest matching row:

| Score | Collected dose |
| --- | --- |
| 1.00 | At least 4.5 ml delivered; less than 0.05 ml still airborne; spoon within 15 mm of its initial rest centre and released for at least 1 second. |
| 0.75 | At least 4.5 ml delivered, but the return/release condition is incomplete. |
| 0.50 | At least 3.0 ml delivered. |
| 0.25 | At least 1.5 ml delivered. |
| 0.00 | All other outcomes. |

Any of these makes the entire trial zero, regardless of collected dose:

- More than 0.25 ml spills outside the oral collector.
- The spoon hits the face outside the lip opening (2 mm geometric tolerance),
  or its leading edge enters more than 18 mm behind the lip plane.
- A robot link or gripper enters the head's conservative clearance envelope.
- The spoon tip moves faster than 0.12 m/s while within 50 mm of the mouth.
- The reduced model becomes invalid or fails conservation. It also rejects a
  captured spoon movement faster than 1.2 m/s, beyond its supported regime.

The collector, not proximity to the mouth, determines delivery. Pouring onto
cheeks, over the head or into its rear does not count. Partial credit measures
progress in this inert-liquid training task; it is not a clinical dosing standard.

## Simple physical checks

Use side/front video, a ruler beside the mouth and a graduated oral syringe.
Measure the recovered collector liquid to 0.1 ml; measure or weigh spillage on
a dry catch tray (0.1 g resolution is sufficient for these water trials).
Use the side video and ruler to check insertion, approach distance and movement
over known frame intervals. Face contact is visible; a light washable transfer
coating on the manikin can confirm ambiguous contact. The simulation additionally
checks native collision-shape bounds, so a hidden elbow contact is not ignored.

The simulator's head clearance box is relative to the mouth centre:
x = −27 to +120 mm, y = ±80 mm, z = −80 to +135 mm. This conservative envelope
applies to robot parts; the spoon has its own lip-opening and insertion checks.
These are explicit benchmark limits, not medically validated tolerances.
