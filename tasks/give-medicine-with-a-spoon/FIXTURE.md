# Build the training head and collector

This is an inert, life-size spoon-delivery fixture, not a patient or a medical
device. Its mouth, face and nose reproduce the geometry in the submitted run.
No commercial care manikin is required. The supplied parts and instructions
are a buildable design; they have not yet been printed or physically tested.
Complete the leak and recovery checks below before a real robot trial.

## Parts

All STL coordinates are **millimetres**, with the mouth centre at (0, 0, 0).
+x points into the head, +y to its left and +z up. Print at 100% scale.

| Part | Specification |
| --- | --- |
| Face and rear cover | `fixture/face_shell.stl` and `fixture/rear_cover.stl`; PETG, 0.2 mm layers, at least four perimeters. The face is 150 mm wide and 200 mm high. Print separately with supports as needed; remove supports and smooth edges. |
| Nose | `fixture/nose.stl`; PETG. Centre (−8, 0, 36), dimensions 36 × 24 × 50 mm. Its front projects 26 mm beyond the mouth plane. |
| Mouth and drain | `fixture/oral_drain.stl`; PETG, solid walls. Clear mouth 44 × 20 mm; the first 18 mm is straight. Behind this, the channel turns down within approximately 45 mm of the face. Outlet: 6 mm bore, 10 mm outside diameter, last 20 mm straight. |
| Lip | Approximately 120 mm of **4 mm diameter solid silicone cord**, trimmed to fit. Form it around the drain's outer 48 × 24 mm elliptical edge; butt and seal the ends. The finished clear opening is 44 × 20 mm, not 48 × 24 mm. |
| Seal and join | Fully cured silicone sealant for the wet channel/lip; epoxy or small external PETG straps for dry head joints. Follow the adhesive's cure time. No uncured adhesive is exposed to the robot. |
| Drain tube | Clear silicone, 10 mm inside / 14 mm outside diameter, about 250 mm long. Push 15 mm over the outlet; secure externally with a small cable tie without constricting the bore. Route continuously downhill, without a kink or water trap. |
| Collector | A clear, screw-lidded 30–60 ml polypropylene sample pot. Drill one 14 mm tube hole and a 2 mm vent in its lid. Seal around the tube; its end sits above the collected liquid. Cover the vent with a loose downward-facing hood so a spill cannot enter it. |
| Stand | A 200 × 200 × 10 mm plywood base and a 70 × 75 × 70 mm wooden neck block. Cut a 55 mm wide U-channel down from its top, leaving two 10 mm side walls and a 10 mm bottom. This gives access to the drain. Drill a 20 mm side exit for the hose about 55 mm above the block bottom. |
| Positioning | Two small table clamps, non-slip pads, and cured epoxy putty or shims to cradle the head. A ruler sets the mouth height and a square sets its plane. |
| Liquid and measuring tools | Room-temperature water (20–25 °C), food colouring, a 5 ml oral dosing syringe marked every 0.1 ml, and a scale reading 0.01 g. |
| Spill protection | A washable **520 × 420 × 10 mm** shallow plastic tray, impermeable liner, removable absorbent sheets and separate dry gripper-pad covers no thicker than 0.2 mm per face. |

Budget allowance: roughly **US$40–80 for fixture materials**, excluding a printer,
robot and measuring tools. This is an estimate, not a supplier quotation.
The four print solids total about 226 cm³ before slicer supports. All meshes
and their generator are original to this task and covered by its LICENSE.

## Assembly and layout

1. Fit the drain's 48 × 24 mm outside rim into the face opening from behind.
   Keep its opening at x = 0; seal the joint externally. Fit the silicone lip
   around that rim without narrowing the **44 × 20 mm** clear aperture. Check
   both dimensions with a ruler/caliper. Seal and smooth the wetted print
   surfaces; do not leave loose resin or support fragments in the channel.
2. Before closing the head, make an **18 mm diameter vertical service opening**
   in its underside, centred at x = 40, y = 0. Cut only the bottom shell, up to
   z = −55 mm, so the outlet/tube can pass without meeting the face shell.
   This opening surrounds the sealed drain, not its wet interior.
   Bond the nose at its specified coordinates. Mark the eyes as in the setup
   image; they are visual features, not holes. Attach the rear cover at the
   face's rear rim (x = 45 mm). Keep the drain and tubing accessible from below.
   The rear cover adds no route into the collector.
3. Screw the neck block to the base. Run the hose through its U-channel and
   side exit. Bed the head on the side walls using cured putty/shims, with the
   mouth plane vertical and the mouth centre **150 mm above the table**. A
   thin sealing strip around a dry head joint is acceptable; do not enlarge
   the mouth or add a funnel in front of it. Clamp the base to the table.
4. In the YAM frame, put the mouth at **(390, 0, 897.5) mm**; the table is at
   z = 747.5 mm. It faces −x. Keep the neck within its nominal 70 × 75 mm
   footprint behind the mouth. Put the collector beside the neck, outside
   the approach, with the hose descending into it. Its exact downstream
   shape/location is immaterial to the task; only liquid entering through
   the mouth can reach it.
5. Place the spill tray centrally beneath the spoon route and head, long side
   along x, centre approximately (325, 0) mm. Keep its rim below the approach.
   Set the fixed spoon rest and head to their **absolute JUDGING.md coordinates**
   after allowing for the tray/liner thickness. Neither tray nor collector is
   a target or a support for the robot. The drain collector is sealed separately
   from this tray so outside spills never count as delivered liquid.
6. Cover the table under and around the tray with an impermeable liner. Fit
   dry removable covers to the gripper pads, and protect nearby fixed cables
   from drips. Keep joints, sensors and ventilation unobstructed. The robot
   grasps only the dry handle. Stop the physical trial if liquid reaches an
   exposed robot part; clean/dry it before another trial.

Use `fixture/assembly.svg` for the axes, principal dimensions and drain layout.
The stand is a fixture: clamp it securely and check it cannot shift under a
light manual push. It must not move the head or spoon for the robot.

## Commissioning and reset

Before first use, and after any plumbing change:

- Measure the mouth aperture and its 150 mm table height. Check there is no
  lip, sealant bead or support inside the first 18 mm of the oral opening.
- With the robot clear, slowly deliver **5.0 ml** of coloured water through
  the mouth using the dosing syringe. After 10 seconds, recover at least
  **4.9 ml** from the collector. There must be no coloured water in the spill
  tray or a dry tissue wrapped around any external joint. Repeat three times.
  If this fails, smooth/seal the channel or remove a hose kink and repeat;
  do not compensate by changing the task's scoring thresholds.
- With the mouth protected by a temporary plug, place **1.0 ml** on a cheek.
  It must reach the spill tray, with **zero increase in collector mass** at
  0.01 g resolution. Remove the plug before a robot trial.

For each trial, empty and dry the collector, channel, spoon, face and tray.
Replace absorbent sheets and tare their collection bag with the dry tray.
Check the tube is downhill, covers are dry and the fixtures have not moved.
Fill the level spoon with exactly 5.0 ml; use the same food-colouring dilution
for every trial (one drop per 100 ml water is sufficient). Start recording both
views before the robot moves. Reset preparation takes a few minutes once the
fixture is built. No actual medication or human recipient is used.

After a trial, use the oral syringe to read collected liquid to 0.1 ml. Weigh
the spill tray and all wetted exterior absorbent sheets together; for these
room-temperature water trials, **1 g is approximately 1 ml**. Include exterior
face/stand drips by blotting them with the pre-weighed sheets. Keep material
from the inside of the oral channel out of the spill measurement. Judge the
unchanged contact, insertion, speed and return criteria from the two videos.

## What the existing simulation establishes

The simulation keeps the same life-size aperture, nose/face clearance, allowed
insertion, spoon, layout and liquid-delivery criterion. The printed drain and
real collector specify how to implement the previously described mouth drain;
their downstream flow and wetting are not calibrated by the simulation.
The physical shell is rigid PETG with a silicone lip, not living tissue.
The tray, protective covers and internal plumbing are real-trial provisions,
not newly simulated objects. The existing videos remain evidence of robot
motion and the stated reduced liquid model, not of a successful physical build.

To regenerate the printable files: `python fixture/make_parts.py` from this
task folder, with NumPy and trimesh installed. `fixture/mesh-checks.json` records
closed-mesh and dimension checks; these do not replace the commissioning checks.
