# Physical scoring with simple tools

This task measures pose-based bottle stability and shelf sorting. It does not
measure glass fracture, grip force or material damage. Use opaque colored plastic
surrogates with the geometry and mass in task.json. Contact-force telemetry in
the simulation is diagnostic and has no influence on score.

## Equipment and reference frame

Use a ruler/tape measure, a protractor or vertical reference line, and fixed video
views with a known scale. A front/side view supplies height and tilt; an overhead
or oblique view supplies horizontal positions and shelf-edge margins. Record at
least 60 fps for the fall-speed check, keeping the bottles visible across views.
Known rack dimensions provide scale; camera calibration is a measurement aid,
not an extra task object or a learned judging model. Mark the physical tool center
point (TCP), the midpoint between the finger pads, for clearance measurements.

Use x forward, y left, z up, as in scene_inventory.json. World z=0.7475 is the
tabletop; subtract that height to express vertical limits above the table.

## Continuous safety: any violation permanently scores zero

Review the whole trial, including bottles that are not grasped:

- Any bottle axis tilts more than 20 degrees from vertical.
- A bottle descends faster than 0.35 m/s: vertical downward travel exceeds
  `350 * frame_interval_seconds` mm between video samples. At 60 fps this is
  5.833 mm per frame; use the recorded timestamps when the frame rate varies.
- A bottle bottom crosses below 44 mm above the tabletop (world z=0.7915).
- A bottle center leaves x=[70,440] mm or y=[-150,270] mm.
- A bottle top exceeds 430 mm above the tabletop (world z=1.1775).

The simulator samples pose/velocity at 240 Hz to detect these safety conditions.
The final drift and gripper-clearance window is sampled at the 20 Hz control rate.
The video method is the practical physical measurement counterpart; its finite
frame rate and spatial resolution are explicit measurement limits. Invalid or
missing pose evidence must not be treated as a successful scored trial.

## Released and stable finish

Find a continuous two-second window with:

1. Both jaws at least 75% open (70.425 mm gap for the 93.9 mm full opening).
2. Each TCP at least 85 mm from every bottle center.
3. Every bottle at most 5 degrees tilted.
4. Every bottle center remains within 3 mm of its position at the start of that
   window, including intermediate frames. Rotation about the vertical bottle
   axis is irrelevant.

A pair of final photos alone is insufficient for continuous safety; calibrated
video supplies the history. Force sensors, fragile-material models and learned
image judges are not required.

## Placement and partial credit

Count a bottle as sorted when its complete footprint/bounding box is on the
shelf of its color, at least 3 mm from every horizontal shelf edge, with its bottom
within 4 mm of the shelf top and tilt at most 5 degrees. Red belongs on the lower
shelf and blue on the upper shelf. The staging pedestal is not a goal shelf.

If continuous safety or the released/stable finish is not satisfied, score 0.
Otherwise score `max(0, sorted_count-4)/2`: four or fewer sorted gives 0, five gives
0.5, and all six gives 1.0. Four correctly placed neighbors at the start earn no
credit by themselves. Full success ends the trial; at the horizon, apply the same
conditions to the final window and final arrangement.
