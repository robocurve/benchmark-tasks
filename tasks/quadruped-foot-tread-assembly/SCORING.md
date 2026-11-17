# Installed tread and retaining bars

Grade the ending state using actual mating surfaces. No nominal world-position,
angle-to-target, spring-coordinate or microscopic drift limit is part of the
primary outcome. Any arm, grasp, order, camera strategy, equivalent half-turn
cover or exchanged/half-turned bars is allowed.

| Score | Ending state |
|---|---|
| 0 | The supported, wrapped-cover condition is false, or prohibited robot contact occurred. |
| 0.25 | The cover encloses the foot, with all four hooks under their matching ledges and actual support from the foot. |
| 0.5 | That cover plus one mechanically retained bar. |
| 0.75 | That cover plus two mechanically retained bars on opposite sides. |
| 1.0 | All installation and support conditions persist, with every part free of robot contact, for three continuous seconds. |

Partial credit may be earned while holding a part. Full credit is revoked as
soon as installation, support or release is lost. Ordinary motion within the
installed fit does not fail. Any robot-robot or robot-table contact during the
trial makes the episode zero. Intended part/fixture contacts are allowed.

## Simple-tool grading

1. Record the ending assembly for three uninterrupted seconds after fingers are
   clear. The foot must support the cover and both bars; no hand or external tool
   may carry them. A release violation restarts the interval.
2. Inspect the sole/foot fit and four exposed corner interfaces from the sides.
   The guide skirts must enclose the foot and the sole must be seated on it.
   Each actual hook edge must extend beneath its ear by at least1mm of edge length.
   Use a ruler in a side photograph or a caliper at an exposed edge.
3. Inspect both ends of each bar. Each actual retaining edge must lie behind its
   ear with at least1mm vertical overlap. A clip beside an ear is not retained.
   Check opposite sides, regardless of the part names or colour.
4. Apply the table to the ending state. No pull test or microscopic accumulated
   motion measurement is required. The separate opening-load comparison explains
   what the bars do; it is not an extra policy step or scoring condition.

`fabrication/dimensions-and-assembly.pdf` identifies the mating edges. A ruler,
caliper, side photographs and the trial video suffice; there is no need to
reconstruct hidden body origins or measure a simulator-only joint error.

## Exact simulation counterpart

Coordinates below are the actual mesh dimensions, in millimetres in the fixed
foot frame. They define mating regions, not additional human pose tolerances.

- Transform each hook edge from its body: X=sign*25.5, Y=cornerY-7..cornerY+7,
  Z=-14.5. After assigning its actual side/end, at least1mm must lie in the ear's
  underside capture region:25<|X|<27,12<|Y|<19,-20.5<Z<-14.45. The upper0.05mm
  is solely numerical contact allowance at the actual Z=-14.5 ear underside.
- The fixed core top corners(X=+/-25,Y=+/-16,Z=-2.5) must fit within the cover's
  actual guide/sole surfaces in the cover frame:|X|<=25.3,|Y|<=16.3,Z<=-2.5,
  with0.05mm numerical contact allowance. This prevents a nominally wrapped but
  intersecting cover; it is checked physically by the real parts fitting together.
- For each bar-end retaining edge, transform X=-sourceSign*9.5,Y=endSign*18,
  Z=-6..+6 from its actual child body. At least1mm of edge must lie in
  20<|X|<25,12<|Y|<19,-14.5<Z<-2.5. Segment clipping checks the edge itself.
- The cover must contact the fixed foot. Each bar must have support from the
  foot or the already seated cover. Both bars must occupy opposite sides.
- Full credit requires all conditions and zero robot-to-part contacts at every
  physics sample over three seconds:1,441 samples at480Hz. A single lost-support
  or recontact sample resets the full interval.

Pose errors, spring coordinates and drift remain diagnostics only. Missing data,
nonfinite state, broken native constraints, incomplete contacts or failed rendering
invalidate simulation evidence separately; they are not physical grading readings.
The validator's200N/2m/s anomaly screens are numerical diagnostics, not material
failure thresholds, robot load ratings or scored force limits.
