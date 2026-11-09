# Physical outcome rubric — outcome-v2

The scorer evaluates registration at the end of the episode. It does not award
or deduct points for a grasp identity, arm role, camera, viewing action, recovery,
or trajectory. Pre-observation and memory, a reversed grasp, handover, and
supported release/regrasp are all legitimate if the final physical outcome meets
the same conditions. Their mechanical feasibility must be verified separately.

Use the bottom-plane centre of each 30 × 10 mm window and the centre of each
illuminated cyan cross. The two identical crosses can exchange identities;
take the assignment with the smaller maximum planar error. Thus a face-up
180° rotation of the plate is eligible for the same score. An upside-down plate
is not face up and fails the tilt condition.

Over the complete final one-second window:

- **1.0:** both planar centre errors stay at or below 3 mm.
- **0.5:** both errors stay at or below 12 mm, but full-credit precision is absent.
- **0.0:** neither threshold is met, or any required condition below is absent.

Both nonzero scores also require all of the following over that same window:

- Each window's bottom centre is within 2 mm of the goal-board plane.
- Plate normal is within 2° of world up.
- No robot/plate contact occurs, and the goal board supports the plate at every
  240 Hz physics sample. Gripper opening alone does not establish release.
- Across 21 consecutive poses at 20 Hz, maximum pairwise plate-centre displacement
  is at most 1 mm and maximum pairwise orientation difference is at most 1°.

A transient alignment while grasped, unsupported, or moving earns zero. The
score is recomputed from the ending window rather than the best earlier pose.
Missing/invalid monitor data cannot earn a nonzero score in that window;
invalid object poses raise an error rather than being reported as task success.

Implementation revision outcome-v2 normalizes quaternions for the angular
envelope, includes both stated threshold boundaries despite floating-point
roundoff (numeric allowance1e-12 in SI units), and verifies12 physics steps per
control interval. The contact window includes the first pose's contact state
and the following20 complete intervals, exactly one second; it does not add
an extra preceding50 ms. Physical tolerances and score bands are unchanged.

The simulation uses rigid-body poses and contacts as measurement instruments.
A physical grading procedure would need calibrated images or simple gauges for
the 3 mm/12 mm alignment, seating and tilt thresholds, plus a visible release
and stillness interval. That procedure has not yet been validated with hardware.
Contact logs currently provide diagnostics, not a separately scored global
safety condition. No official review approval or real-hardware calibration is implied.
