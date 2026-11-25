# Real-unit checks before scoring trials (not yet run on a unit)

Use a complete XGO-lite V2 (ElecFreaks XGO Robot Kit V2) as supplied, with its arm folded on its back and the jaws
open, on the level laminate table. Put it in the maker's calibration mode from the XGO app so the servos produce no
torque; do not confirm the calibration or store zeros. Record the unit's firmware, its measured total mass, the
tabletop material and that the table and legs are clean and dry. Photograph the starting pose from all four
reference views before any handling. No added support, stop or geometric lock is allowed at any point.

## Backdrive check (gearbox hold)

Tools: a 0-10 N force gauge with 0.01 N resolution, a steel rule and a protractor.

1. Steady the body and the parent link by hand for this measurement only.
2. Push tangentially on the rigid link: the shoulder at the thigh shaft centre (about 50 mm from its axis), the
   thigh at the knee shaft centre (55 mm), the lower leg at the centre of the toe pad (68 mm). Measure the actual
   perpendicular lever for each sample; the lengths above are from the maker's model.
3. From rest, raise the force slowly until the joint has turned 5 degrees; record the peak force, the angle and the
   direction. Repeat five times in each direction for every joint of every leg. Stop any ramp at 8 N; a stopped
   ramp is not a measured threshold.
4. Torque = peak force x perpendicular lever. Also record the angle on release and 1 s later (spring-back).

The simulation uses 0.06 N m breakaway and 2 N m/rad stiffness, and the assays in fixture/passive-model-assays.json
use the same values. These are estimates. A measured hold below 0.05 N m or above 0.08 N m calls for a model revision
and new simulation runs; it does not by itself show the task is impossible.

## Start-pose check

Set every leg to the illustrated posture, then turn the front lower legs toe-up from it (front left 22, front right
26 degrees). Place the robot at the lot's position and heading, let go, and record 10 s from the side views. The
rear legs must stay in the posture and the front legs must stay on the heels under their knees, out of it. If the
front legs fold into the posture by themselves, the unit does not qualify.

## Hands-off retention check (the scored end state)

1. Set all four legs to the illustrated posture using the reference drawing, the four axis points per leg and
   square-on photos from the four views; each lower leg lies on the table from the heel to the toe pad.
2. Remove both hands and any other support. Record at least 10 s from front or rear, both sides and overhead, with
   enough resolution to read 0.5 degrees and 2 mm.
3. Report whether every leg stays complete (each angle within 7 degrees, thigh within 7 degrees of vertical), the
   body stays within 10 degrees of level with its lowest surface 15 mm or more above the table, and every link stays
   within 2 mm over a continuous 1 s.
4. Repeat ten times.

A unit that cannot stand in the posture without support does not qualify for scored trials. Keep the gauge
calibration record and raw video with any reported real-unit result. No simulated result substitutes for these
observations.
