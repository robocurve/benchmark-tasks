# Real-unit checks before scoring trials (not yet run on a unit)

Use the complete original XGO-mini2S with its original toes and its arm and head parked, on the level laminate
table. Put it in the maker's calibration mode so the servos produce no torque; do not confirm the calibration or
store zeros. Record the unit's model and firmware, its measured total mass, the tabletop material and that the
table and toes are clean and dry. Photograph the starting pose from all four reference views before any handling.
No added support, stop or geometric lock is allowed at any point.

## Backdrive check (gearbox hold)

Tools: a 0-20 N force gauge with 0.01 N resolution, a steel rule and a protractor.

1. Steady the body and the parent link by hand for this measurement only.
2. Push tangentially on the rigid link: the shoulder at the thigh shaft centre (about 50 mm from its axis), the
   thigh at the knee shaft centre (about 60 mm), the lower leg at the centre of the round toe (about 75 mm).
   Measure the actual perpendicular lever for each sample; the lengths above are guides from the maker's model.
3. From rest, raise the force slowly until the joint has turned 5 degrees; record the peak force, the angle and the
   direction. Repeat five times in each direction for every joint of every leg. Stop any ramp at 15 N; a stopped
   ramp is not a measured threshold.
4. Torque = peak force x perpendicular lever. Also record the angle on release and 1 s later (spring-back).

The simulation uses 0.12 N m breakaway and 4 N m/rad stiffness. These are estimates; measured values between
0.06 and 0.20 N m and between 2 and 8 N m/rad are inside the range the model was checked over. Values outside it
call for a model revision and new simulation runs; they do not by themselves show the task is impossible.

## Hands-off retention check (the scored end state)

1. Set all four legs to the illustrated posture using the reference drawing, the four axis points per leg and
   square-on photos from the four views.
2. All four toes must touch the table: a 0.1 mm feeler leaf (or 80 g/m2 paper) must not slide under any toe.
3. Remove both hands and any other support. Record at least 10 s from front or rear, both sides and overhead, with
   enough resolution to read 0.5 degrees and 2 mm.
4. Report whether every leg stays complete (each angle within 10 degrees, thigh within 10 degrees of vertical), the
   body stays within 10 degrees of level with its lowest surface 15 mm or more above the table, all toes stay down
   and every link stays within 2 mm and 0.5 degrees over a continuous 1 s.
5. Repeat ten times from the normal reset of each lot.

A unit that cannot stand in the posture without support does not qualify for scored trials.

## Reset check

Set the shoulders and thighs to the illustrated posture and each lower leg 21 degrees toe-down from it, place the
robot at the lot's position and heading, let it settle and photograph it: no leg may be complete. Keep the gauge
calibration record and raw video with any reported real-unit result. No simulated result substitutes for these
observations.
