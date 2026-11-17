# Mechanical function and evidence

The two bars are retainers for the compliant cover. They limit its corner-opening
motion after the four hooks have wrapped the foot. The task does not require
sequential corner manipulation, a particular arm role or a camera strategy.
The full-score controller presses the cover over all four corners and then fits
the bars; that ordinary successful route is allowed.

An isolated native-physics comparison starts from the actual released full-score
assembly. It removes zero, one or both bars, then ramps the same0.015N*m opening
torque onto each corner. No objects are repositioned after initialisation, no
joint is actively driven, and no proximity latch is present. This is a separate
component test without robots or cameras, not a scored demonstration or a
measured material/load rating. Gravity is disabled in this torque fixture.

| Installed bars | Final corner opening angles (degrees) | Interpretation |
|---|---|---|
| 0 | 15.04, 14.99, 15.15, 15.10 | Four unrestrained corners open |
| 1 | 15.06, 15.01, 3.15, 3.13 | Only the bar-covered side is restrained |
| 2 | 2.00, 2.09, 2.12, 2.03 | All four corners are restrained |

All native hinges and slider joints pass the independent0.2mm/0.3deg numerical
integrity screen over the complete tests; forces/speeds remain below the
conservative anomaly screen. These integrity thresholds are not human grading
conditions. The comparison supports the bars' physical function; it does not
establish that every alternative assembly strategy needs two arms or that this
mechanism alone guarantees novelty under the benchmark's review.

The complete original-YAM scripted trajectory reaches1.0 in1,850 actions/92.5s.
The formal Dream run reaches0.0 after2,400 actions/120s. Both raw records were
replayed with the current surface-based scorer. Scene geometry, model instruction,
reset, cameras, controls and action sequence are unchanged. The full-score video
keeps every original camera frame and updates only the overlaid score.
