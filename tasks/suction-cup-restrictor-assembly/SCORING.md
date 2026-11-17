# Cup and core capture through a plate

Grade actual installed mating geometry at the end. No nominal world-position or
angle-to-target threshold, microscopic drift limit, required arm assignment or
camera strategy is scored. Axial spin of a retained circular part is permitted.

| Score | Ending state |
|---|---|
| 0 | The captured-cup condition is false, or prohibited robot contact occurred. |
| 0.5 | The neck passes through the plate bore, the shoulder is on the entry side, and all four head sectors are behind the opposite rim. |
| 0.75 | That cup plus the restrictor shaft inside its bore, rib past the retaining lips and cap seated against the head. |
| 1.0 | Both stay installed, physically supported and free of all robot contact for three continuous seconds. |

Partial credit may be earned while holding an installed part. Full score requires
actual cup/head-to-plate and restrictor-to-cup/head support. A loss of capture,
support or release immediately revokes full credit and restarts the interval.
Ordinary settling inside the installed fit is allowed. Any robot-robot or
robot-table contact during the trial makes the episode zero. Intended contacts
with the parts, plate and staging stands are allowed.

## Simple-tool grading

1. Inspect both plate faces, using an inspection mirror after the trial if useful.
   The real neck must pass through the hole, with the plate captured between the
   cup shoulder and all four retaining head sectors. A compressed sector still
   inside the plate hole is not captured. No body-origin measurement is needed.
2. Inspect the white core from the cap and side openings. The shaft must be inside
   the real bore; the rib must have passed the inner retaining faces. All four
   sectors overlap its retaining face, rather than remaining spread around the rib.
   The cap must sit against the head with less than0.75mm axial gap; a0.75mm feeler
   provides a direct go/no-go check. The supplied drawing identifies both faces.
3. Record the released assembly retaining both parts for three continuous seconds.
   No hand or external tool may support them. Do not measure accumulated motion
   of a fraction of a millimetre or degree.
4. Apply the ending-state table. Any feasible grasp, bracing or one-handed route
   receives the same score. No vacuum, airflow, load or material-strength test is
   part of this mechanical installation task.

## Exact simulation counterpart

Dimensions below come from the mating geometry, not arbitrary nominal poses.
All geometry calculations use metres; this description uses millimetres.

- Transform the actual cup shoulder ring(radius11,Y=21.4 in cup coordinates).
  Its entire end face must remain on the entry side of the plate(Y=-2 in the
  plate frame). Transform both outer retaining corners of each head sector at
  localY=26: all must be beyond the plate's rear face(Y=+2), outside its9mm radius.
- Check the neck's32-sided8.3mm outer-radius surface over cupY14..24 against the
  actual48-sided hole, including its20-to-18mm entrance chamfer. Clip the neck
  edges at plateY=-2,-1,+2; test the endpoint half-spaces in each convex region.
  This prevents a tilted or offset neck intersecting the plate while its axis
  appears centred.
- In cup coordinates, the core cap's lower face(localY=-0.9) has axial gap
  0..0.75 from the head tips atY=30. Its rib's rear face(radius5.3,localY=-5.5)
  must have passedY=26. Each actual inner head lip must overlap that retaining
  face. The complete8.8mm shaft must fit the9.2mm neck bore at both ends,
  coupling tilt and eccentricity rather than checking those independently.
- A0.05mm contact allowance is used only for simulator surfaces meeting. It is
  not a physical measurement tolerance and does not permit real interpenetration.
  The point of the guard is that actual solid parts fit and capture each other.
- Full credit requires installation, actual support and no robot-to-part contact
  at all2,881 native960Hz samples spanning three seconds.

Native locked-axis error, joint travel, missing/nonfinite data, contact coverage
and image health are simulation-validity checks in validate_run.py. They are
separate from the primary outcome. Its200N/2m/s anomaly screens are not calibrated
material failure or force limits. This rubric certifies neither vacuum sealing
nor flow through the0.8mm bore.
