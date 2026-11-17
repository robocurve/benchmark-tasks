# Physical grading with a ruler, square and short recording

The outcome is a usable opening that remains after the robot lets go. No grader
needs to reconstruct hidden panel angles or measure 1 mm changes between frames.
The policy keeps its original three 320 × 176 cameras; independent grading
photographs are not extra robot observations.

## Tools and views

Use a millimetre ruler, a square or angle card, front and side close-up views,
and an overall recording with a visible time reference. Place rulers beside
the measured features, clear of the robot and carton, without touching the
paper. Keep the front wall, shoulder and pouring edge visible. The front view
shows the two mouth endpoints; the side view shows the apex, central upper lip
and front wall. A second side view can resolve an occluded feature. Arrange
grading views before the run; do not pull a flap aside, insert a probe, or
move the specimen to make a successful result visible.

For a photograph, keep its view approximately normal to the measured plane and
its scale at the feature's depth. Check a second known ruler interval to avoid
perspective errors. A square identifies the wall and shoulder reference lines;
an angle card checks whether body tilt exceeds 5°. There is no required camera
model, image resolution or special motion-capture equipment. The image must
simply resolve the relevant thresholds with a clear margin.

## Grade the ending state

1. In the final three seconds, verify that neither robot touches any carton
   part, the body remains supported by the table, and it stays within 5° of
   upright. Continuous overall footage shows release and support.
2. From the front view, check a mouth width of at least 30 mm. From the side
   view, check both the pouring triangle apex and the central upper lip at
   least 8 mm beyond the front wall, with the upper lip at least 35 mm above
   the shoulder. Refer to the material landmarks in SCORING.md; measuring
   an outer sealing flap instead does not establish a usable opening.
3. Watch the same interval to confirm those conditions persist. If the mouth
   visibly changes, inspect its narrowest / least-projecting state, rather
   than measuring only a favorable final frame. A clear threshold crossing,
   touch, lift or tip restarts the three-second interval. Small settling that
   keeps all dimensions in range is allowed; no edge tracking is needed.
4. Inspect before and after for a torn crease, detached panel or failed join.
   In the simulator this is checked by joint status and a constraint-gap guard;
   it is not an additional real-world 1 mm movement test.
5. If the full hold fails but the final geometry qualifies, award 0.5. Otherwise
   check the 0.25 thresholds (20 mm mouth, apex/lip at or beyond the wall, lip
   height ≥35 mm, tilt ≤5°). All remaining or damaged outcomes score 0.

## Borderline or obscured evidence

Do not infer an unseen endpoint from symmetry or round a measurement up to
pass. Use a clearer simultaneous grading view if a feature is hidden. If an
estimated measurement range straddles a threshold, record the physical evidence
as inconclusive and obtain an adequate view; this is neither a new score nor
permission to alter the numeric boundary. Likewise, blurred or discontinuous
footage cannot establish an uninterrupted hold. These are ordinary limitations
of photo/video evidence, not a requirement for submillimetre metrology.

The native demonstration has a wide margin in its final geometry (about 45.9 mm
mouth and 14.7 mm minimum projection) and more than 21 seconds of released,
supported opening. Its complete 240 Hz data is independently regraded. Those
records validate the simulated outcome; a physical specimen and this physical
grading procedure have not yet been tested. No additional model run is claimed.
