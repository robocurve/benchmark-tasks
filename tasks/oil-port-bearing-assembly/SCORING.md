# Local outcome rubric — released-seated-full-10mm-port-v3

This local assembly-stage rubric evaluates the ending state. No mandated grasp,
partial-entry depth, motion, arm role or camera use. Full insertion by gravity is legal.

- 0.0: no released, supported, stable seating satisfying the conditions below.
- 0.5: sleeve seated and released for a continuous one second, but the specified
  passage is blocked during any part of that final one-second interval.
- 1.0: the same seating conditions, with the passage clear throughout that interval.

Seated: sleeve center within 0.8 mm horizontally and 0.5 mm vertically of the seat
center, axis tilt at most 1 degree from vertical (either axial direction is allowed).
The annular bottom shoulder must support it, with no contact between the sleeve and
either robot at every physics substep. An open gripper command is not evidence of release.
Still: speed at most 3 mm/s and angular speed at most 0.05 rad/s at every 20 Hz sample;
over the full second, displacement from its first pose at most 0.5 mm and orientation
change at most 1 degree. Twenty-one samples span one second; twenty span only 0.95 s.

Passage: the envelope of a straight 10 mm diameter gauge must pass along the fixed
housing oil-port axis, from 7 mm outside the housing to 5 mm inside the sleeve bore,
without intersecting the sleeve or housing material. Exact segment-to-triangle distances
to each convex piece test the continuous probe envelope at the LIVE sleeve pose;
endpoint containment is checked too. This uses the same authored convex vertices as
the collision/display geometry. A 10 nm numerical allowance prevents float roundoff
at nominal tangency; this is NOT a manufacturing tolerance. Floating-point cooking
differences are not calibrated. The full nominal housing port must remain unobstructed;
passing a thinner gauge is insufficient.
CPU PhysX sphere sweeps and 13 sample rays are advisory ONLY: a real rotated-sleeve
control exposed stale/missing moved-body hits despite a successful static-wall guard.
Those earlier query-based scores are superseded. The sleeve must also be present/seated: an empty housing with an
open port earns zero. This test measures geometric access, not oil flow or lubrication.

Missing or malformed state, inadequate contact coverage or an invalid query cannot
contribute to a hold interval. The hold resets on contact, motion or loss of seating;
no historical maximum is retained. A blocked but fully seated assembly gets partial
credit; gripping or hovering above the seat gets none. There is no invented force
threshold or fragile-material damage claim. Contacts are logged for engineering review.

Physical grading analogue: depth/position and axis measurements, a one-second released
hold video, and a straight 10 mm gauge through the housing port to the central bore.
The gauge can be checked after the robot withdraws; it is a grading tool, not an added
policy sensor. This physical protocol and manufacturing fit are not yet validated on
hardware. The scorer does not establish locking, bearing service life or pressure sealing.
