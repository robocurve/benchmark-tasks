# Scoring protocol (contact-witness-v3)

This protocol scores physical outcomes. Camera use, arm assignment, grip opening
percentage and the demonstrated route do not affect the score.

## Construction and measurement

The fixture, both anchors and guide are rigidly fastened to the table. The frame
is one rigid closed body; destructive changes are outside a valid task attempt.
The simulator models fixed mounting and rigid parts, not fracture or loose bolts.

Measure the **centre of the square tunnel**, not the orange handle. Its target is
the centre of the green support, at the guide height. Use a ruler/template and
photos from two directions to compare centres, a protractor/template for angles,
and timestamped video for stillness and contact. The tunnel must contain the
guide continuously between its two open ends; being near the guide is insufficient.

Over the last 1 second, the maximum separation between any two measured
centre positions must be at most 1 mm, and the largest
orientation change at most 2 degrees. Require the whole
elapsed second, not just 20 samples: at 20 Hz there are
21 pose observations spanning this interval. All nonzero scores need
this stillness interval and the guide still passing through the tunnel throughout it.

At goal: tunnel centre within 7 mm of its target in 3D; long
edges parallel to the support within 7 degrees; upright within
3 degrees. Measure the tunnel axis and a flat wall: any of the
four square-section faces may be uppermost, provided the actual body can rest
on the support. The handle does not have to face the same way as in the demo.

For progress, measure the angle from the fixture's arc centre to the tunnel centre.
The start-to-goal angle is110 degrees. Define closeness as1 minus the shortest
angular distance to the goal divided by110 degrees, clamped to[0,1]. This decreases
again on overshoot. It is only eligible while the guide actually passes through
the tunnel; an off-guide object earns0 even if its handle has favourable coordinates.

## Whole-episode contact witness

Zero if any prohibited contact occurs, including a brief touch, even if corrected
later. No other action can erase this violation. There is no100 ms grace period.
Prohibited: either robot against the task fixture/guide/anchors/supports/base;
contact between the two robots; nonfinger robot contact with the collar; collar
contact with the guide or anchors. Allowed: finger/collar grasping, and collar
contact with the supports or base. The original robot's own adjacent-link contacts
and station/table contacts are outside this task-specific contact rule.

Physics checks occur at 240 Hz: one force-bearing prohibited contact report
leaves a persistent private witness. Force magnitude is not a score threshold;
the numerical1e-6 N cutoff discards zero-force speculative contact records only.

For a physical reproduction, use a thin transferable contrast coating/contact film
on protected surfaces and clean contrasting receiving surfaces. Record close baseline
photos, then inspect for any new attributable transfer mark after the run. A mark
is failure evidence; cleaning or restoring a part does not erase failure. Ordinary
dry decorative paint is not a contact witness. Keep intended finger/handle and
collar/support contact regions uncoated; use distinguishable witness colors or
video to attribute other marks to prohibited rather than allowed contact pairs.
Inspect all relevant surfaces, including the tunnel interior after carefully
disassembling the mounted guide without creating new inspection marks. Photograph
the accessible sides before disassembly. Reset/clean the witness every trial.

Before accepting a physical run, check that the selected coating leaves an observable
mark in a light-touch positive control and none in a separated negative control on
the same materials. If coverage, attribution, sensitivity or inspection handling is
ambiguous, the physical trial is ungradable, not presumed contact-free. Only original
three RGB streams are supplied to the policy; post-run grading may use close photos.

Fidelity: simulation uses ideal contact-event memory, not pigment/fluid deposition.
It does not render transfer marks or give a contact flag to the policy. Actual paint
sensitivity has not been physically calibrated here; force-bearing numerical contact
at a small positive separation may precede a visible real mark. This approximation
must be disclosed, not presented as experimentally established paint equivalence.

## Final score (highest satisfied row; otherwise0)

| Score | Conditions, in addition to no contact violation and the stillness/threading interval |
|---|---|
|1.0|At goal throughout the last second, resting on the green support throughout, with no robot/collar contact at any time in that second. Closed empty grippers are allowed.|
|0.75|At goal throughout the last second, but full release/support requirements are not yet met.|
|0.5|Final closeness at least0.65: angular distance to goal at most38.5 degrees.|
|0.25|Final closeness at least0.25: angular distance to goal at most82.5 degrees.|
|0.0|Everything else, including a transient pass through goal, off-guide final state, insufficient stillness, or a latched prohibited contact.|

Thresholds are inclusive. Scores are based on the current ending state, never the
best earlier progress. An earlier safe detour does not invalidate a later legal
completion. Success ends the episode only after the complete final second.

## Instrumentation integrity

Invalid poses, missing/stale contact samples, coverage errors, or monitor failure
invalidate the trial until reset; a numeric0 placeholder is **not** a valid model
failure. The validator must reject such runs. Green-support contact and every
robot/collar touch are checked across all physics substeps, not only the last
substep of each control action. No simulator velocities are needed for stillness.

Real-world tolerances and coating sensitivity require calibration when reproducing
the task physically. The simulation evidence does not constitute a physical trial.
