# Revision: Reattach your detached wrist camera

The task tests recovery of the robot's own sensing configuration. The detached wrist camera remains live; the robot retrieves it and reinstalls it at its original wrist pose. Both arms may cooperate, and returning to READY is not required.

Changes addressing the review:

- Added a camera-side sliding shoe, a connected wrist channel with retaining lips and an end stop, and a retaining-tooth representation. Dimensions, materials and coordinate frames are listed in README and mechanism_dimensions.json.
- Replaced binary scoring with pickup, held alignment, secured installation and released retention stages (0.25 /0.5 /0.75 /1.0).
- Removed the 0.1 mm /0.001 rad sensor consistency requirement from scoring. README describes a practical ruler/video/alignment-template and live-camera check.
- Added per-asset sources/licenses, including the apple and orange, and disclosed dependency license limitations.
- Changed the category to Impact on AI timelines: installation and restoration of the robot’s own sensing hardware.

The camera shoe slides through the wrist channel, seats against the stop and engages the retaining tooth. The scripted demonstration completes installation and release at the original optical pose. Simulation details are specified in the fidelity field.

Recorded evidence: Dream-1 run 0.0; scripted demo 1.0. The revised model run completed 1,800 actions in one continuous session, without expiration or restart. The supplied demo, model video and setup screenshot use the revised task. Session identifiers are omitted from public results.
