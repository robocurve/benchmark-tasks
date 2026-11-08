# Judging the knot

Start with the intact 131 cm outside-length, 10 mm cord in the flat, uncrossed
U shown in the setup. Nothing holds either end. Allow 150 seconds, including
the final five-second release check. Do not move, tighten or repair the knot
while judging it; there is no evaluator pull.

Follow the cord from one coloured end to the other. Exactly one overhand knot
is required: an end passes through a loop. Either handedness is valid. A
crossing, tucked bight, slipknot, figure-eight knot or tangle does not qualify.
Resolve the over/under order from the video, using additional views if needed;
a silhouette alone is insufficient.

| Score | Requirement |
|---|---|
| 0 | No qualifying final released knot, damaged cord or invalid simulation evidence. |
| 0.5 | One intact overhand remains for the final five continuous seconds, with all cord surfaces at least 3 cm from every robot part. |
| 1.0 | The same knot also previously remained intact for one continuous second with its outer tips at least 117 cm apart and every cord surface at least 1 cm above the tabletop. It then passes the five-second release check. |

Use a ruler or calibrated images for the straight-line distance between the
outer coloured tips and the minimum surface clearances. Use the video clock
for durations. Record the suspended span interval and the subsequent release
interval separately. Table contact after release is allowed.

Regrasping restarts the five-second release interval. Untying the knot cancels
its earlier span qualification; a new knot must qualify again. Ambiguous
crossings break a qualifying interval. Score the final outcome, not an earlier
best frame. No particular grasp, handedness or arm sequence is required.

The 117 cm target is outside cord length minus fourteen cord diameters. It
limits slack without defining where the knot starts or ends. It does not
measure tension, strength or maximum tightness.

## Simulation checks

`bimanual_scoring.py` reconstructs the measured capsules and checks qualifying
intervals at every 20 Hz frame. The numerical tip span subtracts the sum of
joint gaps to prevent artificial extension; an intact physical cord has no
such correction. A joint gap over 2 mm, intersecting centrelines, or nonlocal
penetration over 5% of diameter invalidates the recording. Robot clearance
uses complete collision-link envelopes.

`topology.py` uses several exterior closures and views to conservatively
identify an overhand candidate. This numerical certificate supplements video
review. A preformed or crossed starting layout, missing samples, fixtures or
external-force phases cannot establish the task. Source review must also
confirm the absence of hidden attachments or forces. Full credit requires
clean finalization. These checks do not establish what happened between
recorded frames or prove real-cord fidelity.
