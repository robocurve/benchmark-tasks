# Judging: Join a 1×1 block to a 2×2 block

Use the complete video, paused frames, a ruler or calipers, and a protractor.
Follow each brick throughout the trial. Red must finish above blue.
Any one of the four lower studs may be inserted into the upper single-stud brick.

Seated body centres differ by 8 mm along both X and Y, in either direction. One blue corner stud is covered and three remain exposed.

| Score | Observable requirement |
|---|---|
| 0 | No qualifying pickup, or either brick leaves the permitted table area. |
| 0.25 | The robot holds red with its lowest point at least 20 mm above the table for 0.20 s. |
| 0.5 | After pickup, red is seated on blue in the required configuration for 0.20 s. |
| 1 | After seating, hold red only with blue's lowest point at least 20 mm above the table for 1 s; then leave the joined pair upright and released on the table for 1 s. |

**Seating:** each required stud/socket centre is within 1 mm laterally and
0.75 mm vertically of full insertion. Red's underside meets blue's body top;
stud height adds no gap. The bricks' upward axes differ by at most 3°.
The single round connection allows any yaw where the bodies fit.

Infer stud centres from the part drawings and visible body edges. Measure both
horizontal offsets in a top view and the body gap in a side view. The combined
lateral error must be ≤1 mm, not 1 mm per axis. Use timestamps for each hold
and views showing the table, bricks and both grippers. An oblique image alone
cannot establish hidden gaps or submillimetre seating.

**Lift:** every part of both arms stays ≥2 mm from blue, and the required
connection holds for the whole second. Gripping blue, supporting it directly or
balancing a loose stack does not qualify. The automated grasp check requires
both finger collision bounds within 1 mm of red's body and the tool centre
within 35 mm. Bounds cover all collision meshes of both arms.

**Finish:** blue's bottom is within 3 mm of the table and its upward axis within
3° of vertical. Both arms are ≥5 mm from both bricks. Each brick stays within
2 mm and 5° of its pose at the start of a continuous one-second rest. Exceeding
either limit restarts that interval.

**Bounds:** every part of each brick, including studs, stays within X[−0.17,0.89] m
and Y[−0.63,0.63] m. A lowest point >15 mm below the table, or leaving these
bounds, makes the final score zero. The simulation has no destructible parts.

Use the final state. Retries are allowed within 1,200 actions. Pickup credit
survives a normal failed join. Separation or wrong alignment removes connection
credit and the lift proof. Correct seating earns 0.5 without a completed lift
and released finish; earlier higher scores do not persist.

Samples are 50 ms apart; durations use timestamps. Gaps >75 ms reset active
timing windows. Malformed, nonfinite or non-increasing evidence invalidates the
automated score. Replay with the archived source. Visual rulers are useful only
in their own planes; perspective changes the scale elsewhere.
