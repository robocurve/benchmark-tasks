# Judging and evidence

Category: **Societal relevance**. Scorer: **roll-a-die-v3**.

Picking up and rolling a small die is one component of helping someone
participate in tabletop games. This task tests that manipulation skill. It does
not require a particular result or claim that outcomes are random or fair.
The six faces have distinct colours and pips so a reviewer can follow one
physical face through a turnover.

**Current evidence status:** the final v3 demonstration scores **1.0** in
**9.10 simulated seconds** and shows the qualifying face sequence clearly.
The matching final Dream run scores **0.0** after 1,200 actions / 60 simulated
seconds, without a qualifying pickup. Both final runs use the same frozen scene,
scorer and core source. The final videos and setup screenshot are uploaded as
media **M-PV3ZXY**, referenced in `task.json`.
Historical blank-cube recordings and v2 scores are development evidence only,
not feasibility evidence for this revised scene/rubric.

## What the reviewer receives

The task board receives [task.json](task.json), the actual initial setup
screenshot, the complete Dream video and a full-score demonstration if Dream
does not achieve 1.0. The repository supplies the scene, scorer, policy,
[reproduction commands](README.md), versioned validation record and anonymous
measured trajectories. The guide does not guarantee every submission is rerun.
Task text and media must be anonymous; code may identify its authors.

The official [review checklist](https://github.com/robocurve/benchmark-tasks/blob/main/AGENTS.md#what-makes-a-task-pass)
requires all five criteria:

1. **Importance:** meaningful novelty relative to the board and popular task
   sets, plus a clear real-world purpose beyond superficial scene changes.
2. **Construct validity:** the implementation matches the claimed skill and
   unintended strategies cannot achieve credit.
3. **Scoring:** unambiguous outcomes, appropriate partial credit, simple tools,
   coverage of failure cases and tolerance of irrelevant execution differences.
4. **Clarity and reproducibility:** specified objects, dimensions, materials,
   layout, instruction and reset procedure.
5. **Feasibility:** a full-score demonstration within the limit and safe physical
   trials.

## Numeric rubric

Score measured observations at 20 Hz, using simulator timestamps. Both policies
have a maximum of 1,200 control steps / 60 simulated seconds. Action targets do
not determine the score. A TCP is the point between the gripper finger pads,
13 mm short of the pad tips along the approach axis. All hand-clearance distances are measured from it to
the die centre. In the adapter gripper frame this is `TCP_OFFSET=(0, 0, 0.134)` m;
The 13 mm tip offset lets a reviewer locate it with a ruler. The final score
is used, so later violations can erase credit already earned.

| Score | Required evidence |
|---|---|
| 0 | No qualifying pickup, or any permanent invalidation. |
| 0.25 | The robot holds the die with its bottom at least 30 mm above the tabletop for 0.20 continuous seconds; at least one measured TCP is less than 35 mm from the die centre and its normalized opening is below 0.30. |
| 0.5 | After pickup, the die is airborne with its bottom at least 10 mm above the table and both gripper TCPs at least 50 mm from its centre. |
| 1.0 | After release, one identified face first points upward within 30° and later the same face points downward within 30°. The die then rests on a face for one continuous second under the conditions below. |

A face points upward when its outward normal is within the stated angle of
world +Z; downward means within that angle of world −Z. A face is first
registered only at or after verified release. Rotation while held cannot
supply the upward observation. The sequence is retained, so a complete
revolution can qualify even if the die returns to its starting orientation.
Pure yaw and small repeated rocking cannot satisfy an up-to-down face pair.
The two 30° tolerances require at least 120° separation between qualifying face
normals; cumulative small rotations are not an alternative condition.

A downward-facing surface can be hidden by the die itself. Its known opposite
face pointing upward within 30° is equivalent evidence: the two outward normals
are negatives of one another. The fixed opposite pairs are white 1 / yellow 6,
red 2 / cyan 5 and green 3 / purple 4. This is a rigid-body inference from an
actually visible, identified opposite face, supported by the mapping and
continuous footage; an unidentified or fully obscured cube is not sufficient.

Rest requires the die's centre to stay within 2 mm and its full orientation,
including yaw, within 5° of the fixed pose that starts a continuous one-second
interval. During that interval, its bottom must remain within 3 mm of the tabletop, a face must be within 10° of horizontal,
and both gripper TCPs must remain at least 60 mm from its centre. If a condition
fails, start a new rest interval; separated short rests do not add up. An
anchor-envelope violation sets the anchor to the current pose and rest duration
to zero, so earlier frames cannot contribute to the restarted interval.

After release, either TCP entering the 50 mm clearance permanently invalidates
the trial. The whole die must remain inside x = [−0.17, 0.89] m and
y = [−0.63, 0.63] m. A bottom more than 15 mm below the tabletop also invalidates
it. Out-of-bounds failures erase every credit level, including after rest.
The canonical detailed predicates are in [scoring.py](scoring.py).

## Score with ordinary video and a ruler

Use the recorded `review.mp4` side/oblique close-up plus the original wider
views covering both hands and the whole permitted tabletop. `review-video.json`
records frame times; `visuals.json` records the view and face-marking setup.
Keep the original complete recording with timestamps.
The 1600 × 1280 review view includes tabletop and vertical scale references with
5 mm ticks. Their tick spacing is exact in the guide plane; perspective means
it cannot be applied unchanged to a die at another depth. Use the specified
fixed camera geometry and the other views to cross-check depth, or in a physical
trial measure heights, clearance references and boundaries with a ruler before
recording. Use paused frames, an angle guide/protractor and frame timestamps
for the visible tests below. Cross-check angle in both views: an angle measured
on one oblique image is not automatically the physical face angle. Use sufficiently clear footage to distinguish the
colours and pips; an obscured face is not an observed face.

1. **Pickup:** show a grasped die clearing the 30 mm height for at least 0.20 s.
   Check continuous grasp from the close-up rather than inferring it from an
   intended gripper command.
2. **Release:** identify the first airborne frame with at least 10 mm bottom
   clearance and both TCPs at least 50 mm from the die centre. Confirm that the
   die has separated from the robot and has not been placed, pushed or supported
   by another arm link. This frame starts the face-history check.
3. **Turnover:** record a face colour/pip count that is upward within 30° at or
   after release. Find a later frame where that same face points downward within
   30°, or its documented opposite face points upward within 30°. Keep the
   intervening frames and known opposite-face mapping to establish identity.
   A single final photo or an unrelated face is insufficient.
4. **Rest:** over a continuous one-second window, compare die centre and pose to
   the start of that window. Confirm no more than 2 mm displacement or 5°
   orientation change, a level face within 10°, bottom within 3 mm of the table,
   and both TCPs at least 60 mm away.
5. **Failures:** inspect the entire recording for table exits or post-release
   TCP re-entry inside 50 mm. Check the final state; do not select only the best
   earlier segment.

The numerical scorer applies the same pose, time and clearance thresholds to
measured simulation states. Review footage avoids needing acceleration or
angular-velocity estimates, or resolving millimetre-scale gaps between fingers
and die. TCP distance remains a proxy for hand clearance; the numeric scorer
does not measure contact with every robot link. Visual review must reject a
supported, pushed, caught or obstructed motion that contradicts free release.
A score alone cannot establish an unseen face orientation; the documented
opposite-face witness above is valid visual evidence. Samples at 20 Hz can miss
very fast motion; ambiguous evidence needs clearer recording.


## Final demonstration evidence

The final marked-die demonstration, run 002, scores **1.0** after 182 actions /
**9.10 simulated seconds**; runner wall time is **44.457 s**. The complete
recording shows pickup at 4.60 s and release at 7.85 s. In the raised close-up,
the cyan five-pip face is upward at **7.85 s**. At **8.00 s**, its opposite red
two-pip face is upward, so cyan is downward. These frames establish the required
same-face turnover using the fixed opposite pair. The die then rests
continuously from **8.10 to 9.10 s** with the required clearance.

The measured maximum tilt excursion is 160.551° and cumulative motion 272.987°;
those values are explanatory diagnostics and do not substitute for the face
sequence. The earlier, lower-angle v3 camera hid the upward witness; that run
remains development evidence only.

The final Dream run 002 scores **0.0** after its full 1,200-action / 60-second
allowance, without qualifying pickup. Wall time is **184.133 s**. It used the
direct VM integration, with 25 requests, 25 attempts, one session and no retries.
Its **380 returned prediction rows outside action bounds** are disclosed; this
is not a count of executed actions. The returned stream also includes 32 unused
final-chunk rows. `DreamPolicy` returns raw predictions. Inspect's standard
execution bounds clamp and per-step limit review actions before execution, and
the adapter defensively clips to bounds again. An independent action-log audit
found **all 1,200 executed Dream actions and all 182 demo actions within bounds
and per-step limits**. The recorded physical execution and final score,
including this standard clipping behaviour, are the evidence. Reported inference
work of 7.981 s is a runtime diagnostic, not a Dream billing measurement.

## Fidelity and scope

The collision object is a homogeneous 16 mm, 5 g sharp-edged cube. Each visual
face panel is 15.6 mm square; black pips have 1.15 mm radius and 4 mm pattern
offsets. Panels and pips sit 4 µm and 8 µm outside the collision surface to avoid
rendering overlap. They identify orientation but do not model engraved dimples,
rounded edges or a calibrated physical die. The adapter's primitive
contact/friction parameters remain uncalibrated against a real die. Visual marks and the scale
reference do not add collisions or rolling aids. Simulation demonstrates the
specified manipulation, not physical transfer, random-number quality or a
repeat-success rate.

## Comparison examples and acceptance judgment

The following are presentation comparisons, not accepted precedents. Their
statuses were checked on 2026-11-03 during the preceding assessment:

- [Extend a telescoping lamp](https://github.com/robocurve/benchmark-tasks/blob/main/examples/task.json):
  the official illustrative template uses the same category, numeric partial
  credit and explicit fidelity limits.
- [Trace which cable end matches, PR #12](https://github.com/robocurve/benchmark-tasks/pull/12):
  household relevance with correct selection and placement; awaiting review.
- [Uncross two tangled cables, PR #13](https://github.com/robocurve/benchmark-tasks/pull/13):
  household relevance with partial credit; awaiting review.

This revision addresses the two main scoring concerns: accumulated rocking no
longer substitutes for a turnover, and named faces plus a pose-based rubric
make the required motion more directly observable. Acceptance still depends on
clear full-score footage, reproducibility, meaningful novelty and the reviewer's
judgment of the importance of the game-assistance skill. A successful scripted
demonstration is allowed when Dream fails; it does not guarantee acceptance.
