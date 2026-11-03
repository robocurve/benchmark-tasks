# Judging and evidence

Reviewed 2026-11-03. Category: **Societal relevance**. Handling a small game piece
is a component of helping someone participate in tabletop games. This task
demonstrates that physical skill; the blank cube provides no numbered result
and establishes neither randomness nor fairness.

## What the reviewer receives

The submission contains the instruction, context, fixed setup, scoring, run
limit, fidelity limitations and reported scores in [task.json](task.json), plus
the initial setup screenshot, complete Dream recording and successful scripted
demonstration. The source and [README](README.md) supply reproduction and replay
commands; anonymous compressed measured trajectories in `evidence/` let a
reviewer recompute all three recorded outcomes locally with Python and NumPy.
The guide does not guarantee every submission is rerun. Task text and
media must remain anonymous; code may identify authors.

The official [review checklist](https://github.com/robocurve/benchmark-tasks/blob/main/AGENTS.md#what-makes-a-task-pass)
requires all five criteria:

1. **Importance:** meaningful novelty versus the board and popular task sets,
   beyond superficial changes, with clear real-world relevance.
2. **Construct validity:** the implementation matches its claim; unintended
   strategies cannot earn credit.
3. **Scoring:** simple tools, unambiguous outcomes and partial credit, complete
   coverage, and insensitivity to irrelevant execution differences.
4. **Clarity and reproducibility:** specified object types, dimensions, materials,
   layout, instruction and reset.
5. **Feasibility:** demonstrated full score within the allowance and safe physical
   trials.

## Rubric and recorded evidence

The canonical implementation is [`scoring.py`](scoring.py), version
`roll-a-die-v2`, evaluated from measured observations at 20 Hz. Robot action
targets do not determine scores. Report the final score, not the historical
maximum. The allowance is 1,200 actions / 60 simulated seconds.

| Score | Required evidence |
|---|---|
| 0 | No verified pickup, or a permanent invalidation. |
| 0.25 | Cube bottom at least 30 mm above the table for 0.20 s, speed at most 0.15 m/s, with a measured gripper less than 35 mm away and normalized opening below 0.30. |
| 0.5 | Verified free release followed by at least **90 degrees maximum tilt excursion** from its verified free-flight reference pose. |
| 1.0 | The qualifying roll followed by 1 continuous second on a face: bottom within 3 mm of tabletop, speed below 0.01 m/s, angular speed below 0.25 rad/s, face tilt at most 10 degrees, and both TCPs at least 60 mm away. |

Tilt compares world vertical expressed in the cube's coordinates. Pure
world-vertical spin does not count. Small repeated rocking cannot accumulate
roll credit; cumulative rotation is diagnostic only. A full revolution may
qualify even if its final orientation matches its release orientation.

Release requires bottom clearance at least 12 mm, nearest TCP at least 22 mm,
and measured acceleration within 2.5 m/s² of vertical gravity, with horizontal
acceleration at most 3 m/s². Each hand must be at least 70 mm away or provide
2 mm clearance at each jaw, accounting for cube orientation and displacement.
An earlier interval can count only when both endpoints have clear jaws, the
cube is airborne and acceleration is consistent with free flight.

After release, loss of jaw clearance or TCP distance below 20 mm permanently
invalidates the trial. So does any cube extent outside x = [-0.17, 0.89] m or
y = [-0.63, 0.63] m, or its bottom more than 15 mm below the tabletop.

Replaying the retained successful demonstration under v2 gives **1.0**, with
**133.207 degrees maximum excursion** and 164.434 degrees cumulative motion.
This is rescoring existing measured motion, not a new physics trial.

## Practical visual cross-check

1. Check the screenshot against the 16 mm cube and specified layout. For physical
   replication, measure dimensions and positions with a ruler; record side and
   top/oblique views with a visible scale.
2. Watch the complete run: identify pickup, separation, free tumble and rest.
   Check for post-release placing, pushing or catching, and table-boundary exits.
3. Frame-step both views around release. Follow the same visible edges through
   time to distinguish a substantial turnover from a small rock or flat spin.
   Compare the replay's release timestamp and excursion trace; check pickup and
   rest durations from frame timestamps.
4. Keep close-ups/slow-motion alongside the complete source recording. They
   cannot add detail or observations absent from it.

This is a **cross-check, not a claim of exact manual equivalence**. A small,
unmarked symmetric cube can hide face identity; 20 Hz observations can miss fast
motion between samples. Ordinary footage may not resolve 2 mm jaw clearances or
the acceleration thresholds. TCP/jaw geometry is a contact proxy and does not
measure contact with every robot link. Ambiguous footage needs better evidence;
telemetry alone does not establish compliance with the simple-tools criterion.

## Comparisons and assessment

- [Extend a telescoping lamp](https://github.com/robocurve/benchmark-tasks/blob/main/examples/task.json):
  official **illustrative template**, same category; numeric partial credit and
  explicit fidelity limits. It is not an accepted-task precedent.
- [Trace which cable end matches, PR #12](https://github.com/robocurve/benchmark-tasks/pull/12):
  same category, home-office relevance, correct selection and placement;
  **awaiting review** as TK-2TE4ZZ.
- [Uncross two tangled cables, PR #13](https://github.com/robocurve/benchmark-tasks/pull/13):
  same category, household relevance, partial credit for uncrossing and correct
  placement; **awaiting review** as TK-B98F7J.

Statuses were checked on the assessment date; no upstream PR was merged. These
are presentation comparisons, not acceptance standards. The correction improves
validity, but reviewers still judge the significance of this game skill, the
blank-cube abstraction and visual auditability. A scripted demonstration is
allowed when Dream fails; it does not prove physical transfer or repeatability.
