# Submission review

This is one fixed-layout task. The submission contains only this task folder;
recordings and the setup image are uploaded separately with the official script.
No task-board credentials belong in the repository.

| Criterion | Evidence / result |
| --- | --- |
| Importance | Multi-object reclassification between vertically constrained levels while preserving every neighboring bottle; comparison below |
| Construct validity | Contact geometry constrains access; no object attachment or teleportation; irreversible safety latch monitors all six bottles |
| Scoring | Numeric pose, fall, placement, release and two-second drift criteria; 0 / 0.5 / 1 partial credit; initial state scores zero |
| Simple tools | Ruler, protractor and calibrated video protocol in PHYSICAL_SCORING.md; force telemetry does not affect score |
| Reproducibility | Fixed seed and layout; complete scene inventory, dimensions, dependencies, commands and measured runtime supplied |
| Feasibility | Full-score scripted GPU recording within 2,400 steps; regressions and horizontal IK checks pass |
| Fidelity | Opaque rigid plastic surrogates; no fracture or force-based damage claim; finite measurement resolution declared |
| Blind review | Camera-only recordings without audio; sampled frames and public task prose checked for identifiers |
| Metadata length | Every task.json section is at most 500 characters; full setup/scoring details remain in the accompanying files |
| Submission mechanics | Registered GitHub author confirmed; official media upload complete (M-7UJQES); strict preflight passed |

## Related tasks and substantive distinction

The authenticated task board was checked on 2026-11-03. Its closest bottle task
retrieves one selected bottle from a crowded row into an empty tray while keeping
neighbors upright. The present task instead corrects two misplaced bottles
between occupied, roof-constrained shelf levels, finishes with all six classified,
and demonstrates temporary staging and a three-move transfer cycle. The pedestal
is available, not a scored mandatory strategy. Alternative safe solutions earn
the same score. The distinction is the goal and constrained multi-object
rearrangement, not merely bottle colors, shelf placement or narrower spacing.
This remains a reviewer judgment; the board check does not guarantee acceptance.

Published task-set references were also checked for related manipulation goals:

- [LIBERO wine-bottle task](https://github.com/Lifelong-Robot-Learning/LIBERO/blob/master/libero/libero/bddl_files/libero_goal/put_the_wine_bottle_on_the_rack.bddl): single-bottle rack placement.
- [RoboCasa task catalog](https://robocasa.ai/docs/build/html/tasks/composite_tasks.html): related storage and multi-object manipulation.
- [SimplerEnv](https://simpler-env.github.io/): related tabletop manipulation evaluations.
- [BEHAVIOR task catalog](https://behavior.stanford.edu/challenge/tasks/index.html): related sorting and stocking activities.
- [DataCenterBench](https://robocurve.org/datacenterbench/): control and connection tasks.
- [RoboDojo real tasks](https://robodojo-benchmark.com/doc/real-tasks/): related bottle manipulation; its catalog includes standing bottles upright.

These comparisons support the stated distinction, not an exhaustive claim that
no related task exists. No changes were made to existing pending submissions.

## Evidence and publication

validation.json records evaluated and prepared source hashes and measured trial
results. The private evidence bundle retains raw logs, safety traces and exact
evaluated source; the preflight checks their consistency and media hashes.
The prepared task differs from the evaluated task only in its module description;
its executable AST is identical. Force is diagnostic telemetry in both.

The official uploader returned media bundle M-7UJQES, now recorded in task.json.
Strict validate_submission.py passed with all required metadata and no pending-media
exception. Review status is tracked by the GitHub pull request and task board.
