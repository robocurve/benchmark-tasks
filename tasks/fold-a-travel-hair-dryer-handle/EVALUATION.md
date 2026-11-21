# Recorded evaluation protocol

The two scripted successes and one Dream-1 beta outcome were recorded before
the scoring revision, with seed 0 and the same frozen instruction, scene, scorer
and ordinary 1000-action limit. Native physics is
240 Hz, control is 20 Hz. The model sees only three
320 × 176 RGB views and 14 actual arm/gripper positions; body state, contacts,
grades and the additional review camera are unavailable to it.

The pinned stock Dream integration uses 32 preroll steps within the budget,
48 action rows per chunk and a rolling nine-frame window per camera at 5 Hz.
Physics pauses while inference is pending. Stock SDK filtering/seam joining,
ClampGuard and DeltaLimitGuard(max_delta=1.0) are retained.

The model scored 0 after 1000
actions. There were 24 action-request attempts,
21 successful chunks and
3 service-session-expiration retries.
Stock guards changed 237 action steps. Native
grading and the complete model-input-to-executed-action chain were independently
replayed. Session recovery preserves the visual window and task state but may
reset server memory; the integration effects limit interpretation. These are
single fixed-start model observations, not statistical success rates.

The original request-attempt allowance was 79.
No successful or failed task result was retried on the basis of its score.
Every recorded model episode consumed its normal full action budget.

The package contains original-result hashes, explicitly marked result summaries,
portable audit metadata and the board media reference. The complete raw physics,
RGB and API/action archives remain in the original research records. Summaries
cannot replace those inputs for full offline replay. Machine paths have been
omitted from presentation copies; the original report hashes remain unchanged.

The package's portable evaluator changes only task/root discovery and the
credential-file default. The logger is copied byte for byte. No fresh model
call was made to manufacture scores during packaging.

## Revised score check

Revised scoring of seed-0 recordings: scripted 1.0 in both trials, first reached at action 859 of 1,000 (878 recorded). Dream-1 beta 0.0 after 1,000 actions; the hinge stayed at 0.283° throughout. Both archived scripted input streams were fed to the revised native scorer, including the samples after its first success. The archived Dream-1 audit covers all 12,000 physics samples. The largest hinge fold was 0.283°, below the 45° threshold for any positive score. The model check uses the full-run audit of that unchanged prerequisite. No new simulator episode or model call was made. See [revision evidence](evidence/REVISION.json).
