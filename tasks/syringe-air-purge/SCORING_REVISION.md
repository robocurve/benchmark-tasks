# Second-review scoring revision — 14 November 2026

Addresses [PR #1151's scoring feedback](https://github.com/robocurve/benchmark-tasks/pull/1151#issuecomment-6872995745) on the existing submission branch.

| Review issue | Change |
|---|---|
| A successful re-purge receives permanent zero after >0.20 mL aspiration. | Removed aspiration from the persistent-failure condition. Retraction still admits air through the reduced-order model; the final air and liquid-loss limits determine credit. The cumulative counter remains diagnostic and supports conservation checks. |
| An otherwise valid rack return fails when the syringe points the opposite way. | The production rack predicate uses `abs(axis_y)>0.985` with `abs(axis_z)<0.15`, accepting both axial directions. Position, height, speed, release, previous lift and continuous one-second hold requirements are unchanged. |
| Written rules and controls must agree with the implementation. | Updated `task.json`, README, physical protocol, measurement sheet and scenario expectations. `retract_repurge` is now a valid recovery control expected to score 1.0. Preserved historical recordings and their original results with explicit superseded-rule labels. |

There is no keyed nozzle end in the procedural rack. Reversing the syringe is eligible only when it also meets the same rack position, alignment, speed and fingertip-clearance criteria. Arbitrary sideways, tilted, moving or out-of-rack poses remain ineligible. No collision geometry, fluid transport, seal mechanics, nominal demo actions or task instruction changed.

## Validation

Run from this task folder:

```bash
python -m unittest -v check_physics_model check_seal test_sample_visuals test_endpoint_hold test_scoring_revision
python scenario_checks.py --output validation/scoring_revision_20261114/scenario_results.json
```

- **26 tests pass.** Regression cases exercise corrective aspiration of 0.199, 0.200, 0.201, 0.400 and 1.000 mL, successful re-purge and both rack directions through the production fluid, rack and hold functions. A repeated recovery exceeding 8 mL cumulative aspiration also remains eligible.
- A successful recovery still receives only 0.6 before the continuous one-second hold completes. Missing lift/return/release, excess liquid loss, unrecovered air, invalid pose and excess speed retain the appropriate zero or partial score.
- All existing air and liquid-loss tier boundaries remain unchanged.
- **10 fluid-model scenarios pass**, with source hashes and compressed traces in [validation/scoring_revision_20261114/scenario_results.json](validation/scoring_revision_20261114/scenario_results.json). The corrected re-purge case ends with 0 mL air, 0.05 mL liquid loss and 0.40 mL cumulative aspiration, earning 1.0 fluid-quality credit. Its endpoint booleans are supplied by the test; the independent hold/rack regressions are listed above.

## Evidence scope

These are offline regression and reduced-order model checks, not new robot rollouts or hardware trials. Media **M-QDEBV2** retains the 12 November Dream-1 beta recording (0.0, 1,800 steps) and scripted full-score demonstration (1.0, 700 steps), with original source/video hashes. Those videos have not been regenerated for this scoring-only revision; no revised-source model result is claimed. The nominal demonstration's already-passing aspiration history and +Y rack endpoint remain eligible under the relaxed rules, and its one-second hold is unchanged.

The historical retract/re-purge videos and score records still show zero under the old scorer. They are preserved as evidence of the reviewed defect, rather than relabelled as new successful robot runs. No physical-hardware validation or official acceptance is claimed.
