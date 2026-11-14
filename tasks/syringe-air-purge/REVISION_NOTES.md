# PR #1151 — revision preview

> **14 November scoring revision:** the current rules and offline regression evidence are in [SCORING_REVISION.md](SCORING_REVISION.md). The following is retained history from the first revision; its aspiration penalty and original-direction-only rack rule are superseded. Recorded results and hashes are unchanged.

Prepared 12 November 2026. **Local draft only; no push or reviewer reply sent.**

Review: [RoboCurve comment on PR #1151](https://github.com/robocurve/benchmark-tasks/pull/1151#issuecomment-6820098680). The stated unmet criterion is **clarity and reproducibility**. The reviewer asks for a real syringe/seal, real liquid, initial-fill preparation, air/loss measurements at the thresholds and remaining simulation-to-hardware differences. The comment does not ask for a different task or harder physics.

| Review request | Prepared change | Evidence limit |
|---|---|---|
| Real syringe and seal | Named BD Plastipak 300629 with original factory stopper; no guessed elastomer chemistry. Added material, dimension, collar, rack and tray specifications. | Product specification, not a tested robot grasp. Commercial and simulated dimensions differ explicitly. |
| Real sample and preparation | Blue 0.01% w/v aqueous Brilliant Blue FCF tracer; gravimetric 10.50 mL solution preparation; calibrated 12.00 mL internal capacity to establish 1.50 mL air, including nozzle dead space. | Hardware preparation tolerances must be demonstrated. |
| Residual-air measurement | Calibrated cavity volume minus gravimetrically measured retained solution. Explicit uncertainty requirement for the 0.05 mL threshold. | Printed 1 mL syringe graduations and the white rendered air bands are insufficient. |
| Sample loss and other gates | Initial-minus-final solution mass/density; spill cross-check; video for cumulative aspiration, lift, park, release and one-second hold. | Unresolved threshold crossings are indeterminate, not a pass. |
| Transfer differences | Dedicated section on bore, dead space, friction, seal force, wetting, bubble shape and migration. | The 0.6 N seal and 25 mm/s bubble migration remain uncalibrated. |

## Resulting instruction

> Remove the air from the syringe while preserving the sample, then return it to its rack and let go.

This preserves the previously requested short sentence. `task.py` already contained it, but `task.json` still contained the older long instruction. They now match. The 8 November Dream-1 result used the long instruction; its record is preserved and labelled historical. A new short-instruction model run is required before reusing model evidence for this revision.

## Changed files

- `PHYSICAL_PROTOCOL.md`: complete reconstruction, calibration, measurement and adjudication procedure, with equipment sources.
- `MEASUREMENT_SHEET.csv`: blank per-trial measurement/evidence/uncertainty record; no synthetic results.
- `task.json`: short instruction, concrete hardware/measurement summaries, fidelity and honest evidence status.
- `README.md`, `VALIDATION.md`: links and historical-result notices.
- `REVISION_NOTES.md`: this review mapping.

Collision geometry, rigid-body physics, fluid code, seal code and scoring thresholds remain unchanged. Liquid/air visuals and barrel/collar transparency have been revised. The demo controller now includes explicit adversarial scenarios; the recording harness checks every camera and initial-frame validity. See SCENARIOS.md for the new controls and their evidence status. Existing videos therefore illustrate the same simulated mechanics, but do not demonstrate the new physical protocol. New rendered controls are reported below; no physical hardware result is implied.

## Validation performed locally

- `python check_physics_model.py`: **9 tests passed**.
- `python check_seal.py`: **4 tests passed**.
- Static checks: task JSON parses; instruction equals code; fidelity text is below 500 characters; changed Python parses; changes remain confined to this task folder.

Detailed local check record: [LOCAL_CHECKS.txt](LOCAL_CHECKS.txt).

Fresh recorded scripted trials completed on Chrysler in isolated RoboCurve output directories. Existing jobs and the separate physical-verification project were left untouched. Compatible Arena/Isaac Lab sources were selected per process without changing shared installed packages. The initial version-mismatch startup failures are retained separately and are not counted as negative results.

## Additional adversarial revision

[SCENARIOS.md](SCENARIOS.md) now specifies paired comparisons, expected partial credit, legitimate alternatives and evidence limitations. New `scenario_checks.py` results and compressed traces are stored under `validation/`. The demo controls use arm actions only. `run_scenarios.sh` isolates episodes and `verify_scenarios.py` rejects crashes, missing/blank media and unintended failure causes.

## Before resubmission

1. Run Dream-1 using the final short sentence and record its actual outcome, exact instruction and source hashes.
2. Prepare the matching media bundle and update its score/ID only after successful upload; keep the 8 November model result as historical evidence.
3. Review this draft with the owner before pushing to the **existing** `submit-syringe-air-purge-v16` branch / PR #1151. Do not open a replacement PR.

The written review requests are addressed in the draft. Real-hardware calibration and manipulation are not established by these software tests.

## Earlier recorded controls before the colour update — 12 November 2026

| Recorded approach | Score | Air remaining (mL) | Sample loss (mL) | Cumulative aspiration (mL) | Steps |
|---|---:|---:|---:|---:|---:|
| `full` | 1.0 | 0.0106 | 0.0444 | 0.0412 | 675 |
| `horizontal` | 0.0 | 1.5106 | 1.5764 | 0.0257 | 657 |
| `retract_repurge` | 0.0 | 0.0106 | 0.1017 | 0.5419 | 887 |

These historical runs used the superseded aspiration penalty: retract/repurge was recorded as zero despite meeting the final air/loss limits. The 14 November revision removes that penalty and treats successful corrective re-purging as a valid positive control. Historical scores and source hashes below are preserved as recorded, not presented as current-scoring results. Horizontal pushing still fails through excess sample loss.

Fresh processes, seed 1, zero jitter, identical scene/scoring and 20 Hz control. Each trial has a complete trace, exact source snapshot, initial image and three nonblank camera streams. Automated evidence checks and full composite-video decoding passed. These are **scripted rigid-body simulations with reduced-order fluid models**, not Dream-1 or hardware trials. [Recorded results and source/media hashes](validation/recorded_results_20261112.json). The earlier review used downsampled overhead-only copies. The current review below provides all three native-resolution views.

The exact physics/controller sources are frozen in each run. This documentation and the score-summary metadata were updated afterward; the recorded manifest correctly preserves the earlier document versions.

## Coloured-sample update

The current sample is blue 0.01% w/v aqueous Brilliant Blue FCF tracer; see PHYSICAL_PROTOCOL.md for its preparation and density calibration. The barrel/collar are translucent and non-overlapping blue liquid/white air bands follow the phase volumes. This changes visual evidence, not collision geometry, fluid rules or score thresholds. Earlier recorded results retain their original source/visuals; fresh recordings of the coloured version are reported below.

## Historical coloured-sample recordings — 12 November 2026

| Approach | Score | Air (mL) | Sample lost (mL) | Aspiration (mL) | Steps |
|---|---:|---:|---:|---:|---:|
| full | 1.0 | 0.0106 | 0.2059 | 0.0604 | 699 |
| horizontal | 0.0 | 1.5106 | 1.65 | 0.0237 | 652 |
| retract_repurge | 0.0 | 0.0176 | 0.2039 | 0.6502 | 907 |

All three trials use the same current blue-sample visuals, CPU PhysX, RTX-rendered cameras, seed 1 and zero jitter. The attempted GPU-physics launch failed at startup due to memory pressure; its log is retained separately and is not a scored trial. These CPU outcomes supersede the earlier GPU numbers for the current coloured visual version; differences must not be attributed solely to colour. No controller, collision, seal or fluid/scoring thresholds changed for this visual update. The two new phase-display tests and the 13 existing physics/seal tests pass.

[Exact results, source hashes and review-video hashes](validation/recorded_colour_results_20261112.json). Each video shows overhead, left wrist and right wrist at native 640x360 and 20 fps, with current volumes/score in a separate panel. Whole composite videos decode successfully and contain exactly steps+1 frames. Source snapshots preserve the documentation as it was when each trial began.

## Initial-fill preparation clarified after owner review

Expanded PHYSICAL_PROTOCOL.md section 3 into a step-by-step bench procedure: volumetric tracer recipe, submerged slow liquid draw, tip-up bubble expulsion, gravimetric adjustment to 10.50 mL, open-tip air draw to calibrated 12.00 mL capacity, reweighing, computed 1.50 mL air acceptance interval, coalescence, rack-placement checks and explicit reject/reprepare rules. Added the procedure summary to README and raw preparation measurements to MEASUREMENT_SHEET.csv. Preparation is before reset and does not count as in-trial aspiration. These are documentation changes only; no controller, scoring or recorded result changed, and hardware validation remains outstanding.

## Independent review iteration — 12 November 2026

Added reviewer-visible physical setup/metrology and explicit occupational/procedure scope. Corrected the raw-score timeout edge: a complete endpoint must hold continuously1 s before score1.0, not merely before early termination. Source now reports endpoint_hold_s. A reset, interruption or invalid timing clears the hold; this is covered by offline checks. The latest scorer needs fresh robot/model evidence; earlier recordings remain preserved. No push or publication.

## Current held-endpoint recording — 12 November 2026

The corrected scorer completes the full scripted sequence at **700/1800 steps,35.00 s,score1.0** with air0.0106 mL and liquid loss0.2059 mL. All three640x360 camera streams have zero blank frames. The raw score is0.6 at step699 with0.95 s endpoint hold and becomes1.0 at step700 with1.00 s hold. Nineteen offline tests pass. Current task.py hash: `3d3e690bb47d58b83dd262c57ff6fbbc99b0aa555fe3b2777909482a916c05aa`.

Both recordings use CPU PhysX and RTX rendering, seed1, zero jitter and20 Hz; these are scripted runs, not Dream-1 or physical experiments. Complete traces, source snapshots and three-camera footage are retained. Source bytes were independently checked against the separate pre-run run_config hashes. The automatically generated manifest mistakenly hashed its own preceding version; its original and original failed verification were preserved, that self-entry was removed with a correction note, and all remaining source hashes and evidence checks pass. No source/score was changed to repair packaging. The current Dream-1 run subsequently completed with score0.0; see validation/current_dream_20261112.json. Current official media are identified in task.json.


## Final model evidence — 12 November2026

Authentic Dream-1 beta run completed at the declared step limit, score0.0, without evaluation error. Exact current instruction and all Python source hashes match the reviewed implementation. Six evenly spaced model and demo frame composites and full decoding of all three model camera videos were checked. Current model result/provenance: `validation/current_dream_20261112.json`. Full-score scripted evidence is unchanged. Current media are identified in task.json.
