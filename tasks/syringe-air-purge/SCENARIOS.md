# Positive, negative and adversarial scenarios

## What the task measures

Coordinate the orientation of a **free syringe barrel** and the motion of its **independently sliding plunger**, removing air while preserving liquid, then park and release it. The phase at the outlet determines what is expelled: pushing the right distance is insufficient if the syringe is oriented incorrectly. The final low-air state is insufficient after excessive sample loss. Corrective aspiration is allowed if the final air and liquid-loss limits are satisfied. Returning/releasing is a separate endpoint requirement.

The intended robot solution uses one arm to support/orient the barrel and the other to control the plunger. The scorer measures object outcomes and episode history; it does **not** identify which arm did each action or prove that every conceivable one-arm/environment-assisted strategy is impossible. The unsupported-barrel control tests one concrete support ablation, not a universal bimanual-necessity claim.

## Paired comparisons

All robot variants start in a fresh simulator process, seed 1, zero jitter, same geometry/contents/friction, 20 Hz, and the same 1,800-step limit. The current instruction is identical across cases. A controller mode changes the approach, not the instruction, score thresholds or initial condition. No mode writes object poses, plunger state, fluid values or reward directly.

| Case / controller mode | Deliberate change from the positive approach | Required score | What must be observed to support the conclusion |
|---|---|---:|---|
| Positive `full` | Hold barrel, raise nozzle 50 degrees, wait 2 s, advance nominally 1.55 mL, return to rack, release and withdraw. | 1.0 | Air<=0.05 mL, loss<=0.30 mL; lift, park and release gates pass; no failure. |
| Wrong orientation `horizontal` | Keep barrel horizontal while performing the nominal purge stroke; still attempt return/release. | 0 | Loss>0.50 mL: liquid rather than trapped air is expelled. Verify the stroke occurred; a missed grasp is not evidence about orientation. |
| Empty-it shortcut `overpush` | Correct orientation, but advance nominally 4 mL instead of 1.55 mL. | 0 | Loss>0.50 mL despite air removal. A scorer based only on remaining air would reward this. |
| Valid recovery `retract_repurge` | Purge normally; retract 0.40 mL; purge again to the original endpoint; return/release and settle for 1 s. | 1.0 | Final air<=0.05 and loss<=0.30; all endpoint gates and the hold pass. Aspiration>0.20 mL does not latch a failure. |
| Incomplete purge `partial` | Advance nominally 1.00 mL; otherwise execute full sequence. | 0.3 | Air remains in the 0.30-0.75 mL tier, with loss<=0.50 and all endpoint gates satisfied. Partial progress is intentionally rewarded. |
| Omit return `no_return` | Purge correctly; withdraw the plunger arm but keep holding the syringe above the rack. | 0 | Fluid quality passes but `parked=false`. |
| Omit release `no_release` | Purge and return normally but keep the barrel gripper closed. | 0 | Fluid quality and park pass but `released=false`. |
| Remove support `unsupported` | After grasping the plunger, open/withdraw the barrel arm before advancing. | 0 expected; physical run required | Record barrel displacement/drop and failure. If the plunger gripper alone successfully supports the syringe, this is a counterexample to the support claim and must be investigated, not relabelled as success for the test. |
| Baseline `none` | No actions for the full 90 s budget. | 0 | Initial 1.5 mL air remains; no qualifying lift. |

`grasp` is retained as a diagnostic mode, not a claimed positive/negative control. Unknown mode names now raise an error rather than silently running the positive policy.

## Earlier recorded controls before the colour update — 12 November 2026

| Recorded approach | Score | Air remaining (mL) | Sample loss (mL) | Cumulative aspiration (mL) | Steps |
|---|---:|---:|---:|---:|---:|
| `full` | 1.0 | 0.0106 | 0.0444 | 0.0412 | 675 |
| `horizontal` | 0.0 | 1.5106 | 1.5764 | 0.0257 | 657 |
| `retract_repurge` | 0.0 | 0.0106 | 0.1017 | 0.5419 | 887 |

These historical runs used the superseded aspiration penalty: retract/repurge was recorded as zero despite meeting the final air/loss limits. The 14 November revision removes that penalty and treats successful corrective re-purging as a valid positive control. Historical scores and source hashes below are preserved as recorded, not presented as current-scoring results. Horizontal pushing still fails through excess sample loss.

Fresh processes, seed 1, zero jitter, identical scene/scoring and 20 Hz control. Each trial has a complete trace, exact source snapshot, initial image and three nonblank camera streams. Automated evidence checks and full composite-video decoding passed. These are **scripted rigid-body simulations with reduced-order fluid models**, not Dream-1 or hardware trials. [Recorded results and source/media hashes](validation/recorded_results_20261112.json). The earlier review used downsampled overhead-only copies. The current review below provides all three native-resolution views.

The exact physics/controller sources are frozen in each run. This documentation and the score-summary metadata were updated afterward; the recorded manifest correctly preserves the earlier document versions.

## Other evidence

**Historical recorded rigid-body simulations (8 November, previous source snapshot):** full=1.0, horizontal=0.0, overpush=0.0, no-action=0.0, as documented in `VALIDATION.md`. These predate the expanded scenario controller and recording checks. They are not new recordings for this revision.

**Current reduced-order model checks (14 November):** run `scenario_checks.py`. The report is [validation/scoring_revision_20261114/scenario_results.json](validation/scoring_revision_20261114/scenario_results.json); each case includes source hashes, endpoint assumptions, measured model outcomes and a compressed step trace. Results:

| Model comparison | Measured model score |
|---|---:|
| Correct orientation + purge + all endpoint gates | 1.0 |
| Horizontal push | 0.0 |
| Overpush | 0.0 |
| Purge / retract / repurge | 1.0 |
| Leave 0.50 mL air | 0.3 |
| Leave 0.25 mL air | 0.6 |
| Omit return, omit release, omit lift (three separate gate checks) | 0.0 each |
| No piston movement | 0.0 |

The model-only full case ends with 0 mL air and 0.05 mL loss. The retract/repurge case returns to these same fluid quantities but accumulates 0.40 mL aspiration and correctly receives full fluid-quality credit. The separate raw-score hold is checked in test_scoring_revision.py. Endpoint booleans in these checks are supplied by the test; they do not constitute measured grasps, lift/return/release or a robot video. The model-only stroke of 4 mL loses 2.5 mL; the previous robot overpush lost 2.5618 mL. Keep the evidence levels separate.

## Reproduce and record

```bash
python scenario_checks.py --output validation/scoring_revision_20261114/scenario_results.json
python check_physics_model.py
python check_seal.py

# Installed YAM/Isaac environment, with compatible Arena version selected:
export YAM_PYTHON=/path/to/yam-environment/bin/python
bash run_scenarios.sh /new/output/root full horizontal overpush retract_repurge
# Extended physical ablations:
bash run_scenarios.sh /another/new/output/root no_return no_release unsupported partial none
```

The runner refuses an existing output root and starts a fresh process per scenario. Each case contains its own source snapshot/manifest, `trace.json`, `result.json`, three camera videos, `demo.mp4` and a true initial `setup.png`. It then runs `verify_scenarios.py` to require the expected score, intended failure metrics, complete trace, matching source hashes, action budget and nonblank recorded frames from **every** camera. A crash, missing recording, wrong failure cause or changed source is a failed experiment, not a zero-scoring negative example. Manually inspect initial/transition/final frames and play the video before submitting.

For each comparison report the actual score, air, loss, aspiration, lift/park/release, failure flag, steps, seed and source hashes. Preserve unsuccessful positive trials and controls that accidentally pass; debug them rather than selecting only convenient evidence. A fixed-layout demonstration supports this setup, not a success rate over varied syringes or conditions.

## Reviewer-facing details

[PHYSICAL_PROTOCOL.md](PHYSICAL_PROTOCOL.md) specifies the named syringe/seal, water preparation, actual cavity-volume calibration, residual-air/loss measurement, uncertainty rules, pose thresholds and simulation-to-hardware differences. [MEASUREMENT_SHEET.csv](MEASUREMENT_SHEET.csv) is the blank hardware record. The physical protocol remains untested.

The comparison design follows the logic of [push-fit task #662](https://github.com/robocurve/benchmark-tasks/pull/662): remove or mistime a required action and observe the mechanism/outcome. Its current README reports zero for both no-press and early-release controls; 0.25 is the collet-press scoring milestone. Scores must be copied from the actual experiment, not assumed from the narrative.

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
