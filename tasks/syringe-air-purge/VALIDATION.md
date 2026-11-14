# Validation — 8 November 2026

> **14 November scoring revision:** the current rules and offline regression evidence are in [SCORING_REVISION.md](SCORING_REVISION.md). The following is retained history from the first revision; its aspiration penalty and original-direction-only rack rule are superseded. Recorded results and hashes are unchanged.

> **Revision — 12 November 2026.** Physical reconstruction and measurement details are in [PHYSICAL_PROTOCOL.md](PHYSICAL_PROTOCOL.md); comment-by-comment changes and evidence status are in [REVISION_NOTES.md](REVISION_NOTES.md). Historical trials and fresh scripted controls are explicitly separated below. Dream-1 beta ran the current instruction and held-endpoint scorer on12 November2026 and scored0.0. No hardware validation is claimed.

The full scripted return-to-rack task passed with score **1.0** in the fixed-layout Brev run `syringe-seal-v16/full` (seed 1, object jitter 0).

| Measure | Result |
|---|---:|
| Control steps | 675 |
| Simulated time at 20 Hz | 33.75 s |
| Rollout wall time, excluding simulator startup | 115.05 s |
| Residual air | 0.0106 mL |
| Sample loss | 0.0444 mL |
| Total aspirated air | 0.0412 mL |
| Lift / return / release gates | Passed |
| Blank video frames | 0 |
| Worst reported IK position error | 0.1 mm per arm |

The no-action, horizontal-push and excessive-push controls each scored **0.0**. Horizontal push lost 1.5764 mL sample; excessive push lost 2.5618 mL. Both returned and released but correctly failed the persistent sample-loss gate.

Nine fluid/scoring checks and four passive-seal checks pass. The current submission files match the live Brev source used for the successful run. The original diagnostic variants remain outside this submission.

The demo reads simulator state. The fluid and seal models are illustrative, uncalibrated approximations; this single seed establishes simulation feasibility only. Hardware validation is outstanding. Task-board media ID: **M-SEEP5V** (model video, full-score demo and setup screenshot).

## Dream-1 beta — actual model trial

On 8 November 2026, `dreamlabs/dream-1-beta` completed one independent trial using the same physics source and exact submitted instruction. See [dream_result.json](dream_result.json).

| Measure | Result |
|---|---:|
| Task score | 0.0 |
| Control steps / simulated time | 1,800 / 90 s |
| Evaluation wall time, including startup and inference | 231.61 s |
| Mean inference latency | 0.603 s |
| Residual air / sample loss / aspiration | 1.5 / 0.0 / 0.0 mL |
| Lift achieved | No |
| End condition | Step limit |
| Evaluation errors | 0 |
| Recorded frames per camera | 1,801 |
| Blank frames per camera | 0 |

The arms moved around the syringe, but the logged state never achieved the required lift or air removal. Initial, midpoint and final camera frames were inspected. The final logged syringe position remained (0.25, 0, 0.8868) m. A three-camera H.264 model video was assembled from the recorded trial frames; no scripted actions were substituted. The existing full-score scripted demo remains separate.

The automatic video export initially could not find FFmpeg on PATH. Export succeeded using the installed `imageio_ffmpeg` binary; this did not rerun or change the model trial. The runner now resolves that binary automatically. The three required media files were uploaded successfully as **M-SEEP5V**. The model video is H.264, 1280 × 480, 20 fps, 90.05 s (7,475,578 bytes); the full-score demo is H.264, 1280 × 480, 20 fps, 33.8 s (2,797,164 bytes); the setup screenshot is 48,065 bytes. The official Python upload request returned Cloudflare error 1010; curl succeeded with the same documented endpoint, bearer authentication and multipart fields.

## Additional scenario checks — 12 November 2026

See [SCENARIOS.md](SCENARIOS.md) and [machine-readable results](validation/scenario_results.json). These are newly executed reduced-order-model checks, separate from the historical recorded rigid-body trials above. Fresh recorded controller rollouts are reported below. No model-only result is represented as robot or hardware evidence.

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
