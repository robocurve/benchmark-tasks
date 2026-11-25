# Water-trainer validation record

## Articulated simulation and replay

The two YAM arms and the submitted fixture ran in standalone NVIDIA CPU PhysX. Each of the 13 conditions ran twice. A third run through the packaged operator command reproduced the same full summary, giving 39 fresh rollouts in all. Each episode contained a reset plus 14,400 consecutive physics intervals. The 13 pairs produced identical uncompressed trace hashes and identical reports. The original reference and controls use the submitted arm commands. One diagnostic slows the arm commands and closes both grippers at home. The public case is the raised-bed layout. The standalone runner is deterministic. [Results and replay decisions](native_results.json) · [Repeat verification](standalone_repeat.json) · [Runtime versions](standalone_environment.json)

| Condition | Score | First terminal 15 s, mL | Second terminal 15 s, mL |
|---|---:|---:|---:|
| Raised reference | 1.0 | 7.564 | 7.564 |
| Lowered reference | 1.0 | 7.564 | 7.564 |
| Initially aligned | 1.0 | 7.564 | 7.564 |
| Shifted reference | 1.0 | 7.563 | 7.563 |
| Slower reference, parked jaws closed | 1.0 | 7.564 | 7.564 |
| Corrected, left paused | 0.5 | 0.000 | 0.000 |
| Carriage held | 0.0 | 7.551 | 7.551 |
| Idle | 0.0 | 0.000 | 0.000 |
| Flow path blocked in model | 0.0 | 0.000 | 0.000 |
| Lift arm parked | 0.0 | 9.363 | 9.363 |
| Brake arm parked | 0.0 | 0.000 | 0.000 |
| Opened too low | 0.0 | 25.564 | 25.564 |
| Opened too high | 0.0 | 0.000 | 0.000 |

These quantities are model-integrated trainer water. For the earlier native reference video, a separate pixel read of the gauge found 7 and 8 mL in the two final windows. The video has 1,050 frames including reset. Angular compliance, release and contact are recorded separately from water delivery. For the dry PhysX proof, a 0.705 kg load and the 0.470 kg nominal load settled on the same tooth. The largest measured difference/excursion over the 30 s comparison was 0.087 mm. [Video readout](demo_optical_replay.json) · [Dry proof simulation](brake_proof_simulation.json)

## Scorer and geometry

The 53 boundary and integrity cases exercise both 15 s quantities independently, inclusive level and release limits, the 30 s/2 s observation lengths, premature opening, an altered outlet or datum, an interrupted hold, and missing or corrupt records. Valid generated records were also fed through the online monitor. The earlier engineering suite was rerun, including 29 manufactured trajectories and 43 static reach points. The sensitivity calculation perturbed full-open conductance by ±10% and the outlet offset by ±1 mm on the 12 fresh standard trajectories. All 48 combinations retained the expected score. [Threshold tests](../tests/review_thresholds.py) · [Boundary results](threshold_results.json) · [Offline contract](original_offline_results.json) · [Sensitivity](calibration_sensitivity.json)

The fixed-frame contact and jaw posture alone do not enter the score. Positive clearance and a valid physical effect do. The 3 mm routed line uses geometric envelopes in simulation; tubing kink, physical leakage and collector tampering require direct observation on the bench. The assigned half-credit condition also requires an initially wrong setup. [Source-to-hardware comparison](simulation_physical_mapping.json) · [Physical grading](../physical/GRADING.md)

## Fabrication interference and reach

The 11 STEP parts were imported as single valid solids. An interference screen checked 1,155 part/pose combinations through brake travel and tap rotation. The compressible rotor/liner fit is intentional. The final parts were registered to both robot arms for four recorded trajectories, 1,201 samples per layout. Only the intended fingertip/control contact pairs appeared. A separate test screened eight rigid water-hardware envelopes on the same 4,804 samples. Neither arm intersected them. These are finite 20 Hz pose checks. The hardware trial must establish intermediate clearances, mounted fasteners, flexible hose routing and the wet load capacity. [CAD methods and re-run commands](CAD_CHECKS.md) · [Fit results](cad_fit_results.json) · [Rigid plumbing results](water_trajectory.json)

The bench worksheet checker was exercised with 19 synthetic input tests, including invalid data and uncertainty overlapping a decision boundary. Use it with the actual measurements and source video. [Water series checker tests](../tests/test_water_checker.py)

## Completed hosted and scripted episodes

On the public raised-bed reset (seed 0), hosted Dream-1 completed all 1,200 actions and Inspect scored the trial **0.0**. It left the tap closed and the receiver collected 0.0 mL of modeled water. At 35.05 s (physics tick 8,412), the measured arm-to-mannequin safety-envelope clearance reached zero. The circuit integrity check passed throughout and PhysX recorded no mannequin contact pair.

The scripted reference scored **1.0** after 1,049 actions (52.45 s). The receiver gained 7.562 mL and 7.562 mL in the two consecutive 15 s verification windows. Independent replay matched both Inspect scores.

Dream applied 32 preroll actions and 1,168 hosted action rows from 25 requests. The model video contains 1,201 frames and the reference video contains 1,050 frames, each including reset. Both use media group `M-R3WQTE`. [Episode, receipt and video hashes](brev_episode.json)

## Earlier hosted session

The source-matched Inspect 1.0.0 receipt ended in a provider refusal after **1,184 of 1,200 actions** (32 preroll and 1,152 hosted). Inspect returned no task score. Independent replay covered 14,208 physics intervals, or 59.2 seconds, and scored the recorded trajectory **0.0**. Maximum opening was 3.02° and collected water was zero. The earlier model video in media group `M-TPVH8W` has 1,185 frames, including reset. The historical native Inspect checklist reports all 36 scripted control/seed results as expected. That is a distinct record from the fresh deterministic suite. [Refusal, exact provider text and hashes](refusal_receipt.json) · [Runtime and task hashes](source_hashes.json)

## Physical evidence still to collect

The fabrication files, source survey, geometry tests and steady-flow calculations specify and test the design digitally. No hardware was built during this revision. Actual machining tolerances, release force, detent response, filled mass, proof-load drift, wetting, source-head drift, delivery at intermediate angles and installed robot clearance remain commissioning measurements. Record the mechanical, hydraulic and repeatability sheets against the actual build, followed by a loaded robot trial. The physical grade uses synchronized ordinary measurements and video. Independent instrument failure or an unresolved boundary is recorded as indeterminate. [Reset](../physical/RESET.md) · [Mechanical commissioning](../physical/MECHANICAL_CALIBRATION.md) · [Water commissioning](../physical/WATER_CALIBRATION.md)

To regenerate the authored source survey, run `python validation/survey_source.py --check` in the public task folder with OpenUSD installed. The four layouts share the same world carriage limits. The scene, scorer, policy and public instruction retain their reviewed source bytes.
