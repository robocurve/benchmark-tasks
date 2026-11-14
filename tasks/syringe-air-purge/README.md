# Nursing preparation: purge air while preserving the drawn-up liquid
Submission media **M-QDEBV2** records the 12 November scorer: Dream-1 beta **0.0**, scripted demo **1.0**. The 14 November revision changes scoring eligibility only; these videos have not been rerun under the revised scorer. Original source/video hashes remain preserved.


> **Scoring revision — 14 November 2026.** Corrective aspiration is recoverable, and either axial direction is accepted in the rack. See [SCORING_REVISION.md](SCORING_REVISION.md) for the second-review response and validation. The following recording history describes the 12 November implementation. Physical reconstruction and measurement details are in [PHYSICAL_PROTOCOL.md](PHYSICAL_PROTOCOL.md); comment-by-comment changes and evidence status are in [REVISION_NOTES.md](REVISION_NOTES.md). Historical trials and fresh scripted controls are explicitly separated below. Dream-1 beta ran the same instruction and the earlier held-endpoint scorer on 12 November 2026 and scored0.0. No hardware validation is claimed.

**Status:** 26 offline regression tests pass for the revised scorer, including corrective aspiration above 0.20 mL, both rack directions and the unchanged one-second hold. Prior scripted/model recordings remain available; no new robot or hardware trial is claimed for this scoring revision.

A YAM pair performs the **air-management motor subtask of nursing preparation of a manually drawn-up injectable medicine**: support and orient the syringe, bring trapped air to the outlet, and advance the plunger without unnecessary liquid loss. The needle-free blue-tracer rig isolates that manipulation after loading; the robot then returns the practice syringe to its rack. It does not prepare a clinically usable dose.

## Why this belongs in Impact on jobs

The relevant occupation is **registered nurses, SOC 29-1141**. [BLS](https://www.bls.gov/ooh/healthcare/registered-nurses.htm) reports **3,465,400 US jobs in 2025**; [O*NET](https://www.onetonline.org/link/summary/29-1141.00) lists medication administration among the occupation's tasks. These are occupation totals, not a measured count of nurses performing this particular substep every day. The connection is routine injectable-medication preparation: nursing guidance describes managing bubbles using syringe orientation and controlled plunger movement while retaining the liquid. [Nursing preparation reference](https://elsevier-elibrary.com/contents/fullcontent/71232/epubcontent_v2/OEBPS/B9780729541107500387B.htm), ampoule preparation; [Immunize.org](https://www.immunize.org/ask-experts/topic/admin-vaccines/general-issues/) distinguishes air removal in user-filled syringes from manufacturer-prefilled vaccine syringes. Sources checked 12 November 2026.

The robotic difficulty is supporting a free barrel while manipulating its moving plunger and controlling which phase reaches the outlet. Horizontal pushing wastes liquid; retraction reintroduces air that must be purged again to meet the final volume limits. Corrective withdrawal followed by a successful purge is allowed. Those are the physical abilities tested, independent of the nursing label.

### Exact scope of the nursing analogy

| Real subskill | Benchmark representation and limit |
|---|---|
| Orient a manually filled syringe and control the plunger | A floating barrel and contact-driven piston; no fixed purge fixture or object teleportation. |
| Preserve drawn-up liquid while removing air | Blue 0.01% w/v Brilliant Blue FCF tracer replaces medicine. The 10.5 mL liquid and 1.5 mL gas are repeatable challenge loads, not vaccine or prescribed-dose quantities. |
| Work with a syringe and its seal | BD Plastipak 300629 with its factory stopper is the physical practice counterpart; its dimensions and forces require calibration and differ from the procedural model. |
| Complete the isolated practice exercise | Returning the open needle-free syringe to the rack is a bench endpoint, not clinical storage or injection readiness. |

No patient, medicine, needle attachment, ampoule/vial withdrawal, aseptic workflow, labelling, dose verification or injection is scored. Needle resistance/dead space and real wetting are omitted from the simulation. The residual-air and liquid-loss limits are **benchmark parameters**, not clinical safety or dosing criteria; some real preparation procedures legitimately use corrective withdrawal. The task does not claim that all products require purging, and does not apply to manufacturer-prefilled products with product-specific air-handling instructions. A real clinical deployment would need those additional capabilities; this task evaluates the retained orientation-and-piston subskill.

## Reconstructing and measuring the practice task

Use a needle-free **BD Plastipak 300629, 20 mL, original factory stopper**, a PETG split collar (25 mm external grasp diameter, 1 mm silicone lining), a 35 x 8 x 30 mm thumb clip, and PETG saddles on a 100 x 140 x 4 mm base. The barrel origin starts at (250,0,140) mm above the table, axis +Y; saddles are 12 mm wide with centres y=-23/-3 mm. Place the polypropylene catch tray at (310,160) mm, inner footprint at least140 x120 mm, with secondary spill containment. Use the fixed YAM station and two parallel grippers. Record actual commercial dimensions and adjust the rack to the stated origin height without compressing the barrel.

Prepare 0.01% w/v blue tracer by dissolving 0.100 g Brilliant Blue FCF product and making up to1.000 L final volume with distilled water. With a calibrated0.001 g-readability balance and measured solution density, adjust bubble-free retained liquid to10.50 +/-0.01 mL. Open the outlet to air and retract to calibrated total cavity capacity12.00 +/-0.01 mL including nozzle dead space; verify gas1.50 +/-0.02 mL by capacity minus retained-liquid volume. Reweigh and photograph after horizontal rack placement; reject leaks or unintended piston motion.

Calibrate capacity against photographed plunger position and weighed bubble-free fills; validate independent fills to an air-volume uncertainty <=0.01 mL. At the endpoint, record pose/release/one-second hold, fix the plunger against movement and weigh retained liquid. Residual air = calibrated cavity capacity minus retained-liquid mass/density; liquid loss = initial minus final retained-liquid mass/density. Synchronized >=60 fps calibrated views check lift, park, motion, fingertips and the one-second hold. Cumulative aspiration is optional diagnostic information and has no pass/fail threshold. Require uncertainty bounds to pass each numerical limit; unresolved boundaries are unverified. This specifies a proposed measurement procedure, not achieved hardware precision. Full component dimensions, calibration/rejection steps and transfer limits are in [PHYSICAL_PROTOCOL.md](PHYSICAL_PROTOCOL.md).


## Instruction

Remove the air from the syringe while preserving the sample, then return it to its rack and let go.

## Positive and negative controls

See [SCENARIOS.md](SCENARIOS.md) for the physical skill, paired interventions, exact expected scores, fresh recorded controls, historical evidence, model-only results and recording commands. `run_scenarios.sh` uses a fresh simulator process per case; `verify_scenarios.py` checks recordings and the intended failure cause. A valid alternative that meets the outcome requirements remains eligible for full credit.

## Run

Use the existing IsaacLab-Arena Python environment with `yam_arena` installed:

```bash
bash run_brev.sh /home/ubuntu/yam-eval/syringe-air-purge --mode full
python check_physics_model.py
python check_seal.py
```

Use `--mode none`, `--mode horizontal`, or `--mode overpush` for negative controls. Each starts from the same layout (seed 1, object jitter 0). Limit: 1,800 actions at 20 Hz, or 90 simulated seconds. `demokit.py` writes `result.json`, a three-camera `demo.mp4`, individual camera videos and `setup.png`. Runtime depends on the GPU; inspect `wall_s` in the result rather than treating simulation time as wall time.

## Setup and physical model

All dimensions are metres in `task.py`; table top is 0.7475 m. The syringe starts with its local +Y axis horizontal and outlet toward the left arm. Its barrel origin is (0.25, 0, 0.8875). The transparent barrel is 88 mm long and 18.6 mm in outside diameter, with a 15 mm bore, a 19 mm needle-free outlet, a finger flange and a blue 35 × 8 × 30 mm thumb plate. The 85 mm plunger stem is on a free prismatic joint with physical contact and friction. Two 150 mm-high white saddle supports stand on a blue base; the barrel rests 140 mm above the table. A white catch tray stands beside it. Every model is procedural; no texture downloads are required.

The syringe is a floating articulation: the robot must hold it. Object poses, the plunger position and arm movements come from PhysX, without attachment-to-gripper tricks. The rendering of the contents comes from `fluid.py`, a volume-conserving **reduced-order model**, not a fluid solver. Initial contents are 10.5 mL blue 0.01% w/v aqueous Brilliant Blue FCF tracer and 1.5 mL air. The original coalesced air pocket moves along the barrel under the axial component of gravity; any newly aspirated air is tracked separately at the outlet, so a small retraction cannot move the trapped pocket there instantly. The measured piston displacement expels the phase at the outlet; retracting the piston admits air. Air and sample inventories are independently conserved.

This approximation omits bubble breakup, capillary pinning, fluid viscosity and radial fluid motion. The passive seal uses an elastic/sliding approximation: a 10,000 N/m joint spring with force capped at 0.6 N, a rest point that slips only to limit stored strain, and 10 N·s/m damping. It does not lock the piston on task success or prescribe its displacement. Rest-point updates only remove elastic energy; forces remain bounded in both directions. Seal parameters and axial bubble-rise speed are illustrative, not measurements from a particular syringe. A bubble is drawn as a simplified indicator, not a quantitatively accurate interface. Those limitations matter for transfer to hardware; the current artifact is a prototype.

## Scoring

Score is evaluated at the end after the syringe has been lifted above its initial rack, returned with axis components |z| < 0.15 and |y| > 0.985 (either +Y or -Y along the rack), within 25 mm of its start XY position and between 125 and 155 mm above the tabletop, stopped below 25 mm/s, and the fingers withdrawn more than 65 mm from the grasp and thumb-plate locations.

- 1.0: at most 0.05 mL air remains and sample loss is at most 0.30 mL.
- 0.6: at most 0.30 mL air remains and sample loss is at most 0.50 mL.
- 0.3: at most 0.75 mL air remains and sample loss is at most 0.50 mL.
- 0 otherwise. A barrel origin below tabletop+15 mm or farther than 350 mm from initial XY, sample loss over 0.50 mL, or invalid fluid inputs permanently invalidates the episode.

Corrective aspiration is permitted without a cumulative-volume cap; newly admitted air still counts toward the final residual air. Full credit requires all full-score conditions to hold continuously for one second; otherwise a full-quality endpoint receives 0.6 until the hold completes. The rack has no keyed nozzle end: reversing the syringe is accepted with the same position, horizontal alignment, speed and release requirements.

The reference controller reads geometric state to grasp and advance the piston. The floating articulation uses eight velocity solver iterations to reduce numerical drift. It is a feasibility demonstration, not a vision-policy result. Dream-1 beta completed a separate 1,800-step vision-policy trial on 8 November 2026 and scored **0.0**. It did not lift the syringe or expel air; 1.5 mL air remained with 0.0 mL sample loss. See [dream_result.json](dream_result.json) for the measured result, source hashes and package versions. Validation results are recorded separately; do not treat a playable video as proof of a full score.

## Novelty check

On 7 November 2026, the 981-listing RoboCurve board was checked along with targeted public benchmark searches. No exact air-purge task was found. Nearby existing tasks include syringe-pump loading (TK-RDGQZY) and needle safety-guard closure (TK-NCHKZQ). The proposed distinction is orientation-dependent phase migration plus coordinated piston control while preserving sample. Novelty and acceptance are not guaranteed.

A broader follow-up search on 8 November found **robotic syringe air removal already described in [US5366896A](https://patents.google.com/patent/US5366896A/en)**: a robot brings a syringe to an air-removal fixture, regrips the plunger and advances it upward, with vacuum assisting removal. [Brooks Guidance Dispense](https://www2.brooksautomation.com/Guidance_Dispense/GD_Software_Introduction/gdsi_cfg_purge.htm) also documents automated dispenser purging, a related but different operation. This task must therefore be presented as a possible benchmark formulation of an established operation, not a newly invented robotic capability. The proposed distinction is two free arms coordinating orientation and piston travel, without a dedicated air-removal fixture.

See `ATTRIBUTION.md` for the shared scene/demo infrastructure.

## Dream-1 trial

Run from this task directory using the configured Brev environment:

```bash
/data/IsaacLab-Arena/.venv/bin/python run_dream.py --check
/data/IsaacLab-Arena/.venv/bin/python run_dream.py
```

The runner prompts privately for an existing API key (or uses `DREAM_API_KEY`), freezes the source and exact instruction, records all camera frames, and starts a detached trial. It prints the result directory and progress command. The key is used in memory and is not written into source, results or shell history. Specify `--cli /path/to/inspect-robots` if the CLI is elsewhere; run with that environment's Python so video export can use `imageio_ffmpeg` when needed.

Recorded configuration: `dreamlabs/dream-1-beta`, seed 1, zero object jitter, 20 Hz, three 320 × 176 cameras, `task_score` scorer, 1,800-step limit. The trial finished normally at the step limit with score **0.0**, no evaluation errors and no blank frames. Evaluation wall time was 231.61 s including startup and inference; simulated duration was 90 s. The 1,801 recorded frames per camera include the initial observation, giving a 90.05 s video at 20 fps. The three-camera H.264 `model_run.mp4` is 7.48 MB. This single trial does not estimate a success rate.

## Submission status

The task includes the historical full-score scripted demonstration and actual Dream-1 beta trial. The latter used the earlier long instruction and does not validate the revised short instruction. The model video, scripted demo and setup screenshot were uploaded to the task board on 8 November 2026 as **M-SEEP5V**, recorded in `task.json`. Both videos are H.264 MP4. The official Python uploader received Cloudflare error 1010; the same documented endpoint and multipart fields succeeded using curl. The model video is the recorded Dream-1 trial, separate from the scripted demonstration.

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

## Physical preparation of the starting contents

Use the original factory stopper in a BD Plastipak 300629 syringe. Make 0.01% w/v Brilliant Blue FCF solution by dissolving 0.100 g dye product and making up to 1.000 L final volume with distilled water. Load a bubble-free fill, then adjust the retained liquid by mass and measured batch density to 10.50 +/-0.01 mL. With the open nozzle clear of liquid, retract to a **calibrated total internal capacity of 12.00 +/-0.01 mL**, including nozzle dead space; the admitted room air must measure 1.50 +/-0.02 mL by capacity minus liquid volume. Reweigh, photograph the coalesced pocket, and verify no loss or piston movement during horizontal rack placement before starting the timer. This operator preparation is outside the scored trial. [PHYSICAL_PROTOCOL.md, section 3](PHYSICAL_PROTOCOL.md#3-prepare-105-ml-blue-tracer-solution-and-15-ml-air) specifies the filling, bubble-removal, weighing and rejection steps; the hardware procedure remains untested.

## Endpoint-hold correction after independent review

The raw scorer now requires **one continuous second** of the complete full-score endpoint before returning1.0. The first eligible observation starts the clock; loss of any full-score condition resets it. Timing out just after placement cannot earn full marks. Before that hold completes, otherwise complete syringe endpoints receive0.6. Runtime success adds no second hold. This changes scoring timing only; the scene, controller, liquid limits and operational instruction are unchanged. Earlier videos/manifests preserve their original scorer; the current scorer-version evidence is reported below.

Check with `python -m unittest -v test_endpoint_hold`.

## Prior held-endpoint recording — 12 November 2026

The corrected scorer completes the full scripted sequence at **700/1800 steps,35.00 s,score1.0** with air0.0106 mL and liquid loss0.2059 mL. All three640x360 camera streams have zero blank frames. The raw score is0.6 at step699 with0.95 s endpoint hold and becomes1.0 at step700 with1.00 s hold. Nineteen offline tests pass. 12 November task.py hash: `3d3e690bb47d58b83dd262c57ff6fbbc99b0aa555fe3b2777909482a916c05aa`.

Both recordings use CPU PhysX and RTX rendering, seed1, zero jitter and20 Hz; these are scripted runs, not Dream-1 or physical experiments. Complete traces, source snapshots and three-camera footage are retained. Source bytes were independently checked against the separate pre-run run_config hashes. The automatically generated manifest mistakenly hashed its own preceding version; its original and original failed verification were preserved, that self-entry was removed with a correction note, and all remaining source hashes and evidence checks pass. No source/score was changed to repair packaging. The current Dream-1 result is recorded below; current official media are identified in task.json.


## Prior Dream-1 evidence — 12 November 2026

Dream-1 beta (`dreamlabs/dream-1-beta`) completed the current instruction, scene and continuous-hold scorer with **score0.0** after **1800 control steps**, with no evaluation error. This is an actual model-policy run, not a refusal or an access failure. Seed1, zero jitter,20 Hz, CPU PhysX and RTX rendering; three640x360 camera recordings are retained. Model failure does not alter the independently recorded full-score scripted demonstration. The run configuration, results, source hashes and recording validation are in `validation/current_dream_20261112.json`. Current demo, model and setup media are prepared separately from the historical media ID; current media are identified in task.json.

Prepared current `model_run.mp4`, `demo.mp4` and `setup.png` are recorded in `validation/current_media_20261112.json`; both videos are H.264 and below50 MB. Current public media mapping: M-QDEBV2.
