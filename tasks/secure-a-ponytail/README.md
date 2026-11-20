# Secure a ponytail with a plain hair elastic

Secure the prepared, untied synthetic hair on the mannequin head with the plain elastic. Leave it held after both hands let go. This task tests fastening; gathering a full head of hair is outside its scope.

## Run and score

Use the Python 3.12 / Isaac Sim 6.1 / YAM runtime pinned in `ENVIRONMENT.json`.
An NVIDIA GPU is needed for native rendering. From the repository root:

```sh
OMNI_KIT_ACCEPT_EULA=YES python tasks/secure-a-ponytail/run.py --mode demo --out /tmp/secure-a-ponytail-demo
OMNI_KIT_ACCEPT_EULA=YES python tasks/secure-a-ponytail/run.py --mode dream --out /tmp/secure-a-ponytail-dream
python tasks/secure-a-ponytail/rescore_native_precision.py /tmp/secure-a-ponytail-demo
```

Dream receives the ordinary RGB cameras and joint state through Inspect Robots.
The SDK resolves its credential privately. Use one episode per key at a time.
The fixed layout has no random jitter or seed. The limit is 1,800
controls at 20 Hz. Demo and Dream use identical physics and hardware; the demo
commands only the robot joints and grippers.

Each run saves its source, complete state trace, camera recordings and report.
Rescoring checks source hashes and recomputes outcomes from the recorded native
states on CPU; it is not a new physics simulation. Camera strides above 1 are
for diagnostics and are excluded from submission evidence.

## Setup and reset

Use a rigid adult practice head approximately 180 × 156 × 230 mm. An inexpensive [Marianna polystyrene head with a face](https://www.sallybeauty.com/salon-supplies/other-professional-supplies/beauty-student-supplies/styrofoam-head-with-face/SBS-200201.html) is a suitable starting form; measure it and pad the scalp with firm foam if needed to match each dimension within 10 mm. Use a rigid plastic stand 76 mm in diameter and 190 mm high on a 150 × 150 × 15 mm base; clamp the base firmly to the table. The simulator uses a rigid approximation of this supported head.

Prepare the hair once from an ordinary pack of straight synthetic braiding hair. Weigh twelve **3.0 ± 0.1 g** bundles on a 0.1 g scale and trim their free length to **245 ± 5 mm**. Bind only the uppermost 10 mm of each bundle with thread and adhesive; do not join the free lengths. Fix these twelve roots individually to a thin flexible scalp backing, then bond that backing flush to the rear of the head. No backing, glue or thread may extend into the free hair. A root must withstand a 2 N pull before use; replace any loose root.

Mount a 24 × 40 × 14 mm firm-foam root backing at the rear of the head, centred 98 mm behind and 42 mm above the head centre. Fix the roots separately to its lower face in two rows of six, 5.4 mm centre spacing in both directions. The root centres span 27 mm sideways and 5.4 mm front to back. The twelve root heights are 35 mm above head centre; the backing supports only the bound root ends. Comb the free lengths into a compact, straight downward section and remove the comb before reset. No clip, glue, thread or support holds the free hair together. The elastic starts separate, with zero locks enclosed.

The head faces away from the robot. Its centre is (390, 0, 1052.5) mm; the elastic starts at (270, 0, 810.5) mm on two secured 7 × 7 × 62 mm printed-polymer rests. Each top has a 4 × 4 mm open-top pocket, 4 mm deep. The band rests freely on these saddles; there is no clip, adhesive or attachment. Lift it clear before stretching. X points forward, Y left, and the table is 747.5 mm high. Restore the loose hair and separate band each trial. The synthetic locks and contact envelopes are a coarse grouped-fibre model, not individual hair strands.

The finger extensions have 40 mm working length, 24 mm width and 3 mm thickness.
Both arms use their ordinary mounting height and 40 N finger actuator limits.

## Scoring and material model

After the robot releases, wait three seconds, then pull the elastic gently towards the hair ends with a spring scale: ramp to **1 N over 0.5 seconds**, hold **3 seconds**, then unload over **0.5 seconds**. Use a light two-point thread sling under opposite sides of the elastic, without catching hair, and pull the sling downward along the hair with the scale. The evaluator must not support the band. Film the scale and band beside a ruler. The band may move **at most 10 mm** from its starting position throughout this test. A loose ring that slips down the hair fails, even if it previously rested in place.

All credit requires passing this retention test, intact roots/band, and the band centre more than 55 mm behind and between 25 and 160 mm below the head centre (endpoints excluded). Count locks remaining through the band after unloading and award the highest met tier: at least eleven locks and maximum band width ≤50 mm earns 1.0; otherwise at least nine locks earns 0.5, at least six earns 0.25, and fewer earns zero. Other outcomes score zero. The load check is a repeatable gentle-tug criterion, not a claim about every hairstyle or elastic product.

The simulator applies the same downward load, divided over native band contact patches, after three seconds with clear hands and after the band leaves its starting rest. It ramps/holds/unloads without writing material positions. Touching again resets the test. The evaluator force stands in for the scale; the test is recorded in the ordinary native trial and CPU-audited from its state history.

Articulated 5 mm capsules with overlapping rounded ends group fine hair fibres into flexible locks. The native solver uses 960 Hz physics under the same 20 Hz robot controls; this improves thin-body contact resolution.
A native elastic surface models a 16 mm inside-diameter, 3 mm cord silicone band;
its stiffness, simplified 1 g mass and internal damping are approximate. Internal elasticity damping is 0.15 to reduce high-frequency recoil; it does not apply drag to rigid translation of the band. The starting rests are 19 mm apart. These dimensions
are part of the trial setup, not a measured retail-product calibration.

A geometry-gated native vertex attachment represents finger friction only after
closing pad faces enclose a band patch and the remaining jaw gap is no more than 0.75 mm wider than the cord. It holds only material already inside the
24 mm-wide face, follows the real gripper link, and releases on opening, 8 mm slip
or estimated local cord tension above 18 N. This tension estimate is uncalibrated;
it is not a measured native force limit. Local held-patch contacts are filtered
while the attachment acts, to avoid conflicting constraints. A continuous native surface and 32 cord capsules supply hair/head contact. The simulation strip is 1.5 mm wide with a 0.75 mm contact rest offset on each side, staying inside the 3 mm radial cord envelope. Its modulus is doubled to 1.2 MPa to preserve the earlier axial stiffness with half the strip width; material thickness remains 3 mm. These are equivalent-strip parameters, not a calibrated silicone measurement. Root failures and band over-stretch remain active. No hair or band pose is assigned
during a trial; the approximation supplies no automatic gathering or fitting.

The CPU audit verifies source hashes and every action, then independently
reconstructs release time, the pull-force schedule, maximum band movement,
retained-lock count and final score from the frozen state trace. Surface nodes
are restored at their native float32 precision. Complete traces and sources
are preserved; the audit does not rerun or alter the physics.

## Assets

All task geometry is original procedural work under the included MIT license.
The robot, table and runtime come from the pinned YAM / Isaac environment.
No third-party task meshes, scans or textures are used.

## Validation

The scripted demo completes 1,040 controls with score 1.0. After release and the actual 1 N pull, 12 locks remain secured; maximum band movement is 3.26 mm. The matching-source Dream-1 baseline completes 1,800 controls with score 0.00. Native wall times on an L40S are 26.8 minutes for the demo and 40.8 minutes for Dream.

The frozen trace audits reconstruct the score and force schedule independently. The contact audit verifies captures against the actual pad faces and checks continuity of the cord contact envelopes. Both videos retain all frames at 20 Hz and show synchronized close and front views. The evaluator pull is labelled. Evidence JSON files record the source, trace and media hashes.

The evaluator load uses the native permanent-wrench API. All evidence is from the current validated simulation.

The Dream baseline made 73 API requests across 37 reported sessions, with 36 expiry recoveries. These continue one physical episode; see `evidence/dream-run.json` and `evidence/dream-transcript.json`.
