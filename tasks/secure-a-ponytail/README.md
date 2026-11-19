# Secure a ponytail with a plain hair elastic

Gather the synthetic hair on the practice head into a ponytail and secure it with the plain elastic. Leave it held after both hands let go.

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

Use a rigid adult practice head approximately 180 × 156 × 230 mm. An inexpensive [Marianna polystyrene head with a face](https://www.sallybeauty.com/salon-supplies/other-professional-supplies/beauty-student-supplies/styrofoam-head-with-face/SBS-200201.html) is a suitable starting form; measure it and pad the scalp with firm foam if needed to match each dimension within 10 mm. Clamp its mounting base firmly to the table. The simulator uses a rigid approximation of this supported head.

Prepare the hair once from an ordinary pack of straight synthetic braiding hair. Weigh twelve **3.0 ± 0.1 g** bundles on a 0.1 g scale and trim their free length to **245 ± 5 mm**. Bind only the uppermost 10 mm of each bundle with thread and adhesive; do not join the free lengths. Fix these twelve roots individually to a thin flexible scalp backing, then bond that backing flush to the rear of the head. No backing, glue or thread may extend into the free hair. A root must withstand a 2 N pull before use; replace any loose root.

Place the roots on a 72 mm-wide rear scalp patch. In the head frame, root lateral coordinates are `(s − 5.5) × 6.5 mm` for `s = 0..11`; their heights are `35 + 15 cos(s) mm` above the head centre. A ruler-marked paper template makes these positions reproducible within 2 mm. Root coordinates in millimetres (left/right offset, height above centre): (-35.8, 50.0), (-29.2, 43.1), (-22.8, 28.8), (-16.2, 20.2), (-9.8, 25.2), (-3.2, 39.3), (3.2, 49.4), (9.8, 46.3), (16.2, 32.8), (22.8, 21.3), (29.2, 22.4), (35.8, 35.1). The front-to-back coordinate follows the scalp surface. Brush the twelve locks straight and lay them separately in their photographed loose arrangement; **do not tie, braid or join them**. Weigh/prepare before fixing roots, so no trial requires extracting hair from a commercial rooted head.

The head faces away from the robot. Its centre is (390, 0, 1052.5) mm; the elastic starts at (270, 0, 810.5) mm on two secured wooden rests. X points forward, Y left, and the table is 747.5 mm high. Restore the loose hair and separate band each trial. The synthetic locks and contact envelopes are a coarse grouped-fibre model, not individual hair strands.

The finger extensions have 40 mm working length, 24 mm width and 3 mm thickness.
Both arms use their ordinary mounting height and 40 N finger actuator limits.

## Scoring and material model

After the robot releases, wait three seconds, then pull the elastic gently towards the hair ends with a spring scale: ramp to **1 N over 0.5 seconds**, hold **3 seconds**, then unload over **0.5 seconds**. Hook the scale through the elastic without catching hair; the evaluator must not support the band. Film the scale and band beside a ruler. The band may move **at most 10 mm** from its starting position throughout this test. A loose ring that slips down the hair fails, even if it previously rested in place.

All credit requires passing this retention test, intact roots/band, and the band centre at least 55 mm behind and 25–160 mm below the head centre. Count locks remaining through the band after unloading: eleven or twelve earn 1.0, nine or ten 0.5, six to eight 0.25, fewer zero. Full credit also limits the band's maximum width to 50 mm. Other outcomes score zero. The load check is a repeatable gentle-tug criterion, not a claim about every hairstyle or elastic product.

The simulator applies the same downward load, divided over native band contact patches, after three seconds with clear hands and after the band leaves its starting rest. It ramps/holds/unloads without writing material positions. Touching again resets the test. The evaluator force stands in for the scale; the test is recorded in the ordinary native trial and CPU-audited from its state history.

Articulated 5 mm contact envelopes group fine hair fibres into flexible locks.
A native elastic surface models a 30 mm inside-diameter, 3 mm cord silicone band;
its stiffness is approximate. The starting rests are 33 mm apart. These dimensions
are part of the trial setup, not a measured retail-product calibration.

A geometry-gated native vertex attachment represents finger friction only after
closing pad faces enclose a band patch. It holds only material already inside the
central 8 mm of the 24 mm-wide face, follows the real gripper link, and releases on opening, 8 mm slip
or estimated local cord tension above 6 N. This tension estimate is uncalibrated;
it is not a measured native force limit. Local held-patch contacts are filtered
while the attachment acts, to avoid conflicting constraints. Native hair contacts,
root failures and band over-stretch remain active. No hair or band pose is assigned
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

The revised native demo scores **1.0 in 800 actions**: 11 of 12 locks retained, 33.59 mm maximum band width, 9.5 seconds with clear hands and **1.43 mm maximum slip** through the full 1 N retention test. The submitted video labels the evaluator pull so it is not mistaken for robot motion.

The paired Dream-1 run scores **0.0 in 1,800 actions**. It made 73 API attempts across 37 sessions, including 36 SDK-reported expiry recoveries; this is a recovered-session baseline, not an uninterrupted session. It did not pick up the elastic. The transcript records requests, limit clipping and tracking error.

Wall runtimes were 20.8 minutes for the demo and 34.3 minutes for Dream on an RTX PRO 6000 Blackwell GPU sharing validation work. Both full-frame native runs pass frozen-source CPU audits. Seven score-boundary checks reject missing proof, excess slip and damage, and verify partial-credit lock counts. Front, rear and side views were reviewed. Reports, trace hashes, source hashes and session diagnostics are in `evidence/`; runtime versions are in `ENVIRONMENT.json`.
