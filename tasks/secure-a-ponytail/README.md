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

Prepare the hair from straight synthetic braiding hair of twelve distinct solid colours, all of the same fibre type and nominal fibre diameter. Use one colour per bundle; do not tie or glue the free lengths. Number the roots 1–12 on the fixed backing. In increasing Y coordinate within each row: the nearer row is black, white, red, orange, yellow, lime; the farther row is forest green, cyan, blue, violet, magenta, pink. Keep a labelled sample of each colour beside the evaluator’s photograph. Colour identifies a bundle even when its loose fibres intermingle; colour selection is not part of the robot’s objective. Prepare twelve **245 ± 5 mm free lengths**, each weighing **3.0 ± 0.1 g** on a 0.1 g scale. Allow an additional 10 mm at the root for binding, outside that free-length measurement; binding material is excluded from the free-hair mass. Bind only the uppermost 10 mm of each bundle with thread and adhesive; do not join the free lengths. Fix these twelve roots individually to a thin flexible scalp backing, then bond that backing flush to the rear of the head. No backing, glue or thread may extend into the free hair. A root must withstand a 2 N pull before use; replace any loose root.

Mount a 24 × 40 × 14 mm firm-foam root backing at the rear of the head, centred 98 mm behind and 42 mm above the head centre. Fix the roots separately to its lower face in two rows of six, 5.4 mm centre spacing in both directions. The root centres span 27 mm sideways and 5.4 mm front to back. The twelve root heights are 35 mm above head centre; the backing supports only the bound root ends. Comb the free lengths into a compact, straight downward section and remove the comb before reset. No clip, glue, thread or support holds the free hair together. The elastic starts separate, with zero locks enclosed.

The head faces away from the robot. Its centre is (390, 0, 1052.5) mm; the elastic starts at (270, 0, 810.5) mm on two secured 7 × 7 × 62 mm printed-polymer rests. Each top has a 4 × 4 mm open-top pocket, 4 mm deep. The band rests freely on these saddles; there is no clip, adhesive or attachment. Lift it clear before stretching. X points forward, Y left, and the table is 747.5 mm high. Restore the loose hair and separate band each trial. The synthetic locks and contact envelopes are a coarse grouped-fibre model, not individual hair strands.

The finger extensions have 40 mm working length, 24 mm width and 3 mm thickness.
Both arms use their ordinary mounting height and 40 N finger actuator limits.

## Scoring and material model

After the robot releases, wait three seconds, then pull the elastic gently towards the hair ends with a spring scale: ramp to **1 N over 0.5 seconds**, hold **3 seconds**, then unload over **0.5 seconds**. Use a light two-point thread sling under opposite sides of the elastic, without catching hair, and pull the sling downward along the hair with the scale. The evaluator must not support the band. Film the scale and band beside a ruler. The band may move **at most 10 mm** from its starting position throughout this test. A loose ring that slips down the hair fails, even if it previously rested in place.

All credit requires passing this retention test, intact roots/band, and the band centre more than 55 mm behind and between 25 and 160 mm below the head centre (endpoints excluded). A lock counts only if **at least 90% of its free-hair mass** remains threaded through the elastic after unloading. Loose fibres outside the loop do not count, even if they touch it. Apply the physical counting procedure below and award the highest met tier: at least eleven locks and maximum band width ≤50 mm earns 1.0; otherwise at least nine locks earns 0.5, at least six earns 0.25, and fewer earns zero. Other outcomes score zero. The load check is a repeatable gentle-tug criterion, not a claim about every hairstyle or elastic product.

### Physical counting after the trial

Record the final band position, width, retained state and damage before touching anything. Keep the band in that recorded position during inspection. Using the unique fibre colours, separate each colour's strands into those passing through the loop and those outside it at the elastic plane; inspect from both sides with a phone close-up and a blunt needle. Do not move a fibre across the loop while classifying it. Temporarily clip the two identified groups **only after the trial has ended** and photograph them before cutting. Cut each bundle immediately below its bound root, then cut the elastic to recover the already classified free hair. Weigh the captured and escaped portions of each colour separately on a **0.01 g scale**, using tared dishes. Retained fraction is captured mass / (captured + escaped mass). Use the displayed masses without rounding the resulting fraction. A fraction of **0.900 or greater** counts as one lock; lower fractions count as zero. Record all twelve masses and fractions. Replace the consumed hair and elastic before another trial. Deliberate evaluator cuts after the recorded outcome are not task damage; pre-existing robot damage still scores zero. This inspection uses no ties or sleeves during the robot's attempt.

The simulated counterpart tests 72 equal-mass fibre centrelines distributed through each native lock, using the same offsets as its rendered fibres. At least **65 of 72** must cross the interior of the elastic to count (64/72 is below 90%). It reports all twelve captured fractions. These samples move together with their articulated lock; independently sliding individual fibres are not simulated. This is a disclosed approximation of the mass-fraction test, not a claim of strand-level physics.

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

## Revision history

Earlier submission **TK-VJ4WRE**, PR [#2860](https://github.com/robocurve/benchmark-tasks/pull/2860), received three findings: a loose elastic could pass without gripping; width and three-second dwell did not prove secure retention; and preparation of twelve separate 3 g locks was unspecified. The current task adds the 1 N/3-second pull with a 10 mm maximum slip, and specifies an affordable head, individually rooted loose hair, masses, lengths and root template.

Its first revision, TK-VT4XA6, was withdrawn after a control test showed that its old USD force API did not apply the claimed load. The replacement, **TK-REXTLY / #3224**, uses the native permanent-wrench API and replaced both videos; the historical withdrawn load result is not evidence for this task. Its prepared compact section is explicitly untied and tests fastening, not gathering a whole head of hair.

The latest review requested distinguishable bundles and a numerical definition of partial capture. This revision adds twelve whole-fibre colours and the 90% captured-mass rule, with matching sampled-fibre scoring. It retains the native pull test and all existing damage, position and release requirements.

## Validation

The revised native demo completes **1040 controls, score 1.0**. It retains 12/12 bundles under the 90% rule after the actual 1 N pull; maximum band movement is 3.61 mm. The matching Dream baseline completes **1800 controls, score 0.00**. Frozen-source rescoring, contact-location checks and all-frame video checks pass. Native wall time on the shared L40S was 31.4 minutes for the demo and 51.2 minutes for Dream.

Review media pairs synchronized, uncut rear and front views; a trace-driven caption identifies the evaluator pull while force is applied. See the metadata clarification record for wording updates made after recording.

Dream ran through the official loopback SSH bridge, with its credential kept on the local computer. Its client made 73 API attempts across 74 sessions, including 36 expiry recoveries in one uninterrupted physical episode. These are recorded in `evidence/bridge-runtime.json`; earlier videos are superseded.
