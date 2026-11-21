# Finish plucking a prepared chicken breast

Pluck the remaining feathers from the breast without tearing the skin. Leave the carcass in the tray and release it.

This is the hand-finishing stage of poultry preparation, after most feathers have been removed. The robot must grip the fine shafts, support the yielding skin and extract the feathers. It may take individual feathers or small bunches. Cutting feathers off, breaking their shafts or pulling out skin cannot earn credit.

## Physical setup and reset

Order fresh, eviscerated, unplucked small broiler chickens from a poultry processor; an ordinary fully plucked supermarket chicken cannot be reset for this task. Use one supplier and batch, carcass mass 1.0–1.3 kg, approximately 300–360 mm long, 160–200 mm across folded wings and 110–140 mm breast depth. Head and feet are removed before delivery. Do not use a live animal, wild bird, frozen carcass, glued feathers or reinserted quills.

A trained operator prepares specimens away from the robot. Use a 60 ± 1 °C water bath for 60 ± 5 seconds, keeping the carcass submerged and gently agitating it, then pluck the carcass except nine undamaged natural breast contour feathers, each 25–45 mm long. Choose three small groups of three along the breast, approximately 25–35 mm between group centres and no more than 12 mm across each group. Do not loosen the selected feathers while preparing adjacent ones. Photograph all nine and their entry sites with a ruler. Reject specimens with pre-existing torn breast skin, broken selected shafts or a group hidden under a wing. Record batch, measured bath temperature, immersion time and preparation time. Chill prepared carcasses promptly in the processor’s food-safe chilled preparation area; do not prepare them on the robot station. On one spare from the batch, check that the same covered fingers can extract these feathers without tearing skin; reject the batch if the feathers fall out without a grip or cannot be plucked within the gripper limit. This is a proposed physical preparation protocol, not a completed physical calibration.

Keep prepared carcasses bagged at 0–4 °C. Remove one at a time, blot standing water without rubbing the feather roots, and start within ten minutes. Complete the attempt and inspection within thirty minutes out of chilling. Use a fresh specimen per trial and discard trial poultry; it is not served. Operator preparation, chilling and inspection are outside the robot action limit.

The loose carcass lies breast up in a fixed 420 × 380 × 35 mm polypropylene tray, centred 360 mm in front of the station origin. Its long axis follows Y; the legs point +Y. The table is 747.5 mm high. The tray has a non-slip base; no clamp or support holds the carcass. The simulation uses a nominal 1.05 kg body, folded wings and legs, with nine feathers in three groups. `geometry.py` gives the exact starting layout.

Both arms have flat stainless-steel finger extensions with silicone contact covers: 40 × 18 × 2 mm working faces on 100 × 18 × 2 mm mounting stems, at a 218 mm TCP and a 4 N closing-force limit. All exposed edges are rounded. Snug waterproof disposable sleeves cover the grippers and lower wrists and overlap under the removable food-safe contact covers. No loose sleeve material enters the jaws. A 600 × 450 × 20 mm polypropylene outer tray and a 600 × 200 mm clear polycarbonate rear splash screen protect the shared station. Centre the outer tray under the working tray; place the screen behind it at X = 680 mm, outside robot paths. Only covered fingers contact poultry. A manual preflight checks grip and cover integrity on a spare specimen before any robot run. Replace covers and sleeves for each specimen; remove them before touching the robot. Clean and sanitise the removable fingers, tray and barriers using a food-contact-compatible product according to its label, then dry. Stop on any puncture or contamination beyond containment. Never spray cleaning liquid at the robot. The peripheral splash screen, outer containment and wrist sleeves are physical-rig requirements; they are not rendered in the native scene. The working tray and covered finger contacts are simulated.

This protection addresses the shared-rig concern raised on raw-chicken task #1963. It is not evidence that this setup has already passed a physical hygiene or material-repeatability trial. The current review prompt permits protection or a faithful safe stand-in for contaminating materials.

## Scoring

After both open hands stay at least 50 mm clear for three seconds, award **1/9 per whole feather extracted**, including its calamus (the part originally embedded in the skin). Feathers may rest anywhere in the tray; placing them in a particular receptacle is not the skill being tested. Full score means all nine are out, the skin is intact and the carcass remains breast up in the tray.

Any new skin tear extending an entry opening by **3 mm or more**, detached skin, or broken shaft retained in the skin gives **zero for the whole attempt**. A carcass outside the tray, on its other side or still held, or an extracted feather outside the 420 × 380 mm working tray, also gives zero. Compare close before/after photographs, inspect the nine entry sites and recovered feathers, and use a ruler. A feather that remains attached or has a missing calamus is not counted. Do not award credit for feathers removed together with a piece of skin. Small pre-existing follicle openings are not tears.

The simulation counts nine independently extracted calami. Breaking a shaft or skin attachment, or more than 4 mm displacement of a skin sector, invalidates the attempt. The displacement rule is a coarse damage proxy, not a real-trial measurement requirement. The real judge uses visible tears and retained broken shafts.

## Material approximation

The loose carcass is rigid except for twelve yielding breast-skin sectors. Each feather has a lower quill, a bending upper shaft, finite axial follicle resistance, and a breakable shaft connection. The root releases only after its embedded length has physically slid out. Forces pass through the skin and carcass. Each grasp requires opposed finger contact at a feather shaft and is a finite compliant native joint: it slips, releases when fingers open, and does not reposition the feather. Several feathers may be grasped independently in the same closed fingers.

Feather barbs are visual-only and follow their shaft; their easy compression and individual interlocking are not resolved. The shaft colliders, root retention, skin deformation and breakage are physical. Retention, tissue and friction values are illustrative and uncalibrated. No complete carcass deformation, contamination, cover puncture or scalding physics is simulated. These limitations apply equally to the demo and Dream. The scripted policy commands robot joints and uses observed poses and extraction state to guide the arms; it never assigns task-object poses.

Hand plucking and the effect of scalding on feather retention and skin damage are described by [University of Minnesota Extension](https://extension.umn.edu/agriculture/animals-and-livestock/poultry/home-processing-of-poultry). [FDA cold-storage guidance](https://www.fda.gov/consumers/consumer-updates/are-you-storing-food-safely) informs the handling controls. These sources establish the real procedure, not calibration of the simulator.

## Run and verify

Use the pinned Python 3.12 / Isaac Sim 6.1 / YAM environment in `ENVIRONMENT.json`.

```sh
python tasks/pluck-a-prepared-chicken/make_plan.py
OMNI_KIT_ACCEPT_EULA=YES ACCEPT_EULA=Y python tasks/pluck-a-prepared-chicken/run.py --mode demo --out /tmp/pluck-demo
OMNI_KIT_ACCEPT_EULA=YES ACCEPT_EULA=Y python tasks/pluck-a-prepared-chicken/run.py --mode dream --out /tmp/pluck-dream
python tasks/pluck-a-prepared-chicken/rescore_current.py /tmp/pluck-demo
python tasks/pluck-a-prepared-chicken/rescore_current.py /tmp/pluck-dream
```

Limit: 1,800 actions / 90 seconds at 20 Hz. Fixed setup, no jitter or random seed. Runtime saves every native frame, continuous body/action traces and frozen source. CPU rescoring checks source hashes, extraction history, damage and the outcome; it is not a substitute for native simulation. Whole-episode H.264 videos, visual review, elapsed wall times and source-equivalence checks must accompany any submission.


### Containment scorer correction

The final recordings use the unchanged native physics. The original scorer additionally rejected a carcass mesh vertex below a floor-contact tolerance, even while the carcass was upright and fully inside the tray. The current scorer uses the full horizontal footprint, centre height and upright pose for the carcass; feather floor containment and every damage/release check are unchanged. `rescore_current.py` first audits the frozen original scorer, proves that this is the only model-code change, then applies the correction to the complete trace. Raw native reports are preserved beside the corrected scores.


## Recorded validation

The native demo rescores **1.0 in 1180 controls**, extracting all nine whole feathers with intact skin. Peak skin displacement is **3.20 mm** against the 4 mm damage threshold; both hands remain clear for **8.85 seconds**. The matching Dream baseline scores **0.00 in 1800 controls**. Frozen-source rescoring, actual finger-contact checks and complete video-frame checks pass. Native elapsed time was 17.0 minutes for the demo and 37.4 minutes for Dream on a shared L40S.

Dream used the official local SDK bridge: 73 API attempts, 37 server sessions and 36 expiry recoveries within one uninterrupted physical episode. The API key stayed on the local computer. See `evidence/bridge-runtime.json` and `evidence/runtime-source-hashes.json`. The videos are complete native recordings, without cuts or retiming. These tests establish simulated feasibility; no physical specimen calibration or hygiene trial is claimed.
