# Tighten and clip the C-A-T Gen 7 training windlass

Emergency-care trainees practise tightening a windlass tourniquet and maintaining compression while securing its rod. This task isolates that step on a reusable training cylinder. The category is **Other**. The gauge is a trainer indicator. Complete application, initial band routing, the security strap and recording time are outside the task.

Instruction: “Tighten the C-A-T trainer until its gauge is green, lock its rod in its attached clip, then move both hands clear.”

## Product readings and evidence

The figures below were **measured on a C-A-T Gen 7 trainer with a scale by a technician who works with this product, Nov 2026**: pull to keep turning the windlass at lock **2.2 kg at a 65 mm lever**, snug to lock **3/4 turn (270 degrees)**, push to snap the rod into the clip **0.8 kg** and, measured separately, pull to lift it out **0.8 kg**; rod diameter **12 mm** and clip mouth **10 mm**. The executable converts kg-force using 9.80665 N/kg: **1.40235095 N m** lock torque and **7.84532 N** in each clip direction. The record is in `evidence/TECHNICIAN-READINGS-NOV23.md`.

These figures set the winding spring endpoint, the lip fit and the snap force gate. Between snug and lock the winding torque rises linearly to the measured endpoint, and the spring uses that torque as the static recoil at lock. The pressure-angle curve, band tilt stiffness and the clip's internal spring geometry are model choices, listed under the physical model below. No physical robot trial is claimed.

The simulated loads match these measurements. Winding torque at lock equals the measured 1.40235 N m exactly. Native insertion is 8.00 N, 2% above the measured 7.85 N. Native pull-out is 8.80 N, 12% above the measured 7.85 N, because in simulation the clip also carries the band's winding load during the pull. No simulated load is below its measured value.

Insertion here means the rod-lip normal component; complete insertion reaction including base contact and friction was **8.359 N** (+6.5%). The entry witness completed at 67.90 s, before closed-mouth seating at 68.65 s. Released retention lasted 9.95 s; direct hand-to-clip normal contact was zero through entry and pull. These are native simulation observations, not additional trainer measurements. The frozen-source scripted demo scores **1.0/1.0 in A/B**, with two successful trials and zero errors; the single Dream-1 run scores **0.0/0.0**, with no trial errors and both trials reaching 2,400 steps. Earlier estimated-load runs do not qualify these inputs. Source, final policy records and force evidence are in `evidence/VALIDATION-NOV23.md` and `evidence/final-policy-results.json`.

## Product, fixture and lots

Use the [NAR blue C-A-T Gen 7 trainer 30-0033](https://www.narescue.com/combat-application-tourniquet-c-a-t-tb.html), with its **original attached clip and stabilization plate**. The [maker's trainer sheet](https://www.narescue.com/downloadable/download/sample/sample_id/2003/) identifies the model and NSN 6910-01-560-2972. The 12 mm rod diameter and 10 mm clip mouth were measured on the same C-A-T Gen 7 trainer by a technician who works with this product, Nov 2026. Other rod/clip dimensions are enrollment estimates, not maker CAD. The [maker's procedure](https://www.narescue.com/combat-application-tourniquet-c-a-t-instructions.html) says: “Secure the rod inside a clip to lock it in place.” No clip is glued to the training limb.

The reusable cylinder is 127 mm in diameter and 400 mm long, centred 230 mm forward of the arm-base midpoint with its axis 65.5 mm above the table. Its axis lies across the table along +y. Four 30 x 40 x 35 mm wooden chocks and two 20 mm webbing end straps secure it. Screw the chocks to a wooden tabletop; pass each end strap over the cylinder and beneath the tabletop, tensioning it down into the chocks. The simulator fixes these anchored supports. The central blue band is 38 mm wide; the rod approximately 150 mm long and 12 mm in diameter starts level at azimuth 66 degrees, with its centre 150.5 mm above the table. The two factory clip channels lie approximately 50 mm from the windlass at azimuths 156 and 204 degrees, with a 13 mm channel, 10 mm mouth and approximately 21-degree seated tilt. `task.json` and `fixture/layout.svg` document enrollment.

The demo first winds 175 degrees with the left hand, hands the loaded rod to the right, then counterholds through a wrist reset before the right hand regrips. Both hands are still required by the demonstrated clipping motion: one maintains winding torque while the other takes the opposite end and pushes it through the clip lips. The demo does not read simulator state, hidden flags or exact-pose feedback. Fixed positional lag compensation is part of its open-loop motion; jaw angles follow the intended rod angle. Public-layout IK computes fixed joint endpoints; the local demo helper interpolates joints to avoid an unreachable Cartesian wrist-reset path. It reads no simulator feedback.

Only the rigid gauge/stand moves between lots: its face centre is (360,-180,105) mm in A and (400,+220,105) mm in B. The articulated tourniquet, clip joints, cylinder and winding start pose remain identical. Run both lots and average. Zero object jitter is required; seeds do not alter the fixed layouts.

## Reusable bladder and cost

Use a 470 mm section of a Sunlite 26 x 1.90–2.125 bicycle inner tube. A [catalogue supplier lists this tube family around $6](https://www.hermosacyclery.com/accessories/tubes/). Enroll a flat width of 48–51 mm so the 51 mm binder clip spans the full seal; reject wider sections. Tyre-size markings alone do not prove width. Fold one end twice by 15 mm and clamp its full width with a 51 mm binder clip. Insert a 1/4-inch barb 20 mm into the other end, gather the rubber and secure it with two zip ties and tape. This leaves approximately 420 mm active length, enough for the 399 mm circumference plus 21 mm overlap.

Run the port through tubing and a Y to an [MDF Calibra 808M aneroid gauge/bulb kit](https://mdfinstruments.com/products/mdf-calibra-aneroid-sphygmomanometer-black), listed by the maker around $42. The modeled 52 mm face is an estimate to verify on enrollment. Add a removable green bezel mark spanning the physical gauge's 250–300 mmHg graduations, leaving the scale and needle visible; `fixture/dial.svg` is an indexing guide. Pump only under the snug band. Close the bulb branch with a pinch clamp, detach the bulb and cap the open branch. Require <=2 mmHg loss in 5 minutes at 60 mmHg and <=4 mmHg in 2 minutes at green before enrollment. Keep the original band tail fastened and taped, and fold the time strap aside.

A stock [5-inch round foam bolster](https://actionup.com/products/round-foam-bolster-medium-soft-compression.html), F15036RB05, can be cut once to 400 mm. The supplier describes medium-soft upholstery foam. The scene uses rigid cylinder geometry; the endpoint winding resistance uses the measured trainer reading; intermediate winding compliance and the pressure response remain model assumptions requiring enrollment on the actual foam. It is reused. Prices checked November 23: trainer maker page from $34.99, MDF Calibra 808M $41.99, Sunlite tube $6, and the selected F15036RB05 5-inch bolster $13.50. Fittings/clips/tubing $20 and wooden chocks/straps/stand $25 are budget estimates, giving approximately **$142 before tax/shipping**, excluding a borrowed force instrument and reference manometer for enrollment. The bolster is reusable after a one-time cut; newly unpacked foam needs the supplier's stated 2–6 hours to expand before enrollment. Ready specimens then reset in minutes by design; reset time is not measured. Product pages and selected-variant price response are recorded in `fixture/SOURCES-NOV23.md`. No purchases were made. Initial bladder preparation is planned at approximately ten minutes; a normal reset should take a few minutes, but neither duration has been measured.

## Physical model and independent score

The winding joint has a 0.297588 N m/rad spring and 0.20 N m s/rad damping, reaching 1.40235095 N m at 270 degrees. Its reaction is carried by the anchored band. The 0.10 N m/rad tilt spring represents woven-band bending. The rounded clip edge uses the separately measured insertion and extraction forces. The base stiffness fit is 26.265 kN/m. Native per-lip reaction data led to fixed compliance scales 1.59/3.485: lip_a/lip_b stiffnesses 41.761/91.532 kN/m, damping 36.56/54.12 N s/m, 8 g effective mass each, and spring-drive force limits 10.62/23.29 N. Clip static/dynamic friction is 0.05/0.04 with minimum combining; the rod/hand grip material stays unchanged. The bead centre is 3.0 mm above the nominal seated rod centre; a permanent 1.0 mm high flat upper entry face reaches 4.0 mm, while the lower retaining edge stays rounded. This height and the spring/material values are unmeasured contact-model fits, not measured product dimensions or polymer properties. The fixed spring-and-damper drives saturate at the stated internal drive-force limits, in both directions throughout the trial. Slopes and saturation are unmeasured contact-model fits, not a measured material curve; a drive limit does not bound every instantaneous contact reaction. No parameter switches with direction, contact or score. The native hip-clip probe gives the forces above; final policy qualification is separate. Counterheld extraction is not used as a match. Floors, walls and lips retain physical colliders.

Every credited snap needs rod–lip contact of at least **7.84532 N**, at least **1 mm** advancing contact traversal and **1 mJ** contact work during one entry with contact on each lip. Finger-to-lip pressing cannot set that record. No pose write, weld or capture constraint moves or retains the rod. The observer only recognizes physical work. The mouth must remain at least 0.2 mm narrower than the rod after entry. Reversal over 0.2 mm, lost alignment or a different rod end discards an uncompleted attempt. An unpassed entry reaching the seat with a retentive or absent reaction is discarded too: later pressing after capture cannot complete its witness. Held credit needs native hand–rod contact.

The gauge is a pressure proxy: min(300, 60 + 210 × absolute winding degrees / 270) mmHg. The unchanged green range is 250–300 mmHg. No blood flow is simulated. Both hooks keep their independent estimated azimuths 156/204 degrees. The snug rod starts at 66 degrees so the measured 270-degree stroke ends at the original hip hook; enrollment must reproduce that orientation without moving the factory clip.

For a real trial, use gauge/clip photos and video for timing. After the scored hold, the assessor confirms retained seating with a 2 N upward pull along the clip normal at the clipped rod end for 1 s, recording any extraction. This fixed gentle verification load is approximately a 200 g weight, about one quarter of the measured 0.8 kg pull-out force. Below-green retention takes precedence: it earns **0.25 even with an arm nearby**. Otherwise, a retained green rod earns **1.0** after all arm surfaces remain at least 10 mm from the rod and both clips and the winding and tilt angular speeds each remain at most 1 degree/s for one second. Green retained or level-held progress earns **0.5** without that release hold. Everything else earns **0**. Missing observations fail closed. The numeric green range is 250–300 mmHg. Reading the gauge while contacting the thigh, bladder or gauge does not count as green.

The release audit includes live clip lip shapes and fails closed on missing lip poses. It samples broad arm faces and includes a 6 mm conservative sampling margin. The thigh, bladder and gauge contact guard uses that same conservative margin; a contact between sampled surface points cannot authorize a green reading. Native telemetry records the sampled body gap and release clearance. Fingertip contact with the cylinder or attached clip during manipulation is physically allowed; it does not waive arm-link collisions or the final clearance gate. `DEMO_TOUCHES` documents that narrow offline-check exception. Native collisions and forces stay enabled. A current-state certificate prevents substituting an arbitrary state for observed evidence.

| Bypass or near miss | Required score |
|---|---|
| No action, no known contact observations, or loose unlocked rod after release | 0 |
| Early below-green lock with hands near or clear | 0.25 |
| Green rod still held level | At most 0.5 |
| Green retained rod with an arm still near it | At most 0.5 |
| Independent finger pressing of clip lips | No verified snap credit |
| Pressure above 300, or a reading while pressing the gauge/bladder/thigh | No green credit |
| Retained state without representative contact work | 0 |
| Full-score request before a complete second released and still | At most 0.5 |

## Run and reset

In the installed Isaac Lab-Arena / Inspect Robots environment, from this folder:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 --epochs 2 \
  --instruction "Tighten the C-A-T trainer until its gauge is green, lock its rod in its attached clip, then move both hands clear." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir demo-output
```

Use `--policy dream` once on the final frozen source with the same instruction after the full-score demo. Per-trial run limit is 120 seconds of control time; native runtime and actual scores must come from run records. MuJoCo CPU probes are development evidence only. `near_misses.py` contains synthetic score-boundary and face-clearance checks; these are not robot results.

Between trials, lift the rod from its own clip, untwist to the level 66-degree start, inspect the band, chocks and end straps, and restore the gauge to 60 +/-2 mmHg if needed. Isolate and remove the bulb again. Move only the rigid gauge/stand for the selected lot. Inspect wear and seals and repeat enrollment every ten trials or after leaks. No per-trial casting, printing or curing is required.

## Provenance, distinctness and licensing

This independently written task credits [PR 1026, TK-U4G6F6](https://github.com/robocurve/benchmark-tasks/pull/1026). Earlier versions are TK-NMSLU5, TK-NEMG7P and TK-EL693A. Original code and assets were not copied; shared helpers derive from the task kit. PR 917 tests releasing a phlebotomy tourniquet before withdrawing a needle; loaded winding and clip retention are different work.

Task code and original drawings follow the task repository's MIT release. No original-PR asset, third-party mesh or font is included. The dial is a procedurally rendered label using Pillow's default font. Referenced product sheets remain external sources.

## Complete prior verdicts and point-by-point disposition

2026-11-12T09:34:25Z — https://github.com/robocurve/benchmark-tasks/pull/1026#issuecomment-6819993757

> **Not accepted** (TK-NMSLU5).
> 
> Construct validity (not met): The simulated latch holds the rod down, but the specified clip's 18 mm gap between its lips cannot retain a 12 mm rod.
> 
> Clarity and reproducibility (not met): The specified 12 by 22 cm cuff bladder cannot wrap once around the approximately 40 cm circumference of the thigh.
> 
> Feasibility (not met): The real clip's lips do not overlap the rod, so the specified snap-in retention cannot be reproduced.
> 
> What to change: Please revise the trainer geometry so the windlass rod can snap past the clip lips and be retained: the current 4 mm clearance on each side and 1 mm lips leave an 18 mm opening for a 12 mm rod. Specify a cuff bladder long enough to wrap around the approximately 40 cm thigh circumference, and update the pressure calibration and simulated lock as needed to match the corrected build.
> 

2026-11-14T05:50:29Z — https://github.com/robocurve/benchmark-tasks/pull/1026#issuecomment-6872656235

> **Not accepted** (TK-NEMG7P).
> 
> Flags: Prior review feedback for the earlier version was not provided
> 
> Importance (not met): Applying a windlass tourniquet is emergency first aid rather than a scenario most people know firsthand, although its lifesaving impact supports the Other domain.
> 
> Construct validity (not met): The setup removes the tourniquet's own clip and glues a custom clip to the thigh, changing how the windlass is secured in real use.
> 
> What to change: Use a standard windlass tourniquet with its own attached clip rather than replacing it with a clip glued to the training limb; update the setup, simulation, calibration and full-score demo accordingly. Change the domain to Other and explain the task's emergency-care impact. Clarify whether a rod locked below green with an arm still nearby earns 0.25 or 0.5.
> 

2026-11-16T21:11:30Z — https://github.com/robocurve/benchmark-tasks/pull/1026#issuecomment-6929474142

> **Not accepted** (TK-EL693A).
> 
> Construct validity (not met): The simulated clip offers no snap resistance and the assumed green-range torque is only 0.22 N m versus the submission's 1–2 N m real-device estimate.
> 
> Clarity and reproducibility (not met): The custom heat-sealed bladder has dimensions but no repeatable quick preparation method or source for equivalent bladders.
> 
> What to change: The earlier clip, domain, scoring-order and bladder-length changes are addressed. Two new points were not raised before: measure the winding resistance and clip snap force on the specified trainer, update the simulation so the robot performs comparable work when turning and seating the rod, and provide an updated full-score demo; also specify an inexpensive source or a quick, repeatable preparation procedure for the 5 × 42 cm bladder, including its tube and sealed port.
> 

2026-11-19T00:27:30Z — https://github.com/robocurve/benchmark-tasks/pull/1026#issuecomment-6995781191

> **Not accepted** (TK-U4G6F6).
> 
> Construct validity (not met): The trainer's winding resistance and clip snap force have not been measured, so the revised simulation's force fidelity remains unverified.
> 
> What to change: The trainer-force measurements requested in the previous review are still open. Measure the winding resistance and clip snap force on the specified C-A-T trainer, report the results, update the simulated loads to match them, and provide a full-score demo using those loads. The revised resistance model and the bladder preparation procedure address the other points.
> 

2026-11-22T23:47:27Z — https://github.com/robocurve/benchmark-tasks/pull/3833#issuecomment-7087834722

> **Not accepted** (TK-TNDZBX).
>
> Construct validity (not met): The specified trainer's winding and clip forces remain unmeasured, so the defining loaded motions have not been matched to the real device as previously requested.
>
> What to change: The trainer-force measurement request is still open. Measure winding resistance and clip insertion and extraction forces on the specified C-A-T trainer, report the measurements, update the simulated loads to match them, and provide a full-score demo using the matched loads.
>
> To submit a revision, push your changes to this pull request (no need to open a new one). Your idea stays reserved for 48 hours from this review; if no revision arrives by then, the hold lapses and another team can claim the idea.

| Requested correction or flag | Current disposition |
|---|---|
| NMSLU5: clip must overlap the 12 mm rod | Measured 10 mm mouth, 2 mm below the measured 12 mm rod diameter; physical lips and closed-mouth retention check |
| NMSLU5: bladder must wrap the roughly 40 cm circumference | 420 mm active stock-tube bladder, 21 mm overlap; quick folded/clamped seal and leak checks |
| NEMG7P: prior review feedback not provided | All five verdicts are reproduced above |
| NEMG7P: own attached clip, Other, emergency-care impact | Original C-A-T clip/plate retained; Other; emergency-care training context |
| NEMG7P: below-green lock with an arm nearby, 0.25 or 0.5 | Explicit 0.25 precedence, tested synthetically |
| EL693A / U4G6F6: measure winding resistance and clip snap force | Measured on a C-A-T Gen 7 trainer with a scale by a technician who works with this product, Nov 2026 (2.2 kg at 65 mm, 3/4 turn, 0.8 kg in, 0.8 kg out) |
| EL693A: inexpensive quick bladder source/preparation | Stock Sunlite tube, folded full-width seal, barb/Y/clamp/cap and enrollment checks; priced reusable kit |
| TNDZBX: measure winding and clip insertion and extraction | 2.2 kg at 65 mm, 270 degrees; separate 0.8 kg push and pull; provenance and conversions above |
| TNDZBX: report measurements and update simulated loads | Named constants drive the 1.40235 N m endpoint, 10 mm mouth and 7.84532 N snap gate; winding endpoint exact; native insertion 2% and pull-out 12% above the measured values; pressure and intermediate torque shape are model choices |
| TNDZBX: full-score demo with matched loads | Frozen-source native demo A/B 1.0/1.0, then one Dream-1 A/B 0.0/0.0; both zero errors. See evidence/final-policy-results.json |

## Reproduction and further calibration

`LOAD_SOURCE='technician_phone'` uses the trainer scale measurements above and is the mode of every submitted run. A second, optional mode, `LOAD_SOURCE='measured'`, accepts a larger repeat-data file (three winding repeats at three angles, three insertion/extraction repeats per hook, a pressure curve and hashed raw files) and fails closed on incomplete data; see `fixture/CALIBRATION.md`. Earlier substitute-force estimates are kept in `fixture/LOAD-ESTIMATES.md`, superseded by the trainer measurements.
