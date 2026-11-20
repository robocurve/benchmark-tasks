# Fit and cure a gel nail extension

The native demonstration and Dream-1 trial are validated; scoring and proof-test reports are in `evidence/`.

Seat a prepared clear extension on a curved practice nail, cure its gel and leave it securely bonded after release. The robot must align the mating surfaces, hold them through the initial cure, press the lamp button and withdraw without dislodging the tip. Preparation is outside the timed trial; no person is used.

## Equipment and fixed setup

Use Kiara Sky clear Gelly Tips, compatible Builder gel and the Beyond Pro Flash Cure lamp. The manufacturer's [application guide](https://kiarasky.com/blogs/learn/how-to-apply-new-kiara-sky-gelly-tips-gelly-tips-nail-extension-application-gel-manicure) specifies an initial 10-second hold followed by 30 seconds of further exposure. The [lamp](https://kiarasky.com/products/beyond-pro-flash-cure-led-lamp) measures 66 × 58 × 47 mm and has 45/60-second timers. This task uses the 60-second cycle; these timings are specific to the matched system.

Clamp a 70 × 22 × 20 mm silicone practice finger at its proximal end. Fit a replaceable 14 × 12 mm ABS nail insert with 14 mm transverse curvature. A salon training hand with replaceable nails is suitable when one finger is fixed in the photographed pose; [Essential Nails](https://academy.essentialnails.com/nail-trainer/) provides this type of training hardware. Select one insert and matching clear tip size for all trials. The tip is 28 ± 2 mm long and 14 ± 1 mm at its widest point; its distal 8 mm stays dry for gripping.

The procedural geometry gives all fixed coordinates in metres. The practice-finger origin is (340, 0, 886.5) mm, facing the robot. The separate tip rests at (280, −85, 862.5) mm on a 14 × 10 × 50 mm wooden support. The lamp base is at (380, 0, 862.5) mm. A 20 mm spacer gives wrist clearance; the equipment stands on a 240 × 300 × 65 mm work riser. The finger and its proximal clamp clear the lamp floor and rear wall. Match the setup image; do not attach the tip to the insert before the trial.

Both grippers use 45 mm steel extensions with silicone contact faces. The distal faces measure 2 × 4 × 12 mm. Use removable silicone covers to keep uncured gel off the shared robot. The robot grips the dry free edge; there are no added handles on the tip.

## Preparation and reset

1. Select the same tip size each time. Lightly file its proximal outline to fit the insert, without changing its curvature. Buff the bonding surfaces, remove dust, clean and dry them.
2. Apply the compatible system's Prep/Primer and let it dry. Apply a thin Builder base coat to the insert and cure for 30 seconds, following the matched product procedure.
3. Use a 1 mL syringe and clean brush to apply 0.04 ± 0.01 mL of Builder gel to the tip's proximal 14 × 12 mm underside. Keep the distal 8 mm dry.
4. Place the separate tip on its support, uncured side down. Start within 60 seconds under ordinary room light. Use a fresh insert and tip for every trial; restore the photographed layout and reset the lamp timer.

This is a proposed repeatable preparation protocol, not a measured product calibration. The physical trial must use the matched cure guidance and pass the same retention check.

## Judging with simple tools

After release, take a close photograph with a millimetre scale and a removable white background. Outline the 14 × 12 mm bonding region and estimate bonded area with a 1 mm grid. Visible air pockets and lifted regions count as unbonded. Compare proximal-edge midpoints, not geometric centres: the extension deliberately projects beyond the insert.

All credit requires at least 40 seconds of exposed cure, edge alignment within 1.5 mm, orientation within 8°, no gel on the surrounding silicone and no damaged tip or insert. Both hands must be clear for at least three seconds.

Then attach a light removable pull tab at the extension's centreline, 14 mm from its proximal edge. With a spring scale, apply 1 N perpendicular to the nail: ramp up over 0.5 seconds, hold for three seconds and unload over 0.5 seconds. Record a close video against the millimetre scale. Any peak lifting or movement above 1 mm scores zero, even if the tip returns after unloading. The simulation video labels this evaluator load.

If all those checks pass, bonded area of at least 90% earns 1.0, 75% earns 0.5 and 50% earns 0.25. Less earns zero. A held result, incomplete cure or incomplete pull test earns zero. No chemical sensor or sub-millimetre measurement is needed.

## Simulation limits

A rigid curved tip contacts 35 fixed, 0.24 mm gel patches on a curved insert. Tangent collision facets preserve the mating curvature. A 0.15 mm gap margin approximates gel bridging between unresolved facets. Actual seating wets patches; exposure creates finite native bonds. Unseated regions remain unbonded, and spill or bond failure can occur. Intact flash-cured bonds continue receiving exposure during small elastic deflection; broken ones do not.

Gel flow, chemical conversion, individual bubbles and fracture of the rigid tip/insert are omitted. Contact stiffness, bond strength and exposure are uncalibrated approximations. The lamp spacer changes the working distance; this is not a radiometric model. These limits do not remove alignment, physical contact, holding, button operation or released retention.

A compliant native grip approximates friction only after opposed pad contact, with 12 N/0.030 Nm break limits and an 8 mm slip limit. It connects to the real gripper and transmits reaction loads. The 1 N test uses Isaac's native permanent wrench composer at the tip centre of mass. An unbonded control verifies that this API actually applies force. No task-object pose is assigned during a trial.

The scripted controller reads native poses and sends bounded robot joint actions. Dream receives the same scene and hardware, using standard camera and joint observations. All task geometry is original and covered by the included MIT licence; upstream YAM/Isaac dependencies retain their own licences.

## Run and rescore

Use the pinned environment in `ENVIRONMENT.json`; both full native runs are independently audited.

```bash
python tasks/fit-a-gel-nail-extension/run.py --mode demo --out /path/to/new-demo
python tasks/fit-a-gel-nail-extension/run.py --mode dream --out /path/to/new-dream
python tasks/fit-a-gel-nail-extension/rescore.py /path/to/new-demo
python tasks/fit-a-gel-nail-extension/rescore.py /path/to/new-dream
```

The layout is fixed, with no random seed. The demo uses 2,180 actions (109 simulated seconds); Dream gets 2,400 actions (120 seconds). The demo took 39.9 minutes and Dream took 37.8 minutes on the shared RTX PRO 6000 Blackwell GPU. Complete traces, original videos and frozen source are preserved separately; compact audit reports accompany this folder.

## Recorded results

The native demo scores **1.0** in 2,180 actions. Its released extension passes the 1 N pull, with 0.415 mm maximum movement and 100.0% bonded area. Dream-1 scores **0.0** over all 2,400 actions. Both full traces pass frozen-source scoring and an additional audit reconstructing the proof-load trigger, ramp and displacement. Twelve score-boundary checks cover bonded area, spill, damage, incomplete proof and excessive slip.

Dream made 99 API attempts across 50 sessions, including 49 SDK-reported expiry recoveries. This is a recovered-session baseline, not one uninterrupted episode; the transcript retains those diagnostics.
