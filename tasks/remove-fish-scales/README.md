# Remove fish scales without cutting the skin

Finish descaling the exposed side of a prepared whole fish. Hold the loose fish, scrape against the scales with the hand scaler, then release it on the board with its skin intact.

## Setup and material model

The fish lies on one side on a 250 × 300 × 8 mm wooden cutting board. Its underside has already been descaled as operator preparation. The exposed side has overlapping scales. A loose stainless hand scaler lies alongside it. The fish is approximately 260 mm long and 250 g; the tool has an ordinary handle and a compact serrated 24 mm-wide head. Exact geometry and the fixed starting layout are in `geometry.py`.

Coordinates are metres: table Z=0.7475; board centre (0.380, 0, 0.7515); fish centre initially (0.320, 0.015, 0.7745), with its tail toward +Y; scaler handle reference (0.180, −0.170, 0.7825). The 87 g stainless scaler has a 12 × 160 × 10 mm handle, a 24 × 28 × 6 mm head and rounded serrations on its underside. A short neck offsets the handle 30 mm above the head, providing clearance over the fish. This is a generic compact tool, not an exact commercial replica. The board is fixed; the fish and scaler are loose. Gravity settles them before recording. No randomisation is applied.

A rigid native fish body, including the tail, carries skin patches with finite normal, shear and bending compliance, and 26 small groups of overlapping scales. Actual tooth contact, normal load and tail-to-head travel accumulate work; sufficient work releases a scale group. Released groups remain physical objects. Excessive skin compression or a broken tissue joint invalidates the attempt. This approximates tissue and scale retention, not flesh deformation, tail tearing or individual microscopic scale mechanics. Both arms have 40 × 6 × 1 mm rigid stainless-steel finger extensions with a 4 N jaw limit; real trials use the snug protective covers specified below. Contact-checked compliant pinch joints are limited to 5 N and 0.10 Nm, release when fingers open and slip after 3 mm of material movement through the contact. The underside is prepared to avoid repeating the same scraping on a second side; it does not remove the support, force-control or directional-scraping requirement.

## Physical specimen, protection and reset

Obtain **fresh, gutted but unscaled farmed European sea bass (Dicentrarchus labrax)** from one fishmonger/batch. Ask explicitly that it is not descaled. Accept total length 240–280 mm and mass 220–320 g, with the entire left flank intact, its scales attached and no visible cuts, bruised/split skin or missing-scale patches. Use fresh chilled specimens, not previously frozen ones; record supplier, delivery date, length, mass and temperature. The ranges are proposed selection controls, not measured repeatability results.

Store bagged specimens at **0–4 °C**, separate from the robot area, and use within 24 hours of delivery. Prepare one at a time over a separate food tray. A trained operator removes scales from the **right underside only**, keeping the left flank untouched. Trim dorsal, anal and pelvic fin rays flush with their bases using kitchen shears, without cutting flank skin; leave the tail intact for support. Do not scrape, pre-loosen, score, glue or dry the working flank. Blot standing water once with a disposable towel, without rubbing. Place left flank up, tail toward +Y in the photographed layout. Start within 10 minutes of preparation; finish the trial and inspection within **30 minutes out of chilling** and discard the specimen afterward. No trial food is served. Use a fresh fish for each attempt.

Contain the entire board, tool and working area inside a **600 × 450 mm washable polypropylene tray with a 30 mm rim**, on a non-slip waterproof mat. Fix the board inside the tray at the existing coordinates; keep the rim outside all planned robot paths. Use a washable splash screen beyond the rear of the workspace and collect loose scales inside the tray. These stationary barriers do not hold the fish or guide the scaler.

Each steel extension has rounded, deburred edges and a **snug 0.3–0.5 mm food-contact silicone finger cot**, extending over its mounting junction. A disposable waterproof sleeve encloses each gripper and lower wrist and overlaps beneath the cot; close the upper end above the splash zone. No exposed tape, loose folds or sleeve material enters the jaws or scaler head. Change sleeves/cots between specimens. Use dedicated removable extensions; wash and sanitise the scaler, extensions, board, tray and reusable barriers with a food-contact-compatible product according to its label, then dry before reset. Remove contaminated covers before touching the robot. Do not spray cleaner at the robot.

**Manual preflight before robot trials:** on one spare from the same batch, verify the covered extensions can hold the wet tail at the 4 N jaw limit while the specified rounded-tooth scaler lifts scales, without slipping off or puncturing a cot. Inspect both cots over a light and check for leaks after this trial. Check that trimmed fin stubs cannot contact a sleeve and that the tool handle can be held without teeth touching either hand. Discard a sharp/burred scaler or failed cover; do not compensate by increasing robot force or switching to exposed steel. Stop a robot trial on any barrier puncture or contamination beyond containment, and clean before another trial. This is a required physical setup check, **not a test already performed**.

The protective barriers are a real-rig requirement. Their fine thickness, puncture resistance and splash capture are not simulated. The recorded finger contact/friction model is a coarse approximation of covered contact; no new claim of calibrated grip or barrier durability is made. The geometry, controller, run limit and recorded robot motion are unchanged.

Cold storage and separation follow the [FDA seafood handling guidance](https://www.fda.gov/food/buy-store-serve-safe-food/selecting-and-serving-fresh-and-frozen-seafood-safely); the 30-minute trial limit is deliberately shorter than its general two-hour unrefrigerated limit.

## Scoring and reproducible photo grid

The earlier wording “26 equal-area sectors” was incorrect for a curved, variable fish. Use **26 equally weighted, fully specified inspection zones**, not a claim of equal physical surface area. The entire exposed scaled flank is partitioned; the grid is an inspection overlay, not a set of cut or pre-cleared patches.

1. Before the trial, photograph the left flank perpendicular to its broad side, with a ruler in that plane. Trace the scaled flank boundary: behind the gill cover, before the caudal-fin base, and between the dorsal and ventral scale margins. Exclude the head, fins and the already prepared underside. Save this original outline and grid with the trial; never redefine it from the result.
2. Align the longitudinal axis from gill-cover midpoint to caudal-fin-base midpoint. Divide the flank's maximum perpendicular width into **five equal-width longitudinal bands**. In each band, find the first and last longitudinal coordinates at which the traced flank is present. Divide that span into **four equal-length intervals for each outer band**, and **six for each of the three inner bands**. Clip each cell to the original flank outline. This yields 4 + 6 + 6 + 6 + 4 = **26 zones** covering the flank. Number bands dorsal to ventral and cells head to tail. Reject a specimen if any resulting zone is empty. A ruler and printed photograph suffice; [grid-example.svg](grid-example.svg) illustrates the construction and numbering.
3. After release, put the fish back in the same photographic orientation **only for inspection**, without scraping it. Align the saved overlay to the gill cover, fin base and flank outline. Use close views and a smooth, blunt probe to check attachment; the probe may lift an already loose flake but must not detach an attached scale. Count a zone only when it has no attached scales. A scale crossing a boundary makes both touched zones uncleared. Check ambiguous boundary scales from both views; do not round them into cleared zones.

After **three seconds** with both open hands at least **65 mm clear**, score **n/26**, where n is the number of cleared zones. Full score additionally requires no attached scales anywhere on the exposed flank. Loose detached flakes do not reduce the score. The fish must remain supported on its prepared side on the board. Any new skin tear, skin flap separation, exposed flesh or visibly crushed/broken skin gives zero; compare the pre-trial close photographs and inspect under ordinary bright light. Uncertain damage or scale attachment is not awarded as intact/cleared until close inspection resolves it. Incomplete release or a fish off the board/on its other side gives zero.

The simulation has 26 equal-size retained scale groups as a coarse proxy for these zones; its layout is not a species scan. It counts complete group detachment and uses skin-joint breakage or more than 2 mm compression as a damage proxy. It does not measure the exact surface fraction of a real sea bass or simulate every interstitial scale. No scale exists outside those groups in the model, so its full score already entails complete removal. This clarification preserves the recorded scores: demo 26/26, Dream 0/26. Runtime sources and media are unchanged.

## Run and score

Use the pinned Isaac/YAM environment in `ENVIRONMENT.json`. A CUDA GPU is required for native physics and camera rendering; the offline scorer runs on CPU.

```bash
python tasks/remove-fish-scales/run.py --mode demo --out /tmp/remove-fish-scales-demo
DREAM_API_KEY="$DREAM_API_KEY" python tasks/remove-fish-scales/run.py --mode dream --out /tmp/remove-fish-scales-dream
python tasks/remove-fish-scales/rescore.py /tmp/remove-fish-scales-demo
```

The runner saves every 20 Hz action, native object poses, measurements, camera frames, events and an immutable source snapshot. `rescore.py` verifies the recorded source hashes and recomputes scores from that snapshot. It is not a second physics run. `make_plan.py` authors ordinary robot actions; neither reach checks nor synthetic scoring controls demonstrate feasibility.

No task object follows a scripted animation. Geometry is reset once. During execution the robot supplies ordinary joint commands; native bodies, contacts and passive joints determine the outcome. Any finite pinch approximation requires opposed finger contact, starts at the measured pose, transmits reaction force, and releases or slips. The same hardware and physics apply to Dream and the demo.

## Validation

Native demo: **1.0**, 2,390 controls (119.5 simulated seconds). Dream-1: **0.0**, 2,400 controls (120 simulated seconds). The demo detaches all 26 scale groups, leaves the fish supported on the board and retains intact skin after release. Peak skin compression is 1.29 mm, below the disclosed 2 mm damage limit.

Frozen-source rescoring and native finger-contact checks pass for both episodes. The uploaded videos are complete, uncut 20 Hz recordings from the fixed front camera; the setup image is the demo's initial frame. Visual review sampled 12 evenly spaced frames in all three recorded views. `VALIDATION.json` and `evidence/` record scores, source hashes, contact checks, trace/video hashes and the media receipt. The full traces are retained in the team's private evidence archive.

Measured wall time on a shared L40S was 28.0 minutes for the demo and 60.4 minutes for Dream-1, including concurrent validation workloads and model calls. These are measured run times, not real-time performance claims. The layout is fixed and no random seed is used. Environment and policy sources match between the two runs; any unused demo-authoring differences are listed in `evidence/source-equivalence.json`.
