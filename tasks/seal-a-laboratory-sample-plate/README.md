# Seal a laboratory sample plate with adhesive film

Cover the prepared plate with the loose film, press around the wells using the roller, and leave every well individually sealed. This is a common manual sample-storage procedure; an apparently covered plate can still leak through wrinkles between wells.

## Setup and material model

Use a 96-well polypropylene plate with a 127.76 × 85.48 mm footprint, 9 mm well pitch and 34 mm height; 130 × 80 × 0.1 mm adhesive PET storage film; a clean release liner and raised tray; and a padded hand roller with a 50 mm-wide wheel lying freely on the bench. The plate is secured in a fixed 140 × 100 × 30 mm polymer bench holder with four low clips. The holder provides finger clearance and prevents plate movement; removing the plate is outside the task. Operator preparation fills the wells and removes the backing before placing the film adhesive-side down on its non-stick liner. Peeling backing is not part of the task. Alternating coloured and clear water supports later leakage inspection.

The fixed polymer tray is 116 × 62 × 120 mm; its 130 × 62 × 0.1 mm liner leaves both long film edges accessible. The film weighs 1.44 g. The secured plate and prepared contents are represented by a fixed 104 g body. The 40 g roller has a 10 × 16 × 65 mm polymer handle and a 24 mm-diameter rubber wheel. Coordinates are metres: table Z=0.7475, plate centre (0.350, 0, 0.7945), film centre (0.200, 0, 0.8675), roller handle initially (0.180, −0.170, 0.7595), lying along Y. Gravity settles the loose objects before recording. Both arms use 40 × 6 × 1 mm rigid stainless-steel finger extensions and a 4 N jaw limit.

The film uses 216 passive connected sectors, with bending and native collision. The roller wheel rotates freely and its native padding compression supplies the pressing load. Adhesion requires actual rim proximity, roller load and dwell. Each sector has one force-limited point bond anchored at an actually pressed rim sample. The point can rotate, so adhesion does not freeze the film’s bending; it can detach under excess force. Detailed peel-front mechanics are omitted. The model does not simulate liquid flow or adhesive chemistry. A connectivity calculation estimates whether each well has a path to another well or the outside; it does not certify molecular sealing or long-term storage.

The physical design follows an ordinary storage-film and roller procedure, not heat-sealed PCR foil. Relevant manufacturer references: [Azenta roller applicator](https://www.azenta.com/products/adhesive-seal-roller-applicator), [seal selector](https://www.azenta.com/seal-selector). Coloured-water testing follows the principle in [Eppendorf Application Note 239](https://www.eppendorf.com/product-media/doc/en/159628_Application/Eppendorf_Consumables_Application-Note_239_Deepwell-Plates_Microplates_Eppendorf-Deepwell-Plates-Microplates-Investigations-sealing-methods.pdf); that is additional physical validation, not a simulated fluid test or a claim of certified leak-tightness.

## Scoring

After both open hands have been at least 50 mm clear for three seconds, score the fraction of 96 wells with a complete film cover and a continuous pressed adhesive-contact ring separating the opening from every other well and the outside. A missing cover, lifted region or wrinkle channel makes the affected well unsealed. Torn film or broken objects at any time gives zero. The plate is fixed in its holder. Doing nothing gives zero because the loose film is initially separate from the plate.

For a physical assessment, photograph the plate from above and under grazing light with a millimetre ruler. Inspect each well's cover and rim: follow the contact ring all the way around, counting an opening connected through a visible air gap or lifted wrinkle as unsealed. Record the 96 pass/fail results and divide their sum by 96. This is a geometric seal assessment, not a microbiological or long-term storage certification. The simulation uses its disclosed 0.75 mm interface-connectivity proxy for the same coverage and separation goal; it does not execute a liquid inversion test.

Use new film each trial. The plate contains approximately 0.4 mL of alternating blue and clear water per well; this is visual context in simulation. Filling, backing removal and cleaning are operator preparation. The roller starts lying on the bench beside the tray and is not attached to any support.

## Run and score

Use the pinned Isaac/YAM environment in `ENVIRONMENT.json`. A CUDA GPU is required for native physics and camera rendering; the offline scorer runs on CPU.

```bash
python tasks/seal-a-laboratory-sample-plate/run.py --mode demo --out /tmp/seal-a-laboratory-sample-plate-demo
DREAM_API_KEY="$DREAM_API_KEY" python tasks/seal-a-laboratory-sample-plate/run.py --mode dream --out /tmp/seal-a-laboratory-sample-plate-dream
python tasks/seal-a-laboratory-sample-plate/rescore.py /tmp/seal-a-laboratory-sample-plate-demo
```

The runner saves every 20 Hz action, native object poses, measurements, camera frames, events and an immutable source snapshot. `rescore.py` verifies the recorded source hashes and recomputes scores from that snapshot. It is not a second physics run. `make_plan.py` authors ordinary robot actions; neither reach checks nor synthetic scoring controls demonstrate feasibility.

No task object follows a scripted animation. Geometry is reset once. During execution the robot supplies ordinary joint commands; native bodies, contacts and passive joints determine the outcome. The film pinch limit is 0.4 N / 0.001 Nm; the roller pinch limit is 5 N / 0.02 Nm. Any finite pinch approximation requires opposed finger contact, starts at the measured pose, transmits reaction force, and releases or slips. The same hardware and physics apply to Dream and the demo.

## Validation

Native demo: **1.0**, 1,860 controls (93 simulated seconds). Dream-1: **0.0**, 2,400 controls (120 simulated seconds). The demo leaves all 96 wells sealed, with 5.9 seconds of clear-hand retention and no film damage.

Frozen-source rescoring and native finger-contact checks pass for both episodes. The uploaded videos are complete, uncut 20 Hz recordings from the fixed front camera; the setup image is the demo's initial frame. Visual review sampled 12 evenly spaced frames in all three recorded views. `VALIDATION.json` and `evidence/` record scores, source hashes, contact checks, trace/video hashes and the media receipt. The full traces are retained in the team's private evidence archive.

Measured wall time on a shared L40S was 45.1 minutes for the demo and 59.4 minutes for Dream-1, including concurrent validation workloads and model calls. These are measured run times, not real-time performance claims. The layout is fixed and no random seed is used. Environment and policy sources match between the two runs; any unused demo-authoring differences are listed in `evidence/source-equivalence.json`.
