# Blot excess water from wet hair

Blot the wet hair with the flexible towel, then release it. Towel pickup is outside the task: both hands start holding its corners, clear of the hair.

A full-size salon training head has twelve independently moving locks, each 245 mm long. The separate microfiber cloth bends and folds. The robot must bring cloth into actual contact with the hair to absorb water; proximity without surface contact does not remove water. No cloth, hair or object is teleported.

Contact-driven water transfer is a disclosed approximation; it does not simulate droplets or fibre-level capillary flow. The current fixed mechanical masses represent the initially wet hair and dry cloth; water bookkeeping conserves mass but does not update native inertia during uptake. Real-world judging uses towel mass gain and checks for hair damage.

[Wella guidance](https://www.wella.com/professional/en-CA/blog/hair-care/how-to-tame-unruly-hair) describes gently squeezing sections between fabric, avoiding twisting and scrubbing.

## Physical preparation and judging

Use the specified 180 ×156 ×230 mm headform with twelve separately rooted 245 ±5 mm locks, 3.0 ±0.1 g dry mass each, on the same 72 mm rear patch as the photographed layout. Clamp the 190 mm stand and 150 mm-square base. A 180 ×240 mm microfiber cloth has about 300 g/m² areal mass (13 ±1 g dry). Start with the dry towel hanging vertically, its two upper corners held by the robot at x 320 mm, y±90 mm and 305 mm above the table. Its 240 mm length hangs down. The hands start touching these corners; finite pad-contact pinches support the towel during reset. This pre-held setup is shared by demo and Dream and is part of the physical preparation; picking the towel up is not evaluated. The towel starts clear of the wet hair. Both grippers have 40 ×24 ×3 mm silicone-faced extensions.

Prepare the locks from ordinary straight synthetic braiding hair: weigh twelve 3.0 ± 0.1 g dry bundles, trim the free lengths to 245 ± 5 mm, and bind only their uppermost 10 mm to a thin backing glued flush to the rear scalp. Do not join the free lengths. Use an inexpensive [Marianna practice head](https://www.sallybeauty.com/salon-supplies/other-professional-supplies/beauty-student-supplies/styrofoam-head-with-face/SBS-200201.html), padding it to the stated dimensions within 10 mm. Each root must survive a 2 N preparation pull. Root positions in millimetres (lateral offset, height above head centre) are (-35.8, 50.0), (-29.2, 43.1), (-22.8, 28.8), (-16.2, 20.2), (-9.8, 25.2), (-3.2, 39.3), (3.2, 49.4), (9.8, 46.3), (16.2, 32.8), (22.8, 21.3), (29.2, 22.4), (35.8, 35.1), each within 2 mm; the front-to-back coordinate follows the rear scalp. The head centre is (490, 0, 1052.5) mm, facing away from the robot; table height is 747.5 mm. Brush the free locks separately into the photographed layout.

Record the dry head-plus-hair tare and the dry towel mass using a 0.1 g scale of sufficient capacity. Soak the hair in room-temperature water for 60 s, let it drip for 30 s, then weigh it. Adjust and repeat until the retained water is 18 ±1 g, with no free puddle on the head, stand or table. Start within 30 s. Initial excess water is the wet-minus-dry head mass, W 0; the scorer uses the measured value rather than assuming exactly 18 g. Use a dry clean cloth for each trial, restore the photographed loose hair arrangement, and replace damaged locks.

Weigh the towel after the trial. Towel mass gain divided by W 0 earns 0.25 at 25%, 0.5 at 50%, and 1.0 at 70% after both hands have been clear for 3 s. A held result is capped at 0.5. Less than 25%, any torn cloth, detached/broken lock, or removed hair carried away with the towel scores 0. Inspect the head and towel and compare dry hair mass if damage is uncertain. Water on the table or lost by dripping does not count as towel uptake. The setup has no other water source.

The simulator groups fibres into twelve 18-segment flexible locks and a 130-node native surface towel. Surface contact can transfer at most 0.05 g/s per grouped segment into the towel, with a 60 g total capacity; this uncalibrated capillary law conserves all 18 g. There is no background evaporation, automatic drying or transfer at a distance. Mechanical masses stay at their initial values during uptake. Native hair joints can break, the towel can stretch beyond its failure bound, and finite pad-contact pinches can slip. Individual fibres, frizz and microscopic absorbency are omitted. The towel pinch is limited to 18 N/0.08 Nm and releases on opening or 8 mm slip. Its moving world anchor omits grip reaction loads on the robot; no task-material pose is assigned.

## Run and score

Use the Python 3.12 / Isaac Sim 6.1 / YAM runtime pinned in `ENVIRONMENT.json`. An NVIDIA GPU is needed for native rendering. The fixed setup has no random jitter or seed. `python tasks/blot-wet-hair/run.py --mode demo --out /path/to/new-demo` records the 1460-action scripted plan; use `--mode dream` for the 2400-action model trial. `python tasks/blot-wet-hair/rescore.py /path/to/new-demo` audits every action, contact, water transfer, damage event and released outcome. The native demo completes 1,460 actions in 33.1 minutes on a shared RTX PRO 6000 Blackwell GPU. It scores **1.0**: 13.108 g transferred (72.824%), intact material and 8.85 seconds with clear hands.

Dream-1 completes 2,400 actions in 34.1 minutes and scores **0.0**. It drops the towel without transferring water. There were 99 API attempts across 50 sessions, including 49 SDK-reported expiry recoveries. This recovered-session baseline is not an uninterrupted session; the transcript preserves the requests and recoveries.

Both full-frame native runs pass frozen-source audits of every action, contact, water transfer, conservation check, damage event and released outcome. Seven scoring boundary checks cover partial credit, release and damage. Front, rear and side views were reviewed. Reports, source and trace hashes, session diagnostics and visual review are in `evidence/`.

## Assets and licences

The head, stand, grouped hair and towel are original procedural geometry included with the task under the repository's MIT licence. They do not use a scanned person or commercial product mesh. The YAM robot and Isaac runtime retain their upstream licences. Product and hair-care links describe physical preparation; they are not redistributed assets.
