# Remove pin bones from a salmon fillet

Use fishbone tweezers to remove the four remaining pin bones, put them in the waste dish and leave the fillet intact. This is the pin-boning step of food preparation, not filleting a whole fish.



## Run and score

Use the Python 3.12 / Isaac Sim 6.1 / YAM environment pinned in `ENVIRONMENT.json`:

```sh
OMNI_KIT_ACCEPT_EULA=YES python tasks/remove-salmon-pin-bones/run.py --mode demo --out /tmp/salmon-demo
OMNI_KIT_ACCEPT_EULA=YES python tasks/remove-salmon-pin-bones/run.py --mode dream --out /tmp/salmon-dream
python tasks/remove-salmon-pin-bones/rescore.py /tmp/salmon-demo
```

The limit is 1,800 actions, 90 seconds at 20 Hz. There is one fixed reset, without random jitter. Both policies use the same robot, tools, physics and ordinary camera observations. The demonstration commands robot joints only. The offline audit checks frozen source hashes and recomputes the score from recorded native states; it does not simulate physics on a CPU. The recorded demo took 21.3 minutes and Dream took 21.8 minutes on an NVIDIA L40S, including initialization and rendering.

## Physical setup

A fresh, skin-on Atlantic salmon portion, held chilled at 2–5 °C until the trial, lies skin down on a wooden board. Nominal portion dimensions are 150 × 90 × 25 mm; select a central portion within 10 mm of these dimensions. Keep four natural pin bones spaced roughly 20–40 mm along its centre row, removing the others before the trial. Use a fishmonger-supplied portion that has not already been pin-boned. Do not insert replacement bones into an artificial rigid block. Brush the flesh gently against the tips so each end is visible, protruding about 3–5 mm, without loosening the embedded length. Photograph the starting surface beside a ruler.

In the fixed simulated reset, the fillet is centred at (360, 0) mm, its long dimension along Y. Four 28 mm-long, 1.2 mm-diameter bone contact envelopes lean 45 degrees toward −X. Their row coordinates are −55.5, −18.5, 18.5 and 55.5 mm. The simulation uses deliberately uniform geometry; biological specimens vary. The real reset tolerances are not claims that every salmon has these exact bones.

The board is 280 × 260 × 12 mm, centred at (360, 0) mm and held against slipping. A loose 90 mm-diameter, 24 mm-deep waste dish is centred at (320, −215) mm. X points forward from the midpoint of the arm bases, Y left; the tabletop is 747.5 mm above the floor. Both arms start clear.

The robot uses mounted stainless fishbone tweezer leaves: 140 mm overall leaf length, with 40 × 6 × 1 mm working ends. Their mounting gives a 218 mm working-point offset from the gripper origin. Maximum actuator effort is 4 N. The simulator substitutes parallel jaw closure for the flexing handles of handheld fish tweezers. It does not test picking up or reorienting a loose tool. The opposing hand supports the flesh beside the bone. This mounting and its narrow tips are part of the hardware, not an invisible grasp aid.

The food-preparation procedure follows [Sitka Seafood Market's pin-boning guide](https://sitkaseafoodmarket.com/blogs/cooking-guides/how-to-remove-pin-bones-from-salmon): grip the exposed end and draw it in its existing direction. The closest accepted task is [splinter extraction](https://github.com/robocurve/benchmark-tasks/pull/2403), which shares fine tweezer manipulation. Here the objects are naturally retained fish bones in an unrestrained food portion; local flesh support, intact recovery and collecting all remaining bones are assessed.

## Scoring

After both hands have released and stayed at least 50 mm clear for three seconds, award 0.25 for each whole bone fully removed and lying entirely inside the waste dish. Four earn 1.0. A bone still embedded, held or outside the dish earns no credit.

All credit requires the dish upright on the table (within 15° of level), the fillet remaining on the board, no broken bone, no detached flesh piece 5 mm or larger, and no tear extending a starting entry opening by 5 mm or more during the trial. The tiny original pin-bone openings are allowed. Photograph the fillet and recovered bones beside a millimetre ruler. After recording the final surface, gently open each entry site with spare tweezers and recover any remaining bone fragment. A broken-bone fragment 3 mm or longer, or a visibly fractured recovered bone, fails the intact-bone condition. A whole bone left in its original site is an uncompleted extraction, not a fracture, and simply earns no credit. This post-trial inspection is not part of the robot action limit. Use the trial video for damage that later becomes obscured. Any failed common condition scores zero, including a shattered bone even if its pieces reach the dish.

## Material approximation

Twelve yielding sectors approximate local flesh compression and damage; a 5 mm displacement breaks the affected tissue bond irreversibly, the lower skin is one rigid body. Four thin bones each have a bending, breakable joint. The visible fillet surface interpolates small gaps between intact sectors; it separates with the actual sector positions after damage. A directional sliding constraint approximates the distributed tissue resistance along an embedded bone, capped at 0.35 N. It releases only after the entire embedded length has been drawn out. Native reaction forces can move the free fillet, deform its flesh or break a bone. These uncalibrated values are a qualitative material model, not measurements of salmon tissue.

The narrow internal bone channels use collision filtering while tissue constraints provide resistance. Bone/flesh contacts are restored once the bone is clear. This avoids modelling microscopic flesh/bone interfaces. A finite, compliant native pinch is created only when both closing tweezer faces enclose a tip; it transmits reaction forces, releases on opening, and slips at 1.2 N, 0.006 Nm or 3 mm of relative motion. It never sets a bone pose or awards extraction automatically. Source and complete native state traces are retained for auditing.

## Reset and assets

Use a new fish portion for each real trial; extracted bones and torn flesh are not resettable. Restore the board, dish and tool layout from the setup photograph. Keep the fish chilled before use and clean the detachable steel leaves, board and dish between portions; trial food is not served. Fit disposable polyethylene sleeves over the grippers and wrists, leaving only the washable steel working leaves exposed. Place the board and dish on a washable silicone mat to protect the shared table. The sleeves and mat provide hygiene protection outside the manipulation contacts and are omitted from the simulated dynamics.

All task geometry is original procedural work under the included MIT licence. The YAM robot, station and simulator are supplied by the pinned runtime. No third-party task meshes or textures are redistributed.

## Validation

The scripted demo completes 1,720 actions and scores **1.0**: four intact bones in the dish, 5.35 seconds with clear hands and no native breaks. The maximum simulated flesh displacement is 1.44 mm, below the 5 mm damage threshold. The complete 1,800-action Dream run scores **0.0**; it does not extract a bone.

Both runs use identical simulation sources and instruction. Frozen trace rescoring, jaw-contact audits and four-view visual review pass. Videos show synchronized native views at 20 fps without time cuts. The Dream adapter recorded 75 request attempts, 37 server sessions and 36 expiry recoveries; these SDK recoveries may reset server state and are disclosed rather than described as one uninterrupted session.

The CPU rescore reader was corrected after recording to serialize NumPy integers. It verifies the original file hashes and calls the unchanged frozen scorer; the physics, score rules and tolerances are unchanged. Runtime hashes, reports, contact checks and trace hashes are in `evidence/`. Full traces and frozen sources are retained with the trial artifacts.
