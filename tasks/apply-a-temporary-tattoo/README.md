# Apply a water-transfer temporary tattoo

Apply the tattoo inside the marked area on a curved practice forearm, then remove the backing without losing the picture.

The native demo scores **1.0** in 1,560 actions (78 seconds); Dream-1 scores **0.0** over its full 1,800-action limit. Both runs pass independent rescoring.

## Setup and reset

Use a 200 mm long, 64 mm diameter practice form covered in smooth PVC vinyl, supported horizontally on two 25 × 70 × 26 mm grooved rests. Its centre is 45 mm above the table. Mark a 40 × 35 mm rectangle on top, measuring the short dimension around the curve. Fit removable waterproof gripper-pad covers.

The transfer backing is 50 × 60 mm, with the supplied 35 × 30 mm pixel-heart artwork and an unprinted margin. It starts image-side down on a 30 × 40 × 30 mm rest, separate from the form, with its protective film removed. A pre-wetted 40 × 40 × 25 mm cellulose sponge sits in a 70 mm square shallow plastic dish.

The fixed world centres are: form `(0.34, 0.025, 0.7925)` m; paper `(0.34, 0.13, 0.7778)`; sponge `(0.34, -0.16, 0.7725)`. The tabletop is at Z = 0.7475 m. Both hands start open. `geometry.py` gives the remaining parts and the setup image shows the settled layout.

Use classic water-transfer tattoos made from [heart-artwork.svg](heart-artwork.svg), at 100% size without fitting to page. The file contains only the tattoo artwork; its canvas is the required backing size. Order a batch from a temporary-tattoo printer or use compatible printable transfer stock. Use the same stock for every trial. [TemporaryTattoos.com documents](https://www.temporarytattoos.com/blogs/news/how-to-apply-a-temporary-tattoo) wet application and use on smooth plastic; this does not establish compatibility with every PVC cover.

Before a batch, qualify one hand-applied sample on the chosen cover using 30 seconds of wet contact. For each trial, clean and dry the form, replace the tattoo, add 5 ml clean water to the sponge and squeeze out drips. Remove old ink and replace a cover that retains residue. Use no semi-permanent ink or dry-rub transfer. Preparation is outside the robot trial.

## Judging

Use a close photograph, the reference artwork and a ruler:

- **1.0:** at least 95% of the image is intact inside the outline, the backing is completely removed, and both hands are released for three seconds.
- **0.5:** at least 75% is intact inside the outline and the backing is removed.
- **0.25:** at least 50% is intact inside the outline.
- **0:** all other outcomes, including a torn backing.

The 27 equal-area red tiles make missing regions easy to count. The simulation counts transferred tile centres inside the target and rejects merged or displaced image tiles. Inspect the end of the video as well as the score.

## Simulation

The paper consists of twelve hinged bands. The sponge has sixteen passive compression elements. Native collisions and ordinary robot joints perform pickup, placement, pressing and peeling; there are no hand attachments or object-pose edits. The demo regrips the blank margin after wetting using measured object poses. Dream receives only its normal cameras and joint observations.

A reduced material law requires the printed face to contact the form and a loaded wet sponge to contact the backing. Water can wick a short distance through contacting neighbouring paper. After 30 seconds of local wet contact, a weak coating bond becomes available; its physical separation leaves that image tile on the form. Early removal loses that region. This approximates water transport and adhesive release, not individual ink chemistry. The paper's transverse bending, coarse tear limits, sponge stiffness and coating adhesion are uncalibrated. The same material law applies to both runs.

Geometry and artwork are original, under the included MIT license; there are no imported third-party assets.

## Run and score

Use the pinned Python 3.12 / Isaac Sim 6.1 / YAM environment in `ENVIRONMENT.json`:

```sh
python tasks/apply-a-temporary-tattoo/run.py --mode demo --out /tmp/tattoo-demo
python tasks/apply-a-temporary-tattoo/run.py --mode dream --out /tmp/tattoo-dream
python tasks/apply-a-temporary-tattoo/rescore.py /tmp/tattoo-demo
python tasks/apply-a-temporary-tattoo/rescore.py /tmp/tattoo-dream
```

Configure Dream through its normal credential store or the local bridge in `dream_policy.py`. Keep credentials outside this folder. `make_plan.py` regenerates the joint trajectory; `demo_control.py` handles the regrasp.

The limit is 1,800 controls at 20 Hz (90 seconds), with a fixed layout and no random seed. Runs save native videos, start/final images, continuous body-state traces and frozen source. Independent rescoring checks source hashes, action bounds, native break history and the outcome. Judging cameras are not policy inputs. Measured wall times on an RTX PRO 6000 Blackwell were 1008 seconds for the demo and 1720 seconds for Dream (37 requests). These include simulation and inference and depend on hardware load. `VALIDATION.json` records the frozen source hashes and audit results. Uploaded videos place the native front and close side views next to each other, retaining every frame at 20 Hz. The final image contains all 27 transferred tiles; a separate polygon-area check places 98.72% of the ink inside the outline.
