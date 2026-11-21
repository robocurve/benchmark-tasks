# Fit and lock a dining table’s extension leaf

Seat the 400 mm extension on the drawn-out frame to turn a two-seat table into a four-seat table. The broad leaf starts resting on the real support rails as it does after removal from storage. Slide both sides into alignment with the fixed tabletop, then engage the underside thumb latch. [IKEA EKEDALEN 80/120 × 70 cm product dimensions](https://www.ikea.com/ma/en/p/ekedalen-extendable-table-white-70340826/) · [Installation stages and locator hardware, pages 22–24](https://www.ikea.com/nl/en/assembly_instructions/ekedalen-extendable-table-white__AA-1934526-19-100.pdf)

## Physical fixture and reset

Use an IKEA EKEDALEN table of the specified size or a full-size mechanical fixture made to the dimensions below. With the empty table's underframe at its extended stop, steady the base. No contents or consumables are required. Build a prototype from 22 mm veneered particleboard, square-section solid-wood frame stock, nylon guide shoes and steel latch fittings. The scene generator exports the meter-scale solid model and geometry manifest.

```bash
python -c 'import isaacsim; from dining_leaf.builder import create_assets; create_assets("/tmp/dining-table-fixture")'
```

Define the horizontal X-axis as the direction of insertion. The origin is centered between the robot bases at floor level. Y points toward the left arm and Z points up. All measurements below are in millimeters.

| Component | Measurement and pose |
| --- | --- |
| Main table | The 800 × 700 × 22 main top spans X 710 to 1510. Top surface at Z 750, centered on Y 0. Underframe rails remain at full extension. The four 48 × 48 legs center on X 332 and 1472, Y ±309. |
| Open rails | Two parallel 414 × 47 support surfaces span X 305 to 719 at Z 724 and Y ±247. The 4 mm sliding shoes under the leaf carry its weight. |
| Leaf | 400 × 700 × 22 ash-veneered particleboard, 3.5 kg including fittings. Center at approximately X 444 and Y 0, with +1.2° rotation around vertical. Place its shoes on both runners. The resulting center seam gap is 66 mm. |
| Locator fittings | Two 30 × 16 × 13 nylon keys attach 182 mm inward along X from the leaf's center at Y ±205, with their centers 15 mm below the leaf's mid-plane. The two five-centimeter receiver channels on the frame open to the leaf and have 25 mm clear width. |
| Flip latch | A thumb lever pivots about Y at X 309, Y 0 and Z 679. Its 47 mm pivot-to-thumb radius carries the capture end behind the centered underside receiver near the leaf's outer edge. The open position points out from the table. The engagement stop is 83° above open. |
| Robot stands | Two separated mounts at X 0, Y ±305, and Z 750. Bring both arms to ready, clear of the leaf and latch. |

Keep the latch fully open during reset. The offset leaf should sit flat on the rails without contacting either receiver. Verify the whole fixture stays in the same position when the hands withdraw. This is an original rigid-body recreation of the leaflet alignment and thumb-lever operation using readily fabricated guides and strike.

## Scoring

Use a millimeter rule and short straightedge at two points on the seam 25 mm inward from the ends. The main tabletop is the reference. Return 0.30 for both gaps within 28 mm and both side offsets within 18 mm. Return 0.65 for both gaps within 6 mm, both side offsets within 8 mm and both vertical steps within 4 mm. Award full score when the latch also traps the receiver and holds the fitted leaf stationary for one second with the grippers clear. The native latch rotates to 76–86° in the locked position. Check from below that the catch sits outward of the striker with no more than 8 mm play. The catch must overlap the striker by at least 12 mm across the table and 6 mm vertically. An outward pull on the leaf then bears against the catch. Score zero outside the partial-credit bands. Either arm may work either side or operate the latch.

## Run the validation

Use the pinned Isaac Lab-Arena and YAM environment from `pyproject.toml`. Default seed is 1415158438. The cutoff is 370 actions at 20 Hz with 240 Hz native rigid-body contacts.

```bash
python -m pip install -e . --no-deps
python run.py --device cuda:0 --physics-device cpu --output /tmp/dining-leaf-demo
python run.py --no-cameras --policy hold --output /tmp/dining-leaf-idle
python run.py --no-cameras --policy unlatched --output /tmp/dining-leaf-unlatched
```

Inspect Robots embodiment `dining_leaf` and reference policy `dining_leaf_scripted` are registered by the task package. The reference relies on elapsed time and ordinary joint positions. It pushes the real panel at both corners, rotates the latch by contact and withdraws. The stationary control leaves the leaf offset. The unlatched control performs the same alignment without moving the keeper. The runner records diagnostics and score through all actions. Camera mode records the three native 640 × 360 views, including reset and every action frame.

## Asset provenance

All fixture solids are original and released with this task under the MIT license. The YAM arm and cameras use the pinned upstream adapter. [YAM license](https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/yam_arena/assets/yam/LICENSE.i2rt) · [Camera license](https://github.com/ARISE-Initiative/yamlab/blob/ec0455d2b4ce35f21fc126418ea5e74ac567133d/LICENSE)
