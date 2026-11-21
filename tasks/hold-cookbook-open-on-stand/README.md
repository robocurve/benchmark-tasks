# Secure an open cookbook with its page holders

Hold the Lemon Potatoes recipe open so it can be read while preparing food. Both spring holders swivel from the face of the ledge onto the lower edge of the paper. Each holder reacts against one half of the binding as that half tries to spring toward closed. The two arms work on opposite holders. Either arm can operate either holder, and the scoring is independent of their assignment and actuation order. [Maison & White stand and its two page holders](https://www.diy.com/departments/bamboo-recipe-reading-holder-book-stand/5055884519884_BQ.prd)

## Physical setup and reset

Use a flat 1080 × 1250 mm laminate worktop whose surface is 747.5 mm above the floor. The arm mounting planes are 750 mm above the floor. The arm bases are 610 mm apart. X points from the base line away from the robots. Positive Y points toward the left arm.

| Part | Specification and placement |
| --- | --- |
| Recipe stand | Folding 335 × 240 mm bamboo board, 12 mm thick, with a wood ledge, two metal swivels and a rear prop. Recline the backrest 30° from vertical toward positive X. Its front face at the center just above the ledge is [407, 0, 775.5] mm. Put the lower ledge directly below that reference. The ledge is 335 × 30 × 15 mm. A rubber mat or removable bench clamps can keep the stand steady in a physical fixture. |
| Page holders | Use the two swiveling metal page holders supplied with the stand. The simulated steel strips reach 72 mm from pivot to rubber pad center. Pivots are 110 mm either side of center, 44 mm in front of the backrest plane and 9 mm below the datum above. Each modeled rubber pad is 7 × 13 × 12 mm. Start both strips pointing inward along the ledge at 0°. Their raised stops are 103° above that line. |
| Cookbook | A small hardcover 207 mm high and approximately 149 mm wide when closed, with a 12.5 mm paper stack and a 2.4 mm cloth-backed cover on each side of the selected spread. The two bound stacks and spine total 332 g in the simulation. Mount the supplied uncoated prints flat on the two facing leaves at the center opening. Align the lower print edge 2 mm above the bottom of the cover. Center the spine on the ledge with both printed panels facing the arms. |

Qualify the book with the holders stowed and its spine on the ledge. Support one half flat against the backrest. Hold the other half at 5°–10° with a small push force gauge normal to the printed face and 100 mm from the binding. Use a hardcover that takes 0.10–0.60 N to hold on each side. Repeat on the opposite half. After releasing the gauge, each half should settle at 8°–20° from the backrest, equivalent to a 14–34 mm lift at the gauge location. Reset with both holders stowed, the spread exposed and both arms in the standard YAM ready pose.

Generate the original print artwork at the real leaf size with the following command. It writes two PNGs in the current directory. Print each at 143 × 203 mm.

```bash
python -c 'from cookbook_stand.pages import art; [art(i).save(f"page-{i}.png") for i in (24, 25)]'
```

## Physical score

Check a continuous one-second segment ending at the result. The spine must remain centered within 20 mm and resting on the ledge (simulated tolerances of 6 mm down, 12 mm up and 15 mm in depth from its reset datum). Both arms must be clear of the work. Use an overhead or wrist photo and a ruler to see whether the rubber pads are touching the printed face with spring pressure. A side earns 0.5 when all of these conditions hold for that second:

- Its clip points 72°–105° up from the stowed direction.
- The pad presses its own printed face. Measure within 1.5 mm of the paper and no visible loss of spring load. The native scorer quantifies load as at least 0.8 mm of inward spring compression from the unladen stop.
- Pad center is between 18 and 110 mm from the lower page edge and between 20 and 140 mm from its spine hinge.
- The bound page block is within 10° of parallel to the backrest.
- All of its recipe text is clear of both holders. View the printed face straight on, perpendicular to that leaf. The recipe rectangle starts 10.3 mm from the left and 7.1 mm below the top edge of the 143 × 203 mm print, and measures 124.7 × 97.2 mm. Use the digital print template as a photo overlay or use a ruler. The projected outlines of the pivots, spring strips, bends and rubber shoes must stay outside the rectangle, which contains the title, ingredients, quantities, instructions and cooking time. The lower margins accommodate the clips. The native scorer checks the projected bounding rectangle of each holder part.

Both sides retained earn 1.0. One retained side earns 0.5. An attempt with neither side retained earns 0. The camera demo ends when the hold has completed.

The simulation represents each bound half as a passively spring-loaded paper block while preserving the angle, relative clearances and physically driven swivel of each holder. A short prismatic spring at the shoe represents the compliance of the bent metal strip. The stand is fixed in its working pose, with independent native joints at both metal swivel pivots.

## Run and score

Use the accepted-license Isaac Lab–Arena runtime and the pinned adapters in `pyproject.toml`. The fixed seed is 1415158438. Maximum runtime is 450 actions or 22.5 simulated seconds at 20 Hz. The scripted robot uses elapsed time and measured joint positions as standard motion feedback.

```bash
python -m pip install -e .
python run.py --device cuda:0 --physics-device cpu --output /tmp/cookbook-demo
python run.py --no-cameras --policy hold --output /tmp/cookbook-hold
python run.py --no-cameras --policy parked --output /tmp/cookbook-parked
```

The Inspect Robots embodiment is `cookbook_stand`. Each invocation saves a score receipt and simulation trace. The hold control keeps both grippers in the reset pose for 450 actions. The parked control makes an incomplete closure attempt. In camera mode each native stream records the initial state and every action frame at 640 × 360, 20 fps.

## Assets

The worktop, cookbook, spring holders, bamboo stand and printed pages are original geometry and artwork released with this task's MIT license. The original page artwork uses the default renderer font distributed by Pillow. The YAM robot and camera assets come from the pinned robot adapter. [Robot asset terms](https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/yam_arena/assets/yam/LICENSE.i2rt) · [Camera assets license](https://github.com/ARISE-Initiative/yamlab/blob/ec0455d2b4ce35f21fc126418ea5e74ac567133d/LICENSE) · [Pillow font and library license](https://github.com/python-pillow/Pillow/blob/main/LICENSE)
