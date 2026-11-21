# Polish the outside of a wine glass

Polish five dried water marks from the curved outer bowl of an empty wine glass. Support the glass close to the bowl, keep the microfiber against the glass as it turns, then set it upright. Either arm can take either role. The outcome is a clear, intact glass with both objects released. The domestic care method uses a lint-free cloth, keeps torsion off the stem by supporting the bowl independently of the foot, and checks the finish against the light. [Zwiesel Glas handling and care](https://www.zwiesel-glas.com/en/shop/handling-care/)

## Equipment and placement

Measurements below are in millimetres. X extends forward from the midpoint between the YAM bases, Y extends toward the left arm, and Z extends upward from the floor. The two bases are 610 mm apart.

| Item | Physical size and reset |
| --- | --- |
| Worktop | 1,060 × 1,180 flat laminate top at Z = 747.5. The centre is at (343, 0). Keep the work area between X = 200–550 and Y = −350–250 clear. |
| Empty wine glass | A clear, machine-made 142 g stemmed glass, 205 tall, with a 70 mm foot, 12 mm upper stem, a 70 mm open rim and a curved bowl approximately 90 mm at its widest and 2 mm thick. Start it upright at (443, 146) on its foot, dry. |
| Water marks | Five 7–9 mm visible drying rings on the near-right bowl quadrant. Set the centreline 75° from the side facing the robot toward negative Y. The two lower rings are at heights of 130 and angles −18° and 16° from that line, the centre ring is at height 148 and angle 0°, and the two upper rings are at height 166 and angles −16° and 19°. |
| Cloth | An ordinary clean lint-free 300 × 300 microfiber glass cloth folded to roughly 110 × 64, with the near half gathered loosely so the gripper can pinch the doubled cloth. Leave the working edge doubled, about 47 wide. Lay it at (403, −245), with that edge pointing 75° from +X toward +Y and a gathered fold about 20 mm thick. |

Use the same kind of glass across trials. With the glass on its side in a temporary soft cradle, make the water marks from small droplets of hard tap water and let them dry fully. A dropper and a paper arc template locate the five rings. If a water source makes faint spots, use bottled mineral water in the same dropper and use that source for every reset. Remove the preparation tools and cradle before running. Photograph the five visible rings under constant oblique light. Check that a spare ring on the same glass wipes away with lightly moistened microfiber. Prepare the polishing corner with the same light moistening throughout a cohort, using water only.

The physical scene can be exported with `stemware_polish.builder.create_assets(directory)` after starting Isaac Lab. The resulting USD geometry is in metres.

## Check the outcome

Use a fixed light and a corresponding before/after photograph at each marked orientation. After the hands withdraw, rotate the glass for inspection if a single picture does not show all five locations. A ring is clear if no visible arc remains at its original location under the same light. Judge contact at the upper stem or bowl, and confirm the glass returns upright and both objects are released.

- Award 0.16 for each original mark removed from the intact glass, up to 0.8.
- Award 1.0 if all five marks are gone, the clean glass is upright with its foot flat, the cloth rests on the bench, and both hands stay at least 60 mm from the upper stem and 50 mm from the centre of the cloth for 0.75 seconds.
- Award 0 if the glass is visibly cracked, chipped, dropped off the bench or toppled. An actual evaluator can use video and a visual damage check.

Polishing counts only during loaded microfiber contact and tangential movement across a marked area. The native pad joint represents compression of multiple cloth layers. A native deflection of that joint provides the contact estimate. The scorer also checks the pad face against the live bowl geometry before changing a water-mark overlay. It reads displacement and elapsed time. The articulated cloth represents the folded shape and localized compression, and the rounded glass wall is realized by a mesh with convex colliders around an open cavity. A rigid failure proxy checks toppling, out-of-zone falls, and excessive load at the bowl.

## Native evaluation

Install into the accepted-license Isaac Lab-Arena environment with `python -m pip install -e . --no-deps`. Use seed `1415158438`. Each trial is limited to 760 20 Hz actions, or 38 seconds. Inspect Robots registers embodiment `stemware_polish` and reference policy `stemware_polish_scripted`. The standalone runner records state and scores from native physics.

```bash
python run.py --device cuda:0 --physics-device cpu --output /tmp/stemware-demo
python run.py --device cpu --no-cameras --policy hold --output /tmp/stemware-idle
python run.py --device cpu --no-cameras --policy air-pass --max-steps 420 --output /tmp/stemware-air-pass
python run.py --device cpu --no-cameras --policy short-pass --output /tmp/stemware-short-pass
```

The ordinary-action reference supports and rotates the glass by its upper stem while the other arm picks up the cloth, polishes upward and sets it down. It uses the measured cup and cloth poses to follow the bowl and align the foot with the worktop. Native assets are generated by this package under the repository MIT license. The robot and cameras come from the pinned YAM adapter. [YAM license](https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/yam_arena/assets/yam/LICENSE.i2rt) · [Camera asset license](https://github.com/ARISE-Initiative/yamlab/blob/ec0455d2b4ce35f21fc126418ea5e74ac567133d/LICENSE)
