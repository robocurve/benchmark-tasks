# Split-ring revision validation

The revised splitter engages the two touching 1.98 mm turns **25° behind the red tip**. The 7.9 mm offset along the coil allows the brass bow to pass the free tip with the catch still raised. The scorer grades the released state from measurable wire geometry and signed tip passages.

## Revised fixture results

| Scripted trial | Native score | Actions | Released outcome |
| --- | ---: | ---: | --- |
| Full thread | **1.0** | **2,126 / 2,400** | Two turns retained over a continuous 2.00 s hold. [Four-view demo](evidence/full-demo.mp4) |
| One turn | **0.5** | 1,674 | One turn retained, with the nearest marked tip 28.7 mm away. [Partial demo](evidence/partial-demo.mp4) |
| Offset hole | 0.0 | 955 | Key on the table, with zero crossings and zero passages. [Control](evidence/offset-hole.mp4) |
| Forced placement | 0.0 | 935 | Key on the table, with zero crossings and zero passages. The released ring had a 20.03 mm red-end gap. [Control](evidence/direct-placement.mp4) |

The red tip opened to a 4.87 mm surface gap at action 420. The same marked tip passed through the hole at actions 589 and 1,809. During actions 2,087–2,126 the released key's hole stayed at least 219 mm above the table and 34.9 mm from either tip. Both grippers stayed more than 204 mm away from the key and ring throughout that hold. The final outer diameters were 37.998 mm along both axes. The tip gaps were 0.030 and 0.000 mm, with zero visible break.

The one-turn control rotated the vise jaw away from the release path before the right gripper released the key. The final key crossed one strand, with one verified passage. Contact over the aperture rim was about 0.15 mm. The retained-strand check uses the combined 0.16 mm contact offsets as its tolerance. The marked tip still has to fit through both faces with 0.05 mm tolerance.

The revised trials used seed 0, 20 Hz control and 1,000 Hz CPU physics in the pinned Isaac Sim environment. The GPU rendered the native cameras at 10 Hz. The recorded runs took 3.8–8.4 minutes each on the workstation. The controller used fixed setup waypoints and robot IK. The scorer and diagnostics read the measured key, ring and splitter poses.

Rescoring the full and one-turn traces reproduced their native scores and the full hold boundary. Both negative controls used the final collision assets and motor commands. Their traces recorded zero marked-tip passages and rescored to zero with the final rubric.

The 51 scoring checks include a wire break, residual gap, diameter and roundness changes, reversal of a passage, two valid turns from the same free end, and a pressed-through solid bow. The revised STEP file re-imports as 49 solids. CAD interference checks passed for the guide, lift pins, carriage, shoe, catch and blade arm at park, halfway in, and fully in, with the lift at both stops. The ring lot and inspection bands still need physical calibration. [Drawing](fixture/fixture-drawings.pdf) · [Physical fit and repeat-reading procedure](fixture/BUILD.md)

## Dream-1 on the revised fixture

The evaluator initialized the revised scene and logged 32 warmup actions. The first inference request returned `DA-1000 UNAUTHORIZED (disabled)`. Dream-1 returned zero model actions, and the evaluator recorded one errored trial with no score. The submission includes the scripted demo, reset views and the service error. [Evaluation record](DREAM_EVALUATION.json) · [Reset views](evidence/setup.png)

## Earlier geometry and evaluation

**Forced placement scored 0.0 after release**, at action 935 of 2,400. At 28 s the oblique view shows the gripper pressing the key bow across the coil. Around 37 s the gripper opens and the key falls. The final front and edge views show the empty, reclosed ring. The 20 Hz trace places the key hole centre 1.0 mm above the tabletop from action 745 onward. The 10 Hz, 468-frame composite is synchronized and decodes in full. [Forced placement through release](evidence/forced-placement.mp4)

The forced run had zero captured strands and zero marked-tip passages. The native score and post-hoc signed-passage rescoring both stayed at 0.0. Final outer diameters were 37.998 and 38.036 mm, the two tip surface clearances were 0.152 and 0.002 mm, and the largest visible wire separation was 0.000 mm. Both hands cleared the key by at least 164 mm and the ring by 129 mm. Wall time was 40.6 minutes on GPU 0. The simulator wrote all 935 actions, results and closed camera streams before its process received SIGTERM during teardown.

The original approach in this run started opening the red tip at action 365, then the loaded radial carriage backed out. Its maximum tip gap during the opening stroke was 0.296 mm. That trial tests direct placement and release, but does not demonstrate enough splitter clearance for a 2.00 mm key. With a fixed inward bias against the same mechanical end stop, the subsequent full-path trial opened the red tip by 0.803 mm at action 370 before the axial lift and by **4.590 mm at action 400** during lift. The red-tip gap at action 421 was 5.566 mm. The key is 2.00 mm thick. [Native camera comparison at reset and action 400](evidence/opening.jpg)

The first revision's raised catch arm touched the brass bow at action 560. When the catch withdrew, the red tip lay outside the hole. The run ended at action 672 with zero crossings, zero passages and a score of 0.0. A second direct trial stopped during approach at action 258. [Opening and failed entry](evidence/splitter-opening-and-key-entry.mp4)

The original 1.0 scripted demo, 0.5 held one-strand control, and 0.0 Dream-1 run preceded these changes. Original direct placement stopped at action 537 when one link crossed 24°. The first submission used media ID M-FNNFPU. [Original model evaluation](DREAM_EVALUATION.json)
