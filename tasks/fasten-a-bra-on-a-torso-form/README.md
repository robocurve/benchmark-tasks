# Fasten a bra’s back hooks on a torso form

Fasten both hooks in the outermost eye row and leave the band untwisted and closed on the form. The skill is holding two elastic ends under tension while engaging small ungated hooks. The garment is already on the form; putting it on and assessing comfort are outside the task.

## Setup

Use a rigid polymer bust manikin on a 240 × 300 × 10 mm base. Mark a horizontal band line 155 mm above the tabletop, through the two tape centres. At this line the bare oval torso measures 260 mm side to side and 160 mm front to back. Measure both diameters across the bare form before fitting the garment. The main body is 310 mm high and the neck brings the total height to about 383 mm. The non-working garment and 12 mm shoulder straps are supported on the form. The two working elastic wings are free at their closure ends.

Use a stretch-polyamide bra with a 32 mm rear closure edge, fitted with the **[BraBuilders Mini 2 Hook and Eye closure](https://www.brabuilders.com/products/hook-and-eye-closures)**. The supplier specifies 32 mm height, three adjustment positions and 55–56 mm closed width. Use the purchased closure unmodified: do not move eyes beyond the fabric edge, enlarge the hooks or add a latch. A bra sewn from the [Simplicity 8711 pattern, size 32A](https://madalynne.com/wp-content/uploads/2018/08/MADALYNNE-SIMPLICITY-8711-SEW-ALONGGUIDE-AUG2018.pdf) is the reference garment: the maker specifies 30–40% stretch mesh and a two-hook, three-eye closure for this size. This is an underwire bralette; its supported front and wire are simplified in the simulator because only the free rear wings and closure interact with the robot. Fit the sourced mini closure without changing its hardware. The front and side seams are pinned to the form, so variation in the cups does not change the working closure layout.

The simulator approximates the mini closure with a 30 × 32 × 0.6 mm hook tape and 32 × 32 × 0.6 mm eye tape. Two 0.8 mm-wire hooks are 16 mm apart. Their tips face towards the eye tape from the underside of the overlapping hook tape. Three eye rows have 9 mm pitch; the outer crossbar is **3 mm inside** the eye-tape edge. The intended closed layout has about **7 mm tape overlap**, with the hook tape outside the eye tape. Sub-millimetre wire contours and stiffness are approximations, not measured supplier tolerances. The robot must overlap, insert, tension and release the ordinary closure; fabric collision remains active.

For a real reset, pin the side seams so each free working wing has an 83 ± 2 mm anchor-to-tape span at rest. Measure each unloaded span in a straight line from the last fixed wing anchor to the midpoint of the wing-to-tape seam. The anchors are 217.1 mm apart, each 61.3 mm behind the torso centre at band height. Mark the anchor positions on the form and use the same garment and pin marks for each trial. Use a removable ruler template to place the open tape centres 80 ± 2 mm apart at the stated height, then remove it before starting. Keep the unmodified closure faces parallel to the rear plane, with hooks on the underside and eyes facing outwards. Shoulder straps and the non-working cups remain supported; neither working tape is clipped or guided.

Coordinates are metres, with +X forward from the station and +Y to its left. The table is at Z=0.7475. The torso body centre is (0.395, 0, 0.9025); the loose tape centres are (0.270, ±0.040, 0.9025), facing backward. Each working wing has an 83.24 mm rest anchor-to-tape span. `geometry.py` defines the complete fixed layout; there is no object jitter. Both YAM arms use 40 × 6 × 1 mm precision finger extensions with a 4 N jaw limit.

## Scoring

Score the final state using the highest qualifying tier, after both open hands have been at least 55 mm clear for three seconds:

- **1.0:** both hooks remain in their outer-row eyes throughout those three seconds, with neither wing twisted.
- **0.5:** one intended hook remains engaged throughout; the wings are untwisted.
- **0:** otherwise, or if the fabric/hardware is damaged or a working wing exceeds 1.6 times its rest span at any point.

Photographs show the engaged hooks and tape orientation; video shows release and retention under the garment’s own tension. The simulated eye-in-pocket and orientation checks implement those visible conditions. Grasping or bringing the ends together earns no credit. Reset by unhooking both ends and restoring the loose starting layout; replace damaged material.

## Simulation limits

Native compliant end links approximate the working wings, with 20/100/20 N/m translational stiffness and finite rotational compliance. Intermediate ribbon shape is drawn between their endpoints: distributed cloth contact, wrinkling and self-collision are omitted. Reinforced tapes are rigid. Hooks and eyes have native collision; there is **no auto-latch or engagement joint**.

Contact-checked compliant pinch joints approximate finger friction, limited to 5 N and 0.02 Nm. They start at the measured pose, transmit reaction forces, and release on opening or slip. The same model and hardware apply to Dream and the scripted demo. Object poses are written only at reset; the demo supplies ordinary robot joint commands.

## Run and score

Use the pinned Isaac/YAM environment in `ENVIRONMENT.json`. Native physics and cameras require a CUDA GPU; frozen rescoring runs on CPU. Dream reads `DREAM_API_KEY` or the existing private Dream credential file.

```bash
python tasks/fasten-a-bra-on-a-torso-form/run.py --mode demo --out /tmp/bra-demo
python tasks/fasten-a-bra-on-a-torso-form/run.py --mode dream --out /tmp/bra-dream
python tasks/fasten-a-bra-on-a-torso-form/rescore.py /tmp/bra-demo
```

Limit: 1,800 controls, or 90 seconds at 20 Hz. The runner records every action, native pose, event and camera frame, plus a hashed source snapshot. `rescore.py` verifies and uses that snapshot; it does not rerun physics. Seed/layout: fixed, no randomisation.

## Revision

The previous version's outer eye bars projected beyond the tape and removed normal overlap. This revision replaces that arrangement with the sourced closure above: recessed eyes, underside hooks, active tape collision and an overlapping insertion path. No auto-latch or success-pose update is added.

## Validation

The revised native demo completes **580 controls, score 1.0**. Both hooks remain engaged for 5.45 seconds with clear hands, with **6.79 mm tape overlap** and no damage. The matching Dream baseline completes **1800 controls, score 0.00**. Frozen-source rescoring, contact-location checks and all-frame video checks pass. Native wall time on the shared L40S was 10.7 minutes for the demo and 29.0 minutes for Dream.

The complete uncut rear view is the review video. See the metadata clarification record for wording updates made after recording.

Dream ran through the official loopback SSH bridge, with its credential kept on the local computer. Its client made 73 API attempts across 37 sessions, including 36 expiry recoveries in one uninterrupted physical episode. These are recorded in `evidence/bridge-runtime.json`; earlier videos are superseded.
