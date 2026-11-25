# Physical reel build and inspection

This is a proposed build for a non-powered bench fixture. The dimensions reproduce the simulation's winding surface and crank approach. The fixture and physical inspection procedure have yet to be tested on hardware.

## Build the fixture

Use a 12 mm smooth steel shaft and two dry-running 12 mm pillow blocks. The dimensions below use the [igus KSTM-12 drawing](https://igus.partcommunity.com/23d-libs/igus/igubal_gelenklager/stehlager/kstm_ksti_english.pdf), which puts the bore 14 mm above the mounting face. The standalone build uses a flat steel base and two bolted, gusseted pedestals.

To adapt the Garant E, start with the unplugged unit and detach its 240 mm drum and cable together. Set the electrical assembly aside intact. Use the donor frame as the stand and bolt or clamp two rigid bearing brackets to its frame tubes. Lay out their bearing holes using the pedestal dimensions below and shim the stand or brackets until the shaft is level at 160 mm above the table. Fit the same stock 12 mm shaft, bearings, replacement drum, crank and short cord described below. Check that the brackets and donor frame clear the full crank sweep and leave the winding bay open. Use the standalone base if the donor frame cannot provide those clearances.

All dimensions are in millimetres. Set axial position zero at the middle of the winding bay. Positive positions are toward the crank. The table is the vertical datum. Center the hub and both flanges on the shaft.

| Part or feature | Dimension and placement |
|---|---|
| Base | 280 along the axle × 200 front-to-back × 6 steel plate. Center it below the drum and clamp it to the worktop at the four corners. |
| Pedestals and bearing blocks | Center the blocks at axial positions −100 and +100. Set both mounting faces at 146 above the table for a KSTM-12. Use a flat 75 × 30 mounting face on each pedestal, with Ø5.5 holes 46 apart across the axle. Shim under the blocks until the shaft center is 160 ± 1 above the table and level within 1 mm between blocks. |
| Shaft and retention | Ø12 × 310 steel shaft with ends at −138 and +172. Retain it axially with two 12 mm split collars outside the bearing blocks. Leave 0.5–1 mm combined endplay so the collars do not clamp the bushings. |
| Drum | Smooth plastic hub, Ø96 ± 0.5 and 112 ± 0.5 long, with two plastic flanges, Ø164 ± 0.5 × 6. The hub ends and inner flange faces are at −56 and +56. Ream or bore the concentric shaft holes to a slip fit. Bolt the flanges to the hub with countersunk screws and use a 12 mm clamp-on shaft hub screwed to the outside of a flange to transmit torque. Keep screw heads flush inside the bay. Use a hollow hub with internal room for the cord clamp. |
| Crank | Clamp a steel crank arm to the shaft with its outward knob mounting face at +157 ± 2. A 30 mm wide, 8 mm thick arm reaches from the shaft clamp to an M8 tapped hole centered 88 ± 1 from the axle. Use the 25 mm diameter, 82.5 mm exposed rotating grip described in the README. The grip extends toward positive axial positions. |
| Cord | Use 8.0 ± 0.3 mm round PVC-jacketed cable. Feed its inner end through a deburred, radiused hole at axial position −38 and secure it inside the hub with a cushioned clamp. Keep the winding face clear of fasteners. Measure 900 ± 5 along the free jacket from the hub exit to the dummy plug body. Keep both ends electrically disconnected. |

Axial stack, looking down on the reel. The distances label axial positions in mm.

```text
  shaft       bearing    flange         winding bay         flange    bearing           crank     grip tip
   −138         −100    −62 | −56            0             +56 | +62   +100              +157        +239.5
                            |<-------------112--------------->|
                                    cord exit at −38
```

With the empty hub, check for binding or rubbing through a full revolution and measure the flange radius at four points with calipers. The bare axle and loaded drum should turn smoothly from the crank without the reel shifting on the table. Do one hand-wound dry run with the chosen cord before mounting the robot. Check that the first wrap lies on the hub, the guide hand can reach the incoming cord, and the cable stays on the drum after release. Record the actual cord diameter and the model of the bearings. If used for a sim-to-real comparison, also record the empty reel's breakaway force with a spring scale pulling tangentially at the grip center. Multiply newtons by 0.088 m to report torque. This records a physical difference for comparison. It is outside the task score.

## Inspect the released coil

1. Before the run, put matching index marks every 30° on the **outside faces** of both flanges, numbered in the same direction from the cord anchor. Check the depth from the flange-rim straightedge to the empty hub with the gauge described below. For nominal dimensions it should read 34 mm. Keep the gauge, camera scales, and inspection tools clear of the task during the run.
2. After release, photograph the coil in place. Use overlapping views perpendicular to the axle and a scale in the plane of the visible turns. Use a mirror for the bottom sector. Trace the cord from the anchor to the plug and log each occupied sector in order, including the fractional turns at the two ends. Measure the loose tail along the jacket with a flexible tape after taking the photos. Subtract loose tail length from the measured free jacket length at reset. Start the tail measurement at the first exit from the winding bay.
3. In each occupied sector, follow adjacent wraps for their full shared arc. Measure axial separation at the indexes and at the closest and farthest points between them. For the constant-diameter cord, the distance between corresponding same-side jacket edges is the center-to-center distance. Take a close-up with a scale beside the wraps wherever the gap approaches either limit or the order is unclear. The 30° indexes organize the inspection. A defect between indexes still counts.
4. Check the highest point in each occupied sector. Rest a stiff straightedge on both flange rims at matching angles. Use a depth probe perpendicular to the straightedge and directed radially toward the shaft. Touch the top of the jacket lightly, without flattening it. Subtract the measured depth from the local flange radius. At a nominal 82 mm flange radius, the minimum passing depth is 21 mm. Check the shallowest depth again with the gauge reset on the rims.
5. Record the minimum and maximum pitch, maximum outer radius, wound length, turns, and which views show the clear grippers. Repeat near-limit readings with the ruler or gauge aligned at the suspect point. Report any inaccessible section or unresolved reading as insufficient evidence for certifying full credit, with its location in the photos.
