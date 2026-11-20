# Build the two-channel bench pump

The custom bench head is sized for two equal 8.0 mm OD × 4.8 mm ID silicone lines with a shared opening cam. The head fits on a 102 × 101 mm mounting base. Geometry is in millimetres, with X across the two tracks, Y along the tubes toward their outlets, and Z upward. `pump.scad` contains an assembly view and individually exportable parts. It uses the same rotor, track, retainer and hinge positions as the simulation, with a rotating shaft and arcuate pressure shoes for physical water pumping.

## Parts

| Qty | Part |
|---|---|
| 1 each | PETG or nylon housing and cam, motor bracket, cam lever, travel flag and slotted stop plate. Export parts from [pump.scad](pump.scad). |
| 2 each | End retainer, passive pressure shoe, rotor web, parking rest. |
| 4 | 304 stainless sleeves, 18 mm OD × 5.2 mm bore × 47 mm. Use M5 smooth-shank axles, captive washers and locknuts. Sleeves must spin freely. |
| 12 | Split PETG guide rings, 25 mm OD × 18.15 mm bore × 3.7 mm. Cut one split in each ring after printing and fit to the roller centre and outer guide positions as described below. |
| 2 | 608 (8 × 22 × 7 mm) sealed bearings. |
| 2 | Steel shafts, 8 × 145 mm for the rotor and 8 × 180 mm for the cam. Use 8 mm collars to remove end play; file flats at the clamping screws. |
| 4 | M3 shoulder screws for the shoes, 6 mm OD compression springs about 8 mm long and 0.85 N/mm, M3 threaded inserts, and steel shim washers. |
| 2 | Closed-loop steel extension springs, about 45 mm free eye-to-eye length, selected to pull 45–55 N at 66 mm. Use M3 shoulder eye studs, washers and retaining nuts. |
| 1 each | 19 mm OD × 19 mm grip sleeve bored to 5 mm, M5 smooth pin, M3 follower pin, M4 bench fasteners, M3 assembly fasteners, 8-to-5 mm flexible shaft coupler and a clear 3 mm polycarbonate shroud. |
| 1 each | 1.8° 42 mm NEMA 17 bipolar motor with a 5 mm shaft and approximately 0.6 N·m holding torque, such as the [StepperOnline 17HS19-2004S1](https://www.omc-stepperonline.com/nema-17-bipolar-59ncm-84oz-in-2a-42x48mm-4-wires-w-1m-cable-connector-17hs19-2004s1). Use a current-limited STEP/DIR driver such as [Pololu 2133](https://www.pololu.com/product/2133), regulated 12 V supply and an Arduino-compatible board with the [water check sketch](water_check/water_check.ino). |

Print the housing, cam, lever and mount with at least four perimeters and 45% infill. Deburr tube-contact surfaces. Start with 0.2 mm layer height. Ream shaft and bearing fits as needed for the measured printer and parts. PETG prints and common motor, hardware and tubing can be sourced separately.

## Assembly and adjustment

1. Bolt the base to a rigid worktop and set each retainer centred over its two mounting holes at Y = ±46. Heads of the retainer screws sit flush. Install the 608 bearings from the sides against their inner shoulders. The main shaft centre is at Y = 0, Z = 49.
2. Assemble the two rotor webs on the shaft at X = ±26.2. File a 0.8 mm deep matching flat on the 8 mm shaft at the webs so the D bores transmit drive. Put one Ø18 sleeve on each axle, with their centres on the four Ø38 pitch circle holes. Fit flat washers at the ends and secure the axles. Fit split guide rings to the middle of each roller and between X = ±16.15 and ±19.85, preserving the two gaps centred at X = ±9. Use positive mechanical retention or a light press fit verified against hand turning. The rings turn with their individual rollers. Check end play and free rotation with the tube path empty.
3. Insert the hinge shaft along +Y at X = 48, Z = 99. Pin the cam body and lever to the shaft flats with M3 screws. Set the lever pivot at Y = −110 with the grip offset −20 mm in X and Z from the pivot when closed. Fix the positively keyed stop flag on the +Y end. Bolt the stationary stop plate to the outer face of the positive-side pivot ear on 11.5 mm spacers. The flag's 3 mm follower runs through the slot. Shim the follower or add tight end inserts until the closed and open stops measure 0° and 70° within 1°. Leave clear axial running clearance on both sides of the slot.
4. Seat one independently sprung shoe at each track. Fit one shoulder screw and one compression spring at each of its two posts. At nominal closed position the shoes' inner arcs have radius 31.2 mm about the rotor, with 3.2 mm minimum roller-to-pad gap. Set screw grip length and shims so each spring has about 5 mm installed length and the shoe can move at least 1 mm upward under load. The two-screw suspension keeps the arc aligned with the tube. Insert the tubes and turn the coupled shaft by hand through a whole revolution with power off. The unloaded gap stays near twice the tube wall thickness through the occlusion arc. The drive must turn without a hard catch.
5. Hook a spring to each moving ear at X = 17, Z = 112 in the closed pose and to its corresponding fixed ear at X = 73, Z = 76. Use shoulder eye studs with matching stand-offs to keep each straight spring in its side plane at Y = ±45. The line of pull crosses the hinge axis during the motion. Confirm both end positions are retained and that the lever closes on both water-filled tubes.
6. Mount the motor as shown, with the bracket centre offset 80 mm in +X from the pump. Align its shaft with the rotor. Set the motor bracket and coupler after a dry rotation check. Guard the roller, cam, shaft and coupling for every powered check. A clear removable shroud can be made from fixed side panels and a hinged top panel with narrow slots at the retainers for the tubing. Bolt the panels to the worktop. Fit a guard-closed switch and an accessible motor supply disconnect.

## Drive check

Use the driver's full-step mode and a common controller/driver ground. Set the current limit according to the specific motor and driver. Connect STEP, DIR and enable to the pins in the sketch. Connect the normally open guard switch and momentary start button as shown in the sketch comments. Keep RESET and SLEEP enabled per the [driver wiring reference](https://www.pololu.com/product/2133). The driver receives 240 step pulses at 80 Hz, equivalent to 24 rpm for 3.0 seconds. With the guard shut, verify direction toward the receivers and confirm 1.20 revolutions under load via a rotor index mark and timestamped video. Flip the direction constant if needed. Stop and correct the fixture if the rotor binds, stalls or either tube creeps.

Before the benchmark, use the same tubing, priming and water check as a trial. With both channels correctly loaded and the guard closed, verify each channel can collect at least 1.0 g of water over the timed check at the stated speed. During commissioning adjust only the shoe shim stack and spring preload if required for full occlusion. Record the settings and reuse them for the scored trials.
