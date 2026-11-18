# Fit the air shroud in a rack server: tuck both tabs under the fan cage, then lower it to seat

Refit the air shroud: tilt it, put both front tabs into the slots under the fan cage top, then lower it until it is seated.

The air shroud ducts a server's fan air over its processors and memory, and it comes out for almost every job inside: Dell's PowerEdge R760 manual lists "Remove the air shroud" before memory, processor, heat-sink, riser and expansion-card work, and warns: "Never operate your system with the air shroud removed. The system may get overheated quickly". Refitting it is the manual's install step: "Tilt the air shroud at an angle and align the air shroud tabs with the system slots. Ensure that the air shroud tabs are below the surface of the cooling cage fan assembly. Lower the air shroud into the system until it is firmly seated." Lowered flat, both tabs land on top of the fan cage and the shroud sits proud. The robot carries the 426 mm duct with both hands, gets both tabs in at once and lowers it between the heat sinks and memory rows without racking it, as technicians do in the data centres that run AI workloads.

Source: Dell PowerEdge R760 Installation and Service Manual, "Installing the air shroud" (Figure 44),
https://www.dell.com/support/manuals/en-us/poweredge-r760/per760_ism_pub/install-the-air-shroud ; chassis dimensions,
https://www.dell.com/support/manuals/en-us/poweredge-r760/per760_ism_pub/chassis-dimensions .

## Revision: review feedback and what changed

The latest review found the clearances "are still estimates from drawings, and the fit check only verifies the proposed
fixture", and asked: "Measure the relevant clearances against a real R760 or a reliable dimensioned reference, then
update the fixture and simulation as needed to preserve them." We had no R760, and the dimensioned Dell references we
found (R760 Technical Guide, Table 23; manual, "Chassis dimensions") give only the outer size; the inside figures carry
no dimensions. So the values stay estimates. New: where each comes from and its uncertainty (Estimated values), which
gap values keep the same moves and failures (Sensitivity), and a check for a lab with an R760. The collider table gives
fins and modules by count and spacing (same boxes). Scene, scorer, tests, demo and media are unchanged.

## Run and score

From this task folder, with the official YAM Arena environment installed:

```sh
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=fit_air_shroud_in_rack_server.py -E object_jitter=0 -E control_hz=20 \
  --instruction "Refit the air shroud: tilt it, put both front tabs into the slots under the fan cage top, then lower it until it is seated." \
  --scorer task_score --max-steps 2400 --epochs 1 --store-frames --log-dir ./demo-run
```

For a model trial, replace `--policy yam_scripted` with `--policy dream`. `--scorer task_score` reports the score at
the end of the run. Each trial allows 2,400 control steps (120 seconds at 20 Hz). There are no
seeds: the layout is fixed (in our runs the scripted demo also scored 1.0 in three episodes with
`-E object_jitter=0.01`, the shroud's start moved up to 10 mm). The scripted demonstration takes 32.4 s,
the last 3 s holding still. `success()` requires a score of 1.0.

`pytest` in this folder runs the generated offline checks (no GPU): the scorer on the start, the goal and the named
partial and shortcut layouts.

## The scene

Millimetres from the midpoint between the arm bases: x forward, y to the robot's left, heights above the bench top. A Dell PowerEdge R760 2U rack server (steel chassis 434 wide without its rack ears, 722.7 from its front panel to its rear wall, 86.8 high; unpowered and unplugged, system cover off) lies flat on the bench between the arms, front panel toward the robot at x -150, centred on y 0. Inside, from the front: the drive cage under its closed steel top (to x -15); the backplane's cable space; the cooling fan cage of six fans (x 69.5 to 144.5; its steel top 82 above the chassis floor, with two slots 22 wide and 6.5 high in its back face just under that top, centred at y +107 and -107); the shroud's bay over the system board: two finned aluminium processor heat sinks (61 wide, 100 long, 71 high, centred at x 228, y +-107) and four rows of eight DDR5 RDIMMs (133 long, 46 high; outer rows y +-148.5 to +-207, inner rows y +-7.5 to +-65.5); then the riser cages from x 345.5 to the rear wall. The server's own standard (non-GPU) air shroud, black moulded plastic, 426 wide, 165 front to back, 79 high, 0.30 kg, rests right way up on the drive cage's closed top, its middle at x -45, y 0, its tabbed front edge toward the robot: a top panel over the heat sinks and inner memory rows, a duct wall each side of each heat sink, an open trough (wing) over each outer memory row whose back wall is 2.5 thick, an end wall at each end, and two front tabs (16 wide, 7 long, 2 thick) on its front edge at y +107 and -107. Seated, its end walls stand on the chassis floor 3 inside the side walls, its front edge 1 behind the fan cage's back face, each tab 6 through its slot. Every collider is listed below; the real trial (Dell's own shroud and fan cage, stand-ins for the rest, about $210) is under Real-trial setup.

### Collider table

Every collider is a box. The chassis is one fixed body; the shroud is one rigid body (0.3 kg, its centre
of mass and inertia from its colliders at uniform density), listed here where it sits when seated.

| Body | x | y | z |
| --- | --- | --- | --- |
| chassis | -150 to 572.7 | -217 to 217 | 0 to 1 |
| chassis | -150 to 572.7 | 216 to 217 | 0 to 85.8 |
| chassis | -150 to 572.7 | -217 to -216 | 0 to 85.8 |
| chassis | 571.7 to 572.7 | -216 to 216 | 0 to 85.8 |
| chassis | -130 to -128 | 217 to 241 | 0 to 86.8 |
| chassis | -130 to -128 | -241 to -217 | 0 to 86.8 |
| chassis | -150 to -15 | -216 to 216 | 1 to 86.8 |
| chassis | 69.5 to 144.5 | -216 to 216 | 81.5 to 83 |
| chassis | 143 to 144.5 | -216 to 216 | 1 to 75 |
| chassis | 143 to 144.5 | -216 to -118 | 75 to 81.5 |
| chassis | 143 to 144.5 | -96 to 96 | 75 to 81.5 |
| chassis | 143 to 144.5 | 118 to 216 | 75 to 81.5 |
| chassis | 144.5 to 345.5 | -209 to 209 | 1 to 8 |
| chassis | 69.5 to 132.5 | -215 to -145 | 1 to 81.5 |
| chassis | 69.5 to 132.5 | -143 to -73 | 1 to 81.5 |
| chassis | 69.5 to 132.5 | -71 to -1 | 1 to 81.5 |
| chassis | 69.5 to 132.5 | 1 to 71 | 1 to 81.5 |
| chassis | 69.5 to 132.5 | 73 to 143 | 1 to 81.5 |
| chassis | 69.5 to 132.5 | 145 to 215 | 1 to 81.5 |
| chassis (2 heat-sink bases) | 178 to 278 | 76.5 to 137.5 and -137.5 to -76.5 | 8 to 16 |
| chassis (16 fins per heat sink, 0.8 thick, evenly spaced) | 178 to 278 | first 76.5 to 77.3, last 136.7 to 137.5; mirrored at -y | 16 to 72 |
| chassis (8 modules per row, 5 thick, evenly spaced) | 161.5 to 294.5 | outer rows: first 148.5 to 153.5, last 202 to 207; mirrored at -y | 8 to 47 |
| chassis (8 modules per row, 5 thick, evenly spaced) | 161.5 to 294.5 | inner rows: first 7.5 to 12.5, last 60.5 to 65.5; mirrored at -y | 8 to 47 |
| chassis | 345.5 to 571.7 | -215 to 215 | 1 to 81 |
| shroud (seated) | 145.5 to 310.5 | -144.5 to 144.5 | 78 to 80 |
| shroud (seated) | 145.5 to 147.5 | -144.5 to 144.5 | 70 to 78 |
| shroud (seated) | 308.5 to 310.5 | -144.5 to 144.5 | 70 to 78 |
| shroud (seated) | 145.5 to 310.5 | 69.5 to 71.5 | 11 to 78 |
| shroud (seated) | 145.5 to 310.5 | 142.5 to 144.5 | 11 to 80 |
| shroud (seated) | 145.5 to 310.5 | 211 to 213 | 1 to 80 |
| shroud (seated) | 145.5 to 308 | 144.5 to 211 | 51 to 53 |
| shroud (seated) | 308 to 310.5 | 144.5 to 211 | 53 to 80 |
| shroud (seated) | 145.5 to 147.5 | 144.5 to 211 | 53 to 80 |
| shroud (seated) | 138.5 to 145.5 | 99 to 115 | 78 to 80 |
| shroud (seated) | 145.5 to 310.5 | -71.5 to -69.5 | 11 to 78 |
| shroud (seated) | 145.5 to 310.5 | -144.5 to -142.5 | 11 to 80 |
| shroud (seated) | 145.5 to 310.5 | -213 to -211 | 1 to 80 |
| shroud (seated) | 145.5 to 308 | -211 to -144.5 | 51 to 53 |
| shroud (seated) | 308 to 310.5 | -211 to -144.5 | 53 to 80 |
| shroud (seated) | 145.5 to 147.5 | -211 to -144.5 | 53 to 80 |
| shroud (seated) | 138.5 to 145.5 | -115 to -99 | 78 to 80 |

The shroud starts 273 mm nearer the robot and 85.8 mm higher
(its end walls on the drive cage's closed top), turned the same way.

## Scoring

Judged at the end of the run from photos and a 2 mm feeler gauge. A tab is in when its tip is at least 3 mm past the fan cage's back face, inside its slot, under the cage's top. Seated: all four bottom corners of the shroud's end walls within 2 mm of the chassis floor (the 2 mm gauge does not pass under any). Still: no point of the shroud moves more than 1 mm in 0.5 s. 1.0 (success): both tabs in, seated, both grippers at least 3 cm clear of the shroud, and still, all held for 1 s. 0.5: both tabs in and seated, but not yet let go and still for 1 s. 0.25: both tabs in, let go and still for 1 s, but not seated (its back resting high). 0: anything else, including only one tab in, the tabs on top of the fan cage (lowered flat), or the shroud dropped (falling faster than 0.6 m/s) or knocked off the chassis at any time.

How each condition is read in `score()`:

- **Tab in its slot:** the tab's tip is at least 3 mm past the fan cage's back face (x), within
  11 mm of the slot's centre line (y), and the tab's top is no higher than the underside of the cage's top
  (0.5 mm contact tolerance). The slot is the only opening in the cage's back face, so a tab that far in
  is in its slot.
- **Seated:** the bottom corners of both end walls (four points) are within 2 mm of the chassis floor.
- **Let go:** both grippers' centre points at least 30 mm outside the shroud's outline.
- **Still:** none of three points on the shroud (both front top corners, the middle of its back edge) moves more than 1 mm over 0.5 s, held for
  1.0 s.
- **Zero for the rest of the run:** the shroud falling faster than 0.6 m/s (dropped), or leaving the chassis.

Why the shortcuts fail: lowered level straight onto its place (the mistake the manual's note warns about), the
shroud's front comes down on top of the fan cage, the tabs above its top surface instead of under it. In our GPU check
of exactly that (the same demonstration, lowered level with no tilt or draw) the front rested on the cage top while the
back came down to the floor, and the shroud was left leaning 32 degrees, both tabs 78 mm above the slots: score 0.
Placed flat at its seated height with the tabs on the cage top (an offline layout), its front stands 5 mm proud: 0. With
one tab in and the shroud turned a little, the other tab's tip stops short of the cage: 0. Dropped in from above: 0.

## Scripted demonstration

Both hands work together throughout, one pinch on each wing's back wall (the edge away from the robot), fingers closing
across the 2.5 mm wall, 355.5 mm apart. The demonstration reads where the shroud is five
times: at the start (the hands go to where it actually rests, at most 25 mm and 8
degrees from the planned start), after the lift, over the bay, with the tabs 25 mm above the slots, and just before the
draw. After each read after the grasp it moves both hands so the shroud is where the plan has it (a rigid correction
about the tabs' line, at most 40 mm and 10 degrees a read), taking out the
arms' sag under the load and the shroud's droop in the pinches (about 3 degrees, front down).

Steps (s): hands over the wings' back walls 0 to 2.5; pads down beside the walls to 4.0; pinch both walls to 5.2;
settle to 5.6; lift off the drive cage to 7.1; hold to 7.7; carry back over the bay, tipping the front down, to 11.7;
hold to 12.5; lower tilted into the bay to 15.5; hold, tabs 25 mm above the slots, to 16.1; lower tilted, tabs to slot
height, to 18.1; hold short of the cage to 18.7; draw forward, tabs into the slots, to 20.7; swing the back down about
the tabs in three steps to 24.3; hold seated to 24.9; let go to 25.9; rise clear to 27.4; hands to their parking
points to 29.4.

The tilt is 8 degrees front-down; the shroud is lowered 12 mm back from its seated place
(tab tips 6 mm short of the cage), then drawn
12 mm forward so both tabs go 6 mm through their slots, then its
back is swung down about the tabs in three steps. It is never pressed: the hands open with its end walls 0.5 mm above
the floor and it settles under its own weight.

## Simulation and the real part

Rigid bodies, 240 Hz physics, 20 Hz control. The chassis and all its parts are one fixed body of boxes; the shroud is one rigid body of plates (no flex). Only the chassis' outer size is Dell's published figure; the shroud's, tabs', slots' and internals' sizes and its 0.30 kg are estimates (Estimated values, below). In the simulation it may also be lowered level 12 mm back and slid forward; the real shroud may not allow that. The demo reads the shroud's pose to correct its hands.

- Sizes from Dell: the chassis' outer size, 434 wide, 86.8 high, 22.0 + 700.7 = 722.7 from front panel to rear wall
  (Technical Guide, Table 23,
  https://www.delltechnologies.com/asset/en-us/products/servers/technical-support/poweredge-r760-technical-guide.pdf ).
  Plastic-on-steel friction (0.45) is an estimate.
- Static parts: the chassis and everything in it is one fixed body; nothing in it moves. In the real trial the fan
  cage is Dell's own and the other internals are fixed stand-ins of the same outer size (Real-trial setup).
- The robot's own limits: two YAM arms, parallel grippers; the shroud is gripped by its own walls (no fixture, no
  tool).

## Estimated values

None of these values has been checked against a physical R760. Millimetres. "Our estimate": chosen by us within
Dell's outer size and its undimensioned figures (Figure 15,
https://www.dell.com/support/manuals/en-us/poweredge-r760/per760_ism_pub/inside-the-system , and Figure 44).

| # | Value | From | Uncertainty (our judgement) |
| --- | --- | --- | --- |
| 1 | shroud 426 wide, 165 deep, 79 high | width, depth scaled from Figure 15 by the 434 width (420 to 430, 160 to 175); height our estimate | ±5, ±10, ±5 |
| 2 | walls and roof 2 | our estimate | ±1 |
| 3 | gripped back wall 2.5 | our estimate | ±1 |
| 4 | tab 16 wide, 7 long, 2 thick | our estimate from Figure 44 | ±4, ±3, ±1 |
| 5 | tab spacing 214 | our estimate (on the heat sinks' lines) | ±15 |
| 6 | mass 0.30 kg | calculated from the modelled volume: 0.28 (polypropylene) to 0.36 kg (PC/ABS) | ±0.08 kg |
| 7 | chassis inside width 432 | Dell's 434 (Table 23) less two 1 mm steel sides (our estimate) | ±1 |
| 8 | slot 22 wide, 6.5 high | our estimate | ±4, ±2 |
| 9 | slot edges 74 and 80.5 above the floor | our estimate, under Dell's 86.8 height | ±3 |
| 10 | cage wall at the slot 1.5 | our estimate | ±0.5 |
| 11 | shroud to side wall 3 each side | calculated from 1 and 7 | 0.5 to 6.2 |
| 12 | tab to slot: sides 3, above 1.5, below 3 | calculated from 4, 8, 9 | ±2 each; above is over 0 (Dell: tabs below the cage's surface) |
| 13 | wing to memory 4 | our fixture choice; to a standard DDR5 RDIMM edge 7.8 to 9.8, calculated from Micron's 31.25 module height (288-Pin DDR5 RDIMM Core Product Description, https://www.mouser.com/datasheet/2/671/ddr5_rdimm_core-3310292.pdf) and an assumed 2 to 4 seat | ±3 |
| 14 | heat sink to duct wall 5 each side | calculated from our heat-sink width (61) | ±3 |
| 15 | roof to heat sink 6 | calculated from our heat-sink height (64 on the board); Dell lists heat-sink types, not heights | ±5 |
| 16 | duct bottom to board 3 | our estimate | ±2 |

## Sensitivity: which values keep the same task

Calculated offline with the task's own collider boxes as rigid bodies (no physics, no flex, no arm limits):

- The tuck. Seated, each tab tip is 6.0 past the cage's back face and its top 1.5 under the cage top. Lowered level at
  its place, both tabs land on the cage top and the front stands 5.0 proud (gap above 1.5 + tab 2 + cage top 1.5): score
  0. This holds for any gap above the tab over 0 and any tab reach past the face over 0, so for every shroud that seats;
  no value in the table removes the tuck or this failure.
- Tab height. Seated, each tab has 1.5 above and 3.0 below it in its slot; any gaps over 0, as in a real R760, give the
  same moves. The gap above sets how level the shroud must come down: the high tab meets the cage top at a roll of
  0.09° (gap 0.5), 0.26° (1.5, model), 0.53° (3).
- Side gaps. The sideways tolerance is the smallest of rows 11 and 12's side gaps (3, 3) and the duct walls' gaps (4 to
  memory, 5 to heat sink): 3. The yaw that fits between the side walls is 0.35° at a 0.5 gap, 2.19° at 3 (model), 4.84°
  at 6.2. Over 0.5 to 6.2 the moves and failures are the same: smaller is tighter; larger is not easier unless the slots
  are wider too (the tab side gap still sets 3). At 0 it would not fit.
- Memory, heat sinks, board. Along the demo's swing about the tab tips from 8° to level, the gaps under the shroud are
  smallest when seated (memory 4.0, heat sinks 6.0, board 3.0; at 8°: 6.9, 11.5, 4.6). These parts never guide or stop
  it, so any value over 0 gives the same task; at 0 or below the shroud would rest on them, not seat.
- Not covered: the room behind the shroud (35 to the riser cage, our estimate) is what lets the simulation also lower it
  level 12 back and slide it forward; we have no figure for the real room.

## Checking on a real R760

A lab with an R760 (standard air-cooled build, shroud seated) can read these with feeler gauges and calipers. The task
needs no change while each reading is in the band: within a factor of two of the model for gaps that set a tolerance,
over a margin for parts never touched. Outside a band, set that collider and stand-in to the reading.

| Read | Model | No change if |
| --- | --- | --- |
| tab tip past the cage's back face | 6 | 4 to 12 (scoring needs 3; fans 12 past the face) |
| gap above each tab (to the cage top's underside) | 1.5 | 0.75 to 3 |
| gap below each tab | 3 | 1.5 to 6 |
| tab to slot side, each side, shroud centred | 3 | 1.5 to 6 |
| shroud end wall to chassis side wall, each side | 3 | 1.5 to 6 |
| wing underside to memory top | 4 | 2 or more |
| roof underside to heat-sink top | 6 | 2 or more |
| duct wall to heat sink, each side | 5 | 3 or more |
| duct wall bottom to board | 3 | 1 or more |

## Real-trial setup (no server needed, about $210)

A whole R760 costs thousands (an open-box barebone, no processors or memory: $3,932.07,
https://www.ebay.com/itm/188558735310). The trial buys the shroud and the fan cage whose slots take its tabs, and builds
every other part as a stand-in of the same outer size and place, from the collider table.

Bought (prices seen in November 2026):

| Item | Gives | Price and source |
| --- | --- | --- |
| Dell T9P2J ("ASSY,SHRD,REGLR,R760"), the R760's standard air shroud | the real shroud, its walls and both front tabs | $89.70 new (bulk), https://oemcomputerpart.com/product/t9p2j-dell-r760-air-shroud/ ; $99 refurbished, https://www.serverworlds.com/dell-t9p2j-poweredge-r760-plastic-shroud/ ; Dell's description: https://www.pc21.fr/fiche/t9p2j-assy-shrd-reglr-r760-i5789113.html |
| Dell CNYJ3, the R760's cooling fan cage (16th-generation fan tray); the fans are not needed | the real cage top and the slots under it | $19.99 used and tested, with six fans, https://www.ebay.com/itm/376235927394 ; R760 fit ("Fan Cage: CNYJ3"): https://expresscomputersystems.com/products/dell-poweredge-r760-high-performance-gold-fan-2x4xn |
| 1/2 in (12.7 mm) MDF panel, 4 x 8 ft | walls, drive cage, riser block | $45, https://www.homedepot.com/p/202522732 |
| 1/8 in (2.9 mm) tempered hardboard, 4 x 8 ft | floor | $14.25, https://www.homedepot.com/p/202086860 |
| PLA filament, 1 kg | board plate, heat-sink and memory blocks | $22.99 list, https://overture3d.com/products/overture-pla |
| wood glue, screws, two steel L brackets, double-sided tape | assembly | about $20 (estimate) |

Built (mm, to within 0.5). Each stand-in is a "chassis" row of the collider table, or one solid block over a group of
rows, at the table's x and y. The floor sheet is 2.9 thick where the steel floor is 1, so heights below are above the
floor sheet's top (the table's z minus 1) and the whole rig stands 1.9 higher than in the simulation.

- Floor: hardboard 722.7 x 434, x -150 to 572.7, y -217 to 217, clamped or taped to the bench.
- Side walls and rear wall: MDF strips 84.8 high standing on the floor, inner faces at y +216, y -216 and x 571.7 (thicker
  than the steel, outward). Rack ears: hardboard plates 86.8 high at x -130 to -128 on the side walls' outsides, reaching
  y +-241.
- Drive cage: an open-bottom MDF box, x -150 to -15, between the side walls, top 85.8 above the floor. The shroud starts
  on its top.
- System board: a printed plate 7 thick, x 144.5 to 345.5, y -209 to 209 (printed in four tiles).
- Heat sinks: two printed blocks, x 178 to 278, y 76.5 to 137.5 and -137.5 to -76.5, 64 tall on the board plate (top 71
  above the floor). The shroud meets only their outer faces, so the fins are not reproduced.
- Memory: four printed blocks, one per row of eight modules (the shroud never goes between modules), x 161.5 to 294.5,
  y 148.5 to 207, 7.5 to 65.5, -65.5 to -7.5 and -207 to -148.5, 39 tall on the board plate (top 46 above the floor).
- Risers: an open-bottom MDF box, x 345.5 to 571.7, y -215 to 215, top 80 above the floor.
- Fan cage: the CNYJ3 stands on the floor between the side walls with its back face at x 144.5, held by the two L
  brackets screwed into the side walls. The simulation puts its top 82 above the floor; set the real cage's height (shim
  under it if needed) so that the real shroud, stood on the floor, takes both tabs into the slots level, as the two parts
  meet in the server.

Fit check, before the first trial (by hand, ruler and the 2 mm feeler gauge): tilt the shroud, put both tabs into the
slots and lower it. Both tab tips at least 3 mm past the cage's back face, the gauge passing under no end-wall corner,
and the shroud touching only the floor and the slots, with the modelled gaps (table above). If the real shroud meets a
block, move or trim the block until its gap is back. Scoring and reset are unchanged.

## Reset

Lift the shroud out by its edges and set it back right way up on the drive cage's closed top, its middle at x -45, y 0, tabs toward the robot. Nothing is consumed or fastened.
