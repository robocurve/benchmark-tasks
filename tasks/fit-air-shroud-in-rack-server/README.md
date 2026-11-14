# Fit the air shroud in a rack server: tuck both tabs under the fan cage, then lower it to seat

Refit the air shroud: tilt it, put both front tabs into the slots under the fan cage top, then lower it until it is seated.

The air shroud ducts a server's fan air over its processors and memory, and it comes out for almost every job inside: Dell's PowerEdge R760 manual lists "Remove the air shroud" before memory, processor, heat-sink, riser and expansion-card work, and warns: "Never operate your system with the air shroud removed. The system may get overheated quickly". Refitting it is the manual's install step: "Tilt the air shroud at an angle and align the air shroud tabs with the system slots. Ensure that the air shroud tabs are below the surface of the cooling cage fan assembly. Lower the air shroud into the system until it is firmly seated." Lowered flat, both tabs land on top of the fan cage and the shroud sits proud. The robot carries the 426 mm duct with both hands, gets both tabs in at once and lowers it between the heat sinks and memory rows without racking it, as technicians do in the data centres that run AI workloads.

Source: Dell PowerEdge R760 Installation and Service Manual, "Installing the air shroud" (Figure 44),
https://www.dell.com/support/manuals/en-us/poweredge-r760/per760_ism_pub/install-the-air-shroud ; chassis dimensions,
https://www.dell.com/support/manuals/en-us/poweredge-r760/per760_ism_pub/chassis-dimensions .

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

Millimetres from the midpoint between the arm bases: x forward, y to the robot's left, heights above the bench top. A Dell PowerEdge R760 2U rack server (steel chassis 434 wide without its rack ears, 722.7 from its front panel to its rear wall, 86.8 high; unpowered and unplugged, system cover off) lies flat on the bench between the arms, front panel toward the robot at x -150, centred on y 0. Inside, from the front: the drive cage under its closed steel top (to x -15); the backplane's cable space; the cooling fan cage of six fans (x 69.5 to 144.5; its steel top 82 above the chassis floor, with two slots 22 wide and 6.5 high in its back face just under that top, centred at y +107 and -107); the shroud's bay over the system board: two finned aluminium processor heat sinks (61 wide, 100 long, 71 high, centred at x 228, y +-107) and four rows of eight DDR5 RDIMMs (133 long, 46 high; outer rows y +-148.5 to +-207, inner rows y +-7.5 to +-65.5); then the riser cages from x 345.5 to the rear wall. The server's own standard (non-GPU) air shroud, black moulded plastic, 426 wide, 165 front to back, 79 high, 0.30 kg, rests right way up on the drive cage's closed top, its middle at x -45, y 0, its tabbed front edge toward the robot: a top panel over the heat sinks and inner memory rows, a duct wall each side of each heat sink, an open trough (wing) over each outer memory row whose back wall is 2.5 thick, an end wall at each end, and two front tabs (16 wide, 7 long, 2 thick) on its front edge at y +107 and -107. Seated, its end walls stand on the chassis floor 3 inside the side walls, its front edge 1 behind the fan cage's back face, each tab 6 through its slot. Every collider is listed in README.md.

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
| chassis | 178 to 278 | 76.5 to 137.5 | 8 to 16 |
| chassis | 178 to 278 | 76.5 to 77.3 | 16 to 72 |
| chassis | 178 to 278 | 80.5 to 81.3 | 16 to 72 |
| chassis | 178 to 278 | 84.5 to 85.3 | 16 to 72 |
| chassis | 178 to 278 | 88.5 to 89.3 | 16 to 72 |
| chassis | 178 to 278 | 92.6 to 93.4 | 16 to 72 |
| chassis | 178 to 278 | 96.6 to 97.4 | 16 to 72 |
| chassis | 178 to 278 | 100.6 to 101.4 | 16 to 72 |
| chassis | 178 to 278 | 104.6 to 105.4 | 16 to 72 |
| chassis | 178 to 278 | 108.6 to 109.4 | 16 to 72 |
| chassis | 178 to 278 | 112.6 to 113.4 | 16 to 72 |
| chassis | 178 to 278 | 116.6 to 117.4 | 16 to 72 |
| chassis | 178 to 278 | 120.6 to 121.4 | 16 to 72 |
| chassis | 178 to 278 | 124.7 to 125.5 | 16 to 72 |
| chassis | 178 to 278 | 128.7 to 129.5 | 16 to 72 |
| chassis | 178 to 278 | 132.7 to 133.5 | 16 to 72 |
| chassis | 178 to 278 | 136.7 to 137.5 | 16 to 72 |
| chassis | 161.5 to 294.5 | 148.5 to 153.5 | 8 to 47 |
| chassis | 161.5 to 294.5 | 156.1 to 161.1 | 8 to 47 |
| chassis | 161.5 to 294.5 | 163.8 to 168.8 | 8 to 47 |
| chassis | 161.5 to 294.5 | 171.4 to 176.4 | 8 to 47 |
| chassis | 161.5 to 294.5 | 179.1 to 184.1 | 8 to 47 |
| chassis | 161.5 to 294.5 | 186.7 to 191.7 | 8 to 47 |
| chassis | 161.5 to 294.5 | 194.4 to 199.4 | 8 to 47 |
| chassis | 161.5 to 294.5 | 202 to 207 | 8 to 47 |
| chassis | 161.5 to 294.5 | 7.5 to 12.5 | 8 to 47 |
| chassis | 161.5 to 294.5 | 15.1 to 20.1 | 8 to 47 |
| chassis | 161.5 to 294.5 | 22.6 to 27.6 | 8 to 47 |
| chassis | 161.5 to 294.5 | 30.2 to 35.2 | 8 to 47 |
| chassis | 161.5 to 294.5 | 37.8 to 42.8 | 8 to 47 |
| chassis | 161.5 to 294.5 | 45.4 to 50.4 | 8 to 47 |
| chassis | 161.5 to 294.5 | 52.9 to 57.9 | 8 to 47 |
| chassis | 161.5 to 294.5 | 60.5 to 65.5 | 8 to 47 |
| chassis | 178 to 278 | -137.5 to -76.5 | 8 to 16 |
| chassis | 178 to 278 | -137.5 to -136.7 | 16 to 72 |
| chassis | 178 to 278 | -133.5 to -132.7 | 16 to 72 |
| chassis | 178 to 278 | -129.5 to -128.7 | 16 to 72 |
| chassis | 178 to 278 | -125.5 to -124.7 | 16 to 72 |
| chassis | 178 to 278 | -121.4 to -120.6 | 16 to 72 |
| chassis | 178 to 278 | -117.4 to -116.6 | 16 to 72 |
| chassis | 178 to 278 | -113.4 to -112.6 | 16 to 72 |
| chassis | 178 to 278 | -109.4 to -108.6 | 16 to 72 |
| chassis | 178 to 278 | -105.4 to -104.6 | 16 to 72 |
| chassis | 178 to 278 | -101.4 to -100.6 | 16 to 72 |
| chassis | 178 to 278 | -97.4 to -96.6 | 16 to 72 |
| chassis | 178 to 278 | -93.4 to -92.6 | 16 to 72 |
| chassis | 178 to 278 | -89.3 to -88.5 | 16 to 72 |
| chassis | 178 to 278 | -85.3 to -84.5 | 16 to 72 |
| chassis | 178 to 278 | -81.3 to -80.5 | 16 to 72 |
| chassis | 178 to 278 | -77.3 to -76.5 | 16 to 72 |
| chassis | 161.5 to 294.5 | -207 to -202 | 8 to 47 |
| chassis | 161.5 to 294.5 | -199.4 to -194.4 | 8 to 47 |
| chassis | 161.5 to 294.5 | -191.7 to -186.7 | 8 to 47 |
| chassis | 161.5 to 294.5 | -184.1 to -179.1 | 8 to 47 |
| chassis | 161.5 to 294.5 | -176.4 to -171.4 | 8 to 47 |
| chassis | 161.5 to 294.5 | -168.8 to -163.8 | 8 to 47 |
| chassis | 161.5 to 294.5 | -161.1 to -156.1 | 8 to 47 |
| chassis | 161.5 to 294.5 | -153.5 to -148.5 | 8 to 47 |
| chassis | 161.5 to 294.5 | -65.5 to -60.5 | 8 to 47 |
| chassis | 161.5 to 294.5 | -57.9 to -52.9 | 8 to 47 |
| chassis | 161.5 to 294.5 | -50.4 to -45.4 | 8 to 47 |
| chassis | 161.5 to 294.5 | -42.8 to -37.8 | 8 to 47 |
| chassis | 161.5 to 294.5 | -35.2 to -30.2 | 8 to 47 |
| chassis | 161.5 to 294.5 | -27.6 to -22.6 | 8 to 47 |
| chassis | 161.5 to 294.5 | -20.1 to -15.1 | 8 to 47 |
| chassis | 161.5 to 294.5 | -12.5 to -7.5 | 8 to 47 |
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

| Time (s) | Step |
| --- | --- |
| 0.0 to 2.5 | hands over the wings' back walls |
| 2.5 to 4.0 | pads down beside the walls |
| 4.0 to 5.2 | pinch both walls |
| 5.2 to 5.6 | settle the pinch |
| 5.6 to 7.1 | lift off the drive cage |
| 7.1 to 7.7 | hold lifted |
| 7.7 to 11.7 | carry back over the bay, tipping the front down |
| 11.7 to 12.5 | hold over the bay |
| 12.5 to 15.5 | lower tilted into the bay |
| 15.5 to 16.1 | hold, tabs 25 mm above the slots |
| 16.1 to 18.1 | lower tilted, tabs to slot height |
| 18.1 to 18.7 | hold short of the cage |
| 18.7 to 20.7 | draw forward: tabs into the slots |
| 20.7 to 21.9 | swing the back down about the tabs (1) |
| 21.9 to 23.1 | swing the back down about the tabs (2) |
| 23.1 to 24.3 | swing the back down about the tabs (3) |
| 24.3 to 24.9 | hold seated |
| 24.9 to 25.9 | let go |
| 25.9 to 27.4 | rise clear |
| 27.4 to 29.4 | both hands to their parking points |

The tilt is 8 degrees front-down; the shroud is lowered 12 mm back from its seated place
(tab tips 6 mm short of the cage), then drawn
12 mm forward so both tabs go 6 mm through their slots, then its
back is swung down about the tabs in three steps. It is never pressed: the hands open with its end walls 0.5 mm above
the floor and it settles under its own weight.

## Simulation and the real part

Rigid bodies, 240 Hz physics, 20 Hz control. The chassis and all its parts are one fixed body of boxes at true size; the shroud is one rigid body of plates (no flex). Its height, walls, tabs, slots and 0.30 kg, and the heat sinks' and modules' sizes, are estimates from Dell's drawings, to be measured on a real R760. In the simulation it may also be lowered level 12 mm back and slid forward; the real shroud may not allow that. The demo reads the shroud's pose to correct its hands.

- Sizes from Dell: the chassis' outer size (434 x 722.7 x 86.8 mm, front panel to rear wall). Sizes taken from Dell's
  drawings of the R760's inside (Figures 15 and 44 of the manual, scaled by the chassis' width): the shroud's 426 x 165
  outline and where it, the fan cage and the riser cages sit. Estimates, to measure on the real R760 and its standard
  shroud when the trial is set up: the shroud's height (79), wall thickness (2), mass (0.3 kg), the tabs' size
  and position (16 x 7 x 2 at y +-107) and the slots' (22 x 6.5), the heat sinks' and memory modules' sizes, and the
  friction of the plastic on steel (0.45).
- Static parts: the chassis and everything in it is one fixed body; nothing in it moves. The real fan cage, heat
  sinks and memory are left in place, as in service.
- The robot's own limits: two YAM arms, parallel grippers; the shroud is gripped by its own walls (no fixture, no
  tool).

## Reset

Lift the shroud out by its edges and set it back right way up on the drive cage's closed top, its middle at x -45, y 0, tabs toward the robot. Nothing is consumed or fastened.
