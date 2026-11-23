# Low-cost bench fixture for real trials

It stands in for the microtome (about 159 USD in parts, plus an FDM printer or a print service).
A plywood housing carries a printed handwheel with a push-in lock, a head that the crank moves through 59 mm, a sprung cassette clamp and a
knife holder with a dull mock blade and a folding guard. Every surface the robot touches stands where the simulation has it.

Millimetres from the midpoint between the two arm bases: x forward, y left, z up from the bench top. The numbers are those of
`task.py` (its block "every dimension, once"); the STL files, both drawings and the tables below were made and checked from it.

| file | what |
|---|---|
| `assembly.svg` | side view and plan with every height and station |
| `clamp_section.svg` | the clamp cut along its middle, at rest and pushed 15 degrees open, with the spring's length and force in both |
| `stl/` | 15 printable parts, binary STL in mm, laid flat as they should be printed (bed 220 x 220) |
| `cut-list.csv` | plywood panels with their holes, and the bars |
| `BOM.csv` | bought parts with part numbers, sellers, prices and how each price was read |

## Heights and stations

| what | mm |
|---|---|
| bench top | z 0 |
| base plate top | z 36 (12 mm top panel on two layers of 12 mm strips) |
| housing top | z 265 |
| wheel axis | y -178, z 140 |
| lock hole, 13 mm | y -178, z 225 |
| **cassette's centre: start (handle at 10 o'clock)** | **z 135.25** |
| **cassette's centre: upper end (handle at 12 o'clock)** | **z 150** |
| **cassette's centre: lower end** | **z 91** |
| knife edge | y 58, z 105 |
| cassette's back face, block's face | y 45, y 57.5 |
| guard pivot | y 104, z 72 |
| guard bar, over the edge / lying forward | y 60.5, z 112 / y 159.5, z 92.2 |
| clamp lever: pivot, spring, push point above the pivot | 0, 40, 56 |
| clamp's centre line | x 485 |
| handwheel-side wall's outer face | x 360 |
| guide rods | x 455 and 515, y -64, z 36 to 236 |

Heights are nominal: 0.5 mm of play in the crank's slot and 0.2 mm in the jaw put the cassette 0.7 mm lower (134.5 and 149.3 in the simulation).

## The clamp spring

Lee Spring LC 022D 10 S (catalogue page read 2026-11-22): stainless, 7.62 mm outside, 0.56 mm wire, free length 38.1 +-1.22 mm, rate
0.298 N/mm +-10 %, solid height 5.79 mm, works in a 7.95 mm hole or over a 6.10 mm rod. It lies in an 8.0 mm blind bore in the
clamp block, in front of the lever and 40 mm above its pivot; a 5.8 mm peg on the lever keeps it central. The bore's lower side is relieved (a
9 mm circle 3.4 mm below its axis at the block's rear face, narrowing to the bore 1 mm before its bottom) so that the spring can tilt with the lever. Run an 8 mm drill through the
printed bore.

| lever | spring length | spring force | left to solid | push at the lever (56 mm above the pivot, level) |
|---|---|---|---|---|
| at 0 degrees (jaws 28.8 mm apart) | 25.6 mm (compressed 12.5 mm) | 3.7 N | 19.8 mm | 2.7 N |
| 15 degrees open | 15.6 mm (compressed 22.5 mm) | 6.7 N | 9.8 mm | 4.3 N |
| at the stop, 15.5 degrees | 15.3 mm (compressed 22.8 mm) | 6.8 N | 9.5 mm | 4.3 N |

Opening 15 degrees therefore uses 10.0 mm of the 19.8 mm that is left after the preload. Over the spring's own tolerances
(length +-1.22 mm and rate +-10 % together) the push at 15 degrees is 3.6 to 4.9 N, inside the 3.5 to 5.5 N check, and with the bore's
depth 0.3 mm off 3.6 to 5.0 N. With a 28.4 mm cassette in, the lever stands 0.6 degrees past 0. 15 degrees is the lever's top edge 19 mm forward of 0 (rule). The simulated clamp has the same moment at rest (0.149 N m) and at 15 degrees (0.235 N m).

## Printed parts (PLA, 3 perimeters, 20 % fill, about 833 g)

| file | on the bed (mm) | g | use |
|---|---|---|---|
| wheel_disc | 200 x 200 x 12 | 178 | handwheel; the handle's pin slides in it |
| knob | 26 x 26 x 44 | 12 | glued on the handle's pin |
| crank_disc | 80 x 80 x 10 | 28 | crank pin 29.5 mm from the axis, in line with the handle |
| outer_boss, bearing_block | 28 x 28 x 26, 126 x 90 x 35 | 11, 72 | the shaft's bearings |
| carriage_plate, yoke | 84 x 22 x 75, 45 x 159 x 22 | 60, 65 | head on its rods; crank slot 11 mm |
| head, clamp_lever | 50 x 54 x 108.5, 30 x 81.5 x 51.5 | 71, 14 | clamp block with the upper jaw; lever with the lower jaw (both need support) |
| rod_foot, rod_bridge | 84 x 22 x 7.5, 84 x 32 x 16 | 10, 22 | hold the rods |
| knife_holder, guard_arm (2) | 140 x 92 x 62, 35 x 71 x 10 | 190, 5 each | holder with the guard's two stops; the guard's arms |
| tray, cassette | 148 x 108 x 22, 40.4 x 28.4 x 12.5 | 84, 7 | or any grey plastic tray of that size and a real cassette (print the cassette at about 85 % fill for 12 g) |

Plywood: one 12 mm sheet of 603 x 1212 mm (23.75 x 47.75 in) holds every panel. Sheet that is not exactly 12 mm must be planed or shimmed to a 36 mm base: every height
stands on it.

## Fits

| what | value | check |
|---|---|---|
| lock | 8 mm pin, 85 mm from the axis, in a 13 mm hole; out: tip 4 mm off the wall; 6 mm in: 2 mm in the hole; 10 mm: knob meets the disc | enters only within 1.7 degrees of 12 o'clock; elsewhere it stops on the wall after 4 mm |
| crank | 10 mm pin, 11 mm slot | one turn moves the head 59 mm |
| clamp | jaws 28.8 mm apart at 0 degrees, 27.9 at the closed stop | holds a 40.4 x 28.4 x 6.5 mm cassette while the wheel turns once; the upper jaw's ends are 41.2 mm apart (0.4 mm play a side), 2.2 mm deep: the cassette leaves downwards, then forwards |
| blade | 0.3 mm shim, filed dull, its face 0.5 mm in front of the block's face | feeler gauge |
| guard | lies 20 degrees above level; over the edge after 117 degrees | protractor; it falls to either stop (file the arms' hubs until it does; they run 0.2 mm beside the wedge). Over the edge its bar reaches 2 mm past the block face's plane: with the cassette's centre between 98 and 126 mm the block is in its way (the start is 135.25) |

## Operating forces

| what | simulation | fixture | set by |
|---|---|---|---|
| handwheel | 0.35 N m, 4.1 N at the 85 mm knob at 12 o'clock | 3.3 to 5.0 N at the knob, at 12 o'clock, both ways (at 10 o'clock the head's weight makes it 5.4 N to raise and 2.8 N to lower; 5.7 and 2.5 N in the simulation) | neoprene washer squeezed by the shaft collar |
| handle, push in | 4 N | 3 to 6 N | O-ring on its pin |
| clamp lever, 15 degrees open | 4.3 N | 3.5 to 5.5 N, 56 mm above the pivot | the bore's depth (25.6 +-0.3 mm) |
| guard | free | falls to either stop | free on its shoulder screws |

The simulation column gives what is set there, or computed from it: no breakaway force was measured in the simulator. Colours as in the scene: black knob, grey tray and red tape or paint on the guard's bar. Forces are checked once a session with a 10 N spring scale (a cord loop where the scale can only pull). The instrument's own forces are not
published; these are estimates, and the same checks apply to it.

## Assembly

1. Cut and drill the panels. Glue and screw the walls on the base panel, then the strips in two layers under it. Leave the top and the far wall off.
2. Glue the 8 x 60 dowel 16 mm into the knob, put the O-ring in the wheel disc's groove, push the pin through, set the retaining ring 22 mm from the
   knob with a drop of epoxy.
3. Cut the shaft to 148 mm and cross-drill it in one set-up so that the crank pin lines up with the handle. Bolt on the boss and the bearing block, fit the
   wheel disc, the shaft, the neoprene washer (trimmed to the collar), the collar and the crank disc.
4. Glue four LM8UU bearings into the carriage plate and screw on the yoke. Put the spring on the lever's peg, pass the lever up between the cheeks with
   the spring in its bore, fit the M4 x 60 pivot, and screw the head to the plate.
5. Screw the foot down; with the yoke through the front wall's opening drop the rods through the bearings into the foot and screw the bridge on top. At 12 o'clock push
   the crank's dowel through its disc into the slot and glue it, 29 mm standing out.
6. Glue the shim in the holder, edge up, fit the guard, screw the holder down with the blade 0.5 mm in front of the block.
7. Set the forces. Stand the base with the handwheel-side wall at x 360 and its front edge at y 195, clamped to the bench without feet; the tray's centre at x 235, y 165.

## Where it differs from the simulated shapes

The outer boss reaches the wall (1 mm gap in the simulation); the shaft opening is an 18 mm round hole and the lock hole is round; the front block is planed to the
lever's face at 15.5 degrees (the open stop); the closed stop lets the empty clamp go 1.25 degrees further so that it grips the cassette (the simulated jaw stops 0.2 mm
short of it); the blade is 0.3 mm thick (0.6 simulated); stops for the guard, screw heads and the inner bearing are added. Nothing here has been printed
yet: the sizes, clearances, spring and forces are computed. There is no cutting edge: the blade is shim stock with a filed edge.
