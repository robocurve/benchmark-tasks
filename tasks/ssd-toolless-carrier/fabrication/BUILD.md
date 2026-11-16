# Building the real carrier fixture and SSD dummy

These files build the task's carrier and drive for real-world trials: a printed carrier frame, a printed sprung rail
on two steel guide rods with two compression springs, steel dowel pins, and a printed 2.5-inch SSD dummy. Every
dimension is in `params.scad` and comes from `task.py`. Each `task.py` value is tagged with the constant it came
from, and `check_params.py` checks them all.

| File | What |
|---|---|
| `params.scad` | All dimensions (mm): the task's numbers, then the fits and hardware chosen for the build |
| `common.scad` | Two helper modules (`box`, `ybore`) |
| `carrier_frame.scad` | Printed part: handle bezel, floor, ribs, fixed rail, rail stops, the two guide-rod mounts, ballast pockets |
| `guide_rod_mount.scad` | The guide-rod mount's shape (rod bore and spring pocket); printed as part of the frame, see below |
| `sprung_rail.scad` | Printed part: the sprung rail with its pull tab, pin bores and rod bosses |
| `ssd_dummy.scad` | Printed part: the 2.5-inch drive dummy with the task's four side holes |
| `assembly.scad` | Preview of everything in place (set `open` from 0 to 7 to see the rail open); not printed |
| `check_params.py`, `test_fabrication.py` | Check `params.scad` against `task.py`, plus consistency checks (travel stop, spring rate, rod length) |

Run the check from the task folder, with `yam_arena` importable as for `test_scoring.py`:

```bash
python fabrication/check_params.py
python -m pytest fabrication/test_fabrication.py
```

To make the STLs, open each part file in OpenSCAD (2021.01 or later), render it (F6) and export an STL. The parts
come out in millimetres and already in their print orientation.

## How the mechanism works

The sprung rail is a separate part. Two 3 mm steel guide rods are pressed into bosses on its outer face. Each rod
slides through a bore in a mount block that is part of the frame, 7 mm outboard of the boss. A light compression
spring goes over each rod, between the boss and a pocket in the mount, and pushes the rail inward. Two printed stops
(the cross ribs, continued 3 mm high to the rail's inner face) set the rail's closed position, so its inner face sits
0.5 mm clear of the drive's side. When the pull tab is drawn outward, the rail slides out until the bosses meet the
mounts, which is exactly the task's 7 mm travel. Together the two springs give the task's 0.15 N/mm. The rods can't come
out: the stops keep the rail from moving in, and the mounts keep it from moving more than 7 mm out. So no clips are
needed.

The mounts are printed with the frame rather than as loose blocks. That way their bores line up with the rail's
bosses without any adjustment. `guide_rod_mount.scad` holds their shape, and `carrier_frame.scad` places two of them.

The carrier is loose on the table in the task, so there is no base or clamp. The frame's underside is the base.

## Bill of materials

| Qty | Item | Spec |
|---|---|---|
| ~120 g | PLA filament | 1.75 mm. A dark colour for the frame. Blue for the rail, or paint the pull tab blue |
| 4 | Dowel pins | Steel, 2.5 mm diameter x 6 mm (ISO 8734 / hardened dowel pins), or 2.5 mm drill rod cut to 6 mm with chamfered ends |
| 1 | Ground steel rod | 3 mm diameter (silver steel / drill rod / 3 mm linear shaft, h6-h8), at least 60 mm, cut into two 26 mm rods |
| 2 (+spares) | Compression springs | Music wire 0.3 mm, OD 5.0 mm (ID 4.4 mm), 10 active coils (12 total, closed and ground ends), free length 13.5 mm, solid length about 3.6 mm, rate about 0.075 N/mm each (0.15 N/mm the pair). From an assortment: pick 0.3 mm x 5 mm springs and accept them on the force check below |
| 1 | Steel plate | Mild steel, 55 x 55 x 3 mm (about 71 g), for the slab pocket |
| 1 | Steel flat bar | Mild steel, 12 x 6 mm section, 70 mm long (about 40 g), for the bezel pocket |
| 1 | Steel plate | Mild steel, 24 x 60 x 2 mm (about 23 g), for the front floor pocket |
| as needed | Trim weight | Tungsten or lead putty (fishing-tackle type), to trim the carrier to 200 g |
| | Adhesives | Cyanoacrylate (thin) and 5-minute epoxy |
| | Finishing | White paper label (52 x 72 mm) and black paint or tape for the dummy; masking tape for the table outlines |

Tools: calipers, a pin vise with 2.5, 3.0 and 3.3 mm drills (the 3.3 mm drill is the M4 tap drill), a hacksaw or
cut-off wheel and file, a 0.1 g kitchen scale, a 0-5 N (or 0-500 g) spring force gauge or luggage scale with a loop
of thread, and a 0.5 mm feeler gauge.

## Printing

All parts: PLA, 0.4 mm nozzle, 0.2 mm layers. Check the printer's XY calibration first: the printed frame's rail
gap must come out within 0.2 mm (see the acceptance checks).

| Part | Orientation | Settings |
|---|---|---|
| `carrier_frame.scad` | As modelled: base web down | 4 perimeters (the bores and pockets sit in solid walls), 30 % infill. Supports from the build plate only, under the release handle (it overhangs 3 mm above the bed). The 0.7 mm base web is the first 3-4 layers, so make sure the bed is level |
| `sprung_rail.scad` | As modelled: upright, rail bottom down | 4 perimeters, 100 % infill (it is small). Supports from the build plate only, under the pull tab's bracket. The rod and pin bores are horizontal and need no support |
| `ssd_dummy.scad` | As modelled: bottom face down | 3 perimeters, about 90 % rectilinear infill, no supports (the 4 mm holes bridge). Solid PLA would be about 60 g. Weigh the first print and adjust the infill to reach 55 g |

Fits used (in `params.scad`): sliding bores are 0.3 mm over diameter (3.3 mm for the 3 mm rod). Press-fit bores print
at nominal size (2.5 mm and 3.0 mm), because horizontal printed holes come out slightly small. Ballast pockets are
0.3 mm over the plate size. The rail clears the base web by 0.3 mm.

## Assembly

1. **Clean the bores.** Run the 3.3 mm drill by hand through both mount bores until a 3 mm rod slides freely with
   no wobble. Check that a dowel pin pushes into each 2.5 mm pin bore with firm thumb or vise pressure, and the rod
   into each 3.0 mm rail bore. If one won't go, open it slightly with the 2.5 or 3.0 mm drill by hand. Clear the
   spring pockets of stringing.
2. **Ballast.** Bed the 55 x 55 x 3 mm plate in epoxy in the slab pocket. Its top must not stand above the slab, which
   sits 0.5 mm below the drive. Bed the 12 x 6 x 70 mm bar in the bezel pocket from below, and the 24 x 60 x 2 mm plate
   in the front floor pocket from below, with their faces flush with the underside. Before the epoxy sets, weigh the
   frame and press trim putty into the bezel pocket until it reads 200 g. If it is over 200 g, leave out the floor
   plate. Once set, sand any epoxy proud of the underside so the base sits flat.
3. **Fixed pins.** From the fixed rail's outer face, press two dowel pins through the pin bores (a small vise with
   a flat block works) until each pin is flush with its pad outside. It then stands 2.5 mm out of the rail's inner
   face. Wick a drop of thin cyanoacrylate in from the outside.
4. **Sprung-rail pins.** Do the same on the sprung rail. Press each pin from the outer face until flush with its pad,
   so it stands 2.5 mm out of the inner face, and glue it.
5. **Guide rods.** Cut two 26 mm lengths of 3 mm rod. Square and deburr the ends and put a 0.3 mm chamfer on one end
   of each.
6. **Fit the rail.** Set the rail in its place on the frame, inner face against the stops, with the bosses facing the
   mounts. Drop a spring into each mount's pocket. Push each rod, chamfered end first, in from the mount's outer face:
   through the mount bore, through the spring, and into the boss's blind bore. Hold a block against the rail's inner
   face while you press, so the printed stops don't take the force. With the rail closed and the rod bottomed in its
   bore, the rod stands 1 mm out of the mount's outer face. Fix each rod with a drop of cyanoacrylate at the boss only.
   Keep glue away from the mount bore and the spring.
7. **Pull tab.** The tab is printed with the rail. Paint it blue if the rail wasn't printed in blue. Paint or tape
   the release handle (the small block on the bezel's end face) blue too.
8. **SSD dummy.** The connector end is the end whose holes are 9.4 mm in; the front end's holes are 14.0 mm in.
   Stick the white label on top, centred across the width, starting 8 mm from the front end. Seen from above with
   the connector end away from you, blacken the right half of the connector end: its end face and the last 6 mm of
   the top (`CONN_X`, `CONN_Y`).

## Setting up on the table

The carrier is loose on the table, as in the task: don't fasten it. Positions are in cm from the midpoint between the
two arm bases (x forward, y left), as in the README.

- **Carrier:** centred at (30.0, 0.0), long side along y. The handle bezel goes to the left (+y): the release handle's
  tip is at y = +7.4 and the rear edge at y = -7.0. The fixed rail is nearer the arms (its outer face at x = 26.2).
  The sprung rail and the two guide-rod mounts face away from the arms. The rail's outer face is at x = 33.8 and the
  mounts reach x = 36.1.
- **SSD dummy:** centred at (30.0, -20.0), label up, long side along y, connector end toward the right (-y).
- Mark both outlines with masking tape for resets.

## Acceptance checks

Do these with calipers, a ruler, a feeler gauge, a kitchen scale and a force gauge. Target values are the task's,
with a tolerance a lab can hold.

| Check | How | Target |
|---|---|---|
| Rail gap | Calipers between the fixed and sprung rails' inner faces, rail closed | 70.85 mm +/- 0.2 (drive 69.85 + 2 x 0.5) |
| Pin protrusion | Calipers or depth gauge, pin tip to the rail's inner face, all four | 2.5 mm +/- 0.2 (2.0 mm past the drive's side) |
| Pin height | Ruler from the table to each pin's centre | 7.0 mm +/- 0.3 |
| Pin positions | Along the carrier, from the rear edge to each pin's centre | 9.4 mm and 86.0 mm +/- 0.3 |
| Seat height | Ruler from the table to the rib tops | 4.0 mm +/- 0.2 |
| Rail travel | Pull the tab fully open; measure the rail's outward movement | 7.0 mm +/- 0.3, ending on a hard stop |
| Spring force, open | Hook the gauge to the tab with a thread loop and pull outward to full travel (just before the stop) | 1.05 N (107 g) +/- 0.2 N |
| Spring force, nearly closed | Same, at 1 mm open | 0.3 N (30 g) or less |
| Return | Open fully and let go | The rail closes on its own, back against the stops (feeler: no gap at the stops) |
| Carrier drag | Gauge on the bezel, pull the carrier along the table | About 1.8 N (task: friction 0.8 on 0.23 kg). At least 1.5 N, so pulling the tab fully open does not move the carrier. If it is lower, stick thin anti-slip tape (0.5 mm or less) at the base's corners |
| Masses | Kitchen scale | Frame with ballast 200 g +/- 5; SSD dummy 55 g +/- 1 |
| Dummy holes | Calipers | 4.0 mm square, 3.0 mm deep, centres 3.0 mm above the bottom face, 14.0 mm from the front end and 9.4 mm from the connector end |
| Pin engagement | Seat the dummy, let the rail close. Feeler between the drive's side and the sprung rail | 0.5 mm feeler fits, 0.6 mm does not, so each pin is 2.0 mm into its hole. All four pins can be seen in their holes, and the dummy sits flat on the ribs |
| Wrong way round | Turn the dummy end for end, or upside down, and try to seat it | It doesn't seat flat: the pins miss the holes |

## Differences from the simulation

- The rail is carried on round rods instead of an ideal slider. The springs are installed with 0.5 mm preload (0.07 N
  closed instead of 0) so the rail returns firmly against the stops. At full travel the force is 1.12 N, against the
  task's 1.05 N.
- The printed sprung rail with its rods and pins weighs roughly 10-15 g (an estimate), against the task's 30 g. Nothing in the
  task depends on the rail's mass beyond its spring force.
- The pins are round (2.5 mm) instead of 2.5 mm square, which keeps the same 0.75 mm clearance in the dummy's 4 mm
  holes. The frame has a 0.7 mm base web, rail stops and a ballast slab below the drive's seat, none of which the
  drive or the fingers touch. The two guide-rod mounts stand outboard of the sprung rail, on the side away from the
  arms.
- **A real SSD** has threaded M3 side holes (about 2.5 mm across the thread crests). A 2.5 mm pin barely enters them,
  so a real drive is much harder to seat than the task's 4 mm holes allow. Use the dummy for scored trials.
- **Commercial tool-less 2.5-inch carriers** work the same way: pins on the side rails go into the drive's side
  holes, and one rail flexes outward to let the drive past. But their rails are stiffer (a few newtons), their pins
  are about 2.8 mm and their sizes vary by vendor. Use them only as a reference, not for scored trials.
