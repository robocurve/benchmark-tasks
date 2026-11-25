# The machine's front rail, slot blade and the feeder's interface

All dimensions in mm. Positions of the machine's parts (frame rails, legs, bed, bottom camera, control box, gantry)
are taken from the maker's open-source LumenPnP assembly CAD and listed in task.py (`MACHINE`, machine frame: x along
the front rail from the machine's left outer face, y inward, z up from the frame's bottom; the feet stand 100 lower on
the bench). Side view: `bank-side.svg` (looking along the rail; the reel side is to the left of the drawing).

## Front rail and slot blade (machine CAD)

| Part | Size | Position |
|---|---|---|
| Front rail | 20 x 20 x 600 V-slot extrusion | the machine's front frame member; centre line 110 above the bench |
| Top slot (feeder hook) | 5 wide, 4 deep | along the rail's top face (the model uses a plain channel; the selected real V-slot profile and its sloped lips must be measured during enrollment) |
| Slot blade | 28 deep (out from the rail's front face) x 45 tall (35 below to 10 above the rail centre line) | from x_m 103 to 498: 25 slots at a 15.8 pitch, numbered on the face |
| Peg | 5 (along the rail) x 5.5 (out from the blade) x 6 (tall) | one per slot, centred, 10 to 16 below the rail centre line |
| Bed plate | 3 thick | from 85 behind the rail's front face (75 in from the rail centre line, 5 clear of the feeders), top 33 above the rail centre line |

## Feeder (as modelled; envelope from the maker's CAD, interface estimated from photos)

| Feature | Size | Position (from the rail centre line, at the slot centre; u inward) |
|---|---|---|
| Body | 15 wide; 165 long; up to 40 above the rail centre line inside, 30 outside | u -95 to +70; rests on the rail and blade tops; its outer part hangs in front of the blade down to 33 below |
| Hook nose | round, 4 diameter, 14 long along the rail | in the rail's top slot, centre 8.3 above the rail centre line |
| Clip prongs (2) | 4.9 x 6.5 x 40, either side of the peg, 0.1 clear of it | in front of the blade's face, u -46 to -39.5 |
| Bearing pads (2) | on the prongs, 4 tall | touching the blade's face, 22 to 26 below the rail centre line |
| Release lever | yellow, two round teeth (5 diameter) under the blade's bottom edge, thumb tab 3 x 13 outboard | pivot at u -44, 29.3 below; intended spring preload 0.0954 N m (8 N phone catch pull mapped conservatively through 11.92 mm maximum tab radius; arm assumed) |
| Spool arm and reel | reel 178 diameter x 13, turning on its hub | hub at u -186, 15 below the rail centre line |

## Mat

Blue anti-static bench mat, 300 x 420 x 3, 0.5 kg, cut from a stock roll; lot A centre (350, 425), lot B (320, 425)
in the robot frame given in task.json.
