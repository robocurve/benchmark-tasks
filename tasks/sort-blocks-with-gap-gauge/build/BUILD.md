# Building the gauge, the blocks and the trays for a real trial

All files here are generated from `task.py` by `export_gauge.py` (`pip install numpy matplotlib`; from the task
folder: `PYTHONPATH=<path to yam-isaac-lab-arena> python build/export_gauge.py`), so they match the simulation. Units
are millimetres. `test_task.py` checks that the STL sizes and every number in `dimensions.json` (the numbers on the
drawing) equal the task's constants.

| File | What |
|---|---|
| `gauge.stl` | base plate and four anvils, plate bottom at z 0, centred on the plate |
| `block_a.stl` .. `block_f.stl` | the six blocks (60 long, 20 tall, widths below) |
| `dimensions.png` | top and front views with every dimension and anvil-face position |
| `dimensions.json` | the drawing's numbers |

## Gauge

- Base plate 50 x 170 x 10, steel or aluminium, flat to 0.1.
- Four anvils 10 (x) x 12 (y) x 15 tall, standing on the plate, centred on it in x. Their faces across the plate
  (y, from the robot's left): GO pair at +77.65/+65.65 and +24.35/+12.35, NO-GO pair at -13.65/-25.65 and
  -64.35/-76.35, so the gaps between inner faces are GO 41.30 (centred at +45) and NO-GO 38.70 (centred at -45): the
  upper and lower limits of the 40.0 +- 1.3 tolerance.
- Each anvil's inner top edge has a 2.0 x 45 deg lead-in chamfer (the usual lead-in of a fixed gap gauge), so that a
  block set slightly off centre slides into the gap; a block wider than the gap settles between the two chamfers,
  above the gap.
- What matters: the two gap widths, +0.05/-0 (the smallest margins are 0.8 mm a side for a block that must drop in and
  0.3 mm a side for one that must stay out; set each anvil against a 41.30 and a 38.70 gauge block or slip-gauge
  stack before tightening), the chamfers (2.0 +- 0.2 at 45 deg), and the four anvil tops level with each other to
  0.1. Everything else to 0.5.
- Finish: ground or smooth anvil faces and chamfers (steel against aluminium, friction about 0.4), so that blocks
  slide on the chamfers.
- Make: ground steel or aluminium bar for the anvils, each fixed with two M4 screws from under the plate (counterbored)
  plus one dowel; or machine the gauge from one plate. 3D-printed anvils (PETG, 100 % infill) glued or screwed to a
  metal plate also work if the gaps are checked after fixing. Paint the GO anvils green and the NO-GO anvils red; leave
  the tops bare or lightly coated (paint thickness changes the gap only on the inner faces: mask them).

## Blocks

- Six aluminium (6061) blocks, 60 long x 20 tall, widths A 39.3, B 39.5, C 39.7 (in tolerance, 40.0 +- 1.3),
  D 37.1, E 36.8 (undersize), F 41.9 (oversize), each +-0.05, edges deburred (0.2 chamfer at most). Cut from 20 mm
  flat bar and mill to width. Mass 119 to 136 g.
- Mark the letter on the underside only (engraving or paint marker), so it cannot be seen from above.
- 3D-printed blocks are about half as heavy (PLA or PETG); if used, add a steel insert under the top face to bring each
  to the aluminium mass above (within 10 g). Aluminium bar is cheaper and more repeatable.

## Trays

Two open plastic trays of the ordinary parts-tray or desk-organiser kind, one green and one red, inside about
120 x 160, walls about 30 tall. Any tray of that class works; the exact size only has to hold three blocks side by side.

## Check before each session (calipers and the gauge itself)

1. Gauge: a 41.30 gauge block (or slip stack) slides into GO with no play greater than 0.05; a 38.70 one into NO-GO.
   A 41.40 stack must not enter GO, nor a 38.80 stack NO-GO.
2. Blocks: measure each width with calipers at both ends (+-0.05 of the value above).
3. By hand, set each block on each gap: A, B and C drop into GO and stay on the NO-GO anvils; D and E drop into both;
   F stays on the GO anvils.

## Set-up and reset per layout

Positions are in cm from the midpoint between the arm bases (x forward, y to the robot's left). Mark the gauge, trays
and the six start places on the table with tape:

- Gauge plate centred at x 32, y 0, long side across (y). Green tray centred at x 17, y +9; red tray at x 17, y -9.
- Start places (block centres, long side front to back): left row x 18, 28, 38 at y +23; right row the same at y -23.
- Which block goes where depends on the layout (README "Layouts"): seed 0 left row F, C, A and right row B, E, D (near
  to far); seed 1 E, F, B / A, D, C; seed 2 B, A, D / F, C, E; seed 3 E, A, B / C, D, F.

Reset: take every block out of the trays and off the gauge, lay each flat at its start place for the chosen layout,
letter down, and check both trays are empty.
