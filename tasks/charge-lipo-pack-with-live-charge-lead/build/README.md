# Bench bracket for the charge lead's female XT60

In the real trial the charge lead's female XT60 is a panel-mount Amass XT60E-F soldered to the lead's two AWG14 wires.
It is screwed to this bracket, so it stays put while the robot pulls the pack's male XT60 out of it (18 N in the
simulation; the pair must pull apart at 15-20 N, see the check below). The simulation models it as a fixed female
shell 17 mm above the table, mating face toward the robot.

## Parts
| Item | Qty | Size / part |
|---|---|---|
| Panel-mount female XT60 (on the charge lead) | 1 | Amass XT60E-F |
| Bracket (printed, PETG or ABS, 100 % infill on the upright) | 1 | `bracket.stl` from `make_bracket.py`, 60 x 40 x 4 mm base, 40 x 4 x 32 mm upright |
| Screws + nuts for the XT60E-F flange | 2 | M2.5 x 8 mm, steel |
| Bench clamps or screws for the base | 2 | any, holding the base flat on the mat |

## Drawing (mm)
- Base 60 x 40 x 4, upright 40 (across) x 4 (thick) x 32 (high), centred on the base.
- Window for the XT60E-F body: 17.0 x 9.0, centre 17 above the table.
- Two 2.7 mm clearance holes at +-s/2 from the window centre, s = flange hole spacing measured on the XT60E-F
  (`--hole-spacing`; the default 20 mm is a placeholder, measure before printing). The Amass XT60E-F specification
  sheet (https://static.chipdip.ru/lib/380/DOC042380583.pdf, version 2022V1) gives only the outline (flange
  34.00 x 15.80 mm, depth 20.20 mm), not the hole spacing, so the placeholder stays until the part is measured.

## Assembly
1. Measure the XT60E-F flange hole spacing with calipers; run `python make_bracket.py --hole-spacing <s>` and print.
2. Solder the charge lead (red to +, black to -) to the XT60E-F; heat-shrink.
3. Screw the XT60E-F to the upright from the robot side, mating face toward the robot.
4. Clamp the base at 27 cm forward, 22.7 cm left (table frame of the task setup).

## Checks before trials
- Retention: with a push-pull gauge pull the female XT60 axially at 50 N: it must not move (bracket and screws).
- Pair force: mate the pack's male XT60 and pull it out with the gauge: 15-20 N, otherwise replace the pair.
- Height: the XT60 axis is 17 mm above the table (ruler).

## Reset
Stop the charge, pull the pack's XT60 out first, then the banana plugs. Check the screws. Re-mate the pack's XT60 by hand to full depth (housings flush) last.
