# Set a probe positioner on a probe station: slide it to the die, hold it as its magnet locks, trim its tip by knob

A loose MP40-class probe positioner stands on the steel platen of a TS150-class wafer probe station, its magnet lever OFF. The right gripper slides its base along the platen until its needle points at the marked die, lets go and presses the base down by its top while the left pushes the magnet lever to ON and then turns the X knob; the left then presses the top while the right turns the Y knob, until the tip is over the mark, 2 mm short of the other positioner's tip.

## Run

From this folder, on a GPU host (not the Mac):

```
python demokit.py task.py OUT                      # final film, 640 x 360, GPU to yourself
python demokit.py task.py OUT --no-record --mode full,none,one_hand,unheld,no_trim,touch
```

The harness's instruction (Dream-1 run): `--instruction "Set the loose probe positioner up on the marked die. Its magnet is off. Slide it along the platen until its needle points at the die with the tip near the mark, never touching the other needle, the wafer or the microscope. Keep it from shifting while you push its magnet lever to ON. Then turn its X and Y knobs to bring the tip over the mark, short of the other probe's tip. Let go."`

`PP_DX` (default 0.06 m) sets the whole station that far out in x, so both wrists stay inside their joint limits.

## Scoring, and the routes that score less

Read at the end from the station: the tip against the mark in plan, the lever, the tip's shift across the lever's throw, and every fingertip against the needles, wafer and lens. A person reads the 0.3 mm trim through the station's microscope reticle or a printed scale. Tiers are in `task.json`: 0.5 needs the tip within 4 mm of the mark when the lever latches; 0.75 needs the knobs turned at least 90 degrees in all after the latch and the base moved under 0.5 mm since it.

| Mode | What it does | Score (GPU) |
|---|---|---|
| full | right slides the base by its tip's reading, then presses its top; left throws the lever, stopping at ON, and turns the X knob; left presses while right turns the Y knob; part turns, the tip read after each | 1.0 |
| none | nothing | 0 |
| one_hand | left alone throws the lever, base unslid and unheld | 0 |
| unheld | base slid to the die, let go, lever thrown with nobody pressing it | 0 |
| no_trim | slide, press and clamp, knobs untouched (the slide drifts to 6 mm off at the latch, so no tier is met) | 0 |
| touch | the whole act, then the right hand's fingers go in over the needle and meet the microscope lens (a fingertip within 4 mm of the lens, 6 mm of a needle, or low over the wafer) | 0 |

## Scores

Demo: 1.0. Dream-1 run (one session, 3,000 steps, 16 rows per request): 0.0.

## Real trial

One manual positioner (a used MP40 or Signatone S-725 class unit, about USD 300 to 500, estimate) on a steel plate with an R125 opening, a printed wafer mark, a second fixed needle and a post at lens height; read with a USB microscope camera and a printed scale. The base's drift since the latch is read with a dial indicator against the base; the tip's shift across the throw with the microscope reticle before and after. Keep spare tungsten needles and look at each tip before a trial.

## What is code, and what is an estimate

- The magnet is a 40 N downward pull on the base once the lever passes 62 degrees, so the base's friction holds it (estimate; the maker gives no figure). The lever swing (70 degrees), its 3 N, the knob drag (0.02 N m), the base and needle masses, the 100 mm base height, the 14.4 cm needle reach and friction 0.3 are estimates.
- Each stage is a sprung slide that code drives from its knob's angle: 0.3 mm of tip travel a turn (MP40 screw resolution 300 um; estimate).
- Needle collider 1.2 mm across (real 0.51 mm); the scorer reads needle contact from the centre lines, under 0.6 mm apart (real needle 0.51 mm).
- The demo reads positioner 2's pose and joints, as the microscope view would. It slides on the needle tip's reading, and turns each knob in part turns of 150 degrees (the wrist's reach) until the tip is within 0.1 mm.

## Sources

- MPI TS150 manual (EPFL CMI, 2024): https://www.epfl.ch/research/facilities/cmi/wp-content/uploads/2024/11/MPI_TS150_manual_2024.pdf
- MPI TS150 datasheet, about 330 x 235 mm platen, R125 opening, chuck top 28 mm below the platen top: https://www.epfl.ch/research/facilities/cmi/wp-content/uploads/2021/05/MPI-TS150-Manual-Probe-System-Data-Sheet.pdf
- MIT MTL IV-probe SOP (flip the magnetize lever, tips '2-3 mm apart'): https://mtlsites.mit.edu/fabrication/sops/IV-probe.pdf
- MPI MP40 positioner, 65 x 40 mm, 13 mm travel, magnetic mount: https://www.electrorent.com/products/lcr--impedance-analyzers/semiconductor-test-equipment/mpi/mp40/01t1O000004TcjHQAS
- Tungsten probe tips 0.020 in: https://etesters.com/product/656FD3EF-39FC-4CAC-8FCC-C7DEB4B80E36/solid-tungsten-probe-tips
- All meshes are built in `task.py` from boxes, cylinders and tubes; no third-party meshes.

MPI, TS150 and MP40 are trademarks of MPI Corporation. Signatone is a trademark of Signatone. They are named only to say what the modelled sizes follow; this task is not endorsed by or affiliated with the makers.

## Licence

MIT
