# Seat a thermostat square in a vertical engine housing, valve up, let go, then push the water inlet over two studs

Replacing a thermostat is routine cooling-system work for the 676,570 automotive service technicians and mechanics employed in the US (BLS OEWS 49-3023, May 2023), and it hinges on one move: a 60 mm thermostat has to go square into a recess on a vertical face, clocked with its jiggle valve at the top, and stay there by itself while the other hand pushes a cast water inlet over two studs to trap it. A cut-away engine front section stands on the table with its thermostat housing face vertical, facing the arms. The new thermostat stands on its spring end on a shop mat with its valve pointing to the side; the water inlet lies on the mat with its face toward the engine. The right arm pinches the thermostat's bridge, turns it spring end first toward the engine with the valve at the top, and pushes it into the recess; the left arm takes the inlet by its straight neck with a level grip, lines its two holes up on the studs, squares it up 8 mm onto them, pushes it flush over the thermostat, and lets go.

## Run

From this folder, with the harness's Python on a GPU host:

```
python demokit.py task.py OUT                      # final film, 640 x 360, GPU to yourself
python demokit.py task.py OUT --no-record --mode full,none,no_thermostat,held,valve_side,cocked
python check_score.py                              # the scorer on hand-built states, no simulator
```

The harness's instruction (Dream-1 run): `--instruction "Replace the engine's thermostat: seat the new thermostat in the housing with its spring end toward the engine and its jiggle valve at the top, and let go of it, then push the water inlet home over both studs until it sits flush on the housing, and leave it on the studs over the thermostat."`

## Scoring, and the routes that score less

Everything is read each control step (20 Hz) from the two parts' poses and from points over both whole grippers (both fingers through their thickness, width and length, and the gripper housing behind them), measured against each part's full outer shape. The thermostat is seated when the seal's mid-plane is 2 to 4 mm behind the housing face (3.0 mm with the seal home on its seat), its centre within 1.5 mm of the bore's axis, and its axis within 3 degrees of the bore's, spring end in. The valve angle is the jiggle valve's angle from top dead centre about the bore's axis. The inlet counts either way up (its hose neck down or up); the demo takes it neck down.

| Score | When |
|---|---|
| 0.3 | the thermostat seated for 1 s with no part of either gripper within 10 mm of it and each extreme of the inlet's face over 10 mm from the housing (let go, before the inlet comes) |
| 0.6 | as 0.3 with the jiggle valve within 10 degrees of top, and still within 10 degrees at the end |
| 1.0 | after 0.6, the inlet pushed flush over the seated thermostat for 1 s (both holes within 2 mm of the studs' axes, each extreme of its face within 1 mm of the housing face), then left resting on both studs (holes within 3 mm of the studs' axes, its face within 20 mm of the housing, so it still covers the thermostat's bridge, which stands 20.9 mm proud in the model), with the thermostat still seated, its valve within 10 degrees of top, and both grippers over 10 mm from both parts for 1 s |
| 0 (void) | the thermostat seated for 1 s, then cocked or out of its seat at the end (a pinched seal) |

Without its nuts the inlet does not stay flush once released: it tips on the studs about its lower edge until its holes bind, its top 15.6 mm off the face at the end of the demo. So flush is read while the inlet is pushed home, and the end state reads that it is still on both studs, over a thermostat that is still seated and clocked. The 0.3 and 0.6 tiers record that the thermostat was seated and let go before the inlet came; each also reads the end state (seated, else void; the valve still up for 0.6).

| Mode | What it does | Score (GPU) |
|---|---|---|
| full | the thermostat pinched by its bridge, turned spring end in with the valve up, pushed home and let go; the inlet taken by its neck, its holes started on the stud tips, squared 8 mm on, pushed flush, held 1.5 s and let go | 1.0 (921 steps, 46.05 s, on an RTX 3090 Ti; seal mid-plane 2.91 mm deep, tilt 0.18 degrees, valve 0.0 degrees off top) |
| none | nothing | 0.0 |
| no_thermostat | the inlet pushed flush over the empty housing and let go, the thermostat left on the mat | 0.0 (the inlet flush and left on the studs, nothing behind it) |
| held | as full, but the left hand keeps holding the inlet flush to the end | 0.6 (the inlet never left the hand) |
| valve_side | as full, with the thermostat turned so its valve sits 90 degrees from top | 0.3 (the wrist got there only in part: the valve ended 32.1 degrees from top) |
| cocked | as full, with the thermostat pushed in tilted 6 degrees and let go | 0.0 (it stuck 1.7 mm in and 2.6 degrees cocked, never seated; the inlet then pushed it home, which earns nothing; real check: the bridge top reads outside the hand-seated reference, below, before the inlet comes, so it scores 0) |
| Dream-1 | the model, given the instruction above | 0.0 (180 s, one session, no tier reached) |

`check_score.py` runs the scorer on 17 hand-built sequences of states, including the inlet closed over an empty housing, the thermostat put in backwards (pellet out), the thermostat held in by the hand until the inlet comes (0), the valve 9 degrees off top (1.0), the valve turned to the side after the thermostat is let go (0.3), the inlet flush for only 0.5 s (0.6), and the hand 5 mm from the inlet at the end (0.6).

## Simulation

- Rigid bodies on GPU. The engine section is fixed; the thermostat (110 g) and the water inlet (320 g, cast aluminium) are free, colliding bodies. All meshes are built in `task.py`.
- The housing's recess is 61.0 mm across and 6 mm deep (0.5 mm radial clearance on the 60.0 mm seal: the real seat's diameter and the seal's squeeze are not published, so this is a stated model value) with a 1.5 mm 45-degree lead-in chamfer; the coolant passage behind it is 50 mm across. The two M8 studs are 92 mm apart, 30 mm proud, their ends chamfered from 8 mm to 3.6 mm over 2.5 mm. The inlet's holes are 10 mm across, the coarse clearance hole for M8 (ISO 273).
- The thermostat stays in its recess by contact and friction only, and the inlet rides the studs on its holes: there are no joints, snaps or proximity rules. The rubber seal is rigid. A real seal is a soft ring pressed into its seat with a light squeeze, whose force and the real 2GR-FE seat diameter are not published in the sources opened; the model leaves 0.5 mm radial clearance on purpose, which is easier than the real fit, and the real trial plate below keeps the same 0.5 mm. Friction 0.8 static, 0.7 dynamic on both parts and 0.6 / 0.5 on the engine (estimates).
- The thermostat's shape follows the 60 mm Stant 46128 class used on the 3.5 L 2GR-FE V6: a 56.0 mm flange with a 60.0 mm seal, a 34 mm wax-pellet cage reaching 42.9 mm toward the engine, and a bridge 23.9 mm tall on the other side (both measured in the model from the seal's mid-plane, the flange sitting 3.5 mm forward of it; see Real items for the departure) with its crossbar, which the hand pinches: a stamped sheet strap 12 mm wide and 1 mm thick (about 1 mm thick, from the Gates 33868 product photo scaled to its 56 mm flange; the width is an estimate), its two legs 3 mm thick (an estimate). The jiggle valve sits on the flange between the bridge's legs.
- The water inlet: a 72 mm flange with two ears, a domed cap and a straight neck bending 25 degrees down toward the hose, shaped after the Dorman 902-5932.

## Real trial

Parts: a Stant 46128 thermostat with its seal (about $15), a 3.5 L 2GR-FE water inlet such as the Dorman 902-5932 (about $30), and the housing. A junkyard engine front section works; a low-cost equivalent is a 20 mm aluminium or hardwood plate, about 160 x 110 mm, with a 61.0 mm recess 6 mm deep, a 1.5 mm chamfer round its mouth and a 50 mm hole through behind it (a drill press and a hole saw or Forstner bit), and two M8 studs (threaded rod, a few dollars) screwed in 92 mm apart, 30 mm proud, their ends chamfered. Clamp the plate vertical to a heavy block or a bench vice, its recess 15 cm above the table, and lay a rubber mat in front of it.

Reset (about 1 minute): lift the inlet off the studs and lay it on the mat, face toward the housing, on its flange edge and hose end; pull the thermostat out of the recess and stand it on its spring end on the mat, valve to the side. Nothing is consumed; fit a new seal if it has been pinched.

Score: film from a close side camera at bore height with a 1 mm shim as scale, from a top camera over the housing, and from the front. Scribe two marks 10 degrees either side of top on the housing face before the run. Seated and clocked: with the thermostat let go for 1 s, neither gripper within 10 mm of it and the inlet's face over 10 mm from the housing, the jiggle valve sits between the marks in the front view, and the top camera over the housing, with the 1 mm shim as scale, shows the bridge's two legs equally proud within 2.5 mm of one another. Before the run, seat the real thermostat by hand once, square, and measure its bridge-top height above the housing face with a straightedge laid across the bridge top and a caliper depth rod down to the face; a trial reads seated when the bridge top is within 1 mm of that reference (a cocked one reads outside it, and the model's 20.9 mm is not the real value). Flush: with the inlet pushed home, its face shows no gap over 1 mm all round, held 1 s (the side camera at bore height reads the top and bottom edges, a top camera over the housing reads the gap at both ears). After the run, lift the inlet off and read the seal with a depth gauge on the flange face at four points around it: the depth of the hand-seated reference +/- 1 mm and within 3 mm of one another, with the valve still between the marks. End: the inlet still hangs on both studs over the thermostat, its face within 20 mm of the housing face and covering the thermostat's bridge, with neither gripper within 10 mm of either part, held 1 s.

## Real items

- Thermostat: Stant 46128, with its seal, for the 2017 Toyota Camry 3.5 L (2GR-FE V6); parts page https://www.autopartsprime.com/p/2017-toyota-camry-thermostat. Size map (page figures to model): seal 60.0 mm = 2.362 in; flange 56.0 mm = 2.204 in; 42.9 mm = 1.69 in and 23.9 mm = 0.94 in, the page's flange-to-cage-bottom and flange-to-bridge-top figures. Departure: the model measures both from the seal's mid-plane, its flange 3.5 mm forward of it, so the model's bridge top is 3.5 mm lower than the real part's (about 24 mm proud when a real one is seated, against 20.9 mm in the model) and its cage bottom 3.5 mm longer; 28 mm foot = 1.1 in bypass plate. Estimates: the 34 mm pellet cage, the 12 x 1 mm crossbar and 3 mm legs, the 3.5 mm flange offset and the 110 g mass.
- Water inlet (thermostat housing cover): Dorman 902-5932, Toyota OE 16321-31030 / 16321-31031, listed for the 3.5 L V6 in the Camry 2007-2011, Avalon, Highlander, RAV4, Sienna, Venza and Lexus RX350 / ES350 (fitment: https://stockwiseauto.com/dorman-902-5932-engine-coolant-thermostat-housing and https://www.finditparts.com/products/8027545/dorman-902-5932); product photo https://www.rockauto.com/info/42/902-5932-007__ra_m.jpg. The same 2GR-FE engine family as the 2017 Camry thermostat; the 2017 Camry listing for this inlet was not checked. Estimates: the 72 mm flange, two ears, domed cap, 25-degree neck and 320 g mass are shaped after the part, not measured; the 10 mm holes are the ISO 273 coarse M8 clearance.
- Repair manuals: the Toyota 2GR-FE manual, Cooling, Thermostat (https://www.manualslib.com/manual/1088026/Toyota-2gr-Fe.html?page=779) is indexed with the snippet "within 10°on either side", but the page could not be opened, so that quote is as seen in the index. The opened Toyota 2UZ-FE manual (https://lc100e.github.io/pdf/rm/rm630e/m_co_0012.pdf) gives 30 degrees. The 10-degree clocking limit is the stricter 2GR-FE figure. The 1HZ / 1HD manual RM617E (https://lc100e.github.io/pdf/rm/rm617e/m_co_0013.pdf) gives "jiggle valve upward".
- Housing and stud sizes (61.0 mm recess, 92 mm stud spacing, M8) are set to the 60 mm seal with 0.5 mm clearance and the M8 studs; they are a stated model, not measured from a part; the real 2GR-FE seat diameter is not published in the sources opened, and the 0.5 mm clearance (real seals are pressed in with a light squeeze) is a deliberate allowance, kept the same in the real trial.

## Sources

- BLS, Occupational Employment and Wages, May 2023, 49-3023 Automotive Service Technicians and Mechanics (https://www.bls.gov/oes/2023/may/oes493023.htm): 676,570 employed.
- O*NET 49-3023.00 Automotive Service Technicians and Mechanics (https://www.onetonline.org/link/details/49-3023.00), Core task: "Repair and service air conditioning, heating, engine cooling, and electrical systems."
- Toyota 1HZ / 1HD engine repair manual RM617E, Cooling, Thermostat installation (https://lc100e.github.io/pdf/rm/rm617e/m_co_0013.pdf): "Install a new gasket to the thermostat." "Install the thermostat with the jiggle valve upward."
- Toyota 2GR-FE engine repair manual, Cooling, Thermostat (https://www.manualslib.com/manual/1088026/Toyota-2gr-Fe.html?page=779): indexed snippet, page not opened: "within 10°on either side". Toyota 2UZ-FE manual (https://lc100e.github.io/pdf/rm/rm630e/m_co_0012.pdf), opened: 30 degrees. The task uses the stricter 10 degrees.
- Stant 46128 thermostat, 60 mm, and its fit to the 2017 Toyota Camry 3.5 L (https://www.autopartsprime.com/p/2017-toyota-camry-thermostat; 2.362 in with seal, 2.204 in without, 1.69 in to bottom, 0.94 in to bridge top, bypass plate 1.1 in).
- ISO 273: clearance holes for M8, fine 8.4 mm, medium 9 mm, coarse 10 mm.
- Friction values and the inlet's and engine's shapes beyond the figures above are estimates. No third-party meshes are used.

## Licence

MIT
