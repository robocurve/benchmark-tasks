# Plasma-cut the overhanging strip off an installed skid's deck plate along a hand-held straightedge

A small equipment skid stands bolted to the station table: two steel angles carry a 3 mm mild-steel deck, a junction box
is still mounted on the deck, and the skid's control cabinet stands beyond its far end. A 55 mm strip of the deck
overhangs the right-hand angle; a white line marks where it comes off, 140 mm from the deck's far edge to its near edge.
A loose aluminium straightedge lies on the deck along the line, and a hand plasma torch with a drag shield rests on its
stand at the right. One hand presses the straightedge to the deck; the other takes the torch, starts at the far edge
against the cabinet side, drags the shield along the straightedge at a cutting pace until the strip drops into the dross
pan, then lays the torch back on its rest, and both hands move away.

## Instruction

Cut the overhanging strip off the skid's deck plate along the white line with the plasma torch, guiding the torch along
the straightedge, then put the torch and the straightedge back where they were and move both hands away.

## Revision 1 (reply to review TK-CK3CZ3)

> Feasibility (not met): "The live plasma arc operates close to unprotected robot hands and wrists, exposing the shared
> rig to heat and molten spatter."
>
> What to change: "Protect the robot hands, wrists and other exposed rig surfaces from the plasma arc, heat and molten
> spatter. Specify the protective materials and their placement, and ensure they do not prevent the required torch and
> straightedge handling."
>
> Flag: "Confirm the real deflector dimensions and guide offset match the specified cut line."

**Protection, by part.** Clearances are the smallest distance from the arc (the shield rim centre on the line) to the
part's surface over every arc-on step of the scripted demo (229 of 858 steps; joint targets of the GPU-simulator demo
run, forward kinematics; links as 30 mm and fingers as 10 mm capsules).

| Part | Material (listed product) | Placement and coverage | Clearance to the arc |
|---|---|---|---|
| Both forearms and wrists (from the elbow joint to the gripper housing) | Split cowhide leather welding sleeve, Kevlar-thread seams, rated by the maker for sparks and spatter: Tillman 5218, 18 in (457 mm), one sleeve per arm | Over the forearm and wrist links (about 400 mm), snap cuff closed on the gripper housing at least 20 mm behind the finger slides; the fingers and pads stay bare | left gripper housing 185 mm, left wrist 203 mm; right gripper housing 214 mm, right wrist 247 mm; forearms 292 mm or more |
| Fingers and pads | none: the pads must grip the torch's nylon block and the straightedge's handle block | The torch hand's pad tips are 107 mm from the arc (fixed by the torch: the grip block sits 59 mm beside and 80 mm above the shield rim); the other hand's nearest fingertip is 133 mm away and the scorer zeroes the trial if it comes within 45 mm of the nozzle while the arc is on | 107 mm (torch hand), 133 mm (other hand) |
| Torch lead and the arms' exposed cabling at the wrists | Silicone-coated fiberglass fire sleeve, 1 in ID, 260 C continuous, 1250 C peak, VW-1 (Electriduct Thermo Armor) | The torch lead's first 0.9 m from the handle, and any cable or connector left outside the leather sleeves | about 170 mm (geometry: the lead leaves the handle's far end, 19 cm along the handle from the head) |
| Table, skid surroundings and pan | 24 oz acrylic-coated fiberglass welding blanket (already specified) and 10 mm of dry sand in the pan | Over the table round the skid and pan, as before | under the cut |
| Cameras | Shade 5 glass filters (already specified) | Over every camera lens | wrist cameras ride on the gripper housing (185 mm or more, as above); top camera about 0.9 m above the deck |

Why spatter mostly stays below the plate: the cutter's manual says to drag the tip on the work and "make sure that sparks
exit from the bottom of the workpiece ... If sparks spray up, you are not cutting all the way through", and the cut here
starts from the plate's edge, so there is no pierce (Powermax30 AIR operator manual 808840, hand torch cutting
guidelines and "Pierce a workpiece"). The scorer counts a segment as cut only within the cutter's speed window, where
the manual's cut chart gives a full cut.

**Handling is not blocked.** The sleeves stop at the gripper housing and the fire sleeve stays on the lead and the
cabling, so neither touches the finger pads, the torch's grip block, the straightedge's handle block or its working
edge, and nothing is added between the shield and the straightedge. The robot presses no trigger (a supervisor's
hold-to-run switch replaces it). Because the covers change no contact, they are not modelled in the simulator (see
Fidelity).

**Deflector and guide offset.** The Powermax30 AIR uses the copper deflector 420399 on its retaining cap for drag
cutting. Hypertherm publishes the consumable stack's outer diameter, 25 mm (1.0 in), and its length, 51 mm (2.0 in)
(manual 808840, Torch dimensions), but not the deflector tip's diameter. The part that bears on the 6.35 mm
straightedge is the deflector's lowest section, about 12 mm across when scaled from that drawing (unverified, a drawing
scale). The real trial therefore sets the guide from the torch itself rather than from a number: with the deflector's
centre hole over the line and the deflector against the straightedge's working edge, the bar is laid along the line
and the gap from the line to the edge is measured with calipers and recorded (about 6 mm). The nozzle is then on the
line along the whole cut by construction. The simulation keeps its 20 mm copper shield and a 10 mm offset, which stand
for the deflector and its offset.

## Why the task matters (category Other)

The work is cutting up steel equipment where it stands, which is how contaminated plant is taken apart in nuclear
cleanup and how scrap is broken down.

- Scale: the US Treasury's FY 2025 statements list $538.6 billion of the federal government's $667.3 billion
  environmental and disposal liabilities under the Department of Energy (Note 14, as of September 30, 2025).
- Plasma is a main cutting method there. EPRI report 1003029: "Plasma arc cutting was the primary method of segmenting
  the RPV internals at Shoreham and the Yankee Nuclear Power Station." Anderson et al. (University of Strathclyde,
  Nuclear Engineering and Design, 2026): "Laser and plasma cutting systems are routinely used for size reduction tasks
  within the nuclear decommissioning sector"; their test pieces are "3 mm-thick 316 stainless steel plates
  representative of nuclear glovebox material", the same thickness as this deck.
- It is still done by hand, in suits, on equipment in place. At Rocky Flats Building 771, "Workers wearing cumbersome
  protective clothing cut up glovebox sections", and "Using plasma-arc torches to cut up stainless steel gloveboxes
  significantly improved safety" (DOE fact sheet). The first of Sellafield's gloveboxes to be cut up, in 2024, "required
  operators to perform cutting manually while wearing air-fed suits and respirators" (Anderson et al.). The same paper
  cites a study (Kitamura et al., 2023) in which "radiation exposure doses to workers were less than 1/5th of those in
  manual cutting facilities" with robotic glovebox dismantling.
- Scrap: "One of the most common tools used to break apart large metal pieces is the gas cutting torch ... other torches
  use plasma" (OSHA 3348). Robotics researchers working with a scrap recycler: "Currently, the cutting operation is
  conducted manually by skilled workers using a gas torch... This manual operation is slow and laborious", and "We are
  not aware of any work in the literature on robotic methods for gas cutting that is general enough to be applied to
  metal scrap recycling" (Akl, Alladkani and Calli, IEEE CASE 2021; their scope is gas torches).
- Why a guide: the cutter's own manual says "For straight-line cuts, use a straight edge as a guide" and "Drag the torch
  tip lightly on the workpiece to maintain a steady cut speed" (Hypertherm Powermax30 AIR operator manual 808840, hand
  torch cutting guidelines).

Limits: no source gives the share of this cutting done with a hand torch along a guide; the dose ratio is a third-party
study cited by Anderson et al., not a Sellafield measurement.

Sources: https://fiscal.treasury.gov/system/files/2026-04/2025-note-14-financial-statements.pdf ;
https://restservice.epri.com/publicdownload/000000000001003029/0/Product ; https://strathprints.strath.ac.uk/96760/ ;
https://www.energy.gov/sites/prod/files/edg/news/archives/documents/Building771FactSheet.pdf ;
https://www.osha.gov/Publications/OSHA3348-metal-scrap-recycling.pdf ; https://par.nsf.gov/servlets/purl/10300191 ;
Powermax30 AIR operator manual 808840: https://www.hypertherm.com/Download?fileId=HYP135285

## The installed skid and its access constraints

The earlier version of this idea cut an open plate on a bench stand. Here the plate is part of installed equipment and
is cut where it stands:

- **Deck on the skid, not on a stand.** The deck is fixed on two 2 x 2 x 1/4 in steel angles (53.8 mm from the table to
  the deck top). The right-hand angle's outer face is 15 mm back from the line, so the jet clears it; the torch has to
  stay square over the overhang with the steel angle under it.
- **The far end is closed by the control cabinet** (120 x 420 x 250 mm, its face 100 mm past the deck's far end). There
  is no run-in from beyond the far edge: the torch is set down just past the deck's end (14 mm), between the deck and the
  cabinet, and drawn onto the deck from there. The arm working at that end is at its full reach.
- **The junction box stays mounted** on the deck's left part (a 4-11/16 in square steel box, 2-1/8 in deep, with its
  conduit connector). The hand that holds the straightedge works beside it, and keeps 45 mm or more from the nozzle.
- **The straightedge is loose on the deck.** Nothing clamps it: only the hand's press holds it while the shield bears on
  its edge. Left unpressed, the cut loses segments (variant `unheld` below).

## Scene and positions

Positions in cm, x forward from the arm bases' midpoint, y toward the left arm, z up from the table top.

- **Deck:** 3 mm mild steel, x 21.5 to 35.5, from y -15.6 (the strip's edge) to y 18.5; the cut line at y -10 runs
  the deck's full 14 cm. The 55 mm wide strip right of the line weighs 0.18 kg.
- **Skid angles:** 14 cm long along x, outer faces at y -8.5 (right) and y 18.5 (left).
- **Junction box:** 11.9 x 11.9 x 5.4 cm on the deck, centred at (28.5, 11.95).
- **Control cabinet:** its front face at x 45.5, 42 cm wide centred on y -5, 25 cm tall.
- **Dross pan:** steel, x 19.5 to 38.5, y -19 to -8.7, 1.2 cm rim, on the table under the overhang.
- **Torch** (0.10 kg at the hand; the balancer carries the lead and the rest of its weight): a straight 19 cm handle
  along y, 2.4 cm square, with a yellow grip block (3 x 3 x 2.4 cm) clamped on top; at its +y end a vertical head 2.6 cm
  across and 6 cm tall with a 2 cm copper drag shield, the head's axis 5.9 cm from the grip block's centre. It lies on two
  posts of its rest, the handle's axis 8 cm above the table, centred at (25, -25).
- **Straightedge** (0.14 kg): 1 x 1/4 in 6061 aluminium flat bar, 18 cm long, laid flat on the deck from x 20.5 to 38.5
  with its working edge at y -9.0 (10 mm, one shield radius, from the line) and an aluminium tab on its back edge carrying
  a red 3 cm handle block 11.5 cm from the working edge.
- Both grippers start open at the ready pose.

## Scoring

The deck's 14 cm line is 14 segments of 10 mm. The arc is on while the torch is in a hand (a gripper's tool point within
45 mm of the torch's handle centre), within 30 degrees of square, the shield's rim from 1 mm into the deck's top to 2 mm
above it, over the deck between its ends and within 10 mm of the line. Each step with the arc on adds 0.05 s to the
segment under the nozzle.

A segment is **sound** when all of these hold:

| Criterion | Simulation | Real trial |
|---|---|---|
| Cut through | 0.482 s or more of arc time on the segment (travel at 20.75 mm/s or slower: the cutter's maximum speed for 3 mm at 20 A, 1,245 mm/min) | No bridge of metal joins the strip to the deck along the segment (light through the kerf, or a cut face its whole length) |
| Not overheated | 1.575 s or less (6.35 mm/s or faster: half the rated 762 mm/min, a modelling choice; the manual gives no slowest speed) | No dross bead hanging more than 1.5 mm below the edge and the top corner not melted back more than 1 mm |
| On the line | The nozzle never more than 2 mm from the line while the arc is over the segment | Cut edge 18 to 22 mm from a witness line scribed 20 mm from the cut line |
| Square | The torch never more than 10 degrees from square while the arc is over the segment | Engineer's square on the cut face: no gap over 0.5 mm |
| Guided | For 80% or more of the segment's arc time, the shield's side within 1 mm of the straightedge's working edge, with the straightedge flat on the deck (within 1 mm of the deck and 3 degrees of flat) | A 0.5 mm feeler enters nowhere between a steel straightedge laid on the cut edge and the edge; the video shows the shield against the guide |

The strip hangs on the uncut bridges and drops only when every segment is cut through.

Score: **0** if the torch or the straightedge falls off the table (more than 1 cm below its top), or if the hand not
holding the torch comes within 45 mm of the nozzle while the arc is on (both latched). Otherwise **0.8 x the share of
sound segments** (rounded to 0.001; 7 of 14 sound scores 0.4). **0.9** with all 14 sound and the strip in the dross pan (its centre 25 mm or more
below where it started, over the pan). **1.0** with, as well, the torch back on its rest (within 15 mm in plan and 6 mm in
height of its start, tilted under 10 degrees, turned under 20), the straightedge still where it was laid (its handle
block within 20 mm in plan and 6 mm in height of its start, tilted under 3 degrees, turned under 20), both grippers 6 cm
or more from the torch, the straightedge and the strip, and those three still at the end. Doing nothing scores 0.

Real trial: read once the parts are under 50 C, with a 150 mm steel ruler, a 300 mm steel straightedge, a 0.5 mm
feeler gauge and an engineer's square; each segment photographed with the ruler in view.

## Real-trial setup

- **Cutter:** Hypertherm Powermax30 AIR (built-in compressor) on 120 V / 20 A, set to 20 A, with its standard AIR
  consumables and the deflector (420399) on the retaining cap; the shield is dragged on the deck as the manual says. The
  AIR has no separate drag shield, so the simulated 20 mm copper shield stands for its retaining cap and deflector.
- **Arc control:** the robot presses no trigger. The supervisor, outside the screens, holds a hold-to-run switch wired
  in place of the torch's trigger and closes it only while the shield is on the deck (within 2 mm), between its ends and
  within 10 mm of the line, as seen on the filtered camera view. One emergency stop cuts the robots' power and the
  cutter's output.
- **Torch and lead:** a printed or machined nylon grip block clamped on the handle with two screws; the lead hangs from
  an overhead balancer set so a spring scale at the grip reads 0.10 kg.
- **Cutting circuit:** the work clamp on the skid; the skid on insulating pads; both grips insulating.
- **Cameras and arc light:** a shade 5 filter over every camera lens. Before each session a 100 mm test cut on scrap is
  recorded through the filtered robot cameras and the supervisor's view, and must show the torch, the straightedge and
  the cut line throughout (the earlier review's flag).
- **Robot protection:** a Tillman 5218 leather welding sleeve on each forearm and wrist, cuff snapped on the gripper
  housing 20 mm or more behind the finger slides; fire sleeve on the torch lead's first 0.9 m and any exposed wrist
  cabling; finger pads bare (see Revision 1 for the clearances).
- **Guide offset:** set from the torch: deflector centre hole over the line, deflector against the straightedge's
  working edge, bar laid along the line; the line-to-edge gap is measured with calipers and recorded.
- **Sparks, fume, fire:** a welding blanket over the table round the skid and pan; 10 mm of dry sand in the pan;
  welding screens close the cell, nobody inside while the arc can fire; cut outdoors or in an open bay with a fan drawing
  the plume away; only clean uncoated steel; an extinguisher and a 30 minute fire watch.
- **Before a trial:** three test cuts on scrap of the same plate along the straightedge (fast, rated 12.7 mm/s, slow)
  confirm the cut-through and overheating thresholds on the real edge.

## Real trial hardware

Prices seen 2026-11-21 in live US listings (rental or purchase).

| Item | Listing | Price |
|---|---|---|
| Plasma cutter (rent) | Powermax 30 AIR, Miller's Rentall, Edmonds WA: https://www.millersrentall.com/equipment.asp?action=category&category=71&key=PLASMACUT | $100.00 a day |
| Plasma cutter (buy, alternative) | Hypertherm Powermax30 AIR 088096, Welder Supply: https://www.weldersupply.com/P/999/HyperthermPowermaxAir240 | $2,209.90 |
| Spare nozzle and electrode | Hypertherm 420134 and 420132, Hypermax: https://www.hypermaxonline.com/powermax30_air_consumables | $8.53 + $8.53 |
| Skid angles | 2 x 2 x 1/4 in A36 angle, 12 in (cut in two), Speedy Metals: https://www.speedymetals.com/p-3947-2-x-2-x-14-angle-a-36-1020-hot-rolled-steel.aspx | $9.57 |
| Straightedge bar | 1/4 x 1 in 6061-T6511 flat, 12 in, Speedy Metals: https://speedymetals.com/p-2241-14-x-1-6061-t6511-aluminum-extruded.aspx | $4.84 |
| Junction box | RACO 257H 4-11/16 in square steel box, 2-1/8 in deep, Menards: https://www.menards.com/main/electrical/electrical-boxes-covers/electrical-boxes/raco-4-11-16-galvanized-steel-square-electrical-box-with-bracket/257h/p-1444451763574-c-6425.htm | $8.75 |
| Welding blanket | 6 x 8 ft fiberglass, Rig Welder Supply: https://rigweldersupply.com/24-oz-gold-acrylic-coated-fiberglass-welding-blanket-6x8/ | $47.12 |
| Shade 5 filters | 2 x 4 in glass, three, Welders Helper: https://www.weldershelper.com/product/shade-5-glass-filter-plate-2x4-welding-lens/ | 3 x $1.96 |
| Fan | Lasko 3720 20 in box fan, Do it Best: https://www.doitbest.com/product/513824/lasko-20-in-3-speed-white-weather-shield-performance-box-fan-513824/ | $44.99 |
| Robot sleeves | Tillman 5218 18 in leather welding sleeves (pair), Weld Shop Supply: https://www.weldshopsupply.com/products/tillman-18-leather-welding-sleeves-5218 | 2 x $23.00 |
| Fire sleeve | Electriduct Thermo Armor silicone-coated fiberglass, 1 in ID x 3 ft, Walmart: https://www.walmart.com/ip/Thermo-Armor-Silicone-Coated-Fiberglass-Fire-Sleeve-1-3-Feet-Electriduct/658812375 | $16.70 |
| **Priced total (rental route)** | | **$300.91** |

Not priced (gaps): the deck plate (3 mm / 1/8 in A36, one 140 x 341 mm piece per trial; Warrenton Steel lists 12 x 12 x
1/8 in A36 plate with no price shown), the 3 cm hardwood handle block and the straightedge's aluminium tab, the dross pan
(a steel tray of at least the simulated size), the cabinet (any closed steel enclosure of the simulated size), the
balancer, the screens and the hold-to-run switch (standard welding-shop items).

## Run a trial

From the package directory, in the official Arena environment:

```sh
inspect-robots run --embodiment yam_arena -E task=plasma_cut_installed_panel_along_straightedge.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 --policy yam_scripted --scorer task_score --max-steps 2000 --instruction "Cut the overhanging strip off the skid's deck plate along the white line with the plasma torch, guiding the torch along the straightedge, then put the torch and the straightedge back where they were and move both hands away."
```

Dream-1:

```sh
inspect-robots run --embodiment yam_arena -E task=plasma_cut_installed_panel_along_straightedge.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 --policy dream --scorer task_score --max-steps 2000 --instruction "Cut the overhanging strip off the skid's deck plate along the white line with the plasma torch, guiding the torch along the straightedge, then put the torch and the straightedge back where they were and move both hands away."
```

The run limit is 2,000 steps (100 s at 20 Hz); the scripted demo ends in about 860 steps.

Runtime per trial: about 5 min on one RTX 5090 (the local scripted-demo run took 290 s including start-up).

Seeds: none; the start is fixed (`object_jitter=0`).

**The scripted demo** is open loop and reads no simulator state. The left hand comes down with closed fingers on the
straightedge's handle block and presses the bar to the deck. The right hand lifts the torch off its rest by the grip
block, sets the shield down just past the deck's far end beside the cabinet, draws it sideways against the
straightedge's edge, drags it along the edge to the near end at the rated 12.7 mm/s (the first centimetre a little
faster, where the arm lags at full reach), lifts off, and lays the torch back on its rest. The strip drops into the pan
as the last segment is cut. Both hands then rise clear.

Demo results: the scripted demo scored 1.0 in the GPU simulator (local Isaac, 840 steps; all 14 segments sound, the
strip in the pan) and 1.0 in offline MuJoCo (858 steps). On the cloud simulator the demo scored 1.0 (858 steps) and Dream-1 scored 0.0 over the full 2,000-step run.

**Shortcut variants** (development only, not shipped), scored in offline MuJoCo:

| Variant | What it does | Score |
|---|---|---|
| `nothing` | no motion | 0.0 |
| `fast` | travel at 30 mm/s | 0.0 (nothing cut through; the strip stays) |
| `slow` | travel at 5 mm/s | 0.0 (cut through and dropped, every segment overheated) |
| `offguide` | the shield run 1.5 mm clear of the straightedge, still on the line | 0.0 (no segment guided) |
| `half` | the cut stops halfway | 0.343 (6 sound segments; the strip stays) |
| `unheld` | the left hand never presses the straightedge | 0.629 (11 of 14 segments sound) |

The offline checker's own `half` shortcut (it stops the demo half way along) scores 0.4: 7 of 14 segments sound,
0.8 x 7/14, the strip still attached. With the start positions jittered by up to 5 mm and 3 degrees the open-loop demo
scored 0 to 0.743 in that checker, since the loose straightedge then lies off the line the demo follows; the task's
start is fixed (`object_jitter=0`).

## Offline tests

With Arena on the import path: `python -B -m pytest -q -p no:cacheprovider`. The tests drive the cut bookkeeping step by
step with posed states (no simulator): the score tiers, the speed window, the guide and flatness conditions, the zero
latches, the straightedge having to stay where it was laid, holding still scoring 0, and the demo's reach and length.

## Fidelity

Rigid bodies with real contact: the shield slides on the deck and bears on the loose straightedge, which moves if the
hand does not press it. The arc, kerf, heat, sparks and dross are not simulated: cutting is arc time per segment from
the torch's measured pose, and the strip hangs on one weld released when the last segment is cut through
([DEVIATIONS.md](DEVIATIONS.md)). The robot's protective sleeves and the fire sleeve are not modelled: they touch
none of the parts the hands or the shield contact, so they change no contact in the trial.

## Licence

| Path | Origin | Licence |
|---|---|---|
| *.py, task.json, README.md, DEVIATIONS.md | Task, tests, documentation and bundled Arena helpers | MIT |

Every part is built from boxes in the task's own code; no third-party assets. [LICENSE](LICENSE)
