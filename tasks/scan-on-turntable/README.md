# Start a handheld 3D scanner with its button and scan a cast housing all round, up over its top, to compare with its CAD

A cast-iron plummer block housing stands on a manual turntable. A handheld structured-light 3D scanner stands on its head on the table, handle up, switched off. Scanning starts only when button 4 on the scanner's handle is pressed. The scan must then cover 95 % of the housing's surface, leaving out its underside and the insides of its bore and bolt slots, including its top, which only views from above can see. A scanner held still at one height, as on a tripod, covers at most about 88 % (from about 40 degrees down), so 95 % needs views from two heights; a scanner moved too fast, or out of range, stops tracking.

In the demo the right hand takes the scanner by its handle and holds it still while the left hand presses button 4. The right lifts the scanner and rolls it upright. The left hand then turns the turntable in strokes, dragging its rubber top near the rim, while the right holds the scanner inside its working range and climbs it slowly: from 15 degrees down at the part's sides, through 20 degrees over the first turn, on to 60 degrees down over its top through the second.

**Why it matters.** Inspectors, testers, sorters, samplers and weighers (O*NET 51-9061, 602,000 US workers) have as a Core task: "Inspect, test, or measure materials, products, installations, or work for conformance to specifications." The BLS Occupational Outlook Handbook for the same occupation says: "Although some still use hand-held measuring devices, such as calipers and alignment gauges, workers more commonly operate electronic inspection equipment, such as coordinate-measuring machines (CMMs) and three-dimensional (3D) scanners." O*NET's own tool list for the occupation names CMMs but no 3D scanner; the scanner line is BLS's. A scanner maker's foundry case describes castings checked this way, the scan compared with the CAD.

**Ability tested:** holding an instrument within a distance and angle window while moving it over a 3D surface, from low at the sides to pitched over the top, in step with the turntable, and switching the instrument on with its own button.

**Nearest tasks.**
- A heat-shrink sleeve shrunk onto a cable lug with a heat gun, which also turns the work and moves a contact-free tool past it. The differences:
  - There the hand holds the cable and rolls it; here a free platter carries the part and is pushed at its rim in strokes.
  - There the tool runs along one line at 40 to 60 mm. Here it covers two angles, azimuth from the table and elevation from level to pitched over the top, inside a 200 to 700 mm depth band, facing within 45 degrees, with the part's own walls hiding faces.
  - There the goal is a heat dose, an integral with an overheat limit, so dwell must be metered. Here it is coverage, a union: repeat passes never hurt, and the work is planning views so that hidden faces get covered.
  - There speed sets the dose. Here a tracking limit stops counting above 0.25 m/s, until the scanner is brought back over scanned surface.
- A wall swept with a detector for a hidden pipe: a 2D contact sweep to find one point; here a 3D standoff sweep covers a whole surface.
- A book held open for digitising: the hands protect the book and do not scan it.
- A cake smoother held still against a cake while the other hand turns the turntable: there the tool stays still and the result is one smooth side.
- A turntable turned without wrapping its camera cable; reading a barcode with a scanner.

**Arms.** The instruction does not ask for two hands. In the demo the right arm holds the scanner and the left presses its button and turns the table.

## Requirements

`yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source <ARENA_DIR>/yam.env`). `mechkit.py` and `demokit.py` are in this folder. Importing `mechkit` hooks `yam_arena.scene` and `yam_arena.inspect_embodiment` without changing them on disk.

## Run a trial

From this folder:

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 3000 \
  --instruction "Start scanning with button 4 on the back of the scanner's handle, scan the part on the turntable until the scan covers 95 % of its outer surface, its top included, then move the scanner and your hands at least 3 cm away from the part."
```

## Scripted demo

From this folder:

```bash
python demokit.py task.py OUT --mode sweep             # full score, with a video: the scanner climbing while the table turns
python demokit.py task.py OUT --mode full              # full score in two passes, the scanner held still at each (tracking can drop in the first turn)
python demokit.py task.py OUT --no-record --mode oneturn,still,onearm,strike,nobutton,hover,none   # the checks below
SCAN_DEBUG=1 python demokit.py task.py OUT --no-record # prints coverage, tracking and the scanner's pose every second
python check_score.py                                  # the score lines and the coverage geometry on hand-built cases, with no simulator
```

The demo reads the scanner's pose where it stands, to take it by its handle and press its button, and the pose of the scanner in the hand, to plan the views and the press. The views are fixed poses round the turntable's known centre. It reads no contact force and nothing from the scorer.

## The scene and what is simplified

The turntable is a 30 cm platter on a revolute joint with light damping, 3 cm high, with a rubber top. The part is an SKF SY 505 M plummer block housing without its bearing insert: cast iron on a 127 x 38 x 16 mm base with two bolt slots, the bore's axis 36.5 mm up, a 52 mm bore, 0.55 kg. It stands free on the platter and rides round with it by friction.

The scanner is a Shining 3D EinScan H2 of its maker's size and mass, 108 x 110 x 237 mm and 731 g: a free rigid body, its head and handle as boxes. It has no cable; in a real trial the H2's cable hangs from above, clear of the table, and the software tracks on the part's own geometry, with no markers. It stands on its head on the table, handle up, its lens turned away from the part, so that it sees nothing of the part where it stands. The scanning software is in preview, so scanning is off until button 4, on the back of its handle (Preview / Scan / Pause in the maker's quick start guide), is pressed: a fingertip pressed on the button starts scanning, and a second press pauses it, as the button does. The press is read at the button's face: pressed, a fingertip stops there (within 1 mm of it is counted); a fingertip hovering 1.6 to 2 mm off it does not count (`hover` below). Before the press nothing is counted, and the tablet's tracking lamp stays dark.

The tablet runs a simple scanning app, as the scanner's own software (Shining 3D's EXScan H) shows a scan: two live views, from either side and above, empty until scanning starts and showing only the surface captured so far, built up as it is scanned; a distance indicator, ten 7.5 cm cells (green from 22.5 to 67.5 cm, red outside) with a white mark at the scanner's distance from the part; a tracking badge (green while tracking, red when tracking is lost, grey when not scanning); and a coverage bar, one cell for each 10 %. The live views are built from 64 patches of the part's surface: a patch appears once 70 % of its points are scanned, so the views start empty, as the software has no model of the part beforehand. Button 4 is shown in orange so that it reads in the cameras.

Coverage is computed, not rendered. 4,829 points are spread evenly over the housing's surface, leaving out its underside and the insides of its bore and bolt slots; a real trial counts the same surface. 1,007 of them are on the top: faces turned within 30 degrees of straight up. A point is covered once, at a control step while the scanner is tracking, if:

- it lies inside the scanner's view: 200 to 700 mm along its axis (the H2's depth of field), within 24 degrees across and 25 degrees up and down (its 420 x 440 mm field of view at its 470 mm working distance);
- it faces the scanner within 45 degrees;
- the line from the scanner to it passes through neither the part nor the platter.

Tracking holds while the scanner moves relative to the part at under 0.25 m/s and 60 degrees/s, with at least a quarter of the points inside its view. It starts as soon as that holds after the press. Once lost, it returns only on a view of at least 50 points, 30 % of them already covered, as a scanner finds its place again on scanned geometry. The 45 degree facing limit, the tracking limits and the rule for finding tracking again are assumptions: no maker's figure was found, and they are to be measured with a real scanner. The arms do not block the scanner's view in the count.

## Scoring

See `scoring` in `task.json`, and `judge` and `Scan` in `task.py`. `check_score.py` runs `judge` on hand-built cases, and the coverage geometry (view, facing, hiding, distances) on hand-built scanner poses.

| Coverage | Score |
|---|---|
| 95 %, every fingertip and the scanner 3 cm or more from the part, held 1 s | 1.0 |
| 70 % at the end, with half of the top | 0.5 |
| 40 % at the end | 0.25 |
| less, or at any time the part knocked off centre or tipped, the scanner touching the part or the turntable, a fingertip touching the part, or the scanner dropped | 0 |

By the scorer's geometry alone (`check_score.py`): a full turn at the low view reaches 77.5 % with none of the top, so a scanner held still there, as on a tripod, gets 0.25. The best single still height, about 40 degrees down at 0.45 m, reaches 88.0 % with 83.6 % of the top: 0.5. A full turn at 60 degrees down covers the whole top. Both turns together cover the whole surface.

A real trial reads the same: the mesh against the CAD in CloudCompare, a point covered when the scan lies within 1 mm of it; the scanning software's log for when scanning started (the software is left in preview, so one press of button 4 starts the scan); and a camera above the station for touches, drops, and the part against its outline drawn on the platter.

## Results

| Run | What it does | Score |
|---|---|---|
| Dream-1 | Run at 320 x 176, 3,000 steps | 0.0: the model moved its hands near the scanner and the turntable but never pressed button 4, so nothing was scanned; it ran to the step limit |
| `sweep` | The whole task, the scanner climbing while the table turns (the film) | 1.0, at step 1,370 (68.5 s): coverage 97.5 % |
| `full` | Two passes, the scanner held still at each: a full turn low, then a full turn steep | 1.0, at step 1,071 (53.6 s). An earlier run of the same views, before the press fix, lost tracking early in the first turn and found it again only near its end, on faces already scanned: 0.25 (61.6 %) |
| `oneturn` | A full turn at the low view, then the sweep up, but no turn at the steep view | 0.5 |
| `still` | The scanner held still at the low view through two turns, as a scanner on a stand | 0.25: none of the top is seen from there |
| `onearm` | No turning: the right hand alone moves the scanner along the part's near side, low and then steep | 0.0: under the 40 % rung (37.4 % covered in a logged run of the same views before the press fix) |
| `strike` | The two passes, but the scanner touches the part's top between them | 0.0 |
| `nobutton` | The two passes without the press | 0.0: nothing is counted |
| `hover` | The sweep, but the left fingertip stops 2 mm short of button 4 | 0.0: no press, nothing is counted |
| `none` | The arms wait | 0.0 |

Two more cases are checked with the scorer's geometry alone in `check_score.py`, with no simulator. A scanner left on the table and aimed at the part, 20 to 38 cm away, reaches at most 73.1 % in a full turn, none of it on the top: 0.25. With no turning, every view from one side of the part, from level to 80 degrees down, together reaches 53.0 %: 0.25.

## Runtime and seeds

The sweep demo takes 68.5 s of simulated time (1,370 steps), about 8.5 minutes of wall time on one shared GPU at 640 x 360 (start-up apart), about 4 minutes unfilmed. There is no randomness: `object_jitter=0`, so no seed is needed.

## Sources

All meshes are generated by `task.py` and `mechkit.py`. There are no third-party assets.

- Housing: SKF SY 505 M (52 mm seat, 36.5 mm centre height); its mass, 0.55 kg, from a distributor's listing of the housing alone (https://www.acorn-ind.co.uk/p/skf/pillow-blocks/sy505m-skf/). Base length and slots as the SY 25 TF unit's data sheet.
- Scanner size, mass, depth of field and field of view: Shining 3D's EinScan H2 specifications page. Its button 4: the EinScan H2 Quick Start Guide, "Scanner button controls".
- Occupation: O*NET 51-9061.00 (https://www.onetonline.org/link/details/51-9061.00). This includes information from O*NET OnLine by the U.S. Department of Labor, Employment and Training Administration (USDOL/ETA), used under the CC BY 4.0 licence; O*NET is a trademark of USDOL/ETA. BLS Occupational Outlook Handbook, Quality Control Inspectors (https://www.bls.gov/ooh/production/quality-control-inspectors.htm).
- EinScan and Shining 3D are trademarks of SHINING 3D Tech Co., Ltd., and SKF of AB SKF, named here for identification only.
- Casting inspection with a handheld scanner: Shining 3D, "300% More Efficient! Metrology Handheld 3D Scanner Accelerates Casting Inspection" (https://www.shining3d.com/metrology-handheld-3d-scanner-accelerates-casting-inspection).

## Licence

MIT
