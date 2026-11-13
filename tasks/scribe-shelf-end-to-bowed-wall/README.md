# Scribe a shelf end to an out-of-flat wall with a pencil compass

Walls are rarely flat, so a shelf, countertop or cabinet side that must sit tight against one is scribed before it is cut. The board is set against the wall where it touches, and a pencil compass, locked open, is run along the wall with its steel point on the wall and its pencil on the board, its legs kept square to the wall. The pencil copies the wall's line onto the board, offset by the compass setting; cut to that line, the board fits with no gap. If the compass twists or comes off the wall, or the board drifts away from the wall, the line is copied wrong and the cut leaves a gap.

Here a pine shelf board lies on the table with its end against a fixed wall section whose face has two humps and a hollow (up to 5.1 mm of gap along the shelf's end). One hand holds the shelf; the other takes the compass off its stand, scribes one unbroken line across the shelf's end from its front edge to its back edge, and puts the compass down off the shelf. The line may have no gap longer than 2 mm and is read at 14 stations to 0.5 mm.

Category: Impact on jobs. Done by carpenters (O*NET 47-2031.00: "Measure and mark cutting lines on materials, using a ruler, pencil, chalk, and marking gauge."), including finish carpenters and cabinet and countertop installers. The demonstration holds the shelf with the right arm and scribes with the left; the scorer does not care which arm does what.

**Layouts by seed.** The wall is the same in every trial; the trial's seed sets where along it the shelf lies, so each trial copies a different stretch of wall. Layout = trial seed mod 4. The shelf's front edge is then 120, 147, 174 or 201 mm forward, and its end touches the wall where the wall stands out most along that stretch. A line copied from one layout scores 0 on each of the other three, and no straight line scores on any of them.

## Run and score

With the official simulator installed, run from this folder with zero object jitter and 20 Hz:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=scribe_shelf_end_to_bowed_wall.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --seed 0 \
  --instruction "Holding the shelf against the wall, scribe its end to the wall: run the compass from the shelf's front edge to its back edge in one unbroken line, point on the wall, pencil on the shelf, then put the compass down off the shelf and wall." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ./demo-run
```

The model run uses the same command with `--policy dream`. The harness reports task_score at the end. Encode recorded cameras with `inspect-robots video <run.json>`. Scores are in task.json.

**Seeds.** The harness passes each trial a seed made from the eval seed and the epoch (CRC-32 of "eval_seed:scene_seed:epoch"); the task uses that trial seed mod 4. With one epoch, `--seed 1` gives layout 0, `--seed 3` layout 1, `--seed 0` layout 2 and `--seed 2` layout 3. With `--seed 0 --epochs 3` the trials get layouts 2, 0 and 2. Each reset prints `SCRIBE_LAYOUT layout=<k>` with the shelf's shift from layout 0.

**CPU checks.** `python -m pytest -q -p no:cacheprovider` in this folder with yam_arena installed runs the scorer on the start, the goal and named partial and cheat layouts, and `test_red_team.py` steps the pencil-path recorder and scorer through whole trials.

Runtime per trial: at most 2,400 control steps (120 s simulated). The scripted demonstration's plan lasts 38.6 s (772 steps); success needs 1 s more after both hands are clear, 33% of the limit. Wall-clock time depends on the GPU.

## Scoring

**Stations.** Fourteen lines square to the shelf's end, at 7, 17, 27, ... 137 mm from its front edge. At each station the reading is the pencil line's distance from the shelf's end, measured along the shelf's length to the centre of the line.

**Repeated marks.** Where the pencil crossed a station more than once (a second pass, or going back over the line), crossings within 1 mm of each other read as one band, at its centre. If two crossings at a station are more than 1 mm apart (two separate lines), the station is unmarked. A single touch of the pencil counts at a station when it is within 0.25 mm of it.

**Targets.** At each station the line belongs at 10.0 mm (the compass setting) minus the gap between the wall and the shelf's end there. The tables below give every station's target for each layout.

**Bend.** For each layout the README names three gate stations: the two outer ones and the middle one where the wall bends most. There is no line credit unless all three are marked and the middle reading lies at least 1.5 mm off the straight line through the outer two readings, on the same side as the target's bend. A ruler line, which ignores the wall, fails this on every layout.

**One unbroken line.** A scribed line guides the cut, so it must be one line across the shelf's end from its front edge to its back edge (0 to 140 mm from the front edge, across the shelf's depth), with no gap longer than 2.0 mm. To find the gap, follow the line from the front edge to the back edge, through the marks read at the stations. Wherever it breaks, measure the unmarked stretch to where it goes on: straight across the board, from the end of one mark to the nearest point of the next. The stretches from the front edge to where the line starts, and from where it ends to the back edge, count too. The longest stretch is the line's gap. Other marks on the board count neither way, except where they cross a station (Repeated marks).

Why 2.0 mm: a scribed line is drawn in one pass, and the cut follows it by eye along its whole length. A good line breaks only where the lead skips over the grain, for a millimetre or so. 2.0 mm is four times the width of an HB line (0.3 to 0.5 mm), four cells of the 0.5 mm grid on which the simulation draws the mark, and a fifth of the 10 mm station spacing. Single touches at the stations, however well placed, leave 7 to 10 mm unmarked and get no line credit.

**Line credit.**

| Line credit | One unbroken line (gap at most 2.0 mm), the bend met, and stations within tolerance of their targets |
|---|---|
| 1.0 | all 14 within 0.5 mm |
| 0.5 | at least 12 within 1.0 mm |
| 0.25 | at least 10 within 2.0 mm |
| 0 | otherwise: a gap longer than 2.0 mm (separate touches or dashes, or a line that stops short of an edge), the bend not met, or fewer stations within tolerance |

**Finished.**
- The compass is put down clear of the shelf and the wall: its bounding box is at least 2 mm from theirs. Hanging it back on its stand does this.
- Every gripper's TCP is at least 30 mm from the compass's and the shelf's bounding boxes.
- The compass is slower than 10 mm/s and 0.5 rad/s, and the shelf is slower than 2 mm/s and 0.05 rad/s.

| Score | End of trial, with no sticky failure |
|---|---|
| 1.0 | line credit 1.0, and finished for the last 1 s |
| 0.75 | line credit 1.0, but not finished: the compass still held, lying on the shelf, or moving, or a hand still on the shelf |
| 0.5, 0.25 | that line credit |
| 0 | otherwise |

**Sticky failures** (score 0 for the rest of the trial):
- the shelf moved more than 30 mm across the table from its start;
- the shelf or the compass more than 20 mm below the table top.

Success is a score of 1.0.

**In simulation.** The scorer records the pencil lead's lowest point every control step, in the shelf's own frame, while it marks: within 0.2 mm above to 2 mm below the shelf's top face, over the board. Consecutive marking steps join into one stroke; a lift, or a step the scorer did not see, starts a new stroke. Station readings are where these strokes cross the station lines. For the gap, each stroke is the straight pieces between its recorded points (a single recorded point is a dot). The gap is the longest step needed to go from the front edge to the back edge from mark to mark, through every stroke that gives a station reading, on the route that keeps the longest step smallest; a step is the straight-line distance on the board between the two marks' nearest points, or from an edge to a mark's nearest point. The graphite dots drawn on the board follow the same path but are only a picture; the scorer never reads them.

**In a real trial.**
1. Before the trial, note the layout. With the shelf in place, either take the targets from the tables below or measure the wall's gap at each station with feeler gauges (0.05 mm steps); the target is then 10.0 mm minus the gap.
2. After the trial, tick the 14 stations on the shelf's end with a combination square.
3. Find the line's gap (One unbroken line, above). Look along the line from the front edge to the back edge under the loupe, and measure with the steel rule the stretch from the front edge to where the line starts, every break (from the end of one mark straight to the nearest point of the next), and the stretch from where the line ends to the back edge. The longest is the gap; with the 0.5 mm graduations, a gap over 2.0 mm is plain. Over 2.0 mm, the line credit is 0 and the stations need not be read.
4. Read each station with a steel rule graduated in 0.5 mm. Butt the rule's zero end against a square block held flat to the shelf's end, and read the centre of the pencil line under a 10x loupe, interpolating to the nearest 0.1 mm. This is good to about ±0.15 mm.
5. A digital caliper with a depth rod can be used instead: rest the caliper on the end face and run the rod to the line.
6. An HB line is about 0.3 to 0.5 mm wide; read its centre. Where the pencil went over the same place twice, read the centre of the darkened band; if two lines at a station are more than 1 mm apart, the station is unmarked.

## Layouts and targets

Layout = trial seed mod 4. All distances in millimetres. The shelf spans 140 mm forward from its front edge and 300 mm right from its end.

| Layout | Front edge, forward | Shelf end, left of midline | Touches the wall at, from the front edge | Gate stations | Target bend |
|---|---|---|---|---|---|
| 0 | 120 | 207.97 | 27.8 | 27, 87, 137 | middle 2.64 nearer the end |
| 1 | 147 | 207.97 | 0.8 | 7, 77, 137 | middle 4.32 nearer the end |
| 2 | 174 | 208.38 | 128.7 | 7, 57, 127 | middle 3.12 nearer the end |
| 3 | 201 | 208.38 | 101.7 | 37, 97, 137 | middle 2.70 farther from the end |

The end starts 0.05 mm from the wall's most proud point along its stretch. Targets are the line's distance from the shelf's end at each station, by station distance from the front edge:

| Station | 7 | 17 | 27 | 37 | 47 | 57 | 67 | 77 | 87 | 97 | 107 | 117 | 127 | 137 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| Layout 0 | 8.44 | 9.50 | 10.00 | 9.62 | 8.56 | 7.33 | 6.32 | 5.63 | 5.19 | 4.94 | 4.86 | 5.00 | 5.39 | 6.03 |
| Layout 1 | 9.82 | 8.92 | 7.69 | 6.59 | 5.81 | 5.30 | 5.00 | 4.87 | 4.94 | 5.24 | 5.81 | 6.63 | 7.63 | 8.64 |
| Layout 2 | 7.30 | 6.42 | 5.85 | 5.48 | 5.30 | 5.31 | 5.54 | 6.03 | 6.77 | 7.73 | 8.76 | 9.61 | 9.99 | 9.81 |
| Layout 3 | 5.57 | 5.34 | 5.28 | 5.44 | 5.85 | 6.52 | 7.43 | 8.46 | 9.39 | 9.94 | 9.92 | 9.42 | 8.80 | 8.44 |

## Physical setup

Positions are on the table top, measured forward from the line through the two arm bases and left (+) or right (-) of the midline between them; the arm bases are 610 mm apart, 2.5 mm above the table top. Millimetres throughout.

1. **Wall section** (fixed). A 25 mm MDF board on edge, 260 long (100 to 360 forward) and 100 tall, its face toward the robot's right routed to the profile below and painted matte white. The face line is 212 left; the profile stands out from it toward the shelf, so the face runs from 208.0 to 213.2 left and the board is 19.8 to 25.0 thick. Its back is flat at 233 left. It is screwed to a 19 mm MDF foot, 70 deep (233 to 303 left), 260 long, clamped to the table.
2. **Shelf** (free). A planed pine 1x6 board (eastern white pine), 19 x 140 x 300, 0.32 kg. It lies flat with its length across the table (along y) and its square-cut end against the wall. Its front edge and end are as in the layout table.
3. **Pencil compass** (free). A wing-nut pencil compass and scriber of the General Tools 843/1 class: brass-plated steel legs, a hardened steel point and a 9 mm square knurled top grip. A 60 mm HB pencil stub sits in its clamp. It is locked so the pencil line is 10.0 mm from the wall side of the point, and it is 168 mm long overall and 26 g (the maker lists 0.05 lb, 23 g, without a pencil). The point is 2 mm above the pencil tip.
4. **Stand** (fixed). A pine base (50 x 50 x 19) and post (20 x 20, 132 tall to the fork) carry a steel fork: two tines 6 wide, 3 thick and 42 long, 8 apart, opening toward the robot's right. On top of each tine, at the fork's centre, two steel steps 2 long and 1.5 high stand 6 apart and make a notch. The fork's centre is 80 forward and 110 left; the base covers 130 to 180 left. The compass hangs by its hinge knuckle (10 across, 14 long) in the notches, legs down between the tines, point leg toward the wall. Its pencil tip is 8.9 above the table.

Nothing else is on the table.

**Wall profile.** The face's offset toward the shelf, at each distance from the wall's front end (100 forward), as cut:

| From front end | 0 | 5 | 10 | 15 | 20 | 25 | 30 | 35 | 40 | 45 | 50 | 55 | 60 | 65 | 70 | 75 | 80 | 85 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| Offset | 0.29 | 0.49 | 0.78 | 1.16 | 1.64 | 2.19 | 2.77 | 3.30 | 3.71 | 3.94 | 3.96 | 3.74 | 3.33 | 2.79 | 2.17 | 1.55 | 0.98 | 0.48 |

| From front end | 90 | 95 | 100 | 105 | 110 | 115 | 120 | 125 | 130 | 135 | 140 | 145 | 150 | 155 | 160 | 165 | 170 | 175 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| Offset | 0.06 | -0.27 | -0.54 | -0.75 | -0.92 | -1.04 | -1.12 | -1.16 | -1.14 | -1.06 | -0.93 | -0.73 | -0.47 | -0.14 | 0.25 | 0.70 | 1.20 | 1.72 |

| From front end | 180 | 185 | 190 | 195 | 200 | 205 | 210 | 215 | 220 | 225 | 230 | 235 | 240 | 245 | 250 | 255 | 260 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| Offset | 2.23 | 2.71 | 3.10 | 3.39 | 3.55 | 3.55 | 3.42 | 3.17 | 2.86 | 2.54 | 2.27 | 2.08 | 1.96 | 1.88 | 1.76 | 1.55 | 1.26 |

The profile is a sum of four smooth bumps: offset(u) = sum of h exp(-((u - c) / w)^2 / 2), with u the distance from the front end. The (c, h, w) are (48, 4.0, 21), (128, -1.2, 28), (202, 3.6, 25) and (252, 1.2, 14). Its steepest slope is 0.126 (7.2 degrees) and its tightest radius is 108 mm. In simulation the face is an exact triangle mesh through points every 2 mm along this curve.

## Colliders

Every part is a rigid union of boxes, cylinders and spheres, except the wall's face (a triangle mesh). Millimetres; masses as listed.

**Compass**, frame at the hinge knuckle's centre: x across the compass's thickness, +y toward the point leg (toward the wall), z up along the legs. 26 g in all. Friction (estimate) 0.25 static, 0.20 dynamic, except the knurled top grip (collider 0): 1.0 static, 0.9 dynamic against the finger pads.

| # | Part | Shape | Centre (x, y, z) | Size |
|---|---|---|---|---|
| 0 | knurled top grip | box | (0, 0, 22.7) | 9 x 9, 33.4 long (z 6.0 to 39.4) |
| 1 | hinge knuckle | cylinder, axis x | (0, 0, 0) | 10 across, 14 long |
| 2 | point leg | box | (0, 0.5, -52.5) | 3 x 2 x 95 (z -100 to -5) |
| 3 | pencil leg | box | (0, -1.5, -44) | 3 x 2 x 78 (z -83 to -5) |
| 4 | wing nut on the locking arc | box | (5.5, 0, -40) | 8 x 6 x 6 |
| 5 | steel pin | cylinder, axis z | (0, 2.0, -111) | 2.5 across, 22 long |
| 6 | pin taper | cylinder, axis z | (0, 3.6, -124) | 1.6 across, 4 long |
| 7 | point | sphere | (0, 4.4, -126.0) | 1.2 across |
| 8 | pencil clamp ring | cylinder, axis z | (0, -5, -89) | 10 across, 12 long |
| 9 | pencil | cylinder, axis z | (0, -5, -101) | 7.5 across, 39 long |
| 10 | sharpened wood | cylinder, axis z | (0, -5, -122.5) | 5 across, 4 long |
| 11 | lead cone | cylinder, axis z | (0, -5, -126.25) | 2.4 across, 3.5 long |
| 12 | lead tip | sphere | (0, -5, -128.0) | 1.2 across (lowest point z -128.6) |

The setting is the distance from the point's wall side (y 5.0) to the lead's centre (y -5.0): 10.0 mm.

Above the point, the steel pin and the point leg are stepped back from the point's wall side. Held upright, the point is the only part of the compass that can touch the wall. Tilted in its own plane, the compass can lean about 2.5 degrees toward the wall before the point leg reaches the top of the wall: the leg is stepped back 3.5 mm, 80 mm above the point.

The hand grips the top grip with its TCP 21 mm above the knuckle's centre; the pencil tip is then 149.6 mm below the TCP.

**Shelf**: one box 140 x 300 x 19, 0.32 kg, friction (estimate) 0.50 static, 0.40 dynamic.

**Wall** (fixed):
- the face: a triangle mesh, the profile extruded 100 tall, closed by the flat back at 233 left;
- the foot: a box 260 x 70 x 19;
- friction (estimate) 0.50 static, 0.40 dynamic.

**Stand** (fixed), frame on the table under the fork's centre, friction 0.60 static, 0.50 dynamic:

| Part | Box centre (x, y, z) | Size |
|---|---|---|
| base | (0, 45, 9.5) | 50 x 50 x 19 |
| post | (0, 30, 75.5) | 20 x 20 x 113 |
| tine | (+7, 0, 130.5) | 6 x 42 x 3 |
| tine | (-7, 0, 130.5) | 6 x 42 x 3 |
| step (4) | (+-7, +-4, 132.75) | 6 x 2 x 1.5 |

All contact offsets are 1 mm.

## Demonstration and runtime

**What the demonstration knows.** Every target is worked out at reset from the layout the seed chose (the shelf's position and the wall table), with one exception: after the stroke it reads the compass's pose once, to see how far the stroke turned the compass in the hand, and aims the knuckle (not the hand) at the fork for the rest of the way. It reads no contact or score.

**Left arm: the compass.**
1. Above the compass on its stand with the fingers open (1.6 s), it descends (1 s) and closes across the 9 mm square top grip, its flats toward the pads (0.8 s).
2. It lifts the knuckle 10 mm off the tines, draws the compass 35 mm out of the open side of the fork, rises, and moves sideways 28 mm in x so it clears the stand.
3. It carries the compass to the shelf's front corner (1.8 s), with the pencil tip 8 mm above the board and the point 2 mm short of the wall.
4. It lowers the pencil (1 s) to a target 1 mm into the board's top and moves the point onto the wall (0.8 s), to a target 4 mm into it.
5. After 0.4 s it scribes from 4 mm from the front edge to 3 mm past the back edge at 10 mm/s. The lead lands and ends 2.3 to 3.8 mm behind the hand (the arm's steady error, larger at full reach, and the compass leaning under a degree in the hand), so the line runs in one stroke from 0.4 to 1.3 mm in from the front edge to 0.5 to 1.1 mm short of the back edge (GPU runs on all four layouts), inside the 2 mm allowed at each edge. Every target puts the point 4 mm into the wall at that x, from the wall table, and the compass stays upright and square to the wall. This takes 14 s.
6. It dwells 0.6 s, lifts the pencil 4 mm, backs the point 7 mm off the wall while moving 10 mm back along it, lifts, carries the compass back through the same aisle and hangs its knuckle in the notches again, correcting its path by the compass's measured offset in the hand.
7. It opens the hand and slides the open pads 48 mm off the grip, along the knuckle's axis, to 38 mm from the compass's bounding box.

**Right arm: holding the shelf.**
1. While the left arm carries the compass in, the right arm waits clear of it, 180 forward, 220 right and 180 above the table.
2. It then comes over the shelf, 40 mm behind its front edge and 65 mm right of the midline.
3. It presses the closed fingertips down on the board, to a target 3 mm into its top face, and holds there through the stroke.
4. 0.3 s after the point leaves the wall, it lifts and parks 60 mm above the board.

Both arms then hold still for 3.5 s.

**Reach and clearances.** Checked offline for all four layouts with the arm's inverse kinematics:
- Every TCP target is within 0.358 m of its arm base in plan.
- Inverse-kinematics errors are below 0.1 mm and 0.02 degrees.
- Joints 4 and 6 stay at least 6 degrees inside their limits.
- With the links as 40 mm capsules, the two arms stay at least 25 mm apart, and the links stay at least 22 mm from the wall and at least 78 mm from the stand and the shelf.
- During the stroke, the hand is more than 50 mm above the wall's top.

## Fidelity and approximations

- **Pencil line.** The mark is geometric: the lead marks wherever its lowest point is within 0.2 mm above to 2 mm below the board's top face. There is no force threshold and no wear of the lead. The drawn graphite dots (0.7 mm across, every 0.5 mm) cover only the strip 0.5 to 17 mm from the board's end, where every target lies. The scorer uses the recorded path, not the dots. In the 320 x 176 videos the line is faint, as a real pencil line would be at that resolution: it shows as a thin dark curve beside the wall in the left wrist camera once the compass leaves the board (about 28 s into the demonstration), and not at all in the top camera.
- **Wall.** A wall section stands in for a real out-of-flat wall: a CNC-routed MDF board, its profile listed above, with 3 to 5 mm of departure over 140 mm, typical of plaster. The face is a rigid triangle mesh with 2 mm facets, so it has no paint or dents. In the offline checks only, 26 boxes, each 10 mm long and set behind the face, stand in for it.
- **Compass.** One rigid body locked at its setting; the wing nut and hinge do not move. The sub-dimensions of the legs, pin and grip are our estimates for this class of compass: the maker lists only the overall size (0.75 x 3.00 x 6.75 in) and weight (0.05 lb). The top grip is square in section, with flats for the fingers: a round rigid grip held between the rigid simulated pads crept down and turned in the pinch on the GPU (about 10 mm and 15 degrees in 7 s), which real pads that conform to a knurled grip would not allow. The point does not dig into the paint.
- **Fixed parts.** The stand and the wall's foot are fixed to the table.
- **Friction.** All friction values are estimates.
- **Layouts.** The shelf is placed by writing its pose right after the simulator's reset, using the same call the simulator uses for seeded object jitter. No fixed body moves.

## Dimension sources

- General Tools 843/1 pencil compass and scriber (brass-plated steel, positive wing-nut adjustment, full-length hardened steel pin, up to 8 in circles, 0.75 x 3.00 x 6.75 in, 0.05 lb, pencil not included): https://generaltools.com/products/pencil-compass-and-scriber-adjustable-up-to-8-inches
- Nominal 1x6 board, actual 3/4 x 5-1/2 in (19 x 140 mm): American Softwood Lumber Standard, PS 20.
- Eastern white pine, specific gravity 0.35 at 12% moisture (about 390 to 400 kg/m^3): USDA Forest Products Laboratory, Wood Handbook (FPL-GTR-190, 2010), chapter 5, table 5-3: https://www.fpl.fs.usda.gov/documnts/fplgtr/fplgtr190/chapter_05.pdf

## Files and licence

All the code in this folder is the task authors' own and contains no third-party code or assets; all geometry is built from primitives and a procedural mesh.
- `scribe_shelf_end_to_bowed_wall.py`: the scene, scorer and demonstration.
- `scribe_runtime.py`: the per-seed layout placement, the graphite dots and the wall's paint.
- `test_scribe_shelf_end_to_bowed_wall.py`: the scorer on the start, the goal and named partial and cheat layouts.
- `test_red_team.py`: the scorer step-through tests.

All of these were written for this task. `yam_parts.py`, `yam_joints.py` and `yam_mesh.py` are copied kit modules (compound parts, jointed mechanisms and mesh colliders), copied unchanged; they are the task authors' own code too.
