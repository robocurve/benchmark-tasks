# Draw a 50 mm-radius circle with a school compass round a marked cross, holding the loose sheet still

Instruction: `Draw a circle around the cross with the compass, then set the compass down on the table off the board.`

Drawing a circle with a pair of compasses is one of the first geometry skills taught at school and practised in
homework; the US Common Core high-school standard HSG-CO.D.12 asks every student to "make formal geometric
constructions with a variety of tools and methods (compass and straightedge, ...)". It takes both hands. One hand holds
the loose sheet flat: the lead's drag on the paper otherwise turns the sheet round the needle, so the circle smears and
misses its centre. The other hand plants the needle on the centre mark, presses it in, and moves the compass's grip
round the needle with light, even pressure on the lead. Too much pressure spreads the hinge and the line spirals open;
too little or a lifted lead breaks the line; a needle that skates off the mark moves the centre.

Not the same skill as scribing a shelf to a wall with a pencil compass (there the compass is a fixed-spread spacer
dragged along an irregular wall to copy its profile). Here the needle is the pivot: the hand must plant it on a mark,
keep it pinned while turning the grip round it, and keep the lead loaded for a full, closed turn, while the other hand
stops the sheet turning round the needle.

## Requirements & Assets

- `yam-isaac-lab-arena` and `Isaac Lab-Arena 0.3.1` with `inspect-robots`.
- Third-party assets: none. All geometry is generated from USD primitives in `task.py`.

## Run a Trial

```bash
inspect-robots run --embodiment yam_arena \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --scorer task_score --max-steps 1500 \
  --instruction "Draw a circle around the cross with the compass, then set the compass down on the table off the board."
```

Run limit: `1,500` steps (`75 s` at `20 Hz`). The layout is deterministic with `object_jitter=0`.

## Scene (table frame, cm: x forward from the arm-base midpoint, y left, z up from the table top)

| Object | Real item | Size / material | Pose |
|---|---|---|---|
| Cork board | Any cork drawing/notice board, fixed with tape | 26 x 32 x 0.3 cm cork | centred (30, -4) |
| Sheet | One loose A4 sheet of 80 g/m2 copy paper with a printed cross (12 mm arms) | 21.0 x 29.7 cm, 5 g | centre (30, -4), long side along y; cross at (27, -11) |
| Compass | Precision school compass with a free-turning knurled grip sleeve on its stem (e.g. Staedtler Mars comfort 552 type), set with a ruler to 50 mm between needle point and lead | legs 11.5 cm from hinge to tip, steel/zinc, 44 g (needle leg with head 22 g, pencil leg 10 g, sleeve 12 g); sleeve 9 mm dia x 24 mm | standing legs-down in the cup, legs along x, needle leg toward -x |
| Cup | Desk-tidy pen cup (rectangular) | inside 7.4 x 6.0 cm, 4.0 cm deep, 3 mm walls, plastic | centred (12, -25) |

Both arms start at READY with grippers open.

## How the sim models it (fidelity)

- Rigid bodies at true size; the sheet is a 0.5 mm thick rigid plate (a real sheet is ~0.1 mm) that can slide and turn
  on the cork. Its 5 g weight gives so little friction that the lead's drag turns it unless it is held.
- Compass hinge: a revolute joint that holds the set spread with 10 N m/rad (a tight friction hinge); pressing down
  hard enough spreads it. Grip sleeve: a plain free bearing about the stem (revolute joint, no drive; the legs carry 20 /s angular
  damping as a stand-in for the fingers' light friction on the knurl, so the compass does not swing freely), so the hand moves the
  sleeve round the needle and the compass turns under it.
- Needle: pressing the needle point on the sheet with >= 0.6 N pricks it. The point is then pinned sideways in its
  prick hole on the sheet (a joint locking sideways motion only, free to tilt, turn and lift), and leaves the hole once
  its point is lifted 2 mm above the sheet. The pin is made where the needle actually presses, so a needle set off the cross moves the circle.
- Pencil line: wherever the lead touches the sheet (within 0.6 mm) and presses it with >= 0.15 N while the needle is
  pricked in, the mark is recorded in the sheet's own frame (so it moves with the sheet) and drawn as a line. While the lead
  stays down from one control step to the next (20 Hz), the stroke between the two points is drawn too (as a real lead
  leaves a continuous line); a jump of more than 6 mm is not joined. The line is rendered 2 mm wide so the
  cameras show it; scoring uses the recorded points.
- Grasp: force-limited pad grip, not a weld. It engages only when both finger pads touch the part (PhysX contact on each
  pad) and the jaws are commanded closed; it holds through a compliant joint capped at 50 N pull (2 x mu 0.5 x 50 N
  YAM gripper effort), 0.5 N m twist and 1.5 N m tip, slips above a cap, and releases when the jaws open.

## Scoring

All conditions are geometry of the sheet, the drawn line, the compass and the grippers, checked by `score()`:

- **0** if the sheet turns more than 5 deg or slides more than 5 mm at any time.
- A mark counts only where the lead pressed the sheet with >= 0.15 N while the needle was pricked in it.
- **0.3**: at least 180 deg of arc drawn round the cross (1 deg bins).
- **0.6**: at least 340 deg, every recorded mark point (not only one per degree) 50 +- 2 mm from the cross, and the centre of a circle fitted to the marks
  within 2 mm of the cross.
- **1.0**: also no gap longer than 3 mm (a closed circle), the compass set down on the table off the board (both tips,
  both legs and the sleeve >= 1 cm outside the board's outline, needle out of the sheet, the compass still, <= 1 cm of
  motion, for 1 s), both grippers open, every gripper point >= 10 cm from the compass and >= 5 cm above the sheet if
  over it, and the grippers still (<= 1 cm of motion) for 1 s.

Real trial: the graphite line on the sheet is the record of the marks (0.15 N is the sim's threshold for a lead that
leaves a visible line; a lead that only skims leaves none). A clear template printed with 48 mm and 52 mm rings round the cross, laid over
the sheet, shows that every point of the line is within tolerance; a protractor and a ruler give arc, centre and gaps; a tape mark at the sheet's corner shows turning and
sliding; the board's edge shows where the compass was set down; the video shows whether the needle was in the sheet
while each part of the line was drawn (line drawn with the needle out does not count).

## Shortcut checks (scripted runs, same scene)

| Run | Score |
|---|---|
| Do nothing | 0.0 |
| Right arm alone draws, left hand never holds the sheet (sheet turns/slides) | 0.0 |
| Needle planted, lead held just above the sheet (no line) | 0.0 |
| Only about half a circle drawn, then set down | 0.0 |
| Full circle drawn but the compass is never released | 0.6 |
| Full scripted demo | 1.0 |

## Reset

Print the cross (two crossing 12 x 1.5 mm black bars) on an A4 sheet 30 mm toward the sheet's short edge nearer the
robot from its centre and 70 mm to the robot's right of its centre (sheet-local (-30, -70) mm, long side along y), and lay
the sheet on the board's tape marks (sheet centred on the board, cross at table (27, -11) cm). Check the spread on a
ruler (50 mm between needle point and lead) and stand the compass legs-down on the cup floor, hinge up, legs along x,
needle leg toward -x, centred in the cup at (12, -25) cm.

## Scores

Dream-1 run: 0.0; demo: 1.0
