# Scribe a floor board to a door casing with a contour gauge

A door casing is nailed to a wall section beside a door opening and runs down to the floor. A contour gauge lies flat
on the floor in front of it. The robot pushes the gauge flat against the casing, where it meets the wall, until the
frame seats on the casing. It draws the gauge straight back, with its fingers holding the copied outline, and lays it
flat on a floor board in the marking position. In that position the copied wall line sits on the board's pencilled
wall line, and the casing's copied inner edge meets the board's side edge. The outline is then ready to trace as the
cut that fits the board's end around the casing.

**Instruction:** "Push the contour gauge flat against the door casing where it meets the wall to copy its profile, then
lay the gauge flat on the floor board with the copied wall line on the board's pencilled wall line and the casing's
inner edge at the board's side edge, ready to trace the cut, and let go."

## Running it

From this folder, on a machine with the official simulator (`yam-isaac-lab-arena`):

```bash
INSTRUCTION="$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')"

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/contour_gauge_casing_profile.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/contour-gauge-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/contour_gauge_casing_profile.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/contour-gauge-demo
```

Seeds: seed 0, object_jitter=0. The layout is fixed, and the scripted demo uses constants only.

## Why it matters

Fitting flooring, or a board end, around a door casing (scribing) is a routine step for carpenters (SOC 47-2031,
670,090 US workers; BLS OEWS https://www.bls.gov/oes/current/oes472031.htm). The O*NET core tasks
(https://www.onetonline.org/link/summary/47-2031.00) include "Measure and mark cutting lines on materials, using a
ruler, pencil, chalk, and marking gauge." Where a floor board ends at a wall that carries a casing, its end has to be
cut around the casing's moulded leg.

The carpenter does it in three steps:
1. Lay a contour gauge flat on the floor and push it against the casing and the wall. One push copies the whole
   moulding.
2. Lay the gauge on the board with the copied wall line on the board's wall line.
3. Trace along the finger tips. The traced line is the cut.

The gauge's fingers are held only by friction. It must be pushed square, drawn straight back and carried flat without
a finger moving, or the cut will not fit.

## Scene (real counterparts, full size)

| part | real | model | status |
|---|---|---|---|
| contour gauge | 10 in plastic-finger gauge, General Tools 833 class: 254 mm wide, 76 mm fingers held by friction, 32 mm usable depth (maker brochure mirror https://www.autotoolworld.com/assets/General_Tools/General-Tools-Instruments-833-Contour-Gauge-10-Inch-Brochure.pdf); USD 21.95 (https://www.penntoolco.com/general-10-contour-gage-can-duplicate-any-shape-833-1/) | frame 254 mm long, 22 mm thick, 10 mm deep; 31 finger slide joints at 8 mm pitch, 76 mm long, rounded tips, 32 mm in front of the frame when fully out; lies flat | sizes sourced; pitch, finger width and mass estimated |
| grip handles | two L-shaped hardwood handles from offcuts, one taped to each end of the frame with double-sided carpet tape: a 38 mm bridge carrying the frame on past its last finger, then a 58 x 25 x 54 mm block sticking back from it (offcuts, no purchase) | the same shapes on the frame's ends; 140 g gauge in all | scene choice: the hands hold the gauge behind its frame, clear of the fingers and the wall |
| finger hold | fingers stay where they were pushed, held by friction in the frame | per finger: a drive (200 N/m) held at the latch point, which slips past 0.5 N (the on_step hook moves the latch) | 0.5 N = engineering estimate; checked on the real gauge before trials (below) |
| casing | colonial-pattern pine casing, 57 x 17.5 mm (2-1/4 x 11/16 in, the common stock size), made to the profile below so the copy can be checked against known numbers (fabrication below) | fixed; profile across its width from the inner (bead) edge: raised bead 12 mm proud x 10 mm, field 6 mm x 18 mm, a bevel rising to 12 mm over 12 mm, a 5.5 mm step to the 17.5 mm outer band, 17 mm wide; leg cropped 150 mm above the floor | profile authored; every step >= 5.5 mm, every feature >= 10 mm wide (sized to the 8 mm finger pitch) |
| wall and opening | 1/2 in drywall on a 2x4, cropped to a 210 x 150 mm section; the door jamb's edge flush with the wall, its face 6 mm beyond the casing's inner edge (the usual reveal); the opening beyond it | fixed box, its room face at x = 0.40, from the jamb's face (6 mm beyond the casing's inner edge) to 170 mm along the band side; nothing on the opening side | scene choice |
| floor board | 1x6 pine board, 19 x 140 mm actual, 300 mm offcut, pencilled wall line across it 30 mm from its far end | fixed, its side edge in line with the casing's inner edge (the opening side), so the casing's leg falls on its first 57 mm | scene choice |

**Approach constraints.** The casing can only be reached from the room side, at floor level. The wall stands behind
it and on its outer-band side, and the opening is on its bead side. The gauge has to be pushed in flat, along the
floor, squarely at the wall. Each hand holds one grip handle, 75 mm behind the finger tips, so the hands stay clear
of the fingers and of the wall.

**Board-to-casing alignment.** The board is the last board of a floor that stops at the door opening. Its end butts
the wall, and its side edge lines up with the casing's inner edge (the threshold covers the opening). The pencilled
wall line marks where the wall's face will sit, so the casing's leg notches the board's end from the side edge for 57
mm, by up to 17.5 mm. To be ready to trace that notch, the gauge has to lie flat on the board in three ways:
- its fingers point at the board's far end;
- the tips of the fingers that copied the wall lie on the pencilled line;
- the copied inner edge of the casing sits at the board's side edge.

## Casing fabrication (repeatable)

The profile is made of flat steps and one bevel, so it can be cut on a table saw from a 1x3 pine board:
1. Rip the board to 57 mm wide and plane it to 17.5 mm thick.
2. Cut a rabbet 40 mm wide and 5.5 mm deep along the inner edge. This leaves the 17 mm outer band at 17.5 mm and the
   rest at 12 mm.
3. Cut a groove 18 mm wide and 6 mm deep, starting 10 mm from the inner edge. This makes the field and leaves the
   bead.
4. Rip the bevel at 26.6 deg (6 mm rise over 12 mm), from the field's far end to the outer band.

Check it with calipers at the middle of each feature:
- bead 12.0 mm
- field 6.0 mm
- bevel 9.0 mm at its middle
- outer band 17.5 mm

The tolerance is 0.5 mm. Cut a 300 mm leg and nail it to the wall section with the 6 mm reveal.

## Checking the real gauge before trials (arm compatibility)

In the model a full push takes about 15 N along the gauge (31 fingers x 0.5 N). Before trials:
1. Push the real gauge against the casing by its two handles, with a luggage scale behind one handle.
2. Read the peak force. If it needs more than 20 N, work the fingers in and out a few times until it does not.
3. Check that a finger pushed in by hand stays where it was left (the copy has to survive the carry).
4. Push every finger fully out before each trial.

## Scoring (identical to the code's `score()`)

Judged at the end of the trial. 0 unless the copied outline matches the casing laid back against it at some offset
and face. Candidates are screened against every gate, then the lowest RMS is taken:
- at least 5 fingers at least 1 mm onto the casing;
- those fingers within RMS 1.5 mm and worst 3.0 mm of its heights;
- the wall fingers within 30 mm of its outer edge within a peak-to-peak spread of 2.5 mm about one straight line;
- that line (how square to the wall the frame was pushed) within 3 deg.

Then 0.4. The score is 1.0 if, in addition:
- the gauge lies flat (within 3 deg) on the board, its bottom within 3 mm of the board's top;
- its fingers point at the board's far end (within 5 deg);
- the copied wall line is on the pencilled wall line (wall fingers' tips within 2 mm on average and 3 mm each);
- the copied inner edge of the casing is within 3 mm of the board's side edge, with the outer band over the board;
- each hand's tool point is more than 20 cm from the gauge centre in plan, or more than 15 cm above the board.

Possible scores: 0, 0.4, 1.0.

Every feature of the profile is at least 5.5 mm high and 10 mm wide. So the gates tell a real copy from:
- an unpushed gauge;
- a push against the bare wall;
- a skewed push;
- a flat block of the casing's width;
- a plain bevel of the same rise.

The opening side of the casing has no wall, so only the wall on its outer-band side is the copy's reference.

### Scoring a real trial (video plus end measurements)

- **Copy.** Trace the copied outline onto paper. Lay a printed full-size template of the profile (the table above)
  over it, with its wall line on the traced wall line. At 7 points across the casing, 8 mm apart, read the gap with a
  steel rule or feeler gauge: RMS within 1.5 mm, worst within 3 mm. The traced wall line over the 30 mm beside the
  outer band must be straight within 2.5 mm and within 3 deg of the template's wall line. The real gauge's 1.5 mm
  fingers trace a finer outline than the model's 8 mm grain; the check reads it at the model's 8 mm spacing.
- **Marking position.** With the gauge where the robot left it:
  - measure from the wall-line finger tips to the pencilled line at both ends of the 30 mm reference stretch, and in
    its middle, with a steel rule (mean within 2 mm, each within 3 mm);
  - measure from the copied inner edge (the first bead finger) to the board's side edge (within 3 mm);
  - slide a 3 mm feeler under each end of the frame (it must not pass: flat on the board);
  - check that the fingers point at the board's end.
- **Hands.** From the video: both hands let go and end at least 20 cm from the gauge's centre (tape marks on the
  bench), or 15 cm above the board.

## Scripted demo (open loop, constants only)

1. Each hand takes one grip handle, and they lift the gauge 3 mm off the floor, level.
2. They slide it up to the casing and push it straight at the wall, commanded 15 mm past where the frame would seat on
   the outer band (it stops about 11 mm short of seating as the grip yields; the wall fingers are then pushed about
   3 mm). The fingers in front of the casing are pushed back to its profile, those in front of the wall to the wall,
   and those in the opening stay out.
3. They draw it straight back 60 mm, lift it and carry it over the board.
4. They lay it flat on the board in the marking position, let go and park.

## Disclosed estimates and grain

- **Finger hold.** 0.5 N per finger is an estimate; the drive's elastic give at that force is 2.5 mm.
- **Finger pitch.** The model has 31 fingers at 8 mm pitch, while a real gauge has about 1.5 mm fingers, so the model
  copies the profile more coarsely (one model finger stands for about five real ones). Wherever the casing lies fully
  in front of the gauge, 7 or 8 fingers read it.
- **Other estimates:** finger mass 1.6 g; the profile's steps; the grip handles' size.
- **The gauge lies flat throughout, so gravity acts across the fingers, never along them.** A push driven by one hand
  swings the frame out of square with the wall, and the copy fails the 3 deg gate.

## Offline check (no GPU)

`python -m pytest -q test_contour_gauge_casing_profile.py`, with the benchmark runtime (`yam_arena`) importable, covers:
- the latch;
- the casing boxes against the profile;
- the start (0) and the goal (1.0);
- each partial tier and shortcut: a skewed push, an off-centre push, the bare wall, a ramp and a flat block;
- the marking-position tolerances;
- the demo's push, draw-back and final hand clearance.

## Licence and provenance

`labkit.py`, `arena_assets_fix.py` and `pitch_demo.py` are the benchmark kit's helper modules, copied unchanged next
to the task file (one word of a docstring in `arena_assets_fix.py` reworded). The task code and tests are original to this task. No third-party mesh or texture is used. The
repository licence applies.

## Claimed revision

This task builds on pull request #2416, whose revision hold lapsed without a revision. It answers that review
(TK-278Z5R).

| Reviewer point | What changed |
| --- | --- |
| The full-score gauge stands upright with its copied tips above the board | The gauge now lies flat on the board, its finger tips on the board (0.5 mm clear). Full credit needs it flat within 3 deg, its bottom within 3 mm of the board's top, and its fingers pointing at the board's far end. |
| Full credit checks the gauge's centre and upright pose, not whether the copied contour is placed to produce the cut | The scorer now reads the copy itself. The wall-line finger tips must lie on the pencilled wall line (mean 2 mm, each 3 mm), and the copied inner edge of the casing must be at the board's side edge (3 mm), with the outer band over the board. Mirrored, turned or offset placements score 0.4 (tests). |
| Specify the intended board-to-casing alignment | Stated in Scene: the board's end butts the wall, and its side edge lines up with the casing's inner edge. The pencilled line is the wall line. |
| Represent the casing in its surroundings, with constraints on approaching it | The casing is no longer a loose offcut. It is nailed to a wall section beside a door opening and stands on the floor, so the gauge must be pushed flat along the floor, squarely at the wall, from the room side. The opening side has no wall, so the copy's reference is the wall beside the outer band only. |
| No matching standard product or repeatable fabrication for the authored profile | A table-saw fabrication from a 1x3 pine board, with caliper check points and 0.5 mm tolerance, is in Casing fabrication. |
| Explain how to check the profile tolerances with the finer-pitched real gauge | Scoring a real trial: trace the outline, overlay a printed template, read the gaps at the model's 8 mm spacing. |
| Flag: the real gauge's finger friction and push force must suit the arms | Checking the real gauge before trials: measure the push force with a luggage scale (at most 20 N; the model needs about 15 N), and check that a pushed finger stays put. |

Also changed: hardwood grip handles on the frame's ends (the hands hold the gauge there, clear of the fingers and the
wall); the hands-clear distance is 20 cm, since the handles end 16.5 cm from the gauge centre; and the casing-shove cap
is gone, since the casing is now fixed. The revised scene was run in Isaac Sim (seed 0): the scripted demo scores
1.0 (copy RMS 0.56 mm, worst 1.4 mm, 1.6 deg square; wall line 1.4 mm from the pencilled line, inner edge 1.9 mm from
the board's edge); Dream-1 scores 0 (it does not move the gauge in 1,200 steps).

## Origin

This task builds on pull request #2416 in `robocurve/benchmark-tasks`. The code is MIT licensed; see `LICENSE`.
