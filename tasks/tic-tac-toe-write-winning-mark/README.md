# Write the winning mark in a tic-tac-toe game

Writing a specific symbol in a specific place is everyday work on forms, checklists, labels and scoresheets; here the robot must infer whose turn it is from the visible counts and find the one cell that wins. Unlike board games that pick and place pieces, such as RoboDojo's tic-tac-toe task against an opponent, this move is written with a hand-held spring-nib marker and its glyph shape is scored; unlike pen tasks that mark an instructed box or rule a straight line, the instruction specifies neither symbol nor cell. Done by: form-filling clerks and inspectors; assistive use includes playing a board game with a child or an older adult.

The right arm can do this one-handed job; the left stays ready. All assets are locally authored primitives and the included parts/joints kits; no third-party visual assets.

## Run and score

With the official simulator installed, run from this folder. Use zero object jitter, 20 Hz control and `--seed 0`, which reproduces the board in the setup screenshot and both videos: board 6, `.OX/XO./X..`, O to play, winning cell row 2, column 1. The board index is the seed the embodiment receives on reset, modulo 16. inspect-robots does not pass `--seed` through unchanged: the reset seed is the CRC-32 of the text `<--seed>:<scene seed>:<epoch>` (its `derive_seed`; scene seed `None` and epoch 0 for this single-trial command), so `--seed 0` gives board 6, `--seed 1` board 8, `--seed 2` board 11 and `--seed 10` board 0.

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tic_tac_toe_write_winning_mark.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --seed 0 \
  --instruction "It's your turn in this tic-tac-toe game. Make the winning move with the pen, then hang the pen back on its stand." \
  --scorer task_score --max-steps 1800 --store-frames --log-dir ./demo-run
```

For Dream-1, use `--policy dream` with the same task, instruction, seed and limits. The run prints `task_score`; success needs full score held for one additional second. To encode the recorded cameras, run `inspect-robots video <run.json>`. Run offline scorer checks with `python -m pytest -q -p no:cacheprovider` in this folder in an environment with `yam_arena` installed. Scores are recorded in task.json.

## Scoring

| Score | End state |
|---|---|
| 0 | Untouched sheet, wrong/unclear glyph, or permanent violation |
| 0.6 | Correct glyph in the unique winning zone |
| 0.8 | Correct glyph and pen hung; may still be held or moving |
| 1.0 | Correct glyph and pen hung, released and still for 1 s; success holds another 1 s |

Infer the side to play from counts (X first; equal counts X, otherwise O), then its unique winning cell. The winning zone is that 45 mm square expanded 2.5 mm on every side. Each separate nib touch (contact from touch-down to lift) counts its centreline length but at least 4.4 mm, one ink-disc diameter: a shorter touch is scaled up evenly along its path and shared pro rata between inside and outside the zone, and a stationary dab counts 4.4 mm at its point. More than 1.0 mm of counted centreline anywhere on the sheet outside that zone is a sticky zero for the rest of the episode, including after restoration; this is path travelled, so retracing outside the zone counts twice, and any separate dot, dab or line wholly outside the zone is a violation. Writing requires nib travel >=1 mm, tip height within 2 mm of paper top, and tip over the sheet; nib contact off the sheet is neither inked nor penalised. Only poses and joint values determine paths; printed/ink prims are never read. The glyph is read only from the shape of the ink centreline (the middle of the 4.4 mm wide ink line) inside the winning zone, as a top photo shows it: stroke count, order, direction, pen lifts and retracing do not change the shape tests. Up to 5 mm of counted centreline in total may lie off the glyph: one touch-down dab (4.4 mm) or one tick up to 5 mm long passes, two dabs fail, and a retraced tick counts twice. X: two straight lines, one within 25 degrees of 45 degrees and the other within 25 degrees of 135 degrees to the forward (x) axis, hold all but those 5 mm of counted centreline within 3 mm of one of them; the lines cross within 8 mm of the cell centre; along each line the ink within 3 mm of it is continuous (no gap over 4.4 mm, one ink-line width), at least 22 mm long, and continues at least 8 mm past the crossing on both sides, so a V, chevron or lambda fails. O: a least-squares circle is fitted to the centreline samples (every 0.5 mm along each touch, one sample per dab), then refitted to the samples within 18% of its radius; that refitted circle must have its centre within 8 mm of the cell centre and a radius of at least 10 mm, all but those 5 mm of counted centreline must lie within 18% of its radius of it, and the samples in that band must leave no angular gap over 30 degrees around its centre. A square's corners lie 22% out and a triangle's 41%, so they fail; a regular pentagon's lie 14% out and a hexagon's 10%, so those and anything rounder pass as a hand-drawn O. The winning zone caps the size. Both or neither glyph test means unclear. A correct, unambiguous mark earns 0.6. No mark, wrong symbol, unclear/scribbled, mixed or too-small marks earn zero. 0.8 instead when that mark remains and the pen hangs with collar underside within 2 mm vertically of fork top, axis within 5 degrees of vertical, and barrel axis across the slot within 3.5 mm of stand x (3 mm half-clearance plus 0.5 mm tolerance). Along the slot, the barrel axis may sit from -13 mm relative to the stand centre (barrel touching the rear plate face at -20 mm) out to where the circular collar still overlaps both prong tips by at least 3 mm: 25 + sqrt(15^2 - (10 + abs(x offset))^2) - 3 mm, with x offset in mm (prong tips at 25 mm, collar radius 15 mm, inner prong edges at +/-10 mm), which is 33.180 mm at x offset zero and 28.538 mm at +/-3.5 mm. Beyond 25 mm the barrel is past the prong tips and only the collar rests on them, so a released pen there tips off; only a held pen earns 0.8 in that part of the range. These limits are inclusive. 1.0 instead when also released (every gripper TCP at least 30 mm from the whole pen bounding box) and still (base translation <10 mm/s, base spin and all joint speeds <0.05 rad/s or m/s as appropriate), continuously for 1 second. A held or swinging pen earns 0.8 when hung. Success holds the full score for another 1 second. The untouched start scores zero; partial credit reflects the current end state.

A top photo and a ruler are enough to score a real trial: the winning square and the glyph shape tests use only where the ink line lies, never how many strokes drew it, in what order or how fast. The two length allowances are the exception: they count centreline path travelled, so a retraced stretch counts twice, and every separate touch, a dab included, counts at least 4.4 mm, the width of its ink dot. In a photo, any ink dot or line wholly outside the zone is therefore a zero; the 1.0 mm limit only matters for a line that ends just past the zone edge, and the 5 mm allowance admits one stray dot or short tick beside the glyph. Two photos one second apart check that the marker stays hung and that both grippers are clear. The simulation records the nib centreline from the pen's pose; the ink drawn on screen is a 2.2 mm-radius disc around it, so its middle is the same centreline a reviewer measures in the photo.

## Setup and reproduction

World coordinates in metres: x forward from the arm bases, y left, z up; bases (0,+/-0.305,0.750), table top z=0.7475. All origins have zero yaw and identity orientation. A white cellulose paper sheet, 165 x 165 x 1 mm, is taped flat with its lower-centre origin at (0.195,-0.155,0.7475); fixed in simulation with no simulated mass, including its tape restraint. Its top is 0.7485. The charcoal printed 135 mm square tic-tac-toe grid has 45 mm cells and 2.5 mm wide lines; row 0 is far (+x), column 0 left (+y). Printed dark-blue pigment Xs span a 30 mm square using two 42.426 mm diagonal bars 4 mm wide; printed Os have mean radius 15 mm and width 4 mm. Printing has negligible mass and no collision, with centres 0.1 mm above the sheet. Black written ink is a fixed visual grid of 3969 discs, pitch 2.5 mm, radius 2.2 mm, thickness 0.1 mm, centres 0.15 mm above the sheet, covering board plus 10 mm margin. No other props. The red ABS felt-tip marker is free, 135 mm long, base mass 20 g plus 3 g nib. It starts vertical with origin at its top (0.155,-0.310,0.8955), collar underside 1 mm above the fork, nib 13 mm above table. In its z-up local frame, its dark grip is a box centred (0,0,-0.0175), size (0.014,0.014,0.035). Red collar cylinder centre z=-0.037, radius 0.015, height 0.004; barrel centre z=-0.074, radius 0.007, height 0.070; stepped nose cylinders centred z=-0.112,-0.118,-0.124, respective radii 0.006,0.005,0.004 and heights 0.006 each, all x=y=0, axes z. The black felt nib has a cylinder at z=-0.1265, radius 0.0025, length 0.011 and a ball at z=-0.132, radius 0.003; its free tip is z=-0.135. Passive z slide anchored (0,0,-0.121), travel 0 to +0.010, rest -0.002, spring 120 N/m, damping 1 N s/m, no joint friction: positive motion retracts upward. Pen friction 1.2 static/1.0 dynamic, nib 0.15/0.10. The blue-grey steel fork stand is bolted at (0.155,-0.310,0.7475), fixed with no simulated mass. Local box post centre (0,-0.034,0.052), size (0.038,0.008,0.104); rear plate centre (0,-0.029,0.106), size (0.038,0.018,0.004); two fork tines centred (+/-0.013,0.0025,0.106), size (0.006,0.045,0.004). Fork top z=0.8555, slot width 20 mm opens +y; barrel clears each side by 3 mm. Stand/sheet friction 1.2/1.0. Contact offsets: sheet 0.2 mm, pen/stand 0.3 mm. The tape and printing are integrated into the sheet. The setup screenshot and both videos show board 6 (.OX/XO./X.., O to play at row 2, column 1), reproduced with --seed 0. The reset seed modulo 16 selects the following nine-character boards; the reset seed is the CRC-32 of '<--seed>:None:0' for a single trial, so --seed 0, 1, 2 and 10 give boards 6, 8, 11 and 0. Boards (far row first, left column first, dot empty); None selects 0: 0: ..X.OX.XO; 1: ...XOXOX.; 2: X.OX..OX.; 3: OX.XOX...; 4: ..X.OO.XX; 5: OXXO....X; 6: .OXXO.X..; 7: XX...XO.O; 8: X.XO..OXO; 9: ..OXX..O.; 10: .XOOX....; 11: .OX..XO..; 12: X.XXO.O.O; 13: .O...X.OX; 14: X.O.OXXO.; 15: ....OOXX. (end of list)

## Seeded games

X moves first and turns alternate. Equal counts mean X to play; one extra X means O to play. Every unfinished board has one winning cell. Eight games play each symbol; four of each also have an opponent threat elsewhere. Every cell appears as a winning cell; every row, column and diagonal wins twice.

| Board index (reset seed mod 16) | Board (far / middle / near) | Play | Win (row,col) | Line | Opponent threat |
|---|---|---|---|---|---|
| 0 | `..X/.OX/.XO` | O | (0, 0) | diag_back | none |
| 1 | `.../XOX/OX.` | O | (0, 2) | diag_forward | none |
| 2 | `X.O/X../OX.` | O | (1, 1) | diag_forward | none |
| 3 | `OX./XOX/...` | O | (2, 2) | diag_back | none |
| 4 | `..X/.OO/.XX` | O | (1, 0) | row1 | [(2, 0)] |
| 5 | `OXX/O../..X` | O | (2, 0) | col0 | [(1, 2)] |
| 6 | `.OX/XO./X..` | O | (2, 1) | col1 | [(0, 0)] |
| 7 | `XX./..X/O.O` | O | (2, 1) | row2 | [(0, 2)] |
| 8 | `X.X/O../OXO` | X | (0, 1) | row0 | none |
| 9 | `..O/XX./.O.` | X | (1, 2) | row1 | none |
| 10 | `.XO/OX./...` | X | (2, 1) | col1 | none |
| 11 | `.OX/..X/O..` | X | (2, 2) | col2 | none |
| 12 | `X.X/XO./O.O` | X | (0, 1) | row0 | [(2, 1)] |
| 13 | `.O./..X/.OX` | X | (0, 2) | col2 | [(1, 1)] |
| 14 | `X.O/.OX/XO.` | X | (1, 0) | col0 | [(0, 1)] |
| 15 | `.../.OO/XX.` | X | (2, 2) | row2 | [(1, 0)] |

## Scripted demo and runtime

The policy grasps the marker by friction, lifts the collar from the fork, withdraws through the open slot, travels to the selected cell and settles 0.8 s before the first lower (at least 0.3 s before later lowers). X uses two diagonals of a 30 mm square with a lift between; O uses a commanded 16.5 mm radius, 48-gon and 30 degrees overlap, expected radius about 14.7 mm. Draw speed is 20 mm/s averaged over each smoothstep segment (30 mm/s peak); each completed stroke dwells 0.3 s before lifting. The constant (+0.8,+1.8) mm aim correction is a measured arm property applied to every layout. Drawing TCP z=0.8646 m, travel z=0.9175, grasp z=0.8800, release z=0.8850; expected compression 2.0 to 5.5 mm with 0 to 3.5 mm pad sag and 1.4 mm command shortfall (raw geometric range 3.4 to 6.9 mm).

The plan contains fixed X and O slots, padding the unused symbol with stationary hovers. Waypoint positions resolve from the printed starting board when first played, after the embodiment reset chooses its seed; durations and waypoint count are identical for every game. The resolver reads no scorer or object state. It parks by lowering the collar, opening, clearing the pads and backing off 75 mm, then holds 3.5 s. No object is actuated or attached to the robot.

The plan takes 38.954 simulated seconds (86 waypoints). Including the extra success hold, it uses 44.4% of the 1,800-step / 90 s run limit, leaving 55.6% spare. The 0.8x-duration timing variant takes 31.163 s. At the measured calibration throughput of about 0.16 wall seconds per control step, the full nominal plan plus success hold is approximately 2.1 minutes after simulator startup; a full-limit trial is about 4.8 minutes. These wall-time figures are throughput estimates from the pen probes.


The setup screenshot and demo video come from one GPU demo run with `--seed 0` (board 6), scored by the scorer in this folder; it reached 1.0 with success, as recorded in task.json.

Simulator diagnostics print `CLASSIFY` every 40 control steps with board index, inferred symbol, winning cell, winning-zone glyph and classifier details, and outside-zone path length in mm. Offline scoring stays silent; diagnostics do not affect credit.
