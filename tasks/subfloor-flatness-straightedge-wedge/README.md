# Check a subfloor for large-format tile with a straightedge and a measuring wedge, then tag it PASS or FAIL

Files for the `yam_arena` embodiment (Isaac Lab-Arena, bimanual YAM, `station` scene). `task.py` builds the scene,
scores the trial and holds the scripted demo; `yamkit.py` spawns the modeled props (with a convex-hull collider for
the wedge) and `guardkit.py` checks gripper contact. The robot sets a 61 cm straightedge on its edge across a subfloor
section, leaves a measuring wedge in the deepest gap under it, and stands a tag puck in the PASS or FAIL pocket. The
setup, scoring and fidelity notes are in `task.json`.

Instruction (task.py `INSTRUCTION`): "Check whether this subfloor is flat enough for large-format tile, along the strip
between the two pencil lines. Leave the measuring wedge in the deepest gap under the straightedge, and put the tag puck
in PASS or FAIL."

## Run a trial

Each run draws its board (case 1 to 8) at random and prints it to the log (`subfloor-flatness: board case N`);
`FLATNESS_CASE=1` to `8` fixes it. The scripted demo is open loop for the board `FLATNESS_CASE` fixes, like any fixed
layout: it is planned from the case's constants in task.py (where the deepest strip is, how deep it is, and so the
report) and reads nothing from the simulator while it runs. A model gets the random draw and has to find the trough
with the wedge. The demo reaches full score with `FLATNESS_CASE=2` (PASS) and `FLATNESS_CASE=3` (FAIL):

```bash
FLATNESS_CASE=2 inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/subfloor-flatness-straightedge-wedge/task.py -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Check whether this subfloor is flat enough for large-format tile, along the strip between the two pencil lines. Leave the measuring wedge in the deepest gap under the straightedge, and put the tag puck in PASS or FAIL." \
  --scorer task_score --max-steps 1200 --store-frames
```

and again with `FLATNESS_CASE=3`.

Model run (Dream-1): the same command without `FLATNESS_CASE` (a random board), with `--policy dream --max-steps 2400`
(2,400 steps = 2 minutes at 20 Hz).

Render the videos of a run with `inspect-robots video <log.json>`.

Offline checks, no simulator: `python3 test_task.py` (every tier, cap and adversarial state in all eight cases, and the
numbers in the text) and `python3 -m pytest -q test_demo.py` (the demo played in a kinematic world for every case).

## Scoring

`--scorer task_score` reports `score()` at the end of the run; the run ends early once `success()` (the 1.0 state)
has held for 1 s. Each level must hold for the last 1 s (20 steps). Definitions:

- Contact: two rigid faces within 0.5 mm (the simulator's resting gap). In a real trial, a 0.5 mm feeler that will not
  enter is the check.
- Set: the straightedge stands on its edge (height axis within 5 degrees of vertical), its whole bottom lies between
  the pencil lines and over the panel, and within each end's last 5 cm some point of its bottom touches the floor, so
  it rests on the high points; and it is still (under 5 mm/s and 6 degrees/s).
- In a gap: at least 2 mm of wedge under the straightedge's near bottom edge (a tip that only touches the face
  measures nothing), the wedge lying flat (within 3 degrees) and pointing under the edge (within 20 degrees of
  square), its top in contact with the edge and its bottom in contact with the floor there, and still.
- Reading: the wedge's scale where the edge crosses it, 0.3 mm + 0.1 x the length of wedge under the edge. It is read
  at the straightedge's near (robot-side) edge; the trough runs across the panel, so the gap is the same at both edges.

| Score | When | Why this much |
|---|---|---|
| 1.0 | All of 0.75, and the puck standing upright (within 15 degrees) at rest in the correct pocket: PASS if the depth is 1.6 mm or less | The report is the point of the check |
| 0.75 | Set, the wedge in a gap, the reading within 0.3 mm of the trough's depth, and the wedge's whole 3 cm width on the deepest 6 cm | The deepest place was found. The next strip out is 0.5 mm shallower, so a reading there misses by 0.5 mm. An end lifted inside the 0.5 mm contact allowance can add up to about 0.4 mm at the wedge, so the wedge must also lie on the deepest 6 cm: forcing it in on the next strip can't pass. A wedge forced in until it lifts an end more than 0.5 mm fails "Set" |
| 0.5 | Set, and the wedge in a gap elsewhere | A real measurement was made, but not at the deepest place |
| 0.25 | Set | The reference every reading needs, on the line and resting on both high points |
| 0 | Anything less, including doing nothing | No reference, no check |

Caps at 0.5:
- A gripper still in contact with the straightedge, wedge or puck at the end (the gripper's own palm and finger shapes
  within 2 mm of the part). The check isn't finished while a hand holds a tool on the floor, and a hand on the
  straightedge could hold a lifted end down. It caps rather than zeroes because the reading and report may be right.
- The puck in the wrong pocket. A wrong report sends tile onto a bad floor, or a crew to fix a good one.
- The puck in a pocket before the wedge first measured a gap under a set straightedge (two steps in a row). That is a
  guess. The scorer remembers it for the whole trial, so measuring afterwards doesn't undo it.

Arm position is not scored. Possible scores: 0, 0.25, 0.5, 0.75, 1.0.

Real trial: each case is its own test board with a known trough depth (see Real-world build). Check both ends of the
straightedge with a 0.5 mm feeler (it must not enter). Tape the wedge's centerline from the board's end: it must be
within 1.5 cm of the deepest window's center, so the wedge's 3 cm width lies on the 6 cm window. Photograph the wedge's
scale from above where the straightedge's edge crosses it (marks every 0.5 mm of reading, 5 mm apart, so 0.3 mm is
resolved by eye between marks). Take a top photo of the pocket.

## Runtime and seeds

- Scripted demo: 1,091 steps with `FLATNESS_CASE=2` and 1,141 with `FLATNESS_CASE=3` (55 and 57 s at 20 Hz; the run
  ends once the full score has held for 1 s); `--max-steps 1200` covers both.
- Model run limit: 2,400 steps (2 minutes at 20 Hz), about twice the demo's time.
- Seeds: the board is drawn per run unless `FLATNESS_CASE` fixes it. `-E object_jitter=0`: everything else is the same
  in every case.

What the demo does (for the fixed board): both hands lift the straightedge out of its rests and set it on its edge
with its middle at x 36 (its bottom between the pencil lines), then go aside. The right hand takes the wedge off the
table, carries it over the rests' height, lays it on the floor at the deepest strip with its tip 1.5 cm short of the
straightedge's face, closes, and pushes the heel in +x (6 mm/s, then 2 mm/s from 2 mm short of the face) until its goal is 4 mm past where the wedge fills the gap
(the arm gives about that much under the wedge's friction), holds 1 s, backs off 3 mm and goes aside. The left hand
then stands the puck in the pocket for that board, holding it over its whole height and letting go by opening only
partly until the fingertips are out of the pocket.

Cases (`CASES` in task.py; depth, center of the deepest strip):

| Case | Depth | Deepest strip at | Report |
|---|---|---|---|
| 1 | 1.0 mm | y -13 | PASS |
| 2 | 1.5 mm | y 7 | PASS |
| 3 | 2.0 mm | y -5 | FAIL |
| 4 | 2.5 mm | y 15 | FAIL |
| 5 | 3.0 mm | y -3 | FAIL |
| 6 | 1.5 mm | y -9 | PASS |
| 7 | 2.0 mm | y 11 | FAIL |
| 8 | 1.0 mm | y 3 | PASS |

Cases 2 and 6 (1.5 mm) and 3 and 7 (2.0 mm) sit one step either side of the 1.6 mm limit. Depth and place vary
separately, and nothing in the scene shows where the trough is, so a memorized plan or a fixed report can't pass.

## Scene

Positions in cm from the midpoint between the two arm bases: x forward, y to the robot's left, table top z 0.

| Object | What it is | Where |
|---|---|---|
| Subfloor section (fixed) | 3/4 in (1.9 cm) panel, plywood face, 23 x 64 cm. Two pencil lines 6 cm apart mark the check line. The floor is 32 strips 2 cm wide (along y); the case's board lowers 3 strips (6 cm) by the trough's depth and each strip outward 0.5 mm less, in 0.5 mm steps (a sag between joists runs along the joists, so it crosses the line as a trough) | x 22 to 45, y -32 to 32; lines at x 32 and 38 |
| Straightedge | 24 in aluminum box-beam level used on its milled bottom edge: 61.0 x 2.5 x 6.3 cm, 0.50 kg. Too long to carry level by one end, so both hands lift it | on its edge in two fixed rubber rests at x 19, y -20 and 20 (rest top 3 cm up; 2.7 cm slot between 1.5 cm cheeks) |
| Measuring wedge | builder's aluminum measuring wedge: 15 cm long, 3.0 cm wide, 0.3 mm at the tip rising 1:10 to 15.3 mm at the heel, 95 g; a mark every 0.5 mm of reading (5 mm apart), longer every 1 mm, full width every 5 mm | flat on the table at x 19.5 to 34.5, y -39, tip toward +x |
| Report board (fixed) | white plate 18 x 8 cm with two pockets 5.0 cm square, 1 cm deep, 5 mm walls: PASS (green, check mark) and FAIL (red, X) | plate centered x 27, y 39; PASS at x 32.5, y 38; FAIL at x 21.5, y 38 |
| Tag puck | white-anodized aluminum, 3.2 cm across, 2.0 cm tall, 43 g | on a ring at x 27, y 38 |

The panel is 23 cm deep so that a wedge lying with its tip under the straightedge has its center of mass on the
panel (the wedge's heel, 15 cm back, may overhang the front edge).

## Sources

- Carpenters (SOC 47-2031): about 959,000 jobs in 2024 (BLS Employment Projections); O*NET task "Verify trueness of
  structure, using plumb bob and level." Flooring installers and tile setters make the same check.
- ANSI A108.02 (2013 edition 4.1.4.3; 2023 edition 4.2): for tile with any edge 15 in or longer, at most 1/16 in
  (1.6 mm) in 24 in, measured from the high points. A 24 in straightedge is the matching tool.

## Real-world build

- Test boards (one per case): a 16 mm MDF base, 23 x 64 cm, with six 0.5 mm (0.020 in) polystyrene sheets glued on
  (1.9 cm in all, the 3/4 in panel's height). For a trough n steps deep, cut a window across the full 23 cm in the top
  n sheets: the lowest of them 6 cm wide, each sheet above it 4 cm wider (2 cm more on each side), all centered on the
  case's place. The depth is then n x 0.5 mm, checked with feeler gauges under the straightedge. Draw the two pencil
  lines 6 cm apart, and face the top with a plywood-look laminate or paint so that every board looks the same.
- Straightedge: any 24 in aluminum box-beam level with a milled edge (about 1 x 2.5 in, about 1.1 lb).
- Measuring wedge: a builder's aluminum measuring wedge (DIN 18202-type gap wedge) with a 1:10 slope and 0.5 mm
  graduations. Commercial wedges start reading at 0 to 1 mm; use one with a thin tip, or re-mark one to
  0.3 mm + 0.1 x length.
- Report board: a white acetal plate with two 3D-printed pocket frames (green and red), screwed down. Puck: a 32 mm
  aluminum round bar cut 20 mm long and anodized white (or painted).
- Rests: two rubber-faced blocks with a 27 mm slot. Clamp the test board and report board to the table.
- Safety: the heaviest part is the 0.50 kg straightedge, carried 5 cm off the panel; nothing is sharp or hot.

## Files

- `task.py`: scene, scorer (`score`, `success`, the ladder in `tier`, caps) and the scripted demo (`demo`).
- `yamkit.py`, `guardkit.py`: the task's helper modules (props and colliders; gripper contact checks).
- `test_task.py`: hand-made states for every tier, cap and adversarial case (a report without measuring, a guess then
  a measurement, the wedge forced under an end, the straightedge up on a high spot, a shallower strip, the wrong
  pocket, a hand left on, a tip that only touches the face, a sliding wedge) in all eight cases, and the text's numbers.
- `test_demo.py`: the demo played in a kinematic world (arm lag and give) for every case.
