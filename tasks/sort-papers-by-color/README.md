# Sort a sheared pile of colored sheets into trays

A black shelf 15 cm tall and 50 cm long runs away from the arms, left of centre. At its near end lies a pile of
four A6 sheets of colored card (white, yellow and pink, one color twice, in mixed order). The pile is sheared: each
sheet lies 1.8 cm further to the right than the one below it, and the top sheet's edge sticks out 1.8 cm past the
shelf's right edge. The shelf's top is slick. On the table along the shelf's right side stand three shallow trays
in a row, each marked with its color by its outer wall and a strip on its floor: yellow nearest the arms, then
pink, then white.

Sorting loose papers into trays is everyday clerical work (O*NET 43-9061.00, Office Clerks, General: "Compile, copy,
sort, and file records of office activities, business transactions, and other activities"). A sheet lying on a pile
is 0.5 mm thick and flat, so a parallel gripper cannot lift it from above. The top sheet can be pinched by the edge
that sticks out, and only that sheet: the other edges are further in. Once it is gone, the next sheet has to be
brought to the edge by sliding the pile. The task tests that handling of thin sheets; the color only decides which
tray a sheet goes to.

Instruction: *Sort the stack of colored sheets: take each sheet off the stack and put it in the tray marked with the
same color, and leave every sheet lying flat in its tray.*

## Where this comes up

- **Offices and mailrooms.** Clerks separate the copies of multipart forms, which commonly come as a white, a
  yellow and a pink sheet that each go to a different recipient, and sort incoming paperwork into trays. O*NET
  lists "Compile, copy, sort, and file records of office activities, business transactions, and other activities"
  and "Open, sort, and route incoming mail" among the tasks of Office Clerks, General (43-9061.00), an occupation
  of about 2.6 million US workers.
- **Print and copy shops.** Finished sheets are taken off a pile one at a time and stacked by job.
- **Clinics, schools and warehouses.** Color-coded slips such as intake forms, permission slips and delivery
  notes are filed by color.
- **At home.** A robot tidying a desk has to take loose papers off a pile without scattering the rest.

## What it measures

- **Taking one thin sheet, and only one.** The sheet is 0.5 mm thick and lies flat on others. It has to be pinched
  by 1.8 cm of protruding edge, level, without the fingertips reaching the sheet beneath.
- **Carrying and setting it down flat.** A sheet that lands leaning on a tray wall or across a divider does not
  count.
- **Bringing the next sheet out.** The pile has to be slid one step so the next edge protrudes, without spilling it.
- **Not the color.** Color only decides the tray. Pushing the pile off the shelf, the easy way to "move the papers",
  puts every sheet in one tray and scores 0; that is what Dream-1 did.

Nearest tasks on the board when this was built: pulling a slip-sheet from under a load, putting scrap paper in a
recycling bin, and trimming paper on a guillotine. None asks for single sheets to be separated from a pile.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: objects, `success()` and `score()` |
| `sheetkit.py` | Task-file boxes with their own friction, spawned through yam_arena's cuboid spawner |
| `pick_demo.py` | The scripted two-handed demo, and a shortcut that must not score |
| `task.json` | The task's details |

`sheetkit.py` hooks `yam_arena.scene.task_object` without modifying yam_arena, so the task runs with the standard
`yam_arena` embodiment and `task_score` scorer.

## Run a trial

On an instance started from the simulation walkthrough's launchable, in a new shell once its setup has finished,
from this folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Sort the stack of colored sheets: take each sheet off the stack and put it in the tray marked with the same color, and leave every sheet lying flat in its tray." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/papers-dream

for log in ~/yam-eval/papers-dream/*.json; do inspect-robots video "$log"; done   # one MP4 per camera
```

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode. The pile's
order from the top is pink, white, yellow, pink. Setting `PAPER_LOT=B` before the command gives a second order
(yellow, pink, white, yellow) with the same scene.

## Scripted demo

```bash
python pick_demo.py task.py ~/papers-demo              # full score; add --window to watch on the instance's desktop
PAPER_LOT=B python pick_demo.py task.py ~/papers-demo-b
python pick_demo.py task.py ~/papers-sweep --sweep --max-steps 400   # a shortcut that must score 0
```

For each sheet, top first:

1. The right hand, level and pointing left, pinches the top sheet by the edge that sticks out past the shelf. Its
   fingertips reach 1.1 cm onto the sheet, less than the pile's 1.8 cm step. It lifts the sheet 3 cm straight up.
2. It draws the sheet out, carries it along the shelf to the tray of the sheet's color, and lowers it until the
   sheet's far edge is just above the shelf's edge. It opens fully and backs away, and the sheet tips off the edge
   and slides down flat into the tray.
3. The left hand, closed and pointing down, presses on the pile and slides all of it 1.8 cm to the right over the
   slick shelf, so the next sheet sticks out.

The demo is open loop: every pose comes from the layout constants in `task.py`, and the order of colors from the
pile order being run. Object states are read only to report where each sheet ended. It writes `pick_*_cam.mp4`,
`setup.png` (the first top-camera frame) and `result.json` to the output folder.

## Scoring

Evaluated every control step; the score at the end of the episode counts (`SCORE_OVER = "end"`).

- A sheet counts as in a tray when it lies flat there: seen from above, all of it is inside that tray (between the
  middles of its dividers, from the shelf's face to the outside of the outer wall, within 2 mm); it is tilted at
  most 10 degrees; and its lowest point is within 6 mm of the tray's floor. A sheet leaning on a wall, lying across
  a divider or held above the tray does not count.
- Score = (sheets in the tray of their own color minus sheets in a tray of another color) / 4, never below 0. That is
  0.25 for each sheet sorted, and each sheet in a wrong tray cancels one.
- A sheet left on the shelf, on the table or off the table adds and subtracts nothing.
- 1.0 (success, which ends the trial): all four sheets in the trays of their own color and still (under 1 cm/s and
  0.5 rad/s) for 1 s.

## Verification

Run on an L40S with the commands above (`object_jitter=0`).

| Run | Score | Notes |
|---|---|---|
| Scripted demo, pile order A | 1.0 | Success at step 941 (47 s). All four sheets flat in their own trays, tilt 0.0 degrees. |
| Scripted demo, pile order B | 1.0 | Success at step 909 (45 s), same script. All four flat, tilt 0.0 degrees. |
| Dream-1, pile order A | 0.0 | Full 2,400 steps. The left arm pushed the whole pile off the shelf into the yellow tray by about 45 s; no sheet was pinched. |
| Shortcut: sweep the pile off the shelf | 0.0 | 248 steps. All four sheets end in or over the yellow tray: the pink top sheet lies there within 10 degrees of flat (a wrong tray), the others lean against the shelf. |

## Runtime and seeds

Under 3 minutes per demo run and about 7 minutes per 2,400-step Dream-1 run on an L40S, including Isaac Sim's
start-up. The layout is fixed (`-E object_jitter=0`); there are no seeds to choose.

## Fidelity

Each sheet is one rigid plate the size and weight of an A6 sheet of 400 gsm card (14.8 x 10.5 cm, 0.5 mm, 6.2 g);
it does not bend, crease or flutter, and air drag on it is modelled as damping. Card is used because thin office
paper is too floppy to model as a rigid plate. Friction: card on card 0.30 static and 0.25 kinetic; card on the
shelf's slick top 0.16 and 0.135; a finger pad on card 0.40 and 0.375. The shelf and trays are fixed.

## Real-world build

- Sheets: A6 pieces cut from 400 gsm card in white, yellow and pink.
- Shelf: a box 49.6 x 20 x 15 cm fixed to the table, its top covered with PTFE film or polished laminate so that
  card slides on it more easily than card on card.
- Trays: one unit of smooth plastic or laminated board with the dimensions in `task.json`, fixed to the table
  against the shelf's right face; each tray's outer wall and floor strip in its color.

## Design notes

- A squared stack, with one hand sliding the top sheet out for the other to pinch, was tried first. Under a
  fingertip every layer has almost the same friction, so which layer slips is not determined: on the fourth sheet
  three moved together. The sheared pile on a slick shelf removes that dependence.
- The trays are 17 cm deep for a 10.5 cm sheet because a sheet tipping off the shelf can land several centimetres
  out from the shelf's face.

## Third-party assets

None.
