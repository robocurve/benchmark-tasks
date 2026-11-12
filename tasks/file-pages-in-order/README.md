# File four numbered pages in order in a tray

A black shelf 15 cm tall and 50 cm long runs away from the arms, left of centre. At its near end lies a pile of
four white A6 pages of card, each printed with a large black numeral: 1, 2, 3 or 4. The pages are out of order (2,
4, 1, 3 from the top). The pile is sheared: each page lies 1.8 cm further to the right than the one below it, and
the top page's edge sticks out 1.8 cm past the shelf's right edge. The shelf's top is slick, and the rest of it is
free. On the table below the pile, against the shelf's right side, stands one shallow tray.

The pages have to end in the tray in order: page 4 at the bottom, then 3, then 2, with page 1 on top. Only the top
page of the pile can be taken, so a page that comes up before its turn has to be laid on the shelf and fetched
later. A page is 0.5 mm thick and lies flat, so a parallel gripper cannot lift it from above; it can be pinched by
an edge that sticks out past the shelf, and it has to be set down with an edge still sticking out if it is to be
picked up again.

Instruction: *File the four numbered pages in the tray in order: page 4 at the bottom, then 3, then 2, with page 1 on top, each lying flat. The pages are in a mixed pile on the shelf, and you can lay pages on the shelf while you sort them.*

## Where this comes up

- **Offices.** Putting loose pages into numerical order, collating, is routine clerical work. O*NET lists "Compile,
  copy, sort, and file records of office activities, business transactions, and other activities" among the tasks
  of Office Clerks, General (43-9061.00), an occupation of about 2.6 million US workers.
- **Print and copy shops.** Pages that come out of a printer or a folder in the wrong order are laid out on a
  bench and gathered in sequence.
- **Records rooms, clinics and schools.** Numbered forms, charts and exam scripts are put back in order before
  they are filed.
- **At home.** A robot tidying a desk has to put the pages of a document back in order.

## What it measures

- **Sequencing with a place to lay things down.** Only the top page of a pile can be taken, and the tray's pile
  can only be built from the bottom up. The robot has to decide which pages to lay aside and fetch them when
  their turn comes. Both pile orders need two pages laid aside.
- **Setting a page down so it can be picked up again.** A page laid flat on the shelf with no edge sticking out
  cannot be pinched. It has to be placed with 1.8 cm of edge past the shelf.
- **Taking one thin page, and only one.** The page has to be pinched by its protruding edge, level, without the
  fingertips reaching the page beneath.
- **Filing flat.** A page that lands leaning on a wall does not count, and neither does a page above one that is
  out of order.
- **Reading the numerals** is needed but is not the hard part: they are 9 cm tall.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: objects, `success()` and `score()` |
| `sheetkit.py` | Task-file boxes with their own friction, and pages with numerals drawn on them |
| `pick_demo.py` | The scripted two-handed demo, and a shortcut that must not score |
| `task.json` | The task's details |

`sheetkit.py` hooks `yam_arena.scene.task_object` and `yam_arena.scene.object_states` without modifying yam_arena,
so the task runs with the standard `yam_arena` embodiment and `task_score` scorer.

## Run a trial

On an instance started from the simulation walkthrough's launchable, in a new shell once its setup has finished,
from this folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "File the four numbered pages in the tray in order: page 4 at the bottom, then 3, then 2, with page 1 on top, each lying flat. The pages are in a mixed pile on the shelf, and you can lay pages on the shelf while you sort them." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/pages-dream

for log in ~/yam-eval/pages-dream/*.json; do inspect-robots video "$log"; done   # one MP4 per camera
```

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode. Setting
`PAGES_LOT=B` before the command gives a second pile order (3, 1, 4, 2 from the top) with the same scene.

## Scripted demo

```bash
python pick_demo.py task.py ~/pages-demo               # full score; add --window to watch on the instance's desktop
PAGES_LOT=B python pick_demo.py task.py ~/pages-demo-b
python pick_demo.py task.py ~/pages-sweep --sweep --max-steps 400   # a shortcut that must score 0
```

For pile order 2, 4, 1, 3 the demo makes six moves: page 2 from the pile to the shelf, page 4 from the pile to the
tray, page 1 from the pile to the shelf, page 3 from the pile to the tray, then page 2 and page 1 from the shelf
to the tray. For each move:

1. If the page comes from the pile and the pile's top page does not stick out yet, the left hand, closed and
   pointing down, presses on the pile and slides all of it 1.8 cm to the right over the slick shelf.
2. The right hand, level and pointing left, pinches the page by the edge that sticks out past the shelf. Its
   fingertips reach 1.1 cm onto the page, less than the pile's 1.8 cm step. It lifts the page 3 cm.
3. To lay the page aside, it carries the page along the shelf and sets it down there with the same edge sticking
   out. To file it, it draws the page out, carries it to the tray and lowers it until the page's far edge is just
   above the shelf's edge, then opens fully and backs away: the page tips off the edge and slides down flat into
   the tray.

The demo is open loop: every pose comes from the layout constants in `task.py`, and the sequence of moves from the
pile order being run. Object states are read only to report where each page ended. It writes `pick_*_cam.mp4`,
`setup.png` (the first top-camera frame) and `result.json` to the output folder.

## Scoring

Evaluated every control step; the score at the end of the episode counts (`SCORE_OVER = "end"`).

- A page counts as in the tray when it lies flat there: seen from above, all of it is inside the tray (from the
  shelf's face to the outside of the walls, within 2 mm); it is tilted at most 10 degrees; and its lowest point is
  within 6 mm of the tray's floor. A page leaning on a wall or held above the tray does not count.
- Read the tray's pile from the bottom up and count how many pages, starting from the bottom, match the wanted
  order 4, 3, 2, 1; stop at the first that does not.
- Score = (pages in order minus the other pages in the tray) / 4, never below 0. That is 0.25 for page 4 at the
  bottom, 0.5 with page 3 on it, 0.75 with page 2, 1.0 with page 1 on top, and each page in the tray out of order
  cancels one.
- Pages left on the shelf, on the table or off the table add and subtract nothing.
- 1.0 (success, which ends the trial): all four pages in the tray in order and still (under 1 cm/s and 0.5 rad/s)
  for 1 s.

## Verification

Run on an L40S with the commands above (`object_jitter=0`).

| Run | Score | Notes |
|---|---|---|
| Scripted demo, pile order 2, 4, 1, 3 | 1.0 | Success at step 1,307 (65 s). The tray's pile reads 4, 3, 2, 1 from the bottom; every page flat, tilt 0.0 degrees. |
| Scripted demo, pile order 3, 1, 4, 2 | 1.0 | Success at step 1,306, same script. The tray's pile reads 4, 3, 2, 1 from the bottom. |
| Dream-1, pile order 2, 4, 1, 3 | 0.0 | Full 2,400 steps. Both arms reached toward the pile and the tray again and again; no page was taken and the pile stayed on the shelf. |
| Shortcut: sweep the pile off the shelf | 0.0 | 248 steps. Three pages land flat in the tray reading 1, 4, 2 from the bottom, and page 3 stands against the shelf. |

## Runtime and seeds

About 3.5 minutes per demo run and about 7 minutes per 2,400-step Dream-1 run on an L40S, including Isaac Sim's
start-up. The layout is fixed (`-E object_jitter=0`); there are no seeds to choose.

## Fidelity

Each page is one rigid plate the size and weight of an A6 sheet of 400 gsm card (14.8 x 10.5 cm, 0.5 mm, 6.2 g);
it does not bend, crease or flutter, and air drag on it is modelled as damping. Card is used because thin office
paper is too floppy to model as a rigid plate. The numerals are drawn on the top face only, as flat strokes, and
are not felt. Friction: card on card 0.30 static and 0.25 kinetic; card on the shelf's slick top 0.16 and 0.135; a
finger pad on card 0.40 and 0.375. The shelf and tray are fixed.

## Real-world build

- Pages: A6 pieces cut from white 400 gsm card, each printed or marked on one face with a numeral 9 cm tall with
  1.2 cm strokes.
- Shelf: a box 49.6 x 20 x 15 cm fixed to the table, its top covered with PTFE film or polished laminate so that
  card slides on it more easily than card on card.
- Tray: smooth plastic or laminated board with the dimensions in `task.json`, fixed to the table against the
  shelf's right face.

## Third-party assets

None. The numerals are built from rectangles in `task.py`; no font is used.
