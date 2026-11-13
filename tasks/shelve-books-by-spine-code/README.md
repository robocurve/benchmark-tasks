# Shelve a pile of flat-lying books upright, in alphabetical order of their spine codes

Four pocket books lie flat in a pile, each with a four-letter code printed down its spine. An empty shelf stands
beside the pile. The robot has to stand all four books upright on the shelf, spines facing out, in alphabetical
order of their codes from left to right.

What makes it hard is getting a book from lying flat to standing in its place:

- **A flat book cannot be stood up from above.** A hand that comes straight down on the pile can lift a book but
  cannot turn it onto its end. The pile is staggered, each book 14 mm nearer the arms than the one beneath, so each
  book's spine edge sticks out. A level hand can pinch that 14 mm edge with its fingers one above the other, and a
  quarter turn of the wrist then stands the book upright with its spine still facing the arms.
- **The pinch is shallow and the book hangs off it.** The fingers hold 11 mm of an 80 mm deep, 160 g book, and
  the arm sinks about a centimetre under it. Drawn out without being lifted clear, the book drags the next one off
  the pile. Carried too low, its bottom edge catches the front of the shelf's board.
- **Each book goes to a place fixed by its code,** often between two books already standing, with 9 mm of room on
  either side. It is slid in until the fingers reach the neighbours' spines, set down, let go and pushed home.
- **The order has to be read.** The pile's order is unrelated to the codes, and in each layout two codes share
  their first letters (MINT and MOTH; DUNE and DUSK), so a book's place cannot be told from its position in the
  pile, its colour or its first letter.

## Where this comes up

Putting books, files and cased media back on a shelf in alphabetical or call-number order is familiar from homes,
classrooms, offices and libraries. Returned books arrive in piles, lying flat, and have to be stood up and put in
their places one at a time.

## What it measures

- Re-orienting an object in the grasp: pinching it by an edge and turning it a quarter turn with the wrist.
- Carrying a load on a shallow pinch and allowing for how far the arm gives under it.
- Placing objects side by side in a row at positions that depend on reading a printed code.

It differs from shelving tasks that start with the books already standing and accept any order. Here the books
start flat in a pile, have to be turned upright, and each has one correct place.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: books with their spine letters, pile, shelf, `success()` and `score()` |
| `bookkit.py` | Parts built from several boxes, with printed marks that do not collide, for yam_arena task files |
| `demo.py` | The scripted demo |
| `test_scoring.py` | Checks of the scoring rule on hand-made states (no simulator needed) |

`bookkit.py` hooks `yam_arena.scene` and the embodiment's step and reset without modifying yam_arena, so the task
runs with the standard `yam_arena` embodiment and `task_score` scorer.

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough (`source /data/IsaacLab-Arena/yam.env`), from
this folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Stand the four books upright on the shelf with their spines facing out, in alphabetical order of the four-letter codes on their spines, from left to right." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/books-dream
```

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode.

There are two layouts. The default is layout A (codes MOTH, FERN, MINT, BRIM from the top of the pile down). Set
`BOOKS_LOT=B` before the command for layout B (DUSK, WREN, DUNE, LARK), in which the pile and the shelf also stand
6 mm nearer the arms and 2 mm further left.

## Scripted demo

```bash
python demo.py task.py demo_out             # full score: writes demo_*_cam.mp4, setup.png, result.json
BOOKS_LOT=B python demo.py task.py demo_b   # the same script on layout B
```

The left arm swings out of the way. The right hand points level and forward and, for each book from the top of the
pile down:

1. reaches in with its fingers one above the other, pinches the book by 11 mm of its spine edge, lifts it 4 cm and
   draws it out toward the arms;
2. carries it across in front of the shelf and turns its wrist a quarter turn, which stands the book upright, head
   up, with its spine still toward the arms;
3. carries it to its place on the shelf, slides it in until 1.8 cm of spine still sticks out, sets it down and lets
   go;
4. closes its fingers and pushes the spine home, flush with the shelf's front.

The loaded arm sinks about 1.2 cm and its hand rolls about 6 degrees off level. The demo allows for both: it
carries the book that much higher and turns the wrist 6 degrees further.

The demo is open loop. Every pose comes from the layout constants in the task file, which are what the first camera
frame shows with `object_jitter=0`, and each book's place comes from the codes the task file gives for the layout
being run. Object states are read only to write the result file.

## Scoring

Tools: a photo of the shelf from the front and from above, a ruler, a protractor or level app, and the trial video.

The score is the number of books in place, divided by four.

A book is **shelved** when all of these hold at the end:

1. it stands on the shelf's board: its centre is over the board and its bottom is within 4 mm of the board's top;
2. it is upright, head up: its tail-to-head direction is within 10 degrees of vertical;
3. its spine faces out: within 20 degrees of the shelf's front, seen from above;
4. it is still (moving slower than 5 cm/s) and has been let go (no gripper's grasp point within 2 cm of it).

Read the shelved books from left to right as seen from the arms. A shelved book is **in place** when every shelved
book to its left has an alphabetically earlier code and every shelved book to its right a later one.

So four books standing in alphabetical order score 1.0, and each book out of order, lying down, upside down, spine
to the back, off the shelf or still in the pile loses 0.25. The books do not have to touch each other.

## Verification

| Run | Score | What happened |
|---|---|---|
| Scripted demo, layout A | 1.0 | Success at step 1,134 (57 s). BRIM, FERN, MINT, MOTH from the left, each within 1 degree of upright and of facing the front. |
| Dream-1, 2,400 steps | 0.0 | The left gripper went to the top of the pile several times, but no book left the pile and the shelf stayed empty. |

The episode ends at success, which comes as soon as the hand has let go of the fourth book and backed away. The
video therefore stops before that book's last push, with its spine 1.8 cm proud of the other three. The score does
not depend on how far in a book is pushed.

Earlier demo runs scored 0, 0.25 and 0.75 and led to changes that are now in the demo and the scene: the carrying
height and the wrist turn allow for the arm's sag, each book is lifted 4 cm before it is drawn out so that it does
not drag the next one off the pile, and the stand under the pile is 4.5 cm tall because a level hand cannot get a
finger under a book lower than that.

Layout B was checked against the arm's kinematics (reach, joint limits and clearances) but not recorded in the
simulator.

`python test_scoring.py` checks 18 cases of the scoring rule on hand-made states, among them two books the wrong
way round, a book on its head, a book with its spine to the back, a book lying on the shelf and a book still held.

## Runtime and seeds

The layout is fixed (`-E object_jitter=0`); there is no randomness in the task. On an L40S the 2,400-step Dream-1
trial took 6 minutes including start-up, and a demo run takes about 3 minutes.

## Fidelity

- Each book is a rigid box, 80 mm deep, 120 mm tall and 26 mm thick, 160 g: a pocket-format paperback. A real
  paperback flexes when held by one edge and its cover can slip against the pages; this one does neither.
- The codes are block letters 21 mm tall on a cream label down the spine, drawn on the book's surface. At
  320 x 176 they are easy to read in the wrist cameras from the distance the hand works at, and small in the top
  camera.
- The pile is staggered evenly, 14 mm per book, and stands on a fixed 4.5 cm stand. The shelf is fixed.
- Grip force is not judged.

## Real-world build

- Books: four pocket-format paperbacks of about 80 x 120 x 26 mm and 160 g, each with a spine label carrying its
  four-letter code in block capitals 21 mm tall, reading from head to tail.
- Pile: on a stand 4.5 cm tall, lying face up with spines toward the robot and heads to the left, in the order
  given in `task.json` for the layout, each book 14 mm nearer the robot than the one beneath it.
- Shelf: a board 15 cm long and 9.5 cm deep whose top is 2 cm above the table, with a back and a left end 7 cm
  tall, open on the right, fixed to the table at the marks given in `task.json`.

## Third-party assets

None. Every object, and every letter, is built from boxes in `task.py`.
