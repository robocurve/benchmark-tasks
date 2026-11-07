# Shelve three returned books upright and brace the row with a bookend

Three returned books stand in a return tray in front of the left arm; they go onto a shelf between the arms, upright,
in one closed row that starts against the shelf's left end wall, and the bookend in front of the right arm goes
against the last book so none can lean. A book left lying flat on the shelf or on top of another book scores the whole
trial 0. The left arm can reach the tray and the right arm the bookend, so a full score takes both arms, one after the
other.

## Run a trial

On a machine with the official simulator (`yam-isaac-lab-arena`'s `setup.sh`, or the Brev launchable), with a Dream
API key in `~/.dreamlabs/credentials`, from this folder:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=shelve_books_upright_brace_bookend.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Shelve the red, green and blue books from the tray upright on the shelf in one row, in any order, with no gaps, starting against the shelf's left end wall, then stand the black bookend against the right side of the last book." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/shelve-books-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=shelve_books_upright_brace_bookend.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Shelve the red, green and blue books from the tray upright on the shelf in one row, in any order, with no gaps, starting against the shelf's left end wall, then stand the black bookend against the right side of the last book." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/shelve-books-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/shelve-books-*/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. The trial ends at 1.0 once all three books stand in the row with the bookend
against them, everything still, for 2.5 s; otherwise it scores the partial credit below at the last step.

Offline checks of the scorer and the demo's reach (no GPU): `pytest test_shelve_books_upright_brace_bookend.py` in an
environment with `yam_arena` installed.

## Scoring

A book is **shelved** when, seen from above, no part of it is more than 1 cm beyond the shelf board's edges, its
bottom is within 2 mm of the board's top (set down, not held above it), its tall edges are at most 5 degrees from
vertical, and its 76 mm side is within 10 degrees of front to back (either 76 mm end facing the arms). **Gaps** are measured side to side, seen from above,
between the outermost points of the two objects: for a book turned or leaning a little, its corner or top edge
nearest the neighbour. Touching or overlapping outlines count as 0 mm. The **row** is built from the end wall: the
shelved book nearest the wall is in it if at most 5 mm separates its leftmost point from the wall, the next if at most
5 mm separates it from the previous book, and so on. The books may be in any order.

| Credit | For |
|---|---|
| 0.2 | each book in the row |
| 0.1 | instead of 0.2: a book shelved but not in the row |
| 0.4 | the bookend, in any orientation, standing on the board (no part more than 1 cm beyond the board's edges, bottom within 2 mm of its top, at most 5 degrees from vertical) against the right side of the third book of a complete row: its centre right of the book's centre, its leftmost point within 5 mm of the book's rightmost point (at most 5 mm right of it, or at most 5 mm left of it), and its left face overlapping the book's right face by at least 2 cm front to back |
| 0.2 | instead of 0.4: the bookend standing on the board otherwise, while at least one book is in the row (0 with no book in the row) |
| 0 overall | a book with its centre over the board tilted more than 45 degrees (lying on its side or fore-edge), or a book resting on another book or the bookend (outlines overlapping seen from above, its bottom within 1 cm of the other's top, above or below it); a book held more than 1 cm above the others doesn't count |

On a real shelf, a ruler checks each gap between the closest points of neighbouring objects and a phone level app
each tilt; a photo from above shows the books' 76 mm sides pointing front to back and every outline over the board.

## The scripted demo

The left arm takes the books from the tray one at a time, the one nearest the shelf first (blue, then green, then red),
gripping each from above by its front and back faces near the top (the fingers close front to back, so no finger
comes between two books). It carries each book along the front of the shelf, moves it in over the board 12 mm right of
its spot, steps it left and sets it down 2 mm from the end wall or the previous book, letting go 8 mm above the board,
and lifts straight up. It then parks above the left end of the empty tray. The right arm waits above the bookend until then, picks it up
the same way, sets it down 2 mm right of the last book, and parks in front of its own base so the camera sees the row.

## Runtime and seeds

- The demo's plan takes about 49 s of simulated time (the left arm 36 s, the right arm 49 s including its 36 s wait),
  under 1,000 of the 2,400 steps. On the official simulator the scripted demo reached `task_score` 1.0: the trial
  ended on success, with the three books upright in the row and the bookend against the last one, before the right
  arm reached its park. On an L40S, once Isaac Sim has started, a scripted trial takes about 1 to 2 minutes of wall
  time and a full Dream-1 trial 4 to 5 minutes.
- One fixed layout: `-E object_jitter=0`, so the seed doesn't change the start.
