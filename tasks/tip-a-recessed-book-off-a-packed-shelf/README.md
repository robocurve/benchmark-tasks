# Take a recessed book off a packed shelf by tipping it out by its head

Nine hardcover books stand packed on a desktop shelf, 0.4 mm apart. The red one is shallower than its neighbours, so
its spine sits 12 and 14 mm behind theirs and no finger fits at its sides. The robot has to do what people do with a
full shelf: put a fingertip on the top of the book behind its spine, tip the book out toward itself, take the spine
that now stands proud with the other hand, draw the book out and set it down on the desk mat, with every other book
left standing. Taking one book from a full row is done daily in homes, schools and libraries, and it cannot be done by
reaching and grasping: the grasp only exists after the book has been tipped out.

## Parts (real size, real shape)

Positions are in cm from the midpoint between the arm bases, x forward, y to the robot's left.

- **Bookshelf**, white-painted MDF, clamped to the desk: base 20 deep x 32.3 wide x 1.5 thick with its front edge at
  x 38; two end panels 20 x 1.5 x 25 tall standing on the base, 29.3 apart; a back panel 0.6 thick and 26.5 tall
  behind the base. Open above.
- **Desk mat**, cork, 20 x 30 x 0.3, taped down in front of the shelf (x 18 to 38, y -15 to 15), against the base.
  The step from the base top down to the mat is 1.2 cm.
- **Nine cloth-bound hardcover books**, loose, standing on the base with their fore-edges at the back panel and
  their spines to the robot. From the robot's left, thickness x depth x height and mass (0.6 g/cm3):
  navy 2.8 x 17.8 x 23.5, 0.70 kg; green 3.5 x 17.6 x 23.8, 0.88 kg; grey 2.4 x 17.0 x 22.8, 0.56 kg;
  black 4.0 x 17.2 x 23.2, 0.96 kg; **red 3.2 x 16.0 x 24.0, 0.74 kg**, y -1.45 to 1.75, spine at x 41.96;
  mustard 3.0 x 17.4 x 23.4, 0.73 kg; brown 3.8 x 16.8 x 23.0, 0.88 kg; teal 2.6 x 18.0 x 23.6, 0.66 kg;
  slate blue 3.6 x 17.2 x 22.8, 0.85 kg. They total 28.9 cm in the 29.3 cm between the panels: 0.4 mm at every gap.
- **Each book** is built as a book, not a block: two 2.5 mm boards, a rounded spine piece 6 mm deep, and a page
  block that stops 3 mm short of the boards at the head, the tail and the fore-edge. The head therefore has a real
  recess, 3 mm deep and 27 mm wide on the red book, between the boards and behind the spine. The red book's head
  stands 8 mm above the black book's and 6 mm above the mustard one's.

## What the scripted demo does

Both arms, about 68 s (1362 steps in the offline dry run; a contact replay takes the same):

1. **Tip.** The right gripper, fingers closed, leaned 40 degrees to its own side and 20 degrees forward with its
   jaws along the shelf's depth (straight down the arm cannot reach 26 cm up), lowers a fingertip corner into the
   recess at the red book's head, 9 mm behind the spine's inner face, until its encoders show contact. So leaned,
   the fingertip's face toward the robot is vertical, and it meets the inner face of the spine square. The hand
   draws the fingertip toward the robot until the book stirs, then leads it along the arc about the book's foot edge
   until the book leans 22 degrees toward the robot, all the while keeping its commanded height 6 mm under the
   height its encoders report so the fingertip bears down on the page block. If the lean stops gaining for 2.5 s
   the fingertip has slipped off: it lifts, lands again and presses 5 mm deeper (up to three landings). At 22
   degrees the spine stands about 5 cm proud of its neighbours' where it will be taken.
2. **Take.** The left gripper, level, jaws closing across the book, comes in from the front at 17 cm above the table
   and pinches the proud spine across its 32 mm. A measured pad gap of 28 mm or more confirms the book; on a miss it
   tries once more 10 mm lower. All of this happens in front of the shelf.
3. **Free.** The right fingertip lifts straight up off the leaning head and the right arm parks. The left hand draws the book out level
   at 2 cm/s, still leaning 22 degrees: its tail slides on the base, passes the base's front edge and comes down the
   1.2 cm step onto the mat, and trails on across the mat on its foot edge.
4. **Set down.** With the foot edge 10.5 cm in front of the shelf the hand loosens its pinch and stands the book up
   to 9 degrees about its foot edge, pinches again and slides it on until its whole tail is in front of the shelf,
   then loosens once more and stands it upright. While the book turns, the hand is held to where the pinched point
   belongs with the foot edge in its place, not to wherever the foot edge has slid, so the book ends where the hand
   puts it: on the mat's centre line, x 20 to 36, 2 cm inside the mat's near and far edges. The hand then checks
   the book's pose and, if its spine is more than 4 mm from x 20, slides it there on its tail. It opens slowly,
   rises straight up clear of the book's head and parks. The pads keep one pitch throughout; the book turns between
   them. Both grippers end open at their ready poses, 25 cm or more from the book, which stands on its tail.

The demo reads the red book's live pose from the simulator (its lean and its foot edge) to stop the tip at
22 degrees, to place the pinch, to stand the book up and to check where it stands on the mat; everything else comes from its own joint encoders and
gripper opening. It never moves, teleports or edits a book.

Over the shelf, both tool points stay inside the red book's own slot (its thickness plus 12 mm a side) or at least
28 cm above the table, and outside that slot no tracked point of either arm comes below 26.5 cm (1.2 cm over the
tallest head): the hand never works on another book. In the contact replay no part of either arm touched any book
but the red one.

## Fidelity

Each book is one rigid body: a spine piece whose arc is eight facets, two boards and a page block, four convex
solids, with its mass spread evenly through them. Pages do not flex or fan and the cloth is plain friction: books
0.4, shelf 0.3, mat 0.6, averaged per pair (0.35 book on shelf, 0.4 book on book, 0.5 book on mat). The title bands on
the spines are look-only. Speeds are read frame to frame at 20 Hz, as from video. The simulator scores the last step
without opening the grippers, so a book still held, in the air or within 3 cm of a gripper earns only the tip-out
0.25. A real fingertip is soft and grips the head by friction as well as by the 3 mm lip; the gripper's rigid
fingertip relies on the lip and on bearing down. The row was replayed with contact in MuJoCo before the Isaac run;
that replay is softer than PhysX.

## Files

- `tip_a_recessed_book_off_a_packed_shelf.py`: the task file for the `yam_arena` embodiment (scene, `score()`,
  `success()`, `demo_script()`).
- `mechkit.py`, `closedloop.py`, `belimits.py`: the shared kit (multi-part objects and geometry, the closed-loop
  demo controller, footprint checks), unchanged.
- `task.json`: the task's details.

## Run a trial

On an instance set up by the simulation walkthrough, from this folder:

```bash
INSTR="Take the red book off the shelf and put it on the cork mat. Leave the other books standing on the shelf."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tip_a_recessed_book_off_a_packed_shelf.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/book-dream

# Scripted demo (full score)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tip_a_recessed_book_off_a_packed_shelf.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/book-demo
```

- **Seed and layout:** `-E object_jitter=0` is required: the row is packed to 0.4 mm and the scorer measures from
  the starting layout.
- **Runtime per trial:** the demo needs about 68 s of simulated time (1362 steps); a Dream-1 trial runs up to 2,400 steps
  (2 minutes simulated). Wall-clock time on one L40S, including Isaac Sim start-up: about 340 s for the demo and about 380 s for a full Dream-1 trial.
- **Score:** `task_score` reports 1.0 when `success()` has held for 8 s (the book released and at rest on the mat),
  otherwise `score()` at the end of the run.

## Scoring

Three things are latched over the whole run, from the overhead camera and a side camera (20 frames a second; the
side view shows tilt and the drop speed), a tape and a protractor:

- **T, tipped out:** the red book's head front edge 4 cm or more in front of where it started.
- **V, a neighbour disturbed:** any other book tilted over 35 degrees, its spine foot 3 cm or more in front of its
  start, or off the shelf base.
- **D, dropped:** any corner of the red book moving faster than 0.6 m/s, which is more than 3 cm between two video
  frames. A carry or a controlled set-down stays far below it; a fall from 2 cm or more does not.

At the end both grippers must be 3 cm or more from the red book and it must be at rest (under 1 cm/s for 1 s).
**H, home:** it rests on the cork mat on its tail (within 10 degrees of upright) or on a cover (within 10 degrees of
flat), its lowest point within 5 mm of the mat and all of its outline over the mat.

| Score | Outcome |
|---|---|
| 1.0 | H, no V, no D |
| 0.5 | H with V or D; or, not H, at rest on the desk or mat with no part of it over the shelf, and no V |
| 0.25 | T and none of the above: still held, left leaning on the shelf, put back, on top of the row, off the table |
| 0 | never tipped out |

Doing nothing scores 0. A neighbour dragged out with the red book costs half; so does dropping it. Pushing the row
apart is not possible (end panels and back panel), and a book knocked off the shelf is V.

## Real-world reset

Stand the red book back in its gap with its fore-edge against the back panel and push every book back to the back
panel. Square the row so its 4 mm of slack is shared evenly: a card slips between any two books, a finger does not.
The shelf stays clamped and the mat taped.

## Licence

All geometry is procedural, generated by mechkit.py in this folder; no third-party assets.
