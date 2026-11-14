# Check a small engine's oil: pull, wipe, reseat, pull again to read, and report the level

A bench model of a small four-stroke engine (the kind on a lawn mower or a portable generator) stands on the table
with its yellow dipstick seated in the oil fill tube. A folded blue shop rag lies beside it, next to an oil check card
with ADD, OK and OVER pockets and a white marker puck. "Check the oil level on the engine with the dipstick, mark it
on the check card with the puck, then put everything back." means the procedure every small-engine and quick-lube
technician follows: pull the dipstick, wipe the blade clean on the rag, push it back in until the cap seats, pull it
again and hold it still to read the level, push it back in until it seats, put the rag down, and record what the
blade showed. The level changes from trial to trial (below ADD, between the marks, or above FULL), so the puck lands
in the right pocket only if the blade was read. The tempting wrong answer is to read the first pull, which shows a
false smeared level past FULL from oil splashed up the tube.

## Spec

### Scene (cm from the midpoint between the arm bases, x forward, y to the robot's left, table top z = 0)

| Part | What it is | Where |
|---|---|---|
| Engine | Fixed bench model: wooden mounting board (16 x 15 x 0.8 cm), aluminum crankcase and cylinder block (10 x 12 x 6 cm) with a red flywheel shroud on top (z 7 to 10), and a fill boss on the crankcase's front | board x 22 to 38, y -22 to -7 |
| Oil fill tube | Yellow tube, 2.4 cm across with a 2.0 cm bore, rising from the fill boss to its mouth at z 8.5 | axis at x 25, y -12 |
| Dipstick | Prismatic link sliding along the tube's axis: steel blade 1.0 x 0.3 x 8 cm with black ADD and FULL marks 0.8 and 2.0 cm from the tip and a crosshatch between them, a yellow cap disc 3.2 cm across and 1 cm tall that seats on the tube's rim, and a yellow T-handle 3.5 cm wide, 1.8 cm thick, 3.5 cm tall. 12.5 cm overall, 40 g. Starts seated; it falls back down the tube if let go | cap on the rim at z 8.5 |
| Shop rag | Folded blue shop towel pad, 4 x 4 x 2.5 cm, 40 g, loose | centered x 26, y 10 |
| Oil check card | Dark gray tray 6 x 16 x 1 cm, fixed: three pockets 4.4 cm square and 5 mm deep, 0.8 cm walls between them. Each floor carries a white symbol (2.8 cm, 6 mm strokes) so the pockets read without color: ADD amber with a minus sign, OK green with a check mark, OVER red with a plus sign | centered x 35, y 10; pockets at y 15.2 (ADD), 10 (OK), 4.8 (OVER) |
| Marker puck | White puck, 3 cm across, 2 cm tall, 15 g, loose | x 20, y 15 |

The blade carries black ADD and FULL marks 0.8 and 2.0 cm from the tip (2 mm proud of its edges) and a drawn oil
film (visual only): at the start, a ragged smear over its lower 6 cm (the false first reading); the smear disappears
wherever the rag passes; after a wipe and a full seat, the blade shows a clean oil band from the tip up to the
trial's level, which is one of three:

| Level (`OIL_CASE`) | Oil band from the tip | Reads | Pocket |
|---|---|---|---|
| `low` | 0.4 cm, 4 mm under the ADD mark | below ADD | ADD (minus sign) |
| `ok` | 1.4 cm, 6 mm from either mark | between the marks | OK (check mark) |
| `over` | 2.7 cm, 7 mm above the FULL mark | above FULL | OVER (plus sign) |

Each level sits 4 mm or more from a mark, so it reads the same by eye in any camera that sees the blade. The
seeded reset picks the level: every free object is jittered by up to 3 cm, and the rag's offset from its place in
whole millimetres (x + y), modulo 3, picks low, ok or over (the rag's offset is noise, not a cue a camera could use).
With no jitter (`object_jitter=0`) the level is between the marks, and the environment variable `OIL_CASE` sets it
for a run. The band appears while the blade is inside the tube (after the full seat), so it never pops in on camera.

### Instruction

"Check the oil level on the engine with the dipstick, mark it on the check card with the puck, then put everything
back." The wipe-and-reread rule, the full seat and the closing seat are not stated. The instruction names the card
and the puck because a robot cannot speak its reading; the card stands in for the inspection checklist.

### Rule and why

Engine manuals (Briggs & Stratton, Honda, Kohler) and quick-lube procedures say to remove the dipstick, wipe it
clean, reinsert it fully and remove it again to read. Oil splashed up the fill tube while the engine ran (or while it
was carried) coats the blade above the true level, so a first-pull reading overstates the oil and a low engine gets
run. A dipstick pushed only part way in before the second pull reads low or picks up the tube's splash again. The
dipstick goes back in until its cap seats, so dirt cannot get into the crankcase, and the rag goes back down. The
level found goes on the shop's inspection checklist (quick-lube checklists have ADD / OK / OVER boxes), because it
decides the next job: top up a low engine, or drain an overfilled one.

Category: Impact on jobs. Automotive service technicians and mechanics (BLS Occupational Outlook Handbook, SOC
49-3023, about 826,000 jobs) check the oil with a dipstick on every oil service and inspection, and small engine
mechanics (SOC 49-3053) do the same on mowers and generators; the push-in dipstick and the wipe-and-reread rule are the
same on both.

### Scoring (thresholds and reasons)

Everything is scored from object states: the dipstick's joint position q (how far the cap is above its seat), the
rag's and the puck's positions and bounding boxes, the grippers' poses and finger openings (for contact with their own
shape), and the trial's level.

- Out = q at least the blade's length plus 5 mm (the tip clear of the mouth).
- Seated = q of 3 mm or less (a 3 mm lift of the cap is visible against the rim).
- Held = a gripper with both fingers touching the T-handle (contact, as for let go below) and their opening stopped by
  it, across its 1.8 cm thickness or its 3.5 cm width (0.10 to 0.40 of full opening: the 1.8 cm less 15 %, to the
  3.5 cm plus 3 %). An empty fist, an open hand round the handle, one finger on it or a hand hovering over it does not
  hold it, so a dipstick that falls back into the tube beside a hand has not been pushed home by hand.
- Wipe = while the blade is out and still smeared, the rag touches it when its middle is within 2.7 cm of the blade's
  axis (half the pad, 2 cm, plus the blade's half-thickness and about 0.5 cm of cloth pressed round it; a rag hovering
  clear sits farther out), its outline is within 2 mm of the blade's faces (a rigid pad held 5 mm clear does not
  count) and its height overlaps the blade above the mouth. The scorer follows the rag's middle along the blade in
  1 mm bins from the tip and marks the stretch it slides over between two touching steps, at most 5 mm a step;
  nothing is credited across a step without contact, so a lift and a touch elsewhere adds nothing. The blade is wiped once the middle has slid over 80% (3.2 cm) of the blade's lower 4 cm in that pull: the pad
  reaches 2 cm either side of its middle, so a stroke between 4 cm and the tip drags cloth over the whole 6 cm smear
  and the crosshatch, and 80% lets a stroke start or stop 8 mm short (the arm lags its target about 1.6 cm).
  Touching is not wiping: a dab, or dabs at several heights, slide over nothing, and half a stroke (2 cm) leaves the
  marks smeared. Strokes add up within one pull, so two half strokes that together cover the span count, since a
  mechanic who wipes in two passes also ends with a clean blade; the tip going back into the tube clears the count.
  Either hand may move: drawing the rag along a blade held still is how most mechanics do it.
- Reseat = the wiped blade pushed back in, by hand (held), until seated. A dipstick dropped back in, or seated without
  a wipe, starts the procedure again. A wiped blade that goes back into the tube part way and comes out again is
  smeared again and needs a new wipe.
- Reading = after the reseat, the dipstick pulled out again and held, out and still (moving under 1 cm/s), for 1 s.
  The dipstick stays on the tube's axis, so it is read tip down, as manuals advise (tipping it up runs oil up the
  blade).
- Report = the puck down in a pocket. The robot cannot speak its reading, so it marks it the way a mechanic ticks a
  checklist box. Down in a pocket means: its middle within 1.2 cm of the pocket's middle along x and y (a puck lying
  flat in the 4.4 cm pocket can be at most 7 mm off; one hanging on a wall with its middle outside does not count)
  and its bottom at least 2 mm below the card's top (in the 5 mm pocket, not lying on the rim or on a wall between
  two pockets). The report counts at the end, with the puck at rest and let go (no gripper touching it),
  because a puck still in the hand has not marked anything. The pocket it ends in is the report; moving it after
  the reading is allowed, as a mechanic may correct a checklist. A real trial reads the pocket off a top photo and
  checks from the side that the puck sits below the card's top.
- Guess = the puck down in any pocket for 2 control steps (0.1 s) in a row before the reading has been taken: the
  level was not known yet, so whatever pocket it went into was chosen without looking. A guess scores 0 even if the
  puck later ends in the right pocket. One step is allowed for a physics pop. A real trial compares the frame the puck
  goes in with the end of the second pull's 1 s hold.
- Closed up = after the reading, the dipstick pushed back in by hand until seated, and let go. Rag down = the rag at
  rest on the table (its bottom 1 cm or less above it) and let go.
- Let go = no gripper touching the object, judged by contact with the gripper's own shape (its palm and both fingers
  at their present opening, as boxes fitted to its collision meshes): touching means a gap of 2 mm or less, the
  solver's resting gap plus the fit of the boxes to the meshes. A hand
  hovering beside the object, however close, has let go; fingers still closed on it have not.

| Outcome | Score | Why it is worth that |
|---|---|---|
| A wrong report (the puck left in another level's pocket), or a guess (the puck in a pocket before the reading), whatever else was done | 0 | A wrong level on the checklist sends the next job the wrong way (oil added to a full engine, or a low one left running), and a wrong report must never pay more than none |
| Nothing done, or a first-pull reading (pulled, read, put back) without a wipe, or a wipe never followed by putting the dipstick back | 0 | No true reading is possible without the wipe and a full reseat |
| Wiped and seated by hand, but the second pull never held out still for 1 s | 0.25 | The procedure that makes a true reading possible is done; the reading is not |
| True reading, but no report (the puck not in a pocket, on a wall or rim, or still held) | 0.5 | The level was looked at but not recorded, so nobody downstream can act on it |
| The right report, but the dipstick left out, part way in, dropped back or still held, or the rag not put down | 0.75 | The check is done and recorded, but the engine is left open to dirt or the bench is not put back as the instruction asks |
| All of it (where the arms end up is not scored) | 1.0 | |

Possible scores: 0, 0.25, 0.5, 0.75, 1.0. A wrong report scores below no report, so guessing never pays: a policy
that always puts the puck in one pocket averages 1/3 over the three levels (1.0 once, 0 twice), below the 0.5 for
reading and not reporting, and a report taken from the smeared first pull (always past FULL) is wrong two times in
three. A real trial is checked from the video: count the pulls, check the rag is drawn along the lower blade (over
at least 3.2 cm of its lower 4 cm) between them, look for the gap under the cap after each push (a 3 mm feeler
strip), time the second pull's hold, note the frame the puck goes into a pocket and which pocket it ends in, and
check the rag is back on the table.

### Self-QC (each case is a test in `test_task.py`)

Do nothing (0); first pull read and put back (0); wipe skipped, pull-reseat-pull-read (0); rag held 6 cm off the blade
(0); 3 cm wipe (0); a dab, rag held still on the blade (0); dabs at 4, 2.5 and 0.5 cm, lifted between (0); half
the length, 4 to 2 cm (0), and scrubbed back and forth over that half (0); touch at the top, lift, touch at the bottom
(0); half a stroke, the tip back in the tube, then the other half (0); a full stroke down, up, or stopping 6 mm short
of the tip (1.0); two half strokes covering the span (1.0); a full stroke with a 2-step contact gap (1.0); rag rubbed on the cap with the dipstick seated (0); half-seated reinsert before the second pull
(0); second pull not held still 1 s (0.25); dropped back in after the wipe (0); left out, left half in, still
held at the end with the right report (0.75); rag still in hand (0.75); arms left away from home (1.0: where the arms end is not scored); dipstick let
fall to its seat beside an empty closed fist (0) and the same push with the handle gripped (1.0) or gripped across
its width (1.0); reseated with an open hand round the handle (0), with one finger on it (0) or by a closed hand
hovering 5 mm over it (0); second pull
"held" by an empty fist (0.25), then a report (0, a guess); rag drawn down the still blade by the other arm
(1.0, a real wipe); half-dipped, then re-wiped and done right (1.0); history reset by a new trial (0).
The report, at each level: the right pocket (1.0, and success); each wrong pocket (0); no report (0.5); the right
report with the rag in hand (0.75); at the low level, the first pull read and OVER reported (0); the puck in the right
pocket but not let go (0.5); let go with the open hand hovering 1 cm over it, not touching (1.0), then closed on it again (0.5); straddling the wall between ADD and OK (0.5); on the right pocket's rim (0.5); a wrong
pocket after the reading, then moved to the right one (1.0); the right pocket, then moved to a wrong one (0); the right
pocket before the second pull (0); dipped in a pocket before the reading, taken out, then reported right (0); a
one-step pop into a pocket before the reading, then done right (1.0); the right pocket without a wipe (0); a wrong
report with the dipstick left out (0); always guessing one pocket averages 1/3; with no jitter the level is between
the marks and seeded jitter spreads the three levels about evenly; the demo aims for the pocket of the level it is
given; the drawn band follows the level.

## How it differs from nearby tasks

- Checking a pressure gauge before opening a filter housing, or a fire extinguisher's pin and gauge, reads an
  instrument once and then chooses an action. Here the reading is only valid after a procedure: a
  two-pull reading in which the first reading must be discarded, and the report is scored on that second reading.
- Hand-pumping to a dial's pressure is a repeated stroke against a gauge, with no wipe or reseat.
- Cleaning a fiber connector's end face and then plugging it in is a single wipe-then-insert. Here the wipe sits between
  two pulls and is followed by a full reseat, a second pull held still, and a closing seat.
- Our kettle task (fill between the MIN and MAX lines) scores a level set by pouring; here the level is not changed,
  only read by the right procedure.

## Files

| File | What it is |
|---|---|
| `task.py` | Scene, scorer (`score()`, `success()`) and scripted demo (`demo()`), for the `yam_arena` embodiment (Isaac Lab-Arena, bimanual YAM, `station` scene). Its docstrings explain every rule. |
| `yamkit.py` | Helper that spawns the engine as an articulation (the sliding dipstick), the rag, the card and the puck, gives the scorer the joint state and the grippers, and adds tilted demo waypoints. It patches `yam_arena` in memory at import; no files change. |
| `guardkit.py` | Contact with the gripper's own shape (palm and fingers at their present opening): the let-go checks use its `touching()`, the held check its `gripping()` (both fingers on the handle, the opening stopped by it). Trimmed to these two checks. |
| `test_task.py` | Offline scorer checks (the self-QC cases above). No simulator needed. |
| `task.json` | The task card. |

## Run a trial

Scripted demo (reaches full score; the level is between the marks with no jitter, and `OIL_CASE=low` or
`OIL_CASE=over` in the environment runs the other two levels):

    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=tasks/oil-dipstick-wipe-reinsert-read/task.py \
      -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "Check the oil level on the engine with the dipstick, mark it on the check card with the puck, then put everything back." \
      --scorer task_score --max-steps 1500

Model run (Dream-1): the same command with `--policy dream --max-steps 2400`, run at all three levels (`OIL_CASE=low`, `ok`
and `over`, or seeded jitter), since at one fixed level always marking the same pocket would pass. Videos: `inspect-robots video <log.json>`.
Scorer tests: `python tasks/oil-dipstick-wipe-reinsert-read/test_task.py` (prints `all pass`).

## The demo

1. Right arm: grips the dipstick's handle from above and pulls it until the tip is 3 cm clear of the mouth.
2. Left arm: picks up the rag, rolls its wrist about 75 degrees so the gripper reaches in from the left side (away
   from the right hand) holding the pad on edge, presses the pad's broad face onto the blade beside the tip and
   draws it up the blade over 3 s until its middle is about 4 cm above the tip, then takes it away. (Drawn downward,
   the tilted pad's leading edge dug into the blade and stalled after about 1 cm.)
3. Right arm: pushes the dipstick back in until the cap seats, pulls it again (tip 2 cm clear) and holds it still
   2.5 s (the reading), pushes it back in until it seats and lets go.
4. Left arm: puts the rag back down where it was, then, while the right arm holds the second pull, rolls its wrist as
   for the wipe and stops 5 cm beside the blade, so its wrist camera shows the oil band against the ADD and FULL marks.
5. Left arm: picks up the puck, waits over the pocket until the reading has been taken, sets the puck in the pocket
   and lets go. Both arms go home and hold.

The demo is a scripted demonstration, not a policy: it is told the trial's level (`OIL_CASE`, or between the marks
with no jitter) and aims for that pocket; it does not read the blade. The scored policy is told nothing and has to
read the oil band off the blade against the ADD and FULL marks. The demo shows that every motion, including the report, can be done within the run
limit, and its left wrist camera shows that the level can be read from the robot's own view. The top camera, about
55 cm away, sees the blade edge-on as a line about 2 pixels wide at 320 x 176, too small to read; a policy has to
bring a wrist camera to the blade during the second pull and read the level itself. A demo that reads the level from
its own camera would be future work; it is not needed to show the task can be done.

## Runtime and seeds

- Demo: 589 control steps (29.5 s simulated) at each of the three levels.
- Model run limit: 2,400 steps (2 minutes at 20 Hz).
- One fixed layout (`object_jitter=0`); the oil level is the one thing that changes between trials. Score a model on
  all three levels (`OIL_CASE=low`, `ok`, `over`), or on seeded resets, which also pick the level.

## Fidelity

The dipstick is real size for a mower or generator engine (12.5 cm overall); a longer one cannot be pulled clear by
arms that lift about 22 cm above the table. It slides on the tube's axis even when out of the tube (a guide), so it
cannot be laid down or tilted; it falls back down the tube if let go. The guide means the sim never asks the robot to
line the tip up with the tube's 2.0 cm bore on the way back in (the blade, 1.0 x 0.3 cm, has about 5 mm of play each
side). A real trial uses a free dipstick, so it does test that reinsertion; a dipstick dropped on the bench there is
left out (0.75 after a right report), which matches what the scorer gives a dipstick let fall down the guide
without a hand pushing it. The oil is drawn, not simulated: the smear,
its removal by the rag and the trial's level are visuals driven by the scorer's state. The rag is a rigid pad.

## Real-world build

A small-engine bench model or a real mower or generator engine (cold) with a push-in dipstick and fill tube, and a
folded shop towel. Before each trial, roll a die for the level (1-2 below ADD, 3-4 between the marks, 5-6 above
FULL), set the oil to 0.4, 1.4 or 2.7 cm on the blade with a syringe (check it with a pull and a wipe, out of the
camera's view), wipe the blade, smear oil on its lower 6 cm and up the inside of the tube, and seat it. The check card
is a tray of two layers of 5 mm foam board (or a 3D print), 6 x 16 cm, with three 4.4 cm square holes cut in the top
layer, the floors painted amber, green and red, labelled ADD, OK and OVER and marked with the same white minus sign, check
mark and plus sign as the sim, taped to the bench at x 35, y 10. The
puck is a white checkers piece glued to a second one (3 cm across, about 2 cm tall). Score from the video as described
under Scoring: the level a real trial shows on the blade is the oil itself, and the report is which pocket the puck
ends in.

## Licenses

All code here was written for this task; no third-party code or assets are included. Every shape is generated by
`yamkit.py`.
