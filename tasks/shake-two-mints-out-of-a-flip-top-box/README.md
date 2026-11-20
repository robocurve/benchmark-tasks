# Shake exactly two mints out of a flip-top mint box

A flip-top pocket box of mints lies on the table with its lid shut, part full. Beside it stands a small dish. The
instruction is "Shake two mints out of the mint box into the dish, then close the box and put it back on the table."

The box is the size of a 1 oz retail pack, 47 x 17 x 73 mm. Its mints are half a gram each. They leave through a
12 x 10 mm opening in one corner of the lid end, under a hinged flap that snaps shut.

## What the task asks of a robot

- **A snap lid worked by a fingertip.** The flap is 2 cm long and its lip overhangs the side of the box by 2.5 mm.
  One hand has to hold the box still while a fingertip of the other catches that lip and pushes it past the snap,
  about 0.9 N at the lip; the flap then springs open. Closing it is the reverse: the open flap is swept round and
  pressed until it snaps.
- **Dispensing by shaking.** The opening passes one mint at a time and the mints jam behind it. Tipped right over
  and held still for three seconds, the box let out a single mint (the `pour` control run). Mints come out when the
  box is shaken, one or two at a burst or none, and which burst frees one depends on how the mints happen to lie.
- **Stopping at two.** The count is in the instruction and is the same in both layouts, so nothing has to be worked
  out. What is hard is physical: letting out two half-gram mints and no third. Shaking on after the second mint
  put five in the dish and scored 0 (the `long` control run); stopping one burst early left the dish empty (the
  `short` control run).
- **Nothing on the table.** A mint that misses the dish costs most of the score, so the opening has to be over the
  dish before the box is tipped and while it is shaken.
- **Finishing.** The lid has to be snapped shut and the box put down and let go.

Both arms are used: the left holds the box throughout, the right works the lid.

## Files

| File | What it is |
|---|---|
| `task.py` | The scene, the instruction and the scoring. |
| `mintkit.py` | The box with its opening, the sprung flap on its hinge, the mints, the dish. |
| `demo.py` | A scripted demo that reaches full score, and five control runs. |
| `test_scoring.py` | 43 checks of the scoring rule, the counting, the lid's snap, the shapes, both layouts and the demo's geometry; no simulator needed. |
| `task.json` | The task's description for the board. |

## Running and scoring a trial

```bash
# a policy (Dream-1 here), 1,500 steps at 20 Hz; the score is printed at the end and written to the log
inspect-robots run --embodiment yam_arena --policy dream -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1500 \
  --instruction "Shake two mints out of the mint box into the dish, then close the box and put it back on the table."

python demo.py task.py out_dir                 # the scripted demo: videos, setup.png, result.json
MINT_LOT=B python demo.py task.py out_dir      # the second layout
python -m pytest -q test_scoring.py            # the offline checks
```

- `object_jitter=0` is required: the flap is placed to match the box exactly, and the mints start inside it.
- There are no seeds. `MINT_LOT` picks the layout: A (box at x 30, y 3 cm, 30 mints, dish at x 20, y 19) or B (box at
  x 31.5, y 0, turned 10 degrees, 24 mints, dish at x 21.5, y 16.5).
- A trial is at most 1,500 steps (75 s). It ends early once full score has held for 1 s. The demo takes 708 steps
  (35 s) on layout A and 936 steps (47 s) on layout B; a simulated trial of that length took 8 to 15 minutes of wall
  time on a GPU shared with other jobs, part of it start-up.
- `result.json` from the demo holds the score, the task's own report at the start and end (mints in the dish, in
  the box and spilled, the lid's angle, the step at which each mint left the box), and a trace every 5 steps.

## Scoring

Scored at the end of the trial, from where things are then.

- A mint is **in the box** if its centre is inside the box's outline; **in the dish** if its centre is over the
  dish's floor and below the top of its rim; **spilled** otherwise (on the table, on the rim, on the floor, in the
  air).
- The lid is **shut** within 3 degrees of closed. Left to itself the flap is either shut or wide open: below 14
  degrees it snaps shut, above it springs open.
- The box is **down** when its lowest point is within 3 mm of the table, no part of it is over the dish, it is at
  rest (under 2 cm/s and 0.5 rad/s) and both grippers are at least 3 cm from it.

| Score | What it takes |
|---|---|
| 1.0 | Exactly two mints in the dish, none spilled, lid shut, box down. |
| 0.5 | Exactly two in the dish and none spilled, but the lid is open or the box is not down. Or: one or three in the dish, none spilled, lid shut, box down. |
| 0.25 | One or three in the dish and none spilled, with the lid open or the box not down. Or: exactly two in the dish and at least one mint spilled. |
| 0 | No mint in the dish, or four or more. Or: a mint spilled and not exactly two in the dish. |

Other ways to try it, and what they score: tipping the whole box out over the dish, 0 (four or more in the dish);
leaving the open box standing in the dish, at most 0.5 (the box is not down); two mints in the dish and the rest
scattered, 0.25; putting the box down unopened, 0.

## Scoring a real trial

From one photograph at the end, and the last second of the video:

- Count the mints in the dish. Look for any mint on the table, on the dish's rim or on the floor.
- The lid is shut if the flap lies flush with the top end of the box. A flap that is not snapped shut stands wide
  open.
- The box is down if it lies or stands on the table, clear of the dish, with both grippers at least 3 cm away,
  and did not move in the last second.

The mints still in the box need not be counted. The task has not been run on a physical rig.

## The demo

`demo.py` is **open loop**. The strokes that shake the mints out were tuned by trial in this simulator, separately
for each layout, until exactly two came out; the script does not look at the dish and would not notice a third
mint. It shows that a full score is within the arms' reach and speed. A real trial, a different fill or a
different simulator needs the dish watched and the shaking stopped at the second mint.

It uses only the layout constants in `task.py`, the sizes in `mintkit.py`, and each hand's own measured position
and direction (the robot's proprioception), to correct for the arm's sag. It never reads the state of the box, the
lid or a mint.

1. The left hand comes straight down over the lying box, pads 6.6 cm apart, 8 mm toward the box's bottom end from
   its middle, and closes on its 4.7 cm width with about 11 N a pad.
2. The right hand leans 55 degrees in from the right. A corner of its closed fingertips is laid against the box's
   side, 8 mm short of the lid's lip and 14 mm above the table, bearing lightly on the side, and is drawn 2.7 cm
   toward the lid end. It catches the lip and pushes the flap past the snap; the flap springs open to 110 degrees.
3. The left hand lifts the box 8 cm, carries it to the dish and lowers it until the opening is 4.8 cm above the
   table over the middle of the dish, 2.3 cm above the rim. Then the strokes, with the opening kept over that
   point:
   - Layout A: it leans forward until the lid end is 55 degrees below level, and gives two bursts of three shakes
     along the box's length, 15 mm each way, 0.3 s a shake, 0.6 s between bursts. Both mints leave in the second
     burst, 0.15 s apart.
   - Layout B: it rocks the box five times between 60 and 15 degrees (one mint leaves on the fifth rock), then holds
     it at 55 degrees and gives five bursts of three 20 mm shakes (the second mint leaves in the fifth burst).
   It levels the box again.
4. It carries the box back, lays it on its mark and keeps hold of it. The right hand, now leaning 50 degrees
   forward behind the lid end, sweeps its fingertips across 9 mm beyond the lid end, which turns the open flap
   most of the way round, then presses toward the box until the flap snaps shut.
5. Both hands let go and draw away.

| | Layout A | Layout B |
|---|---|---|
| Left hand closed on the box | step 115 | step 115 |
| Flap past the snap and wide open | step 205 | step 205 |
| Opening over the dish, first stroke begins | step 330 | step 330 |
| First mint leaves the box | step 405 | step 469 |
| Second mint leaves the box | step 408 | step 623 |
| Box back on its mark, lid closing begins | step 540 | step 770 |
| Flap snapped shut | step 625 | step 845 |
| Hands clear, box at rest: score 1.0 | step 670 | step 900 |
| Trial ends with full score | step 708 | step 936 |
| Mints in the dish, in the box, spilled at the end | 2, 28, 0 | 2, 22, 0 |
| Lid end below level during the strokes, at most | 56 degrees | 59 degrees |

(Steps of leaving the box are exact; the others are read from a trace written every 5 steps.)

The same strokes gave the same result each time they were run: layout A three times (mints leaving at steps 405
and 408 each time), layout B twice (steps 469 and 623; one of the two runs went on for five more bursts, which freed
nothing). The strokes are tuned per layout because the same strokes do different things with a different fill:
layout A's strokes let out its two mints in one burst and, used on layout B, let out none in twelve.

## Control runs

All on layout A.

| Run | Score | What happened |
|---|---|---|
| `--mode still` | 0.0 | Nothing moves. The box stays shut on its mark. |
| `--mode open`: the full demo, but the lid is left open | 0.5 | Two mints in the dish, as in the demo, and the box put down and let go; the flap stands open at 110 degrees. Half of the full score. |
| `--mode short`: the demo's strokes without the second burst | 0.0 | No mint leaves the box. The lid is opened and shut again and the box put down: the dish is empty. |
| `--mode pour`: the box tipped until its lid end is 75 degrees below level and held still for 3 s | 0.5 | One mint leaves while the box is being tipped (step 362); none in the three seconds it is held. One in the dish, lid shut, box down. |
| `--mode long`: the demo's strokes, then four bursts of six shakes through the box's thickness, 8 mm each way | 0.0 | Two mints leave in the second burst, as in the demo (steps 405 and 408); two more in the first of the added bursts (steps 451 and 452); a fifth as the box is levelled (step 592). Five in the dish. |

## Dream-1

One run, 1,500 steps, layout A (after a 200-step test, which also scored 0): **0.0**. In the first few seconds both
arms folded down, each with its gripper closed and held low over the table in front of its own base. They stayed
there, drifting a little, for the rest of the 75 seconds. Neither gripper reached the box. At the end the box lay
where it had started and the dish was empty.

## Building it for real

- A 1 oz (60-count) flip-top box of small hard mints, 47 x 17 x 73 mm, with the label peeled off. Count out all
  but 30 mints (24 for layout B), shut the lid, shake the box level so that the mints lie in one layer, and lay it
  on its mark, lid end toward the robot, with the corner the opening is in on the robot's right.
- A square stoneware or glass dish about 11 cm inside and 2.5 cm deep, fixed to the table with putty.
- Tape marks for the box and the dish in the two layouts.

Resetting: tip the mints from the dish back into the box, or top it up to the count; shut the lid; lay the box on
its mark. Mints that have been on the table should not be eaten.

## What the simulation simplifies

- Everything is rigid. A mint is a capsule.
- **Sourced:** the pack's outside size, 47 x 17 x 73 mm (a retailer's listing of the 1 oz pack: about
  1.85 x 0.67 x 2.87 in); 60 mints in a 1 oz pack and 0.5 g a mint (the maker's page).
- **Estimates, not measurements:** the mint's size (11.5 x 6.5 mm; with the labelled 0.5 g that is the density of
  sugar, 1.6 g/cm3); the opening (12 x 10 mm); the flap (22.5 x 15 x 1.5 mm, hinged 20 mm from the side, opening to
  110 degrees) and its 2.5 mm lip; the walls (1 mm) and the plate the opening is cut in (2 mm); the empty box's
  9 g; friction (0.35 for a mint, 0.4 for the box's plastic, 0.9 between the box and a gripper pad) and bounce
  (0.2). No maker publishes these and none was measured.
- **The snap** is a torque law on the hinge, applied once per control step, not a moulded bead: 0.020 N m toward
  shut below 8 degrees (0.9 N at the lip), 0.005 N m toward open above 16 degrees, changing over in between, with
  hinge damping of 0.001 N m s/rad. All four numbers are estimates. A real flap is held shut by a small plug in the
  opening and held open by its plastic hinge; the model has no plug.
- The flap weighs 2 g, about four times a real one, so that the contact solver holds it against a fingertip.
- The box weighs 9 g but has the inertia of a 60 g body: a constant upward force carries the other 51 g. Squeezed
  between the fingers at its real mass, the shell was not held by the contact solver (the fingers sank into it).
- Grippers, the table and the dish meet the box as a solid block of its outline; only the mints collide with its
  thin walls from inside. The pads meet the box's sides over a patch of 8 mm radius (an estimate for rubber pads),
  which is what keeps the box from turning in a two-finger hold.
- The box is drawn as tinted clear plastic without a label.
- The dish is fixed to the table.
- The mints start in one neat layer. In a real box they lie as the last shake left them.

## Related tasks on the board

No accepted or pending task has a mint box, and none dispenses a count of loose pieces by shaking (board checked
19 November 2026).

- "Dispense one serving of cereal from a twist-knob dispenser into a bowl" (accepted) also stops a flow of loose
  pieces at an amount. Its flow is metered by a knob on a fixed dispenser and judged by weight within 20 percent;
  here the container is hand-held, the flow is started by shaking, and the count is exact.
- "Measure powdered formula into a 4-ounce bottle: two unpacked scoops, each leveled against the tin's leveling
  bar" (accepted) measures a dose with a tool that fixes the amount per scoop. Here nothing meters the amount but
  the shake.
- "Put five quarters in a piggy bank: pinch each flat coin off the table and turn it on edge to fit the slot"
  (accepted) moves a count of small pieces one at a time, each one grasped. Mints are never grasped here; they
  leave a held container through its own opening.
- "Fill Monday and Thursday in a snap-lid weekly pill organizer, holding the strip while each lid is lifted and
  snapped" (accepted) has snap lids worked while the other hand holds the container, and pieces placed by count.
  Its caplets are picked and placed; here the pieces are shaken out of a box they cannot be picked from.
- "Count ten tablets out of a stock bottle on a pill counting tray" (pending) is the nearest in purpose: a count of
  small pieces out of a container. There the tablets are poured onto a tray and the count is made on the tray
  with a spatula. Here the count is made at the opening, by how the box is shaken.
- "Shake exactly 3 tablets out of a pill bottle" was not accepted: its tablets went onto a bare table, and its
  scorer gave credit when the bottle was emptied. Here the container is the pocket mint box people shake every day,
  the mints go into a dish, a mint that misses the dish costs most of the score, and four or more in the dish
  score 0.
- "Buy a gumball from a coin gumball machine" and "Untwist and dispense a wrapped candy" (both pending) are the
  other tasks with sweets; one works a coin mechanism and the other a twisted wrapper.

## Sources

- The maker's product page for its fresh-mint flavour (tictac.com/us/en/product/freshmints), read directly:
  "Serving Size 1 mint (0.5g)"; pack sizes include "1 oz 60 Tic Tac® Mints".
- A retailer's listing of the 1 oz pack (groceryeshop.us/products/tic-tac-freshmint-1-oz): "The package measures
  one point eight five inches in length, zero point six seven inches in width, and two point eight seven inches in
  height."

- Neither gives the size of a mint, of the opening or of the flap, or the force of the snap. Those are estimates
  (see "What the simulation simplifies").
