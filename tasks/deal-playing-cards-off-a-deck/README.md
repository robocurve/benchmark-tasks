# Deal a card to each of three places off a deck

A full deck of 52 playing cards lies face down, squared, on a dealing board. Three places are marked on the board
round it: one to the left, one near the robot on the right, one to the right. The instruction is "Deal one card to
each place, in turn: left, then near, then right. Slide each card off the top of the deck, one card at a time, and
leave it face down at its place. Leave the rest of the deck squared on its mark."

The cards are poker size, 63.5 x 88.9 mm, and 0.29 mm thick. The pile is 15 mm high.

## What the task asks of a robot

- **Taking one card off a pile by friction.** A card 0.29 mm thick lying flush on 51 others cannot be gripped, and
  nothing that thin can be pinched off a table either. It is moved by a fingertip pressing on its back. The
  fingertip has to grip the card better than the card grips the one under it, and has to be on the pile's top, not
  against its edge: set 4 mm low against the deck's near edge, it pushed nine cards off together (the `double`
  control run).
- **Pressing with the right force on something 15 mm high.** Too light and the fingertip slides over the card; too
  hard and the pile moves with it. The height of the pile's top changes with every card dealt.
- **Keeping the card under the fingertip over the deck's edge.** As the card leaves the pile it tips down onto the
  board and its tail drops off the deck. The fingertip has to follow it down, and must not press its tail back
  onto the pile, which drags the next card out.
- **Three times, in turn.** The deck is in a known order, so the card found at each place at the end shows whether
  the cards were dealt one at a time and in turn. Two cards that come off together end up at the same place, and
  every place after it then holds the wrong card.
- **Leaving the deck alone.** The 49 cards left must still be on the deck's mark, squared to within 1 cm.

Both arms are used: the left hand deals to the left place, the right hand to the near and right places.

## Files

| File | What it is |
|---|---|
| `task.py` | The scene, the instruction and the scoring. |
| `cardkit.py` | A playing card, the dealing board, and the marks drawn on it. |
| `demo.py` | A scripted demo that reaches full score, and the control runs. |
| `test_scoring.py` | 71 checks of the scoring rule, how cards are sorted, the shapes, both layouts, the task description and the demo's geometry; no simulator needed. |
| `task.json` | The task's description for the board. |

## Running and scoring a trial

```bash
# a policy (Dream-1 here), 1,000 steps at 20 Hz; the score is printed at the end and written to the log
inspect-robots run --embodiment yam_arena --policy dream -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1000 \
  --instruction "Deal one card to each place, in turn: left, then near, then right. Slide each card off the top of the deck, one card at a time, and leave it face down at its place. Leave the rest of the deck squared on its mark."

python demo.py task.py out_dir                  # the scripted demo: videos, setup.png, result.json
CARDS_LOT=B python demo.py task.py out_dir      # the second layout
python -m pytest -q test_scoring.py             # the offline checks
```

- `object_jitter=0` is required: the cards start as a squared pile, each exactly on the one under it.
- There are no seeds. `CARDS_LOT` picks the layout. In both, the deck lies at x 24, y 0 cm with its long sides
  across the table; the places differ.
  - A: places at (33, 22.5) left, (20, -19) near, (34.5, -22.5) right.
  - B: places at (30.5, 20.5) left, (21.5, -17) near, (36, -20.5) right.
- A trial is at most 1,000 steps (50 s). It ends early once full score has held for 1 s. The demo takes 715
  steps (36 s) on layout A and 701 steps (35 s) on layout B. A simulated trial of that length took 15 to 25
  minutes of wall time on a GPU shared with other jobs: 52 thin cards in a pile are slow to solve.
- `result.json` from the demo holds the score, the task's own report at the start and end (cards at each place,
  whether each place's own card is there, cards left on the deck, strays, cards face up, how far the pile reaches
  outside its outline), and a trace.

## Scoring

Scored at the end of the trial, from where the cards lie then and which cards they are.

- A card is **at a place** if its centre is within 7 cm of that place's mark.
- Each place has its **own card**. Counting from the top of the deck at the start: the 1st card is the left
  place's, the 2nd the near place's, the 3rd the right place's. Those are the cards each place gets when the cards
  are dealt one at a time, in turn.
- A place is **complete** if exactly one card is at it and it is its own card, lying flat (within 10 degrees of
  level, no corner more than 5 mm above the board) and face down.
- A card is **still on the deck** if its centre is within 4 cm of the middle of the deck's outline. The deck is
  **square** if no corner of any such card is more than 1 cm outside the outline.

| Score | What it takes |
|---|---|
| 1.0 | All three places complete; every other card still on the deck and the deck square; no card face up or on edge; every card at rest (under 2 cm/s); both grippers at least 3 cm from every card. |
| 0.75 | All three places complete, but one of the other conditions fails. |
| 0.5 | Two places complete. |
| 0.25 | One place complete. |
| 0 | No place holds its own card. |

Other ways to try it, and what they score:

- pushing the whole deck, or a wad of cards, to a place: 0 for that place (more than one card at it);
- two cards off together to the left place, then one each to the near and right places: 0 (the left place has two
  cards, the others have cards that are not their own);
- dealing near, left, right, out of turn: 0.25 (only the right place has its own card);
- all three dealt right but the deck knocked 11 mm off its mark, or a hand left resting on a card: 0.75;
- nothing moved: 0.

## Scoring a real trial

From one photograph at the end, and the last second of the video:

- Use a deck in new-deck order, or put the ace, two and three of one suit on top, ace uppermost.
- At each place, measure from the mark to the middle of the card: within 7 cm it is at the place. Check that it
  lies flat and face down, then turn it over and read it: left the ace, near the two, right the three.
- No card lies anywhere else.
- The deck is square if no card of the pile sticks out more than 1 cm beyond the outline drawn round it.
- Both grippers are at least 3 cm from every card, and nothing moved in the last second.

The task has not been run on a physical rig.

## The demo

`demo.py` is **open loop**. It uses only the layout constants in `task.py`, the sizes in `cardkit.py`, the number
of cards it has dealt so far (which gives the height of the pile's top), and each hand's own measured position (the
robot's proprioception), to correct for the arm's sag. It never reads the state of a card, and it would not notice
a card left behind.

Before dealing, each hand hovers for half a second over every place it will bring a card to, 12 mm above the board,
and notes its own sag there. How hard a hand presses is set by how far below the surface it is commanded, so the
sag has to be known where it presses.

Each card is then dealt the same way:

1. A hand comes over the deck, fingers closed and pointing down, leaning 20 degrees forward, so that the rear edge
   of its fingertips is lowest. It lowers that edge onto the back of the top card, 1 cm in from the card's near
   edge, and is commanded 2 mm into it. (Leaning, the arm gives more under a push than it does pointing straight
   down, so an error in height changes its press less. Pointing straight down and commanded the same 2 mm, it
   pressed 0.26 mm into the pile and took three cards together.)
2. It moves straight away from the robot at 7 cm/s. The card comes with it by friction and runs out over the
   deck's far edge, held flat by the fingertips on its tail.
3. As the fingertips pass the deck's edge the card tips down onto the board. The hand slows to 3.5 cm/s and
   follows it down. When the card's tail leaves the deck it drops flat.
4. The hand lifts 1 cm, sets its fingertips down again on the middle of the card, slowly over the last few
   millimetres, commanded 2 mm into it, and slides it across the board at 12 cm/s to its place, then lifts off.
   The card for the near place is slid sideways first and then drawn back toward the robot, so that it passes
   clear of the deck.

The left hand deals to the left place and the right hand to the near and right places. The hand that is not dealing
waits beside its own place.

Every card is slid sideways or drawn back, never pushed on away from the robot: with the hand leaning forward, a
card pushed on ahead of it across the board was left behind or left short in four trial runs out of four.

| | Layout A | Layout B |
|---|---|---|
| Hands note their sag over the places | steps 46 to 145 | steps 46 to 145 |
| First card: fingertips come down on the deck | step 146 | step 146 |
| First card leaves the deck | step 196 | step 196 |
| First card at the left place: score 0.25 | step 278 | step 274 |
| Second card leaves the deck | step 375 | step 370 |
| Second card at the near place: score 0.5 | step 470 | step 460 |
| Third card leaves the deck | step 565 | step 561 |
| Third card at the right place: score 0.75 | step 644 | step 632 |
| Hands 3 cm clear, cards at rest: score 1.0 | step 670 | step 656 |
| Trial ends with full score | step 715 | step 701 |
| Each card's middle, from its place's mark (left, near, right) | 7, 49 and 1 mm | 6, 21 and 11 mm |
| The pile's farthest corner outside its outline at the end | 2.5 mm | 3.0 mm |

(A card has left the deck when its middle is 4 cm from the middle of the deck's outline. Steps are read from a
trace written every 2 steps.)

The card for the near place stops short of its mark, 49 mm short on layout A: the fingertips slide over it for part
of the way as they draw it back toward the robot. It counts within 7 cm.

## Control runs

All on layout A.

| Run | Score | What happened |
|---|---|---|
| `--mode still` | 0.0 | Nothing moves. The deck stays on its mark. |
| `--mode skip`: a card to the left place, then one to the right place; the near place left out | 0.25 | The left place has its own card. The right place has one card, but it is the deck's second, the near place's own: nothing for it. The near place is empty. The deck is square (0.8 mm). |
| `--mode order`: near place first, then left, then right | 0.25 | The first card stopped 78 mm from the near mark: the fingertips slid over it as they drew it back. The second came off the deck turned 15 degrees with a corner still on the pile; the fingertips lost it 3 cm into its slide, 7 cm from the deck, and the pile was dragged 15 mm askew. The third reached the right place, which in this order is its own. One place complete, two cards astray. |
| `--mode double`: fingertips set against the deck's near edge, 4 mm below its top, and pushed through | 0.0 | Nine cards left the deck together. Eight ended at the right place and one was left on the way, propped on the others; 43 were left on the deck, pushed up to 40 mm outside its outline. |
| the same, 1.2 mm below the top (`CARDS_LOW=0.0012`) | 0.0 | The slanted fingertips rode up onto the pile and pushed the top card 8 mm. No card left the deck. |
| `--mode heap`: the same, 1 cm below the top | 0.0 | 31 cards were pushed off the deck and left scattered beyond it, none within 7 cm of a place's mark. |

The `order` run was meant to show only that cards dealt out of turn end up at the wrong places. It also shows how
closely the scripted demo is tuned to the one sequence it was built for: with the right hand dealing first, two of
the three cards were lost on the way.

No control run turns a card face up: a card slid by a fingertip never leaves the board. That condition is
covered by the offline checks.

## Dream-1

One run, 1,000 steps, layout A: **0.0**. In the first seconds both arms drew back and folded over their own bases
with their grippers open. They stayed there, shifting a little, for the rest of the 50 seconds. Neither reached the
deck, and no card moved.

Two 200-step tests on earlier versions of the task (two rounds of three; then one round with the middle place
straight ahead of the deck) also scored 0.

## Building it for real

- A standard 52-card poker-size deck (63.5 x 88.9 mm cards) with a plain coated finish, in new-deck order or with
  the ace, two and three of one suit on top, ace uppermost.
- A smooth laminate-faced board (melamine-faced hardboard or a laminate offcut), 47 x 66 cm and 3 mm thick, fixed
  to the table with tape at its edges.
- Drawn on the board with a marker: the deck's outline (63.5 x 88.9 mm) and a 3 cm cross at each place, at the
  positions given above for the layout.

Resetting: gather the three dealt cards, put them back on top of the deck in order, square the deck and set it on
its outline, face down.

## What the simulation simplifies

- **The cards are rigid.** A real card bends a little as it tips over the deck's edge; these do not.
- **Sourced:** the card's size, 63.5 x 88.9 mm (a card printer's page for poker-size cards); its thickness,
  0.29 mm, and weight, 1.55 g (a hobbyist's measurements of one deck: 52 cards 15.15 mm thick, ten cards 15.5 g).
- **Estimates, not measurements:** every friction value. The engine gives a pair of bodies the average of their
  two coefficients, and the robot's fingertips have its default, 0.5. The card's coefficient (0.26 static, 0.20
  sliding) and the board's (0.10) are chosen to give: card on card 0.26 and 0.20; fingertip on a card's back 0.38
  and 0.35; card on the board 0.18 and 0.15. No source was found for any of the three.
- **The dealing board is there for its friction.** On the bare table of this simulator a card would be gripped by
  the table exactly as well as by the fingertip (both have the default coefficient), which is not true of rubber
  and of a smooth table top; a card could not be slid on it at all. The board gives the cards a smooth surface with
  a stated friction.
- **Solver settings.** Each card is solved with 255 position iterations, a contact offset of 0.8 mm and a
  depenetration speed limit of 0.5 m/s. With 64 iterations the pile of 52 sank 4 mm into itself and trembled; with
  255 it stands at its full height and still. The cards have their true mass and thickness: no added weight, no
  thickened colliders, and nothing moves a card but contact.
- **The deck's long sides lie across the table in both layouts,** so that a card pushed straight away leaves over
  a long edge after 6 cm. With the deck turned a quarter turn the card has to travel 9 cm on the pile, and in this
  simulator the card under it crept 2.6 mm forward on the way and was caught by the tail of the card being dealt.
- **The marks are drawn only.** Nothing collides with them.
- **The faces.** The top three cards show one, two and three pips and the rest a plain panel; the backs are plain
  red.
- The board is fixed to the table.

## Related tasks on the board

No accepted or pending task deals playing cards, or takes one thin sheet off a stack by friction (board checked
20 November 2026).

- "Play one turn of Jenga: find a loose block, push it through, draw it out and stack it crosswise on top"
  (accepted) is the other familiar table game. It works one block of a tower by pushing and gripping; here nothing
  can be gripped, and the piece is taken by friction from the top of a pile.
- "Shuffle a sorted deck so no colours touch, then deal" was not accepted: it used six coloured blocks, "not a
  realistic dealer shuffle". Here the deck is 52 cards at full size and thickness, and the cards are dealt as a
  dealer deals them.
- "Stand two playing cards by leaning their top edges together" was not accepted: "a stacking puzzle". Here the
  cards are used as cards.

## Sources

- A card printer's page for its poker-size cards (makeplayingcards.com/design/jumbo-index-traditional-size.html),
  through the page fetcher: "Dimensions: 63.5mm x 88.9mm, 2.5" x 3.5"".
- A hobbyist's review of paper playing cards (homepokertourney.org/reviews/cards-review-paper.htm), through the
  page fetcher: "The thickness of all 52 playing cards was 15.15mm." and "Ten cards weighed 15.5 grams."
- Wikipedia, "Standard 52-card deck", through the page fetcher: two makers' poker cards listed at 3.5 x 2.5 in;
  "In a sample of 67 poker-sized cards, the weight of a card varied from 1.4 g to 2.78 g".
- None gives a friction coefficient. Those are estimates (see "What the simulation simplifies").
