# Play a card from a hand of cards

The robot starts holding a hand of five playing cards in its left gripper, fanned and pinched together at one corner:
an ace, a two, a six, a four and a five, the ace in front. The instruction is "Play the ace from the hand of cards you
are holding: take it out of the hand and lay it face up on the blue cross. Keep the other four cards in your hand."

## Why it matters

Playing a card from your hand is the basic move of nearly every card game: in trick-taking games (whist, bridge,
hearts, spades), rummy and its relatives, and poker draws, a player holds the hand fanned in one hand and draws one
chosen card out with the other, while the rest of the hand stays held. Anyone who has played cards has done it, and
anyone watching a few seconds of it knows what is happening: it falls under societal relatability.

What it tests is **selective extraction from a stack the robot is holding itself**: one thin sheet taken out of its
own grip without losing the others. That is not what existing tasks ask. Taking a card or sheet off a pile lying on
a table (dealing, feeding paper) moves the top sheet by friction against a pile that stays put on its own; picking
an object out of a bin or a rack grasps something that nothing is holding. Here the same robot is both holding the
stack and taking from it: the holding hand has to relax its grip just enough for one card to slide, while the other
hand draws it, and close again before the rest fall. That coordination of two hands' forces on one stack is the
skill, and we found no task that asks for it in the known suites (LIBERO, RoboCasa, SimplerEnv, BEHAVIOR,
DataCenterBench, RoboDojo) or among the accepted tasks here.

## What the task asks of a robot

- **Easing a grip, and no more.** The left gripper presses the five cards together, so the ace is held between the
  pad and the card behind it. A pinch of one 0.29 mm card cannot pull it out of a firm grip (in this simulation the
  left hand closed on the cards squeezes them with about 3 N; a pinch of one card gets 0.3 to 0.7 N). The left hand
  has to relax enough for the ace to slide, and no more: open it too far and the other four fall out of the hand. In
  the demo the window is narrow: easing by 0.13 mm a side works; 0.10 mm lets the hand settle loosely (0.75), and
  0.16 mm or 0.20 mm still holds the ace too hard to draw (0).
- **Pinching one thin card in a stack.** The right hand can close on the ace only where no other card lies behind
  it, and its pads, a few millimetres from the next card's edge, must not touch that card on the way out.
- **Drawing it out without disturbing the rest**, then closing the left hand again on the four cards left.
- **Laying a card face up** where it is wanted: the ace has to land flat, face up, within 5 cm of the cross.

## Files

| File | What it is |
|---|---|
| `task.py` | The scene, the starting hold, the instruction and the scoring. |
| `cardkit.py` | A playing card and a tape mark, and the hooks that set the starting state (adapted from the card kit of the accepted card-dealing task, TK-ZPAQZ4). |
| `demo.py` | The scripted demo, its planner, and the control runs. |
| `test_scoring.py` | 30 offline checks: the scoring rule, how cards are judged in and out of the hand, the fan's geometry, the demo's plan, and why the played card is the front one. |
| `fan_template.svg` | A 1:1 template (A4) for laying out the hand of cards for a real trial. |
| `task.json` | The task's description for the board. |

## Running and scoring a trial

```bash
# a policy (Dream-1 here), 1,000 steps at 20 Hz; the score is printed at the end and written to the log
inspect-robots run --embodiment yam_arena --policy dream -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1000 \
  --instruction "Play the ace from the hand of cards you are holding: take it out of the hand and lay it face up on the blue cross. Keep the other four cards in your hand."

python demo.py task.py out_dir                     # the scripted demo: videos, setup.png, result.json
python demo.py task.py out_dir --mode tight        # control runs: --mode hold | tight | loose
python -m pytest -q test_scoring.py                # the offline checks (needs yam_arena importable)
```

- `object_jitter=0` is required: the cards start in the gripper, each exactly where the fan puts it.
- There are no seeds and one layout.
- A trial is at most 1,000 steps (50 s). It ends early once full score has held for 1 s. The demo takes 470 steps
  (23.5 s). One simulated trial takes about 5 minutes of wall time on an L40S, longer when several share the GPU.
- The trial starts with the hand of cards already held: at every reset, `cardkit.py`'s `on_start` hook puts the
  left arm at its holding pose with its fingers on the five cards, places the cards in the fan between them, and
  holds for 0.5 s with the gripper closed, before the policy's first action (as an operator hands the robot its
  cards). Nothing is moved by the code after that.

## Scoring

Scored at the end of the trial, from where the cards are; one condition covers the whole trial.

- A card is **in the hand** if it is held between the left gripper's pads (within 12 mm of them) and no part of it
  is lower than 3 cm above the table.
- The ace is **out of the hand** when it is more than 3 cm from the left pads; it **lies on the table** if no corner
  of it is more than 5 mm up; **flat** within 10 degrees of level; **face up** with its face up.

| Score | What it takes |
|---|---|
| 0 | Any of the other four cards left the hand at any time, or the ace is still in the hand. |
| 0.25 | The ace is out of the hand, not lying on the table (still held, or dropped elsewhere). |
| 0.5 | The ace lies on the table, but face down, tilted more than 10 degrees, or more than 10 cm from the cross's centre. |
| 0.75 | The ace lies face up and flat, its centre within 10 cm of the cross's centre. |
| 1.0 | The ace lies face up and flat within 5 cm of the cross's centre, the right gripper is at least 3 cm from it, and every card is at rest (under 2 cm/s). |

Other ways to try it, and what they score: opening the left hand and dropping the whole hand: 0, however well the
ace is laid (the `loose` control); pulling without easing the grip: the ace stays, 0 (`tight`); drawing the ace
together with the card behind it: 0 (that card leaves the hand); taking the whole fan out with the right hand and
handing four back: 0 (they left the hand).

### Scoring a real trial

From one photograph at the end and the trial video:

- Watch the video: none of the two, six, four or five ever slips out from between the left gripper's fingers or
  touches the table.
- The ace lies face up and flat on the table; measure from the cross's centre to the middle of the ace: within 5 cm
  (full score) or 10 cm (0.75).
- The right gripper is at least 3 cm from the ace, and nothing moves in the last second.

## The demo

`demo.py` is open loop. Before it moves it plans everything from the task's layout (`plan`, about 15 s of
geometry and the arm's own IK): where the right hand pinches the ace (its pads on the ace, at least 4 mm from every
other card's edge, its palm and finger carriages clear of every card and of the left hand), where it comes in from,
how far the ace is drawn, and where it is let go. The only feedback while it runs is each hand's own measured
position (proprioception), to correct for the arms' sag. It never reads a card's state.

1. The right hand comes down beside the fan with its fingers open 28 mm, slides in along the fan's plane either
   side of the ace's free edge, and closes on it (about 200 mm2 of pad on the ace).
2. The left hand eases its grip: it is commanded 0.13 mm a side inside the five cards' thickness, so it still presses
   on them, lightly.
3. The right hand draws the ace out along its length at 3 cm/s until it has left the left pads (40 mm), and the left
   hand closes again on the four cards left. The right hand draws on until the ace is clear of them (110 mm in all).
4. It lifts the ace off the fan, carries it face up over the cross, tipped 30 degrees, lowers its lowest corner to
   4 mm above the table and opens. The ace falls flat, face up.

| | Step |
|---|---|
| Right hand comes in; closes on the ace | 5 to 170 |
| Left hand eases its grip | 170 |
| Ace drawn out of the left pads; left hand closes again | 190 to 225 |
| Ace clear of the hand: score 0.25 | 250 |
| Ace let go over the cross; falls flat: score 0.75 | 405 to 430 |
| Right hand 3 cm clear, all at rest: score 1.0 | 435 |
| Trial ends with full score | 470 |
| The ace's middle from the cross's centre at the end | 6 mm |
| The other four cards' furthest distance from the left pads during the trial | 0.4 mm |

## Control runs

| Run | Score | What happened |
|---|---|---|
| `--mode hold` (nothing moves) | 0.0 | The hand of cards stays as it started for 50 s; nothing moves by more than 0.2 mm. |
| `--mode tight` (never eases the grip) | 0.0 | The right hand closes on the ace and pulls: the ace does not move (0.5 mm); the pinch slides off it. |
| `--mode loose` (opens the left hand instead of easing it) | 0.0 | The ace is drawn and laid face up 2.5 mm from the cross, but the other four cards fall out of the hand. |
| Easing by 0.10 mm a side | 0.75 | Ace laid 3 mm from the cross, the other four still held; the hand was still settling at the end. |
| Easing by 0.16 mm or 0.20 mm a side | 0.0 | The ace is held too hard to come out; the pinch slides off it. |

## Dream-1

One run, 1,000 steps: **0.0**. In the first 6 seconds the left hand opened and the arm moved away, and all five
cards fell onto the table, face up. Both arms then rose and stayed raised, nearly still, for the rest of the 50
seconds. Dropping the whole hand scores 0.

## Building it for real

- Five standard poker-size playing cards (63.5 x 88.9 mm, plastic-coated), an ace, two, six, four and five.
- `fan_template.svg` printed at 100% on A4 (check its 50 mm scale bar). Lay the cards face up on their outlines, the
  five first and the ace last, on top.
- Move the left arm to its holding joints (rad): -0.232, 1.787, 0.456, 1.329, 1.339, -1.046, with its gripper open.
  Slide the pads over the template's cross along the six's length until their tips are 19 mm past it, close the
  gripper, and lift the template away. The fan then lies as in the setup: its pivot at x 26, y 8 cm, 15 cm above the
  table, its plane rising 30 degrees away from the robot, the six pointing to the robot's right.
- A blue tape cross, two 40 x 5 mm strips, centred at x 33, y -17 cm (from the midpoint between the arm bases).

Resetting: put the ace back on top of the fan on the template and hand the robot the cards again as above.

## What the simulation simplifies

- **Rigid cards.** They do not bend; a real card flexes a little as it slides out.
- **Sourced:** card size (a card printer's page, 63.5 x 88.9 mm); thickness 0.29 mm and weight 1.55 g (a measured
  deck: 52 cards 15.15 mm, ten cards 15.5 g). **Estimates:** every friction value (card on card 0.26/0.20, fingertip
  on card 0.38/0.35, card on table 0.38/0.35), the same as in the accepted card-dealing task whose card model this
  reuses. Each card is solved with 255 position iterations.
- **The gripper.** The simulated gripper is a stiff position-controlled spring (5000 N/m) with rigid pads, so how
  hard it presses is set by how far inside the cards' thickness it is commanded. A real gripper's pads are softer, so
  the right amount of easing on a rig will differ; the need to ease and re-close does not.
- **Why the ace is the front card.** In a reclined fan each card rests on the one under it, so the card lying on top
  of a drawn card is pressed onto it harder (by its own weight) than onto the card above. With one friction
  coefficient for every pair, a middle card drawn out takes the card on top of it along; in this simulator that
  happened in every trial (`test_a_middle_card_would_take_the_one_above_it`). The front card has only the pad on top,
  and comes out alone.
- **The starting hold** is set by the reset (above), as an operator would hand the robot its cards.
- **The cross is drawn only.** Nothing collides with it.

## Related tasks on the board

- "Deal a card to each of three places off a deck" (TK-ZPAQZ4, accepted) slides one card off the top of a pile on
  the table by fingertip friction. Here the cards are held in the robot's own hand: one card is pinched and drawn out
  of a grip the other hand has to ease and close again, and laid face up.
- "Pull one decommissioned cable out of a clipped bundle" (accepted) also takes one item out and leaves the rest:
  there the cable is drawn out of fixed clips in a tray, which hold the other cables by themselves. Here nothing
  holds the other cards but the robot's own left hand, which has to ease and close again while the right hand draws.
- "Play one turn of Jenga" (accepted) draws one block out of a tower that stands by itself on the table. Here the
  stack is held in the robot's grip, and the card is 0.29 mm thick: it is pinched, not pushed.
- No accepted task in this repository plays a card from a held hand (checked 22 November 2026).

## Credits and sources

- `cardkit.py` is adapted from the card kit of the accepted card-dealing task (TK-ZPAQZ4) in this repository: its
  card, friction and solver settings.
- A card printer's page for its poker-size cards (makeplayingcards.com): "Dimensions: 63.5mm x 88.9mm".
- A hobbyist's review of paper playing cards (homepokertourney.org): "The thickness of all 52 playing cards was
  15.15mm." and "Ten cards weighed 15.5 grams."
- No third-party meshes or scans: every object is built from boxes in code.
