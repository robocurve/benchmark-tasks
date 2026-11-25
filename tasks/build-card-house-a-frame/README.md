# Build the first A-frame of a house of cards

Take two playing cards out of their place-card stands and lean them against each other on a felt mat so
that they stand up on their own: the first A-frame of a house of cards. Built for the Robocurve YAM / Isaac
Lab-Arena stack.

## Why this task

Building a house of cards is a pastime almost everyone has tried, and "house of cards" is a byword for
anything fragile. Every house starts with this A-frame. What it tests that a pick-and-place does not:

- A card is 0.29 mm thick and weighs 1.8 g. It can only be pinched at an edge, and anything that brushes
  it moves it.
- Nothing joins the two cards, unlike a folded greeting card. Only friction where their top edges meet
  holds them up, so the edges must meet at matching angles. If one card's edge comes down on the other's
  face, it slides down that face and pushes the frame over.
- Both cards have to be set down together, and both hands have to let go and slide off without nudging
  either card. A few millimetres of disturbance and the frame falls.

## Geometry and physics

All values are in `spec.py` and `scene.py`. The origin is on the floor midway between the two pedestals'
centres; x points forward, y to the left, z up.

- Table: wooden, 500 x 700 mm top, 760 mm high, centred at x = 450 mm, y = 0.
- Felt card mat: green, 400 x 560 x 3 mm, centred at x = 430 mm, y = 0.
- Stands: two walnut place-card stands, blocks 20 x 60 x 15 mm, 40 g, with a slot 1.6 mm wide and 10 mm
  deep along the top. They stand loose on the felt at x = 370 mm, y = -150 mm (in front of the right arm)
  and +150 mm (left arm), slots along y.
- Cards: standard poker-size paper playing cards, 63.5 x 88.9 x 0.29 mm and 1.8 g, white faces with red
  backs. Each stands short edge down in its stand's slot, leaning on the slot's far wall, red back away
  from the arms.
- Friction: card on card 0.4, card on felt 0.5 (PhysX averages the two materials' 0.4 and 0.6), card on
  walnut 0.43. These are estimates.
- Contact: a paper-thin box lets an edge pushed into a face pass straight through it. So each card's
  collision surface stands 0.4 mm off its drawn faces (a PhysX rest offset), and the cards collide as
  1.09 mm slabs with rounded edges. A card therefore leans 3 degrees in its slot rather than the 7.5 a real
  0.29 mm card would. Cards are rigid and cannot bend.
- Two YAM arms on 750 mm pedestals at y = +/-305 mm. Grippers: 75 mm aperture, 40 N per finger. Physics
  runs at 960 Hz and control at 20 Hz.

In this simulation an A-frame whose cards each lean 15 degrees from vertical, top edges together, stands
indefinitely. At 22 degrees, or with one card's top edge resting a few millimetres down the other card's
face, the cards slid apart within half a second. Once a top edge bears on a face, it needs friction of at
least tan(lean) to hold, which is 0.40 at 22 degrees.

## Score ladder

An **A-frame** means all of the following:
- both cards are on the felt, their lowest points within 2 mm of it;
- each card leans 10 to 35 degrees from vertical, towards the other card (each card's top is tilted
  towards the other card's bottom edge);
- the two top edges are within 5 mm of each other, and the two bottom edges at least 20 mm apart;
- neither card touches a stand.

The ladder:
- **0.25** if the robot holds both cards out of their stands at the same time. A card is out when its
  lowest point is above its stand's top, or its centre is more than 40 mm from the stand.
- **0.5** if the cards form an A-frame at any moment, held or not.
- **1.0** if they stand as an A-frame for 5 s (100 control steps) in a row, with nothing touching either
  card except the felt and the other card. Any touch restarts the count.

The trial scores 0 for good if:
- a card is creased, bent or torn;
- a card falls off the table (more than 50 mm below its top);
- the table, mat or pedestals move more than 3 mm;
- anything leaves x -0.4 to 1.4 m, y -1.4 to 1.0 m, z 0 to 1.6 m.

The highest level reached counts. In simulation, a touch is any contact PhysX reports between the robot
(or a stand) and a card, checked at every physics step.

Physical judging: a camera beside the mat looking along the frame's ridge, with a protractor printed on a
card behind it, shows the leans; a ruler on the felt shows the gaps; the video shows the 5 s with nothing
touching. Real trials are safe: the objects weigh under 50 g each.

## External robot asset

The external YAM robot asset is MIT licensed (I2RT YAM via the pinned YAM Isaac Lab-Arena adapter).
It is not redistributed here. See `DEPENDENCIES.json` and retain upstream `LICENSE.i2rt`.

## Run

```bash
"$ISAAC_PYTHON" -m unittest discover -s . -p 'test_*.py'
bash run_inspect.sh --policy hold --steps 200 --check-reset --output runs/hold
bash run_inspect.sh --policy demo --steps 600 --check-reset --output runs/demo
python scorer.py runs/demo/evidence/episode-00/trace.jsonl
"$ISAAC_PYTHON" audit_run.py runs/demo --require-full
```

Dream (only after a full-score demo; shared GPU lock; ledger required):

```bash
bash run_inspect.sh --policy dream --steps 600 --output runs/dream \
  --budget-ledger /path/to/persistent/dream-budget.jsonl
```

Runtime per trial on one GPU, including scene start-up: about 2 minutes for the demo (full score at step
334, plus the reset check) and about 4 minutes for a 600-step Dream-1 run. Seed 0, no object jitter. The
demo observes only the three RGB cameras; Dream also receives standard joint and gripper readings. No
object poses or scorer state reach the policy. The trace records which robot bodies touched each card at
every step (`robot_card_bodies`).

The scripted demo replays an offline plan open loop (`demo.py`):
1. Each hand pinches its card's outer side edge with the last 15 mm of its pads, below mid-height. The
   fingers point in across the card and 45 degrees down, closing through the card's thickness.
2. It lifts the card out of its stand and swings it to a 15 degree lean, 6 mm above the felt, the two top
   edges 16 mm apart.
3. It brings the top edges together in the air (a foot dragging on the felt would pivot the card in the
   grip), then lowers both cards onto the felt at once. The cards sit 24 mm apart sideways, so neither
   hand's inner finger is in front of the other card.
4. It opens to 8 mm, waits half a second and slowly draws its fingers out sideways past the card's outer
   edge. Opening wider brings the fingers against the cards, and the arms' wrists are soft.

## Sources

- Familiarity: https://en.wikipedia.org/wiki/House_of_cards (the structure and the idiom);
  https://www.guinnessworldrecords.com/world-records/tallest-house-of-cards (record houses built from
  the same two-card A-frames)
- Card size: https://en.wikipedia.org/wiki/Standard_52-card_deck#Card_size (poker size 3.5 x 2.5 in, that is
  88.9 x 63.5 mm; measured poker-size cards 0.24-0.34 mm thick and 1.4-2.78 g)
