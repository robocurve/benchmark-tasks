# Roll five dice from a dice cup into the game-box lid: every die tumbles out, lands flat and stays in the lid

Rolling dice out of a dice cup is how every turn of Yahtzee, Farkle or Liar's dice starts, and most people have done
it. The lid of a square board-game box lies open side up on the table as the dice tray, and beside it stands a
felt-lined leather dice cup with five dice in it. The instruction asks for the five dice to be rolled out of the cup
into the box lid; it does not say how.

What makes it hard is the hand skill. The cup is round and the dice are loose inside it: one hand takes the cup by its
side (the fingers must stay off the dice), lifts it over the lid and turns it mouth-down in one quick turn of the
wrist, so the dice fall out together and tumble as they land. Tipped slowly and low, with the lip just over the lid
floor (the way one pours from a cup), the dice only slide out and lie as they were: that is not a roll. Turned less
than about 120 deg, some dice stay in the cup (the felt lining holds them). Dropped in a heap, a die can come to rest
cocked, leaning on another die or on a wall, and does not count. One arm is enough; the difficulty is in how the
cup is held and turned.

The task is the task file [`roll_five_dice_from_a_dice_cup.py`](roll_five_dice_from_a_dice_cup.py) for the
`yam_arena` embodiment (scene, scoring and the scripted demo), with [`bodykit.py`](bodykit.py) next to it.

## Why it matters

Dice games are among the most familiar table games, and the dice cup is how they are played: Yahtzee lists "1 Dice cup
or shaker" in its equipment ([Yahtzee Manifesto, rules][rules]) and each turn starts by rolling all five dice with it.
Someone watching a ten-second clip knows at once what the robot is doing. A roll only counts if each die tumbles and
comes to rest flat on one face; a die left propped against a wall or balanced on an edge must be re-rolled
([backgammon federation rule FAQ on valid rolls][cocked]), and a tray "is a good way to keep your cubes in check"
([dice etiquette guide][tray]). The same hand skill, emptying loose small parts out of a held container with one
controlled turn so they spread rather than slide out in a heap, is used whenever a cup or scoop of small things is
emptied onto a tray.

The cup is a common felt-lined bicast-leather dice cup sold with five dice: 3 3/4 in (95 mm) tall and 3 1/4 in
(83 mm) across. The dice are standard 16 mm (5/8 in) acrylic dice with rounded corners, about 4.5 g each.

## Requirements

[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) (tested at commit `0059d02`) on Isaac
Lab-Arena 0.3.1 / Isaac Sim 6.1, on Linux with an NVIDIA RTX GPU. Its `setup.sh`, or the program's Brev launchable,
installs everything, including the Dream-1 policy. The task needs no models from Arena's object library and uses no
third-party assets: the lid, the cup and the dice are built in these files from measurements.

`yam_arena` task files place single boxes, cylinders and spheres. The lid (a floor and four walls), the cup (a base
and a wall of 24 panels, felt lining drawn inside) and the dice (rounded cubes with their pips drawn on) are built with
`bodykit.py`, which adds such multi-shape bodies: importing it hooks `yam_arena.scene` when the simulator starts,
without changing yam_arena, and gives the task file each body's pose every control step. This copy also gives a body a
restitution (bounciness).

## The scene

Positions are in cm from the midpoint between the two arm bases: x forward (away from the arms), y to the robot's
left, z up from the table top.

- Box lid, fixed (taped down), open side up: cardboard 2 mm thick, 25.4 x 25.4 cm outside, 3.0 cm tall, 25.0 x 25.0 cm
  inside, centred at x 25.0, y 7.5; its inner floor 2 mm above the table.
- Dice cup, loose, standing upright at x 27.0, y -16.5 (to the right of the lid, 7 cm from its wall): 83 mm across,
  95 mm tall, a 3 mm wall and a 4 mm base, felt-lined, 75 g.
- Five dice inside the cup, in one layer on its floor: one in the middle and four around it, 19.5 mm apart (centre to
  centre) along x and y, each standing square (faces along x and y), 1 face up. 16 mm, edges and corners rounded to
  2 mm, 4.5 g each.
- Both arms start at the ready pose, grippers open. The layout is the same in every trial.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` has held for 1 s, otherwise the score at the end of the run. Every
control step the task file reads the pose of the cup and of each die, and the points of each finger's collision hulls.
A die counts (0.2 each) when all of these hold:

- it has left the cup: its centre is outside the cup's inner cylinder (radius 38.5 mm, from the base to the rim);
- no gripper finger has touched it at any time in the trial (no finger point within 1.5 mm of it; once touched, the
  die never counts again in that trial);
- it rolled: after leaving the cup it turned over by 90 deg or more at some moment (the angle by which its rotation
  since leaving the cup tips the vertical; spinning flat on the floor does not count). A die that slides out and lands
  on the face it lay on in the cup has turned only about 20 to 70 deg;
- it rests flat on the lid floor: its lowest point within 1.5 mm of the floor, its faces within 5 deg of level (not
  cocked on another die or against a wall), its centre inside the lid's walls, at least 14 mm from the cup (not
  touching it), and still (moving under 1 mm in 1 s).

Score: 0.2 x the number of such dice. Success (1.0, held 1 s): all five. Doing nothing scores 0. A real trial is scored
from the trial video and a photo from above at the end: for each die, whether it left the cup untouched, whether it
turned over at least once (a quarter turn over an edge) after leaving the cup, and whether it lies flat on the lid
floor (a die that a second die laid on top would slide off is cocked) and not touching the cup.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=roll_five_dice_from_a_dice_cup.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Roll the five dice out of the dice cup into the box lid." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/dice-dream
```

The scripted demo, which reaches full score (open loop from the known layout; the left arm stays at its ready pose).
The right hand comes from behind the cup, pointing forward and 20 deg down, its fingers closing across the cup 60 mm
above the table, closes 4 mm inside the cup's sides, lifts it and gives it a gentle rattle (two 1 cm sideways moves).
It brings the cup, upright, over the right part of the lid with the grip 25 cm above the table. Then, in one turn of
0.6 s, it turns the wrist 150 deg about its pointing direction, so the cup's mouth turns down toward the open part of
the lid (the cup ends tilted 130 deg), while sweeping the cup 6 cm left: the dice fall 15 to 20 cm, tumble and spread
over the lid floor. After 0.6 s it turns the cup back upright while moving it 10 cm right, clear of the lid; the
trial ends on success at step 332 (16.6 s), while the cup is carried back to its place.

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=roll_five_dice_from_a_dice_cup.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Roll the five dice out of the dice cup into the box lid." \
  --scorer task_score --max-steps 700 --store-frames --log-dir ~/yam-eval/dice-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/dice-dream/*.json; do inspect-robots video "$log"; done
```

`DICE_DEBUG=1` prints, every 5 steps (`DICE_DEBUG_EVERY=1` for every step), the number of dice that count, the cup's
position, height and tilt, the right gripper's position and opening, and for each die its position, the height of its
lowest point over the lid floor, its tilt from flat, its largest turn since leaving the cup, and flags: X it has left
the cup, T a finger touched it, OUT outside the lid, G it counts.

`DICE_MODE=slide` runs the demo the wrong way: the same grip and turn, but the cup is tipped slowly (over 1.6 s) and
low, its lip ending about 15 mm over the lid floor, without the sweep, the way one pours from a cup. The dice slide
out over the lip and lie down on the face they lay on in the cup (largest turns 7 to 66 deg): none rolled, score 0.

How reliable the roll is: with the same grip and a turn of 145 to 150 deg, seven variants of the throw (grip 20, 23,
25, 30 or 35 cm above the table; turn 0.4 to 0.6 s; with or without the 6 cm sweep) scored 1.0 six times and 0.8
once (from 35 cm one die landed after turning 82 deg); no die left the lid in any of them. The same turn made low, the
lip 5 to 7 cm over the floor, scored 0.2 to 1.0 (mean 0.6 over 14 variants): many dice slid out and turned less than 90 deg. Slow, low
tips like `DICE_MODE=slide` scored 0 to 0.2. In an early test a cup turned only to 114 deg left two dice inside it.

## What is modelled and what is not

- Rigid bodies. The dice are 16 mm cubes with edges and corners rounded to 2 mm (crossed boxes, a capsule on each edge
  and a ball at each corner); the pips are drawn on (they do not collide). The cup's wall is 24 flat panels (a
  24-sided wall, 83 mm across its flats); its felt lining is drawn on.
- Restitution (bounciness, estimates): dice and lid cardboard 0.30, the cup's felt 0.05; PhysX averages the two
  touching materials. Friction (static / sliding, estimates): dice 0.40 / 0.35, lid 0.40 / 0.35, cup 0.60 / 0.50.
- The rattle in the cup is gentle and only for show; the dice do not need shaking to score. Dice falling on each other
  and on the lid are simulated as they happen; a real lid's cardboard flexes a little, which is not modelled.
- The lid is fixed to the table (a real one would be taped down for the trial).

## Runtime per trial

On one NVIDIA RTX PRO 6000 Blackwell Server Edition shared with other simulation jobs, the scripted demo with
1280 x 720 cameras (frames stored, videos rendered) took about 2 minutes and ended on success at step 332 (16.6 s of
simulated time); the Dream-1 trial of 2,400 steps at 320 x 176 took about 7 minutes. Debug runs of the demo at
320 x 176 (400 steps) took about 5.5 minutes each with six running at once. Isaac Sim's first start on a new machine
adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed and the scripted demo is deterministic (the same run
repeated gave the same final dice positions to the millimetre).

## Building it for real

A felt-lined dice cup with five 16 mm dice (sold together, about 10 USD); the lid of a square board-game box about
25 x 25 cm inside and 3 cm deep (or any cardboard box lid of that size), taped to the table, open side up; tape marks
for the lid's corners and a circle for the cup's base. For each trial put the five dice back in the cup, one face up,
one in the middle and four around it, and stand the cup on its circle.

[rules]: https://www.yahtzeemanifesto.com/rules.php
[cocked]: https://usbgf.org/rule-faq-understanding-dice-on-checkers/
[tray]: https://www.vaultgames.com.au/blogs/news/the-tabletop-gamers-essential-guide-to-dice-etiquette
