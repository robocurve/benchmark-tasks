# Slide a vegetable kebab off its skewer onto a plate with a fork: one hand holds the skewer, the other pushes

A vegetable kebab lies across a dinner plate, as served. Four pierced pieces (pepper, onion, courgette, mushroom) grip a 3 mm bamboo skewer. A push along the stick that moves them also drags a loose skewer across the plate, so one hand holds the skewer by its bare end while the other takes a dinner fork, sets its tines behind the last piece and drives the pieces off the point onto the plate. Then the bare skewer is put down on the board beside the plate. Recipes say it plainly: "Use a fork to slide the vegetables off of the skewers."

## Run

From this folder:

```
python demokit.py task.py OUT                      # final film, 640 x 360, GPU to yourself
python demokit.py task.py OUT --no-record --mode full,none,one_arm,hold_only,hover,fingers,offplate,no_aside
```

The harness's instruction (Dream-1 run): `--instruction "Take the vegetables off the kebab onto the plate with the fork: hold the skewer by its bare end and push the pieces off its point with the fork, then put the empty skewer down on the board beside the plate. Do not touch the vegetables with your grippers."`

`KB_SKM` (stick mass, default 0.010 kg), `KB_GRIP` (slide force per piece, default 1.0 N), `KB_PIN` (pinch distance from the blunt end, default 0.04 m), `KB_YAW` (the fork's heading, default -35 degrees), `KB_TZ` (the tines' height over the stick's axis while pushing, default 0.005 m), `KB_OVER` (how far the tines go past the point, default 0.035 m) and `KB_DZ` (the pinch's height over the board when the skewer is laid down, default 0.004 m) are debug knobs.

**Result** (scripted, GPU physics, 20 Hz): the full mode scores 1.0 on a lab PC (901 steps, 45.1 s, against a limit of 1,200). Each route in the table below ran on GPU on the final scorer.

**Dream-1 run** (the harness's own policy, 1,200 steps, one trial, on a lab PC, one session): 0. The policy did not hold the skewer or take anything off the kebab in 1,200 steps (no tier reached). The film is `model_run.mp4` (left, right and top cameras side by side).

## Scoring, and the routes that score less

| Mode | What it does | Score (GPU) |
|---|---|---|
| full | left comes straight down on the bare stick 40 mm from its blunt end, over the board, and pinches it (40 N); right takes the fork by its handle, brings the tines just over the stick behind the mushroom (the fork 55 degrees off the stick's line), pushes about 0.16 m along it; the four pieces drop onto the plate in turn; the right lays the fork back on the board; the left lifts the bare skewer, draws it 0.12 m back and 0.13 m toward the robot, off the plate, lays it on the board and lets go | 1.0 (901 steps, 45.1 s; the stick drifts 15 mm at most in the pinch) |
| no_aside | as full, but the bare skewer is let go on the plate | 0.5 (all four pieces pushed onto the plate; the skewer not put aside) |
| hold_only | the left pinches the stick and holds it, nothing else | 0.25 (held 1 s) |
| hover | the left hand open, fingertips 8 mm over the bare stick, 4 s, nothing else | 0.0 (jaws not shut: not held) |
| one_arm | the right alone pushes with the fork, the skewer loose | 0.0 (the kebab slides 0.17 m across the plate with the pieces; nothing comes off) |
| fingers | no fork: the left holds the skewer, the right pinches the mushroom and strips the pieces off with its fingers | 0.0 (a gripper on a vegetable: void; three pieces came off with no fork pushing: FAULT) |
| offplate | the left lifts the kebab and carries its point 0.17 m out beyond the plate's far rim, then the fork pushes the pieces off there | 0.0 (the skewer carried 0.27 m from where it lay counts as not held, so the first piece off the point is a FAULT) |
| none | nothing | 0.0 |

The score is read from the state at the end; only the failures latch (a gripper on a vegetable, a piece off the point without the fork pushing and the skewer held, a piece landing on the bench). The 0.25 tier reads the hold now, not a latched flag. `check_score.py` runs the tier table on hand-built records with no simulator.

## Sizes

| What | In the scene | Published or estimated | Source |
|---|---|---|---|
| Skewer | 300 x 3 mm bamboo, axis 26 mm over the bench, point toward -y; 10 g in sim | 12 in x 3 mm; about 2 g | Roland 71516; mass: see "What is modelled" |
| Pepper | 6 x 30 x 30 mm, 6 g, red | | estimate |
| Onion | three layers, 18 x 25 x 30 mm, 12 g | | estimate |
| Courgette | round, 12 mm thick, 32 mm across, 9 g | | estimate |
| Mushroom | domed cap 35 mm across, 30 mm tall, with a stem stub, 15 g; rests on the plate's floor and carries the kebab | | estimate |
| Run | 66 mm long, front face 40 mm and rear face 106 mm from the point; 194 mm bare behind it; each piece pierced on the stick (the sim's 4 mm outline is a collision clearance only) | | estimate |
| Slide force | 1.0 N per piece along the stick | | estimate, to measure |
| Plate | 260 mm across, floor 8 mm, rim wall 20 mm high, centre (0.33, -0.16); the kebab lies across it, its bare end over the rim | 26 cm dinner plate | estimate |
| Fork | 190 mm; four tines 2.5 mm wide on 7.5 mm centres (5 mm gaps), head 25 mm wide, neck raised 8 mm, handle 3 mm thick widening from 7 mm at the neck to 16 mm at its rounded end; lies on its tines and handle end; 45 g | IKEA FORNUFT dinner fork, 19 cm | IKEA (size); shape and mass estimates |

## Colliders

| Body | Colliders | Free or fixed |
|---|---|---|
| Plate | a cylinder for the floor, 28 box segments for the rim | fixed |
| Skewer | one 300 x 3 x 3 mm box | free, 10 g |
| Pepper, onion, courgette, mushroom | the outline as four convex hulls round a 4 mm hole, so nothing overlaps the stick at spawn | free, each on the stick by a sliding joint (below) |
| Fork | a hull each for the handle, the neck, the head and each tine | free, 45 g |
| Fingers | the harness's YAM finger colliders | the robot |

## Robot figures the demo keeps within

The YAM's jaws open to 94 mm and grip about 40 N a finger; each arm pushes about 20 N and reaches about 60 cm. The left pinches the 3 mm stick at 40 N (jaws across x, fingers straight down); the right pinches the fork's 3 mm handle at 40 N and pushes at 20 mm/s with a few newtons. Loads about 0.02 N m.

## Real trial

Cut fresh pieces for every trial to the sizes above, pierce each piece on the 12 in bamboo skewer itself (or with a 2.5 mm awl first); do not preform a 4 mm hole, which leaves clearance instead of a friction fit. Place the run so its front face is 40 mm from the point and its rear face 106 mm from the point (40 mm plus the 66 mm run, as in the sim). Check each piece slides on about 1 N: hold the skewer, loop string round the piece and pull along the stick with a luggage scale or kitchen scale; replace a piece that moves under about 0.5 N or needs over about 2 N. Lay the kebab across a 26 cm plate, its pieces on the plate's floor, the point 18 mm past the plate's centre and the bare end over the rim on the board; lay a dinner fork on the board at the front right. Keep the pieces at room temperature and use them within 5 minutes.

What the video and the top photo see, by the scorer's terms:
- HELD: the left hand's jaws shut on the bare stick (both pads within 4 mm of it, jaws 6 mm apart or less; a hand open over it is not holding), the stick staying within 30 mm of where it lay.
- PUSHED: each piece leaves the point with the fork behind the run, the stick held.
- ON THE PLATE: the piece resting inside the plate's rim after the run (top photo).
- PUT ASIDE: the bare skewer lying on the board, all of it outside the plate's rim, and both grippers clear of the pieces over the last 2 s.
- Void: a gripper touching a vegetable; a piece coming off the point with no fork pushing it; a piece on the bench.

| Real-trial kit | Use | Price (about) |
|---|---|---|
| Bamboo skewers 12 in x 3 mm (100) | the stick | $5 |
| Red bell pepper, red onion, courgette, button mushrooms | the pieces | $8 |
| 26 cm dinner plate, IKEA FORNUFT dinner fork (pack of 4) | scene | $8 |
| Phone and tripod | top photo and video | $0 to $15 |
| Total | | under $40 |

## Safety for a real trial

Raw vegetables only; wipe the fork and plate between trials. The skewer's point is sharp: keep faces clear of the plate while the arms work, and protect the gripper pads from it in real trials (blunt the tip or fit a cap).

## What is modelled, and what is not

- Each piece is a rigid body of its real size and mass, held on the skewer by a joint made on the live stage: locked in every direction but the slide along the stick, where a drive caps the force at 1 N (an estimate). The joint is taken off when the piece's rear face passes the point, and the piece falls free onto the plate. A real pierced piece is soft and grips by friction; the force is a measurement to take on the real kit.
- The skewer is 10 g in sim, not 2 g: a 2 g stick flicks at about 1 m/s when pinched (contact jitter) and the pieces slide off it. The 40 N pinch lets the stick drift up to 15 mm under the push; the scorer allows 30 mm.
- The fork and plate are free or fixed bodies with real colliders; the fork is a rigid steel shape of the IKEA FORNUFT's length and outline, its tines' 5 mm gaps clear of the 3 mm stick.
- The demo reads the skewer's and the fork's poses to aim the tines and to push along the stick; a mushroom stem is a stub.
- A piece rests on the plate, or not; juice, softness and crumbling are not modelled.

## Sources

- 'Use a fork to slide the vegetables off of the skewers. Serve as soon as possible.' (oven-roasted or grilled veggie kabobs, https://www.justapinch.com/recipes/side/vegetable/oven-roasted-veggie-kabobs-or-grilled.html?r=3)
- 'Tell your guests to remove the veggies from the skewers using a fork' (grilled vegetable kebab serving tip, https://eatsomethingvegan.com/simple-vegetable-skewers/)
- 'Hold the skewer horizontally over your plate ... use your fork to gently push or slide the food items off' (https://ova.it.com/dining-etiquette/how-do-you-remove-food-from-a-skewer)
- Bamboo skewer 12 in x 3 mm: Roland 71516, https://rolandfoods.com/product/71516-bamboo-skewers-12-x-3mm
- Dinner fork: IKEA FORNUFT fork, stainless steel, 19 cm, https://www.ikea.com/gb/en/p/foernuft-fork-stainless-steel-40428482/
- Piece sizes, masses, slide force, plate size, fork shape and mass: estimates.

Roland and IKEA are named for identification only; this task is not affiliated with them.

## Licence

MIT
