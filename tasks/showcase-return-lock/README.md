# Put the shown watch box back in a stepped showcase, slide both glass doors shut and push the plunger lock in

A countertop showcase stands on the counter with its sliding glass back doors toward the arms. Inside, the display is a two-step stair facing the customer: a 5 cm white-velvet riser right behind the doors, with four watches standing on it on clear C-ring stands, and the black velvet deck in front of it, the lower step. A watch case that a customer was shown waits on the counter beside the showcase. The robot carries the watch case in through the door opening, over the riser and between two of the watch stands, and sets it on its marked spot on the lower step without knocking a stand; it then slides the inner (back-track) door shut, slides the outer (front-track) door shut over it, and pushes the keyless plunger lock on the outer door in until it latches, so that neither door can slide. Then it takes its hands away.

In the demo the left hand takes the watch case from behind and above, its fingers pointing forward and 45 degrees down and closing across the case's 7.6 cm width with about 20 N, lifts it so its base is 14 cm above the counter (5 cm over the riser, level with the watch stands), carries it in through the 22 cm opening at the left end and through the 14.2 cm gap between the two left stands, and lowers it onto the taped spot in 1.5 mm steps until it sits on the deck. It lets go, backs off along its fingers, closes them, and backs out through the same gap. With its fingers closed and pointing to the right and 50 degrees down, it then pushes the inner door's glass edge 6 cm to its stop, 21 cm up, high enough that its housing passes over the watch stands behind the glass, and draws back. The right hand takes the outer door's bar pull, slides the door 29 cm to its stop, lets go, and pushes the plunger's head in with its closed fingertips, about 16 N along the plunger's axis, until it latches at full travel. Both hands then draw back out of the case's outline.

**Why it matters.** Impact on jobs. Retail salespeople who work locked cases (watches, jewellery, phones, fragrance, the locked aisles of drugstores and big-box stores) show an item from behind the case and lock it again after each showing, many times a shift. O*NET 41-2031.00 Retail Salespersons (3,998,100 employed in 2025, O*NET OnLine from BLS OEWS) lists "Watch for and recognize security risks and thefts, and know how to prevent or handle these situations" and "Ticket, arrange, and display merchandise to promote sales". Only the salespeople who work locked cases do this act, not all 4.0 million. The U.S. Census Bureau counted 23,096 jewelry stores with employees in 2013 (County Business Patterns, NAICS 448310), each with locked cases; this excludes watch, phone and drugstore counters and is a 2013 figure. CRL's showcase lock pages describe the lock used here: "The plunger can be activated without the key, so it takes only seconds to secure the bypassing doors" (KML41); the key is only for unlocking.

**Ability tested:** a free carry through a narrow door opening and between two watch stands that tip at a touch (a 9.4 cm wide hand and box in a 14.2 cm gap, 5 cm over the riser), set down on a marked spot on the lower step; two glass panels slid by fingertip and pull to their end stops (the back door first, as in practice: once the front door is shut, the back door's edge is behind it; the scorer reads end states and does not check the order); and a straight push of a small sprung plunger to full travel. One arm can do it all; the natural split is one hand per door, and the scorer does not check which hand does what.

## Requirements

`yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source <ARENA_DIR>/yam.env`). `mechkit.py`, `demokit.py`, `gatekit.py` and `taskspace.py` are in this folder. Importing `mechkit` hooks `yam_arena.scene` and `yam_arena.inspect_embodiment` without changing them on disk.

## Run a trial

From this folder:

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1600 \
  --instruction "Close up the showcase after the showing. Put the watch box back on its marked spot on the lower step, at the front of the case, without knocking over or shifting the watches standing on the higher step; slide the inner glass door shut, then the outer glass door, and push the plunger lock on the outer door in until it latches. Then take your hands away."
```

## Scripted demo

The demo stays inside the YAM's figures: 20 N a finger on the 150 g watch case, 20 N on the pull, about 16 N on the plunger (push limit about 20 N), door friction 3 N.

```bash
python demokit.py task.py OUT --mode full                                  # full score, with a video
python demokit.py task.py OUT --no-record --mode knock,near_tier,drop_high,doors_open,no_lock,inner_open,hand_on   # the check runs below
python3 check_score.py                                                     # the tiers and the hands-away test on hand-built states, no simulator
```

The demo reads the doors' and the plunger's joint positions only to print them, and the watch case's height to stop lowering it when it sits on the deck. It moves nothing except through the grippers.

## The scene and what is simplified

Positions are in cm from the midpoint between the arm bases (x forward, y left), heights above the counter.

| Part | In the scene | Source |
|---|---|---|
| Showcase | 61 x 30.5 x 45.7 cm (24 x 12 x 18 in), aluminium frame, 6 mm glass top, front and sides, black display deck 4 cm above the counter; back face at x = 37, centred | Webstaurant 89129359 locking countertop display case, 24 x 12 x 18 in |
| Doors | two 31 x 40 cm panes of 6 mm glass in a double track, 1.4 cm apart; inner shuts over y -30 to +1, outer over y -1 to +30 (2 cm overlap); start: outer slid fully right (29 cm), inner 6 cm open | case size; 1/4 in glass |
| Pull | 15 mm round chrome bar, 15 cm long, on 25 mm standoffs, on the outer door 7 cm from its overlapping edge | a standard bar pull |
| Plunger lock | CRL KML41-style keyless push lock in the outer door, 3.5 cm from its overlapping edge, 12 cm up: 16 mm head standing 17 mm out of a 24 mm barrel, 12 mm travel, spring rising to 10 N (assumed; a real KML41 may also latch with a door open) | CRL sliding glass door lock catalogue (KML41, KML41GL); spring force assumed in CRL's class |
| Riser (higher step) | white velvet over a block, 58.4 x 6.4 x 5.1 cm (2.5 in deep, 2 in high), from x = 40 to 46.4, fixed; the black velvet deck in front of it (x = 46.4 to 66.9) is the lower step | velvet-covered and acrylic display risers are sold 2 in high in many widths (Azar Displays, Plymor); or a 2 x 3 in board cut to the case's inside width and wrapped in white velvet, about US$20 |
| Watch stands | four watches on clear C-ring watch stands: a 5.6 x 3.8 x 0.6 cm base, a short upright, a 5.2 cm C-ring with a black or dark brown leather strap round it, a 42 mm steel watch head with a black bezel and a dark blue or white dial on the ring's front, turned 55 degrees up toward the customer; 9.9 cm tall, 150 g each (25 g stand, 125 g watch, assumed); centred at x = 43.2 and y = +26, +8, -8, -23, free bodies | clear C-ring watch display stands, 2.2 x 1.5 x 3.9 in, sold in 10-packs (about US$1 each; listing 529715248 on desertcart, from Amazon); any quartz watch on a leather strap |
| Watch case | single-watch case with pillow, 9.7 x 7.6 x 7.1 cm (3.8 x 3 x 2.8 in), 150 g, at (30, +42) on the counter, long side forward | Prestige & Fancy single watch case, Walmart 5341909840; mass assumed |
| Spot | white tape square, 12 x 12 cm, centred at (52.5, +17) on the lower step, in line with the gap between the stands at +26 and +8 | |

Departures, each stated once:
- The case frame and the riser are fixed to the counter (a real case weighs about 15 kg and does not move under a hand; a riser can be taped down).
- Each watch and its stand are one rigid body, its shape built from boxes and cylinders; masses and friction are assumed.
- The doors are prismatic joints with end stops; track friction 3 N is assumed (2 to 5 N for glass doors on plastic shoes).
- The plunger's catch is a rule: when the plunger reaches 11.5 mm of its 12 mm travel with both doors within 5 mm of their stops, the plunger and both doors are held where they are, standing in for the plunger's tip blocking the inner door's edge. The tip has no collider; with a door short of its stop the plunger goes in but does not latch and springs back.
- Physics: rigid bodies at 240 Hz on the GPU.

## Scoring

Positions, checkable with a ruler and a photo. Four tiers, one per key substep, and one failure that voids the run. Terms:
- **Box on its spot:** the watch case's centre within 2.5 cm of the spot's centre on each axis, resting on the deck (its base within 6 mm of it), upright within 10 degrees, and no gripper point within 5 mm of it.
- **Door shut:** within 5 mm of its closed stop.
- **Latched:** the plunger held in at full travel by its catch.
- **Hands away:** every point of both grippers (fingers root to tip, pads, palm and housing, on a grid of about 6 mm) and of each wrist (the axis from link5 to the gripper) at least 1 cm outside the case's outline box, which runs from the pull's front face to the case's front.

| Score | Condition, read from the state now |
|---|---|
| 0.25 | the box on its spot |
| 0.5 | and the inner door shut |
| 0.75 | and the outer door shut |
| 1.0 | and the plunger latched with both hands away, all held for 2 s (40 steps at 20 Hz) |

Void, score 0, at any time: a watch stand on the higher step tilted more than 20 degrees, or its base moved more than 1.5 cm from where it stood (a stand knocked over or shoved); or the watch case dropped or tipped over 45 degrees at any time. Simulator: the box's base 8 mm lower than one control step (50 ms) before: any free fall of 1.6 cm or more puts that much into one step, and the demo lowers the box at most 5.6 mm a step. Real trial: the video shows the box let go with its base more than 1.5 cm above the deck, or falling free. So a stand knocked over on the way in scores 0, and so does a box left on the higher step or outside the case; a case shut and locked with the box outside scores 0. Possible scores: 0, 0.25, 0.5, 0.75, 1.0.

## Building it for a real trial

Real trial: a 24 x 12 x 18 in sliding-door countertop showcase (about US$250, Webstaurant 89129359 or similar) with a CRL KML41 plunger lock fitted to the front door (about $25), a 2 in white velvet riser cut to the case's width (about $20), four C-ring watch stands (about $1 each) with four inexpensive watches on leather straps (about $15 each), a single-watch case (about $8), and white tape. Mark each stand's base outline on the riser with thin tape. Reset, about 2 minutes: unlatch with the key, set the stands back on their outlines, slide the outer door fully right and the inner door 6 cm open, set the watch case on its tape mark on the counter, send the arms to their ready pose. Film from straight above (the box on its spot), from behind the arms (doors and plunger) and from the front through the glass at deck height (release height).

## Results

Run on GPU physics at the README's commands; the check runs record no video.

| Run | What it does | Score |
|---|---|---|
| `full` (the film) | the whole task, 640 x 360 | 1.0, at step 815 (40.8 s); latched at step 739 |
| `knock` | the box carried in 7 cm off the gap, into the stand at +8, then the rest done | 0.0 (void: the stand knocked over) |
| `near_tier` | the box set down on the riser between the stands, the rest done | 0.0 (box not on its spot) |
| `drop_high` | the box let go 4 cm above the deck on its spot, the rest done | 0.0 (void: dropped) |
| `doors_open` | box on its spot, both doors left open | 0.25 |
| `no_lock` | box on its spot, both doors shut, the plunger never pushed | 0.75 |
| `inner_open` | box on its spot, the inner door left 6 cm open, the outer door shut, the plunger pushed in | 0.25 (the plunger goes in but does not latch) |
| `hand_on` | everything done, the right fingertips left on the plunger's head | 0.75 (hands not away) |
| `Dream-1` | the model, one run of 1,600 steps from the task instruction, seed 1 | 0.0 (box not placed on its spot) |

In the `full` run the box ends 3.9 mm and 1.6 mm from the spot's centre, both doors at their stops, the plunger latched 12 mm in, and the four watch stands where they stood (the one at +8 moved 0.8 mm, none tilted).

## Sources

All meshes are generated by `task.py` and `mechkit.py`. There are no third-party assets.

- Workers: O*NET 41-2031.00 Retail Salespersons (https://www.onetonline.org/link/summary/41-2031.00). This includes information from O*NET OnLine by the U.S. Department of Labor, Employment and Training Administration (USDOL/ETA), used under the CC BY 4.0 licence; O*NET is a trademark of USDOL/ETA.
- Jewelry stores: U.S. Census Bureau, Facts for Features, Jewelry stores (2013 County Business Patterns, NAICS 448310) (https://www.census.gov/content/dam/Census/newsroom/facts-for-features/2016/cb16-ff02_graphic.pdf).
- Lock: CRL KML41 and KML41GL showcase plunger locks (https://www.crlaurence.com).
- Showcase: Webstaurant 89129359 locking countertop display case, 24 x 12 x 18 in (https://www.webstaurantstore.com).
- Watch stands: clear C-ring watch display stands, 2.2 x 1.5 x 3.9 in, 10-pack (https://www.desertcart.uy/products/529715248).
- Watch case: Prestige & Fancy premium single watch case with PU leather pillow, 3.8 x 3 x 2.8 in (https://www.walmart.com/ip/5341909840).
- CRL, Webstaurant, Plymor, Azar Displays and Prestige & Fancy are named here for identification only; the names are their owners' trademarks.

## Licence

MIT
