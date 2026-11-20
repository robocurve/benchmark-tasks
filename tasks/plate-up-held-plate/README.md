# Plate up a canteen meal on a hand-held dinner plate: steak, hot dog, two wings and a bread roll, all inside the rim

A serving counter holds a short stack of 26 cm melamine dinner plates, two steam-table pans of food (a seared rump steak and a hot dog in its bun in one, two whole chicken wings and a crusty bread roll in the other) and, across the counter, the pick-up zone on the pass: a shelf pad outlined by a yellow tape line, where the guest collects the plate. One hand takes the top plate off the stack by its rim and holds it up in the air for the whole loading; the other hand picks up a pair of spring serving tongs, lifts the five portions out of the pans with them and lays each on the held plate so that nothing sticks out past its edge; then the served plate is delivered into the pick-up zone and let go.

In the demo the right hand first presses the top plate's far rim down with about 15 N, so the plate tips 5.5 degrees on the plate below and its near rim rises about 1.5 cm. The left hand slides level jaws round the raised near rim, 11 cm from the plate's centre, and pinches it with 40 N (the YAM's figure), one pad under the rim and one on top. The right hand lets go of the far rim, and the left lifts the plate 6.5 cm straight up off the stack and brings it toward the pans. From then on the left wrist reads the plate's tilt and turns about the pinch to bring it level, before the loading and after each item. The right hand takes the 12 in spring tongs off their towel by one handle 11.5 cm from the hinge, squeezes the handles to close them 18 degrees, and turns them tips down; from then on the gripper's opening sets the tips' gap, which the hinge spring opens to 10.8 cm when the fingers part. The fingers never touch the food. With the tongs the right hand lays the steak (taken across its 8.2 cm middle) along the plate on the side toward the pans, then the hot dog beside it, then the bread roll on the far end of the steak's top and one wing on its near end, and last the other wing on the plate floor past the hot dog. Each item is carried beside the plate, lifted to 18 cm (the last wing to 23 cm, over the hot dog) and laid from above, and the tips ease open only 4 mm each side to let it go, so they do not sweep the portion beside it. The tongs are then held up and clear. The left hand then raises the loaded plate 8 cm (its housing hangs below the jaws and would drag across the stack) and carries it over the pick-up zone in 5 cm steps, levelling it after each. The palm and housing reach 5 cm below the jaws, so a level pinch cannot lay the plate flat: the wrist pitches the fingers 10 degrees down, the plate's far edge goes down onto the pad, the jaws open to their approach gap while the hand drops by its sag under the load (or it would spring up and flick the rim), and the hand sinks while pitching to 25 degrees, so the near rim comes down on the lower finger. It then draws its fingers 10 cm clear of the rim before lifting away.

**Why it matters.** Impact on jobs. On a staff-served line (school and hospital cafeterias, university dining, carveries) the server takes a plate, holds it by its edge and fills it from the pans. O*NET lists "Apportion and serve food to facility residents, employees, or patrons" for Cooks, Institution and Cafeteria (35-2012; 435,640 US workers, 137,290 in schools; BLS OEWS May 2023) and "Select food items from serving or storage areas and place them in dishes, on serving trays, or in take-out bags" for Fast Food and Counter Workers (35-3023, titles include Cafeteria Server; 88,840 in schools and 141,760 in contract food service). A food-safety rule for serving staff: "HOLD plates by the bottom or at the very edge" (Alberta Health Services).

**Ability tested:** packing five rigid shapes onto a free, round, wall-less plate held in the air by its rim: the two long portions only fit in bands across the plate, the others must go on top of them or in the space left, each must be laid without nudging those already there off the open rim, and the holding hand must keep the plate level as the load grows. Neither hand can do the other's job: the plate must stay held off the counter while the other hand serves.

## Requirements

`yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source <ARENA_DIR>/yam.env`). `mechkit.py`, `demokit.py`, `gatekit.py` and `taskspace.py` are in this folder. Importing `mechkit` hooks `yam_arena.scene` and `yam_arena.inspect_embodiment` without changing them on disk.

## Run a trial

From this folder:

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 4000 \
  --instruction "Take the top plate off the stack and, holding it up the whole time, serve the steak, the hot dog, both chicken wings and the bread roll onto it from the pans with the serving tongs, no gripper touching the food and nothing sticking out past the plate's edge. Then deliver the served plate to the pick-up zone on the pass and let go."
```

## Scripted demo

The demo stays inside the YAM's figures: the plate pinch is 40 N a finger, the tongs held by one handle at the gripper's force, squeezed 18 degrees against a hinge spring of about 9 N at the tips (an estimate), the press on the far rim about 15 N, the loaded plate 0.915 kg (0.38 kg plate and 0.535 kg of food by the listed masses), held 11 cm from its centre, within the 1.5 kg payload; the demo's left arm lifts, carries and sets it down at this mass.

From this folder:

```bash
python demokit.py task.py OUT --mode full                    # full score, with a video
python demokit.py task.py OUT --no-record --mode none,stack_prop,set_down,idle_server   # the check runs below
python demokit.py task.py OUT --no-record --mode roll_over_edge,dropped_wing,no_handoff
python demokit.py task.py OUT --no-record --mode drop_on_mat
```

The demo reads the plate's pose to find its rim, to level it and to aim each item at its place on the plate, and each item's pose in its pan to take it. It moves nothing except through the grippers.

## The scene and what is simplified

Every part is at true size. Positions are in cm from the midpoint between the arm bases (x forward, y left), heights above the counter, which is the station table.

| Part | In the scene | Source |
|---|---|---|
| Plates | eight Carlisle Dallas Ware 4350002 melamine dinner plates, 26.0 cm across, 2.15 cm tall, 0.38 kg each, stacked at (30, +10), nesting 1.2 cm apart; the seven lower ones one fixed body, the top one free | Boelter listing; rim section and nesting pitch estimated from photos and the case size (48 plates, four stacks of 12 in a 6.38 in box) |
| Pans | two Vollrath 30222 half-size pans, 32.5 x 26.5 x 6.5 cm, fixed as if set in the line's wells, centred (38, -30) and (38, -57) | Vollrath |
| Steak | a seared rump steak with a fat edge along one side, 15 cm long, 7.1 cm wide at one end widening to 9.3 cm (8.2 cm on average), 2.2 cm thick, 270 g, in pan A at (38, -24.5), long axis along x, narrow end toward -x | food-safe display replica, Nasco Life/form grilled strip steak SKU 9283, brought to 270 g with steel shot in a food-safe pouch |
| Hot dog in bun | 16 x 6 x 5 cm, 100 g, in pan A at (38, -36) | display replica, Display Fake Foods hot dog in bun SKU 4634, weighted to 100 g |
| Wings | two whole chicken wings (drumette, flat and tip joined, slightly bent), 9.5 x 4.3 x 3 cm, 60 g each, in pan B at (29, -53) and (29, -64), drumette toward -x | a whole wing printed in PETG from a scan or the shape above, weighted to 60 g (shape from a photo of grilled wings) |
| Bread roll | a round crusty bread roll, 7.6 cm across, 5 cm tall, 45 g, golden-brown domed crust split across the top, floury base, in pan B at (41, -57) | display replica, Display Fake Foods hard roll SKU 9211, weighted to 45 g |
| Serving tongs | 12 in (30.5 cm) one-piece spring tongs, 170 g, tips 10.8 cm apart when open, on a folded cotton bar towel 35 x 12.6 x 0.8 cm centred at (31, -10), hinge at x 46, tips toward -x | Vollrath 4781230 (see Real items); hinge spring force an estimate |
| Pick-up zone | a 35 x 35 x 1 cm stainless shelf pad at (38, +44), just left of the stack, outlined on the counter by a 1.5 cm yellow tape line round it; it takes one 26 cm plate with 4.5 cm to spare each way | the pass where a served plate is collected; real trial: any flat 1 cm board of that size and floor-marking tape |

Departures, each stated once:
- The food is rigid simple solids generated at real portion sizes, each with one convex-hull collider (the hot dog's takes in the frank on top of the bun; a wing's takes in the small notch where it bends). The real trial uses food-safe display replicas, not cooked food (see Building it for a real trial). Friction is estimated: food on melamine 0.4, food on food 0.5, plate on plate and on the pad 0.8.
- The seven lower plates are one fixed body (2.7 kg of plates that do not shift under a hand).
- Physics: rigid bodies at 240 Hz on the GPU.
- Touch is read from distances, not contact sensors: a gripper within 4 mm of an item counts as touching it (Served with the tongs), and the tongs within 8 mm count as still holding it (On the plate). Real trial: the side and top cameras show whether a finger met the food and whether the tongs have opened and left each item.

## Scoring

Mostly positions: four tiers, one per key substep (plate off the stack, all five served onto it, plate delivered into the zone and let go, delivery kept), and one failure that voids the run. Terms:
- **On the plate:** the item is served with the tongs, rests on the plate or on food on it, the tongs are not touching it (no tong point within 8 mm of its collider's surface: let go), and the item's collider lies wholly inside the plate's 13.0 cm outer radius, in the plate's own frame (real: a photo from straight above), its centre above the plate's floor, it has not fallen to the counter or the stack since leaving its pan, it is moving slower than 0.2 m/s, and no gripper point (fingers root to tip, palm and housing, the wrist link) is within 8 mm of its collider's surface.
- **Rests on:** a plate point (72 round its edge, 72 round its foot) within 3 mm of the counter or the zone's pad, or a support point within 3 mm of the plate's real surface (its whole section, foot to edge). The support points lie about 5 mm apart on each pan's wall tops and on the stack: its top plate's well, rim and edge (144 directions x 6 radii) and its outer side (144 directions, every 5 mm up). A food item under the plate within 3 mm of it also counts. `check_score.py` lowers a plate onto each of these (the stack centred, its foot side over the stack's edge every 10 degrees round, a pan wall from three sides, the counter, the zone, a plate on edge against the stack) and checks the test fires within 3.5 mm of the touch.
- **Held clear:** the plate rests on nothing, a gripper point is within 6 mm of its surface (a gripper holds it), and its lowest point is at least 5 cm above the counter (real: the front camera at counter height, against one mark 5 cm above the counter).
- **Served with the tongs:** no gripper point (fingers, pads, palm, housing and wrist link) has come within 4 mm of the item's surface at any time in the run; an item a gripper touched never counts as on the plate.
- **Off the stack:** held clear, or touching no part of the stack with its centre at least 26 cm from the stack's centre.
- **In the zone:** the plate's centre within 4.5 cm of the zone's centre in x and y (so the whole 26 cm plate lies inside the 35 cm line), its lowest point within 3 mm of the pad, and its tilt at most 6 degrees.
- **Dropped:** an item low over the counter outside the pans, or on the stack, and not on the plate. It never counts again.

| Score | Condition, read from the state now |
|---|---|
| 0.25 | the plate is off the stack |
| 0.5 | the plate is off the stack, with all five items on it |
| 0.75 | the plate is in the zone with all five items on it, both grippers (fingers, palm, housing) and the tongs at least 8 mm clear of the plate and the food, and the plate and every item slower than 3 cm/s |
| 1.0 | the 0.75 state, unbroken for the last 2 s (40 steps at 20 Hz) |

Void, score 0: food on the plate while the plate rests on anything (the stack, the counter, a pan's rim, the zone or food under it) before all five items have been on it together while it was held clear; or the plate let fall at any time (any point of its edge or foot going down faster than 0.45 m/s, a free fall from 1 cm, so a plate that tips off a finger counts too). So loading the plate on the stack or propped on its rim, or setting it down to free a hand, scores 0; one arm alone cannot score above 0.25. The 0.2 m/s, 4 mm, 8 mm, 3 cm/s and 0.45 m/s tests are simulation only; real: any visible contact between a gripper and the food in the wrist or counter video voids that item, the item seen still on the plate, a visible gap between fingers and plate or food, nothing moving for 2 s, and the plate never seen released above the pad or sliding off a finger in the front video. Possible scores: 0, 0.25, 0.5, 0.75, 1.0.

## Real items

- Serving tongs: Vollrath 4781230, 12 in one-piece spring tongs (https://www.vollrathfoodservice.com/products/smallwares/kitchen-essentials/tongs/one-piece-color-coded-kool-touch-tongs/4781230). The hand holds one handle and squeezes the pair shut against the hinge spring; the tips open 10.8 cm when let go.

## Building it for a real trial

Real trial (about US$160): the plates (about $5 each), two half-size pans (about $30), the Vollrath tongs (about $8) and a bar towel, the food as food-safe display replicas (steak, hot dog in bun and roll about $15 to $25 each, two PETG-printed wings, steel shot and food-safe pouches to the listed masses) and the pick-up zone (a 35 x 35 cm flat board about 1 cm thick, about $15, and a roll of yellow floor tape). The food's temperature does not matter: neither the act nor the scoring reads it, and replicas are served at room temperature. The grippers are protected because they never touch the food (the scorer voids any item a gripper comes within 4 mm of); for a cooked-food trial, fit nitrile or silicone finger covers and wipe the tongs between runs. The loaded plate weighs 0.915 kg, within the arm's 1.5 kg payload.

## Results

Run on GPU physics at the README's commands, on an RTX 4070 Ti SUPER or an RTX 3090 Ti; the check runs record no video.

| Run | What it does | Score |
|---|---|---|
| `full` (the film) | the whole task, 640 x 360 | 1.0, at step 3340 (167.0 s) |
| `none` | nothing done | 0.0 |
| `bare_hands` | the whole task with the right gripper taking each item by hand instead of with the tongs | 0.0 (gripper contact voids every item, and nothing counts without the tongs) |
| `stack_prop` | the plate lifted and held, then lowered until its foot side rests on the stack's rim (tilted 8 degrees, still in the hand), the steak and hot dog laid on it there, then lifted clear | 0.25 (the steak and hot dog fell off the tilted plate onto the stack; the void for food on a plate that rests on the stack is shown by `set_down`) |
| `set_down` | the plate lifted, put back on the stack, the steak and hot dog laid on it there, then lifted and delivered to the zone | 0.0 (void: food on the plate while it rests on the stack) |
| `idle_server` | the plate lifted and held, the other arm never serves | 0.25 |
| `roll_over_edge` | the roll set on the rim, half past the plate's edge, the plate held to the end | 0.25 |
| `dropped_wing` | one wing dropped on the counter, picked up again and served, the plate held to the end | 0.25 (dropped food never counts) |
| `no_handoff` | all five served, the plate held to the end, never delivered | 0.5 |
| `last_in_tongs` | all five served, the last wing laid on its place but the tongs never let it go, the plate held to the end | 0.25 (the wing still in the tongs never counts as on the plate) |
| `latch_then_rest` | four served, the last wing held 1 cm over its place in the tongs, the plate delivered to the zone, then the wing laid on the resting plate | 0.0 (void at step 3203: food laid on the plate while it rests; the served latch never fired while the wing was in the tongs) |
| `drop_on_mat` | all five served held clear, then the plate let go 4 cm above the zone | 0.0 (void: all five were on the plate, so the void is the drop rule) |
| Dream-1 | the Dream model through the harness's walkthrough command, one valid session, seed 1, 20 rows per request, 4,000 steps (200 s), an RTX 3090 Ti, on the final code | 0: no item was served with the tongs. Film: attached to the PR |

The `full` run scored 1.0 at step 3340 (167.0 s), with no item touched by a gripper; the tong tips close to about 4 mm on each item and open 4 mm each side to let it go. The demo is not certain: an item laid next to another can be nudged by the tips, and the last wing is carried 5 cm higher than the others because at the common height it knocked the hot dog off the plate in an earlier run. A run that loses an item before all five are on the held plate scores 0 once the plate is set down.

## Sources

All meshes are generated by `task.py` and `mechkit.py`. There are no third-party assets.

- Workers: O*NET 35-2012.00 Cooks, Institution and Cafeteria (https://www.onetonline.org/link/summary/35-2012.00) and 35-3023.00 Fast Food and Counter Workers (https://www.onetonline.org/link/summary/35-3023.00). This includes information from O*NET OnLine by the U.S. Department of Labor, Employment and Training Administration (USDOL/ETA), used under the CC BY 4.0 licence; O*NET is a trademark of USDOL/ETA. BLS OEWS May 2023, 35-2012 (https://www.bls.gov/oes/2023/may/oes352012.htm) and 35-3023 (https://www.bls.gov/oes/2023/may/oes353023.htm).
- Plate handling: Alberta Health Services, Serving food properly (https://www.ahs.ca/assets/wf/eph/wf-eh-serving-food-properly.pdf).
- Plate: Carlisle 4350002 Dallas Ware 10 1/4 in melamine dinner plate (https://shop.boelter.com/tabletop-serving/dinnerware/melamine-dinnerware/carlisle-melamine-dinnerware/dallas-ware/carlisle-4350002-dallas-ware-10-1-4-white-dinner-plate-melamine.html).
- Food replicas: Nasco Life/form grilled strip steak SKU 9283 (https://displayfakefoods.com/grilled-strip-steak/), hot dog in bun SKU 4634 (https://displayfakefoods.com/hot-dog-in-bun-with-mustard/), hard roll SKU 9211 (https://displayfakefoods.com/hard-roll/).
- Wing shape: Grilled chicken wings, Wikimedia Commons (https://commons.wikimedia.org/wiki/File:Grilled_chicken_wings.jpg).
- Tongs: Vollrath 4781230 (https://www.vollrathfoodservice.com/products/smallwares/kitchen-essentials/tongs/one-piece-color-coded-kool-touch-tongs/4781230).
- Pans: Vollrath 30222 Super Pan V, half size, 2 1/2 in deep.
- Carlisle, Vollrath, Nasco and Display Fake Foods are named here for identification only; the names are their owners' trademarks.

## Licence

MIT
