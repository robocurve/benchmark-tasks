# hot-swap-failed-drive-sled

## Instruction
"One bay has a red lamp. Replace its drive sled with the spare that matches the bay's key tab, and put the removed sled on the discard pad, without moving the other two sleds."

## Category
Impact on jobs

## Why this is a real job
**Real operation.** In a storage server, drives sit in hot-swappable sleds (caddies) in front-loading bays. When one fails, a technician pulls that sled while the other bays keep running, and slides a spare in whose key matches the bay, then checks that the bay lamp goes green. The failed sled goes to a discard bin.

**What goes wrong if the hard step is skipped.** A sled pulled without releasing its latch is forced out and can bend the latch or the backplane connector. Disturbing a neighbouring healthy sled drops that drive out of the array. A wrongly keyed spare does not engage and the bay stays red.

**Which simulated feature carries the difficulty.** Three sleds sit side by side in 104 mm bay slots with only 12 mm of clearance on each side of a 80 mm sled, so a hand pulling the middle one can easily knock a neighbour (a 5 mm disturbance is a fault). The latch is a real spring slider (100 N/m, 8 mm travel) that must be held down by one arm while the other pulls, and the spare has to be picked out of two look-alikes by matching its handle colour to the key tab of the failed bay.

**What is simplified, and why that does not remove the difficulty.** The rack is bench-top scale (160 x 400 x 60 mm, a quarter of a real 1U chassis). The sleds slide freely in the slot; the latch retention, the keying and the lamp are software rules (see "Simulated in software"). The difficulty that remains is bimanual coordination in a tight space and choosing the right part.

## Build specification (everything needed to rebuild the fixture)
All positions are in metres, world frame: x forward, y left, z up; arm bases at `(0, +0.305)` and `(0, -0.305)`; table top `z = 0.7475`. Friction values are the simulator's single (static = dynamic) coefficient per body. Every number is defined once in `geometry.py`.

| Part | Material (real build) | Size (mm) | Mass | Friction | Fixing / position |
|---|---|---|---|---|---|
| Rack chassis | 10 mm floor and 9 mm back plate, 8 mm top plate, dividers and outer walls of aluminium, steel ballast under the floor | 160 x 400 x 60, open front | 10.0 kg | 1.0 | Not bolted: its mass and rubber feet hold it. Front face at `x = 0.330`, centre `(0.410, 0.000, 0.7775)`. The top plate starts at `x = 0.355` (25 mm behind the front face). |
| Bay slots | formed by the dividers | three slots 104 wide, pitch 120, centres `y = +0.120`, `0.000`, `-0.120` | included | 1.0 | 12 mm clearance each side of an 80 mm sled; 12 mm of head room above a 25 mm sled (bay floor at `z = 0.7575`). |
| Sled (x3) | aluminium tray, 1.5 mm walls | 150 x 80 x 25 | 0.050 kg | 1.5 | Seated: centre `x = 0.405`, `z = 0.770`. Free sliding rigid body in the simulator. |
| Sled handle (x3 plus 2 spares) | black plastic block on the failed sled and the healthy sleds; blue or orange on the spares | 30 deep x 45 wide x 30 high, on the sled front face | part of sled | 1.5 | Seated handle centre `x = 0.315`. |
| Latch button (x3) | brass button on a spring plunger | 40 x 40 x 12 | 5 g | n/a | On the top plate at `x = 0.405`, 55 mm to the left (+y) of each bay's centre line. Real slider: 100 N/m spring, 5 N s/m damping, 8 mm stroke to a hard stop; pressed when travel >= 6.4 mm. |
| Key tab (x3) | coloured anodised aluminium tab on the top plate | 20 x 20 x 3 | part of rack | 1.0 | At `x = 0.372` in front of each bay's latch button (centre on the bay centre line): bay 0 and bay 2 orange, **bay 1 (the failed bay) blue**. A spare fits a bay when its handle colour equals the bay's key tab colour. |
| Status lamp (x3) | 25 mm marker (no collision) | 25 diameter x 4 | 5 g | n/a | On the top plate at `x = 0.455` over each bay. Red on the failed bay at the start; green on bays 0 and 2. A separate green marker is shown over bay 1 once it is solved. |
| Spare stand | 20 mm aluminium plate | 180 x 180 x 20 | 4.0 kg | 1.0 | Clamped to the table, centre `(0.280, +0.300)`, top at `z = 0.7675`. |
| Blue spare | sled as above, blue handle | 150 x 80 x 25 | 0.050 kg | 1.5 | Exact start position `(0.280, +0.250, 0.780)`, long side along x, handle on the side facing the robot (the same orientation as a seated sled). |
| Orange spare | sled as above, orange handle (wrong key for bay 1) | 150 x 80 x 25 | 0.050 kg | 1.5 | Exact start position `(0.280, +0.350, 0.780)`. |
| Discard pad | rubber pad | 130 x 130 x 10 | 2.0 kg | 1.0 | Clamped to the table, centre `(0.280, -0.300)`, top at `z = 0.7575`. A sled resting on it has centre `z = 0.770`. |

### Latch-to-sled retention and spare keying in a real build
- **Latch retention.** Each sled has a 25 mm long detent slot in its right rail. A spring-loaded 3 mm steel latch pin rides in that slot and keeps the sled from leaving the bay by more than 3 mm. Pressing the brass button (8 mm stroke, spring rate 100 N/m) lifts the pin out of the slot so the sled can be drawn out.
- **Keying.** Each sled has a 6 x 6 mm key rib on its underside at an offset set by its key colour (blue +20 mm from the sled centre line, orange -20 mm). The bay floor has a matching 6 x 6 mm groove. A wrongly keyed spare stops about 12 mm before it is seated and its lamp stays red.
- **Lamp.** A bay lamp turns green when a correctly keyed sled is seated and the latch is released.

## Layout facts the scorer uses (tolerances in one table)
| Quantity | Value |
|---|---|
| Failed sled counts as pulled | centre moved >= 60 mm from seated (`x <= 0.345`) while the latch was pressed (an event, kept) |
| Latch counts as pressed | button displacement <= -6.4 mm |
| Forced-pull FAULT | failed sled pulled more than 3 mm and up to 25 mm while the latch is not pressed. Latched. |
| Healthy-bay FAULT | a healthy sled moves more than 5 mm in x, y or z. Latched. |
| Removed sled counts as on the discard pad | **at the current step**: centre within 40 mm of the pad centre in x and in y, and within 10 mm of resting height (`z = 0.770`) |
| Spare counts as seated | spare centre within 15 mm of the bay centre in y, within 15 mm of resting height, and within 8 mm of the seated position in x (>= 95 % of the 150 mm length in the bay), **at the current step** |
| Lamp green | the blue spare is seated and the latch is released |
| Arms count as clear | no arm point with `x >= 0.280` and `|y| <= 0.250` |
| Arms count as still | every arm below 5 mm/s linear and 0.2 rad/s angular speed |
| Hold | 1.0 s = 20 control steps at 20 Hz |

## Scoring Ladder (end-state: the score is that of the current step)
- `0.25`: the failed sled was pulled out >= 60 mm with the latch held (an event, kept; no fault).
- `0.50`: the removed sled is resting on the discard pad now.
- `0.75`: the removed sled is resting on the discard pad **and** the blue spare is seated >= 95 % in the failed bay now (this is the condition the scorer applies; it now matches the published text).
- `1.00`: lamp green (blue spare seated, latch released), removed sled on the pad, both grippers open, both arms clear and still, healthy sleds undisturbed, all held for 1.0 s / 20 steps. It stays 1.00 only while those conditions keep holding.
- A latched FAULT forces 0.0 for the rest of the run. The score never keeps a best value: moving the spare out again or lifting the removed sled off the pad lowers the score.

## Zero-Gates
- **Forced pull:** moving the failed sled more than 3 mm out of its bay (up to 25 mm) while the latch is not pressed. Score 0.0, latched.
- **Healthy bay disturbed:** any healthy sled moves more than 5 mm. Score 0.0, latched.

## Simulated in software (what is not physical)
- The latch retention is a rule: the sled slides freely in the slot (friction 1.5), and pulling it more than 3 mm within the first 25 mm without the latch held is scored as a FAULT. No retaining pin is modelled.
- Keying is a rule on the handle colour of the spare seated in the bay; no rib or groove is modelled, so a wrong spare can be pushed in, and the scorer holds the lamp red.
- The lamp is a marker that the scorer moves.
- Real materials in the table are what the fixture would be built from; the simulator uses only the sizes, masses and friction coefficients listed.

## Timing and Margins
The plan runs one arm at a time near the middle of the rack and is stretched by `DEMO_TIME_SCALE` (1.8). The run limit is 2,500 steps (125 s at 20 Hz); the dream run uses all 2,500 steps. Joint margins and clearances are checked by `check_reach.py` and `check_collisions.py`.

## Ablations and Predicted Scores
| Mode | What it does | Predicted | Why |
|---|---|---|---|
| `demo` | left holds the latch, right pulls the sled to the pad, left seats the blue spare | 1.00 | full ladder |
| `no_latch` | pulls the sled without the latch | 0.00 | forced-pull FAULT |
| `wrong_bay` | pulls a healthy bay's sled | 0.00 | healthy-bay FAULT |
| `wrong_spare` | seats the orange spare | 0.50 | the sled is on the pad but the lamp stays red, so no 0.75 |
| `release_early` | lets the latch go before pulling | 0.00 | forced-pull FAULT |
| `settle` | nothing moves for 10 s | 0.00 | start state scores zero |
| `dream` | the Dream-1 policy, 2,500 steps | 0.00 | nothing completes |

## Measured in the Simulator
Measured with this version (end-state scorer, 40 mm / 10 mm pad tolerance, key tabs in the scene). Score is the final step of each run. Run tags `rv1` to `rv3`.

| Mode | Predicted | Measured (GPU) | Steps |
|---|---|---|---|
| demo | 1.00 | 1.00 | 519 |
| no_latch | 0.00 | 0.00 (forced pull FAULT) | 564 |
| wrong_bay | 0.00 | 0.00 (healthy bay moved FAULT) | 323 |
| wrong_spare | 0.50 | 0.50 | 618 |
| release_early | 0.00 | 0.00 (forced pull FAULT) | 633 |
| settle | 0.00 | 0.00 | 400 |
| Dream-1 | 0.00 | 0.00 (nothing moved into a goal state) | 2,500 |

The earlier version (best-score scorer, loose pad area, no key tab) measured the same scores. The demo finishes in 519 steps, well inside the 2,500-step run limit.

## Changes made after the first simulator runs
1. The rack was a solid block containing the sleds. It is now an open-front shell with a slot per bay (104 mm wide, 12 mm clearance per side); the top plate starts 25 mm behind the front face so the gripper palm can reach the handles.
2. The sleds are 25 mm thick, light (50 g), friction 1.5, with 30 mm deep handles, so the fingers can lift and carry them.
3. The arms crowded each other at the middle of the rack. The roles are mirrored: the left arm holds the latch and fetches the spare from the left stand; the right arm pulls the old sled and parks it on the right pad.
4. The plan runs one arm at a time near the middle and pulls the sled fully out (160 mm) before lifting it.
5. The green lamp is a separate marker.

## Changes made in this revision (reviewer feedback and checks)
1. **Published 0.75 now equals the scorer.** The ladder text said "blue spare seated" while the scorer also required the removed sled on the pad. Both now require the sled on the pad.
2. **The instruction now tells the robot to put the removed sled on the discard pad** and gives the goal and the limit, not the method (no "hold the latch, pull"). The spare is named by the key tab, not by its handle colour.
3. **Added the key tabs to the scene** so the correct spare can be identified from what the robot sees.
4. **Specified the pad and arm-clear tolerances** (table above) and tightened the pad tolerance from "anywhere within the 130 mm pad" to 40 mm / 10 mm.
5. **Added the build specification**: materials, sizes, masses, friction, exact spare start positions, and a real latch-to-sled retention and keying mechanism, with the software rules listed separately.
6. **End-state scoring.** The score no longer keeps the best value; the pad and spare flags are current-state; tests cover undoing a goal, the pad tolerances, the 0.75 condition and the key tabs.

## Board Check
Titles below were read from the board; I looked at the PR descriptions that exist.
- **#611, swap a failed drive for the spare and send the failed one to secure destruction (closed).** A drive swap with a disposal step. Different: here the swap is in a three-sled rack where a healthy neighbour must not move, a latch must be held down while the sled is pulled (a forced pull is a fault), and the spare must be chosen by matching a key tab against two look-alikes.
- **#639, find the drive the controller released by its offset, and replace it with the spare (closed).** Different: there the failed drive is identified by an offset reading; here the failed bay is shown by its lamp and the difficulty is the latch, the tight neighbours and the keying.
- **#638, push a wide GPU sled home with both arms without racking it (closed).** Different: there a wide sled is pushed in with both arms; here a sled is pulled out with a held latch and a different sled is inserted.
