# spring-drawer-hold-and-retrieve

## Instruction
"Take the white block out of the cabinet and put it on the green shelf, without letting the drawer close on anything, then leave the drawer closed."

## Category
Impact on jobs

## Why this is a real job
**Real operation.** Parts cabinets, tool chests and dispensing units use self-closing drawers so that they are shut when nobody is using them. Taking a small part out of the back of one means pulling the drawer open against its return spring and keeping it open while the part is reached and lifted out.

**What goes wrong if the hard step is skipped.** A drawer that is let go while a hand or a part is still inside closes on it. The pinch crushes the part or the hand, or jams the slides. The drawer has to be held open by one hand while the other reaches in, and it may only be released once the part is out of the cabinet mouth.

**Which simulated feature carries the difficulty.** The drawer is a real prismatic slider with a return spring and a damper (18 N/m, 5 N s/m, 140 mm travel), so it really does close on its own when released. The block sits under a cover plate and can only be reached when the drawer is out about 90 mm. Holding the handle with one arm while the other lifts the block is therefore physically required, and the scorer latches a zero if the drawer is allowed to close with the block still in the cabinet mouth.

**What is simplified, and why that does not remove the difficulty.** The cabinet is a bench-top size (220 x 160 x 80 mm, a quarter of a real tool-chest drawer) and the spring is soft (18 N/m, at most 2.5 N) so that the soft simulated arms can pull it. The real force on the hand is lower than a real self-closing drawer, so a real pinch would be harder. The difficulty that remains is the coordination: one hand must hold the drawer for the whole time the other hand is inside. The "pinch" itself is a software rule (see "Simulated in software" below); no force on the block is simulated.

## Build specification (everything needed to rebuild the fixture)
All positions are in metres, world frame: x forward, y left, z up; arm bases at `(0, +0.305)` and `(0, -0.305)`; table top `z = 0.7475`. Friction values are the simulator's single (static = dynamic) coefficient for each body. Every number is defined once in `geometry.py` and quoted here.

| Part | Material (real build) | Size (mm) | Mass | Friction | Fixing / position |
|---|---|---|---|---|---|
| Cabinet shell | 19 mm birch plywood sides, 15 mm back, 10 mm floor, steel ballast plate under the floor | 220 x 160 x 80 outer, open front | 8.0 kg total | 1.0 (rubber feet) | Not bolted down: its mass and rubber feet hold it against the 2.5 N maximum pull. Front face at `x = 0.305`; centre `(0.415, 0.000, 0.7875)`. |
| Cover plate | 20 mm plywood, part of the shell | covers the rear half, `x >= 0.380`, 20 mm thick, bottom face at `z = 0.8075` | included above | 1.0 | Hides the block until the drawer is out about 90 mm. |
| Drawer | 10 mm HDPE walls, 6 mm HDPE floor | 190 x 120 x 30 outer, open top | 0.40 kg | default | Runs in the cabinet on two ball-bearing runners; hard stops at 0 and 140 mm. Closed position: front at `x = 0.305`, centre `x = 0.400`, `z = 0.7725`. |
| Return spring | Steel extension spring, zero preload when closed | rate 18 N/m | n/a | n/a | Anchored on the cabinet back wall at one end and the drawer rear wall at the other. Force is 18 N/m x travel, 2.52 N at 140 mm. |
| Damper | Oil damper in parallel with the spring | 5 N s/m | n/a | n/a | Mounted beside the spring; removes the bounce at the end of the stroke. |
| Handle | Anodised aluminium block, orange | 25 x 60 x 35 | part of drawer | default | Bolted to the drawer front, centre `(0.2925, 0.000, 0.7725)` closed. |
| White block | ABS cube with a 3 mm silicone skin | 40 x 40 x 40 | 0.10 kg | 0.8 | Complete start position: `(0.450, 0.000, 0.7845)`, resting on the drawer floor under the cover plate, upright, axis-aligned (no rotation). |
| Green shelf | 10 mm aluminium plate with a rubber top pad | 130 x 130 x 10 | 3.0 kg | 1.0 | Clamped to the table, centre `(0.300, -0.280)`, top face at `z = 0.7575`. A block resting on it has centre `z = 0.7775`. |
| Open lamp | 4 mm cyan marker (no collision) | 25 x 25 x 4 | 5 g | n/a | On the cover plate at `(0.450, 0.000)`; shown when the drawer is out 90 mm or more, hidden otherwise. |

## Layout facts the scorer uses (the tolerances, in one table)
| Quantity | Value | Where it is used |
|---|---|---|
| Drawer counts as open | displacement >= 90 mm, for a total of 1.0 s | 0.25 step |
| Block counts as lifted | block centre >= 30 mm above its rest height (`z >= 0.8145`) | 0.50 step (an event, latched) |
| Block counts as on the shelf | centre within 65 mm of the shelf centre in x and y (inside the 130 mm footprint) and within 20 mm of rest height `z = 0.7775`, **at the current step** | 0.75 step |
| Drawer counts as closed | displacement <= 5 mm | 1.00 step |
| Arms count as clear | no arm point with `x >= 0.245` and `|y| <= 0.130` (60 mm in front of the cabinet face and within 50 mm of its width) | 1.00 step |
| Arms count as still | every arm below 5 mm/s linear and 0.2 rad/s angular speed | 1.00 step |
| Hold | 1.0 s = 20 control steps at 20 Hz | 1.00 step |
| Pinch | drawer less than 30 mm out while the block is within 60 mm behind the cabinet front face (`x <= 0.365`), inside the drawer width, below the cabinet top | zero-gate, latched |
| Dropped | block centre below `z = 0.6975` | zero-gate, latched |

## Scoring Ladder (end-state: the score is that of the current step)
- `0.25`: drawer opened >= 90 mm for a total of 1.0 s (the opening is an event, kept once it has happened).
- `0.50`: block has been lifted >= 30 mm clear of the drawer floor while the drawer was opened (an event, kept).
- `0.75`: block is resting on the green shelf **now** (a block taken off the shelf again drops back to 0.50).
- `1.00`: drawer closed (<= 5 mm), block on the shelf, grippers open, arms clear and still, all held for 1.0 s / 20 steps. It stays 1.00 only while those conditions keep holding.
- A latched zero-gate (pinch or drop) forces 0.0 for the rest of the run. The score never keeps a best value: undoing a goal lowers the score again.

## Zero-Gates
- **PINCH:** the drawer is less than 30 mm out while the block is still within 60 mm behind the cabinet front face. Score 0.0, latched.
- **Dropped block:** the block falls below the table. Score 0.0, latched.

## Simulated in software (what is not physical)
- The cyan lamp is a marker moved by the scorer when the drawer displacement is >= 90 mm. It is not an electrical lamp.
- The pinch is a position rule. The simulator does not compute a squeezing force on the block or a hand; the rule stands for "the drawer is closing on the block's position".
- "Lifted" and "on the shelf" are position checks on the block centre. The scorer does not read which gripper holds the block or the handle.
- Real materials in the table are what the fixture would be built from; the simulator uses only the masses, sizes and friction coefficients listed.

## Timing and Margins
The plan is stretched by `DEMO_TIME_SCALE` (1.8). The run limit is 2,500 steps (125 s at 20 Hz) and the dream run uses all 2,500 steps. The demo finishes inside the limit (see the measured table). Joint margins and clearances are checked by `check_reach.py` and `check_collisions.py`.

## Ablations and Predicted Scores
| Mode | What it does | Predicted | Why |
|---|---|---|---|
| `demo` | hold the handle, lift the block out, shelve it, release | 1.00 | full ladder |
| `no_hold` | opens the drawer, lets go, the drawer closes before the block is reached | 0.25 | the drawer opened but the block was never lifted |
| `pinch` | lets the drawer go while the block is still in the cabinet mouth | 0.00 | latched pinch |
| `leave_open` | shelves the block but keeps holding the handle to the end | 0.75 | drawer is never closed |
| `settle` | nothing moves for 10 s | 0.00 | start state scores zero |
| `dream` | the Dream-1 policy, 2,500 steps | 0.00 | nothing reaches the block |

## Measured in the Simulator
Measured with the end-state scorer. Score is the final step of each run. Run tags `rv2` and `rv4`.

| Mode | Predicted | Measured (GPU) | Steps |
|---|---|---|---|
| demo | 1.00 | 1.00 | 532 |
| no_hold | 0.25 | 0.25 | 284 |
| pinch | 0.00 | 0.00 (PINCH fault) | 446 |
| leave_open | 0.75 | 0.75 | 680 |
| settle | 0.00 | 0.00 | 400 |
| Dream-1 | 0.00 | 0.00 (nothing moved into a goal state) | 2,500 |

The previous scorer (best score kept) measured the same scores. The demo finishes in 532 steps, well inside the 2,500-step run limit.

## Changes made after the first simulator runs
1. The cabinet and drawer were solid boxes containing the block. They are now an open-front shell and an open-top drawer (floor and four walls); the block rests on the drawer floor and a cover plate (front edge at `x = 0.380`) hides it until the drawer is out about 90 mm.
2. The `-X` slider reports a negative displacement when pulled out; the scorer uses its absolute value.
3. The handle is held with a real grasp (50 mm jaw gap on the 60 mm handle, approached from the front); the block is approached with a 60 mm jaw gap so the fingers clear the drawer walls.
4. The PINCH rule is "drawer < 30 mm out while the block is within 60 mm behind the cabinet front face".
5. `leave_open` keeps the left hand on the handle until the end.

## Changes made in this revision (reviewer feedback and checks)
1. Added the build specification above: materials, wall and cover thicknesses, runners, spring mounting, damper, how the cabinet is held, and the complete block start position.
2. The instruction now states the goal and the limit ("without letting the drawer close on anything") and no longer gives the method.
3. End-state scoring: the shelf check is now the current state, and the score no longer keeps the best value of the run. Added tests that undoing a goal lowers the score, that 1.00 needs the full hold, and that the start state scores 0.
4. Corrected stale comments: the drawer wall is 10 mm (not 15 mm) and the lamp is on the cover plate at `(0.450, 0.000)`.

## Board Check
Titles below were read from the board; I looked at the PR descriptions that exist.
- **#787, keep-a-key-turned-while-pulling-a-drawer-open (closed).** One hand keeps a key turned while the other pulls a drawer by its handle. Different: there the held thing is a key in a lock and the drawer does not close by itself; here the held thing is the drawer itself against a return spring, a part must be taken out through a covered back compartment, and a latched pinch rule applies if the drawer closes on the part.
- **#1191, take a drawer right out of its chest past the anti-pull-out stop (closed).** Different: that task removes the whole drawer past a stop; this one keeps the drawer in the cabinet and retrieves a part from it.
- **#1215, take the socket out of a tool chest whose drawer interlock holds it shut (closed).** Different: an interlock has to be defeated before the drawer opens; here nothing locks the drawer, the spring closes it.
