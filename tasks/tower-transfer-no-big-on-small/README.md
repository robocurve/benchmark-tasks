# Tower Transfer No Big On Small (`tower-transfer-no-big-on-small`)

Move a three-block tower from the left target mark to the right target mark using an intermediate middle mark as a relay, keeping the biggest block at the bottom and never setting a bigger block on a smaller block.

## Instruction
"Move the three-block tower from the left mark to the right mark, keeping the biggest at the bottom, and never set a bigger block on a smaller one."

## Category
Impact on jobs

## Physical Layout
| Object | Dimensions (mm) | Start Position (m) | Mass / Material | Role |
|---|---|---|---|---|
| Left Mark | 100 x 100 x 2 | `(0.240, +0.090, 0.7485)` | Flat white mark, collision=False | Initial tower staging base |
| Middle Mark | 100 x 100 x 2 | `(0.320, 0.000, 0.7485)` | Flat white mark, collision=False | Bimanual relay buffer mark |
| Right Mark | 100 x 100 x 2 | `(0.240, -0.090, 0.7485)` | Flat white mark, collision=False | Target tower assembly base |
| Large Block (Blue) | 80 x 80 x 25 | `(0.240, +0.090, 0.7600)` | 0.06 kg, friction 1.2 | Level 1 bottom foundation |
| Medium Block (Orange) | 60 x 60 x 25 | `(0.240, +0.090, 0.7850)` | 0.05 kg, friction 1.2 | Level 2 intermediate block |
| Small Block (Yellow) | 45 x 45 x 25 | `(0.240, +0.090, 0.8100)` | 0.04 kg, friction 1.2 | Level 3 top block |

## Move Allocation Table (Optimal 7-Move Hanoi)
| Move | Block | From | To | Acting Arm | Rationale / Hand-over |
|---|---|---|---|---|---|
| 1 | Small | LEFT | RIGHT | Left Arm | Starts on LEFT; Right arm clear at home |
| 2 | Medium | LEFT | MIDDLE | Left Arm | Starts on LEFT; staged to relay mark |
| 3 | Small | RIGHT | MIDDLE | Right Arm | Starts on RIGHT; placed on Medium at MID |
| 4 | Large | LEFT | RIGHT | Left Arm | Starts on LEFT; placed on target base |
| 5 | Small | MIDDLE | LEFT | Left Arm | Retrieved from MID; clears Medium |
| 6 | Medium | MIDDLE | RIGHT | Right Arm | Ends on RIGHT; placed on Large at RIGHT |
| 7 | Small | LEFT | RIGHT | Left Arm | Starts on LEFT; crowns tower at RIGHT |

## Scoring Ladder
| Score | Condition |
|---|---|
| 0.25 | Small block resting on MIDDLE or RIGHT mark (first move completed) |
| 0.50 | Large block off LEFT mark and resting on the table (Level 1) at MIDDLE or RIGHT |
| 0.75 | Large block on RIGHT mark with a smaller block on it OR both other blocks on MIDDLE |
| 1.00 | All three blocks on RIGHT in order (Large bottom, Medium middle, Small top), offsets <= 12 mm from RIGHT mark centre, tilt < 5 deg, arms clear (> 60 mm), held stable for 2.0 s (40 steps at 20 Hz) |

## Zero-Gates
- **Bigger on smaller rule**: Any bigger block resting on a smaller block (centre within 40 mm in xy and higher by >= 20 mm after resting < 5 mm/s for 0.3 s) permanently latches `rule_broken` = score 0.00.
- **Floor fault**: Any block falling off the table (`z < TOP - 0.05 m`) immediately and permanently zeroes the score to 0.00.

## Timing Margin Table
| Requirement | Specification | Scripted Arm Achieved | Safety Margin |
|---|---|---|---|
| Vertical descent | <= 50 mm/s | 25 mm/s | +50% |
| Grasp squeeze hold | >= 0.2 s | 0.3 s | +50% |
| Release dwell pause | >= 0.2 s | 0.35 s | +75% |
| Placement overshoot | >= 3 mm | 5 mm (`DEMO_PLACE_DZ`) | +66% |
| Lift after release | Straight up >= 40 mm | 60 mm before lateral move | +50% |
| Final stable hold | 2.0 s (40 steps at 20 Hz) | 2.5 s (50 steps) | +0.5 s (+25%) |

## Measured in the Simulator (GPU, 20 Hz)

| Mode | Predicted | Measured | Note |
|---|---|---|---|
| `demo` | 1.00 | 1.00 | all three blocks on RIGHT, held 3.95 s (2.0 s needed) |
| `big_on_small` | 0.00 | 0.00 | `rule_broken` latched |
| `leave_one` | 0.75 | 0.50 | predicted 0.75; the run stops early and the tower is left on the LEFT mark, so only the 0.50 milestone was reached |
| `wrong_target` | 0.50 | 0.50 | tower built on MIDDLE |
| `settle` | 0.00 | 0.00 | idle |
| `dream` | 0.00 | 0.00 | Dream-1 baseline |

Change made after the first GPU run: the demo ended at 0.75 because the medium and small blocks landed 13.6 mm and
12.3 mm from the RIGHT mark centre (limit 12 mm). Every placement on that mark came out 8-10 mm short in x, so
placements on the RIGHT mark are now shifted 8 mm forward (`DEMO_R_PLACE_DX`, default 0.008). The 12 mm limit is unchanged.

## Ablation Predictions (Honest offline numbers)
- `demo`: 1.00 (correct 7-move Hanoi sequence)
- `big_on_small`: 0.00 (Medium placed on Small at MIDDLE, latches `rule_broken` = 0.00)
- `leave_one`: 0.50 (incomplete run; first predicted 0.75, measured 0.50, see the measured table)
- `wrong_target`: 0.50 (tower built on MIDDLE instead of RIGHT)
- `settle`: 0.00 (idle hold at home position)
- `dream`: 0.00 (baseline)

## Fidelity Notes
Simulated in Isaac Lab Arena with PhysX 5 rigid body physics, friction coefficient 1.2 on all blocks, exact geometric collisions, and realistic 20 Hz control loop. No artificial magnets, kinematic attachments, or simulated state cheats are used.
