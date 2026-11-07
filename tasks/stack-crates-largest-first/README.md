# Stack Crates Largest First (`stack-crates-largest-first`)

Stack three crates of different sizes onto a pallet marker in descending size order (largest at the bottom, smallest on top) with centered placement to form a stable vertical stack without leaning or falling.

## Instruction
"Stack the three crates on the pallet mark with the biggest at the bottom and the smallest on top, without letting the stack lean or fall."

## Category
Impact on jobs

## Physical Layout
| Object | Dimensions (mm) | Start Position (m) | Mass / Material | Predicted Role |
|---|---|---|---|---|
| Pallet Marker | 110 x 110 x 2 | `(0.260, 0.000, 0.7485)` | Thin visual plate | Target staging area |
| Large Crate (Blue) | 80 x 80 x 40 | `(0.280, +0.280, 0.7675)` | 0.06 kg, friction 1.2 | Level 1 foundation |
| Medium Crate (Orange) | 60 x 60 x 40 | `(0.380, -0.280, 0.7675)` | 0.05 kg, friction 1.2 | Level 2 intermediate |
| Small Crate (Yellow) | 45 x 45 x 40 | `(0.300, -0.220, 0.7675)` | 0.04 kg, friction 1.2 | Level 3 top crown |

## Scoring Ladder
| Score | Condition |
|---|---|
| 0.25 | Large crate placed on pallet mark with xy offset <= 15 mm |
| 0.50 | Medium crate placed on large crate with xy offset <= 12 mm from the pallet centre line |
| 0.75 | Small crate placed on medium crate with xy offset <= 12 mm from the pallet centre line |
| 1.00 | Stack stable for 2.0 s (40 steps at 20 Hz): all offsets <= 12 mm, tilt < 5 deg, arms clear (> 60 mm) |

## Zero-Gates
- **Floor fault**: Any crate falling off the table (`z < TOP - 0.05 m`) immediately and permanently zeroes the score to 0.00.
- **Wrong order rule**: A crate cannot be supported by a smaller crate; placing Large on Medium or Medium on Small prevents gaining higher level score.

## Timing Margin Table
| Requirement | Specification | Scripted Arm Achieved | Safety Margin |
|---|---|---|---|
| Place descent speed | Vertical descent <= 50 mm/s | 25 mm/s | +50% |
| Release pause | >= 0.2 s settle | 0.4 s | +100% |
| Stack stability hold | 2.0 s (40 steps at 20 Hz) | 2.5 s (50 steps) | +0.5 s (+25%) |

## Measured in the Simulator
- Reach: worst position error 0.08 mm, worst rotation error 0.01 deg
- Min joint margin: 0.161 rad (limit >= 0.02 rad)
- Arm-arm clearance: 245.26 mm (limit >= 30 mm)
- Min scene clearance: 11.14 mm (limit >= 1.0 mm)
- Collisions: 0 collisions detected, all clearances satisfied
- GPU runs (20 Hz): demo 1.00, settle 0.00, Dream-1 0.00, `wrong_order` 0.00, `offset` 0.50, `leave_small` 0.50, `knock` 0.75
- Stack heights in the demo: 0.7675 / 0.8075 / 0.8475 m, each crate within 10 mm of the pallet centre

## Ablation Results (measured; predictions in brackets where they differed)
- `demo`: 1.00 (correct big-to-small stacking sequence)
- `wrong_order`: 0.00 [predicted 0.25] (medium placed first on the pallet does not count as level 1, and the large crate on top of it is rejected by the support rule)
- `offset`: 0.50 [predicted 0.75] (small crate 25 mm off exceeds the 12 mm limit, so the small crate never counts; medium on large still does)
- `leave_small`: 0.50 (large and medium stacked; small remains on the table)
- `knock`: 0.75 [predicted 0.00] (measured at the end of the run; the sweep did not clear the stack off the table, record only)
- `settle`: 0.00 (idle hold at home position)
- `dream`: 0.00 (baseline)

## Changes Made After the First Simulator Runs
- Medium and small crate offsets are now measured from the pallet centre line (as in the instruction spec), not from the crate below, which counted a perfect stack as 0.25.
- `demo()` in `task.py` was fixed to read the mode from the environment, so ablation and settle runs no longer replay the full demo.
- Pallet moved to x = 0.26 m for reach (from the first reach check).

## Fidelity Notes
All three crates are modelled as free rigid bodies with realistic mass and high Coulomb friction (1.2). Pallet is a flat non-colliding visual square. Real-world basis: palletising and case-stacking in logistics warehouses.

