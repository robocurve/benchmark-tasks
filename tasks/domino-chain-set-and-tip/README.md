# Domino Chain Set and Tip (`domino-chain-set-and-tip`)

Stand four dominoes on end on white target marks spaced 30 mm apart along the X axis, then tip the first domino to initiate a cascading toppling reaction that fells all four dominoes.

## Instruction
"Stand the four dominoes on end on the white marks 30 mm apart, then tip the first one so every domino falls."

## Category
Impact on jobs

## Physical Layout
| Object | Dimensions (mm) | Start Position (m) | Mass / Material | Role |
|---|---|---|---|---|
| Mark 0 | 20 x 40 x 2 | `(0.260, 0.000, 0.7485)` | Flat white mark, collision=False | Base for first domino |
| Mark 1 | 20 x 40 x 2 | `(0.310, 0.000, 0.7485)` | Flat white mark, collision=False | Base for second domino |
| Mark 2 | 20 x 40 x 2 | `(0.360, 0.000, 0.7485)` | Flat white mark, collision=False | Base for third domino |
| Mark 3 | 20 x 40 x 2 | `(0.410, 0.000, 0.7485)` | Flat white mark, collision=False | Base for fourth domino |
| Domino 0 (Orange) | 20 x 40 x 70 | `(0.280, +0.200, 0.7825)` | 0.02 kg, friction 0.45 | First domino in chain |
| Domino 1 (Blue) | 20 x 40 x 70 | `(0.380, +0.200, 0.7825)` | 0.02 kg, friction 0.45 | Second domino in chain |
| Domino 2 (Yellow) | 20 x 40 x 70 | `(0.280, -0.200, 0.7825)` | 0.02 kg, friction 0.45 | Third domino in chain |
| Domino 3 (Green) | 20 x 40 x 70 | `(0.380, -0.200, 0.7825)` | 0.02 kg, friction 0.45 | Fourth domino in chain |

## Task Execution Sequence
1. Left Arm picks Domino 0 from left staging and stands it on Mark 0 (`(0.260, 0.000)`).
2. Left Arm picks Domino 1 from left staging and stands it on Mark 1 (`(0.310, 0.000)`).
3. Right Arm picks Domino 2 from right staging and stands it on Mark 2 (`(0.360, 0.000)`).
4. Right Arm picks Domino 3 from right staging and stands it on Mark 3 (`(0.410, 0.000)`).
5. Both arms return home with fingers open, clearing the domino line.
6. Nearest arm (Left Arm) moves behind Domino 0 and executes a closed-finger push forward at 60 mm height (`z = TOP + 0.060 m`).
7. First domino tips forward (> 20 deg), impacts Domino 1, cascading through Domino 2 and 3 until all 4 are fallen toward +x.
8. Left arm retreats straight up and returns home, leaving both arms clear (> 60 mm).

## Scoring Ladder
| Score | Condition |
|---|---|
| 0.25 | Two dominoes standing on end on marks (tilt < 6 deg, resting 0.3 s, on mark within 8 mm) |
| 0.50 | All four dominoes standing on end on marks |
| 0.75 | First domino tipped (tilt > 20 deg) while other dominoes are standing (cascade initiated) |
| 1.00 | All four dominoes fallen toward +x (tilt > 60 deg toward +x) within 5.0 s of tip, arms clear (> 60 mm) |

## Zero-Gates
- **Premature fall**: Any domino falling (tilt > 45 deg) before all four dominoes are standing marks `premature = True`, capping the score at 0.25 max.
- **Floor fault**: Any domino falling off the table (`z < TOP - 0.05 m`) zeroes the score permanently.

## Timing Margins & Release Lesson 6
- **Straight up release (lesson 6)**: After releasing each 70 mm tall domino, the arm ascends vertically to clearance height before moving laterally, avoiding contact with upright dominoes.
- **Arm-arm separation**: Left arm operates on +y staging, Right arm on -y staging. When one arm places on midline (y = 0.0), the other waits at home (|y| = 0.280 m), maintaining > 230 mm clearance.

## Ablation Predictions (Honest offline numbers)
- `demo`: 1.00 (correct bimanual placement, tip push, and cascade)
- `gap_too_wide`: 0.25 (measured; with the wide gap the placement itself knocks dominoes over before all four stand, so the premature cap applies. The first offline prediction of 0.75 assumed all four would stand)
- `tip_too_early`: 0.25 (first domino tipped prematurely before second arm places remaining dominoes)
- `no_tip`: 0.50 (all four dominoes placed standing on marks, arms return home without tipping)
- `settle`: 0.00 (idle hold at home position)
- `dream`: 0.00 (baseline)

## Measured in the Simulator
| Mode | Predicted | Measured | Notes |
|---|---|---|---|
| `demo` | 1.00 | 1.00 | all four set, first tipped, all four fell |
| `gap_too_wide` | 0.75 | 0.25 | premature cap: dominoes fell before all four were standing (prediction corrected above) |
| `tip_too_early` | 0.25 | 0.25 | premature cap |
| `no_tip` | 0.50 | 0.50 | four standing, nobody tips the first |
| `settle` | 0.00 | 0.00 | arms hold at home |
| `dream` | 0.00 | 0.00 | Dream-1 baseline |

The demo took three tuned runs. The last fix was the right arm's retreat path after placing the last domino: it swept
past the third domino and knocked it over early.
