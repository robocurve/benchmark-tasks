# sliding-tile-sort-three-tiles

## Instruction
"Slide the three tiles in the frame into the order shown on the card using the empty cell, without lifting a tile out of the frame."

## Layout Table
| Part | Geometry / Colliders | Mass / Friction | Initial Pose (x, y, z) | Destination / Role |
|---|---|---|---|---|
| Frame | Fixed 2x2 rimmed frame (outer 164x164x4 mm, inner 152x152 mm, rim 4 mm tall) | Fixed (10.0 kg, mu=1.0) | (0.280, 0.000, 0.7495) | Encloses 4 cells of pitch 76 mm |
| Goal Card | Fixed flat plate (80 x 80 x 2 mm) | Fixed (1.0 kg) | (0.280, -0.160, 0.7485) | Visual goal pattern (B:0, O:1, Y:2, E:3) |
| Tile Blue | Free box 64 x 64 x 15 mm | 0.04 kg, mu=0.5 | Cell 3 (0.242, -0.038, 0.7550) | Goal: Cell 0 (0.318, +0.038) |
| Tile Orange | Free box 64 x 64 x 15 mm | 0.04 kg, mu=0.5 | Cell 2 (0.242, +0.038, 0.7550) | Goal: Cell 1 (0.318, -0.038) |
| Tile Yellow | Free box 64 x 64 x 15 mm | 0.04 kg, mu=0.5 | Cell 0 (0.318, +0.038, 0.7550) | Goal: Cell 2 (0.242, +0.038) |

## Scoring Ladder
| Milestone | Score | Condition |
|---|---|---|
| 1 Tile in Goal | 0.25 | Exactly 1 tile in its designated goal cell |
| 2 Tiles in Goal | 0.50 | Exactly 2 tiles in their designated goal cells |
| 3 Tiles in Goal | 0.75 | All 3 tiles in their designated goal cells |
| Full Task Success | 1.00 | All 3 tiles in goal cells, empty cell at 3, tilt < 5 deg, resting, arms clear, held 2.0 s |

## Zero-Gates
- `escaped`: Latched if any tile is lifted > 12 mm above table (z > 0.7745 m) or moves outside the frame footprint (|x - 0.28| > 0.09 or |y| > 0.09). Score is immediately capped at 0.00.

## Timing and Margins
| Action | Parameter / Threshold | Nominal Value | Timing / Hold |
|---|---|---|---|
| Hover transit | `DEMO_HOVER_Z` | 0.850 m | - |
| Push height | `DEMO_PUSH_Z` | 0.7565 m (mid-tile, +5 mm over 4 mm rim) | - |
| Push end points | `DEMO_PUSH_XEND`, `DEMO_PUSH_Y2END`, `DEMO_PUSH_X3END`, `DEMO_PUSH_Y4END` | tip stops at x 0.305 (moves 1, 5), y -0.040 (move 2), x 0.240 (move 3), y 0.072 (move 4); the tile follows the tip, so a longer push rides it onto the frame rim | 1.0 s hold at end of each push |
| Cell pitch | `CELL_SIZE` | 76 mm | Push stroke 96 mm |
| Arm clearance retract | Home pose | (0.200, +/-0.280, 0.850) | >= 2.6 s final hold (req 2.0 s) |
| Demo stretching | `DEMO_TIME_SCALE` | 1.8 | - |

## Measured in the simulator
| Mode | Run tag | Predicted | Measured |
|---|---|---|---|
| `demo` | st_7 | 1.00 | 1.00 |
| `wrong_order` | st_wrong | 0.50 | 0.50 |
| `lift_tile` | st_lift | 0.00 | 0.00 |
| `settle` | st_settle | 0.00 | 0.00 |
| `dream` | st_dream | 0.00 | 0.00 |

All five measured scores match the predictions. Earlier demo runs scored 0.0 because the push end points
drove a tile onto the frame rim (tile centre above 0.7745 m latches the "escaped" gate); the end points
above were calibrated from the per-step tile positions in the trace (`dbg_blue`, `dbg_orange`, `dbg_yellow`).

## Fidelity Notes
Rigid body PhysX simulation with Coulomb friction (0.5), accurate mass (0.04 kg per tile), low rim containment geometry (4 mm outer rim), and compliant arm dynamics. Pushing and tile-to-tile sliding contacts are resolved through physical contact without artificial magnetic snapping.

