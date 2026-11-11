# clear-lane-then-push-block-to-dock

## Instruction
"Move the two loose blocks out of the lane, then push the cargo block along the lane to the dock without touching the red post."

## Layout Table
| Part Name | Body Type | Dimensions (x, y, z mm) | Initial Position (x, y, z m) | Mass (kg) | Friction | Color (RGB) |
|---|---|---|---|---|---|---|
| `rail_left` | Fixed box | 240 x 10 x 20 | (0.260, +0.055, 0.7575) | 10.0 | 1.0 | (0.55, 0.55, 0.60) |
| `rail_right` | Fixed box | 240 x 10 x 20 | (0.260, -0.055, 0.7575) | 10.0 | 1.0 | (0.55, 0.55, 0.60) |
| `stop_plate` | Fixed box | 10 x 98 x 40 | (0.380, 0.000, 0.7675) | 10.0 | 1.0 | (0.45, 0.45, 0.50) |
| `dock_mark` | Visual box | 80 x 80 x 2 | (0.340, 0.000, 0.7485) | 1.0 | 1.0 | (0.95, 0.95, 0.95) |
| `red_post` | Free box | 20 x 20 x 80 | (0.340, +0.090, 0.7875) | 0.02 | 0.5 | (0.85, 0.15, 0.15) |
| `blue_block` | Free box | 40 x 40 x 40 | (0.220, 0.000, 0.7675) | 0.04 | 0.8 | (0.15, 0.35, 0.85) |
| `yellow_block`| Free box | 40 x 40 x 40 | (0.280, 0.000, 0.7675) | 0.04 | 0.8 | (0.90, 0.80, 0.15) |
| `cargo_block` | Free box | 60 x 60 x 40 | (0.150, 0.000, 0.7675) | 0.06 | 0.4 | (0.95, 0.45, 0.05) |

## Scoring Ladder
| Score | Requirement |
|---|---|
| 0.25 | Both loose blocks cleared out of the lane (`\|y\| > 0.075` m) |
| 0.50 | Cargo block moved 80 mm along x (`x >= 0.230` m) without loose block in the lane |
| 0.75 | Cargo block at dock (within 25 mm of mark center `x=0.340` m) |
| 1.00 | Cargo block on mark within 12 mm, yaw within 10 deg, red post undisturbed (`tilt <= 10` deg, `shift <= 10` mm), held 2 s, arms clear |

## Zero-Gate List
- `post_hit`: Red post tilted > 10 degrees or moved > 10 mm from starting position latches `post_hit` and caps score at 0.25.
- `physical blockage`: Pushing cargo without clearing loose blocks causes cargo to physically collide with the 40 mm blocks inside the narrow lane, preventing progress.

## Timing-Margin Table
| Parameter | Env Knob | Value | Description |
|---|---|---|---|
| Demo Time Scale | `DEMO_TIME_SCALE` | 1.8 | Motion duration multiplier |
| Hover Height | `DEMO_HOVER_Z` | 0.860 m | Safe transit elevation above rails (20 mm) and red post (80 mm) |
| Push Height | `PUSH_Z` | 0.7675 m | Mid-height contact elevation for cargo block |
| Push Overshoot | `DEMO_PUSH_OVERSHOOT` | 0.025 m | 25 mm overshoot past mark center (to x=0.365 m) |
| Push Hold | `DEMO_PUSH_HOLD_S` | 1.2 s | Hold duration at push completion before vertical retreat |
| Final Settle Hold | `DEMO_FINAL_HOLD_S` | 2.6 s | Stationary hold at home pose ensuring success window |
| Success Hold | `HOLD_TIME_S` | 2.0 s | 40 consecutive steps required for score 1.00 |

## Ablation Predictions
| Mode | Predicted Score | Description |
|---|---|---|
| `demo` | 1.00 | Full bimanual sequence: clear both blocks, push cargo to dock, clear arms, hold 2 s |
| `push_first` | 0.00 | Attempt push without clearing; cargo jams against Blue block at lane entrance |
| `hit_post` | 0.25 | Blocks cleared and cargo pushed, but pusher veers into red post (cap 0.25) |
| `stop_halfway`| 0.50 | Blocks cleared, cargo pushed 80 mm past entrance, then push aborts |
| `settle` | 0.00 | Idle at home |
| `dream` | 0.00 | Baseline unactuated check |

## Measured in the Simulator

| Mode | Predicted | Measured | Run |
|---|---|---|---|
| `demo` | 1.00 | **1.00** (cargo at x = 0.345, held 2 s, post untouched) | `cl_3` |
| `push_first` | 0.00 | **0.00** | `cl_4` |
| `hit_post` | 0.25 | **0.25** (post knocked, cap applied) | `cl_9` |
| `stop_halfway` | 0.50 | **0.50** | `cl_6` |
| `settle` | 0.00 | **0.00** | `cl_7` |
| `dream` | 0.00 | **0.00** | `cl_8` |

Two fixes came out of the simulator runs. The first demo (`cl_1`, `cl_2`) scored 0.75: the cargo stopped at x = 0.360, 20 mm past the
mark centre, because the stop plate sat at x = 0.395; it is now at 0.380 (front face 0.375, 98 mm wide), so the cargo comes to rest at
0.345 and the 12 mm tolerance can be met. The first `hit_post` (`cl_5`) scored 0.75: the pad rode over the rail and never touched the
post. The ablation now lifts to the post's mid-height, crosses the rail and sweeps through it.

## Honest Fidelity Notes
- Rigid-body simulation accurately models 100 mm channel containment and block-rail contact friction.
- The fragile red post is simulated as an unanchored free body; any lateral contact causes rapid tipping past the 10-degree threshold.
- Lane rails and stop plate are rigidly anchored collision bodies.

## Board Check
- Mechanism collisions checked against `work/BOARD_PR_TITLES_2026-11-09.txt`.
- No conflicting mechanism: PR #1169 ("move the boxes together first") packs boxes to make room, PR #424 ("turn-parcel-into-bay") performs yaw turning into bay. This task evaluates sequential obstacle clearing from a transit lane followed by bounded straight pushing with a fragile adjacent obstacle.
