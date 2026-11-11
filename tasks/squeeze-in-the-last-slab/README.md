# squeeze-in-the-last-slab

## Instruction
"Stand the last slab into the one gap between the standing slabs without knocking either neighbour."

## Layout Table
| Part | Geometry / Colliders | Mass / Friction | Initial Pose (x, y, z) | Destination / Role |
|---|---|---|---|---|
| White Strip | Flat visual strip (260 x 80 x 2 mm) | Fixed | (0.280, 0.040, 0.7485) | Placement line on table |
| Slab 1 | Free box 12 x 60 x 80 mm (grey) | 0.03 kg, mu=1.0 | (0.190, 0.040, 0.7875) | Standing neighbour |
| Slab 2 | Free box 12 x 60 x 80 mm (grey) | 0.03 kg, mu=1.0 | (0.235, 0.040, 0.7875) | Standing neighbour |
| Slab 3 | Free box 12 x 60 x 80 mm (grey) | 0.03 kg, mu=1.0 | (0.300, 0.040, 0.7875) | Standing neighbour, left of the slot |
| Slab 4 | Free box 12 x 60 x 80 mm (grey) | 0.03 kg, mu=1.0 | (0.350, 0.040, 0.7875) | Standing neighbour, right of the slot |
| Slab 5 (last) | Free box 12 x 60 x 80 mm (blue) | 0.03 kg, mu=1.0 | (0.190, +0.220, 0.7875), standing on its edge | Carried and set into the slot at (0.325, 0.040) |

The slot between slabs 3 and 4 is 50 mm centre to centre, so 38 mm clear for the 12 mm slab (13 mm each side).
The fifth slab starts standing: the arms grip straight down (yaw only) and cannot turn a flat slab on end (a first run
that grasped it flat scored 0.0).

## Scoring Ladder
| Milestone | Score | Condition |
|---|---|---|
| Standing on Strip | 0.25 | Slab 5 standing anywhere on strip (z within 10 mm, tilt <= 15 deg) |
| In Gap Leaning | 0.50 | Slab 5 in gap (|x-0.325| <= 8 mm) but leaning (6 deg <= tilt <= 15 deg) |
| In Gap Low Tilt | 0.75 | Slab 5 standing in gap with tilt < 6 deg |
| Full Task Success | 1.00 | Slab 5 standing in gap, tilt < 4 deg, neighbours undisturbed (< 4 mm, tilt < 5 deg), held 2 s |

## Zero-Gates
- `knocked`: Any neighbour tilted > 10 deg or displaced > 8 mm latches `knocked`, permanently capping score at 0.50.

## Timing and Margins
| Action | Parameter / Threshold | Nominal Value | Timing / Hold |
|---|---|---|---|
| Hover transit | `DEMO_HOVER_Z` | 0.885 m (carried slab bottom clears the 0.8275 m neighbour tops) | - |
| Grip across 60 mm width | `DEMO_GRASP_FINGER` | -0.021 (jaw gap ~42 mm, 18 mm squeeze; -0.030 is zero squeeze and lifts nothing) | `DEMO_GRASP_S` 0.8 s |
| Grip height | `DEMO_GRIP_DZ` | 0.0 (TCP at the slab centre, 0.7875 m) | - |
| Gap insertion descent | `DEMO_INSERT_DZ`, `DEMO_INSERT_S` | 0.7905 m (3 mm above the strip), 1.2 s | 0.5 s hold before release |
| Arm clearance retract | Home pose | (0.200, +/-0.280, 0.885) | >= 2.6 s final hold (req 2.0 s) |
| Demo stretching | `DEMO_TIME_SCALE` | 1.8 | - |
| Ablation offset | `DEMO_KNOCK_DX` | 0.015 m | - |

## Simulator Verification (Measured vs Predicted)
- `demo`: Predicted 1.00 | Measured in the simulator: 1.00 (slab in the gap, tilt ~0 deg, neighbours moved < 0.01 mm, held 2 s)
- `knock_neighbour`: Predicted 0.00 | Measured in the simulator: 0.00 (neighbour toppled to 90 deg and moved 47 mm; the slab does not end in the gap or on the strip; same result at 15 mm and 18 mm off-centre)
- `stop_over_gap`: Predicted 0.00 | Measured in the simulator: 0.00
- `settle`: Predicted 0.00 | Measured in the simulator: 0.00
- `dream`: Predicted 0.00 | Measured in the simulator: 0.00

## Fidelity Notes
Rigid body PhysX simulation with Coulomb friction (1.0), true mass distribution (0.03 kg per slab), exact thin-slab contact dynamics, compliant gripper contacts, and realistic arm compliance. Standing stability and tipping cascades are resolved purely through multi-body contact forces without kinematic pinning.

