# Weigh and Reject Heavy Block (`weigh-and-reject-heavy-block`)

An automated quality control inspection task. Three grey cubic blocks (40 x 40 x 40 mm) look identical to the camera, but one is dense (0.30 kg vs 0.10 kg). A physical measurement on a spring scale (600 N/m) compresses the platform, driving a needle indicator into a red reject band (> 32 mm) for the heavy block. Because neither arm can reach both sides of the station, blocks are physically relayed across the workspace through the scale platform.

## Instruction
"Weigh each grey block on the scale, then put only the heavy one into the red reject tray, and leave the other two on the green shelf."

## Objects and Physical Layout
- **Three Grey Blocks**: 40 x 40 x 40 mm, masses 0.10 kg, 0.30 kg (heavy), 0.10 kg. Middle position (index 1) is the heavy block.
- **Supply Rack**: 300 x 80 x 10 mm dark plate on left at `(0.250, +0.300)` holding blocks at `(0.150, +0.300)`, `(0.250, +0.300)`, `(0.350, +0.300)`.
- **Spring Scale**: 140 x 140 x 20 mm base at `(0.300, 0.000)` with 100 x 100 x 8 mm platform on a Z-axis prismatic joint (travel 12 mm, spring stiffness 600 N/m, damping 12.0 N s/m).
- **Readout Meter**: 50 x 160 x 15 mm housing at `(0.450, 0.000)` with a needle marker moving along Y with x10 gain (heavy block ~49 mm, light block ~16 mm, red threshold 32 mm).
- **Green Shelf**: 260 x 90 x 10 mm on left at `(0.300, +0.120)`.
- **Red Reject Tray**: 130 x 130 x 20 mm (15 mm walls) on right at `(0.300, -0.300)`.

## Physical Bimanual Relay Constraint
- Left arm base to Reject Tray = **0.675 m** (> 0.50 m reachable limit; Left arm CANNOT reach the reject tray).
- Right arm base to Supply Rack = **0.655 m** (> 0.50 m reachable limit; Right arm CANNOT reach the supply rack).
- Both arms reach the central scale platform at `(0.300, 0.000)`, enforcing a physical bimanual relay.

## Timing Margin Table
| Requirement | Specification | Scripted Arm Achieved | Safety Margin |
|---|---|---|---|
| Scale Settling & Weigh Hold | >= 1.0 s steady | 1.5 s hands-clear | +0.5 s (+50%) |
| Final Success Settle | >= 1.0 s (20 steps) | 2.0 s (40 steps) | +1.0 s (+100%) |

## Kinematics and Clearances
- **Minimum Arm-Arm Clearance**: Verified >= **30.0 mm**.
- **Scene Clearances**: All unintended clearances >= **1.0 mm**.
- **Reach Margins**: Minimum joint margin **0.335 rad** (threshold >= 0.02 rad), worst IK error **0.10 mm** / **0.01 deg**.

## Scoring Ladder
- **0.25**: All three blocks weighed on the scale (each >= 1.0 s steady deflection with arm clear).
- **0.50**: Heavy block placed in reject tray AND it was weighed.
- **0.75**: Additionally both light blocks rest on the green shelf.
- **1.00**: Additionally scale is empty, nothing on the floor, grippers open, arms still for 1.0 s (20 steps at 20 Hz).

## Zero-Gates
- **Wrong Rejection**: Placing a light block in the reject tray after being weighed caps score at 0.25.
- **Shortcut Penalty**: Placing the heavy block in the reject tray without weighing it gives no credit.
- **Dropped Block**: Any block falling off the table drops score to 0.0.

## Modes and Predicted Scores
- `demo`: 1.00 (full bimanual weighing and sorting sequence)
- `wrong_pick`: 0.25 (all three weighed, but a light block is rejected)
- `no_weigh`: 0.00 (heavy block moved without weighing)
- `weigh_too_short`: 0.00 (blocks lifted before 1.0 s settling)
- `settle`: 0.00 (idle hold at home)
- `dream`: 0.00 (baseline)

## Measured in the simulator (GPU, 20 Hz)

| Mode | Predicted | Measured | What happens |
|---|---|---|---|
| `demo` | 1.00 | 1.00 | all three blocks weighed, heavy one rejected, light ones on the shelf, success held 1.0 s |
| `wrong_pick` | 0.25 | 0.25 | all three weighed, a light block goes into the reject tray |
| `no_weigh` | 0.00 | 0.00 | heavy block moved without being weighed |
| `weigh_too_short` | 0.00 | 0.00 | blocks lifted before 1.0 s on the scale |
| `settle` | 0.00 | 0.00 | arms hold at home for 8 s, nothing moves |
| `dream` | 0.00 | 0.00 | Dream-1 baseline |

## Changes made after the first simulator runs
The first simulator run scored 0.5, then 1.0 after the fixes below.
1. **Lift higher after releasing on the scale.** The scale hover height was 0.840 m, only 12 mm above the 40 mm block top (0.8275 m), so the finger pads still straddled the block and the sideways move home dragged it off the platform before it had been weighed for 1.0 s. It is now 0.880 m.
2. Scene fix: every fixed part (scale, supply rack, shelf, tray, meter, blocks) was built with its box centre at the world position instead of inside its own frame, which threw the parts far off the table. They are now shifted into the part frame.
3. Task and run scripts: `run.sh` uses the working `inspect-robots` call, `reset` takes the environment argument, and `demo()` follows the mode from the environment, so the ablation and settle modes actually run instead of replaying the demo.

