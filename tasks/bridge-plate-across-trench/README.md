# `bridge-plate-across-trench`

## Task Overview
A bimanual gap-bridging and payload transit task in the "Safety and alignment" category.

### Instruction
"Lay the bridge plate across the trench, then push the block over it onto the far platform without letting the plate or the block fall into the trench."

### Station Setup
- **Platforms**: Two grey platforms 120 (y) x 260 (x) x 30 mm tall, separated by a 90 mm trench along x at y = 0.
  - Near platform lip: `y in [0.045, 0.165]`
  - Far platform lip: `y in [-0.165, -0.045]`
  - Trench: `y in [-0.045, 0.045]`, depth 30 mm to table floor.
- **Bridge Plate**: Free body 80 (x) x 130 (y) x 6 mm, mass 0.05 kg, friction 1.5, orange, at `(0.300, +0.200, TOP + 0.033)`.
- **Block**: Free body 40 x 40 x 30 mm, mass 0.05 kg, friction 0.5, blue, at `(0.250, +0.140, TOP + 0.045)`.
- **Target Mark**: 70 x 70 x 2 mm flat white visual mark on the far platform at `(0.300, -0.140, TOP + 0.031)`.

### Scoring Ladder
- **0.25**: Bridge plate seated across trench (both y-edges overlap platform lips by >= 20 mm, tilt < 3 deg, resting).
- **0.50**: Block on the bridge (centre within bridge footprint, crossing zone).
- **0.75**: Block on the far platform past the trench lip (`y <= -0.045`).
- **1.00**: Block centred on target mark within 15 mm and still, bridge still seated, arms clear (> 60 mm), held stable for 2.0 s (40 steps at 20 Hz).
- **Zero-Gate**: Bridge plate or block falling into trench (`z < TOP + 0.030 - 0.020 m`) permanently latches `fell = True` (score 0.0).

### Coordination and Demonstration Plan
1. **Left Arm (Bridge Placement)**:
   - Grasps the 80 mm narrow side of the bridge plate.
   - Carries and places it across the trench at `y = 0.0` (span `[-0.065, +0.065]`, providing exactly 20 mm overlap on each lip).
   - Releases and returns home.
2. **Right Arm (Block Push)**:
   - Approaches behind the block with closed fingers.
   - Pushes the block along -Y across the near lip, over the bridge, onto the far platform.
   - Pushes onto the target mark with 25 mm overshoot and hold (Lesson 4), then lifts and returns home.
3. Both arms hold at home for 2.0 s clear of the target mark.

### Ablations: predicted and measured in the simulator

| Mode | Predicted | Measured | Notes |
|---|---|---|---|
| `demo` | 1.00 | 1.00 | bridge seated, block pushed over it and centred on the mark (6 mm off), held 3.95 s |
| `no_bridge_push` | 0.00 | 0.00 | block pushed at the trench with no bridge: falls |
| `bridge_short_overlap` | 0.25 or 0.00 | 0.00 | bridge placed 8 mm onto the far lip: far overlap -6 mm, tilt 20.7 deg, never seated; the block does not cross |
| `skip_target` | 0.50 | 0.50 | block left on the bridge |
| `settle` | 0.00 | 0.00 | arms hold at home |
| `dream` | 0.00 | 0.00 | Dream-1 baseline |

The demo took four GPU runs. Two causes were fixed in `demo_plan.py`: the pushing fist sat too low and rode the block up
8 mm (`DEMO_PUSH_DZ` 0.028, pad at block mid-height), and the return path at z 0.81 swept the block back off the bridge
(`DEMO_TRANSIT_Z` 0.880). A slow creep for the last stretch of the push and a back-off before the lift were also added
(`DEMO_FAR_CREEP_Y`, `DEMO_FAR_CREEP_S`, `DEMO_FAR_BACKOFF`).
