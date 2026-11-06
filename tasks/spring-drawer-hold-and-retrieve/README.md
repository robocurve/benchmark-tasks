# spring-drawer-hold-and-retrieve

Hold a spring-return drawer open while retrieving an enclosed block.

## Category
Impact on jobs

## Physical Layout
- **Cabinet**: 220 x 160 x 80 mm open-front shell (floor, back wall, two side walls) at `(0.415, 0.000, 0.7875)` with front face at `x = 0.305 m`, and a cover plate over its rear half (`x >= 0.380 m`).
- **Spring-Return Drawer**: 190 x 120 x 30 mm open-top box with 10 mm walls. Slides 140 mm along -X against an 18 N/m spring (peak force <= 2.5 N). Front handle: 25 x 60 x 35 mm.
- **White Block**: 40 mm cube, mass 0.10 kg, starts on the drawer floor under the cover plate at `x = 0.450 m`. Only accessible when drawer is out >= 90 mm.
- **Green Shelf**: 130 x 130 x 10 mm on the right at `(0.300, -0.280)`.
- **Indicator Lamp**: 25 mm cyan marker on the cover plate at `(0.450, 0.000)`, lights when drawer displacement >= 90 mm.

## Rules and Zero-Gates
- **Scoring Ladder**:
  - `0.25`: Drawer opened >= 90 mm (cyan lamp lit) for >= 1.0 s.
  - `0.50`: White block grasped and lifted >= 30 mm clear of drawer floor.
  - `0.75`: White block resting on green shelf.
  - `1.00`: Drawer closed (< 5 mm) afterwards with both arms clear of cabinet, held for 1.0 s / 20 steps.
- **Zero-Gates**:
  - PINCH: drawer closing (< 30 mm out) while the block is still in the cabinet mouth zone (block centre within 60 mm behind the cabinet front face). Score = 0.0.
  - Dropped block off table: Score = 0.0.

## Ablations and Predicted Scores
- `demo`: 1.00
- `no_hold`: 0.25
- `pinch`: 0.00
- `leave_open`: 0.75
- `settle`: 0.00


## Measured in the simulator
| Mode | Predicted | Measured (GPU) |
|---|---|---|
| demo | 1.00 | 1.00 |
| no_hold | 0.25 | 0.25 |
| pinch | 0.00 | 0.00 (PINCH fault) |
| leave_open | 0.75 | 0.75 |
| settle | 0.00 | 0.00 |
| Dream-1 | 0.00 | 0.00 |

## Changes made after the first simulator runs
1. The cabinet and drawer were solid boxes containing the block, so the block started inside solid geometry. They are now an open-front shell and an open-top drawer (floor + four walls), the block rests on the drawer floor, and the cover plate (front edge at `x = 0.380 m`) hides it until the drawer is out about 90 mm. The cyan lamp moved onto the cover plate.
2. The `-X` drawer slider reports a negative displacement when pulled out; the scorer now uses its absolute value.
3. The hold of the handle is a real grasp (jaw gap 50 mm on the 60 mm handle, approached from the front) instead of a closed fist; the block is approached with a 60 mm jaw gap so the fingers clear the drawer walls.
4. The PINCH rule is now "drawer < 30 mm out while the block is still within 60 mm behind the cabinet front face" (the old 20 mm-of-the-lip rule could never fire when the block sits inside the drawer). The `pinch` ablation now waits for the right hand to hold the block before the left lets go.
5. `leave_open` keeps the left hand on the handle until the end (otherwise the spring closes the drawer and the run scores 1.00).
