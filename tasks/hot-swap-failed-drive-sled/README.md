# hot-swap-failed-drive-sled

Hot-swap a failed drive sled in a rack without disturbing healthy bays.

## Category
Impact on AI timelines

## Physical Layout
- **Server Rack**: 160 x 400 x 60 mm chassis (open-front shell: floor, back wall, two dividers, two outer walls and a top plate that starts 35 mm behind the front face) at `(0.410, 0.000, 0.7775)` with front face at `x = 0.330 m`. Three drive bays along `y` at `-0.120`, `0.000`, and `+0.120 m` (pitch 120 mm), each a 104 mm wide slot (12 mm clearance per side).
- **Three Sleds**: 150 x 80 x 25 mm sled bodies (50 g) with 30 x 45 x 30 mm front handles. Sled 1 (middle bay at `y = 0.0`) has failed (red status lamp).
- **Latch Buttons**: 40 x 40 x 12 mm button caps on the rack top plate at `x = 0.405 m`, 55 mm to the left of each bay's centre line (8 mm travel to hard stop, 100 N/m spring).
- **Spare Stand**: 180 x 180 x 20 mm plate at `(0.280, +0.300)` holding:
  - Blue-handled spare (keyed correctly for bay)
  - Orange-handled spare (wrong keying; stays red)
- **Discard Pad**: 130 x 130 x 10 mm pad at `(0.280, -0.300)`.
- **Arms**: the left arm holds the latch and fetches and inserts the spare; the right arm pulls the old sled and parks it on the pad. Each arm can only reach its own side (the right arm cannot reach the spare stand, the left arm cannot reach the discard pad).

## Rules and Zero-Gates
- **Detent Zone**: Between 0 and 25 mm of pull travel, the latch button must remain depressed (`<= -6.4 mm`). Moving > 3 mm without the latch triggers a forced pull FAULT, latching score to 0.0.
- **Healthy Bay Disturbance**: Moving any healthy sled by > 5 mm triggers a FAULT, latching score to 0.0.
- **Scoring Ladder**:
  - `0.25`: Failed sled pulled out >= 60 mm with latch held (no fault).
  - `0.50`: Failed sled completely clear and resting on discard pad.
  - `0.75`: Blue spare seated >= 95% in failed bay.
  - `1.00`: Lamp green (latch released, both arms clear, healthy sleds undisturbed, held 1.0 s / 20 steps).

## Ablations and Predicted Scores
- `demo`: 1.00
- `no_latch`: 0.00 (FAULT: forced pull without latch)
- `wrong_bay`: 0.00 (FAULT: healthy bay sled disturbed)
- `wrong_spare`: 0.50 (orange spare doesn't key, lamp stays red)
- `release_early`: 0.00 (FAULT: latch released before the pull -> forced pull without latch)
- `settle`: 0.00

## Measured in the simulator
| Mode | Predicted | Measured (GPU) |
|---|---|---|
| demo | 1.00 | 1.00 |
| no_latch | 0.00 | 0.00 (forced pull FAULT) |
| wrong_bay | 0.00 | 0.00 (healthy bay moved FAULT) |
| wrong_spare | 0.50 | 0.50 |
| release_early | 0.00 | 0.00 (forced pull FAULT) |
| settle | 0.00 | 0.00 |
| Dream-1 | 0.00 | 0.00 |

## Changes made after the first simulator runs
1. The rack was a solid block that contained the three sleds, so they were thrown out at start-up. It is now an open-front shell with a slot per bay; the slots are 104 mm wide (12 mm clearance per side) so the held spare can be lined up by hand, and the top plate starts 35 mm behind the front face so the gripper palm can reach the handles.
2. The sleds are 25 mm thick (was 35 mm), light (50 g) and have friction 1.5, and the handles are 30 mm deep, so the fingers can lift and carry them without slipping.
3. The two arms crowded each other when both worked at the middle of the rack. The roles are now mirrored: the left arm holds the latch (the caps moved 55 mm to the left of each bay and back to `x = 0.405`) and fetches the spare from a stand on the left; the right arm pulls the old sled and parks it on a pad on the right. The wrong_bay ablation now pulls the right-hand healthy bay (bay 2).
4. The plan runs one arm at a time near the middle (left presses the latch, then the right takes the handle, then it pulls), grips the handle 12 mm short of its centre (the palm stops against the rack front), pulls the sled fully out (160 mm) before lifting it, and carries the spare in at its mouth height.
5. The "lamp turns green" now shows a separate green marker; before it reused the red one.
6. Plans are stretched by `DEMO_TIME_SCALE` (1.8) like the other tasks.
