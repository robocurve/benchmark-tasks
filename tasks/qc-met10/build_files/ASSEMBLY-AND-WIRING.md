# Assembly and wiring

## Mounting (mm, axes as in `REAL-TRIAL-PARTS.md`)
```
 fixed end (+y)                                              moving end (-y)
 [extrusion frame]--[block A + grip A]   <- 115..150 mm jaw faces ->   [grip B + block B]--[stage carriage plate]--[200 mm ball-screw stage]--[NEMA17]
                         shank 10x10x25 into the 30x30x30 block pocket (10.2 sq, 20 deep), 2 x M4 set screws
```
- `stl/grip_block.stl` x 2: clamp block, 30 x 30 x 30 mm, pocket 10.2 x 10.2 x 20 mm; drill two M4 set-screw holes through one side wall (see `assembly.json` drill list).
- Block B bolts to a printed carriage plate on the stage's four M4 holes; the hole spacing is published only as an image on the vendor page, so draw the plate from that hole pattern. Block A bolts to the 2040 extrusion at the fixed end with M4 T-nuts. Both grips' slots face the same side (+x lever side toward the robot) and their loading axes are one line: set the height with shims under block A.
- Stage along y, 200 mm travel; the task uses 35 mm of it (115 mm to 150 mm between the jaw faces).

## Wiring (ZK-SMC02-class controller, terminals as named in its manual: COM, STOP, CCW, CW, OUT)
| from | to |
|---|---|
| 24 V power supply (+, -) | controller DC input (10-30 V) |
| controller motor outputs (A+, A-, B+, B-) | stage NEMA17 coils (match pairs with a multimeter) |
| LEFT push button (22 mm, normally open contact) | between COM and CW |
| RIGHT push button (22 mm, normally open contact) | between COM and CCW |
| nothing | STOP, OUT (not used) |
- Controller setting: the work mode in which the motor rotates only while a button is held and stops when it is released. The manual describes it in these words: "Motor rotates when keep press buttons and stop if release button" (ZK-SMC02 manual, copy read 2026-11-17 at https://manuals.plus/cnc/zk-smc02-cnc-stepper-motor-driver-manual.pdf; also https://m.media-amazon.com/images/I/61rXd-xe47L.pdf; mode numbers differ between versions, so none is written here).
- Speed: potentiometer set so the stage moves 500 mm/min (8.3 mm/s = 100 RPM on the 5 mm lead); the controller's range is 0.1-999 RPM.
- Which direction a button gives depends on the motor wiring: swap one coil pair if LEFT moves the carriage the wrong way (LEFT = toward the fixed end, +y).
