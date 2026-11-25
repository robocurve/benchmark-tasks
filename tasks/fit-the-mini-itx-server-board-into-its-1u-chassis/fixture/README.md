# Bench fixture for the real trial

Everything that is scored is a stock product (the board, the chassis and its I/O shield). Only two simple items
are made for the trial: the foam pad the board starts on, and two bench stops that put the chassis in the same
place every trial. Dimensions in millimetres; x away from the robot, y to the robot's left, origin midway between
the two arm bases at bench height.

## Parts

| Item | What | Source |
|---|---|---|
| Server board | Supermicro A2SDi-8C-HLN4F, Mini-ITX 170 x 170 mm, unpowered (a retired unit is fine) | Supermicro (MBD-A2SDi-8C-HLN4F) or resellers |
| Chassis | Supermicro CSE-505-203B, 1U mini, 437 x 249 x 43 mm, lid removed, power supply and drive bracket fitted | Supermicro or resellers |
| I/O shield | Supermicro MCP-260-00085-0B, listed on the board's product page as its optional 1U shield for SC505/SC504 chassis (with EMI gasket), fitted in the rear window | Supermicro |
| Standoffs | the chassis's M3 motherboard standoffs at the Mini-ITX C, F, H, J positions (fitted height 6.35 mm, estimate) | supplied with the chassis |
| Foam pad | conductive (black) polyethylene foam, cut to 150 x 150 x 10 mm, corner marks drawn on it | any ESD foam sheet |
| Bench stops | two hardwood strips, 190 x 20 x 20 mm (rear) and 100 x 20 x 20 mm (left), screwed to the bench | made |
| Protection | ESD wrist strap and nitrile gloves for whoever resets the board; ESD-safe foam tape on both gripper pads | any |
| Measuring | feeler gauges 0.3 mm and 1.0 mm, steel rule, straightedge, protractor, phone inclinometer, phone camera, four M3 x 5 mm screws for the hand-start check | any |

## Layout (top view, lot A)

```
                         rear (x = 448)            rear stop (x 448-468, y -290..-100)
   y=+146 ┌──────────────────────────────────────┬─────────┐ y=-291
          │ I/O shield (x 447-448, y -25..+125)  │   PSU   │
          │   ┌─────────────────┐                └─────────┤
 left     │   │ board's place   │                          │
 stop ──► │   │ 170 x 170       │                          │
          │   │ centre (360,50) │                          │
          │   └─────────────────┘               ┌──────┐   │
          │      [fan holder]                   │drive │   │
          └─────────────────────────────────────┴──────┴───┘
                         front (x = 199), toward the robot

   foam pad: 150 x 150, centre (272, 330), left of the chassis
```

- The rear stop runs along y at x 448 to 468, from y -290 to -100: the chassis's rear face rests against it behind
  the power supply, clear of the connectors that stand through the I/O shield. Pushing the board into the shield
  presses the chassis against this stop.
- The left stop runs along x at y +146 to +166, from x 300 to 400: the chassis's left face rests against it.
- Both are hardwood strips 20 x 20 mm, screwed to the bench.
- The foam pad's edges are at x 197 to 347 and y 255 to 405. Mark the board's two start outlines on it in pencil
  (lot A: centre 272, 330, connector edge toward +x; lot B: centre 284, 338, connector edge toward -y).

## Checks before each trial

1. The chassis touches both stops (rear and left).
2. The I/O shield sits fully in the rear window, every cut-out clear.
3. All four standoffs are tight and nothing lies on the chassis floor.
4. The board lies on its outline for the lot, components up.
