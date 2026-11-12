# Replace a server's CR2032 system battery in its latched socket

Replace the server's system battery and put the spent cell in the disposal tray.

Every rack server keeps its real-time clock and firmware settings alive on a CR2032 lithium coin cell in a
horizontal socket on the system board. Replacing it is a standard customer/field-replaceable procedure in the
vendors' service guides:

- Lenovo ThinkSystem SR650, "Remove the CMOS battery": "Press the battery clip as shown and carefully lift CMOS
  battery out of the socket" (https://pubs.lenovo.com/sr650/remove_the_cmos_battery).
- Lenovo ThinkSystem SR630 V3, "Install the CMOS battery": positive side up; "Tilt the battery and insert it to the
  positive end on the socket ... Press the battery down until it clicks into the socket"
  (https://pubs.lenovo.com/sr630-v3/install_the_cmos_battery); Lenovo HX1310 guide: "Tilt the battery so that you
  can insert it into the socket on the side opposite the battery clip. Press the battery down into the socket until
  it clicks into place. Make sure that the battery clip holds the battery securely."
- Dell PowerEdge, "How to Replace a CMOS Battery" (KB 000234357): hold the battery with the positive side up, slide
  it under the securing tabs, "press the battery into the connector until it snaps into place"; firmly support the
  connector; tape over the terminal of the removed cell because a short can cause a fire.
- HPE ProLiant SL165z G6 guide: squeeze the metal clamp so the battery pops up, then lift it out.

The skill is a mechanism skill at millimetre scale: push a sprung latch back so the spent cell pops up under its
contact spring, pinch the 3.2 mm-thick cell by its rim and lift it clear of the fixed hook, then get the new cell's
edge under the hook and press it flat until the latch snaps over it.

Nearest tasks: the accepted 2x2 18650 pack-in-a-jig task (#1303) and dexhand's "Remote Battery Swap" place
cylindrical cells in open cradles or behind a door; flashlight tasks (#281, #230) drop a cell into a tube. None has a
cell retained by a sprung latch and a fixed hook that must be released, popped, extracted and re-engaged.

## Run and score

From this task folder, with the official YAM Arena environment installed:

```sh
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=replace_server_system_coin_battery.py -E object_jitter=0 -E control_hz=20 \
  --instruction "Replace the server's system battery and put the spent cell in the disposal tray." \
  --scorer task_score --max-steps 1800 --store-frames --log-dir ./demo-run
```

For a model trial replace `--policy yam_scripted` with `--policy dream`. The output of `--scorer task_score` is the
score. One trial allows 1,800 control steps (90 s at 20 Hz); the scripted demo plan lasts 41.9 s and reaches full score with `success` termination at about 40 s (792 steps) and uses only the
authored layout and elapsed time (it never reads object state). Wall-clock time per trial on one GPU: about 6 minutes (simulation about 0.25 s per step plus start-up and video).
Seeds: the start layout is fixed (`object_jitter=0`); the cells are free bodies and the socket, board, chassis tray,
stand and disposal tray are fixed.

## Scene (metres, world frame: x forward, y left, z up; table top z = 0.7475; arm bases at (0, +-0.305))

| Object | Real item | Model | Pose / size |
|---|---|---|---|
| chassis | Server chassis tray, 1.0 mm galvanised steel (SGCC) | fixed; floor + 12 mm flanges | floor x 0.060-0.270, y -0.090-0.090, 1.0 mm thick on the table |
| standoffs | 4 x M3 brass hex standoffs, 6 mm | fixed cylinders r 2.75 mm | under the board corners, 6 mm inset |
| board | Section of a server system board, FR-4, 1.6 mm | fixed box | x 0.085-0.245, y -0.075-0.075; top z 0.7561 |
| PCIe slot | PCIe x16 connector body, 89 x 7.5 x 11.25 mm | fixed box on the board | centre x 0.120, y 0 (long axis along y) |
| holder | Horizontal CR2032 board socket with spring latch (motherboard type, e.g. LOTES AAA-BAT-0xx family, ~24 x 22 mm, ~8.6 mm high) | fixed-base mechanism, 2 sprung slides | centre (0.150, 0.000) on the board top |
| old_cell | Spent CR2032 (IEC CR2032, 20.0 x 3.2 mm, 3.0 g, "+" can face up) | free, 2 cylinders | latched in the socket, centre z 0.7590 |
| new_cell | New CR2032, "+" up | free, 2 cylinders | on the staging boss, centre (0.150, 0.125, 0.7601) |
| stand | Staging boss: 40 x 40 x 3 mm ESD plate with a 12 mm dia x 8 mm boss | fixed | plate centre (0.150, 0.125) |
| bin | ESD disposal tray, 60 x 50 x 20 mm, 2 mm walls and floor | fixed | centre (0.150, -0.130) |

### CR2032 cell (IEC 60086 CR2032; e.g. Panasonic CR2032 data sheet: 20.0 mm dia, 3.2 mm, 3.1 g)

Can (positive) cylinder r 10.0 mm, 2.9 mm high; negative cap cylinder r 8.0 mm, 0.3 mm high on the "-" face. Mass
3.0 g (density about 3.0 g/cm^3). Friction 0.5/0.4 (nickel-plated steel).

### Socket (holder frame: origin at socket centre on the board top; +x is away from the robot)

| Feature | Material | Geometry (mm) | Mechanics |
|---|---|---|---|
| Seat | black PBT moulding | disc r 9.0, 1.0 high; seat under the hook x -10.4..-8.0, y +-4, 1.0 high | fixed |
| Fixed hook | PBT | post x -11.8..-10.3, y +-3, 0-5.2 high; lip x -10.3..-9.2, y +-3, underside 4.5, top 5.2 (0.8 mm over the cell edge) | fixed |
| Locating posts | PBT | four 1.6 x 1.6 x 3.0 posts centred at (+-8.2, +-8.2) | fixed |
| Housing wall behind latch | PBT | x 13.8..15.0, y +-4.5, 0-8.6 high | fixed |
| Latch | spring steel, nickel plated | leg x 10.2..12.0, y +-3.5, z 1.2-4.5; lip/lead-in: section (9.2,4.5)-(12.0,4.5)-(12.0,6.5)-(10.7,6.5)-(9.2,5.0), 7 wide (45-degree lead-in, lip 0.8 mm over the cell edge); upturned release tab x 11.0..12.0, z 6.5-8.6 | slides along +x, 0-1.5 mm; spring 400 N/m preloaded 1.0 mm: 0.4 N closed, 1.0 N fully open; damping 1 N s/m; friction 0.2/0.15 |
| Negative contact | phosphor bronze leaf | tip 4 x 5 x 0.5 at x +2.0, top 1.3 above the board when a cell is latched | rises 0-3.0 mm; 200 N/m preloaded to 0.8 N on a latched cell, 0.2 N at full lift; damping 0.3 N s/m |

With a cell latched, the contact holds it up against the hook lip and latch lip (cell bottom 1.3 mm, top 4.5 mm).
Pushing the latch back 0.8 mm or more frees the far edge, and the contact tips the cell up about 15 degrees about the
hook (far edge up about 5 mm). Pressing a cell down onto the lead-in pushes the latch back; when the cell is flat the
latch snaps back over its edge.

Calibrating a real socket: push the latch tab outward with a push-pull gauge (0.4-1.0 N over 0-1.5 mm here); press a
cell into the socket with a gauge on its latch side (about 2-3 N to snap). Real latches vary; record the measured
values with the trial.

## Scoring

Scored from the end state (object poses; in a real trial a top photo and a side photo with a scale, and video for
drops). Two independent sub-goals add up:

| Sub-goal | Condition | Credit |
|---|---|---|
| A. Spent cell disposed of | the spent cell's bounding box lies inside the disposal tray's inner walls (2 mm tolerance), on its floor and below its rim; released and still | 0.25 |
| B. New cell latched | centre within 1.5 mm of the socket centre in plan and within 0.8 mm of the latched centre height (2.9 mm above the board); tilt 5 degrees or less with the "+" face up; latch back within 0.5 mm of closed (its lip over the cell edge); released and still, held for 1 s | 0.75 |
| B'. New cell in the socket, not latched | centre within 5 mm of the socket centre, lowest point 2.5 mm or less above the board, "+" up within 45 degrees, released and still | 0.25 (instead of B) |

Score = A + (B or B'): 0, 0.25, 0.5, 0.75 or 1.0. `success()` is score 1.0; the harness ends the trial after it has
held for another second.

- Released: each gripper's centre point (between the pads) at least 30 mm from the cell's bounding box, read from the
  arms' joint state.
- Still: over the last 1 s the cell moved less than 0.5 mm and turned less than 1 degree (pose-based, because the
  contact between a 3 g cell and its socket chatters in reported velocity while the pose does not move).
- Polarity is judged at the end only: a cell inserted "+" down and later corrected scores normally.
- Permanent 0 for the rest of the trial (sticky): a released cell ever lying on the board or the chassis floor outside
  the socket (centre over the chassis tray, more than 13 mm from the socket centre, lowest point within 12 mm of the
  board top) - a loose lithium cell on a system board is a short-circuit and fire hazard (Dell's guide warns to tape
  the terminal); or a cell more than 30 mm below the table top (dropped off the bench).

Cheats checked in the generated tests (expected score): doing nothing (0); new cell lying on top of the hook and latch
lips (0.25: A only); new cell latched "+" down (0.25); new cell flat but the latch held open by a gripper (0.25); new
cell hovering 5 mm above the seat (0.25); new cell latched but still pinched (0.25); spent cell left loose on the board
or in the chassis tray (0); a cell dropped on the board earlier and picked up again (0); a cell off the table (0); cells
swapped (spent cell back in the socket, new one in the tray: 0); both cells in the tray (0.25).

How a person measures it: the tray check and the "+" face from a top photo; latch engagement by looking at the lip
over the cell edge (or a 0.5 mm feeler under the latch tab); flatness and height with a side photo against a mm scale
(a seated cell's top is 4.5 mm above the board, level with the underside of the hook); the 30 mm release and the 1 s
stillness from video.

## Demo (scripted, open loop; uses only the authored layout and time)

1. Left arm parks clear (the two arms collide if both work over the socket).
2. Right arm, fingers closing along x: one pad inside the latch's upturned tab, the other outside the socket's housing
   wall; it squeezes the latch back 1.5 mm, then slides up off the tab; the contact spring tips the spent cell up
   about 20 degrees.
3. Right arm pinches the raised cell across its rim (light grip) and lifts; the cell pivots in the pads and its low
   edge slides out from under the hook. It is lowered into the tray and released, and the arm parks.
4. Left arm pinches the new cell on the staging boss, carries it over the socket offset 2 mm toward the latch and
   lowers it onto the latch tab: the cell tilts in the light grip. It slides 4.5 mm toward the hook, which puts the low
   edge under the hook, and lets go.
5. Left arm closes its fingers and presses the raised (latch) half of the cell down: the edge rides the 45-degree
   lead-in, pushes the latch back, and the latch snaps over the edge. The arm rises and parks.

## Real-trial build, reset and safety

Parts: two CR2032 lithium coin cells (IEC 60086-2 CR2032, e.g. Panasonic or Energizer); a horizontal latch-type
CR2032 board socket (the motherboard type with a spring-steel latch and a moulded hook, e.g. LOTES AAA-BAT series)
soldered to a 160 x 150 mm section of a scrap server system board or to a 1.6 mm FR-4 blank, with a PCIe x16 slot
beside it at the stated position; four 6 mm M3 brass standoffs on a 1.0 mm steel tray (or the board left in its
chassis); a 40 x 40 x 3 mm ESD plate with a 12 mm dia x 8 mm boss (3D printed) as the staging stand; a 60 x 50 x 20 mm
ESD parts tray. Fix the tray, stand and disposal tray to the bench with double-sided tape.

Reset: latch a spent cell "+" up in the socket, place a new cell "+" up on the boss, empty the disposal tray.

Safety: the board is never powered; a real trial uses the robot's guarding and e-stop as usual. Insulate the
terminals of spent cells (tape) before discarding them. No other hazard is present.

## Licences and assets

All geometry is built from primitive boxes and cylinders and one extruded convex section with the team's own parts
kit (`yam_parts.py`, `yam_joints.py`, `yam_mesh.py`, included). No third-party models, textures or fonts.
