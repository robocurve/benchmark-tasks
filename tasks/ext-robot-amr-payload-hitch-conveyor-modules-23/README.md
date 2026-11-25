# Drop a motorized roller's hex ends point-up into its frame slots without pulling its cable, then fit its torque plate

Building the roller-conveyor top module of a mobile robot (AMR) means fitting 24 V motorized drive rollers into the
module's side frame. A drop-in frame has an open-top slot in each rail; the roller's two hex shaft ends must go into
both slots at once, blind under the tube, and the hex must be turned point-up: a hex left flat-up sits 1 mm proud and
scores only 0.25, while the V seat and the torque plate's notch need it point-up. In a
motorized roller the shaft is the fixed part: the motor and gearbox are mounted on it and the tube turns round it, so
the hex is clocked by turning the shaft itself (at its hex end), not the tube. The roller's power cable leaves through the motor-side shaft and is short: a pulled lead strains the seal and conductors
where it enters the shaft, so the roller is handled by its tube, never by or against its cable. Finally an anti-rotation (torque) plate slides down a guide on the rail and locks over the hex, which
takes the motor's reaction torque. The hard parts are physical: a 0.7 kg round part tethered by a short flexible
cable, two shaft ends aligned and clocked by feel into slots the hand cannot see, then a second part dropped over the
seated hex. The nearest accepted task, feeding a hex shaft through two bearings, slides a bare shaft along its own
axis; here a tethered 0.7 kg round part must have both shaft ends clocked and dropped blind into two slots at once
while its own cable stays slack, and then a second part is dropped over the seated hex.

## The parts (real size)

| Part | Size and material | Real product |
|---|---|---|
| Motorized drive roller | Two parts that turn on each other. Shaft: 280 mm overall, 11 mm hex only on the 27.5 mm stub out of each tube end (round 10 mm through the tube), carrying the motor and gearbox (0.40 kg). Tube: zinc-plated steel, 38 mm OD x 1.2 mm x 225 mm with end caps (0.30 kg), turning on the shaft against the gearbox's drag. 0.70 kg in all | Itoh Denki Power Moller 24, PM380LS (38 mm tube, 24 V DC, hexagon shaft standard; Itoh's catalogue: motor and gearbox inside the tube, shaft held fixed by the frame, tube turns round it). The hex size, stub length, mass split and drag are design values: measure the roller you buy and size the slots to it |
| Power cable | Black, 6 mm OD, 180 mm from the motor shaft to its connector, leaving the left shaft end along the axis | The roller's own lead; 180 mm is a design value (a short lead, as a tight module needs), so measure the purchased roller's lead and lay the cable to the same slack |
| Driver card housing | Green box 60 x 22 x 32 mm on the left rail's outer face; the cable's connector (22 x 14 x 10 mm) is fixed beside it | Itoh Denki HB-510 driver card (a housing of the same footprint is enough for a trial) |
| Side rails | Two dark blue anodised aluminium plates 170 x 75 x 6 mm, inside faces 236 mm apart (between-frame width: tube + 11 mm, as the Itoh catalogue sizes it); open-top slot at mid-length, 13 mm wide, chamfered to 22 mm at the top, 120 deg V seat, apex 44 mm up, hex axis seated 50 mm up | `side_rail.stl` (fabricate.py), or cut from 6 mm plate; two 12 x 236 x 20 mm cross ties |
| Torque plate | SUS304 stainless, 34 x 70 x 4 mm, 72 g (7.93 g/cm3 x 9.03 cm3, the 34 x 70 mm blank less the notch), a 13 mm wide notch at the bottom (its mouth flared to 18 mm over the lowest 5 mm) ending in a point-up 120 deg apex | Modelled on Ultimation's hex-point-up bracket MBB-081 for Itoh Denki rollers; `torque_plate.stl` (laser-cut from 4 mm SUS304) |
| Plate guide | Two posts 8 x 14 mm with 6 x 4 mm front lips (28 mm opening), 28 to 70 mm up, over a floor ledge; pocket 40 x 10 mm (3 mm a side on the plate) | `plate_guide.stl` |
| Roller V blocks | Two grey 90 deg V blocks, 60 x 16 x 28 mm (V apex 12 mm up), at y +-7.5 cm | `v_block.stl` |
| Plate rack | Grey slotted stand, 52 x 30 mm base, 30 mm walls, 6 mm slot | `plate_rack.stl` |

Layout (cm from the midpoint between the arm bases, x forward, y left): rails from x 21.5 to 38.5, slots at x 30;
roller on its V blocks at x 16.5, y 0, axis 3.9 up, hex stubs flat-up (a flat on top, 30 deg from point-up); cable
from the left shaft end (y +14.0) out to y 21.5, back along x to 23.5 and down to the connector at x 23.5, y 15 to 17;
torque plate upright in its rack at x 35.5, y 21.5.

## What the scripted demo does

The demo never grips the tube to clock the hex: it turns the shaft by its hex stub. The score reads the hex angle on
the shaft only; the tube is free to turn on the shaft against its 0.10 N m drag. Both arms work, one after the other.
The right arm reads the shaft's live pose, reaches the roller's right hex stub (the end away from the cable) where it
lies on its V blocks and pinches it across two side flats near its outer end (14 mm of pad on the hex, the finger body
7.5 mm clear of the tube end), servoing its measured tool point onto the stub before closing, the wrist turned about
the roller's axis by the hex's measured corner angle (30 deg at the flat-up start). It then levels the wrist about the
stub's own axis, which turns the shaft to point-up inside the tube: the tube stays on its V blocks (their friction,
about 0.18 N m, beats the 0.10 N m drag and the damping at that slow turn), so the shaft turns relative to the tube, as it would on a real roller. The
hand opens and rises, the demo reads the hex angle again, and it repeats the stroke (up to 4 in all) until a corner is
within 1 deg of up; then the right arm parks. This first clocking is not trusted to last: the cable cannot twist and
the shaft is held to the tube only by the 0.10 N m drag and the key, so the shaft can turn back while the roller is
carried by its tube (in one Isaac run the hex reached its slots unclocked, perched about 1 mm proud, and the plate
then sat leaning on it). The shaft is clocked again, at its seat, before the plate goes on (below). When the pinch opens, the shaft can settle back through the key's free
play in its sleeve (about 2.4 deg), so each turn goes past point-up by a spring-back allowance. It starts at the key play and is corrected from the angle read after each stroke
(more if the hex fell short, less if it went past). Each pinch (the stub, then the plate) is checked by the gripper
opening after closing; on a reading that looks empty the hand opens and tries once more, and the second close goes
ahead whatever it reads, since Isaac's pads read about 4 mm under the part and a real miss leaves the part unscored.

The left arm then grasps the tube at its middle (its live position), pads across the tube, lifts it straight up and carries it over the
rails at 12.5 cm (the cable spans at most 12.3 cm of its 18.0 cm). The shaft keeps its clock in the carry because the
keyed sleeve and the drag hold it to the tube. Over the slots it servoes its measured tool pose (encoder forward
kinematics) onto the target, cancelling the sag the 0.7 kg roller puts on the wrist. With the hex corners just over
the rail tops it reads the shaft's live midpoint and moves the hand until the shaft is over the slot line and centred
between the rails (up to 3 corrections of at most 8 mm: the tube can sit 2 mm either way along the shaft, and the hand
lags under the load), then lowers both hex ends into the slots together and presses until the arm lags its command
(the hex on its V seats). It opens and rises, and waits over the plate rack while the right arm clocks the seated
shaft: the demo reads the shaft's angle, and if a hex corner is not within 1 deg of up the right hand pinches the right
hex stub across two side flats where it stands out past its rail (pads centred 13.5 mm outboard of the rail's outer
face, about 10 mm of pad on the hex), the hand leaned 25 deg outboard about x so the fingers stay off the rail top (a
lean about x keeps the pads flush on the flats), and turns the wrist about the stub's own axis until a corner points
up. The shaft is turned directly; the tube, free between the rails, follows through its drag or not. A point-down
lower corner sits flush in its 120 deg V seat, which holds the clock once the pinch opens. The hand backs out along
its fingers, the demo re-reads the angle, and it repeats (up to 4 strokes) until a corner reads within 1 deg of up;
only then does the left hand pick the torque
plate from its rack by a pinch across its two faces 18 mm below its top edge (a pinch on its 4 mm edges let the
plate squirt out of the pads) and carries it over the guide pocket. A pinch across the faces lets the plate swing in its
own plane (in one Isaac run it swung about 30 deg in the pads and wedged diagonally on a guide post), so the demo
servoes the plate, not the hand: it reads the plate's live pose, averaged over half a second because a hanging plate
keeps swinging, turns the wrist about the pinch axis by the plate's lean and shifts the hand by half the notch's x and
y error (up to 4 corrections 2.5 cm over the guide). It then lowers the plate in four short servoed stages (bottom 6 mm
over the guide tops, then notch 22, 14 and 6 mm over the hex), each keeping the hand's trim offset and re-centring the
plate on its live pose; a single long move dropped the offset and in the replay the plate drifted 11 mm onto the hex
top. With the notch 6 mm over the hex and the plate 24 mm down in the guide, it opens and lets the plate drop onto the
point-up hex, the guide holding it upright. It reads the plate's pose again; if the plate is not down on its seat, it picks the
plate up where it lies, wrist leaned with it, and tries again (up to 3 tries). Then the arm swings back to its ready
pose.

What the demo reads: the fixed layout (object_jitter=0), its joint encoders and gripper openings, and from the
simulator the shaft's pose (the cable's end pose: where the stubs are and the hex's corner angle), the tube's
position and the torque plate's pose, to aim the stub pinch, size each clocking stroke and its spring-back allowance,
aim the tube grasp, centre the shaft over the slots, square and centre the held plate over its guide and check that it
dropped onto its seat. It never sets a pose or edits the scene. It prints `RCDIAG` lines to the run log: the roller's
start pose, the hex angle after each clocking stroke, each centring correction, the hex angle once the roller is
seated, the hex angle before and after each in-slot clocking stroke, each plate correction and each plate drop.

What the scorer reads, each step: the shaft's pose (the cable's end body: both stub axes, the axis direction and the
hex corner angle), the cable's segment centres and its anchored start (span and where the cable lies), the torque
plate's pose, and whether the grippers are within 3 cm of the roller, cable or plate. A real trial reads the same
things with a tape, a feeler gauge, an angle gauge on an end flat and the trial video.

## Fidelity

Full size. The roller is two bodies, as a real motorized roller is:

- **Shaft:** both hex stubs, a 10 mm round shaft through the tube, and at its middle a flat key 100 x 24 x 12 mm
  standing in for the motor and gearbox housing fixed to the shaft (0.40 kg in all). It is the end body of the cable: a
  9-link chain (20 mm links, 6 mm) whose other end is anchored at the connector, so the cable stays attached to the
  shaft whatever the tube does. The cable is limp, as a PVC power lead is: each joint is elastic only to 3 deg
  (0.01 N m/rad) and then keeps the bend it is given (ropekit's plastic set), so it lies where the roller's move leaves
  it instead of springing back to its laid shape. An elastic cable would push the shaft along its axis and turn a
  clocked hex back.
- **Tube:** a 38 x 1.2 mm steel shell with end caps (0.25 kg) on a hinge about the roller axis, riding on a drive
  sleeve (0.05 kg) that surrounds the key with 0.5 mm a side (about +-2.4 deg of free play, like a gearbox's
  backlash) and is closed at both ends by
  plates the key cannot pass (2 mm of axial play). The tube turns on the shaft through about +-170 deg (far more than the 30 deg clocking needs) against a joint drag of
  0.10 N m (Coulomb, to start it turning) plus 0.1 N m s/rad of damping. These stand for the gearbox's back-drive
  resistance; Itoh publishes no figure for it (its manuals only warn against forcing a stopped roller round), so they
  are design values to measure on the real roller. The tube is a separate free body kept on the shaft by geometry; the
  shaft, not the tube, sets the clocking that is scored.
- The chain cannot stretch, so a pull shows as its span reaching 95% of its length, which score() latches. No motor
  turns. Slots, V seats and the plate's notch are contact geometry with 1 mm clearance on the hex side flats; the V
  seat and notch flanks match the hex flats, so a hex turned 30 deg off point-up rests 1 mm high and a plate over it
  1 mm high (check.py). In the simulator each V block is built from native boxes (one slab under each 45 deg face, two
  outer pillars, a base) with the printed block's outline, so the tube rests on plain box faces.
- The simulator scores the last step without opening the grippers, so a part held more than 1 cm above its seat counts
  as not seated, and the last 0.25 needs both hands 3 cm clear of the shaft and cable, the tube and the plate.
- Offline checks (check.py): the key is free at 1.5 deg in its slot and blocked at 4 deg; the sleeve cannot slide off
  the key; the tube turned 30, 90 and 180 deg on its hinge stays at least 0.9 mm clear of the shaft; the seated tube
  clears both rails across its axial play; every shaft offset a seated roller can reach (7.5 mm) leaves at least
  3 mm of hex on each rail. A MuJoCo replay of the demo with the cable left out (MuJoCo has no ropes) turned the shaft
  to point-up in three strokes (4.4, 1.6 and 0.6 deg after each) while the tube stayed on its V blocks (the tube's
  hinge read -30 deg on the shaft afterwards: the shaft turned inside the tube), corrected a 5.7 mm offset of the
  shaft along y over the slots, seated the hex at 2.0 deg (the in-slot check then read 0.1 deg, so no in-slot stroke),
  seated the plate, and scored 1.0, held 8 s, at 91.8 s. A second replay skipped the pre-seat clocking, so the hex
  reached its slots 26.5 deg off point-up (as in the failed Isaac run): one in-slot stroke turned it to 0.1 deg, with
  no finger touching the frame, the plate seated 0.2 mm from its seat, and it scored 1.0 at 75.3 s. check.py runs
  both replays and requires the final shaft within 2 deg of point-up and the plate notch within 1 mm of its seat.

## Files

- `ext_robot_amr_payload_hitch_conveyor_modules_23.py`: the task file for `yam_arena` (scene, `score()`,
  `success()`, `demo_script()`).
- `mechkit.py`, `ropekit.py`, `closedloop.py`, `belimits.py`: the shared kit (geometry, the cable, the closed-loop
  demo controller, geometry helpers), unchanged.
- `fabricate.py`: writes the STL files of the custom parts, including the inert roller's drag hub
  (`python3 fabricate.py out_dir`).
- `task.json`: the task's details.

## Run a trial

```bash
INSTR="Seat the roller's hex ends point-up in the frame slots without pulling its cable, then drop the torque plate over the hex."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=ext_robot_amr_payload_hitch_conveyor_modules_23.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --instruction "$INSTR" --scorer task_score \
  --max-steps 3600 --store-frames --log-dir ~/yam-eval/roller-dream

# Scripted demo (right arm clocks the shaft, left arm seats the roller and fits the plate)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=ext_robot_amr_payload_hitch_conveyor_modules_23.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --instruction "$INSTR" --scorer task_score \
  --max-steps 3600 --store-frames --log-dir ~/yam-eval/roller-demo
```

- **Seed and layout:** `-E object_jitter=0` is required; the cable is never jittered.
- **Runtime per trial:** the demo's script runs about 67 to 85 s of simulated time (1,350 to 1,700 steps; up to about 2,600 if
  the in-slot clocking or the plate drop is retried), then `success()` must hold
  8 s; a Dream-1 trial runs up to 3,600 steps (3 minutes simulated).
- Wall-clock time on one L40S, including Isaac Sim start-up: about 1130 s for the demo and about 1850 s for a full Dream-1 trial.
- **Score:** `task_score` reports 1.0 when `success()` holds for 8 s (score 1.0 with the plate and cable still),
  otherwise `score()` at the end of the run.

## Scoring

At the end, with both grippers open:

1. **Score 0** if the cable was ever pulled straight (its span from connector to shaft end over 95% of its 18.0 cm
   length for 0.25 s or more; in a real trial, a tape laid from the connector to the shaft end reads 17.1 cm or more, checked frame by frame on the trial video against a tape fixed beside the cable run), or it ends between the rails, in a slot or in
   the torque plate's pocket (the cable's attachment point on the shaft end is not counted: it is fixed to the shaft and lies just
   outside the rail when the roller is seated).
2. **0.25** when both hex stubs sit in their slots: axes within 1.5 mm of the seat height (50 mm up) and 2 mm of the
   slot centre line, each hex stub over at least 2 mm of its own 6 mm rail, and level within 3 deg. A seated shaft
   can sit up to 7.5 mm off centre (the tube ends stop 5.5 mm from the rails and the tube has 2 mm of axial play on
   the shaft), which still leaves 3 mm of hex on each rail, so only a stub off its rail fails this.
3. **0.50** when also the hex is point-up within 5 deg (a digital angle gauge on an end flat reads 90 +- 5 deg). This
   is read on the shaft only (the tube turns on it against its drag). The shaft's free play in its sleeve (about
   2.4 deg) is inside the 5 deg band, but a hex clocked with no allowance for it can settle back out of the band.
4. **0.75** when also the torque plate is down in its guide over the hex: its notch apex within 1.5 mm of its seat and
   3 mm of the slot line, upright within 10 deg.
5. **1.0** when also both hands are 3 cm clear of the roller, cable and plate.

In a real trial the seat heights are judged with a 1.5 mm feeler gauge: it must not pass under either hex stub's
lower corner at its V seat, nor under the plate's notch apex on the hex; the slot centre line and clocking are read
with a steel rule and a digital angle gauge.

Doing nothing scores 0 (the roller stays on its V blocks). Seating the roller flat-up scores 0.25; a plate dropped
with no roller seated falls to its ledge, 12 mm under its seat, and scores nothing.

## Real-world build and reset

Bill of materials (a real trial needs no power; leave the roller unplugged from any supply):

| Item | Source | Price (estimate unless a listing is cited) |
|---|---|---|
| Motorized roller, Itoh Denki PM380LS (or any 24 V Itoh roller; the PM486FE, 48.6 mm, lists at $231 for a 17 in BF roller at ultimationinc.com) | Itoh Denki distributor, quote | about $230 to $300 |
| Cheaper inert route, a roller whose tube really turns on its shaft: a 7/16 in (11.1 mm) steel hex bar cut to 280 mm (the shaft and both hex stubs); two United Sales UGFH.4375X1375-2RS sealed hex-bore conveyor bearings (7/16 in hex bore, 1.375 in = 34.9 mm OD, listed at zoro.com) pressed into the tube ends on one wrap of 0.3 mm shim; a 38 mm OD x 1.2 mm steel tube cut to 225 mm; the printed `drag_hub.stl` with a 27 x 3.5 mm nitrile O-ring; lead tape; 6 mm two-core cable 18 cm | metal supplier, conveyor parts seller, hardware store | about $45 |
| Side rails, plate guide, V blocks, rack (fabricate.py, PETG, about 150 g) | any printer | about $5 |
| Torque plate, 4 mm SUS304 laser-cut from `torque_plate.stl` | online laser-cut service | about $15 |
| Base board 40 x 40 cm, M4 screws, double-sided tape | hardware store | about $15 |

Total about $80 with the inert roller, about $270 with a real roller.

The inert roller reproduces the PM380LS's 38 mm tube and 11 mm-class hex shaft (Itoh's dimensions; 7/16 in hex is
11.1 mm, so size the printed slots from the bar you buy) and the way it is built: the shaft is one rigid part through
the tube, and the tube turns round it on two bearings against a drag. The drag hub stands in for the motor and gearbox
fixed on the real shaft.

Inert roller assembly: slide the drag hub onto the hex bar and glue it at the middle; fit the O-ring in its groove;
wrap lead tape round the hub until bar and hub weigh 400 g; slide the bar into the tube and press a bearing into each
tube end over the bar, leaving 27.5 mm of bar out of each end. Set the drag: hold the bar, wind a string round the tube
and pull it with a luggage scale; the tube should start turning at about 5.3 N (0.10 N m at the 19 mm radius). Too
stiff: sand the O-ring groove root; too free: a thicker O-ring. Solder or tape the cable to the left bar end so it
leaves along the axis.

Frame assembly: screw both rails to the base board 236 mm apart (inside faces), slots opposite each other at x 30 cm, with
the two cross ties between them; screw the plate guide to the left rail's outer face, centred on its slot, ledge
28 mm up; fix the driver-card housing beside it and tape the cable connector to the board at x 23.5, y 15 to 17; tape
the V blocks and the plate rack down at their marks and tape the cable's start outline on the board.

Reset: lift the plate back into its rack and the roller back onto the V blocks; turn the shaft by its right hex stub
until a flat is on top of each stub (the tube need not move; printed V blocks grip the tube less than the gearbox drag, so on a real
roller the tube may turn with the shaft, which does not change the score, or line the V blocks with rubber tape to hold it), and lay the cable along its outline. Wipe the hex stubs
and slots dry of any grease; nothing else needs cleaning. Nothing is consumed and nothing is wet; every force is under 10 N (a 7 N roller, a
0.7 N plate), well inside the YAM's 40 N per finger.

## Licence

All geometry is procedural, generated by mechkit.py and ropekit.py in this folder; no third-party assets.
