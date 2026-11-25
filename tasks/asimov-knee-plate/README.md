# Turn a loose Asimov humanoid leg onto its back, seat its knee side plate flush, then start and tighten two M3 bolts

Robotics technicians assemble humanoids from their makers' guides; Menlo's open-source Asimov 1 is built this way, in a market the IFR counted at about 7,000 humanoids sold in 2025 and Omdia at about 13,000 shipped. Left Leg Step 8: turn the leg so the thigh's bearing faces up, fit side plate 600_08A on housing 600_10C "such that the protrusion fits into the bearing and then all the holes ... align with the threaded insert", then lightly tighten M3x10 flat bolts (DIN 7991) into the inserts. The leg, 3.03 kg (twice the arm's 1.5 kg payload), lies loose on its side and is tipped onto its back, not lifted. Then three fits meet on one axis within 5 mm: plate in pocket 0.3 mm a side, 34.8 mm pad in a 35.0 mm bore, 11.85 mm boss in a 12.0 mm bearing. A cocked plate puts its holes off the inserts, and the bolts will not start. Last, a hex driver takes a 3 mm bolt from a stand on its magnetic bit, sets it in a countersunk hole on its insert's first thread, and turns it in, 9 turns of a 0.5 mm pitch with the bit pressed square in a 2 mm hex socket, until its head is down and snug; then a second bolt.

**Instruction.** Turn the leg onto its back, seat the knee side plate flush in its pocket, then start and tighten M3 bolts in the two holes on its long axis nearest its narrow end, and move both hands clear.

## Requirements

`yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source <ARENA_DIR>/yam.env`). `mechkit.py`, `demokit.py`, `gatekit.py` and `knee_plate.npz` (the maker's geometry as meshes and colliders) are in this folder.

## Run a trial

From this folder:

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 8000 --seed 1 \
  --instruction "Turn the leg onto its back, seat the knee side plate flush in its pocket, then start and tighten M3 bolts in the two holes on its long axis nearest its narrow end, and move both hands clear."
```

## Scripted demo

```bash
python demokit.py task.py OUT --mode full   # the whole act, with a video
python demokit.py task.py OUT --no-record --mode side_slide   # lazy route on this code, on GPU
python check_score.py                       # the scorer on hand-built states, no simulator
```

The demo reads the poses of the leg, the plate, the screwdriver and the bolt stand to plan its moves; it moves no object except through the grippers. Both hands grip the leg's ends from above and swing together about the edge the leg rests on, toward the robot, past its balance point at 38 degrees; they let go and it settles onto its back. The right hand pinches the plate across its neck, lifts it out of its post, squares it to the face as the leg lies, lets go just above the face on the bearing's axis and presses it home. Then the right hand takes the screwdriver from its stand by the handle, bit down; the left hand's closed fingertips rest on the plate's round end. For each bolt the bit comes down onto the head of a bolt standing in the stand, the magnet takes it, the bolt is carried high over its hole, squared to the hole and let down until it rests on the insert's first thread. The bit, pressed into the socket at 6 to 8 N, is turned by the wrist in strokes of about 160 degrees, the ratchet free on each back stroke, until the head is down and snug. The demo is not reliable past the first bolt: on the runs in Results it starts and drives one bolt about 5.3 to 5.9 turns (score 0.6) or none (0.4); the second bolt and the 1.0 state are not reached. Last the right hand sets the screwdriver back on its stand and the left hand lifts off the plate.

## The scene and what is simplified

Station: two YAM arms with parallel grippers, bases 61 cm apart at the near edge of the table. Positions in cm from the midpoint between the arm bases: x forward, y to the robot's left; heights above the table top. All Asimov parts are from Menlo's Asimov 1 repository (github.com/menloresearch/asimov-1, CERN-OHL-S-2.0): shapes from the maker's STEP files (the thigh from the maker's URDF mesh).

- Leg: the thigh assembly (URDF left_hip_yaw_link: 1.949 kg, with its centre of mass and inertia) slotted into knee housing 600_10C (MJF PA12, 1,069.5 cm3 at 1.01 g/cm3: 1.080 kg), 3.03 kg in all and 54 cm long, loose on the table. Its knee turns freely from straight to 1.50 rad bent (URDF) against the unpowered knee actuator's published friction, 0.70 N m to start and 0.02 N m moving, with 0.034 kg m2 reflected inertia (Menlo's actuator table, EC-A4315-P2-36). It starts straight, on its side along y, shank toward -y, the plate side facing +x, the pocket's centre at (46, -7), 6.3 cm up. On its back it rests on three points, face up about 4 degrees off level, the pocket's centre at about (31, -6), 10.3 cm up.
- Pocket: 5.0 mm deep, the plate's outline with 0.3 mm clearance, a 35.0 mm bore at its centre, the thigh's flanged bearing 600_11X (12.0 mm bore) on its axis, and nine M3 brass heat-set inserts in the housing's insert bores (4.4 mm across, 4.5 mm deep under a 0.5 mm lead-in, 5.5 to 10 mm under the face, STEP).
- Plate 600_08A: aluminium 7075, 5.0 mm sheet, 149.4 mm long, 79.4 mm across its round end and 39.4 mm across its neck; a 34.8 mm pad and an 11.85 mm boss on one face; nine 3.2 mm holes countersunk 6.72 mm at 90 degrees; 120 g. It starts boss down in the 36 mm, 12 mm deep socket of a kitting post 50 mm across and 30 mm tall at (20, -17).
- Bolts: nine M3x10 flat bolts, DIN 7991, steel (3.0 mm thread, 0.5 mm pitch, head 6.0 mm across and 1.7 mm tall, 2 mm hex socket), stand head up in a printed bolt stand at (13, -25): a 40 x 40 x 8 mm deck with nine 3.2 mm holes on a 12 mm grid, a tower beside it on the robot's side (34 mm square, 44 mm tall, a 14 mm hole) holding the screwdriver bit down. The stand with its bolts is one loose 50 g body.
- Screwdriver: Bahco 808050S stubby ratchet screwdriver (95 mm, magnetic 1/4 in bit holder) with a Wera 840/1 Z Hex-Plus 2.0 x 25 mm bit, 13 mm of it out of the holder; 90 g. The handle's profile and the 90 g are estimates from the maker's photographs and size.
- Thread (what is modelled and how): each of the two bolts the task uses is a slider along its hole's axis on a stiff seat spring (40,000 N/m, through which the bit's push is read) and a hinge about it, both children of the housing. A bolt carried on the bit starts when its tip is put into the hole of the seated plate (the plate's hole within 0.6 mm of its insert's axis, within 1 mm of flush and square within 3 degrees), with the bit within 0.9 mm of the hole's axis, within 5 degrees of square, and let down to the first thread (its head's top 4.5 mm above the face). From there the bolt turns only with the bit: each tightening turn of the bit counts while its tip is in the socket (within 0.9 mm of its axis, at least 0.5 mm down), within 10 degrees of the bolt's axis and pushed in with at least 5 N, and drives the bolt 0.5 mm down (M3 coarse pitch); the bolt resists with 0.10 N m. 9 turns bring the head to the countersink; a further 20 degrees raise the torque to 0.30 N m (snug: Step 8 says only "lightly tighten"; 0.30 N m is an estimate, under a quarter of the 1.37 N m that tightening tables give an M3 steel bolt). The ratchet drives only the tightening way. Cross-threading is not modelled: a bolt more than 5 degrees off square does not start at all, where a real one may start crossed; the Real trial counts a crossed bolt as not started.
- Pick: the bit takes a bolt standing in the stand when its tip rests on the head within 0.9 mm of the socket's axis and 1.2 mm of the head's top, square within 10 degrees: a steel M3 bolt (0.5 g) held by the magnetised bit. The carried bolt is drawn, not collided, until it starts.
- Physics: rigid bodies at 240 Hz. Plate (45 convex colliders), pocket (217) and bearing (48) are the maker's STEP geometry; no collider widened: rim 0.30 to 0.34 mm a side, pad 0.10 to 0.12, boss 0.075 to 0.09. Friction 0.3. The knee's dry friction is a coded drive. The inserts are visual parts inside their bores. The bolt holes not used by the task (seven of nine) stay empty.

## Real items

- Menlo Asimov 1, left leg: plate 600_08A, knee housing 600_10C, flanged bearing 600_11X, thigh assembly (Left Leg Step 7): https://github.com/menloresearch/asimov-1 and the assembly guide, https://docs.menlo.ai/asimov/1/build/assembly-steps/left-leg-module/knee-and-calf-assembly/step-8
- Bolts: M3x10 flat (countersunk) hex socket, DIN 7991 / ISO 10642, steel 10.9 (Menlo part id m3-x-x-10mm-din-7991), e.g. https://bolts.co.uk/m3-x-10mm-socket-screw-countersunk-din-7991-grade-10-9-bzp-pack-of-100-p-SKMCZP03010 (100 pcs) or Cromwell box of 200 at about 3.49 GBP ex VAT; steel, so the magnetised bit holds them.
- Screwdriver: Bahco 808050S stubby ratchet screwdriver, https://www.rapidonline.com/bahco-808050s-stubby-ratcheting-screwdriver-86-2182, with a Wera 840/1 Z Hex-Plus 2.0 x 25 mm bit (05056305001), https://www.buerklin.com/en/p/wera/screwdrivers-bits-and-bitholders/05056305001/02L3938 (about 2.39 EUR).
- Inserts: M3 brass heat-set inserts for the 4.4 mm bores (for example ruthex M3 inserts, https://www.3djake.com/ruthex/threaded-insert-m3-100-pieces, 100 pcs about 8.34 USD ex VAT; the bores are the STEP's, so choose the insert that fits a 4.4 mm bore, 4.5 mm deep).
- Bearing 600_11X: the flanged 12.0 mm bore, 23.0 x 5.0 mm bearing is a catalogue part, its STEP in the Menlo repository above (mechanical/ASV1/600/OFF_THE_SHELF/ASV1_600_11X.STEP); buy a flanged 12 x 23 x 5 mm deep-groove bearing of that size.
- Torque check: Wiha TorqueVario-S 0.1 to 0.6 N m with a 1/4 in bit holder (36852), https://eshop.hoffmann-group.com/GB/en/houk/p/36852-.
- Gauges: a 0.01 mm digital depth gauge, Duratool D03198 0 to 25 mm, https://cpc.farnell.com/duratool/d03198/digital-depth-gauge-0-25mm/dp/TL19623 (about 14.5 GBP ex VAT; any 0 to 25 mm, 0.01 mm digital depth gauge serves the 0.5 mm flush threshold); a 2.4 mm pin gauge, Vermont Gage class ZZ plus 2.40 mm, 112102400, https://zoro.com/vermont-gage-pin-gage-plus-class-zz-240mm-112102400/i/G9975500 (about 4.15 USD); a digital inclinometer reading 0.1 degree, Digi-Pas mini pocket-size digital level DI87342, https://zoro.com/digi-pas-mini-pocket-size-digital-level-electronic-angle-gauge-with-01-degree-accuracy-di87342/i/G811233908 (about 44 USD).

## Fidelity: each number the bolt act depends on

| Number | Sim | Real | Source or bound | Real-trial check, accepted range | Direction |
|---|---|---|---|---|---|
| Push on the bit for a turn to count (PUSH_MIN) | 5 N | a hand presses a stubby driver with a few N so the bit does not cam out | estimate, bound 3 to 8 N | kitchen scale under the leg while the bit turns: 3 to 8 N | a turn without the push counts in neither; end runs not run (Results) |
| Running torque (RESIST) | 0.10 N m against the turn, at full value once the bolt turns at 0.02 rad/s, about 1 degree a second (Coulomb drag) | an M3 bolt running in a brass heat-set insert, no load on the head | estimate, bound 0.05 to 0.20 N m, under the 0.43 N m ISO 2320 allows a prevailing-torque M3 nut at first fitting | torque screwdriver on a bolt turned in by hand: 0.05 to 0.20 N m, else clean or replace the insert | both ends run as one-bolt runs (Results): a bolt started and was driven 5.3 to 5.5 turns at each |
| Snug torque (TIGHT_T) | 0.30 N m after a further 20 degrees with the head down | Menlo Step 8: "lightly tighten" | estimate, bound 0.2 to 0.45 N m, under the 1.37 N m tightening tables give an M3 steel bolt | torque screwdriver set to 0.30 N m clicks before the bolt turns | ends run as one-bolt runs (Results): no bolt started at either end in those runs |
| Thread's axial stiffness (SEAT_K) | 40,000 N/m: 10 N gives 0.25 mm | steel in brass thread, far stiffer | solver stand-in for a rigid thread, through which the push is read | none needed: the head is read with no push on it | the head reads up to 0.35 mm low under a 14 N push, inside HOME_H |
| Bolt home (HOME_H) | head within 0.5 mm of the face, 9 turns | countersunk head flush in its 90 degree countersink | ISO 10642 head 1.7 mm in the 6.72 mm countersink | depth gauge (0.01 mm): head within 0.5 mm of the face | same rule |
| Start window | bit axis within 5 degrees of the hole's, the head's top from 1.5 mm below to 0.8 mm above its first-thread height (4.5 mm proud), after the bolt hung at least 6 mm over the first thread, on the hole's axis and square, and came straight down | an M3 starts square | estimate from the 3.0 mm shank in the 3.2 mm hole | the bolt sits square on its first thread before the first turn (video) | same |
| Drive window | bit tip within 0.9 mm of the socket's axis, at least 0.5 mm down, within 10 degrees | 2.0 mm hex bit in a 2 mm hex socket | the socket's wall holds the bit to the real fit; the window only reads it | a turn counts only with the bit seated in the socket | same: a bit on the rim is not in the window |
| Pick window | bit tip on the head within 0.9 mm of the socket's axis and 1.2 mm of the head's top, within 10 degrees | magnetised bit takes a 0.5 g steel 10.9 bolt (A2 stainless is not magnetic) | socket depth 1.1 mm (ISO 10642) | the bit lifts a bolt from the stand and holds it through a 30 cm carry | same |
| Bit in socket (BIT_SOCKET_R) | 0.05 mm a side, as a twelve-sided wall | socket 2.014 to 2.074 mm (ISO 10642, s E11) on a 1.96 to 2.00 mm bit (ISO 1173): 0.007 to 0.057 mm a side | solver departure: hex flats at 0.05 mm jitter in contact at 240 Hz, so the wall is round-ish and the turn reaches the bolt through its coded hinge while the bit is in the drive window | none | easier: no cam-out from the flats; the 5 N push and 10 degree rules stand in for it |
| Snug cap | the snug turn stops 30 degrees past head-down (0.45 N m) | the head stops in its countersink | geometry: a countersunk head cannot turn further in | torque screwdriver clicks at 0.30 N m | same |
| Hole over its insert for a start (LAND_LAT, LAND_DEG) | the plate's hole within 0.6 mm of its insert's axis, square within 3 degrees, the plate within 1 mm of flush | an M3 starts only within about 0.6 mm: 0.1 mm of play in the 3.2 mm hole plus the insert's 0.5 mm lead-in | geometry of the hole and insert | a bolt that does not start within a quarter turn is not started | same |
| Cross-threading | none: a bolt more than 5 degrees off square does not start | a bolt started off square can cross | the sim has no cross-threading | a crossed bolt (stops before 2 turns or sits tilted) is not started | easier: a crossed start cannot happen in the sim |
| Screwdriver (DRV_KG) | 90 g, handle from the maker's photographs | Bahco 808050S | estimate, bound 70 to 120 g | kitchen scale | none: the gripper holds 120 g |
| Bolt stand and tower | printed, 40 x 40 x 8 mm deck, 34 mm square 44 mm tower with a 14 mm hole, 50 g with its bolts, loose | the same print | dimensions given here | printed within 0.2 mm, set at (13, -25) | same |

## Real trial

- Rig cost (bought tools and parts, USD, from the listings in Real items): Bahco 808050S about 22 (16.82 GBP ex VAT); Wiha TorqueVario-S 36852 about 190 (148 GBP ex VAT); depth gauge about 19; pin gauge about 4; inclinometer about 44; bolts, bit and inserts about 15. TOTAL about 295 USD, under 300; the printed parts and the 5 mm 7075 sheet for the plate are extra.

(a) Setup, positions in cm as above (x forward from the midpoint between the arm bases, y to the robot's left). Print the thigh and housing 600_10C from the published STEP files (PA12 by MJF if you have a service, PLA otherwise) and ballast the pair to 3.03 kg with the URDF centres of mass: weigh the prints, glue the missing mass as steel shot in one pocket in the thigh and one in the housing, and check by the tip test (tipped over its lower edge it passes its balance point at about 38 degrees). Fit the real bearing 600_11X in the thigh with a 12 mm steel pin as the knee, and a friction screw on the pin set so a spring scale at 0.30 m reads 2.3 N to start it (0.70 N m). Melt nine M3 brass inserts into the housing's insert bores with a soldering iron and insert tip, flush with the bore floor. Cut the plate from 5 mm 7075 sheet to the STEP outline with its turned pad and boss (or print it in PA12 and ream to size). Lay the leg on its side, pocket centre at (46, -7), plate side away from the robot, on a 3 mm rubber mat, and mark its outline with tape. Set the plate boss down in a printed post (50 mm across, 30 mm tall, 36 mm socket 12 mm deep) at (20, -17). Print the bolt stand (PLA, 40 x 40 x 8 mm deck, nine 3.2 mm holes on a 12 mm grid, a 34 mm square, 44 mm tall tower with a 14 mm hole on its -x side) and set it loose at (13, -25); stand nine bolts head up in it and the screwdriver bit down in the tower. Stick 1 mm silicone pads on the fingertips so the grippers do not mark the plate.

(b) Reset: unscrew the two bolts with the screwdriver and stand them back in the stand; lift the plate out by its neck's flats and set it boss down in the post; straighten the knee and lay the leg back on its side on its tape marks; stand the screwdriver in its tower.

(c) Steps: the robot is given the instruction and runs to the end of its time (400 s). Film the whole trial at 30 fps from above the leg, with a 10 cm scale bar on the mat in view, and a second camera level with the plate.

(d) Scoring by hand, at the end:
- 0.2: the leg on its back on the mat, the face within 10 degrees of level (a digital inclinometer on the face, 0.1 degree), resting on the mat (not held).
- 0.4: the plate seated flat: a depth gauge (0.01 mm dial) on the plate's top against the face reads within 0.5 mm of flush at ten points round the outline; a 2.4 mm gauge pin passes through each hole with no bolt in it into its insert's thread (all nine before the bolts, the seven left at the end); a hole with a bolt in it is read by its bolt.
- 0.6: one bolt, in one of the two holes on the plate's long axis nearest its narrow end, started in its insert and driven at least 2 turns by the robot: read by the head's height with the depth gauge: 4.5 mm proud on the first thread, at most 3.5 mm proud after 2 turns (0.5 mm pitch). A bolt set in the hole but never turned, or crossed (it stops before 2 turns, or its head stands tilted against the countersink), does not count.
- 0.8: two bolts each so.
- 1.0: both bolts home: the head down in its countersink (the depth gauge reads the head within 0.5 mm of the face), and snug: a torque screwdriver set to 0.30 N m clicks before the bolt turns. Both hands at least 10 cm from the plate and 8 cm from the leg's centre line, and the plate and leg still for 1 s (video).
- The score is 0 if the plate or the leg falls off the table. The score stays at 0.2 if the plate was set in the pocket before the leg was on its back, if the leg slid more than 15 mm or turned more than 3 degrees once the plate reached the pocket, or if the plate never left the post in a gripper: each read from the overhead video against the scale bar.

(e) Bench checks before trials, each with its accepted window:
- Plate fit: the plate drops into the pocket by its own weight once square and centred; with gauge pins, rim 0.25 to 0.35 mm a side, pad in bore 0.05 to 0.15, boss in bearing 0.05 to 0.10. Outside: re-print or sand the pocket and re-check.
- Insert thread: a bolt turned by hand starts within a quarter turn and runs in the full 4.5 mm of thread. An insert that binds or sits crooked (the bolt leans more than 2 degrees) is replaced.
- Running torque: with the torque screwdriver, a bolt runs in at 0.05 to 0.20 N m (the sim uses 0.10 N m; one-bolt runs at both ends each started and drove a bolt, Results). Outside: clean or replace the insert.
- Snug torque (an estimate of "lightly tighten"): set the torque screwdriver at 0.30 N m; the bolt, home in its countersink, clicks it before turning. One-bolt runs with 0.20 and 0.45 N m started no bolt on the host used (Results).
- Push on the bit: the driver pressed square in the socket with at least 5 N (a kitchen scale under the leg reads the push).
- Magnet: the bit, touched on a bolt's head in the stand, lifts it and holds it through a 30 cm carry; if not, magnetise the bit with a bit magnetiser.
- Leg: 3.03 kg within 30 g; balance point at 38 degrees within 3; knee starting torque 0.70 N m within 0.10.

## Scoring

Score 0 if the plate or the leg falls off the table. Rungs are cumulative: 0.2 with the leg on its back on the table (the face within 10 degrees of level, at its resting height within 12 mm); 0.4 with the plate seated flat (every point of its top within 0.5 mm of flush, tilt under 1 degree, boss down, every hole with no bolt in it within 0.45 mm of its insert's axis: a bolt started in a hole pins that hole); 0.6 with that and one bolt, in one of the two holes on the plate's long axis nearest its narrow end, started on its insert by the robot and driven at least 2 turns by the bit (a bolt in another hole is not counted); 0.8 with both; 1.0 with both bolts home (9 turns, the head within 0.5 mm of the face, snug at 0.30 N m, an estimate of Menlo's "lightly tighten"), no gripper within 10 cm of the plate or within 8 cm of the leg's centre line, and the plate and leg still (plate under 1 mm in 1 s, leg under 2 mm/s and 0.6 deg/s), held for 1 s and still true at the end. A plate set in the pocket before the leg was on its back, pressed home with the leg moved 15 mm or its face turned 3 degrees from how it lay when the plate first reached the mouth, or never carried by a gripper (10 steps in a row 1 cm up with a fingertip within 3 cm), scores at most 0.2. The bolt rungs read the plate as seated at the same moment: a bolt driven into a plate that has lifted does not count.

## Results

Each run is on the final scorer, seed 1, nothing random, on an RTX 3090 Ti. GPU physics differs slightly between PCs, so the same seed can end differently. One-bolt runs stop after the first bolt (their top score is 0.6). "Open hole" is the offset, from its insert's axis, of a hole with no bolt in it at the last reading (limit 0.45 mm); "n/r" means the reading was not kept.

| Run | Score | Open hole | What happened |
|---|---|---|---|
| Scripted demo, full (`--mode full`), first PC | 0.6 | 0.08 mm | 5,282 steps (264 s); bolt from slot 2 started in hole 8 (the bit 0.69 mm off the hole's axis at the start; the limit is 0.9 mm), stopped when the 3 re-centres the demo allows were used up, at 5.93 turns, head 1.53 mm above the face; second bolt not started; driver back on its stand, 0.5 mm off axis |
| Scripted demo, full, second PC | 0.4 | 0.19 mm | 4,690 steps (234 s); a bolt picked, none started; driver back on its stand, 0.3 mm off axis |
| Scripted demo, full, third PC | 0.6 | n/r | 5,282 steps (264 s); same bolt, hole and 5.93 turns as the first PC |
| Push threshold 3 N and 8 N (one bolt) | not run | | |
| Running torque 0.05 N m (one bolt) | 0.6 | n/r | 3,648 steps; bolt started in hole 8, 5.45 turns |
| Running torque 0.20 N m (one bolt) | 0.6 | n/r | 3,664 steps; bolt started in hole 8, 5.26 turns |
| Snug torque 0.20 N m (one bolt) | 0.4 | 0.11 mm | 3,159 steps; bolt picked, none started |
| Snug torque 0.45 N m (one bolt) | 0.4 | 0.11 mm | 3,159 steps; bolt picked, none started |
| Dream-1, on the revised scene and instruction, run before the final bolt start rule (hover 6 mm, on axis) was set | 0.0 | | task score 0; no bolt touched; not rerun on the final rule |
| Dream-1 film, a run on the earlier version of this task (before the bolts) | 0.0 | | the leg not turned |

PUSH_MIN range ends: not run. The scorer credits two bolts (0.8) and both home (1.0); the scripted demo reaches one bolt at best, so no run above scores 0.8 or 1.0. A bolt starts only after it has hung 6 mm over its hole on the hole's axis and come straight down; on the PC that scored 0.4 the demo's bolt was not started.

Lazy routes. none (the arms stay still) 0.0, undriven (the leg turned, no bolt touched) 0.2 and placed (the plate seated, no bolt) 0.4 ran on GPU on the 802f4bda0 build, before the final start rule (the tier rules are unchanged); only side_slide ran on the final code, c5d2b40d9: side_slide (a carried bolt slid sideways onto its hole instead of let down its axis) 0.4, 0 bolts started. Checked on hand-built end states with `check_score.py` (no simulator; the demo does not reach them): one bolt driven 2 turns or more, no second 0.6; two bolts started 0.8; both home and snug 1.0. A bolt only set in its hole and never turned, a bolt driven before the plate is seated, and a screwdriver left in a head with the hands on it score no bolt rung.


## Runtime and seeds

Nothing is random: seed 1, object_jitter=0. The run limit is 8,000 steps (400 s at 20 Hz); the scripted demo ends at 4,690 to 5,282 steps (Results). On a 24 GB lab-grade GPU the simulation runs at about 2.5 to 4 steps a second, so a full demo takes 30 to 45 minutes of wall time.

## Sources

Menlo Asimov 1 assembly guide, Left Leg Step 8, instructions 3 to 5 (the nine M3x10 Flat bolts, DIN 7991, lightly tightened to align the plate, then each threadlocked in a star order): https://docs.menlo.ai/asimov/1/build/assembly-steps/left-leg-module/knee-and-calf-assembly/step-8. Menlo Asimov 1 joint design and actuation, actuator table (left_knee_joint, EC-A4315-P2-36): https://docs.menlo.ai/asimov/1/reference/joint-design-and-actuation. Part STEP files and URDF, Menlo Research, CERN-OHL-S-2.0: https://github.com/menloresearch/asimov-1 (mechanical/ASV1/600/FABRICATION/ALU_7075/ASV1_600_08A.STEP, MJF_PA12/ASV1_600_10C.STEP, OFF_THE_SHELF/ASV1_600_11X.STEP; sim-model/asimov_1_25dof/urdf/asimov_1_25dof.urdf). Heat-insert drawing for 600_10C (22 M3 inserts; 1 to 9 round the plate's pocket): mechanical/ASV1/Docs/Heat_Insert_parts/Heat_inserts_ASV1_600.pdf in the same repository. Left leg parts list (M3x10 Flat bolt): https://docs.menlo.ai/asimov/1/build/assembly-preparations/part-organisation/left-leg. DIN 7991 / ISO 10642 M3 (head 6.0 mm, 1.7 mm, 2 mm hex, socket depth at least 1.1 mm) and ISO 261 (M3 coarse pitch 0.5 mm): the standards' tables. Humanoid counts: IFR, about 7,000 humanoids sold worldwide in 2025 (as reported at https://aiweekly.co/alerts/ifr-counts-just-7000-humanoid-robots-sold-worldwide-in-2025); Omdia, about 13,000 shipped (as reported at https://www.analyticsinsight.net/robotics/7000-humanoid-robots-sold-in-2025-market-eyes-12m-by-2030). Densities: aluminium 7075 2.81 g/cm3; MJF PA12 1.01 g/cm3 (HP's datasheet). Torque screwdriver: Wiha TorqueVario-S 0.1 to 0.6 N m (https://uk.rs-online.com/web/p/torque-screwdrivers/2436301).

The Asimov 1 part files (plate 600_08A, housing 600_10C, bearing 600_11X, the thigh's URDF mesh) are Menlo Research's, under CERN-OHL-S-2.0; `knee_plate.npz` is derived from them.

Menlo Research, Asimov, Bahco, Wera and Wiha are named for identification only; the names are their owners' trademarks.

## Licence

MIT
