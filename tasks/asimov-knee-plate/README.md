# Turn a loose Asimov humanoid leg over on the table, then seat its knee side plate flush in the pocket

Robotics technicians assemble humanoids from their makers' guides; Menlo's open-source Asimov 1 is built this way, in a market the IFR counted at about 7,000 humanoids sold in 2025 and Omdia at about 13,000 shipped. Left Leg Step 8: turn the leg so the thigh's bearing faces up, fit side plate 600_08A on housing 600_10C "such that the protrusion fits into the bearing and then all the holes ... align with the threaded insert", then lightly tighten nine M3x10 flat bolts (DIN 7991) into the nine inserts to align it. A cocked plate puts its holes off the inserts, and the bolts won't start. The leg, 3.03 kg (twice the arm's 1.5 kg payload), lies loose on its side and is tipped onto its back, not lifted. Then three fits meet on one axis within 5 mm: plate in pocket 0.3 mm a side, 34.8 mm pad in a 35.0 mm bore, 11.85 mm boss in a 12.0 mm bearing. The review's request did not say whether the fasteners were to be driven, so we added them to match the real-world context: the plate's nine M3x10 bolts stand in a holder and a small hex screwdriver (a Wiha SoftFinish 356, 2.0 x 60 mm) lies beside it, both free bodies at true size with colliders, laid out for the next step. This task is turning a non-cylindrical humanoid leg over and seating its knee plate on its locating features, ending with all nine holes on their inserts; driving the screws is a separate, well-covered act, outside this task's scope and not scored here.

**Instruction.** Turn the leg onto its back, then seat the knee side plate flush in its pocket with its nine bolt holes over their inserts, and move both hands clear.

## Requirements

`yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source <ARENA_DIR>/yam.env`). `mechkit.py`, `demokit.py`, `gatekit.py` and `knee_plate.npz` (the maker's geometry as meshes and colliders) are in this folder.

## Run a trial

From this folder:

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 2400 --seed 1 \
  --instruction "Turn the leg onto its back, then seat the knee side plate flush in its pocket with its nine bolt holes over their inserts, and move both hands clear."
```

## Scripted demo

```bash
python demokit.py task.py OUT --mode full   # full score, with a video
python demokit.py task.py OUT --no-record --mode undo,turn,none   # the lazy routes and the undo, on GPU
python check_score.py                       # the scorer on hand-built states, no simulator
```

The demo reads the leg's and the plate's poses to plan its grasps and its moves; it reads no contact force and moves no object except through the grippers. Both hands grip the leg's ends from above (the calf's end and the thigh's top end, across their thickness) and swing together about the edge the leg rests on, toward the robot, to 66 of its 86 degrees: past its balance point at 38, as far as the wrists follow. They let go and it settles onto its back. Then the right hand pinches the plate across its neck, lifts it out of the post, squares it to the face as the leg lies, lets go 0.75 mm above the face with its axis on the bearing's, lets go only once the plate reads on the bearing's axis within 0.12 mm, at its height within 0.5 mm and square within 0.3 degrees (levelling and correcting again until then), and presses it home, until it reads flush and at most six times, where it stands highest if it has not slid home (and over the boss if a press leaves it where it was), and both hands clear. `undo` does all that, holds the 1.0 for 1 s with both hands clear, then brings the right hand back and rests its fingertips on the seated plate to the end: the score is read from the state at the end, so it must fall to 0.75. `check_score.py` feeds hand-built runs through the scorer itself: each rung, the leg's poses (on its side, half turned, held up, face down, on its end, off the table, sliding or rocking under the plate), the zeros, the hands and stillness gates, and the lazy routes.

## The scene and what is simplified

Station: two YAM arms with parallel grippers, bases 61 cm apart at the near edge of the table. Positions in cm from the midpoint between the arm bases: x forward, y to the robot's left; heights above the table top. All parts are from Menlo's Asimov 1 repository (github.com/menloresearch/asimov-1, CERN-OHL-S-2.0): shapes from the maker's STEP files (the thigh from the maker's URDF mesh). Leg, as after the guide's instruction 3, off the body and loose on the table, nothing holding it: the thigh assembly (URDF left_hip_yaw_link: 1.949 kg, with its centre of mass and inertia) slotted into the knee housing 600_10C (MJF PA12, 1,069.5 cm3 at 1.01 g/cm3: 1.080 kg; the URDF's 2.48 kg knee link includes the calf's later parts), 3.03 kg in all and 54 cm long. Its knee joint is free: the housing turns on the thigh about the bearing's axis, from straight to 1.50 rad bent (URDF), against the unpowered knee actuator's published friction, 0.70 N m to start it and 0.02 N m once it moves, with 0.034 kg m2 reflected inertia (Menlo's actuator table, EC-A4315-P2-36, docs.menlo.ai/asimov/1/reference/joint-design-and-actuation). It starts straight, lying on its side along y with the shank toward -y, the housing's outer face (the plate side) upright and facing away from the robot (+x), the pocket's centre at (46, -7), 6.3 cm up. On its back, the task's turned pose, it rests on three points with that face up, about 4 degrees off level, the pocket's centre at about (31, -6), 10.3 cm up: from its side it tips 86 degrees toward the robot over its lower edge, its centre of mass rising from 6.2 to 7.9 cm (over the edge at 38 degrees) and falling to 5.3. In the outer face, the pocket for the plate: 5.0 mm deep, the plate's own outline with 0.3 mm clearance (80.0 mm across its round end, 40.0 mm across its neck, its narrow end toward the shank), with a 35.0 mm bore at its centre and nine M3 threaded inserts. Its walls stand full depth round the round end and the neck's root and fall away along the neck, where the housing's surface curves down. The thigh's flanged bearing 600_11X (12.0 mm bore, 5.0 mm wide) is on the pocket's axis, its face 11 mm below the housing's face. Plate 600_08A: aluminium 7075, a 5.0 mm sheet 149.4 mm long, 79.4 mm across its round end and 39.4 mm across its neck, 1 mm chamfers on its edges; on one face a 34.8 mm pad (5 mm), a 15 mm step (1 mm) and an 11.85 mm boss (5 mm, 0.9 mm tip chamfer); nine 3.2 mm countersunk holes; 120 g (42.6 cm3 at 2.81 g/cm3). It starts boss down in the socket (36 mm across, 12 mm deep) of an acetal kitting post 50 mm across and 30 mm tall at (20, -17), its sheet on the post's rim, its narrow end toward -y. Friction 0.3 (PA12 and aluminium on the table, on acetal and on each other). Fasteners (Step 8, instruction 5), laid out for the next step and not used in this one: nine M3x10 flat bolts, DIN 7991 (3.0 mm thread, head 6.0 mm across and 1.7 mm tall, 2 mm hex socket), stand head up in a printed holder 40 x 40 x 8 mm at (17, 3), and a hex screwdriver (Wiha SoftFinish 356, 2.0 x 60 mm: 164 mm long, 23 mm handle, 60 mm blade, 25 g) lies on the table, its middle at (12, -6), heading 75 degrees from x, both clear of the leg's and the plate's paths and of both arms at their ready pose. Nine brass M3 heat-set inserts, 4.4 mm across and 4.5 mm long with the thread's 2.5 mm minor bore, sit in the housing's nine insert bores (the STEP: 4.4 mm across, 4.5 mm deep under a 0.5 mm lead-in, 5.5 to 10 mm under the face), under the plate's nine holes. Seated, each bolt's head fills its hole's countersink (6.72 mm across at 90 degrees, the STEP) and its last 5 mm of thread reaches into the insert. Both arms start at their ready pose. Low-cost stand-in for a real trial (costly: the 7075 plate and the actuated leg; nothing here needs the real robot): print the thigh and the knee housing 600_10C from the published STEP files (PA12 if you have a service, PLA otherwise; solid prints are heavier than MJF PA12 at 1.01 g/cm3, PLA being about 1.24 g/cm3, so weigh them). Ballast: the leg must weigh 3.03 kg with its centre of mass where the URDF puts it. Weigh the printed pair, then add the missing mass as steel or lead-free shot or bar glued into two pockets, one in the thigh and one in the housing, sized so the sum of mass times distance matches the URDF thigh (1.949 kg) and housing (1.080 kg) centres. Check by balancing the leg on a knife edge and by the tip test: tipped over its lower edge it passes its balance point at about 38 degrees. Knee: keep the thigh's bearing seat to the STEP size (12.0 mm bore, 23.0 mm flange, 5.0 mm wide) and fit the real flanged bearing 600_11X or a plain 12 mm bearing in it, with a 12 mm steel pin as the hinge, 0 to 1.50 rad. Set the friction by tightening a friction screw or strap on the pin until a spring scale at 0.30 m from the axis reads 2.3 N to start it (0.70 N m) and read what it moves at (the target is 0.02 N m, 0.067 N at 0.30 m; the moving value is the hard one: report what you measure). Side plate: cut 5 mm aluminium sheet to the STEP outline (laser or waterjet from the STEP's drawing), with the 34.8 mm pad and 11.85 mm boss as a turned part fixed to it, or print the one-piece plate in PA12 or PLA with the same outline (a printed plate is lighter, about 53 g solid in PLA against 120 g). Print the pad, the boss and the pocket's outline undersize or oversize as the fit needs, then sand, ream or turn to the STEP sizes and check with the real bearing, a caliper and gauge pins: the fits are 0.3, 0.1 and 0.075 mm a side. Keep the pocket 5.0 mm deep, the 35.0 mm bore and the nine M3 threaded inserts (brass heat-set) at the STEP positions. Plate holder: a printed block (PETG or PLA) 50 mm across and 30 mm tall with the 36 mm, 12 mm deep socket, in place of the acetal post.  Grips and protection: the demo grips at 38 N a finger (the YAM's limit is about 40 N) and presses about 10 N. Stick 1 mm silicone or rubber pad covers on the fingertips, so the grippers do not mark the plate's 1 mm-chamfered aluminium edges, and put a soft mat under the leg. No cost is given here: no published price was found to cite.

- Fasteners: the nine bolts in their holder (one body) and a small hex screwdriver (Wiha SoftFinish 356, 2.0 x 60 mm) lying on the table are free bodies with colliders at true size, laid out for the next step. Nothing in the act or the score uses them, and the demo never touches them (a run prints where they end). Driving the screws is outside this task's scope (see the first paragraph). The inserts are visual parts of the housing, inside its bores, below where the plate reaches.
- Physics: Rigid bodies at 240 Hz. Plate (45 convex colliders), pocket (217) and bearing (48) are the maker's STEP geometry; no collider widened: rim 0.30 to 0.34 mm a side, pad 0.10 to 0.12, boss 0.075 to 0.09. The leg's outside is coarse convex slabs of its meshes. Friction 0.3. The knee's dry friction (0.70 start, 0.02 moving N m) is a coded drive; inertia 0.034 kg m2. The thigh sits in the housing as in the CAD. The arm presses through soft joints, about 10 N.
- The leg on its back, pressed through the plate (gate runs): 10 N and 20 N over the boss for 2 s move it 0.00 mm; 10 N at the plate's edge 35 mm off its axis on the far side moves it 0.00 mm, and on the near side rocks it about 2 degrees onto its other three points (the face 4 mm over), until the press stops. Left alone on its side for 5 s it does not move.

## Sizes

Every size a hand, the plate or the scorer meets, and the fasteners, against their source. `size_check()` in task.py measures the scene's meshes and prints these at the start of each demo run; its output is below the table.

| Part, dimension | In the scene | Source |
|---|---|---|
| Plate 600_08A: sheet, length, round end, neck | 5.0, 149.4, 79.4, 39.4 mm | 600_08A STEP |
| Plate: pad, boss | 34.8 mm, 11.85 mm | 600_08A STEP |
| Plate: nine holes, countersink at the top | 3.2 mm, 6.72 mm at 90 degrees | 600_08A STEP |
| Plate mass | 120 g | STEP volume 42.6 cm3, 7075 at 2.81 g/cm3 |
| Pocket in 600_10C: depth, clearance a side | 5.0 mm, 0.3 mm | 600_10C STEP |
| Bore, bearing 600_11X bore | 35.0 mm, 12.0 mm | 600_10C, 600_11X STEP |
| Insert bores in 600_10C: count, size | 9 (of 22 M3), 4.4 mm across, 4.5 mm deep | Menlo heat-insert drawing for 600_10C; 600_10C STEP |
| Inserts: outside, minor bore, length | 4.4, 2.5, 4.5 mm | the bore (STEP); M3 minor diameter; insert make not published: estimate |
| Bolts: count, type | 9, M3x10 flat, DIN 7991 | Step 8, instruction 5 |
| Bolt: length overall, thread, head, head height, hex | 10.0, 3.0, 6.0, 1.7, 2.0 mm | Step 8; DIN 7991 M3 |
| Bolt holder | 40 x 40 x 8 mm, printed | estimate (any screw tray) |
| Hex screwdriver: length, blade, handle, mass | 164 mm, 60 mm, 23 mm, 25 g | Wiha SoftFinish 356, 2.0 x 60 mm (item 32265), catalogue; the handle's profile is an estimate |
| Hex bit of the driver and the bolts' socket | 2.0 mm | Wiha 32265; DIN 7991 M3 |
| Leg: mass, length | 3.03 kg, 54 cm | URDF thigh 1.949 kg; 600_10C 1.080 kg (STEP volume, MJF PA12 1.01 g/cm3) |
| Knee friction: start, moving; inertia | 0.70, 0.02 N m; 0.034 kg m2 | Menlo actuator table, EC-A4315-P2-36 |

```
[size] plate 600_08A thickness: scene 5.01 mm, published 5.0 mm (600_08A STEP)
[size] plate length: scene 149.40 mm, published 149.4 mm (600_08A STEP)
[size] plate hole: scene 3.21 mm, published 3.2 mm (600_08A STEP)
[size] plate countersink, top: scene 6.73 mm, published 6.72 mm (600_08A STEP)
[size] insert bore in housing 600_10C: scene 4.41 mm, published 4.4 mm (600_10C STEP)
[size] insert, outside / bore / length: scene 4.4 / 2.5 / 4.5 mm, published 4.4 / 2.5 / 4.5 mm (600_10C STEP bore; M3 minor diameter)
[size] holes and inserts (count): scene 9, published 9 (Menlo heat-insert drawing, 600_10C, inserts 1 to 9)
[size] bolt length overall: scene 10.00 mm, published 10.0 mm (Step 8: M3x10 Flat bolt, DIN 7991)
[size] bolt thread / head / head height: scene 3.0 / 6.0 / 1.7 mm, published 3.0 / 6.0 / 1.7 mm (DIN 7991 M3)
[size] bolt hex socket: scene 2.00 mm, published 2.0 mm (DIN 7991 M3)
[size] bolts (count): scene 9, published 9 (Step 8, instruction 5)
[size] screwdriver: length / blade / handle: scene 164 / 60 / 23 mm, published 164 / 60 / 23 mm (Wiha SoftFinish 356, 2.0 x 60 (32265))
[size] screwdriver mass: scene 25 g, published 25 g (Wiha 32265)
```

## Scoring

Measured at the end with an inclinometer and a ruler on the housing's face, a depth gauge on the plate's top against the face at ten points round its outline (round end, fillets, neck, narrow end), a 2.4 mm pin gauge and a tape. Score 0 if the plate or the leg falls off the table. Rungs are cumulative: 0.25 with the leg on its back on the table (the face within 10 degrees of level, at its resting height within 12 mm: lying there, not held up); 0.5 with that and the plate in the pocket's mouth, its boss's axis within 1.5 mm of the bearing's and no point of its top more than 4.0 mm above the face (the rim, pad and boss each at least 1 mm in); 0.75 with the plate seated flat, every point within 0.5 mm of flush and level with the face (tilt under 1 degree, boss down); 1.0 with that, every bolt hole on its insert, ready for its M3x10 bolt: the pin gauge passes through each of the nine 3.2 mm holes into its insert's thread (axis within 0.45 mm: 0.4 through the hole, 0.05 into the 2.5 mm minor diameter), no gripper within 10 cm of the plate and the face round it or within 8 cm of the leg's centre line (thigh's end, knee, calf's end), and the plate and leg still (plate under 1 mm in 1 s; leg under 2 mm/s and 0.6 deg/s), held for 1 s and still true at the end. A plate set in the pocket before the leg was on its back, pressed home with the leg moved 15 mm, or its face turned 3 degrees, from how it lay when the plate first reached the mouth, or never carried by a gripper (10 steps in a row 1 cm up with a fingertip within 3 cm), scores at most 0.5. Driving the bolts is not scored. No hold is needed during the press: the leg stays put on its own, so no hand should hold it. Real trial: the same gauges, tape marks round the leg, and a photo, with the bolt holder and the screwdriver set aside.

## Results

Scripted runs on GPU physics (RTX 4070 Ti SUPER lab PCs unless named).

| Run | What it does | Score |
|---|---|---|
| Dream-1 | Run at 320 x 176, 2,400 steps | 0.0: the leg stayed untouched (face 89.9 degrees from up, 124 mm from the pocket axis at the end); 1 trial, hit the 2,400-step limit |
| `full`, `python demokit.py task.py OUT --mode full` (no video), RTX 3090 Ti lab PC | The whole task | 1.0 in 1,319 steps (66 s); the bolt holder and the screwdriver did not move (0.00 mm) |
| `full`, filmed at 640 x 360, RTX 5090 | The whole task | 1.0 in 1,275 steps (64 s); 0 blank frames; seated flush, the props untouched |
| `none` | The arms wait | 0.0 |
| `turn` | The leg turned onto its back, the plate left on its post | 0.25 |
| `undo` | Seated and held at 1.0, then a hand rests on the plate to the end | 0.75: the top rung is read from the state at the end |

Scorer checks on hand-built states (`python check_score.py`, no simulator; the ones below are the routes a reviewer would try first; all 54 cases pass):

| Route | Score |
|---|---|
| Plate seated first, then the leg turned (the order reversed) | 0.5 |
| Plate cocked on the mouth, then pressed home with the leg moved 15 mm or turned 3 degrees | 0.5 |
| Plate never carried by a gripper (thrown, flicked, prodded from 12 cm away) | 0.5 |
| Plate seated, a hand or a fingertip within 10 cm of the plate or on the leg's ends or middle | 0.75 |
| Plate seated, then pulled out 3 cm or knocked 2 mm proud | 0.25 / 0.5 |
| Leg half turned, held up 3 cm, face down, on its end, or off the table | 0.0 |
| Bolt holder or screwdriver on the plate at the end | not scored: set them aside before measuring |

## Runtime and seeds

The demo takes 1,275 to 1,319 steps (64 to 66 s at 20 Hz): the turn, then the plate. The 2,400-step limit leaves room for a slower policy. Nothing is random (`object_jitter=0`). Re-timed at 320 x 176, the policy's resolution, on a lab PC: 1.8 steps a second, the slowest 30 steps 34.7 s (1.16 s a step), so a policy that loses its session after about 8 s idle should be sent at most 3 rows a request.

## Sources

Menlo Asimov 1 assembly guide, Left Leg Step 8, instructions 3 to 5 (read 11-16): https://docs.menlo.ai/asimov/1/build/assembly-steps/left-leg-module/knee-and-calf-assembly/step-8. Menlo Asimov 1 joint design and actuation, the full actuator table (left_knee_joint, EC-A4315-P2-36: range, armature, static and dynamic friction; read 11-16): https://docs.menlo.ai/asimov/1/reference/joint-design-and-actuation. Part STEP files and URDF, Menlo Research, CERN-OHL-S-2.0: https://github.com/menloresearch/asimov-1 (mechanical/ASV1/600/FABRICATION/ALU_7075/ASV1_600_08A.STEP, MJF_PA12/ASV1_600_10C.STEP, OFF_THE_SHELF/ASV1_600_11X.STEP; sim-model/asimov_1_25dof/urdf/asimov_1_25dof.urdf). Step 8, instruction 5 (the nine M3x10 Flat bolts, DIN 7991, Menlo's part id m3-x-x-10mm-din-7991, lightly tightened to align the plate, then each threadlocked in a star order), same page. Bolt measure (a flat bolt is measured overall, head included): https://docs.menlo.ai/asimov/1/build/assembly-steps. Left leg parts list (M3x10 Flat bolt, 18 in the leg): https://docs.menlo.ai/asimov/1/build/assembly-preparations/part-organisation/left-leg. Heat-insert drawing for 600_10C (22 M3 inserts; 1 to 9 round the plate's pocket): mechanical/ASV1/Docs/Heat_Insert_parts/Heat_inserts_ASV1_600.pdf in the repository below. Tools (metric Allen keys, 2 mm among them): https://docs.menlo.ai/asimov/1/overview/tools. DIN 7991 M3 (head 6.0 mm, 1.7 mm, 2 mm hex): the standard's table. Humanoid counts: IFR, about 7,000 humanoids sold worldwide in 2025 (as reported at https://aiweekly.co/alerts/ifr-counts-just-7000-humanoid-robots-sold-worldwide-in-2025); Omdia, about 13,000 shipped (as reported at https://www.analyticsinsight.net/robotics/7000-humanoid-robots-sold-in-2025-market-eyes-12m-by-2030, secondary reports). Densities: aluminium 7075 2.81 g/cm3; MJF PA12 1.01 g/cm3 (HP's datasheet for PA12 parts).

The Asimov 1 part files (plate 600_08A, housing 600_10C, bearing 600_11X, the thigh's URDF mesh) are Menlo Research's, under CERN-OHL-S-2.0; `knee_plate.npz` is derived from them.

Menlo Research and Asimov are named for identification only; the names are their owner's trademarks.

## Licence

MIT
