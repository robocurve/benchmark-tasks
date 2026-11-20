# Close a wall belt barrier: draw the belt, pass its end hand to hand against the spring, and lower it into the wall clip

Station, mall and airport staff close retractable belt barriers across escalators, aisles and queues daily. The belt of a wall unit is pulled out by its end, against the retractor, and the end hung on a clip on the facing wall: its fitting is lifted over the clip and lowered down into the clip's slot, whose lips hold it against the pull. If the belt slips from the hand, the retractor takes it back; an end pushed at the clip from the side, or held above it, does not go in. Working from one spot, as a fixed robot does, the belt changes hands mid-pull: the unit and the clip are 90 cm apart, each beyond the reach of the arm nearer the other. The webbing pulls 8.9 N throughout, so the second hand must take the belt just behind its 17 x 58 mm end before the first lets go, then lift the end 54 mm, square its 6 mm bead to an 8 mm channel and lower it 54 mm to the clip's foot. Unlike a loose cable passed between hands, the belt is under pull the whole way and its end must finish seated. Tensabarrier is a Tensator trademark, named for identification only.

**Instruction.** Close the belt barrier: pull the belt out, lift its end over the wall clip and lower it down into the clip's slot.

## Requirements

`yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source <ARENA_DIR>/yam.env`). `mechkit.py`, `demokit.py` and `gatekit.py` are in this folder.

## Run a trial

From this folder. One trial is at most 2,400 steps (2 minutes of simulated time at 20 Hz; the scripted demo ends at about 830 steps). Nothing is random: seed 1.

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 2400 --seed 1 \
  --instruction "Close the belt barrier: pull the belt out, lift its end over the wall clip and lower it down into the clip's slot."
```

## Scripted demo

```bash
python demokit.py task.py OUT --mode full                                     # full score, with a video
python demokit.py task.py OUT --no-record --mode none,right,rightfar,rightdiag,letgo,face,carry,hover,short,under,flatpinch,farhold,farback,rightback,palm,latestill   # the lazy routes below
```

The scripted demo is a reference, not a policy: it reads the end fitting's pose from the simulator to steer. The right hand comes straight down on the end, docked against the unit's mouth, and closes on its two faces above the bead, with about 40 N a finger; the demo prints the belt's 8.9 N against the pinch's friction (2 x 40 N x 0.6 = 48 N) and the pull's moment about the pinch (0.25 N m, against about 1.2 N m the pads hold). It draws the belt out 0.37 m to y = -6 cm. The left hand comes in level from the front, turned 15 degrees toward the unit, and closes across the belt's width on its two edges just behind the end, with 20 N a finger (2 x 20 N x 0.6 = 24 N against the 8.9 N pull); the right opens and lifts away while the left holds the end where it is. The left carries the belt, the end leading, at the belt's height to 3 cm short of the clip, steering by the end's own centre, and lifts it 54 mm, so the bead's bottom is 4 mm over the lips' top and its top 4 mm under the clip's head. Then it aims by feedback: each pass reads the end's heading, pitch and roll and the bead's offset from the channel's line, turns the hand about the end's centre to null the angles, and steers the bead onto the line, until it is within 0.6 mm and 0.6 degree. It brings the bead in through the window over the channel, lowers the end 54 mm at 2 cm/s, kept on the channel's line, until the bead stands on the clip's foot, lets go of the belt, and both hands go to rest over the middle of the table.

## The scene

Station: two YAM arms with parallel grippers at the near edge of the table. Wall: a 1220 x 610 mm (4 x 2 ft) sheet of 18 mm plywood standing on the table, its face along x = 45 cm. Tensabarrier 896-type wall unit: yellow case 100 mm deep, 115 mm long, 70 mm tall, 2 in (50 mm) belt, 13 ft, screwed to a 40 mm timber batten on the board; the belt leaves it through a slot in its mouth, and the end fitting, larger than the slot, stops against the mouth. Belt: woven polyester webbing 50 mm wide, 1.5 mm thick, 40 g/m. Belt end: a metal end fitting 17 mm long, 58 mm tall, 14 mm thick, 50 g; on its front face a neck 2.5 mm thick and 4.5 mm long and behind it a bead 6 mm across and 3.5 mm long, both 28 mm tall, centred on the end's height. Wall receiver: a Tensabarrier 898 wall clip, black metal, 112 mm tall, 16 mm wide along the belt and 11 mm deep, its two 4 mm screw holes 90 mm apart (the maker's drawing), screwed to a stand-off of three 2x4 offcuts (38 x 89 mm, 130 mm long) screwed face to face, 114 mm deep, so that its face sits on the belt's line. The clip's slot runs top to bottom, opening toward the unit: two lips 3 mm thick with a 4 mm gap between them, and behind them a channel 8 mm across and 5 mm deep, closed at the bottom by the clip's 11 mm foot and 50 mm tall; the lips' top inner edges are chamfered 1 mm. Above the channel the lips are cut away for 36 mm (the entry window), under a 15 mm head that carries the top screw. The bead goes in through the window, over the channel, and is lowered down it: the neck runs in the gap, the bead behind the lips, which it overlaps by 1 mm on each side, until the bead stands on the foot. There is no latch: the lips hold the bead against the belt's pull, and its weight keeps it down (metal on metal friction 0.25). The maker publishes the clip's outline and screw spacing; the slot's sizes and the end fitting are our estimates, read from the drawing and the photograph of the end, and are measured on the real parts (Building it). Layout in the line frame (cm, table top z = 0; x forward from the arm bases, y to the robot's left): arm bases at y = -30.5 and +30.5; the belt runs along the wall in the plane x = 33, its centre at z = 13; the unit's mouth is at y = -44 and the clip's face toward it at y = +46, 90 cm apart, 17 and 15 cm from the board's ends; the clip's foot is 10.5 cm above the table. The belt is a chain of 37 rigid links 25 mm long, each hinged to the next, in turn about the belt's width (it bends flat round what it meets) and about its thickness (edgewise, stiffer); it sags under its weight and cannot stretch. The end fitting is its last link, held by nothing but the belt, the hands, the unit's mouth and the clip. Inside the unit the belt runs back to the reel, which pulls it in with 8.9 N, constant: the pull of a 2 lb (8.9 N) constant-force spring. The maker publishes no retractor force; we estimate 5 to 12 N for a unit of this size. A lab checks the pull with a 1 kg spring scale on the end at 0.1, 0.5 and 0.86 m out, and uses the unit as sold if it reads 7 to 11 N (the range the demo was run at), or else fits that spring (8.9 N within 0.5 N). The retractor's brake is modelled as damping of the reel's travel, 7.2 N s/m, so a released end runs back at about 1.2 m/s. Both grippers start at the arms' ready pose. Start state, the same every trial: the belt fully in, the end against the unit's mouth.

### How the belt is modelled

The belt is one jointed body fixed to the unit. Inside the case a carriage on a slide along the belt's line stands for the reel: a constant 8.9 N pulls it in, as the constant-force spring does, and damping of 7.2 N s/m on its travel stands for the brake; the belt out is the carriage's travel. From the carriage the webbing runs out through the mouth's slot (8 mm across) as 37 rigid links, 25 x 50 x 1.5 mm and 1 g each (40 g/m), with colliders, so the belt meets the hands, fingers and the clip as webbing does. Each link is hinged to the next, in turn about the belt's width (flatwise bending, 0.08 N m/rad a hinge, from a bending stiffness of 0.004 N m2, our estimate 0.001 to 0.01 for 50 mm polyester webbing) and about its thickness (edgewise, 5 N m/rad, from 0.25 N m2, estimate 0.1 to 1); its first hinge, at the carriage, lets the belt tilt in the mouth. The hinges' damping is small (0.01 flatwise and 0.03 edgewise N m s/rad, estimates; at a fifth of these a seated end rattled 2 to 3 mm in the channel). The belt cannot stretch, and twist is not modelled (the belt stays on edge). Under the 8.9 N pull a run of 0.9 m sags about 2 mm, as a taut real belt does (wL^2/8T is 4.5 mm with no stiffness). The end fitting is the chain's last link, 17 x 58 x 14 mm, 50 g. The links' inertia is floored at 1e-6 kg m2 and each hinge carries 1e-5 kg m2 of armature, for the solver; until the scene has settled, before the pull acts, the edgewise hinges and the tilt at the carriage are held stiff, then given their own values. Links that have run back past the case's back are hidden. Wall clip: Tensabarrier 898 outline as published (112 x 16 x 11 mm); its slot (lips 4 mm apart, channel 8 mm) and the end's 6 mm bead are estimates from the maker's drawing and photo. No latch: the lips and gravity hold the end. The demo reads the end fitting's pose.

### Size table

| Dimension | Scene | Source |
|---|---|---|
| Belt width | 50 mm | 2 in webbing, Tensabarrier 896 page |
| Belt thickness, mass | 1.5 mm, 40 g/m (37 links of 25 mm, 1 g each) | estimate: polyester barrier webbing 1 to 2 mm, 30 to 50 g/m; weighed and measured on the real belt |
| Belt stiffness | flatwise 0.004 N m2, edgewise 0.25 N m2 | estimate (0.001 to 0.01 and 0.1 to 1); a lab reads the flatwise figure from the droop of a 20 cm cantilevered length (droop = w L^4 / 8 EI) |
| Belt length | scene 0.925 m, of which 0.88 m is drawn out at the clip; the real unit holds 13 ft (3.96 m; the page's metric figure is 3.65 m) | Tensabarrier 896 page |
| End fitting | 17 long x 58 tall x 14 thick mm, 50 g | the maker's photograph of the anti-tamper end (a sleeve about 17 mm long and 58 mm tall on the 50 mm belt); thickness and mass our estimates (mass 30 to 90 g); measured on the real end (Building it) |
| End fitting | neck 2.5 thick x 4.5 long, bead 6 across x 3.5 long, both 28 mm tall | estimate, read from the 898 drawing (the end must pass a slot a few mm wide in an 11 mm deep clip); measured on the real end |
| Case | 100 deep x 115 long x 70 tall mm | estimate (the 896 page gives 1.5 lb, no size) |
| Wall clip | Tensabarrier 898: 112 tall x 16 wide x 11 deep mm, two 4 mm screw holes 90 mm apart | Tensabarrier 898 page and its drawing |
| Clip slot | foot 11, channel 50 (8 across x 5 deep) behind lips 3 thick with a 4 mm gap, window 36, head 15; 1 mm chamfers on the lips' tops | estimate, scaled from the 898 drawing (its slot runs about 50 mm of the clip's length); measured on the real clip |
| Unit mouth to the clip's face | 900 mm | layout |
| Wall board | 1220 x 610 mm, 18 mm plywood | nominal 4 x 2 ft sheet; the panel listed below is 1213 x 603 x 17.8 mm, so the case's outer end sits 52 mm (scene 55 mm) inside the board's end |
| Batten, stand-off | batten 40 mm thick, 115 mm long behind the case; stand-off 114 x 89 x 130 mm behind the clip | layout; the batten a 2x2 furring strip (35 mm actual) packed out 5 mm; the stand-off three 2x4 offcuts (38 mm each) |
| Belt pull | 8.9 N, constant | 2 lb; real unit estimated 5 to 12 N; the spring listed below is 2.09 lb (9.3 N) nominal and is checked to 8.9 N within 0.5 N |
| Lift to enter, drop to seat | 54 mm up from the belt's line; 54 mm back down to the clip's foot | from the clip's sizes above: the bead clears the lips' top by 4 mm and the head by 4 mm |

`size_check()` in task.py prints the scene's values against these: belt 50.0 mm wide against 50.8 (2 in), 1.5 mm thick, 40 g/m, 0.925 m long in 25 mm links; end fitting 17.0 x 58.0 x 14.0 mm, 50 g; wall clip 112.0 x 16.0 x 11.0 mm against the 898 page; channel 50.0 mm; bead 28.0 mm tall; unit mouth to the clip's face 900.0 mm; stand-off 114.5 mm against 114 (three 2x4s); wall board 1220.0 mm wide; belt pull 8.9 N (2 lb). The 0.8 mm under 2 in on the belt's width is rounded to 50 mm.

## Building it for a real trial

`rig_layout.svg`, in this folder, is the dimensioned drawing (mm): 1, plan view with the board, unit, clip and stand-off, belt line, the end's start, hand-over and seated positions, and the arm bases; 2, front elevation with the heights; 3, the 898 clip seen from the robot, with the bead at its entry height and seated, and a plan section across its slot; 4, the end fitting. `python draw_rig.py` draws it from task.py's own constants, so its numbers are the scene's.

### Parts list

| Item | Product | Published size | Price | Source |
|---|---|---|---|---|
| Belt barrier | Tensator Tensabarrier 896 heavy-duty retractable wall-mounted barrier, 13 ft belt (sold with one 898 wall clip) | 2 in webbing, 7 ft 6 in or 13 ft; 1.5 lb; case size not published (scene 100 x 115 x 70 mm, estimate) | 99 USD | https://www.tensabarrier.com/us/product/tensabarrier-heavy-duty-retractable-wall-mounted-barrier/ |
| Wall clip | Tensabarrier 898 wall receiver belt clip, black metal (one comes with the 896; a spare listed) | 112 x 16 x 11 mm, 0.4 lb, two 4 mm holes 90 mm apart | 17 USD | https://www.tensabarrier.com/us/product/tensabarrier-wall-receiver-belt-clip |
| Stand-off | 2 in x 4 in x 8 ft stud (1.5 x 3.5 in actual, 38 x 89 mm), three 130 mm offcuts | 38 x 89 mm | estimate 5 USD | any timber yard |
| Wall board | Handprint 23/32 in x 2 ft x 4 ft sanded plywood project panel (Home Depot 202093836) | 0.7 x 23.75 x 47.75 in actual (17.8 x 603 x 1213 mm) | 23.76 USD | https://www.homedepot.com/p/Handprint-23-32-in-x-2-ft-x-4-ft-Sanded-Plywood-Project-Panel-300969/202093836 |
| Battens | 2 in x 2 in x 8 ft furring strip board (Home Depot 202518911) | 1.375 x 1.37 in actual (35 mm) | not read; estimate 4 USD | https://www.homedepot.com/p/202518911 |
| Board feet | Simpson Strong-Tie A23 galvanized angle, 2 | 2 x 1-1/2 x 2-3/4 in, 18 gauge | 0.98 USD each | https://www.homedepot.com/p/Simpson-Strong-Tie-2-in-x-1-1-2-in-x-2-3-4-in-Galvanized-Angle-A23/100374944 |
| Clamps | IRWIN QUICK-GRIP 6 in medium-duty one-handed bar clamp, 2 | 6 in opening | 22.99 USD each | https://hemlockhardware.com/products/396109-irwin-quick-grip-6-in-medium-duty-one-hand-bar-clamp |
| Force gauge | Pesola 41000-KIT Medio-Line spring scale with pressure set (pulls and pushes) | 1000 g, 10 g (0.1 N) divisions, within 0.3% | 107.36 USD | https://www.globaltestsupply.com/product/pesola-41000-kit-medio-line-force-gauge-kit-with-pressure-set |
| Caliper | Mitutoyo 500-196-30 digital caliper | 0 to 150 mm, 0.01 mm | 202 USD | https://www.hisco.com/Product/500-196-30-65737 |
| Tape | 3M ScotchBlue 2090 painter's tape | 1.41 in x 60 yd | 6.98 USD | https://homedepot.com/p/3M-ScotchBlue-1-41-in-x-60-yds-Original-Multi-Surface-Painter-s-Tape-2090-36AP/202077779 |
| Hardware | wood screws (#8, 1-1/4 and 2 in), one M6 x 40 mm bolt and nut | | estimate 10 USD | |
| Spring, only if the unit fails the pull check | McMaster-Carr 9293K21 constant-force spring, 301 stainless | 2.09 lb (9.3 N), 45 in extended, 0.75 x 0.012 in, 1.6 in ID, 25,000 cycles | not on the page; estimate 20 USD | https://www.mcmaster.com/constant-force-springs |
| Spring drum, with the spring | Charlotte Pipe 1-1/2 in x 2 ft PVC DWV Schedule 40 pipe (PVC071120200HA), a 25 mm slice | 1.900 in (48.3 mm) OD | 5.98 USD | https://www.homedepot.com/p/Charlotte-Pipe-1-1-2-in-x-2-ft-PVC-DWV-Schedule-40-Pipe-PVC071120200HA/100581718 |

### Steps

- **Board.** Screw the two angles to the board's foot and clamp them to the table, the board's face 450 mm in front of the arm bases and centred on them (view 1).
- **Unit.** Screw a 115 mm batten to the board and the 896 to it, the mouth's centre 440 mm to the robot's right of centre (170 mm from the board's end) and 130 mm above the table (view 2). Pack the batten out until the belt's centre runs 120 mm in front of the board: 40 mm for the scene's 100 mm case; the 896's depth is not published, so measure it.
- **End.** Measure the 896's end and the 898's slot with the caliper (scene: end fitting 17 x 58 x 14 mm; neck 2.5 mm, bead 6 mm across, 28 mm tall; lips 4 mm apart, channel 8 mm across and 50 mm tall, window 36 mm; all estimates). Mark the board level with the end's top edge when it stands seated on the clip's foot.
- **Clip.** Screw the three 2x4 offcuts face to face (114 mm), screw the stand-off to the board, its face toward the unit 460 mm to the robot's left of centre (150 mm from the board's end), and screw the 898 to its front, its slot toward the unit, its foot 105 mm above the table, so that a seated end's centre is on the belt's line, 130 mm up (views 1 to 3). Pack the stand-off out with a washer if the clip's face sits short of the belt's line.
- **Belt pull.** Hook the gauge on the end at 0.1, 0.5 and 0.86 m out. The demo scores 1.0 with the pull at 7, 8.9 and 11 N, so a unit that reads 7 to 11 N at all three points is used as sold, its own twin-shoe brake standing for the scene's damping. Otherwise take out the cassette's spool, mount the constant-force spring on the PVC drum on the M6 bolt inside the case (measure the case first: the coil on its drum is about 53 mm across), rivet the webbing to its free end, and check 8.9 N within 0.5 N at the three points. A released end then runs back faster than the scene's 1.2 m/s: no damper we found gives 7 N s/m at that speed (estimate).
- **Tape.** Tape on the board every 10 cm from the unit's mouth, to read the belt out from the video.

## Scoring

Terms: seated means the end's bead is down the wall clip's channel, behind its lips, both its top and bottom within the channel, and its bottom within 3 mm of the clip's foot, so it was lowered at least 47 of the channel's 50 mm (real trial: the end's top edge at the mark on the board, by video). Belt out is the length of belt drawn out of the unit's mouth (real trial: tape on the wall every 10 cm). A hand holds when its jaws are shut on the end or on the belt within 20 cm of it. Straight means the belt drawn out has no fold or snag: nowhere is it bent flat more than 30 degrees (a taut run bends under 5; real trial: video). Clear means no fingertip, gripper body or palm of either hand within 5 cm of the end's centre lines (along its length and its height) or of the last 20 cm of belt (its middle line and both edges), and no hand shut on the belt anywhere along it; still means the end's centre moves under 1 mm in that second (real trial: video). The tiers are read at the last step, the 1.0 over the last 1 s; a run-back of the belt on the way costs nothing by itself. Four steps, one tier each. 0.2: the belt drawn out, belt out at least 0.5 m with a hand holding. 0.4: handed over, that, with the left hand the one holding. 0.6: seated in the clip. 1.0: left to the clip, seated and clear and still for the last 1 s, held by the clip and its own weight alone, belt out at least 0.5 m. One failure voids the run: the belt folded or snagged at the end (not straight) scores 0. 0 otherwise. Possible scores: 0, 0.2, 0.4, 0.6, 1.0.

The same rule is applied in the real trial and in score(): the end state alone decides the tier.

## Results

The scripted demo scores 1.0 (seated, belt out 0.88 m, 834 steps) on GPU physics in the run of `python demokit.py task.py OUT --mode full`, with a film and a still. The belt holds together throughout (hinges bent under 8 degrees). Belt damping is 0.01 and 0.03 N m s/rad: at a fifth of that the seated end rattled 2 to 3 mm in the channel and was never still for 1 s. At the hand-over the right lets go with the belt out 0.37 m and the left holds it by its edges 21 mm behind the end; at the clip the end is aimed by feedback, and the bead is lowered to the foot. After the release both hands go to rest over the middle of the table. The scorer bounds the bead's depth: its bottom at most 2 mm under the clip's foot and its top under the clip's head. The lazy routes below are run on the same scene on GPU physics.

## Failure table

Each lazy route, run on GPU physics (`--no-record --mode none,right,rightfar,rightdiag,letgo,face,carry,hover,short,under,flatpinch,farhold,farback,rightback,palm,latestill`), must score no more than its cap. `letgo` and `face` are also the undo checks: each reaches a partial state (belt out with a hand on it) and then gives it up, and the score is taken from the end, so neither keeps a partial credit it held on the way.

| Route | Mode | Cap | Result |
|---|---|---|---|
| Do nothing | `none` | 0 | 0 |
| Right hand only: draws the belt toward the clip as far as it reaches (the end to y = 0) and holds it | `right` | 0.2 (drawn out only if 0.5 m is out; the left never holds) | 0 (0.43 m out) |
| The right hand alone draws the end as far as it reaches along the wall (to y = 0.31) and holds it | `rightfar` | 0.2 (drawn out, the left never holds) | 0 (the hand stops 76 mm short; the end is held only by the belt) |
| The right hand alone draws the end diagonally to (0.10, 0.05) at the belt's height, past the case's corner, belt out about 0.54 m, and holds it | `rightdiag` | 0.2 (drawn out, the left never holds) | 0 |
| The left lets go of the belt 3 cm short of the clip | `letgo` | 0 (the belt runs back) | 0 |
| The end carried at the belt's height straight into the clip's face, as if it clicked in from the side, then let go | `face` | 0 (the lips stop the bead; the belt runs back) | 0 |
| The left takes the end from the right, carries it 0.15 m on and holds it | `carry` | 0.4 (drawn out and handed over, not seated) | 0.4 |
| The end lifted into the window over the channel and held there, not lowered | `hover` | 0.4 (handed over, not seated) | 0.4 |
| Lowered 30 mm down the channel and held there, 20 mm short of the foot | `short` | 0.4 (handed over, not seated) | 0.4 |
| The end carried 40 mm under the belt's line to under the clip, bead behind the lips' plane, and held there | `under` | 0.4 (handed over, not seated: the bead is below the clip's foot) | 0.4 |
| The right hand pinches the belt flat (jaws on its two faces) 0.10 m behind the end and draws it 0.5 m out; the end is never in the clip | `flatpinch` | 0.2 (drawn out, the left does not hold) | 0 |
| The end seated, then the left shuts on the belt 0.12 m behind it | `farhold` | 0.6 (seated, a hand on the belt) | 0.4 (the hand's grip lifted the end off the foot; under the cap) |
| The end seated, then the left shuts on the belt 0.30 m behind it and holds the pull | `farback` | 0.6 (a hand on the belt anywhere is not 1.0) | 0.6 |
| The end seated and held by the left; the right pinches the belt flat 0.65 m behind it, then the left lets go and withdraws | `rightback` | 0.6 (a hand on the belt anywhere is not 1.0) | 0.6 |
| The end seated, the left lets go and its hand is turned about so that its body and palm rest 2 cm from the end, the fingertips 15 cm away | `palm` | 0.6 (a gripper body within 5 cm is not clear) | 0.6 |
| The full motion, the run cut about 1.1 s after the hands are clear | `latestill` | 1.0 (still for 1 s is enough) | 1.0 |

## Sources

Tensator, Tensabarrier 896 heavy-duty wall-mounted barrier: 2 in woven polyester belt, 7 ft 6 in or 13 ft, metal case, sold with one wall clip (https://www.tensator.com/shop/us/product/tensabarrier-heavy-duty-retractable-wall-mounted-barrier/). Tensator, Tensabarrier wall receiver belt clip, ref. 898: metal, 112 x 16 x 11 mm, 0.4 lb, fits the standard anti-tamper webbing end, 17 USD; its drawing gives the outline and two 4 mm holes 90 mm apart, and shows the slot running down the clip, where the end goes in from above and is lowered (https://www.tensabarrier.com/us/product/tensabarrier-wall-receiver-belt-clip). The maker publishes no latch for it; the drawing shows a small block with a screw over the slot, which it does not describe and which is not modelled. Tensator, Tensa Midi wall unit sheet: braking cassette, 50 mm webbing (https://d4sr6trxn2jcb.cloudfront.net/Tensa_Midi_Wall_Mounted_Belt_Barrier_Technical_Specification_XL_Displays.pdf). Tensator, Tensabarrier heavy-duty wall unit (EU page): the cassette's twin-shoe brake slows the belt's retraction on release (https://www.tensator.com/shop/eu/product/tensabarrier-heavy-duty-retractable-wall-mounted-barrier/). Lavi Industries Beltrac install sheet 87-065: pull the belt by its end across the aisle and fix the end. The maker publishes neither the retractor force nor the slot's inner sizes; both are estimates, stated above with how a lab checks them. All meshes are built in code; nothing is downloaded. Tensabarrier is a Tensator trademark, named for identification only.

## Licence

MIT
