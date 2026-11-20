# Take a used commode pail out from under its seat: draw it forward off its brackets, keep it level, set it down

A used bedside-commode pail hangs by its rim flange on two pail brackets, 15 mm under the seat, so it cannot be lifted out. Drive's drop-arm commode manual puts the pail in with "Slide pail in from front of commode", so it comes out the same way. Its handle is the real pail's fold-down bail, free on two pivots at the pail's ends. One gripper swings the bail up level and draws the pail forward by it. The frame's front tube keeps the bail from standing up over its pivots until the pail is 0.18 m out, but the pail's centre passes the brackets' ends at 0.12 m: from there, held by a level bail, the 1.3 kg hangs 15 cm from the grip and the pail pitches forward. So the other gripper takes the rim's front strip first and holds the front up. Both draw it clear, bring it down level, set it on the floor in front of the commode (on a disposable underpad in a real trial, where it is then covered), and let go. The scorer reads the pail's tilt and landing speed: over 15 degrees, or a landing over 0.3 m/s, is a splash.

## Run

From this folder:

```
python demokit.py task.py OUT                      # final film, 640 x 360, GPU to yourself
python demokit.py task.py OUT --no-record --mode full,none,one_hand,one_hand_rim,drag,leave,drop,undo,hold_on,drop_low15,drop_low19,drag2,tip_release,tip_gentle,slow_late
```

The harness's instruction (Dream-1 run): `--instruction "Take the used pail out from under the commode seat and set it down on the floor in front of the commode without spilling it."`

`CP_CX`, `CP_CY` (default 0.70, 0.0 m) set where the commode stands.

**Result** (scripted, GPU physics, 20 Hz): the full mode scores 1.0 on an RTX 4070 Ti SUPER (699 steps, 34.95 s, with the final scorer, against a limit of 1,200; 0 blank frames; peak tilt 9 degrees), and 1.0 at 320 x 176 on RTX 4070 Ti SUPER hosts (699 steps). Every route in the table below ran on this scene on GPU.

**Dream-1 run** (the harness's own policy, 1,200 steps, one trial): 0. It hit the step limit without taking the pail out. The film is `model_run.mp4`.

## Scoring, and the routes that score less

| Mode | What it does | Score (GPU) |
|---|---|---|
| full | right goes in under the folded bail's grip, swings the bail up level and pinches it, draws the pail 90 mm; left pinches the rim's front strip; both draw it 0.31 m clear between the front legs, level it by the pail's own pitch, set it down, let go | 1.0 (699 steps, 34.95 s) |
| one_hand | right alone, by the bail held level, draws it until the pivots pass the front tube (0.18 m), then would stand the bail up and carry it | 0.0 (as its centre passes the brackets' ends the grip turns in the pinch and the pail tips over, 75 degrees) |
| one_hand_rim | right draws it 90 mm by the bail and lets the bail go; the left alone takes the rim's front strip and draws it out | 0.0 (one hand cannot hold it level past the brackets' ends) |
| drag | as one_hand, then dragged along the floor | 0.0 (from the one-hand tip, not from a drag rule: the scorer has none) |
| drag2 | as full to the set-down, both still holding, then dragged 10 cm along the floor, then let go | 0.4 (the drag tilts the pail to 16.6 degrees: a SPLASH; a drag that stays under 15 degrees would not be penalised) |
| hold_on | as full, but only the right lets go; the left keeps its pinch on the rim | 0.5 (OUT, not DOWN: a hand 4 mm from the pail) |
| drop_low15, drop_low19 | as full, the pail held with its lowest corner 15 or 19 mm over the floor, then both let go | 0 (the pail tips over on landing, 75 degrees; HARD trips) |
| slow_late | as full to 80 mm up, then lowered fast to 40 mm up (about 0.3 m/s), braked to a stop and set down at 0.05 m/s; the left then presses the front down until the pail is level, as full does, and both let go | 1.0 (touches at 0.05 m/s, lets go at 0.25 m/s: a pail lowered fast, slowed and set down gently is not HARD) |
| tip_gentle | as tip_release, but the front lifted only 5 mm | 1.0 (a 5 mm settle is not a hard landing; the tipped pitch is 1 degree) |
| tip_release | as full, but the left lifts the pail's front 60 mm so it rests on its back edge, then both let go and it slaps flat | 0 (HARD: the front corners touch down faster than 0.3 m/s; held pitch 3 degrees, peak tilt 9.6, under the 15 degree splash limit, so it is HARD that trips) |
| leave | right draws it 100 mm by the bail and lets go on the brackets | 0.25 (drawn, still on the brackets) |
| drop | as full, both let go 12 cm over the floor | 0 (HARD: the corners touch the floor at about 0.9 m/s; the pail tips) |
| undo | as full to the set-down, both let go for 0.5 s, then the left takes the rim again and lifts it 8 cm and 20 cm back toward the brackets | 0.25 (drawn, not OUT: the flange is back over the brackets' ends). Lifted but still out of the brackets it would score 0.5 |
| none | nothing | 0.0 |

The score is read from the state at the end: only the failures latch (SPLASH, tilt over 15 degrees; HARD, a hard landing: the speed at first floor contact. Each of the bottom's corners is read the first step it is under 2 mm from the floor, and any corner then falling faster than 0.3 m/s voids the run (the demo touches down at 0.06 m/s; speed higher up does not count, so a pail lowered fast and slowed before it lands is not HARD; the corner's velocity is the pail's linear velocity plus its angular velocity crossed with the corner's arm, so a pail let go tipped on one edge that slaps flat counts), an estimate, read after every physics step and not only at the 20 Hz control step, so a short fall cannot slip between samples; a spill is read from the SPLASH tilt, as the contents are rigid; DROPPED; tipped over), so drawn, OUT and DOWN count only while true and an undone step loses its credit. HARD scores 0; DOWN needs every point of both grippers (fingertips, pads, palm, housing and wrist link: a lattice 2 cm apart over each gripper from 12 cm behind its frame to the fingertips) 30 mm or more from the pail's whole outer surface (flange, walls and bottom, measured to a 4 mm grid over it) and from its bail and the pail still for 1 s, counted as a running count that resets.

## Sizes

| What | In the scene | Published or estimated | Source |
|---|---|---|---|
| Commode | Drive 11135-1 deluxe bariatric drop-arm commode, shaped as in its manual's figure 1 (tube frame, drop arms, half-moon back rest, moulded seat and lid) | | Drive spec sheet, product page and manual |
| Frame tube | 25 mm; seat frame tubes 0.40 to 0.425 m up, the front one over the pail's way out | | estimate from the manual's figure |
| Width outside; between the arms | 0.667 m; 0.654 m (arm pads 50 mm wide) | 26.25 in; 25.75 in | Drive spec sheet; pad width estimate |
| Legs | four, 25 mm square, at the footprint's corners: centres 0.642 apart across (0.617 clear), 0.508 front to back | 26 in between legs | Drive product page; tube size estimate |
| Depth; overall height | 0.533 m; back rest to 0.838 m | 21 in; 33 in | retail listings (Zoro) |
| Seat and lid | seat 0.591 x 0.470 m, top 0.445 m up (lowest setting), 20 mm thick, a round opening about 0.25 x 0.27 m; lid 15 mm, closed | 23.25 x 18.5 in; 17.5 to 22 in high | Drive spec sheet; thickness and opening estimates |
| Drop arms | up, tube 0.64 m above the floor, foam pads 0.26 m long | | estimate |
| Pail brackets | flat bars from 5 mm behind the seat's front edge to the back tube, 0.314 apart centres, 20 mm wide and 10 mm thick, tops 0.375 m up, hung from the side tubes by two cross arms | | estimate |
| Pail | tapered, round-cornered: rim flange 0.348 x 0.25 m (10 mm thick, 30 mm strips), body 0.288 x 0.189 m at the rim and 0.24 x 0.15 m at the bottom, 0.19 m deep, 4 mm walls; rim front flush with the seat's front; its cover off, on the floor beside the commode | 12 qt; 13.7 x 9.8 x 8.5 in | Drive product page (12 qt); retail listing of Drive's 12 qt bucket (Carewell, 11108); shape an estimate |
| Handle | Drive 11108's bail: a moulded plastic band 14 x 6 mm, a ridged grip 16 mm round and 120 mm long at its middle, 0.37 m across between its ends, 0.15 m from its pivot axis to the grip, 50 g; on a free hinge through two lugs (16 mm round) on the pail's end walls, 40 mm under the rim and below the brackets; at rest folded forward against the pail's front, 38 degrees off straight down | a fold-down plastic bail with a ridged centre grip, pivoting on two lugs below the rim's end tabs; Carewell: 'a plastic handle and cover' | Carewell listing and its product photos of the 11108; sizes scaled from the photo by the listed 13.7 in length, estimates |
| Pail mass | 0.5 kg empty + 0.8 kg contents | | estimate |
| Friction on the pail | 0.6 static, 0.5 dynamic | | estimate |

## Colliders

Every body is built in `task.py` from tubes, lofts and plates (no third-party meshes); the colliders are below.

| Body | Colliders | Free or fixed |
|---|---|---|
| Commode | a convex hull round each straight tube (four legs, the seat frame's four tubes, four side braces, side and back ties, each drop arm's three runs, the back loop's three runs); a cylinder for each rubber tip; boxes for the two pail brackets, their four cross arms and four straps; hulls for the two arm pads and the back pad; one box for the seat (its opening lies under the closed lid) and one for the lid | fixed |
| Pail cover | one box | fixed |
| Pail | the tapered body as one convex hull, four rim-flange strips, the two pivot lugs (the 0.8 kg of contents are part of its uniform mass) | free, 1.25 kg |
| Bail | a convex hull round each of the band's 14 straight runs and one round the grip; it collides with the commode, the floor and the hands (a link does not collide with its own pail, so its stops against the pail's front and back walls are the hinge's limits, 0 and -284 degrees) | on a free hinge in the pail, 0.05 kg |
| Floor | the harness's bench top | fixed |
| Fingers | the harness's YAM finger colliders | the robot |

## Robot figures the demo keeps within

The YAM's jaws open to 94 mm and grip about 40 N a finger; each arm pushes about 20 N, carries 1.5 kg and reaches about 60 cm. The right hand takes the bail's grip 58 cm from its base with its jaws 40 mm apart, swings the bail up level and pinches its 16 mm grip; the left pinches the 10 mm rim strip at the front, 44 cm from its base, after a 90 mm draw, in front of the seat and well inside the front legs (the hands work within 0.12 m of the centre line; the legs stand 0.32 m out). The 1.3 kg pail is shared: the bail holds its pivots, the left its front. One hand alone cannot: with the bail held level under the front tube the pail's weight is 15 cm from the grip (about 1.9 N m), the round grip turns in the pinch as the pail's centre passes the brackets' ends, and the pail tips; the bail can be stood up over its pivots only 60 mm later, once they pass the front tube (`one_hand`).

## Real trial

Stand a Drive 11135-1 drop-arm commode (its pail slides in from the front), arms up and seat at its lowest, on the bench top, which stands for the floor, front toward the robot. Clamp it or weigh it down with sandbags on its frame so that it does not slide when the pail binds. Lay a towel with a disposable underpad on it in front of the commode, and mark a tape line on the bench 40 mm in front of the brackets' ends, plus a tape mark on one bracket 50 mm in front of the flange's start. Fill the pail with 0.8 kg of water (0.8 litre), never real waste. The pail's own bail is used, free on its pivots as sold, pinched at its ridged centre grip.

The maker's steps (11135-1 manual, 'assembling commode seat and lid'): 'Align the toilet seat clamps onto the back frame rail. (Both clamps should sit outside the pail brackets.)' and 'Slide pail in from front of commode.' The pail comes out the same way: drawn forward off its brackets by its handle, with the seat and lid left down ('Toilet seat must be in down position before use'). Film the pail's side with a phone at 240 frames a second against a printed millimetre scale and protractor beside the pail (pitch), and its front with a second phone and protractor (roll).

What the video sees, by the scorer's terms:
- DRAWN and OUT: the flange passing the bracket's 50 mm mark, then clear of the brackets' ends.
- SPLASH: a drop of water on the pad, or a pitch (side phone) or roll (front phone) over 15 degrees against a protractor at any time.
- HARD (a hard landing): the speed over the last 5 mm before contact on the 240 fps side video: the bottom's lowest corner (the low edge as it slaps down too) taking fewer than 4 frames (17 ms) to cross its last 5 mm is over 0.3 m/s. A knock you can hear or a visible bounce is only a prompt to measure. A pail lowered fast and slowed before it touches is not HARD. Spillage: after the run, photograph the floor and the pail; water on the pad or floor, or a pitch or roll over 15 degrees, is a SPLASH.
- DOWN: the pail resting on the pad past the tape line; within 10 degrees of level by a phone level app on the rim after the run; every part of both hands 30 mm clear and the pail still in the video's last second, against the scale. (The sim reads a lattice of points over the whole of each gripper, wrist link to fingertips, so a palm or wrist resting on the pail counts as touching, as in the real trial. The lattice is a box around the gripper, a little larger than its real housing, so it reads slightly conservative; no demo rests a palm on the pail, as the arm cannot reach such a pose here, so that reading is checked by its geometry, not by a run.)
- DROPPED and tipped: by eye on the video.

| Real-trial kit | Use | Price (about) |
|---|---|---|
| Drive 11135-1 drop-arm commode with its 12 qt pail | the object | $110 |
| Two bar clamps or two sandbags | hold the commode | $20 |
| Disposable underpads (pack) and a towel | the set-down place, and the splash check | $15 |
| Two phone tripods, printed millimetre scale and two protractors, masking tape | the 240 fps side video, the front video for roll, their marks | $25 |
| Silicone pad sleeves for the grippers | pad protection | $5 |
| Total, with two phones already to hand | | about $175 |

## Where the used pail goes down

A nursing-assistant checklist for the bedside commode reads 'Remove the receptacle from the commode and cover it.', then 'Take the receptacle to the bathroom.' Drawn out from the front, the pail needs both hands until it is down, so it is set down to be covered. MEIKO's bedpan cleaning guide (Step 1, 'Put the bedpan down and take care of the patient') gives 'Best storage space: a side trolley, the floor with an underlay' and 'Never put it down on the patient's bed, bedside table or chair.' The task follows that: the floor in front of the commode, on a disposable underpad. Some care-home procedures fit the lid while the pail is still in the commode ('Cover commode pan with lid and carefully remove from the commode to avoid spillages', Harrogate NHS trust); here the pail comes out by the maker's front draw, with the seat down, and the lid goes on once it is out.

## Safety for a real trial

Use water, never real waste. The underpad and towel catch any splash; keep the water side of the pail away from the arms' wrist cameras and cables, and dry the bench after each trial. Silicone sleeves keep the gripper pads dry. Clamp the commode so it cannot tip toward the robot. Keep people clear of the arms' reach while the robot runs and keep an emergency stop in reach.

## What is modelled, and what is not

- Frame, brackets, seat, lid and pail are built in `task.py` from tubes, lofts and plates (no third-party meshes), shaped from the manual's figure 1. The commode is Drive's 11135-1 at its published sizes (outside width, width between arms and legs, seat, seat height, depth and height), with all four legs, the seat and its lid, both drop arms up and the back rest. Bracket, seat-opening, pail-shape and mass figures are estimates. The bail is shaped from Drive's photos; its sizes are not published. Friction on the pail is 0.6 static, 0.5 dynamic (estimates).
- The contents are part of the pail's rigid body (no fluid), so the splash rule is a tilt and a landing speed, not a fluid. 0.8 L lies about 16 mm deep in the 0.288 x 0.189 m body, so still water would reach the 0.19 m rim only past about 50 degrees; the 15 degree limit is a margin for slosh as the weight arrives at once (estimate).
- The bail swings freely on its pivots (a hinge with 0.002 N m of friction) and collides with the frame, the floor and the hands. It does not collide with its own pail; where its band meets the pail's front or back wall, the hinge's limits stop it.
- The seat clamps on the back frame rail ('Both clamps should sit outside the pail brackets') are not modelled; the pail never touches them.
- The demo reads the pail's pose and the bail's angle to find the grip, the pivots and the rim, and to level the pail.

## Sources

- Drive Medical 11135-1 Deluxe Bariatric Drop-Arm Commode: spec sheet (Drive catalogue page, https://www.globalindustrial.com/site/images/Drive%20Medical/11135-1_v3_2_19.pdf): width 25.75 in between arms and 26.25 in outside, seat 23.25 x 18.5 in, height 17.5 to 22 in, "Pail easily slides in from the front of the commode", "Comes complete with 12 qt. commode bucket"; Drive's product page (https://shop.drivemedical.com/us/en/p/11135-1): width between legs 26 in; depth 21 in and height 33 in from retail listings (Zoro, https://www.zoro.com/drive-medical-bariatric-drop-arm-bedside-commode-chair-11135-1/i/G9301698/).
- Drive's 12 qt commode bucket with handle and lid (11108): 'For use with Drive commodes 11114KD, 11125PSKD, 11130, 11135, 11105N, 11117N', 'a plastic handle and cover made from recycled materials', 'a width of 9.80 inches, a height of 8.50 inches, and a depth of 13.70 inches' (Carewell, https://www.carewell.com/product/drive-commode-bucket-with-handle-and-lid-12-qt/?sku=11108-EA1); the bail and rim tabs from Drive's product photos on that page.
- The pail's way in and out: Drive's drop-arm commode manual (11135-1, https://static.helpjuice.com/helpjuice_production/uploads/upload/image/14616/3147799/DRIVE_11135-1_DROP_ARM_BARI_COMMODE.pdf): "Slide pail in from front of commode", with the seat clamps on the back frame rail sitting "outside the pail brackets". The bracket sizes are an estimate from listing photos.
- O*NET 31-1131.00 Nursing Assistants (Core, importance 85: 'Supply, collect, or empty bedpans.'): https://www.onetonline.org/link/details/31-1131.00 . O*NET's Core criteria are 'relevance >= 67% and ... a mean importance rating of >= 3.0' (https://www.onetcenter.org/dictionary/29.0/excel/task_statements.html), so at least about 0.97 million of the 1,448,910 do it. The task statement names bedpans; the commode pail is the same receptacle care for a patient who can sit up, as the checklist below shows.
- Employment: BLS Occupational Employment and Wage Statistics, May 2025, Nursing assistants, 1,448,910 (Table 1, https://www.bls.gov/news.release/ocwage.t01.htm).
- Bedside commode skills checklist for nursing assistants (Bryan University): 'Remove the receptacle from the commode and cover it. Close the commode lid', 'Take the receptacle to the bathroom', 'Empty and clean the receptacle per facility policy. Replace it in the commode.' https://src.bryanuniversity.edu/wp-content/uploads/Assist-w_-Bedside-Commode_Skills-Checklist.pdf
- Where a used pan is set down: MEIKO, 'Cleaning bedpans instructions', Step 1: 'Best storage space: a side trolley, the floor with an underlay, the rubbish bin can if necessary (when standing securely)'; 'Never put it down on the patient's bed, bedside table or chair.' https://www.meiko.com/en-hk/news-press/magazine/cleaning-bedpans-instructions
- Harrogate and District NHS Foundation Trust, 'Cleaning and disinfecting a commode or bed pan manually for Care Home settings' (2020): 'Cover commode pan with lid and carefully remove from the commode to avoid spillages.' https://www.infectionpreventioncontrol.co.uk/wp-content/uploads/2020/07/Cleaning-a-commode-pan-Care-Home-July-2020-Version-2.01-1.pdf
- Bracket, pail and bail sizes, masses, frictions, and the 15 degree and 0.3 m/s landing limits: estimates.

O*NET text is from USDOL/ETA under CC BY 4.0; O*NET is a trademark of USDOL/ETA. Drive Medical, Zoro, Carewell and MEIKO are named for identification only; this task is not affiliated with them.

## Licence

MIT
