# Set a date stamp to the work order's date and stamp it

Configure, then apply, with a permanent record: the stamp is first set to the work order's date, then used as a tool, inked, pressed once inside the box the right way round, and returned. A wrong setting becomes a wrong impression that cannot be undone, so the scoring rewards configuring before applying. Board tasks that set a device to a value read elsewhere (#746, a clock set from another clock; #237, a busybox set from a setup sheet) end with the configured state; here nothing counts until the setting has been applied. Dated stamps keep receiving, legal and quality records. Done by: shipping and receiving clerks, legal records clerks, quality inspectors.

Category: Impact on jobs, because the skill configures a record-making tool in clerical and inspection work. This one-handed desk job uses the right arm; the left stays ready. All assets are locally authored primitives and the included kits. No external visual assets or licences are needed.

## Run and score

With the official simulator installed, run from this folder. The harness derives the reset seed with CRC32; --seed 0 selects layout 6 and no --seed selects layout 3 (see the mapping below). Use zero object jitter and 20 Hz. The printed date is read with the right wrist camera.

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=set_date_stamp_from_work_order.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --seed 0 \
  --instruction "Stamp the work order's date in its marked box, then return the stamp to its cradle." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ./demo-run
```

For Dream-1 use `--policy dream` with the same task, instruction, seed and limit. The run prints task_score. Use `inspect-robots video <run.json>` to encode the recorded cameras. Scores are recorded in task.json. CPU scorer checks: `python -m pytest -q -p no:cacheprovider` in this folder with yam_arena installed.

## Scoring

| Score | End state |
|---|---|
| 0 | Untouched start (every layout), no band credit, or a sticky violation: an incorrect impression, a second impression or a smear |
| 0.15 | No impression; one band that needed a change shows its requested number (or, in layouts 1, 7, 14 and 15, the day is right but the month has been moved off its requested number) |
| 0.3 | No impression; both bands show the requested date |
| 0.7 | Exactly one correct impression, stamp not seated |
| 0.9 | Correct impression and seated stamp, held or moving |
| 1.0 | Correct impression, seated, released and still for 1 s; success holds another 1 s |

Band reading: day shown = (start day - 1 + round(day angle / (360/31) degrees)) modulo 31 + 1; month likewise modulo 12 with a 30 degree pitch; positive is counter-clockwise seen from above; a band more than 4 degrees from its nearest detent is blank. Inked: face compressed >=1 mm with its centre over the 90x60 mm pad, the bottom of its centre within 2 mm of the pad top and tilt <=10 degrees; ink stays on the face for the rest of the trial. Impression: the inked face compressed >=1 mm with its centre off the pad and the lowest point of its 40x18 mm face (its lowest bottom corner, at any tilt) within 2 mm of the paper top or the table top; dry presses and contacts compressing the face <1 mm leave no impression. Each press makes one impression, recorded at its first sample at >=1 mm: face centre XY, lowest-point height, yaw, tilt, both band readings. The next impression can start only after compression falls below 0.5 mm and the face's lowest point has risen >=3 mm above its lowest height in that press. Smear: before that re-arm, any off-pad contact compressed >=1 mm with the face's lowest point within 2 mm of the paper or table top and the face centre >2 mm from, or yaw >2 degrees from, the recorded impression. Correct impression: both bands nonblank and showing the requested date; face centre on the 150x130 mm work order, the face's lowest point within 2 mm of the paper top; every corner of every active segment and of the decimal square inside the 80x44 mm box with >=5 mm clearance from the inner edge of each 1 mm line; yaw within 5 degrees of the box; tilt <=5 degrees. Zero for the rest of the trial, whatever happens later: an incorrect impression (wrong or blank band, margin, yaw, tilt, off the sheet, on the table; an edge-first press tilted more than 5 degrees is one), a second impression of any kind, or a smear. With no impression, current band credit: 0.15 for each band that needed a change and now shows its requested number; a band whose requested number equals its start number earns 0.15 only while both bands show their requested numbers (layouts 1, 7, 14, 15: 0 until the day is right, then 0.3); untouched start 0 in every layout. Exactly one correct impression: 0.7, whatever the bands show later; 0.9 if also the stamp is tilted <=5 degrees, its whole rotated 180x60 mm housing footprint is inside the 200x84 mm cradle pocket and its root is within 3 mm of its seat height (table + 13 mm); 1.0 if that holds with every gripper TCP >=30 mm from the stamp's bounding box, stamp speed <0.01 m/s, spin <0.05 rad/s, face and both plunger slide speeds <0.05 m/s and both knobs within 1 degree of a detent, all continuously for 1 s. A held or moving seated stamp earns 0.9. Success: 1.0 held for a further 1 s.

### Why the rules are what they are

- Band credit. In layouts 1, 7, 14 and 15 the month already shows the requested number. Paying for it on its own would pay for doing nothing, or for knocking the day knob. So a band that needed no change counts only when the whole date is right. Band credit uses the current knob angles, so turning a band back loses it.
- One threshold. Ink pickup and printing both need 1 mm of face compression. The face spring is preloaded to 0.8 N at 800 N/m, so 1 mm takes a contact force of about 1.6 N. A lighter touch does not move the face, and the readout treats it as leaving no ink.
- Re-arm after a 3 mm rise. On first contact the cushioned face can bounce or chatter, and its compression can dip below 0.5 mm. Without the 3 mm rise, one press would count as two impressions. A recontact before the re-arm prints the same marks only if it lands within 2 mm and 2 degrees of the recorded impression. Anywhere else it is a smear, which scores 0.
- Knob stillness by angle. Knobs held by preloaded detent contacts can report solver velocity while not moving. So stillness checks that each knob is within 1 degree of a detent, not its reported velocity. A knob turning at 0.5 rad/s leaves a 1-degree window in 0.07 s. In the simulator the detents pull a released knob to within 0.03 degrees of centre. A knob resting 1 to 4 degrees off a detent still reads correctly, but caps the score at 0.9.
- What the scorer reads. It uses only stamp poses, joint angles and speeds, and the trial's press record. It never reads the visuals or the robot's commands.

Measuring a real trial needs a ruler and an overhead photo for the printed date, segment-to-line distances, orientation, footprint and seat height. Two images taken one second apart check release and stillness. Count every impression made during the trial, including a later mark on the table and a partial mark from a face pressed edge-first: ink errors are permanent.

## Setup and reproduction

World x is forward from the arm bases, y left, z up, all metres; table z=0.7475 and bases (0,+/-0.305,0.750). All objects start yaw zero, upright. The orange ABS custom date stamp is free: 197 g housing/handle plus two 20 g polymer setting wheels, two 3 g spring plunger assemblies and a 15 g rubber die, total 258 g. Its root starts (0.275,-0.417,0.7615), 1 mm above its seat. Local housing box centre (0,0,0.0345), size (0.180,0.060,0.045); post centre (0,0,0.0705), size (0.010,0.010,0.027); charcoal rubber grip centre (0,0,0.097), size (0.030,0.018,0.026), centred over the mass. Day wheel at x=+0.060, month at x=-0.060, y=0, both vertical hinges anchored z=0.065, limits -350 to +350 degrees, damping 0.00035 N m s/rad, armature 0.000001 kg m2, no spring; every joint starts at zero. Each hub is a z cylinder radius 12 mm (day) or 22 mm (month), height 54 mm, centre z=88 mm. Day bottom disk radius 21.5 mm, height 2 mm, centre z=60 mm; month disk radius 28 mm, height 2 mm, centre z=58.5 mm. Day has 31 steel pins of radius 1.5 mm, height 8 mm on a 20 mm ring; month has 12 pins of radius 3 mm, height 12 mm on a 25 mm ring. Pin centres have local z=65 mm, angles 90+180/count+k*360/count degrees. Orange day fin is 30x16x22 mm, blue month fin 44x16x22 mm, both centres (wheel x,0,126 mm), along x at zero. A steel day ball radius 2.5 mm sits at (60,23.347888,65) mm, with y-axis rod radius 0.8 mm, length 2 mm, centre y=26.347888 mm. Solid POM month ball radius 5 mm at (-60,28.852712,65) mm, y rod radius 1.5 mm, length 4 mm, centre y=33.852712 mm. Each y slide is anchored at its ball centre, lower -0.2 mm, upper 2 mm day or 8 mm month, spring 300 N/m, rest -3 mm (0.9 N preload), damping 0.4 day or 2 month N s/m. Both moving wheel/plunger bodies use friction 0.1/0.08 and self-collide. The dark rubber die is a 40x18x4 mm plate centred (0,0,2 mm), carried by a dark rubber die carrier 40x18x8 mm from local z 4 to 12 mm (up to the housing bottom; visual only, no collider, it moves with the die), with a z slide anchored there, travel 0..4 mm upward, spring 800 N/m, rest -1 mm, damping 2 N s/m, friction 0.6/0.4. Its underside carries a mirrored raised DD.MM pattern: four 10 mm high digits with centres x=-11.5,-4.5,+4.5,+11.5 mm, 6 mm width, 1.5 mm segments, and a 1.5 mm decimal square at (0,-4) mm. Two ivory 17x13 mm top windows centred (+20,-22) and (-20,-22) mm, z=57.1 mm, show 10 mm dark day/month digits; bars are visual-only. The blue steel cradle is fixed at (0.275,-0.417,0.7475), no simulated mass: floor box centre (0,0,2 mm), size 208x92x4 mm; two seat rails centred (0,+/-24.5,14.5) mm, size 180x9x21 mm; x walls centred (+/-102,0,16.5) mm, size 4x92x25 mm; y walls centred (0,+/-44,16.5) mm, size 200x4x25 mm. Pocket is 200x84 mm; housing bottom rests at table+25 mm, stamp root seat at table+13 mm; face clears floor by 9 mm. The black felt ink pad in a rigid integrated tray is fixed, no simulated mass, origin (0.225,-0.290,0.7475), box centre (0,0,5 mm), size 90x60x10 mm. The white cellulose work order, including its tape restraint, is fixed, no simulated mass, origin (0.225,-0.130,0.7475), box centre (0,0,0.5 mm), size 150x130x1 mm. Its date is printed for reading with the right wrist camera, four 24 mm high dark seven-segment digits with 3.6 mm wide segments DD.MM centred (0,35) mm above an 80x44 mm marked target box centred (0,-20) mm; lines 1 mm wide. An integrated blue pigment header band at (0,57) mm is 120x6 mm. Printing is visual-only, negligible mass, at z=1.1 mm locally. Written black ink is one fixed PointInstancer of 30996 discs over the whole sheet, pitch at most 0.8 mm, radius 0.5 mm, thickness 0.1 mm, centres 0.15 mm above paper. Stamp, cradle, pad and paper have friction 1.2 static/1.0 dynamic except the links stated above, all contact offsets 0.1 mm. No other props. Layout index selects start day/month -> requested day/month: 0: 03.12 -> 05.01; 1: 11.02 -> 14.02; 2: 24.01 -> 28.03; 3: 29.01 -> 03.04; 4: 10.06 -> 16.05; 5: 23.08 -> 30.06; 6: 05.10 -> 07.07; 7: 18.08 -> 21.08; 8: 04.08 -> 02.09; 9: 21.08 -> 18.10; 10: 31.08 -> 27.11; 11: 14.01 -> 09.12; 12: 31.03 -> 25.01; 13: 17.05 -> 10.02; 14: 06.03 -> 04.03; 15: 25.04 -> 22.04.

The harness passes a derived seed to reset: zlib.crc32(f"{eval_seed}:{scene_seed}:{epoch}".encode()) & 0xFFFFFFFF (inspect_robots.rollout.derive_seed); the layout index is that value modulo 16. With no scene seed (scene_seed=None) and epoch=0, the CLI mapping is:

| --seed | Layout index |
|---|---|
| 0 | 6 |
| 1 | 8 |
| 2 | 11 |
| 3 | 5 |
| 4 | 12 |
| 5 | 2 |
| 6 | 1 |
| 7 | 15 |
| 8 | 3 |
| 9 | 13 |
| 10 | 0 |
| 11 | 14 |
| 12 | 13 |
| 13 | 3 |
| 14 | 10 |
| 15 | 4 |
| 16 | 7 |
| 17 | 9 |
| no seed | 3 |

Direct reset calls use their received seed modulo 16 (None=0); a CLI run with no --seed instead hashes eval_seed=None and selects layout 3.

## Dates by layout index

| Layout index | Starting DD.MM | Requested DD.MM | Day clicks | Month clicks | Day turn degrees | Month turn degrees |
|---|---|---|---|---|---|---|
| 0 | 03.12 | 05.01 | +2 | +1 | +23.226 | +30.000 |
| 1 | 11.02 | 14.02 | +3 | +0 | +34.839 | +0.000 |
| 2 | 24.01 | 28.03 | +4 | +2 | +46.452 | +60.000 |
| 3 | 29.01 | 03.04 | +5 | +3 | +58.065 | +90.000 |
| 4 | 10.06 | 16.05 | +6 | -1 | +69.677 | -30.000 |
| 5 | 23.08 | 30.06 | +7 | -2 | +81.290 | -60.000 |
| 6 | 05.10 | 07.07 | +2 | -3 | +23.226 | -90.000 |
| 7 | 18.08 | 21.08 | +3 | +0 | +34.839 | +0.000 |
| 8 | 04.08 | 02.09 | -2 | +1 | -23.226 | +30.000 |
| 9 | 21.08 | 18.10 | -3 | +2 | -34.839 | +60.000 |
| 10 | 31.08 | 27.11 | -4 | +3 | -46.452 | +90.000 |
| 11 | 14.01 | 09.12 | -5 | -1 | -58.065 | -30.000 |
| 12 | 31.03 | 25.01 | -6 | -2 | -69.677 | -60.000 |
| 13 | 17.05 | 10.02 | -7 | -3 | -81.290 | -90.000 |
| 14 | 06.03 | 04.03 | -2 | +0 | -23.226 | +0.000 |
| 15 | 25.04 | 22.04 | -3 | +0 | -34.839 | +0.000 |

The day changes use each sign eight times and every magnitude 2..7. Month changes are zero four times, positive six and negative six, with every magnitude 1..3. Requested dates are valid non-leap-year dates and span all twelve months. All joints start at zero. The layout index comes from the derived reset seed, using the CLI mapping above. Reset chooses the date before the original reset's first observation, sets the printed date and starting windows, and clears both independent press histories and ink. The bands map (start-1+nearest detent index) modulo count +1; positive rotation increases the number counter-clockwise from above.

## Demo and runtime

The empty right gripper first views the order at TCP (0.262895,-0.094670,0.9075) m, yaw zero, for a 1.5 s move and 0.8 s settle. Day then month: hover, settle, descend at yaw zero, close on the fin, turn the shortest signed number of detents (at most 90 degrees), hold, open, lift and settle. A band needing no change uses the same waypoints at yaw zero. The policy builds its fixed 42-waypoint plan before reset; durations are the same in every layout.

What the demo knows. The scripted demo is open loop and cannot read camera images. It is given the trial's layout tuple: the starting day and month, which the stamp windows show at reset, and the requested day and month, which the work order prints. These are the same four numbers a camera policy reads from the scene, and nothing else is given. From them the demo computes only the two twist yaws. It reads no object state, joint angle, scorer result or press record. The demo is an existence proof that the motion reaches full score within the run limit, not a perception policy: reading the date from the images is the policy's job.

The robot grasps the centred handle, lifts straight out and travels over the pad. After a 0.8 s settle it lowers for 3 mm planned face compression, holds 0.5 s and lifts. It repeats over the corrected target for 3.5 mm planned compression. Return has a 0.8 s settle, lowers to 2.5 mm above the seat, opens, rises, backs off to TCP (0.220,-0.110,0.7875) m and holds 4 s while the wrist camera frames the marked box. Nothing attaches the stamp to the robot: friction holds the grip, the joints are passive and gravity does the final seating.

World TCP z (m): read 0.9075; park 0.7875; travel 0.9175; fin pinch 0.8935; handle 0.8575; pad press 0.8536; paper press 0.8441; return open 0.8635. A gripped stamp rides 3.5 mm low and the arm stops 1.4 mm short of a commanded height under load, and both are applied at each surface. In the simulator, arm compliance and slip in the grip take about 1 mm more off the compression. So the targets expect about 2.0 mm at the pad and 2.5 mm at the paper: 1.0 and 1.5 mm above the 1 mm threshold. If the compression is larger, the die reaches its 4 mm travel stop. The slide's upper limit then carries the excess load, and arm and contact compliance take up further commanded motion. At 4 mm compression plus 1 mm extra arm travel, a rigid-pose check confirms that only the face can touch either surface. Every other stamp shape stays at least 7 mm above it and every gripper shape at least 79 mm above it.

The paper target carries a constant aim correction of (+5.0,+4.7) mm in XY and -2.0 degrees in yaw. It is the same in every layout. It offsets an impression bias of about (-5,-4.7) mm and +2 degrees, measured on the simulator, that comes from the arm and the grip. All five paper travel, settle, lower, press and lift targets use TCP XY (0.2300,-0.1453) m and yaw -2 degrees; the return travel restores yaw zero.

The plan lasts 55.5 simulated seconds. With the extra success hold, 56.5 s is 47.1% of the 2400-step / 120 s limit, leaving 52.9% spare. The timing copy scales every duration by 0.8, giving 44.4 s. At about 0.16 wall seconds per control step, a nominal plan plus hold takes about 3 minutes after startup and a full-limit trial about 6.4 minutes.

## Physical margins and rendering

The day detent has a 40 mm pin circle, 3 mm pins, a 5 mm ball and a 300 N/m spring preloaded 3 mm. Month has 12 pins on a 50 mm circle, 6 mm pins, a 10 mm ball and the same spring and preload. Seated y is 23.347888/28.852712 mm, crest travel 0.652112/4.147288 mm, crest force 1.095634/2.144186 N, peak quasi-static torque 0.01231808/0.03744621 N m (day/month). Both hinges have damping 0.00035 N m s/rad. The light 20 g wheels and 3 g plungers keep the centre of mass under the handle. Both hubs are 54 mm tall and the fins are centred at z=126 mm, so opening and twisting fingers clear the centred handle. The 180x60x45 mm housing holds both wheels with the handle between them, which keeps the wrist within its limits during the turns.

The 200x84 mm cradle pocket gives 10/12 mm nominal housing clearance per side. Every +/-7 mm XY, +/-3 degree yaw return remains inside it, with worst x/y residual clearance 1.553/0.331 mm. The walls end 4 mm above seat level, so no wall-top landing occurs in that envelope; face/floor gap is 9 mm. Static friction mu=1.2 and 258 g weight give 3.037176 N lateral resistance and at least 0.060744 N m yaw resistance at the 20 mm rail inner edges. The wheel torque is 0.03744621 N m. Assuming a residual pinch push of at most 1 N, the summed friction utilisation is 0.9457, so the stamp is not expected to slip. The low rails catch larger slips. Passive seating and traction depend on the dynamics, which offline replay does not prove.

The order glyphs are 24 mm tall with 3.6 mm segments. The 69.6x24 mm date envelope is 7 mm clear of the header and 20.5 mm clear of the box, with at least 18 mm to a sheet edge. At the centred read hover, the four nominal glyph heights are 18.182..20.017 pixels in the right wrist camera and 4.585..4.712 in the top camera at 320x176. At the final park the full box projects to [184.29,45.94]..[280.24,159.02] px in the wrist camera. Nominal centred impression glyphs are 15.786..17.151 px tall. The corrected aim projects to [206.96,81.83]..[230.43,123.32] px for layout 0 and [206.96,82.11]..[231.49,122.41] px for layout 5, with glyphs 15.986..17.387 px tall. These projections use the calibrated camera intrinsics, camera orientation and gripper-to-TCP offset.

One PointInstancer holds 30996 fixed ink dots, pitch <=0.8 mm, radius 0.5 mm, authored with vectorised arrays. A 16761-dot sheet spawned in 0.044 s on the simulator, so about 0.083 s is expected here; the simulator prints the measured time as DATER_SPAWN. Every digit, segment and dot is authored at spawn. At runtime only visibility changes, driven by a separate pose/joint press history that follows the same press rule as the scorer. The die carrier is a visual cube on the face link with no collider, so it adds nothing to contact. Visuals never drive scoring.

Each band's starting number is an offset on its wheel angle, equivalent to the wheel having been turned before the trial; physical detents set the steps. Ink is a visual readout from the face's spring travel and pose, not fluid; the sheet is rigid. Raised die segments and the die carrier are visual only. Primitive ABS housing, polymer wheels, steel pins and passive springs approximate a custom dater. No contact or force readings. Contacts that compress the cushioned die by less than 1 mm (under about 1.6 N) are treated as leaving no ink. This is a simplification: a real inked die can leave a faint smudge on a light graze. A cushioned rubber die needs firm, even pressure to print a legible character, and a check of the stamped record counts legible marks. Any contact firm enough to compress the die 1 mm, at any tilt, counts.

## Offline checks

Kinematic replay cannot turn the passive wheels or compress the face, so offline replay ends at 0 and is advisory; the GPU demo is the evidence. The included pytest files score all sixteen seated goal states and the press-history, threshold and smear rules.
