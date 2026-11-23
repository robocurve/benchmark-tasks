# Take a phone panorama: lift the iPhone from its stand, tap the shutter, turn it about its camera past three boards

An iPhone 12 stands in a desk phone stand with the Camera app open in Pano. Three boards stand on a fan round a tape
cross on a mat. Each board carries a printed vertical line, and a thin steel rod stands in front of it on the line's
sight line from the cross. The instruction is:
**"Take a panorama that shows all three boards in full, turning the phone about its camera above the cross, then put
the phone back in its stand."**

Seen from a camera above the cross, each rod hides its line. Seen from a camera that has moved sideways off a board's
sight line, the near rod and the far line separate and the line shows beside the rod. This is the photographer's
no-parallax test, and here it reads the turn from the phone's own panorama. The robot has to lift a 7.4 mm thick
phone, start the capture with a fingertip tap on the on-screen shutter, turn the phone through 114 degrees about its
camera, stop the capture with a second tap, and stand the phone back in its stand.

The hand cannot hold the phone at its camera. Apple's drawing of the iPhone 12 says "Do not obstruct the imaging
features: front camera, rear cameras, rear flash", and the edge buttons work the camera: a volume press starts or
stops a capture, and the side button locks the phone. Every grip that keeps off the camera corner and the buttons
holds the phone 30 mm or more from its lens (42 mm in the demo). Turning the wrist alone then swings the lens round a
circle of that radius and the lines show beside their rods. The hand has to circle as the phone turns, so that the
camera stays over the cross.

## Why it matters

- Google, "4 tips from a photographer on using Pixel's new panorama mode" (blog.google, 1 May 2025), from a
  photographer on the Pixel development team. Tip 1 is "Move your Pixel, not yourself": "Rather than rotating your
  whole body when taking a panorama, just rotate the camera as much as you can while standing in place", because
  "you'll eliminate perspective shift — or those jagged edges you can get when each transition doesn't quite line
  up — and create a smooth stitch."
- Apple, iPhone User Guide, "Take panoramic photos with your iPhone camera": "Pan slowly in the direction of the
  arrow, keeping it on the center line", and "To finish, tap the Shutter button again".
- The no-parallax test that panorama photographers use: line up a near and a far vertical in the viewfinder and pan;
  "If the nearer object appears to shift its position relative to the farther object, parallax is present. This
  indicates that the axis of rotation is not coincident with the entrance pupil" (Fstoppers, "A Beginners Guide to
  Parallax and How to Avoid It When Shooting Panoramas").

A phone panorama is one of the most familiar camera gestures. The parallax people see in it comes from swinging the
phone on an arc: a table, a room or a group a metre away comes out with doubled edges and broken lines at the seams.
Turning the phone about its camera is the exact form of Google's "rotate the camera ... while standing in place".
The cross and the rods do not tell the robot what to do; they are the standard way to see whether it was done, the
way a tape mark shows a tripod spot.

Nearest tasks. A public-suite task (bimanual photograph) holds a camera still before a logo while the other hand presses its
shutter: one pose at one instant. Board tasks with phones (a screenshot by both buttons on an iPhone in a stand; a
pinch-zoom on a tablet) and cameras (a single-use camera's wind-on and shutter) keep the device still. Here the photo
is taken during a 114-degree turn, and the phone's own panorama is the evidence of how it turned. The pick-up, the
taps and the set-down are ordinary steps round the turn.

## Files

| File | What it is |
|---|---|
| `task.py` | the yam_arena task file: the phone and the fixtures, the Camera rule (taps, button presses, the capture) and the score |
| `geometry.py` | every size and the layout, the phone's features from Apple's drawing, and the rule's constants |
| `build_assets.py` | writes `assets/*.usda`, the textures and the boards' prints from `geometry.py` (needs `pxr` and PIL, not the simulator) |
| `assets/print/board_1.pdf` ... `board_3.pdf` | the three boards' prints at true size (A4, print at 100 %) |
| `tests/test_scoring.py` | scoring checks without the simulator (the Camera rule driven by hand-built phone paths; yam_arena's IK and forward kinematics place the arms' collision hulls) |
| `scenekit.py` | USD bodies for task files, and the physics-rate hook |
| `demokit.py`, `demo.py` | the scripted demo and the scorer checks in simulation |

All assets are built from scratch in `build_assets.py` (no third-party meshes, scans or textures); the boards' digits
and the screen's labels use the DejaVu Sans fonts. The robot is yam_arena's.

## Run a trial

```
source <ARENA_DIR>/yam.env
python build_assets.py                                    # once
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    --scorer task_score --max-steps 3000 --policy ...
python tests/test_scoring.py                              # the scoring checks (no simulator)
```

No seeds are used: the layout is the same in every trial. A 3,000-step trial takes about 15 minutes of wall time on
one shared GPU (240 Hz physics, three cameras, the Camera rule every step). `success()` is never true: every trial
runs to its step limit and is scored on its end state and the panoramas saved during it.

## Dream-1 run

Dream-1 (dreamlabs/dream-1-beta), run once with the task's exact instruction, `object_jitter=0`, 20 Hz and the
3,000-step limit, refused at its first inference request, before any action (only the 32 pre-roll steps at the start
pose had run). inspect-robots reported: "PolicyError: DreamAPI refused the inference request: DA-4000 BAD_REQUEST
(malformed): instruction invades privacy, refused: Instruction asks to take a panorama showing the room and boards."
Nothing was scored, so there is no model run video; the full-score demo is attached. Under this task's scoring a
refusal reads 0, the same as doing nothing.

## Scripted demo

```
python demo.py task.py demo_out [--no-record] [--slit] [--hold S] \
    [--variant wrist|plateau|button|fast|short|nostop|keep|drop|nothing]
```

The demo reads object poses from the simulator (privileged: the phone's pose). In order:
1. **Pick-up (left arm).** The gripper tilted 20 degrees to the stand's recline, a pinch across the phone's thickness
   on its top edge, centred 16.5 mm from the screen-left edge (55 mm from the camera side, clear of the camera
   corner); the fingertips 16 mm down the faces. Lifted 45 mm along the phone so its bottom edge clears the lip,
   stood upright and carried to the cross, where it turns the camera to face 100 degrees. The phone's pose in the
   hand is measured once it is gripped; from then on every command for the hand is solved from a goal for the phone
   (lens over the cross, camera level), with a correction in position and tilt updated every step from the phone's
   measured pose.
2. **Start tap (right arm).** Fingers closed, the hand tipped 15 degrees down, the end faces brought to the
   on-screen shutter at 0.5 mm a step until they touch the glass, held there without pressing for 0.1 s, then off
   the glass at 2 mm a step and home.
3. **The sweep (left arm).** The camera turned from 100 to -14 degrees in 10 s (17 degrees a second at the most),
   the hand circling the point above the cross at 42 mm.
4. **Stop tap (right arm),** through a point beside and above the phone that keeps the right arm 10 mm or more from
   the phone and the left arm; then home.
5. **Set-down (left arm).** The camera turned back to 150 degrees, the phone carried round in front of the stand's back
   plate, tilted to the recline 25 mm in front of its place, moved back onto the plate and lowered into the slot in
   closed loop, let go; the hand backs off along the phone and goes home.

**Result: 1.0.** Run twice: every logged number came out the same (see the margins below). The capture runs from step
397 to step 779 (19 s); the score reaches 1.0 by step 1,140, once the phone has stood still in its stand for 1 s with
both hands clear; the script ends at step 1,159 with both arms home, so the 3,000-step run limit is about 2.6 times
the demo.

**Margins** (from `task.py`'s rule during the demo):

| Check | In the demo | Needed |
|---|---|---|
| taps | 0.10 and 0.20 s on the glass; starting 1.2 and 2.7 mm from the shutter's centre, 1.6 and 3.9 mm at the most | 0.5 s or less, within 6 mm throughout |
| board 1 in full | the panorama's left end at 125.9 degrees; the sheet's far edge at 121.1 | the sheet inside (4.8 degrees to spare) |
| board 3 in full | the panorama's right end at -40.1 degrees; the sheet's far edge at -35.1 | the sheet inside (5.0 degrees to spare) |
| aligned | the lens 0.10, 0.31 and 0.84 mm sideways off the three sight lines | T or less |
| clean | 17.1 degrees a second at the most; the view drifting 0.59 degrees at the most | under 40; under 5 |
| level | camera tilt 0.34 degrees, roll 0.31 at the most | within 3 and 4 |
| hands clear | no robot point in the first frame's left half, any step's central strip or the last frame's right half; the camera corner untouched | none |
| the left arm's base | 7.4 degrees outside the first frame (its nearest point 33.4 degrees left of the frame's centre) | outside the frame (26) |
| rods and boards | never touched | untouched |
| set down | in the slot, screen to +x, leaning 20.2 degrees, still, both hands home | for 1 s at the end |

**Scorer checks in simulation** (`demo.py --variant NAME`, the demo's code, each from the start state):

| Run | What the script does | Score |
|---|---|---|
| demo | as above | **1.0** |
| check `--wrist` | the same pinch, the sweep from 104 degrees turned by the wrist alone (the tool point fixed where it is when the lens is over the cross with the camera at 0 degrees) | 0.5 (all three boards in full; the lens 34.2, 6.1 and 7.5 mm off the sight lines, against T) |
| check `--plateau` | a pinch on the top edge over the camera plateau, the sweep turned by the wrist alone (the wrist's axis 4 mm from the lens) | 0.5 (aligned: the lens 1.8, 5.7 and 1.5 mm off the sight lines; but the camera corner touched, and the grip over the plateau rolls the camera 16 degrees, so not level either) |
| check `--button` | the phone lifted by its width 45 mm below the top (the pads on the volume and side buttons) | 0 (the squeeze presses the side and volume buttons at step 104: locked; the two taps later do nothing) |
| check `--fast` | the sweep at 60 degrees a second | 0.5 ("slow down": 70.6 degrees a second at the most; otherwise aligned and set down) |
| check `--short` | the stop tap with the camera at 40 degrees | 0.25 (boards 1 and 2 in full and aligned; the panorama ends at 13.9 degrees, short of board 3) |
| check `--nostop` | no stop tap; the phone turned back and put away | 0 (the capture ended unsaved when the phone turned back; the phone set down) |
| check `--keep` | the demo, the phone not put back (held at the end) | 0.75 (a perfect panorama: errors 0.16, 0.00 and 0.36 mm; the phone still in the hand at the end) |
| check `--drop` | the capture as in the demo, then the phone let go 80 mm above the mat | 0 (a perfect panorama saved, then the phone fell) |
| check `--nothing` | the arms stay still | 0 (no panorama; the phone stays in its stand) |

**Scorer checks without the simulator** (`python tests/test_scoring.py`, 34 of 34 pass): the Camera rule driven by
hand-built phone paths (taps and button forces placed by hand, the arms' hulls placed by IK):

| Case | Score |
|---|---|
| turned about the lens, set down / not set down / with the left arm's pinch carried along | 1.0 / 0.75 / 1.0 |
| turned by the wrist alone with the demo's pinch (the lens 42.3 mm from the axis), as `--wrist` | 0.5 |
| turned by the wrist alone with the lens 29.6, 35.5 or 42.3 mm from the axis, the axis at its best place | 0.5 each |
| the best wrist-only turn with any start that keeps board 1 in full and any end that keeps board 3 in full (lens 29.6 mm from the axis) | 11 mm or more off a sight line |
| a pinch over the camera plateau, wrist-only (aligned, but the camera corner touched) | 0.5 |
| a grip across the width at the top touches the camera corner | (corner touched) |
| the pivot point 5.5 mm / 6.5 mm off board 2's sight line (either side of T) | 1.0 / 0.5 |
| the lens 25 mm higher (height is not checked) | 1.0 |
| 60 degrees a second; tipped up 5 degrees; rising 50 mm | 0.5 each |
| stopped at 40 degrees / at 70 degrees; started at 90 degrees | 0.25 / 0 / 0.25 |
| no stop tap; turned back 10 degrees mid-capture | 0 / 0 |
| a 0.8 s touch on the shutter; a touch 8 mm from it; a touch sliding 8 mm off it | no capture |
| volume presses start and stop it | 1.0 |
| the side button pressed by a grip across the width | locked: 0 |
| a robot point in the first frame's left half / in a step's central strip / 1.7 m away | 0.5 / 0.5 / 1.0 |
| rod 1 touched | 0.5 |
| a two-board panorama, then a full one | 1.0 (the best counts) |
| a perfect panorama, then the phone dropped 60 mm | 0 |

Media: `demo.mp4` opens with the phone's panorama from the demo (what the run ends with), then shows the run in four
views: the top camera, a close camera on the phone over the cross (from the second demo run, identical but for the
top camera moved to that view), and the left and right arm cameras. It ends with the demo's whole panorama, the
`--wrist` check's, and each board's line between its ticks for both: hidden behind its rod in the demo, beside it in
the check. The panoramas are rendered from each run's logged lens path (see Simulation fidelity). `setup.png` is the
first top-camera frame; `closeup.png` shows the phone in its stand from the screen side (Pano open) and from the
camera side.

## Scoring

Judged on the phone's own output and the end state, read every control step (20 Hz):
- **In full:** a board's four sheet corners are in the panorama; in a real trial also its number appears once and its
  stripe is unbroken except behind the rod (no step or gap of 3 mm or more on the sheet at a seam). The simulation
  checks the sheet against the frames' coverage (below).
- **Aligned:** on every board, between the two ticks, no black sliver of the line 1 mm wide or more (on the sheet,
  seen at 100 % zoom) shows beside the rod; and the robot never touched a rod, its block, a sheet or a holder (in a
  real trial: a rod off its tape marks or out of plumb counts as touched). Only the camera's sideways distance from
  each board's sight line matters, not its height or its distance along the line.
- **Clean:** no Pano warning appeared during the capture.
- **Level:** the stripes' centres lie within 5 % of the image height of one another, none sloping more than 1 in 15.
- **Hands clear:** no robot part appears in the panorama, and no robot part touched the phone's camera corner (every
  outer face within 37 mm of the top and 35 mm of the camera-side edge: the camera plateau, the back glass round it and
  the top and side edges along it) during the capture.
- **Set down:** at the end the phone is back in the stand, screen facing +x, bottom edge on the seat, still for 1 s,
  both grippers 30 mm or more from it.

| Outcome | Score |
|---|---|
| the phone fell 50 mm or more with no gripper on it, or left the table | 0 |
| otherwise, from the best saved panorama: | |
| all three boards in full, aligned, clean, level and hands clear, and the phone set down | 1.0 |
| that panorama, the phone not set down | 0.75 |
| all three boards in full otherwise | 0.5 |
| exactly two boards in full | 0.25 |
| one board or none in full, no panorama saved, doing nothing | 0 |

**The rule in simulation** (the Camera app is not simulated; `task.py` stands in for it):
- A **tap** is a taped fingertip end face (the tape stops 2 mm short of the pad's edge) within 0.5 mm of the glass for
  0.5 s or less, the touch staying within 6 mm of the shutter's centre (on the centre line, 21 mm above the bottom
  edge). A hand's two fingertips make one touch. Other touches do nothing.
- A **press** is 3 N or more from the robot on a button band (per link touching the band: the PhysX contact force
  between it and the phone, or for a fingertip its squeeze from the finger's drive, whichever is larger):
  volume up and down on one edge (33.7 to 45.0 and 47.7 to 59.0 mm from the top), the side button on the other (37.4
  to 55.2 mm).
- A tap or a volume press starts a **capture**; the next one ends and saves it, as does a 240-degree turn. A
  side-button press locks the phone (a running capture is lost, and taps do nothing after). Turning back more than 5
  degrees ends a capture unsaved; one still running at the end is lost.
- During a capture: "slow down" if the heading turns faster than 40 degrees a second over 0.25 s; "off the line" if
  the view's tilt change plus the lens's height change seen at 0.45 m passes 5 degrees; both are warnings. Level: the
  optical axis within 3 degrees of horizontal and the roll within 4 at every step.
- A saved capture covers from its first frame's left edge to its last frame's right edge (frames 52 by 60 degrees). A
  board is in full when its sheet lies inside that and inside the height band all frames share.
- Each line is judged from the frame that puts it into the panorama, as Pano stitches: the first frame for a line left
  of that frame's centre, the last frame for a line right of its centre, otherwise the frame whose centre crosses it.
  There the lens must be within T = 6 mm of the board's sight line, sideways.
- Hands clear fails on any robot contact with the camera corner during the capture, or any point of the arms'
  collision hulls within 1.5 m inside the first frame's left half, a step's central strip (at least 1 degree either
  side) or the last frame's right half.

Which arm holds or taps, and the speed under the warning, don't matter.

**Why a turn of the wrist alone fails.** A grip off the camera corner and the buttons is a pinch on the top edge 35 mm
or more from the camera side (or a grip across the width, which touches the corner at the top or presses the side
button lower down), so the lens is at least 29.6 mm to the side of the wrist's axis. Turned by the wrist alone, the
lens swings round that axis. With the axis anywhere, any start that keeps board 1 in full and any end that keeps board
3 in full, it stays 11 mm or more off some board's sight line (`tests/test_scoring.py`), against T allowed. A pinch
over the camera plateau puts the axis 4 mm from the lens and aligns, but it touches the camera corner (`--plateau`).

**Real trial.** From the phone's own output, right after the run:
1. **Panorama:** open the trial's panorama in Photos; check each board's four corners, its number once, its stripe
   (no step or gap of 3 mm or more at a seam); zoom to 100 % on each line between the ticks and look for a black
   sliver beside the rod.
2. **Warnings:** step through the screen recording for any Pano warning during the capture.
3. **Level:** measure the stripes' centres in the panorama.
4. **Hands clear and set down:** the trial video (no finger on the camera corner during the capture; the phone in
   the stand at the end, both grippers 30 mm or more from it); no robot part in the panorama.
5. **Rods:** each block on its tape marks, each rod plumb by the torpedo level.

## Real-world build

Positions in mm from the midpoint between the arm bases, x forward, y to the robot's left; heights above the table.

| Item | Details | Price |
|---|---|---|
| Phone | Apple iPhone 12, 64 GB, white, no case (refurbished). Camera open in Pano, arrow pointing right (the default); Preserve Settings > Camera Mode on; Auto-Lock Never; flash off; record the iOS version; an on-phone screen recording running during trials | about $150 |
| Stand | aluminium desk phone stand, Lamicall type: base 90 x 80, back plate reclined 20 degrees, seat 30 up and 12 deep with an 8 mm lip; base centred at (340, 250), taped down, facing +x | about $12 |
| Mat and cross | black cloth mouse pad, 300 (x) by 400 (y), 3 thick, centred at (330, 170), taped down; a white tape cross (two 20 x 3 strips) centred at (300, 120) | about $8 |
| Boards | three A4 prints (`assets/print`, at 100 %) in acrylic A4 T-base sign holders (sheet bottom 25 up), taped down, each face square to the cross with its centre 450 from it: board 1 at (161, 548), board 2 at (629, 427), board 3 at (717, -49) (bearings 108, 43 and -22 degrees). Each print: a 5 mm black vertical centre line with two 30 mm ticks across it 155 and 215 above the table, a 20 mm coloured stripe centred 185 above the table, the board's number upper left on red, green or blue | about $20 and printing |
| Rods | three straight 6 mm steel round rods, 300 long, upright in drilled 50 x 50 x 18 wooden blocks, taped down with tape marks round each, on each board's sight line 300 from the cross: (207, 405), (519, 325), (578, 8); plumb by a torpedo level both ways | about $15 |
| Sighting jig | a 20 x 20 x 200 mm post on a 100 x 100 mm base with a centre mark; a card with a 1 mm pinhole 185 up on the post's centre line. Set on the cross, it sets each rod so its line between the ticks is centred behind it | about $3 |
| Backdrop | a plain light wall or hung sheet 1.5 m or more behind the boards; 300 lux or more | |
| Touch | conductive fabric tape on each fingertip's end face, stopping 2 mm short of the pad's edge, wired to the robot's grounded frame; pads plain rubber | about $10 |

About $220 and an hour to set up. Nothing is sharp or heavy: the phone is 164 g, and the arms move slowly near the
boards (the left arm's collision hulls pass 77 mm or more from them and their holders along the demo's path).

## Simulation fidelity

Rigid PhysX, 240 Hz physics (12 substeps per 20 Hz control step), GPU dynamics.
- **Phone:** one rigid body at Apple's drawing size (146.71 x 71.52 x 7.39 mm, the outline's rounded corners) and
  164 g, with its camera plateau (0.73 mm proud) and lens rings (1.51 mm proud) as colliders; its screen shows a still
  of the Camera app in Pano. The buttons are drawn on its edges.
- **Fixtures:** one fixed body: the mat and cross, the stand (base, arm, back plate, seat, lip), the three boards in
  their holders, and the rods in their blocks.
- **The Camera app** is not simulated: the rule above reads the phone's pose, the taped end faces' places against the
  glass (the arms' collision hulls at their link poses) and the robot's contact forces on the phone each step.
- **Estimates** (values under Scoring): the "slow down" and "off the line" warnings, the frame's size, the shutter's
  place and size (about 12 mm across), the press force and T; and the frictions, 0.5.
- **The panorama pictures** in the media are rendered after the run by ray casting the scene's geometry (table, mat,
  stand, boards with their prints, rods, blocks, a light backdrop and the arms' collision hulls) from the lens's
  logged path: a cylindrical panorama laid out by heading, each part cast from the lens of the frame that holds it in
  the panorama (the first frame's left half, each step's central strip blended with its neighbours, the last frame's
  right half). They are not Pano's own blending.
- **Demo:** the scripted demo reads object poses from the simulator (privileged); a model under test sees only the
  cameras.

## Reset between trials

Stop and keep the screen recording; check, then delete the trial's panoramas in Photos. Reopen Camera (it stays in
Pano), check the arrow points right, start a new screen recording, and stand the phone back in its stand, screen facing
+x. Check the tape marks of the stand, boards and rod blocks and the rods' plumb; re-set a moved rod with the sighting
jig. Nothing is used up. About 1 minute.

## Sources and licences

- Google, "4 tips from a photographer on using Pixel's new panorama mode" (1 May 2025):
  https://blog.google/products/pixel/tips-how-to-use-pixel-new-panorama-mode/
- Apple, iPhone User Guide, "Take panoramic photos with your iPhone camera":
  https://support.apple.com/guide/iphone/take-panoramic-photos-iph7e06402b4/ios
- Apple, iPhone 12 dimensional drawing (2020-10-16), Accessory Design Guidelines:
  https://developer.apple.com/download/files/accessories/dimensional-drawings/iphone-12.pdf (note 3: "Do not
  obstruct the imaging features: front camera, rear cameras, rear flash"; the lens, plateau and button positions)
- Apple, iPhone 12 technical specifications: https://support.apple.com/en-us/111876
- Fstoppers, "A Beginners Guide to Parallax and How to Avoid It When Shooting Panoramas".
- Licences: every asset is built from scratch by `build_assets.py` (no third-party meshes, scans or textures). The
  boards' digits and the screen's labels use the DejaVu Sans fonts (DejaVu fonts licence: free to use, embed and
  redistribute). The robot, its collision hulls and the station scene are yam_arena's.
