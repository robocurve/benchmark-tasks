# Slide a handheld mic into its clip on a light desk stand, holding the stand, and aim it at a marked spot

Turn the stand's clip toward the red circle. Holding the stand, slide the mic tail first into the clip until snug, then let go.

## Scoring

Judged at the end of the run from the top-camera video, a top photo, a side photo and a ruler (in simulation from the poses). Score 0 if the stand tipped over at any time (its tube more than 20 degrees from upright), if the mic is not held in the clip (its handle is not inside the clip's jaws, it touches the table, or it fell off), if a gripper is still within 3 cm of the mic or the stand's tube, or if the mic moved more than 1 mm in its last second. Otherwise the score is the lowest of these four numbers. Seat: 1.0 if at least 8 mm of handle sticks out behind the clip's back face (the taper is wedged snug), else 0.5. Aim, on the top photo: extend the mic's axis from its tail end through the grille's centre and measure the shortest distance from the red circle's centre to that line, on the grille side: 1.0 up to 30 mm, 0.75 over 30 up to 60 mm, 0.5 over 60 up to 100 mm, 0.25 over 100 mm. Stand: its tube's centre within 20 mm of the taped cross 1.0, over 20 up to 50 mm 0.75, over 50 mm 0.5. Tilt, on the side photo: the mic's axis within 12 degrees of level 1.0, otherwise 0.5. Doing nothing scores 0.

How each check is made on the real rig: the side photo shows the handle inside the clip's C with the jaws closed
round it; a ruler laid along the handle reads how far it sticks out behind the clip's back face (8 mm or more means
the taper is wedged snug in the A25D, which grips 25-30 mm barrels); on the top photo a straightedge laid along the
mic from the middle of its tail end through the grille's centre gives the aim line, and a ruler gives the circle
centre's distance from it; a ruler from the taped cross to the tube's centre gives the stand's shift; a phone level
or protractor on the mic in the side photo gives its tilt. No force or sub-millimetre reading is needed. In simulation the same checks are read from the poses: the clip's cradle centre within 5 mm of the
mic's axis and the two axes within 18 degrees (in the clip; beyond that the mic has levered out of the C), the cradle centre's distance from the tail end (seat),
the miss distance of the axis' top-view line from the circle's centre (aim), the stand's root position (shift), the
mic axis' elevation (tilt), the tube's lean (tipped if over 20 degrees at any time), every gripper's TCP at least 30
mm from the mic's box and the tube, and the mic moved under 1 mm and turned under 2 degrees over the last 1 s.

In our recorded demonstration (no object jitter) the left hand takes the stand by its tube, lifts it, turns it 39 degrees and sets it down 4 mm from its spot, then holds it; the right hand picks up the mic near its balance point, carries it to the clip's front end, slides it in tail first and lets go; full credit at 29.0 s. At the end 12.1 mm of handle sticks out behind the clip, the mic's axis passes 7 mm from the circle's centre, it droops 8.8 degrees in the clip (the head-heavy mic tips the jaws a little once let go), the stand is upright. Three trials with every object shifted up to 10 mm: all three reached full credit.

Wrong way, tried in the simulator: the stand turned the same way, then let go and the mic slid in with the stand unheld. As the taper wedged, the stand tipped away from the push (one foot off the table, 3.7 degrees), the mic stopped with 7.1 mm of handle out behind the clip and the stand dropped back when the hand let go: 0.5 (not snug).

## Run and score

From this task folder, with the official YAM Arena environment installed:

```sh
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=slide_a_mic_into_its_desk_stand_clip_and_aim_it.py -E object_jitter=0 -E control_hz=20 \
  --instruction "Turn the stand's clip toward the red circle. Holding the stand, slide the mic tail first into the clip until snug, then let go." \
  --scorer task_score --max-steps 2400 --epochs 1 --store-frames --log-dir ./demo-run
```

For a model trial use `--policy dream`. Each trial allows 2,400 control steps (120 s at 20 Hz). Shifted-object
checks used object_jitter=0.01. `pytest` checks the scorer on named layouts.

## What the robot has to do

The stand weighs 363 g on three rubber feet and the mic is pushed in about 19 cm above the table, so a push of a few
newtons tips or slides it unless the other hand holds it: one hand holds the stand's tube below the clutch, the other
slides the mic in (the way the clip is fitted by hand: hold the clip or stand with one hand, push the mic with the
other). The stand is turned first: lifted a little by its tube and turned 40 degrees so the clip points at
the circle (turning it on its feet would need more twist than a pinch gives). The mic is picked up near its balance
point (it is head-heavy), carried so its tail end is on the clip's axis, put into the clip's open front end and slid
tail first until about 10 mm of handle sticks out behind the clip and the taper is wedged; then both hands let go. The arms work about 12 cm apart at the
closest: the left hand holds the tube low and from behind; the right works at the clip's front.

## Real objects (sources fetched; nothing here was measured)

- Mic: Shure SM58 (SM58-LC). Shure specification sheet: net weight 298 g, 162 mm long x 51 mm wide; accessories list
  the A25D microphone clip:
  https://www.talamas.com/sites/default/files/sales/Shure/application/pdf/sm58-specification-sheet-english.pdf .
  Retailer listing (head 51 mm, shank 29/32 in = 23 mm, 162 mm, 298 g; includes one A25D swivel stand adapter):
  https://goknight.com/shure-sm58-cn-cardioid-dynamic-vocal-microphone-with-25-xlr-cable/ . Shure user guide
  (stand adapter "which rotates 180"): https://www.fullcompass.com/common/files/1625-ShureSM58UserGuide.pdf .
- Clip: Shure A25D, supplied with the SM58: fits a 3/4 in barrel, typically 25-30 mm; flared top, used both snap-in
  and slide-in; insertion can be stiff: https://equipboard.com/items/shure-a25d-microphone-clip .
- Stand: On-Stage DS7425 Tripod Desktop Mic Stand: height adjusts 4.3-6.7 in, 7.5 in tripod base, non-slip rubber
  feet, includes a tapered mic clip (not used here), 29.95 US dollars: https://on-stage.com/products/view/51337 .
  Listed weight 12.8 oz (363 g) on its Amazon listing (B0035LCFRC).
- Red circle: a 60 mm red paper disc taped flat. Everything costs under 150 US dollars.

## Estimates (not published; how to check each in a minute)

A25D: bore 25.4 mm across at rest, 36 mm long, top opening 21.5 mm, swivel axis 20 mm and mic axis 41 mm above the
stand's thread, slide-in push about 4-6 N (stand the clip upright on a kitchen scale and push the mic in tail first:
the reading rises by the push). SM58 handle 26.4 mm across at 60 mm
from the tail and 34 mm at 112 mm (calipers or a ruler); its balance point (balance it on a finger: about 100 mm from
the tail in the model). DS7425 tube 19 mm, clutch 28 mm at 125-139 mm, hub 34 mm, feet on a 180 mm circle, mass split
(ruler, kitchen scale). Frictions: rubber feet 0.8, clip plastic 0.35, mic paint 0.4.

## Simulation model

- Units metres and kilograms; x forward from the arm bases' midpoint, y left, z up; table top z = 0.7475.
- `stand` (free mechanism): base (328 g; the articulation root, it carries all three joints) = three convex
  die-cast legs, rubber feet, hub, tube and clutch; link `head` (inner shaft, the A25D's threaded boss, swivel ears and
  body, 55 g) on a vertical slide held by the height clutch (static friction 40 N, dynamic 30 N); links `jaw_l`,
  `jaw_r` (6 g each) hinged along the cradle's bottom (their hinge lines sit on the clip body while the clutch holds),
  spring 6 N m/rad with 0.5 degree of preload against the closed stop, 15 degrees of travel, armature 1e-4 kg m2,
  damping 0.02; each jaw is five convex wall pieces plus a flared front piece. The swivel's thumb nut is tight, so the
  clip is fixed level.
- `mic` (free, 298 g, uniform density): two convex 20-sided handle tapers, a grille cup and a 51 mm ball.
- `speaker_mark` (fixed): the 60 mm red disc, 1 mm thick.
- Colours as sold: a black stand with a chrome tube; the mic's handle dark grey, its grille silver.
- The demo reads the stand's and the mic's poses (allowed for scripted demos) to line its hands up; it never moves an
  object. The left hand points forward over the bench, dipped 15 degrees; the right hand leans 20 degrees from straight
  down and is turned 4 degrees so one pad lies along the handle's taper.
- Credit: the run-context, gripper-reading and `ever` helpers and the grip-matrix patch in the module are adapted from
  the task folder set-toilet-closet-bolts-in-the-flange-slots.

## Fidelity

Rigid bodies. The clip's flexing plastic C is two hinged jaws on a stiff spring with a small preload (estimated so the slide-in takes about 4-6 N, under the arm's limits); the swivel is tight, so the clip is fixed level; the height clutch holds by joint friction (40 N, estimate). The mic's handle is two straight tapers and its grille a ball, of uniform density. Clip, tube, leg and taper sizes, frictions and the stand's weight split are estimates; the clip's front 6 mm is a flared lead-in.

## Reset

About 1 minute. Holding the stand, pull the mic out of the clip toward its grille and lay it back at its start spot on its grille and tail end, grille toward the robot's right. Stand the stand back on its taped cross turned to heading -20 degrees (the taped line under its leg), and check the clip is still level (phone level along it); retighten the thumb nut if the swivel moved. The A25D's jaws loosen with use: if the mic slides out of the clip under its own weight with the grille tipped down once 8 mm of handle sticks out, fit a new A25D. Return both arms to the ready pose.
