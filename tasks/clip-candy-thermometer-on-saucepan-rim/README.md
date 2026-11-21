# Clip a candy thermometer onto a saucepan's rim, probe hanging clear of the floor, while the other hand holds the pan

Holding the saucepan by its handle, clip the candy thermometer onto its rim so it hangs inside, dial up, tip clear of the floor.

## Scoring

Judged at the end of the trial from the top-camera video, a side photo and a ruler (in simulation from the poses). Hooked: the pan's rim is between the clip's two legs (one leg inside the pan, one outside) and the clip's top bend is no more than 16 mm above the rim. Bend height: from the rim's top to the underside of the clip's top bend, ruler, side photo. Tip gap: the tip's height above the inside floor, read as 92 mm + bend height - (152 mm - L), where L is the stem's length from the dial's back to the clip's top bend (ruler); a tip that touches the floor is seen directly. Pan shift: the base's largest offset from its tape outline. Tilt: the stem's angle from vertical (phone level on the dial's face). Score 0 if: the thermometer is not hooked on the rim, or a gripper still touches it (any gripper within 3 cm of it), or its tip is outside the pan or within 1 mm of the floor (touching), or the pan moved more than 30 mm, tipped more than 20 degrees, or anything left the bench. Doing nothing scores 0. Otherwise the score is the lowest of: bend height (1.0 up to 3 mm; 0.5 over 3 to 10 mm; 0.25 over 10 mm); tip gap (1.0 from 10 mm; 0.5 from 5 to under 10 mm; 0.25 from 1 to under 5 mm); pan shift (1.0 up to 10 mm; 0.75 over 10 to 20 mm; 0.5 over 20 to 30 mm); tilt (1.0 up to 10 degrees; 0.5 over 10 to 20; 0.25 over 20). A trial whose thermometer is still moving (more than 1 mm or 2 degrees in its last second) scores at most 0.75.

How each check is made on the real rig: hooked, bend height and tilt from a side photo with a ruler and a phone level
held on the dial's face; the tip gap from one ruler reading along the stem (the clip's position on the stem fixes how
far the tip hangs below the rim), or seen directly if the tip touches; pan shift against the tape outline round the
pan's base. Nothing needs a force reading or a sub-millimetre measurement. In simulation the same checks are read
from the poses: the point under the middle of the clip's top bend lies within 4 mm (radially) of the rim's middle
and 16 mm or less above the rim's top (hooked); the tip is inside the pan's inner wall and its height above the
inside floor is the tip gap; the stem's axis against vertical; the pan's base centre against its start; every
gripper's TCP at least 30 mm from the thermometer's bounding box; still = moved under 1 mm and turned under 2 degrees
in the last 1 s. An object below the bench top at any time makes the score 0 for the rest of the trial.

Our recorded demonstration (no object jitter) scores 1.0 and ends by success at step 477 (23.9 s). The right hand holds the handle; the left hand takes the thermometer by its dial, turns it to hang, carries it over the rim, lines the clip's mouth up with the rim and pushes it on along the wall's lean until the bend sits on the rim (0.0 mm), then lets go and backs away. At the end the tip hangs 18.3 mm off the floor, the stem is 1.4 degrees off vertical, the pan has not moved and the stem has slipped 1.3 mm through the clip. With object_jitter 0.01 the same demonstration scored 1.0 in 3 of 3 trials.

Our wrong-way run stops pushing with the clip's bend 12 mm over the rim and lets go: the clip, resting on its lead-ins, does not hold, and the thermometer falls into the pan, leaning 50 degrees with its tip on the floor. Score 0.0 (not hooked).

## Run and score

From this task folder, with the official YAM Arena environment installed:

```sh
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=clip_candy_thermometer_on_saucepan_rim.py -E object_jitter=0 -E control_hz=20 \
  --instruction "Holding the saucepan by its handle, clip the candy thermometer onto its rim so it hangs inside, dial up, tip clear of the floor." \
  --scorer task_score --max-steps 2400 --epochs 1 --store-frames --log-dir ./demo-run
```

For a model trial use `--policy dream`. Each trial allows 2,400 control steps (120 s at 20 Hz). Our shifted-object
checks used object_jitter=0.01. `pytest` here checks the scorer on named layouts.

## What the robot has to do

One hand holds the light pan by its handle. The other turns the lying thermometer to hang stem-down, lifts it in
over the rim (the dial about 27 cm up), lowers it so the clip's U straddles the 2.55 mm rim and pushes it on along
the wall's 4.7 degree lean (straight down, the inner leg would meet the wall's top) until the bend sits on the rim,
then lets go. The push needs a few newtons; past about 20 N the stem slides down through the clip until the tip
rests on the floor, where a real thermometer reads the hot metal instead of the food.

## Real objects (sources fetched; nothing here was measured)

- Saucepan: Vollrath 682115 Wear-Ever 1.5 qt tapered aluminium sauce pan, natural finish, black silicone handle,
  NSF. Maker's page (diameter 6.219 in, usable diameter 6.018 in, base diameter 5.594 in, height 4.21 in, usable
  height 3.625 in, depth with handle 13.16 in, 11 gauge 3004 aluminium, double-thick rim and bottom, beadless):
  https://www.vollrathfoodservice.com/products/smallwares/cookware-bakeware/wear-ever-cookware/wear-ever-sauce-pans/682115 .
  Retailer (top 6 in, bottom 5 1/4 in, 4 in high, about 31 US dollars):
  https://www.webstaurantstore.com/vollrath-wear-ever-1-5-qt-tapered-aluminum-sauce-pan-with-black-silicone-handle-682115/922682115.html .
- Thermometer: Taylor 3505 TruTemp candy/deep-fry thermometer, 2 in dial, 6 in stainless stem, stainless pan clip,
  100-380 F, NSF (about 5-15 US dollars):
  https://www.webstaurantstore.com/taylor-3505-trutemp-6-candy-and-deep-fry-thermometer/913CD400K.html ;
  https://www.hammondhardware.com/p/taylor-trutemp-candy-deep-fryer-kitchen-thermometer-077784035054 . The
  retailers' photos show the dial's face square to the stem and the clip on a sleeve on the stem.
- Also: masking tape for the pan's outline, a ruler. Everything costs under 60 US dollars.
- Occupation: O*NET 35-2014.00 Cooks, Restaurant (https://www.onetonline.org/link/details/35-2014.00), core task
  "Ensure food is stored and cooked at correct temperature by regulating temperature of ovens, broilers, grills,
  and roasters"; about 1.4 million US restaurant cooks (BLS OEWS 35-2014).

## Estimates (not published; how to check each in a minute)

Pan: wall 2.55 mm (half the difference of the maker's two diameters; calipers), bottom 4.6 mm, handle 24 x 16 mm
rising to 107 mm, riveted bracket, 0.55 kg (ruler, kitchen scale). Thermometer: stem 4.8 mm across, dial case 13 mm
deep (14 mm with lens and needle), 40 g without the clip (ruler, kitchen scale); clip 12 mm wide, 0.6 mm strip
(1 mm colliders), top bend 14 mm out from the stem's axis, inner leg 16 mm with a 5 mm lead-in bent 35 degrees
toward the stem, outer leg 14 mm with an 8 mm flare bent 40 degrees outward, legs 4.5 mm apart just under the
bend (a loop wider than the rim) closing to 2.2 mm at the outer leg's end, 5.5 g
(ruler); its grip on the rim about 1 N (pull the seated clip up with a luggage scale); the stem slides through
the clip's sleeve past about 20 N, 16 N once moving (hang a 2 kg weight from the dial while holding the clip). Frictions:
stainless 0.35, aluminium 0.40.

## Simulation model

- Units metres and kilograms; x forward from the arm bases' midpoint, y left, z up; bench top as in the station.
- `saucepan` (free, 0.55 kg): 64 tapered convex wall pieces (outer radius 71.0 mm at the bench to 79.0 mm at the
  rim, 2.55 mm thick, 96.7 mm tall), a 4.6 mm floor disc, the handle as a sloped convex box (24 x 16 mm, from 81 to
  255 mm out, middle 84 to 99 mm high) on a convex bracket, two rivet heads inside the wall. Contact offset 1 mm.
- `thermometer` (free mechanism): base = the clip (4 g: sleeve, top bend, inner leg and its lead-in); link `stem`
  (40 g: stem cylinder 2.4 x 152.4 mm, dial case 25.4 x 13 mm, face, needle, blue marker) on a prismatic joint along
  the stem (-60 to +40 mm, joint static friction 20 N, dynamic 16 N, damping 0.5, armature 0.005 kg); link `leaf`
  (1.5 g: outer leg and flare) on a revolute joint about the clip's width at the top bend's outer corner (-25 to 0
  degrees, drive stiffness 0.11 N m/rad toward +6 degrees, so preloaded shut against its stop; armature 2e-6 kg
  m2). Each leg and its bent end is one convex collider. The clip is the base so that the body carrying both joints
  is the articulation's root. Contact offset 1 mm.
  It starts lying with the stem level; it settles onto the dial's rim and the clip's flare. The joint friction is
  not perfectly static: in our runs the stem crept about 1.3 mm through the clip during the push and hold.
- The demo reads the objects' poses (allowed for scripted demos) to line its hand up; it never moves an object. The
  run-context and waypoint helpers follow the task set-toilet-closet-bolts-in-the-flange-slots.

## Fidelity

Rigid bodies. The clip's spring is its outer leg on a hinge with a preloaded spring (about 1 N on the rim); its sleeve's grip on the stem is a slide that moves only past about 20 N (16 N moving); the clip cannot turn on the stem. The pan's wall is 64 convex pieces at the maker's sizes; the stem's tip is flat. Empty, cold pan: no liquid or heat. Clip shape, spring, grip, masses and frictions are estimates.

Reset: see task.json.
