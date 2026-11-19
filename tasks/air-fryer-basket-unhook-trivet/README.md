# Lift a hot fry basket out of its outer basket by the handle's guarded release button and set it on a heat-proof mat

## Instruction

Lift the fry basket out of its outer basket: slide the button guard forward, hold the release button on the handle while lifting by the grip, set the fry basket flat on the mat, and let go of everything; touch nothing hot.

## Context

Taking the fry basket out of a basket-type air fryer is an everyday kitchen chore and a classic burn risk: everything but the cool-touch handle is hot. Source: 5.3 qt Air Fryer model CM1708 owner's manual (QVC item K46295): 'Move Sliding Button Guard forward to expose Basket Release Button. Press the Basket Release Button (top photo) and lift the Fry Basket out of the Outer Basket.' and 'Do not move Sliding Button Guard forward until Fry Basket and Outer Basket are resting on a level, heat-resistant surface.' Here the outer basket rests on a trivet; one hand works the guard and holds the button, as the thumb does, while the other lifts by the grip and sets the fry basket on a mat, touching nothing hot. Closest cards: TK-F55KZM (pot off a lit burner), TK-5CPBS2 (battery release buttons). New: a guarded release held while lifting; hot keep-out on every finger.

## Setup

Positions are object centres in cm from the midpoint between the arm bases (x forward, y left), z above the bench (smooth surface, friction 0.5). Modelled on the 5.3 qt Air Fryer model CM1708 (owner's manual for QVC item K46295: fry basket handle, Basket Release Button, Sliding Button Guard). Material as modelled: rigid bodies with only the size, mass and friction stated here; appliance body and outer basket painted steel, fry basket steel with a rubber-coated plastic handle, lever ABS and guard clear polycarbonate, trivet bamboo, mat silicone. Appliance body (fixed; HOT, not heated: shown by colour, a lit amber lamp and four glowing vent tiles): 26 x 26 x 28 cm on a 3 cm bamboo board at (70, 0), its drawer bay empty. Trivet (fixed, bamboo, not hot): 26 x 26 x 1 cm at (35, 3.5). Outer basket (loose painted-steel pan, 600 g, friction 1.0; HOT): 24 x 22.4 cm at (35, 3.5) on the trivet (floor z 1.0 to 1.6), walls to z 7.6, front wall x 23.0 to 23.6, with a catch lip 0.5 x 2.4 x 0.8 cm on the outside of its front wall (x 22.5 to 23.0, y 2.3 to 4.7, z 5.5 to 6.3). Fry basket (loose, 250 g, friction 1.5 for the whole body, standing for the manual's 'Large Easy-Grip Handle', rubber-coated; walls and floor HOT): floor 17.8 x 20.4 cm (x 23.7 to 41.5) on the outer basket's floor, walls to z 8.6, 1 mm inside the outer basket's front wall and 49 mm of play toward its back. Handle (cool-touch, on the fry basket), an inverted L as on the CM1708: an arm 2.4 cm wide from the rim out to x 16.75 (y 2.3 to 4.7, z 9.1 to 11.1) and a vertical grip 2.5 x 2.4 x 8 cm hanging from its outer end (x 16.75 to 19.25, z 1.1 to 9.1); the arm's inner 2.2 cm is a slot between two rails. Basket Release Button: a sprung lever (15 g) hinged across the slot at x 22.5, z 10.3, its thumb pad 2.0 x 2.2 cm on top of the handle where it meets the rim (x 24.5 to 26.5, z 12.7 to 13.3), 7.5 cm from the grip. Its tooth hangs straight down from the pivot outside the outer basket's front wall; its foot sits 4.5 mm under the catch lip and 1 mm below it, so lifting the handle lifts both baskets. Pressing the pad turns the lever (0.35 N m/rad, stop 0.28 rad); from 0.09 rad (pad 3 mm down, 1 N) the foot is clear of the lip and the fry basket lifts out alone; hook friction 0.2 (smooth plastic). Sliding Button Guard (8 g, slide in +x, 6 cm travel, damped 3 N s/m, no spring): a clear 3 x 2.8 cm cap 2 mm above the pad (z 13.5 to 13.9) with a thumb tab at its back edge (x 23.6 to 24.0, z 13.5 to 15.1); closed at the start, so a press lands on the guard; slid forward it leaves the pad open. Mat (fixed silicone, friction 1.2): 30 x 30 x 1 cm at (41.4, 33.5). Both arms start open at ready poses. Reference appliance: the CM1708 (fry basket and outer basket on a trivet, as its manual requires). Real CM1708 handle, measured from the manual's front-view photo scaled by the published 15 x 14.4 in unit size (the two scales differ by about 20 %): 16 to 19 cm from head top to grip end, its face 4 to 5 cm wide; a button about 2 cm square at the head's outer end, directly above the grip; the clear guard slides about one button length toward the basket. Two downward grippers cannot press that button while holding the grip below it, so the robot trial uses a printed handle, lever and guard built to the dimensions above on a CM1708-sized basket and pan; its Large Easy-Grip Handle is rubber-coated (about 1.2 to 1.5 on a gripper pad).

## Scoring

End state plus latched facts. PADS = four 12 mm boxes along each finger (tool point, fingertip, 3 and 6 cm up), eight per hand, at the measured finger opening. HOT_CONTACT = at any step a pad box overlaps any of the 16 HOT boxes (appliance body, outer basket with its lip, fry basket walls and floor; inflated 2 mm). PRESSED = lever turned 0.13 rad or more (the button pushed down 4.4 mm at the press point; the hook clears the lip from 3.1 mm). RELEASE_IN_AIR = PRESSED at a step when the outer basket is 1 cm or more off the trivet (the manual's caution: separating the baskets anywhere but on a level, heat-resistant surface drops the hot outer basket). UNLATCHED = the fry basket rises 12 mm out of the outer basket (still over it, outer basket on its trivet) with the lever never PRESSED. DROPPED = either basket falls more than 5 cm within 0.25 s. ON_MAT = all four floor corners inside the mat (x 26.4 to 56.4, y 18.5 to 48.5), underside within 5 mm of the mat top (z 1.0), tilt < 5 deg. PAN_HOME = outer basket within 1.5 cm of its start, 5 mm in height, tilt < 5 deg. LEVER_OUT = lever at most 0.02 rad (button within 0.7 mm of rest). RELEASED = both gripper centres and every pad box >= 3 cm from the fry basket, handle, lever and guard. STILL = fry basket moves < 1 mm in the last 1 s. Permanent zero: HOT_CONTACT, RELEASE_IN_AIR, UNLATCHED or DROPPED. 1.0: ON_MAT, PAN_HOME, LEVER_OUT, RELEASED, STILL. 0.75: ON_MAT and PAN_HOME, but LEVER_OUT, RELEASED or STILL fails. 0.5: PAN_HOME, the fry basket out of the outer basket (risen 7 cm at some step), ending upright (tilt < 20 deg) on the bench 12 cm or more from it, not on the mat. 0.25: PRESSED while the fry basket was 2 cm or more out, and it ends seated back in the outer basket (within 1 cm, tilt < 5 deg). Else 0. Gripper orientation: body_quat_w is read as (x, y, z, w), verified on the GPU against the simulator's own finger links (tip_left, tip_right): the finger links placed from it lie 0.0 mm from the reported positions on every step of the demo and the check run (1,395 steps, both arms, wrists turned up to 90 degrees); read as (w, x, y, z) they are 4.8 cm or more off (run.sh posecheck logs this as tip_err_mm). Real trial: ON_MAT = the basket's whole round base lies inside the mat outline and sits flat on it (no gap under its rim; tilt < 5 deg); PRESSED = button down 4.4 mm or more (a gauge mark at 4.4 mm), seen as the fry basket lifting free of the outer basket; LEVER_OUT = button back up flush; PAN_HOME = outer basket within 1.5 cm of a tape mark; HOT_CONTACT = any finger touching a hot surface on the overhead and wrist video. Possible scores: 0.0, 0.25, 0.5, 0.75, 1.0.

## Fidelity

Baskets square (CM1708 round), empty; not heated (heat shown by colour, lamp and tiles; scored on 12 mm finger boxes). Grip-to-button 7.5 cm and guard travel 6 cm, larger than the real handle (button above the grip, about 2 cm slide): a gripper's fingers are 1.5 to 7 cm wide and move only straight down, so one cannot press a button above a grip the other holds; a thumb reaches it from the side. Latch: one sprung lever under one lip; outer basket walls 6 cm.

## Reset

Outer basket on the trivet with the fry basket seated and latched, lever at rest, button guard closed, lamp and vent tiles lit, both grippers open and clear at ready.

## Scores

Category: Societal relatability

Dream-1 run: 0.0 (1,500 steps; guard and button never touched, fry basket stayed latched). Scripted demo: 1.0 (full at step 736). Right hand idle: 0.0 (left hand carried both baskets; the latch gave way in the air). Guard left closed: 0.0 (press landed on the guard; same).

## Execution

```bash
bash run.sh settle     # 8 s hold at ready pose (200 steps)
bash run.sh demo       # scripted two-arm demo, 1,500 steps
bash run.sh ablation   # the right arm stays idle: lifting the grip carries both baskets on the latch until it gives way in the air (scores 0); 900 steps
bash run.sh noguard    # the right arm presses without sliding the guard: the press lands on the guard, the latch holds and both baskets are carried (scores 0); 900 steps
bash run.sh posecheck  # check run: both wrists turned 0 to 90 degrees; the scorer logs the simulator's finger link positions against those predicted from the gripper quaternion (trace.jsonl.pads.jsonl, tip_err_mm); ends on a deliberate hot touch, so it scores 0; 640 steps
bash run.sh dream      # Dream-1 policy, 1,500 steps
python -m pytest tests # offline unit tests (35), no simulator needed (yam_arena installed, or its source named by YAM_ARENA_SRC)
```

Run limit: 1,500 control steps (75 seconds at 20 Hz). The scripted plan lasts 40 s (800 control steps at 20 Hz); the demo reaches full score at step 736 and the episode ends at step 755 (37.8 s), when the full-score state has held for 1 s. Object jitter is disabled (object_jitter=0), so there are no random seeds.

## Why the handle spacing differs from the CM1708

On the real CM1708 the release button sits on the handle head directly above the grip, and the clear guard slides about 2 cm.
A person presses it with the thumb, reaching from the side, while the fingers hold the grip. The station's grippers move only
straight down, and their geometry (from the robot's collision meshes, fingers closed on a 4 cm grip) rules out the real spacing:

- Two grippers: the gripper holding the grip comes down over the handle, so its body (8.3 cm above its fingertips, 6.5 cm
  across) covers the head where the other gripper would press.
- One gripper pressing with its body: within 15 mm of its centre line, the underside is 83 mm above the fingertips for only
  2.4 cm along the handle, and 74 mm beyond that. A flat real-size handle head therefore stops the gripper 9 mm before the
  button could move.

The simulated handle keeps the CM1708 layout (inverted-L handle, thumb button on top at the rim, sliding guard) but sets the
button 7.5 cm from the grip and the guard travel to 6 cm, the smallest that the two grippers can work. For the real trial, the
handle, lever and guard are printed to the simulated dimensions and fitted to a CM1708-sized basket and pan.
