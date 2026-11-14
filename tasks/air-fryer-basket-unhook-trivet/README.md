# Take the fry basket out of a hot air fryer's drawer by its release button and handle, and set it on a heat-proof mat

## Instruction

Take the basket out of the air fryer drawer by its release button and handle, set it flat on the mat, and let go of everything; touch nothing hot.

## Context

Taking the basket out of a basket-type air fryer is an everyday household chore and a classic burn risk: after every cook the basket, drawer and housing are too hot to touch except the cool-touch handle. The basket is locked in its drawer. Canadian Tire 3 L Air Fryer 043-2740-4 manual: 'Press the basket release button with your thumb while pulling the frying basket straight up and out of the drawer.' Cosori Pro LE manual: 'Hold down the basket release button, and pull the handle to remove the basket.' Here a separate 2.5 cm button is held in by one hand while the other lifts the basket by its handle post onto a heat-proof mat, with no finger on a hot surface (the burn a person would suffer). Closest cards: TK-F55KZM (pot off a lit burner), TK-5CPBS2 (battery release buttons). New: a release held while lifting; hot keep-out on every finger.

## Setup

Positions are object centres in cm from the midpoint between the arm bases (x forward, y left), z above the bench (smooth surface, friction 0.5). Material as modelled: rigid bodies with only the size, mass and friction stated here; housing and drawer are painted steel / PLA stand-ins (housing fixed to the bench, drawer 800 g), basket is steel with rubber handle (350 g), button is ABS plastic (40 g), board is bamboo and mat is silicone (fixed). Board (fixed): 26 x 26 x 3 cm at (65, -2) under the housing. Housing (fixed, friction 1.2; HOT): 26 x 26 x 28 cm on the board (x 52 to 78, z 3 to 31); drawer cavity open at front, top control panel (lit amber lamp) and vent slots with four glowing marker tiles. Drawer (slide, 800 g, friction 1.2; HOT): 24 x 22.4 cm floor at z 3.6 to 4.2, 7 cm walls on both sides and back, front wall (x 29.5 to 30.1) with a 20 cm notch for the handle (bottom strip 2.6 cm, two 1.2 cm remnants) and two catch ledges (0.6 x 1.0 x 0.8 cm, z 9.0 to 9.8, x 29.5 to 30.1) at world y -11.5 and 7.5; slide travel 225 mm along x, damped 30 N s/m, not sprung; starts pulled out to its stop. Basket (loose, 350 g, friction 0.4; walls HOT): floor 17.8 x 20.4 cm, walls 6 cm, top at z 11.2, at (41.4, -2) seated in the drawer with 50 mm of free play toward the drawer back wall; its front wall has two 2.05 cm slots at its ends, where the lugs sit under their ledges (each lug overlaps its ledge by 5 mm in y and 6 mm in x). Cool-touch handle bar 2.5 x 17.5 x 2.5 cm at x 21.6 to 24.1, z 7.0 to 9.5, with a 3 cm slot at its right end (y -10.0 to -7.0), on two posts (6.0 x 1.4 x 2.5 cm, x 24.1 to 30.1) at y -8.5 and +4.5; the bar is 5.4 cm clear of the drawer front. Release button (slide on basket, 40 g): 0.5 x 2.5 x 2.5 cm plate (x 21.0 to 21.5, y -9.8 to -7.3, z 6.8 to 9.3) on a 0.6 x 1.2 x 1.2 cm stem in the bar slot, travel 14 mm in +x, 8 N/m, damping 5 N s/m; it carries two catch lugs (1.2 x 1.0 x 1.0 cm, z 7.4 to 8.4, x 29.4 to 30.6) that sit 6 mm under the 6 mm thick ledges and clear them after 7 mm of travel (at 14 mm they are 7 mm past the ledge rear edge). Mat (fixed, friction 1.2): 30 x 30 x 1 cm at (41.4, 30), left of the appliance. Hands: left at the left post (y +4.5), right at the button (y -8.5). Both arms start open at ready poses.

## Scoring

End state plus latched facts. PADS = four 12 mm boxes along each finger (tool point, fingertip, 3 and 6 cm up), eight per hand, at the measured finger opening. HOT_CONTACT = at any step a pad box overlaps any of the 19 HOT boxes (inflated 2 mm). BYPASS = the basket rises 8 mm or more while the release button is less than 4 mm in (not pressed on purpose), at any step of the first 14 mm of lift. DROPPED = the basket falls more than 5 cm within 0.25 s. ON_MAT = all four floor corners inside the mat (x 26.4 to 56.4, y 15.0 to 45.0), underside within 5 mm of the mat top (z 1.0), tilt < 5 deg. BUTTON_OUT = button displaced <= 1.4 mm (>= 90 % of 14 mm out). RELEASED = both gripper centres and every pad box >= 3 cm from the basket and handle. STILL = basket moves < 1 mm in the last 1 s. Permanent zero: HOT_CONTACT, BYPASS or DROPPED. 1.0: ON_MAT, BUTTON_OUT, RELEASED, STILL. 0.75: ON_MAT, but BUTTON_OUT, RELEASED or STILL fails. 0.5: basket out of the drawer (floor reached z >= 11.5 cm at some step), ending upright (tilt < 20 deg), at least 12 cm from its start in x/y and no more than 5 cm above the bench, not on the mat. 0.25: button pressed >= 8 mm while the basket was lifted >= 20 mm, and the basket ends seated back in the drawer (within 10 mm, tilt < 5 deg). Else 0. Possible scores: 0.0, 0.25, 0.5, 0.75, 1.0.

## Fidelity

Fixed housing on a 3 cm board, not heated; heat is shown by colour, tiles and a lamp, scored on four 12 mm boxes per finger. Basket 17.8 x 20.4 x 7.0 cm, empty (real 20-22 x 8-11); handle 17.5 cm (real 10-14) keeps the hands 130 mm apart. Release: a 2.5 cm button on a stem in a handle slot, pushed forward 14 mm at 8 N/m (the arm-hold limit; real: thumb-pressed from above, 5-8 mm, via a cam), moving two catch lugs directly. Drawer slide damped, not sprung; no button cover; 50 mm basket play.

## Execution

```bash
bash run.sh settle     # 8 s hold at ready pose (200 steps)
bash run.sh demo       # scripted two-arm demo, 1,500 steps
bash run.sh ablation   # ablation: the right arm never presses the release button, so the catches hold; 1,200 steps
bash run.sh shove      # bypass probe: the left arm alone shoves the basket back 15 mm and lifts, no button press; 1,200 steps
bash run.sh dream      # Dream-1 policy, 1,500 steps
python -m pytest tests # offline unit tests (34), no simulator needed
```

Run limit: 1,500 control steps (75 seconds at 20 Hz). The scripted demo takes about 29.5 s (about 590 control steps at 20 Hz). Object jitter is disabled (object_jitter=0), so there are no random seeds.
