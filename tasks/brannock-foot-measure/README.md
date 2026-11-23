# Measure a foot form on a Brannock device

A right-foot form stands on a shoe-fitting foot measuring device (the Brannock type), 22 to 28 mm forward of the heel
cup. The robot measures it as the device's maker instructs: one hand steadies the ankle so the heel stays seated
against the back of the heel cup, while the other slides the arch pointer to the ball joint and the width bar in
against the side of the foot. Then both hands let go. Every credit level is read off the device's own scales or
checked with a 3 mm feeler gauge, a photo or the trial video.

## Why it matters

The device is how shoe shops size feet. It is still made and sold (Brannock Device Co., https://brannock.com; the maker
reports more than one million sold, https://brannock.com/pages/about-us) and is described as "still widely used in
shoe stores" (Retail Brew, 4 Sep 2024,
https://www.retailbrew.com/stories/2024/09/04/aetrex-introduces-a-3d-foot-scanning-device-for-children). Most people
have had a foot measured on one, often as a child. The readings decide the shoe: the maker says to take the larger of
the arch size and the heel-to-toe size, then read the width at that size.

## Why it is hard

- Two hands at once: one hand keeps the heel seated while the other works the sliders. If the form is pushed out of
  the cup while a slider is being used, the trial is lost (latched zero).
- Light, controlled contact on curved anatomy: the pointer's curve has to end on the ball joint (the widest point of
  the inner side of the forefoot) and the width bar on the outer side of the ball, without shoving the form.
- The readings are fine: half a size on the arch scale is 2.8 mm of pointer travel, one width is 4.8 mm of bar travel.

## The maker's procedure

Brannock Device Co., "Instructions for use / fitting tips", https://brannock.com/pages/instructions-fitting-tips
(short quotes; read the page for the full steps):

1. Prepare the device: "The width bar should be set to its widest position and the arch length indicator should be
   slid back".
2. Position the foot: the heel goes into the heel cup, and the fitter makes sure it is against the back of the cup "by
   grasping the customer's ankle and device together".
3. Heel-to-toe length: read at the longest toe.
4. Arch (heel-to-ball) length: "Slide the pointer (A on diagram) forward so the inside curve of the pointer fits the
   ball joint of the foot"; then "the lower middle rib will be against the ball joint on the side of the foot". For the
   size, "Generally you'll use the larger of the two measurements as the correct shoe size."
5. Width: "Slide the width bar firmly to the edge of the foot." and "view the width measurement indicated by the
   properly determined shoe size".

In this task one hand holds the form's ankle post while the other works the sliders; the order of the two sliders is
up to the robot (the score reads the final state).

### Why the form can slide, and why the hand on the ankle is scored

A standing customer puts much of their weight on the measured foot, so friction keeps it from sliding. The forms here
are 229 to 284 g of PLA on a painted metal plate, so a slider or a gripper can push one out of the cup. That is the
situation the maker's one-hand steadying is for (it is also how seated customers and children are measured, where the
foot carries little weight). The score therefore makes the steadying count: once the heel has been seated and either
slider has since moved 2 mm or more from its start, the heel coming 10 mm or more off the cup's back wall latches the
score to 0.

## Sizing data (published) and the reference readings

Wikipedia, "Brannock Device", section "Sizing system" and its table, revision of 2026-05-30
(https://en.wikipedia.org/w/index.php?title=Brannock_Device&oldid=1356918829), CC BY-SA 4.0; the maker states the
same steps on https://brannock.com/pages/about-us ("Each additional size is 1/3 inch longer", "Each width is
separated by a distance of 3/16 of an inch"):

- heel-to-toe length = 7 1/3 in + size x 1/3 in
- heel-to-ball (arch) length = 5 18/50 in + size x 11/50 in
- width = 1 10/16 in + size x 1/8 in + (steps up from AAAA) x 3/16 in

`sizing.py` holds these formulas; `test_scoring.py` checks them against rows of the table.

| Form | Men's size, width | Heel-to-toe | Heel-to-ball | Ball width |
|---|---|---|---|---|
| layout 0 | 8 C | 10 in (254.0 mm) | 7 3/25 in (180.8 mm) | 3 9/16 in (90.5 mm) |
| layout 1 | 9 D | 10 1/3 in (262.5 mm) | 7 17/50 in (186.4 mm) | 3 7/8 in (98.4 mm) |
| layout 2 | 10 1/2 E | 10 5/6 in (275.2 mm) | 7 67/100 in (194.8 mm) | 4 1/4 in (108.0 mm) |

The reference readings are these published values: the form's size on the arch scale and its width letter at that
size. They are the same in the simulator and in real trials, not anyone's measurement. Acceptance check, once per
printed form: press the form into the cup by hand, bring the pointer and bar to it; it must read within half a size
and half a width of its reference, otherwise reprint it.

## Parts for real trials

| Part | Maker, part number | Size, material | Price | Link |
|---|---|---|---|---|
| Foot measuring device | Brannock Device Co., Combination Brannock Device (US men's and women's sizes) | metal base with heel cup, sliding arch pointer and width bar | $89.65 | https://www.brannock.com/products/combination-brannock-device |
| Three right-foot forms | this task's own design: `forms/foot_form_m8C.stl`, `foot_form_m9D.stl`, `foot_form_m10_5E.stl` | PLA, printed at 100 % scale (sizes above; ankle post 60 mm across, top 130 mm above the sole) | filament below | this folder |
| Placement cards | this task: `forms/placement_layout0.pdf`, `_1.pdf`, `_2.pdf` | paper, printed at 100 % on A4 | - | this folder |
| Two grasp tabs | this task: `forms/grasp_tab.stl` | PLA, a 24 x 5 x 24 mm fin on a 30 x 12 x 2 mm flange; one stuck to the top of each slider with the mounting tape (on the pointer the fin runs along the guide; on the width bar it stands at the bar's outer end, along the device's length); they do not change the sliders' travel | filament above | this folder |
| PLA filament | Hatchbox PLA, 1.75 mm, 1 kg spool (the three forms use about 0.8 kg) | PLA | $19.13 list ($17.66 on sale) | https://robotshop.com/products/3d-printing-canada-usa-hatchbox-transparent-blue-standard-pla-filament-175mm-1kg |
| 3 mm feeler | Threadcheck MFB3.00-300CS | carbon steel blade, 3.00 mm thick, 300 mm long | $19.00 | https://www.threadcheck.com/carbon-steel-metric-feeler-gauges-style-mfb-300mm-length/?p=2 |
| Mounting tape (holds the device to the table) | 3M Scotch 410S, 1 in x 60 in | double-sided foam tape | $6.39 | https://www.webstaurantstore.com/3m-410s-scotch-1-x-60-clear-indoor-mounting-tape/3994010.html |

Total $134.17 with the filament at list price.

### Printing the forms

PLA, 100 % scale, 15 % gyroid infill, 3 perimeters of 0.4 mm, 5 top and bottom layers at 0.2 mm, 0.2 mm layers, sole
down with the ankle post up (support under the arch). Each STL is the union of overlapping closed shells (body, five
toes, ankle post); slicers merge overlapping shells. Expected masses from the mesh volume with these settings
(`footform.print_mass_g`): 8 C 229 g, 9 D 251 g, 10 1/2 E 284 g. Weigh each form on a kitchen scale and reprint if it
is off by more than 15 %. The simulated forms use the same masses.

The forms are this task's own work (parametric shape in `footform.py`: rounded heel, a medial arch lifted off the sole,
ball bulges on both sides, five toes and an ankle post), released under CC BY 4.0 with the rest of this folder. Only
the three lengths in the table come from the published sizing data; the rest of the shape is this design's. On every
form the inner (medial) ball bulge sits 52 mm from the heel's centre line and the outer bulge takes the rest of the
ball width.

## Modelled dimensions (not published by the maker; check on the purchased unit)

The maker publishes the sizing rule but not the body outline, heel cup or slider shapes. They are modelled from the
maker's product photos and the device's patents (US 1725334, US 1682366). Every value below is "modelled, not
published by the maker; check on the purchased unit" (`sizing.DEVICE`):

| Feature | Modelled value |
|---|---|
| base plate | 340 x 222 mm, 10 mm thick, black |
| heel cup | 37 mm inside radius, 5 mm wall, 28 mm tall, straight sides to 58 mm from the back wall |
| inner guide (carries the arch scale) | face 53 mm from the centre line, 12 mm wide, 26 mm tall, from 80 to 300 mm |
| arch pointer | rides on the guide; its round curve (5 mm radius, 7 to 21 mm above the plate) sits flush with the guide face, so it touches the ball joint without pushing the form (check this on the purchased unit: if the curve stands proud of the guide, the feeler check and the contact change); red index line through the curve's centre; travel 140 to 236 mm from the cup's back wall; carries a grasp tab |
| width bar | 110 mm long face, 12 mm thick, 22 mm tall; travel 60 to 132 mm from the guide face; a 30 mm bridge to its grasp tab 40 mm further out; a red index line runs to the width chart beyond the toes |
| slider friction | the simulator's joint friction setting 1.5 static, 1.2 moving (Isaac Sim joint friction effort); the real sliders' friction is not published |

The printed scales follow the published formulas: arch-scale marks at the arch length from the cup's back wall; the
width chart has, for each whole size 7 to 12, a mark for each width letter at the bar-face distance from the guide.

## Scoring

Scored at the end of the trial (`SCORE_OVER = "end"`); 1.0 ends the episode once it has held for 1 s. Every number:

| Quantity | Threshold | Real-trial check |
|---|---|---|
| heel seated | the 3 mm feeler does not fit behind the heel at the cup's centre | feeler gauge |
| a slider on the foot | the 3 mm feeler does not fit between the pointer's curve (or the bar's face) and the foot | feeler gauge |
| width reading | within one width letter of the reference (the chart's resolution: one letter = 3/16 in) | width chart at the bar's index |
| arch reading | within half a size of the reference (the arch scale is marked in half sizes; fitters read to the half size) | arch scale at the pointer's index |
| not held | no gripper within 20 mm of the form (for the heel) or of the slider being credited | trial video |
| clear | every gripper 50 mm or more from the form, the pointer and the bar | trial video / photo |
| still | for the last 1 s: form below 5 mm/s and 3 deg/s, each slider below 2 mm/s (the scorer's test); in 20 fps video: over the last 20 frames the form moves under 5 mm and each slider index under 2 mm | trial video |

| Score | Condition |
|---|---|
| 0.25 | heel seated with the form not held, and at least one slider on the foot and not held |
| 0.5 | as 0.25, with that slider's reading in tolerance |
| 0.75 | heel seated with the form not held; both sliders on the foot, not held, both readings in tolerance |
| 1.0 | all of 0.75, every gripper clear, and still |

No level is earned by a single push: seating the heel alone scores 0. Latched zeros, at any moment: the form falls
off the device (any part of it on the table), tips (the ankle post more than 20 degrees from vertical), or, once
the heel has been seated and a slider has since moved 2 mm from its start, the heel comes 10 mm or more off the cup's
back wall. All three are
visible in the trial video.

## Layouts and placement cards

Trial k uses layout k mod 3 (`TRIAL=k`, or `BRANNOCK_LAYOUT=0|1|2`): the 8 C, 9 D or 10 1/2 E form, heel 22, 28 or
25 mm forward of the cup's back wall, 3, 4 or 2 mm toward the outer side, toes out 1.5, 2 or 1 degrees. The pointer
starts at 146 mm (slid back), the width bar at 128 mm (widest), as the maker says to prepare the device.

Real trials set the start pose with the printed card for the layout (`forms/placement_layout<k>.pdf`, made by
`placement_cards.py`): print at 100 %, check its 100 mm bar, cut out its two paper pieces. The heel piece fits inside
the heel cup against its back wall; the toe piece lies along the guide's inner face with its hook round the guide's
front end. Each carries the form's outline at the start pose (1 mm clearance). Lay both on the plate, set the form
against both outlines, lift the pieces straight up.

World placement in the simulator: the cup's back wall on the device centre line at x 0.29 m, y 0.13 m on the station
table; the device lies across the table, heel cup toward the left arm.

## Results

| Run | Layout 0 (8 C) | Layout 1 (9 D) | Layout 2 (10 1/2 E) |
|---|---|---|---|
| scripted demo, 3 runs each | 1.0, 1.0, 1.0 (669 steps) | 1.0, 1.0, 1.0 (699 steps) | 1.0, 1.0, 1.0 (678 steps) |
| Dream-1 (trial 0) | 0.0 (both grippers reached over the form and withdrew; the heel was not seated with a slider on the foot) | | |

The demo (`demo.py`) is closed-loop: it reads the form's pose and the slider joints to plan, and never moves an object
except through the grippers. Left hand: grasp the ankle post, seat the heel (up to six corrections), then ease the grip
to steady the ankle. Right hand: slide the width bar in until it rests on the foot and the ball joint is on the guide,
then slide the pointer to the ball joint. Both hands then let go and move away. Run limit: 2,400 steps (2 minutes at
20 Hz), about 3.4 times the longest demo.

## Running

Requirements: the `yam_arena` embodiment for Inspect Robots (Isaac Lab Arena). From this folder:

    python build_assets.py                      # USD assets, scale textures and the three STLs (a few minutes)
    python placement_cards.py                   # the three placement cards
    python -m pytest -q test_scoring.py          # scorer tests with negative controls (no simulator)
    python reach_check.py                        # IK reach and clearance along the demo paths (no simulator)
    BRANNOCK_LAYOUT=1 python demo.py out/demo1   # closed-loop scripted demo: videos and result.json
    TRIAL=0 inspect-robots run --embodiment yam_arena --policy dream -E task=brannock_foot_measure.py \
        -E object_jitter=0 -E control_hz=20 --instruction "<instruction from task.json>" --scorer task_score \
        --max-steps <run limit>

The task file builds the assets itself if `assets/` is missing.

## Files

- `brannock_foot_measure.py`: the task file (objects, layouts, score, latched zeros).
- `scoring.py`: readings and score, pure numpy; `test_scoring.py`: its tests.
- `sizing.py`: the published sizing formulas and the modelled device layout.
- `footform.py`: the forms' parametric shape; `forms/`: STLs and placement cards.
- `build_assets.py`: the USD assets and scale images; `simhook.py`: spawns USD assets and reads joint state without
  modifying yam_arena.
- `layout.py`, `plan.py`, `demo.py`, `reach_check.py`, `placement_cards.py`: placement, key poses, the demo, the
  reach check, the cards. `forms/grasp_tab.stl`: the grasp tab (written by `build_assets.py`).

## Third-party material

No third-party models or images. The sizing formulas come from the Wikipedia article cited above (CC BY-SA 4.0) and
the maker's site; the device's appearance is modelled, not copied.
