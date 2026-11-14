# Measure powdered formula into a 4-ounce bottle

A wide-neck bottle holds 120 mL (4 fl oz) of water, next to an opened tin of formula powder with a leveling bar
across its opening; the tin's scoop lies on a stand. "Measure the formula into this bottle."
(Capping and shaking come after and are not part of the task.) US standard formulas are mixed at one unpacked, level
scoop per 2 fl oz of water, so the bottle takes exactly two level scoops (17.8 g). A scoop dug through the powder
comes out heaped (about 29% over); the worker draws it up against the leveling bar so the heap falls back into the
tin, then tips it into the bottle. Heaped, pressed-down or extra scoops make the feed too
strong, and powder in water cannot be taken back out.

Run (from the repo root, Isaac Lab-Arena installed; the kit files yamkit.py, pourkit.py, dosekit.py and
guardkit.py sit next to task.py):

    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=tasks/infant-formula-level-scoops/task.py \
      -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "Measure the formula into this bottle." --scorer task_score --max-steps 1500
    python tasks/infant-formula-level-scoops/test_task.py      # offline scorer checks, prints "all pass"

Use `--policy dream --max-steps 2400` for a Dream-1 trial. The scene is deterministic (no seeds, object_jitter=0).
The scripted demo plans 69.5 s (1,390 steps); its run reaches full credit, held 1 s, at 1,289 steps, once
the scoop is let go on its stand. A full trial is 2,400 steps. Set
`DOSEKIT_DEBUG=1` to print grams per bin, packing and spill each step.

## Spec

Positions in cm from the midpoint between the two arm bases: x forward, y to the robot's left, table top z 0.

| Object | What it is | Where |
|---|---|---|
| Formula tin (fixed) | white steel, 14.6 cm across (14 cm inside), 10 cm tall, lid off, plain pale-blue label band; 700 g of pale cream powder, top about 0.65 cm below the rim | axis at x 31, y 11 |
| Leveling bar (fixed, with a collider) | gray, 1 cm wide, 0.4 cm thick, across the opening along x; its underside at the rim is the cutting edge | center line y 12.5, z 10 to 10.4 |
| Bottle (fixed) | clear 270 mL wide-neck plastic bottle, 6 cm across, 11.5 cm tall, 5 cm neck opening, in a gray holder ring 9 cm across; 120 mL of water, the 120 mL line marked | axis at x 40, y -3 |
| Scoop (loose, 45 g) | dark-blue plastic; round handle 2.5 cm across, 7 cm long, along y; neck down to a round bowl 3.9 cm across outside, 3.6 cm inside, 1.75 cm deep (17.8 mL); rim 3.9 cm below the handle's axis | handle center x 23, y -12.5, z 8; bowl center y -4.75 |
| Scoop stand (fixed) | white; post 1.6 x 6.6 cm, 6.75 cm tall under the handle; pad 3.5 x 3.5 x 2.2 cm under the bowl | x 23, y -12.5 |

Powder (dosekit `Material`): loose 0.497 g/mL (as poured), 0.50 when dipped (labels give the scoop's weight dipped
and leveled), 0.64 packed; angle of repose 42 degrees. One level scoop: 17.8 mL x 0.50 = 8.9 g (US labels: 8.6 to 8.8
g). A scoop filled under the surface and lifted out level is heaped to 11.5 g; a fingertip pressed into it packs the top 2 cm.

The demo scoops the way a hand does: the bowl tipped 35 degrees toward the robot, lip first, is pushed under the
surface and dragged 3 cm through the powder, then turned level while still under it and lifted straight out heaped
(11.5 g); a straight-down plunge would fill the same way in the model, so the score does not depend on the motion.

How the powder moves (dosekit, from the poses each control step): the scoop's mouth more than 4 mm under the
powder's surface fills it to the brim plus a cone at the angle of repose (while tipped, only as much heap as stands
at 42 degrees less the tilt, so turning it level under the surface tops it up; heap shed from a tipped scoop under
the powder stays in the tin); the leveling bar is a `dk.Edge` on the tin
along its underside (`below=0.002`: a scoop can only be drawn up against the bar, and one carried over its top cuts
nothing), and wherever the scoop's heap passes within 4 mm of it, the heap is cut down to the bar and falls back
into the tin (with the rim pressed up to the underside or up to 1 mm short of it the result is within the dose
window; 3 mm or more short, enough heap stays to go over it); the kit also records which of the scoop's mouth cells
the bar has passed over since each dip (`dk.swept_since`), counting the whole band the mouth crossed between control
steps, so a fast draw (1 cm a step, 20 cm/s) is not skipped over; the scoop tipped past 45 degrees pours from its lowest rim point straight down, into the bottle
if over its neck, else onto the table. At the demo's full turn (123 degrees planned, about 118 reached) the bowl's
lower wall slopes only 28 to 33 degrees below level, under the 42 degree angle of repose, so the model keeps about 0.2
to 0.3 g on it, the small residue a real scoop also keeps. The video shows it as a little cream powder in the
corner of the bowl's floor and the wall it was tipped onto; the model counts it in the scoop, not the bottle. The heap is drawn in
the powder's own cream color, each cell kept inside the bowl's mouth.

## Scoring

The powder in the bottle is weighed at the end of the run, and the result must hold for 1 s.

| Score | What earns it | Why it is worth that |
|---|---|---|
| 1.0 | 16.6 to 19.1 g in the bottle, every scoop drawn along the leveling bar, under 1 g spilled, the scoop let go and at rest on the table or back on its stand (not in the tin or on the bottle) | the correct feed, finished: the hand is free to cap the bottle |
| 0.5 | the same dose, but the scoop still held, moving, off the table, or left in the tin or across the bottle, or 1 g or more spilled | the feed is right, but the job is not finished cleanly |
| 0.25 | one level scoop, 8.3 to 9.5 g | half the feed, measured right; a second scoop can still finish it |
| 0 | a scoop reached the bottle without being drawn along the bar; the bottle ever held over 19.1 g; any other amount; doing nothing | an unleveled or over-strong feed cannot be fixed, since powder in water cannot come back out |

### How each part is measured

- Dose. Two level scoops, 17.8 g, at the label's one level scoop per 2 fl oz. The window is plus or minus 7%, because
  a scoop drawn 1 mm short of the bar's underside keeps about 0.5 g (5.7% of a scoop) more. One heaped scoop with a
  level one is up to 20.3 g, two heaped 22.9 g, three level 26.5 g, and scoops pressed down 20 g or more. Part of a heap
  can fall off on the carry (the sim's heaped-then-level run ended at 19.0 g, inside the window), so the strike-off
  rule, not the grams, is what catches an unleveled scoop.
- Leveled at the bar. Each time powder arrives in the bottle, the kit checks that the bar has passed over all of the
  scoop's mouth, at its underside, since that scoopful was dipped (`dk.swept_since`, 99% of the mouth). The label's
  level scoop is leveled with the tin's leveler (or by drawing the scoop up the side of the container). The model's
  powder is weightless and sheds its heap exactly when the scoop is tipped just under 45 degrees; a real cohesive
  powder sheds unevenly, so tipping alone is not a level scoop. A load that was tipped and then also drawn along the
  bar is level and counts.
- Let go. No gripper's own shape (palm and both fingers, at their present opening) within 2 mm of the scoop's handle,
  neck or bowl (guardkit `touching`), and the scoop moving under 1 cm/s, either back on its stand (handle within 1.5
  cm of where it started) or lying on the table (handle center under 7 cm up, outside the tin's and the bottle
  holder's footprints). A scoop left in the tin or across the bottle's neck is in the way of capping the bottle, so
  it earns 0.5. Contact decides it, not distance: an open
  hand resting a pad on the handle still holds it, and one lifted clear of it does not. The arms' final pose is not
  scored.
- Spill. Under 1 g on the table is a clean job (a level scoop is 8.9 g).
- Real trial. Weigh the bottle with its water on a 0.1 g kitchen scale before and after, weigh any powder brushed off
  the table, and check on the video that each scoop was drawn along the bar before it went to the bottle and that the
  hand let go of the scoop. Calibrate the windows to the tin used (see Real-world build).

Offline tests (`test_task.py`): two level scoops with the scoop let go on its stand or on the table 1.0, in
the tin or across the bottle's neck 0.5; the tipped dig and turn-up comes out heaped at 11.5 g and strikes to one level
scoop; the heap is drawn in the powder's color; do nothing 0; two level scoops 1.0, also with the hand left open just over the scoop
(arm-home is not scored); still closed on the handle 0.5, fingers opened around it 1.0, an open pad resting on it 0.5,
lifted 5 mm clear 1.0; scoop sliding 0.5; scoop held in the air by its neck or bowl 0.5; one level 0.25; a second
scoop poured beside the bottle 0.25; two level and a third on the table 0.5; one heaped 0; heaped + level 0
(latched); two heaped 0; pressed with a finger then leveled 0; three level 0; a third level scoop partly tipped in 0
(latched); a third dipped and dumped back into the tin 1.0; drawn 1 cm under the bar 0; rim at the bar, 1 mm under it
and pressed 3.5 mm into it (stopped by its collider) 1.0, 3 mm under 0; carried over the bar's top 0; heap shed by
tipping, no bar 0 (also when only the second scoop is tipped); tipped and then drawn along the bar 1.0; the draw along
the bar done in 10, 5 or 3 steps (up to 1 cm a step) 1.0, and fast draws over the bar's top or 1 cm under it 0; held
level over the bottle, never tipped 0; scoop knocked off the stand or off the table 0; fingers poked into the tin and
hovering at the bottle 0; a new trial after an over-strong one 1.0; and the scripted demo replayed with perfect
tracking (less the 5.5 mm the loaded arm and grip sag in the sim), the bar's collider emulated, 1.0 (17.5 g in the
bottle).

## Real-world build

- A 12.4 to 30.8 oz powdered formula tin with its own scoop (any US brand; cover the brand with a plain label band),
  the scoop's handle wrapped in a printed or heat-shrink sleeve to 2.5 cm round with a few washers inside for weight.
- If the tin has no built-in leveler, a 10 x 4 mm aluminum flat bar across the opening, epoxied to the rim.
- A 9 oz wide-neck bottle in a clamp-on bottle holder or a weighted ring; 120 mL of room-temperature water.
- A scoop stand: a 1.6 cm wood post and a 2.2 cm block, glued to a base.
- Calibrate before trials with each tin or lot: the sim's 8.9 g level scoop is the nominal label value (US labels
  8.6 to 8.8 g), not the real tin's. Weigh 10 level scoops of the product used, each leveled against the bar, take
  their mean m (check all 10 fall within 3% of m), and score 2m +- 7% for the 1.0 and 0.5 dose, m +- 7% for 0.25 and
  latch 0 above 2m + 7%. The 7% is the 1 mm leveling error, wider than hand-leveled scatter.
- A 0.1 g kitchen scale for scoring. Formula powder is food: safe for the arms and the people around them.

## How it differs from nearby board tasks

- Warming a baby's bottle in a water bath and hanging a washed bottle to dry: bottle handling, no measuring.
  BEHAVIOR-1K's 1,016 activity definitions have no formula or bottle-preparation activity (nearest:
  wash_baby_bottles, filling_sugar, make_chocolate_milk, none measured by level scoops).
- Choosing a medicine dosing tool: picks the tool, measures nothing.
- Weighing 1.00 g on a balance, scooping beads into a bowl, screeding a fill to rails: weighing to a readout,
  moving loose solids, or striking off a surface. Here the amount is set by the fill method of a fixed measure: level
  versus heaped versus packed, with no readout to watch.
