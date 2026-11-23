# Start a travel slow cooker for dinner: unhook both lid latches left locked from the car ride before switching it on

Instruction: "Switch the slow cooker on to LOW for dinner."

## Scoring

Judge the top-camera video and end photo. A latch is off only when its wire crossbar is outside its hook; a raised lever alone does not count. OFF means dial angle at most 10 degrees clockwise from OFF; LOW means 33 to 57 degrees inclusive. Score 0 if the dial ever exceeded 10 degrees with either wire hooked, even if released later; cooker displacement ever exceeded 3 cm or tilt exceeded 10 degrees; the lid was lifted off its rim; or neither wire is off at the end. Otherwise take the lowest applicable value: 0.25 with exactly one wire off and dial OFF; 0.5 with both off and dial OFF; 0.75 with both off and dial ON outside LOW, or dial LOW with a gripper centre less than 3 cm from the dial bar; 1.0 with both off, dial LOW and both gripper centres at least 3 cm from the bar. Displacement greater than 1 cm and at most 3 cm caps any score at 0.75; exactly 1 cm has no cap. Doing nothing scores 0.

Scorer coordinates: x from front to back, y toward the left handle, z up; origin under the footprint's centre.

- **Wire clip in its hook (hooked):** the centre of the wire's bottom bar lies between the body (150 mm out) and the
  middle of the hook's lip (171.8 mm out), strictly inside both boundaries, and strictly above 175.5 mm (the lip's bottom less the wire's radius). The crossbar centre must also be strictly within 37 mm of the hook's x centre.
  Everything else is unhooked. The lever's angle is not scored: a lever flipped up with its wire still in the hook
  is still hooked.
- **Knob:** its hinge angle, clockwise from OFF as seen from the robot. ON beyond 10 degrees; LOW within
  12 degrees of 45. Hands clear: both grippers' centre points at least 30 mm from the middle of
  the knob's bar.
- **Remembered for the rest of the run:** the knob ON while either wire was hooked; the cooker's origin more than
  30 mm (zero) or 10 mm (cap) from where it first stood; tilt over 10 degrees.

`success()` is a score of 1.0, held 1 s. Why the shortcuts fail: turning the knob first scores 0 for good, whatever is
done afterwards; a wire cannot be pulled out of its hook while its lever is down (it hangs at the bottom of its
swing, the hook's lip outboard and the body inboard of it), so the latch has to be flipped round; shoving the cooker
over 3 cm scores 0. Wrong-way GPU run: dial turned to 47.4 degrees with both wires still hooked; score stayed 0.0. Cooker stayed in place.

## Run and score

From this folder with YAM Arena installed:

```sh
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=unhook_slow_cooker_lid_latches_before_switching_on.py -E object_jitter=0 -E control_hz=20 \
  --instruction "Switch the slow cooker on to LOW for dinner." \
  --scorer task_score --max-steps 1800 --epochs 1 --store-frames --log-dir ./demo-run
```

For a model trial use `--policy dream`. Each trial allows 1,800 control steps (90 s at 20 Hz);
the scripted demonstration takes 27.4 s. `pytest` here checks the scorer on named layouts.

## The scene

One object, the cooker, with the positions and products in task.json's `setup`. Both arms start at their ready pose.
Modelled in `slow_cooker.py` as one free articulation on four feet (friction 0.6/0.5), all at true size.
Root mass 5.22 kg plus 79 g of moving links totals about 5.30 kg; those link masses are estimates.

- **Base:** oval stainless body 255 x 320 mm; lid rim 199 mm up, dome 224 mm, loop knob 250 mm.
  Handles reach 207.5 mm each side. Each has a 20 mm-wide clip hook with a 4 mm lip.
  The lid is fixed to the base; removing it cannot be exercised in simulation.
- **Latch levers** (`lever_l`, `lever_r`, 20 g): 48 x 20 x 8 mm, each on a hinge along x at
  169 mm out, 212.5 mm up; closed (0) flat pointing in, up to 185 degrees flipped out over the handle;
  static/dynamic friction 0.08/0.06 N m (about 2.6 N at the lever's middle: estimate of the wire's over-centre pull).
- **Wire clips** (`wire_l`, `wire_r`, 7 g): U-shaped 3 mm wire, legs 54 mm apart, 41.5 mm from the hinge to the
  bottom bar; hinged on the lever 3 mm in from and 7 mm above its pivot (damping 1e-4 N m s/rad). Flipping the lever
  up carries the wire's hinge out and down. The 1.5 mm of play beneath the hook's nose represents gasket compliance; the bottom bar leaves the 4 mm lip at about 80 degrees of lever lift and swings out under gravity.
- **Knob** (`dial`, 25 g): round, 45 mm across and 18 mm deep, with a raised bar 40 x 10 x
  12 mm, on a hinge along x 55 mm up, 0 to 140 degrees, static/dynamic friction 0.06/0.05 N m.

## Scripted demonstration

Each hand approaches its latch from outboard, tilted 35 degrees from vertical, and pinches once
31 mm from the pivot (17 mm commanded on a 20 mm-wide lever). Every 0.35 s it reads the
lever angle and targets 25 degrees further round, up to 182. An offset computed from that arm's position
stiffness leads the hand to push about 7 N across the lever (2 N at the stop; estimates), avoiding a
push down its length. Once the wire drops out, the hand releases and slides off. The right hand goes round to
the front, pitched 40 degrees down, pinches the knob's raised bar (7 mm commanded on 10 mm), turns
48 degrees clockwise about its axis, releases and backs off. Both arms park clear.
The demo reads cooker pose once and lever angles at each step; the model policy receives camera images.

Demo (version 26f38e763452): 1.0 at 22.3 s, held through 23.25 s. Both wires cleared before the dial moved; levers ended at 106.8 and 105.7 degrees, wire crossbars visibly outside the hooks. Dial 47.4 degrees (LOW); cooker stayed in place and both gripper centres were at least 3 cm from the dial (the full-score gate). The target of 182 degrees is not reached at this conservative pinch; only actual wire release counts.

## Simulation and the real objects

True-size rigid cooker: 5.22 kg root plus 79 g links on table friction; lid fixed and opaque; cord omitted. Levers use 0.08 N m friction for over-centre resistance; hinged wire clips release by hook contact. The 1.5 mm play under each hook represents gasket compliance. Knob friction 0.06 N m represents detents. Forces, friction and component sizes are estimates. Unplugged and empty in real trials; no heat or breakage. Demo reads cooker pose and lever angles for force-sized hand leads (~7 N).

- Cooker: Crock-Pot 6-Quart Cook & Carry manual slow cooker, SCCPVL600 series (for example SCCPVL600-S). Maker's
  owner's guide for 4-7 quart manual Cook & Carry cookers (safeguards 18, 21 and 22; latch parts, locking and
  unlocking with Figures 1 to 4): https://s7d9.scene7.com/is/content/NewellRubbermaid/DASH/S7_int/2020/4-7Qt-Manual-Cook-Carry-SlowCooker-IB.pdf .
  Overall size and weight from a published review of the SCCPVL600S, "H:250mm W:415mm D:300:mm" and "5.22kg with
  the cooking pot weighing in at 2.9kg": https://www.cookuk.co.uk/slow-cooker/review-crock-pot-sccpvl600s.html . The
  maker's current product page: https://www.crock-pot.com/slow-cookers/6-quart/crock-pot--manual-6-quart-cook-carry-slow-cooker/SP_528892.html .
  That page names a newer SCCPVL600-S-BR revision with different dimensions and mass; this scene uses the older
  SCCPVL600S review's dimensions. Check the purchased revision with a ruler and kitchen scale before real trials.
- Estimates: component sizes, masses, friction, 45-degree dial spacing, lever lift (2.6 N at 31 mm) and knob
  torque (0.06 N m, about 3 N at the bar's end). One-minute checks: ruler for sizes and hinge offsets, kitchen
  scale for masses, panel marks/protractor for angles, spring scale at the stated radius for forces. Lever
  friction is 0.08 N m throughout travel; the real draw latch mainly resists the initial over-centre snap.
- Real trials: unplugged and empty, stoneware and lid in place; visible latches, knob and lid are unchanged.
  Nothing is hot or live. Judge the video and end photo.
- Steadying: at these forces (a few newtons) the 5.2 kg cooker does not slide, so the demonstration does not hold
  it by a handle; pushing it over 1 cm is still penalised.

## Budget and preparation

Retail listing opened November 21, 2026: Home Depot SCCPVL600-S, US$44.99,
https://www.homedepot.com/p/204499307 . Allow US$10 more for tape and a ruler (budget estimate); total about US$55
before tax. Confirm the purchased revision against the size reference above. No consumables are used per trial. Preparation
takes a few minutes: place the empty stoneware and lid, keep the cooker unplugged, coil its cord behind it, mark the
feet with tape and lock both latches. No pad covers are needed because the pads touch only dry levers and the dial.

## Reset

About a minute. Turn the knob to OFF. On each side lock the latch as the guide's Figures 1 to 3 show: flip the lever up and out, hook the wire clip's bottom bar into the clip hook above the handle, flip the lever up and over to the top and push it down flat. Check the lid sits square on its gasket. Stand the cooker back on its tape marks, knob facing the robot. It stays unplugged and empty for every trial (stoneware in, lid on, cord coiled behind it), so nothing is live or hot; the cameras see the same cooker, latches and knob as in use. Every 50 trials check with a spring scale that each lever still needs about 2 to 4 N at its middle to lift and that the knob still turns to LOW; replace the latch if a wire clip has bent. Both arms back to the ready pose, grippers open.
