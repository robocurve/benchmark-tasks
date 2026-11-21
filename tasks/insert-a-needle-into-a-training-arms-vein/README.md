# Put a blood-draw needle into a training arm's vein

An adult venipuncture training arm lies strapped down, palm up, on the table. Beside it a single-use tube holder with
a 21 gauge, 1 inch flashback needle on it lies in a cradle, uncapped and bevel up. The robot has to do the stick of a
routine blood draw: with one hand draw the skin below the site toward the wrist and keep it drawn, and with the other
put the needle into the vein at the elbow crease, bevel up, at a shallow angle, stop when the tip is in the vein and
keep it still.

Everything is at full size. Nothing marks the site on the skin; the vein is drawn in blue.

## Changes in this revision

The first version's needle met nothing, and its second hand only had to rest a fingertip on the arm. Both are changed.

- **The needle now meets resistance, applied by the physics solver.** From the moment its point reaches the skin the
  needle slides on its own line against a force: 1.0 N until the skin is pierced, 0.10 N of shaft friction after
  that, 0.40 N at each wall of the vein. The hand has to push to get through the skin, and the needle surges when the
  skin gives unless the push is eased at once. Details and sources: "What the needle meets".
- **A needle at the wrong angle or in the wrong place is not refused.** It meets the same 1.0 N and goes in when it is
  pushed, as a real needle does, and the trial is then void as a bad stick. Blocking it would have been less real.
- **The anchoring hand has to apply skin traction.** The forearm's skin below the site is a patch that slides along
  the arm against a spring. The vein counts as held only while that skin is drawn at least 5 mm toward the wrist, and
  a fingertip that only rests on it does not hold the vein (control run below: 0.25).
- **A miss, a vein entry and a through-and-through stick are told apart by what can be seen.** The needle is now a
  flashback needle: blood shows in its hub's view window when the tip enters a vein. After the trial an evacuated
  tube is pushed onto the holder: it fills only if the tip is still in the vein. See "Scoring a real trial".
- **The hidden 0.5 mm tip rule is gone.** Stillness is read from the holder's back end, outside the arm: under 1 mm of
  movement and under 2 degrees of turn over the last second. The rule against sweeping the shaft sideways in the skin
  is gone too: the tissue now resists that, and it could not be seen from outside.
- **A lower-cost alternative to the training arm** is given under "Building it for real", with a way to build one.
- **The gripper pads are protected from the needle**: see "Building it for real" and the reset notes.
- The run limit is now 800 steps (it was 600): drawing the skin and pushing through it take longer.

## What makes it hard

- **A push that has to be eased the moment it succeeds.** The skin takes 1 N to pierce and the tissue behind it
  0.1 N. A hand that is still pushing with 1 N when the skin gives drives the needle on; the vein's far wall is a few
  millimetres ahead and takes only 0.4 N.
- **A small target in three directions.** The vein is 4 mm across and its middle is 3.5 mm under the skin. At 25
  degrees the tip is in it from 3.5 mm to 13 mm of needle. The needle has to cross the skin within 8 mm of the vein's
  line, and a tip more than 2 mm to the side of the vein's middle misses it.
- **An attitude, not just a place.** 10 to 30 degrees to the skin, pointing up the arm, bevel up.
- **Two hands doing different things at once.** One presses on the skin and keeps it drawn toward the wrist against
  its spring from before the needle reaches the vein until the end; the other makes the stick 11 cm away. The
  uncapped needle may never come within 15 mm of the hand that is not holding it.
- **A tool taken from a rest.** The holder is a 19 mm barrel lying in a V groove; how it sits in the fingers decides
  where the needle's tip is, 6 cm ahead of them.
- **Stopping.** The stick ends with the needle held still in the vein, not with letting go.

## What the needle meets

- **The shaft is not a collider.** A 0.8 mm shaft cannot be simulated as a body here. Instead, at the control step
  when the needle's point comes within 0.2 mm of the skin over the elbow, the kit joins the holder to the arm
  (`veinkit.engage`). From then on the solver, at every physics step (240 a second), makes the needle slide on the
  line it had at that moment: a damper along the line that tops out at the tissue's resistance, and springs that hold
  the tip to the line (400 N/m) and the shaft to its direction (1 N m/rad). The needle can still turn about its own
  axis. When the tip is drawn back out of the skin the joint is taken off.
- **The resistance** is the most the damper can exert. Below it the needle only creeps (0.2 mm/s per newton); above
  it the needle moves and the resistance stays at that value. The task file sets it 20 times a second from where the
  tip is (`veinkit.resistance`); the force itself is the solver's.

| Where the tip is | Resistance | From |
|---|---|---|
| On the skin, or less than 1 mm under it | 1.0 N | A needle maker's penetration tests of 21 gauge needles through the standard 0.4 mm polyurethane foil (DIN 13097-4): peak penetration force 1.00 to 1.25 N across three needle types, single needles 0.80 to 1.56 N. |
| Through the skin | 0.10 N | The same tests' friction phase: 0.06 to 0.17 N. |
| Within 0.25 mm of the vein's wall, near or far | 0.40 N | An estimate; no measurement was found. |

  The foil is a test medium, not skin, and the 1 mm of skin is an estimate. A training arm's vinyl skin and latex
  veins will differ; measure them (a spring scale on the holder is enough) before a real trial.
- **A bad stick is not stopped.** Off the site, too steep, or bevel down, the needle meets the same resistance and
  goes in. The trial is then void.
- **Where the needle crossed the skin, at what angle, with which side up, how deep its tip is, and whether the tip is
  in the vein or through it** are read from the holder's pose every control step (`veinkit.Stick`).

## Skin traction, and the vein rolling

- **The skin below the site is a patch on a sprung slide.** It covers the top of the forearm from 8 to 15 cm below
  the crease, 40 degrees either side of the vein's line, and stands 2 mm proud. It slides along the forearm, up to
  15 mm toward the wrist, against a spring of 80 N/m (0.8 N per cm, an estimate) that brings it back. Nothing moves
  it but friction under a fingertip (0.8, an estimate): the finger has to press and pull at once, and keep doing so.
  An ink line across the patch meets two ink ticks on the arm when the skin is at rest; how far the line has moved
  from the ticks is the traction.
- **The vein rolling aside is a rule, and the only one.** If the skin is not drawn at least 5 mm toward the wrist at
  the moment the tip gets as deep as the vein's near wall, the vein is taken to have moved 5 mm to the side, away
  from the needle, and it stays there while the needle is in. Why a rule: nothing here simulates a vein moving under
  loose skin, and that is the reason the manual gives for traction ("to prevent the vein from moving"; the training
  arm's listing: "life-like durable vein roll-over"). The 5 mm, and that the vein always rolls when the skin is slack,
  are choices, not measurements: a real vein rolls sometimes.
- **The patch is 8 to 15 cm below the site, not the 2 to 5 cm a thumb is.** The holder is 7 cm long behind the skin and
  lies almost on the forearm at these angles, and the hand on it fills the space above. A gripper's fingertips fit
  only below that. This is the nearest the two hands can work.

## Files

| File | What it is |
|---|---|
| `task.py` | The layout, the instruction, the record kept every control step, the scoring rule. |
| `veinkit.py` | The training arm, its skin patch and slide, the holder and needle, the cradle, the needle's joint and the needle's record (`Stick`). |
| `demo.py` | A scripted demo that reaches full score, and the control runs. |
| `test_scoring.py` | 24 checks of the shapes, the resistance, the traction, the needle's record and the scoring rule; no simulator needed. |
| `task.json` | The task's description for the board. |

## Running and scoring a trial

```bash
# a policy (Dream-1 here), 800 steps at 20 Hz; the score is printed at the end and written to the log
inspect-robots run --embodiment yam_arena --policy dream -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 800 \
  --instruction "Put the needle into the training arm's vein at the elbow crease for a blood draw: with your other hand draw the skin below the site toward the wrist and keep it drawn, go in bevel up at a shallow angle, and stop when the needle is in the vein. Keep it still there."

python demo.py task.py out_dir                 # the scripted demo: videos, setup.png, result.json
VEIN_LOT=B python demo.py task.py out_dir      # the second layout
python -m pytest -q test_scoring.py            # the offline checks
```

- `object_jitter=0` is required: the arm and the cradle are fixed and the layouts are exact.
- There are no seeds. `VEIN_LOT` picks the layout: A, or B (the arm 1 cm nearer the robot and 1.5 cm to the left, the
  cradle 1 cm further and 2 cm to the right).
- A trial ends at full score held for 1 s, or at the run limit of 800 steps (40 s).
- Time per trial on one L40S: about 4 minutes for the recorded demo (473 steps, with start-up) while three other runs shared
  the GPU; 2 minutes 13 seconds for the 800-step Dream-1 run with the GPU to itself. No demo run was timed alone.
- `result.json` from the demo holds the score, the needle's record, the traction and a trace every five steps.

## Scoring

Scored at the end of the trial.

| Score | State at the end |
|---|---|
| 0 | The needle is not in the skin, or the trial is void (below). |
| 0.25 | The needle is in the skin (tip at least 1 mm under it) after a good stick, but its tip is not in the vein. |
| 0.5 | Its tip is in the vein, but the holder is not in a gripper, or the holder is not still, or the needle is not at 5 to 30 degrees to the skin. |
| 0.75 | In the vein, held in a gripper, still, at 5 to 30 degrees; but the skin was let slack at some time after the tip reached the vein. |
| 1.0 | In the vein, held, still, at 5 to 30 degrees, and the skin kept drawn ever since the tip reached the vein. |

The numbers:

- **A good stick:** the needle crosses the skin within 8 mm of the vein's line and 12 mm of the elbow crease, at 10 to
  30 degrees to the skin, with its bevel facing away from the skin within 30 degrees.
- **In the vein:** the tip is inside the vein's lumen, 4 mm across, its middle 3.5 mm under the skin on the vein's line
  (or 5 mm to the side of it, if the skin was not drawn when the tip reached it).
- **The skin is drawn** while the patch's ink line is at least 5 mm toward the wrist of the ticks beside it.
- **Still:** over the last second the holder's back end (the middle of its flange) has moved less than 1 mm and the
  holder has turned less than 2 degrees.

The whole trial scores 0, whatever the end state, if at any time:

- the needle (tip to hub) came within 15 mm of the gripper that is not holding the holder: a needlestick;
- the needle touched (came within 1.5 mm of) the table, or the arm away from the site, before it went in;
- it made a bad stick: into the skin away from the site, outside 10 to 30 degrees, or with the bevel not up;
- its tip went deeper than the vein's far wall under the vein: through and through.

Doing nothing scores 0. The score depends on the state at the end and on those events; nothing is added up over time.

Ways through that were considered, and what they score:

- A fingertip pressed on the skin but not drawn: 0.25. The stick is good but the vein has rolled aside (run below).
- Drawing the skin only once the needle is in: 0.25. The vein rolled when the tip reached it.
- Going in steep to be sure of reaching the vein: 0 (bad stick; run below). Going in deep: 0 (through; run below).
- Going in anywhere on the arm and steering the tip to the vein under the skin: 0 (away from the site).
- Letting the skin go once the needle is in: 0.75 (run below). Letting go of the holder: 0.5 (run below).

## Scoring a real trial

Tools: a phone on a stand at the arm's side, level with the elbow, with a protractor card and a millimetre ruler
behind the site; a second phone above; an evacuated blood tube; the ink line and ticks on the skin.

What each state looks like from outside:

| State | What is seen |
|---|---|
| Missed the vein (0.25 at most) | No blood in the hub's view window at any time. |
| In the vein (0.5 or more) | Blood appears in the view window, and the tube pushed on after the trial fills. |
| Through and through (void) | Blood appeared in the view window, but the tube pushed on after the trial does not fill (the tip is past the vein), or fluid collects under the skin. |

The needle in the task is a multi-sample needle with a flash window: its hub is clear plastic, and blood from the vein
shows in it as soon as the tip is in the lumen, before any tube is on (a trade description of one such needle: "The
needle has a translucent plastic hub, which functions as a view window. If venipuncture is successful, the view window
is coloured red due to the blood flow."). A plain multi-sample needle shows nothing until a tube is pushed on; do not
use one. The training arm's veins are filled with its simulated blood, under the slight pressure its maker directs,
which is what gives the flash (its listing: "Realistic flashback").

1. Site (ruler, overhead video): the needle goes into the skin within 8 mm of the vein's line and 12 mm of the crease.
   Rule a faint pencil cross on the skin after the trial, not before.
2. Angle and bevel (side photo, protractor card): the needle is 10 to 30 degrees to the skin as it goes in, and 5 to
   30 degrees at the end; the hub's white mark is within 30 degrees of straight up.
3. Missed, in, or through: the table above. Watch the view window on the side video from the moment the needle
   touches the skin. With the robot's hands left where they stopped, push an evacuated tube onto the holder and see
   whether it fills.
4. Traction (overhead video, ruler): the skin's ink line is at least 5 mm toward the wrist of the ticks at the moment
   blood first shows in the view window, and at every moment from then to the end (1.0), or not at every moment (0.75).
   If it was less than 5 mm when the needle went in and no blood showed, the vein rolled: that is the miss above.
5. Still (side video, ruler): over the last second the holder's flange moves less than 1 mm against the ruler and
   the holder turns less than 2 degrees against the protractor card. A gripper is holding the holder.
6. Void (either video): the needle touches the table or the arm before the site, or passes within 15 mm of the other
   gripper (the ruler in the frame gives the scale).

On a real arm the vein's rolling is the arm's own behaviour, not the rule used here: a slack-skin miss shows as no
flash, the same way. This procedure has not been tried on a physical rig.

## The demo

`demo.py` reaches 1.0 on both layouts. It never reads a part's state: every target comes from the layout constants
and the kit's sizes, and the only feedback is each hand's own measured position.

- **The right hand draws the skin.** Its closed fingertips lean 45 degrees toward the wrist, press on the patch 11 cm
  below the crease (sent 16 mm into it, which the arm's give turns into a press), and are then sent toward the wrist
  until the hand itself has moved 9 mm. It holds that command to the end.
- **The left hand sticks by feel.** It takes the holder from above, across the barrel, tips it so that the needle
  points up the arm 25 degrees below level with the hub's mark up, and brings the tip to 2 cm short of the skin on the
  needle's own line. Then it goes forward at 5 mm/s until it is 4 mm behind its command and no longer moving: the
  point is on the skin. It keeps raising its command until its own hand has moved on by the skin's thickness along
  the needle: the skin has given. It drops its command to 1 mm ahead of where it is, goes on at 3 mm/s to 9 mm past
  where the skin was, sets its command to where it is, and leaves it there.

| Step (layout A) | What happens |
|---|---|
| 90 | The right hand's fingertips are pressed on the skin patch (the press has pushed it 1.9 mm toward the elbow). |
| 115 | The left hand, having corrected for its sag over the cradle, has closed on the barrel. |
| 130 | The right hand has drawn the skin past 5 mm toward the wrist: the vein is held. It ends 7.6 mm drawn and stays there. |
| 215 | The holder is over the arm, tipped: the needle points up the arm, 25 degrees below level. |
| 260 | The tip is 2 cm short of the skin on the needle's own line. |
| 360 | The point is on the skin and meets 1.0 N. The hand stops; its command runs on ahead of it (6.8 mm at most). |
| 395 | The skin has given (crossed at 25.6 degrees, bevel 0.4 degrees off, 0.1 mm from the vein's line, 1.0 mm short of the crease). The command is dropped back; the tip is at the vein's near wall, 1.5 mm under the skin, where it meets 0.4 N: 0.5. |
| 420 | The tip is through the wall and in the lumen, against 0.1 N. |
| 440 | The tip is 3.5 mm under the skin, 9 mm of needle past where the hand felt the skin; the command is set to where the hand is. |
| 455 | The holder has been still for a second: 1.0. |
| 473 | Full score has held for 1 s: the trial ends as a success. |

For a moment near step 415 the score also read 1.0: the needle had been stopped at the vein's near wall for a second
with its tip just inside the lumen. It then went through the wall and the score was 0.5 again until step 455.

Layout B: on the skin at step 375, through it at 410 (crossed at 26.2 degrees, bevel 2.4 degrees off, 1.1 mm from the
vein's line), in the vein at 435, 1.0 at 490 with the tip 3.3 mm under the skin, success at 505. The skin was drawn
7.7 mm. Steps are read from a trace kept every five steps. In both runs the needle's nearest approach to the right
gripper was 46 mm, and it touched nothing before the skin.

The hand finds the skin itself: on layout B its own measure put the skin 2.4 mm further along the needle's line than
the layout says (the holder does not sit in the fingers exactly as planned), and the tip still ended in the middle of
the vein. What the arm's give does is part of the task: with the hand 6.8 mm behind its command when the skin gives,
a hand that did not ease off would drive the needle about that far on.

## Control runs

All on layout A.

| Run | Score | What happened |
|---|---|---|
| `--mode still` | 0.0 | Nothing moves (200 steps). |
| `--mode notraction` (the fingertips press on the skin but do not draw it) | 0.25 | The skin never moved toward the wrist (the press pushed it 1.9 mm the other way). A good stick at 25.6 degrees, but the vein was not held when the tip reached it (step 395): it is 5 mm aside and the tip, 3.5 mm under the skin, is not in it. |
| `--mode release` (the right hand lifts off once the needle is in and still) | 0.75 | In the vein and still at step 455; then the fingers left the skin, which sprang back from 7.6 mm to 0.3 mm. |
| `--mode letgo` (the left hand lets go of the holder once the needle is in and still) | 0.5 | From about step 470 the holder is in no gripper: it hangs in the arm by its needle, tip 3.9 mm under the skin, in the vein. |
| `--mode steep` (the same stick at 45 degrees) | 0.0 | The needle met the same 1.0 N, was pushed through at 46 degrees: a bad stick (void from step 397). |
| `--mode through` (the hand goes on to 18 mm of needle) | 0.0 | In the vein at step 395; pushed on through the far wall's 0.4 N at step 470, ending 6.6 mm under the skin. |
| `VEIN_BEVEL=down` (the holder starts turned over; the same moves) | 0.0 | The needle went in with its bevel 179 degrees off: a bad stick (void from step 394). |

## Dream-1

One run, 800 steps, layout A, after a 200-step test (also 0.0): **0.0**. From frames of the overhead video: the left
arm reached out toward the cradle and stayed beside it, and the right gripper moved out over the hand end of the
forearm. The holder was never taken out of its cradle, so the needle did not come near the skin.

## Building it for real

- **Training arm.** An adult venipuncture and injection training arm with fluid-filled veins at the elbow crease and
  flashback: the Life/form Venipuncture and Injection Training Arm (LF00698U). Strap it to the table palm up,
  shoulder end to the robot's left. Its listing gives no arm, vein or weight figures: measure the arm, find its vein
  at the crease, and set the site and the heights from what is measured. Its skin slides over its core when pressed
  and pulled; ink the line and the ticks 10 cm below the crease.
- **A lower-cost alternative.** Two ways, both strapped on a forearm-shaped block (a half-round of wood or foam 8.4 cm
  across at the site end, narrowing to 7.2 cm, 30 cm long, flat side down):
  1. A venipuncture training pad. One listing: a "Four-Vein Venipuncture Training Aid", "$ 149.60", "6.5 x 3.3 x
     1.4 in", "0.33 lb", with "four barely discernable blue veins in three different sizes"; "When accessing, the
     skin rolls as the vein is palpated." Its page does not say whether its veins can be filled to give a flash. If
     they cannot, use the second way.
  2. A built pad (sizes chosen to match the task; nothing here is a product's measurement): a 20 cm length of soft
     latex or silicone tubing, 4 mm bore, laid along the top of the block in a groove so that its middle is 3.5 mm
     under the skin; both ends led out, one to a raised bag of red-coloured water about 30 cm above the block and
     the other clamped, so that the tube is full and under slight pressure; over it a sheet of 2 mm silicone rubber
     as skin, glued down at the site and for 6 cm above it, left unglued from 8 to 15 cm below the site and held
     there by two elastic bands to the block's wrist end, chosen so that drawing the sheet 1 cm takes about 0.8 N on
     a spring scale. Replace the tubing and the skin over the site when they leak.
  Either way, mark the ink line and ticks, and measure the force to pierce the skin with the holder on a spring scale.
- **Holder and needle.** A single-use tube holder for 13 and 16 mm tubes with a 21 gauge, 1 inch multi-sample
  flashback needle (clear view window in the hub) screwed in, the sheath taken off just before the trial by the
  person setting up.
- **Cradle.** Any block with a V groove that holds the barrel level with its axis 3 cm above the table, needle
  pointing left, hub mark up, and leaves the barrel's sides free.
- **Protecting the gripper pads.** Fit thin stainless-steel or hard plastic guards over the pads of the gripper that
  draws the skin, so that a needle that strays cannot cut the rubber; leave the pads of the hand that holds the
  holder bare. Check the pads after every trial. Stop a trial in which the needle comes within 15 mm of the other
  gripper: it is void anyway.
- **Safety.** The needle is sharp and uncapped. Use a new sterile needle on a training arm or pad only, keep people
  out of the arms' reach during a trial, and discard the needle in a sharps container afterwards without recapping it.

## What the simulation simplifies

- **The needle's shaft** is a joint, not a body, as said above: it cannot bend, and it meets the skin only over the
  elbow (anywhere else on the arm or the table, a needle within 1.5 mm voids the trial).
- **Resistance** depends on where the tip is, not on how fast it moves or how sharp it is, and the same force resists
  drawing the needle back. The 1.0 N and 0.10 N are from foil tests, not from skin; the 0.40 N, the skin's 1 mm, the
  springs that hold the needle to its line (400 N/m and 1 N m/rad) and the damper (5,000 N s/m) are estimates.
- **Skin** is one rigid patch on a slide with a spring of 80 N/m and a friction of 0.8 under the finger (estimates). It
  does not stretch or wrinkle, and the skin at the site does not move with it.
- **The vein** rolling is a rule (above). The vein is straight, 4 mm across and 3.5 mm deep (estimates); no blood is
  simulated: "blood in the view window" is the tip being in the lumen.
- **Uncapping is not part of the task.** The needle starts uncapped in its cradle; in practice the sheath comes off
  just before the stick.
- **No tourniquet, no cleaning of the site, no tube.** The site is taken as prepared; the stick ends before a tube is
  pushed on.
- **The arm** is rigid and fixed, built from round sections: upper arm 8.4 cm across at the elbow to 10 cm, 27 cm
  long, with a 2 cm shoulder cap; elbow 8.4 cm, level for 6 cm either side of the crease; forearm narrowing to 5.8 cm
  at the wrist over 19 cm; a flat hand 18 cm long, 2.6 cm thick, with a thumb. All of these are estimates of an adult
  arm, not measurements of the named training arm.
- **The holder**: barrel 19 mm across and 48 mm long, flange 34 x 24 x 2 mm, hub 10 mm long (estimates of a
  single-use holder for 13 and 16 mm tubes); needle 21 gauge, 1 in (the catalogue designation: 0.8 mm across, 25.4 mm
  long). Holder and needle weigh 5 g here (an estimate); the simulated body weighs 20 g and a steady upward force
  carries 15 g of that, so that it rests in the fingers as a 5 g part does while the solver has a heavier body.
- **Touching** (the needle against the table or the arm, the needle near a gripper) is judged by distance once per
  control step (20 times a second), from the kit's shapes and from sample points on the grippers' fingers and palm.
- Friction: 0.6 between the holder and the grippers' pads (an estimate).

## Related tasks on the board

- "Fill a blood tube on a needle already in a training arm's vein and take it off without moving the needle"
  (accepted) starts with the needle in the vein. This task is the step before it: getting the needle there.
- "Give a 1 mL intramuscular injection into a deltoid trainer with a retractable safety syringe" (accepted) puts a
  needle in at 90 degrees, anywhere in a muscle site found from a landmark, and pushes a plunger. Here the needle goes
  in at a shallow angle into a 4 mm vein while the other hand keeps the skin drawn.
- "Fill four tubes and tick each one off" (accepted) fills sample tubes with a transfer pipette and keeps count; no
  needle and no arm.
- A pending task whose title begins "Fingerstick a trainer hand: twist the lancet's tab off, hold the ring finger"
  lances a fingertip while the finger is held; no vein is entered and no needle is held at an angle and a depth.

## Sources

- O*NET OnLine, 31-9097.00 Phlebotomists: "143,900 employees" (2025); "Draw blood from veins by vacuum tube, syringe,
  or butterfly venipuncture methods." 31-9092.00 Medical Assistants: "833,900 employees" (2025); "Collect blood,
  tissue, or other laboratory specimens, log the specimens, and prepare them for testing."
- MSD Manual Professional Edition, "How To Do Venous Blood Sampling"
  (msdmanuals.com/professional/critical-care-medicine/how-to-do-peripheral-vascular-procedures/how-to-do-venous-blood-sampling):
  "Apply gentle traction to the vein distally using the thumb of your nondominant hand to prevent the vein from
  moving." "Insert the needle proximally (ie, in the direction of venous blood flow), with the bevel facing up,
  along the midline of the vein at a shallow angle (about 10 to 30 degrees) to the skin." "Blood will appear in the
  needle hub (called a blood flash ) when the needle tip enters the lumen of the vein." "Take care not to puncture
  too deeply and go through the vein." "Keep the needle motionless."
- Needle forces: a needle maker's white paper on penetration force tests to DIN 13097-4
  (gbo.com/fileadmin/media/GBO-International/02_Downloads_Preanalytics/SALES_White_Papers/WP_Penetration_Force_Semi_Automatic_SBC_Sets_MELAB1_2Rev_2.pdf):
  "Testing Medium: PU Foil 0.4 mm", "Testing Speed: 100 mm/min"; for three 21 G (0.8 x 19) blood collection sets, ten
  needles each: F0 (tip penetrates) 0.75, 0.55, 0.75 N; "Peak penetration force (F1/F2)" 1.00 (0.85-1.12), 1.25
  (1.10-1.56), 1.00 (0.80-1.20) N; FR (shaft glides) 0.10 (0.07-0.13), 0.17 (0.07-0.24), 0.06 (0.01-0.13) N.
- Flashback needle: a trade article on one maker's multi-sample needle with a view window
  (healthcare-in-europe.com/en/news/vacuette-visio-plus-needle.html): "The needle has a translucent plastic hub,
  which functions as a view window. If venipuncture is successful, the view window is coloured red due to the blood
  flow."
- Training arm: a distributor's listing of the Life/form Venipuncture and Injection Training Arm, LF00698U
  (boundtree.com/training-simulation/trainers/life-form-venipuncture-and-injection-training-arm/p/group004652):
  "35in x 12in x 6in"; "Veins are located at common sites, including the antecubital fossa, along the forearm, and at
  the back of the hand."; "life-like durable vein roll-over"; "Realistic flashback". It gives no weight and no arm or
  vein sizes.
- Lower-cost pad: a maker's listing of a four-vein venipuncture training aid
  (3bscientific.com/us/four-vein-venipuncture-training-aid-dermalike-ii-latex-free-1017971-w46519-vata-1354,p_34_17201.html):
  "$ 149.60"; "Dimensions: 6.5 x 3.3 x 1.4 in"; "Weight: 0.33 lb"; "four barely discernable blue veins in three
  different sizes"; "When accessing, the skin rolls as the vein is palpated." It gives no vein sizes and does not
  mention fluid.
- The World Health Organization's "WHO guidelines on drawing blood: best practices in phlebotomy" is the usual
  reference for these steps; its pages could not be fetched when this was written, so nothing here is quoted from it.
