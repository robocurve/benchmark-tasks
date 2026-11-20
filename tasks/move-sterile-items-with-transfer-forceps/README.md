# Move scissors, a syringe and a medicine cup onto a sterile field with thumb forceps

A treatment-room counter is being set up for a minor procedure. A Mayo instrument tray under a sterile drape is the
sterile field. Beside it lies a sterile supply pack that has been opened flat, with three things on the inside of its
wrapper: iris scissors, a 1 mL syringe and a stainless medicine cup. Sterile thumb dressing forceps lie across the near
edge of the field, their handle sticking out over the edge. The robot has to move the three items onto the field with
the forceps and lay the forceps back down.

The robot's grippers stand for a nurse's or medical assistant's bare hands: they are not sterile.

## Why it needs the tool, and what makes it hard

Sterile technique as nursing and medical-assisting texts teach it (sources at the end) comes down to four rules, all
of them scored here:

- **Sterile touches only sterile.** The three items, the working end of the forceps and the inside of the field and
  the pack are sterile. The grippers, the table and the forceps' handle are not. Whatever sterile thing touches an
  unsterile one is contaminated, and contaminates what it touches next.
- **The outer 2.5 cm (1 in) of a sterile field is not sterile.** An item has to rest wholly inside that border.
- **Nothing unsterile passes over the field.** No part of either arm may be above the sterile part of the field or of
  the pack, at any height. Only the forceps reach in.
- **Forceps are held tips down.** While they are held, the tips may not be raised above the handle.

So the gripper cannot take an item, cannot hover over one, and cannot hold the forceps the easy way (straight down
from above, which puts the hand over the field). What works is what a person does:

- Take the forceps by the handle that sticks out over the edge and hold them as an extension of the fingers, pointing
  forward and down, so that the tips are ahead of and below the hand.
- Work them by squeezing: the pads' gap sets the tips' gap (3.2 mm at the pads moves the tips 15 mm), and the pads'
  force sets the grip (worked out from the fingers' position gain and the lever, not measured).
- Take each item the way its shape allows. The three are different:

| Item | Where the tips close | What it asks for |
|---|---|---|
| Medicine cup, standing | On its wall at the side, one tip inside and one outside, just under the rolled rim | A 35 g cup held at one point of its rim hangs tipped (28 to 36 degrees in the demo runs); it is set down from that tilt and rights itself. Forceps 44 degrees down. |
| 1 mL syringe, lying | Astride the barrel (6.9 mm) at its balance point | The tips' middle has to come down to 4 to 5 mm above the pack, with 1.5 mm of room either side of the barrel. Held at its balance point the syringe is carried level. It lies 9 cm long pointing away from the robot, so the forceps have to be held flatter (34 degrees down) to reach its middle while the hand stays in front of the pack. |
| Iris scissors, lying flat | Astride one shank (3.5 mm wide, 3 mm tall) where the scissors balance | The tips' middle has to come down to about 2 mm above the pack, one tip between the two shanks. The shank that is held is 8 mm to one side of the scissors' centre of mass, so the squeeze has to keep them from rolling (2.5 N at the tips in the demo). Forceps 34 degrees down. |

- Lay the forceps back with their working end on the field and their handle out over the edge.

Soft dressings (gauze sponges, cotton balls) are what forceps move most often in practice. They are left out: nothing
in this simulation models cloth, and a rigid stand-in would not be picked up the way gauze is.

## Files

| File | What it is |
|---|---|
| `task.py` | Layout, the contamination record kept every control step, the scoring rule. |
| `xferkit.py` | The forceps (two bowed blades on a sprung hinge), the cup, the syringe, the scissors, the arms' shapes read from the robot's URDF. |
| `demo.py` | A scripted demo that reaches full score, and five control runs. |
| `test_scoring.py` | 52 checks of the shapes, the contamination record and the scoring rule; no simulator needed. |
| `task.json` | The task's description for the board. |

## Running and scoring a trial

```bash
# a policy (Dream-1 here), 2,400 steps at 20 Hz; the score is printed at the end and written to the log
inspect-robots run --embodiment yam_arena --policy dream -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 2400 \
  --instruction "Using the sterile forceps, move the scissors, the syringe and the medicine cup from the opened sterile pack onto the sterile field without contaminating anything, then lay the forceps back across the field's edge and let go."

python demo.py task.py out_dir                 # the scripted demo: videos, setup.png, result.json
XFER_LOT=B python demo.py task.py out_dir      # the second layout
python -m pytest -q test_scoring.py            # the offline checks
```

- `object_jitter=0` is required: the field and the pack are fixed and the layouts are exact.
- There are no seeds. `XFER_LOT` picks the layout: A or B. In B the cup and the syringe change places on the pack (to
  within 0.5 cm), the scissors lie 0.3 cm further away and 0.7 cm to the left, and the forceps 0.5 cm to the left.
- A trial of 2,400 steps is 120 s of robot time. The demo takes 1,576 steps (79 s). On one L40S a demo run took
  about 14 minutes of wall time with two other runs sharing the GPU (about 7 alone, estimated from a shorter run);
  the 2,400-step Dream-1 run took 18 minutes with other runs sharing the GPU.
- `result.json` from the demo holds the score, what was contaminated and why, and a trace every five steps.
- The overhead camera's view of the pack is partly hidden by the right arm at rest. The right wrist camera's first
  frame shows all three items on the pack and the forceps' tips on the field.

## Scoring

Scored at the end of the trial, from a record kept at every control step.

| Score | State at the end |
|---|---|
| 0 | The field is contaminated, or no item rests on it. |
| 0.25 | One item rests on the field's sterile part, uncontaminated, let go, at rest. |
| 0.5 | Two items do. |
| 0.75 | All three do. |
| 1.0 | All three, and the forceps have been put down and let go and lie at rest, uncontaminated, with their whole working end resting on the sterile part of the field or of the pack. |

The numbers:

- **Touching** is being within 1.5 mm; for any part of an arm, within 4 mm.
- **Sterile:** the scissors, the syringe and the cup; the forceps' working end (the 5 cm from the tips); the tops of
  the field and of the pack more than 2.5 cm from their edges.
- **Contaminated** is anything sterile that touches the table, a 2.5 cm border, the tray's side, an arm, the forceps'
  handle (the 6.5 cm from the tail), or anything already contaminated. The field or the pack is also contaminated when
  any part of an arm is above its sterile part. The forceps are also contaminated when, while they are held (an arm
  within 3 cm of them), their tips are more than 5 mm higher than the end of their handle.
- **An item rests on the field** when every part of it is inside the border and it is on the field's surface: the cup
  upright within 10 degrees; the syringe and the scissors lying, with no part more than 2.5 cm above the field (so
  not propped on the cup). **Let go:** both arms 3 cm from it and the forceps' tips 2 cm from it. **At rest:** under
  2 cm/s.
- **The forceps are put down** when both arms are 3 cm from them and they are at rest.

"Any part of an arm" means the collision shapes of every link from the upper arm to the fingertips, both arms,
placed where the simulator has them (`xferkit.arm_points`). A contaminated field scores 0 whatever lies on it:
in practice the set-up is thrown away and started again. The score depends only on the state at the end and on what
was touched; nothing is added up over time.

Unintended ways through, and what they score: carrying an item in the gripper, 0 (run below); pushing or dragging an
item across with the forceps, 0, because it crosses the pack's border, the table and the tray's side; reaching deep
into the field with the forceps, 0, because the hand follows them over it (run below).

## Scoring a real trial

From an overhead video (which shows whether an arm was above a sterile part: mark the 2.5 cm borders on the drape and
the wrapper with tape under the cloth, or measure them on the video) and a side view (touches, and whether the tips
were raised). The end state is read from the last frame with a ruler: each item inside the taped border, the cup
standing, the scissors and the syringe lying flat. This has not been tried on a physical rig.

## The demo

Only the right arm works. Its only feedback is its own measured position and attitude, used to correct for the arm's
sag (for the scissors and the syringe the correction is worked out at the forceps' tips); it never reads where an
item or the forceps are.

The scissors are not picked up by a finger ring. That was tried first, at the back of one ring, and dropped after
five simulator runs: with the forceps 44 degrees down (steeper would put the hand over the pack) only the last 1 to
2 mm of the blades' lower corners reach a 3.5 mm bar that lies on the pack, and the scissors, lifted by one ring,
rolled and slipped off the tips.

| Step | What happens (layout A) |
|---|---|
| 115 | The hand, leaning 72 degrees forward, has closed on the handle; the tips are about 6.5 mm apart. |
| 180 | The handle has been lifted and the forceps drawn back, pointing 44 degrees down. They are then held flatter, 34 degrees down. |
| 370 | The tips close on the scissors' shank. |
| 560 | The scissors lie on the field: score 0.25. |
| 740 | The tips close on the syringe's barrel. |
| 945 | The syringe lies on the field: score 0.5. |
| 1,165 | The forceps point 44 degrees down again; the tips close on the cup's wall. |
| 1,325 | The cup stands on the field: score 0.75. |
| 1,560 | The forceps lie across the field's edge again and the hand is 3 cm away: score 1.0. |
| 1,576 | Held for a second: success. |

Layout B: 0.25 at step 560, 0.5 at 945, 0.75 at 1,320, 1.0 at 1,560, success at 1,577. Steps are read from a trace
kept every five steps. In both runs nothing was contaminated; the scissors and the syringe stayed within 3 degrees of
level while they were carried, and the cup hung tipped up to 36 degrees.

## Control runs

All on layout A.

| Run | Score | What happened |
|---|---|---|
| `--mode still` | 0.0 | Nothing moves (200 steps). |
| `--mode hand` (the gripper takes the cup and sets it on the field) | 0.0 | The cup ends up standing on the field, but the hand passed over the pack (step 13) and the field (step 158) and held the cup. |
| `--mode over` (the demo's cup routine alone, the cup set 5 cm further into the field) | 0.0 | The cup ends up standing on the field, touched by nothing unsterile, but the hand followed the forceps over the field's sterile part (first at step 411). |
| `--mode cup` (the cup only, then the forceps laid back) | 0.25 | The cup on the field from step 510; nothing contaminated. `--mode scissors` and `--mode syringe` do the same with one of the other items: 0.25 each (from steps 560 and 570). |
| `--mode keep` (all three items, the forceps then kept in the hand) | 0.75 | All three on the field from step 1,325, nothing contaminated; the forceps were never put down (run to 1,500 steps). |

## Dream-1

One run, 2,400 steps, layout A, after a 200-step test (also 0.0): **0.0**. Both arms drew back toward their bases in
the first few hundred steps and stayed there, the right hand in front of the pack. The forceps were not picked up, and
the scissors, the syringe and the cup were still on the pack at the end.

## Building it for real

- A Mayo instrument tray (49 x 32.5 x 1.9 cm, the standard size) under a sterile drape or towel folded to the tray's
  size, taped so that it does not slide.
- A 30.5 cm (12 in) square of sterilization wrap taped flat.
- Straight iris scissors, 11.5 cm (4.5 in), closed, laid flat with one shank pointing straight away from the robot.
- A 1 mL luer slip syringe without needle, plunger pushed in.
- A 2 oz stainless graduated medicine cup (5.1 cm across, 4.45 cm tall).
- 14 cm (5.5 in) stainless thumb dressing forceps with serrated tips, laid on edge across the tray's near edge with
  5 cm of handle overhanging. A strip of tape on the drape marks where the tips go.

Measure the real scissors and syringe before a real trial: apart from their catalogue sizes, their shapes here are
estimates (next section).

## What the simulation simplifies

- Everything is rigid. The drape and the wrapper are flat surfaces; there is no cloth and no fluid.
- The forceps are two rigid bowed blades on a sprung hinge at the tail. Real ones are one piece of spring steel that
  bends along its length. The closing force (2.5 N at the finger grip), the blade thickness (1.5 mm), the tips' gap at
  rest (15 mm) and the mass (25 g) are estimates, not measurements.
- The cup's diameter and height are the catalogue's. Its base diameter (3.7 cm), wall (0.8 mm), rolled rim and mass
  (35 g) are estimates. Its wall is made of 20 flat pieces.
- The syringe's barrel (0.27 in across, 2 11/16 in long) is from a distributor's table. Its slip tip (4 x 10 mm),
  finger flange (20 x 11 x 1.5 mm), the plunger rod showing behind the flange (11 mm), the thumb rest (10 mm across)
  and its mass (2 g) are estimates. It is one rigid piece: the plunger does not move. The simulated body weighs 10 g
  and a steady upward force carries 8 g of that, so that it presses on what holds it as a 2 g syringe does while the
  solver has a heavier body to work with; it therefore speeds up and slows down more sluggishly than a real one.
- The scissors' length (115 mm) is the catalogue's. Everything else about them is an estimate from pictures: rings of
  3.5 mm round bar, 26 x 22 mm outside; shanks 3.5 x 3 mm; the screw 70 mm from the back of the rings; the closed
  blades 7 mm wide at the screw; 20 g. They are one rigid piece, closed. Each ring is 12 straight pieces of bar.
- Friction between the serrated tips and what they hold is 0.6 (an estimate).
- A gripper pad is rubber and meets a blade over a patch; the simulator's pads are rigid, so the blades are given a
  4 mm contact patch.
- Touching is judged by distance once per control step (20 times a second), not from contact forces: a touch shorter
  than a step can be missed, and passing within 1.5 mm counts as a touch.
- The forceps start lying on edge, the two blades side by side. Dropped, they may fall onto one blade's flat face,
  where a parallel gripper cannot squeeze them.

## Related tasks on the board

- "Peel open a sterile pouch with both hands and drop its forceps onto the sterile field" (pending) and "Transfer
  sterile forceps from their peel pouch onto a sterile drape" open a package and drop its contents. Here nothing is
  dropped: the forceps are the tool, and they carry other things.
- "Lay a surgical instrument set on a sterile field without touching its unsterile border" and "Set instruments on a
  sterile field and keep them off its unsterile border" place things on a field with the gripper. Here the gripper
  may touch neither the things nor the field, and may not pass over it.
- "Draw a splinter out of a fingertip with tweezers" and "Take out an interrupted suture: forceps lift the knot, a
  stitch cutter cuts flush" (both pending) also work a pair of forceps. They pull a small thing out; this carries
  three different things under sterile rules.

## Sources

- O*NET OnLine, 29-1141.00 Registered Nurses and 31-9092.00 Medical Assistants (task statements and 2025 employment).
- G. R. Doyle and J. A. McCutcheon, *Clinical Procedures for Safer Patient Care* (BCcampus), section 1.5 "Surgical
  Asepsis and the Principles of Sterile Technique". The four rules scored here, in its words: "Place large items on
  the sterile field using sterile gloves or sterile transfer forceps." "Once a sterile field is set up, the border of
  one inch at the edge of the sterile drape is considered non-sterile." "Non-sterile items should not cross over the
  sterile field. For example, a non-sterile person should not reach over a sterile field." "Keep the tips of forceps
  down during a sterile procedure to prevent fluid travelling over entire forceps and potentially contaminating the
  sterile field."
- OpenStax, *Fundamentals of Nursing* (2024), section 10.3 "Sterile Technique": the 1 in (2.5 cm) border of a sterile
  package.
- Open RN, *Nursing Skills*, section 4.4 "Sterile Fields": "Any part of the field within 1 inch from the edge" is
  treated as contaminated, and no part of the body is to be directly over the field.
- Thompson Rivers University School of Nursing, "Principles of Asepsis" (video transcript): the 2.5 cm border, sterile
  forceps tips arranging and adding items, the handles kept at the border.
- Catalogue sizes: 2 oz stainless graduated medicine cup, 2 in across by 1 3/4 in tall; Mayo instrument tray,
  19 1/4 x 12 3/4 x 3/4 in; thumb dressing forceps, 5 1/2 in.
- Straight iris scissors: "4.5"-115mm long" (indigoinstruments.com/straight-iris-scissors-medical-stainless-22208.html).
  The page gives no other size and no weight.
- 1 mL luer slip polypropylene syringe: barrel OD 0.27", ID 0.18", Lg. 2 11/16" (the plastic syringe table at
  mcmaster.com/products/syringes/capacity~1-ml/). The same distributor's 3 mL syringes are 0.43 to 0.47 in across,
  too wide for these forceps to go astride, which is why the syringe here is a 1 mL one.
