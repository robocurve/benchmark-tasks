# Pass a sterile device cord off the field to the console

An operating-room bench set-up (no patient present; the risk it stands for is infection of the patient whose procedure the field serves). The left half of the table holds a raised sterile field: a 5 cm tall box
of plywood under a blue cotton drape whose right-hand edge lies on the table's midline. A white PVC-jacketed 8 mm, 40 cm device cord leaves a
white polypropylene holster fixed on the drape and lies straight across it. It ends in a black ABS 125 x 16 x 14 mm connector (plug body
with its moulded strain-relief grip; long enough for the far arm to grip it on the tray). A grey stainless-steel cord holder sits at the field's right edge, on a line aimed at the
console. It has an open-top slot on the drape (8.5-12.5 cm in from the edge, measured along that line) and a flat
hand-off tray at drape height that reaches 17.5 cm beyond the edge. A grey device console (moulded ABS housing, 12 x 16 x 7.5 cm) stands on the right half;
its jack faces the holder, 18 cm beyond the field edge along the same line.

The left arm plays the scrubbed (sterile) team member and keeps its gripper on the field side of the edge; its gripper
is its sterile glove, and once it touches anything non-sterile it must not touch the field or the sterile cord again. The right
arm plays the non-sterile circulator: no part of it may come over the field, at any height. Cord that has become
non-sterile must never come back over the field. The left arm lays the cord in the holder slot and leaves the
connector on the tray, pointing off the field. The right arm takes it from the far end of the tray, draws it further
out, re-grips it and pushes it fully into the console jack. The connector must never touch the table, and both arms
let go at the end. The instruction gives only the goal and says which arm and which surface are sterile. The rules
are tested, and listed under Scoring.

Why two arms, and why the hand-off is wide: the right arm's wrist housing sits about 8-9 cm to the side of its
gripper centre, towards the midline. To keep every part of that arm off the field, its gripper centre must stay
about 9 cm or more beyond the field edge. The left arm's gripper stays on the field side. The plug therefore has to
bridge that gap on the tray, and the left arm cannot carry it to the jack (its gripper would have to cross the edge).

Grounding: O*NET 29-2055.00 Surgical Technologists, task "Maintain a proper sterile field during surgical
procedures." The OpenRN Nursing Skills chapter "Sterile Fields" says to be "careful to not allow any part of your body
to be directly over the field", and lists among the "areas of sterile fields that are considered contaminated": "Any
part of the field that extends below the planar surface (i.e., a drape hanging down below the tray tabletop)." The
scorer applies these ideas as measurable rules: no part of the non-sterile arm over the field, and cord that has
touched a non-sterile surface (outside the field at or below drape height, the console, the non-sterile arm) never
comes back over the field, and the sterile gripper, once it has touched anything non-sterile, never touches the
field or the sterile cord again. A contaminated field is an infection hazard to the patient the field serves, so the robot
must keep the sterile and non-sterile sides apart itself. Handing a sterile cord end to a non-sterile colleague who
connects it to a console is ordinary operating-room practice; the 12.5 cm connector and the pull-then-re-grip are
choices made for this robot (its wrist housing, below), not claims about a particular device. The bench has no patient
and no live electrics.

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 installed (`source IsaacLab-Arena/yam.env`).
- `cablekit.py`, `cabledemo.py` and `vision.py` are in this folder (cord simulation, demo runner, camera helpers).

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1200 \
    --instruction "Plug the sterile white cord into the grey console's jack, with the cord lying in the grey holder's slot. The left arm and the blue drape are sterile, the right arm is not. Let go with both arms at the end."
```

## Scripted demo

```bash
python cabledemo.py task.py demo_out --max-steps 1200
```

The demo chooses its motions from the top camera image, the arms' own poses and the fixed layout only. It finds
the connector as a dark blob, back-projects it to the plane of its top face and takes its centre, heading and ends.
It never reads object poses from the simulator. The left arm moves out of the camera's view and looks. It pinches
the connector 4 mm from its cord end and turns it to point along the hand-off line. It brings it in along the
holder line from behind the slot, so the cord trailing behind it lies in the slot. It sets it on the tray with its
own gripper centre 15 mm inside the field edge, so its finger pads (13 mm either side of the centre along the
connector) end about 4 mm inside the edge and touch only connector material still over the field; then it moves
back. With both arms out of view, the camera looks again.
The right arm pinches the connector 10 mm from its tip, at the far end of the tray, and draws it 5 cm further out.
It lets go and clears the view, and the camera looks a third time. The right arm then re-grips 45 mm from the tip
(wrist camera towards the jack), pushes it along the line into the jack, lets go and backs off.

## Scoring

Tiers by end state (cord in the holder slot and both arms let go in each case): 1.0 connector tip at least 15 mm
inside the jack, lined up (see below); 0.3 connector resting on the tray with its centre beyond the field edge, not in the jack;
otherwise 0.

`score()` checks every control step. Any break is latched until reset and gives 0:
- **Scrub arm zone**: the left gripper centre must stay at y >= 0 (the field side of the edge). The left arm is the
  sterile (scrubbed) side, whose hands and forearms work over the field; its base sits beside the field and its upper
  arm necessarily passes the near edge, so the zone rule is on the gripper centre. Its gripper is also checked as a
  glove, by its geometry (next rule).
- **Sterile glove (left gripper)**: the scorer places the collision-hull points of the left gripper housing and both
  fingers (read from the arm model like the right arm's, from the live body poses) every step. The glove becomes
  non-sterile, and stays so, the first time any of these points: (a) is off the drape's footprint at or below drape
  height + 3 mm (the bare table, the drape's sides below the planar surface, the tray beyond the edge); (b) is within
  3 mm of the console or jack sleeve; (c) is within 1 cm of a right-arm hull point; (d) touches non-sterile cord or
  connector material (a glove point within 9.5 mm of a non-sterile cord point's centre, i.e. 3 mm from the cord
  surface plus half the 5 mm point spacing; or within 11 mm of the connector's axis along its length, which counts as
  touching the connector point nearest to it along the axis). After that, the run scores 0 if any glove point touches
  the drape (over its footprint at or below drape height + 3 mm), the holder slot walls or the holster (within 3 mm),
  or any cord or connector material that is still sterile (same test as (d)). The gripper centre plays no part in
  this rule, so a gripper whose centre stays on the field but whose fingers touch the table beside it is caught.
- **Non-sterile arm geometry**: for every rigid body of the right arm (base, link1-link5, gripper body and both
  fingers), the scorer reads the live body pose each step. It places that body's collision-hull vertices (read from
  the arm model, one per 4 mm cell, about 3,700 points per arm). No point may lie over the drape's footprint
  (x 8-52 cm, y 0-48 cm) at any height above the table. Every step the quaternion layout of the body poses is
  checked against the reported gripper position (within 4 mm); a mismatch is an error and gives 0. The log names the
  first link that broke this rule.
- **Cord contamination**: the scorer follows fixed material points of the cord: every segment centre plus three
  points between each pair, and 33 points (3.9 mm apart) along the connector. A point becomes non-sterile, and stays so, once any of
  these happens: (a) it is off the drape's footprint with its underside at or below drape height + 3 mm (lying on the
  tray beyond the edge, on the table, or hanging lower); (b) it is within 5 mm of the console or jack sleeve (up to 12 mm above the console top); (c) it
  is within 1 cm (of the cord surface or connector side) of any right-arm hull point. If any non-sterile point later
  comes 5 mm or more over the field edge, the run scores 0.
- **Connector on the table**: the connector's lowest point must stay more than 2 mm above the table.
- **Hand-off off the field**: when the right gripper centre first comes within 3 cm of the connector centre, that
  centre must already be off the field (y < 0).

At the end it also gives 0 if no cord segment lies inside the holder slot, or if a gripper centre is within 4.5 cm
of the connector's centre line (end to end) or 3 cm of the cord. A step where the arm or cord poses cannot be read
counts as a break. Otherwise: 1.0 when the connector tip is at least 15 mm inside the jack and lined up with it
(centre within 8 mm of the jack axis sideways and vertically, with the horizontal part of the connector's unit axis having a dot product of at least 0.9 with the jack axis, i.e. within 25.8 deg of it). 0.3 when the connector lies on the tray
with its centre beyond the field edge (the left arm's half done). 0 otherwise. The episode ends early with success
once 1.0 has held for 1 s with the cord at rest.

Shortcut variants (test only, `STERILE_CHEAT=<name> python cabledemo.py ...`). All score exactly 0 except `tier03`:

| variant | what it does | why it scores 0 |
|---|---|---|
| `nothing` | waits | nothing in the jack, cord not in the slot |
| `lean_over` | right gripper centre stays at y = -3 cm, its wrist leans over the drape; then the full run | non-sterile arm geometry over the field |
| `drag_back` | after setting the connector on the tray, the scrub arm drags it 5 cm back over the field, sets it out again; then the full run | non-sterile cord points back over the field |
| `reach_over` | right arm takes the connector by its cord end at the field edge | right arm over the field; connector material it made non-sterile comes back over the field |
| `scrub_carry` | left arm carries the connector past the edge itself | left gripper left the field |
| `noclip` | left arm comes in from the side, the cord misses the slot | cord not in the slot |
| `drop` | right arm drops the connector beside the tray onto the table | connector touched the table; not in the jack |
| `hold` | seats the connector and keeps gripping it | not let go |
| `hold_cord` | seats it; the scrub arm closes on the cord near the slot, then the right arm lets go and backs off | not let go |
| `glove_touch` | left gripper, centre at y = +57 cm (on the field side of the edge), lowers its fingers onto the bare table beyond the drape's far edge, lifts, then does the full run | non-sterile left gripper then touches the sterile connector on the drape |
| `tier03` | left arm's half only, connector left on the tray | 0.3 (partial-credit check) |

### Checking the rules in a real trial

The arm and glove rules use only the robot's own joint encoders and its collision model: forward kinematics gives the
same hull points of every link each control step, and they are compared with the measured layout (drape footprint
and height, console and holder positions, surveyed once before the trials). The cord rules need the cord's path: an
overhead camera tracks the cord, which carries narrow coloured bands every 2 cm and a marked connector, and its path
is fitted each frame; the same distance tests then mark cord points and the glove as non-sterile. As an independent
check that needs no tracking, each trial can also be audited the way aseptic technique is taught: fluorescent tracer
powder is dusted on the non-sterile surfaces (the bare table, the drape's sides, the tray beyond the edge, the
console and the right gripper) before the trial, and after it a UV lamp shows any powder on the drape, on the part of
the cord still on the field, or on the left gripper's fingers. Powder there means a non-sterile contact was carried
onto the field or the sterile cord, and the trial scores 0. The connector's tip end may carry powder, as it rests on
the tray and is handled by the right arm.

## How this differs from similar tasks

Unlike tasks where a part is simply handed from one arm to the other, a boundary splits the arms by role. The left
arm's gripper may never leave the draped area, and no part of the right arm may come over it. So the connector is
set on a tray past the field edge, and the right arm takes it from there. Cord that has left the field must not come
back. The cord must also end in a holder slot and the connector must never touch the table. This tests asymmetric,
constraint-defined arm roles with a cord, not a free handover.

## What changed from the earlier version

The previous review said: "The supposedly sterile gripper can touch an unsterile surface outside the drape footprint
while its centre remains at y >= 0, then handle the sterile cord without penalty", and that the scoring "does not
detect contamination of the left gripper". It asked for a sterility check on the left gripper, a description of how
it is checked in real trials, and a rerun of the full-score demonstration. Changes:

- New **sterile glove** rule (above), on the left gripper's actual geometry (housing and finger hull points, not its
  centre): touching anything non-sterile, including contaminated cord, latches it non-sterile, and touching the
  drape, slot, holster or sterile cord after that scores 0.
- The connector is now followed at 33 points instead of 9, so the glove test can tell which part of the connector
  the pads touch.
- The right arm's rule was already on the hull points of every link; it is unchanged.
- The scripted demo now grips the connector 4 mm from its cord end and sets it down with its gripper centre 15 mm
  inside the edge, so its pads never touch the part of the connector lying on the non-sterile tray. With the old
  placement they did, and the new rule scored that run 0.
- New shortcut variant `glove_touch` (scores 0). "Checking the rules in a real trial" (above) is new.

## Runtime and seeds

Run limit 1,200 steps (60 s at 20 Hz). The scripted demo scored 1.0 and ended in 636 steps. The layout is
fixed with `object_jitter=0`, and the cord is not jittered. `VARIATION` in task.py lists the ranges for a seeded
variation runner; the demo finds the connector from the camera wherever it lies.
