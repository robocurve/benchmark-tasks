# Get an angle grinder ready for its next user: flip the guard's cam lever open, turn the guard, flip it shut

A cordless angle grinder lies upside down on the bench with a grinding wheel's stand-in on. Its tool-free guard is turned the wrong way:
the open side faces back along the motor housing and side handle, where the next person's hands will be, so a burst
wheel or a shower of sparks would reach them. The robot is only told that a welder grinds weld seams flush with the
grinder next and to get it ready. For grinding with a Type 27 wheel, safety rules (OSHA 1910.215, ANSI B7.1) and the
grinder's own manual say the guard's closed side goes toward the operator, locked; a safety officer stops a grinder
handed over like this on sight. Doing it is physical: flip the guard's cam lever open over its sprung detent, carry the
guard about 135 degrees round the spindle on a free 1.9 kg tool without dragging the tool, and flip the lever shut so
the band locks again. Forcing the guard round with the lever still shut, or turning the whole grinder instead, earns
nothing.

The nearest board tasks set a bench grinder's tool rest and tongue guard gaps (screws on a fixed machine, checked
with a feeler) and free a circular saw's lower guard from a shim (releasing a propped spring guard). This one differs
in ability: a lever flipped over a sprung detent, a locked part carried about its axis on a free tool steadied by the
other hand, and a relock. The folder and file names come from the first version's title and are kept so the pull
request stays the same; the lock is the cam lever described below.

## How the real guard locks (Makita's manual)

Makita's instruction manual for the XAG03 and XAG04 (document 015078, page 9, "Installing or removing wheel guard",
section "For tool with clamp lever type wheel guard", figures 015084 and 015085) shows the guard this tool ships with:
a band round the bearing box, a hinged lever on the band (item 4 in figure 015084) and an adjusting screw (item 3).
Its steps: "Loosen the lever on the wheel guard ... Then rotate the wheel guard to such an angle that it can protect
the operator according to work." "Tighten the lever to fasten the wheel guard. If the lever is too tight or too loose
to fasten the wheel guard, loosen or tighten the screw to adjust the tightening of the wheel guard band." The figures
show the lever hinged on the band's ear about an axis parallel to the spindle: shut, it lies along the band; open, it
stands about 35 degrees away from it (figure 015085, arrow showing the closing swing). The guard turns to any angle
(stepless) while the lever is open and is locked while it is shut. Makita lists the guard as part 123144-0, "4-1/2 in
Tool-less Wheel Guard ... for model XAG03 ... a convenient lever that allows user to quickly install or adjust the
wheel guard without the use of any tools" (https://makitatools.com/products/details/123144-0). The adjusting screw is set once
and is not part of the task.

## Parts (all full size; store-bought except the printed stand-in wheel)

Positions in cm from the midpoint between the arm bases (x forward, y to the robot's left), heights above the table.

| Part | Real product, size and material | Where |
|---|---|---|
| Angle grinder | Makita XAG03 18 V LXT brushless 4-1/2 / 5 in angle grinder, bare tool (XAG03Z, US$179.00 at The Home Depot), battery out. Gear head painted cast aluminium 62 x 72 x 58 mm; 41 mm guard neck; teal motor housing 58 mm across, 170 mm long; black rear cap (battery mount) 54 mm across, 30 mm; black nylon side handle 23 mm across, 110 mm, screwed into the head's side. 1.9 kg with guard, wheel and handle (Makita lists 2.5 kg, 5.5 lb, with a 3.0 to 5.0 Ah battery) | upside down, spindle up at x 27, y -9; gear head back face down; housing along +y to y +14.5; handle toward +x |
| Wheel (stand-in) | a nonabrasive stand-in for a Type 27 grinding wheel: a smooth flat brown PETG disc printed from `fabricate.py` (`stand_in_wheel.stl`) at the size of the Makita 741423-B 4-1/2 x 1/4 x 7/8 in, 24 grit Type 27 grinding wheel: 115 mm outside diameter, 6 mm thick, 22.23 mm bore. No grit, so the gripper pads that pinch the skirt 6 mm above it are not worn. Held by the tool's steel inner flange and 38 mm lock nut | on the spindle, wheel top 8.3 cm up |
| Wheel guard | Makita 123144-0 4-1/2 in tool-less wheel guard for the XAG03 (US$20.59 at Zoro), bright zinc-plated steel, 0.25 kg: a band round the neck with an ear for the cam lever and its adjusting screw, a 190 degree plate and skirt (129 mm across, 25 mm deep, 2.5 mm wall, rim 10 cm up). A strip of orange tape (15 mm) marks the centre of its closed side | closed side starts pointing 45 degrees right of straight ahead (q = -135 deg, open side toward the housing) |
| Cam lever | the guard's own zinc-plated steel cam lever, 58 mm, hinged on the band's ear 32 mm from the spindle, 140 degrees clockwise of the tape (on the open side), about an axis parallel to the spindle. Shut, it lies along the band (75 degrees off straight out); it swings 35 degrees anticlockwise, away from the band, to open, over a sprung over-centre detent. A printed red PETG thumb cap (`lever_cap.stl`, 10 x 14 x 40 mm) on its end stands beside the wheel's edge and reaches 17.5 mm above the wheel's face | shut at the start |

## What the scripted demo does

First the left hand presses its closed fingers, leaned forward, on the rear cap to steady the tool. The right hand,
fingers closed, then comes down beside the cam lever's thumb cap and pushes it anticlockwise round its pivot, away
from the band. Its command sits a lead ahead of the cap along the push line, recomputed every step from the lever's
reading, and the lead grows (up to 40 mm) while the lever stalls, because the arm pushes through its own lag under
load. Once the lever passes the detent's midpoint the spring flips it open; the hand brings its command back to where
its fingers are before backing off, so nothing lunges. The demo then checks that the lever reads open, and pushes
again (up to three times) if it does not.

Only with the lever reading open does the right hand pinch the guard's skirt at its closed-side centre (10 mm of pad
on the 2.5 mm wall, pad tips 6 mm above the wheel) and carry it round the spindle at 12 deg/s, closing its command on
the guard's measured angle. It stops a stroke at once if the lever reading drops toward shut or the tool itself starts
to turn (4 degrees), so it never drags a locked guard or the whole grinder. It carries the guard in two strokes (to
q -60, then to about -5, since pushing the lever shut may carry the guard on a little) with a regrip between them, so
the wrist stays in its range. Before each release it unloads the arm, so the opening fingers do not drag the guard on.
If the guard stops short of a stroke's end, the hand reopens the lever if needed and tries the stroke again. The left
hand then lifts off and parks, the right hand pushes the cap back clockwise, toward the band, until the detent snaps
the lever shut and the lever reads shut (again up to three pushes), and both hands park well clear.

**What the demo reads:** the guard's angle, the lever's angle and the grinder's pose from the simulator (privileged
state), plus its own joint encoders and gripper opening. It never moves the guard, the lever or the tool except
through the fingers.

## Fidelity

Rigid bodies. The cam lever follows Makita's manual (page 9, figures 015084 and 015085): a straight 58 mm arm hinged
on the band's ear (the real arm is curved round the band; its pivot, length and swing are kept), 35 degrees of travel
read from the figures, and its over-centre action modelled as a detent spring (0.545 N m/rad, rests shut and fully
open, flips at the 17.5 degree midpoint). That spring needs 3.2 N at the thumb cap to carry the lever over its
midpoint. The source for that force is the manual itself: the lever's closing effort is set by the band's adjusting
screw ("If the lever is too tight or too loose ... loosen or tighten the screw"), and for this task the screw is set so
the lever flips with 3.2 N at the cap, inside the 3 to 4.5 N the arm's fingers push; a real trial checks it with a
small spring scale hooked on the cap. A thumb flips a stock-set lever with more, so this is the light end of the
screw's range, still enough to lock the guard: the cam's over-centre throw multiplies the lever force many times into
band tension, and band friction round the neck adds more.

The lock is modelled as joint friction (a hold) on the guard's hinge, set every control step from the lever angle:
2.5 N m shut (over 40 N at the skirt, more than a finger can give), falling linearly to 0.4 N m (the open band's
drag, 6.3 N at the skirt) between 0.1 and 0.35 rad open, as a real over-centre cam keeps the band tight until the
lever passes centre. So the guard turns, stepless, only while the lever is open. The hold is an effort, as the kit's
simulator runs measured; it acts on a vertical-axis hinge, so it carries no gravity load and does not creep. The
push that flips the lever puts 0.27 N m on the guard, under the open band's drag, so flipping does not turn the guard.
The guard's collar stop limits its turn to q -180 .. +48 degrees.

The wheel is a flat nonabrasive PETG stand-in at a Type 27 grinding wheel's size (115 x 6 x 22.23 mm): its depressed
centre and grit are not reproduced, so the pads that work next to it are not abraded; in the scene it is the same flat
6 mm disc, inert. The wheel does not turn and the motor is inert; nothing runs. The simulator scores the final step
without opening the grippers, while a real trial opens them first: a gripper pad within 3 cm of the guard or lever at
the end scores 0, and a grinder lifted 1 cm at any time scores 0, so a held tool never scores better than a released
one.

## Files

- `set_an_angle_grinder_guard_toward_the_operator_and_lock_its_band_clamp.py`: the task file for the `yam_arena`
  embodiment (scene, cam lever law, `score()`, `success()`, `demo_script()`).
- `mechkit.py`: jointed parts and procedural geometry (grinder, wheel, guard, cam lever). `closedloop.py`: the closed-loop
  scripted demo controller. `belimits.py`: shared geometry helpers (tilt).
- `fabricate.py`: print files for the cam lever's thumb cap, the nonabrasive stand-in wheel and the scoring gauge
  (see Real-world build).
- `task.json`: the task's details.

## Run a trial

```bash
INSTR="A welder grinds weld seams flush with this angle grinder next. Get it ready for them, then let go of it."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=set_an_angle_grinder_guard_toward_the_operator_and_lock_its_band_clamp.py -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/grinder-guard-dream

# Scripted demo (full score): right hand flips the cam lever open, turns the guard, flips it shut; left hand steadies the tool
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=set_an_angle_grinder_guard_toward_the_operator_and_lock_its_band_clamp.py -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/grinder-guard-demo
```

- **Seed and layout:** `-E object_jitter=0` is required: the grinder's 3 cm and 15 degree limits are measured from its
  start pose. `-E control_hz=20`: the cam lever law runs once per control step.
- **Runtime per trial:** the demo takes about 66 s of simulated time (about 1,320 steps), then holds still; the
  episode ends 8 s after the guard is locked toward the operator with both hands clear (`SUCCESS_HOLD_S = 8`). A
  Dream-1 trial runs up to 2,400 steps (2 minutes simulated).
- Wall-clock time on one L40S, including Isaac Sim start-up: about 360 s for the demo and about 350 s for a full Dream-1 trial.
- **Score:** `task_score` reports 1.0 when `success()` holds for 8 s (full score and the grinder still), otherwise
  `score()` at the end of the run.

## Scoring

q is the guard's turn from the motor axis: the angle, seen from above, between the orange tape's direction from the
spindle and the motor housing's direction (0 when the closed side faces the operator; 135 degrees at the start). In a
real trial read it from a top photo with a protractor, or with the printed gauge. The cam lever is shut when it is within
0.1 rad (6 degrees) of its shut stop (lying along the band).

1. **Score 0** if at any time the grinder body moved more than 3 cm, turned more than 15 degrees, tilted more than 10
   degrees or lifted 1 cm off the table; or the guard turned more than 10 degrees while its lever was shut (forced
   round against the locked band); or, at the end, a gripper pad is within 3 cm of the guard or lever.
2. **Otherwise:** 1.0 for q of 20 degrees or less with the lever shut; 0.5 for q of 20 degrees or less with the lever
   open; 0.25 for q between 20 and 60 degrees (lever either way); 0 for more.

Doing nothing scores 0 (q = 135). q is measured on the tool itself, so turning the whole grinder until the guard
faces the operator leaves q unchanged and scores 0 (and turning it more than 15 degrees is a fault anyway). Possible
scores: 0, 0.25, 0.5, 1.0.

## Real-world build

- **Parts list and cost (real-trial route):**

  | Item | Source | Price |
  |---|---|---|
  | Makita XAG03Z 18 V LXT brushless 4-1/2 in cut-off/angle grinder, tool only (ships with the 123144-0 cam lever guard, side handle, flange, lock nut and wrench) | The Home Depot, https://www.homedepot.com/p/Makita-18-Volt-LXT-Lithium-Ion-Brushless-Cordless-4-1-2-in-Compact-Cut-off-Angle-Grinder-w-Automatic-Speed-Change-Tool-Only-XAG03Z/205623191 | US$179.00 |
  | Spare guard, Makita 123144-0 (optional; the tool's own guard is used) | Zoro, https://www.zoro.com/makita-4-12-wheel-guard-123144-0/i/G3998946 | US$20.59 |
  | Red and brown PETG for the thumb cap (4 g) and stand-in wheel (75 g) | any 1 kg spool, about US$20 | about US$2 |
  | Orange vinyl tape, masking tape, 3M VHB 4910 strip | shop stock | about US$2 |

  Total about US$183 (US$204 with the spare guard). No battery is bought or fitted: the tool never runs.
- **Cam lever thumb cap (custom, carries the push):** `python3 fabricate.py` writes `stl/lever_cap.stl`
  (millimetres): a red PETG block 10 x 14 x 40 mm with a through slot 8.6 x 6.6 mm. It reproduces the end of the
  Makita 123144-0 guard's cam lever (its 35 degree swing, pivot on the band's ear and over-centre action come from the
  stock lever, per the XAG03 manual's figures 015084 and 015085, page 9) and only extends that end upward, because with
  the tool upside down the lever lies just under the wheel's plane where a gripper coming from above cannot reach it.
  Slide it onto the lever's free end so the slot is flush with the end, and bond it with a strip of VHB tape; measure
  your lever's end first and set `LEVER_SLOT` in `fabricate.py` if it differs from 8 x 6 mm. It adds 4 g at the tip.
- **Lever force:** with the cap fitted, hook a small spring scale (any 0 to 10 N luggage or fish scale) on the cap
  and pull it open square to the lever. Turn the band's adjusting screw (the manual's item 3, page 9) until the lever
  flips over its midpoint at 3 to 3.5 N. Then check that the shut guard cannot be turned by a firm hand push at the
  skirt; if it can, the screw is too loose.
- **Stand-in wheel:** the same run writes `stl/stand_in_wheel.stl`: a flat disc 115 mm across, 6 mm thick, 22.23 mm
  bore, the Makita 741423-B's (4-1/2 x 1/4 x 7/8 in Type 27 grinding wheel) outside diameter, thickness and arbor
  hole. Print it flat in brown PETG, 100 percent infill (about 75 g). It replaces the abrasive wheel so the pads,
  which pinch the guard's skirt a few millimetres above the wheel, never rub grit; the guard and cam lever are
  untouched by the swap.
- **Scoring gauge (optional):** the same run writes `stl/q_gauge.stl` (PLA or PETG, no supports): a 110 mm disc
  with a cup that fits over the 38 mm wheel lock nut, a tall fin to sight along the motor housing and short fins at
  plus and minus 20 and 60 degrees. It sits inside the skirt, clear of the wheel; set it on after the run, read where
  the orange tape falls, lift it off.
- **Assembly:** fit the stand-in wheel on the inner flange and tighten the lock nut by hand as the manual shows for a
  wheel; fit the guard as the manual's page 9 shows (lever open, the band's protrusion in the bearing box's notch);
  fit the thumb cap and set the lever force (above); stick 15 mm of orange tape on the skirt at the centre of the
  closed side; turn the guard to the start mark (tape 45 degrees right of straight ahead, open side toward the
  housing) and flip the lever shut; lay the tool upside down on its taped outline.
- **Forces:** the lever flips with 3 to 3.5 N at the cap and the open guard turns with about 6 N at the skirt;
  nothing the task needs comes near 40 N per finger. The shut band holds far more than a finger can turn, which is
  the point of the lock.
- **Done and safe:** the battery is out for the whole trial and the wheel is plastic, so nothing can start or cut. A
  trial is done when the grippers have opened and both arms are parked; then read q and whether the lever lies shut
  along the band.

## Real-world reset

Battery stays out. Flip the cam lever open, turn the guard back to the start mark, flip the lever shut, and set the
grinder on its taped outline. Every few trials check the lever force with the spring scale and that the thumb cap is
still bonded. Nothing is consumed and nothing gets dirty: the wheel never runs, so there is no dust or sparks; wipe
the guard if gripper pads leave marks.

## Licence

All geometry is procedural, generated by mechkit.py in this folder; no third-party assets.
