# Cope a scotia moulding to fit an inside corner

Two-handed trim carpentry at true size in a room corner. A loose length of 18 x 18 mm pine scotia has its end cut at
45 degrees, which shows the moulding's profile as a curved pencil line on the cut face. One hand must pinch the scotia
upright on an offcut block (it stays up only while leaning on a skirting offcut; without that it would tip back
43 degrees onto its relief chamfer); the other hand saws along the curved line with a coping saw so that the end takes
the shape of the installed scotia it will butt against in the corner. Then the saw goes back on the floor and both
hands let go. After the trial the evaluator offers the coped end up to the installed scotia and reads the widest gap
with feeler gauges.

## Run

    inspect-robots run --embodiment yam_arena --policy dream \
      -E task=tasks/fit-the-inside-scotia-joint/task.py -E object_jitter=0 -E control_hz=20 \
      --instruction "Cope the pencilled end of the loose scotia with the coping saw so it fits the installed scotia in the corner, then put the saw down and let go of the scotia." \
      --scorer task_score --max-steps 2400 --epochs 2

`--epochs 2` runs lot A then lot B (trial k uses lot k mod 2); `TASK_LOT=A` or `TASK_LOT=B` pins one lot. Run with
`object_jitter=0`. The scripted demo is `--policy yam_scripted` with the same arguments (a 64 s plan per trial). A
trial takes about 3 to 4 minutes on an L40S. Simulator: yam-isaac-lab-arena (the program's YAM embodiment for Isaac
Lab-Arena). The demo uses `tilt.py` (leaned grips); `parts.py` builds the parts; `lots.py` sets the two layouts.

## Why it matters

Coping is how carpenters join skirting trim and scotia at inside corners: a mitred joint opens when the corner is not
square or the wood shrinks, while a coped end keeps a closed line. The cut is made by eye along a curve with a narrow
blade, holding the moulding steady with the other hand. Carpenters (O*NET 47-2031.00, about 890,000 US jobs, BLS
2025) list as a core task "Shape or cut materials to specified measurements, using hand tools, machines, or power
saws", and "Install structures or fixtures, such as windows, frames, floorings, trim, or hardware".

## Scene (lot A; world x away from the arms, y to their left, heights above the floor)

The station's timber top is the room's floor; the two arms stand on it facing a room corner.

| Item | What it is | Where |
| --- | --- | --- |
| Far wall | painted plasterboard wall, fixed | face at x = 53.0 cm |
| Side wall | painted plasterboard wall on the robot's left, fixed | face at y = +46.0 cm |
| Skirting | 68 x 18 mm primed pine skirting on both walls, fixed; the far one runs into the corner, the side one butts its face | faces at x = 51.2 cm and y = +44.2 cm |
| Installed scotia | 18 x 18 mm pine scotia (Wickes 121192, sold in 2.4 m lengths), pinned to the far skirting with brad nails for 600 mm, its square end tight against the side skirting | back on the far skirting, from y = +44.2 to -15.8 cm |
| Offcut block | 200 mm length of 90 x 45 mm MGP10 pine, 0.40 kg, lying on its wide face | centre (18.5, +2.4) cm |
| Skirting offcut | 50 mm length of the same skirting, 31 g, lying flat on the block behind the scotia | centre (25.0, +5.3) cm, top 63 mm up |
| Loose scotia | the same scotia, 200 mm long at its back, 16 g (pine 510 kg/m3, estimate), standing upright on the block with its back face toward the side wall, leaning back on the skirting offcut | back face at y = +1.9 cm; its 45-degree end (long point at the back) at x = 32.0 cm, 35 mm past the block's end; bottom 45 mm up |
| Pencil line | the curved line where the 45-degree cut meets the scotia's front face, drawn dark | on the cut face |
| Coping saw | Stanley FatMax 0-15-106: 165 mm 14 tpi blade, 170 mm throat, about 0.35 kg (retailer listing 0.77 lb); handle modelled as an 8-sided prism 30 mm across flats, 100 mm long | flat on the floor, handle centre (20.0, -28.0) cm, handle toward the robot's right, blade running toward its middle, frame toward the far wall |

Lot B: the block, the skirting offcut and the loose scotia 12 mm nearer the robot and 20 mm to its right; the saw 15 mm
farther from the robot and 12 mm to its right. The screenshot shows lot A.

The scotia profile is traced from a retailer's published end drawing of 18 x 18 mm natural pine scotia (B&Q
3663602872818); 18 x 18 mm pine scotia is a commodity section and the Wickes length is taken to match it. The profile:
two back faces at right angles with a relief chamfer between them (8.5 mm along the floor face, 8.0 mm up the back
face), a 4.9 mm toe face at the front, a 5.4 mm flat on top, and between them a cove of radius 11.76 mm. One-minute
check: lay the end of a stick of the scotia on paper and trace it. Its centre of mass lies 7.3 mm from its back,
behind its 8.5-18 mm floor face, which is why it tips back unless something holds it or it leans on the offcut.

## Scoring (read in a real trial)

Score 0 for the whole trial if the video, stepped frame by frame, shows any of these:
- the blade moving into the wood while the saw is not being stroked (it must slide at least 7 mm along its blade each
  half second), or moving through the wood faster than 30 mm/s;
- the saw carried through the wood with its teeth facing away from the way the cut goes (more than 75 degrees off),
  or twisted more than 30 degrees across the cut;
- wood sawn while no hand pinches the scotia (both fingertips of one gripper on it, across its depth);
- the blade touching the installed scotia, a skirting board, a wall or the floor (marks on them), or cutting the
  loose scotia more than 24 mm back from its 45-degree end.

In a real trial the wood itself resists most of these: a coping saw pushed without stroking, or carried against its
teeth, binds rather than cuts. The rules say what the simulator, which has no saw forces, treats the same way.

After the trial the evaluator, not the robot, offers the coped end up by hand, as a carpenter checks a cope: slides
the scotia along the side skirting into the installed scotia with about 10 N until it stops, and reads the widest
gap across the joint along its visible front edge with 1, 2, 4 and 6 mm feeler gauges.

If the offcut has come away (under 40% of the wood behind the pencil line still attached):
- 1.0: at most 1 mm, with the saw put down on the floor (its lowest point within 6 mm of the floor) and not held,
  the scotia let go, and both grippers at least 3 cm from the scotia and the saw for 1 s with both still (under
  5 mm/s);
- 0.75: at most 2 mm;
- 0.5: at most 4 mm;
- 0.25: at most 6 mm.

Otherwise (a gap over 6 mm, or the offcut still on): 0.25 if at least half the pencil line (by its height up the
scotia) has been sawn within 1 mm of it, read with a ruler and a photo of the cut face; else 0.

Trials are run on lots A and B and averaged. Safety for real trials: the holding gripper pinches the scotia about
160 mm from the cut, but fit its fingers with cut-resistant pads anyway and keep people outside the arms' reach.

### How the simulator scores it

The cut is a declared state rule (task.py, `cut_step`): the wood at the 45-degree end is a grid in the scotia's own
frame (0.25 mm along it and up it, 0.5 mm across its depth, the first 24 mm from the end). Every control step the
blade's line (between its two pins) is intersected with each depth slice, and the grid cells within 0.4 mm of the
blade's path since the last step (a 0.8 mm kerf) are removed when all of these hold: the blade has slid at least
7 mm along its length in the last 0.5 s (sawing); one gripper pinches the scotia (both its fingertips within 1 mm of
the scotia's wood, at least 3 mm apart across its depth: `pinched`); the blade lies within 30 degrees of the
scotia's depth; the cut has been advancing within 75 degrees of where the teeth face (judged from the blade's mean
position over the last 2 s against the 2 s before, so the wobble of the strokes cancels); and the new wood swept in
that step is at most 1.5 mm deep. A sawing blade that rubs sideways in its kerf cuts nothing. The trial scores 0
(`_set_zero` names the fault) if the blade sweeps more than 0.3 mm of wood while not sawing, not held or twisted, more
than 1.5 mm of wood in one step, or more than 3 mm of wood in 4 s that it could not cut because it was going against
its teeth (a saw carried through wood the wrong way round). The offcut is the wood no longer joined to the rest of
the scotia (face-connected cells, `main_wood`); it has come away, and is hidden from view, once under 40% of it is
still joined (`waste_off`).

The fit (`fit`) slides the remaining wood along the scotia's length into the installed scotia's outline until it
stops; wood that would sink in across the joint by less than 0.4 mm does not stop it (pine that proud crushes when the
piece is pushed home, an estimate). It then takes the widest gap across the joint (shortest distance to the installed
scotia's outline) along the coped end's front edge. The partial tier (`line_sawn`) counts the rows of height up the
pencil line where the wood within 1 mm of the line has been cut in at least half the scotia's depth there. The 1 s
clearance uses points sampled about 4 mm apart over every face of both grippers and fingers (`_measure_hands`) and
counts 20 steps in a row with both grippers 3 cm clear and the scotia and the saw under 5 mm/s; the saw is on the
floor when its lowest point is within 6 mm of it. If the gripper state cannot be read, nothing counts as held (no wood
is removed) and there is no 1.0.

## Unintended routes (each scores 0)

| Shortcut | Why it scores 0 | Case in near_misses.py |
| --- | --- | --- |
| Leave the end uncut | the offcut never comes away, nothing sawn | "not cut at all" |
| A square cut across the end | offcut off but gap 12.6 mm, and under half the line sawn within 1 mm | "a plain square cut ..." |
| Saw a third of the line | offcut still on, under half the line sawn (over half earns 0.25) | "a third of the line sawn ...", "over half the line sawn ..." |
| Push the blade along the line without sawing | blade forced into the wood | "the line sawn without stroking ..." |
| Drop the blade straight through | fed faster than it cuts | "the blade dropped through ..." |
| Saw with the teeth facing away from the cut | the blade is carried through wood it is not cutting | "sawn with the teeth facing up" |
| Saw with one hand, the scotia not held | sawn while nothing holds it | "sawn while nothing holds the scotia" |
| Park the other gripper against the scotia without pinching it | not a pinch, so sawn while nothing holds it | "... both fingertips on that face ...", "... 3 mm short of its face ..." |
| Run the blade into the installed scotia, skirting or floor | whole-trial zero | three "coped, then the blade ..." cases |

One-hand check: the scotia cannot be sawn with one hand. It stands only by leaning on the skirting offcut, and the cut
rule removes no wood unless one gripper pinches it across its depth; a gripper holding the saw cannot also pinch the
scotia, and a gripper merely resting against it does not count.

## Fidelity

True size throughout. The wood is not cut by contact: the blade has no collider and the declared rule above removes
wood. The saw exerts no force on the scotia (no drag along the stroke, no push), so the hold is enforced by the rule
(a pinch of both fingertips) rather than by the scotia being pulled about. Tooth direction is reduced to a 75-degree
cone. The 7 mm minimum stroke is a simplification: a real coping saw stroke is around 50 mm or more, and the demo
strokes 6 mm each way. The feed limit and the 0.4 mm crush allowance are estimates, not measurements. The loose scotia
is drawn and collided as its ideally coped shape plus a visible offcut that disappears when sawn free, and the kerf is
not drawn, so the video shows the ideal cope whatever was cut; the score uses the wood actually left. The saw's mass
is spread over its colliders, so its centre of mass sits nearer the handle than a real steel frame's. Two layouts, so
one memorised motion can't serve both. No third-party assets: every part is built from boxes, cylinders and convex
pieces in task.py.

## How this differs from close tasks

- TK-8E7CVB (accepted): saw a 45-degree mitre in a mitre box. The box guides a straight cut; here the cut is a free
  curve steered by eye with a narrow blade, and the score is the fit of the curve to a mating part.
- TK-N9C8DA (accepted): hold a loaf with one hand and saw a slice off with the other. The same hold-and-saw pattern;
  here the cut is a curve that must match another part's profile to within a millimetre.
- TK-NY6WTD (accepted): file a steel block down to a straight scribed line. A straight line with a file; here a curved
  line with a coping saw, two-handed, in wood that must be held.
- TK-ZFGXMT (accepted): scribe a shelf to a wall. Marks the line to cut; here the marked line is sawn and fitted.
- TK-DV4MVS (pending): saw an installed drain pipe square. A straight cut through a fixed part.
- TK-278Z5R: transfer a casing profile with a contour gauge. Measures a profile; nothing is cut or fitted.

## The scripted demo

Open loop from the published layout (`demo()` reads only the lot's declared positions); a 64 s plan. The left hand
pinches the upright scotia across its depth 185 mm from the 45-degree end, high on it (fingertips 11 mm up, where the
back face and the cove face each other at the same height, so the squeeze does not roll it) and holds it on the
block. The right hand takes the saw by its handle (30 mm in from the handle's middle), raises the frame about
40 degrees so the teeth face forward and down, brings the blade straight down over the line, strokes 6 mm each way
every 0.6 s while it feeds 0.6 mm along the line per stroke (round the cove, under its foot, down the toe), drops out
below the end, and puts the saw back where it lay. The left hand lets go and both arms park. The demo's grip targets
carry corrections measured on the GPU for the saw's weight (it droops 3.4 degrees in the pads and turns 1.4 degrees;
the hand sags about 14 mm back and 22 mm down). They aim the blade 0.5 mm, and on parts of the line up to 1.2 mm, to
the waste side of the line to allow for the blade's swing during each stroke, and run the level pass under the cove's
foot 0.5 mm lower (`LEVEL_DROP`; the blade rose up to 1.6 mm there on the GPU). These are demo settings only; the
scoring does not use them.

## Reset

1. Remove the coped scotia and its offcut. 2. Cut a fresh 200 mm length of the scotia (square at one end) and cut its
other end at 45 degrees as an inside mitre (long point at the back) in a mitre box; darken the profile line on the
cut face with a pencil. 3. Lay the 90 x 45 mm offcut flat at the lot's position, the 50 mm skirting offcut flat on it,
and stand the scotia upright on the block, back toward the side wall, leaning on the skirting offcut, its 45-degree
end 35 mm past the block's end. 4. Lay the coping saw flat on the floor at the lot's position, handle toward the
robot's right. Check the installed scotia for saw marks and replace it if marked. Lot A and lot B differ only by the
shifts given above.

## Scores

| Run | Lot A | Lot B | Mean |
| --- | --- | --- | --- |
| Scripted demo (`yam_scripted`, 1212 steps each) | 1.0 (widest gap 0.71 mm) | 1.0 (widest gap 0.70 mm) | 1.0 |
| Dream-1 (2400-step limit) | 0.0 | 0.0 | 0.0 |

In the Dream-1 run the right gripper went to the saw's handle and the left arm moved over the block, but no wood was
cut in either lot, so the offcut never came away.

## Files

`task.py` (scene, cut rule, scoring, demo), `near_misses.py` (offline cases), `parts.py`, `lots.py`, `tilt.py`.
