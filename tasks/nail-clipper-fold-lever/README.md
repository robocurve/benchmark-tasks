# Fold a lever nail clipper shut for storage

A steel lever nail clipper, 85 mm long and 54 g, lies loose in a tray slot with its lever raised for cutting: pointing
to the tail, 19 degrees up, its heel down on the upper blade. The robot folds it shut for the drawer. The lever has to
be swung half a turn about its pin and turned over, so that it ends flat along the upper blade, pointing to the tail,
heel side up. The clipper stays in the slot and both grippers end clear of it.

## The tool

Large lever clippers on sale are 82 to 92 mm long, 12.7 to 19 mm wide and 45 to 60 g (Trim 3.25 x 0.5 in; Seki Edge
SS-107, 92 mm, 2.1 oz; Tweezerman, 45 g). Ours: 85 x 15 mm, 54 g, steel. The lower blade (85 x 15 x 2 mm) lies on the
slot floor. The upper blade (2 mm thick) is riveted to it at the tail, rises 5.5 degrees from the tail and runs level
over its last 22 mm, round the pin, to the jaws, where the cutting edges stand 2.4 mm apart. A 3.5 mm pin 14 mm behind
them passes through a hole in the upper blade and swivels freely in the lower one. The lever (57 mm hinge to tip, 12 mm
wide, 2.2 mm thick, 10 g) hinges on the pin, 12.5 mm above the floor, with a heel 6 mm from the hinge that stands 3.5 mm
proud and bears on the level part of the upper blade: the lever starts raised about 19 degrees, the cutting position.
On a real clipper, pressing the tip turns the lever on its heel and closes the jaws (lever ratio about 9.5); here the
two blades are one rigid part, so a pressed lever only bears on its heel (see Simplifications).

## Why it is hard

1. **Pressing it does not fold it.** Pushed down, the lever only bears on its heel. Pressing the lever down and
   holding it there scores 0.
2. **Two axes at once.** The pin swivels in the blades and the lever hinges on the pin, so the lever has to be swung
   half a turn about the pin (it then points out past the jaws, about 19 degrees up, its tip 31 mm high) and turned
   over its hinge, over the top, down onto the blade.
3. **Small and loose.** A 54 g part with 1.5 mm of play each way in the slot, pushed and flipped by a closed gripper
   (tip 16 x 27 mm) that must not drag it out of the slot, and must end 30 mm clear.

Two routes get there: swing the raised lever half a turn and flip it over the top, or flip it over first (it then lies
pointing out past the jaws, heel side up) and swing it half a turn. Either way round the pin works. Nothing latches:
the lever stays where gravity and the blade leave it.

## Scoring

Read at the end of the run; nothing is latched. Plan directions are taken along the lower blade, from the pin, in the
clipper's own frame (so turning the whole clipper over proves nothing). "Folded" is: the lever points to the tail within
10 degrees in plan (at least 20 mm long in plan), its heel side faces away from the lower blade, the underside of its tip
is no more than 2 mm above the upper blade, all four bottom corners of the lower blade are within 1.5 mm of the slot
floor, each gripper's fingers are 30 mm or more from every part of the clipper, all of it on each of the last 20 steps,
and neither the lever's tip nor the two ends of the lower blade moved more than 1 mm over those steps.

| Outcome | Score |
|---|---|
| Folded | 1.0 |
| The lever to the tail within 10 degrees with its heel side away from the lower blade (over the top, but not flat, not clear or not still) | 0.5 |
| The lever out past the jaws, within 30 degrees of the blade's line, either way up (half of the fold, by either route) | 0.25 |
| Anything else, including the start (to the tail, heel toward the blade) and a lever pressed down | 0 |

## Results in simulation

GPU run 2026-11-14: L40S, Isaac Lab-Arena, `-E object_jitter=0 -E control_hz=20`, one trial each,
run limit 800 steps (40 s). An earlier build (two free assemblies joined by a fixed joint) scored 0 because that joint
did not hold; this build is one articulation.

| Run | What it does | Score |
|---|---|---|
| Scripted demo | the right arm: swing, flip, retreat | 1.0 (success held from step 490) |
| Right arm only (left idle) | the same demo; the task is one-handed | 1.0 |
| Left arm only (right idle) | nothing reaches the clipper | 0.0 |
| Dream-1 | the task's instruction, full run limit | 0.0 |
| Swing only (`DEMO_VARIANT=swing_only`) | the lever ends out past the jaws, not folded | not run on the GPU |
| Press down (`DEMO_VARIANT=press_flat`) | straight down on the lever, held to the end | not run on the GPU |

The task needs one hand; it does not claim two.

## Demo

Open loop: timed waypoints from the known start layout, the right arm only, the gripper closed (tip 16 x 27 mm) for the
whole run; the tray is in the right arm's column, slot along the table's x axis, jaws toward the robot.

| Phase | What the right gripper does |
|---|---|
| Swing | straight down, pads closing along x, tips 17 mm over the slot floor. From beside the lever's long edge, 45 mm from the pin, round the jaw end on that circle at about 25 mm/s, pushing the lever's edge half a turn; the last move presses the 16 mm side face of the pads to the lever's edge (it squares the lever to the blade), then backs off 5 mm. |
| Flip, phase 1 | pitched 35 degrees about the clipper's y: the outer face of one pad faces the tail and upward, the fingertips point to the tail and down. The lever's tip rides that face, 15 mm above the fingertip edge, up to 55 degrees. |
| Flip, phase 2 | the fingertip edge rides the lever's heel-side face 42 mm from the hinge, over the top and down toward the tail, drawing in to 33 mm over the last 30 degrees, and ends 0.5 mm above the stowed lever. Past 145 degrees of lever the pitch eases off to 6 degrees the other way, so that the tip face lies parallel to the lever and the pad's other tip corner never dips into it. |
| Retreat | straight up 60 mm, then away; hold 1.5 s. |

A straight-down face would jam in the flip: on a vertical face the tip only rises if the tangent of the rise exceeds
the friction (31 degrees against about 0.7 on a finger), hence the pitch.

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=task.py -E object_jitter=0 \
  -E control_hz=20 --instruction "<the instruction>" --scorer task_score --max-steps 800
```

For Dream-1 use `--policy dream`. `DEMO_IDLE_ARM` (left or right) drops one arm's plan, `DEMO_VARIANT` runs a
counterfactual, `TASK_TRACE=trace.jsonl` writes one line per control step: the scorer's readings and the key poses
(the clipper's base, both joints, the lever's link, its tip and its hinge point).

## Simplifications

- Size and mass are those of large lever clippers on sale (82 to 92 mm, 45 to 60 g). The lever ratio of 9.5 and the
  heel are ours, in the range of such clippers (ratio 8 to 13, effort arm 40 to 60 mm, resistance arm 3 to 7 mm:
  littlemechanic.com, its own estimates).
- Blades and lever are flat plates and the cutting edges square lips, not ground. The upper blade is a bent plate,
  sloping from the tail and level over its last 22 mm; a real blade is curved.
- The two blades are one rigid part: the jaws do not close. A real pair close 2.4 mm under about 4 N at the lever's tip
  and spring back; pressing the lever is a wrong way in either case, since it leaves the lever unfolded.
- The clipper is one articulation (the blades, the pin that swivels in them, the lever that hinges on the pin). The
  simulator does not let parts of one articulation touch each other, so the lever's two rests on the blade are stops in
  its hinge, set where the real parts meet: the heel on the level seat (19 degrees up, the same at every swing angle,
  since the heel stands within 7 mm of the pin) and, turned over, the tip on the sloping blade (6 degrees below level;
  pointing out past the jaws the lever meets the jaw end within 0.1 mm of that). Held out sideways, turned over, a real
  lever would sag further than the stop lets ours.
- The pin turns in place and does not rattle or slide. Nothing is latched or moved by code.
- Steel on steel friction 0.25 (static), 0.2 (dynamic); the fingers' pads are 1.2, so a finger sees about 0.7.

Third-party assets: none. Sources: the product pages named above and "compound lever" (Wikipedia). The swing and flip
is described as the common way such clippers fold; no source is cited for it and no maker's force figure is used.

## Real build and reset

One lever nail clipper as sold, a shallow tray with an 88 x 18 mm slot, a top and a side camera. Nothing is used up,
powered or hot; the clipper does not cut.

Reset: put the clipper in the slot, jaws toward the robot. Turn the lever to the cutting position: pointing to the tail,
heel down on the upper blade, raised about 20 degrees. Check that the jaws stand open and that the lever swivels
freely. Open both grippers and park them 150 mm or more from the clipper.

## Files

`task.json`, `task.py` (the two objects, `success()`, `score()`, `demo()`), `layout.py` (dimensions, thresholds,
shapes, the demo's paths), `scoring.py`, `offline_clipper.py` (kinematics of the clipper where there is no
simulator), `mechkit.py`, `rigstate.py`, `tiltgrip.py` (the mechanism kit, the arms' state and the pitched grip,
copied as they are), and `tests/` (NumPy only): `python3 -m unittest discover -s tests -v`.
