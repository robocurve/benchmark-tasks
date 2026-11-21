# Set up a folded tabletop easel with its rear leg out at the hinge stop, then rest a canvas on its ledge

Unfold the easel so its rear leg is out at its stop, stand it on its three feet facing the robot, and rest the canvas on its ledge leaning back against the easel, then let go of both.

## Scoring

Judged at the end from the top-camera video, a photo and a ruler. Before the trials, open the easel by hand to its stop and measure its full depth D once (front of the front feet to the back of the rear foot, along the table; about 214 mm). Score 0 if the easel is not standing on its three feet at the end (lying, on its face, a foot off the table or still held up). Otherwise the lowest of: rear leg 1.0 if the trial's depth is within 10 mm of D, 0.75 if 10 to 30 mm short, 0.5 if more; canvas 1.0 if it rests on the ledge (its bottom on the ledge or lip along its whole length) and leans back flat against the legs (its face within 10 degrees of the easel's), else 0.5; let go 1.0 if both grippers are at least 3 cm from the easel and the canvas and both have been still for 1 s, else 0.25.

How `score()` reads each condition (the easel's frame: origin on the front face at the foot end of the centre line,
x up the frame, z out of its face):

- **Standing:** the lowest corner of each front foot and of the rear foot within 4 mm of the table, the frame
  leaning back 10 to 35 degrees from upright, face up. Anything else (lying, on its face, lifted) scores 0.
- **Rear leg:** the ruler depth, front of the front feet to the back of the rear foot along the easel's facing, against
  the full depth at the stop (214 mm in the model): within 10 mm is 1.0 (the leg within about 2 degrees
  of its stop), 10 to 30 mm short 0.75, more 0.5.
- **Canvas seated:** both of the canvas's bottom back corners, in the easel's frame, between 29 and 44 mm up the
  frame (the ledge's top is at 32, the lip's at 37) and -3 to 21 mm out from the legs' front
  face, and its face within 10 degrees of the easel's (leaning back flat on the legs).
- **Let go:** both grippers' centre points at least 30 mm outside the easel's and the canvas's bounding boxes;
  **still:** over the last 1 s each moved under 1 mm and turned under 2 degrees.

`success()` is a score of 1.0. Why the shortcuts fail: a folded easel set on its feet has nothing behind it and falls
back flat (0); held up or propped, it is not standing (0); a canvas set on a leg that is short of its stop still
scores the leg's 0.75 or 0.5. A run that skips opening the leg (it tips the folded easel up onto its feet and lets go) scores 0: the easel falls back flat within half a second.

## Run and score

From this task folder, with the official YAM Arena environment installed:

```sh
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=set_up_folded_tabletop_easel_and_rest_canvas.py -E object_jitter=0 -E control_hz=20 \
  --instruction "Unfold the easel so its rear leg is out at its stop, stand it on its three feet facing the robot, and rest the canvas on its ledge leaning back against the easel, then let go of both." \
  --scorer task_score --max-steps 2400 --epochs 1 --store-frames --log-dir ./demo-run
```

For a model trial use `--policy dream`. Each trial allows 2,400 control steps (120 s at 20 Hz);
the scripted demonstration takes 42.4 s.

## The scene

Products, sizes and positions are in task.json's `setup` (mm from the midpoint between the arm bases; x forward, y to
the robot's left; heights above the table). Both arms start at their ready pose.

- **Easel (free two-body mechanism, 190 g), built lying face up.** Front frame (147 g): two front legs
  279 x 18 x 13 mm (convex pieces) whose centre lines run from 86 mm either side of the middle at the feet
  to 36 mm at the top; a top crossbar 231-249 mm up; the ledge 20-32 mm up, 19 mm deep in front of
  the legs and 190 mm long, with a 5 x 5 mm lip along its front edge. Rear leg (43 g, 18 x 13 mm,
  236 mm from the pin to its foot) on one hinge whose pin is on the crossbar's back face 240 mm up: free from folded
  (0) to a hard stop at 47 degrees open, damping 0.002 N m s/rad, static friction 0.004 N m (the leg drops
  open under its own weight). Open on the table the frame leans back 25.0 degrees: about 252.8 mm tall
  and 214 mm deep. Beech friction 0.5/0.4.
- **Canvas (one free body, 170 g):** 177.8 x 127 x 38.1 mm: a 2 mm face plate and four 19 mm
  stretcher bars behind it (hollow back). Friction 0.55/0.45.

## Scripted demonstration

The demonstration reads (never moves) where the easel and the canvas lie, the held easel's pose once after the lift
and once before it is set on its feet, and the standing easel's pose before the canvas goes on, and moves the hands
accordingly. Left hand (the easel): a
straight-down pinch across the left front leg 120 mm up from its foot; it tips the frame up about the folded
rear foot, lifts it 125 mm so the rear leg drops open onto its stop, sets the rear foot down on the easel's spot and
rolls the easel about it until its front feet are just over the table. A last read of the easel's pose and hinge
then moves it to standing with the leg on its stop round the rear foot where it actually is (the front feet drawn
out a few mm, as a person pulls an easel's feet apart), and the hand lets go and backs out along its approach.

| Time (s) | Left hand |
| --- | --- |
| 0.0 to 2.5 | over the left leg |
| 2.5 to 4.0 | fingers down round the leg |
| 4.0 to 5.0 | pinch the leg |
| 5.0 to 7.7 | tip the frame up |
| 7.7 to 9.1 | lift it clear: the leg drops open |
| 9.1 to 10.3 | hold while the leg settles on its stop |
| 10.3 to 12.3 | carry it over its spot |
| 12.3 to 13.5 | rear foot down |
| 13.5 to 18.0 | roll it onto its front feet |
| 18.0 to 19.2 | draw the front feet out: the leg on its stop |
| 19.2 to 20.0 | let go of the leg |
| 20.0 to 21.2 | fingers back off the leg |
| 21.2 to 22.7 | clear of the easel |
| 22.7 to 24.2 | out of the canvas's way |
| 24.2 to 42.4 | hold clear |

Right hand (the canvas): a straight-down pinch across the canvas's depth at the middle of its top edge; it carries the
canvas upright round the easel's right side to just in front of its legs, slides it across, sets it on the ledge's lip,
tips it back a little and lets go, and the canvas settles back against the legs; the hand then rises, backs off and
moves out to the right of the easel.

| Time (s) | Right hand |
| --- | --- |
| 0.0 to 2.5 | over the canvas |
| 2.5 to 24.2 | wait for the easel |
| 24.2 to 25.7 | fingers down over its top edge |
| 25.7 to 26.7 | pinch the canvas |
| 26.7 to 27.9 | lift it |
| 27.9 to 29.9 | beside the easel, in front of its legs |
| 29.9 to 31.9 | in front of the easel |
| 31.9 to 33.4 | down onto the ledge |
| 33.4 to 34.4 | tip it back a little |
| 34.4 to 35.2 | let go |
| 35.2 to 36.4 | fingers up off its top edge |
| 36.4 to 37.9 | clear of the canvas |
| 37.9 to 39.4 | out to the right of the easel |
| 39.4 to 42.4 | hold clear |

The recorded demonstration (no start jitter) scores 1.0, with success after 38.9 s (796 control steps): the rear leg ended 46.0 degrees open (ruler depth about 4 mm short of the full depth) and the canvas's face flat on the legs; both hands end more than 5 cm outside the easel's and the canvas's bounding boxes. Earlier runs of the same easel handling with 1 cm of start jitter (3 of 3) also ended with the easel standing 46.0 to 46.2 degrees open and the canvas seated. The Dream-1 run scored 0.0: the easel stayed lying folded.

## Simulation and the real objects

Rigid bodies. The easel is two bodies on one hinge: a hard stop at 47 degrees (estimate from the published 8.5 in depth) and 0.004 N m of hinge friction (estimate); the brass hinge's own shape is not modelled. Bar sections, the crossbar, ledge, lip and pin positions, the rear leg's length, both masses (147 + 43 g; published under 0.5 lb), the canvas's bars and mass (170 g) and all frictions are estimates. The canvas is a 2 mm face plate on a hollow frame.

- Easel: U.S. Art Supply USA E-003, 10.5 in Small Wooden Tabletop Easel, beechwood, $13.99
  (https://usartsupply.com/products/usa-e-003): open 7.5 in wide x 8.5 in deep, 10.5 in set-up height, 0.75 in canvas
  ledge, folds to 11 in, under 0.5 lb, holds canvas art up to 10 in high, "tilts with locking leg".
- Canvas: U.S. Art Supply 5 x 7 in Gallery Depth 1-1/2 in Profile Stretched Canvas, 5-pack USA GSC05X07G-5PK, $24.99
  (https://usartsupply.com/products/usa-gsc05x07g-5pk): cotton over kiln-dried pine stretcher bars.
- Estimates, each checkable in a minute with a ruler or a kitchen scale: the bar sections, where the crossbar, ledge,
  lip and hinge sit, the rear leg's length and the stop angle (from the published depth), the easel's and the
  canvas's masses, the canvas's bar width, the hinge's friction (hold the easel up by its frame: the leg drops open)
  and all frictions. The real kit (one easel, one canvas) costs under $40.
- The listing calls the rear leg "locking"; its photos show a plain brass hinge, so the lock is modelled as the
  hinge's open stop alone (check by hand: open the leg and see whether it catches or only stops).
- Credit: the run-context, read and waypoint helpers follow the task garnish-glass-rim-with-lime-wedge.

## Reset

About 1 minute. Fold the rear leg flat against the back of the frame and lay the easel face up on its rear leg at its start place, feet toward the robot (mark the table at its foot end and side with low-tack tape once). Stand the canvas on its long edge at its place, blank face toward the robot. Wipe nothing: there are no liquids. If the easel or the canvas fell off the table, check the hinge still opens to its stop and the canvas corners are square before the next trial. Return both arms to the ready pose.
