# Stand sample tubes in a water bath with every top above the water line

Lab technicians stand sample tubes in a water bath so the contents sit in the heat while the tops stay dry. Here a steel
bath stands in front of the arms, and four orange tubes wait on the bench: two tall (60 g) and two short (45 g),
weighted so they sink. The water line is a steel ledge 12 x 1.2 x 0.4 cm fixed to the inside of each side wall, top face
painted or taped blue, 1.2 cm wide so the top camera can see it, with its centre 6 cm above the table. Every tube has to
end upright in the bath with its top 1 to 4 cm above the line, which is 7 to 10 cm above the table.

No one spot suits every tube. A tall tube fits only on the bath floor (top at 9 cm); on the submerged rack it stands
too high (11.5 cm). A short tube fits only on the rack (top at 8.5 cm); on the floor it is flooded (6 cm). So the robot
has to match each tube's own height to a support, and one wrong tube fails the run. Bring a storage shelf under its
sprinkler clearance line sets one upper limit and Put groceries in the fridge assigns shelves by category; here a
two-sided band picks each tube's support, and every tube must go into a walled bath.

## Files

- `tubes_in_the_bath_caps_above_the_line.py`: the task file for the `yam_arena` embodiment (scene, `score()`,
  `success()`, `demo()`).
- `belimits.py`: shared geometry helpers the task file imports (outline overlap, tilt, height).
- `task.json`: the task's details.

## Run a trial

On an instance set up by the simulation walkthrough, from this folder:

```bash
INSTR="Stand all four sample tubes upright in the water bath. Each tube's top must end 1 to 4 cm above the blue water line: lower floods the sample, higher leaves it out of the heat."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tubes_in_the_bath_caps_above_the_line.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 1200 --store-frames --log-dir ~/yam-eval/tub-dream

# Scripted demo (full score): one arm at a time. The right arm parks beside the bench while the left sets its short
# tube on the rack and its tall tube on the floor and parks; then the right arm does the same on its side
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tubes_in_the_bath_caps_above_the_line.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 1200 --store-frames --log-dir ~/yam-eval/tub-demo
```

- **Seed and layout:** `-E object_jitter=0` is required: it keeps every tube exactly where the task file puts it, and
  the demo's grasps are for those positions.
- **Runtime per trial:** the demo's plan takes about 55 s of simulated time (1089 steps) and the run ends about 2 s
  later, when `success()` has held for 6 s; a Dream-1 trial runs up to 1,200 steps (1 minute simulated).
- **Score:** `task_score` reports 1.0 when `success()` holds for 6 s (all four tubes placed and still), otherwise
  `score()` at the end of the run. The 6 s hold keeps the run going after the last set-down, so the video shows the
  gripper open, rise and park clear of the bath.

Wall-clock time on one L40S, including Isaac Sim start-up: about 200 s for the demo and about 200 s for a full Dream-1 trial.

## Scoring

At the end of the run, open both grippers, then measure each tube's top to the nearest 0.2 cm with a ruler stood on the
bath rim (rim height 7 cm), so the band of 7 to 10 cm reads 0 to 3 cm above the rim. An upright tube's underside height
is its top minus its length (8 or 5 cm). A tube is **in the bath** if any part of it, seen from above, is inside the
walls (x 18 to 32 cm, y -9 to +9 cm).

1. **Score 0** if any tube in the bath has its top below 7 cm or above 10 cm, rests more than 1 cm above the floor or
   rack under its centre, or any tube has left the table.
2. **Otherwise 0.25 per tube** that is wholly inside the walls, still, standing on the bath floor or the rack
   (underside within 1 cm of it), tilted 10 degrees or less (high and low edges of its top within 0.5 cm on the
   ruler; 0.5 cm is 3 cm x sin 10 degrees), with its top from 7 to 10 cm. A tube outside the bath earns 0.

Possible scores: 0, 0.25, 0.5, 0.75, 1.0. Doing nothing scores 0.

Some outcomes that score 0 for the run: all four tubes on the floor (the short ones are flooded, top 6 cm); all four on
the rack (the tall ones stand too high, top 11.5 cm); a short tube dropped onto the floor even if the other three are
right; a tube lying flat in the bath, on the floor or on the rack; a tube stacked on another; a tube lying across a wall
with its underside at 7 cm and its top at 10 cm, even with the other three placed, because it rests more than 1 cm above
the floor under its centre.

Outcomes that only cost the tube itself: two tubes placed and two left on the bench scores 0.5; a tube knocked over
outside the bath earns 0 without failing the run; and a tube propped at a slant in the bath with its top in the band,
such as a tall tube fallen across the rack's edge, earns 0 without failing the run. The bath's 7 cm walls stop a tube
being pushed in along the table: it has to be lifted over them. The ledges leave y -7.8 to +7.8 cm open, so a standing
tube's centre stays within y -6.3 to +6.3 cm.

In simulation the checks use each tube's actual pose. The simulator cannot open the grippers before scoring, so a
tube still held more than 1 cm above the support under it counts as not placed, and over the bath it scores 0 for the
run; one held lower is judged by the top it would have once released. The rack's height only counts over the rack, so
a short tube hanging over the open floor at the right height does not pass. The arms' path is not scored; only where the
tubes end up.

## Real-world reset

Mark the four tube spots on the table before the first trial. Between trials, dry the tubes, stand them back at x 12 cm,
tall at y +15 and -15, short at y +6 and -6, and top the water up to the blue line with room-temperature tap water. The
bath stays clamped or screwed to the table and the rack stays screwed or epoxied to the bath floor. The tubes are capped
and sand-weighted so they sink on either support and when lying down: a tall tube's density is 1.06 and a short tube's
1.27.

With all four tubes in, the water rises about 0.5 cm, still below the 7 cm rim. The band is measured from the marked
blue line, not from the water surface. The demo's finger pads stop at 6.5 cm, about level with the risen water at the
last set-downs, so damp pads are acceptable.

Real trials: tap water only, room temperature; keep cables and electronics clear of the bath; the operator stops the run
if any part of the arm other than the finger pads reaches the water, and wipes spills before the next trial. A stopped
run is scored as the tubes stand at the stop, after opening both grippers. The simulation does not show or score this.
