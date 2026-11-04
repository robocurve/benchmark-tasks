# Stack a carton on another without covering its shipping label

A small blue carton stands on the table in front of a big one and to its right. The big carton's top carries a white
shipping label on its left part.
The robot must stack the small carton on the big one, fully on its top, without ever bringing any part of it over
the label lower than 3 cm above it. The obvious centred stack would cover the label, so a full score needs the
carton set down on the free part of the top (12 cm wide for a 6 cm carton) and lowered there, never over the label.
Covering the label at any moment scores 0, even if the carton ends up in the right place.
The 3 cm clearance is a safety margin: a carton carried low over a label can drag on it or be dropped onto it.

## Files

- `stack_without_covering_the_label.py`: the task file for the `yam_arena` embodiment (scene, `score()`,
  `success()`, `demo()`). Its footprint and height helpers are copied
  unchanged from a shared, tested helper kit, so the task is one file.
- `task.json`: the task's details.

## Run a trial

On an instance set up by the simulation walkthrough, from this folder:

```bash
INSTR="Stack the small carton on the big carton, fully on its top. The white patch is the shipping label: no part of the small carton may ever be over it lower than 3 cm above it, at any moment."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=stack_without_covering_the_label.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 1200 --store-frames --log-dir ~/yam-eval/swcl-dream

# Scripted demo (full score): the right arm lifts the small carton straight up, carries it in one straight line to
# above the free part of the big carton's top, lowers it straight down there, lets go and moves clear to its own
# side; the carton, the hand and the arm stay clear of the label's strip in the world at every height, not only in
# the top camera's view; the left arm is not needed and holds its ready pose
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=stack_without_covering_the_label.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 600 --store-frames --log-dir ~/yam-eval/swcl-demo
```

- **Seed and layout:** `-E object_jitter=0` is required: the free part of the top leaves 3 cm on each side of the
  small carton, and a jittered start would use up much of that margin before the robot moves.
- **Runtime per trial:** the demo's plan lasts about 16 s of simulated time (312 steps), then holds for 3 s with the arm
  clear of the carton; a Dream-1 trial runs up to 1,200 steps (1 minute simulated).
  Wall-clock time on one L40S, including Isaac Sim start-up: about 90 s for the demo and about 600 s for a full Dream-1 trial.
- **Score:** `task_score` reports 1.0 when `success()` holds for 10 s (the carton fully on the top, the label never
  covered, the carton still), otherwise `score()` at the end of the run. The 10 s leave time to let go and move clear:
  the scorer cannot tell a carton held just above the top from one that has been released.

## Scoring

1. **Score 0** if at any moment any part of the small carton is over the label (x 19 to 29 cm, y +2 to +10 cm)
   lower than 3 cm above the label's surface (11.2 cm above the table): set down on it, leaning on it, slid across
   it, or lowered past that line over it. Carrying the carton over the label higher than that is allowed. In a
   real trial this is judged from an overhead and a side video, with the 3 cm line marked on a card behind the
   carton. For real trials only (not in the simulation or setup.png): stand a 30 x 20 cm white card on the table
   against the big carton's left face, with a horizontal line 11.2 cm above the table, and place the side camera
   on a tripod off the table's right side at x 24, y -90 cm (outside both arms' reach), lens 11.2 cm above the
   table, looking along +y, so the line reads level with the 3 cm limit over the label. A pass within 0.5 cm of
   the line that the side video cannot resolve counts as clear; the simulation uses the exact line. Also score 0
   if the label is torn, creased or lifted, or the big carton has moved more than 1 cm from its taped outline
   (neither can happen in the simulation, where both are fixed).
2. **Otherwise, at the end** (both grippers opened):
   - **1.0** if the small carton lies flat on one face (tilted 10 degrees or less), its underside within 1 cm
     above the big carton's top and all four lower corners on that top (at most 2 mm past its edges);
   - **0.5** if its centre is over the big carton's top and its underside within 1 cm of that top's height, but it
     overhangs an edge or tilts more than 10 degrees;
   - **0** otherwise: left on the table, leaning against the big carton from the table, still held, or off the
     table.

Doing nothing scores 0, and so does the centred stack (at y 0 the carton reaches y +3 cm, 1 cm over the label).
In simulation the label test clips the carton's actual pose to the column above the label and takes the lowest
point inside it, so a tilted or turned carton is judged by its real shape. The simulator scores the last step
without opening the grippers, so a carton more than 1 cm above every support is treated as still held: not
placed, and a breach if it is over the label. The scorer remembers a breach until the next trial, which it
detects when the carton is back within 2 mm of its start after being more than 3 cm away on the previous step
(an environment reset); a robot that covered the label and then put the carton back exactly on its start mark in
one step would clear it, which a real trial's video would not.

## Real-world reset

Mark the small carton's starting outline on the table before the first trial. Between trials, stand it back on
its outline. The big carton (weighted with sand) stays taped to the table and the label stays glued on; replace the
label if it is torn or creased.

For real trials only (not in the simulation or setup.png): stand a 30 x 20 cm white card on the table against the
big carton's left face, with a horizontal line 11.2 cm above the table, and place the side camera on a tripod off
the table's right side at x 24, y -90 cm (outside both arms' reach), lens 11.2 cm above the table, looking along +y, so the line reads level with the 3 cm limit over the label.
