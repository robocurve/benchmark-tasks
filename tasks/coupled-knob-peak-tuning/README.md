# Tune two coupled adjusters to the peak of a meter in a few readings

A bimanual calibration task for the Robocurve benchmark. Two sliders (an orange one on the left track, a teal one on the right)
change one hidden reading. The reading is shown only when the white READ button is pressed, and only 8 presses are allowed. The
best setting lies on a narrow diagonal ridge, so adjusting one slider at a time wanders; the robot has to take a few readings at
clearly different settings, infer where the peak is, push both sliders there, and let go.

Status: all numbers below marked "offline" were measured on this build without the GPU (unit tests, IK, hull clearances, an
idealised replay of every plan through the real scorer). **Nothing has been run in the simulator yet.** What only the GPU can
tell is listed in `work/tasks/GATE_G.md` and `VM_NOTES.md`.

Instruction (identical in `task.py`, `run.sh`, `task.json`):

> Press the white READ button between the two tracks to show the red needle for the current positions of the orange (left) and teal (right) sliders; use at most eight READ presses, including at least three at slider positions 10 mm or more apart. Then set both sliders so the needle stands on the green band at the end of the scale, and let go.

## What it tests

1. **Inference under a measurement budget.** The reading is r(x, y) = exp(-(u^2 / (2 * 45^2) + v^2 / (2 * 9^2))) with
   u = ((x - x0) + (y - y0)) / sqrt(2) (along the ridge) and v = (-(x - x0) + (y - y0)) / sqrt(2) (across it); x is the left
   slider's setting and y the right's, in mm. The peak (x0, y0) is hidden. The best single-axis search (exact line searches,
   which no real policy achieves) is still at r = 0.930 after 8 readings (variant 0).
2. **Contact manipulation with two arms.** Each slider is an 80 g block in a U-channel held by friction (mu 0.5, 3 mm play per
   side); the closed fingers of the arm push it from behind (or, for a correction, from in front). The READ cap is a spring
   (100 N/m) pressed to a hard stop. The scripted demo moves one arm at a time (the other waits at home), so nothing in the
   scoring forces two arms; two arms are needed only for speed and for reaching the far track.
3. **Not memorisable (partly).** The peak has three documented variants: 0 = (80, 50) mm (default, the one in the video),
   1 = (88, 72), 2 = (95, 55), selected by `TASK_VARIANT` / `VARIANT=1 ./run.sh ...`. A policy that hard-codes variant 0 scores
   1.0 on variant 0 and 0.25 / 0.50 on variants 1 / 2 (offline prediction). The scored rungs need 2 (0.75) or 3 (1.0)
   **separated** counted presses, so a run with no or one reading is capped. A memorising policy that takes three readings and
   then goes to the remembered peak still scores 1.0 on variant 0: with a single fixed layout per trial that cannot be prevented,
   and the video shows variant 0 only. Trials on variants 1 and 2 are what tests inference.

## Scene (all lengths mm; geometry.py is the single source)

| Part | Size / properties |
|---|---|
| Base plate | 300 x 200 x 10, dark grey, 5 kg, fixed; centre x = 300 from the arm-base midpoint |
| Tracks (2) | channel 36 wide, 140 long, walls 6 thick x 20 tall, 5 mm end stops (130 inner), 2 mm floor; y = +60 (left) and -60 (right) |
| Position bands | 2 mm floor tiles, 20 mm long, alternate blue / light grey from the near stop (0-20 blue, 20-40 light, ... 80-100 blue), then grey |
| Knobs (2) | 30 x 30 x 14, 80 g, friction 0.5, orange (left track) and teal (right track); start with the rear face 30 mm from the near stop |
| READ button | pocket ring 48 x 48 x 16 (42 x 42 inside) at x = 335; white cap 40 x 40 x 12, 20 g, 100 N/m, 4 N s/m, 8 mm travel to a hard stop (cap top 34 mm above the table at rest) |
| Meter housing | 40 x 200 x 30 at x = 400 |
| Scale bar | white, 160 x 24 x 4 on the housing, 0 mark at y = -80; green band 8 long (152 to 160 mm) x 36 wide |
| Needle | bright red 34 x 12 x 8, moved by code to 160 * r mm from the 0 mark on every READ press |

Setting of a knob = distance from the near stop's inner face to the knob's rear (arm-side) face. The needle is at the green
band (centre on its left edge) exactly when r >= 0.95.

## Scoring ladder (scoring.py; the text in task.json matches it)

Final settings decide r. 0.00 nothing moved and no press; 0.25 a knob moved >= 3 mm or READ pressed; 0.50 r >= 0.50;
0.75 r >= 0.80, <= 8 presses in total and >= 2 separated counted presses; 1.00 r >= 0.95, <= 8 presses, >= 3 separated
counted presses, both grippers open, both knobs <= 5 mm/s, for 20 consecutive control steps (1.0 s; the scorer runs once per
control step, checked by `tests/test_modes.py`). Separated = at least 10 mm from every earlier separated press in max(|dx|,
|dy|). Presses beyond 8 move the needle but do not count and disable the top two rungs. Zero for the whole run (latched) if a
knob's centre leaves its track line by more than 10 mm or rises above 45 mm over the table, or the base plate moves more than 5 mm.

Tolerance for r >= 0.95: one knob alone within 4.0 mm; opposite errors (x + e, y - e) within 2.1 mm; along the ridge up to
14 mm. The demo's own placement error is at most about +/-1.5 mm per knob (below).

## Modes, step counts and predicted scores (offline)

`python predict_scores.py` replays each plan through the real `Scorer` at 20 Hz with idealised physics (knobs jump to the plan's
setting, READ fires while the pad is down, +/-1.45 mm placement offsets tried at every corner). Measured output:

| run.sh mode | variant | plan steps | harness run end | STEPS | predicted score | final settings, r |
|---|---|---|---|---|---|---|
| demo | 0 | 681 | 672 | 1000 | 1.00 (all offsets) | (80, 50), 1.000 |
| demo | 1 / 2 | 696 / 691 | 687 / 682 | 1000 | 1.00 / 1.00 | peak |
| recover | 0 | 795 | 786 | 1100 | 1.00 | (80, 50) |
| axis_search | 0 / 1 / 2 | 881 / 883 / 888 | 881 / 883 / 888 | 1250 | 0.75 / 0.50 / 0.75 | r = 0.930 / 0.725 / 0.892 |
| no_read | 0 | 197 | 197 | 400 | 0.50 | (80, 50), no presses |
| memorize | 0 / 1 / 2 | 465 | 456 / 465 / 465 | 650 | 1.00 / 0.25 / 0.50 | (80, 50) on every variant |
| settle | - | 160 | 160 | 400 | 0.00 | - |
| dream | - | - | - | 2400 | unknown (GPU) | - |

The demo reads 5 times (variant 0: needle shows r = 0.034, 0.721, 0.358, 0.901, 1.000 at (30,30), (55,30), (55,42), (72.5,48),
(80,50)) and uses 6 pushes. The run limit in `task.json` is 2,400 steps; the scripted demo ends after 710 steps in the simulator (30 percent of it).

## Physics models that the design rests on (offline, with sources)

* **READ cap vs arm compliance.** At the press pose the arm's vertical compliance is 3.13 mm/N (319.6 N/m), from the joint
  gains in `yam_arena/constants.py` (shoulder 80, wrist 10 N m/rad) through the numerical Jacobian. The first design (625 N/m
  spring, 7 mm command) gave 2.4 mm of travel against the 6.4 mm trigger: READ could never have registered. Now the spring is 100
  N/m and the pad is commanded 13 mm below the resting cap top, 5 mm beyond the end stop: the cap reaches the 8 mm stop (the
  stop, not the arm, sets the travel) with 0.8 N spring force and 1.6 N total arm force; travel stays at the 8 mm stop (margin >= 1 mm) for any arm
  stiffness above 132 N/m, 41 percent of the computed value (`tests/test_physics_models.py`).
* **Wedge fingers.** The closed fingers are not a box: half-depth along the push direction is 6.5 mm at the pad bottom and
  grows 0.4 mm per mm of height (measured from the real collision hulls, table in `geometry.py`). A pushed block is therefore
  touched at its top edge (11.25 mm half-depth at 14 mm knob height with the pad 2.5 mm above the floor). Knobs were lowered
  from 25 to 14 mm so that contact height is below the 15 mm tipping height even for the worst friction combine. The descent
  clearance to the knob is 3 mm at that top edge, and the finger clears the near end stop by 2.7 mm at wall-top height (model; the hull check below measures it).
* **Push force and accuracy.** A quasi-static model (`geometry.push_force_model`) gives 0.49 N horizontal, 0.86 mm finger
  lift (below 1 mm) and a 2.9 mm stall lag against the arm's 5.9 mm/N x-compliance, so the plan commands 1.45 mm beyond the
  nominal contact and the knob lands within about +/-1.45 mm of the plan.
* **Backward pushes** (finger in front of the knob) work only from settings up to 70 mm because the wedge needs room before
  the far end stop; overshoot beyond 70 mm cannot be corrected by pushing. Position bands are 5.4 mm/px at best on the
  calibrated top camera (a camera policy cannot place a knob better than about 1 px), and the green band is about 1.5 px long.

## Offline checks (measured, see GATE_G.md for the full output)

* `python -m unittest discover -s tests`: unit tests for geometry, logic, scoring, modes, task glue, predictions, physics models.
* `check_reach.py` (all 10 plans): every sample reachable, IK error <= 0.10 mm / 0.01 deg, minimum joint margin 0.246 rad.
* `check_collisions.py`: every sample of every plan against every named object, per class and per arm; exit 1 on a clearance
  below 1 mm, an intended contact deeper than its stroke, or an arm-arm distance below 5 mm. Results: see GATE_G.md.

## Known weaknesses (honest)

Memorisation of one variant works; the inference is a 2-D log-quadratic fit; the physics (friction, wedge contact, arm stall)
and the camera legibility of the bands and the needle are unverified in the simulator; backward correction is limited to
s <= 70 mm; real policies see about 5 mm per pixel, so reading the knob positions from the camera is coarse.

## Files

`task.json` (spec), `task.py` (task file), `geometry.py`, `logic.py` (reading, variants, press logic), `scoring.py`,
`demo_plan.py` (modes), `predict_scores.py`, `parts.py` (compound-part helper, `write_pose` with warnings and read-back),
`check_reach.py`, `check_collisions.py`, `run.sh`, `VM_NOTES.md`, `tests/`.

## Credits

`parts.py` follows the compound-part and runtime-teleport technique of the benchmark repository's `tasks/bearing-press/partkit.py`.

## Measured in the simulator

Three runs of `./run.sh demo` (fresh resets, same code) each ended at step 710 with score 1.0: the knobs finished at 82.2 mm
(left) and 53.1 mm (right) from their near stops, a reading of 0.994, with five counted READ presses. One Dream-1 run
(`./run.sh dream`) scored 0.0: it ended after 1,661 steps without a counted READ press or a knob moved. The push overshoot
(`DEMO_PUSH_OVERSHOOT_MM`, 8 mm) was set from the first run: the 1.45 mm model value left every push 3 to 7 mm short.
