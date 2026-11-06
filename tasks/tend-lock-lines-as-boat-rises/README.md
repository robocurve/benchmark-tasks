# Tend a boat's bow and stern lines while the lock fills, then make both fast

A model navigation lock stands on the table. A grey lock wall with a rounded coping runs across the table in front of
the robot; beyond it, in the lock chamber, an orange model boat lies with its fenders against the wall. Its bow line
(robot's left) and stern line (robot's right) run from cleats on the boat up over the coping, down the wall face and
along the table towards the robot. Three seconds after the start the lock fills and the boat rises 11 cm in 12 s, while a steady push (the filling water) drives it away from the wall. As the boat rises, each
line slackens by about 7 cm. The robot must take in both lines continuously, at the same time, so that neither end of
the boat swings off the wall. When the boat stops rising, it makes each line fast by laying it into its slotted cleat
at the toggle that leaves the line just taut, and lets go.

The tested ability is tending two lines to a load that moves by itself: the line goes slack at a rate set by the
water, not by the robot, so a line held still, or taken in late, lets the boat drift. Line handlers in a lock do
exactly this: a US Army Corps of Engineers boater guide says the procedure "will require a person at each end of the
boat to pay out or take in mooring lines as the water level changes", and 33 CFR 207.476(f)(5) requires "constant
attention to the mooring lines, to provide slack or retain tautness as needed." O*NET 53-5011.00 (Sailors and Marine
Oilers) lists the task "Handle lines to moor vessels to wharfs, to tie up vessels to other vessels, or to rig towing
lines."

Sources:
- USACE Mobile District, "Locking Through" brochure, item 10: "This procedure will require a person at each end of the
  boat to pay out or take in mooring lines as the water level changes." Item 11: "Stand by to pay out or take in
  mooring line as the water level in the lock rises or falls."
  https://webarchive.library.unt.edu/web/20130228073039id_/http://operations.sam.usace.army.mil/Locking_Through_Brochure.pdf
- 33 CFR 207.476(f)(5): "(5) While moored in the lock, the operator of the craft shall maintain constant attention to
  the mooring lines, to provide slack or retain tautness as needed." https://www.ecfr.gov/current/title-33/section-207.476
- O*NET 53-5011.00 Sailors and Marine Oilers: "Handle lines to moor vessels to wharfs, to tie up vessels to other
  vessels, or to rig towing lines." https://www.onetonline.org/link/summary/53-5011.00

The instruction states only the goal ("The lock is filling and the orange boat will rise. Using only its two lines,
keep the boat against the lock wall the whole time it rises. When it stops rising, make each line fast in its slotted
cleat, taut, with the boat against the wall, and let go of both lines."). It does not say how fast to take in, that
the slack must be taken out before the fill, or which toggle leaves the line taut: that is what the task tests.

Skills: constraint-keeping (a moving load kept in place through a line), routing and seating (laying a line into a
slot behind a stop), placement. Industry: transport (marine). Two arms: reach-forced and simultaneous (below).

## Scene (station table; cm from the midpoint between the arm bases, x forward, y left, heights above the table)

| object | type | size | material (real rebuild) | position |
|---|---|---|---|---|
| lock wall | fixed box | 5 x 64 x 10.5 cm | grey-painted MDF | x 32-37, y -32..32 |
| coping | fixed half-round on the wall top | 2.5 cm radius, 64 cm long | half of a 50 mm PVC pipe, grey | axis at x 34.5, height 10.5 (top at 13) |
| boat | free rigid block, 150 g | 8 x 50 x 2.5 cm | orange PLA or painted wood | wall-side edge at x 38.5 (1.5 cm off the wall: the fender stop), y -25..25, bottom 0.5 cm up |
| lift carriage | scripted vertical slide under the boat | travel 11 cm | lead-screw linear stage | raises the boat from t 3 s to t 15 s at a constant 0.92 cm/s |
| cross slide and yaw bearing | boat mount on the carriage | x 0-10 cm away from the wall, yaw +-25 deg | linear slide + 608 bearing | the boat can drift away from the wall and swing; the stop at x 0 is the fender contact (a stiff spring, 400 N/m) |
| outward push | constant force on the boat from t 3 s | 0.08 N away from the wall, 2 N s/m drag | 8 g weight on a pulley, light oil dashpot | - |
| deck cleats | on the boat deck | 10 x 30 x 6 mm | black PLA | x 39.5, y +-23 |
| bow / stern line | 6 mm cord, 42 cm, 25 g each | blue braided polyester cord | tied to its deck cleat, up over the coping, down the wall face, along the table at y +-23 |
| toggles | 4 per line, fixed on the cord | 10 (along) x 28 (across) x 8 mm | white PLA, glued to the cord | 21.4, 26.4, 31.4 and 36.4 cm from the deck cleat (start: 6 cm up the wall face, 1.4 cm up at its foot, then on the table at x 25.8 and 20.8; the tail end at x 15.3) |
| slotted cleats | 2 fixed posts each | 12 x 12 x 60 mm posts, 12 mm slot between them | black PLA | slot centred at x 18, y +-27.5 (4.5 cm outboard of each line) |
| water | blue panel in the chamber | 25 x 64 cm | blue card on the carriage | rises with the boat (visual only) |

With the boat at the top and on its fenders, the second toggle (26.4 cm from the deck cleat) lies just in front of
the posts when the line is laid into the slot (the toggle may sit anywhere from the table to 5 cm up the posts; that
changes the boat's position by about +-1.2 cm). The first toggle cannot reach the posts without pulling the boat 5 cm
through its fenders; the third leaves 5 cm of slack, so the line no longer holds the boat (in the model it stays put
only because the slack line's friction on the coping holds it), and the scoring counts only the second toggle.

## How this differs from similar tasks

- Married falls / two-line load transfer: the robot moves the load through two lines. Here the load moves by itself on
  a fixed schedule; the robot cannot speed it up or stop it, only keep up with it.
- Tensioning a messenger wire and hanging its ring at a mark (board): the wire is anchored and still, and only the
  slider is placed. Here the line's slack is created continuously by an outside motion and must be removed as it
  appears.
- Other board tasks with lines and boats: hauling a dock line across with a messenger and dropping its eye over a
  bollard, taking a round turn on a bollard, making an anchor line's bitter end fast before lowering the anchor,
  taking up patch-cord slack on spools, and herding cans with a seine rope are static belays or hauls with the load
  still or moved by the robot. Letting a carton down a parcel slide is paying out a load the robot holds back.
- Board tasks with outside motion: lifting a cable so a cart can pass, smoothing a cake as a turntable turns and
  feeding a seam past a needle react to a motion, but nothing is tethered to the moving part. No board task has a load
  that moves by itself while tethered, where two lines must be tended through the motion and then made fast at the
  right length.
- Paying out a tether while a crawler moves (screened out earlier): paying out needs stored slack; this task takes
  line in.

## Why two arms

Reach-forced and simultaneous. The bow line, its toggles and its slotted cleat are reachable by the left arm only, the
stern line and its cleat by the right arm only. Offline IK with the gripper pointing down, 1.5 cm and 6 cm above the
table, at the bow line's working points (x 8, 14, 20, 26 and 28 cm at y +23; x 16, 20 and 24 cm at y +27.5): the left
arm reaches every point at yaw 0.4 rad with 0 mm error; the right arm finds no solution at any of them for any yaw
from -90 to +90 degrees (13 yaws, 6 seeds each). By symmetry the same holds for the stern side with the arms swapped.
Both lines slacken at the same rate during the same 12 s, and a line left
alone lets its end of the boat swing more than 3 cm off the wall (the one-arm run below), so both lines must be taken
in at once.

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 installed (`source IsaacLab-Arena/yam.env`).
- `cablekit.py`, `cableshape.py` and `cabledemo.py` are in this folder (the cable simulation, the shaped start of the
  lines and the scripted-demo runner).

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1200 \
    --instruction "The lock is filling and the orange boat will rise. Using only its two lines, keep the boat against the lock wall the whole time it rises. When it stops rising, make each line fast in its slotted cleat, taut, with the boat against the wall, and let go of both lines."
```

## Scripted demo

```bash
python cabledemo.py task.py demo_out --max-steps 1200
```

Result (`demo/`, recorded after the last change to `task.py`): score 1.0, success after 440 of 1,200 steps. The demo
takes up about 4.5 cm of sag in both lines before the fill, then takes in both lines together in 0.25 s steps closed
loop on each boat end's drift; the largest drift during the rise was 8.9 mm. It then lays each line into its slot with
the second toggle in front of the posts and lets go; the boat ends finish 2.7 mm and 0.2 mm inside their fender
positions. Wrist-camera mean brightness (0-255, every 2 s): left 115-153, right 130-161; the top camera shows the whole
lock, both lines and both arms.

## Scoring (`score()`, evaluated at the end; `success()` must hold 1 s)

Score 0 if any of these holds:
- at any control step until 1 s after the rise ends (t 16 s), either end of the boat's wall-side edge was more than
  3 cm from its fender position (latched for the whole trial);
- a gripper came within 4 cm of the boat at any time (latched: only the lines may move the boat);
- the trial ended before t 16 s;
- at the end, a gripper is within 3.5 cm of any point of either line or any toggle (the lines were not let go).

Otherwise the score is 0.5 per line made fast. A line is made fast when its second toggle (26.4 cm from the deck cleat:
the one that leaves the line taut with the boat on its fenders; the third and fourth leave 5 and 10 cm of slack) has
its centre 0-2.5 cm in front of the robot-side face of that line's slotted cleat (within 14 mm of the slot's centre
line sideways, below the post tops), the line passes through the 12 mm slot below the post tops, and that end of the
boat is within 1.5 cm of its fender position (either way).

| score | end state |
|---|---|
| 1.0 | both lines laid in their slots with the second toggle in front of the posts, both boat ends within 1.5 cm of the fenders, boat kept within 3 cm throughout the rise, both lines let go |
| 0.5 | the same for one line only; the other line not in its slot, made fast at a slack toggle, or its end more than 1.5 cm off the fender |
| 0 | any of the zero conditions above, or no line made fast |

Touching, grasping or pulling a line earns nothing by itself: the lowest tier needs a line through its slot with a
toggle stopped behind it and the boat held at the wall, after a rise in which neither end drifted 3 cm.

## Shortcuts (scripted variants of the demo, `LOCK_DEMO=<variant>`; evidence in `scratch/sc_<variant>.json/.log`)

| variant | what it does | score | why |
|---|---|---|---|
| none | waits | 0 | both boat ends drift 72 mm during the rise |
| hold | takes both lines and holds them still | 0 | the rise slackens the held lines; drift 72 mm |
| onearm | the left arm tends and makes fast the bow line; the stern line is left | 0 | the stern end drifts 76 mm |
| pullend | waits for the rise, then takes in both lines and makes both fast (both toggles end seated, boat back at the wall, lines let go) | 0 | drift 72 mm during the rise is latched |
| fastfirst | makes both lines fast before the fill, at the toggle nearest the posts then | 0 | a line made fast at its pre-fill length goes slack as the boat rises; drift 72 mm |
| wrongloop | tends correctly, makes fast at the third toggle | 0 | the boat does not take up the extra 5 cm, the third toggle never reaches the posts and the line is not in the slot; a seated third toggle counts nothing in any case |
| holdend | correct, but keeps holding both lines after laying them in the slots | 0 | grippers within 3.5 cm of the lines at the end |
| tier05 | tends both, makes only the bow line fast, lets go of the stern line | 0.5 | the bow line made fast, the stern line not in its slot (partial tier check) |

## Runtime and seeds

- Run limit 1,200 steps (60 s at 20 Hz); the rise is over at t 15 s. One trial takes about 5-8 minutes wall time on one
  GPU including the 1-2 minute Isaac Sim start-up.
- The layout is fixed with `object_jitter=0`; the boat is held by its mount and the lines are laid in the same shape
  at every reset, so every trial starts identically.
