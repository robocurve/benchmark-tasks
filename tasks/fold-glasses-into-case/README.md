# Put glasses away: fold both temples, lay them in a hard case and snap the lid shut

A pair of adult eyeglasses (size 52-18-140, black acetate, 25 g) stands open on the table the way people put them
down: on the bottom of its rims and its two earpiece tips, lenses toward the robot. To its right lies an open hard
clamshell glasses case (160 x 60 x 38 mm, 90 g) whose spring holds its lid open at 105 degrees. The instruction states
only the goal: *Fold both arms of the glasses, put them in their case and snap the case shut.*

About 64% of US adults wear prescription eyeglasses (Vision Council VisionWatch, 2021: 166.5 million), and putting them
away folded in their case is a daily act. What makes it hard for a robot is physical:

- **A light, articulated object.** Each temple is a 4 x 7 mm bar on a small hinge with friction (0.01 N m, an
  estimate). Pushed on its own, the 25 g frame slides or turns on the table, so one hand has to hold the front still
  while the other pushes the temple round its hinge, and the second temple has to be folded over the first.
- **A snug case.** The case is 56 mm wide and 34 mm deep inside. The folded frame has to be turned lenses down and laid
  flat in it. A temple left more than about 10 degrees open, or glasses half outside, hold the lid open: the case
  itself checks the fold.
- **A spring lid.** The lid stands open on its spring. Pushed about 10 degrees toward shut, past its dead centre, it
  snaps shut. Pushed too hard, the loose 90 g case slides or tips.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: the glasses and the case, the lid's over-centre spring, `score()` and `success()` |
| `mechkit.py` | Multi-part objects with hinges (friction, springs) for yam_arena task files; the shared kit used by other tasks in this repository, unchanged |
| `demo.py` | The scripted demo |
| `test_scoring.py` | Offline checks (no GPU): real sizes, the snug case, every scoring outcome and attempts to score without doing the task, success only at 1.0, the lid spring, the demo's reach, and that `task.json` states the numbers in the code |

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough, from this folder:

```bash
INSTRUCTION=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/glasses-dream
```

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode. Set
`GLASSES_DEBUG=1` to print the temple angles, the lid gap and the hold count every step.

## Scripted demo

```bash
python demo.py task.py demo_out --max-steps 2400 --cam 640x352   # demo_*_cam.mp4, setup.png, result.json
DEMO_CALIBRATE=1 python demo.py task.py cal_out --no-record      # prints the CAL numbers (see below)
PYTHONDONTWRITEBYTECODE=1 python -m pytest -p no:cacheprovider test_scoring.py   # offline checks
```

1. The left hand takes the top rim of the left lens between its fingertips (closing across the 5 mm front) and lifts
   the glasses 25 mm off the table, so the earpiece tips do not drag on it.
2. The right hand pushes the right temple round its hinge with the backs of its closed fingers, 60 mm from the hinge,
   until it lies along the back of the front.
3. The left hand sets the glasses back down on the table; the right hand takes the top rim of the right lens, the left
   lets go, and the right lifts them again.
4. The left hand pushes the left temple round in the same way until it rests on the right one, and goes back.
5. The right hand lifts the folded glasses, turns them lenses down with the far (lower) edge 40 degrees low, carries
   them over the case, lowers them in until that edge is on the case floor, and lets go; they settle flat.
6. The right hand reaches behind the open lid and nudges it toward shut; past its dead centre, 10 degrees from open,
   the spring snaps it shut. The right hand goes back.

While a temple is pushed, the pushing hand leans up to 35 degrees out (away from the holding hand) and 20 degrees back (away
from the push): the YAM fingers widen 20 mm above their tips, and that wider part has to pass over the front's top rim
as the temple comes home; the lean also keeps the two wrists apart.

The demo runs open loop: it reads no simulator state. Where the glasses stand once settled, where they hang once each
hand has lifted them, and where they sit in the right hand on the way to the case are measured once with
`DEMO_CALIBRATE=1` (which reads those poses and uses them at once) and fixed in `CAL` in `demo.py`. The layout is fixed
and the simulation deterministic. `result.json` reports `score()` on the last step as the trial's score.

## Scoring

Scored every control step from the state at that step; the score on the last step counts (`SCORE_OVER = "end"`).

- A temple is **folded** within 15 degrees of fully folded (in a real trial: its angle to the back of the front, on a
  photo from above).
- The glasses are **in the case** when every part of them is inside the case's inner walls, above its floor and below
  the shut lid's underside (to 1 mm).
- The lid is **shut** when its front edge is within 1 mm of the base's rim (feeler gauge).

| Score | When |
|---|---|
| 1.0 | in the case, both temples folded, lid shut, case upright within 10 degrees, every finger pad point of both grippers 3 cm or more from the case and the glasses, glasses and case still (under 3 mm/s and 3 deg/s), all held for 1 s (20 steps in a row) |
| 0.75 | both temples folded and the glasses in the case (lid open or shut, held or not) |
| 0.5 | both temples folded |
| 0.25 | one temple folded |
| 0 | anything else; and for the rest of the run once the glasses or the case have fallen off the table |

`success()` is true only on steps where `score()` is 1.0, and the runtime ends the episode after it has held for its
own 1 s, so an episode can only end with success at 1.0.

**Why a shut lid can be judged from outside.** Lying flat on the case floor (lenses up or down), the folded frame
stands 31 mm tall (with the second temple resting on the first, 9 degrees short of its stop, as the demo leaves it);
the case is 34 mm deep inside. With either temple 10.5 degrees or more short of folded it stands
34 mm or taller, and at 15 degrees 44 mm, so the lid rests on it and cannot shut (`test_scoring.py` checks this
for both temples, both ways up, from 15 to 90 degrees open). On edge the frame is 42 mm tall and does not fit either.
So in a real trial 1.0 is judged from the video (lid shut with no gap at its front edge, the glasses nowhere on the
table, the hands off and nothing moving for 1 s), and opening the case after the trial confirms it. Partial credit is
judged on the final photo.

Attempts that do not pay: shutting the empty case scores nothing; shutting it on glasses half outside leaves the lid
open and scores at most 0.5 (for the folds); glasses dropped in unfolded are not in the case (an open temple sticks out)
and the lid will not shut; holding the shut case scores 0.75; a case knocked over is not upright; anything knocked off
the table scores 0.

## How it is modelled

All parts are rigid and at real size and mass. The glasses and the case are each one articulation (`mechkit.Mech`).

- **Temples.** Revolute joints on vertical axes with 0.01 N m of Coulomb friction (`HINGE_HOLD`, PhysX joint friction),
  from open (a stop) to fully folded (a stop at 90 degrees standing in for the temple meeting the back of the front;
  in PhysX a joint's two bodies do not collide). The two temples collide with each other, so the second one folded
  comes to rest on the first, about 9 degrees short of its stop, as on real frames.
- **The lid.** A revolute joint along the base's far top edge, open stop at 105 degrees, shut stop on the rim. Its
  spring is a torsion spring (0.05 N m/rad, `LID_SPRING`) whose rest is set every control step by the over-centre rule
  (`lid_rest()`): within 10 degrees of open the rest is open (it holds the lid open with at most 0.009 N m); past that
  it is 20 degrees beyond shut (it snaps the lid shut and keeps it pressed on the rim with about 0.017 N m besides the
  lid's weight). Damping 0.01 N m s/rad makes the snap take about 0.3 s. The lid and the base do not collide (the shut
  stop stands in for the rim); the lid collides with the glasses and the grippers.
- Everything else is contact: the frame on the table and in the case, the fingers on the rim and the temples, the lid
  on the glasses.

**Simplifications.** The front is flat (real fronts curve round the face), the lenses are flat plates, and the nose
pads do not collide. The temples are straight boxes with one bend. Acetate and a steel-core case are stiff enough to
treat as rigid. A real over-centre spring's torque falls smoothly to zero at its dead centre; here it switches there.

## Numbers and where they come from

| Quantity | Value | Source |
|---|---|---|
| Frame size | 52-18-140 | A common adult size (lens width, bridge, temple length in mm), printed inside the temple of most frames |
| Frame width, height | 139 mm, 42 mm | 2 x 52 + 18 + rims and endpieces; 35 mm lens height |
| Mass | 25 g (front 14 g, temples 5.5 g) | Acetate full-rim frames weigh about 20 to 30 g |
| Temple section | 4 x 7 mm | Typical acetate temple near the hinge (measure the frame used) |
| Hinge friction | 0.01 N m | **Estimate, to be measured** (`HINGE_HOLD`): see below |
| Case | 160 x 60 x 38 mm, 90 g | Hard clamshell cases are sold at 156 to 160 mm long, 57 to 60 mm wide, 35 to 38 mm tall; steel-core cases weigh about 90 to 100 g |
| Lid spring | 0.05 N m/rad, flips 10 degrees from open | Estimate (`LID_SPRING`, `FLIP`) |
| Frictions | case 0.6, acetate 0.5 | Estimates |

### Measuring the hinge friction

Lay the glasses flat on their front with one temple open and pointing up from the table edge, hold the front, and hook
a small spring or luggage scale (or a thread to a kitchen-scale weight) to the temple 100 mm from the hinge, pulling
square to it. Note the reading at which the temple starts to turn: torque (N m) = force (N) x 0.1. Use the median of
ten pulls, both temples, in `HINGE_HOLD`. Measure the lid spring the same way, at the lid's front edge (60 mm from the
hinge), at open and at 30 degrees from shut.

## Real-world set-up

All parts are bought. Examples whose listed sizes we found (checked November 2026; sizes are as listed, not measured by us):

- **Glasses:** Zenni Optical #108221, black full-rim acetate rectangle frame, listed 52-18-144, 138 mm frame width,
  25 g, 28 mm lens height (https://www.zennioptical.com/p/mens-fullrim-acetate-plastic-rectangle-eyeglass-frames/1082/108221).
  Against the model: temples 4 mm longer and lenses 7 mm shorter; width and mass match. Any full-rim acetate frame
  near 52-18-140 serves.
- **Case:** Eyeglass Supply Store "Large Eyeglass Clamshell Case" (metal body, rubberized outside, felt inside,
  burgundy available), listed 155 x 63 x 40 mm (https://eyeglasssupplystore.com/products/large-clamshell-case).
  Against the model: 5 mm shorter, 3 mm wider, 2 mm taller, so about 36 mm deep inside instead of 34; the frame still
  fits its length, and a temple must then be within about 11 to 12 degrees of folded, not 10.5, for the lid to shut
  (still inside the 15-degree fold rule). The listing says "strong hinge closer" but not "spring": before use, check
  that the lid stays open at about 105 degrees and snaps shut when pushed past its centre, as specified.

Mark the glasses' and the case's positions on the table with tape.

## Runtime and seeds

The layout is fixed (`-E object_jitter=0`); there are no seeds to choose. A trial runs at most 2,400 steps (2 minutes
at 20 Hz). The demo reaches full score at step 1,364 (68 s) and the episode ends with success at step 1,383; its
recording is `demo.mp4` (top camera, 640 x 352). The Dream-1 run (`dream.mp4`, 320 x 176) scored 0.0: in 2,400 steps it
reached toward the glasses and the case but ended with neither temple folded.
