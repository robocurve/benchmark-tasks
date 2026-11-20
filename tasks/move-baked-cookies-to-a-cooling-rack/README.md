# Move baked cookies from a sheet pan to a cooling rack with a turner

Created 11_15_26 22:40 EST.

Four brittle cookies sit on a quarter sheet pan. The robot takes an offset turner from its rest, gets the 0.6 mm blade
under each cookie, carries it level over the pan's rim, lets it slide off onto a wire rack, and puts the turner back.
No part of the robot may touch a cookie, and nothing is turned over.

## Files

| File | What |
|---|---|
| `task.py` | The task file: layout, scorer (`note_step`, `score`, `success`) and the scripted open-loop `demo()` |
| `cookie_rack/parts.py` | Sizes, masses, friction, the break criterion (`loads`), and the USD builders for every object |
| `cookie_rack/embodiment.py` | Embodiment `yam_cookie`: the stock `yam_arena` embodiment plus these objects and the contact-force monitor |
| `cookie_rack/plan.py` | Gripper-pose legs solved to joint targets with `yam_arena`'s IK; played by the stock `yam_scripted` policy |
| `test_score.py` | 34 scorer tests on made-up states: every band and each shortcut |
| `dev/` | `run.sh` (one trial), `media.sh` (videos and screenshot), `reach.py` (offline reach test), `trace.py` (log summary) |
| `task.json` | The task card |

## Run and score a trial

Needs `yam_arena` (Isaac Lab-Arena 0.3.1, Isaac Sim 6.1) and Inspect Robots, as set up by `yam-isaac-lab-arena`'s
`setup.sh`. From this folder:

```bash
source ~/IsaacLab-Arena/yam.env
uv pip install -e . --no-deps --python "$(which python)"       # registers the embodiment yam_cookie

INSTR="Move the four cookies from the sheet pan onto the cooling rack with the spatula, without touching or breaking them, then put the spatula back on its rest."

# the scripted demo (open loop; reaches 1.0)
inspect-robots run --embodiment yam_cookie -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --policy yam_scripted --no-prompt --instruction "$INSTR" \
  --scorer task_score --max-steps 2400 --seed 0 --store-frames --log-dir ~/yam-eval/cookie-rack-demo

# a model: the same command with its policy, for example --policy dream
```

The run prints `task_score: <0..1>` at the end and the log keeps it. `COOKIE_DEBUG=1` prints every object's pose, each
cookie's state and the peak measured loads five times a second. `python -m pytest test_score.py` runs the scorer tests
(no simulator; needs `numpy`, `yam_arena` and this folder on the path). `python dev/reach.py` tests the demo's reach
offline.

- **Runtime per trial:** the demo takes 94 s of simulated time (about 1,870 of the 2,400 steps) and ends itself on success;
  about 3 minutes of wall time on an L40S, including the simulator's start.
- **Seeds:** the layout is fixed (`-E object_jitter=0`) and the demo is open loop, so the seed changes nothing; `--seed 0`
  is used throughout. Two demo trials in a row gave the same score and the same final poses.

## Scoring

Each cookie ends as one of:

| | Points | Means | Real trial: how to tell |
|---|---|---|---|
| good | +0.2 | flat (within 8 degrees) with its whole outline on the rack's wires, not on another cookie or on the turner, at rest, in one piece, never touched by the robot, never dropped | photo of the rack; video |
| spoiled | -0.1 | broken at any time; or touched by any part of the robot; or dropped about 18 mm or more; or ending anywhere but the pan or the rack | count of pieces; video |
| left | 0 | still on the pan, whole and untouched; or on the rack but overhanging the frame, leaning or stacked | photo |

Score = 0.2 x good - 0.1 x spoiled, floored at 0. The last 0.2 needs all four good and, for one second: the pan where it
started (within 30 mm and 10 degrees, flat), the turner on its rest (within 15 mm and 15 degrees), both grippers 80 mm
clear of the pan, rack and turner, nothing moving. The trial scores 0 if the pan is ever tipped past 10 degrees or lifted
20 mm (no pouring from a hot pan). Doing nothing scores 0.

Why a touched cookie is spoiled rather than the whole trial zeroed: in a kitchen a handled cookie is thrown away, and the
rest of the tray is still good. Picking all four by hand therefore scores 0 (four spoiled), never more.

### What was measured (simulator, `yam_scripted`, seed 0; x86 L40S unless noted)

| Trial | Switch | Score | What happened |
|---|---|---|---|
| demo, three times (two on the L40S, one on an aarch64 host) | none | 1.0, 1.0, 1.0 | four good; peaks on any cookie (L40S runs): bending 0.012 N m, squeeze 0.03 N, press 0, fall 0.28 m/s, robot touch 0; pan moved under 1 mm |
| do nothing | `COOKIE_VARIANT=idle` | 0.0 | nothing moved |
| pick a cookie up in the gripper | `COOKIE_VARIANT=hand` | 0.0 | the cookie reached the rack flat, but the contact report showed 80 N of gripper force (touched) and 38 N of squeeze (broken): spoiled |
| shove the cookies with the blade held 4 mm off the pan | `COOKIE_VARIANT=shove` | 0.0 | the blade never got under; the cookies and the loose pan were pushed 23 cm; nothing reached the rack and the pan left its place |
| tip the cookies off from 40 mm above the wires | `COOKIE_VARIANT=drop` | 0.0 | all four landed flat on the rack, three broke on landing (bending up to 0.025 N m): one good, three spoiled |

Revision (cookie mass 30 g to 35 g, to match the maker's specification): the demo was rerun twice on the L40S and
scored 1.0 both times, and one Dream-1 run (`--policy dream`, 2,400 steps, on the aarch64 host) scored 0.0: the arms
moved towards the pan and the rack, the turner was never picked up and no cookie moved. The submitted videos and
screenshot are from these 35 g runs. The shortcut variants in the table above were run at 30 g and not repeated at 35 g;
`test_score.py` (34 tests, passing at 35 g) covers their scoring.

Not run in the simulator, covered by `test_score.py`: tipping or lifting the pan to pour (0 for the trial), cookies
overhanging, leaning, stacked, upside down or still on the blade (not counted), a cookie on the table (spoiled), the
turner off its rest, a gripper not clear, the pan pushed away (each loses only the last 0.2).

## How the physics is modelled

- **Getting under.** The blade is a 0.6 mm thick collider, true size. A cookie's base is flat with its outer 6 mm curling
  up 1.5 mm. The blade is slid along the pan pitched 6 degrees nose-down with its front edge pressed 3 mm into the floor
  (the arm's compliance turns that into a light press). Friction: cookie on pan 0.70, cookie on blade 0.25, so the
  cookie stays put while the blade wedges under, then rides up it. A blade held 4 mm up only pushes the cookie along.
- **Carrying.** Level, held by friction alone; a cookie slides off a blade tilted past about 14 degrees.
- **Setting down.** The blade is brought to about 4 mm above the wires and tipped 18 degrees nose-down; the cookie slides
  off onto the wires and the blade is drawn back.
- **Breaking.** After every physics step (240 Hz) PhysX reports every contact on every cookie: point, direction, force.
  From these the task works out, across eight diameters, the bending moment (the moment of the contact forces on one
  side of the diameter, less that half's weight) and the squeeze (force pressed in from both sides), and the total
  force pressing down on its top. Averaged over 1/60 s, a cookie breaks at 0.02 N m of bending (a 2 N load at the
  middle of a 40 mm span), 3 N of squeeze or 10 N of press. A broken cookie shows a red crack.
- **Touching.** The same contact reports give the robot's force on each cookie; 0.01 N or more is a touch. It is a
  contact force, not a distance.
- **Dropping.** A cookie moving down faster than 0.6 m/s (a free fall of about 18 mm), read after every physics step,
  is dropped. A shorter fall can still break it by the bending criterion, as the 40 mm trial shows (it slides off the blade
  rather than falling free, reaching 0.49 m/s).

## Real-world setup and safety

Objects as in `task.json`'s setup; all are ordinary bakery items.

- **Cookies, a standard source.** Otis Spunkmeyer Sweet Discovery Triple Chocolate frozen cookie dough, item 58814: 1.33 oz
  (38 g) pre-shaped pucks, 240 to a case (about $145 a case from foodservice suppliers such as WebstaurantStore, so about
  $0.60 a cookie). The maker's finished-food specification gives the baked cookie as 2.85 to 3.25 in (7.2 to 8.2 cm)
  across, target 3.05 in (7.7 cm), and 1.25 oz (35 g); the simulated cookie is 76 mm across, 10 mm thick and 35 g. The
  thickness is not in the specification: 10 mm is an estimate (a 35 g disc 77 mm across at about 0.75 g/cm3 is 10 mm
  thick), to be confirmed with a ruler on the first batch.
- **Preparation, about 35 minutes for a batch of 12 to 16.** Bake from frozen on a parchment-lined tray at the maker's
  setting (convection oven 300 F for 11 to 14 minutes; rack oven 300 F for 12 to 14). Cool on the tray at room
  temperature for 20 minutes, until firm and below 40 C (contact or infrared thermometer). The 20 minutes is this task's
  choice, so that every trial uses cookies at the same, firm but brittle, state; the maker says only to cool at room
  temperature.
- **Acceptance, ruler and kitchen scale.** Use a cookie only if it is 72 to 82 mm across (mean of two perpendicular
  diameters, the maker's range), 31 to 39 g, whole, and sits flat without rocking. Its underside is flat and browned from
  the tray, with the edge rounded up about 1 to 2 mm (the simulated cookie's base edge curls up 1.5 mm).
- **Placing.** Set four accepted cookies flat side down on the bare, clean, dry trial pan at the four marked places with
  a spatula. The trial pan is not heated; the task does not need a hot pan, and none is used.
- Mark the pan's outline, the rack's outline and the rest's place on the table with tape at setup. Fix the turner's rest
  to the table (tape or a clamp). The pan and rack stay loose.
- The turner's blade is thin but not sharpened; use a turner with rounded corners. Nobody reaches into the workspace
  while the arms are enabled. Forces are low: the turner weighs 69 g and a cookie 35 g.
- Cookies handled by the robot's grippers, or that leave the pan and rack, are discarded, not eaten.
- Reset: an accepted cookie (same preparation and acceptance) for any broken or touched one; pan, rack and turner back on
  their marks. A batch left more than 2 hours after cooling is not used, so the cookies' crispness stays the same.

| Quantity | In the simulator | In a real trial |
|---|---|---|
| Cookie on the rack, flat, wholly on the wires | pose of the cookie in the rack's frame | photo from above; a ruler for the overhang |
| Cookie in one piece | break criterion on measured contact forces (bending, squeeze, press) | look: whole or in pieces, at the end and on the video |
| Robot touched a cookie | PhysX contact force between any robot link and the cookie, 0.01 N or more | video from the top and wrist cameras; any contact counts |
| Cookie dropped | downward speed over 0.6 m/s | video: a fall of about 2 cm or more |
| Pan in place, never tipped or lifted | pan pose every step | tape outline and a ruler at the end; video for tipping |
| Turner on its rest | turner pose against its starting pose | photo; ruler against the tape mark |
| Grippers clear, all still | fingertip midpoints 80 mm from the three objects' boxes; speeds | ruler; video |

## What is simplified

- A cookie is one rigid body. It cannot crumble, chip, bend or shed crumbs, and a "broken" cookie keeps its shape (it is
  marked, and scored as broken). The break thresholds are chosen in the range of soft, still-fragile cookies, not
  measured on a batch; PhysX's contact forces during an impact depend on its time step, so the height at which a drop
  breaks a cookie is approximate (the separate drop criterion does not depend on it). The margins are narrow: the demo's
  gentlest set-down loads a cookie to 0.012 N m against the 0.02 N m limit, and a 40 mm tip-off to 0.021-0.025 N m.
- Friction coefficients are fixed and uniform. Cookies do not stick to the pan, so nothing has to be released, and the
  pan is not hot.
- The blade does not flex. The cookie's contact keeps a 0.3 mm rest gap so that it cannot sink through the 0.6 mm
  blade; without it the simulator let a cookie pass through the blade in one trial.
- With the demo's light touch the pan moves less than 1 mm, so the second arm is not needed to steady it and the demo
  leaves it parked. A clumsy push does move the pan (23 cm in the shove trial), and the scorer scores its place.
- The demo holds the handle with the fingers 3 mm past its faces rather than fully closed: fully closed, both fingers sit
  at their force limit and the handle buzzes sideways in the simulator, which walks a cookie off the blade.
- The cookies start on the far two thirds of the pan, so that the blade can be set down in front of each one. The layout
  is fixed; nothing is randomised.
