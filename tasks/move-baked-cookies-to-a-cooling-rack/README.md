# Move baked cookies from a sheet pan to a cooling rack with a turner

Four brittle cookies sit on a quarter sheet pan. The robot takes an offset turner from its rest, gets the 0.6 mm blade
under each cookie, carries it level over the pan's rim, lets it slide off onto a wire rack, and puts the turner back.
No part of the robot may touch a cookie, and nothing is turned over. Real trials use a non-perishable cookie stand-in
of the same size, mass and friction that comes apart in two when bent as hard as the simulated cookie breaks (see
"Cookie stand-in" below).

## Credit, and how the review is met

This task takes up the idea of an earlier submission by GitHub user `bfefferm-pers`, "Move baked cookies from a sheet pan to a cooling rack with a turner, without touching or breaking them", reviewed as TK-67G3V5 and TK-7D26TA. The hold on the idea lapsed with no further revision. This version starts from that submission's files and answers each review point by point below.

| Reviewer point | What changed |
| --- | --- |
| Specify a non-perishable stand-in for the cookies that keeps fidelity to the real context | New section "Cookie stand-in": a reusable printed PETG disc with the simulated cookie's size, shape (the curled base edge the blade gets under), mass and friction, made as two halves held by four small magnets so that it comes apart under the bending moment at which the simulated cookie breaks. Acceptance checks for each stand-in: mass, size, base curl, flatness, a three-point break test and tilt-slide tests on the pan, the turner's blade and the rack, each tied to the simulated value. "Real-world setup and safety", the measurement table, `task.json`'s setup, scoring and reset notes now use the stand-in instead of baked cookies. |
| Earlier: change the domain to societal relatability | Kept. |
| Earlier: a standard source and size for the cookies | Kept as the reference the stand-in copies (size and mass of the named baked cookie). |
| Results on this revision | The scripted demo (1.0 in 1,872 steps) and Dream-1 (0.0 at the 2,400-step limit) were rerun on this revision's task file, and the 34 scorer tests passed. |

The stand-in is specified to the simulated cookie's mass, size and friction, so the simulation parameters, the scorer,
the scripted demo and the tests are unchanged. Whether a printed base reaches the simulated friction on the pan and on
the blade together has not been measured; "Cookie stand-in" says what to change if it does not. The scripted demo
and Dream-1 were rerun on this revision's task file (see "What was measured").

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
| spoiled | -0.1 | broken at any time; or touched by any part of the robot; or dropped about 18 mm or more; or ending anywhere but the pan or the rack | a stand-in's halves parted; video |
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

These runs were made on earlier revisions of this task.

Earlier revision (cookie mass 30 g to 35 g, to match the maker's specification): the demo was rerun twice on the L40S and
scored 1.0 both times, and one Dream-1 run (`--policy dream`, 2,400 steps, on the aarch64 host) scored 0.0: the arms
moved towards the pan and the rack, the turner was never picked up and no cookie moved. The shortcut variants in the
table above were run at 30 g and not repeated at 35 g; `test_score.py` (34 tests, passing at 35 g) covers their scoring.

Runs of this revision (task file SHA-256 prefix `5bec6a6dc68e`, seed 0), 19 November 2026:

- **Scripted demo (`yam_scripted`): 1.0, ended on success in 1,872 steps.** Log: `runs/demo/adhoc_b97eec3b.json`
  (with `runs/demo/run.log`).
- **Dream-1 (`--policy dream`, `dreamlabs/dream-1-beta`): 0.0, ended at the 2,400-step limit (`max_steps`).** Log:
  `runs/dream/adhoc_52f0b56b.json` (with `runs/dream/run.log`).
- **Scorer tests:** the 34 tests in `test_score.py` passed.

The submitted videos and screenshot are from these two runs.

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

Objects as in `task.json`'s setup; all but the cookie stand-ins are ordinary bakery items.

- **Cookies: a non-perishable stand-in.** Real trials use the printed stand-in specified in "Cookie stand-in" below,
  not baked cookies. It copies the simulated cookie (76 mm across, 10 mm thick, 35 g, the base's outer 6 mm curled up
  1.5 mm, the same friction on the pan, the turner and the rack) and comes apart in two halves under the bending moment
  at which the simulated cookie breaks. The size and mass are those of the earlier revision's standard cookie, a baked
  Otis Spunkmeyer Sweet Discovery Triple Chocolate puck (item 58814), whose finished-food specification gives 2.85 to
  3.25 in (7.2 to 8.2 cm) across, target 3.05 in (7.7 cm), and 1.25 oz (35 g).
- **Placing.** Set four accepted stand-ins flat side down on the bare, clean, dry trial pan at the four marked places,
  each with its joint running left to right as the robot sees it (along y, parallel to the turner's front edge as the
  blade comes in from the robot's side). The trial pan is not heated; the task does not need a hot pan, and none is used.
- Mark the pan's outline, the rack's outline and the rest's place on the table with tape at setup. Fix the turner's rest
  to the table (tape or a clamp). The pan and rack stay loose.
- The turner's blade is thin but not sharpened; use a turner with rounded corners. Nobody reaches into the workspace
  while the arms are enabled. Forces are low: the turner weighs 69 g and a cookie 35 g.
- The stand-ins are not food. Nothing in the cell is eaten.
- Reset: any stand-in that came apart is pressed back together (its own two halves, marked as a pair) and checked flat;
  any stand-in that was dropped on the floor is re-checked with the acceptance tests below before reuse; pan, rack and
  turner back on their marks. The pan, blade and rack are wiped dry with a clean cloth once per session, since grease
  or dust changes the friction.

| Quantity | In the simulator | In a real trial |
|---|---|---|
| Cookie on the rack, flat, wholly on the wires | pose of the cookie in the rack's frame | photo from above; a ruler for the overhang |
| Cookie in one piece | break criterion on measured contact forces (bending, squeeze, press) | the stand-in's two halves still joined, at the end and on the video (it parts when bent past the simulated limit) |
| Robot touched a cookie | PhysX contact force between any robot link and the cookie, 0.01 N or more | video from the top and wrist cameras; any contact counts |
| Cookie dropped | downward speed over 0.6 m/s | video: a fall of about 2 cm or more |
| Pan in place, never tipped or lifted | pan pose every step | tape outline and a ruler at the end; video for tipping |
| Turner on its rest | turner pose against its starting pose | photo; ruler against the tape mark |
| Grippers clear, all still | fingertip midpoints 80 mm from the three objects' boxes; speeds | ruler; video |

## Cookie stand-in

A real trial needs four cookies that are the same every time and keep for months, so it uses a printed stand-in rather
than food. It is made once (eight, so there are spares) and reused; it is accepted by the checks below, which tie it to
the numbers the simulation uses. The simulated cookie is unchanged by this revision.

**What it is.** A disc printed in tan or light-brown PETG, 76 mm across and 10 mm thick, with the simulated cookie's
shape: a flat base whose outer 6 mm curls up 1.5 mm (a straight ramp from the flat to the rim is enough), and a flat
top. It is printed as two half-discs, split along a diameter, held together by four 1/8 x 1/16 in N52 neodymium disc
magnets (K&J Magnetics D21B-N52 or equal): two in each half's joint face, 20 mm either side of the centre at mid-height,
glued (cyanoacrylate) into 3.2 mm pockets with opposite poles facing across the joint. Joined, it is one rigid body, as in
the simulation. Bent across the joint past about 0.02 N m, the joint opens and the halves part: that is a broken cookie.
The two halves of a stand-in are marked as a pair.

**Printing.** 0.4 mm nozzle, 0.2 mm layers, printed upside down (the top face on a smooth PEI sheet), so the curled base
is the print's top surface, finished with ironing on. Three perimeters and four solid top and bottom layers; the infill
(gyroid, about 35 to 45 percent) is set so that the finished stand-in, magnets in, weighs 35 g. A solid disc of this
size in PETG (1.27 g/cm3) would weigh about 56 g. The magnets' pocket depth sets the break: each magnet's face sits about
0.3 mm back from the joint face to start with, and deeper pockets make a weaker joint.

**Acceptance checks** (each stand-in, once after making it and after any drop on the floor):

| Check | How | Accept | Simulated value |
|---|---|---|---|
| Mass | kitchen scale reading 0.1 g | 34.0 to 36.0 g | 35 g |
| Size | calipers: two perpendicular diameters and the thickness at the centre | 75.5 to 76.5 mm across, 9.7 to 10.3 mm thick | 76 mm, 10 mm |
| Base curl | on a flat steel plate: a 1.0 mm feeler gauge enters under the rim all round, a 2.0 mm one does not | both | 1.5 mm |
| Sits flat | on the trial pan's floor, pressed at the rim | no rocking | flat |
| Break | three-point bend across the joint: the stand-in rests on two round rods 40 mm apart with the joint midway, and a cup on a thread hangs from a 6 mm bar laid across the top along the joint; weights are added to the cup | holds 160 g (1.6 N, 0.016 N m); parts at or before 250 g (2.45 N, 0.025 N m) | 2.0 N at the middle of a 40 mm span, 0.02 N m |
| Friction on the pan | tilt-slide: the stand-in flat on the trial pan's floor, the pan tilted slowly (about 1 degree a second) about its short side; a phone inclinometer on the pan's rim reads the angle at which the stand-in starts to slide; the mean of three tries | 33 to 37 degrees (friction 0.65 to 0.75) | 0.70 (35.0 degrees) |
| Friction on the turner | the same on the turner's blade, the turner clamped with its blade level and then tilted about the handle's cross axis | 12 to 16 degrees (0.21 to 0.29) | 0.25 (14.0 degrees) |
| Friction on the rack | the same on the cooling rack, tilted about its short side, the stand-in across the wires | 24.5 to 28.5 degrees (0.46 to 0.54) | 0.50 (26.6 degrees) |

Tuning: if the mass is off, change the infill; if the break is off, change the pocket depth; if a friction angle is off,
change the base's surface finish (ironed, as printed, or sanded with 400-grit paper) and test again. The simulated
coefficients are the targets. Whether a printed PETG base reaches the pan's 0.70 and the turner's 0.25 together has not
been measured. If no finish passes both, measure the stand-in's three coefficients, set the cookie pairs in `MU` in
`cookie_rack/parts.py` to them (PhysX averages the two surfaces' coefficients; the comment there gives the pairs), and
rerun the demo and the shortcut variants before any real trial; then the simulation matches the stand-in rather than the
other way round.

**What the stand-in does not reproduce.** It shows a break only by bending across its joint, so it records the bending
breaks the turner causes (bridging between the blade's edge and the pan while the blade gets under, and hanging over the
blade's edge while it slides off onto the wires), not a bend about another diameter, a squeeze or a press; the
simulation scores all of these. It does not crumble or shed crumbs, and it does not stick to the pan, as in the
simulation.

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
