# Replace a razor's blade cartridge and put the old one in a tray

A cartridge razor lies on the mat with a used cartridge (orange mark) latched on. A dispenser holds the replacement
(green mark). The robot must take the old cartridge off, fit the replacement to the same handle so that it stays on
after release, and leave the old cartridge in the tray. The scripted demo uses both arms: the right hand works the
handle, the left takes the old cartridge to the tray.

**Instruction:** `Replace the cartridge on the razor handle with the replacement in the dispenser. Put the old cartridge in the collection tray.`

**Category:** Societal relatability

**Context:** Replacing a used blade cartridge is a routine part of home grooming with a reusable cartridge razor. This
task completes the consumable-replacement operation: the old cartridge is removed and placed in a collection tray, and
the replacement is fitted securely to the same handle. The result is an assembled razor whose replacement cartridge
remains attached after release.

## Status

Simulation only. No physical razor, cartridge or robot was used. Dimensions, masses and the latch spring are estimates
for a Gillette Mach3-type handle and cartridge, and must be replaced by measurements of the sample (including the
latch's pull-off and button forces) before the task is frozen. The scripted demo has been checked in a PyBullet
stand-in (`check_demo.py`), not yet in Isaac Sim.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: handle with two latches, two cartridges, dispenser, tray; the latched, retained and stowed checks; `score()`, `demo()` and the counterexamples |
| `housekit.py` | Multi-part objects with sliding and hinged pieces, spring preload, per-step watchers and external pushes (for the hang check) |
| `household.py` | The mat, the tray, and the geometric checks shared by this group of tasks |
| `check_demo.py` | PyBullet stand-in for yam_arena: plays `demo()` or a counterexample and prints the score (`pip install pybullet`) |
| `test_replace_a_razor_cartridge.py` | Offline tests: every score tier, the latched and retained definitions, backward fitting, demo reach |
| `run.sh` | Runs the task in Isaac Sim and writes the videos to the home folder |
| `task.json` | Submission details |

## Objects

Mat coordinates in mm: origin at the mat's centre, x to the robot's right, y away from the robot, z up from the mat's
top. The mat's centre is 300 mm in front of the midpoint between the two arm bases.

| Object | Size and mass | Pose | Source |
|---|---|---|---|
| Silicone mat | 600 x 400 x 2 mm, light grey, fixed | centre at the origin | brief |
| Handle | 131 long; head 16 x 16 x 10 with a red release button on each side; connector post 5.5 x 7.5 x 7.5 with a narrower tip; 35 g, friction 0.6 | axis along y, cartridge end +y, the razor's middle at the origin, buttons up, butt resting on the mat | brief (pose); estimate (size) |
| Latch teeth (2) | 1.1 mm square teeth turned 45 degrees, one per side of the post, each on a slide with its button; spring 1000 N/m, 2 N preload | out, in the old cartridge's windows | estimate |
| Old cartridge | blade unit 14 x 38 x 6 on a sprung hinge; connector block 12 x 14 with a 6.5 x 8.5 socket, bevelled mouth, a latch window in each side wall; about 3.7 g; orange identity mark | latched on the handle | estimate |
| Replacement cartridge | the same, green identity mark and green lubricating strip | blade-down in the dispenser's pocket, socket up | estimate |
| Dispenser | 40 x 60 x 12, white, pocket 20 x 44 x 8 deep, 30 g, loose | centre x +180, y -80, turned 120 degrees | brief (position); estimate (rest) |
| Collection tray | inside floor 120 x 100, walls 15 tall, 60 g, loose | floor centre x -180, y -80 | brief |

The dispenser is turned 120 degrees so that a handle fitted to the replacement the right way round points its butt away
from the right arm's base (a top-down gripper cannot turn a full 180 degrees there).

How the connection works (all by contact, springs and stops; nothing is attached, welded or released by a rule):

- The handle's post fits down into a cartridge's socket. Pressing it home ramps both teeth in; they snap out into the
  socket's windows. Pulling it off then takes several newtons (it holds 3 N and lets go near 6 N in a pull test).
- Each tooth is one piece with a release button. Pinching the head's sides pushes both buttons flush, which pulls both
  teeth back into the post: lifting the handle then leaves the cartridge behind.
- A cartridge fitted backwards, or only resting in the dispenser under the handle, is not counted as attached.

## Scoring

Final state, highest tier fully met.

| Score | Final conditions |
|---|---|
| 0 | Anything fell off the table at any time, or the old cartridge is latched on the handle, or none of the below |
| 0.25 | Old cartridge detached but not stowed; replacement not retained |
| 0.5 | Old cartridge stowed; replacement not retained |
| 0.75 | Replacement retained; old cartridge detached but not stowed |
| 1.0 | Replacement retained and old cartridge stowed |

- **Latched:** the cartridge's socket is on the post (within 1.2 mm), square to the handle and the right way round
  (within 12 degrees), and both teeth stand out (neither pushed in more than 0.4 mm).
- **Detached:** not latched, and at least 10 mm from the handle.
- **Stowed:** the old cartridge rests on the tray's floor with its whole footprint inside the inner floor boundary, no
  finger pad touching it, for 2 s.
- **Retained:** the replacement is latched and out of the dispenser; handle and cartridge are untouched for 2 s; then
  for 2 s the handle's connector is pushed away from the cartridge with twice the handle's weight (the cartridge held
  with the same force the other way). This is the brief's hang check: the handle's own weight on the connector, with a
  margin. It must stay latched throughout and at the end.

## Scripted demo and checks

`demo()` uses both arms:

- **Right hand:** pinches the head (pushing both buttons in) and lifts the handle 15 mm off the old cartridge, keeping
  its butt on the mat; turns it to the dispenser's heading, carries it over and lowers the post into the replacement's
  socket; presses it home and lets go (the teeth snap out). It then takes the handle by the shaft, lifts the razor out
  of the dispenser, sets it down on the mat and lets go.
- **Left hand:** once the handle has been lifted away, pinches the old cartridge's connector block, carries it to the
  tray and lets go just above the floor.

891 control steps (44.5 s, the plan plus the policy's 3 s) of the 3,600 allowed.

PyBullet results (`python check_demo.py`, `--sweep`, `--counter NAME`):

| Run | Score |
|---|---|
| `demo()`, nominal | 1.0 |
| `demo()`, friction x0.7 / x1.4 / x2.0, arm stiffness x0.7 / x1.4, both low, both high | 1.0 in all eight |
| `pull_without_release`: grip the shaft and lift without pressing the buttons | 0.0 (the latches hold; the old cartridge stays on) |
| `old_left_on_mat`: replacement fitted and retained, old cartridge left where it came off | 0.75 |
| `leave_in_dispenser`: old stowed, handle fitted but the razor left in the dispenser | 0.5 |
| `keep_holding`: everything done but the razor still held at the end | 0.5 |

Friction x2.0 is included because PhysX averages two objects' friction coefficients where PyBullet multiplies them.
These are simulation results in a stand-in, not real-robot trials and not Isaac Sim runs.

## Running it

```bash
python -m pytest -q                 # offline tests
python check_demo.py                # PyBullet check of the demo (needs pybullet)
bash run.sh                         # Isaac Sim, scripted demo
bash run.sh dream                   # Isaac Sim, Dream-1
```

## Reset

1. Inspect the handle, both cartridges and both latches.
2. Refit the old (orange) cartridge on the handle until both latches click, and check it holds a light pull.
3. Put the replacement (green) cartridge blade-down in the dispenser's pocket, socket up.
4. Restore the handle, dispenser and tray to their marked poses and empty the tray.

## What is approximated

- All dimensions, masses and the latch spring are estimates; the real Mach3 latch geometry and forces were not
  measured.
- Everything is rigid boxes. The real latch is a single sprung part; here it is two sprung slides.
- The blades are not modelled as sharp; the blade unit is a box on a sprung hinge.
- The mat is rigid.

## Duplicate check

A keyword search of the public repository's pull requests on 13 November 2026 for "razor" and "cartridge" is still to
be run when the branch is pushed.
