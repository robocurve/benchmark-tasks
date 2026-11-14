# Cap the bare rebar stubs on a footing form, leaving the capped ones as they are

A plywood footing form stands on the table with five vertical #4 reinforcing bars (12.7 mm steel) sticking up out of it
in a row. Two of the bars already wear a red plastic safety cap. Three are bare. Three more caps stand on grey pads in
front of the bare bars. The instruction is: **"Make the rebar stubs safe for the crew to walk the formwork."**

To do it, the robot has to:
- tell which stubs are still bare (the two outer ones are already capped, and must stay that way);
- pick a loose cap up by its square head, carry it clear over the bar tips, and bring its flared mouth down onto a bare bar;
- press it down until the bar is about 30 mm up the cap's bore, so the cap is plumb and covers the tip;
- do that for all three bare stubs, then take both arms away from the form.

## Why it matters

Reinforcing bars that stick up out of formwork, footings and slabs are an impalement hazard to anyone who trips or falls
onto them. OSHA's rule for concrete and masonry construction, 29 CFR 1926.701(b) (https://www.osha.gov/laws-regs/regulations/standardnumber/1926/1926.701,
read on 14 November 2026), says: "All protruding reinforcing steel, onto and into which employees could fall, shall be
guarded to eliminate the hazard of impalement." The usual guard is a plastic cap pressed over each bar end. Construction
laborers and other site workers place them by hand, and a site is walked by everyone on it. The
instruction does not say what to do or with what: the robot has to see the hazard (bare steel tips among capped ones) and
find the caps. The manipulation is the hard part: a cap's blind bore leaves about 0.65 mm of play round the bar, so the cap
has to be brought down plumb over the tip, guided by a flared mouth that forgives about 20 mm of error, and pressed home;
a soft arm lands 3 to 6 mm off and the pinch itself shifts the cap in the hand by up to 12 mm.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: the footing form, the five caps, the monitor, `success()` and `score()` |
| `logic.py` | The scoring (pure Python and NumPy, no Isaac Sim): which bars wear a seated cap, arms clear, hold |
| `geometry.py` | Every dimension, shared by the task, the assets, the scoring, the demo and the tests |
| `build_assets.py` | Writes `assets/form.usda` (plywood floor, five bars, three pads) and `assets/cap.usda` (the safety cap) |
| `demo.py` | The scripted demo and its ablation modes |
| `demokit.py`, `scenekit.py` | Helpers for scripted demos and USD bodies in a yam_arena task file (unchanged template files) |
| `check_plan.py` | Offline check of every demo mode against the arms' IK (reach, joint margins) |
| `run.sh` | `./run.sh dream` and `./run.sh demo` (and the ablation modes) |
| `tests/` | Unit tests for the geometry, the scoring, the demo's numbers and task.json |

## Run a trial

    source /data/IsaacLab-Arena/yam.env                    # the Isaac Lab-Arena environment
    cd tasks/cap-exposed-rebar-stubs-on-a-footing-form
    python build_assets.py                                 # only if assets/ is missing (needs Isaac Sim's Python)
    ./run.sh dream                                         # Dream-1 policy, 2,400 steps (expected 0.0)
    ./run.sh demo                                          # scripted demo (expected 1.0, 1,312 steps)

or directly:

    inspect-robots run --embodiment yam_arena --policy dream \
      -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "Make the rebar stubs safe for the crew to walk the formwork." \
      --scorer task_score --max-steps 2400 --store-frames --log-dir out/dream

Runtime: a demo trial is 1,312 control steps (about 66 s of simulated time); wall time depends on the machine's load. There are no seeds: the layout is fixed and `object_jitter=0`.

## Scripted demo

`python demo.py task.py demo_out` plays the plan, scores it, and writes one MP4 per camera, `setup.png` and `result.json`.
For each bare stub, in order from the right (y = -0.11) to the left (+0.11), the arm on that side (the middle stub is done by
the left arm) pinches the cap standing in front of it by its 70 mm head, with the fingers closing along x, lifts it 10 mm
above the bar tips, carries it over the stub, lowers it until the flared mouth finds the bar, presses it down to the seat,
opens, moves the open fingers back to centre round the cap (the fixed pad still touches it), lifts away, and goes home. The plan is open loop: fixed waypoints computed from `geometry.py`, plus one measured number per
pick (`GRASP_DX`): closing the jaws pushes a cap along x until it meets the pad fixed to the hand, which leaves it up to 12 mm
off the tool point, and how far varied from run to run, so the plan moves the tool point 6 mm the other way and the 21 mm
flare takes the rest. It does not read the
simulator's object poses while it runs.

## Scoring

The score is the state at the current step; nothing is latched. A cap is **seated** on a bar when:
- the bar tip is at least 26 mm up the cap's 30 mm bore;
- the bore axis is within 3 mm of the bar axis, horizontally;
- the cap's axis is within 8 degrees of vertical.

With n of the five bars wearing a seated cap (two do at the start):

| Score | State |
|---|---|
| 0 | n = 2 or fewer (nothing done, or progress undone) |
| 0.25 | n = 3 |
| 0.50 | n = 4 |
| 0.75 | n = 5, but not yet held with the arms clear |
| 1.00 | n = 5, both tool points at least 15 cm (horizontally) from every bar or above 25 cm over the table, held 3.0 s (60 steps at 20 Hz) |

Knocking a cap off a bar lowers the score again, because that bar is bare again. A cap standing on a bar's tip, lying on
its side, or left on the pad does not count. A cap pressed onto a bar that was already capped does not count twice.

**Real-world scoring**: photograph the form from above and measure with a ruler: a seated cap's head top face is 82 to
88 mm above the floor (bar 70 mm, cap 44 mm tall, bar 26 to 32 mm inside) and a protractor against the cap reads at most
8 degrees. Count the caps that pass; the rest is as above.

## Real-world build

- Footing form: a 100 x 600 x 30 mm board of plywood on the table.
- Bars: five 12.7 mm (#4) steel rebar cut-offs, 100 mm long, each stood 30 mm deep in a drilled hole in the plywood (70 mm
  showing), at 110 mm centres along the long side.
- Caps: seven (five on the form, plus spares) rebar safety caps in red polyethylene, square head about 70 mm, with a stem
  that fits #4 bar (these are sold as "rebar caps" for 1/2 in bar; choose ones with a square 14 mm bore or measure the bore
  and adjust the play). The two outer caps are pushed on by hand until the bar tip meets the bore roof.
- Pads: three 70 x 70 x 4 mm rubber pads at x = 0.175 m, y = -0.11, 0 and +0.11.
- Safety: the bars are cut and filed, the caps are put on by hand before each trial and the form is handled with gloves;
  no pinch point larger than the gripper's.

## Simulation fidelity

- Rigid bodies, 240 Hz physics, 20 Hz control; the arms are the YAM's soft-position-controlled arms.
- The bars are 12.7 mm steel cylinders. The caps are box colliders: a stem ring, a head ring and a roof round a 14 x 14 mm
  square bore, with a 45 degree flared mouth (56 mm across at the opening, down to the bore over 21 mm) built from tilted boxes. The simulated cap holds on by its bore and gravity
  only: there is **no interference fit or snap**, so a cap would also stay if it were simply dropped onto a bar from a
  few centimetres. A real cap's friction fit would resist the first millimetres of travel.
- Cap friction is 0.45 (plastic on steel); the footing form is 0.8. The bars are 70 mm tall, much shorter than the 300 mm or
  more of typical starter bars, so that the arms can reach a cap carried over the tips and seated; a taller stub puts the carried cap above the arms' reach. The cap, the bar and the form's footprint are real size; the cap's mass
  (0.05 kg) is in the range of real caps.
- The scene is one fixed layout; the capped bars are the outer two.

## Ablations

Layout as shipped, one run each, `./run.sh <mode>`; scores are the scorer's final state.

| Mode | What it does | Expected | Measured | Steps |
|---|---|---|---|---|
| `demo` | All three caps, arms home | 1.00 | 1.00 (run twice, identical) | 1,312 |
| `one_cap` | Only the first cap (stub y = -0.11) | 0.25 | 0.25 | 560 |
| `two_caps` | The first two caps | 0.50 | 0.50 | 960 |
| `off_target` | The demo with the hand shifted 40 mm along x, so each cap arrives 28 to 52 mm off its bar, beyond the 21 mm the flared mouth can catch | 0.00 | 0.00 | 1,360 |
| `stay` | All three caps, then the right arm waits over the form | 0.75 | 0.75 | 1,310 |
| `settle` | No motion | 0.00 | 0.00 | 280 |
| `dream` | Dream-1 baseline policy | 0.00 | 0.00 | 2,400 |

All measured values are the final scene and plan, one run each except the demo. Earlier versions of the cap (a narrower flared
mouth) failed on the third cap when the pinch left it 17 mm off the tool point; that is why the mouth is 56 mm across.
In the simulator a cap released a few millimetres above its seat slides the rest of the way by itself, because there is no
interference fit, so pressing it home is not what is being tested so much as bringing it down plumb and within about 20 mm
of the bar.

### Dream-1 recording

The Dream-1 run completed all 2,400 steps with no errors and scored 0.0. In its video the arms rise from their ready pose and
hover beside their bases, pointing up; no gripper reaches the form or the caps, all three loose caps stay on their pads and
the two fitted caps stay seated. The run limit is 2,400 steps, and the demo needs 1,312 of them.

## Reset between trials

Put the two outer caps back on their bars (seated, tip against the bore roof) and the three loose caps on their pads, mouth
down. Nothing is consumed.

## Nearest board tasks

Compared with the accepted and pending tasks on the board on 14 November 2026:
- *Fit a blanking panel at U6 of a server rack*, *Land a ferruled wire in a push-in terminal* and *Seat a strut across a keep-out*
  are placements into a fixed fixture. None has a flared guide, a loose cap on a bar, or a hazard that is left in the scene
  until it is covered.
- *Lock a bayonet cap into its socket* and *Take the cap off a child-resistant medicine bottle* turn a cap; this task presses a
  cap down on a bar, with no rotation.
- No accepted, pending, sent-back or planned task covers reinforcing-bar guarding.
