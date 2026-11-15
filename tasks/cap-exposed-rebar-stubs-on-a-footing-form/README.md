# Cap the bare rebar stubs on a footing form with impalement-rated steel-plate caps

A plywood footing form stands on the table with five vertical #4 reinforcing bars (12.7 mm steel) sticking up out of it in a
row. Two of the bars already wear an impalement-rated steel-plate safety cap. Three are bare. Three more caps stand on grey pads
in front of the bare bars. The instruction is: **"Make the rebar stubs safe for the crew to walk the formwork."**

To do it, the robot has to:
- tell which stubs are still bare (the two outer ones are already capped, and must stay that way);
- pick a loose cap up by its grip boss, carry it clear over the bar tips, and bring its flared mouth down onto a bare bar;
- press it down until the bar is at least 32 mm up the cap's bore and both of the cap's gripping fins hold the bar, so the cap is
  plumb, covers the tip and stays on;
- do that for all three bare stubs, then take both arms away from the form.

## What changed after the first review (TK-H8XM6N)

The first version used a plain plastic cap with a loose fit. The review said such a cap is not shown to give impalement
protection and asked for caps rated for it, with their construction and the manufacturer's fit specified, and the real-world
setup, simulation geometry, fidelity note, scoring and demo updated to match. This version does that:

| Review point | What changed |
|---|---|
| Use caps rated for impalement protection | The cap is now a steel-plate-topped cap of the kind sold as impalement protection (reference part and the rating that applies are in "The cap" below). A plain plastic cap is what OSHA says does **not** protect, and the task no longer uses one |
| Specify construction and manufacturer-required fit | "The cap" and "Real-world build" give the construction (steel plate at least 4 x 4 in, polymer body, gripping fins) and the installation rule (push on until it is firmly seated and centred and the top is level), and a pull test for a secure fit |
| Real-world setup | New cap, new stub spacing (120 mm, so the 4 in plates clear each other), a grip boss and a pull test are in `task.json` and "Real-world build" |
| Simulation geometry | The cap is now an articulated body: a 101.6 mm steel plate and grip boss on a polymer body, with a flared mouth and two spring-loaded gripping fins pressed 1 mm out by the bar. A cap rested on a bar does not push the fins apart |
| Fidelity note | Rewritten: what is modelled (plate, body, boss, fins, mass, friction) and what is not (the drop test, fin flex, the real cap's exact dimensions) |
| Scoring | A cap counts as seated only when the bar is 32 mm up the bore **and** both fins are pushed out by it (the grip), as well as the position and tilt tests |
| Full-score demo | The demo now has to open the fins by pressing each cap on; a cap pressed only part way, or let go above the bar, does not count (see Ablations) |

## Why it matters

Reinforcing bars that stick up out of formwork, footings and slabs are an impalement hazard to anyone who trips or falls onto
them. OSHA's rule for concrete and masonry construction, 29 CFR 1926.701(b)
(https://www.osha.gov/laws-regs/regulations/standardnumber/1926/1926.701, read on 15 November 2026), says: "All protruding
reinforcing steel, onto and into which employees could fall, shall be guarded to eliminate the hazard of impalement." How well a
guard works matters: OSHA's 1997 memorandum on plastic mushroom-style rebar covers (https://www.osha.gov/laws-regs/standardinterpretations/1997-05-29-0)
reports that standard plastic caps gave no protection when sand bags were dropped on them and should not be used where there is an
impalement hazard, and names steel-reinforced covers and wooden troughs as the devices that meet the benchmark: withstanding a
250 lb weight dropped from 10 ft (OSHA's 2014 letter, https://www.osha.gov/laws-regs/standardinterpretations/2014-08-26-0, repeats
that benchmark and says a protective device that eliminates the hazard satisfies the standard). California's rule, 8 CCR section
344.90 (https://www.dir.ca.gov/title8/344_90.html), turns the benchmark into a test: a 250 lb bag of dry sand dropped from 10 ft
(7.5 ft for workers at grade or on the same level as the bar) onto the cover on a #4 bar, and a minimum cover size of 4 x 4 in
(4.5 in if round) for work up to 7.5 ft above grade. So the right guard is a steel-plate cap that is **properly installed and stays
on**, not a loose plastic one. Construction laborers and other site workers place them by hand, and a site is walked by everyone
on it. The instruction does not say what to do or with what: the robot has to see the hazard (bare steel tips among capped ones),
find the caps, and fit each one so that it grips. A cap's fins hold it on only if it is brought down plumb over the tip, within
about 20 mm of the bar (the flared mouth takes the rest), and pressed home hard enough to push the fins apart; a soft arm lands 3 to
6 mm off, and the pinch itself shifts the cap in the hand by up to 17 mm.

## The cap

**The real part.** The reference is the Dayton PC110 "Suprotek" impalement safety cap, listed by a distributor
(https://certifiedmtp.com/dayton-pc110-rebar-safety-caps-3-10mm-to-11-36mm-100-cs/, read on 15 November 2026) as a safety-orange top
over a heavy-duty steel plate, rated for a 250 lb drop at 7.5 ft, citing Cal/OSHA 344.90 and OSHA 1926.701(b), fitting #3 to #11 bar,
and "properly installed and securely fastened". The listing gives Cal/OSHA approval numbers, which this task has not checked against
Cal/OSHA's records, and no dimensions or weight, so a real build should use the manufacturer's data sheet and the cap's approval
label. Plain "OSHA rebar caps" without a steel plate or a drop-test rating (for example the ones listed as for cuts and abrasions at
grade) are not impalement caps and must not be used. A supplier's 4 x 4 in polymer cap with an internal metal insert weighs 0.5 lb
(https://www.ocm-inc.com/product-category/rebar-tools-and-accessories/osha-rebar-cap/) and its instructions are to push the cap on
until the metal insert meets the bar end, centred, with its top square to the bar; the same page says to discard a cap with damaged
fins, because the fins give the grip.

**The simulated cap** (all dimensions chosen here, since the listing gives none):

| Part | Size | Notes |
|---|---|---|
| Steel top plate | 101.6 x 101.6 x 1.5 mm (4 in square) | the guard itself; 4 in is the smallest size 8 CCR 344.90 allows for covers used up to 7.5 ft above grade |
| Polymer body | 64 x 64 x 40 mm, 4 mm roof | blind bore 36 mm deep, 15 mm wide across y (1.15 mm of play either side of the bar); the mouth flares at 45 degrees from 56 mm across down to the bore over 21 mm |
| Gripping fins (two) | 6 mm thick steel blocks, 12 mm tall with a 3 mm chamfer on the lower inner edge | each slides along x on a 4 N/mm spring, 9.5 mm of travel; at rest the gap between them is 10.7 mm, so the 12.7 mm bar pushes each out 1.0 mm and is gripped with about 4 N a side |
| Grip boss | 60 x 60 x 24 mm polymer block on the plate | **an addition to the real cap**, so a two-finger gripper can hold it: the 4 in plate is wider than the gripper's 94 mm opening |
| Mass | 0.23 kg | 0.5 lb; the plate, body, boss and fins are modelled as one articulated body |

With friction of about 0.6 on the bar the two fins hold the cap with about 5 N, against its 2.3 N weight, so a cap that is simply
rested on a bar tip, or pressed only part way, does not seat itself.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: the footing form, the five articulated caps, the fins' spring drives, the monitor, `success()` and `score()` |
| `logic.py` | The scoring (pure Python and NumPy, no Isaac Sim): which bars wear a seated, gripping cap, arms clear, hold |
| `geometry.py` | Every dimension, shared by the task, the assets, the scoring, the demo and the tests |
| `build_assets.py` | Writes `assets/form.usda` (plywood board, five bars, three pads) and `assets/cap.usda` (the cap: base, two fins, joints) |
| `demo.py` | The scripted demo and its ablation modes |
| `demokit.py`, `scenekit.py` | Helpers for scripted demos and USD bodies in a yam_arena task file (template files; `scenekit.py` has one added option, a contact-patch setting for articulated bodies) |
| `check_plan.py` | Offline check of every demo mode against the arms' IK (reach, joint margins) |
| `check_collisions.py` | Offline clearance audit of every demo mode against the scene (the arms' collision hulls against the table, form, bars, pads and caps, and arm against arm) |
| `run.sh` | `./run.sh dream` and `./run.sh demo` (and the ablation modes) |
| `tests/` | Unit tests for the geometry, the scoring, the demo's numbers and task.json |

## Run a trial

    source /data/IsaacLab-Arena/yam.env                    # the Isaac Lab-Arena environment
    cd tasks/cap-exposed-rebar-stubs-on-a-footing-form
    python build_assets.py                                 # only if assets/ is missing (needs Isaac Sim's Python)
    ./run.sh dream                                         # Dream-1 policy, 2,400 steps (expected 0.0)
    ./run.sh demo                                          # scripted demo (expected 1.0, 1,348 steps)

or directly:

    inspect-robots run --embodiment yam_arena --policy dream \
      -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "Make the rebar stubs safe for the crew to walk the formwork." \
      --scorer task_score --max-steps 2400 --store-frames --log-dir out/dream

Runtime: a demo trial is 1,348 control steps (about 67 s of simulated time); wall time depends on the machine's load. There are
no seeds: the layout is fixed and `object_jitter=0`.

## Scripted demo

`python demo.py task.py demo_out` plays the plan, scores it, and writes one MP4 per camera, `setup.png` and `result.json`.
For each bare stub, in order from the right (y = -0.12) to the left (+0.12), the arm on that side (the middle stub is done by the
left arm) pinches the grip boss of the cap standing in front of it, with the fingers closing along x, lifts it 10 mm above the bar tips,
carries it over the stub, lowers it until the flared mouth finds the bar, presses it down (the tool point is commanded 6 mm below
the seat, so the push opens the fins), opens, lifts 15 mm, moves the open fingers back to centre round the boss (one pad is fixed to the hand and still touches it),
lifts away, and goes home. The plan is open loop: fixed waypoints computed from `geometry.py`, plus one number per pick
(`GRASP_DX`): closing the jaws pushes a cap along x until it meets the pad fixed to the hand, which leaves it up to 17 mm off the
tool point, and how far varied from run to run, so the plan moves the tool point 8.5 mm the other way and the 21 mm of flare takes
the rest. It does not read the simulator's object poses while it runs.

## Scoring

The score is the state at the current step; nothing is latched. A cap is **seated** on a bar when:
- the bar tip is at least 32 mm up the cap's 36 mm bore;
- both gripping fins are pushed out at least 0.6 mm by the bar (the cap is gripping it);
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

Knocking a cap off a bar lowers the score again, because that bar is bare again. A cap standing on a bar's tip, lying on its side,
sitting part way down the bar or left on the pad does not count. A cap pressed onto a bar that was already capped does not count twice.

**Real-world scoring**: photograph the form from above and measure with a ruler: a seated cap's steel plate (its top face) is
63.5 to 69.5 mm above the board (bar 60 mm showing, plate 41.5 mm above the cap's mouth, bar 32 to 38 mm up the bore) and a
protractor against the cap reads at most 8 degrees. Then pull each cap straight up with a spring scale on its boss: it must stay on at 5 N (about twice its weight).
Count the caps that pass; the rest is as above.

## Real-world build

- Footing form: a 100 x 600 x 20 mm board of plywood on the table.
- Bars: five 12.7 mm (#4) steel rebar cut-offs, 80 mm long, each pushed 20 mm through a drilled hole in the plywood and
  epoxied (60 mm showing), at 120 mm centres along the long side.
- Caps: five impalement-rated steel-plate caps for #4 bar: the Dayton PC110 or an equivalent with a steel plate, at least 4 x 4 in
  (101.6 mm), a rating of 250 lb dropped from 7.5 ft or more, and a Cal/OSHA approval number on the cap. Check the rating label, not
  the product name. Add a 60 x 60 x 24 mm block of polymer (printed or cut) bonded to the middle of each plate as the grip boss; this
  changes the cap only by adding the boss. Install each of the two outer caps as the manufacturer says: push it on by hand until it is firmly seated and centred
  and its top is level. Spares: two.
- Pads: three 70 x 70 x 4 mm rubber pads at x = 0.175 m, y = -0.12, 0 and +0.12.
- Safety: the bars are cut and filed, the caps are put on by hand before each trial and the form is handled with gloves; no
  pinch point larger than the gripper's. Never drop-test a cap on a person or an unsupported bar; the rating is the manufacturer's.

## Simulation fidelity

- Rigid bodies, 240 Hz physics, 20 Hz control; the arms are the YAM's soft-position-controlled arms.
- The bars are 12.7 mm steel cylinders. Each cap is an articulation: a base made of box colliders (the plate, the boss, a body with a
  blind bore and a 45 degree flared mouth built from tilted boxes) and two gripping fins on prismatic joints with spring drives (4 N/mm,
  damping 20 N s/m, 9.5 mm of travel). The fins' friction on the bar is about 0.6 (the cap's 0.45 averaged with the bar's 0.8).
- **Not modelled:** the drop test (the cap's impalement rating is that of the real part, by specification: the simulator only
  models its shape, mass and grip); the flex of real plastic or steel fins (these are rigid blocks on springs); the real cap's exact
  dimensions (the listing gives none, so the plate thickness, body size and spring rate are modelling choices); the grip boss, which
  the real cap does not have.
- The bars are 60 mm tall, much shorter than the 6 in (152 mm) #4 bar that the test uses; the arms cannot reach a cap carried and
  seated on a taller stub, and a short stub is still a trip-and-fall hazard. The cap, the bar and the plate are real size.
- The scene is one fixed layout; the capped bars are the outer two.

## Ablations

Layout as shipped, one run each, `./run.sh <mode>`; scores are the scorer's final state.

| Mode | What it does | Expected | Measured | Steps |
|---|---|---|---|---|
| `demo` | All three caps pressed on, arms home | 1.00 | 1.00 (run twice, identical) | 1,348 |
| `one_cap` | Only the first cap (stub y = -0.12) | 0.25 | 0.25 | 572 |
| `two_caps` | The first two caps | 0.50 | 0.50 | 984 |
| `half_press` | Each cap pressed only until the bar is 14 mm up the bore, then let go: the fins hold it part way | 0.00 | 0.00 | 1,396 |
| `release_above` | Each cap let go with its mouth 6 mm above the bar tip, with no press: it is not pressed on | 0.00 | 0.00 | 1,396 |
| `off_target` | The demo with the hand shifted 40 mm along x, so each cap arrives 28 to 52 mm off its bar, beyond the 21 mm the flared mouth can catch | 0.00 | 0.00 | 1,396 |
| `stay` | All three caps, then the right arm waits over the form | 0.75 | 0.75 | 1,346 |
| `settle` | No motion | 0.00 | 0.00 | 280 |
| `dream` | Dream-1 baseline policy | 0.00 | 0.00 | 2,400 |

All measured values are the final scene and plan, one run each except the demo. `half_press` and `release_above` are the ablations that
test the secure fit: a cap that is not pushed all the way on, or merely rested on the bar, does not count.

### Dream-1 recording

The Dream-1 run (final scene, 2,400-step limit) completed all 2,400 steps with no errors and scored 0.0. Its record shows 50
requests to the Dream service, every one answered (some after a retry), and the arms tracked the commanded rows to within 0.10 rad
at most. In the video the arms rise from their ready pose and move about near their bases; late in the run the left arm reaches
out over the left end of the form. No cap is lifted or moved: in the last frame all three loose caps are still on their pads and the
two fitted caps are still on their bars. The demo needs 1,348 of the 2,400 steps.

## Reset between trials

Put the two outer caps back on their bars (seated, tip against the bore roof, fins gripping) and the three loose caps on their
pads, mouth down. Nothing is consumed.

## Nearest board tasks

Compared with the accepted and pending tasks on the board on 14 November 2026:
- *Fit a blanking panel at U6 of a server rack*, *Land a ferruled wire in a push-in terminal* and *Seat a strut across a keep-out*
  are placements into a fixed fixture. None has a loose guard that must grip a bar, a flared guide, or a hazard that is left in the
  scene until it is covered.
- *Lock a bayonet cap into its socket* and *Take the cap off a child-resistant medicine bottle* turn a cap; this task presses a cap on
  a bar until its fins grip, with no rotation.
- No accepted, pending, sent-back or planned task covers reinforcing-bar guarding.
