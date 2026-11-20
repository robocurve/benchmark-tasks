# Prepare an infant car seat left latched on one side of its base in a car's back seat

A rear-facing infant car-seat base is installed in the centre seat of a car's back bench, its front pushed against
the seat back. An empty infant carrier sits on it and looks seated, but only the inboard end of its locking bar is
locked. The door-side end rests on top of its lock's pawl, about 1.4 cm high, and the carrier is rolled 2.4 degrees.
The robot stands at the open rear door and reaches in across the door-side seat. The instruction is only: **Prepare
the infant car seat for use.** The robot has to notice that one side is not locked and push that front corner down
until the pawl clicks over the bar. Then it has to let go and leave the carrier locked on both sides, level and
untouched.

## Response to review

The previous version was not accepted on construct validity, scoring, and clarity and reproducibility. Each point,
and what changed:

1. **"The fixed base on an open table omits the vehicle seat and cabin constraints."** The base now sits on a car's
   rear bench, rear-facing, in the position its manual gives: "Place the base flat on the vehicle seat. Push it back
   until the front of the base touches the vehicle seat back" (manual p. 27; base 100% on the cushion, against the
   manual's 80% minimum). The cabin is modelled around it:
   - the rear cushion (50 cm deep, sloping 5 degrees down towards the seat back) and the reclined seat back, which
     sits 5 to 9 cm behind the carrier's foot-end corners;
   - the two front seats' backs, reclined towards the carrier, and the console between them, beyond its head end;
   - the door opening's two pillars, either side of the arms.

   The carrier lies along the car, and the robot works from the side door, as a parent does. **What differs from
   the most common real case, and why:**
   - **Centre seat instead of the door-side seat.** The arms always start at a fixed ready pose, with both grippers
     33 cm in front of their bases and 26 cm up. A carrier in the door-side outboard seat would enclose one of them.
     Placed far enough in to clear them, its far lock is about 70 cm out, beyond the arms' reach (checked with the
     simulator's inverse kinematics). The centre seat is the outboard-most position that clears the ready pose (6 cm)
     and keeps the door-side lock in reach. The manual allows the centre seat where the vehicle's manual does ("the
     centre of the back seat can be the safest during a possible side impact", p. 21), and reaching across the
     outboard seat to it is ordinary.
   - **The faulty lock is the door-side one.** It is the one in reach from the door. The inboard lock is 73.5 cm out,
     beyond reach, so the inboard-side version of the fault could not be demonstrated.
   - **No roof.** A roof about 90 cm above the cushion would hide the overhead camera. In the demo the arm stays below
     40 cm above the cushion, so the roof would not constrain it.
   - **Footwell floor too high.** The table top is the footwell floor, only 3 cm below the cushion instead of about
     30 cm. Nothing in the task uses the footwell except the two distractors.
2. **"The scorer retains full credit after its three-second success hold even if the robot subsequently tilts or
   holds the carrier."** The full-credit latch is gone, and nothing ends the trial early. `success()` always returns
   false, so every run goes to its step limit, and the result is the score at the last step (`SCORE_OVER = "end"`).
   Full credit needs every ready condition to have held without a break over the last 3 seconds of the run. Any step
   that is not ready restarts the count. A carrier that is ready and then pressed on, tilted, lifted or held scores
   what its final state earns. A new red-team variant does exactly this: it latches the carrier, lets go for more
   than 3 seconds, then comes back and presses on it until the end. It scores 0.5, not 1.0 (see Validation). The
   hands-off check is now a distance from the carrier's actual surfaces rather than its bounding box, which took in
   the open seat under the carry handle.
3. **"The real-trial instructions do not specify which actual carrier and base to use or establish that their
   half-latched state and runner checks match."** The real trial now names the product and the vehicle position. It
   gives a step-by-step way to stage the one-sided latch on that product, with a check that it is staged. It also
   gives a measurement for each lock separately, using the manual's own pull-up check, a luggage scale and a steel
   rule (see Real-world trial). The simulated rear runners and pads are not used in the real measurement: on the real
   base they are hidden. They are replaced by an inclinometer reading against a reference taken on the same carrier
   when correctly latched, and by a corner-height comparison.
4. **"Provide an updated full-score demo."** The demo was rebuilt for the new layout and scored 1.0 twice (see
   Validation). It now holds the ready state for the last 11 seconds of the run. The video and screenshot are
   re-uploaded from this version.

## Run and score

Use the Python environment that has `yam_arena` and `inspect-robots`. From the repository root:

```bash
# scripted demo
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/prepare-infant-carrier-on-base/task.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "Prepare the infant car seat for use." \
  --scorer task_score --max-steps 400 --store-frames \
  --log-dir ~/yam-eval/prepare-infant-carrier-on-base/demo

# model trial
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/prepare-infant-carrier-on-base/task.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Prepare the infant car seat for use." \
  --scorer task_score --max-steps 2400 --store-frames \
  --log-dir ~/yam-eval/prepare-infant-carrier-on-base/dream
```

The trial prints `task_score`. The layout is fixed (`object_jitter=0`) and nothing depends on the seed. The demo is
a fixed open-loop plan from the published starting layout; it reads no simulator or scorer state. The scripted
policy stops when its plan ends, after 15.5 simulated seconds (about 310 control steps plus the policy's own tail).
Its final score is read then, after the carrier has been ready and untouched for the last 11 seconds. The model
limit is 2,400 steps (2 minutes at 20 Hz); a model trial always runs to its limit unless the policy stops itself.
A trial takes about 1 to 2 minutes of wall time once the GPU's shaders are compiled.

## The scene

Positions are in centimetres. x is forward from the arm bases into the car through the open rear door, y is to the
arms' left (towards the car's front), and heights are above the table top.

- **Rear bench**, fixed: cushion x 10 to 118, 50 cm deep from the seat back (y -36) to y 14. Its top is 3 cm up
  at the back, rising 5 degrees. The seat back is 14 cm thick and reclined 20 degrees. Seams at x 37 and 79 mark the
  door-side, centre and far seating positions.
- **Front seats**, fixed: backs at x 0 to 44 and 74 to 118, 12 cm thick. Their rear faces are at y 50, 20 cm up,
  reclined 20 degrees towards the carrier. Between them is a console, x 46 to 72, rear face at y 48, 32 cm high.
  The front seats are set so that their backs clear the carrier's head end by about 8 cm.
- **Door pillars**: 6 x 8 x 90 cm, at x -6, y -47 and 57.
- **Base**: rear-facing in the centre seat, centred on x 58, from y -36 (against the seat back) to 9. It is fixed,
  as an installed base belted to under 2.5 cm of movement.
- **Carrier**: foot end towards the seat back. The locking bar's axis is at y -33.5; its locks are at x 42.5 (door
  side) and 73.5.
- **Distractors**: on the footwell floor (see Assets).

## The mechanism

The base has a lock at each end of the carrier's locking bar, 31 cm apart. Each lock is a pocket (floor, a guide
wall on the bar's front side, a rear block) with a sprung pawl above it on a slider joint. The pawl's face towards
the bar is a 60-degree ramp; its underside is flat, 2 mm above the seated bar. Pushing the bar down the ramp cams the
pawl back (16 mm travel). Once the bar is past, the spring snaps the pawl forward over the bar. Lifting the bar then
presses on the flat underside, which the slider's spring does not oppose, so a locked side holds.

Each pawl spring is 800 N/m with 8 N of preload against its stop, so the bar goes in with roughly 20 N pushed down
at the corner. The 3.45 kg carrier's own weight does not push it in. These forces are estimates, not measurements of
the product; the real trial measures it (see below).

During the 0.5 s settle before the first frame, the pawl springs are held stiff, so the carrier comes to rest on the
door-side ramp. At reset (mechkit's `on_reset` hook) they are switched to the 800 N/m, 8 N preload spring used for
the whole trial. The first frame shows the carrier at rest, with the door-side pawl pushed back 5.3 mm by the bar.

The carrier is rebuilt from primitives to the size and weight of the Graco SnugRide 35 Lite LX: a 70 cm long
plastic shell with walls, a reclined fabric seat pad, a harness and buckle, and an upright carry handle, which the
manual requires in the vehicle. Two runners under its head end rest on pads on the base.

## Scoring

`Trial.step()` runs once per control step (mechkit `on_step`); `score()` only reads it. `success()` always returns
false, so the trial is never cut short, and the result is the score at the last step.

- **1.0:** at the end, all of the following, held without a break over the last 3 seconds of the run:
  - both sides latched: each bar end within 3 mm of seated height and within 8 mm of its lock centre front to back,
    and each pawl within 1.5 mm of fully forward;
  - all four rear runner points within 3 mm of their pads;
  - carrier tilt at most 1 degree;
  - carrier speed under 5 mm/s and spin under 0.05 rad/s;
  - every sampled fingertip point (mechkit `fingertips`, both arms) more than 3 cm from the carrier's surface,
    sampled at 1.2 cm or closer.
- **0.5:** both sides latched at the end, but the full condition not held over the last 3 seconds. Examples: still
  pressing on it, or moving it after it was ready.
- **0:** anything else, including the starting state.
- **Harm:** a carrier that ever tilts more than 25 degrees, or either of whose bar ends drops more than 5 cm below
  its seated height (off the base), scores 0 for the rest of the trial. This record is cleared only at reset (mechkit
  `on_reset`), never because the carrier looks right again.

Lifting the carrier off and setting it back down locked is allowed; only the final state is scored.

## Real-world trial

**Equipment:**
- A Graco SnugRide 35 Lite LX infant car seat with its matching SnugRide 35 Lite LX base (Click Connect). The
  carrier is sold with one base; extra bases are sold separately.
- A car whose owner's manual allows a child restraint in the rear centre seat.
- A luggage scale reading to 1 N (or 0.1 kg).
- A 30 cm steel rule.
- A phone inclinometer app.

**Installation:**
1. Install the base in the rear centre seat, rear-facing, with the vehicle belt, as the manual's section 3 says. Push
   it back until its front touches the seat back. Tighten it until it moves less than 2.5 cm at the belt path. Set
   the recline foot until the level indicator is in the 0+ months range.
2. Slide the front seats so that their backs clear the carrier's head end by 6 to 10 cm.
3. Open the rear door on the side the robot works from. Place the robot at the door, with its arm bases about 58 cm
   from the base's centreline: inside the door line, so it reaches across the outboard seat. Nothing sits on the
   outboard seat. Put a 0.5 L water bottle and a paper coffee cup on the footwell floor in front of it.

**Reference** (once per carrier and base):
1. Set the carrier in the base and push down on the front until both sides click.
2. Pull up on each front corner with the luggage scale hooked under the shell's corner lip, to 50 N. Neither may rise
   more than 3 mm against the steel rule. This is the manual's own check ("Pull up on the front corners of the seat
   to make sure it is locked", p. 18).
3. Record each front corner's rim height above the base's top edge with the rule (H_door, H_in).
4. Record the inclinometer reading on the shell's rim at the carry-handle hubs, along and across the carrier
   (A_ref, C_ref).
5. Measure the latch force once: stage the fault (below), hook the luggage scale's strap over the door-side corner
   lip from below the base, and pull down slowly until it clicks. Record the peak; the simulation assumes about 20 N.

**Preparation (staging the one-sided latch):**
1. Squeeze the base release handle at the back of the carrier and lift the carrier off (manual p. 17).
2. Holding the carry handle, tilt the carrier so its door-side front corner is about 3 cm higher than the inboard
   one. Lower the inboard end of the bar into its lock and press the inboard front corner down until it clicks.
3. Lower the door-side corner until the bar's end rests on top of its lock, without pushing, and let go.
4. Check the staged state; all three must hold:
   - the door-side corner's rim is 10 to 20 mm above H_door;
   - the door-side corner lifts by hand with under 10 N;
   - the inboard corner pulled up with 50 N rises no more than 3 mm.

   If the door-side side clicks under the carrier's weight alone, repeat from step 1. If it always does on that
   sample, the fault cannot be staged on it: record that and do not run the trial. The manual's instruction ("Push
   down on the front of the car seat. You will hear a click when it is locked") implies that setting the carrier
   down does not lock it by itself.

**Measurement after the trial** (the state at the end of the run limit, or when the robot stops):
- Hands-off and stillness over the last 3 seconds are read from the video: no part of either gripper within 3 cm of
  the carrier, and no visible movement.
- **Each lock is checked separately**, door side first. A side counts as latched when:
  - its corner rim is within 3 mm of its reference height (H_door or H_in);
  - pulling straight up on that corner lip with 50 N on the luggage scale raises it by no more than 3 mm against
    the rule.
- **Seated and level:** both inclinometer readings are within 1 degree of A_ref and C_ref.
- **Score:**
  - 1.0 if both sides are latched, the carrier is seated and level, and the hands-off and stillness conditions hold;
  - 0.5 if both sides are latched but the rest does not hold;
  - 0 otherwise;
  - 0 if the carrier was ever tipped more than 25 degrees or came off the base (from the video).

The pawl position the simulation reads is hidden inside a real base; the pull test stands in for it. The rear
runners and pads are hidden too; the inclinometer stands in for them.

**Reset:** squeeze the release handle, lift the carrier off, and stage it again as above. Nothing is consumed.

**Safety:** the trial uses an empty carrier, never an infant. The robot pushes with about 20 N.

## Validation

All runs are of this version (centre seat, final-state scoring).

| Run | File | Score |
|---|---|---:|
| demo 1 | `task.py` | 1.0 (371 steps; ready for the last 11.3 s) |
| demo 2 | `task.py` | 1.0 (371 steps) |
| do nothing to the carrier | red-team variant | 0.0 |
| press and never let go | red-team variant | 0.5 |
| latch, let go for over 3 s, then press on it again until the end | red-team variant | 0.5 |

The red-team variants were copies of `task.py` with a different `demo()`, deleted after the runs. Under
the previous scorer, which latched full credit after the first 3-second hold, the last variant would have scored 1.0.

In its one 2,400-step run in this scene, Dream-1 scored 0.0. It drew both grippers back towards the door and held them
there without touching the carrier, so the door-side end of the bar stayed on its pawl's ramp.

## Importance and sources

- The [Graco SnugRide 35 Lite LX owner's manual](https://www.gracobaby.ca/on/demandware.static/-/Sites-gracoca-master/en_CA/dw5f1dde52/ProductManualPdf/NWL0001004345B%20SnugRide35%20Lite%20LX%208BS04%20ENG.pdf)
  is the source for the mechanism and installation:
  - Section 2-C, "Attaching Car Seat to Base" (p. 18), describes the mechanism: "Align the locking bar at the front of
    the car seat with the locks in the base. Set car seat in the base. Push down on the front of the car seat. You
    will hear a 'click' when it is locked. Pull up on the front corners of the seat to make sure it is locked."
  - Its checklist ("Did You Do Everything Correctly?") lists "Car Seat Is Properly Attached To Base".
  - Section 2-B (p. 17) gives the removal: "From the back of the car seat, squeeze the base release handle and lift
    the car seat out of the base."
  - The base's placement (p. 27): "Place the base flat on the vehicle seat. Push it back until the front of the base
    touches the vehicle seat back", with "a minimum of 80% on vehicle seat". The centre seat (p. 21): "The centre of
    the back seat can be the safest during a possible side impact."
- The carrier's weight is the carrier-only weight measured in [BabyGearLab's review of the SnugRide Lite LX](https://www.babygearlab.com/reviews/vehicle-safety/infant-car-seat/graco-snugride-lite-lx),
  7.6 lb. Its 70 x 38 x 47 cm envelope follows the overall dimensions on its
  [Target listing](https://www.target.com/p/-/A-79267447), 27.4 x 15.5 x 18.07 in. A Graco answer on the same page
  gives the width as 14.41 in.
- NHTSA's [National Child Restraint Use Special Study](https://crashstats.nhtsa.dot.gov/Api/Public/ViewPublication/812157)
  (DOT HS 812 157, 2015) estimated that 46 percent of car seats and boosters were misused, and 49 percent of
  rear-facing infant seats. That figure covers all kinds of misuse, not carriers left half-latched in particular.

The simulated carrier and base are a reconstruction of this product's layout and size, not a CAD replica: the base's
footprint and the lock positions are estimated, not measured. No product force was measured; the real trial's
reference step measures the latch force.

## Assets and helpers

The carrier, base and cabin are authored structural and mechanical parts (`mechkit.py`). The two distractors are
from the Hugging Face dataset [`nvidia/simready-assets`](https://huggingface.co/datasets/nvidia/simready-assets),
pinned to revision `9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd`. They are downloaded at first run:

- [`food_beverage_bottledWater_b01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/food_beverage_bottledWater_b01):
  a 0.5 L plastic water bottle, 6.8 x 24.5 cm, at (15, 21) cm; mass set to 0.52 kg.
- [`food_beverage_coffeeCup_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/food_beverage_coffeeCup_a01):
  a paper coffee cup, 8.5 x 11.7 cm, at (26, 23) cm; mass set to 0.25 kg.

They are © NVIDIA, licensed [CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/). They sit loose on the
footwell floor and scoring ignores them. Changes at load time: masses set as above, joints (if any) welded into one
rigid body. Nothing from the dataset is committed here.

`simready.py` is the open-source loader from `tasks/cutters-off-pegboard`. `mechkit.py` is the shared mechanism kit
from `tasks/heat-shrink-lug`, unchanged. The hands-off check uses SciPy's `cKDTree` when SciPy is installed, and
plain NumPy otherwise.
