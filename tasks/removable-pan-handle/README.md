# Attach a removable saucepan handle and use it

> Attach the detachable handle to the saucepan and use it to move the pan onto the trivet.

The physical skill is assembling a load-bearing connection and using it: one
arm steadies the pan while the other seats the jaw and closes the handle. The
script then releases the closing lever, lifts through the housing and transfers
the pan. Both arms use their standard parallel grippers. Swapping arm roles,
regrasping and other valid routes are allowed.

The pan is cold and empty. This is a familiar household assembly task, not a
claim about handling hot cookware or preventing injury. There is no proximity
snap, grasp weld, pan-to-handle constraint, or policy command to a mechanism joint.

## Setup

The reference pair is the 18 cm stainless-steel saucepan and compatible handle
supplied in the Tefal Ingenio Emotion L9254S14 set. Use the compatible retail pair
for a physical reproduction, not a generic clamp on an unrelated pot. The
simulation is an original, estimated reconstruction; detailed product dimensions,
masses and force curves have not been measured.

Coordinates are metres relative to the tabletop: X forward from the midpoint
between arm bases, Y left, Z above the tabletop (world Z = 0.7475 m).

| Object | Size and material | Start |
|---|---|---|
| Empty saucepan | Stainless steel; nominal 180 mm diameter, estimated 196 mm outer lip, 95 mm high, 650 g | Base centre (0.30, 0.16, 0), upright |
| Open detachable handle | Black polymer housing and lever, metal jaw/linkage; approximately 182 mm long, 55 mm across the release controls, 261 g; housing width 36 mm | Grip body approximately X=0.10, Y=-0.06; on its side, jaw toward +X; settled front datum 21.5 mm above table |
| Cork trivet | Round, 240 mm diameter, 12 mm thick; thin double-sided tape secures it to the table | Bottom centre (0.30, -0.17, 0) |
| Robot | Standard bimanual YAM with parallel grippers, calibrated top and wrist cameras | Ready posture, grippers open |

The screenshot shows the actual settled reset. The pan and handle are free;
only the taped trivet is fixed. There is no stove, lid, food, liquid or heat.
Reset by returning the empty pan, detached open handle and trivet to this layout.
No consumables are needed. The handle's exact simulation transforms and passive
joint settings are in `geometry.py` and `task.py`.

## Score

One primary score, 0–1. Budget: **1,400 actions at 20 Hz (70 simulated seconds)**.

| Score | Required outcome |
|---|---|
| 0 | No retained attachment proof, a slip without a successful retry, or an invalid/fallen apparatus |
| 0.5 | Pan base held at least 20 mm above the table for 1 continuous second, supported solely through the mechanically closed handle; attachment remains retained |
| 1.0 | After that proof, continuous handle-supported carry from the source to the trivet, then upright placement within 20 mm of its centre and release from both robots for 2 continuous seconds |

The proof and final hold require pan tilt at most 5°, observed translation speed
at most 20 mm/s and observed angular speed at most 0.1 rad/s. Final placement
requires pan-base height 12±2 mm above the tabletop, actual trivet support and
no robot contact with any part of the pan or handle. The handle must meet the
same motion limits during the final hold.

Bracing the pan during attachment is allowed. During the proof and carry, the
pan may contact only the intended jaw and fixed gripping faces; grippers must
hold the housing. Support by a pan-grasping hand, moving-jaw grasp, squeezed
lever, release button, housing used as a shelf, table or forearm is ineligible.
Opposed fingertip contacts establish the housing grasp. The closing lever must
remain within 2° of its closed limit without external support. Pan-to-handle
motion must remain within 2 mm / 2° of the established connection.

Carry must start with the pan on the table within 30 mm of its source centre;
connection must continue until trivet support. Direct pan transfer, throwing it
onto the trivet, or bringing a loose handle near it cannot establish full credit.
Setting the pan down elsewhere interrupts delivery; returning it and retrying is
allowed. A slip cancels the lift proof. Half credit is retained only with the
connection intact and the pan supported through the handle or resting on the
table/trivet. Detaching the handle safely after a completed placement is allowed;
dropping it is not. Either hand may take either role.

All 480 Hz physics poses and contacts are checked. Translation and rotation are
summed over each 50 ms interval, including back-and-forth motion, and divided by
elapsed time. This defines **observed motion**. Engine-reported velocities are
retained separately and do not determine the score. Any recorded apparatus body
falling more than 100 mm below tabletop height or a loaded contact penetrating
more than 2 mm invalidates the simulation result and gives zero. Missing or
incomplete contact records invalidate evaluation rather than earning credit.

For a physical reproduction, use a ruler and continuous video to check the lift,
load path, retention, target placement and released stability. The contact
telemetry implements those observable conditions; it does not add a force-sensor
requirement. Material damage in a physical trial is a failure; rigid simulation
bodies do not model fracture.

## Fidelity and evidence limits

The hollow pan uses separate convex wall/lip segments. The handle has a passive
jaw, lower over-centre lever, compliant linkage and paired rocker release controls.
The robot actuates these through contact. Product manuals and SEB patents inform
the topology, but do not verify the selected retail revision's internal dimensions.
The reconstruction uses a 20 kN/m linkage spring and a 1.2 kN/m jaw return spring;
these, component masses, friction and collision shapes remain estimates.

**The engine-angular-velocity discrepancy remains unresolved.** In the scripted
transfer's loaded hold, median engine angular speed is about 0.536 rad/s, while
summed pose motion gives about 0.026 rad/s. Two engine velocity APIs agree, so this
is not explained by a stale API cache. The original engine-speed gate fails; the
submitted score uses observed pose motion. A separate solver-iteration comparison
did not resolve the discrepancy, and its settings are not used here. This
limitation is disclosed rather than presented as a validated physical product.

The scripted native transfer completes in 1,348 actions / 67.4 simulated seconds
and scores 1.0 under this pose-based rule. Explicit replay with the current
source-eligibility fix preserves every original live score. Real negative trials
with moving-jaw support (862 actions) and direct pan support (1,018 actions) score
zero with exact live/replay agreement. The jaw-support case is not a lever-only
support test. Additional release, slip and parameter-sensitivity checks remain
unvalidated. No physical robot trial has been run.

The scripted controller uses privileged body pose, lever angle and finger state;
it commands only robot joints/grippers. Dream-1 receives camera images and robot
joint state, never the mechanism state or scorer milestones. Dream-1 beta completed all 1,400 actions and scored 0.0 (29 successful model
requests, 57 attempts including session-expiry retries; no errored trial). The
model moved the arms but did not establish an attachment proof. Actual scores
and the uploaded media identifier are recorded in `task.json`.

## Run and score

Use the native Isaac environment from the challenge walkthrough. Recorded
versions: Python 3.12, Isaac Sim 6.1.0.0, Isaac Lab 17.0.2, IsaacLab-Arena 0.3.0,
yam-arena 0.1.0. The launcher verifies hostname `visnu`; this is the host used
for these recordings. The current launcher deliberately refuses another host
rather than attributing unverified results to it.

The shared runtime and scripted trajectory helper are pinned below. Scene,
mechanism, scoring and Dream entry point are supplied in this task folder. Use a
fresh checkout/output for each run; do not edit a running source checkout.

```bash
# Run from the benchmark-tasks repository root, in the native Python environment.
submission_task=$(realpath tasks/removable-pan-handle)
git clone https://github.com/spindro/resist.git /tmp/pan-handle-runtime
git -C /tmp/pan-handle-runtime checkout 49832b12cf8ed1a75a3721130258899bcd161cf1
cp "$submission_task/"*.py /tmp/pan-handle-runtime/tasks/removable-pan-handle/
cp "$submission_task/pilot.json" "$submission_task/trajectory.json" /tmp/pan-handle-runtime/tasks/removable-pan-handle/
cd /tmp/pan-handle-runtime
export PYTHONPATH="$PWD"

# Scripted full-score demo; seed 0 is the default.
python tools/run_recovery_pilot.py tasks/removable-pan-handle /tmp/pan-demo-new \
  --policy deliver_pan --device cuda:1 --seed 0
python tasks/removable-pan-handle/validate_scoring.py /tmp/pan-demo-new --expected 1

# Actual hosted Dream-1; API key configured as in the official walkthrough.
python tasks/removable-pan-handle/run_dream.py /tmp/pan-dream-new --device cuda:1
```

The Dream runner uses Inspect Robots with the CLI's default action guards and
stores a JSON evaluation, actions, all camera frames, videos, native contacts,
source snapshots and receipt. `validate_scoring.py OUTPUT --expected SCORE`
checks the completed recording's hashes and live/replay agreement; it is a
read-only replay, not a substitute for native simulation. An intentional evaluator
revision requires explicit `--rescore`, which reports both hashes and all changed
steps without rewriting the original receipt.

A scripted trial took about 20.6 minutes of wall time on a shared RTX 3090;
Dream additionally waits for hosted inference. Different GPU load changes wall
time, not the 70-second simulated budget. Camera views and physics are identical
between policies; the Dream client resizes images internally for its model.

## Assets and licenses

- **Saucepan, handle, trivet and task code:** original parametric reconstruction,
  MIT (see `LICENSE`). No third-party product CAD, photographs, textures or patent
  illustrations are bundled.
- **I2RT YAM robot model and meshes:** external dependency from
  [I2RT Robotics](https://github.com/i2rt-robotics/i2rt), MIT; upstream asset commit
  `120c3c81400171174604e503943f8d1ebc891058`. The installed assets contain
  `LICENSE.i2rt` and `SOURCE_COMMIT`. This asset license does not license unrelated
  simulator integration code.
- **Product/mechanism references:** the official Tefal set listing, user manuals
  and SEB patent WO2017006039A1 are linked individually in `SOURCES.json`.
  They support the mechanism description, not rights to redistribute their
  images or a claim of measured geometric fidelity.
