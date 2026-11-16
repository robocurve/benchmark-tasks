# Pack a carton and seal it using an ordinary tape roll

**Incomplete feedback draft:** see [DRAFT_STATUS.md](DRAFT_STATUS.md) for current measured evidence and missing work. No full-score demo or competition-ready submission is claimed.

Native mechanics draft. Forty local material/contact/scorer checks pass (NumPy required for native-geometry unit tests). Native ribbon/roll bench probes exist, but none yet validates the full task: **no robot packing/sealing demonstration or model score exists.** The tape ledger alone is accounting, not physics.

Pack two objects into an open carton, fold its four top flaps in the correct
order, find and peel the end of an ordinary roll of packing tape, pull out a
strip, press it across the seam and down both end faces, cut it with the
guarded cutter and park the roll/tool. Leave a sealed carton after releasing
both hands. No pre-laid strips or tool-path-triggered deposition.

## Ordinary tape form factor

Provisional design is a **loose roll**, 48 mm wide, 50 µm film-plus-adhesive,
100 mm outside diameter and 76.2 mm cardboard core diameter. The core is open,
so a robot can potentially grasp its rim rather than needing an invented
handle. Starting end is flush on the outer wrap: **no prepared pull tab or
protruding leader**. Extracting that edge with the YAM fingertips is a major
unproven physical challenge, not a step we assume succeeds automatically.

`model.py` tracks finite natural-length payout, shrinking roll radius and
verified blade severing. The dimensions imply about 65.9 m of tape; the
native implementation need only represent the active wound layer/ribbon
while conserving that supply. A 220 mm seam and two 40 mm wraps need at least
300 mm of connected material. Maximum permitted strain is 5%.

`adhesion.py` is a candidate local contact/cohesion law: only the adhesive
face can bond, within 0.5 mm of a surface under at least 0.5 N compression for
100 ms. Each 5 × 48 mm material cell has a provisional 2,000 N/m cohesive
spring and 2 N break load; these are **uncalibrated design values**. An adapter
must apply equal/opposite forces at physics substeps, preserve material
identity, model contact/slip and peel, and derive adhesion from actual ribbon
contact. It must not hold flaps shut by changing their poses or snapping
closed merely because a roller/finger passed over the seam.

The requested realism is therefore a required mechanics development step,
not a claim that this first draft already achieves it. If the fingertips
cannot peel a flush tape edge, we must report that and choose a justified
ordinary tool with the user rather than secretly adding a convenient tab.

## Carton and contents

Provisional carton: 220 × 180 × 100 mm, 3 mm corrugated board, ordinary four
top flaps; bottom factory-sealed. Objects are two rigid products, 80 × 50 ×
30 mm and 60 × 40 × 30 mm, initially outside the box. Native masses, materials,
starting positions, flap interactions and safe guarded cutter integration
remain to be frozen after reach/contact checks. No artificial handles.

## Proposed end-state scoring

- 0.25: both products wholly inside, at least 3 mm clear of walls and below
  the top flaps, product speeds below 20 mm/s. Full bounding boxes are checked.
- 0.50: above, all four top flaps closed within 5 degrees, short flaps beneath
  long flaps.
- 0.75: above, one severed connected tape strip bonds at least 80% of its
  width across **every** 5 mm seam cell, with at least 35 mm bonded on both
  40 mm end wraps. It is detached from the roll.
- 1.00: above, both hands clear, roll/cutter parked, carton speed below
  10 mm/s, held for two seconds after release. Native flap restoring forces
  remain active; adhesive forces must hold the seal without the robot.

Invalid material accounting, over 5% strain, carton joints separated by over
4 mm, dropped objects, nonfinite state or missing samples invalidates the
trial. Restoring forces and measured bond forces must be recorded for the
release hold. No external opening-test fixture is part of the task. Cosmetic
tape patches and tool paths do not establish a bond.

Proposed allowance is 4,800 controls (four minutes at 20 Hz), subject to a real
full-score demonstration. This task is harder than the tape-gun-only #1893 in
both product/flap preparation and actual tape sourcing/adhesion; this does not
guarantee novelty acceptance. Recheck the board before eventual submission.

```sh
python3 -m unittest discover -s tasks/pack-and-tape-carton -p 'test_*.py' -v
```

Next: hollow-roll/native ribbon representation, flush-edge peel probe with real
gripper geometry, winding/payout mechanics, force-coupled adhesion/peel,
scissor/cutter interaction, native packed-carton flap scene, then a robot
controller and complete validation. The native `run.py` currently runs only a material bench; do not describe synthetic scoring states as a physically feasible robot trajectory.

## Current native material bench

```bash
/data/IsaacLab-Arena/.venv/bin/python run.py --out /path/to/new-bench
```

The hollow roll has 72 connected 5 mm film cells explicitly wound on its outer
layer. Native breakable tack and width-axis bend joints must physically free
material; there is no prepared leader or tool-path deposition. The bench clamps
the roll and drives its end for characterisation only. Those fixtures are not
part of a scored robot task.

Require the coil to stay wound before loading. The first apparent peel pass
failed this negative control; transform ordering and initial mat overlap were
fixed. Native003 had 14.5 um rest drift, but its 432 um joint error exceeded
the 250 um/5% strain limit. Native004 is being tested at 2 kHz without mass
scaling or relaxed criteria. `contact_native.py` creates finite-strength passive
bonds only after measured contact pressure, geometric adhesive-face overlap
and dwell. The new substep sensor adapter is implemented and remains natively
unvalidated; the earlier bench result does not validate this changed adapter.
No body or film pose is ever set by a policy. Full robot peeling, product/flap
manipulation, cutting, force-supported adhesion and released retention remain
required before this folder can claim task completion.


## Native contact integration (implemented; runtime validation pending)

`contact_runtime.py` adds one multi-body contact sensor for all 72 film roots,
with ordered filters for the twelve carton-panel roots and cutter root. It
recreates annotated scene configuration fields, then reads `force_matrix_w`
after the native `scene.update` at **every physics substep**. It rejects control
period or batched updates and unexpected sensor body names/shapes. Native contact
processing is explicitly enabled when that setting exists in the installed
simulation configuration. Sensor body order is read from `body_names`, rather
than assuming lexicographic tape order.

Adhesion requires the geometric adhesive face, at least 0.5 N measured normal
compression and 100 ms of uninterrupted contact. The native approximation is a
passive fixed joint with 2 N break force and 0.008 N m break torque scaled by
contacted width; it does not use the CPU reference spring law. No policy can
create a bond or move a material/body pose. Current measured anchor separation
and footprint determine which created bonds still count. Separation above 1 mm
permanently disables the bond. The pinned runtime did not emit reliable wound
joint break events: a broken but still coincident unloaded adhesive constraint
cannot be identified by this geometric check. Native pressure, peel and released
retention probes are required before claiming adhesive mechanics are validated.

`seal_geometry.py` measures coverage from current adhesive patches on one
connected material component: twelve width bins and four longitudinal samples
per 5 mm seam/end-wrap cell. Historical tool travel contributes no coverage.
Cutting needs a measured film/cutter contact force and relative blade crossings
covering every width bin of a material hinge. The actual measured 14 x 0.5 x 8 mm
blade collider bounds are used, so a handle contact alone cannot cut. A complete
crossing disables that native material hinge; all other hinge constraints stay
passive. Cut coordinates follow the finite outer strip's material order. Native
joint creation/disable propagation, sensor force direction, filter mapping and
blade interaction have **not** been tested in Isaac by these CPU tests.

`run.py --mode bench` retains its explicit roll clamp/peel drive and records
contact telemetry without activating carton adhesion or cutting. It cannot earn
robot task credit. `--mode products-probe` contains no bench fixture and enables
the measured adapter, but its controller only attempts product packing; it is
not a complete packing/sealing demonstration. Both modes save substep counts,
measured contact-force samples/maximum, active bonds and measured cut hinges.
The product-only mode retains 240 Hz physics; any later controller loading or
peeling the thin film must use the bench-validated 2 kHz setting.

Local checks:

```bash
/private/tmp/robocurve-media-env/bin/python -m unittest discover -s tasks/pack-and-tape-carton -p 'test_*.py' -v
python3 -m compileall -q tasks/pack-and-tape-carton
```

Native commands to validate the new integration (not executed here; use the
actual installed Python path and a fresh output directory):

```bash
/data/IsaacLab-Arena/.venv/bin/python tasks/pack-and-tape-carton/run.py --mode bench --out /path/to/new-contact-bench
/data/IsaacLab-Arena/.venv/bin/python tasks/pack-and-tape-carton/run.py --mode products-probe --out /path/to/new-products-probe
```

A successful packing-only probe still does not establish film pickup, adhesive
retention or blade severing. Dedicated loaded-contact negative controls and a
full released robot seal are outstanding.


The product containment scorer uses the native floor's nominal 6 mm support
height (two 3 mm bottom-board layers), with **0.2 mm numerical contact tolerance**
on the product's lowest vertex. The products-probe002 native trace settled at
5.988815 and 5.992245 mm, respectively 11.185 and 7.755 micrometres below that
plane. Such contact-solver settling passes containment; penetration greater
than 0.2 mm does not. The full product AABB must still satisfy the original
horizontal bounds (X ±104 mm, Y ±84 mm), upper Z bound 92 mm, and speed below
20 mm/s. This tolerance affects floor contact only and supplies no flap, tape,
cutting or release credit.


## Product packing plus four-flap closure probe

```bash
/data/IsaacLab-Arena/.venv/bin/python tasks/pack-and-tape-carton/run.py --mode closure-probe --out /path/to/new-closure-probe
```

`ClosureProbe` preserves the 32-second product-controller command prefix, then
closes inner east/west flaps followed by outer north/south flaps. It grasps an
ordinary lateral flap rim 40 mm from its hinge and 14 mm inside its side edge.
No new handles, grip joints, fixtures or body-pose writes are introduced. The
outer free-edge arc was rejected in CPU reach checks because the upright flap
would lift it beyond the arm's useful workspace.

The gripper closes across the board normal. A smooth approach tilt of at most
3 degrees avoids the wrist pitch limit on west/south while retaining at least
99.8% normal projection. Actual TCP alignment, native finger aperture,
measured hinge progress and the acquired contact offset gate entry, pinching
and folding. A lost grip or stalled flap fails explicitly. The last degree of
ordinary gripper target motion supplies at most about 0.7 mm seating overtravel
at this lever radius; the native hinge limit still stops the board at closure.
Release requires an actual angle within 0.3 degree of closed. Each released
flap must remain within 5 degrees before the next fold. Completion requires all
four measured closed angles and inner faces beneath outer faces for one second
after both arms return to their parking poses. The task scorer receives only
native geometry and can report 0.5 for packing plus closure; no phase supplies
score or tape adhesion.

The probe allows at most 3,600 controls (180 seconds) and stops early on failure
or measured closure completion. Traces include normalized actions, actual arm
and finger joints, FK TCP poses, aperture, target/contact errors and hinge
angles. This is a partial task attempt; sealing is still absent.

Top-flap crease stiffness is now an explicit **0.00002 N m per degree**, retaining
the existing -160 degree weak rest memory. USD angular-drive stiffness is torque
per degree, so this is about 0.001146 N m per radian and 0.0032 N m opening torque
at closed. The earlier 0.003 value was an unvalidated scaffold, about 0.48 N m
at closed. The new uncalibrated worn-crease candidate permits the 18 g flap's
roughly 0.00768 N m gravity torque to retain closure. Bottom and corner drives
are unchanged. Native grasp/closure trials must validate these revised physical
parameters; the earlier products-only result used the prior top stiffness.

CPU verification with the pinned YAM source available on `PYTHONPATH`:

```bash
PYTHONPATH=/path/to/pinned/yam-arena python check_closure_reach.py --out /tmp/closure-reach.json
```

The local check covers 32 hover/grasp targets through -175 to 0 degrees. Maximum
URDF IK error is 1.197 mm and 0.01275 rad. Triangle/box tests on all 136 collision
meshes of the gripper and two fingers find no target-board intersection with
20 mm open jaws; the 1.5 mm close command intersects the 3 mm board, confirming
that native contact must stop the closing jaws. These checks do not establish
whole-arm, inter-arm or other-panel collision clearance. An ideal joint-tracking
and retained-grasp dry run completes the final controller in 131.3 seconds; it
assumes perfect contact and is not a native physics demonstration.

The separate adhesion coupon remains a diagnostic: its 5 kg free support body
now rests above the 2 mm mat, and its initial film coordinates are Python floats
for the classic PhysX backend. It does not appear in closure-probe.
