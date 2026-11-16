# Dry long hair evenly with a handheld dryer

**Incomplete feedback draft:** see [DRAFT_STATUS.md](DRAFT_STATUS.md) for current measured evidence and missing work. No full-score demo or competition-ready submission is claimed.

Native mechanics and controller under development; no complete drying demo yet.

Use a long-haired mannequin representing a woman's hairstyle. Pick up and
operate the dryer, expose hair sections as needed, and dry the full length,
both exposed sides and under-layers. Leave no persistently wet region. The
second arm can separate or lift locks; a rigid painted sheet cannot count as
successful hair manipulation.

Candidate native representation: multiple articulated flexible locks with
self-contact and scalp-root attachment. Fine fibre geometry can be rendered
around the physical bundles. Coarse bundles, bending/contact forces, airflow
and moisture/temperature models must be documented and calibrated; they are
not interchangeable with individual-strand fluid simulation.

Drying must depend on measured dryer position/direction, switch state,
airflow attenuation/occlusion, distance, exposure and local moisture. Merely
sweeping a target trajectory cannot flip hair regions to dry. Test front-only
exposure, hidden inner layers, near/far operation, no airflow, manipulation,
and released stable hair. Exact moisture/heat thresholds and task allowance
remain unset until the model and native reach/access probes are established.

Native geometry, thermal/occlusion kernels and incomplete robot probes now exist. No complete drying score, demo or submission readiness is claimed.
DYMO-Hair is relevant prior work on hair dynamics/manipulation, not proof that
this implementation can model or complete drying. See the batch record.

## Current implementation and probes

The mannequin head is fixed to its ordinary stand by a passive world fixed
joint at its declared HEAD position and identity orientation. It cannot swivel
for coverage: only the arms/dryer and the flexible hair locks may move. The
sixty passive hair bending joints remain free; no head motor, kinematic pose
write or reset loop maintains the mount. This fixed-head revision has CPU/USD
tests only; native constraint accuracy and a new robot run remain pending. A normal open U-shaped dryer holder supports the barrel with its handle
accessible below; no extra grasp handles are added. The real handle slider
sets power from measured travel. The robot braces the handle, pushes the
slider using measured phase gates, confirms retained power after horizontal
retraction, and verifies actual dryer lift. It then holds the powered dryer;
full drying motion remains unimplemented.

`airflow.py` traces from the measured nozzle to four surface quadrants on every
hair segment. The scalp and other measured capsule locks can occlude airflow.
Local heat/moisture loss is energy limited; power-off and hidden patches do
not dry. Candidate pressure drag is capped at 0.25 N total with equal/opposite
dryer recoil. The dryer uses an ordinary **45 °C low-heat setting**; the scoring limit
remains 50 °C. This candidate source temperature is not an empirical
calibration. Heat, moisture and drag values remain uncalibrated.
The hair model is a coarse bundle approximation, not individual-fibre CFD.

```bash
/data/IsaacLab-Arena/.venv/bin/python run.py --mode dynamics --out /path/to/new-probe
/data/IsaacLab-Arena/.venv/bin/python run.py --mode power-probe --out /path/to/new-attempt
/data/IsaacLab-Arena/.venv/bin/python run.py --mode slider-coupon --out /path/to/new-slider-coupon
```

All modes are unscored diagnostics. Power-only runs report any scorer readback
separately; operating and lifting the dryer does not demonstrate complete drying.

### Passive slider retention candidate

The historical hair-power013 trial used the superseded swivelling-head mount.
Its video does not demonstrate this fixed-head task. It reached full measured
travel, then returned to
OFF during finger retraction. Its joint had travel limits and an authored friction attribute, without
a detent. The current candidate adds the passive constitutive potential
`U(q) = 0.5 k min(q², (q − L)²)`, with `L = 4 mm`, `k = 100 N/m`, and native
joint damping `c = 0.05 N s/m`. Both wells have equal energy; the elastic peak
is 0.2 N and the barrier is 0.2 mJ. Original authored friction, mass, geometry, travel,
thermal parameters and task scoring remain unchanged. These values are an
uncalibrated approximation, not a selected commercial switch mechanism.
The [`jointFriction = 0.08` attribute](https://docs.omniverse.nvidia.com/kit/docs/omni_usd_schema_physics/latest/physxschema/class_physx_schema_physx_joint_a_p_i.html) is dimensionless and articulation-only;
the current standalone slider excludes articulation, so this attribute is
ignored. It does not provide a 0.08 N friction force.

A native linear joint drive supplies the reaction. A measured-coordinate-only
callback chooses the nearest well before each configured 480 Hz physics step,
using direct native rigid-body transforms. It has no robot-phase, power-flag,
timer or pose-write input. The harness checks 24 callbacks per 20 Hz action and
records selected targets and the spring-energy change at each target switch.
The [PhysX pre-step callback](https://isaac-sim.github.io/IsaacLab/v3.0.0/source/concepts/physics-backends/physx.html)
is distinct from policy-rate updates. This remains a **sampled approximation**:
CPU energy tests and callback counts do not establish native drive-update
latency or physical retention. Those native claims remain unverified.

`slider-crossings.json` contains bounded response windows: eight raw native
samples before each well crossing, the crossing sample, and 24 after it,
retaining at most 16 events and counting evictions. Each sample includes callback
index, displacement, body poses/COM velocities, selected/authored target,
constitutive potential and its change. These observations can reveal target
freshness or delayed response; attribute readback alone never marks native
target-application latency verified. Long steady drying adds no trace events.

The separate 7.7-second `slider-coupon` mode spawns only the ordinary dryer and
slider, with an explicitly diagnostic native world-to-dryer clamp. It applies
an unscored +0.35 N axial pulse from 1–1.5 s, removes force for two seconds,
ramps a reverse load to −0.10 N over 0.2 s, holds it for 0.3 s, ramps back over
0.2 s, then observes one second of zero-force recovery. It finally applies
−0.35 N from 5.2–5.7 s and removes force for two seconds. The ramp is updated
at every 480 Hz substep. No robot,
hair scorer, video or demo credit is involved; these external pulses and the
fixture cannot enter the robot harness. Every native substep contributes to
retention extrema, avoiding 20 Hz aliasing: final one-second ON/OFF holds must
remain above 80% / below 10%, reverse-load travel must remain above 2 mm, and
the final 0.25 s of recovery must remain above 80% ON. The coupon measures
substep attachment/overtravel errors. Joint reaction is inferred
from native COM acceleration minus known external force and gravity, including
drive and limits; it is not a direct native force-sensor measurement.
This prepares a native mechanism check; it has not been run. Even a passing
coupon would leave robot withdrawal, tilt/lift and OFF operation to validate.

Before using it for a drying demonstration, run this separate retention plan:

1. Verify OFF at rest and native drive/limit readback; record raw displacement,
   callback cadence and lateral/angular slider-joint error.
2. Use ordinary robot contact to push ON, verify travel above 90% for five
   frames, retract the finger horizontally, and observe at least two seconds
   above 80% with no switch contact.
3. Verify a real 75 mm or greater lift. With collision/reach certificates,
   tilt the held dryer through orientations that reverse its slider-axis
   gravity load, holding each for two seconds without touching the switch.
4. Press the actual upper slider face downward using ordinary robot commands.
   Verify travel below 10% for five frames, withdraw, and retain OFF for two
   seconds. Inspect both directions for overtravel or joint separation.
5. Verify that measured OFF produces no new drying, and review native target
   application timing and switch-energy telemetry. Do not credit the test as
   all-patch drying, switch-off/parking completion, or a successful task demo.

The first separate-body hair probe passed its passive test, but moving-head
robot trials exposed excessive joint error. A reduced-coordinate tree is now
being validated for both integrity and speed. Current slider/pickup attempts
are incomplete. Full inner-layer manipulation, all-surface drying, switch-off,
released parking and native scoring integration are still required. Do not
infer successful drying from a planned arm sweep or a warm-coloured render.

The switch probe clips the unmodified finger collision triangles to the actual
slider footprint and starts 1 mm below its lower face. Alignment and insertion
require five measured settled frames (TCP <1 mm, orientation <1 degree,
dryer drift <1 mm); upward pushing occurs only afterwards at 1 mm/s. It logs
measured support height and finger joints, and aborts on missing slider motion,
tracking timeout, brace disturbance or failed pickup. Up to 1,600 control
steps / 80 seconds are allowed for this engineering probe, not a complete task.
The trace includes local patch water, temperature, exposure and occluder IDs,
and the measured minimum scalp-to-capsule clearance every frame. Connected
root capsules now collide with the scalp; adjacent lock segments remain
filtered. Native clearance and controller success still require GPU validation.

The left grasp is 25 mm below the handle midpoint to clear the second hand.
An elevated approach avoids a low-pose IK limit. Before insertion the probe
uses actual loaded jaw joints and TCP pose to certify at least 1 mm between
both hands throughout the full insertion/stroke/retraction translation volume.
The nominal CPU certificate covers loaded jaw positions from −12 to −20 mm
and gives at least 5.24 mm separation; native frictional pickup still needs
validation. Neither the finger geometry nor the grip force target is altered.

Native collision repair (hair-skin012): the backend filtered the root
articulation-link/scalp pair even with joint collision enabled. The650g head
is now split into a600g head and50g invisible spherical collision skin joined
by a passive fixed joint excluded from the articulation. Its rendered head and
physical radius are unchanged; it is an anatomical rigid attachment, not a
robot grasp constraint. Native maximum attachment gap0.645micrometres, strand
anchor gap0.242micrometres; sphere/capsule contact check passed throughout the
9-second force diagnostic. This is not a drying-completion demonstration.


## Fixed-head requirement and measured scoring guard

The declared reference is HEAD=(0.43,0,1.045)m with identity world rotation,
not the first measured frame. The stand mount uses a world-to-head FixedJoint,
with the same rest anchors and identity local rotations; it is excluded from
the hair articulation. All60 passive bending DOFs, skin attachment, dryer
slider/detent, masses, temperatures, moisture parameters and drying thresholds
are unchanged. Covering hidden locks must use physical hair manipulation and
arm/dryer motion; moving the head cannot earn even partial credit.

Every native trace row includes `head_pose` and numeric `head_motion` translation
in metres/rotation in degrees; result checks report maximum displacement and
`head_fixed`. The scorer independently reads the raw measured pose and latches
invalidation for translation greater than2mm or rotation greater than1degree.
Missing/nonfinite poses and invalid zero quaternions also invalidate, including
partial drying scores. Quaternion signs are equivalent; values are normalized
for motion measurement without rebasing the declared reference.

Forty-two CPU tests pass in the local USD environment, including real OpenUSD
FixedJoint/rest-anchor topology, all60 passive joints, retained skin attachment
and scorer negative controls. The local USD wheel omits NVIDIA PhysxSchema;
its topology tests provide only an unused import namespace and exercise real
UsdPhysics APIs, not PhysX simulation or mocked constraint behavior. Native
fixed-head retention remains pending. Older swivel-head traces/videos, including
hair-power013 (score0, ON retention failure), are historical evidence only.
