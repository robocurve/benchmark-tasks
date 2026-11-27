# Cap and sort four pens

Fit each pen with a cap of the same color. Put the two red pens on the left
and the two blue pens on the right. Leave the capped pens released and resting
flat in their marked destination areas.

## Relevance and evaluation scope

Recapping markers before putting them away is a familiar object-care task:
the closure must engage the barrel and stay attached during handling and
storage. The robot must stabilize one part while aligning and advancing the
other, then transfer and release the completed assembly. This task evaluates
that coordinated insertion and handover, followed by color-correct placement.
Sorting alone earns no credit for an uncapped pen. **Societal relatability**
is the intended category, reflecting this everyday manipulation task.

The central manipulation is cap seating with the opposite arm supporting the
pen. Repeating it for four pens exercises consistent completion and correct
placement in a fixed, reproducible layout.

## Response to the review concerns

The earlier review identified the oversized bare cap bore, missing material and
retention specifications, catalog-asset attribution and category fit. The later
review requested workable engagement, a matching validated simulation
approximation, fresh demonstration/model evidence, and resolution of the
unverified asset-redistribution issue. The current submission addresses those
requests as follows:

| Concern | Current specification and evidence |
| --- | --- |
| A 25 mm shell bore cannot grip a 19.4 mm barrel. | The shell now contains a silicone insert with a **19.0 mm free bore**. Inserting the 19.4 mm barrel creates **0.2 mm nominal radial interference** over **3.9 mm nominal straight overlap**. The original shell clearance is occupied by the insert; the bare shell is not the retaining interface. |
| Physical materials and a reproducible retention feature were missing. | Specify PP bodies/shells, polyester felt nibs and **Smooth-On SORTA-Clear 40 silicone**. Four molded-through headed keys capture the liner without relying on a PP–silicone adhesive bond. Dimensions, tolerances, mass construction and the distinction between specified and tested properties are given below. |
| A synthetic joint alone did not establish a functioning cap. | The joint now represents a **specified interference sleeve**, rather than unspecified retention across an empty gap. It is created only after actual measured approach, axial overlap, alignment, eccentricity and nib-clearance checks remain eligible for 0.2 s. It preserves the reached pose without snapping either object into place. The noncolliding compliant zone and uncalibrated force response are explicitly disclosed below. |
| A new full-score demo and matching model run were required. | Both recordings use the revised lined-cap geometry and current original pen visuals: **demo 1.0, 4,910 actions; Dream-1 beta 0.0, 6,000 actions**. They use identical frozen runtime inputs and the same 6,000-action budget. The demo records four engagements, no retention losses and four correct released placements. |
| Redistribution rights for the adapted catalog mesh were unresolved. | Both scan-derived barrels were replaced with original analytic geometry. No external mesh is bundled, referenced or required by the generator. Regeneration and self-contained-asset tests pass. The former asset and named license are identified in the provenance section as historical attribution, not as a claim of redistribution permission. |
| Validation and physical claims needed to be separated. | **81 tests and the runtime geometry/reset audit pass.** The demo establishes completion of the stated simulation approximation. No manufactured specimen, measured insertion/pull-off force or physical retention qualification is claimed. A concrete physical qualification procedure is supplied separately. |

The revised task therefore has an explicit engagement mechanism, numerical
seating criteria, release-sensitive scoring and a completed robot-driven demo
of all four assemblies. The remaining fidelity limits concern elastomer
response and physical construction, not an omitted cap feature. These limits
are part of the submission's stated scope and are not inferred from the full
simulation score.

## Current recorded evidence

| Policy | Final score | Actions | Simulated time | Wall time |
| --- | ---: | ---: | ---: | ---: |
| Scripted demo | **1.0** | 4,910 | 245.5 s | 600.33 s |
| Dream-1 beta | **0.0** | 6,000 | 300 s | 737.58 s |

Both trials completed without errors, using seed 0, no jitter, CPU PhysX at
240 Hz, 20 Hz control, three native 640 × 360 cameras and a common 6,000-action
limit. The runtime input revision for both is
`ee5269d7fc4721b5eaae64e4656608f4d8d1ce1748182df31957f008cc03f902`.
The demo's engagement events occur at 39.35, 100.05, 158.70 and 220.05 seconds;
all four released placements retain full credit through the final hold.
Dream manipulates loose caps and an uncapped pen but establishes no retention.

**Current media: `M-CYJ5YA`.** The attached demo and Dream videos
are the native station-camera recordings of these two trials. The setup image
is from the same recorded initial layout. Synchronized wrist views are retained
in the evidence package. Numerical engagement records supplement the partly
occluded wrist insertion views. Exact source/asset hashes, terminal-result
checksums and review records are in `validation.json` and `GPU_VALIDATION.md`.

## Reproduce

Use Isaac Sim 6.1.0.0, Isaac Lab 17.0.2, Arena 0.3.0, Inspect Robots 0.60.0 and
YAM Arena 0.1.0. Source pins: YAM `0059d02a1a91bc6834c1d0846022891df0c78730`;
Arena `c7edf3a9d5d8276565208f72422f8f8a0806d6cf`;
Isaac Lab `91a0906f93802eb859192dcb0bca879f891e92d6`.

Activate the installed Arena/YAM environment, from the benchmark repository:

```sh
export TASKDIR="$PWD/tasks/pen-capping"
export PYTHONPATH="$TASKDIR${PYTHONPATH:+:$PYTHONPATH}"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
uv pip install --no-deps -e "$TASKDIR"
inspect-robots run --embodiment yam_pen_capping --policy pen_capping_scripted \
  -E task="$TASKDIR/task.py" -E object_jitter=0 -E device=cpu \
  -E top_cam=demo -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "Fit each pen with a cap of the same color. Put the two red pens on the left and the two blue pens on the right." \
  --scorer task_score --max-steps 6000 --seed 0 --store-frames --log-dir pen-demo
```

For Dream replace `--policy pen_capping_scripted` with
`--policy dream -P model=dreamlabs/dream-1-beta`; use a separate log directory.
Both policies use the same embodiment, cameras, physics and task revision.
The scripted policy uses privileged object poses to command robot joints.

## Fixed setup

Coordinates are metres, x forward, y left, z up; tabletop z=0.7475. Existing
YAM station arm bases are `(0, ±0.305, 0.750)`. Source layout is defined in
`task.py`: four thick pens and four independent caps, with known varied yaws,
zero jitter and seed 0. Two flat marked destination areas are 240×200 mm,
centered at `(0.300, ±0.230)`, red left and blue right. The prints are visual
surfaces 0.3 mm above the existing tabletop, with no raised receptacle walls.

Nominal roots before settling (x,y,z,yaw°):

| Color | Pen | Cap |
| --- | --- | --- |
| Red | (0.16,−0.16,0.7575,−100) | (0.18,0.15,0.7638,−120) |
| Blue | (0.29,−0.15,0.7575,−80) | (0.38,0.32,0.7638,−105) |
| Red | (0.42,−0.16,0.7575,−65) | (0.40,0.10,0.7638,−145) |
| Blue | (0.21,−0.34,0.7575,−110) | (0.24,0.05,0.7638,−125) |

Pens use original analytic barrel, shoulder, nib and tail geometry, with red/blue
surface colors; no downloaded mesh is included or required. The same four
analytic collisions as the previous lined-cap revision give a 165.2 mm overall pen,
19.4 mm barrel diameter and 35 g mass. Caps are separate rigid bodies, 35 mm
long, approximately 32.6 mm diameter, 8 g. Sixteen individual convex wall boxes
surround an open bore with 12.5 mm inner apothem. One 3 mm thick disk closes
the far end. The shell bore is 25 mm across flats except for a 25.8 mm recessed liner
seat over cap local x=10 to 17.5 mm. The silicone liner specified below
bridges this clearance; there is no solid collider across the mouth.

For a physical reproduction, specify pigmented polypropylene (PP) for the pen
barrel, shoulder, tail and cap shell, a polyester felt nib, and printed paper
destination markers on a rigid tabletop. These are chosen reproduction
materials, not a claim that a physical assembly has been manufactured or tested. The simulation represents all pen and cap parts as rigid;
its red/blue surface shaders specify appearance, not PP elasticity or felt
compliance. The destination markers are textures, with no paper mechanics.
The retention insert is specified as Smooth-On SORTA-Clear 40 silicone rubber,
manufacturer nominal Shore A 40, mechanically keyed into the recessed PP seat.
The [manufacturer's material specification](https://www.smooth-on.com/products/sorta-clear-40/)
gives nominal Shore A 40 and density 1.08 g/cm³; neither value establishes a
retention force for this assembly.
Four molded-through keys with recessed outer heads capture the liner without
assuming a PP–silicone adhesive bond. Full dimensions, mass construction and
the physical qualification procedure are in [PHYSICAL_REPRODUCTION.md](PHYSICAL_REPRODUCTION.md).
The moving pen target is **35 ± 1 g**, including its PP body, felt reservoir/nib
and any enclosed mass adjustment. The complete cap target is **8.0 ± 0.5 g**,
including the silicone insert. Use hollow PP pen bodies and lightened/ribbed
cap shells, with a nominal 1 mm outer skin and full bosses around the liner
keys; adjust enclosed relief or ballast to the finished mass without changing
fit or external grasp surfaces. Dense simulation wall boxes are collision
proxies, not a claim that a solid PP shell has the specified mass.

For each liner, four **3 mm radial through-holes** at cap x=13 mm lie on flats
at 0°, 90°, 180° and 270°. Each has a **5 mm diameter × 0.6 mm deep external
counterbore**. Cast the sleeve and keys as one silicone piece, filling the
counterbores with headed keys and finishing their outer faces flush with the
shell. Retain PP bosses around the ports when lightening the shell. The heads
mechanically capture the sleeve; no chemical bond to PP is assumed. The ports
span x=11.5–14.5 mm, inside the straight-grip interval and outside the free bore.

The proposed finished barrel diameter is **19.40 ± 0.05 mm**, and free liner
bore **19.00 ± 0.05 mm**, giving 0.15–0.25 mm radial interference before
eccentricity. A physical set should be checked over ten insertion/removal
cycles, with the proposed qualification targets of insertion force ≤5 N,
liner/key migration ≤0.2 mm, and retention against 0.5 N axial separation for
ten seconds. These are **unperformed fixture-qualification targets**, separate
from the scored one-second hand-clear checks. They are not experimental
results or manufacturer guarantees. Full construction and measurement steps
are in `PHYSICAL_REPRODUCTION.md`.

Headless runs explicitly disable asynchronous renderer scheduling before scene initialization; this is process-local and does not change physics or camera geometry. CPU PhysX 240 Hz, control 20 Hz, observations 640×360. Static/dynamic friction
is 1.2/1.0; task contact/rest offsets are 0.2/0 mm. Top camera is the existing
station demo sensor with native pose and intrinsics, without crop or zoom.
Calibrated wrist cameras are unchanged. Regenerate all authored assets with
`python build_assets.py`, or use `--pens-only` / `--caps-only` for the indicated
subset. No download or source mesh argument is needed.

## Asset provenance

All current task-object geometry is generated from original analytic primitives
in `build_assets.py`. In each pen asset the barrel cylinder is now visible;
there is no scanned mesh, external USD reference or catalog download. The four
pen colliders, masses, relative transforms and materials are unchanged. The
caps and destination assets are unchanged from the previous lined-cap revision.
Destination textures contain rendered text; the font itself is not bundled.
`assets/SOURCE.txt` and `assets/provenance.json` document the current source and
asset hashes. Earlier recordings and historical Git commits contain the former
catalog-derived barrel visual; they are retained as historical evidence, not
bundled as the current revision's assets or recordings.

The historical barrel asset was NVIDIA's catalog
[`040_large_marker.usd`](https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.2/Isaac/Props/YCB/Axis_Aligned/040_large_marker.usd),
under `Isaac/Props/YCB/Axis_Aligned/`, associated with the
**NVIDIA Isaac Sim Additional Software and Materials License**. Earlier versions
trimmed and stretched its central scan mesh. This names the prior asset and
license requested in the earlier review. The current pen files instead contain
four original cylinders and no scan mesh; replacement removes the current
submission's dependency on redistributing that adapted asset. No new permission
claim is made for the historical files. Runtime-supplied station/robot assets
remain dependencies of the documented simulator environment and are not bundled
as task-object meshes here.

## Liner dimensions and retention approximation

The chosen insert is a short mechanically captured silicone sleeve in the PP cap shell.
The physical nominal dimensions below are design specifications, not measured
properties of an existing pen/cap product. All axial coordinates are cap-local:

| Feature | Dimension |
| --- | --- |
| Pen barrel diameter / front edge | 19.4 mm / pen x=−70 mm |
| Original shell bore | 25 mm across flats |
| Recessed liner seat / liner outside | 25.8 mm across flats, x=10 to 17.5 mm |
| Straight liner free bore | 19.0 mm diameter, x=10 to 16 mm |
| Entrance taper | ID 19 to 21 mm, x=16 to 17.5 mm |
| Nominal sleeve radial thickness at a shell flat | 3.4 mm |
| Nominal barrel radial interference | 0.2 mm (about 5.9% of that thickness) |
| Rigid simulation backing aperture | 21.2 mm across flats |

The silicone outside follows the shell's 16-sided seat. Its visible bore uses
64 segments. The 0.4 mm recessed seat and four molded-through radial keys locate
and capture the physical insert; the keys are a physical manufacturing detail
omitted from the simulated rigid shell. No adhesive is required. No pull-off force follows from the dimensions
or Shore hardness alone. The insert occupies space that was previously empty;
the shell dimensions, nominal assembly target and initial object layout stay
the same. The chosen total cap mass remains 8 g, a simulation design parameter.

Pen local −x points toward the nib; cap local +x is its mouth. At the nominal
cap root x=−82.1 mm in pen coordinates, the barrel front lies at cap x=12.1 mm.
It therefore engages 3.9 mm of the 6 mm straight liner, with 4.2 mm nib-to-end
clearance. The entrance has 0.8 mm nominal radial clearance around the barrel.

The geometric insertion gate uses the full relative orientation and intersects
the finite 19.4 mm barrel with x=10 to 16 mm of the straight liner. It trims
the axial overlap for tilt at the barrel rim. It requires at least 3 mm of
full-section overlap, at most 0.3 mm axis eccentricity throughout that overlap,
at most 0.55 mm maximum radial compression demand, and at least 0.4 mm nib
clearance from the closed end. A 2° axis limit also rejects gross misalignment;
the eccentricity test is tighter for a cap tilted about its root. These are
chosen simulation acceptance limits, not experimentally established silicone
tolerances. A preceding aligned approach at cap root x<−115 mm, lateral
error<3 mm and angle<10° is required, followed by four eligible control steps
(0.2 s). Merely crossing the old nominal root-position window is insufficient.

**The inner silicone zone is visible geometry without a rigid collider.**
Only the outer annular backing is represented by 16 convex rigid colliders,
with a 21.2 mm aperture that leaves 0.9 mm radial clearance around the barrel.
This prevents a rigid 19.0 mm bore from making the specified interference
impossible to insert. It does not simulate compression, contact pressure,
frictional holding or contact-only retention. The rigid backing and shell may
produce contact; such contact must not be reported as elastic liner retention.

After geometric engagement, a fixed joint preserves the reached pose without
moving either object. It approximates the unresolved compliance and holding
behavior, has no calibrated break force or pull-off curve, and stays attached
until reset or geometric invalidation. The same engagement test invalidates
a separated, excessively eccentric or penetrating pair. It cannot validate
physical pull-off strength. Wrong-color caps fit but receive no credit.

Offline checks: `python -m pytest -q` (81 passed, no skips). These cover fit,
scoring, controller/recovery limits, solver settings and regeneration of the
original pen assets without a source mesh. The runtime audit also verifies
actual geometry, applied solver settings, joint/reset cleanup and motion-history
reset. The full normal-camera demo and matching Dream run use identical frozen
runtime inputs. Exact source hashes, terminal results and numerical/visual review
records are in `validation.json`, with a summary in `GPU_VALIDATION.md`.
These checks validate the stated simulation approximation, not physical silicone
holding force. The former scan-visual runs remain explicitly historical in
`previous-lined-validation.json` and `PREVIOUS_GPU_VALIDATION.md`.

## Controller and current validation

The scripted controller uses privileged object pose feedback only to command
robot joints. It aligns axes, compensates rotation about the held cap root,
limits near-mouth advance to 3 mm/s with at most 2.5 mm lead over the measured
root, smooths near-mouth TCP commands over 150 ms, and waits for actual engagement before releasing the supporting hand.
The cap is grasped 8 mm behind its root to clear the opposing wrist housing.
The alignment reference smoothly removes its initial transverse offset while
the pen moves into position. A single bounded withdrawal/recentering retry handles reported backing-contact
stalls. After engagement it preserves the last actuator command during handover.
The controller never moves task objects directly or changes fit criteria.

The maximum four-pair plan is 256 seconds, plus a 5-second final hold and at most
24 seconds of retention waits. Unused insertion/seating time is skipped after
0.5 seconds of observed retention. Recovery permits five seconds to center and
eight seconds for a 3 mm/s reinsertion. The common demo/model budget is 6,000 actions
(300 seconds at 20 Hz). This replaces the historical 4,400-action budget to
allow a slower contact approach. Numerical diagnostics without camera updates
are development evidence only; final demo and Dream use the normal cameras.

The current results and media are listed above. The source manifests for the
demo and Dream match exactly. The top timeline was sampled every two seconds
for the demo and every five seconds for Dream, with wrist samples around each
engagement or selected model-run windows. No team-identifying text was visible
in the sampled media. These are simulation results; physical force performance
of the specified sleeve/key construction remains unmeasured.

## Score and reset

End score is 0.125 per correctly colored retained cap, confirmed after one
continuous second with both hands clear. Another 0.125 requires that capped
pen currently released and resting inside its correct destination for one
continuous second. Wrong-color or uncapped pens earn no placement credit.
Capping credit persists while the correct pair remains retained; placement
credit is lost if it is moved out or held again.

Release requires each TCP more than 60 mm from the full pen centerline segment
(local x −100 to +73 mm) and from its cap root. Rest requires pen root 6–25 mm
above the table, axis within 8° of horizontal and
`(root displacement + 0.06 × full orientation change in radians) / 0.05 s
< 0.015 m/s` at each 20 Hz control interval. Full orientation change includes
spin about the pen axis. This measures resolved motion rather than a noisy
last-substep solver velocity; translation, rotation and interval-to-interval
oscillation still reset the one-second rest counter.
The declared projected pen/cap envelope must fit inside the destination
rectangle with 1 mm inset. Full completion means all four retained caps and
all four released, resting, correct placements.

Reset removes every retention joint, restores separate pens and caps to the
fixed layout, clears insertion/release/scoring history and returns fresh
camera observations after temporal-history flushing. Scorer tests cover
partial and full outcomes, wrong colors, uncapped placement, missed insertion,
side/angle errors, held/moving objects, destination errors and release duration.

## Calibration and historical evidence

The native station-camera pose is approximately
`(-0.166495,-0.009000,1.541821)` m, quaternion xyzw
`(0.212629,-0.212629,-0.674380,0.674380)`, OpenGL convention. At 640×360 the
intrinsics are fx=392.195618, fy=391.722351, cx=318.389435, cy=177.876312.
Wrist-camera calibration is unchanged from the pinned YAM station rig.

`historical-validation.json` describes the original **unlined** cap recordings;
`previous-lined-validation.json` and `PREVIOUS_GPU_VALIDATION.md` describe the
later lined-cap recordings with the former scan-derived barrel visuals. They
are retained as historical records. The current `validation.json` and media ID
above identify the new original-asset demo and Dream run. Earlier results and
media IDs are not evidence for this revision's geometry or current recordings.
