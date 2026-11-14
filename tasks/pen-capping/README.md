# Cap and sort four pens

Fit each pen with a cap of the same color. Put the two red pens on the left
and the two blue pens on the right. Pens rest flat in marked destination areas.

## Reproduce

Use Isaac Sim 6.1.0.0, Isaac Lab 17.0.2, Arena 0.3.0, Inspect Robots 0.60.0 and
YAM Arena 0.1.0. Source pins: YAM `0059d02a1a91bc6834c1d0846022891df0c78730`;
Arena `c7edf3a9d5d8276565208f72422f8f8a0806d6cf`.

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

Pens reuse the central catalog marker-barrel mesh, stretched axially 1.4×;
scanned ends are trimmed and materials replaced with clear red/blue colors.
Analytic barrel, shoulder, nib and tail collisions give a 165.2 mm overall pen,
19.4 mm barrel diameter and 35 g mass. Caps are separate rigid bodies, 35 mm
long, approximately 32.6 mm diameter, 8 g. Sixteen individual convex wall boxes
surround an open bore with 12.5 mm inner apothem. One 3 mm thick disk closes
the far end. The shell bore is 25 mm across flats except for a 25.8 mm recessed liner
seat over cap local x=10 to 17.5 mm. The silicone liner specified below
bridges this clearance; there is no solid collider across the mouth.

For a physical reproduction, specify pigmented polypropylene (PP) for the pen
barrel, shoulder, tail and cap shell, a polyester felt nib, and printed paper
destination markers on a rigid tabletop. These are chosen reproduction
materials, not verified material identities of the catalog scan or a tested
physical assembly. The simulation represents all pen and cap parts as rigid;
its red/blue surface shaders specify appearance, not PP elasticity or felt
compliance. The destination markers are textures, with no paper mechanics.
The retention insert is specified as silicone rubber, nominal Shore A 40,
primer-bonded into the recessed PP seat. This is a proposed material hardness
and attachment, not a verified commercial silicone grade, adhesive system or
tested assembly. Bond compatibility, manufacturing tolerances, insertion force
and pull-off force must be established on physical specimens.

CPU PhysX 240 Hz, control 20 Hz, observations 640×360. Static/dynamic friction
is 1.2/1.0; task contact/rest offsets are 0.2/0 mm. Top camera is the existing
station demo sensor with native pose and intrinsics, without crop or zoom.
Calibrated wrist cameras are unchanged. Regenerate with
`python build_assets.py CATALOG_MARKER_USD` using the recorded source.
`python build_assets.py --caps-only` rebuilds just the revised caps offline
without downloading or changing the historical catalog barrel meshes.

## Asset attribution and license

The barrel visual geometry comes from NVIDIA's Isaac Sim 6.2 catalog asset
[`040_large_marker.usd`](https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.2/Isaac/Props/YCB/Axis_Aligned/040_large_marker.usd),
under `Isaac/Props/YCB/Axis_Aligned/`. Its downloaded SHA-256 is
`75673b47d4fcddfc2d7d5e825cf43b071f5e385c9f237126f8fe95abfc82df07`.
The derived `assets/pen-red.usdc` and `assets/pen-blue.usdc` retain the central
barrel mesh, trim the scanned ends, stretch the axis 1.4× and replace its
materials. Pen ends/colliders, separate caps and destination prints are
authored for this task; the catalog scan supplies no removable-cap mechanism.

The named license is the
[NVIDIA Isaac Sim Additional Software and Materials License](https://www.nvidia.com/en-us/agreements/enterprise-software/isaac-sim-additional-software-and-materials-license/).
NVIDIA's [asset licensing FAQ](https://docs.isaacsim.omniverse.nvidia.com/6.2.0/common/license-faq.html)
identifies this license for NVIDIA-provided assets, separately from Isaac Sim
source-code licensing. Attribution does not establish permission to redistribute
the modified mesh. The linked terms restrict distribution (§2.2) and derivative
works (§2.4), prohibit notice removal (§2.5), and address separate component
terms (§6). Asset-specific rights for the existing derived barrels remain
unverified; resolve them before further distribution or replace the mesh with
geometry whose rights are established. No permissive asset license is asserted.
Source verification and provenance details are in `assets/SOURCE.txt` and
`assets/provenance.json`.

## Liner dimensions and retention approximation

The chosen insert is a short silicone sleeve in the existing PP cap shell.
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
64 segments. The 0.4 mm recessed seat provides a bonding location. A PP/silicone-compatible
primer and adhesive system still needs to be selected and verified by bond
testing. No pull-off force follows from the dimensions
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

Offline checks: `python -m pytest -q test_scoring.py test_retention.py test_alignment.py`.
Runtime geometry, reset and normal-camera trials are complete on the same
frozen source. The scripted demo scores 1.0 and Dream-1 beta scores 0.0.
Measured engagement and retention through support release accompany the
recorded views; all four scripted placements hold full credit at the end.
These checks validate the simulation approximation, not real silicone holding
force or the proposed adhesive bond. Exact results and source hashes are in
`validation.json`, with a summary in `GPU_VALIDATION.md`.

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

The revised scripted demo completed with score **1.0**, 4,910 actions and
245.5 simulated seconds (613.21 seconds wall time) under the 6,000-action cap.
All four caps remained attached and all four released placements held full
credit through the final hold. Dream-1 beta completed 6,000 actions and
300 simulated seconds with score **0.0** in 746.78 seconds wall time, with no
trial errors. Both runs use identical frozen sources and settings.
The runtime geometry/reset audit passed, including contact and motion-history
cleanup; 77 task tests and one external approach regression passed.
See `validation.json` for exact source hashes and records, and
`GPU_VALIDATION.md` for the current evidence summary.

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

## Historical recorded runs (original unlined caps)

These records and `historical-validation.json` are preserved from the submitted
revision. They do not validate the liner geometry or revised insertion gate.
These historical records are separate from the revised results above.

Runtime revision: `2425d4f3d93b1a1e701c9ccc8df52523d2a369f5066168d618d4e5acc2fa4e58`.

| Policy | Score | Steps | Video time | Wall time |
| --- | ---: | ---: | ---: | ---: |
| Scripted | 1.0 | 3,540 | 177.05 s | 365.77 s |
| Dream-1 beta | 0.0 | 4,400 | 220.05 s | 485.50 s |

Both completed without trial errors, using the same source revision, cameras,
physics, seed 0, zero jitter and 4,400-step limit. The recorded videos and submission text were reviewed and approved for
submission. The uploaded demo, Dream run and setup image are recorded as
media `M-MYJ9A2` in `task.json`; synchronized wrist views are retained locally.

The native top-camera pose is approximately `(-0.166495,-0.009000,1.541821)` m,
quaternion xyzw `(0.212629,-0.212629,-0.674380,0.674380)`, OpenGL convention.
At 640×360 its intrinsics are fx=392.195618, fy=391.722351,
cx=318.389435, cy=177.876312. The historical calibration and result records are in
`historical-validation.json`. Physics and reset checks in that record predate this liner revision.
The recorded full score applies to its original synthetic-retention setup.

The one-pen milestone demonstrated insertion and synthetic joint retention
after release, without validating a physical retention feature.
The next milestone capped and placed one red and one blue pen before scene
expansion to four. The final 4,400-step budget was tightened from the initial
4,800 after the full-score demo showed that 3,540 steps were sufficient.
