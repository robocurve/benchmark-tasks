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
uv pip install --no-deps -e "$TASKDIR"
inspect-robots run --embodiment yam_pen_capping --policy pen_capping_scripted \
  -E task="$TASKDIR/task.py" -E object_jitter=0 -E device=cpu \
  -E top_cam=demo -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "Fit each pen with a cap of the same color. Put the two red pens on the left and the two blue pens on the right." \
  --scorer task_score --max-steps 4400 --seed 0 --store-frames --log-dir pen-demo
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
the far end. There is no solid collider across the mouth or inside the bore.
The enlarged rigid bore gives insertion clearance; no cap-wall deformation
or frictional snap-fit is simulated.

For a physical reproduction, specify pigmented polypropylene (PP) for the pen
barrel, shoulder, tail and cap shell, a polyester felt nib, and printed paper
destination markers on a rigid tabletop. These are chosen reproduction
materials, not verified material identities of the catalog scan or a tested
physical assembly. The simulation represents all pen and cap parts as rigid;
its red/blue surface shaders specify appearance, not PP elasticity or felt
compliance. The destination markers are textures, with no paper mechanics.
This material specification alone does not provide cap retention; the missing
engagement geometry and its compliant material remain pending below.

CPU PhysX 240 Hz, control 20 Hz, observations 640×360. Static/dynamic friction
is 1.2/1.0; task contact/rest offsets are 0.2/0 mm. Top camera is the existing
station demo sensor with native pose and intrinsics, without crop or zoom.
Calibrated wrist cameras are unchanged. Regenerate with
`python build_assets.py CATALOG_MARKER_USD` using the recorded source.

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

## Insertion and synthetic retention

Pen local −x points toward the nib; cap local +x is its open mouth. Before a
cap may retain, the embodiment must observe an aligned approach with cap
root local x<−115 mm, lateral error<3 mm and axis alignment within 10°.
Seating requires cap root at x=−82.1±1.5 mm, lateral error<1.5 mm and axes
within 5° for four consecutive control steps (0.2 s). This covers the nib
and overlaps the barrel by approximately 5.4 mm through the hollow opening.

Only then a fixed retention joint is created at the actual reached relative
pose and orientation. Neither object is moved to a prescribed pose. This
joint substitutes for cap retention; it is not an arm attachment and does not
demonstrate a physical snap-fit or friction fit.
If the measured pair later differs from the nominal seated displacement by
more than 6 mm or its axes differ by over 8°, retention is invalidated and the
joint is removed. Retention is otherwise non-removable until reset.
Wrong-color caps may physically fit but earn no credit.

**Physical retention remains unresolved.** The 25 mm bore across flats exceeds
the 19.4 mm barrel diameter by 5.6 mm, leaving at least 2.8 mm radial clearance
in the coaxial configuration. Neither the analytic pen colliders nor the cap
walls include an engaging lip, liner or detent. Increasing friction cannot
bridge this clearance without contact pressure. The fixed joint supplies the
holding constraint in the current runs; it has no calibrated pull-off force.
Naming PP as a shell material does not repair that discrepancy.

Later physics revision, requiring separate simulation authorization:

1. Design a compatible physical engagement, for example a compliant annular
   liner inside the PP shell. This is a candidate, absent from the current
   assets. Dimension its attachment, free inner diameter, interference,
   lead-in and axial overlap against the 19.4 mm barrel and nominal 5.4 mm
   overlap; it must bridge the 2.8 mm radial gap. Select a specific elastomer
   grade and tolerances from insertion/pull-off measurements.
2. Implement matching visible and collision geometry in `build_assets.py`
   and material/compliance behavior in `pen_capping/embodiment.py`. Validate
   actual contact retention with the synthetic joint disabled first. If a
   joint approximation is still necessary, gate it on the implemented
   engagement and calibrate its failure force to the physical design.
3. On a newly verified authorized allocation, check insertion force, contact
   penetration, off-axis rejection, withdrawal, release, reset and retention
   under gravity. Include an axial proof load of at least 0.35 N for one second
   (enough to support the 35 g pen), plus the task's transfer motions. Recheck
   scoring against missed engagement, wrong colors and released placements.
4. Obtain a new full-score demonstration and, when authorized, matching model
   evidence under the revised geometry and physics. Record new hashes, forces,
   result logs and media; the existing runs cannot validate that future design.

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
`norm(linear_velocity)+0.06*norm(angular_velocity)<0.015 m/s`.
The declared projected pen/cap envelope must fit inside the destination
rectangle with 1 mm inset. Full completion means all four retained caps and
all four released, resting, correct placements.

Reset removes every retention joint, restores separate pens and caps to the
fixed layout, clears insertion/release/scoring history and returns fresh
camera observations after temporal-history flushing. Scorer tests cover
partial and full outcomes, wrong colors, uncapped placement, missed insertion,
side/angle errors, held/moving objects, destination errors and release duration.

## Recorded runs

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
cx=318.389435, cy=177.876312. The exact calibration and result records are in
`validation.json`. Physics and reset checks were performed before the camera
revision; their physics inputs are unchanged. The recorded demos demonstrate
full scores under the final camera configuration.

The one-pen milestone demonstrated insertion and synthetic joint retention
after release, without validating a physical retention feature.
The next milestone capped and placed one red and one blue pen before scene
expansion to four. The final 4,400-step budget was tightened from the initial
4,800 after the full-score demo showed that 3,540 steps were sufficient.
