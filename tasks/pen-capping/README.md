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
or frictional snap-fit is simulated. Source URLs, hashes, changes and asset-use
terms are in `assets/provenance.json` and `assets/SOURCE.txt`.

CPU PhysX 240 Hz, control 20 Hz, observations 640×360. Static/dynamic friction
is 1.2/1.0; task contact/rest offsets are 0.2/0 mm. Top camera is the existing
station demo sensor with native pose and intrinsics, without crop or zoom.
Calibrated wrist cameras are unchanged. Regenerate with
`python build_assets.py CATALOG_MARKER_USD` using the recorded source.

## Physical insertion and retention

Pen local −x points toward the nib; cap local +x is its open mouth. Before a
cap may retain, the embodiment must observe an aligned approach with cap
root local x<−115 mm, lateral error<3 mm and axis alignment within 10°.
Seating requires cap root at x=−82.1±1.5 mm, lateral error<1.5 mm and axes
within 5° for four consecutive control steps (0.2 s). This covers the nib
and overlaps the barrel by approximately 5.4 mm through the hollow opening.

Only then a fixed retention joint is created at the actual reached relative
pose and orientation. Neither object is moved to a prescribed pose. This
joint approximates a cap's snap retention; it is not an arm attachment.
If the measured pair later differs from the nominal seated displacement by
more than 6 mm or its axes differ by over 8°, retention is invalidated and the
joint is removed. Retention is otherwise non-removable until reset.
Wrong-color caps may physically fit but earn no credit.

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

The one-pen milestone proved physical insertion and retention after release.
The next milestone capped and placed one red and one blue pen before scene
expansion to four. The final 4,400-step budget was tightened from the initial
4,800 after the full-score demo showed that 3,540 steps were sufficient.
