# Mark answers on a printed form

Pick up the pen and mark A for question 1, C for question 2, D for question 3,
and B for question 4. The uncapped marker makes permanent blue marks only from
measured nib-to-paper contacts. The scripted demo draws four simple checks.

## Reproduce

Use Isaac Sim 6.1.0.0, Isaac Lab 17.0.2, Arena 0.3.0, Inspect Robots 0.60.0,
and YAM Arena 0.1.0. Tested source pins:

- YAM Arena: `0059d02a1a91bc6834c1d0846022891df0c78730`.
- Isaac Lab Arena: `c7edf3a9d5d8276565208f72422f8f8a0806d6cf`.

Activate the installed Arena/YAM environment. From the benchmark repository:

```sh
export TASKDIR="$PWD/tasks/answer-sheet"
uv pip install --no-deps -e "$TASKDIR"
inspect-robots run --embodiment yam_answer_sheet --policy answer_sheet_scripted \
  -E task="$TASKDIR/task.py" -E object_jitter=0 -E device=cpu \
  -E top_cam=demo -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "Pick up the pen and mark A for question 1, C for question 2, D for question 3, and B for question 4." \
  --scorer task_score --max-steps 2400 --seed 0 --store-frames --log-dir answer-demo
```

For Dream, replace `--policy answer_sheet_scripted` with
`--policy dream -P model=dreamlabs/dream-1-beta` and use a different log directory.
Both policies use the same embodiment, scene, cameras, physics and revision.
The scripted policy uses privileged pose feedback but commands only robot joints.

## Fixed setup

Coordinates are metres: x is forward, y is left, z is up. The existing station's
arm bases are at `(0, ±0.305, 0.750)`; tabletop z is `0.7475`.

The fixed sheet is 235 × 235 × 1 mm, centred at `(0.285, 0.020)` with its top
1 mm above the table. Question rows 1–4 are at x = 0.355, 0.310, 0.265, 0.220.
Columns A–D are at y = 0.080, 0.040, 0.000, −0.040. Boxes are 22 mm square,
at 45 mm row pitch and 40 mm column pitch. Labels are printed on the sheet.

The marker starts flat along x, nib toward −x, root `(0.240, −0.240, 0.7575)`.
Its measured resting bounds are 120.00 × 19.44 × 19.44 mm; mass is 25 g.
The catalog barrel has analytic barrel/shoulder/end collisions and an exposed
2 mm-radius spherical writing tip. Static/dynamic friction is 1.2/1.0;
marker contact/rest offsets are 0.3/0 mm. No marker joints or grasp latch are used.
Source and asset-use terms are recorded in `assets/provenance.json` and
`assets/SOURCE.txt`. Regenerate geometry/print with `python build_assets.py`.

One seed (0), zero jitter, 240 Hz CPU PhysX and 20 Hz control are used. Camera
observations are 640 × 360. The top view uses the existing station demo sensor
with its native pose and intrinsics, without a crop or zoom override.
The calibrated wrist mounts are unchanged.

## Scoring and fidelity

The end score is 0.25 per correctly answered row. A qualifying mark is one
connected ink component spanning at least 12 mm, with at least 20 occupied
0.5 mm cells in the box interior (inset 0.6 mm). Any ink in another answer box
invalidates that row, including a dot. A dot, disconnected short dashes, or
hovering cannot earn answer credit. Other unmarked rows earn zero.

Ink is a contact-based visual approximation, not a fluid simulation or
handwriting recognizer. The embodiment accepts only the writing sphere's paper
contact reports, with separation at most 0.05 mm and contact height within
0.45 mm of the sheet. It joins contact samples separated by at most two physics
steps, rejects implausible jumps, and renders permanent 1.3 mm strokes.
The sheet stays rigid and fixed. Friction is a uniform grippy approximation.

Reset restores initial poses, removes all ink and score state, and settles the
ready robot while refreshing camera history. A same-process simulator test
verified both empty ink geometry and a blank returned observation after marking.

## Recorded runs

Runtime revision: `01b108f4fb38e7df7e1110b8a0df76346c9b1542496f44b2f8021bcc561c84ab`.

| Policy | Score | Steps | Video time | Wall time |
| --- | ---: | ---: | ---: | ---: |
| Scripted | 1.0 | 1,401 | 70.10 s | 165.23 s |
| Dream-1 beta | 0.0 | 2,400 | 120.05 s | 522.77 s |

Both completed without trial errors, using the same source revision, cameras,
physics, seed 0, zero jitter and 2,400-step limit. The recorded videos and submission text were reviewed and approved for
submission. The uploaded demo, Dream run and setup image are recorded as
media `M-XSHMV8` in `task.json`; synchronized wrist views are retained locally.

The native top-camera pose is approximately `(-0.166495,-0.009000,1.541821)` m,
quaternion xyzw `(0.212629,-0.212629,-0.674380,0.674380)`, OpenGL convention.
At 640×360 its intrinsics are fx=392.195618, fy=391.722351,
cx=318.389435, cy=177.876312. The exact calibration and result records are in
`validation.json`. Physics and reset checks were performed before the camera
revision; their physics inputs are unchanged. The recorded demos demonstrate
full scores under the final camera configuration.

Run the seven scorer checks after installation with `pytest -q test_ink.py`.
They cover complete/partial answers, wrong-answer marks and dots, stationary
jitter, hovering/gaps, disconnected dashes, out-of-box marks and large jumps.
