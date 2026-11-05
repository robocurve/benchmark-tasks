# Separate nested trays and park them for reuse

Preparation status: not yet submitted. A valid completed Dream evaluation and task-board
media upload are pending; no media ID or completed model score is fabricated. Public PR
comparison alone does not establish meaningful novelty. In particular, the loose trays
can be solved with ordinary pick-and-place; private-board and broader benchmark comparison
must be completed before submission.

The exact language prompt is `task.json:instruction`; both the runner and this task load it
from that file. Scene geometry, demo, scorer and runtime tools all live in this directory.

## Run without Brev

Run these commands from this task directory. Requirements: Linux, Docker with NVIDIA GPU
support, an RTX GPU supported by Isaac Sim 6.1, and access to the NVIDIA base image and
public asset URLs. Building accepts the NVIDIA Omniverse/Isaac Sim license through the
image's `ACCEPT_EULA` setting. No viewer image or host virtual environment is required.

```bash
python3 build_runtime.py
python3 launch.py --gpu 0 --policy scripted --episodes 2
```

`--gpu` selects the host GPU at execution time; the image contains no GPU number.
`--image` can select an existing equivalent runtime. `--output` selects the output directory
(default `.runs/gpuN/`, ignored by Git). A standalone run exits when recording finishes.
The simulator runner also accepts `--keep-open` when used with a separate live viewer.

The build pins Isaac Sim 6.1.0, Arena `release/0.3.1` commit
`c7edf3a9d5d8276565208f72422f8f8a0806d6cf`, Isaac Lab commit
`91a0906f93802eb859192dcb0bca879f891e92d6`, YAM Arena commit
`0059d02a1a91bc6834c1d0846022891df0c78730`, Dream adapter commit
`b5e936a201cec66c54c297e522c920d98445e339`, and Dream client 1.0.14.
Arena's shared room assets resolve under the public Isaac 6.0 asset URL; this does not
change the simulator version. Known runtime compatibility adjustments are in the Dockerfile.

In an already provisioned matching Python environment:

```bash
python run.py --policy scripted --episodes 2 --output-dir ./outputs --livestream 0 --viz none
python -m unittest discover -s . -p 'test_*.py' -v
python rescore.py outputs/denest-trays/RUN/episode-01/trajectory.jsonl --expect outputs/denest-trays/RUN/episode-01/result.json
```

No simulator or network is needed for the unit tests or trajectory rescoring; they use
NumPy and the installed YAM data types. The scorer requires measured grippers and TCPs,
exact world root poses, consecutive step numbers and persistent trial state. Calling a
stateless object-only score function is intentionally unsupported.

## Dream-1 recording and cost control

Use an owner-readable credentials file outside the checkout (`chmod 600`), containing
`DREAM_API_KEY=...`. Use one shared budget ledger across tasks and GPUs. For a genuinely
new budget only, initialize it with `python dream_budget.py init --path /path/to/ledger.sqlite3`.
Never initialize a new ledger to bypass spending already recorded in another ledger.

```bash
python3 launch.py --gpu 0 --policy dream --episodes 1 \
  --credentials /path/to/dream.env --budget /path/to/ledger.sqlite3
python3 dream_budget.py status --path /path/to/ledger.sqlite3 --task denest-trays
```

The selected checkpoint is `dreamlabs/dream-1-beta`. Model observations are the top and
both wrist RGB cameras plus measured 14-D joint/gripper state. No object poses, score,
demo trajectory or goal coordinates are sent as privileged observations. Ground-truth
physics data is used only for scoring. Three input frames are resized by the upstream
adapter to 320x176; the local overview is not sent to the API.

The first 32 control ticks fill the observation window. Each response supplies up to
48 action rows, played at 20 Hz. An uninterrupted 800-tick episode typically uses 16
inference requests. The cost guard conservatively reserves $0.0015 for each attempted
request, before transmission, including failures. Limits: $0.03 per task across runs,
$0.60 per rolling 24 hours, $12 total, expiring 20 days after initialization. These are
local spending controls, not benchmark rules or actual account billing. They do not
cover requests sent by other clients or unverified price changes.

Every request uses a fresh HTTP/TLS connection. One server session replacement per
rollout may be recovered using the current observation, without resetting the physical
scene or the step counter. The recovery request consumes the same shared budget.
Other API errors or a second session replacement terminate the run. No task is
restarted automatically. A budget/API interruption is recorded with
`evaluation_complete: false`; it must not be reported as a completed model failure.

## Outputs and submission files

Each run writes a source snapshot, `summary.json`, `index.html`, and one directory per
episode containing four H.264/yuv420p MP4s, initial/final PNGs, `trajectory.jsonl`,
`result.json`, and (for Dream) request/budget details in `dream.json`.
The overview is 1280x720; the three robot cameras are 640x360. Video time is simulation
time (20 frames/s), excluding paused API waits; wall-clock time is recorded separately.
A scripted trial takes about 26 simulated seconds; measured wall-clock timing is in
its result file. Reset settles the fixed scene for 30 ticks before recorded control starts;
that setup interval is not part of the 800-tick model allowance. Environment seed is 0.

From the benchmark repository root, after choosing a valid model run and full-score demo:

```bash
python tools/upload_media.py tasks/denest-trays \
  --video /path/to/model/overview_cam.mp4 \
  --demo /path/to/demo/overview_cam.mp4 \
  --screenshot /path/to/model/overview_cam-setup.png
```

The official uploader reads `ROBOCURVE_TEAM` and `ROBOCURVE_TOKEN` from environment
variables and writes the returned `media` ID to `task.json`. Set `scores` to the measured
results. Upload the camera render, not a browser/desktop capture containing identity.
Videos must be H.264 MP4 and below 50 MB; the screenshot must be below 10 MB.
Keep raw videos, credentials, runtime caches and budget databases out of the PR.
Only this task directory belongs in the submission commit.
Run the local evidence check before publishing:

```bash
python3 check_submission.py --model-result /path/to/model/result.json --demo-result /path/to/demo/result.json
```

`--allow-unuploaded` skips only the media-ID check while preparing an upload. It still
rejects interrupted or mismatched model runs. This is a local evidence-quality check;
the organizers' public rules do not specify a detailed API-error policy. The checker
cannot certify novelty, physical safety, or acceptance by reviewers.

## Fixed setup

Units are metres. World x points away from the arms, y to their left, z up.
The table top is z=0.7475. YAM mounts, calibration and lights use the shared Dream station.
Both trays start centred at x=0.25,y=0; outer bottom z=0.7495, inner bottom z=0.7565.
Both start upright with zero yaw; reset has no random placement or jitter.

| Item | Body dimensions | Mass | Details |
|---|---|---|---|
| Blue outer tray | 160x180 mm, 47.5 mm maximum wall height | 160 g | 5 mm floor/walls; raised tab on +y side |
| Orange inner tray | 130x140 mm, 36 mm maximum wall height | 100 g | 4 mm floor/walls; raised tab on -y side |
| Blue bay | centre (0.25,0.23), 190x210 mm | fixed | thin boundary lines |
| Orange bay | centre (0.25,-0.23), 190x210 mm | fixed | thin boundary lines |

The complete tab, bridge and wall geometry is specified in `task.py:PARTS` and bundled
as `assets/*.usda`. `build_assets.py` regenerates it using the image's Python. All geometry
is original procedural cuboids; no external textures/models are needed. Surface friction
is static 0.8 / dynamic 0.6, restitution 0. These are approximations for dry plastic.
The outlines identify body-centre targets; the raised grasp tabs may project beyond the lines.

## Scoring

The run limit is 800 control steps (40 simulated seconds at 20 Hz). A full score must
hold continuously for 2 seconds. The evaluator reads actual physics state, not desired
waypoints. Grippers must be open beyond 80%, and each measured TCP must be at least 5 cm
outside both tray bounding boxes. A tray falling 2 cm below the tabletop at any time
latches a failure for that episode.

* **0:** still nested/overlapping; a tray airborne, moving, fallen or outside orientation
  tolerances; or a gripper still close to/holding either tray.
* **0.5:** both trays released, upright and stationary on the table, with at least 10 mm
  horizontal separation between their bounding boxes, but a full-score condition (including the 2-second hold) is still unmet.
* **1:** all partial-credit conditions, and each tray body origin within 20 mm of its bay centre.

Upright means tilt <8 degrees and yaw error <10 degrees. Stationary means linear speed
<10 mm/s and angular speed <0.1 rad/s. Grounded means bottom within -5 to +8 mm of the table.
On a real rig, measure bay-centre distances and separation with a ruler and inspect a
settled hands-clear video. Reset by nesting the trays at the stated coordinates.

## Meaning and limitations

This measures clearance-aware removal of nested lab/food-service containers and placement
for reuse. The inner tray must clear the outer rim before lateral transport. The scripted
demo steadies the outer tray, but simultaneous bimanual use is not a scoring requirement:
another valid separation strategy is accepted.

This is an example prototype, not a claim of worldwide task novelty or benchmark acceptance.
See `novelty.md` for the public PR comparison. Trays are ideal rigid compound boxes with
generous clearance. There is no vacuum seal, plastic flexure, damage or snap-fit retention.
Thus the task does not measure unsticking tightly jammed real trays. Rounded, lightweight
plastic equivalents and slow supervised motion are appropriate for real YAM validation;
real-hardware performance and safe force limits have not been tested here.

Public submission is deferred. No fabricated model score or media ID is included in `task.json`.


## Packaged-run validation

The packaged runner was tested in run `20261105-051333-a30636`: two fixed-seed resets,
both scoring 1.0 at 519 steps (25.95 simulated seconds), with 40 consecutive qualifying
frames. Wall time was 113.72 and 95.63 seconds, excluding simulator startup. Initial
object states were identical. `rescore.py` reproduced both final scores from the recorded
measured states. All eight videos have 519 frames, H.264/yuv420p encoding, and are below
50 MB. `validation.json` records media hashes and the geometry/scorer hashes.

Twenty-four offline tests cover scoring, concurrent persistent budget accounting and
bounded session recovery. These tests made no API requests. The matching prebuilt runtime
was used for the live demo validation; the bundled fresh-image build recipe was not rebuilt
from scratch as part of this preparation. A completed Dream evaluation remains pending.
