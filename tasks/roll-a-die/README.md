# Pick up and roll a die

Category: **Societal relevance**. Scorer: **roll-a-die-v3**.

Pick up the small coloured die and roll it on the table. Release it so a marked
face turns from facing up to facing down while free, then leave it untouched
until it stops. Handling game pieces is one component of helping someone
participate in tabletop games. No particular face value, randomness or
statistical fairness is required.

**Validation status:** the final v3 demonstration scores **1.0** after
182 actions / **9.10 simulated seconds**, with clearly visible face-turnover
evidence. The final Dream run scores **0.0** after its full **1,200 actions /
60 simulated seconds**, without a qualifying pickup. Both final runs use the
same frozen scene, scorer and core source. Media is prepared; upload is pending.
Earlier blank-cube trials are historical evidence only, not results for this
marked-die scene and revised rubric.
See [JUDGING.md](JUDGING.md) for the review protocol and
[VALIDATION.json](VALIDATION.json) for versioned evidence.

## Fixed setup and scoring

The only loose object is a **16 × 16 × 16 mm, 5 g die**, with visually distinct
coloured faces and pips. Opposite faces sum to seven. Face markings identify the
same physical face through a roll; they do not change the collision shape or
assign a requested outcome.

| Local face | Pips | Colour |
|---|---:|---|
| +Z | 1 | White |
| −Z | 6 | Yellow |
| +X | 2 | Red |
| −X | 5 | Cyan |
| +Y | 3 | Green |
| −Y | 4 | Purple |

The die starts at world `(0.20, 0.16, 0.7555)` metres, identity orientation,
on the calibrated YAM station's horizontal oak-coloured table. The tabletop is
1.10 × 1.30 m, centred at `(0.36, 0)` m, with surface height `z=0.7475` m.
World X points away from the arm bases, Y left and Z up. Both arms start in the
station ready pose; there is no object jitter. The review-only fixed close-up camera records at 1600 × 1280 pixels from
`(0.84, -0.25, 1.55)` m towards `(0.25, 0.17, 0.90)` m, with a 40 mm lens and
20.955 mm aperture. It does not add a model observation. A render-only tabletop
ruler spans X=0.14–0.42 m at Y=0.275–0.292 m; a vertical guide spans X=0.102–0.138 m,
Y=0.295 m, from the tabletop to 207 mm above it. Both use 5 mm ticks and
10 mm major ticks; the vertical guide labels 0, 3, 10 and 20 cm and highlights
30 mm in red. These guides have no
collision or support geometry. Perspective must be considered when comparing
objects outside a guide’s plane.

| Score | Requirement |
|---|---|
| 0 | No qualifying pickup, or a permanent invalidation. |
| 0.25 | Hold the die with its bottom at least 30 mm above the table for 0.20 s; the measured holding TCP is less than 35 mm away and its normalized opening is below 0.30. |
| 0.5 | After pickup, release it with its bottom at least 10 mm above the table and both gripper TCPs at least 50 mm from its centre. |
| 1.0 | After release, a named face is seen pointing up within 30°, then the same face points down within 30°; the die subsequently rests on a face for one continuous second with both TCPs at least 60 mm away. |

Rest means position stays within 2 mm and full orientation, including yaw,
stays within 5° of a fixed pose at the start of the rest interval,
with a face within 10° of level and the bottom within 3 mm of the table.
After release, either TCP entering the 50 mm clearance or the die leaving the
permitted tabletop area permanently erases all credit. Report the final score,
not its historical maximum. Exact dwell, pose and boundary rules are in
[scoring.py](scoring.py) and the [judging protocol](JUDGING.md).

A TCP is the tool centre between the gripper finger pads, 13 mm short of the
pad tips along the approach axis. The visible
up-to-down face sequence requires a substantial turnover. Flat yaw
spin, repeated small rocking, turning while held, or a final pose alone cannot
establish it. A full revolution can qualify even if it returns to its starting
orientation. When the downward face is hidden underneath the die, its known
opposite face pointing upward is equivalent evidence; preserve continuous
footage and the fixed opposite-face mapping.

## Run and reproduce

Use the competition's Brev simulation launchable with the Isaac Lab-Arena/YAM
environment. Configure the Dream SDK credential securely on the VM, outside
Git and logs. Run from the repository root and use a new directory for every
attempt; the runner refuses to overwrite an existing trajectory.

```bash
source ~/IsaacLab-Arena/yam.env
python -m unittest discover -s tasks/roll-a-die -p 'test_*.py' -v
python tasks/roll-a-die/demo_policy.py --check

python tasks/roll-a-die/run.py --policy demo --max-steps 1200 \
  --seed 0 --log-dir artifacts/die-v3-demo-new

python tasks/roll-a-die/run.py --policy dream --max-steps 1200 \
  --seed 0 --log-dir artifacts/die-v3-dream-new

python tasks/roll-a-die/replay.py artifacts/die-v3-demo-new/trajectory.jsonl
python tasks/roll-a-die/replay.py artifacts/die-v3-dream-new/trajectory.jsonl
```

Both policies receive the same task instruction, scene, seed, scorer, control
rate and joint/gripper velocity limits. The limit is **1,200 actions / 60
simulated seconds** for either policy. The fresh Dream run uses the standard
direct integration; `--dream-server-url` is omitted. The retained optional
`dream_bridge.py` supports the earlier batched SSH transport, but is not required
for direct execution.

The offline policy check verifies reachable targets and joint limits; it does
not prove a grasp or roll. The scripted policy commands joints and grippers
only. It never moves the die directly or changes the scorer. Keep the scene,
scorer and action limits fixed when comparing policies. Scene or rubric changes
require newly identified evidence, rather than relabelling old recordings.

Tests and trajectory replay can run locally without a GPU. Replay needs Python
and NumPy. The complete test suite also uses Inspect Robots, the Dream
integration and `dreamlabs-client`; the policy check needs the YAM adapter.

## Runtime and artifacts

Control runs at **20 Hz**, physics at **240 Hz**, and the original three policy
camera frames are **320 × 176 pixels**. The separate **1600 × 1280** close-up is
for review only; preserve
the complete trial and the setup image. A trial's control duration is not its
wall time: simulator startup, rendering and model requests also take time.
The final demonstration used **44.457 seconds of wall time**; final Dream used
**184.133 seconds**.

| Current v3 run | Score | Actions / simulated time | Wall time |
|---|---:|---:|---:|
| Demonstration 002 | 1.0 | 182 / 9.10 s | 44.457 s |
| Dream 002 | 0.0 | 1,200 / 60.00 s | 184.133 s |

The demonstration's pickup was credited at 4.60 s, release at 7.85 s and
turnover at 8.00 s. The cyan five-pip face is visible upward at 7.85 s; its
opposite red two-pip face is visible upward at 8.00 s, establishing cyan down.
Rest is continuous from 8.10 to 9.10 s. Maximum tilt excursion of 160.551° and
cumulative motion of 272.987° are diagnostics, not alternative scoring rules.
Dream ran directly on the VM: **25 requests, 25 attempts, one continuous
session and zero retries**. The model reported 7.981 s of inference work, not a
billed dollar amount. It returned **380 prediction rows containing values
outside action bounds**. This is a prediction count, not an executed-action
count: the returned stream also includes 32 unused rows from the final chunk.
`DreamPolicy` returns raw predictions; Inspect's standard execution bounds clamp
(`ClampApprover`) and per-step limit (`DeltaLimitApprover`) review actions before
execution, and the adapter clips to bounds again defensively. An independent
audit found **all 1,200 executed Dream actions and all 182 demo actions within
bounds and per-step limits**. The recorded result includes this standard
execution behaviour. Thirty-five scorer tests and ten bridge tests pass.

The initial deployed environment used these revisions; every new run records
its actual packages and source hashes in `runtime.json`:

| Component | Version or revision |
|---|---|
| YAM Arena adapter | `0059d02` |
| Isaac Lab-Arena | `c7edf3a9` |
| Isaac Lab | `91a0906` |
| Isaac Sim | `6.1.0.0` |
| Inspect Robots / YAM | `0.60.0` / `0.37.0` |
| Dream integration | `b5e936a201cec66c54c297e522c920d98445e339` |
| Dream client | `1.0.14` |

Each trial saves:

- `trajectory.jsonl`: measured die pose/bounds/velocity, measured hands and
  joints, timestamps and scorer diagnostics at each control step.
- `score-summary.json`: scorer version, final score, events and any invalidation.
- `setup.png` and `setup-closeup.png`: the settled initial scene and review
  close-up before the first policy action.
- `review.mp4`: the complete close-up streamed at 20 Hz to H.264;
  `review-video.json` records frame timing and `visuals.json` records face
  markings and the camera configuration. These review artifacts are not model
  observations.
- `runtime.json` and `task-source/`: actual runtime, configuration, package
  versions, source hashes and the Python source used by that trial.
- The Inspect evaluation and recorded camera frames, from which complete videos
  are rendered. The task score is also in
  `samples[0].epochs[0].task_score`; evaluation status `success` only means the
  evaluation completed without an infrastructure error.

Replay uses measured states, not policy targets or saved per-frame scores.
Use a trial's archived scorer when reproducing its historical result. Tests
establish properties of the rubric and transport, not physical realism,
repeatability or competition acceptance.

## Historical v1/v2 evidence

The earlier scene used an unmarked blue cube. V1 accumulated post-release
rotation and could reward repeated small rocking. V2 changed that criterion to
at least 90° maximum tilt excursion from a verified free-flight reference.
Existing measured trajectories were rescored offline; the physics recordings
were not changed. [VALIDATION-v2.json](VALIDATION-v2.json) preserves the previous
manifest separately from the current validation record.

| Historical trial | V2 score | Max tilt excursion | Original wall time |
|---|---:|---:|---:|
| Partial demonstration 003 | 0.25 | 29.656° | 46.005 s |
| Demonstration 004 | 1.00 | 133.207° | 321.206 s |
| Clean Dream baseline 002 | 0.00 | 0° | 197.996 s |

The historical demo completed in 9.30 simulated seconds. The historical Dream
trial used 60 seconds, 25 requests, one session and no retries. Its preceding
attempt had repeated session expiry and is not a clean model baseline. These
numbers explain development decisions; **none is a current v3 result**. V3 adds
recognizable faces, review footage and a simpler pose-based rubric, so the new
scene and both policies were recorded afresh. An initial v3 camera angle
hid the qualifying upward face, despite a numerical full-score demo. Those
first v3 runs are development evidence only; the final camera is raised to
show the face sequence. The final demonstration above uses this raised view;
the matching final Dream recording uses the same frozen task source.

## Media and submission

First export the three policy views from the actual evaluation JSON, then
build a complete review video from the trial directory:

```bash
inspect-robots video RUN_DIR/EVALUATION.json --out RUN_DIR/media --fps 20
python tasks/roll-a-die/render_review.py RUN_DIR --output OUTPUT.mp4
```

The renderer uses `review.mp4` and the three standard per-camera MP4 files in
`RUN_DIR/media`. Replace `EVALUATION.json` with the run's actual evaluation
filename. Missing camera videos produce an error with export instructions; the
review renderer does not export them automatically. The result combines the full
1600-pixel review view with 640-pixel policy views; it preserves the complete
recorded trial and does not invent intermediate frames. Use a different output
path for each trial.

Include the complete Dream trial, a full-score demonstration when Dream does
not reach 1.0, and the actual setup screenshot. Select views that show the
pickup, hand clearance, named-face turnover and final rest. Close-ups or slow
motion supplement the full recording; they must not invent missing frames.
Videos must be H.264 MP4, at most 50 MB each, with browser-compatible fast start.
The setup PNG must be at most 10 MB. Text and media must not identify the team.

Only report measured scores and upload the final verified media. The media ID
returned by `tools/upload_media.py` belongs in `task.json`. Media upload and a
ready task folder do not by themselves authorize opening a competition PR.
