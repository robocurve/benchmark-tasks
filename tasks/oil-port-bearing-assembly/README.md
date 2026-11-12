# Seat an oil-feed sleeve with its radial passage registered

Place the free sleeve fully into the housing and leave it released and still, with
its radial hole opening the complete housing oil passage into the central bore.
A circular sleeve can seat at the wrong angle: depth alone does not establish the
required connection. The task ends before a separate retaining operation.

The sleeve's real hole can be hidden by a grasp. Inspecting before grasping,
remembering orientation, choosing another grasp, rotating a wrist, using either
original wrist camera or regrasping are all legal. The score requires no particular
hand, camera use or motion. The supplied demonstration uses a known-layout plan
and ordinary joint feedback; it does not perform visual servoing.

## Scope and practical context

This is the registration stage of a replaceable oil-feed sleeve in a guide housing.
There is no circumferential external oil groove to make every angle equivalent.
A related real design distinction is documented in [Carr Lane's oil-groove bushing
technical information](https://www.carrlane.com/engineering-resources/technical-information/manual-workholding/drill-bushings/oil-groove-bushings):
Type 1 requires oil-hole alignment, whereas Type 2 has an external circumferential groove.
The simulated part is not a dimensional copy of that product. It does not model
internal lubrication grooves, locking hardware, operating loads, pressure sealing
or oil flow. Completion here means seated, released geometric registration.

## Fixed setup

Use the original calibrated dual-YAM station, its two six-joint arms and parallel
grippers, and its original top/left-wrist/right-wrist RGB cameras. Camera inputs
are 320 x 176, with the original 14D joint observation and action interface.
Physics runs at 240 Hz and control at 20 Hz. No added sensor is supplied to a policy.
All task geometry is authored from bundled source; no external custom USD is needed.

- Sleeve: 52 mm OD, 38 mm ID, 32 mm height, one 12 mm radial hole, mass 270 g.
  Bronze exterior/end/drill surfaces and silver-white cylindrical inner bore.
  The visual material is static from reset, adds no thickness and is not a scoring signal.
- Housing: 53.6 mm bore, 110 mm OD, 38 mm shell height, one 10 mm radial feed hole,
  eight-sector annular seat, and a four-piece 135 x 125 x 14 mm base with a central opening.
- Staging pad: 80 x 80 x 8 mm, fixed. The sleeve starts ungrasped on this pad,
  with 45-degree yaw. The housing and pad are fixed; no object jitter is used.
- Fit: 0.8 mm radial clearance. The seated sleeve top is 3 mm below the housing rim.
  This is a clearance fit, not press fitting. Gravity seating after partial entry is legal.

Exact world poses, dimensions and material/solver settings are in `scene.json`;
`prototype/layout.py` and `prototype/geometry.py` are the executable definitions.
The annulus uses 192 sectors. Subtracting a circumscribed 64-sided drill prism
preserves the complete nominal holes (maximum radial excess approximately
6.03 microns for the housing and 7.24 microns for the sleeve). Render triangles
follow the same convex hull as collision geometry; physics cooking error is not calibrated.
The opposite inner wall remains solid and visible through the aligned radial ports.
Silver-white appearance helps distinguish that wall from a blocking bronze exterior.

## Runtime and commands

Use the already installed official simulation-walkthrough environment. Exact source
commits and package versions are in `runtime-lock.json`. `runtime_check.py` reads
installed metadata and verifies the pins; it does not install or upgrade anything.
The required stack is Python 3.12, Isaac Sim 6.1.0.0, the pinned Arena/YAM/Dream
adapters, NumPy 2.3.1 and SciPy 1.17.0. Video export also uses the runtime's OpenCV
and the `ffmpeg` executable. See the [official walkthrough](https://tasks.robocurve.org/walkthrough)
for environment installation. Do not substitute a different simulator or robot.

From this task directory, set the interpreter path for your installation:

```bash
export ROBOCURVE_PYTHON=/absolute/path/to/IsaacLab-Arena/.venv/bin/python
"$ROBOCURVE_PYTHON" runtime_check.py
"$ROBOCURVE_PYTHON" run.py --mode setup --steps 40 --gpu auto --log-dir /tmp/oil-port-setup
"$ROBOCURVE_PYTHON" run.py --mode demo --gpu auto --log-dir /tmp/oil-port-demo
"$ROBOCURVE_PYTHON" validate_run.py /tmp/oil-port-demo
"$ROBOCURVE_PYTHON" export_media.py /tmp/oil-port-demo /tmp/oil-port-media --name fullscore
```

Use a new empty evidence directory each time. Each trial starts a fresh simulator
process; within-process repeated resets are deliberately refused. `--gpu auto`
selects the GPU with most free memory, requiring at least 8 GiB; it does not reserve
a GPU, terminate other jobs or alter driver/system settings. Console output is saved
beside the run directory. The wrapper validates the native result and raw score
trace; a CLI exit code by itself is insufficient.

The common episode limit is 2,400 actions / 120 simulated seconds, allowing time
for inspection, grasping, insertion and retries. The verified scripted plan reaches
full score in 450 actions / 22.5 simulated seconds. Rendering/startup and model
latency are wall time outside that simulation budget; startup speed depends on hardware.
The bundled `demo_plan.json` is known-layout joint targets, not evaluated-policy input.

## Dream-1 evaluation

The following command is a FREE offline configuration check. It neither imports a
model client nor starts simulation, reads credentials or sends inference inputs:

```bash
"$ROBOCURVE_PYTHON" run.py --mode dream --config-only --log-dir /tmp/oil-port-dream-config
```

After explicit authorization for paid inference and with SDK credentials configured,
run one real evaluation using:

```bash
"$ROBOCURVE_PYTHON" run.py --mode dream --gpu auto --log-dir /tmp/oil-port-dream
"$ROBOCURVE_PYTHON" export_media.py /tmp/oil-port-dream /tmp/oil-port-media --name dream
```

The checkpoint is `dreamlabs/dream-1-beta`, using the pinned official adapter,
48-row chunks, 32 preroll steps, `record=false`, and `dry_run=false`. No paid warmup
is performed by these scripts. `record=false` disables extra cloud recording;
normal inference still sends the original images, joint state and task instruction.
The Dream branch does not install the demo controller or load a scripted plan,
and always starts the canonical unassembled scene. The original RGB image-health
guard aborts a near-uniform overview failure before further policy requests.
Private poses/contact data are used for scoring and logs only.

## Scoring and evidence

The complete numeric rubric is in `SCORING.md`. At the ending state:

- 0: no valid one-second released, supported, stable seating interval.
- 0.5: that seating interval is achieved, but the full housing-port envelope is obstructed.
- 1.0: seating plus the full 10 mm gauge envelope unobstructed throughout the same interval.

A 21-sample interval at 20 Hz spans one second. Release and support are checked at
every physics substep. The continuous gauge uses the actual live sleeve pose and
authored convex solids; neither colour, a target-angle label nor isolated sample
rays determine the passage score. Legacy CPU scene-query diagnostics are logged
but never grade the task because they previously missed moving-body geometry.

`validate_run.py` verifies the archived source, native status, all three RGB streams,
contact coverage, continuous geometry calculation and raw ending-score replay.
It also distinguishes initialized diagnostic cases from robot assembly and rejects
scripted-controller artifacts on the Dream branch. The archive allows later replay
even if working source changes. Invalid or incomplete runs are not valid model scores.

Known-layout free demonstration: 1.0. Initialized seated-but-wrong-port control: 0.5;
this control is not a robot assembly demonstration. Force values are uncalibrated
engineering diagnostics, not a damage or load-safety certificate. The physical
manufacturing fit, retained mechanism and a hardware gauge protocol remain unvalidated.

## Recorded evaluation and publication status

Dream-1 (`dreamlabs/dream-1-beta`): **0.0**, 2,400 actions / 120
simulated seconds, `record=false`. The scripted known-layout demonstration scores
**1.0** in 450 actions / 22.5 seconds. Both use the same scene, scorer, reset and instruction.
The formal model run and free demo passed archived-source/raw-score replay and
three-camera integrity checks. This single trial is not a success-rate estimate or
a causal attribution of any failure to perception.

Serving diagnostics: 50 returned chunks required 99 SDK attempts,
including 49 automatic session-expiry retries in the official adapter.
This remained one episode with the complete action and image record. The adapter
flagged 2066 returned rows outside the station action bounds;
the native execution log contains bounded actions after the standard approval/clipping
path. It does not retain the discarded raw overshoots. No extra evaluation or paid
warmup was run. These serving/control facts limit interpretation of the single result.

The final local package includes the genuine model video, full-score demo and setup
image. Video includes the initial frame, so durations are 120.05 s and 22.55 s.
Private raw evidence is retained separately. Only reporting text (`scores` and this
README section) changed after evaluation; executable source is unchanged. The run
validator replays its archived source rather than trusting a subsequently edited file.

Submission media are uploaded under `M-LM9B8D`, referenced in `task.json`.
The model video, full-score demonstration and setup image are hosted separately
from the task source. Internal reports, raw logs, credentials and local handoff
guides are excluded from the submission.
