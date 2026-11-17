# Form the pouring spout of an unsealed gable carton

Unfold an inward paper pouring spout and leave a usable opening after releasing
both hands. This is the familiar preparation for pouring from a cap-free milk
or juice carton at home. The task begins after the outer seal has been opened and ends before
pouring; no seal-breaking strength or liquid-flow behavior is claimed.

## Fixed setup

One free carton on the original calibrated dual-YAM station. Its square footprint
is70×70mm, body height110mm, roof rise35mm, outer fin height12mm, inner paper-fin
height24mm and panel thickness0.45mm. The lower body and contents have lumped
mass520g; moving panels add2.701g at350g/m² (522.701g total). The shoulder origin is(0.18,0.05,0.8575)m. The outer
folds initially spread120°, while the inner pouring spout points inward. The
surface arrow identifies the pouring side. All poses are deterministic; no jitter.

The specified material is pre-creased liquid-packaging paperboard, nominally
350 g/m² and 0.45 mm thick, with a polymer barrier lining; the lining is part of
the nominal sheet, not a separately simulated layer. These are design specifications,
not measurements from a purchased carton. See [PHYSICAL_SETUP.md](PHYSICAL_SETUP.md)
for the complete printable A3/A4 blank, cut/crease/glue map, assembly, fresh-specimen
pre-creasing and mass procedure. Digital geometry checks are supplied; no physical
specimen has yet been fabricated. See [REVISION.md](REVISION.md) for evidence scope.

The original three320×176RGB cameras, joint observations, robot controls and20Hz
action interface are retained. Native physics runs at240Hz. There are no extra
policy sensors, added tools, fixtures or altered robot/camera calibration.

## Run

Use the pinned official walkthrough runtime in `runtime-lock.json`. `runtime_check.py`
checks installed source revisions and versions without launching a simulator.
Use its Python interpreter as `python` below. No installation or upgrade is performed
by these scripts. Each trial needs a fresh process and a new output directory
outside this task folder. A single GPU is selected by available memory; this does
not reserve capacity against other users. Renderer or contact-buffer errors
invalidate a run; inspect `validation.json`, not only the process exit status.

```bash
python runtime_check.py
python run.py --mode setup --steps 40 --gpu auto --log-dir /tmp/carton-setup
python run.py --mode demo --steps 1200 --gpu auto --log-dir /tmp/carton-demo
python export_media.py /tmp/carton-demo /tmp/carton-media --name fullscore-demo
```

The known-layout demonstration uses the bundled `demo_plan.json` and ordinary
joint feedback. It is a feasibility controller, not a general image-based policy.
A60-second demo allows a long final released hold. Task budget is120seconds or
2,400actions; every mode scores the ending state. See `SCORING.md` for numeric
partial credit and the three-second sustained-opening/support/release requirement.

The following first command is offline and free. The second sends original
images and joint state to Dream and incurs API charges; run it only with explicit
approval and configured SDK credentials. `record=false` disables additional
cloud recording, not inference data transfer.

```bash
python run.py --mode dream --config-only --log-dir /tmp/carton-dream-config
python run.py --mode dream --steps 2400 --gpu auto --log-dir /tmp/carton-dream
```

Dream uses `dreamlabs/dream-1-beta`,48rows/request and32preroll actions. It loads
no demonstration controller, trajectory or private pose/contact information.
The launcher, task metadata and scripted run share one instruction. The formal
model result and media ID are recorded below and in task.json.

## Evidence and scoring

Each run archives source and configuration, all three original RGB streams,
240Hz body states and18-body robot contacts, support contacts, crease-memory
history, and measured camera poses. Private evidence is never added to policy
observations. A three-frame grey-overview detector stops a lost renderer before
further policy actions. `validate_run.py` independently replays `replay_score.py`.
`audit_camera_clearance.py` checks the actual camera path against conservative
paper surfaces; a20mm sphere bound includes the original10mm near plane.
The media exporter retains every recorded frame at20fps, with the enlarged
overview above the two wrists; labels remain outside the camera pixels.

```bash
python validate_run.py /tmp/carton-demo
python audit_camera_clearance.py /tmp/carton-demo
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q tests
```

## Fidelity and reset

Rigid facets and twelve native hinge constraints approximate folded paperboard.
Rendered paper strips and their native collision shapes agree, with omitted
0.5mm crease zones. Robot/table contacts remain enabled. Internal sheet-to-sheet
contacts are omitted in this closed-loop fold surrogate. Buckling, crushing,
tearing, glue failure and fluids are not modeled.

Passive elastic-perfectly-plastic crease memory uses stiffness0.0005Nm/degree,
yield torque0.001Nm and damping0.00001Nm·s/degree. The rest angle changes only
when measured deformation exceeds the2° elastic range. There are no target-open
angle commands, task-state triggers, object-pose writes or post-release latches.
The material parameters are uncalibrated. Task-body solver iterations192/16
resolve the fold loop; the original robot solver settings are retained. Only
single-scene GPU buffer capacities are reduced; contact overflow invalidates
rather than simplifies a run. These limitations are part of the task definition.

Reset reconstructs every panel and initial crease rest angle. A physical repeat
uses one fresh custom specimen and the specified one-cycle preparation protocol. The24mm extended inner paper
edge is a chosen design dimension, informed by the variant in
[Elopak US5816487A,figures8–11](https://patents.google.com/patent/US5816487).
It is not claimed to be a measured marketed carton or validated paper material.
The material and physical-preparation specification does not establish measured
paper stiffness, reproducible crease forces or fluid-tightness. No physical
prototype has been tested. [PHYSICAL_SETUP.md](PHYSICAL_SETUP.md) separates those
remaining transfer uncertainties from the reproducible simulation reset.

## Runtime resource settings

The launcher disables RTX geometry streaming for its own process with
`/UJITSO/geometry=false`, retaining the same renderer, full meshes, lighting and
original camera calibration. This avoids missing-scene frames observed under
shared GPU load; it changes no persistent setting. Per-run renderer settings are
archived. Single-scene contact buffers are262,144 pairs/contacts and65,536 patches;
solver settings and timesteps are unchanged, and any SDK overflow invalidates
the run. These are resource settings, not force/contact tolerances.

## Verified local demonstration

The independently copied source completes1,200actions/60seconds with official,
live and raw-replay score1.0. It saves1,201frames per original camera and14,520
physical samples. The final released, table-supported opening holds21.2875s;
maximum final20-second panel drift is0.0044mm/0.0381°. Peak crease separation
is0.1916mm. No interarm or robot–table contact was reported. The minimum left
camera distance from paper is50.77mm; it never enters the carton envelope.
This is one verified final-trajectory run, not a measured success rate or a
validated paperboard force model. The formal Dream-1 run scores0.0 after2,400actions/120seconds.
It uses dreamlabs/dream-1-beta with record=false; no scripted plan or private
object-state observation is provided to the policy. No robot–carton contact was recorded and the carton was not opened.
The SDK returned50 chunks over103 attempts,
recovering53 session-expiry refusals within this one episode.
There was no second trial. The cause of frequent session expiry is unconfirmed;
this should not be described as an uninterrupted server session.
All3x2,401 original camera frames and28,920 physical samples were verified;
official, live and archived-source replay scores agree. The simulation, control and instruction match the full-score demonstration.
Revision3 regrades both complete raw recordings with the sustained-opening
rubric; final scores remain1.0 and0.0. These are regraded existing episodes,
not new model calls or evidence of the revised scorer running in the old process. This one model run
does not establish a failure rate or a causal explanation for the behavior.

## Revision verification

The original task geometry, dynamics, input instruction and camera streams are
unchanged. [REVISION.md](REVISION.md) explains the full blank, mass clarification,
revised sustained functional-opening rule and preserved-run evidence.
[GRADING_PROTOCOL.md](GRADING_PROTOCOL.md) defines external physical grading
views and uncertainty handling, without adding model sensors.

```bash
python make_blank.py
python replay_score.py /path/to/original-run --output /tmp/regraded.json
```

Blank generation is an offline documentation operation using numpy, scipy,
matplotlib and shapely from the supplied runtime. Pre-generated PDF/SVG files
need no software to print. Regrading to a separate output preserves the original
run's source and score records. Do not run the current validate_run.py against
an old-source directory: its live/source agreement check deliberately rejects
a different scorer. Use archived original source for original-run validation.

## Asset sources and licenses

| Asset | Source and terms |
|---|---|
| Carton facets, paper strips, front arrow, blank and assembly drawings | Procedurally authored in this task's geometry/drawing code; no downloaded carton mesh, texture, scan or commercial CAD. |
| YAM robot meshes / URDF / collision geometry, including split black/white link meshes | I2RT Robotics, MIT; original asset source commit `120c3c81400171174604e503943f8d1ebc891058`, carried by the pinned official walkthrough adapter. The complete notice is included in [ASSET_LICENSE_I2RT.txt](ASSET_LICENSE_I2RT.txt). |
| Table, floor, camera stand and lighting | Procedural boxes/cylinders and lights from the pinned adapter's station room; solid colors, no external table/room texture or HDR image. No library fruit or other downloaded scene prop is used. |

The robot provenance and notice are also recorded in the pinned adapter's
[asset directory](https://github.com/kairosiann/yam-isaac-lab-arena/tree/0059d02a1a91bc6834c1d0846022891df0c78730/yam_arena/assets/yam)
and [README](https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/README.md).
The asset MIT notice applies to the I2RT assets; it is not a claim that the
entire adapter or NVIDIA runtime is MIT-licensed. No dependency code or robot
meshes are redistributed in this task folder. Runtime dependencies retain
their own terms and exact revisions in [runtime-lock.json](runtime-lock.json).
The carton patent linked above is a design reference, not a licensed digital
asset or a claim of exact product reproduction.
