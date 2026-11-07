# Carry an opaque collar around a curved guide without contact

Move the orange-handled closed collar from the blue support to the green support,
leaving the guide inside its tunnel at completion and avoiding prohibited contacts. Release it
and leave it still. Any safe strategy is valid; camera motion and named arm roles
are not scoring requirements.

The capability of interest is maintaining useful views as manipulation changes
occlusion. The collar has an80 mm deep opaque tunnel, so an informative view into
its clearance changes direction over the110-degree route. This is a precision
bench fixture, not a claimed industrial installation. Ring-on-wire handling is a
known mechanism; the proposed distinction is the changing occluded observation.
See NOVELTY.md for related tasks and limits to this claim.

## Official runtime and independent execution

Activate the official walkthrough's Arena Python environment. This folder is
self-contained; no parent development scripts or absolute author paths are needed.
Use a new output directory for each run. The runner checks exact package/source
pins, preserves all evidence and validates the real result rather than exit alone.

```bash
python runtime_check.py
python run.py --mode setup --log-dir /tmp/guide-setup
python run.py --mode demo --log-dir /tmp/guide-demo
python validate_run.py /tmp/guide-demo
python export_media.py /tmp/guide-demo /tmp/guide-media --name fullscore_demo
# Free negative control and full-budget stationary check:
python run.py --mode lift_probe --log-dir /tmp/guide-contact
python run.py --mode stability --log-dir /tmp/guide-stability
# Optional software checks in the same pinned environment:
PYTHONPATH=. python -m pytest -q tests
```

See ENVIRONMENT.md and runtime-lock.json for installation and source provenance.
The original calibrated YAM station, robot, cameras and controls are unchanged:
20 Hz14-D actions,240 Hz physics, three320x176 RGB policy streams. One fixed reset,
zero object jitter, original READY arm positions/open grippers. No new sensor is
provided to the evaluated policy. Rendering uses the stock pipeline, including its
existing driver-dependent DLSS fallback. Task geometry uses local primitive solids.

The default formal budget is2,400 actions/120 simulation seconds. Success ends early.
The first verified run reached full score in872 actions/43.6 s. Demo capture now
continues through withdrawal to at least49 s, checking the score throughout; Dream
has no such recording delay. The demo uses scripted Cartesian
waypoints, inverse kinematics and image gates. Startup/rendering make wall time
longer than simulated time. The runner caps a free run at900 wall seconds and a
Dream run at2,400 wall seconds; timeout is an incomplete run, not a scored success.

## Scoring and physical grading

Read **SCORING.md** for the complete contact-witness-v3 contract. Numeric constants
are in guide_task/rules.py. The public protocol is generated from that source.

Any forbidden contact leaves an irreversible failure witness, including brief
contact; there is no100 ms tolerance. In simulation this is private contact-event
memory, sampled after every physics substep. In a physical reproduction it can be
checked with transferable contrast coating and clean before/after close photos,
including the tunnel interior. Real coating sensitivity, attribution and handling
must pass light-touch/no-contact controls. No real paint test is claimed here.

All nonzero scores require threading through both tunnel openings and a full final
second with a centre-motion envelope<=1 mm and rotation envelope<=2 degrees.
1.0 requires goal-centre error<=7 mm, tunnel-axis alignment<=7 degrees and flat-face
tilt<=3 degrees throughout that second, green support throughout and no robot/collar
contact during it.0.75 is the same goal interval without full release/support.
Otherwise0.5/0.25 require angular distance to goal<=38.5/82.5 degrees. All other
outcomes score0. Invalid instrumentation makes a trial invalid, not a model failure.
Equivalent square-collar faces and closed empty grippers are allowed.

The private monitor covers18 robot bodies, the collar and77 fixed collision shapes.
Prohibited: robot/fixture, interarm, nonfinger robot/collar and collar/guide or anchors.
Finger/collar and collar/support/base contacts are allowed. Native same-arm adjacent
link contacts and station/table contacts are outside this task-specific rubric.

## Geometry and fidelity

scene.json lists every part, exact transform/color, physical-material analogue and
collar physics. guide_task/layout.py constructs the same fixed geometry. The guide
is72 overlapping rigid8x8 mm segments on a90 mm radius semicircle; supports are
60x80 mm. The80 g collar has a48 mm square aperture,80 mm depth,60 mm outer section,
four6 mm walls, orange handle/bridge and eight pink visual corner markers.
The simulation does not model pigment deposition, deformation, damage, loose bolts,
coating thickness or wear. A force-bearing PhysX contact can precede visible touching
within its contact margin. Physical marking equivalence is an idealization disclosed
in task.json; no calibrated hardware correspondence or general safety guarantee.

## Demonstration and observation limits

The demo uses known-layout waypoints, not private object poses/contact flags. During
travel it checks original left-wrist RGB for three/four corner markers, an adequate
opening projection and guide pixels inside. A missing fourth corner may be inferred
from three near-orthogonal observed corners. Loss of a usable view pauses route
progress; three fresh usable frames allow resumption. The scheduled observer transit
occurs while the collar is held stationary. This is an image-gated scripted route,
not a learned visual policy, general visual servo or a3-D clearance estimator.

A counterfactual test of the actual controller advances with a real wrist image and
holds with that image blacked out, without importing the private scoring runtime.
The nominal safe run does not itself trigger a visual pause. The fixed-view diagnostic
shows a changing view; its later interarm collision must not be attributed solely to
perception. No exhaustive proof that every fixed-view or alternative-grasp strategy
fails is claimed. Any such safe strategy still earns the appropriate physical score.

## Dream evaluation — only after authorization

The same runtime/contact/scorer extension is retained. The scripted controller and
RGB gate are not installed in Dream mode. The official fixed adapter uses
`dreamlabs/dream-1-beta`,20 Hz control,48-row chunks,32 initial history steps,
`record=false`, `dry_run=false` and300 s request timeout. The simulation runs locally;
the SDK transmits the original images and robot state for remote inference. No scene
archive or source upload is required. record=false does not disable inference inputs.

```bash
# Free configuration inspection; no physics, SDK client or network call:
python run.py --mode dream --config-only --log-dir /tmp/guide-dream-config
# PAID: execute only after the user authorizes the real trial:
python run.py --mode dream --gpu auto --log-dir /tmp/guide-dream
python export_media.py /tmp/guide-dream /tmp/guide-media --name model_run
```

Use the SDK-supported DREAM_API_KEY environment or ~/.dreamlabs/credentials; never
put keys in this folder, logs or command arguments. Dream dry_run is not a free test.
The valid formal Dream-1 run scored **0.0 in 2,400 actions/120 s**, with 2,401 original
frames from each camera and valid image/contact monitoring throughout. It made a
small collar displacement (16.4 mm net) but did not reach the green target (131.1 mm
final centre error). There were zero prohibited contact reports. 50 chunks returned
through 51 SDK attempts, including one retry. This single result does not establish
that perception alone caused the failure. The full-score video remains the reviewed
1.0/980-action/49s RGB-gated known-layout demonstration.

An earlier attempt was interrupted after rendering failed and is excluded from the
reported model score/video. The final run used the same task physics, instruction
and scoring, with the image-integrity guard below. No additional sensors or scripted
model actions were introduced. Media is null until an authorized official upload;
a real media ID must be filled in before publication.

`--gpu auto` chooses the GPU with the most free memory, requiring at least8GiB at
launch; it does not reserve capacity or terminate other processes. The image-integrity
guard aborts if the fixed overview becomes almost spatially uniform for three frames,
before another policy action can consume it. It preserves the original images and
scoring rules. Blank wrist views and a stationary textured overview remain valid.
The guard detects the observed grey-render fault; it is not a complete renderer test.

The export is H.264/yuv420p,640x576,20 fps: overview above the original two wrist
images, no crop or temporal edit. It includes reset, adding0.05 s to action time.
Source snapshots, raw RGB, native actions, substep contacts and replay validation
are saved per run. Internal evidence is separate from submission media.

## Latest related-task check

A final public snapshot contained 987 PRs; the separately fetched board contained 980
cards. New related [PR960](https://github.com/robocurve/benchmark-tasks/pull/960) moves
multiple seal segments around a covered ring groove, with handoff, seating and a
closure bar. This task instead moves one already-threaded opaque collar without
contact with its internal curved guide; the observation question is maintaining
useful views of the changing tunnel opening. Both involve curved constrained motion.
[PR950](https://github.com/robocurve/benchmark-tasks/pull/950) uses bore/shaft-step
ordering, and [PR941](https://github.com/robocurve/benchmark-tasks/pull/941) assembles
split collar halves around a captive shaft. Earlier casting-ring and buzz-wire
precedents in NOVELTY.md remain relevant. These description comparisons are not
exhaustive code/video review or official novelty clearance.
