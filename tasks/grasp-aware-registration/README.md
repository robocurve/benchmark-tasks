# Register a windowed plate behind an inspection screen

Transfer the passive registration plate from the blue support to the white board,
centre both illuminated cyan crosses in its pink-cornered windows, release it and
leave it still for a complete second. The fixed scene has no locating pins, snap
attachments or prescribed grasp. Exact construction data are in scene.json and
prototype/layout.py; the public instruction and score bands are in task.json.

The central and end handles can both carry the plate, but leave different views
through its windows. A fixed opaque screen hides the marks from the original
overview. The intended capability is choosing grasps and motions that preserve
later visual feedback, or recovering when feedback is insufficient. Alternative
camera routes, pre-observation/memory, half-turns and supported regrasp are legal.
Only the final physical outcome earns points; no arm, camera or recovery is required.

## Recorded evaluations

| Policy | Score | Actions | Simulated time |
|---|---:|---:|---:|
| Dream-1 beta, one formal trial | 0.0 | 2400 | 120 s |
| RGB-conditioned supported recovery demo | 1.0 | 1269 | 63.45 s |
| RGB-conditioned direct continuation reference | 1.0 | 575 | 28.75 s |

The formal Dream rollout made small robot/plate contacts but did not lift or
transport the plate; maximum plate translation was0.0036mm. All original frames
and raw scoring records passed validation. It returned50action chunks in51SDK
attempts,including one automatic session-expiry retry within the same trial.

Both reference routes succeeded in two independent resets of the same fixed layout.
The submitted full-score video shows the recovery route: inspect, return to the
blue support and release, regrasp at the end, visually refine, release and settle.
At17.75s,20 native wrist frames choose a known route using the visible alignment
features. Coarse paths and initial grasp are specified; final XYfeedback uses RGB,
calibration, measured robot joints and commanded/fixed plane heights, not live
object truth. This is a specialized reference controller, not autonomous grasp-pose
planning. The real Dream branch installs no scripted controller or observer pose.
One Dream result does not isolate a causal capability failure.

The recovery ends at2.947mm error (0.053mm within the3mm full-score threshold).
Deterministic repeatability is demonstrated; randomized robustness is not claimed.
The direct reference ends at2.240mm. Each has a complete released, supported,
still final second. The renderer's three original RGB streams were retained.

Validation also included55 offline checks, raw archived-scorer replay, a2400-action
stationary RGB/physics check, and an independent real held-negative run. In that
negative, the final second remained aligned within1.881mm, seated and still, but
actual robot contact persisted at all241 physics endpoints, correctly giving0.
Diagnostic contact records are not a separate whole-episode safety guarantee.

## Run in the official pinned environment

Use the original Isaac Sim / IsaacLab-Arena YAM station. Required source pins and
package versions are in runtime-lock.json; runtime_check.py fails on mismatches.
No absolute workspace paths or external task assets are needed. The launchers do
not install packages, change drivers or alter system settings.

From this folder, use the pinned Arena Python3.12 and new empty output directories:

```bash
python runtime_check.py
python run.py --mode setup --gpu auto --log-dir /tmp/registration-setup
python run.py --mode adaptive_visual --initial-grasp central --gpu auto --log-dir /tmp/registration-demo
python validate_run.py /tmp/registration-demo
python export_media.py /tmp/registration-demo /tmp/registration-media --name fullscore_demo
```

Use `--initial-grasp end` for the direct visual reference. All modes except `dream`
are free local scripted modes. Default budget is2400actions (setup40/view120).
No object jitter is applied; one deterministic starting layout, no variable seeds
or layout lots. Reload/reset between episodes; no consumed materials.

Dream requires standard Dream SDK credentials and an explicit paid run:

```bash
python run.py --mode dream --steps 2400 --config-only --log-dir /tmp/registration-config
python run.py --mode dream --steps 2400 --gpu auto --log-dir /tmp/registration-model
```

The first command only records and verifies configuration, with no client,
simulation or inference. The second is paid. Fixed checkpoint dreamlabs/dream-1-beta,
48rows/request,32preroll actions,record=false,dry_run=false,300s request timeout.
Control20Hz, physics240Hz, original three320x176RGB cameras and robot controls.
The scene and physics run locally; necessary RGB history and robot state go to
Dream inference and returned actions drive the local robot. record=false disables
extra cloud recording, not inference input transmission. No policy object-truth
input, scripted observer pose, warmup or demo-only termination delay is added.

Trials receive at most2400steps/120simulated seconds and can end early on full
score. Wall time depends on rendering and API latency. The outer wall caps are
900s for free runs and2400s for Dream. Automatic GPU selection requires at least
8GiB free and does not reserve it or stop other workloads. The image guard rejects
three near-uniform overview frames; legitimate blank wrist views are allowed.
Native completion/error, complete frame counts, image health, archived source
hashes and raw scorer replay are checked; process exit0 alone is insufficient.
Videos keep20fps and include the initial frame (+0.05s), with overview above wrists.

## Construction, physical rubric and limitations

The120g rigid plate has two30x10mm actual openings and identical34x28x20mm raised
handles. Pink corners are physical visual references. The boards and screen/posts
are fixed. The goal-cross pair is offset(-10,-15)mm from the board centre; centring
the board outline alone misses the partial-credit band. No material is transparent
only to the policy; all camera images are original rendered RGB.

See SCORING.md for the exact outcome-v2 rubric: over the final second, both bottom
window centres within3mm for1.0 or12mm for0.5, each within2mm of the board plane,
plate tilt<=2deg, continuous board support and no robot contact, position envelope
<=1mm and rotation envelope<=1deg over21poses. Otherwise0. The two identical crosses
can exchange identities. Wrong/held/unsupported/transient outcomes get no credit.
The score uses the ending window, not the best earlier state; invalid data cannot
be a valid task result. Scoring thresholds were unchanged for the formal runs.

The fixture approximates rigid fabricated parts: plate/handles,flat boards and an
opaque screen. Plate friction0.9static/0.7dynamic,zero restitution; other contacts
use pinned official defaults. The cyan surfaces idealize a continuously powered
backlit registration panel with emissive materials. Brightness,glare,compliance,
wear and material properties have not been hardware-calibrated. Private poses and
contacts serve as measurement instruments; they are not policy observations.

For physical grading, inspect the final release/stillness interval and measure
registration using calibrated near-normal photos or a simple transparent scale;
use a straightedge/feeler gauge for seating and a square/inclinometer for tilt.
Resolve the bottom opening centres rather than an oblique view of the top rim.
These tools are external grading aids, not extra policy sensors or rendered
feedback. Hardware measurement uncertainty and frame-by-frame contact witnessing
have not been calibrated; no real-hardware equivalence claim is made.

## Related tasks and scope

Task-board text and focused pinned task/README comparisons were refreshed on
2026-11-09. Relevant overlap remains with visibility #130/#492, regrasp #558,
hidden alignment #739, exposing labels #744, stencil holding #758, and active
observation #575/#1027. The incremental claim is the grasp changing access to
later precision feedback, not that active perception or regrasp is new.

New comparisons: [empty-case inspection #1159](https://github.com/robocurve/benchmark-tasks/pull/1159)
opens lids to decide a destination; [jig plate #1198](https://github.com/robocurve/benchmark-tasks/pull/1198)
keys differently sized holes onto locating pins across layouts; [flexible panel #1203](https://github.com/robocurve/benchmark-tasks/pull/1203)
chooses hold points to limit bending. This scene instead uses two physically
usable handles and image-based registration without a mechanical locator.
These are scoped comparisons, not exhaustive code/video review or official
novelty approval. A finite viewpoint search cannot rule out all simpler strategies.

The task directory is the code submission candidate. The three media are uploaded
separately through the official tool; media=null is intentionally pending a real
upload ID. Local logs,verification archives,Chinese usage notes and ZIP are not
part of the task directory. No publication is implied by local preparation.
