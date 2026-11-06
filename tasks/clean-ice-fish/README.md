# Serve clean ice

Instruction sent to the policy: **Use the scoop to put clean ice into the bowl.**

The open bin contains a raw-fish prop and storage ice. The covered bin contains
serving ice. Full credit requires using the scoop to deliver at least one clean
cube, with lid-opening history and without contaminating the scoop or bowl.
The clean-source requirement is explicit. The policy must identify the covered
bin from the visual setup and work out the lid-removal and scooping sequence.

## Recorded outcomes

The exact new model instruction is **Use the scoop to put clean ice into the bowl.**

| Run | Outcome | Control steps | Simulated time |
|---|---|---:|---:|
| Dream-1 beta, explicit clean-ice instruction | 0.0 | 2,400 | 120 s |
| Retained full-score scripted physical demo | 1.0 | 1,246 | 62.3 s |
| Preceding scripted repeat | 1.0 | 1,188 | 59.4 s |

The new-prompt evaluation completed with aggregate task score **0.0**. Its final goal-success flag was `false`. The evidence records the actual outcome without substituting the scripted result.

See [the Dream evidence](evidence/dream_run.json). This recording and result were
generated with the exact instruction above.

Both scripted demos physically removed the cover, grasped the scoop, scooped
clean ice, carried it and poured it into the bowl. Each delivered `CleanIce_006`,
`CleanIce_011` and `CleanIce_019` without fish-source delivery or contamination.
These physical recordings were retained from before the one-word prompt
clarification. They already demonstrate the unchanged clean-ice goal; all
geometry, poses, physics, joint-action controller and scoring are identical.
[Demo provenance](evidence/demo_reuse_provenance.json) identifies the original
and current wording. The scripted controller uses privileged state, not a
language-model policy. Only robot joint/gripper targets are commanded during
execution; object pose writes occur only at reset.

## Fixed setup

Units are metres, kilograms and seconds, with Z up. The 1.10 x 1.30 x 0.06 m
wooden tabletop is centered at (0.36, 0, 0.7175), with its top at Z=0.7475 m.
The YAM bases are at (0, +0.305, 0.75) and (0, -0.305, 0.75). Positive Y is
the fish-bin side in the setup image.

| Object | Size, material and behavior | Starting origin (X, Y, Z) |
|---|---|---|
| Fish bin | Blue plastic; 250 x 275 x 105 mm shell, excluding side handles; fixed, open | (0.385, 0.180, 0.7475) |
| Fish | Silver textured raw-fish prop, about 203 mm long; fixed in the open bin | (0.385, 0.180, 0.7875), yaw -90 degrees |
| Fish-source ice | 60 translucent bevelled cubes, each 20 mm and 7.3 g; dynamic | Individual poses in `layout.json` |
| Clean bin | Same blue plastic bin; fixed, initially covered | (0.385, -0.180, 0.7475) |
| Clean ice | 26 cubes with the same geometry, mass and material; dynamic | Individual poses in `layout.json` |
| Clean lid | Blue rim and frosted panel, about 254 x 279 mm; 115 g; removable, no hinge or latch | (0.385, -0.180, 0.8525) |
| Scoop | Metal bucket with a 105 x 60 mm floor and open leading edge; dark 24 mm-wide rubber handle; counterbalanced, 125 g; dynamic | (0.130, 0.090, 0.7485), yaw 30 degrees |
| Bowl | White glazed open bowl; about 210 mm outside diameter, 62 mm high; fixed and initially empty | (0.140, -0.165, 0.7475) |

The lid has a 24 mm-wide bridge grip and 57 mm free space below its handle.
Both grippers can grasp the provided handles. The scripted demo uses the right
arm while the left arm stays ready; using both arms is not required for credit.
There are 92 task objects, of which 88 are dynamic (86 cubes, lid and scoop).

`layout.json` specifies every object pose, quaternion where applicable, source
identity and grasp. Labels such as `ice_source` are scoring metadata, not visual
labels shown to the model. [Station calibration](assets/base/station_layout.json)
records the original cameras and robot bases. `scene.usda` is the complete
authoring/visualization scene; live trials are built by `task.build_environment`
using the supplied Arena YAM station and the same reusable object assets.

## Score a trial

The policy runs at 20 Hz and physics at 240 Hz. The run limit is **2,400 control
steps / 120 simulated seconds**. `runtime.Reader.score()` calls
`scoring.IceScorer.update()` after every control step. Each positive candidate
level must persist for 20 consecutive control steps before it becomes the score.
Report the final score, rather than the maximum over the trajectory. A full-score
state terminates a model trial early.

| Score | Exact meaning |
|---:|---|
| 0 | No qualifying progress, invalid/nonfinite state, or unhealthy contact reporting |
| 0.25 | The lid has been opened, or a physically scooped cube is currently held clear of its source bin |
| 0.5 | At least one cube has been delivered by the scoop, but full-credit conditions are unmet |
| 1.0 | At least one uncontaminated clean-source cube delivered by the scoop, lid-opening history present, no fish ice ever in the bowl, and scoop and bowl uncontaminated |

One clean cube suffices; additional clean cubes are allowed. Fish-source,
mixed-source and dirty-scoop delivery are capped at 0.5. A cube directly touched
by either robot receives no pickup/delivery credit. Lid-opening progress can still
earn 0.25. History is latched: the lid may close later, and removing previously
served fish ice does not restore eligibility for 1.0.

The numeric predicates in the supplied scorer are:

- **Lid opened:** lid origin is more than 40 mm above its reset height, or its
  horizontal displacement exceeds 150 mm. Opening history must be true when
  full credit is evaluated; the scorer does not impose an additional action-order
  predicate beyond the physical cover and these history conditions.
- **Scooped:** for three consecutive control steps, the cube center is within the
  scoop's local cavity (`-8 < X < 113 mm`, `|Y| < 33 mm`, `5 < Z < 60 mm`),
  scoop contact occurred within two steps, the nominal cube bottom (center Z minus
  10 mm) is more than 5 mm above its source bin rim, and the robot has never
  directly touched that cube. Pickup history is then latched.
- **Delivered:** cube center is less than 77 mm from bowl center in XY and
  strictly between 7 and 62 mm above the bowl's reset base. Linear speed is at
  most 10 mm/s, angular speed at most 0.5 rad/s, no scoop contact has occurred for
  more than four control steps, and both TCPs are at least 60 mm from the cube.
  Pickup history must exist and direct-robot-handling history must be absent.
- **Contamination:** fish, fish bin and fish-source cubes start dirty. Contact
  pairs with separation at most 0.5 mm are sampled after every 240 Hz physics
  step. Dirt propagates through tracked task objects and robot bodies and never
  clears within a trial. The common table/room is excluded. Any fish-source
  cube center entering the bowl's numeric region is remembered, even if it is
  later removed. Full credit also requires that the served clean cube, scoop
  and bowl have never become dirty.

See [the scorer tests](tests/test_scoring.py) for positive and negative examples.

## Runtime and reproducibility

The evidence was collected with the supplied `yam-inspect-isaac:6.1.0` image:

```text
sha256:0e5a603c33c29eab2751be8977127a74a469f64dad70ca4bc3e50eacc3f504a8
```

Observed stack: Isaac Sim 6.1.0-rc.26; Isaac Lab source VERSION 3.0.0
(installed distribution 17.0.2); Isaac Lab Arena 0.3.0; `yam-arena` 0.1.0;
`inspect-robots` 0.60.0; `inspect-robots-dream` 0.1.0; Dream client 1.0.14.
NumPy, PyTorch, Pillow, OpenCV, USD Python bindings and FFmpeg/libx264 are also
provided by that image. The image is an external prerequisite and is not bundled
or rebuilt. These versions describe the tested setup, rather than a promise of
compatibility with arbitrary newer Isaac releases.

All authored references remain inside this folder. The custom USD task uses the
public Inspect embodiment interface in `benchmark.py`; use the bundled runners
below. Model observations contain three 320 x 176 RGB images and 14 measured
joint/gripper values. Actions are 14 absolute targets: left six joints, left
normalized gripper, right six joints, right normalized gripper. Grippers use 0
closed and 1 open. The model does not receive privileged poses or contact state.

Every run uses one fixed layout, seed 0 and no object jitter. Reset restores
authored poses and zero velocities, settles 60 ready-pose control steps, refreshes
camera frames and clears scoring/contact history. Loose fish ice can settle
about 27 mm differently across resets. Authored reset error was at most 3.1e-8 m;
the clean pile, scoop and lid passed the saved repeatability checks. Full bitwise
trajectory determinism is not claimed.

### Create a private container

From this task folder on a host that already has the supplied image:

```bash
export ICE_CONTAINER="clean-ice-review-$(python3 -c 'import uuid; print(uuid.uuid4().hex[:10])')"
export ICE_RUNTIME="${TMPDIR:-/tmp}/$ICE_CONTAINER"
python3 review_container.py --name "$ICE_CONTAINER" --runtime-dir "$ICE_RUNTIME" --gpu 2
```

The helper creates a fresh container name and new runtime directory. It exposes
only the selected GPU, mounts this task read-only, and gives the container its
own home, caches and output directory. It publishes no ports. Offline runs have
no networking. No image rebuild is performed. The helper checks for at least
16 GiB free VRAM before startup; the operator should monitor available memory
while sharing a GPU. Stop only the container named by `ICE_CONTAINER` afterward.

### No-API scene and adapter checks

```bash
docker exec "$ICE_CONTAINER" /isaac-sim/python.sh -B -m unittest discover -s /workspace/task/tests -v
docker exec "$ICE_CONTAINER" /isaac-sim/python.sh -u -B /workspace/task/run.py --mode check --steps 40 --out /outputs/check
docker exec "$ICE_CONTAINER" /isaac-sim/python.sh -u -B /workspace/task/evaluate.py --policy hold_ready --max-steps 5 --epochs 2 --out /outputs/hold-ready
```

The short hold-policy test is expected to score 0; it checks the public evaluation
interface, not task completion. The saved check ran two epochs without errors.

### Reproduce the full-sequence physical demo

```bash
docker exec "$ICE_CONTAINER" /isaac-sim/python.sh -u -B /workspace/task/run.py --mode demo --episodes 1 --out /outputs/demo
```

The script replays the supplied joint/gripper plan in `validation/ik_plan.json`
for lid removal and scoop pickup. It then uses measured grasp calibration and
IK joint targets for the physical shovel, carry, pour and withdrawal. It does not
attach or teleport objects. The demo emits `demo_report.json`,
`episode-00/episode.json`, actions, trajectory/contact evidence, setup/final PNGs
and native top/left/right H.264 videos. Read the JSON `passed` and `score` values
to assess the result.

The primary demonstration took about 233 wall-clock seconds of recorded
execution for 62.3 simulated seconds, plus simulator startup. The preceding
trial took about 222 seconds for 59.4 simulated seconds. Startup/cache and shared
hardware load add variable time.

```bash
docker stop "$ICE_CONTAINER"
```

### Run Dream

Create a separate network-enabled container with a fresh name/runtime. Set
`DREAM_CREDENTIALS_FILE` to an existing private credentials file containing
`DREAM_API_KEY=...`, with mode 600. The helper mounts that file read-only and
does not copy the key into source, Docker environment arguments or the image.

```bash
export ICE_DREAM_CONTAINER="clean-ice-dream-$(python3 -c 'import uuid; print(uuid.uuid4().hex[:10])')"
export ICE_DREAM_RUNTIME="${TMPDIR:-/tmp}/$ICE_DREAM_CONTAINER"
python3 review_container.py --name "$ICE_DREAM_CONTAINER" --runtime-dir "$ICE_DREAM_RUNTIME" --gpu 3 --dream --credentials-file "$DREAM_CREDENTIALS_FILE"
docker exec "$ICE_DREAM_CONTAINER" /isaac-sim/python.sh -u -B /workspace/task/dream_run.py --max-steps 2400 --out /outputs/dream
docker stop "$ICE_DREAM_CONTAINER"
```

Outputs are in the chosen host runtime directory under `outputs/`. The runner
records native camera video, an Inspect log/raw frames, reset checks, request
metadata and `dream_result.json`. A refusal or API error leaves
`evaluation_completed=false` and `task_score=null`; it is not converted into a
numerical aggregate. The recorded attempt used `dreamlabs/dream-1-beta`, optional
cloud episode recording disabled, and one trial. The new attempt took 823.7 wall-clock seconds including fresh-container
startup for 120 simulated seconds of control. Startup and API latency
are separate from the 2,400-step action limit. Inference observations necessarily go to Dream.

## Fidelity and assets

This is a rigid-body task. Hollow bin/scoop/bowl collision shells and physical
frictional grasps are simulated; object shapes, friction and mass distributions
are approximations. Ice does not melt, and water, temperature, bacteria and fish
tissue deformation are absent. Contamination is a persistent source/contact
proxy. The fixed fish, bins and bowl simplify the real setting. The scoop has
a visible counterbalancing end cap. See [asset provenance](THIRD_PARTY_NOTICES.md).

The recorded videos and setup PNG are stored separately through the organizer's
media uploader. The `media` field in `task.json` identifies their uploaded bundle:
the Dream recording, the complete full-score scripted demo and the
initial setup screenshot. Wrist-camera clips remain optional supporting evidence.
