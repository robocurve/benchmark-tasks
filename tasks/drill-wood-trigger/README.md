# Drill on a wood target using a second arm for the trigger

The right arm picks up a cordless drill, points its blunt bit vertically down,
and places the bit on the red target on a secured wood block. The left arm
presses the drill's separate, spring-loaded red trigger while the bit stays on
the target. A hole does not form and the motor does not run.

This is an independent task, authored in `task.py`. The task-local `runtime.py`
adds the bit, physical trigger joint, and contact sensors to the YAM adapter
inside this process. It does not modify the installed adapter or other tasks.
Use `run.py` to include that extension when loading the task file.

## Runtime and installation

Use Linux with an NVIDIA RTX GPU. If the YAM environment is already installed,
activate it and run this folder's `run.py`.

For a fresh installation, start in the benchmark repository root, using new
checkout directories:

```bash
TASK_DIR="$(pwd)/tasks/drill-wood-trigger"
ADAPTER_DIR="$HOME/yam-arena-wood-trigger"
ARENA_DIR="$HOME/IsaacLab-Arena-wood-trigger"
git clone https://github.com/kairosiann/yam-isaac-lab-arena.git "$ADAPTER_DIR"
git -C "$ADAPTER_DIR" checkout 0059d02a1a91bc6834c1d0846022891df0c78730
git clone --branch release/0.3.1 https://github.com/isaac-sim/IsaacLab-Arena.git "$ARENA_DIR"
git -C "$ARENA_DIR" checkout -B release/0.3.1 c7edf3a9d5d8276565208f72422f8f8a0806d6cf
git -C "$ARENA_DIR" apply --check "$TASK_DIR/patches/arena-production-assets.patch"
git -C "$ARENA_DIR" apply "$TASK_DIR/patches/arena-production-assets.patch"
ARENA_DIR="$ARENA_DIR" \
  DREAM_SPEC="git+https://github.com/kairosiann/inspect-robots-dream@b5e936a201cec66c54c297e522c920d98445e339" \
  "$ADAPTER_DIR/setup.sh"
source "$ARENA_DIR/yam.env"
cd "$TASK_DIR"
```

The reference environment uses Python 3.12.16, Isaac Sim 6.1.0.0, the pinned
Arena release/0.3.1 source, `yam-arena` 0.1.0, Inspect Robots 1.0.0,
`inspect-robots-yam` 0.38.0 and `dreamlabs-client` 1.1.3. The pickup works with
the original upstream waypoint module. The included one-line Arena asset-root
patch selects the production asset mirror used by the recordings; it changes no
scene geometry, scoring or control logic.
Installation and the first asset load need network access. Simulation,
rendering and scripted control run locally. Dream inference uses the Dream Labs
API; configure its credentials outside the repository as in the adapter docs.

## Run and record

```bash
RUN_ROOT="$HOME/yam-eval/drill-wood-trigger"
python run.py demo --log-dir "$RUN_ROOT/demo"
```

Add `--headless false` to see the Isaac Sim window on the local display. For a
new trial choose a fresh log directory. The default creates a timestamped folder
under `$YAM_LOG_ROOT/drill-wood-trigger` (or `~/yam-eval/drill-wood-trigger`).

Both arms receive only ordinary 14-dimensional joint commands. The demo uses
measured tool pose to align the drill and approach its trigger. The simulated
drill and trigger move through gravity, friction, grasping, and joint/contact
forces. The demonstration does not write object poses or trigger states.

The run uses seed 0, no object jitter, 20 Hz control, 240 Hz physics, 320 x 176
policy camera frames, and at most 1,600 control steps (80 simulated seconds).
The demo requests a stop after 70 simulated seconds if it has not succeeded.
Measured wall-clock runtime and scorer results belong in the results section
below after a completed trial.

Render a completed evaluation log into camera videos:

```bash
inspect-robots video /path/to/adhoc_LOG_ID.json
```

Each trial stores its scorer result in the evaluation JSON, detailed geometric
and force measurements in `measurements.jsonl`, and input-file hashes in
`source_manifest.json`. The latter records the source files used by that trial.

## Shared station fixtures

The task adds only the wood, red marking, drill/probe and attached trigger listed
in `task.json`. There are no fruit, loose plates or other station props. The
wood's securing fixture is approximated by its kinematic body and is not rendered.
The initial layout is the uploaded setup image.

The floor is a fixed 200 x 200 x 0.02 m box with top Z=-0.0125 m. The overhead
camera is at (-0.166495, -0.009, 1.691821), with OpenGL XYZW quaternion
(0.183011, -0.183011, -0.683013, 0.683013). Its stand has a 110 x 110 x 16 mm
foot at (-0.236495, -0.009, 0.7555), a 12 mm radius mast with 928.321 mm spine,
a 9 mm radius support with 53.852 mm spine, and a 50 x 42 x 22 mm housing.
The calibrated wrist cameras are fixed to the grippers. Exact stand rotations,
lighting, camera intrinsics, robot meshes and wrist mounts come from
`station.py`, `scene.py` and the bundled robot assets at the pinned adapter commit.

## Setup and scoring

World X points forward, Y left, and Z up. The station table top is at 0.7475 m.
The secured wood block is 180 x 140 x 30 mm, centred at XY (0.26, -0.08) m, so its
top is at Z=0.7775 m. The red target radius is 10 mm. The position was checked
against the YAM kinematics for both a vertical right-arm tool pose and a
horizontal approach by the left arm.

The 0.5 kg drill starts on its side at XY (0.25, -0.30) m with yaw 90 degrees.
Its 35 mm long, 6 mm diameter blunt bit points along the model's -X axis. The
red 14 x 20 x 40 mm trigger slides 5 mm along model +X, against a 150 N/m spring
and 2 N s/m damping. A filtered contact sensor measures contact specifically
between the trigger and the left arm's finger bodies.

The three filtered contact sensors update at each 240 Hz physics step. Their
reported forces are the mean over the 12 physics steps in the current 20 Hz
control interval, so a brief numerical contact bounce does not erase contact
that occurred during that interval.

Full score requires all of these together for 0.5 seconds (10 control steps):

- The right TCP is within 55 mm of the handle point, the jaw gap is below 75 mm,
  and EACH gripping finger contacts the drill with filtered mean force above
  0.05 N. Contact from one finger or with a different object does not count.
- The bit tip is within the target's 10 mm radius and within 4 mm vertically of
  the top surface, with measured tool-to-wood normal force above 0.05 N.
  Aiming from above the target is insufficient. The target is a visual marking
  and does not add a separate collision surface.
- The bit axis is within 12 degrees of vertically down.
- The attached trigger travels 3 to 7 mm, including solver tolerance around its
  5 mm stop, with transverse joint displacement below 3 mm.
- The left fingertips apply more than 0.05 N to the trigger.

`task_score` awards 1.0 only after the uninterrupted hold. Final-state partial
credit is 0.25 for holding the drill, 0.5 for holding it vertically, or 0.75 for
also touching the target. Otherwise the score is zero. A detached or overtravelled
trigger cannot finish the task. Pressing without target contact cannot finish it.

For physical reproduction, use an unpowered drill with a blunt contact probe
and the documented mechanical trigger proxy. A ruler, protractor and calibrated
top/side video at 20 fps or more can check capture between the right jaws, tip
location, visible contact, trigger stroke and the half-second overlap. The low
simulation force thresholds detect contact; calibrated force regulation is not
the task. No powered drilling is needed for the scored interaction.

The wood is fixed to approximate a secured workpiece. Contact materials and
drill inertias are approximate. Motor operation, bit cutting, holes, chips, heat,
and material deformation are omitted.

## Results

Both recordings use the final two-finger grasp check, the same four Python source
files, the same instruction and fixed setup, seed 0, and an 80-second run limit.

| Policy | Score | Control steps | Simulation time | Recorded wall time |
| --- | --- | --- | --- | --- |
| Scripted reference (`wood_drill_demo`) | 1.0 | 764 | 38.20 s | 111.16 s |
| Dream-1 beta (`dreamlabs/dream-1-beta`) | 0.0 | 1,600 | 80.00 s | 198.90 s |

The demo ended successfully after 10 consecutive qualifying control steps.
Its final target error was 2.30 mm, tip height error was
0.043 mm, tilt was 0.84 degrees, trigger travel was
5.02 mm, and both right gripping fingers contacted the drill.
The model trial ended at its step limit without an execution error.

`demo_result.json` and `dream_result.json` record the measured outcomes and source
hashes. These are individual fixed-layout trials, not success-rate estimates.
The submitted camera recordings and setup image are referenced by the media ID
in `task.json`. Wall-clock runtime includes startup, rendering and inference;
first-time asset downloads or shader compilation can take additional time.

## CPU scoring checks

```bash
uv pip install --python "$(command -v python)" "pytest==9.1.1"
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q tests/test_scoring.py
```

The 21 checks cover both-arm and two-finger grasp requirements, hovering and off-target failures,
penetration, tilt, trigger travel and attachment, and the continuous success hold.
Disabling automatic pytest plugin loading avoids unrelated ROS plugins on the
reference workstation.

## Model run and submission

To repeat the model evaluation, run the model separately with the same task:

```bash
python run.py dream --log-dir "$HOME/yam-eval/drill-wood-trigger/dream"
```

Dream-1 inference uses the configured Dream Labs API. The simulator remains
local. Record actual scores, render H.264 videos, capture the initial setup,
upload media with the repository's `tools/upload_media.py`, and reference the
returned ID in `task.json`. Commit only this folder in its own pull request.

The drill model is loaded from Arena's `power_drill` asset library. The original
asset retains NVIDIA's additional asset terms. The wood, target, blunt bit, and
trigger are generated primitives. YAM geometry comes from the adapter's bundled
I2RT robot assets. Keep the relevant dependency licenses when reproducing it.
