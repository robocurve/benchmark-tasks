# Point a drill at a target on the table

Instruction: **Pick up the drill and hold it with the chuck pointing straight down at the red dot.**

This task isolates tool positioning before drilling: grasp a handled tool, orient its chuck, and aim above a marked work location while keeping the tool clear of the surface. The drill is unpowered. The task does not simulate cutting a hole.

## Files

| File | Purpose |
| --- | --- |
| [task.json](task.json) | Submission details, numeric scoring, recorded scores, and uploaded media ID. |
| [drill_task.py](drill_task.py) | Station scene, drill and marker, geometric measurements, partial score, success condition, and scripted demonstration. |
| [drill_geometry.json](drill_geometry.json) | Drill support vertices and source-asset hash used to measure physical clearance. Keep this beside `drill_task.py`. |
| [patches/yam-arena-quaternion-waypoints.patch](patches/yam-arena-quaternion-waypoints.patch) | The adapter change used for the recorded demo, including its regression tests. |
| [tests/test_drill_scoring.py](tests/test_drill_scoring.py) | CPU tests for geometry, score gates, diagnostics, and the continuous success hold. |

## 1. Install the simulation environment

Use Linux with an NVIDIA RTX GPU and a working NVIDIA driver, as required by [yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena). Installation and the first asset load need network access. The setup script installs Isaac Lab-Arena, the YAM adapter, Inspect Robots, and the Dream and agent policies.

The reference environment used Python 3.12.16, Isaac Sim 6.1.0.0, Isaac Lab-Arena 0.3.1, `yam-arena` 0.1.0, Inspect Robots 1.0.0, `inspect-robots-yam` 0.38.0, and NumPy 2.3.1. The commands below pin the adapter, Arena source, and Dream client to the recorded versions. Use fresh directories for these checkouts.

Start in the root of this `benchmark-tasks` checkout:

```bash
DRILL_TASK_DIR="$(pwd)/tasks/drill-point"
DRILL_ADAPTER_DIR="$HOME/yam-isaac-lab-arena-drill-point"
DRILL_ARENA_DIR="$HOME/IsaacLab-Arena-drill-point"

git clone https://github.com/kairosiann/yam-isaac-lab-arena.git "$DRILL_ADAPTER_DIR"
git -C "$DRILL_ADAPTER_DIR" checkout 0059d02a1a91bc6834c1d0846022891df0c78730
git -C "$DRILL_ADAPTER_DIR" apply --check \
  "$DRILL_TASK_DIR/patches/yam-arena-quaternion-waypoints.patch"
git -C "$DRILL_ADAPTER_DIR" apply \
  "$DRILL_TASK_DIR/patches/yam-arena-quaternion-waypoints.patch"

git clone --branch release/0.3.1 \
  https://github.com/isaac-sim/IsaacLab-Arena.git "$DRILL_ARENA_DIR"
git -C "$DRILL_ARENA_DIR" checkout -B release/0.3.1 \
  c7edf3a9d5d8276565208f72422f8f8a0806d6cf

ARENA_DIR="$DRILL_ARENA_DIR" \
  DREAM_SPEC="git+https://github.com/kairosiann/inspect-robots-dream@b5e936a201cec66c54c297e522c920d98445e339" \
  "$DRILL_ADAPTER_DIR/setup.sh"

source "$DRILL_ARENA_DIR/yam.env"
uv pip install --python "$DRILL_ARENA_DIR/.venv/bin/python" \
  --override "$DRILL_ADAPTER_DIR/overrides.txt" \
  "inspect-robots==1.0.0" "inspect-robots-yam==0.38.0" "numpy==2.3.1"

inspect-robots list embodiments
```

The embodiment list must include `yam_arena`. Activate `source "$DRILL_ARENA_DIR/yam.env"` in each new shell, and set `DRILL_TASK_DIR` to this task folder again if needed. Configure a Dream Labs API key using the adapter's [Quickstart](https://github.com/kairosiann/yam-isaac-lab-arena#quickstart) before running Dream-1. Credentials belong outside this submission.

The commands below run with the default headless renderer. Add `-E headless=false` to watch the Isaac Sim window on a local desktop, as in the recordings. On a remote machine, the adapter's optional `setup.sh --desktop` installs its browser desktop; see its [desktop instructions](https://github.com/kairosiann/yam-isaac-lab-arena#watch-a-run-live).

## 2. Create the task file and its scene

The authored task is included as `drill_task.py`. Inspect Robots loads it through `-E task=...`; the `yam_arena` adapter builds the environment from its declarations:

- `SCENE = "station"` supplies the two YAM arms, table, floor, camera stand, and three policy cameras.
- `OBJECTS` adds the loose `power_drill` asset and the fixed red cylinder marking the target.
- `success(objects)` checks clearance, chuck tilt, and aim. `SUCCESS_HOLD_S = 1.0` tells the adapter how long those gates must hold together.
- `distance(objects)` reports horizontal aiming error in metres. `score(objects)` reports partial credit.
- `demo()` supplies timed right-arm waypoints to grasp, lift, rotate, and hold the drill above the marker. The adapter converts the waypoints to joint targets; PhysX handles the grasp and drill motion.

The drill's L-shaped bounding box contains empty space. Measuring clearance from the box's lowest corner can incorrectly reject a lifted drill. `drill_geometry.json` therefore supplies 551 support vertices extracted from the source USD mesh. The scorer scales them with the asset, transforms them into world coordinates, and measures their lowest point. The JSON records the source USD SHA-256 so the geometry can be checked against the asset version.

The dependency patch adds `Waypoint.quat_xyzw`, allowing the demo to command the full wrist orientation needed to turn the chuck down. It must be applied to the pinned adapter before running the demo.

## 3. Run and score a trial

Run these commands from the repository root after activating the environment. Both use seed `0`, a fixed starting layout, 20 Hz control, and RGB observations at 320 by 176 pixels.

### Dream-1 model run

```bash
DRILL_TASK_DIR="$(pwd)/tasks/drill-point"
DRILL_SCORING_LOG="$HOME/yam-eval/drill-point/dream/measurements.jsonl" \
inspect-robots run --embodiment yam_arena --policy dream \
  --policy-checkpoint dreamlabs/dream-1-beta \
  -E "task=$DRILL_TASK_DIR/drill_task.py" -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Pick up the drill and hold it with the chuck pointing straight down at the red dot." \
  --scorer task_score --seed 0 --max-steps 2400 --store-frames \
  --log-dir "$HOME/yam-eval/drill-point/dream"
```

### Scripted full-score demonstration

```bash
DRILL_SCORING_LOG="$HOME/yam-eval/drill-point/demo/measurements.jsonl" \
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E "task=$DRILL_TASK_DIR/drill_task.py" -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Pick up the drill and hold it with the chuck pointing straight down at the red dot." \
  --scorer task_score --seed 0 --max-steps 600 --store-frames \
  --log-dir "$HOME/yam-eval/drill-point/demo"
```

The official trial limit is 2,400 control steps, or 120 seconds of simulation time. The demonstration's 600-step budget is a shorter run within that limit. Successful trials terminate early. Use a new log directory for each trial; `DRILL_SCORING_LOG` appends diagnostic measurements if a file already exists.

### Scoring

Use the final `task_score` printed by Inspect Robots and stored in the evaluation JSON under `results.metrics.task_score`.

Clearance is the lowest physical drill point above the tabletop. Tilt is the angle between the chuck axis and vertically down. Aim error is the horizontal distance from the marker's centre to the forward chuck-axis ray's intersection with the tabletop plane.

| Score | Condition |
| --- | --- |
| 1.0 | Clearance strictly greater than 0.020 m, tilt strictly below 15 degrees, and aim error strictly below 0.040 m, all held continuously for 1.0 second (20 control steps). |
| 0.0 | No control steps, or final clearance at most 0.020 m. |
| 0.25 | Final clearance above 0.020 m, with tilt at least 15 degrees. |
| 0.50 | Final clearance above 0.020 m and tilt below 15 degrees, with aim error at least 0.050 m. |
| 0.75 | Final clearance above 0.020 m, tilt below 15 degrees, and aim error below 0.050 m, without achieving full success. |

Partial credit uses the final state and has no hold requirement. Leaving the full-success region resets the continuous hold counter. Speeds are logged for diagnosis but do not gate the score. `score(objects)` itself returns at most 0.75; the `task_score` evaluator awards 1.0 when the adapter terminates the trial with `termination_reason="success"`.

For a physical trial, use an unpowered drill. Check clearance with a ruler, tilt from calibrated side views, and aiming error from the chuck axis projected to the marked tabletop. Use the trial video to check the one-second hold.

## Recorded results and runtime

| Run | Score | Control steps | Simulation time | Recorded elapsed time |
| --- | --- | --- | --- | --- |
| Dream-1 beta (`dreamlabs/dream-1-beta`) | 0.0 | 2,400 | 120.00 s | 290.66 s, about 4 minutes 51 seconds |
| Scripted demo (`yam_scripted`) | 1.0 | 443 | 22.15 s | 56.27 s |

These elapsed times come from the saved Inspect Robots evaluation logs for the recordings and include simulator startup and inference/rendering overhead. Hardware, headless versus windowed rendering, asset downloads, and initial shader compilation change elapsed time. First-time setup and shader compilation can take additional minutes.

The model recording used task revision `07bed1e9edf4`, before the clearance/scoring correction. Its reported score was 0.0 and its final drill is on the table. The corrected full-score demo used revision `0515d5b6ba29`. The included `drill_task.py` preserves that Python implementation; packaging only removed an extra blank line at the end of the file. The model has not been rerun on that final implementation. The demo recording's instruction was "Pick up the drill and aim it straight down at the red dot," which describes the same goal as the submitted instruction.

## Reset and seeds

The recorded evaluation seed is `0`; Inspect Robots 1.0.0 derives trial seed `1415158438` for the single scene and trial. Keep `-E object_jitter=0` to preserve the fixed layout. Resetting or starting a new run restores the arm ready poses, opens the grippers, and returns the drill to its initial pose. The table and red marker stay fixed. No task objects are consumed or modified.

The setup dimensions, object positions, materials/appearance, and coordinate frame are listed in `task.json`. The complete station fixture and camera geometry come from the pinned adapter's `station.py`, `scene.py`, and bundled YAM assets. Use the uploaded setup screenshot to check the starting layout.

## Verify the task without starting Isaac Sim

After environment activation, install the test dependency and run the task and patched waypoint tests:

```bash
uv pip install --python "$DRILL_ARENA_DIR/.venv/bin/python" "pytest==9.1.1"
python -m pytest -q "$DRILL_TASK_DIR/tests/test_drill_scoring.py"
python -m pytest -q "$DRILL_ADAPTER_DIR/tests/test_demo.py"
```

Together these cover 28 cases, including clearance for an L-shaped drill, aiming tolerances, partial scores, quaternion handling, the uninterrupted one-second hold, and full-orientation waypoint interpolation. The submission package was checked against a clean copy of the pinned adapter with the included patch applied.

## Videos and screenshot

Media set `M-BF9VA9` contains the model video, corrected 1.0-score demo, and starting-layout screenshot. `task.json` references this set. Desktop sections showing identifying information were removed, and the videos were converted to H.264 MP4 with `yuv420p` pixel format and fast-start metadata.

To render camera videos for a new trial:

```bash
for log in "$HOME/yam-eval/drill-point/demo"/*.json; do
  inspect-robots video "$log"
done
```

To upload replacement media, run the repository uploader with team credentials already set in environment variables:

```bash
python tools/upload_media.py tasks/drill-point \
  --video model_run.mp4 --demo demo.mp4 --screenshot setup.png
```

Keep credentials and video files outside the committed task folder. The uploader writes the replacement media ID into `task.json`.

## Third-party assets and licenses

- Power drill: NVIDIA Isaac Lab-Arena's [`PowerDrill` object-library entry](https://github.com/isaac-sim/IsaacLab-Arena/blob/c7edf3a9d5d8276565208f72422f8f8a0806d6cf/isaaclab_arena/assets/object_library.py), asset `power_drill`, file `power_drill_physics.usd`. NVIDIA-provided assets are covered by the [NVIDIA Isaac Sim Additional Software and Materials License](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/common/license-isaac-sim-additional.html), as described in NVIDIA's [asset license FAQ](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/common/license-faq.html). The USD and textures are loaded through Arena. The included support-vertex data are derived from this asset and retain its applicable asset terms. Source USD SHA-256: `265adb323c65bcc0d87c48251b98ed8d2760ead5239efb098cd1f97c36ef36aa`.
- Robot meshes and URDF: I2RT YAM assets bundled by the adapter, with the [MIT license shipped alongside them](https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/yam_arena/assets/yam/LICENSE.i2rt).
- The marker, table, floor, camera housings, and stand use the adapter's generated primitives and simulator materials. NVIDIA-provided simulator materials use the additional-software/materials terms linked above.

Isaac Lab-Arena's Python source is [Apache-2.0 licensed](https://github.com/isaac-sim/IsaacLab-Arena/blob/c7edf3a9d5d8276565208f72422f8f8a0806d6cf/LICENSE.md). That source-code license is separate from the licenses for model assets.
