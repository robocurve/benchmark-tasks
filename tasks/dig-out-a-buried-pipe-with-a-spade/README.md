# Expose a buried pipe with a bimanual spade

Category: **Impact on jobs**. This is a local development task; see
[DESIGN.md](DESIGN.md) for the occupational evidence, relevant PR feedback,
scoring rationale and remaining fidelity limits.

![Fixed starting layout](setup.png)

## Run the digging demonstration and export videos

From the repository root:

```bash
conda activate robocurve
python tasks/dig-out-a-buried-pipe-with-a-spade/run_demo.py
```

The runner uses the existing Isaac Lab/YAM environment, saves a source snapshot,
records all three cameras, and **automatically exports MP4s** under a new
`~/yam-eval/pipe-dig-YYYYMMDD-HHMMSS/videos/` directory. It also writes
`trace.jsonl` and `summary.json` with exposure, delivered/spilled mass, physical
grip forces, pipe/robot contact and failure information. `latest-soil.npz` saves
the latest particle positions and orientations, sampled once per simulated
second for inspection. No API key is needed.

Choose a run directory or step limit:

```bash
python tasks/dig-out-a-buried-pipe-with-a-spade/run_demo.py \
  --out "$HOME/yam-eval/pipe-dig-test-$(date +%Y%m%d-%H%M%S)" \
  --max-steps 2500 --seed 0
```

Use a new directory for each run. `--no-video` skips MP4 export but retains raw
frames. To export those later:

```bash
inspect-robots video /path/to/adhoc_RUN_ID.json --out /path/to/videos
```

The runner's process success means the evaluation executed; **task success is
`task_score == 1.0` with no latched failure**, not merely a finished video.

The current development environment is Conda `robocurve`, Python 3.12, with
`inspect-robots==1.0.0`, `inspect-robots-yam==0.38.0`, Isaac Sim 6.1.0.0,
NumPy 2.3.1 and PyTorch 2.11.0+cu128. Editable simulator checkouts are:

| Repository | Revision |
|---|---|
| IsaacLab-Arena | `c7edf3a9d5d8276565208f72422f8f8a0806d6cf` |
| IsaacLab submodule | `91a0906f93802eb859192dcb0bca879f891e92d6` |
| yam-isaac-lab-arena | `bd62771e60320433893bd1d2df0fc79b5ddcd6d1` |

Runs record installed package versions in `environment.json`. The layout and
seed are fixed (`0`); GPU contact simulation may still vary numerically between
runs. The control limit is 2,500 steps (125 simulated seconds). Raw frames from
a full run occupy about 5 GB, in addition to the much smaller exported videos.

## Shortened task

The demo now plans three scoops instead of twelve. The initial fill is reduced
from 5,965 grains (4.381 kg) to 3,655 grains (2.669 kg), with at most about 23 mm
of cover above the crown before reset settling. The 80 mm exposure target,
120 g delivery target and three loaded trips are unchanged.

`prepare_soil.py` deterministically builds the shallow layout from the original
settled `soil_angular_start.npz`; it is a preparation step, never an episode hook.
No grains are removed or repositioned during a run.

## Scene and controller

`task.py` contains the excavation fixture, marked pipe, settled fill, spade and
spoil tray. `soil_shallow_start.npz` contains grain positions, radii and
orientations. Grains are angular convex bodies with mineral density 2,650 kg/m³;
their physical masses use their actual polyhedral volumes. They are not deleted,
attached to the blade or moved by a scripted soil model during an episode.

`spade.py` builds one free rigid body with a cupped steel blade, shaft, open
D-handle and two flat grip sleeves for the parallel fingers. Four large blade
collision panels preserve the cup without internal triangle collider seams.
Blade friction is 0.35 static / 0.25 dynamic (minimum combine); grip/body friction
is 1.2 / 1.0 (average combine). These are specified model parameters, not measured
soil/tool calibration. The tool has mass 0.45 kg and starts on two supports.

`digplan.py` plans the shared tool trajectory. `run_demo.py` follows the two TCP
trajectories and uses observed joint positions to compensate motor compliance.
It retimes both arms together to respect the default command-delta limit and
sends ordinary joint targets, with unchanged motor torque limits. There are
no grasp welds, external support forces or tool-pose writes. `yam_scripted` alone
plays the nominal trajectory without that feedback and is not the validated
controller for this task.

`contact_monitor.py` measures contact at every physics substep. Both opposing
finger pads on each grip must contact the tool; merely placing a closed hand
near the shaft is insufficient. `scoring.py` measures physical outcomes
independently of the scripted trajectory. The rubric is in `task.json` and
`DESIGN.md`; no Dream-1 result or uploaded media ID is fabricated.

Full success requires 80 mm of contiguous exposed pipe crown, at least 120 g
delivered by spade over at least three loaded trips, no more than 30 g spilled,
and the emptied blade clear of the fixture (at most 5 g remaining on it).
Spade–pipe contact is allowed; direct robot excavation still invalidates the trial.
The pipe remains a physical collider, and contact forces are logged diagnostically.

## Offline checks

```bash
python -m pytest -q tasks/dig-out-a-buried-pipe-with-a-spade/test_scoring.py
python tasks/dig-out-a-buried-pipe-with-a-spade/fastik.py
```

## Original stationary preview

`proto_soil.py` remains the smaller original soil-bed preview with stationary
arms. Its score is always zero. It shares the updated spade asset, but it does
not run the digging task:

```bash
run_dir="$HOME/yam-eval/pipe-static-$(date +%Y%m%d-%H%M%S)"
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/dig-out-a-buried-pipe-with-a-spade/proto_soil.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  -P hold_s=14 --instruction "Preview the soil bed, buried pipe and spade." \
  --scorer task_score --max-steps 400 --seed 0 --no-prompt \
  --no-rerun --store-frames --log-dir "$run_dir"
for log in "$run_dir"/adhoc_*.json; do
  inspect-robots video "$log" --out "$run_dir/videos"
done
```

## Provenance and submission status

The procedural spade dimensions originated in the existing `digplan.py` and
Claude's local reachability experiments. No downloaded third-party shovel mesh
is used. The task uses the separately installed YAM robot and Isaac Lab assets,
whose upstream licenses remain applicable.

This folder is a development implementation, not a submitted or accepted
benchmark. The authenticated task board could not be inspected without sign-in;
public PR references and unresolved review requirements are documented in
`DESIGN.md`. Real-soil fidelity remains unvalidated: no cohesion, wet soil,
fracture, compaction calibration or pipe-breakage prediction is claimed.
