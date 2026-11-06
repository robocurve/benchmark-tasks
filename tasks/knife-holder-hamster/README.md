# Store a knife in an obstructed holder

Instruction: **Put the knife into the holder.**

A rigid hamster figurine occupies the bay beneath the only available knife slot.
The robot must clear that bay and store the loose knife. The four other knives
and the rack are fixed fixtures. The toy and the loose knife are movable.

## Recorded results

| Policy | Score | Control steps | Simulated time | Wall time |
|---|---:|---:|---:|---:|
| Dream-1 beta (`dreamlabs/dream-1-beta`) | 0.0 | 2,400 | 120 s | 359 s |
| Scripted demonstration | 1.0 | 1,233 | 61.65 s | 198 s of actions, 229 s including startup |

The demonstration removes the toy with the right arm, places it on the table,
and inserts the knife with the left arm. Both arms return to READY. It issues
robot joint/gripper actions only. Dream receives three RGB images, 14 joint/gripper
values and the instruction; privileged object poses are used for scoring and the
scripted demonstration. These are results from individual recorded trials.

## Runtime

Use an existing Linux/RTX YAM runtime with Isaac Sim, Isaac Lab-Arena, `yam_arena`,
Inspect Robots and the Dream plugin installed. The tested environment is recorded
below: Isaac Sim
6.1.0-rc.26, Arena 0.3.0, yam-arena 0.1.0 at commit
`0059d02a1a91bc6834c1d0846022891df0c78730`, Inspect Robots 0.60.0,
Dream client 1.0.14 and inspect-robots-dream 0.1.0. Python is 3.12.
The `isaaclab` distribution reports metadata version 17.0.2 in this installation.

Validation used the supplied local image `yam-inspect-isaac:6.1.0`; its immutable
image ID is `sha256:0e5a603c33c29eab2751be8977127a74a469f64dad70ca4bc3e50eacc3f504a8`. This local tag is not a public registry
URL. Access to an equivalent runtime must be confirmed before public reproduction;
this package does not install or rebuild the shared runtime.

Expose exactly one RTX GPU to the runtime, provide a writable `/outputs` directory
and HOME, and run from this task folder. Inside the tested image:

```bash
ISAAC_PYTHON=/isaac-sim/python.sh
knife_run_dir="/outputs/knife-holder-$(date +%Y%m%d-%H%M%S)"
```

Set `ISAAC_PYTHON` to the simulator's Python launcher if it lives elsewhere.
Simulation is headless at 240 physics steps/s and 20 control steps/s. Native camera
frames are 640 x 360; Dream observations are resized to 320 x 176. The top camera
is 10 cm lower and tilted 4 degrees upward relative to the supplied calibrated
station pose; wrist cameras and lenses retain their supplied calibration. Lighting
is one neutral dome at intensity 2500 and color `(0.9, 0.9, 0.9)`.

## Run and score

No-API integration check (holds the arms ready; expected task score 0):

```bash
"$ISAAC_PYTHON" evaluate.py --policy hold_ready --max-steps 5 \
  --out "$knife_run_dir/check"
```

Full physical demonstration, with three camera videos and a numeric report:

```bash
"$ISAAC_PYTHON" run.py --mode demo --episodes 1 \
  --out "$knife_run_dir/demo"
```

Dream evaluation, after the operator configures their personal Dream API key in
`~/.dreamlabs/credentials` or `DREAM_API_KEY`:

```bash
"$ISAAC_PYTHON" evaluate.py --policy dream --max-steps 2400 --epochs 1 \
  --out "$knife_run_dir/dream"
```

The saved Inspect log reports `results.metrics.task_score`. The scripted report
is `demo_report.json`, with per-episode scores and measured object/TCP trajectories.
[SCORING.md](SCORING.md) defines the 0 / 0.5 / 1 end-state conditions. A qualifying
state must hold for one continuous second; the score at the end is used. Full
success ends model evaluation early. The model run limit is 2,400 control steps,
not a wall-clock limit. Reset uses the same fixed layout, zero object jitter and
evaluation seed 0. Restore object poses and velocities and both READY arm states.

Export model frames to H.264 MP4, one file per camera:

```bash
for knife_log in "$knife_run_dir/dream"/store-knife_*.json; do
  "$ISAAC_PYTHON" -c 'from inspect_robots.cli import main; main()' video "$knife_log"
done
```

The environment uses a process lock below `/outputs` to prevent overlapping
simulators in the same runtime. A separate GPU visibility check requires one GPU.

## Scene files and assets

`task_objects.usda` contains the task objects and relative asset references.
`task.py` reads those objects and combines them with the installed YAM station;
`benchmark.py` exposes the result through the public Inspect Robots interface.
This custom-USD task is run through `evaluate.py`; it is not a primitive-object
file for the ordinary `-E task=...` loader.

The installed YAM embodiment supplies the robot and table assets. Keep the custom
`assets/` folder beside `task_objects.usda`. The public task includes the knife,
holder and toy assets and their textures.

The holder is 432 x 104 x 176 mm, with a 90 mm center bay and 46 x 8 mm slot.
The knife is 265 mm long and 105 g, with a 2.6 mm blade, black riveted handle,
flat grip faces and a flat cylindrical bolster. The 65 g toy is approximately
58 x 55 x 88 mm, with a 47 mm body grip width. Materials and exact poses appear
in `task.json`, `layout.json` and the assets. The holder is at world X = 0.355 m.

The toy uses rigid compound collision shapes. Fur is visual; deformation, animal
behavior, cutting and damage are not simulated. Background knives are fixed.
The loose knife and toy move by physical contacts and friction, with no grasp
attachments or object pose writes.

## Recorded validation

The task has a recorded full-score physical demonstration and a completed
Dream-1 beta trial, with the measured scores listed above and in `task.json`.
Local independent scorer tests and USD dependency/geometry checks passed.
The scripted demonstration helpers are included so its full-score sequence can
be reproduced. Asset-authoring tools, copied preview-station meshes and private
development evidence are retained in the local review bundle.
