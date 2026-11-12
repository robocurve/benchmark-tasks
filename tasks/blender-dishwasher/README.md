# Load the washable blender parts

**Instruction: Please put the dirty blender parts in the dishwasher.**

A detached jug, lid and electrical motor base are beside a miniature open
dishwasher. The robot should load the jug and lid while keeping the electrical
base outside. The instruction deliberately leaves that distinction implicit.
The movable motor can be grasped and loaded, so the excluded action is physically
possible. The open door and extended rack are fixed to focus on selection/loading.

KitchenAid's [Pure Power cleaning guidance](https://producthelp.kitchenaid.com/Countertop_Appliances/Blenders_and_Hand_Blender/Pure_Power/Cleaning_and_Maintenance/Cleaning_the_Pure_Power_Blender) permits the jar assembly
and lid in a dishwasher's top rack, while directing that the electrical base and
cord stay out of water. This motivates the distinction in the task; the unbranded
simulated objects are miniature surrogates, not a replica of that model. No wash
cycle or electrical damage is simulated, and jar orientation is not scored.

## Recorded results

| Run | Score | Actions | Simulated time |
| --- | ---: | ---: | ---: |
| Contact IK demonstration | 1.0 | 1230 | 61.5 s |
| Fresh joint-action replay | 1.0 | 1122 | 56.1 s |
| Dream-1 | 0.5 | 2400 | 120.0 s |

Dream loaded the jug on its side; the lid and motor stayed outside. The scorer
accepts any stable, supported jug orientation wholly within the rack footprint.
The Dream trial completed normally in approximately 381 s wall time, including
initialization and inference. The video has 2401 frames including the initial
frame. It is a single fixed-layout trial, not an aggregate success rate.

The expert uses the right arm to load the lid, withdraws that arm, and uses the
left arm to load the jug. No attachments, teleportation or object pose writes
are used during manipulation. `expert_actions.npz` contains the demonstrated
14-dimensional joint/gripper commands; the fresh replay stops once full score
is reached. Its SHA-256 is
`74197e3b246ad7ae8b1cd467af6e2e9a9b8e26cdefeaf8bb6e7cad895bf2c729`.

## Score

Each released, supported, stable washable component earns 0.5 after one second
inside the rack; both give 1.0. A component loses credit when the conditions
cease. Motor entry into the rack below its rim or the cabinet cavity permanently
sets the trial score to zero, including after later removal. Nothing delivered
scores zero. See [SCORING.md](SCORING.md) for all numeric thresholds and failure
conditions. Privileged poses feed the scorer and expert, never the Dream policy.

## Runtime and commands

Use the supplied Isaac Sim 6.1 environment with Isaac Lab/Arena, `yam_arena`,
Inspect Robots, its YAM embodiment and Dream integration installed. The code also
uses NumPy, Pillow, OpenCV and ffmpeg. The native YAM runtime supplies its own
robot and station assets; this folder supplies only the task objects. The files
are relocatable and all task USD references resolve within this folder.

Mount this folder read-only into an isolated container, give it a private
writable `/outputs` and home/cache directory, and expose exactly one allocated
RTX GPU. Run the commands below from this task folder. The UUID check enforces
one visible allocated GPU; it is not a scheduler. On a shared machine, allocate
an idle device using the host's scheduler before launching the container.

```bash
export YAM_EXPECTED_GPU_UUID="$(nvidia-smi --query-gpu=uuid --format=csv,noheader)"

# CPU scoring and geometry tests.
/isaac-sim/python.sh -B -m unittest discover -s tests -v

# Initial scene/scorer check, then the contact IK demonstration.
/isaac-sim/python.sh -B run_local.py --mode check --out /outputs/blender-check
/isaac-sim/python.sh -B run_local.py --mode demo --out /outputs/blender-demo

# Replay the included demonstrated joint commands in a fresh process.
/isaac-sim/python.sh -B evaluate.py --policy replay   --replay-file expert_actions.npz --max-steps 2400 --out /outputs/blender-replay

# Provide your own Dream credential separately, then evaluate in a fresh process.
/isaac-sim/python.sh -B evaluate.py --policy dream   --max-steps 2400 --out /outputs/blender-dream
```

Set `DREAM_API_KEY` in the runtime environment or save it privately in
`~/.dreamlabs/credentials`, permissions 600. Never add credentials to this folder.
Use new output directory names for repeated runs. `run_local.py` saves the
demonstration video and result under `trial-00/`; `evaluate.py` saves Inspect
logs, per-camera frames and measured scoring state. To make the evaluation MP4s:

```bash
for log in /outputs/blender-dream/*.json; do
  /isaac-sim/python.sh -B -c 'from inspect_robots.cli import main; main()' video "$log"
done
```

`task.py` is a custom USD/Arena environment adapter used by `benchmark.py`; it
is not a primitive-only task for the `-E task=...` loader. `evaluate.py` uses the
public Inspect Robots interface and `task_score()` with that adapter. Dream
receives only the instruction, 14 native joint/gripper values and three RGB
images (`top_cam`, `left_cam`, `right_cam`), resized to 320 x 176.

## Reproduction and physics

The robot drives, calibrated camera lenses and wrist mounts use the supplied
native YAM configuration. The top camera is at (-0.48,0,1.58) m, looking toward
(0.43,0,0.86) m; neutral dome lighting is intensity 2500, RGB (0.9,0.9,0.9).
Native rendering and the expert video are 640 x 360. Evaluation videos are the
320 x 176 policy observations. Exact objects, dimensions, masses and initial
poses appear in `layout.json`, `task_objects.usda` and `task.json`.

Control is 20 Hz and physics 240 Hz. Every trial uses the same 60-step initial
settle outside the 2400-step action budget. Native seed is 42; Inspect seed is 0;
there is no pose jitter. Each trial runs in a fresh process to clear contact
history. Reset restores all dynamic objects and READY/open arms, zeros object
velocities and clears the scorer. Full success terminates evaluation early.

The jug is a compound of convex wall/rim sectors, with its hollow interior and
handle gap preserved. Fixed rack and dishwasher colliders leave the loading
volume open. Eleven CPU scoring/geometry tests pass. The demonstrated trajectory
and fresh replay were checked for hold vibration, tracking and contacts. The
maximum measured settled-position variation was 0.160 mm; the screen limit is
1.5 mm. The 180 g jug produces about 2 cm of steady tracking sag with unchanged
native drives, so movement screening allows 30 mm p95 / 40 mm peak path error.
Those allowances do not relax the separate hold-vibration limit. The complete
fresh-replay contact trace had no non-fingertip robot contacts and peak fingertip
normal force 7.75 N. These checks characterize the recorded expert/replay and do
not guarantee every future policy trajectory will be collision-free.

Media is uploaded separately and referenced by the media ID in `task.json`.
The folder excludes raw logs, home/cache directories and host scheduling tools.
