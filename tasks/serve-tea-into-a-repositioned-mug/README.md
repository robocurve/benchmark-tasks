# Adapt a drink pour when the recipient repositions their mug

**Societal relatability.** “Pour my tea to the line on my mug and put the pitcher down.” A recipient can move the mug after serving begins. The benchmark measures whether a robot can adapt the stream and fill the mug before putting the pitcher down. A pause-and-resume server and a continuous tracker can both earn full credit.

At 48 g delivered, the surrogate-held mug travels 83 mm in one second. The counterfactual holds the receiver steady and replays exactly the same numerical commands in both scenes. Results from this revision are recorded in [controls.json](controls.json). The moving receiver is the official condition. Camera order is fixed overview, left wrist and right wrist. The policy sees the three RGB images and both arms' measured joint positions. The scripted demonstration replays a tape generated with privileged scene information.

## Score a trial

Use a scale for the mug, surrogate collector and other spill, a side/overview recording and a ruler for final positions. A visible discharge, more than 3 mm of pitcher travel, or more than 3° orientation variation interrupts the two-second dwell. A persistent telltale trips at 3 mm of sideways displacement of the mug assembly relative to its moving carrier, on either witness axis. A tripped telltale, robot interference with the examiner’s fixture or a pitcher drop earns zero, as does more than 2 g on the surrogate or more than 10 g spilled elsewhere. Half credit requires 30–130 g in the mug and a complete two-second stopped and stable dwell with the pitcher supported. Full credit requires 100–130 g, completed receiver motion, counter support with ≤6° pitcher tilt, release and 35 mm whole-arm separation throughout the dwell. Measurement uncertainty and examiner error produce an invalid trial requiring repetition. Full credit takes precedence.

The [bench protocol](BENCH_PROTOCOL.md) links the dimensioned station, motion and witness drawings, parts list, calibration instructions and grading worksheet. It describes containment, gripper protection and authorization for room-temperature water on the chosen robot. The physical setup requires commissioning. Numerical fluid parcels and rigid-body contacts approximate water flight and grasp mechanics. The model omits secondary splash and slosh.

## Reproduce the simulation

Use Python 3.11+, EGL-capable Linux, FFmpeg/libx264, ffprobe and a fresh output directory. The published run uses seed 0, 240 Hz physics, 20 Hz control and a 60-second official limit. Wall time depends on rendering and hosted inference.

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -e '.[inspect]'
export MUJOCO_GL=egl OPENBLAS_NUM_THREADS=1 LP_NUM_THREADS=2
python -m unittest discover -s tests -q
python -m tea.run suite --out runs/revision-check/trials

export TEA_OBSERVATION_TRACE="$PWD/runs/revision-check/demo-plant.jsonl.gz"
inspect-robots run --embodiment yam_tea_service --policy tea_replay \
  -P actions=demo_actions.npy -E profile=reposition -E cam_width=320 -E cam_height=176 \
  --instruction 'Pour my tea to the line on my mug and put the pitcher down.' \
  --seed 0 --max-steps 1200 --scorer tea_outcome --no-prompt --grader none \
  --no-rerun --store-frames --log-dir runs/revision-check/demo
```

Use a project interpreter configured with the installed hosted Dream adapter and its station environment for the model trial. The optional bridge URL comes from that adapter's configuration.

```bash
bridge=()
if [[ -n "${DREAM_BRIDGE_URL:-}" ]]; then bridge=(-P "server_url=$DREAM_BRIDGE_URL"); fi
export TEA_OBSERVATION_TRACE="$PWD/runs/revision-check/model-plant.jsonl.gz"
inspect-robots run --embodiment yam_tea_service --policy dream \
  -P model=dreamlabs/dream-1-beta "${bridge[@]}" -E profile=reposition -E cam_width=320 -E cam_height=176 \
  --instruction 'Pour my tea to the line on my mug and put the pitcher down.' \
  --seed 0 --max-steps 1200 --scorer tea_outcome --no-prompt --grader none \
  --no-rerun --store-frames --log-dir runs/revision-check/model
```

## Asset attribution

The YAM robot description and nine visual meshes are by I2RT Robotics, pinned at commit `120c3c81400171174604e503943f8d1ebc891058` and distributed under the MIT license. File hashes, source locations, original fixtures and generated components are inventoried in [ASSETS.md](ASSETS.md).
