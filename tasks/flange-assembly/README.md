# Flange assembly

Transfer two loose pipe flanges onto a round holder, align their bolt circles and install four removable locating pins. This models the alignment step of equipment and pipework assembly with lightweight training parts. The pins do not lock, seal or clamp the joint.

The scripted demo reaches full score (1.0). Dream-1 completed the full action limit and scored 0.0. The Dream video, full-score demo and initial setup screenshot are uploaded under media ID `M-KSJ5FA`, recorded in `task.json`.

The layout, object inventory, instruction, numerical final-state scoring and reset procedure are specified in `task.json`. Positions use the station world frame: x forward, y left, z up, table top at 0.7475 m. The initial lower flange is inverted, neck down; the upper flange is neck up. Both flange bounding-box centres settle 80.5 mm above the table on their staging blocks. The task file gives spawning origins: because the CAD origin is at the mating face, these differ from bounding-box centres.

The demo places the lower flange with the left arm, then the upper flange with the right arm. After the upper flange is released and the placing arm withdraws, pin installation alternates left, right, left, right. The inactive arm parks outside the working arm's path. The holder is 280 mm in front of the arm-base line and 80 mm toward the right arm so all four insertion poses are reachable with an upright wrist. The final approach descends slowly and holds the pin before opening. These are demo actions; the final-state scorer does not enforce an arm assignment or action sequence. The scripted oracle reads measured simulator object poses to correct XY placement and upper-flange yaw before release, then centres each held pin over a matched hole pair. It changes robot joint targets only; it never writes object poses during the rollout. Dream receives its standard cameras and joint state and does not use this oracle feedback.

## Installation

Use the benchmark YAM environment on Linux with an RTX GPU. The tested stack uses Python 3.12.16, Isaac Sim 6.1.0.0, Isaac Lab-Arena package 0.3.0, yam-arena 0.1.0, Inspect Robots 0.60.0, inspect-robots-dream 0.1.0, NumPy 2.3.1 and PyTorch 2.11.0+cu128. The simulator environment ID reports Arena 0.3.1; the installed distribution reports 0.3.0. Tested source commits:

- `yam-isaac-lab-arena`: `0059d02a1a91bc6834c1d0846022891df0c78730`.
- `IsaacLab-Arena`: `c7edf3a9d5d8276565208f72422f8f8a0806d6cf`.
- Hardware: one NVIDIA L40S, driver 565.57.01; CPU PhysX with RTX rendering.

For a new environment, use the supplied [YAM setup](https://github.com/kairosiann/yam-isaac-lab-arena) at the recorded commit. In the initialized YAM shell, change to this task directory and install it as an editable package so the relative USD asset path is preserved:

```sh
source ~/IsaacLab-Arena/yam.env
uv pip install -e . --no-deps
python -m pytest -q test_score.py
```

Use `~/.local/bin/uv` if `uv` is absent from the shell's PATH. The extension registers embodiment `yam_flange` and policy `yam_flange_scripted`; the base package supplies scorer `task_score`. The generic `yam_arena` embodiment alone does not spawn the CAD flanges, holder or tapered pins correctly.

## Reproduction

Run from this directory. Logs and frames should be stored outside the submission folder. The fixed seed is 0, reset jitter is zero, the control rate is 20 Hz and physics runs at 240 Hz. Three cameras use the station's calibrated poses and 480 x 270 observations. All trials use the same scene and camera configuration. Dream's client resizes observations to its 320 x 176 input contract.

```sh
inspect-robots run --embodiment yam_flange --policy yam_flange_scripted \
  -E task="$PWD/task.py" -E object_jitter=0 -E device=cpu -E control_hz=20 \
  -E cam_width=480 -E cam_height=270 \
  --instruction 'Place both flanges on the holder, then insert four pins.' \
  --scorer task_score --max-steps 2700 --seed 0 --store-frames \
  --log-dir ~/yam-eval/flange-demo
```

After a successful demo, use Dream with credentials configured outside this repository:

```sh
inspect-robots run --embodiment yam_flange --policy dream \
  -P model=dreamlabs/dream-1-beta \
  -E task="$PWD/task.py" -E object_jitter=0 -E device=cpu -E control_hz=20 \
  -E cam_width=480 -E cam_height=270 \
  --instruction 'Place both flanges on the holder, then insert four pins.' \
  --scorer task_score --max-steps 2700 --seed 0 --store-frames \
  --log-dir ~/yam-eval/flange-dream
```

The script requests a stop after its plan and a three-second final hold. Dream receives the full 2,700-step limit. Score is computed at the end, not the best intermediate state. The log's generic embodiment/policy names may be `yam_arena`/`yam_scripted` because the extension inherits their information objects; the commands above identify the registered implementations actually used.

The completed scripted demo scored **1.0** in **2,651 control steps**, with **367.53 seconds** wall time including simulator startup. Its 132.60-second video includes the initial observation and final hold. All four pins are seated, both grippers are open and withdrawn, and the reported score is the final score. The Dream-1 beta trial scored **0.0** after the full **2,700 control steps**, with **286.82 seconds** wall time including startup and inference. Its checkpoint was `dreamlabs/dream-1-beta`; mean reported inference latency was 0.544 seconds. No pins were inserted and neither flange was assembled on the holder. Both trials used seed 0, zero jitter and the same task revision and camera configuration.

Task-file SHA-256: `5fbee0488ed00608b5f1153544cb4a7fbeece6cbdbcc3484ee66d464f64b77f7` (log environment revision `5fbee0488ed0`). The extension source and relative USD asset must also match this folder; the task-file hash alone does not hash them. A process ending successfully does not by itself establish task completion.

Render the completed JSON with `inspect-robots video <log.json>`. Preserve the full top-camera trajectory and optional wrist views. Encode review media as H.264 MP4 with yuv420p and faststart; use the first frame of the same setup as the screenshot. Raw logs and media are kept outside this task folder.

## Scoring and limitations

`test_score.py` covers synthetic assembled, incomplete, misaligned, duplicated, hovering, dropped, tilted and moving states. It does not establish physical insertion; that requires reviewing the completed simulation and its final poses. The exact orientation test is squared quaternion components greater than 0.996; the approximately 7.25-degree tilt threshold assumes normalized quaternions. Position and resting checks do not prove release, grasp contact or the required sequence.

The flange CAD is rendered directly and collides as 55 convex pieces per rigid flange. An offline clearance check found a 10 mm diameter sampled passage in each nominal 16 mm hole; this geometric screening is not a manufacturing-tolerance claim. The holder uses 32 convex sectors, inner radius 67 mm, outer radius 85 mm, top 60 mm above the table. Each pin is one rigid body with a convex tapered shaft and cylindrical head. Flanges are 0.15 kg and 0.25 kg; pins are 0.035 kg. They represent lightweight training parts rather than steel hardware. Flange friction is 1.2 static/1.0 dynamic, holder friction 1.5/1.2; pin friction uses the simulator default. No friction, inertia or density calibration against real components was performed. No threads, gasket, locking clip, deformation, preload, fluid seal or pressure model is included.

## Assets

`assets/source-flange.step` is the unmodified [Innerscene pipe-flange CAD](https://www.innerscene.com/tools/library/3d-parts/pipe-flange-original-mep-cad-2a6f5077), released under [CC0 1.0](https://creativecommons.org/publicdomain/zero/1.0/). `assets/flange.usda` includes its tessellated visuals and all 55 collision meshes; no external geometry download is needed at run time. The nominal CAD dimensions are 150 mm outer diameter, 54 mm bore, 51 mm overall height and four 16 mm bolt holes on a 110 mm bolt circle. These are layout dimensions, with no certified load rating or flange-standard compliance. Holder and pin geometry are authored by the included extension.
