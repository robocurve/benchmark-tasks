# Hold and align pipe flanges for horizontal pin insertion

Hold the loose pipe flange against the fixed flange and insert two locating pins horizontally in the top and bottom holes. The loose flange is grasped by the side lug; the demonstration ends with the flange and upper pin still held. It does not claim the pins support the flange after release.

## Setup

All distances are metres and coordinates are in the calibrated YAM station frame, with `TOP = 0.7475`. The fixed flange is vertical, with its mating face at `(0.3145, 0.06, TOP + 0.17)`, supported by a fixed horizontal pipe 0.10 long and 0.054 in diameter and a 0.05 × 0.08 × 0.13 support. The loose 0.25 kg vertical flange starts at `(0.19, 0.20, TOP + 0.076)`, on three fixed staging blocks. It has a short pipe stub, a square coupling collar and a side lifting lug; these are rigidly attached. Both flanges use the CAD visual and a 0.016-thick perforated collision approximation with 0.016 diameter holes, on a 0.05499 bolt-circle radius.

Two 0.035 kg steel-like rigid locating pins lie parallel, with their heads toward negative x, on identical 0.05 × 0.028 × 0.185 supports. Their nominal initial positions are `(0.32, -0.30, TOP + 0.195)` and `(0.32, -0.18, TOP + 0.195)`. Each has a 0.05-long tapered shank of diameter 0.014 and a 0.02-diameter head. Task-file coordinates, sizes, colors and reset poses are specified in `task.py`; the generated collision assets and `build_collisions.py` are included. The station and all supports are fixed. There is no object jitter; both arms reset to the ready pose.

## Scoring

The final state alone is scored. Both flange axes must lie within 7.25 degrees of the horizontal pipe axis; all four corresponding bolt-hole centers must lie within 0.003 in 3D for the alignment credit of 0.4. A pin scores 0.3 when its axis points toward negative x within 7.25 degrees, its center is within 0.0045 of the top or bottom hole axis, and its bounding-box center is between 0.005 toward the head and 0.008 toward the tip from the mating plane. Only one pin counts per target hole; the side holes do not count. Without alignment the score is zero. Maximum is 1.0.

The scorer checks poses, not grasp contacts or velocities; a pin in a target hole can earn credit while held, and transient placements can be scored if present at the end. The demo stops with the second pin held, so it shows insertion but does not demonstrate release or unsupported retention. `python -m pytest -q test_score.py` exercises the geometry and partial-credit cases.

## Reproduction

Use Isaac Lab Arena commit `c7edf3a9d5`, Inspect Robots 0.60.0, Isaac Lab Arena 0.3.0 and Isaac Lab 17.0.2. Install this directory as an editable package in the YAM environment. The extensions register `yam_flange` and `yam_flange_scripted`. The scripted demo uses measured simulator object poses as feedback for alignment; those privileged poses are not provided to Dream-1.

```sh
inspect-robots run --embodiment yam_flange --policy yam_flange_scripted \
  -E task="$PWD/task.py" -E object_jitter=0 -E device=cpu \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Hold the loose pipe flange against the fixed flange and insert two pins horizontally in the top and bottom holes." \
  --scorer task_score --max-steps 2700 --seed 0 --store-frames --log-dir ./demo-run
```

For Dream-1, substitute `--policy dream` and a separate log directory; retain the same task, instruction, seed, control rate, cameras and run limit. The scripted demonstration reported 1.0 at 2,231 steps in 226.7 seconds, environment revision `178813d85202`. Dream-1 reported 0.0 at 2,700 steps in 265.6 seconds, with the same environment revision. After both recordings, only the task file’s module docstring and one comment were updated; its resulting SHA-256 is `97b1eb57ed9ad668ad2720501a8608c995280b4025854bfeefa40fb5293a6071`, with no behavior change.

The simulation uses rigid bodies, simplified 16 mm perforated colliders, a rigid pipe stub and side lifting lug, and approximate friction. It omits gasket, threads, retaining clips, deformation, preload and fluid seal. It does not establish that the two pins retain the pipe when unsupported. The original CAD source is [Innerscene pipe flange](https://www.innerscene.com/tools/library/3d-parts/pipe-flange-original-mep-cad-2a6f5077), [CC0 1.0](https://creativecommons.org/publicdomain/zero/1.0/).

The demonstration video places the station and right-wrist cameras from the same recorded run side by side; the Dream video uses the station camera.
