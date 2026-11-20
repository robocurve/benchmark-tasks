# Remove and stow a YAM Linear 4310 gripper

Local simulation prototype on a de-energized donor-wrist fixture. The arm grips a powered driver, loosens the fixture fasteners, supports the module during separation, and releases it into an empty cradle.

## Hardware evidence

| Detail | Revision and status |
| --- | --- |
| Wrist and gripper visual geometry | Documented in I2RT v1.3.6, YAM v1 and Linear 4310. The URDF supplies the joint-6 reference frame linking wrist and gripper. It gives no separate mounting fasteners, bracket or driver geometry. |
| Stock assembly and camera | ABC shows pushing the gripper onto the end of the arm, fastening with supplied long screws and mounting the wrist camera. The photographed assembly identifies the crank-style context in the accompanying instructions. The Linear 4310 screw paths, access with the camera installed, bracket order and cable routing remain unverified. |
| Screw, bit and driver | Stock thread, head, captive status, bit and engagement are unknown. The M3×0.5 socket heads, 2.5 mm bit, guides and collar are provisioned training components. |
| Release and resistance | The training thread uses two complete turns through effective 1.0 mm engagement. The stop includes 80° of additional clearance. Breakaway and running resistance are engineering assumptions for the fabricated fixture. |

[I2RT CAD revision](https://github.com/i2rt-robotics/i2rt/tree/120c3c81400171174604e503943f8d1ebc891058) · [I2RT YAM product documentation](https://doc.i2rt.com/products/yam) · [ABC assembly guide](https://abc.bot/hardware.html#mounting-the-wrist-camera-and-gripper)

## Asset provenance

The asset generator uses I2RT v1.3.6 at commit `120c3c81400171174604e503943f8d1ebc891058`. The local inputs match the source blobs at that commit. All I2RT files in the table use the MIT License, copyright I2RT Robotics. The full text ships as `LICENSE.i2rt`. The runtime YAM station is supplied by the installed Arena environment. The other task-local scene geometry is procedural.

| Source under `i2rt/robot_models/` | Derived use |
| --- | --- |
| `arm/yam/v1/assets/link5.stl` | Donor wrist visual in `assets/yam_wrist_fixture.usda` |
| `gripper/linear_4310/assets/gripper.stl` | Gripper housing in `assets/linear_4310_module.usda` |
| `gripper/linear_4310/assets/tip_left.stl` | Left jaw in the same module USD |
| `gripper/linear_4310/assets/tip_right.stl` | Right jaw in the same module USD |
| `arm/yam/v1/yam.urdf` | Wrist, housing and tip transforms baked into the USDs |
| `gripper/linear_4310/linear_4310.xml` | Cached vendor reference, with no MJCF geometry embedded separately |

[Pinned MIT license](https://github.com/i2rt-robotics/i2rt/blob/120c3c81400171174604e503943f8d1ebc891058/LICENSE)

## Scoring and controls

Each flag turns from red to green when its captive head reaches the disengagement stop after at least two complete turns measured with the bit seated. Both flags are required for 0.50. The scorer checks gripper contact through withdrawal and requires the module to be still and released in the cradle for one second for 1.00. Forced or unsupported removal invalidates the trial. The controls omit the second screw or power the driver above both sockets.

Run from the task folder after activating the installed Arena environment. Use `FAILSAFE_CASE` to select a negative control (one-engaged, free-spin). The cap is 1,500 steps.

```bash
source ~/IsaacLab-Arena/.venv/bin/activate
source ~/IsaacLab-Arena/yam.env
steps=$(jq -er '.run_limit | split(" ")[0] | gsub(",";"") | tonumber' task.json)
inspect-robots run --policy yam_scripted -P hold_s=2 --embodiment yam_arena --scorer task_score \
  -E task=arena_task.py -E headless=true -E object_jitter=0 -E control_hz=20 \
  -E cam_width=256 -E cam_height=144 --seed 0 --instruction "$(jq -er .instruction task.json)" \
  --max-steps "$steps" --no-prompt --store-frames --log-dir logs/demo
```

Run `python task.py --ik` for the scorer and waypoint regressions.

## Driver and torque

Use the [adapter drawings and assembly steps](fabrication/README.md). The assumed class 8.8 screw runs in a steel insert in an aluminum bridge. The 0.50 N·m breakaway target is a conservative training input in the range of small-fastener examples from Bossard. The table describes tightening and does not establish OEM breakaway. Running resistance is modeled as Coulomb drag at 20% of breakaway after 5° of real screw travel. The modeled driver supplies up to 1.30 N·m. Its bore alignment and seated-bit geometry gate torque transfer to the actual screw joint. The low-torque local spindle animation indexes the unloaded tool. Loaded hex transfer is a reduced joint torque model, and the model omits local contact stress and full reaction torque at the robot wrist.

| Assumed breakaway | Assumed running resistance | Modeled capacity minus breakaway |
| ---: | ---: | ---: |
| 0.25 N·m | 0.05 N·m | 1.05 N·m |
| 0.50 N·m | 0.10 N·m | 0.80 N·m |
| 1.00 N·m | 0.20 N·m | 0.30 N·m |

The sensitivity checks exercise those three points and confirm that output below breakaway stalls, and that an unseated bit transfers zero. These are model checks. A physical evaluator marks each head, counts at least two complete loosening turns from the fixed datum on video, then checks that each screw is loose and the supported module slides without forcing. Observe the module throughout separation and check that it stays released and still in the cradle for one second. [Bossard torque reference](https://www.bossard.com/global-en/-/media/bossard-group/website/documents/technical-resources/en/f-047-en.pdf) · [HIOS CL-6500](https://hios.com/EN/products/driver/CL-6500/)


## Revised simulation evidence

The 0.50 N·m baseline scripted run scored **1.00**. With one fastener left engaged the rollout scored **0.00**. Spinning outside the sockets transferred zero modeled hex torque and scored **0.00**. The unsupported separation, drop and held-at-cradle checks are scorer regressions. All results are simulation evidence.
