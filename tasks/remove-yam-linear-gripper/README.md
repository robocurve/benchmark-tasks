# Remove and stow a YAM Linear 4310 gripper

The de-energized donor wrist sits in a tabletop saddle at (248, 8, 54) mm relative to the table datum. The installed gripper weighs about 695 g. The custom 62 mm OD split collar carries two captive M3 × 0.5 screws with 5.5 mm heads. Each sits 27 mm behind the mating face, 36 mm above the flange axis and 15.5 mm to either side. The blue driver has a 2.5 mm hex bit and starts in a clutch-shoulder dock near (195, −192, 9) mm. The empty cradle is at (385, 188, 56) mm.

The approximately 206 g driver uses a spring-fed quill and a torque-limited spindle. The hex bit turns each head through physical contact. The simulator derives axial rise from the measured rotation and 0.5 mm pitch. The scripted policy uses measured position, tilt and quill compression to guide engagement. The collar and its guide bushings are a bench adapter. Check fit on stock hardware before a physical trial. The supplied I2RT CAD license is in `LICENSE.i2rt`.

The scorer awards 0.50 once both engaged heads have turned at least 670° and risen at least 0.85 mm. Full credit requires the old gripper stationary within 13 mm and 12° of the cradle target, with the tool and gripper released and both screws still loose for one second. Forced extraction or a drop invalidates the trial. Reset restores the installed gripper, seated captive screws, docked driver and empty cradle.

## Run the trials in YAM Arena

From this task folder, activate the existing Arena environment and load the instruction and limit from the task specification. Use a clean reset, seed 0 and zero object jitter. Both trials run at 20 Hz with the `task_score` scorer.

```bash
source ~/IsaacLab-Arena/.venv/bin/activate
source ~/IsaacLab-Arena/yam.env
instruction=$(jq -er .instruction task.json)
steps=$(jq -er '.run_limit | split(" ")[0] | gsub(",";"") | tonumber' task.json)
```

### Scripted demo

The recorded demo used the graphical display and 640×352 frames. Set `-E headless=true` to run without a display.

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted -P hold_s=3 \
  -E task=arena_task.py -E object_jitter=0 -E headless=false -E control_hz=20 \
  -E cam_width=640 -E cam_height=352 --instruction "$instruction" \
  --scorer task_score --seed 0 --max-steps "$steps" --no-prompt --store-frames \
  --log-dir logs/demo
```

### Dream-1

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=arena_task.py -E object_jitter=0 -E headless=true -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --instruction "$instruction" \
  --scorer task_score --seed 0 --max-steps "$steps" --no-prompt --store-frames \
  --log-dir logs/model_run
```

Read the final `task_score` under `metrics` in `inspect-robots inspect <saved-eval.json>`.

The scripted demo scored **1.0** in the saved evaluation `adhoc_9b2b58f0.json`.

Dream-1 scored **0.0** in the saved evaluation `adhoc_5e41a707.json`.
