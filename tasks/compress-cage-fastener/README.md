# Compress a spring cage fastener before installing it

This task tests active compression of two retaining legs while the other arm steadies a loose rack coupon. The legs must pass through a square opening and expand below it. The upper flange supports the fastener after release. This differs from seating a keyed plug in a socket because the robot must hold two spring loaded legs inward during insertion.

## Runtime

Use the official YAM Isaac Lab Arena environment. Install the local embodiment extension in that environment.

```bash
source /data/IsaacLab-Arena/yam.env
uv pip install --python /data/IsaacLab-Arena/.venv/bin/python --no-deps -e runtime -e policy
OPENBLAS_NUM_THREADS=1 python validate.py
```

The supplied runtime authors the geometry locally. It does not download asset meshes. Both YAM arms use the official rig and joint controllers. The fastener contains two passive prismatic joints. The coupon is a free rigid body.

```bash
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=2 MKL_NUM_THREADS=1
inspect-robots run --embodiment yam_articulated --policy yam_scripted \
  -E task="$PWD/task.py" -E object_jitter=0 -E control_hz=20 \
  -E cam_width=1280 -E cam_height=720 --seed 0 --epochs 1 \
  --instruction "Fit the blue spring cage fastener into the square opening in the rack coupon. Compress both spring legs before passing the hooks through the opening. Release the legs behind the coupon so both hooks retain it. Keep the coupon within 5 mm and 5 degrees of its starting pose. Release both grippers when the fastener is seated. All equipment is unpowered." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir runs/demo
```

## Fixture construction

Use rigid plastic prototype parts with ballast to match the masses. The fastener root is 40 g. Each sliding leg is 6 g. Each leg needs a guided 8 mm linear stroke. Its spring rate is 80 N per metre. The simulation uses linear damping of 1.2 N seconds per metre and a 2 N drive force limit. Mechanical travel stops define the end positions. The slider guides are represented by ideal prismatic joints.

The rack coupon has a 34 mm square opening in a 4 mm plate. Four feet elevate the underside of that plate to 56 mm above the tabletop. The top is 60 mm above the table. A long tab gives the supporting arm a separate grasp site. The complete loose fixture has mass 240 g.

The upper flange is 52 mm wide. Each retaining hook extends beyond the opening after release. The vertical gap between the upper flange underside and the hook upper face is 6 mm. This leaves 2 mm total vertical clearance around the 4 mm plate. The full part geometry and nominal root poses are listed in scene_inventory.json. Coordinates use metres. x points forward. y points left. z points up. The table height is 0.7475 m.

## Evaluation

The scorer observes the fixture and both passive joints at 20 Hz. It records whether both legs were compressed more than 5 mm at the same time. Full score also requires both legs to return within 1.5 mm of their outer stops. The fastener centre must be within 3 mm on each axis of its target. Tilt and yaw error must each be below 5 degrees. A half turn about z is equivalent. The final position must vary by less than 1 mm over 20 samples. Both measured grippers must be open beyond 35 mm of travel. Full success holds for one second.

The target centre is x 280 mm and y negative 70 mm and z 74 mm above the table. Partial score is 0.25 for reaching within 30 mm horizontally while lifted more than 20 mm. It is 0.5 for alignment within 6 mm with the required orientation. It is 0.75 for the completed installation before both grippers open. Moving the coupon more than 5 mm or rotating it more than 5 degrees makes the entire episode score zero. Dropping the fastener more than 8 mm below the tabletop also scores zero.

To reset the fixture place the coupon at its recorded root pose and place the fastener upright at its source pose. Return both passive sliders to their outer stops. Nothing is consumed. validation.json records the actual trials.

The crossing monitor checks each hook top in the actual coupon frame. Each hook must move from above 22 mm to 16 mm or below relative to the coupon root. Both legs must stay compressed by at least 4 mm while the hook tops cross this band. Starting another approach clears the crossing credit.

## Current model evaluation

The selected model evaluation uses Dream 1 with checkpoint `dreamlabs/dream-1-beta` and a limit of 2400 control steps at 20 Hz. It uses seed 0 and no object jitter. The separate scripted demonstration is not a model result.

Install `dreamlabs==1.0.37` and `dreamlabs-client==1.1.3` in the bridge environment. Start the supplied bridge with `DREAM_BRIDGE_PORT=8241 JAX_PLATFORMS=cpu python policy/current_api_bridge.py` in the bridge environment. Keep credentials outside this repository. Run the evaluation command with `--policy dream_current` and `-P server_url=http://127.0.0.1:8241` and a separate log directory. The bridge maps the overhead camera and both wrist cameras to the official YAM state fields. It uses the documented paused simulator convention for action timing. The simulator applies control at 20 Hz. Session expiry fails the trial rather than restarting its model memory.

Render the completed log with `inspect-robots video LOG.json`. Each selected camera video is native 1280 by 720 H264 at 20 frames per second. The model video must contain every recorded frame. No time padding is used. See validation.json for the final measured scores and source hashes after validation finishes.

## Selected current trials

Dream 1 scored 0.0 in 2400 control steps. The separate scripted demo scored 1.0 in 477 control steps. Both used the same task source and seed 0. Neither trial reported an evaluation error. These are single trials and do not estimate a general success rate.

The task source SHA256 is `9398a3efc8f82ad89b9ea7248faf81f9c8d988247cb27bb9eac39f6e4250cc49`. validation.json identifies the selected receipts. video_audit.json records the checksums and native camera frame counts. The score plot shows measured scores through the recorded trajectories.

This submission replaces PR 112. It preserves our original task idea and is not an additional new idea.
