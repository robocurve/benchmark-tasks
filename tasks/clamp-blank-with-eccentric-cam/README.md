Clamp a blank with an eccentric cam

Machine loading needs a blank to stay registered after the robot lets go. This task combines registration with a passive cam clamp rather than stopping at placement.

Setup

Use the official bimanual YAM station. The tabletop is at z 747.5 mm. x points forward. y points left. z points up. All props are rigid PA12 polymer prototypes. All joint positions start at zero. The cam fixture has an initial outer envelope of 210 by 300 by 38 mm. Its authored root is at x 280 mm and y -35 mm and z 747.5 mm. Its initial yaw is 0.0 radians. Its root is fixed to the tabletop. The blank has an initial outer envelope of 60 by 154 by 65 mm. Its authored root is at x 160 mm and y 140 mm and z 769.5 mm. Its initial yaw is 0.0 radians. Its mass is 0.155 kg. Its root is free to move. The starting pad has an initial outer envelope of 80 by 80 by 8 mm. Its authored root is at x 160 mm and y 140 mm and z 751.5 mm. Its initial yaw is 0.0 radians. Its root is fixed to the tabletop. The inventory lists each collider and body mass and joint anchor and travel limit. Local solid positions are measured from the named body root. Outer envelope sizes describe the reset pose.

Scoring

Score zero if the blank falls more than 4 mm below the tabletop. Registered placement earns 0.25 when the blank centre is within 3 mm on each axis of x 280 mm and y 77 mm and z 788 mm with tilt and yaw below 5 degrees. Loading must be observed with the cam below 5 degrees. After loading cam travel of 45 degrees or more earns 0.5 while the blank stays registered. Closing past 92 degrees earns 0.75 only when the compliant stop is deflected by at least 0.1 mm and the signed cam to blank gap is between negative 2 mm and positive 1 mm. Blank speed must be below 8 mm per second and rotation below 0.1 radians per second. The gap is measured from the cam circle to the blank clamping body in its current XY frame. The cam radius is 28 mm and eccentricity is 8 mm. Both grippers open beyond 35 mm raises this to 1.0 only when no robot body is in contact with the blank or cam or compliant stop. Contact separation of 1 mm or less counts as contact. Full success must hold for 2 seconds. All other outcomes score the highest satisfied stage.

Fidelity

Rigid PA12 polymer props. The cam has a passive revolute joint and a physical end stop. The backstop spring stiffness is 1500 N per metre. Deflection provides a preload check. An XY circle to box gap approximates cam contact under the 5 degree tilt limit. A PhysX view checks robot contact each physics step with a 1 mm threshold. No deformation or fracture or calibrated holding load rating is claimed.

Run and reset

Use the official YAM launchable. Run from this task folder. Install the runtime in the simulator environment. Install the policy adapter included in this folder. It uses inspect_robots_dream with the current Dream SDK bridge. Install dreamlabs version 1.0.37 and dreamlabs-client version 1.1.3. Start policy/current_api_bridge.py in a separate process with DREAM_BRIDGE_PORT=8241 and JAX_PLATFORMS=cpu. The Dream credential stays outside the repository.

```bash
source /data/IsaacLab-Arena/yam.env
uv pip install --python /data/IsaacLab-Arena/.venv/bin/python --no-deps -e runtime -e policy
TASK_INSTRUCTION=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 inspect-robots run \
  --embodiment yam_articulated --policy dream_current \
  -P server_url=http://127.0.0.1:8241 \
  -E task="$PWD/task.py" -E object_jitter=0 -E control_hz=20 \
  -E cam_width=1280 -E cam_height=720 --seed 0 --epochs 1 \
  --instruction "$TASK_INSTRUCTION" --scorer task_score --max-steps 2400 \
  --store-frames --log-dir runs/model
```

For the demo use yam_scripted and omit the server_url option. Use a separate log directory. Each trial starts from a fresh reset. Restore every object to the starting pose in scene_inventory.json and every joint to zero. Nothing is consumed. Use one epoch with seed 0 and object_jitter=0.

The current model and demo receipts are stored in validation.json. Scores come from the measured task scorer. A failed API trial is not scored as a valid zero. Model session expiry fails the trial. The model has no scripted fallback. Native camera videos are 1280 by 720 at 20 frames per second. Every recording includes its initial frame and every control step. Render a completed log with inspect-robots video LOG.json.

The source hashes and video checksums describe the evaluated revision. This is an unpowered simulation prototype. The mechanical idea is unchanged from the earlier submission. The revision strengthens the safety gates and supplies separate genuine model evidence.

## Selected current trials

Dream 1 scored 0.0 in 2400 control steps. The separate scripted demo scored 1.0 in 396 control steps. Both used the same task source and seed 0. Neither trial reported an evaluation error. These are single trials and do not estimate a general success rate.

The task source SHA256 is `36bf8fffccdf4c9135f1fbea71d33794ee44a821945cf61e3e6cc6dfa86a0122`. validation.json identifies the selected receipts. video_audit.json records the checksums and native camera frame counts. The score plot shows measured scores through the recorded trajectories.

This submission replaces PR 180. It preserves our original task idea and is not an additional new idea.
