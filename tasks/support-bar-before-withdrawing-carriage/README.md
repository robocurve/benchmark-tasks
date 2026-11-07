Support a bar before withdrawing its carriage

A support must not be removed from beneath a precision part until another support carries the load. This task checks the order and continuous control during a two arm transfer. The optical bar is represented by a printed training part.

Setup

Use the official bimanual YAM station. The tabletop is at z 747.5 mm. x points forward. y points left. z points up. All props are rigid PA12 polymer prototypes. All joint positions start at zero. The support carriage has an initial outer envelope of 240 by 220 by 80 mm. Its authored root is at x 260 mm and y -70 mm and z 747.5 mm. Its initial yaw is 0.0 radians. Its root is fixed to the tabletop. The optical bar has an initial outer envelope of 190 by 30 by 60 mm. Its authored root is at x 260 mm and y 0 mm and z 837.5 mm. Its initial yaw is 0.0 radians. Its mass is 0.1 kg. Its root is free to move. The storage cradle has an initial outer envelope of 230 by 80 by 30 mm. Its authored root is at x 160 mm and y 190 mm and z 747.5 mm. Its initial yaw is 0.0 radians. Its root is fixed to the tabletop. The inventory lists each collider and body mass and joint anchor and travel limit. Local solid positions are measured from the named body root. Outer envelope sizes describe the reset pose.

Scoring

Score zero if the bar lower face ever falls below z 754.5 mm or tilt exceeds 10 degrees. A downward change greater than 12 mm between adjacent 20 Hz observations also scores zero. Let q be the signed carriage travel in millimetres. Retraction makes q negative. The support line has y equal to q. Its positive y edge is q plus 12.5 mm. The carriage is clear only after more than 105 mm of retraction and when the bar lower y bound is more than 5 mm beyond that edge toward positive y. Until it is actually clear the bar lower face must remain at or above z 814.5 mm. Retraction beyond 40 mm earns 0.25 only if the bar centre is also more than 40 mm toward positive y from the support line. Moving both parts together earns no progress. Actual clearance while meeting the safety limits earns 0.5. After that the bar centre must be within 4 mm on every axis of x 160 mm and y 190 mm and z 787.5 mm. Yaw must be below 5 degrees. Translation must be below 8 mm per second and rotation below 0.1 radians per second. This earns 0.75. Both measured grippers open beyond 35 mm raises this to 1.0. Full success must hold for 2 seconds. All other outcomes score zero.

Fidelity

The optical bar is a rigid printed training surrogate with a raised handling tab. No glass fracture or optical performance is simulated. The carriage is a passive prismatic joint with physical support contacts. The white cradle has a floor and two retaining rails. Rigid contact and numerical drop limits approximate damage avoidance. No person or powered machinery is present.

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
