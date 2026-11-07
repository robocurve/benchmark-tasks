Feed a captive nut under channel lips

A fastener in an enclosed channel must enter through the open end. Its wide head must finish under both retaining lips. This tests a constrained assembly path while the other arm braces the loose channel.

Setup

Use the official bimanual YAM station. The tabletop is at z 747.5 mm. x points forward. y points left. z points up. All props are rigid PA12 polymer prototypes. All joint positions start at zero. The channel has an initial outer envelope of 92 by 330 by 47 mm. Its authored root is at x 260 mm and y 30 mm and z 747.5 mm. Its initial yaw is 0.0 radians. Its mass is 0.6 kg. Its root is free to move. The nut has an initial outer envelope of 44 by 30 by 63 mm. Its authored root is at x 170 mm and y -190 mm and z 762.5 mm. Its initial yaw is 0.0 radians. Its mass is 0.035 kg. Its root is free to move. The mark has an initial outer envelope of 90 by 3 by 1 mm. Its authored root is at x 260 mm and y 60 mm and z 748 mm. Its initial yaw is 0.0 radians. Its root is fixed to the tabletop. The inventory lists each collider and body mass and joint anchor and travel limit. Local solid positions are measured from the named body root. Outer envelope sizes describe the reset pose.

Scoring

Channel rotation above 5 degrees also scores zero. Entry requires nut tilt and yaw below 5 degrees. Entry credit clears if the nut centre leaves x 250 mm to 270 mm or y negative 100 mm to positive 100 mm. It also clears if nut height differs from its entry height by 8 mm or more or tilt or yaw leaves that limit. Score zero if the channel centre ever moves more than 5 mm from its starting position or if the nut goes more than 5 mm below the table. The nut must pass the front entry gate with x within 5 mm of 260 mm and y between negative 85 mm and negative 55 mm and its lower face within 6 mm of z 757.5 mm. This earns 0.25. After entry a nut x within 6 mm of 260 mm and y between negative 20 mm and 66 mm with the wide head upper face below z 786.5 mm earns 0.5. A seated nut must have x within 4 mm of 260 mm and y within 5 mm of 60 mm and its lower face within 4 mm of z 757.5 mm. Tilt and yaw must be below 5 degrees. It must move below 8 mm per second and 0.1 radians per second. This earns 0.75. Both measured grippers open beyond 35 mm raises this to 1.0. Full success must hold for 2 seconds. All other outcomes score zero.

Fidelity

The channel and nut are rigid printed polymer solids. The broad nut head physically fits below both rigid lips. There is no threaded engagement or screw tightening. The long nut handle gives the gripper access above the lips. Contacts use PhysX defaults with a 1 mm contact offset. No elastic material deformation is simulated.

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

Novelty comparison

This replaces our original PR 181. It is not a new idea counted again. PR 770 from another team also handles a channel nut. That task inserts a narrow T nut through the top opening and turns it by a quarter turn. This task uses a 44 mm head that cannot pass through the 24 mm top slot. It must enter from the open channel end and slide below both fixed lips. The loose channel must stay within its pose limits. The shared channel hardware makes this a close neighbor. The entry geometry and required operation are different.

## Selected current trials

Dream 1 scored 0.0 in 2400 control steps. The separate scripted demo scored 1.0 in 512 control steps. Both used the same task source and seed 0. Neither trial reported an evaluation error. These are single trials and do not estimate a general success rate.

The task source SHA256 is `cf4cb14528fe26747d9236890febfaba3ec995d033422f85025ff6784dec1683`. validation.json identifies the selected receipts. video_audit.json records the checksums and native camera frame counts. The score plot shows measured scores through the recorded trajectories.

This submission replaces PR 181. It preserves our original task idea and is not an additional new idea.
