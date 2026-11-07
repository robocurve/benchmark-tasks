Stow a footrest with its index latch

Clearing an empty footrest from a transfer space is useful when preparing an assistive device. The training module requires a separate hand to withdraw its latch before the footrest can swing. No person or occupant is present.

Setup

Use the official bimanual YAM station. The tabletop is at z 747.5 mm. x points forward. y points left. z points up. All props are rigid PA12 polymer prototypes. All joint positions start at zero. The footrest module has an initial outer envelope of 100 by 200 by 82.5 mm. Its authored root is at x 160 mm and y 0 mm and z 747.5 mm. Its initial yaw is 0.0 radians. Its root is fixed to the tabletop. The spring latch has an initial outer envelope of 80 by 185 by 89 mm. Its authored root is at x 160 mm and y 115 mm and z 792.5 mm. Its initial yaw is 0.0 radians. Its root is fixed to the tabletop. The transfer space has an initial outer envelope of 160 by 110 by 1 mm. Its authored root is at x 160 mm and y -115 mm and z 748 mm. Its initial yaw is 0.0 radians. Its root is fixed to the tabletop. The inventory lists each collider and body mass and joint anchor and travel limit. Local solid positions are measured from the named body root. Outer envelope sizes describe the reset pose.

Scoring

During the swing the latch must stay withdrawn by at least 25 mm after 8 degrees until the footrest first reaches 87 degrees. A later return below 80 degrees starts this requirement again. Violating it scores zero. The latch must first retract at least 25 mm. If the footrest rotates beyond 8 degrees before this happens the entire episode scores zero. Correct latch retraction earns 0.25. After that a footrest angle from 87 degrees to 92 degrees earns 0.5. Latch travel at most 3 mm with angular speed below 0.03 radians per second raises this to 0.75. Both measured grippers open beyond 35 mm raises this to 1.0. Full success must hold for 2 seconds. At this angle the footplate clears the yellow transfer region by at least 10 mm. All other outcomes score zero.

Fidelity

Rigid PA12 polymer props on ideal passive joints. A spring returns the separate pin into actual slots in the footrest index wheel. Contacts use 1 mm offsets. The latch has simulated clearance during final capture. Full score requires final index angle and rest rather than a proof load. The station is unoccupied and unpowered. No person or human load is simulated. Fracture and wear are omitted.

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
