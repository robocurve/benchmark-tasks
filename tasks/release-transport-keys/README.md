Remove transport keys before unloading a captive carrier

Commissioning and unloading automated equipment requires removing transport restraints before motion. This task tests a physical prerequisite and retention of the removed parts. Newport motion stage documentation describes a bracket that locks the carriage during transport. This is a small unpowered analogue of that prerequisite.

Setup

Use the calibrated bimanual YAM station. All prototype parts are rigid PLA. Ballast the loose parts internally to the stated masses. Secure the base and channel walls and retaining lips and return trays and recovery pad to the table. The horizontal origin is midway between the arm bases. x points away from the robots. y points left. z points up in the station world frame. The table is at z 0.7475 m. carrier has overall dimensions 80 mm by 130 mm by 55 mm. Its centre is x 0.3 m and y 0 m and z 0.795 m. Yaw is 0 radians. It is loose with mass 0.18 kg. stage base has overall dimensions 80 mm by 130 mm by 20 mm. Its centre is x 0.3 m and y 0 m and z 0.7575 m. Yaw is 0 radians. It is fixed. blue key has overall dimensions 26 mm by 22 mm by 71 mm. Its centre is x 0.3 m and y 0.04 m and z 0.785 m. Yaw is 0 radians. It is loose with mass 0.025 kg. orange key has overall dimensions 26 mm by 22 mm by 71 mm. Its centre is x 0.3 m and y -0.04 m and z 0.785 m. Yaw is 0 radians. It is loose with mass 0.025 kg. left return has overall dimensions 160 mm by 110 mm by 22 mm. Its centre is x 0.19 m and y 0.2 m and z 0.7585 m. Yaw is 0 radians. It is fixed. right return has overall dimensions 160 mm by 110 mm by 22 mm. Its centre is x 0.19 m and y -0.2 m and z 0.7585 m. Yaw is 0 radians. It is fixed. recovery pad has overall dimensions 90 mm by 150 mm by 1 mm. Its centre is x 0.17 m and y 0 m and z 0.748 m. Yaw is 0 radians. It is fixed. channel wall right has overall dimensions 130 mm by 12 mm by 44 mm. Its centre is x 0.3 m and y -0.075 m and z 0.7695 m. Yaw is 0 radians. It is fixed. retaining lip right has overall dimensions 130 mm by 26 mm by 6 mm. Its centre is x 0.3 m and y -0.062 m and z 0.7925 m. Yaw is 0 radians. It is fixed. channel wall left has overall dimensions 130 mm by 12 mm by 44 mm. Its centre is x 0.3 m and y 0.075 m and z 0.7695 m. Yaw is 0 radians. It is fixed. retaining lip left has overall dimensions 130 mm by 26 mm by 6 mm. Its centre is x 0.3 m and y 0.062 m and z 0.7925 m. Yaw is 0 radians. It is fixed. Each key stem is 8 mm square and 53 mm long. Each key handle is 26 mm by 22 mm by 18 mm. Two 14 mm square bores pass through the carrier and into the base. Their centres are at y plus or minus 40 mm. Retaining lips prevent lifting the carrier out. Each return tray has a 152 mm by 102 mm inner opening and a 4 mm floor. scene_inventory.json lists every solid in each compound object in local coordinates.

Scoring

Each correctly stored key is worth 0.25. Its complete bounding box must lie inside its return tray opening. The tray opening spans x 114 to 266 mm. The left opening spans y 149 to 251 mm. The right opening spans y negative 251 to negative 149 mm. Its bottom must be 3 to 7 mm above the table and its top at most 100 mm above the table. The carrier is worth 0.5 after both keys have cleared the fixture and both have been stored. Its centre must be within 6 mm on every axis of x 170 mm and y 0 mm and z 28.5 mm above the table. Tilt and yaw error must each be below 8 degrees. A half turn about z is equivalent. Its bottom must be within 3 mm of the recovery pad top at 1 mm above the table. All credited parts must move below 10 mm per second and 0.1 radians per second. The score is capped at 0.75 until both measured grippers are open beyond 35 mm of finger travel. Full success must hold for 1 second. The entire episode scores zero if the carrier moves more than 5 mm before both keys have been stored or while a key remains engaged. A key is clear when its lowest point rises above 41 mm above the table. A key is considered engaged when its centre remains within 20 mm of its original horizontal centre and its bottom is at most 41 mm above the table. Dropping any loose part more than 8 mm below the tabletop also scores zero. These checks run at 20 Hz. All other outcomes score the sum of the completed credits.

Recorded trials

Dream 1 scored 0.0 in 2400 control steps. The separate scripted demo scored 1.0 in 650 steps. Both use seed 0 and the same task source with no object jitter. These are single trials on the official YAM GPU instance. The model checkpoint is dreamlabs/dream-1-beta. A score of zero is a valid completed result. It is not a demo result or an API failure.

The model recording has 2401 frames at 20 frames per second. All camera recordings are native 1280 by 720 H264. Each model episode uses one continuous Dream session. Session expiry fails the trial. No scripted fallback supplies model actions.

Run from this task folder in the official YAM environment.

```bash
source /data/IsaacLab-Arena/yam.env
uv pip install --python /data/IsaacLab-Arena/.venv/bin/python --no-deps -e runtime -e policy
uv pip install --python /data/IsaacLab-Arena/.venv/bin/python dreamlabs==1.0.37 dreamlabs-client==1.1.3
```

Store the private Dream credential in ~/.dreamlabs/credentials with file mode 600. Start the bridge in another terminal in the same environment.

```bash
DREAM_BRIDGE_PORT=8241 JAX_PLATFORMS=cpu python policy/current_api_bridge.py
```

Run the model trial.

```bash
TASK_INSTRUCTION=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 inspect-robots run \
  --embodiment yam_industrial --policy dream_current \
  -P server_url=http://127.0.0.1:8241 \
  -E task="$PWD/task.py" -E object_jitter=0 -E control_hz=20 \
  -E cam_width=1280 -E cam_height=720 --seed 0 --epochs 1 \
  --instruction "$TASK_INSTRUCTION" --scorer task_score --max-steps 2400 \
  --store-frames --log-dir runs/model
```

For the demo use yam_scripted as the policy and omit the server_url option. Use runs/demo as the log directory. Render each completed log with inspect-robots video LOG.json. Reset before each trial. The simulator runs at 20 control steps per second. Network waits do not advance simulation time. The SDK bridge uses the paused simulator timing convention from the official adapter.

Fidelity

Rigid cuboid geometry with open bores and retaining lips. PhysX drives the official bimanual YAM rig. Compound contact offsets are 1 mm. Mass is uniform within each shape. Materials do not deform or break. No powered stage or electrical system is simulated. Contact forces are not graded. History checks run at 20 Hz. This is a scaled procedural analogue of transport restraint removal. Contact materials use simulator defaults.

Reset

Replace the carrier under both retaining lips. Insert both keys through the carrier into the base bores. Empty the return trays and recovery pad. Restore every pose from scene_inventory.json. Nothing is consumed. Run one epoch per process with seed 0 and object_jitter=0.
