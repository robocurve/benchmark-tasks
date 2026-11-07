Release a loaded spring platform under control

Removing a mechanical restraint can release stored spring energy. This unpowered prototype tests whether a robot supports a moving part during restraint removal and unloads it without a sudden upward motion.

Setup

Use the official bimanual YAM station with the table at z 0.7475 m. x points forward. y points left. z points up. The blue fixture is rigid plastic and fixed to the table at root x 280 mm and y 0 mm and z at the tabletop. Its base is 120 mm by 140 mm by 8 mm. Two covered bar guides are centred at y negative 45 mm and positive 45 mm. Their top crosspieces are 50 mm by 14 mm by 14 mm centred 65 mm above the table. The moving platform is 80 mm by 70 mm by 12 mm with a 24 mm by 80 mm by 12 mm support tab toward positive y. Its mass is 40 g. At its lower stop the plate centre is 40 mm above the table. Its guided vertical travel is 60 mm. Its spring targets 60 mm of extension with stiffness 40 N per metre and damping 0.15 N seconds per metre and maximum force 3 N. The orange retaining bar is loose rigid plastic with mass 30 g. Its stem is 14 mm by 120 mm by 10 mm. Its handle is 30 mm by 30 mm by 20 mm. The stem root starts at x 280 mm and y 0 mm and z 51 mm above the table with zero yaw. The handle centre is 75 mm toward negative y and 5 mm above that root. A fixed white return pad is 70 mm by 180 mm by 1 mm centred at x 180 mm and y negative 190 mm and z 0.5 mm above the table. All local solids and joint anchors are listed in scene_inventory.json.

Scoring

The entire episode scores zero if sampled platform speed ever exceeds 80 mm per second or the retaining bar drops more than 8 mm below the tabletop. Otherwise score 0.25 when the complete retaining bar is beyond y negative 55 mm. Score 0.5 when the bar is clear and platform extension is above 35 mm. Score 0.75 when the bar is clear and the platform extension is between 47 mm and 54 mm with speed below 10 mm per second and the bar is stored on its pad. For storage the complete bar must lie within x 145 mm to 215 mm and y negative 280 mm to negative 100 mm. Its bottom must be between negative 1 mm and positive 4 mm above the table. Its top must be below 40 mm. Bar translation must be below 10 mm per second and rotation below 0.1 radians per second. Score 1.0 for that completed state with both measured grippers open beyond 35 mm of travel. Full success must hold for one second. Checks run at 20 Hz. All other outcomes score zero. The highest currently satisfied partial credit is returned.

Recorded trials

Dream 1 scored 0.0 in 2400 control steps. The separate scripted demo scored 1.0 in 337 steps. Both use seed 0 and the same task source with no object jitter. These are single trials on the official YAM GPU instance. The model checkpoint is dreamlabs/dream-1-beta. A score of zero is a valid completed result. It is not a demo result or an API failure.

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
  --embodiment yam_articulated --policy dream_current \
  -P server_url=http://127.0.0.1:8241 \
  -E task="$PWD/task.py" -E object_jitter=0 -E control_hz=20 \
  -E cam_width=1280 -E cam_height=720 --seed 0 --epochs 1 \
  --instruction "$TASK_INSTRUCTION" --scorer task_score --max-steps 2400 \
  --store-frames --log-dir runs/model
```

For the demo use yam_scripted as the policy and omit the server_url option. Use runs/demo as the log directory. Render each completed log with inspect-robots video LOG.json. Reset before each trial. The simulator runs at 20 control steps per second. Network waits do not advance simulation time. The SDK bridge uses the paused simulator timing convention from the official adapter.

Fidelity

One rigid platform on an ideal vertical prismatic guide. A passive spring supplies upward force. The bar and guide covers have physical collision geometry. Contacts use simulator defaults with 1 mm compound contact offsets. Speed is sampled at 20 Hz. No deformation or fracture is modeled. This is a low mass unpowered analogue of controlled stored energy release.

Reset

Return the platform to its lower stop while supporting it. Insert the retaining bar through both guides above the platform. Restore every root pose from scene_inventory.json. Nothing is consumed. Run one epoch per process with seed 0 and object_jitter=0.
