Assemble a split spacer around a captive shaft

A complete spacer cannot pass over the wide shaft head. This task tests assembly around a captive shaft without taking the shaft apart. The same access problem occurs when maintaining machines with inaccessible shaft ends.

Setup

Use the official bimanual YAM station. The tabletop is at z 747.5 mm. x points forward. y points left. z points up. All props are rigid PA12 polymer prototypes. All joint positions start at zero. The shaft has an initial outer envelope of 107 by 300 by 72 mm. Its authored root is at x 280 mm and y 0 mm and z 747.5 mm. Its initial yaw is 0.0 radians. Its root is fixed to the tabletop. The left half has an initial outer envelope of 80 by 115 by 35 mm. Its authored root is at x 160 mm and y 160 mm and z 757.5 mm. Its initial yaw is 0.0 radians. Its mass is 0.055 kg. Its root is free to move. The right half has an initial outer envelope of 80 by 115 by 35 mm. Its authored root is at x 160 mm and y -160 mm and z 757.5 mm. Its initial yaw is 0.0 radians. Its mass is 0.055 kg. Its root is free to move. The inventory lists each collider and body mass and joint anchor and travel limit. Local solid positions are measured from the named body root. Outer envelope sizes describe the reset pose.

Scoring

Score 1.0 when both halves enclose the shaft and both seam endpoint gaps are at most 4 mm. Each half must have its lower face within 3 mm of the table. Tilt and yaw must be below 4 degrees. Translation must be below 8 mm per second and rotation below 0.1 radians per second. The shaft must have at least 1 mm of clearance to each inner arm face and each inner crossbar. For partial alignment credit a seam may be from 6 mm past the shaft centre to 20 mm before it along the entry direction. Score 0.4 when at least one half meets those alignment checks but the full enclosure is incomplete. Full enclosure scores 0.75 until both measured grippers are open beyond 35 mm. Full success must hold for 2 seconds. A loose part more than 5 mm below the tabletop makes the entire episode score zero. All other outcomes score zero. Checks run at 20 Hz.

Fidelity

Rigid PA12 polymer halves surround a captive square shaft beneath a wide head. The head prevents lowering a closed collar from above. Actual rotated collider corners recover each half root before measuring shaft clearance and both seams. There is no snap fit or material deformation. Contacts use simulator defaults. This is an unpowered geometric assembly prototype.

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

For the demo use yam_scripted and omit the server_url option. Use a separate log directory. Each trial starts from a fresh reset. Restore both halves to their seam midpoint starting positions. Keep the shaft and guides fixed. Nothing is consumed. Run one epoch with seed 0 and object_jitter=0.

The current model and demo receipts are stored in validation.json. Scores come from the measured task scorer. A failed API trial is not scored as a valid zero. Model session expiry fails the trial. The model has no scripted fallback. Native camera videos are 1280 by 720 at 20 frames per second. Every recording includes its initial frame and every control step. Render a completed log with inspect-robots video LOG.json.

The source hashes and video checksums describe the evaluated revision. This is an unpowered simulation prototype. The mechanical idea is unchanged from the earlier submission. The revision strengthens the safety gates and supplies separate genuine model evidence.

## Selected current trials

Dream 1 scored 0.0 in 2400 control steps. The separate scripted demo scored 1.0 in 302 control steps. Both used the same task source and seed 0. Neither trial reported an evaluation error. These are single trials and do not estimate a general success rate.

The task source SHA256 is `9bfc80f608a8c98d74f01ca58f7cb3122856681b47ae7c9a3fe2f33d3bd47d60`. validation.json identifies the selected receipts. video_audit.json records the checksums and native camera frame counts. The score plot shows measured scores through the recorded trajectories.

This submission replaces PR 179. It preserves our original task idea and is not an additional new idea.
