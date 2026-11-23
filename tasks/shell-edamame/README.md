# Shell an edamame pod into a serving bowl

**Societal relatability.** Preparing edamame requires opening a flexible pod without scattering the beans, transferring the beans to a serving bowl, and clearing the shell as food waste.

## Physical setup and reset

Use a stock two-arm YAM tabletop station with its overhead and wrist cameras. Put an intact, unopened three-bean edamame pod on a 185 mm ceramic plate. Use a 156 mm ceramic serving bowl and a square 165 mm open plastic food-waste caddy about 133 mm high. The simulated pod measures 66 mm long, 17 mm wide and 14 mm thick. The beans are about 9–10 mm across. Measure the actual pod and photograph the setup. Use a fresh three-bean pod of the same preparation and temperature for each trial. Cultivar measurements provide size references. [Iowa State variety trial](https://www.extension.iastate.edu/organicag/files/documents/edamamenk02.pdf) · [Virginia Tech cultivar trial](https://doi.org/10.1002/plr2.20140)

Coordinates are in metres in the station frame. The tabletop is at Z = 0.7475. Place the plate at (0.357, −0.052), the serving bowl at (0.445, 0.124), and the caddy at (0.515, −0.222). Orient the pod's long axis along X. Check that all containers are empty, the seam is closed, and the arms are in their ready poses.

## Scoring

Use the overhead and wrist views to confirm robot contact on each half, opening and bean removal. Verify the final bean count by direct inspection or from the overhead view. Use a clock or video frames to verify the hold. Every positive tier requires meeting the opening criterion. Award the highest tier met.

| Score | Required outcome |
| --- | --- |
| 0.00 | The pod stays closed or the trial invalidates. |
| 0.25 | The robot handles both halves, exposes the interior along the seam, and opens the halves by more than 35°. |
| 0.50 | One extracted bean is wholly inside the serving bowl. |
| 0.70 | Two extracted beans are wholly inside the serving bowl. |
| 0.85 | All three extracted beans are wholly inside the serving bowl. |
| 1.00 | All three beans remain in the bowl, the empty shell is within the caddy, and the final state holds with both arms clear for at least one second. |

A bean escaping while the pod remains closed, or any pod section or bean leaving the tabletop work area, invalidates the trial. The simulator checks the three independent cohesive seam links as the opening witness. It counts beans only when the whole width fits inside the bowl boundary. Shell retention uses the four physical shell links, their motion, clearance from both arms and contact reports.

## Run the trial

Run these commands from this task folder with the provided Isaac Lab-Arena environment and stock YAM embodiment installed.

```bash
source ~/IsaacLab-Arena/.venv/bin/activate
source ~/IsaacLab-Arena/yam.env
uv pip install --python "$VIRTUAL_ENV/bin/python" --no-build-isolation --no-deps -e .
instruction=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')
export EDAMAME_JOINED_SHELL=1 EDAMAME_PLATE_RECOVERY_LAYOUT=1

EDAMAME_LIFT_FIRST=1 EDAMAME_END_GRASPS=1 EDAMAME_POUR_AFTER_LIFT=0 \
  EDAMAME_ANGLED_APPROACH=0 EDAMAME_PLATE_RECOVERY=1 EDAMAME_MEASURE_ARM_CONTACT=1 \
  EDAMAME_GRIP_FORCE_LIMIT_N=3 EDAMAME_RELEASE_ABOVE=1 \
  inspect-robots run --embodiment yam_arena --policy edamame_scripted --scorer task_score \
  -P hold_s=2 -P oracle=true -P plate_transfer=true \
  -E task="$PWD/arena_task.py" -E headless=true -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$instruction" --seed 0 --max-steps 2000 --no-prompt --store-frames \
  --log-dir /tmp/edamame-expert

inspect-robots run --embodiment yam_arena --policy dream --scorer task_score \
  -E task="$PWD/arena_task.py" -E headless=true -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$instruction" --seed 0 --max-steps 2000 --no-prompt --store-frames \
  --log-dir /tmp/edamame-dream
```

The run limit is 2,000 actions, or 100 s at 20 Hz. The expert reads shell and bean poses, seam states and fingertip loads, and sends stock YAM joint and gripper commands. It opens the pod over the plate, collects each bean, regrips the empty shell, and lifts it above the caddy rim before opening the jaws. The first full-score run reached 1.00 in 1,502 actions with zero monitored inter-arm contacts. Seeds and evaluation scores are recorded in `task.json`.

## Simulation fidelity and attribution

Adjacent convex sectors approximate the continuous curved wall. The three upper sections bend about compliant hinges. Equal and opposite cohesive forces act on the seam until measured deformation breaks each bond. The three beans are separate rigid bodies. Material stiffness, seam forces, bend limits and friction are development estimates. A physical reproduction should record the opening and bean-extrusion forces on several pods from the selected batch.

The pod, beans and containers are procedural task geometry. Run `python edamame_geometry.py` to regenerate the USDs. This folder uses the stock YAM and camera models from the installed embodiment and does not redistribute them.
