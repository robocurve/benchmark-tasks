# Calibrate a fixed pointed tool

| Item | Rule |
|---|---|
| Tool | A pointed steel shaft stays fixed to the left gripper housing, its mount between the open fingers. |
| Reference | An exposed silver contact, 2 mm across, at x 27, y 22 cm (from the midpoint between the arm bases), 5.1 cm above the table. |
| Pose credit | 0.25 after one second of valid contact. Four poses earn 1.0. |
| Contact | Physical touch. Simulation: contact force at least 0.02 N. Real trial: a continuity tester beeps. |
| Tip position | Within 1 mm horizontally of the contact's centre and 0.25 mm vertically of its top. |
| Stillness | Tip drift at most 0.5 mm. Axis drift at most 2 degrees during the one-second hold. |
| Tool axis | Tilt 10–35 degrees. Each axis differs from every earlier axis by at least 15 degrees. |
| Fixed mount | Translation error at most 0.5 mm. Rotation error at most 0.5 degree. |
| Time limit | 2,400 control steps at 20 Hz: 120 seconds. |

## Run a trial

Use the official Robocurve launchable. Validated with the YAM adapter commit `0059d02` that it installs.

1. Install this package into the simulator's environment. On the Robocurve launchable, from the repository root:

```bash
source /data/IsaacLab-Arena/yam.env
uv pip install --python /data/IsaacLab-Arena/.venv/bin/python --no-deps -e tasks/tool-tip-calibration
```

It registers the embodiment `yam_tool_calibration` (the stock robot plus the bolted tool, the post and the contact
sensor) and the policy `tool_calibration_scripted` (the demo and the two failure checks).

2. For scoring tests without Isaac Sim, run:

```bash
python -m pytest -q tasks/tool-tip-calibration/test_scoring.py
```

3. For a demo on the official simulator, run from the repository root:

```bash
INSTR="$(python -c 'import json; from pathlib import Path; print(json.loads(Path("tasks/tool-tip-calibration/task.json").read_text())["instruction"])')"
inspect-robots run --embodiment yam_tool_calibration --policy tool_calibration_scripted \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  -E telemetry=runs/tool-calibration-demo/telemetry.jsonl \
  --instruction "$INSTR" --scorer task_score --max-steps 2400 --store-frames \
  --log-dir runs/tool-calibration-demo/inspect
```

4. For a model trial, replace the policy with `--policy dream`. Use the complete instruction from `task.json`.
5. For a hover check, use `--policy tool_calibration_scripted -P mode=hover`. Expected score: 0.0.
6. For a repeated-pose check, use `--policy tool_calibration_scripted -P mode=onepose`. Expected score: 0.25.

## Check a real trial

Only simple tools: a continuity tester, a protractor, calipers, a scribe, a steel rule with 0.5 mm marks and two
fixed phone cameras. Nobody reaches in while the arm moves; every reading comes from photos, video or the robot's
joint log.

1. Fix the pointed tool to the gripper housing with its rigid bracket, the mount between the open fingers. Measure
   from the tip to the gripper housing with calipers.
2. Wire a low-voltage continuity tester (buzzer) between the tool and the silver contact.
3. Scribe a witness line across the mount and the gripper housing, and photograph it with the rule.
4. Fix two phone cameras at the contact's height: one looking along y (it sees x and height) and one along x (it
   sees y and height), for example at x 27, y 2 and at x 47, y 22 cm. Use the main camera at 2x about 20 cm away,
   so each frame is at most 15 cm wide (many phone telephoto lenses can't focus that close), with focus and
   exposure locked on the contact. Film in 4K at 30 fps or more, with the buzzer audible.
5. **Calibrate the video:** before the run, hold a steel rule (0.5 mm marks) on the post top against the contact,
   square to each camera, and photograph it to fix that view's scale. A 15 cm wide 4K frame gives at least 25
   pixels per mm, so 0.5 mm is at least 12 pixels. Remove the rule. The 2 mm contact rechecks the scale in every frame.
6. **Contact:** the tester beeps. The contact stands 1 mm above the post, so the tip shows in silhouette in both
   views. The frames show the point on the contact's top: within 1 mm of its centre and 0.25 mm (6 pixels) of its
   top, so a touch on the contact's side does not count.
7. **Hold and drift:** a pose needs at least 1 s of unbroken beep (video timestamps); any second of it may count,
   as in simulation. In both views, mark where the tip meets the contact on 20 evenly spaced frames through that second
   (the simulator's 20 control steps), and convert to mm: `dx` from the view along y, `dy` from the view along x,
   `dz` averaged from both. Drift from the first mark, `sqrt(dx^2 + dy^2 + dz^2)`, must stay at most 0.5 mm.
8. **Lean:** on a still from each view, read the tool's angle from vertical with a protractor: `a` from the camera looking along
   y (tilt toward x), `b` from the camera looking along x (tilt toward y). Take `a` positive when the tool's top
   leans toward +x and `b` positive toward +y; otherwise the direction can come out 90 or 180 degrees off. Then `lean = atan(sqrt(tan(a)^2 + tan(b)^2))`, which must be 10 to 35 degrees, and
   `direction = atan2(tan(b), tan(a))`. A single side photo is not enough: a lean toward the camera looks vertical.
9. **Spread:** the angle between two poses is
   `acos(cos(lean1) cos(lean2) + sin(lean1) sin(lean2) cos(direction1 - direction2))`. It must be at least 15 degrees
   for every pair of poses. The same formula, from the first marked frame of a hold to its middle and last ones,
   checks the 2 degree axis drift.
10. **Mount:** after the trial, measure from the tip to the gripper housing with calipers again; it must agree with step 1
    within 0.5 mm. Photograph the witness line with the rule: no offset over 0.5 mm. The exact 0.5 degree mount
    limit is checked in simulation only.

The robot's joint log gives the same lean and direction through forward kinematics, as a cross-check.

## Results (November 12, 2026)

Measured on one L40S shared with other simulations, with the commands above (one episode each).

| Run | Score | Credited poses | Simulation time | Wall time |
|---|---|---|---|---|
| Scripted demo | 1.0 | 4 (at 14.4, 29.4, 46.0 and 73.4 s) | 73.4 s | 554 s |
| Hover 6 mm above the contact (`-P mode=hover`) | 0.0 | 0 | 120 s (the limit) | 934 s |
| One pose only (`-P mode=onepose`) | 0.25 | 1 | 19.3 s | 134 s |
| Dream-1 | 0.0 | 0 | 120 s (the limit) | 829 s |

- **The demo reads no simulator state.** It knows the tool pose only from the robot's own joint angles (forward
  kinematics; the tool is bolted at the stock tool point) and the reference position given in the setup. It computes
  the tip from the joint angles and treats the tip within 0.04 mm of the contact's known height as touching; it
  commands at most 3 mm past first touch, and the contact stops the tip. Peak contact force: 2.8 N.
- The smallest angle between two credited tool axes was 23.8 degrees.
- Every credited second was re-checked from the logged samples: contact force, tip position, stillness, lean,
  mount and axis spread all within the limits above.
- Runtime per trial: about 75 s of simulated time for the demo, 120 s for a full 2,400-step run.
- Seeds: one fixed layout; the seed changes nothing.
