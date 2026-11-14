# Tap a beer keg with a Sankey D coupler

Instruction given to the policy: `Tap the keg with the coupler.`

The instruction does not say how. The coupler has to go onto the valve neck with its ears on the lugs, be turned a
quarter turn clockwise to its stop, and only then have its handle pushed down past over centre, which drives the
probe into the valve. Pushing the handle before the coupler is turned home latches the score to 0.

Files: `task.json`, `keg_sankey_coupler.py` (scene, `success()`, partial-credit `score()`, the scripted `demo()`
and an offline self-test), `mechkit.py` (joints, sliders, catches and a revolute catch added to the stock runtime in
memory), `README.md`.

## Environment

The program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim, L40S) with yam-arena and inspect-robots as it
installs them. Before every shell: `source ~/IsaacLab-Arena/yam.env`; the Dream key is in `~/.dreamlabs/credentials`.

## Offline check, no GPU

```bash
python tasks/keg-sankey-coupler/keg_sankey_coupler.py
```

It asserts the scoring rules on hand-built states and replays both arms' demo plans (and the degenerate
`no_turn` variant) through the kinematics at 20 Hz (joints inside the URDF limits, IK error under 0.5 mm).

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/keg-sankey-coupler/keg_sankey_coupler.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Tap the keg with the coupler." \
  --scorer task_score --max-steps 1400 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/keg-sankey-coupler-dream
```

`task_score` is 1.0 when `success()` has held for 1 s, otherwise `score()` at the last step.

## Run and score the full-score demo

Same command with `--policy yam_scripted` (log dir `~/yam-eval/keg-sankey-coupler-demo`). The demo is `demo()`:
open loop and deterministic. The right arm grips the coupler's body, carries it (beer and gas lines trailing) over the
chime and sets it over the valve neck, ears on the lugs, lets go and presses it down by the beer outlet; the left arm comes down beside the handle's root and sweeps the coupler a
quarter turn clockwise to the stop, then rides the handle's top face down past over centre to its stop and clears.

The beer and gas lines are chains of rigid segments on soft spring joints, hung from the coupler's outlet and inlet
and welded at their far ends to a fixed tap tower and regulator (`kit.tie` in mechkit.py, authored with the scene;
no joint is created during a run). Pulling either end more than 5 mm off its fitting latches the score to 0.

Degenerate check: `KEG_DEMO=no_turn` runs the same demo without the quarter turn (seat, then press the handle);
it scores 0. `KEG_DEBUG=1` prints the coupler's state every second.

## Videos and screenshot

```bash
for log in ~/yam-eval/keg-sankey-coupler-*/*.json; do inspect-robots video "$log"; done
ffmpeg -y -i ~/yam-eval/keg-sankey-coupler-demo/frames/<run>/scene-0-e0_top_cam.mp4 -frames:v 1 setup.png
```

## Run limit, runtime and seeds

- Run limit: 1,400 steps (70 seconds at 20 Hz).
- One fixed layout: `-E object_jitter=0`. Dream-1 is hosted and not seeded; the demo is deterministic.
- Runtime per trial on an L40S: roughly 1 minute of wall clock per 400 steps, plus Isaac Sim warm-up.

## Reset between trials

Lift the handle back up past centre, turn the coupler back 90 deg and lift it off the valve, stand it on the drip
tray with the handle pointing at the robot, and set the keg on its outline with the lugs left and right.
