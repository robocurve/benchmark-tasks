# Torque a busbar lug bolt with a click-type torque wrench

Instruction given to the policy: `Torque the lug bolt with the torque wrench on it. The wrench is already set to the bolt's torque. Then let go.`

Files: `task.json`, `click_wrench_stop_at_click.py` (scene, `success()`, partial-credit `score()`, the scripted `demo()` and an offline
self-test), `README.md`.

## Environment

The program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim, L40S) with yam-arena and inspect-robots as it
installs them. Before every shell: `source ~/IsaacLab-Arena/yam.env`; the Dream key is in `~/.dreamlabs/credentials`.

## Real hardware and calibration

The simulated joint is this real one, at full size (sizes as in `setup`):

- **Joint:** a 20 x 6 mm copper flat bar on two glass-polyester standoff insulators; a tinned-copper tube lug for
  16 mm2 (about 6 AWG) cable with an M5 hole (palm 18 x 12 x 3 mm, barrel 8 mm across), crimped on a flexible copper
  cable, black PVC, 8 mm over the insulation; an M5 x 0.8 x 16 mm class 8.8 zinc-plated steel hex bolt (8 mm across
  flats) with a flat steel washer, through the lug palm and the bar into an M5 self-clinching nut pressed into the bar.
- **Specified torque: 3 N*m, marked on the lug and on the panel label** (the label is not modelled). Tolerance
  90-110 % (2.7-3.3 N*m).
- **Wrench:** an adjustable click-type torque wrench, ISO 6789-1 Type II class A (permissible deviation +/-6 % under
  10 N*m), 1/4-inch square drive, 2-10 N*m range, about 26 cm long; the grip's load mark 21.5 cm from the drive axis,
  so 3 N*m takes **14 N at the load mark** (snug 7 N; the tolerance's top 15.3 N). Check it at 3 N*m on a torque-wrench
  tester before the trials and calibrate it as ISO 6789-2 asks.
- **Inline transducer (in the scene):** a rotary torque-angle transducer with 1/4-inch square drive in and out
  (strain-gauge torque cell, 0-10 N*m, angle encoder, peak hold), 26 x 26 mm and 4 cm tall, between the wrench's drive
  and an 8 mm 6-point socket. It raises the wrench to 9.7 cm above the table, as in the sim.
- **Snug (the zero for every angle):** run the bolt down by hand, then tighten until the transducer reads 1.5 N*m
  (half the marked torque). Paint a line across the bolt head, washer and lug. The sim starts here: the bolt turns on
  at half the set torque.
- **Torque-angle calibration** (the turn-of-nut idea: a defined snug, then an angle): on the actual joint, or a joint
  simulator built from the same bolt lot, washer, lug, bar and nut, tighten from snug slowly and steadily and record
  the transducer's torque against angle: the angle at 3 N*m and at 3.3 N*m. On three sacrificial bolts carry on until
  the curve's slope falls (yield).
  - The sim's angles are an estimate from the joint's stiffness, not a measurement. With a nut factor of 0.2 the
    preload is 1 kN per N*m, so snug to set adds 1.5 kN; the bolt (with its head and engaged thread) and the lug-bar
    stack give about 7.5 um per kN, so about 11 um of stretch, about 5 deg on a 0.8 mm pitch (the sim uses 4.9 deg, a
    straight line of 3.27 deg per N*m above snug). On that line 3.3 N*m (110 %) is at 5.9 deg. A class 8.8 M5 starts
    to yield under tightening at about 7 kN of preload (640 MPa on 14.2 mm2, less the share taken by thread torsion),
    about 6.9 N*m or 230 % of the set torque, which the same line puts at 17.7 deg, as in the sim.
- **Angles are sim-only.** A real trial is scored from the transducer's peak-hold reading, not from angles.
- **Peak torque in a real trial:** the transducer stays in the drive line for every trial, and its peak-hold reading
  is the trial's peak torque P: 2.7-3.3 N*m with the click is in tolerance (1.0); over 3.3 N*m, as when the hand pulls
  on through the click, scores 0. The video shows the click (the handle's kink).
- **Force scale:** the sim's torques are 1/7 of the real ones (0.43 N*m set, 2 N at the grip, not 14 N), because
  the simulated arm is soft (at 2 N on the grip it already lags its command by about 11 mm). Sizes, angles and ratios (snug 50 %, tolerance +/-10 %,
  yield 230 %) are the real joint's.

## Offline check, no GPU

From the root of this repository, with yam-arena importable:

```bash
python tasks/click-wrench-stop-at-click/click_wrench_stop_at_click.py
```

It asserts the scoring rules on hand-built states and replays every arm's demo plan through the kinematics at 20 Hz
(every joint inside the URDF limits, IK error under 0.5 mm). Expected output:

```
scoring: 23 cases pass (set torque 0.43 N*m = 2.0 N at the grip, reached 4.9 deg past snug; tolerance 90-110 % = 3.9-5.9 deg; yield 0.99 N*m = 4.6 N at 17.7 deg; real joint x7.0: 3 N*m set)
left: 18.2 s plan, worst IK 0.090 mm / 0.013 deg
right: 18.2 s plan, worst IK 0.089 mm / 0.015 deg
```

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/click-wrench-stop-at-click/click_wrench_stop_at_click.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Torque the lug bolt with the torque wrench on it. The wrench is already set to the bolt's torque. Then let go." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/click-wrench-stop-at-click-dream
```

`task_score` is 1.0 when `success()` has held for 1 s (the episode ends with `termination_reason="success"`),
otherwise `score()` at the last step. The CLI prints `task_score: <value>`.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/click-wrench-stop-at-click/click_wrench_stop_at_click.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Torque the lug bolt with the torque wrench on it. The wrench is already set to the bolt's torque. Then let go." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/click-wrench-stop-at-click-demo
```

The demo is `demo()` in the task file: open loop and deterministic, 18 s of plan. In sim it pulls the grip
round until the wrench clicks (the bolt's peak 4.60 deg past snug, 97 % of the set torque, inside the 90-110 %
band), stops, eases off and opens the fingers; the bolt springs back to 4.10 deg and the handle returns in line.
The hand lifts 7 cm straight up and moves back to its park pose, about 16 cm from the grip and over 10 cm outside
the wrench's bounding box, so the success hold (1 s) only starts once the release is plain in the video. The
episode ends on success at step 318 of the 1,200-step run limit (about 3.8x).

## Videos and screenshot

```bash
for log in ~/yam-eval/click-wrench-stop-at-click-*/*.json; do inspect-robots video "$log"; done
ffmpeg -y -i ~/yam-eval/click-wrench-stop-at-click-demo/frames/<run>/scene-0-e0_top_cam.mp4 -frames:v 1 setup.png
```

## Run limit, runtime and seeds

- Run limit: 1,200 steps (60 seconds at 20 Hz).
- One fixed layout: `-E object_jitter=0`, so the seed does not change the start. Dream-1 is hosted and not seeded;
  the demo is deterministic.
- Runtime per trial on an L40S: roughly 1 minute of wall clock per 400 steps for Dream-1, plus Isaac Sim warm-up on
  the first run.

## Reset between trials

Back the bolt off (fit a new bolt if its peak went over 110 %); run it down by hand, then to 1.5 N*m on the transducer, and paint-mark the head and lug; check the wrench is set to 3 N*m and fit it on the bolt in line, the handle pointing toward the right arm.
