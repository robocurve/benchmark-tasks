# Zero an unpowered robot arm joint: hold the link on its zero mark and insert the calibration pin

The robot lifts the free-swinging link of an unpowered arm joint onto its zero mark with one hand and holds it there,
while the other hand pushes the calibration pin through the aligned holes. The link is a 10 x 20 x 160 mm aluminium
arm with an actuator can on its end (150 g) on a free hinge 90 mm above the table; the pin is a 6 mm steel pin with a
16 mm knob in a guide on the housing. At the end the pin is home through the link's 7 mm window, the link rests on it
within 1 degree of the zero mark, and both grippers have let go. Robot builders and technicians do this on every arm
they build or service after fitting a motor or encoder.

The ability is holding a gravity-loaded hinged link on a mark to within a degree from one side while the other hand
drives a pin that fits only on that mark. Nothing is picked and placed and nothing is a lock or a thread. Physics, not
the instruction, makes both hands necessary: with the motors off the link falls to its -30 deg stop (gravity torque
0.12 N m at zero). The window sits 45 mm from the hinge and has 0.5 mm of play each way, so the pin passes only with
the link within about 0.64 deg of the mark. The hand that holds the link cannot also push the pin, and the hand that
pushes the pin has nothing to hold the link.

Instruction given to the policy (26 words; it states the goal only, never how):

`Zero the robot arm joint: raise the link to its zero mark, push the calibration pin home so the link rests on it, then let go.`

Files in this folder: `task.json`, `task.py` (scene, whole-run record, scoring and the scripted demo), `linkkit.py`
(articulated rigid-link bodies for yam_arena), `linkdemo.py` (closed-loop scripted-demo runner),
`tests/test_scoring.py` (scorer edge cases), `README.md`.

## Requirements

Built on the program's Brev launchable (Isaac Lab-Arena, Isaac Sim, Isaac Lab, one L40S). Versions used for the demo
and cheat runs below:

- inspect-robots 1.0.0 (`inspect-robots --version`)
- yam-arena at commit 0059d02a1a91 (`git -C ~/yam-isaac-lab-arena rev-parse --short=12 HEAD`)
- inspect-robots-dream 0.1.0, Dream checkpoint `dreamlabs/dream-1-beta` (for the Dream-1 trial)
- task file SHA-256, first 12 hex: 1d64e228670e (`sha256sum task.py | cut -c1-12`; equals `eval.environment_revision` in the
  Dream-1 log)

Before every shell: `source ~/IsaacLab-Arena/yam.env` (on our VM, `/data/IsaacLab-Arena/yam.env`).

`linkkit.py` and `linkdemo.py` must sit next to `task.py`. Importing `linkkit` installs an import hook that teaches
`yam_arena.scene` to spawn the joint assembly as a PhysX articulation (a fixed housing, a free revolute arm link with
limits, and a sprung prismatic pin with a brake for its detent, all USD joints with limits and drives spawned through
Isaac Lab's `ArticulationCfg`) and calls the task's per-step record after every control step. No fork of yam_arena and
no third-party assets: every body is built from box and cylinder colliders in `task.py`. The 7 x 7 mm window is four
real boxes around a hole, so the pin's passage is plain contact physics.

## Offline check, no simulator

```bash
python -m pytest -q tests
```

20 tests feed synthetic records to the scorer: a demo-shaped trace scores 1.0; nothing, the pin pushed home through
empty air with the link on its stop, and the link held without the pin all score 0; the 1.0 deg window (0.99 counts,
1.01 does not, both signs); the 0.25 rung at 18.0 mm of travel and not at 17.9; the pin home at 26.0 mm and not at
25.9; the pin pulled back after being home keeps 0.5; a hand 29 mm away or moving at 6 mm/s at the end scores 0.75;
0.9 s of release is not enough and 1.1 s is; the link not still (3 mm/s or 2.1 deg/s) scores 0.75; a fast swing of the link
(121 or 400 deg/s) at any time costs nothing when the end state is right; a fall with no recovery scores 0; and the record is clean after a reset.

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Zero the robot arm joint: raise the link to its zero mark, push the calibration pin home so the link rests on it, then let go." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/robot-joint-zeroing-pin-dream
```

Scoring is automatic. `task_score` reports 1.0 when the task file's `success()` holds for 1.0 s and the episode ends
with `termination_reason="success"`; otherwise it is the task file's `score()` at the end of the episode
(`SCORE_OVER = "end"`). Repeat trials: add `--epochs K`; with `object_jitter=0` the seed changes nothing in the scene.

Scored runs: 1200 steps, 2 min wall clock, log `adhoc_3d48199d.json`, task_score 0.0, max_steps.

```bash
inspect-robots inspect adhoc_3d48199d.json
jq '{status, scores: .samples[0].epochs, ends: .samples[0].termination_reasons, limit: .eval.max_steps, seed: .eval.seed, task_sha12: .eval.environment_revision}' adhoc_3d48199d.json
```

## Scoring

Measured every control step from reset: th = link angle from the zero mark (deg), qp = pin travel (mm). The record is a
linkkit History updated every control step and cleared only by the simulator reset, so withdrawing the pin and
starting again never clears it.

- No speed gate and no penalty for a wrong first try: the link may be swung, dropped and lifted again at any speed.
  Only what stands at the end, and the rungs below, decide the score.
- Zeroed: th within 1.0 deg (the can end within 2.8 mm of the zero height).
- 0.25: the pin 18 mm in (its tip 3 mm into the window) with the link zeroed at that time.
- 0.5: the pin home (26 mm of travel or more) with the link zeroed, at some time.
- 0.75: at the end the pin is home and the link is within 1 deg of zero.
- 1.0: that, with the link still (under 2 mm/s at the can end and 2 deg/s) for 1 s, and the release gate: every pad of
  both grippers at least 30 mm from the assembly and both grippers under 5 mm/s for the last 1 s. Success must hold 1 s.

Real-trial checks, each with a named tool, matching the scorer:

| Check | Tool |
|---|---|
| Link zeroed (within 1 deg) | scribe lines on housing and link; a height gauge on the can end reads 90 mm over the bench at zero, 1 deg = 2.8 mm |
| Pin 18 mm in, pin home (26 mm or more) | calipers from the knob's front face to the guide face: 29 mm minus the travel, so 11 mm or less at 18 mm in and 3 mm or less for home |
| Link still | 60 fps side video; can end moves under 2 mm in 1 s |
| Hands clear | ruler marks on the table; 30 mm from every pad to the assembly, pads moving under 5 mm/s |

The knob front starts 29 mm from the guide face at 0 travel, so the gap is 29 mm minus the travel (11 mm at 18 mm in,
3 mm at 26 mm).

## Scripted demo and cheat variants

```bash
python linkdemo.py task.py demo_out                          # scripted demo
python linkdemo.py task.py cheat_out --variant left_only     # a cheat variant (or DEMO_VARIANT=left_only)
```

`linkdemo.py` runs `demo_script()` from `task.py` closed loop at 20 Hz and writes `top_cam.mp4`, `left_cam.mp4`,
`right_cam.mp4`, `setup.png` and `result.json` (score, steps, IK error and the scorer's record). The script reads only
what a camera could measure: link poses, the hinge angle computed from them, and the robot's own grippers. It never
reads joint values or the record.

1. Left arm: pinches the flat ends of the actuator can (34 mm, pads closing along the hinge, 3 mm squeeze), swings the
   link up 30 deg with the wrist turning with it, then trims from the seen angle (0.79 deg, then 0.02 deg) and holds it
   at 0.03 deg.
2. Right arm: pinches the pin knob across its diameter, hand pitched 45 deg down so it reaches, and pushes it 28 mm
   along the hinge line. The tip passes the window and the detent holds the pin at 26 mm.
3. The right hand lets go and backs off along its approach. The left hand opens and backs off 40 mm. The link rests
   on the pin at -0.64 deg (the window's play), still, both hands clear.

The demo scores 1.0 (success at step 551 of 1,200): pin home at 26.2 mm, link at -0.64 deg, still for 4 s, both hands clear for 2 s.
Worst IK error 0.09 mm on the right arm; the left arm's error appears only in the final 40 mm back-off after the
release.

Variants (`DEMO_VARIANT`), each run in Isaac Sim with the submitted task file (1d64e228670e); every one scores exactly 0:

| Variant | What it does | Result | task_score |
|---|---|---|---|
| `nothing` | Both arms stay at the ready pose. | The link stays on its stop at -30 deg, the pin at 0. | 0.0 |
| `left_only` | Left arm lifts and holds the link at zero; the right arm idles at home. | The link is held at 0.05 deg from 12 s on, but the pin never moves (0 mm): no rung. | 0.0 |
| `right_only` | Right arm pushes the pin home; the left arm idles at home. | The link is on its stop at -30 deg; the pin passes 11 mm above it, 26.1 mm of travel through empty air, never with the link zeroed: no rung. | 0.0 |
| `pin_first` | Right pushes the pin home first, then the left arm lifts the link against it. | The pin sits in the window path, so the link cannot reach zero with the pin in: the pin is never home with the link zeroed; at the end the link sits at -30 deg. | 0.0 |
| `drop` | Left holds the link at zero, then lets go before the pin is in. | The link falls to its stop; the pin is never pushed: no rung. | 0.0 |
| `short_pin` | Left holds the link at zero; right pushes the pin only 15 mm, then both let go. | The pin tip never enters the window (under the 18 mm rung) and the spring returns it to 0; the link falls to -30 deg: no rung. | 0.0 |

## Revision

Reviewer point, Scoring (not met): the permanent zero for exceeding 120 degrees per second also penalised controlled,
harmless link movement. Requested change: remove it or replace it with an objectively measurable damage condition, so
controlled fast movement does not invalidate a successful trial.

- Removed the 120 deg/s gate from the scorer, the scoring text, the real-trial checks (the 60 fps fall-gate row) and the
  tests. No other speed or fall condition replaces it: the score is the final state (pin home with the link within
  1 deg, link still, hands clear) and the ladder rungs, so a fast or fumbled first try that is recovered costs nothing.
- The old cheats `pin_first` and `drop` scored 0 through the gate; they now score 0 on the end state alone (no rung
  reached, the link not zeroed with the pin in). Added the cheat `short_pin` (pin stopped under the 18 mm rung); it
  scores 0.
- Tests: the gate test became "a fast transient does not zero the run" plus "a fall with no recovery scores 0".

## Build log

| Run | Change | Result |
|---|---|---|
| 1 to 2 | first build, idle arms; pin and link placed 0.207 m, yaw 60 deg | scene still; left arm out of reach (IK error 127 mm), right pusher out of reach |
| 3 | offline reach search over position, yaw and hand pitch; moved to (0.240, 0.000), yaw 75 deg, pusher pitched 45 deg | right arm pushes the pin; left still out of reach on the arc |
| 4 to 6 | left pinch 1 mm squeeze slipped; firm squeeze and follow 1.0 held but the hand struck the pin guide at the bar | link stuck at -23 deg, hand 33 mm under its command |
| 7 | pinch moved to the actuator can's flat ends, 145 mm from the hinge, clear of the guide | link zeroed to 0.03 deg |
| 8 | full demo | 1.0 at step 557, left back-off beyond reach (shortened to 40 mm) |
| 9 to 13 | four cheats and the final demo with `--check-reset` | 1.0 at step 551; every cheat 0; reset check clean |

## Setup

- Joint housing: 6061 aluminium, 1.2 kg, bolted to the table: a 50 x 24 x 110 mm block on a 120 x 150 x 8 mm plate,
  hinge 90 mm above the table, a pin guide of two 10 x 10 x 24 mm cheeks and a post 45 mm out along the link.
- Arm link: aluminium 10 x 20 x 160 mm bar and a 28 mm dia x 34 mm actuator can, 150 g, friction 0.9 / 0.8 on a free
  hinge (friction 0.004 N m, damping 0.01 N m s/rad) between -30 deg (stop) and +25 deg; a 7 x 7 mm window 45 mm from
  the hinge.
- Calibration pin: 6 mm dia x 40 mm steel, 13 g, with a red 16 mm dia x 8 mm knob; 28 mm travel, 0.8 N return spring
  at full travel, an O-ring detent from 26 mm.
- Positions, colours and friction for every body are in `task.json` (setup) and at the top of `task.py`.

## Reset between trials

Nothing is consumed. Let the link rest on its -30 deg stop (can end 25 mm above the table), pull the pin back to its
start so the detent is released, check both zero marks are clean, and move both grippers to the ready pose. The
housing is bolted down and needs no reset.

## Licence

MIT. Copyright (c) 2026 the task authors. `task.py`, `linkkit.py`, `linkdemo.py` and `tests/test_scoring.py` are our
own code (SPDX header in each file; `linkkit.py` and `linkdemo.py` carry the full MIT text). They use only public
Isaac Lab and USD Physics techniques, copy no code from other tasks in this repository, and the scene uses no
third-party assets or brands.
