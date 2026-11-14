# Seat a robot controller's add-on board on its two rows of header pins without bending any pin

The robot picks up an add-on board (a shield with two rows of 20 pins) that sits cocked on the tips of a controller's
header pins, brings it level, and presses it home evenly so that every pin enters its socket. The controller is a green
board on an aluminium carrier plate with a black 2 x 20 female header; the add-on board is blue with a 25 mm aluminium
heat sink on top. At the end the board is fully seated (underside within 0.3 mm of the header top), no pin has been
bent, and both grippers have let go. Electronics assemblers and robotics technicians do this whenever they build or
repair a robot controller stack.

The ability is aligning a tilted part square, then pressing it evenly. Pressed as it rests (cocked 7 deg lengthwise and
3 deg across), the leading pins carry the load and bend. The pins enter cleanly only when the board is within 1.5 deg
lengthwise and 4 deg across as the tips pass 4 mm above the seat, and the mating force (10 N, 0.25 N per pin) needs a
firm, central press. The board is a thin sheet with nothing to grip except the heat sink, so the robot must pinch the
heat sink, bring the board level while holding it, and then press through the heat sink's centre.

Instruction given to the policy (21 words; it states the goal only, never the technique):

`Plug the blue add-on board fully onto the controller's header pins without bending any pin, then let go with both grippers.`

Honest limits. Only one arm has a role; the second arm is idle (the controller is bolted down). The pins and sockets are
not collision-modelled: the board rides a vertical slide with a spring-like latch and two tilt hinges, and the bend is a
rule on its tilt at 4 mm above the seat (see `task.json`, fidelity). The 10 N is the peak of that spring, not measured
pin friction.

Files in this folder: `task.json`, `task.py` (scene, whole-run record, scoring and the scripted demo), `linkkit.py`
(articulated rigid-link bodies for yam_arena), `linkdemo.py` (closed-loop scripted-demo runner),
`tests/test_scoring.py` (scorer edge cases), `README.md`. `linkkit.py` and `linkdemo.py` are the same kit files as the
program's other linkage tasks, unchanged.

## Requirements

Built on the program's Brev launchable (Isaac Lab-Arena, Isaac Sim, Isaac Lab, one L40S). Versions used for the demo and
cheat runs below:

- inspect-robots 1.0.0 (`inspect-robots --version`)
- yam-arena at commit 0059d02a1a91 (`git -C ~/yam-isaac-lab-arena rev-parse --short=12 HEAD`)
- inspect-robots-dream 0.1.0, Dream checkpoint `dreamlabs/dream-1-beta` (for the Dream-1 trial)
- task file SHA-256, first 12 hex: 59eeb6c46d1e (`sha256sum task.py | cut -c1-12`; equals `eval.environment_revision`
  in the Dream-1 log)

Before every shell: `source ~/IsaacLab-Arena/yam.env` (on our VM, `/data/IsaacLab-Arena/yam.env`).

`linkkit.py` and `linkdemo.py` must sit next to `task.py`.

## Offline check, no simulator

```bash
python -m pytest -q tests
```

16 tests feed synthetic records to the scorer (they import `task.py`, so run them in the simulator's Python
environment): a demo-shaped trace scores 1.0; nothing scores 0; lengthwise tilt 1.5 deg is no gate and 1.6 deg (either
sign) is; across 4.0 deg is no gate and 4.1 is; a cocked board above 4 mm is no gate; a bend beats a clean seated end;
0.3 mm high is seated and 0.31 is not; the 0.25 rung needs a gripped (fingers apart) and levelled board (1.0 deg and
2.0 deg limits) while closed fingertips on top earn nothing; a seated board with a hand on it scores 0.75; the release
gate is 30 mm and 1.0 s, and a hand over 5 mm/s, a board over 3 mm/s or 3 deg/s, or a late movement keeps 0.75; the
record is clean after a reset.

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Plug the blue add-on board fully onto the controller's header pins without bending any pin, then let go with both grippers." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/header-pins-align-press-dream
```

Scoring is automatic. `task_score` reports 1.0 when the task file's `success()` holds for 1.0 s and the episode ends
with `termination_reason="success"`; otherwise it is the task file's `score()` at the end of the episode
(`SCORE_OVER = "end"`). Ladder: 0 = a pin bent at any time; 0.25 = held and levelled above the pins; 0.5 = pressed
within 2 mm of the seat, unbent; 0.75 = seated at the end; 1.0 = seated, still 1 s, and both grippers 30 mm clear and
slower than 5 mm/s for 1 s.

## Scripted demo and cheat variants

```bash
python linkdemo.py task.py demo_out                           # scripted demo
python linkdemo.py task.py cheat_out --variant cocked_press   # a cheat variant (or DEMO_VARIANT=cocked_press)
```

`linkdemo.py` runs `demo_script()` from `task.py` closed loop at 20 Hz and writes the camera videos, `setup.png` and
`result.json`. The script reads only what a camera could measure (the board's pose and the tilt angles computed from it,
and the robot's own grippers).

1. The right arm pinches the heat sink (pads across its 25 mm), reads the board's pose, and over three corrections of
   1.5 s rotates the board about its centre until the seen roll and pitch are level (seen -0.3 deg and -0.2 deg).
2. It opens and lifts away; the pin tips prop the board where it was left (seen 0.9 deg lengthwise afterwards).
3. It presses with closed fingertips straight down on the heat sink centre (commanded 30 mm past the top) until the
   board is home, opens and lifts away.

The demo scores 1.0 (`success`, episode ended at step 455 of 1,200): no pin bent, seat height 0.0 mm, and both grippers
clear. Worst IK error 22 mm and 15 deg (the arm sags under the 10 N press).

Variants (`DEMO_VARIANT`), each run in Isaac Sim with the submitted task file (59eeb6c46d1e); each scores 0:

| Variant | What it does | Result | task_score |
|---|---|---|---|
| `nothing` | Both arms stay at the ready pose. | The board stays cocked on the pin tips. | 0.0 |
| `cocked_press` | Pinches the heat sink and pushes straight down without levelling. | The pinch slips; the board reaches 5.8 mm above the seat and is never levelled. | 0.0 |
| `cocked_top_press` | Closed fingertips straight down on the cocked board's heat sink. | The pins enter at 7 deg lengthwise and bend (gate at 5.8 s). | 0.0 |

## Build log (key runs, shared L40S)

| Run | Change | Result |
|---|---|---|
| 1 | tilt hinges with friction | 0.0: the board fell to the hinge limit at once (it is an inverted pendulum) |
| 2 | brakes hold the tilt until a gripper has the board | stable; the pinch levelled it but pushing slipped on the heat sink at 6 mm |
| 3 | let go after levelling; brakes hold it; press with closed fingertips | 0.5: pressed to 2 mm, sprang back when the hand lifted |
| 4 | the slide latches what is pushed in; 10 N | 1.0 at step 455 |

## Dream-1 trial

The Dream-1 run is not done yet. The table below is filled in from the run.

| Run | inspect-robots | yam-arena | task SHA | steps | minutes | log | task_score | reason |
|---|---|---|---|---|---|---|---|---|
| Dream-1 (`--policy dream`, command above) | 1.0.0 | 0059d02a1a91 | 59eeb6c46d1e | 1200 | 4 | adhoc_a6ec7eaf.json | 0.0 | max_steps |

```bash
inspect-robots inspect adhoc_a6ec7eaf.json
jq '{status, scores: .samples[0].epochs, ends: .samples[0].termination_reasons, limit: .eval.max_steps, seed: .eval.seed, task_sha12: .eval.environment_revision}' adhoc_a6ec7eaf.json
```
